#!/usr/bin/env python3
"""Sign and verify BenchPod firmware and blob images (docs/design/firmware-signing.md).

Every asset gets a detached 128-byte manifest, `<asset>.sig`:

  off size field
    0    4 magic "BPSG"
    4    2 format (1)
    6    1 target: 0 firmware, 1 gw0, 2 gw1, 3 esp
    7    1 flags (0)
    8    4 image size
   12   32 SHA-256 of the image
   44    4 asset version (firmware: packed major<<16|minor<<8|patch; gateware: its version)
   48    4 release: packed firmware version of the release the asset belongs to
   52    2 fw_info layout (firmware only, else 0)
   54    2 fw_info min_flash_kb (firmware only, else 0)
   56    8 key_id: first 8 bytes of SHA-512(public key)
   64   64 Ed25519 signature over CONTEXT || bytes 0..63

All integers little-endian. A key file holds the 32-byte Ed25519 seed as 64 hex characters.

  fwsign.py keygen OUT                      new key (file mode 0600); prints the public key only
  fwsign.py pubkey KEY                      public key + key_id of a key file
  fwsign.py sign --key KEY --target T --release X.Y.Z [--version N] IMAGE [-o OUT]
  fwsign.py sign-blobs --key KEY --release X.Y.Z blobs-manifest.json
  fwsign.py verify --pub HEX|FILE [--pub ...] [--target T] IMAGE [SIG]
  fwsign.py keys-header --pub FILE [--pub FILE ...] -o OUT.h
  fwsign.py vectors -o OUT.json [--c-header OUT.h]   deterministic test vectors for the C and Go checks

KEY may be `env:NAME` to read the seed from an environment variable (CI).
"""
import argparse
import hashlib
import json
import os
import struct
import sys

# `cryptography` is imported only by the commands that sign or verify, so the firmware build
# (keys-header) runs on a machine without it.
def _ed25519():
    try:
        from cryptography.hazmat.primitives import serialization
        from cryptography.hazmat.primitives.asymmetric import ed25519
        from cryptography.exceptions import InvalidSignature
    except ImportError:
        sys.exit("fwsign.py: this command needs the Python package `cryptography` (pip install cryptography)")
    return serialization, ed25519, InvalidSignature

MAGIC = b"BPSG"
FORMAT = 1
CONTEXT = b"benchpod-fw-sign:v1\x00"
MANIFEST_LEN = 128
SIGNED_LEN = 64
TARGETS = {"firmware": 0, "gw0": 1, "gw1": 2, "esp": 3}
FW_INFO_OFFSET = 0x400
FW_INFO_MAGIC = 0x57465042  # "BPFW"


def pack_version(text):
    """'3.5.1' or 'v3.5.1' -> 0x030501."""
    parts = text.lstrip("v").split("-")[0].split(".")
    if len(parts) != 3 or not all(p.isdigit() for p in parts):
        raise ValueError("version must look like X.Y.Z: %r" % text)
    major, minor, patch = (int(p) for p in parts)
    if major > 0xFFFF or minor > 0xFF or patch > 0xFF:
        raise ValueError("version out of range: %r" % text)
    return (major << 16) | (minor << 8) | patch


def unpack_version(v):
    return "%d.%d.%d" % (v >> 16, (v >> 8) & 0xFF, v & 0xFF)


def raw_public(priv):
    serialization, _, _ = _ed25519()
    return priv.public_key().public_bytes(serialization.Encoding.Raw, serialization.PublicFormat.Raw)


def key_id(pub):
    return hashlib.sha512(pub).digest()[:8]


def load_seed(spec):
    if spec.startswith("env:"):
        text = os.environ.get(spec[4:], "")
        if not text:
            sys.exit("environment variable %s is empty" % spec[4:])
    else:
        with open(os.path.expanduser(spec)) as f:
            text = f.read()
    seed = bytes.fromhex(text.strip())
    if len(seed) != 32:
        sys.exit("a key is a 32-byte seed in hex")
    return _ed25519()[1].Ed25519PrivateKey.from_private_bytes(seed)


def load_pub(spec):
    if os.path.exists(os.path.expanduser(spec)):
        with open(os.path.expanduser(spec)) as f:
            spec = f.read().split()[0]
    pub = bytes.fromhex(spec.strip())
    if len(pub) != 32:
        sys.exit("a public key is 32 bytes in hex")
    return pub


def fw_info_of(image):
    """(layout, min_flash_kb) from a firmware image's fw_info block, (0, 0) without one."""
    blk = image[FW_INFO_OFFSET:FW_INFO_OFFSET + 16]
    if len(blk) < 16:
        return 0, 0
    magic, layout, min_kb = struct.unpack_from("<IHH", blk)
    return (layout, min_kb) if magic == FW_INFO_MAGIC else (0, 0)


def build_body(image, target, version, release, kid):
    t = TARGETS[target]
    layout, min_kb = fw_info_of(image) if t == 0 else (0, 0)
    body = MAGIC + struct.pack("<HBBI", FORMAT, t, 0, len(image)) + hashlib.sha256(image).digest()
    body += struct.pack("<IIHH", version, release, layout, min_kb) + kid
    assert len(body) == SIGNED_LEN
    return body


def sign(priv, image, target, version, release):
    body = build_body(image, target, version, release, key_id(raw_public(priv)))
    return body + priv.sign(CONTEXT + body)


def parse(sig):
    """The manifest's fields as a dict, or raise ValueError."""
    if len(sig) != MANIFEST_LEN:
        raise ValueError("manifest is %d bytes, want %d" % (len(sig), MANIFEST_LEN))
    if sig[:4] != MAGIC:
        raise ValueError("bad magic")
    fmt, target, flags, size = struct.unpack_from("<HBBI", sig, 4)
    if fmt != FORMAT:
        raise ValueError("unknown format %d" % fmt)
    version, release, layout, min_kb = struct.unpack_from("<IIHH", sig, 44)
    names = {v: k for k, v in TARGETS.items()}
    if target not in names:
        raise ValueError("unknown target %d" % target)
    return {
        "target": names[target], "flags": flags, "size": size, "sha256": sig[12:44].hex(),
        "version": version, "release": unpack_version(release), "layout": layout,
        "min_flash_kb": min_kb, "key_id": sig[56:64].hex(),
    }


def verify(sig, image, pubs, target=None):
    """'ok', or the reason it fails (same words as the firmware and the Go check)."""
    try:
        m = parse(sig)
    except ValueError:
        return "format"
    pub = next((p for p in pubs if key_id(p).hex() == m["key_id"]), None)
    if pub is None:
        return "unknown-key"
    _, ed25519, InvalidSignature = _ed25519()
    try:
        ed25519.Ed25519PublicKey.from_public_bytes(pub).verify(sig[64:], CONTEXT + sig[:64])
    except InvalidSignature:
        return "signature"
    if target is not None and m["target"] != target:
        return "target"
    if m["size"] != len(image) or m["sha256"] != hashlib.sha256(image).hexdigest():
        return "image"
    return "ok"


def cmd_keygen(a):
    if os.path.exists(a.out):
        sys.exit("%s exists; not overwriting a key" % a.out)
    serialization, ed25519, _ = _ed25519()
    priv = ed25519.Ed25519PrivateKey.generate()
    seed = priv.private_bytes(serialization.Encoding.Raw, serialization.PrivateFormat.Raw,
                              serialization.NoEncryption())
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    fd = os.open(a.out, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, "w") as f:
        f.write(seed.hex() + "\n")
    pub = raw_public(priv)
    print("%s key_id=%s" % (pub.hex(), key_id(pub).hex()))


def cmd_pubkey(a):
    pub = raw_public(load_seed(a.key))
    print("%s key_id=%s" % (pub.hex(), key_id(pub).hex()))


def cmd_sign(a):
    image = open(a.image, "rb").read()
    release = pack_version(a.release)
    version = int(a.version, 0) if a.version is not None else (release if a.target == "firmware" else 0)
    sig = sign(load_seed(a.key), image, a.target, version, release)
    out = a.out or a.image + ".sig"
    with open(out, "wb") as f:
        f.write(sig)
    print("%s: %s %s key_id=%s" % (out, a.target, a.release, sig[56:64].hex()))


def cmd_sign_blobs(a):
    """Sign every blob a blobs-manifest.json lists (tools/blob_manifest.py), next to the file."""
    m = json.load(open(a.manifest))
    base = os.path.dirname(os.path.abspath(a.manifest))
    priv = load_seed(a.key)
    release = pack_version(a.release)
    for b in m["blobs"]:
        # A blob that was not built has "file": null in a dev tree (the release job refuses that).
        path = os.path.join(base, b["file"]) if b.get("file") else None
        if not path or not b.get("size") or not os.path.exists(path):
            print("%s: not built, not signed" % (b.get("file") or b["name"]))
            continue
        image = open(path, "rb").read()
        with open(path + ".sig", "wb") as f:
            f.write(sign(priv, image, b["name"], int(b.get("version") or 0), release))
        print("%s.sig: %s %s" % (b["file"], b["name"], a.release))


def cmd_verify(a):
    image = open(a.image, "rb").read()
    sig = open(a.sig or a.image + ".sig", "rb").read()
    result = verify(sig, image, [load_pub(p) for p in a.pub], a.target)
    if result == "ok":
        print("ok:", json.dumps(parse(sig)))
    else:
        sys.exit("signature check failed: " + result)


def cmd_keys_header(a):
    lines = ["/* Generated by tools/fwsign.py keys-header. Do not edit. */",
             "#ifndef FW_SIGN_KEYS_GEN_H", "#define FW_SIGN_KEYS_GEN_H",
             "#define FW_SIGN_KEY_COUNT %d" % len(a.pub), "#define FW_SIGN_KEYS_INIT { \\"]
    for spec in a.pub:
        pub = load_pub(spec)
        name = os.path.basename(spec)
        lines.append("    /* %s, key_id %s */ \\" % (name, key_id(pub).hex()))
        lines.append("    { %s }, \\" % ", ".join("0x%02x" % b for b in pub))
    lines += ["}", "#endif", ""]
    text = "\n".join(lines)
    if not os.path.exists(a.out) or open(a.out).read() != text:
        with open(a.out, "w") as f:
            f.write(text)


def cmd_vectors(a):
    """Fixed test key and images, so every implementation checks the same bytes."""
    ed = _ed25519()[1].Ed25519PrivateKey
    priv = ed.from_private_bytes(bytes(range(32)))
    other = ed.from_private_bytes(bytes(range(32, 64)))
    pub = raw_public(priv)
    fw = bytearray(b"\x5a" * 0x420)
    struct.pack_into("<IHH", fw, FW_INFO_OFFSET, FW_INFO_MAGIC, 2, 1024)
    fw = bytes(fw)
    enforcing = bytearray(fw)
    struct.pack_into("<I", enforcing, FW_INFO_OFFSET + 8, 1)   # fw_info reserved[0]: enforces signatures
    enforcing = bytes(enforcing)
    blob = bytes((i * 7) & 0xFF for i in range(1000))
    good = sign(priv, fw, "firmware", pack_version("3.6.0"), pack_version("3.6.0"))
    good_blob = sign(priv, blob, "gw1", 45, pack_version("3.6.0"))
    flipped = bytearray(good)
    flipped[100] ^= 1
    body_flip = bytearray(good)
    body_flip[48] ^= 1   # release field changed after signing
    bad_magic = b"XPSG" + good[4:]
    images = {"firmware": fw, "blob": blob, "firmware_changed": fw[:-1] + b"\x00",
              "firmware_short": fw[:-1], "firmware_enforcing": enforcing}
    cases = [
        ("good firmware", good, "firmware", "firmware", "ok"),
        ("good gateware blob", good_blob, "blob", "gw1", "ok"),
        ("firmware manifest used for a blob", good, "firmware", "gw1", "target"),
        ("image changed", good, "firmware_changed", "firmware", "image"),
        ("image shorter", good, "firmware_short", "firmware", "image"),
        ("signature byte flipped", bytes(flipped), "firmware", "firmware", "signature"),
        ("signed field changed", bytes(body_flip), "firmware", "firmware", "signature"),
        ("other key", sign(other, fw, "firmware", 0, 0), "firmware", "firmware", "unknown-key"),
        ("bad magic", bad_magic, "firmware", "firmware", "format"),
        ("short manifest", good[:127], "firmware", "firmware", "format"),
        ("good firmware that enforces signatures",
         sign(priv, enforcing, "firmware", pack_version("3.7.0"), pack_version("3.7.0")),
         "firmware_enforcing", "firmware", "ok"),
    ]
    out = {
        "context": CONTEXT.decode("ascii").rstrip("\x00"),
        "public_key": pub.hex(),
        "key_id": key_id(pub).hex(),
        "images": {k: v.hex() for k, v in images.items()},
        "cases": [],
    }
    for name, sig, image, target, want in cases:
        assert verify(sig, images[image], [pub], target) == want, name
        out["cases"].append({"name": name, "sig": sig.hex(), "image": image,
                             "target": target, "want": want})
    with open(a.out, "w") as f:
        json.dump(out, f, indent=1)
        f.write("\n")
    if a.c_header:
        write_c_vectors(a.c_header, pub, images, out["cases"])


def c_bytes(b):
    return ", ".join("0x%02x" % x for x in b)


def write_c_vectors(path, pub, images, cases):
    names = list(images)
    lines = ["/* Generated by tools/fwsign.py vectors. Do not edit. */",
             "static const uint8_t vec_pub[32] = { %s };" % c_bytes(pub)]
    for n in names:
        lines.append("static const uint8_t vec_img_%s[] = { %s };" % (n, c_bytes(images[n])))
    lines.append("static const struct { const uint8_t *data; size_t len; uint8_t sha256[32]; } vec_images[] = {")
    lines += ["    { vec_img_%s, sizeof(vec_img_%s), { %s } }," % (n, n, c_bytes(hashlib.sha256(images[n]).digest()))
              for n in names]
    lines.append("};")
    lines.append("static const struct { const char *name; uint8_t sig[128]; size_t sig_len; "
                 "int image; uint8_t target; const char *want; } vec_cases[] = {")
    for c in cases:
        sig = bytes.fromhex(c["sig"])
        lines.append('    { "%s", { %s }, %d, %d, %d, "%s" },' % (
            c["name"], c_bytes(sig), len(sig), names.index(c["image"]), TARGETS[c["target"]], c["want"]))
    lines += ["};", ""]
    with open(path, "w") as f:
        f.write("\n".join(lines))


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = p.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("keygen"); s.add_argument("out"); s.set_defaults(fn=cmd_keygen)
    s = sub.add_parser("pubkey"); s.add_argument("key"); s.set_defaults(fn=cmd_pubkey)
    s = sub.add_parser("sign")
    s.add_argument("--key", required=True)
    s.add_argument("--target", required=True, choices=TARGETS)
    s.add_argument("--release", required=True, help="firmware release, X.Y.Z")
    s.add_argument("--version", help="asset version (default: the release for firmware, 0 else)")
    s.add_argument("-o", "--out")
    s.add_argument("image")
    s.set_defaults(fn=cmd_sign)
    s = sub.add_parser("sign-blobs")
    s.add_argument("--key", required=True)
    s.add_argument("--release", required=True)
    s.add_argument("manifest")
    s.set_defaults(fn=cmd_sign_blobs)
    s = sub.add_parser("verify")
    s.add_argument("--pub", action="append", required=True)
    s.add_argument("--target", choices=TARGETS)
    s.add_argument("image")
    s.add_argument("sig", nargs="?")
    s.set_defaults(fn=cmd_verify)
    s = sub.add_parser("keys-header")
    s.add_argument("--pub", action="append", required=True)
    s.add_argument("-o", "--out", required=True)
    s.set_defaults(fn=cmd_keys_header)
    s = sub.add_parser("vectors")
    s.add_argument("-o", "--out", required=True)
    s.add_argument("--c-header", help="also write the vectors as a C header (host tests)")
    s.set_defaults(fn=cmd_vectors)
    a = p.parse_args()
    a.fn(a)


if __name__ == "__main__":
    main()
