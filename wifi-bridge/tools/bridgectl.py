#!/usr/bin/env python3
"""Reference client for the Wi-Fi bridge control protocol (docs/PROTOCOL.md).

Talks to a bridge over raw Ethernet (EtherType 0x88B5) from a Linux machine wired to the
bridge's RJ45, standing in for the pod during bring-up. The bridge learns this machine's MAC as
the "pod" MAC, so the machine also gets its network through the bridge.

    sudo ./bridgectl.py -i eth0 hello            # wait for a HELLO and print it
    sudo ./bridgectl.py -i eth0 pair             # offer a new key (bridge unpaired, or paired with us)
    sudo ./bridgectl.py -i eth0 status
    sudo ./bridgectl.py -i eth0 scan
    sudo ./bridgectl.py -i eth0 set-wifi "lab net" "passphrase"
    sudo ./bridgectl.py -i eth0 clear-wifi | reboot | factory-reset
    sudo ./bridgectl.py -i eth0 ota build/benchpod-wifi-bridge.bin

Pairing: an unpaired bridge pairs with the first device that sends PAIR. Both sides compute the
key with X25519 (it is never sent); after that every request is signed (HMAC-SHA256) with it and
the bridge's per-boot nonce from HELLO, and set-wifi seals the passphrase (XChaCha20). The secret,
the key and the request counter are kept in --state (default ~/.config/benchpod-bridge/<MAC>.json).
The crypto is pure Python (no packages), checked against the C code in test_bridgectl.py.

Linux only (AF_PACKET), needs root or CAP_NET_RAW. The codec has no such requirement and is
unit-tested in test_bridgectl.py.
"""
import argparse
import hashlib
import hmac
import json
import os
import secrets
import socket
import struct
import sys
import time

ETHERTYPE = 0x88B5
MAGIC = b"BW"
VERSION = 1
RESPONSE = 0x80

HELLO, STATUS, SCAN, SET_WIFI, CLEAR_WIFI = 0x01, 0x02, 0x03, 0x04, 0x05
OTA_BEGIN, OTA_DATA, OTA_END, REBOOT, FACTORY_RESET, OTA_ABORT = 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B
PAIR = 0x0C

T_RESULT, T_MODEL, T_FW_VERSION, T_HW_REV, T_STATE, T_BRIDGE_MAC, T_POD_MAC = 1, 2, 3, 4, 5, 6, 7
T_SSID, T_PASSPHRASE, T_CHANNEL, T_RSSI, T_BSSID, T_AUTHMODE, T_AP = 8, 9, 10, 11, 12, 13, 14
T_IMAGE_SIZE, T_OFFSET, T_DATA, T_UPTIME, T_REASON, T_COUNTERS, T_MESSAGE, T_FLAGS = 15, 16, 17, 18, 19, 20, 21, 22
T_NONCE, T_PUBKEY, T_COUNTER, T_AUTH, T_ENC_PASSPHRASE = 0x17, 0x18, 0x19, 0x1A, 0x1B
F_PAIRED = 0x20

STATES = ["unconfigured", "wait-pod", "joining", "connected", "auth-failed", "no-ap"]
RESULTS = ["ok", "bad-request", "unsupported", "busy", "storage", "ota-not-started", "ota-bad-offset",
           "ota-verify", "ota-too-large", "internal", "ota-wrong-image", "not-paired", "auth",
           "already-paired"]
AUTH = ["open", "wep", "wpa", "wpa2", "wpa/wpa2", "wpa2-ent", "wpa3", "wpa2/wpa3"]
COUNTERS = ["eth_rx", "wifi_tx", "wifi_tx_err", "wifi_rx", "eth_tx", "eth_tx_err", "dropped"]
BROADCAST = b"\xff" * 6


def tlv(tag, value):
    if isinstance(value, str):
        value = value.encode()
    return struct.pack(">BH", tag, len(value)) + value


def encode(dst, src, mtype, seq, tlvs=b""):
    frame = dst + src + struct.pack(">H", ETHERTYPE) + MAGIC + struct.pack(">BBHH", VERSION, mtype, seq, len(tlvs))
    frame += tlvs
    return frame.ljust(60, b"\0")


# ---- crypto, matching components/bridge_core/bwp_crypto.c (Monocypher 4.0.2)

_P = 2 ** 255 - 19


def x25519(k, u):
    """RFC 7748 X25519 scalar multiplication."""
    k = bytearray(k)
    k[0] &= 248
    k[31] &= 127
    k[31] |= 64
    k = int.from_bytes(k, "little")
    x1 = int.from_bytes(u, "little") & ((1 << 255) - 1)
    x2, z2, x3, z3, swap = 1, 0, x1, 1, 0
    for t in reversed(range(255)):
        kt = (k >> t) & 1
        swap ^= kt
        if swap:
            x2, x3, z2, z3 = x3, x2, z3, z2
        swap = kt
        a, b = (x2 + z2) % _P, (x2 - z2) % _P
        aa, bb = a * a % _P, b * b % _P
        e = (aa - bb) % _P
        c, d = (x3 + z3) % _P, (x3 - z3) % _P
        da, cb = d * a % _P, c * b % _P
        x3, z3 = (da + cb) ** 2 % _P, x1 * (da - cb) ** 2 % _P
        x2, z2 = aa * bb % _P, e * (aa + 121665 * e) % _P
    if swap:
        x2, z2 = x3, z3
    return (x2 * pow(z2, _P - 2, _P) % _P).to_bytes(32, "little")


def x25519_public(secret):
    return x25519(secret, (9).to_bytes(32, "little"))


def pair_key(my_secret, peer_pub, pod_pub, bridge_pub):
    shared = x25519(my_secret, peer_pub)
    if shared == bytes(32):
        raise ValueError("bad public key")
    return hashlib.blake2b(b"BWP pairing v1" + pod_pub + bridge_pub, digest_size=32, key=shared).digest()


def _rounds(st):
    def qr(a, b, c, d):
        st[a] = (st[a] + st[b]) & 0xFFFFFFFF; st[d] ^= st[a]; st[d] = ((st[d] << 16) | (st[d] >> 16)) & 0xFFFFFFFF
        st[c] = (st[c] + st[d]) & 0xFFFFFFFF; st[b] ^= st[c]; st[b] = ((st[b] << 12) | (st[b] >> 20)) & 0xFFFFFFFF
        st[a] = (st[a] + st[b]) & 0xFFFFFFFF; st[d] ^= st[a]; st[d] = ((st[d] << 8) | (st[d] >> 24)) & 0xFFFFFFFF
        st[c] = (st[c] + st[d]) & 0xFFFFFFFF; st[b] ^= st[c]; st[b] = ((st[b] << 7) | (st[b] >> 25)) & 0xFFFFFFFF
    for _ in range(10):
        qr(0, 4, 8, 12); qr(1, 5, 9, 13); qr(2, 6, 10, 14); qr(3, 7, 11, 15)
        qr(0, 5, 10, 15); qr(1, 6, 11, 12); qr(2, 7, 8, 13); qr(3, 4, 9, 14)


_SIGMA = struct.unpack("<4I", b"expand 32-byte k")


def xchacha20(key, nonce24, data):
    """XChaCha20 as Monocypher's crypto_chacha20_x: HChaCha20 sub-key, then ChaCha20 (djb, 64-bit counter)."""
    st = list(_SIGMA) + list(struct.unpack("<8I", key)) + list(struct.unpack("<4I", nonce24[:16]))
    _rounds(st)
    sub = struct.pack("<8I", *(st[0:4] + st[12:16]))
    out = bytearray()
    for blk in range((len(data) + 63) // 64):
        init = list(_SIGMA) + list(struct.unpack("<8I", sub)) + [blk & 0xFFFFFFFF, blk >> 32] + \
            list(struct.unpack("<2I", nonce24[16:24]))
        st = init[:]
        _rounds(st)
        ks = struct.pack("<16I", *((x + y) & 0xFFFFFFFF for x, y in zip(st, init)))
        chunk = data[blk * 64:(blk + 1) * 64]
        out += bytes(c ^ k for c, k in zip(chunk, ks))
    return bytes(out)


def seal_passphrase(key, nonce24, passphrase):
    """ENC_PASSPHRASE: nonce || XChaCha20(BLAKE2b(key, "BWP passphrase v1"), [len, pass, zeros])."""
    p = passphrase.encode() if isinstance(passphrase, str) else passphrase
    if len(p) > 64:
        raise ValueError("passphrase longer than 64 bytes")
    k = hashlib.blake2b(b"BWP passphrase v1", digest_size=32, key=key).digest()
    return nonce24 + xchacha20(k, nonce24, (bytes([len(p)]) + p).ljust(65, b"\0"))


def auth_tag(key, nonce, signed_part):
    """Truncated HMAC-SHA256 over nonce + frame bytes from the magic up to the AUTH TLV."""
    return hmac.new(key, nonce + signed_part, hashlib.sha256).digest()[:16]


def encode_signed(dst, src, mtype, seq, tlvs, key, nonce, counter):
    tlvs += tlv(T_COUNTER, struct.pack(">I", counter))
    blen = len(tlvs) + 3 + 16
    head = dst + src + struct.pack(">H", ETHERTYPE) + MAGIC + struct.pack(">BBHH", VERSION, mtype, seq, blen)
    frame = head + tlvs
    frame += tlv(T_AUTH, auth_tag(key, nonce, frame[14:]))
    return frame.ljust(60, b"\0")


def verify(frame, key, nonce):
    """True if the frame carries a valid AUTH as its last TLV."""
    blen = struct.unpack_from(">H", frame, 20)[0]
    end = 22 + blen
    if blen < 19 or frame[end - 19] != T_AUTH or frame[end - 18:end - 16] != b"\x00\x10":
        return False
    return hmac.compare_digest(auth_tag(key, nonce, frame[14:end - 19]), frame[end - 16:end])


def parse_tlvs(body):
    """List of (tag, value). Raises ValueError on a truncated TLV."""
    out, i = [], 0
    while i < len(body):
        if len(body) - i < 3:
            raise ValueError("truncated TLV header")
        tag, n = struct.unpack_from(">BH", body, i)
        if i + 3 + n > len(body):
            raise ValueError("TLV runs past the body")
        out.append((tag, body[i + 3:i + 3 + n]))
        i += 3 + n
    return out


def decode(frame):
    """Returns a dict, or None if the frame is not a BWP v1 frame."""
    if len(frame) < 22 or struct.unpack_from(">H", frame, 12)[0] != ETHERTYPE or frame[14:16] != MAGIC:
        return None
    version, mtype, seq, blen = struct.unpack_from(">BBHH", frame, 16)
    if version != VERSION or 22 + blen > len(frame):
        return None
    return {"dst": frame[0:6], "src": frame[6:12], "type": mtype, "seq": seq, "tlvs": parse_tlvs(frame[22:22 + blen])}


def first(tlvs, tag, default=None):
    return next((v for t, v in tlvs if t == tag), default)


def mac_str(b):
    return ":".join(f"{x:02x}" for x in b)


def describe(msg):
    """Human-readable lines for a HELLO, STATUS or SCAN_RESULT."""
    t = msg["tlvs"]
    lines = []
    for tag, v in t:
        if tag == T_RESULT:
            lines.append(f"result: {RESULTS[v[0]] if v[0] < len(RESULTS) else v[0]}")
        elif tag in (T_MODEL, T_FW_VERSION, T_HW_REV, T_SSID, T_MESSAGE):
            name = {T_MODEL: "model", T_FW_VERSION: "fw", T_HW_REV: "hw", T_SSID: "ssid", T_MESSAGE: "message"}[tag]
            lines.append(f"{name}: {v.decode(errors='replace')}")
        elif tag == T_STATE:
            lines.append(f"state: {STATES[v[0]] if v[0] < len(STATES) else v[0]}")
        elif tag in (T_BRIDGE_MAC, T_POD_MAC, T_BSSID):
            lines.append(f"{ {T_BRIDGE_MAC: 'bridge-mac', T_POD_MAC: 'pod-mac', T_BSSID: 'bssid'}[tag]}: {mac_str(v)}")
        elif tag == T_CHANNEL:
            lines.append(f"channel: {v[0]}")
        elif tag == T_RSSI:
            lines.append(f"rssi: {struct.unpack('b', v)[0]}")
        elif tag == T_UPTIME:
            lines.append(f"uptime: {struct.unpack('>I', v)[0]} s")
        elif tag == T_REASON:
            lines.append(f"last-disconnect-reason: {v[0]}")
        elif tag == T_FLAGS:
            lines.append(f"flags: 0x{v[0]:02x}" + (" (paired)" if v[0] & F_PAIRED else ""))
        elif tag == T_COUNTERS:
            vals = struct.unpack(">7I", v)
            lines.append("counters: " + " ".join(f"{n}={x}" for n, x in zip(COUNTERS, vals)))
        elif tag == T_AP:
            a = parse_tlvs(v)
            auth = first(a, T_AUTHMODE, b"\xff")[0]
            lines.append(f"{struct.unpack('b', first(a, T_RSSI))[0]:4d} {first(a, T_CHANNEL)[0]:2d} "
                         f"{AUTH[auth] if auth < len(AUTH) else 'other':9s} {mac_str(first(a, T_BSSID))} "
                         f"{first(a, T_SSID).decode(errors='replace')}")
    return lines


class State:
    """Our X25519 secret, the pairing key and the request counter for one bridge, in a JSON file."""

    def __init__(self, path):
        self.path = path
        try:
            with open(path) as f:
                d = json.load(f)
        except FileNotFoundError:
            d = {}
        h = lambda k: bytes.fromhex(d[k]) if d.get(k) else None  # noqa: E731
        self.secret, self.bridge_pub, self.key = h("secret"), h("bridge_pub"), h("key")
        self.nonce = bytes.fromhex(d.get("nonce") or "")
        self.counter = d.get("counter", 0)

    def learn_bridge_pub(self, pub):
        """The bridge's public key from HELLO: recompute the key (its pairing may be newer)."""
        if self.secret and pub and pub != self.bridge_pub:
            self.bridge_pub = pub
            self.key = pair_key(self.secret, pub, x25519_public(self.secret), pub)
            self.save()

    def next_counter(self, nonce):
        if nonce != self.nonce:  # the bridge rebooted: a new counter sequence
            self.nonce, self.counter = nonce, 0
        self.counter += 1
        self.save()
        return self.counter

    def save(self):
        os.makedirs(os.path.dirname(self.path) or ".", exist_ok=True)
        fd = os.open(self.path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
        with os.fdopen(fd, "w") as f:
            hx = lambda b: b.hex() if b else None  # noqa: E731
            json.dump({"secret": hx(self.secret), "bridge_pub": hx(self.bridge_pub), "key": hx(self.key),
                       "nonce": self.nonce.hex(), "counter": self.counter}, f)


class Link:
    def __init__(self, ifname):
        self.sock = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(ETHERTYPE))
        self.sock.bind((ifname, 0))
        self.mac = self.sock.getsockname()[4]
        self.bridge = None
        self.nonce = None
        self.paired = False
        self.bridge_pub = None
        self.state = None
        self.seq = 0

    def recv(self, timeout):
        end = time.monotonic() + timeout
        while (left := end - time.monotonic()) > 0:
            self.sock.settimeout(left)
            try:
                raw = self.sock.recv(2048)
                msg = decode(raw)
            except socket.timeout:
                return None
            if msg:
                if msg["type"] == HELLO:
                    self.bridge = msg["src"]
                    self.nonce = first(msg["tlvs"], T_NONCE, self.nonce)
                    self.paired = bool(first(msg["tlvs"], T_FLAGS, b"\0")[0] & F_PAIRED)
                    self.bridge_pub = first(msg["tlvs"], T_PUBKEY) if self.paired else None
                msg["raw"] = raw
                return msg
        return None

    def hello(self, timeout=5.0):
        end = time.monotonic() + timeout
        while (left := end - time.monotonic()) > 0:
            msg = self.recv(left)
            if msg and msg["type"] == HELLO:
                return msg
        raise SystemExit("no HELLO from a bridge")

    def request(self, mtype, tlvs=b"", timeout=1.0, retries=3, sign=None):
        dst = self.bridge or BROADCAST
        self.seq = (self.seq % 0xFFFF) + 1
        if sign is None:
            sign = self.paired and self.state.key is not None
        for _ in range(retries):
            # A retry is a new frame with a new counter: the bridge rejects a repeated one.
            if sign:
                key = self.state.key
                frame = encode_signed(dst, self.mac, mtype, self.seq, tlvs, key, self.nonce,
                                      self.state.next_counter(self.nonce))
            else:
                frame = encode(dst, self.mac, mtype, self.seq, tlvs)
            self.sock.send(frame)
            end = time.monotonic() + timeout
            while (left := end - time.monotonic()) > 0:
                msg = self.recv(left)
                if msg and msg["type"] == mtype | RESPONSE and msg["seq"] == self.seq:
                    last = first(msg["tlvs"], T_COUNTER)
                    if sign and result_of(msg)[0] == 12 and last:  # REPLAY: continue above the bridge
                        self.state.counter = max(self.state.counter, struct.unpack(">I", last)[0])
                        break  # resend with a fresh counter
                    if sign and result_of(msg)[0] == 0 and not verify(msg["raw"], key, self.nonce):
                        raise SystemExit("response has no valid AUTH: not our bridge?")
                    return msg
        raise SystemExit(f"no response to request 0x{mtype:02x}")


def result_of(msg):
    r = first(msg["tlvs"], T_RESULT, b"\x09")[0]
    m = first(msg["tlvs"], T_MESSAGE)
    return r, (m.decode() if m else "")


def check(msg):
    r, m = result_of(msg)
    if r:
        raise SystemExit(f"error: {RESULTS[r] if r < len(RESULTS) else r} {m}")
    print("ok")


def ota(link, path):
    image = open(path, "rb").read()
    check(link.request(OTA_BEGIN, tlv(T_IMAGE_SIZE, struct.pack(">I", len(image))), timeout=30))
    off = 0
    while off < len(image):
        chunk = image[off:off + 1024]
        msg = link.request(OTA_DATA, tlv(T_OFFSET, struct.pack(">I", off)) + tlv(T_DATA, chunk))
        r, m = result_of(msg)
        nxt = struct.unpack(">I", first(msg["tlvs"], T_OFFSET, b"\0\0\0\0"))[0]
        if r not in (0, 6):  # 6: bad offset, resume from the bridge's offset
            raise SystemExit(f"error at {off}: {RESULTS[r] if r < len(RESULTS) else r} {m}")
        off = nxt
        print(f"\r{off}/{len(image)}", end="", flush=True)
    print()
    check(link.request(OTA_END, timeout=30))
    check(link.request(REBOOT))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-i", "--interface", required=True)
    ap.add_argument("--state", help="pairing state file (default ~/.config/benchpod-bridge/<bridge MAC>.json)")
    ap.add_argument("command", choices=["hello", "pair", "status", "scan", "set-wifi", "clear-wifi", "reboot",
                                        "factory-reset", "ota"])
    ap.add_argument("args", nargs="*")
    a = ap.parse_args()
    link = Link(a.interface)
    hello = link.hello()
    path = a.state or os.path.expanduser(
        f"~/.config/benchpod-bridge/{mac_str(link.bridge).replace(':', '')}.json")
    link.state = State(path)
    link.state.learn_bridge_pub(link.bridge_pub)
    if link.paired and link.state.key is None and a.command not in ("hello", "pair"):
        raise SystemExit(f"bridge is paired with another device and {path} has no key: "
                         "press PAIR on the bridge, then run `pair`")
    if a.command == "hello":
        print("\n".join(describe(hello)))
    elif a.command == "pair":
        secret = secrets.token_bytes(32)
        pub = x25519_public(secret)
        # Unpaired: unsigned (trust on first use). Paired with us: signed, rotates the key.
        msg = link.request(PAIR, tlv(T_PUBKEY, pub), sign=link.paired and link.state.key is not None)
        r, m = result_of(msg)
        if r:
            raise SystemExit(f"error: {RESULTS[r] if r < len(RESULTS) else r} {m}")
        bpub = first(msg["tlvs"], T_PUBKEY)
        link.state.secret, link.state.bridge_pub = secret, bpub
        link.state.key, link.state.counter = pair_key(secret, bpub, pub, bpub), 0
        link.state.save()
        print(f"ok: paired, key in {path}")
    elif a.command == "status":
        print("\n".join(describe(link.request(STATUS))))
    elif a.command == "scan":
        print("\n".join(describe(link.request(SCAN, timeout=10, retries=1))))
    elif a.command == "set-wifi":
        if len(a.args) not in (1, 2):
            raise SystemExit("set-wifi <ssid> [passphrase]")
        if not link.state.key:
            raise SystemExit("not paired with this bridge: run `pair` first")
        body = tlv(T_SSID, a.args[0])
        if len(a.args) == 2 and a.args[1]:  # sealed: the passphrase never crosses the cable in clear
            body += tlv(T_ENC_PASSPHRASE, seal_passphrase(link.state.key, secrets.token_bytes(24), a.args[1]))
        check(link.request(SET_WIFI, body))
    elif a.command == "clear-wifi":
        check(link.request(CLEAR_WIFI))
    elif a.command == "reboot":
        check(link.request(REBOOT))
    elif a.command == "factory-reset":
        check(link.request(FACTORY_RESET))
    elif a.command == "ota":
        if len(a.args) != 1:
            raise SystemExit("ota <image.bin>")
        ota(link, a.args[0])


if __name__ == "__main__":
    sys.exit(main())
