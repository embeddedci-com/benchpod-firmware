#!/usr/bin/env python3
"""After the link: extract the W25Q images (.xip and .xip_prog, src/xip_image.h), patch each one's
length and SHA-256 into the firmware (.xip_hash, .xip_prog_hash), and publish them as blobs.

  usage: xip_finish.py build/bench_pod_stm32.elf build/blobs

Writes <blobs>/blob-xip.bin and blob-xip_prog.bin and adds or replaces the "xip" and "xip_prog"
entries in <blobs>/blobs-manifest.json (written before the link by tools/blob_manifest.py), so
installers send them like the other blobs. The ELF is patched in place; the .bin and .hex are
made from it afterwards.
"""
import hashlib
import json
import os
import struct
import subprocess
import sys
import tempfile

OBJCOPY = os.environ.get("OBJCOPY", "arm-none-eabi-objcopy")
MAGIC = 0x48504958  # "XIPH", xip_hash_t.magic

# (blob name = slot name, image section, the hash section in internal flash)
IMAGES = (("xip", ".xip", ".xip_hash"), ("xip_prog", ".xip_prog", ".xip_prog_hash"))


def finish(elf, blobs, name, section, hash_section):
    out = os.path.join(blobs, "blob-%s.bin" % name)
    subprocess.run([OBJCOPY, "-O", "binary", "-j", section, elf, out], check=True)
    data = open(out, "rb").read()
    sha = hashlib.sha256(data).digest()
    with tempfile.NamedTemporaryFile(delete=False) as t:
        t.write(struct.pack("<III", MAGIC, len(data), 0) + sha)   # magic, len, version 0, sha256
    try:
        subprocess.run([OBJCOPY, "--update-section", hash_section + "=" + t.name, elf], check=True)
    finally:
        os.unlink(t.name)
    return {"name": name, "file": "blob-%s.bin" % name, "size": len(data), "sha256": sha.hex(), "version": 0}


def main():
    elf, blobs = sys.argv[1], sys.argv[2]
    os.makedirs(blobs, exist_ok=True)
    entries = [finish(elf, blobs, *img) for img in IMAGES]

    man_path = os.path.join(blobs, "blobs-manifest.json")
    if os.path.isfile(man_path):
        man = json.load(open(man_path))
        names = {e["name"] for e in entries}
        man["blobs"] = [b for b in man["blobs"] if b.get("name") not in names]
        man["blobs"].extend(entries)
        with open(man_path, "w") as f:
            json.dump(man, f, indent=2)
            f.write("\n")
    for e in entries:
        print(f"{e['name']}: {e['size']} bytes in the W25Q image (blob-{e['name']}.bin), "
              f"sha256 {e['sha256'][:16]}...")
    return 0


if __name__ == "__main__":
    sys.exit(main())
