#!/usr/bin/env python3
"""After the link: extract the W25Q image (.xip, src/xip_image.h), patch its length and SHA-256
into the firmware (.xip_hash), and publish it as a blob.

  usage: xip_finish.py build/bench_pod_stm32.elf build/blobs

Writes <blobs>/blob-xip.bin and adds or replaces the "xip" entry in <blobs>/blobs-manifest.json
(written before the link by tools/blob_manifest.py), so installers send it like the other blobs.
The ELF is patched in place; the .bin and .hex are made from it afterwards.
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


def main():
    elf, blobs = sys.argv[1], sys.argv[2]
    os.makedirs(blobs, exist_ok=True)
    out = os.path.join(blobs, "blob-xip.bin")
    subprocess.run([OBJCOPY, "-O", "binary", "-j", ".xip", elf, out], check=True)
    data = open(out, "rb").read()
    sha = hashlib.sha256(data).digest()
    with tempfile.NamedTemporaryFile(delete=False) as t:
        t.write(struct.pack("<II", MAGIC, len(data)) + sha)
    try:
        subprocess.run([OBJCOPY, "--update-section", ".xip_hash=" + t.name, elf], check=True)
    finally:
        os.unlink(t.name)

    man_path = os.path.join(blobs, "blobs-manifest.json")
    if os.path.isfile(man_path):
        man = json.load(open(man_path))
        man["blobs"] = [b for b in man["blobs"] if b.get("name") != "xip"]
        man["blobs"].append({"name": "xip", "file": "blob-xip.bin", "size": len(data),
                             "sha256": sha.hex(), "version": 0})
        with open(man_path, "w") as f:
            json.dump(man, f, indent=2)
            f.write("\n")
    print(f"xip: {len(data)} bytes in the W25Q image (blob-xip.bin), sha256 {sha.hex()[:16]}...")
    return 0


if __name__ == "__main__":
    sys.exit(main())
