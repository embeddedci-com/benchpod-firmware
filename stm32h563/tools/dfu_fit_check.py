#!/usr/bin/env python3
"""Refuse to DFU an image the pod in DFU mode cannot hold.

The image's fw_info block (src/fw_info.h, offset 0x400: "BPFW", layout, min_flash_kb) says the
smallest flash it fits; the ST bootloader's `dfu-util -l` listing names the flash it has
("@Internal Flash   /0x08000000/256*08Kg" = 256 sectors of 8 KB). Images without the block were
all built for the 2 MB H563. Exit 1 when the image does not fit, 0 otherwise (also when the
listing cannot be read: then nothing is checked).
"""
import re
import subprocess
import sys


def needs_kb(path):
    data = open(path, "rb").read(0x410)
    if len(data) < 0x410 or data[0x400:0x404] != b"BPFW":
        return 2048
    kb = data[0x406] | data[0x407] << 8
    return kb or 2048


def flash_kb(dfu_util):
    try:
        out = subprocess.run([dfu_util, "-l"], capture_output=True, text=True, timeout=10).stdout
    except (OSError, subprocess.TimeoutExpired):
        return 0
    m = re.search(r"/0x08000000/([0-9*,KMBg ]+)", out)
    if not m:
        return 0
    total = 0
    for part in m.group(1).split(","):
        g = re.match(r"\s*(\d+)\*(\d+)([KMB ])", part)
        if not g:
            return 0
        n, size, unit = int(g.group(1)), int(g.group(2)), g.group(3)
        total += n * size * (1024 if unit == "M" else 1) // (1024 if unit in "B " else 1)
    return total


def main():
    image, dfu_util = sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else "dfu-util"
    need, have = needs_kb(image), flash_kb(dfu_util)
    if have == 0:
        print("dfu-fit: could not read the pod's flash size; not checking the image fits")
        return 0
    if need > have:
        print("dfu-fit: this image needs %d KB of flash and the pod has %d KB" % (need, have))
        return 1
    print("dfu-fit: image needs %d KB, pod has %d KB" % (need, have))
    return 0


if __name__ == "__main__":
    sys.exit(main())
