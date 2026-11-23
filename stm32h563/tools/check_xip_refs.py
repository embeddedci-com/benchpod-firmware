#!/usr/bin/env python3
"""The rules for code that runs from the W25Q (src/xip_image.h), checked after every link.

  usage: check_xip_refs.py build/bench_pod_stm32.elf build

1. Internal code and data never refer to an XIP symbol, except the table (xip_table): no direct
   branch into .xip, and no word in .text/.rodata/.data/.isr_vector that equals an XIP symbol's
   address (a function pointer or a constant's address).
2. Inside .xip, function addresses appear only in the table: XIP code hands no callback to
   anything that could run it outside a session.
3. XIP objects (xip_mod_*.o) have no .data or .bss.
"""
import glob
import os
import re
import struct
import subprocess
import sys
import tempfile

NM = os.environ.get("NM", "arm-none-eabi-nm")
OBJDUMP = os.environ.get("OBJDUMP", "arm-none-eabi-objdump")
OBJCOPY = os.environ.get("OBJCOPY", "arm-none-eabi-objcopy")
SIZE = os.environ.get("SIZE", "arm-none-eabi-size")
INTERNAL = (".isr_vector", ".text", ".rodata", ".data", ".xip_hash")


def sections(elf):
    out = subprocess.run([OBJDUMP, "-h", elf], capture_output=True, text=True, check=True).stdout
    return {m.group(1): (int(m.group(3), 16), int(m.group(2), 16))
            for m in re.finditer(r"^\s*\d+\s+(\S+)\s+([0-9a-f]+)\s+([0-9a-f]+)\s", out, re.M)}


def section_bytes(elf, name):
    with tempfile.NamedTemporaryFile(delete=False) as t:
        path = t.name
    try:
        subprocess.run([OBJCOPY, "-O", "binary", "-j", name, elf, path], check=True)
        return open(path, "rb").read()
    finally:
        os.unlink(path)


# (name, output section, table symbol, start symbol): one entry per image (src/xip_image.h)
IMAGES = (
    ("usb", ".xip", "xip_table", "__xip_start"),
    ("prog", ".xip_prog", "xip_prog_table", "__xip_prog_start"),
)
# The XIP objects of each image, by object file pattern, for the "no static state" rule.
OBJECTS = ("xip_mod_*.o", "xip_prog_mod.o", "prog_updi.o", "prog_pic.o", "prog_c2.o", "prog_tpi.o", "prog_pdi.o")


def symbols(elf):
    """Symbols by section: {section: [(addr, name, is_function)]}, from objdump -t. (nm cannot say
    which of two images that share one RAM region a symbol belongs to.)"""
    out = subprocess.run([OBJDUMP, "-t", elf], capture_output=True, text=True, check=True).stdout
    res = {}
    for line in out.splitlines():
        if "\t" not in line or not re.match(r"^[0-9a-f]{8} ", line):
            continue
        left, right = line.split("\t", 1)
        sec, flags = left[17:].strip(), left[9:16]
        parts = right.split()
        if sec.startswith(".xip") and len(parts) >= 2:
            res.setdefault(sec, []).append((int(left[:8], 16), parts[-1], "F" in flags))
    return res


def main():
    elf, build = sys.argv[1], sys.argv[2]
    secs = sections(elf)
    syms_by_sec = symbols(elf)
    bad = []
    all_syms = {}    # address -> name, over every image (internal code may hold none of them)
    allowed = set()  # the tables and the images' first bytes
    ranges = []      # (lo, hi) of each image
    info = []
    for img, sec, table, start in IMAGES:
        if sec not in secs:
            print(f"check_xip_refs: no {sec} section")
            return 1
        lo, size = secs[sec]
        ranges.append((lo, lo + size))
        funcs, names = set(), {}
        for a, n, is_f in syms_by_sec.get(sec, []):
            if lo <= a < lo + size:
                all_syms.setdefault(a & ~1, n)
                names.setdefault(a & ~1, set()).add(n)
                if is_f:
                    funcs.add(a & ~1)
        allowed |= {a for a, n in names.items() if n & {table, start}}
        tlo = next((a for a, n in names.items() if table in n), lo)
        info.append((img, sec, lo, size, funcs, all_syms, tlo))

    # Internal code and data: no branch into any image, no word equal to an image symbol's address.
    dis = subprocess.run([OBJDUMP, "-d", "--no-show-raw-insn"] + sum((["-j", s] for s in (".text", ".data", ".isr_vector") if s in secs), []) + [elf],
                         capture_output=True, text=True, check=True).stdout
    cur = None
    for line in dis.splitlines():
        m = re.match(r"^[0-9a-f]{8} <(.+)>:", line)
        if m:
            cur = m.group(1)
            continue
        m = re.match(r"^\s*[0-9a-f]+:\s+(b\S*|cb\S*)\s+([0-9a-f]+)\s", line)
        if m and any(lo <= int(m.group(2), 16) < hi for lo, hi in ranges):
            bad.append(f"branch from {cur} into an XIP image ({m.group(2)})")

    for name in INTERNAL:
        if name not in secs:
            continue
        vma, _ = secs[name]
        b = section_bytes(elf, name)
        for i in range(0, len(b) - 3, 2 if name == ".text" else 4):
            w = struct.unpack_from("<I", b, i)[0] & ~1
            if w in all_syms and w not in allowed:
                bad.append(f"{name}+{i:#x} ({vma + i:#010x}) holds the address of {all_syms[w]}")

    # Inside an image: function pointers only in its table.
    for img, sec, lo, size, funcs, _, tlo in info:
        tsize = 12 + 4 * 64
        b = section_bytes(elf, sec)
        for i in range(0, len(b) - 3, 2):
            w = struct.unpack_from("<I", b, i)[0]
            if (w & 1) and (w & ~1) in funcs and not (tlo <= lo + i < tlo + tsize):
                bad.append(f"{sec}+{i:#x} holds a pointer to the XIP function {all_syms[w & ~1]} (callbacks are not allowed)")

    # Images that do not share addresses (memory-mapped builds) must not refer to each other at all;
    # the RAM overlay shares addresses, so the rule there is the table rule above plus the linker's
    # section-by-object assignment. Cross references show up as branches into the other image:
    for img, sec, lo, size, funcs, _, tlo in info:
        for img2, sec2, lo2, size2, funcs2, _, _ in info:
            if img == img2 or lo == lo2:
                continue
            b = section_bytes(elf, sec)
            for i in range(0, len(b) - 3, 2):
                w = struct.unpack_from("<I", b, i)[0]
                if lo2 <= (w & ~1) < lo2 + size2 and not (tlo <= lo + i < tlo + 12 + 4 * 64):
                    bad.append(f"{sec}+{i:#x} refers into the {img2} image")

    for pat in OBJECTS:
        for obj in sorted(glob.glob(os.path.join(build, pat))):
            out = subprocess.run([SIZE, "-A", obj], capture_output=True, text=True, check=True).stdout
            for line in out.splitlines():
                p = line.split()
                if len(p) >= 2 and p[0].startswith((".data", ".bss")) and p[1].isdigit() and int(p[1]) > 0:
                    bad.append(f"{os.path.basename(obj)} has {p[0]} ({p[1]} bytes): XIP modules keep no static state")

    if bad:
        print("check_xip_refs: FAIL (rules in src/xip_image.h)")
        for x in bad:
            print("  " + x)
        return 1
    for img, sec, lo, size, funcs, _, _ in info:
        print(f"check_xip_refs: {img}: {size} bytes in {sec}, only the table is referenced")
    return 0


if __name__ == "__main__":
    sys.exit(main())
