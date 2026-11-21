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


def main():
    elf, build = sys.argv[1], sys.argv[2]
    secs = sections(elf)
    if ".xip" not in secs:
        print("check_xip_refs: no .xip section")
        return 1
    lo, size = secs[".xip"]
    hi = lo + size
    syms, funcs, names = {}, set(), {}
    for line in subprocess.run([NM, "-n", elf], capture_output=True, text=True, check=True).stdout.splitlines():
        p = line.split()
        if len(p) == 3:
            a = int(p[0], 16)
            if lo <= a < hi:
                syms.setdefault(a & ~1, p[2])
                names.setdefault(a & ~1, set()).add(p[2])
                if p[1] in "tTwW":
                    funcs.add(a & ~1)
    # The table, and the image's start (the same address): internal code reads both.
    allowed = {a for a, n in names.items() if n & {"xip_table", "__xip_start"}}
    bad = []

    dis = subprocess.run([OBJDUMP, "-d", "--no-show-raw-insn"] + sum((["-j", s] for s in (".text", ".data", ".isr_vector") if s in secs), []) + [elf],
                         capture_output=True, text=True, check=True).stdout
    cur = None
    for line in dis.splitlines():
        m = re.match(r"^[0-9a-f]{8} <(.+)>:", line)
        if m:
            cur = m.group(1)
            continue
        m = re.match(r"^\s*[0-9a-f]+:\s+(b\S*|cb\S*)\s+([0-9a-f]+)\s", line)
        if m and lo <= int(m.group(2), 16) < hi:
            bad.append(f"branch from {cur} into .xip ({m.group(2)})")

    for name in INTERNAL:
        if name not in secs:
            continue
        vma, _ = secs[name]
        b = section_bytes(elf, name)
        for i in range(0, len(b) - 3, 2 if name == ".text" else 4):
            w = struct.unpack_from("<I", b, i)[0] & ~1
            if w in syms and w not in allowed:
                bad.append(f"{name}+{i:#x} ({vma + i:#010x}) holds the address of {syms[w]}")

    tbl = sorted(allowed)
    tlo = tbl[0] if tbl else lo
    tsize = 12 + 4 * 64
    b = section_bytes(elf, ".xip")
    for i in range(0, len(b) - 3, 2):
        w = struct.unpack_from("<I", b, i)[0]
        if (w & 1) and (w & ~1) in funcs and not (tlo <= lo + i < tlo + tsize):
            bad.append(f".xip+{i:#x} holds a pointer to the XIP function {syms[w & ~1]} (callbacks are not allowed)")

    for obj in sorted(glob.glob(os.path.join(build, "xip_mod_*.o"))):
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
    print(f"check_xip_refs: {size} bytes in .xip, {len(syms)} symbols, only the table is referenced")
    return 0


if __name__ == "__main__":
    sys.exit(main())
