#!/usr/bin/env python3
"""The rules for code that runs from the W25Q (src/xip_image.h), checked after every link.

  usage: check_xip_refs.py build/bench_pod_stm32.elf build
         check_xip_refs.py --self-test     (links two small ELFs and checks the checker on them)

1. Internal code and data never refer to an XIP symbol, except the table (xip_table): no direct
   branch into .xip, and no word in .text/.rodata/.data/.isr_vector that equals an XIP symbol's
   address (a function pointer or a constant's address). In .text the words are the data words
   the assembler marks with $d mapping symbols (the literal pools, which objdump prints as
   ".word"), not every halfword: two adjacent instructions read as one word can equal an address
   in an image (an IT then a MOVS reads 0x2009bfxx, inside the RAM-linked image), a false hit
   that comes and goes with the compiler version and the layout. A hit prints the disassembly
   around it.
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
OBJECTS = ("xip_mod_*.o", "xip_prog_mod.o", "prog_updi.o", "prog_pic.o", "prog_c2.o", "prog_tpi.o", "prog_pdi.o", "prog_sbw.o")


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


def cur_at(dis_lines, k):
    """The function (or section label) a disassembly line belongs to."""
    for ln in reversed(dis_lines[:k]):
        m = re.match(r"^[0-9a-f]{8} <(.+)>:", ln)
        if m:
            return m.group(1)
    return "?"


SELF_TEST_LD = """
SECTIONS {
  .text 0x08000000 : { *(.text*) }
  .rodata : { *(.rodata*) }
  .xip 0x2009b000 : { *(.xip) }
  .xip_prog 0x90241000 : { *(.xip_prog) }
}
"""
# Both images: the table first (its 12-byte header and 64 entries), then the XIP function wait at
# 0x2009b10c.
SELF_TEST_XIP = """
    .syntax unified
    .thumb
    .section .xip,"ax"
    .global xip_table, __xip_start, wait
__xip_start:
xip_table:
    .space 12 + 4 * 64
    .type wait, %function
    .thumb_func
wait:
    bx lr
    .section .xip_prog,"ax"
    .global xip_prog_table, __xip_prog_start
__xip_prog_start:
xip_prog_table:
    .space 12 + 4 * 64
"""
WAIT = 0x2009b10c
SELF_TEST_CASES = (
    # (name, internal code, expected to fail, message the failure must carry)
    ("a literal-pool word holding wait's address", """
    .syntax unified
    .thumb
    .text
    .global _start
    .thumb_func
_start:
    ldr r0, =wait
    bx r0
    .ltorg
""", True, "holds the address of wait (a literal-pool word"),
    ("a .rodata word holding wait's address", """
    .syntax unified
    .thumb
    .text
    .global _start
    .thumb_func
_start:
    bx lr
    .section .rodata
    .word wait
""", True, "holds the address of wait"),
    # Two instructions, at .text+2: as one word they read 0x2009b10c, wait's address. They are
    # code, not data, so this links clean (the halfword scan up to v64 flagged it).
    ("two instructions that read as wait's address", """
    .syntax unified
    .thumb
    .text
    .global _start
    .thumb_func
_start:
    nop
    .inst.n 0xb10c
    .inst.n 0x2009
    bx lr
""", False, None),
)


def self_test():
    import contextlib
    import io
    prefix = OBJDUMP[:-len("objdump")] if OBJDUMP.endswith("objdump") else "arm-none-eabi-"
    fails = 0
    with tempfile.TemporaryDirectory() as d:
        def path(n):
            return os.path.join(d, n)
        open(path("t.ld"), "w").write(SELF_TEST_LD)
        open(path("xip.s"), "w").write(SELF_TEST_XIP)
        subprocess.run([prefix + "as", "-mcpu=cortex-m33", "-o", path("xip.o"), path("xip.s")], check=True)
        for i, (name, src, want_fail, want_msg) in enumerate(SELF_TEST_CASES):
            open(path(f"c{i}.s"), "w").write(src)
            subprocess.run([prefix + "as", "-mcpu=cortex-m33", "-o", path(f"c{i}.o"), path(f"c{i}.s")], check=True)
            elf = path(f"c{i}.elf")
            subprocess.run([prefix + "ld", "-T", path("t.ld"), "-o", elf, path(f"c{i}.o"), path("xip.o")], check=True)
            if not want_fail:
                # the case must really contain the look-alike word at a halfword offset
                b = section_bytes(elf, ".text")
                if struct.unpack_from("<I", b, 2)[0] != WAIT:
                    print(f"check_xip_refs self-test: '{name}': the instructions do not read as {WAIT:#x}")
                    fails += 1
                    continue
            out = io.StringIO()
            with contextlib.redirect_stdout(out):
                rc = main(elf, d)
            ok = (rc != 0) == want_fail and (want_msg is None or want_msg in out.getvalue())
            if not ok:
                fails += 1
                print(f"check_xip_refs self-test: '{name}': want {'FAIL' if want_fail else 'pass'}, got rc {rc}:")
                print(out.getvalue())
    if fails:
        print(f"check_xip_refs self-test: {fails} of {len(SELF_TEST_CASES)} cases wrong")
        return 1
    print(f"check_xip_refs self-test: {len(SELF_TEST_CASES)} cases ok")
    return 0


def main(elf, build):
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
    dis_lines = dis.splitlines()
    text_words = []  # (address, value, line index) of each data word in .text ($d: literal pools)
    dsec = None
    for k, line in enumerate(dis_lines):
        m = re.match(r"^Disassembly of section (\S+):", line)
        if m:
            dsec = m.group(1)
            continue
        m = re.match(r"^[0-9a-f]{8} <(.+)>:", line)
        if m:
            cur = m.group(1)
            continue
        m = re.match(r"^\s*([0-9a-f]+):\s+\.word\s+0x([0-9a-f]+)", line)
        if m:
            if dsec == ".text":
                text_words.append((int(m.group(1), 16), int(m.group(2), 16), k))
            continue
        m = re.match(r"^\s*[0-9a-f]+:\s+(b\S*|cb\S*)\s+([0-9a-f]+)\s", line)
        if m and any(lo <= int(m.group(2), 16) < hi for lo, hi in ranges):
            bad.append(f"branch from {cur} into an XIP image ({m.group(2)})")

    if ".text" in secs:
        vma, size = secs[".text"]
        b = section_bytes(elf, ".text")
        for a, v, k in text_words:
            # objdump's value must be the bytes there: a guard against a misparsed line
            if struct.unpack_from("<I", b, a - vma)[0] != v:
                bad.append(f".text+{a - vma:#x} ({a:#010x}): objdump says .word {v:#010x}, the bytes disagree")
                continue
            w = v & ~1
            if w in all_syms and w not in allowed:
                ctx = "\n".join("      " + ln for ln in dis_lines[max(0, k - 12):k + 3])
                bad.append(f".text+{a - vma:#x} ({a:#010x}) holds the address of {all_syms[w]} "
                           f"(a literal-pool word; the code around it, in {cur_at(dis_lines, k)}):\n{ctx}")
        if not text_words and size > 0x100:
            bad.append(".text has no .word in objdump's output: the literal pools were not found, so the check cannot run")
    for name in INTERNAL:
        if name not in secs or name == ".text":
            continue
        vma, _ = secs[name]
        b = section_bytes(elf, name)
        for i in range(0, len(b) - 3, 4):
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
    if sys.argv[1:] == ["--self-test"]:
        sys.exit(self_test())
    sys.exit(main(sys.argv[1], sys.argv[2]))
