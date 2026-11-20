#!/usr/bin/env python3
"""What the board has is decided in one place: board_caps.h.

board_rev.c reads the straps and latches board_caps(); everything else asks board_caps() for what
the board has (src/board_caps.h explains why).  This check fails when code outside those files
compares the revision itself: board_rev_get(), board_rev_is_v3() or a BOARD_REV_V* constant.

The analog front end works the same way: board_variant.c decides it and latches it, with the
module EEPROM's facts, in board_caps()->analog.  The check also fails when code outside
board_variant/module_link reads that decision or those facts itself: a BOARD_ANALOG_* value,
module_adc_low_range(), module_current_in_100r(), or the old board_has_analog() and
board_analog_is_module() getters.

Comments and string literals are ignored, so a comment may still name them.
"""
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
SCAN = ("src", "port", "bsp")
# (what, the pattern, the files allowed to use it)
RULES = (
    ("board revision gates",
     re.compile(r"\b(board_rev_get|board_rev_is_v3|BOARD_REV_V\w*)\b"),
     {
         "src/board_rev.c", "src/board_rev.h",      # reads the straps, latches the caps
         "src/board_caps.c", "src/board_caps.h",    # the revision -> capabilities map
         "bsp/board_pins.h",                        # the strap pins and their nominal voltages
     }),
    ("analog front end gates",
     re.compile(r"\b(board_has_analog|board_analog_is_module|BOARD_ANALOG_(?!CAPS_)\w+"
                r"|module_adc_low_range|module_current_in_100r)\b"),
     {
         "src/board_variant.c", "src/board_variant.h",   # decides it, latches board_caps()->analog
         "src/module_link.c", "src/module_link.h",       # reads the module EEPROM
     }),
)
# Comments and string/char literals, kept as newlines so line numbers stay right.
STRIP = re.compile(r'//[^\n]*|/\*.*?\*/|"(?:\\.|[^"\\\n])*"|\'(?:\\.|[^\'\\\n])*\'', re.S)


def blank(m):
    return re.sub(r"[^\n]", " ", m.group(0))


def scan(files, gate, allowed):
    bad = []
    for rel, code in files:
        if rel in allowed:
            continue
        for n, line in enumerate(code.split("\n"), 1):
            for m in gate.finditer(line):
                bad.append(f"{rel}:{n}: {m.group(1)}")
    return bad


def main():
    files = []
    for d in SCAN:
        for p in sorted((ROOT / d).rglob("*")):
            if p.suffix in (".c", ".h"):
                files.append((p.relative_to(ROOT).as_posix(),
                              STRIP.sub(blank, p.read_text(errors="replace"))))
    rc = 0
    for what, gate, allowed in RULES:
        bad = scan(files, gate, allowed)
        if bad:
            print(f"{what}: FAIL (ask board_caps() instead; see src/board_caps.h)")
            for b in bad:
                print("  " + b)
            rc = 1
        else:
            print(f"{what}: PASS (read only in {', '.join(sorted({a.split('/')[-1].split('.')[0] for a in allowed}))})")
    return rc


if __name__ == "__main__":
    sys.exit(main())
