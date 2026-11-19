#!/usr/bin/env python3
"""The board revision is decided in one place: board_caps.h.

board_rev.c reads the straps and latches board_caps(); everything else asks board_caps() for what
the board has (src/board_caps.h explains why).  This check fails when code outside those files
compares the revision itself: board_rev_get(), board_rev_is_v3() or a BOARD_REV_V* constant.
Comments are ignored, so a comment may still name a revision.
"""
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
SCAN = ("src", "port", "bsp")
ALLOWED = {
    "src/board_rev.c", "src/board_rev.h",      # reads the straps, latches the caps
    "src/board_caps.c", "src/board_caps.h",    # the revision -> capabilities map
    "bsp/board_pins.h",                        # the strap pins and their nominal voltages
}
GATE = re.compile(r"\b(board_rev_get|board_rev_is_v3|BOARD_REV_V\w*)\b")
# Comments and string/char literals, kept as newlines so line numbers stay right.
STRIP = re.compile(r'//[^\n]*|/\*.*?\*/|"(?:\\.|[^"\\\n])*"|\'(?:\\.|[^\'\\\n])*\'', re.S)


def blank(m):
    return re.sub(r"[^\n]", " ", m.group(0))


def main():
    bad = []
    for d in SCAN:
        for p in sorted((ROOT / d).rglob("*")):
            if p.suffix not in (".c", ".h"):
                continue
            rel = p.relative_to(ROOT).as_posix()
            if rel in ALLOWED:
                continue
            code = STRIP.sub(blank, p.read_text(errors="replace"))
            for n, line in enumerate(code.split("\n"), 1):
                m = GATE.search(line)
                if m:
                    bad.append(f"{rel}:{n}: {m.group(1)}")
    if bad:
        print("board revision gates: FAIL (ask board_caps() instead; see src/board_caps.h)")
        for b in bad:
            print("  " + b)
        return 1
    print("board revision gates: PASS (the revision is read only in board_rev/board_caps)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
