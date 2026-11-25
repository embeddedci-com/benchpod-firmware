#!/usr/bin/env python3
"""Every indirect OCTOSPI1 command goes through xspi_guard (src/xspi_guard.h).

While the W25Q is memory-mapped for XIP, an indirect command must first take the controller out of
memory-mapped mode. xspi_guard_enter()/exit() do that, and they sit in exactly two functions:
psram.c xspi_xfer() and w25q.c fxfer_cs(). This check fails when a HAL_XSPI command, receive,
transmit, abort, polling or memory-mapped call, or a direct OCTOSPI1 register access, appears
anywhere else, unless the (file, function) pair is allowed below with its reason.

Comments and string literals are ignored.
"""
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
CALL = re.compile(r"\b(HAL_XSPI_(Command|Receive|Receive_DMA|Transmit|Transmit_DMA|Abort|AutoPolling"
                  r"|AutoPolling_IT|MemoryMapped|Command_IT|Receive_IT|Transmit_IT)|OCTOSPI1\s*->)")
ALLOWED = {
    ("src/psram.c", "xspi_xfer"): "the PSRAM's command path, guarded",
    ("src/w25q.c", "fxfer_cs"): "the W25Q's command path, guarded",
    # ota_commit's RAM-resident copy reads the staged image with the internal flash erased; an
    # XIP session is refused while an update is staged or committing (heavy_in_flight).
    ("src/ota_commit.c", "ram_psram_read_burst"): "OTA commit, never during XIP",
}
# Functions that own the mapping itself (they run with the guard's hooks).
MAPPERS = {"src/xip_session.c"}

FUNC = re.compile(r"^[A-Za-z_][\w \*]*?\b(\w+)\s*\([^;]*$")


def strip(text):
    text = re.sub(r"/\*.*?\*/", lambda m: "\n" * m.group(0).count("\n"), text, flags=re.S)
    text = re.sub(r"//[^\n]*", "", text)
    return re.sub(r'"(\\.|[^"\\])*"', '""', text)


def main():
    bad = []
    for path in sorted((ROOT / "src").glob("*.c")):
        rel = str(path.relative_to(ROOT))
        if rel in MAPPERS:
            continue
        func = None
        for n, line in enumerate(strip(path.read_text()).split("\n"), 1):
            m = FUNC.match(line)
            if m and not line.startswith(("if", "for", "while", "switch", "return")):
                func = m.group(1)
            if CALL.search(line) and (rel, func) not in ALLOWED:
                bad.append(f"  {rel}:{n} in {func}(): {line.strip()}")
    if bad:
        print("xspi guard: FAIL (an indirect OCTOSPI1 access outside xspi_guard, see src/xspi_guard.h)")
        print("\n".join(bad))
        return 1
    print(f"xspi guard: PASS ({len(ALLOWED)} allowed paths)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
