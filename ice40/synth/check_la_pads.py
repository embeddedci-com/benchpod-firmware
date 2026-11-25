#!/usr/bin/env python3
"""Post-synthesis check: every LA pad is a registered-output, registered-enable SB_IO.

Up to gateware v63 la_bank drove each LA pad with a combinational output and output enable
(PIN_TYPE 6'b1010_01).  When a channel changed hands, the two cones behind the pad settled at
different, placement-dependent times, and the pad could put a few-ns pulse on the DUT's line.  No
timing report shows that (a combinational pad path has no capture register), so it slipped past
every seed gate.  Since v64 both are flops in the IO cell (PIN_TYPE 6'b1101_01, rising clk edge),
so the race cannot happen on any placement.  This reads the synthesized JSON and fails unless there
is one SB_IO per LA channel in la_bank, each with that PIN_TYPE, clocked on the rising edge by a
real clock net (not a constant).

usage: check_la_pads.py <netlist.json> [channels]   (channels: 14 on the v2 board)
"""
import json
import re
import sys

PAD = re.compile(r"la_bank_i\.g_io\[(\d+)\]\.io_i$")
WANT_PIN_TYPE = 0b110101   # PIN_OUTPUT_REGISTERED_ENABLE_REGISTERED + PIN_INPUT


def pint(v):
    return int(v, 2) if isinstance(v, str) else int(v)


def main():
    net = sys.argv[1]
    want_n = int(sys.argv[2]) if len(sys.argv) > 2 else 14
    mods = json.load(open(net))["modules"]
    top = next(m for m in mods.values() if m.get("attributes", {}).get("top"))
    pads = {}
    for name, c in top["cells"].items():
        m = PAD.search(name)
        if m:
            pads[int(m.group(1))] = (name, c)
    errs = []
    if sorted(pads) != list(range(want_n)):
        errs.append(f"found la_bank pads {sorted(pads)}, want 0..{want_n - 1}")
    for ch, (name, c) in sorted(pads.items()):
        if c["type"] != "SB_IO":
            errs.append(f"LA{ch + 1} ({name}) is a {c['type']}, not an SB_IO")
            continue
        p = c["parameters"]
        pt = pint(p.get("PIN_TYPE", "0"))
        if pt != WANT_PIN_TYPE:
            errs.append(f"LA{ch + 1} ({name}) PIN_TYPE 6'b{pt:06b}, want 6'b{WANT_PIN_TYPE:06b} "
                        "(registered output + registered output enable)")
        if pint(p.get("NEG_TRIGGER", "0")):
            errs.append(f"LA{ch + 1} ({name}) has NEG_TRIGGER set (the pad flops must load on "
                        "the rising edge, with the engines)")
        clk = c["connections"].get("OUTPUT_CLK", [])
        if not clk or any(isinstance(b, str) for b in clk):
            errs.append(f"LA{ch + 1} ({name}) OUTPUT_CLK is {clk or 'unconnected'}, want the clk net")
    if errs:
        print(f"check_la_pads: {net}: FAILED", file=sys.stderr)
        for e in errs:
            print(f"  {e}", file=sys.stderr)
        print("  A combinational LA pad lets a hand-over glitch reach the DUT (see la_bank.v).",
              file=sys.stderr)
        sys.exit(1)
    print(f"check_la_pads: {net}: {len(pads)} LA pads, registered output + enable")


if __name__ == "__main__":
    main()
