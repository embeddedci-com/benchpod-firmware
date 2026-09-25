#!/usr/bin/env python3
"""Post-synthesis check: every dsp_counter / dsp_counter2 SB_MAC16 survived synthesis intact.

RTL simulation cannot see what synthesis does to an explicitly instantiated primitive: yosys'
ice40_dsp pass once rewrote dsp_counter's SB_MAC16 cells (CLK tied to 0, load inputs zeroed,
multiplier output selected) and every bench still passed.  This reads the synthesized JSON and
fails unless there is one SB_MAC16 per dsp_counter / dsp_counter2 instance in the RTL, each still
clocked, with its load enable connected and the accumulator configuration its module sets.  The two
differ only in the top adder's carry-in: dsp_counter chains it to the bottom adder (one 32-bit
counter), dsp_counter2 ties it to 1 (two 16-bit counters).

usage: check_dsp.py <netlist.json> <src dir>
"""
import json
import pathlib
import re
import sys

WANT = {"TOPOUTPUT_SELECT": 1, "BOTOUTPUT_SELECT": 1, "TOPADDSUB_UPPERINPUT": 0,
        "BOTADDSUB_UPPERINPUT": 0, "BOTADDSUB_CARRYSELECT": 1,
        "TOPADDSUB_LOWERINPUT": 0, "BOTADDSUB_LOWERINPUT": 0}
# TOPADDSUB_CARRYSELECT per module: 2 = the bottom adder's carry (32-bit), 1 = constant 1 (dual 16)
TOP_CARRY = {"dsp_counter": 2, "dsp_counter2": 1}


def pint(v):
    return int(v, 2) if isinstance(v, str) else int(v)


def main():
    net, src = sys.argv[1], pathlib.Path(sys.argv[2])
    texts = [f.read_text() for f in src.glob("*.v")]
    rtl = {m: sum(len(re.findall(rf"^\s*{m}\s*#", t, re.M)) for t in texts) for m in TOP_CARRY}
    mods = json.load(open(net))["modules"]
    top = next(m for m in mods.values() if m.get("attributes", {}).get("top"))
    cells = {n: c for n, c in top["cells"].items() if c["type"] == "SB_MAC16" and n.endswith(".mac")}
    errs = []
    for m, carry in TOP_CARRY.items():
        n = sum(pint(c["parameters"].get("TOPADDSUB_CARRYSELECT", "0")) == carry for c in cells.values())
        if n != rtl[m]:
            errs.append(f"{n} {m} SB_MAC16 cells in the netlist, {rtl[m]} {m} instances in the RTL")
    if len(cells) != sum(rtl.values()):
        errs.append(f"{len(cells)} counter SB_MAC16 cells in the netlist, {sum(rtl.values())} in the RTL")
    for n, c in cells.items():
        conn, par = c["connections"], c["parameters"]
        if any(isinstance(b, str) for b in conn["CLK"]):
            errs.append(f"{n}: CLK is tied to a constant")
        # (the load VALUE may be constant — the LA region always starts at 0 — but the load
        #  ENABLE must not be tied off: a counter that can never load is dead)
        if all(isinstance(b, str) for b in conn["OLOADTOP"] + conn["OLOADBOT"]):
            errs.append(f"{n}: load enable OLOADTOP/OLOADBOT is tied to a constant")
        for k, v in WANT.items():
            if pint(par.get(k, "0")) != v:
                errs.append(f"{n}: {k}={pint(par.get(k, '0'))}, want {v}")
    if errs:
        print(f"check_dsp: {net}: FAIL", file=sys.stderr)
        for e in errs:
            print(f"  {e}", file=sys.stderr)
        sys.exit(1)
    print(f"check_dsp: {net}: {len(cells)} counter SB_MAC16 cells intact "
          f"({rtl['dsp_counter']} dsp_counter, {rtl['dsp_counter2']} dsp_counter2)")


if __name__ == "__main__":
    main()
