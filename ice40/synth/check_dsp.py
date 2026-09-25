#!/usr/bin/env python3
"""Post-synthesis check: every dsp_counter SB_MAC16 survived synthesis intact.

RTL simulation cannot see what synthesis does to an explicitly instantiated primitive: yosys'
ice40_dsp pass once rewrote dsp_counter's SB_MAC16 cells (CLK tied to 0, load inputs zeroed,
multiplier output selected) and every bench still passed.  This reads the synthesized JSON and
fails unless there is one SB_MAC16 per dsp_counter instance in the RTL, each still clocked, with
its load enable connected and the accumulator configuration dsp_counter sets.

usage: check_dsp.py <netlist.json> <src dir>
"""
import json
import pathlib
import re
import sys

WANT = {"TOPOUTPUT_SELECT": 1, "BOTOUTPUT_SELECT": 1, "TOPADDSUB_UPPERINPUT": 0,
        "BOTADDSUB_UPPERINPUT": 0, "TOPADDSUB_CARRYSELECT": 2, "BOTADDSUB_CARRYSELECT": 1,
        "TOPADDSUB_LOWERINPUT": 0, "BOTADDSUB_LOWERINPUT": 0}


def pint(v):
    return int(v, 2) if isinstance(v, str) else int(v)


def main():
    net, src = sys.argv[1], pathlib.Path(sys.argv[2])
    rtl = sum(len(re.findall(r"^\s*dsp_counter\s*#", f.read_text(), re.M)) for f in src.glob("*.v"))
    mods = json.load(open(net))["modules"]
    top = next(m for m in mods.values() if m.get("attributes", {}).get("top"))
    cells = {n: c for n, c in top["cells"].items() if c["type"] == "SB_MAC16" and n.endswith(".mac")}
    errs = []
    if len(cells) != rtl:
        errs.append(f"{len(cells)} dsp_counter SB_MAC16 cells in the netlist, {rtl} dsp_counter instances in the RTL")
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
    print(f"check_dsp: {net}: {len(cells)} dsp_counter SB_MAC16 cells intact")


if __name__ == "__main__":
    main()
