#!/usr/bin/env python3
"""DMM sweep of one BenchPod DAC output path in 16-bit codes (rev3 and later).

The 34401A sits on the path's output SMA, unloaded.  dac_out takes volts and reports the 16-bit
code it chose, so the sweep records (code16, DMM volts) pairs and fits V = a + b*code16.

    python3 dac16_sweep.py --path 5v --pod 192.168.1.220 --out ../rev3/dac16_5v.json

Needs the embeddedci SDK (pip install embeddedci) and pyusb for the Mac-direct DMM driver.
"""
import argparse
import datetime
import json
import time

from dmm import Agilent34401A

FULL = {"3v3": (0.0, 3.30), "5v": (0.0, 5.00), "12v": (-11.9, 11.9)}
FLOOR = {"3v3": [0.0, 0.002, 0.005, 0.01, 0.02, 0.05], "5v": [0.0, 0.002, 0.005, 0.01, 0.02, 0.05],
         "12v": []}


def linfit(xs, ys):
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    b = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    return my - b * mx, b


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--path", choices=list(FULL), required=True)
    ap.add_argument("--pod", default="192.168.1.220")
    ap.add_argument("--steps", type=int, default=41)
    ap.add_argument("--settle", type=float, default=0.4)
    ap.add_argument("--samples", type=int, default=3)
    ap.add_argument("--fine", type=int, default=16, help="consecutive codes at mid-scale")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    from embeddedci.benchpod import BenchPod
    pod = BenchPod(args.pod)
    dmm = Agilent34401A(nplc=10, vrange=100 if args.path == "12v" else 10)
    lo, hi = FULL[args.path]

    targets = list(FLOOR[args.path])
    targets += [lo + (hi - lo) * i / (args.steps - 1) for i in range(args.steps)]

    def point(volts):
        r = pod.command({"cmd": "dac_out", "path": args.path, "volts": round(volts, 6)})
        time.sleep(args.settle)
        v = dmm.read_dc(samples=args.samples)
        print(f"{volts:9.5f} V -> code16 {r['code16']:5d}  pod {r['uv'] / 1e6:9.6f}  dmm {v:9.6f}", flush=True)
        return {"req": volts, "code16": r["code16"], "pod_v": r["uv"] / 1e6, "dmm_v": v}

    sweep = [point(v) for v in targets]
    # Consecutive 16-bit codes at mid-scale: step by the current fit's LSB.
    mid = next(p for p in sweep if p["req"] >= (lo + hi) / 2)
    lsb = (hi - lo) / 65535 * 1.0007
    fine = [point(mid["pod_v"] + k * lsb) for k in range(args.fine)]

    pod.command({"cmd": "dac_stop"})
    pod.command({"cmd": "dac_out", "path": "off"})
    dmm.close()

    out = {
        "type": "dac16_sweep", "board_rev": "v3", "pod": args.pod, "path": args.path,
        "date": datetime.datetime.now().isoformat(timespec="seconds"),
        "method": "HP 34401A on the output SMA, unloaded, NPLC 10, mean of %d; dac_out volts, "
                  "code16 as reported" % args.samples,
        "sweep": sweep, "fine": fine,
    }
    with open(args.out, "w") as f:
        json.dump(out, f, indent=1)
    print("wrote", args.out)


if __name__ == "__main__":
    main()
