#!/usr/bin/env python3
"""ADC fit through an internal loopback (cal1 = 5 V path, cal2 = ±12 V path), rev3 and later.

Sets the DAC with 16-bit dac_out, routes the loopback, and records the ADC count (adc_read's
own burst, averaged over --reads) against the DAC voltage.  The reference voltage is the DMM
reading when --dmm is given (the DMM on that path's output SMA, if the loopback leaves it
connected; the DMM reads the output path, then the loopback reads the same code), else the 16-bit DAC fit passed with --dac-a/--dac-b (V = a + b*code16).

    python3 adc16_sweep.py --source cal2 --dmm --out ../rev3/adc16_cal2.json
"""
import argparse
import datetime
import json
import time

PATH = {"cal1": ("5v", 0.05, 5.0), "cal2": ("12v", -11.9, 11.9)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", choices=list(PATH), required=True)
    ap.add_argument("--pod", default="192.168.1.220")
    ap.add_argument("--steps", type=int, default=25)
    ap.add_argument("--reads", type=int, default=8)
    ap.add_argument("--settle", type=float, default=0.4)
    ap.add_argument("--dmm", action="store_true")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    from embeddedci.benchpod import BenchPod
    pod = BenchPod(args.pod)
    dmm = None
    if args.dmm:
        from dmm import Agilent34401A
        dmm = Agilent34401A(nplc=10, vrange=100 if args.source == "cal2" else 10)
    path, lo, hi = PATH[args.source]

    rows = []
    for i in range(args.steps):
        volts = lo + (hi - lo) * i / (args.steps - 1)
        r = pod.command({"cmd": "dac_out", "path": path, "volts": round(volts, 6)})
        time.sleep(args.settle)
        v = dmm.read_dc(samples=2) if dmm else None   # output SMA, before the loopback cuts it off
        pod.command({"cmd": "analog_path", "path": args.source})
        time.sleep(args.settle)
        counts = [pod.command({"cmd": "adc_read", "source": args.source})["count"]
                  for _ in range(args.reads)]
        row = {"req": volts, "code16": r["code16"], "pod_v": r["uv"] / 1e6, "dmm_v": v,
               "counts": counts}
        rows.append(row)
        print(f"{volts:8.4f} V code16 {r['code16']:5d} dmm {v if v is None else round(v, 6)} "
              f"counts {min(counts)}..{max(counts)}", flush=True)

    pod.command({"cmd": "dac_out", "path": "off"})
    if dmm:
        dmm.close()
    with open(args.out, "w") as f:
        json.dump({"type": "adc16_sweep", "board_rev": "v3", "pod": args.pod,
                   "source": args.source,
                   "date": datetime.datetime.now().isoformat(timespec="seconds"),
                   "rows": rows}, f, indent=1)
    print("wrote", args.out)


if __name__ == "__main__":
    main()
