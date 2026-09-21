#!/usr/bin/env python3
"""Full-range ADC calibration via an external 0-5 V source.

The board DAC only reaches ~3.1 V, so to calibrate the whole ADC range we drive
the ADC input from an external adjustable source; the DMM measures truth and the
board reports the averaged ADC count. Fit V = a + b*count over the full range.

Two source modes:
  - manual (default): you dial the supply; the script captures each setpoint.
  - programmable: implement set_source_v() for your SCPI PSU/calibrator.

  python3 adc_cal.py --host 192.168.1.50            # manual knob
  python3 adc_cal.py --host 192.168.1.50 --programmable
"""
import argparse
import json
import sys
import time

from cal_client import BenchClient, linfit


def set_source_v(volts):
    """Hook for a programmable 0-5 V source. Fill in for your SCPI PSU/calibrator.
    Return True once the source has settled at `volts`."""
    raise NotImplementedError("programmable source not configured — run without --programmable")


def main():
    ap = argparse.ArgumentParser(description="Full-range ADC calibration via external source")
    ap.add_argument("--host", required=True)
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--nplc", type=float, default=10.0)
    ap.add_argument("--adc-avg", type=int, default=64)
    ap.add_argument("--settle", type=float, default=0.5)
    ap.add_argument("--programmable", action="store_true",
                    help="drive a SCPI source via set_source_v(); default is manual knob")
    ap.add_argument("--targets", default="0.1,0.5,1,1.5,2,2.5,3,3.5,4,4.5,4.9",
                    help="comma volts: setpoints (programmable) or rough guide (manual)")
    ap.add_argument("--out", default="adc_cal.json")
    args = ap.parse_args()

    c = BenchClient(args.host, args.port)
    print(f"[cal] server health: {c.health()}")
    print(f"[cal] DMM: {c.dmm_idn()}")
    targets = [float(x) for x in args.targets.split(",") if x.strip()]

    volts, counts = [], []

    def capture():
        time.sleep(args.settle)
        v = c.dmm_dc(nplc=args.nplc)
        adc = c.board_adc(avg=args.adc_avg)["count"]
        volts.append(v)
        counts.append(adc)
        print(f"  captured: DMM={v:.6f} V  ADC={adc:.2f}")

    if args.programmable:
        for t in targets:
            if not set_source_v(t):
                print(f"  source did not settle at {t} V; skipping", file=sys.stderr)
                continue
            capture()
    else:
        print("\nMANUAL mode: set the external source, then press Enter to capture.")
        print(f"Spread points across 0-5 V (suggested: {targets}). Type 'd' + Enter to finish.\n")
        i = 0
        while True:
            hint = f" ~{targets[i]} V" if i < len(targets) else ""
            s = input(f"[{len(volts)} captured] set source{hint}, Enter=capture (d=done): ").strip().lower()
            if s == "d":
                break
            try:
                capture()
            except Exception as e:  # noqa: BLE001
                print(f"  capture failed: {e}", file=sys.stderr)
            i += 1

    if len(volts) < 2:
        print("need >=2 points to fit", file=sys.stderr)
        sys.exit(1)

    a, b, r2, inl = linfit(counts, volts)
    result = {
        "type": "adc_cal_fullrange",
        "adc": {"v_eq": "V = a + b*count", "a": a, "b": b, "r2": r2, "max_resid_v": inl},
        "points": [{"dmm_v": v_, "adc": cnt} for v_, cnt in zip(volts, counts)],
    }
    with open(args.out, "w") as f:
        json.dump(result, f, indent=2)
    print(f"\n[cal] ADC : V = {a:.6f} + {b:.6f}*count  "
          f"R2={r2:.6f}  max|resid|={inl * 1000:.2f} mV  (full 0-5 V)")
    print(f"[cal] wrote {args.out}")


if __name__ == "__main__":
    main()
