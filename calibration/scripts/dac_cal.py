#!/usr/bin/env python3
"""DAC calibration (and lower-range ADC cal) against the DMM, via the bench server.

Sweeps the board DAC, reads true volts from the DMM and the averaged ADC count
at each code, then fits two lines:
  DAC:  V = a_dac + b_dac * code   (to command a voltage: code = (V - a)/b)
  ADC:  V = a_adc + b_adc * count  (lower ~0-3.1 V range, since the DAC is the source)

Runs on the host:
  python3 dac_cal.py --host 192.168.1.50 [--port 8080] [--channel 0]
"""
import argparse
import json
import time

from cal_client import BenchClient, linfit


def main():
    ap = argparse.ArgumentParser(description="DAC calibration via the bench server")
    ap.add_argument("--host", required=True, help="Raspberry Pi bench-server IP")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--channel", type=int, default=0)
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--stop", type=int, default=255)
    ap.add_argument("--step", type=int, default=8)
    ap.add_argument("--nplc", type=float, default=10.0)
    ap.add_argument("--adc-avg", type=int, default=32)
    ap.add_argument("--settle", type=float, default=0.3, help="DAC settle (s) before reading")
    ap.add_argument("--out", default="dac_cal.json")
    args = ap.parse_args()

    c = BenchClient(args.host, args.port)
    print(f"[cal] server health: {c.health()}")
    print(f"[cal] DMM: {c.dmm_idn()}")

    codes, volts, counts = [], [], []
    print(f"\n{'code':>4} {'DMM_V':>11} {'ADC':>7}")
    for code in range(args.start, args.stop + 1, args.step):
        c.board_dac(code, args.channel)
        time.sleep(args.settle)
        v = c.dmm_dc(nplc=args.nplc)
        adc = c.board_adc(avg=args.adc_avg)["count"]
        codes.append(code)
        volts.append(v)
        counts.append(adc)
        print(f"{code:>4} {v:>11.6f} {('%.2f' % adc) if adc is not None else '   -':>7}")

    a_dac, b_dac, r2_dac, inl_dac = linfit(codes, volts)

    # ADC fit only where the ADC is above its floor (DAC-reachable lower range)
    act = [(cnt, v) for cnt, v in zip(counts, volts) if cnt and cnt > 0]
    adc_fit = None
    if len(act) >= 2:
        xs = [cnt for cnt, _ in act]
        ys = [v for _, v in act]
        a_adc, b_adc, r2_adc, inl_adc = linfit(xs, ys)
        adc_fit = {
            "v_eq": "V = a + b*count",
            "a": a_adc, "b": b_adc, "r2": r2_adc, "max_resid_v": inl_adc,
            "range": "DAC-reachable lower range (~0-3.1 V)",
        }

    result = {
        "type": "dac_cal",
        "channel": args.channel,
        "dac": {"v_eq": "V = a + b*code", "a": a_dac, "b": b_dac,
                "r2": r2_dac, "max_resid_v": inl_dac},
        "adc_low": adc_fit,
        "points": [{"code": c_, "dmm_v": v_, "adc": a_}
                   for c_, v_, a_ in zip(codes, volts, counts)],
    }
    with open(args.out, "w") as f:
        json.dump(result, f, indent=2)

    print(f"\n[cal] DAC : V = {a_dac:.6f} + {b_dac:.6f}*code   "
          f"R2={r2_dac:.6f}  max|resid|={inl_dac * 1000:.2f} mV")
    if adc_fit:
        print(f"[cal] ADC : V = {adc_fit['a']:.6f} + {adc_fit['b']:.6f}*count  "
              f"R2={adc_fit['r2']:.6f}  max|resid|={adc_fit['max_resid_v'] * 1000:.2f} mV  (lower range)")
    print(f"[cal] wrote {args.out}")


if __name__ == "__main__":
    main()
