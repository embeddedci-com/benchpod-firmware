#!/usr/bin/env python3
"""Direct DAC-output calibration — DMM (HP/Agilent 34401A over the NI GPIB-USB-HS,
via the in-repo ni-gpib-usb-hs driver) and the board (USB-serial console) both on
this host.  Sweeps ONE SMA output path, fits V = a + b*code, and merges the result
into dac_cal.json keyed by path (3v3/5v/12v), so any tool can convert:
    code -> volts : V = a + b*code
    volts -> code : code = (V - a) / b

Cal switches are left OFF (calsw 0 0 0 0): this measures the raw DAC output at the
SMA, unaffected by the ADC cal path.

    python3 dac_cal_direct.py --path 3v3
"""
import argparse, glob, json, os, statistics, sys, time
import serial

# in-repo NI GPIB driver (repo_root/ni-gpib-usb-hs)
_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, os.path.join(_REPO, "ni-gpib-usb-hs"))
from ni_gpib_usb_hs import Agilent34401A

# path -> (dacmux args, sel, DMM range volts).  The 12V output is DIFFERENTIAL:
# TO_12V (CTRL1 sel 2) is the signal and TO_12V_VMID (CTRL2 sel 0, via U48) is the
# reference — both feed the U51 diff-amp, so CTRL2 must also be enabled.
PATHS = {
    "3v3": ("1 0",     0, 10.0),
    "5v":  ("1 1",     1, 10.0),
    "12v": ("1 2 1 0", 2, 100.0),
}


def linfit(xs, ys):
    n = len(xs); mx = sum(xs) / n; my = sum(ys) / n
    b = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sum((x - mx) ** 2 for x in xs)
    a = my - b * mx
    resid = [y - (a + b * x) for x, y in zip(xs, ys)]
    ss_res = sum(r * r for r in resid); ss_tot = sum((y - my) ** 2 for y in ys)
    r2 = 1 - ss_res / ss_tot if ss_tot else 1.0
    return a, b, r2, max(abs(r) for r in resid)


def main():
    ap = argparse.ArgumentParser(description="Direct DAC-output cal via 34401A + serial")
    ap.add_argument("--path", required=True, choices=list(PATHS))
    ap.add_argument("--step", type=int, default=16)
    ap.add_argument("--reads", type=int, default=3, help="DMM reads per point (median)")
    ap.add_argument("--settle", type=float, default=0.4)
    ap.add_argument("--gpib", type=int, default=22)
    ap.add_argument("--serial", default="")
    ap.add_argument("--out", default=os.path.join(os.path.dirname(__file__), "..", "dac_cal.json"))
    args = ap.parse_args()
    dacmux_args, sel, rng = PATHS[args.path]

    dev = args.serial or sorted(glob.glob("/dev/cu.usbmodem*"))[0]
    p = serial.Serial(dev, 115200, timeout=0.2)

    def send(c, w):
        p.reset_input_buffer(); p.write((c + "\r\n").encode()); p.flush(); time.sleep(w)
        return p.read(8192).decode("utf-8", "replace")

    send("calsw 0 0 0 0", 0.4)          # cal path OFF: raw DAC output at the SMA
    send(f"dacmux {dacmux_args}", 0.5)  # route DAC -> this SMA path (+VMID for 12V)
    dmm = Agilent34401A(addr=args.gpib)
    print(f"[cal] DAC '{args.path}' (dacmux sel {sel}, DMM range {rng}V) | {dmm.idn()}")
    print(f"  {'code':>4} {'DMM_V':>11}")
    codes, volts = [], []
    for code in range(0, 256, args.step):
        send(f"dacraw {code}", 0.4); time.sleep(args.settle)
        v = statistics.median([dmm.voltage_dc(rng=rng) for _ in range(args.reads)])
        codes.append(code); volts.append(v)
        print(f"  {code:>4} {v:>11.6f}")
    dmm.close()
    send("dacraw 0", 0.4); send("path off", 0.4); p.close()

    a, b, r2, resid = linfit(codes, volts)
    out = os.path.abspath(args.out)
    data = {"type": "dac_cal", "paths": {}}
    if os.path.exists(out):
        try:
            data = json.load(open(out))
            data.setdefault("paths", {})
        except Exception:
            pass
    data["paths"][args.path] = {
        "sel": sel, "v_eq": "V = a + b*code", "code_eq": "code = (V - a)/b",
        "a": a, "b": b, "r2": r2, "max_resid_v": resid,
        "points": [[c, v] for c, v in zip(codes, volts)],
    }
    json.dump(data, open(out, "w"), indent=2)
    print(f"\n[cal] {args.path}: V = {a:.6f} + {b:.6f}*code   "
          f"R2={r2:.6f}  max|resid|={resid * 1000:.2f} mV")
    print(f"[cal] fullscale ~{a + b * 255:.4f} V ; wrote {out}")


if __name__ == "__main__":
    main()
