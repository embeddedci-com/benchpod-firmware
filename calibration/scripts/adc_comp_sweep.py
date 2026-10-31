#!/usr/bin/env python3
"""ADC input divider compensation check: a sine sweep through the cal1 loopback, rev3 and later.

The ADC input divider (R90 top, R91 bottom) is compensated with C140 across R90 and C72 across
R91.  Flat needs R90 x C_top = R91 x C_bottom; a wrong top capacitor shows as a gain step between
low and high frequencies.  This sweeps a sine from the 5 V DAC stage through the cal1 loopback
(the same divider, no cable, nothing on the SMAs), fits the amplitude per frequency, and fits the
divider model to the gain curve:

    H(f) = |1 + jw Rt Ct| / |1 + jw (Rt||Rb)(Ct + Cb)| x DAC hold sinc x one extra roll-off pole

The sweep only fixes the RATIO of the legs (Ct/Cb): the reported Ct assumes a bottom leg of
--c-bottom (C72 + strays).  The verdict uses the ratio, so it holds whatever the strays are.

Measured 2026-10-20 on baea06 (rev3 with the 2-6 pF trimmer): top 11.4 pF against 107 pF, flat
needs 9.7 pF, HF gain +16 % over DC.  v3r2 and the analog module fix it with 9.1 pF + 100 pF 1 %.

--range low (analog module v3r4 and later): relay K5 swaps the bottom leg for R182 910k || C269
47 pF and puts C271 47 pF across R90, so the model is top 910k || 56.4 pF (9.1 + 47 + 0.3 stray)
against bottom 910k || ~55 pF (47 + ~8 stray), flat at equal capacitors.  The sine stays under
4 V (offset 2.0 V, peak 1.8 V by default): the low range's full scale is +-5.46 V.  Run it on both
ranges on the first board to settle C269/C271 (C270 is a DNP trim pad).

    python3 adc_comp_sweep.py --pod usb --out ../rev3/comp_sweep.json
    python3 adc_comp_sweep.py --pod usb --range low --out ../v3r4/comp_sweep_low.json
    python3 adc_comp_sweep.py --fit ../rev3/comp_sweep.json          # refit a saved sweep
    python3 adc_comp_sweep.py --pod usb --store                      # also record it on the pod

--store records the HF/DC gain and the top-leg capacitance in the pod's calibration, for reference
(firmware 3.9.5+, normal range): nothing scales a reading with them.

Prints a JSON summary as the last line (for scripts/ship-pod); exits 1 when the HF/DC gain is
off by more than --max-error.  Leaves the DAC stopped and the analog path off.
"""
import argparse
import datetime
import json
import math
import sys
import time

import numpy as np

FREQS = [200, 1000, 3000, 6000, 10000, 15000, 20000, 30000, 45000, 60000, 80000, 100000,
         120000, 150000]
FS = 400000.0          # ADC rate the sweep runs at
DAC_RATE = 888888.0    # DAC update rate: its sample-and-hold sinc


# Per range: the divider and the sine that stays inside the range.
RANGES = {
    "normal": {"r_top": 910e3, "r_bottom": 82.5e3, "c_bottom": 107e-12, "amplitude": 2.4, "offset": 2.5},
    "low":    {"r_top": 910e3, "r_bottom": 910e3,  "c_bottom": 55e-12,  "amplitude": 1.8, "offset": 2.0},
}


def route(pod, rng):
    """The cal1 loopback in `rng`.  The raw command, so any SDK version works; the reply says
    the range the pod is in, so a board without the low range fails here, not in the fit."""
    r = pod.command({"cmd": "analog_path", "path": "cal1", "range": rng})
    if r.get("range", "normal") != rng:
        raise SystemExit(f"the pod is in the {r.get('range')} range, not {rng}")


def measure(pod, freqs, samples, amplitude, offset=2.5, rng="normal"):
    """Sine amplitude in ADC counts per frequency, through the cal1 loopback."""
    if offset + amplitude > (4.0 if rng == "low" else 5.0) or offset - amplitude < 0.0:
        raise SystemExit(f"the sine {offset - amplitude:.2f}..{offset + amplitude:.2f} V does not fit "
                         f"the {rng} range on the 5 V output (low: under 4 V)")
    rows = []
    route(pod, rng)
    try:
        for f in freqs:
            pod.generate("sine", freq_hz=f, amplitude=amplitude, offset=offset, dac_path="5v",
                         route=False)
            time.sleep(0.3)
            cap = pod.capture_adc(samples, sample_rate_hz=FS)
            pod.dac_stop()
            fs = cap.sample_rate_hz or FS
            x = np.array(cap.counts, dtype=float)
            x = np.where(x > 32767, x - 65536, x)       # counts are two's complement
            f_fit, amp, resid = fit_sine(x, fs)
            rows.append({"f": f, "f_fit": f_fit, "amp": amp, "resid_rms": resid, "fs": fs})
            print(f"{f:7d} Hz fit {f_fit:10.2f} amp {amp:9.3f} counts resid {resid:.2f}",
                  flush=True)
    finally:
        try:
            pod.dac_stop()
        finally:
            pod.analog_path("off")
    return rows


def fit_sine(x, fs):
    """Frequency, amplitude and residual RMS of the sine in x: FFT peak, then least squares."""
    t = np.arange(len(x)) / fs
    n = len(x) * 8
    spec = np.abs(np.fft.rfft((x - x.mean()) * np.hanning(len(x)), n))
    f0 = np.fft.rfftfreq(n, 1 / fs)[int(np.argmax(spec[1:])) + 1]
    best = None
    for ff in np.linspace(f0 - fs / n * 2, f0 + fs / n * 2, 81):
        a = np.column_stack([np.sin(2 * np.pi * ff * t), np.cos(2 * np.pi * ff * t),
                             np.ones_like(t)])
        c, res, _, _ = np.linalg.lstsq(a, x, rcond=None)
        r = float(res[0]) if len(res) else 0.0
        if best is None or r < best[0]:
            best = (r, ff, c)
    r, ff, c = best
    return float(ff), float(math.hypot(c[0], c[1])), math.sqrt(r / len(x))


def model(f, rt, rb, ct, cb, fp):
    w = 2 * np.pi * f
    rp = rt * rb / (rt + rb)
    h = np.abs(1 + 1j * w * rt * ct) / np.abs(1 + 1j * w * rp * (ct + cb))
    return h * np.abs(np.sinc(f / DAC_RATE)) / np.abs(1 + 1j * f / fp)


def fit_divider(rows, rt, rb, cb):
    """Top-leg C (for a bottom leg of cb) and the extra pole that best match the gain curve."""
    f = np.array([r["f_fit"] for r in rows])
    m = np.array([r["amp"] for r in rows]) / rows[0]["amp"]
    best = None
    for ct in np.arange(0.2 * rb * cb / rt, 2.5 * rb * cb / rt, 0.005 * rb * cb / rt):
        for fp in (2e5, 3e5, 4e5, 5e5, 7e5, 1e6, 2e6, 1e9):
            e = float(np.sum((model(f, rt, rb, ct, cb, fp) - m) ** 2))
            if best is None or e < best[0]:
                best = (e, ct, fp)
    e, ct, fp = best
    need = rb * cb / rt
    return {
        "c_top_pf": round(ct * 1e12, 2),
        "c_bottom_pf": round(cb * 1e12, 1),
        "c_top_flat_pf": round(need * 1e12, 2),
        "hf_dc_gain": round(ct / (ct + cb) / (rb / (rt + rb)), 4),
        "extra_pole_khz": round(fp / 1e3),
        "fit_rms_pct": round(math.sqrt(e / len(f)) * 100, 2),
        "points": [{"f": round(float(fi)), "gain": round(float(mi), 4),
                    "model": round(float(model(fi, rt, rb, ct, cb, fp)), 4)}
                   for fi, mi in zip(f, m)],
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--pod", default="usb", help="connection: usb, a host, or embeddedci:<name>")
    ap.add_argument("--fit", metavar="JSON", help="refit a saved sweep instead of measuring")
    ap.add_argument("--out", help="write the sweep and the fit to this JSON file")
    ap.add_argument("--store", action="store_true", help="record the result on the pod (normal range)")
    ap.add_argument("--freqs", help="comma list of frequencies in Hz")
    ap.add_argument("--samples", type=int, default=32768)
    ap.add_argument("--range", choices=list(RANGES), default="normal",
                    help="ADC input range: low = K5 on, analog module v3r4+ (sets the defaults below)")
    ap.add_argument("--amplitude", type=float, help="sine peak in volts (normal 2.4, low 1.8)")
    ap.add_argument("--offset", type=float, help="sine centre in volts (normal 2.5, low 2.0)")
    ap.add_argument("--r-top", type=float, help="R90 (910k)")
    ap.add_argument("--r-bottom", type=float, help="R91 82.5k, low range R182 910k")
    ap.add_argument("--c-bottom", type=float,
                    help="bottom leg C + strays in farads: C72 107p, low range C269 + ~8p = 55p")
    ap.add_argument("--max-error", type=float, default=0.05,
                    help="fail when the HF/DC gain is off by more than this (0.05 = 5 %%)")
    args = ap.parse_args()

    if args.fit:
        with open(args.fit) as fh:
            saved = json.load(fh)
        rows, pod_name = saved["rows"], saved.get("pod", "")
        if args.range == "normal" and saved.get("range"):
            args.range = saved["range"]   # refit with the range it was measured in
    for k, v in RANGES[args.range].items():
        if getattr(args, k) is None:
            setattr(args, k, v)
    if not args.fit:
        from embeddedci.benchpod import BenchPod
        freqs = [int(v) for v in args.freqs.split(",")] if args.freqs else FREQS
        pod_name = args.pod
        pod = BenchPod(args.pod)
        rows = measure(pod, freqs, args.samples, args.amplitude, args.offset, args.range)

    fit = fit_divider(rows, args.r_top, args.r_bottom, args.c_bottom)
    ok = bool(abs(fit["hf_dc_gain"] - 1) <= args.max_error)
    for p in fit["points"]:
        print(f"{p['f']:9d} Hz gain {p['gain']:.4f} model {p['model']:.4f}")
    print(f"{args.range} range: top leg {fit['c_top_pf']} pF against {fit['c_bottom_pf']} pF, flat needs "
          f"{fit['c_top_flat_pf']} pF: HF/DC gain {fit['hf_dc_gain']:.3f} "
          f"(fit {fit['fit_rms_pct']} % rms) -> {'ok' if ok else 'OFF'}")
    if args.out:
        with open(args.out, "w") as fh:
            json.dump({"type": "adc_comp_sweep", "pod": pod_name, "source": "cal1", "range": args.range,
                       "date": datetime.datetime.now().isoformat(timespec="seconds"),
                       "r_top": args.r_top, "r_bottom": args.r_bottom, "rows": rows,
                       "fit": fit}, fh, indent=1)
        print("wrote", args.out)
    if args.store:
        if args.fit or args.range != "normal":
            ap.error("--store takes a fresh sweep in the normal range")
        pod.store_adc_compensation(fit["hf_dc_gain"], fit["c_top_pf"])
        print("recorded on the pod")
    summary = {k: v for k, v in fit.items() if k != "points"}
    print(json.dumps({"ok": ok, "range": args.range, **summary}))
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
