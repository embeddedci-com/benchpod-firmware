#!/usr/bin/env python3
"""Characterize the DAC->ADC loopback: per-code noise/variance, linearity/INL,
and a software-stepped triangle 'wave' reconstructed through the loop."""
import re, time, math, statistics, serial

PORT, BAUD = "/dev/tty.usbserial-0001", 115200
SAMP_RE = re.compile(r"ADC probe samples:\s*([0-9,]+)")

def send(ser, cmd, settle=0.35):
    ser.reset_input_buffer()
    for c in cmd + "\r":
        ser.write(c.encode()); ser.flush(); time.sleep(0.005)
    time.sleep(settle)
    out = ser.read(ser.in_waiting or 1).decode("utf-8", "replace")
    while True:
        time.sleep(0.1)
        chunk = ser.read(ser.in_waiting or 0).decode("utf-8", "replace")
        if not chunk: break
        out += chunk
    return out

def probe_samples(ser):
    m = SAMP_RE.search(send(ser, "adc-probe"))
    return [int(x) for x in m.group(1).split(",")] if m else []

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.3); time.sleep(0.3)
    send(ser, "", settle=0.3)

    # ---- 1) DC sweep: per-code mean + spread (noise), linearity/INL ----
    print("== DC transfer: per-code noise + linearity ==")
    print(f"{'code':>4} {'mean':>6} {'spread':>6}   (spread = max-min over 32 samples)")
    codes, means = [], []
    for code in range(0, 256, 8):
        send(ser, f"dac-set {code} 0", settle=0.3)
        s = probe_samples(ser) + probe_samples(ser)   # 2 captures = 32 samples
        if not s: continue
        mean = statistics.mean(s); spread = max(s) - min(s)
        codes.append(code); means.append(mean)
        print(f"{code:>4} {mean:>6.1f} {spread:>6}")

    # linear fit over the active region (mean rising, not stuck at floor)
    act = [(c, m) for c, m in zip(codes, means) if m > 1 and c >= 24]
    cx = [c for c, _ in act]; my = [m for _, m in act]
    n = len(cx); mx = sum(cx)/n; mm = sum(my)/n
    b = sum((c-mx)*(v-mm) for c, v in zip(cx, my)) / sum((c-mx)**2 for c in cx)
    a = mm - b*mx
    res = [v - (a + b*c) for c, v in zip(cx, my)]
    ss_res = sum(r*r for r in res); ss_tot = sum((v-mm)**2 for v in my)
    r2 = 1 - ss_res/ss_tot
    print(f"\nfit (active region): ADC = {a:.2f} + {b:.4f}*code")
    print(f"  R^2={r2:.5f}  max|INL|={max(abs(x) for x in res):.2f} LSB  "
          f"span={min(my):.0f}..{max(my):.0f} of 255 ({(max(my)-min(my))/255*100:.0f}% of ADC range)")

    # ---- 2) software triangle 'wave' through the loop ----
    print("\n== triangle wave pushed through DAC->ADC (code 24..200..24) ==")
    up = list(range(24, 201, 16)); tri = up + up[::-1][1:]
    rec = []
    for code in tri:
        send(ser, f"dac-set {code} 0", settle=0.25)
        s = probe_samples(ser)
        rec.append((code, s[-1] if s else None))
    amax = max(v for _, v in rec if v is not None)
    for i, (code, v) in enumerate(rec):
        bar = "#" * int((v or 0) / amax * 48)
        print(f"  t{i:02d} code={code:>3} adc={v:>3} |{bar}")
    send(ser, "dac-stop")
    ser.close()

if __name__ == "__main__":
    main()
