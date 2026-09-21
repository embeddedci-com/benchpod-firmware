#!/usr/bin/env python3
"""ADC noise characterization at fixed DC: separate fast (within-capture) from
slow (across-capture) variation, and histogram the codes. Helps tell real ADC
noise from boundary dither / DAC drift / measurement artifact."""
import re, time, statistics, collections, serial

PORT, BAUD = "/dev/tty.usbserial-0001", 115200
SAMP_RE = re.compile(r"ADC probe samples:\s*([0-9,]+)")

def send(ser, cmd, settle=0.2):
    ser.reset_input_buffer()
    for c in cmd + "\r":
        ser.write(c.encode()); ser.flush(); time.sleep(0.004)
    time.sleep(settle)
    out = ser.read(ser.in_waiting or 1).decode("utf-8", "replace")
    while True:
        time.sleep(0.08)
        chunk = ser.read(ser.in_waiting or 0).decode("utf-8", "replace")
        if not chunk: break
        out += chunk
    return out

def probe16(ser):
    m = SAMP_RE.search(send(ser, "adc-probe"))
    return [int(x) for x in m.group(1).split(",")] if m else None

def characterize(ser, code, n=120):
    send(ser, f"dac-set {code} 0", settle=0.3)
    caps = []
    for _ in range(n):
        c = probe16(ser)
        if c: caps.append(c)
    alls = [v for c in caps for v in c]
    means = [statistics.mean(c) for c in caps]
    within = statistics.mean(statistics.pstdev(c) for c in caps)   # fast noise
    across = statistics.pstdev(means)                              # slow drift/LF
    hist = collections.Counter(alls)
    print(f"\n-- dac-set {code} (n={len(caps)} captures, {len(alls)} samples) --")
    print(f"   mean={statistics.mean(alls):.2f}  p-p={max(alls)-min(alls)} LSB  "
          f"overall_sigma={statistics.pstdev(alls):.3f} LSB")
    print(f"   within-capture sigma (fast/12MS-s) = {within:.3f} LSB")
    print(f"   across-capture sigma (slow/LF)     = {across:.3f} LSB")
    print("   histogram (code:count): " +
          "  ".join(f"{k}:{v}" for k, v in sorted(hist.items())))

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.3); time.sleep(0.3)
    send(ser, "", settle=0.3)
    characterize(ser, 80)    # stable mid-code (earlier spread 0)
    characterize(ser, 184)   # boundary code (earlier fractional mean ~93.1)
    characterize(ser, 40)    # boundary code (earlier ~9.5)
    send(ser, "dac-stop")
    ser.close()

if __name__ == "__main__":
    main()
