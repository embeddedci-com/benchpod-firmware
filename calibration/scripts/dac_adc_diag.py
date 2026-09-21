#!/usr/bin/env python3
"""Diagnose the DAC->ADC loopback: dump the full 16-sample ADC array per DAC
code, repeated, to tell noise from unstable sample-and-hold settling."""
import re, time, serial

PORT, BAUD = "/dev/tty.usbserial-0001", 115200
SAMP_RE = re.compile(r"ADC probe samples:\s*([0-9,]+)")

def send(ser, cmd, settle=0.5):
    ser.reset_input_buffer()
    for c in cmd + "\r":
        ser.write(c.encode()); ser.flush(); time.sleep(0.006)
    time.sleep(settle)
    out = ser.read(ser.in_waiting or 1).decode("utf-8", "replace")
    while True:
        time.sleep(0.12)
        chunk = ser.read(ser.in_waiting or 0).decode("utf-8", "replace")
        if not chunk: break
        out += chunk
    return out

def probe_samples(ser):
    out = send(ser, "adc-probe")
    m = SAMP_RE.search(out)
    return [int(x) for x in m.group(1).split(",")] if m else None

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.3); time.sleep(0.3)
    send(ser, "", settle=0.3)
    for code in [0, 32, 64, 128, 192, 255]:
        send(ser, f"dac-set {code} 0", settle=0.4)
        print(f"\n--- dac-set {code} (ch0) ---")
        for rep in range(4):
            s = probe_samples(ser)
            if s is None:
                print(f"  rep{rep}: <no parse>")
            else:
                mn, mx = min(s), max(s)
                print(f"  rep{rep}: {s}   (min={mn} max={mx} spread={mx-mn})")
    send(ser, "dac-stop")
    ser.close()

if __name__ == "__main__":
    main()
