#!/usr/bin/env python3
"""Sweep the MCP4728 DAC and read the AD9280 ADC at each step, to characterise
the DAC->ADC transfer over an external BNC loopback cable (DAC-out -> ADC-in)."""
import re, time, serial

PORT, BAUD = "/dev/tty.usbserial-0001", 115200
ADC_RE = re.compile(r"ADC = (\d+)")

def send(ser, cmd, settle=0.6):
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

def probe(ser):
    out = send(ser, "adc-probe")
    m = ADC_RE.search(out)
    return int(m.group(1)) if m else None

def dac_mv(code8):
    v12 = (code8 * 4095 + 127) // 255
    return -196 + (v12 * (3100 - (-196))) // 4095

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.3); time.sleep(0.3)
    send(ser, "", settle=0.3)
    send(ser, "dac-stop")                       # zero all channels

    print("== ch0 fine sweep (the expected DAC BNC channel) ==")
    print(f"{'code':>4} {'~mV':>6} {'ADC':>4} {'ADC_hex':>7}")
    for v in list(range(0, 256, 16)) + [255]:
        send(ser, f"dac-set {v} 0", settle=0.4)
        a = probe(ser)
        print(f"{v:>4} {dac_mv(v):>6} {a if a is not None else '?':>4} "
              f"{('0x%02x'%a) if a is not None else '?':>7}")
    send(ser, "dac-set 0 0")

    print("\n== which channel is on the BNC? (set each to full-scale, others 0) ==")
    for ch in range(4):
        send(ser, "dac-stop")
        send(ser, f"dac-set 255 {ch}", settle=0.4)
        a = probe(ser)
        print(f"ch{ch} = 255 -> ADC {a}")
    send(ser, "dac-stop")
    ser.close()

if __name__ == "__main__":
    main()
