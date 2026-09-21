#!/usr/bin/env python3
"""RP2350B bench-pod console driver (pyserial).

Runs on the Pi (board console USB-serial attached) and is wrapped by
bench_server.py. Mirrors the proven char-by-char send + read-until-quiet from
the dac_adc_* scripts (the console RX path can drop characters if fed too fast).
A MockBoard mirrors the v1 DAC->ADC transfer so the API/cal pipeline can be
tested with no hardware.
"""
import re
import time

ADC_RE = re.compile(r"ADC probe samples:\s*([0-9,]+)")


class BoardConsole:
    def __init__(self, port, baud=115200):
        import serial  # pyserial
        self.ser = serial.Serial(port, baud, timeout=0.3)
        time.sleep(0.3)
        self._send("", settle=0.3)

    def _send(self, cmd, settle=0.4):
        self.ser.reset_input_buffer()
        for ch in cmd + "\r":
            self.ser.write(ch.encode())
            self.ser.flush()
            time.sleep(0.005)
        time.sleep(settle)
        out = self.ser.read(self.ser.in_waiting or 1).decode("utf-8", "replace")
        while True:
            time.sleep(0.1)
            chunk = self.ser.read(self.ser.in_waiting or 0).decode("utf-8", "replace")
            if not chunk:
                break
            out += chunk
        return out

    def ping(self):
        out = self._send("spi-ping")
        return ("FPGA OK" in out or "reachable" in out), out

    def dac_set(self, code, channel=0, settle=0.4):
        out = self._send(f"dac-set {int(code)} {int(channel)}", settle=settle)
        ok = ("readback confirmed" in out) or ("held at" in out)
        return ok, out

    def adc_probe(self):
        m = ADC_RE.search(self._send("adc-probe"))
        return [int(x) for x in m.group(1).split(",")] if m else []

    def adc_avg(self, n=32):
        acc = []
        while len(acc) < n:
            s = self.adc_probe()
            if not s:  # RX dropped — stop rather than spin
                break
            acc.extend(s)
        mean = sum(acc) / len(acc) if acc else None
        return mean, acc

    def raw(self, line):
        return self._send(line)


class MockBoard:
    """Reproduces the measured v1 DAC->ADC transfer so dac_cal.py + the HTTP
    pipeline can be validated end-to-end with no hardware."""

    def __init__(self):
        self.last_code = 0

    def ping(self):
        return True, "[mock] FPGA OK gateware version 5"

    def dac_set(self, code, channel=0, settle=0.0):  # noqa: ARG002
        self.last_code = int(code)
        return True, f"[mock] DAC ch{channel} held at {code}/255"

    def _count(self):
        return max(0, min(255, round(0.582 * self.last_code - 13.69)))

    def adc_probe(self):
        return [self._count()] * 16

    def adc_avg(self, n=32):
        c = self._count()
        return float(c), [c] * max(1, n)

    def commanded_voltage(self):
        # measured DAC cal: code 0 -> -196 mV, code 255 -> 3.1 V (linear)
        return (-196.0 + self.last_code * 3296.0 / 255.0) / 1000.0

    def raw(self, line):
        return f"[mock] {line}"
