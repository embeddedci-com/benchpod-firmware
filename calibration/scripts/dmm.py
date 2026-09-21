#!/usr/bin/env python3
"""Agilent/Keysight 34401A DMM driver over GPIB (NI GPIB-USB-HS, user-space libusb).

Talks to the 34401A through an NI GPIB-USB-HS using the pure-Python user-space
driver in ni_gpib_usb.py — NO linux-gpib, NO NI software, NO kernel module. This
runs directly on the calibration host (incl. macOS/Apple Silicon), so the DMM,
board console, and flashing can all live on one machine (the Pi bench-head is no
longer required for GPIB).

Background: linux-gpib's ni_usb kernel driver could not drive this (genuine)
adapter on the bench Pi; a from-scratch libusb port does — see ni_gpib_usb.py.

Requires:  brew install libusb  &&  pip install pyusb
A MockDMM (no hardware) is provided for exercising the cal pipeline offline.
"""


class DMMError(RuntimeError):
    pass


class Agilent34401A:
    """34401A DC-volts reader. Default GPIB primary address 22, 10 V range.

    `board` is accepted for backward compatibility and ignored (there is no
    linux-gpib board index anymore); the controller uses GPIB address 0.
    """

    def __init__(self, gpib_addr=22, board=0, nplc=10, vrange=10):
        try:
            from ni_gpib_usb import NIUSBGPIB, GpibError
        except ImportError as e:
            raise DMMError(
                "pyusb not found (import usb failed). Install it: "
                "`brew install libusb && pip install pyusb`."
            ) from e
        try:
            self.gpib = NIUSBGPIB(my_pad=0)
        except GpibError as e:
            raise DMMError(f"opening NI GPIB-USB-HS failed: {e}") from e
        self.addr = gpib_addr
        self.nplc = nplc
        self.vrange = vrange
        self._configure()

    def _write(self, s):
        self.gpib.write(self.addr, s.encode() if isinstance(s, str) else s)

    def _query(self, s):
        return self.gpib.query(self.addr, s)

    def _configure(self):
        # DC volts, fixed range (the 10 V range covers 0-5 V), high NPLC for
        # low reading noise, autozero on. Immediate trigger so READ? is one-shot.
        self._write("*CLS")
        self._write("SYST:REM")
        self._write(f"CONF:VOLT:DC {self.vrange}")
        self._write(f"VOLT:DC:NPLC {self.nplc}")
        self._write("ZERO:AUTO ON")
        self._write("TRIG:SOUR IMM")

    def idn(self):
        return self._query("*IDN?")

    def read_dc(self, nplc=None, samples=1):
        if nplc is not None and nplc != self.nplc:
            self._write(f"VOLT:DC:NPLC {nplc}")
            self.nplc = nplc
        vals = []
        for _ in range(max(1, int(samples or 1))):
            s = self._query("READ?")
            try:
                vals.append(float(s))
            except ValueError as e:
                raise DMMError(f"unparseable DMM reading: {s!r}") from e
        return sum(vals) / len(vals)

    def close(self):
        if getattr(self, "gpib", None) is not None:
            self.gpib.close()
            self.gpib = None


class MockDMM:
    """Synthetic DMM tied to a MockBoard so a full DAC-cal run yields a clean
    fit without hardware. Returns the voltage the board was last commanded to,
    plus a deterministic sub-LSB dither (no RNG, so runs are reproducible)."""

    def __init__(self, board=None):
        self.board = board
        self._n = 0

    def idn(self):
        return "Agilent Technologies,34401A,MOCK0,11-5-2"

    def read_dc(self, nplc=None, samples=1):  # noqa: ARG002
        self._n += 1
        v = self.board.commanded_voltage() if self.board is not None else 0.0
        return v + ((self._n % 3) - 1) * 5e-6

    def close(self):
        pass
