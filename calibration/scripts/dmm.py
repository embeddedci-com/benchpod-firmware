#!/usr/bin/env python3
"""Bench DMM drivers for calibration: DC volts from whichever supported meter is plugged in.

Supported meters:

  34401a  HP/Agilent/Keysight 34401A over GPIB, through an NI GPIB-USB-HS and the pure-Python
          user-space driver in ni_gpib_usb.py (no linux-gpib, no NI software, no kernel module,
          works on Apple Silicon). Needs libusb + pyusb.
  owon    OWON XDM1041 / XDM2041 over its USB port, a USB serial port (115200 8N1) speaking the
          SCPI dialect of OWON's "XDM1000&2000 Series Programming Manual". Needs pyserial.

open_dmm() finds the meter: kind="auto" (the default) tries each driver in DRIVERS and returns the
first that answers *IDN? with its model; kind="34401a" or "owon" opens that one only. Adding a
meter = a class with the same small interface (detect, idn, read_dc, set_range, close, name)
appended to DRIVERS.

Ranges are given as "readings up to this many volts" (or "auto"), and each driver picks its
smallest native range that holds that: 5 means the 34401A's 10 V range and the OWON's 5 V range
(which reads up to 5.5 V). The 34401A's own range values (0.1, 1, 10, 100, 1000) still map to
themselves, so older callers keep their range.

Requires:  brew install libusb  &&  pip install -r requirements-host.txt (pyusb, pyserial)
A MockDMM (no hardware) is provided for exercising the cal pipeline offline.
"""

import os
import time


class DMMError(RuntimeError):
    pass


def pick_range(volts, native, overrange):
    """The smallest native range (volts) that reads up to `volts`, or "auto".

    `overrange` is how far past full scale the meter reads: 1.2 for the 34401A (120 %), 1.1 for
    a 55,000-count OWON (5.5 V on the 5 V range)."""
    if volts is None or str(volts).strip().upper() in ("AUTO", "DEF"):
        return "auto"
    try:
        v = abs(float(volts))
    except ValueError:
        raise DMMError(f"not a range: {volts!r} (a number of volts, or auto)") from None
    for n in native:
        if v <= n * overrange * (1 + 1e-9):
            return n
    raise DMMError(f"{v:g} V is beyond the meter's largest range ({native[-1]:g} V)")


def _fmt_range(r):
    return "auto" if r == "auto" else f"{r:g} V"


class Agilent34401A:
    """34401A DC-volts reader. Default GPIB primary address 22, 10 V range.

    `board` is accepted for backward compatibility and ignored (there is no
    linux-gpib board index anymore); the controller uses GPIB address 0.
    """

    name = "HP/Agilent 34401A"
    kind = "34401a"
    RANGES = (0.1, 1, 10, 100, 1000)
    OVERRANGE = 1.2

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
        self.vrange = pick_range(vrange, self.RANGES, self.OVERRANGE)
        try:
            self._configure()
        except Exception:
            self.close()
            raise

    @classmethod
    def detect(cls, gpib_addr=22, nplc=10, vrange=10, **_):
        """The meter if an NI GPIB-USB-HS is plugged in and a 34401A answers on `gpib_addr`.
        Raises DMMError with the reason otherwise."""
        try:
            import usb.core
            from ni_gpib_usb import NI_HS_PID, NI_VID
        except ImportError:
            raise DMMError("pyusb is not installed (needed for the NI GPIB-USB-HS)") from None
        try:
            present = usb.core.find(idVendor=NI_VID, idProduct=NI_HS_PID) is not None
        except Exception as e:  # no libusb backend
            raise DMMError(f"cannot list USB devices ({e}); is libusb installed?") from None
        if not present:
            raise DMMError("no NI GPIB-USB-HS on USB")
        try:
            dmm = cls(gpib_addr=gpib_addr, nplc=nplc, vrange=vrange)
        except DMMError:
            raise
        except Exception as e:  # GpibError: nothing answers on that address
            raise DMMError(f"no answer on GPIB address {gpib_addr} ({e})") from None
        idn = dmm.idn()
        if "34401A" not in idn.upper():
            dmm.close()
            raise DMMError(f"GPIB address {gpib_addr} is not a 34401A: {idn.strip()!r}")
        return dmm

    def _write(self, s):
        self.gpib.write(self.addr, s.encode() if isinstance(s, str) else s)

    def _query(self, s):
        return self.gpib.query(self.addr, s)

    def _configure(self):
        # DC volts, fixed range, high NPLC for low reading noise, autozero on.
        # Immediate trigger so READ? is one-shot.
        self._write("*CLS")
        self._write("SYST:REM")
        self._apply_range()
        self._write(f"VOLT:DC:NPLC {self.nplc}")
        self._write("ZERO:AUTO ON")
        self._write("TRIG:SOUR IMM")

    def _apply_range(self):
        if self.vrange == "auto":
            self._write("CONF:VOLT:DC DEF")
            self._write("VOLT:DC:RANG:AUTO ON")
        else:
            self._write(f"CONF:VOLT:DC {self.vrange:g}")

    def set_range(self, volts):
        """Readings up to `volts` (or "auto") from here on."""
        r = pick_range(volts, self.RANGES, self.OVERRANGE)
        if r != self.vrange:
            self.vrange = r
            self._apply_range()  # CONF resets the integration time and the trigger
            self._write(f"VOLT:DC:NPLC {self.nplc}")
            self._write("ZERO:AUTO ON")
            self._write("TRIG:SOUR IMM")

    @property
    def range_label(self):
        return _fmt_range(self.vrange)

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


class OwonXDM:
    """OWON XDM1041 / XDM2041 DC-volts reader over the USB serial port.

    Written from OWON's "XDM1000&2000 Series Digital Multimeter Programming Manual" (V1.0.1):
    SYST:REM enters SCPI mode, CONF:VOLT:DC {500E-3|5|50|500|1000|AUTO} selects DC volts and the
    range (and resets the other settings), RATE {S|M|F} sets the reading speed, MEAS1? returns the
    main display's value in scientific notation, SYST:LOC hands the front panel back. The manual
    gives no serial settings; 115200 8N1 with newline-terminated lines is what the meter's
    open-source clients use. Not every command is documented to answer, so after a command the
    driver reads and drops whatever the meter sends back within a short window.

    Reading speed: there is no NPLC, so `nplc` maps onto RATE: 5 and above = S (slow, lowest
    noise, what calibration wants), 0.5 to 5 = M, below 0.5 = F.
    """

    name = "OWON XDM"
    kind = "owon"
    RANGES = (0.5, 5, 50, 500, 1000)
    OVERRANGE = 1.1                  # 55,000 counts: the 5 V range reads up to 5.5 V
    BAUD = 115200
    USB_VIDS = (0x1A86,)             # WCH CH340/CH341, the USB serial chip in these meters
    # Seconds between two polls of MEAS1? at each rate, so averaged samples are new conversions,
    # and the settling time after the range or rate changes. Conservative: the manual gives none.
    POLL_S = {"S": 0.5, "M": 0.12, "F": 0.03}
    SETTLE_S = 1.5
    DRAIN_S = 0.25                   # how long to wait for a reply to a non-query command

    def __init__(self, port=None, nplc=10, vrange=10, transport=None, timeout=2.0):
        if transport is None:
            if not port:
                raise DMMError("no serial port given for the OWON")
            try:
                import serial
            except ImportError:
                raise DMMError("pyserial is not installed (pip install pyserial)") from None
            try:
                transport = serial.Serial(port, self.BAUD, bytesize=8, parity="N", stopbits=1,
                                          timeout=timeout, write_timeout=timeout)
            except Exception as e:
                raise DMMError(f"cannot open {port}: {e}") from None
        self.port = port
        self.t = transport
        self.timeout = timeout
        self.vrange = pick_range(vrange, self.RANGES, self.OVERRANGE)
        self.rate = self._rate_for(nplc)
        self.nplc = nplc
        self._last_read = 0.0
        try:
            self._idn = self._hello()
            if "XDM" not in self._idn.upper():
                raise DMMError(f"{port or 'the port'} is not an OWON XDM: {self._idn!r}")
            self._configure()
        except Exception:
            self.close(local=False)
            raise

    # ---- detection -----------------------------------------------------------------------

    @classmethod
    def candidate_ports(cls):
        """Serial ports that look like an OWON XDM: the CH340 USB VID, or OWON/XDM in the name."""
        try:
            from serial.tools import list_ports
        except ImportError:
            raise DMMError("pyserial is not installed (pip install pyserial)") from None
        out = []
        for p in list_ports.comports():
            text = " ".join(str(x) for x in (p.description, p.manufacturer, p.product) if x).upper()
            if p.vid in cls.USB_VIDS or "OWON" in text or "XDM" in text:
                out.append(p.device)
        # macOS lists each port twice (tty.* and cu.*); cu.* is the one to open
        cu = [d for d in out if "/cu." in d]
        return sorted(set(cu or out))

    @classmethod
    def detect(cls, port=None, nplc=10, vrange=10, **_):
        """The meter on `port`, or on the first candidate port that answers as an OWON XDM.
        Raises DMMError with the reason otherwise."""
        ports = [port] if port else cls.candidate_ports()
        if not ports:
            raise DMMError("no USB serial port that looks like an OWON XDM (CH340); "
                           "pass --port if it is on another one")
        errors = []
        for p in ports:
            try:
                return cls(port=p, nplc=nplc, vrange=vrange)
            except DMMError as e:
                errors.append(str(e))
        raise DMMError("; ".join(errors))

    # ---- transport -----------------------------------------------------------------------

    def _send(self, s):
        self.t.write((s + "\n").encode("ascii"))
        flush = getattr(self.t, "flush", None)
        if flush:
            flush()

    def _readline(self, timeout=None):
        old = self.t.timeout
        if timeout is not None:
            self.t.timeout = timeout
        try:
            return self.t.readline().decode("ascii", "replace").strip()
        finally:
            self.t.timeout = old

    def _drain(self, window=None):
        """Read and drop what the meter sends until it has been quiet for `window` seconds."""
        window = self.DRAIN_S if window is None else window
        while self._readline(timeout=window):
            pass

    def _command(self, s):
        self._send(s)
        self._drain()

    def _query(self, s):
        self.t.reset_input_buffer()
        self._send(s)
        line = self._readline()
        if not line:
            raise DMMError(f"no answer to {s} within {self.timeout:g} s")
        return line

    def _hello(self):
        """*IDN?, entering SCPI mode first when the meter does not answer straight away."""
        try:
            idn = self._query("*IDN?")
        except DMMError:
            self._command("SYST:REM")
            try:
                idn = self._query("*IDN?")
            except DMMError:
                raise DMMError(f"{self.port or 'the port'}: no answer to *IDN? (is it an OWON "
                               "XDM, and on?)") from None
        self._drain()
        return idn

    # ---- meter ---------------------------------------------------------------------------

    @staticmethod
    def _rate_for(nplc):
        n = 10 if nplc is None else float(nplc)
        return "S" if n >= 5 else "M" if n >= 0.5 else "F"

    def _configure(self):
        self._command("SYST:REM")
        self._apply_range()
        self._command("CALC:STAT OFF")       # no relative/dB/statistics math on the reading
        self._command(f"RATE {self.rate}")
        func = self._query("FUNC?").strip().strip('"').upper()
        if func != "VOLT":
            raise DMMError(f"the meter did not switch to DC volts (FUNC? = {func!r})")
        self._settle()

    def _apply_range(self):
        arg = "AUTO" if self.vrange == "auto" else f"{self.vrange:g}"
        if self.vrange == 0.5:
            arg = "500E-3"
        self._command(f"CONF:VOLT:DC {arg}")  # resets rate and math, so both follow it

    def _settle(self):
        time.sleep(self.SETTLE_S)
        self._query("MEAS1?")                   # drop the first conversion after a change, unread
        self._last_read = time.monotonic()

    def set_range(self, volts):
        """Readings up to `volts` (or "auto") from here on."""
        r = pick_range(volts, self.RANGES, self.OVERRANGE)
        if r != self.vrange:
            self.vrange = r
            self._apply_range()
            self._command("CALC:STAT OFF")
            self._command(f"RATE {self.rate}")
            self._settle()

    @property
    def range_label(self):
        return _fmt_range(self.vrange)

    def idn(self):
        return self._idn

    def _measure(self):
        s = self._query("MEAS1?")
        up = s.upper()
        if "OL" in up or "OVER" in up:
            raise DMMError(f"overload on the {self.range_label} range ({s!r})")
        try:
            v = float(s.split(",")[0])
        except ValueError:
            raise DMMError(f"unparseable DMM reading: {s!r}") from None
        if not abs(v) < 1e9:
            raise DMMError(f"overload on the {self.range_label} range ({s!r})")
        return v

    def read_dc(self, nplc=None, samples=1):
        if nplc is not None and self._rate_for(nplc) != self.rate:
            self.rate = self._rate_for(nplc)
            self.nplc = nplc
            self._command(f"RATE {self.rate}")
            self._settle()
        vals = []
        for _ in range(max(1, int(samples or 1))):
            wait = self.POLL_S[self.rate] - (time.monotonic() - self._last_read)
            if wait > 0:
                time.sleep(wait)
            vals.append(self._measure())
            self._last_read = time.monotonic()
        return sum(vals) / len(vals)

    def close(self, local=True):
        if getattr(self, "t", None) is None:
            return
        try:
            if local:
                self._send("SYST:LOC")          # hand the front panel back
        except Exception:
            pass
        try:
            self.t.close()
        finally:
            self.t = None


# Tried in this order by open_dmm(kind="auto"). Append a class to support another meter.
DRIVERS = [Agilent34401A, OwonXDM]
KINDS = ["auto"] + [d.kind for d in DRIVERS] + ["mock"]


def open_dmm(kind=None, vrange=10, nplc=10, gpib_addr=22, port=None):
    """Open the bench DMM. kind: "auto" (default; also $DMM), a driver kind, or "mock".
    port: the OWON's serial port (default $DMM_PORT, else auto-detected)."""
    kind = (kind or os.environ.get("DMM") or "auto").lower()
    port = port or os.environ.get("DMM_PORT") or None
    opts = dict(vrange=vrange, nplc=nplc, gpib_addr=gpib_addr, port=port)
    if kind == "mock":
        return MockDMM()
    drivers = [d for d in DRIVERS if kind in ("auto", d.kind)]
    if not drivers:
        raise DMMError(f"unknown DMM {kind!r}; one of {', '.join(KINDS)}")
    reasons = []
    for d in drivers:
        try:
            return d.detect(**opts)
        except DMMError as e:
            reasons.append(f"{d.name}: {e}")
    raise DMMError("no DMM found. " + " | ".join(reasons))


class MockDMM:
    """Synthetic DMM tied to a MockBoard so a full DAC-cal run yields a clean
    fit without hardware. Returns the voltage the board was last commanded to,
    plus a deterministic sub-LSB dither (no RNG, so runs are reproducible)."""

    name = "mock"
    kind = "mock"
    range_label = "mock"

    def __init__(self, board=None):
        self.board = board
        self._n = 0

    def idn(self):
        return "Agilent Technologies,34401A,MOCK0,11-5-2"

    def set_range(self, volts):  # noqa: ARG002
        pass

    def read_dc(self, nplc=None, samples=1):  # noqa: ARG002
        self._n += 1
        v = self.board.commanded_voltage() if self.board is not None else 0.0
        return v + ((self._n % 3) - 1) * 5e-6

    def close(self):
        pass
