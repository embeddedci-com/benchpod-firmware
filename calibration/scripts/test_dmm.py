#!/usr/bin/env python3
"""Host tests for dmm.py, no hardware: python3 -m unittest calibration/scripts/test_dmm.py

The OWON is a fake serial port that answers the way OWON's programming manual describes; the
34401A is only checked for its range mapping (its transport is ni_gpib_usb.py, tested on the
bench)."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import dmm  # noqa: E402
from dmm import DMMError, OwonXDM, open_dmm, pick_range  # noqa: E402


class FakeOwon:
    """A serial port with an OWON XDM2041 behind it."""

    def __init__(self, volts=3.0, needs_remote=False, ack_commands=False, idn="OWON,XDM2041,2310123,V4.3.0,3"):
        self.volts = volts
        self.remote = not needs_remote
        self.ack = ack_commands
        self.idn = idn
        self.func = "VOLT AC"
        self.range = None
        self.rate = "F"
        self.math = True
        self.out = []
        self.sent = []
        self.timeout = 2.0
        self.closed = False

    def write(self, data):
        for cmd in data.decode().splitlines():
            self.sent.append(cmd)
            self._handle(cmd.strip())

    def _reply(self, s):
        self.out.append((s + "\r\n").encode())

    def _handle(self, cmd):
        up = cmd.upper()
        if up == "SYST:REM":
            self.remote = True
        elif not self.remote:
            return  # not in SCPI mode: silent
        if up == "*IDN?":
            self._reply(self.idn)
        elif up.startswith("CONF:VOLT:DC"):
            arg = cmd.split()[1] if " " in cmd else "AUTO"
            self.func, self.range, self.rate, self.math = "VOLT", arg, "S", True
        elif up.startswith("RATE "):
            self.rate = cmd.split()[1]
        elif up == "CALC:STAT OFF":
            self.math = False
        elif up == "FUNC?":
            self._reply(f'"{self.func}"')
        elif up == "MEAS1?":
            full = 1e9 if self.range == "AUTO" else float(self.range) * 1.1
            self._reply("OL" if abs(self.volts) > full else f"{self.volts:.5E}")
            return
        elif up == "SYST:LOC":
            self.remote = False
        else:
            if not up.endswith("?") and self.ack:
                self._reply("")
            return
        if self.ack and not up.endswith("?"):
            self._reply("")

    def readline(self):
        return self.out.pop(0) if self.out else b""

    def reset_input_buffer(self):
        self.out.clear()

    def flush(self):
        pass

    def close(self):
        self.closed = True


class Fast(OwonXDM):
    SETTLE_S = 0.0
    DRAIN_S = 0.0
    POLL_S = {"S": 0.0, "M": 0.0, "F": 0.0}


class PickRange(unittest.TestCase):
    def test_34401a_values_map_to_themselves(self):
        A = dmm.Agilent34401A
        for r in (0.1, 1, 10, 100, 1000):
            self.assertEqual(pick_range(r, A.RANGES, A.OVERRANGE), r)
        self.assertEqual(pick_range("5.5", A.RANGES, A.OVERRANGE), 10)
        self.assertEqual(pick_range(12, A.RANGES, A.OVERRANGE), 10)

    def test_owon(self):
        O = OwonXDM
        self.assertEqual(pick_range(5.5, O.RANGES, O.OVERRANGE), 5)
        self.assertEqual(pick_range(3.3, O.RANGES, O.OVERRANGE), 5)
        self.assertEqual(pick_range(0.4, O.RANGES, O.OVERRANGE), 0.5)
        self.assertEqual(pick_range(10, O.RANGES, O.OVERRANGE), 50)
        self.assertEqual(pick_range("auto", O.RANGES, O.OVERRANGE), "auto")
        with self.assertRaises(DMMError):
            pick_range(2000, O.RANGES, O.OVERRANGE)


class Owon(unittest.TestCase):
    def test_configures_dc_volts_slow_no_math(self):
        t = FakeOwon(volts=3.00012)
        m = Fast(transport=t, vrange=5.5, nplc=10)
        self.assertEqual(t.range, "5")
        self.assertEqual(t.rate, "S")
        self.assertFalse(t.math)
        self.assertIn("XDM2041", m.idn())
        self.assertAlmostEqual(m.read_dc(samples=3), 3.00012, places=6)
        # CONF resets rate and math, so they come after it
        i = t.sent.index("CONF:VOLT:DC 5")
        self.assertLess(i, t.sent.index("RATE S"))
        self.assertLess(i, t.sent.index("CALC:STAT OFF"))
        m.close()
        self.assertEqual(t.sent[-1], "SYST:LOC")
        self.assertTrue(t.closed)

    def test_enters_scpi_mode_when_silent(self):
        t = FakeOwon(needs_remote=True)
        m = Fast(transport=t)
        self.assertIn("XDM2041", m.idn())
        self.assertEqual(t.sent[:2], ["*IDN?", "SYST:REM"])

    def test_command_acks_do_not_shift_replies(self):
        t = FakeOwon(volts=-4.5, ack_commands=True)
        m = Fast(transport=t, vrange=5)
        self.assertAlmostEqual(m.read_dc(), -4.5, places=6)

    def test_overload(self):
        t = FakeOwon(volts=6.0)
        m = Fast(transport=t, vrange=5)
        with self.assertRaisesRegex(DMMError, "overload on the 5 V range"):
            m.read_dc()
        m.set_range(10)
        self.assertEqual(t.range, "50")
        self.assertAlmostEqual(m.read_dc(), 6.0, places=6)

    def test_small_range_and_auto(self):
        t = FakeOwon(volts=0.1)
        Fast(transport=t, vrange=0.3)
        self.assertEqual(t.range, "500E-3")
        t = FakeOwon(volts=7)
        m = Fast(transport=t, vrange="auto")
        self.assertEqual(t.range, "AUTO")
        self.assertEqual(m.range_label, "auto")

    def test_rate_follows_nplc(self):
        t = FakeOwon()
        m = Fast(transport=t, nplc=1)
        self.assertEqual(t.rate, "M")
        m.read_dc(nplc=0.1)
        self.assertEqual(t.rate, "F")

    def test_not_an_owon(self):
        t = FakeOwon(idn="RIGOL TECHNOLOGIES,DM3058E,DM3O1234,01.01.00.01.11.00")
        with self.assertRaisesRegex(DMMError, "not an OWON XDM"):
            Fast(transport=t)
        self.assertTrue(t.closed)
        self.assertNotIn("SYST:LOC", t.sent)

    def test_no_answer(self):
        t = FakeOwon(needs_remote=True)
        t._handle = lambda cmd: None
        with self.assertRaisesRegex(DMMError, "no answer to \\*IDN\\?"):
            Fast(transport=t)

    def test_wrong_function(self):
        t = FakeOwon()
        orig = t._handle

        def stuck(cmd):
            orig(cmd)
            t.func = "CURR"
        t._handle = stuck
        with self.assertRaisesRegex(DMMError, "did not switch to DC volts"):
            Fast(transport=t)


class OpenDMM(unittest.TestCase):
    def setUp(self):
        self.saved = dmm.DRIVERS[:]

    def tearDown(self):
        dmm.DRIVERS[:] = self.saved

    def _driver(self, kind, ok):
        class D:
            name = kind.upper()

            @classmethod
            def detect(cls, **opts):
                if not ok:
                    raise DMMError("not here")
                return (kind, opts)
        D.kind = kind
        return D

    def test_auto_takes_first_that_answers(self):
        dmm.DRIVERS[:] = [self._driver("a", False), self._driver("b", True), self._driver("c", True)]
        kind, opts = open_dmm("auto", vrange=5.5, port="/dev/x")
        self.assertEqual(kind, "b")
        self.assertEqual(opts["vrange"], 5.5)
        self.assertEqual(opts["port"], "/dev/x")

    def test_named_kind_only(self):
        dmm.DRIVERS[:] = [self._driver("a", True), self._driver("b", True)]
        self.assertEqual(open_dmm("b")[0], "b")

    def test_none_found_lists_reasons(self):
        dmm.DRIVERS[:] = [self._driver("a", False), self._driver("b", False)]
        with self.assertRaisesRegex(DMMError, "no DMM found. A: not here \\| B: not here"):
            open_dmm()

    def test_unknown_and_mock(self):
        with self.assertRaisesRegex(DMMError, "unknown DMM"):
            open_dmm("fluke")
        self.assertEqual(open_dmm("mock").kind, "mock")


if __name__ == "__main__":
    unittest.main()
