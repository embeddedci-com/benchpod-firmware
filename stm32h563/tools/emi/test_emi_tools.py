#!/usr/bin/env python3
"""Host tests for emi_watch.py and emi_log_dump.py (run by `make -C stm32h563/test`)."""
import os
import struct
import sys
import unittest
import zlib

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import emi_log_dump as dump   # noqa: E402
import emi_watch as watch     # noqa: E402

POD = bytes([0x02, 0x00, 0x00, 0x30, 0xA9, 0xF1])


def bpemi(seq, up):
    hdr = bytes([0x02, 0x42, 0x50, 0x45, 0x4D, 0x49]) + POD + b"\x88\xb6"
    return hdr + b"BPEMI" + struct.pack("<II", seq, up) + bytes(40)


def bpmon(text):
    return b"\xff" * 6 + POD + b"\x88\xb6" + text.encode() + b"\0" * 8


def pcap(frames):
    out = struct.pack("<IHHiIII", 0xA1B2C3D4, 2, 4, 0, 0, 512, 1)
    for ts, f in frames:
        out += struct.pack("<IIII", int(ts), int((ts % 1) * 1e6), len(f), len(f)) + f
    return out


class WatchTest(unittest.TestCase):
    def run_frames(self, frames, chunk=7):
        lines = []
        w = watch.Watcher(lines.append, stall=2.0, silent=12.0)
        ps = watch.PcapStream()
        data = pcap(frames)
        for i in range(0, len(data), chunk):   # fed in odd pieces, like a pipe
            for ts, f in ps.feed(data[i:i + chunk]):
                w.tick(ts)
                w.frame(ts, f)
        return w, lines

    def kinds(self, w):
        return [a[1] for a in w.alerts]

    def test_clean_stream(self):
        t0 = 1_700_000_000.0
        frames = [(t0 + i * 0.001, bpemi(i, 10_000 + i)) for i in range(500)]
        frames.append((t0 + 0.5, bpmon("BPMON run 3, reset power-on, uptime 10500 ms, seq 500, "
                                       "emi: running, led green, dac all")))
        w, lines = self.run_frames(frames)
        self.assertEqual(w.alerts, [])
        p = next(iter(w.pods.values()))
        self.assertEqual((p["frames"], p["lost"], p["run"], p["led"]), (500, 0, 3, "green"))
        self.assertTrue(any("BPMON run 3" in l for l in lines))

    def test_gap_stall_reboot_led(self):
        t = 1_700_000_000.0
        frames = [(t, bpemi(10, 1000)), (t + 0.001, bpemi(11, 1001)),
                  (t + 0.002, bpemi(15, 1002)),                                   # 3 lost
                  (t + 0.5, bpmon("BPMON run 3, reset power-on, uptime 1500 ms, seq 16, emi: running, led red")),
                  (t + 5.0, bpmon("BPMON run 3, reset power-on, uptime 6000 ms, seq 16, emi: stopped, led blue")),
                  (t + 9.0, bpemi(16, 9000)),                                     # stalled, resumed
                  (t + 30.0, bpemi(0, 50)),                                       # silent, rebooted
                  (t + 30.5, bpmon("BPMON run 4, reset watchdog, uptime 600 ms, seq 1, emi: waiting for boot start, led green (starting)"))]
        w, _ = self.run_frames(frames)
        k = self.kinds(w)
        self.assertIn("LOST", k)
        self.assertEqual(next(iter(w.pods.values()))["lost"], 3)
        self.assertEqual(k.count("LED"), 3)   # red, blue, then green again
        self.assertIn("STOPPED", k)
        self.assertIn("RESUMED", k)
        self.assertIn("SILENT", k)
        self.assertIn("BACK", k)
        self.assertEqual(k.count("REBOOT"), 2)   # the stream sequence, then the run number

    def test_ignores_other_traffic(self):
        other = b"\xff" * 6 + POD + b"\x08\x00" + bytes(46)
        w, _ = self.run_frames([(1.0, other)])
        self.assertEqual(w.pods, {})

    def test_main_reads_file(self):
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            f = os.path.join(d, "c.pcap")
            with open(f, "wb") as fh:
                fh.write(pcap([(1_700_000_000.0, bpemi(1, 5)), (1_700_000_000.1, bpemi(3, 6))]))
            log = os.path.join(d, "w.log")
            rc = watch.main(["-r", f, "--log", log, "--quiet"])
            self.assertEqual(rc, 1)   # an alert (LOST)
            with open(log) as fh:
                self.assertIn("[LOST]", fh.read())


class LogDumpTest(unittest.TestCase):
    def test_crc_matches_firmware(self):
        # The same vector as test_emi_log.c: an all-zero EVENT record.
        b = bytearray(64)
        struct.pack_into("<HB", b, 0, 0x4C45, 2)
        self.assertEqual(zlib.crc32(bytes(b[:60])), 0xA17A5D28)
        struct.pack_into("<I", b, 60, 0xA17A5D28)
        self.assertIsNotNone(dump.decode(bytes(b)))   # the firmware's encoding of it decodes

    def test_roundtrip_and_reject(self):
        r = {"type": "START", "led": "starting", "seq": 7, "run": 3, "uptime_ms": 1234,
             "fw": "3.9.6", "reset": "power-on", "core_mhz": 250, "dac": "all", "net_mbit": 20,
             "gw_version": 48}
        b = dump.encode(r)
        d = dump.decode(b)
        for k, v in r.items():
            self.assertEqual(d[k], v, k)
        self.assertIsNone(dump.decode(b"\xff" * 64))
        bad = bytearray(b)
        bad[20] ^= 1
        self.assertIsNone(dump.decode(bytes(bad)))
        self.assertEqual(dump.fmt(d), "#7 run 3 +1.2s START fw 3.9.6 reset power-on core 250 MHz "
                                      "dac all net 20 Mbit/s gw 48")

    def test_summary_record(self):
        r = {"type": "SUM", "seq": 77, "run": 3, "uptime_ms": 600000, "cloud_drops": 8,
             "c3_restarts": 2, "i2c_clears": 1, "can_busoff": 0, "psram_fail": 0, "cdone_lost": 1,
             "psram_retry": 4, "rev_block_1": 31, "rev_block_2": 0, "eth_crc": 5, "eth_missed": 0,
             "bridge_tx_err": 977642, "cloud_down_s": 125}
        x = dump.decode(dump.encode(r))
        self.assertEqual(x["bridge_tx_err"], 977642)
        line = dump.fmt(x)
        # the same line the pod prints (test_emi_log.c checks the C side)
        self.assertTrue(line.startswith("#77 run 3 +600.0s SUM cloud drops 8 (down 125 s), c3 restarts 2, i2c clears 1"))
        self.assertIn("eFuse rev blocks 31/0, eth crc 5 missed 0, bridge tx err 977642", line)

    def test_ring_order_and_summary(self):
        # A wrapped ring: the newest records sit at the start of the dump, then empty slots, then
        # the oldest ones.
        def cp(seq, run, up, led="green", **kw):
            return dump.encode(dict(type="CHECK", seq=seq, run=run, uptime_ms=up, led=led, **kw))
        slots = [cp(10, 2, 300_000), cp(11, 2, 600_000, captures=99), b"\xff" * 64, b"\xff" * 64,
                 dump.encode(dict(type="START", seq=5, run=1, uptime_ms=10, fw="3.9.6", reset="power-on")),
                 dump.encode(dict(type="EVENT", seq=6, run=1, uptime_ms=20_000, kind="led",
                                  led_from="green", led_to="red", text="capture_dual: busy")),
                 b"\x00" * 64,                                       # a spoiled slot
                 dump.encode(dict(type="START", seq=9, run=2, uptime_ms=10, fw="3.9.6", reset="watchdog"))]
        recs = dump.records(b"".join(slots))
        self.assertEqual([r["seq"] for r in recs], [5, 6, 9, 10, 11])
        s = dump.summarize(recs)
        self.assertIn("run 1: fw 3.9.6, reset power-on", s[0])
        self.assertIn("LED: FAILED (failing over 60 s", s[0])
        self.assertIn("run 2: fw 3.9.6, reset watchdog", s[1])
        self.assertIn("no failure", s[1])
        self.assertIn("captures 99", s[1])
        self.assertIn("led green -> red", dump.fmt(recs[1]))

    def test_download_retries_busy(self):
        import base64
        slots = [dump.encode(dict(type="CHECK", seq=i + 1, run=1, uptime_ms=i)) for i in range(5)]
        raw = b"".join(slots) + b"\xff" * 64 * 3

        class FakePod:
            calls = 0

            def command(self, req):
                self.calls += 1
                if "count" not in req:
                    return {"slots": 8, "valid": 5, "run": 1, "max_count": 3}
                if self.calls % 2 == 0:
                    raise RuntimeError("busy: an EMI capture is using the bus, retry in a moment")
                o, c = req["offset"], req["count"]
                return {"offset": o, "count": c,
                        "data": base64.urlsafe_b64encode(raw[o * 64:(o + c) * 64]).decode().rstrip("=")}
        got, _ = dump.download(FakePod(), progress=lambda s: None)
        self.assertEqual(got, raw)


if __name__ == "__main__":
    unittest.main(verbosity=1)
