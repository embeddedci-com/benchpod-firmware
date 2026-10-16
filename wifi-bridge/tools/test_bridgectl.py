#!/usr/bin/env python3
"""Checks the Python codec against the same example frames as the C tests and docs/PROTOCOL.md.

    python3 tools/test_bridgectl.py
"""
import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bridgectl as b  # noqa: E402

BRIDGE = bytes.fromhex("02bb00000001")
POD = bytes.fromhex("0080e1123456")

HELLO = bytes.fromhex(
    "ffffffffffff" "02bb00000001" "88b5" "4257" "01" "01" "0000" "003d"
    "02001462656e6368706f642d776966692d627269646765"
    "030005302e312e30" "04000141" "05000101" "1600010b"
    "06000602bb00000001" "0700060080e1123456")

SET_WIFI = bytes.fromhex(
    "02bb000000010080e112345688b54257010401020011"
    "080003" "6c6162" "090008" "7365637265743132").ljust(60, b"\0")

SET_WIFI_RESPONSE = bytes.fromhex("0080e112345602bb0000000188b54257018401020004" "01000100").ljust(60, b"\0")


class Codec(unittest.TestCase):
    def test_hello_golden(self):
        m = b.decode(HELLO)
        self.assertEqual(m["type"], b.HELLO)
        self.assertEqual(m["src"], BRIDGE)
        self.assertEqual(b.first(m["tlvs"], b.T_MODEL), b"benchpod-wifi-bridge")
        self.assertEqual(b.first(m["tlvs"], b.T_POD_MAC), POD)
        self.assertIn("state: wait-pod", b.describe(m))

    def test_set_wifi_golden(self):
        body = b.tlv(b.T_SSID, "lab") + b.tlv(b.T_PASSPHRASE, "secret12")
        self.assertEqual(b.encode(BRIDGE, POD, b.SET_WIFI, 0x0102, body), SET_WIFI)
        r = b.decode(SET_WIFI_RESPONSE)
        self.assertEqual((r["type"], r["seq"]), (b.SET_WIFI | b.RESPONSE, 0x0102))
        self.assertEqual(b.result_of(r), (0, ""))

    def test_rejects(self):
        self.assertIsNone(b.decode(b"\0" * 60))
        bad = bytearray(HELLO)
        bad[16] = 2
        self.assertIsNone(b.decode(bytes(bad)))
        with self.assertRaises(ValueError):
            b.parse_tlvs(b"\x01\x00\x05ab")

    def test_scan_result(self):
        ap = b.tlv(b.T_SSID, "lab") + b.tlv(b.T_RSSI, struct.pack("b", -55)) + b.tlv(b.T_CHANNEL, b"\x06") + \
            b.tlv(b.T_AUTHMODE, b"\x03") + b.tlv(b.T_BSSID, BRIDGE)
        frame = b.encode(POD, BRIDGE, b.SCAN | b.RESPONSE, 3, b.tlv(b.T_RESULT, b"\0") + b.tlv(b.T_AP, ap))
        lines = b.describe(b.decode(frame))
        self.assertEqual(lines[0], "result: ok")
        self.assertIn("-55  6 wpa2", lines[1])
        self.assertTrue(lines[1].endswith(" lab"))


if __name__ == "__main__":
    unittest.main()
