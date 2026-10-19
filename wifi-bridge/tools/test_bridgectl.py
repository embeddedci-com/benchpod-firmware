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
    "ffffffffffff" "02bb00000001" "88b5" "4257" "01" "01" "0000" "0048"
    "02001462656e6368706f642d776966692d627269646765"
    "030005302e312e30" "04000141" "05000101" "1600010b"
    "06000602bb00000001" "0700060080e1123456" "1700080102030405060708")

# test_main.c --dump: CLEAR_WIFI seq 0x0103 signed with key 00..1f, nonce 01..08, counter 5
KEY = bytes(range(32))
NONCE = bytes(range(1, 9))
CLEAR_WIFI_SIGNED = bytes.fromhex(
    "02bb000000010080e112345688b54257" "01050103001a" "19000400000005"
    "1a0010" "8c32e64d4c6382dfc653effee85b0837").ljust(60, b"\0")

# test_main.c --dump: SET_WIFI "lab" / "secret12", key 00..1f, bridge nonce 01..08,
# seal nonce 01..18, counter 6, and the bridge's signed OK.
SET_WIFI = bytes.fromhex(
    "02bb000000010080e112345688b5425701040102007c"
    + "080003" "6c6162"
    + "1b0059" "0102030405060708090a0b0c0d0e0f101112131415161718"
    + "e7ade5fd4e5a39fe3109acd136c8439e426b3bfc208365698487818325230aaa59c2cf"
    + "c763e012f361e2943a040037fc02ab33f0d128f84a3b210a1e9b1bd87a9b"
    + "190004" "00000006"
    + "1a0010" "e4470dd119d15970a77b3f6e3150ea3f")

SET_WIFI_RESPONSE = bytes.fromhex(
    "0080e112345602bb0000000188b54257" "01840102001e" "01000100" "19000400000006"
    "1a0010" "0559f3f49576806254b35ae686cc2d9c").ljust(60, b"\0")



class Codec(unittest.TestCase):
    def test_hello_golden(self):
        m = b.decode(HELLO)
        self.assertEqual(m["type"], b.HELLO)
        self.assertEqual(m["src"], BRIDGE)
        self.assertEqual(b.first(m["tlvs"], b.T_MODEL), b"benchpod-wifi-bridge")
        self.assertEqual(b.first(m["tlvs"], b.T_POD_MAC), POD)
        self.assertIn("state: wait-pod", b.describe(m))

    def test_set_wifi_golden(self):
        # Sealed passphrase + AUTH: the same bytes as the C encoder, and no cleartext in them.
        sealed = b.seal_passphrase(KEY, bytes(range(1, 25)), "secret12")
        body = b.tlv(b.T_SSID, "lab") + b.tlv(b.T_ENC_PASSPHRASE, sealed)
        frame = b.encode_signed(BRIDGE, POD, b.SET_WIFI, 0x0102, body, KEY, NONCE, 6)
        self.assertEqual(frame, SET_WIFI)
        self.assertNotIn(b"secret12", frame)
        r = b.decode(SET_WIFI_RESPONSE)
        self.assertEqual((r["type"], r["seq"]), (b.SET_WIFI | b.RESPONSE, 0x0102))
        self.assertEqual(b.result_of(r), (0, ""))
        self.assertTrue(b.verify(SET_WIFI_RESPONSE, KEY, NONCE))

    def test_signed_golden(self):
        self.assertEqual(b.encode_signed(BRIDGE, POD, b.CLEAR_WIFI, 0x0103, b"", KEY, NONCE, 5), CLEAR_WIFI_SIGNED)
        self.assertTrue(b.verify(CLEAR_WIFI_SIGNED, KEY, NONCE))
        self.assertFalse(b.verify(CLEAR_WIFI_SIGNED, KEY, bytes(8)))
        tampered = bytearray(CLEAR_WIFI_SIGNED)
        tampered[28] ^= 1  # counter
        self.assertFalse(b.verify(bytes(tampered), KEY, NONCE))
        self.assertFalse(b.verify(b.encode(BRIDGE, POD, b.STATUS, 1), KEY, NONCE))
        self.assertEqual(b.first(b.decode(HELLO)["tlvs"], b.T_NONCE), NONCE)

    def test_x25519_rfc7748(self):
        a_sk = bytes.fromhex("77076d0a7318a57d3c16c17251b26645df4c2f87ebc0992ab177fba51db92c2a")
        b_sk = bytes.fromhex("5dab087e624a8a4b79e17f8b83800ee66f3bb1292618b6fd1c2f8b27ff88e0eb")
        self.assertEqual(b.x25519_public(a_sk).hex(), "8520f0098930a754748b7ddcb43ef75a0dbf3a0d26381af4eba4a98eaa9b4e6a")
        self.assertEqual(b.x25519_public(b_sk).hex(), "de9edb7d7b7dc1b4d35b61c2ece435373f8343c85b78674dadfc7e146f882b4f")
        shared = "4a5d9d5ba4ce2de1728e3bf480350f25e07e21c947d19e3376f09b3c1e161742"
        self.assertEqual(b.x25519(a_sk, b.x25519_public(b_sk)).hex(), shared)
        self.assertEqual(b.x25519(b_sk, b.x25519_public(a_sk)).hex(), shared)

    def test_pairing_and_seal_match_c(self):
        # test_main.c --dump: pod secret 10..2f, bridge secret 80..9f, nonce 01..18, "secret12"
        pod_sk = bytes(range(0x10, 0x30))
        br_sk = bytes(range(0x80, 0xa0))
        pod_pk, br_pk = b.x25519_public(pod_sk), b.x25519_public(br_sk)
        self.assertEqual(pod_pk.hex(), "d89e3bad79437dbed9f843418304f460ff05c7fe81fe4a9577a804cb9367ff66")
        self.assertEqual(br_pk.hex(), "493e82fc74464a59268817623d2053c5eb8e2cc4a988b4fee179ec6b010d531d")
        key = b.pair_key(pod_sk, br_pk, pod_pk, br_pk)
        self.assertEqual(key, b.pair_key(br_sk, pod_pk, pod_pk, br_pk))
        self.assertEqual(key.hex(), "dc8611ac709267622ccade0b6fa65275fb949bdbc61160abb2ef995334fce541")
        sealed = b.seal_passphrase(key, bytes(range(1, 25)), "secret12")
        self.assertEqual(sealed.hex(), (
            "0102030405060708090a0b0c0d0e0f101112131415161718cf63328194e542fd"
            "88764c826a162ce235582a9f2bbd933dbdf73e548e4ab35316ffcf37e4c891f8"
            "cb689288b9a924077c2c118c6f9c93f14e336a3cf25323861a"))
        with self.assertRaises(ValueError):
            b.pair_key(pod_sk, bytes(32), pod_pk, bytes(32))

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
