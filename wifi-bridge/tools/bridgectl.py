#!/usr/bin/env python3
"""Reference client for the Wi-Fi bridge control protocol (docs/PROTOCOL.md).

Talks to a bridge over raw Ethernet (EtherType 0x88B5) from a Linux machine wired to the
bridge's RJ45, standing in for the pod during bring-up. The bridge learns this machine's MAC as
the "pod" MAC, so the machine also gets its network through the bridge.

    sudo ./bridgectl.py -i eth0 hello            # wait for a HELLO and print it
    sudo ./bridgectl.py -i eth0 status
    sudo ./bridgectl.py -i eth0 scan
    sudo ./bridgectl.py -i eth0 set-wifi "lab net" "passphrase"
    sudo ./bridgectl.py -i eth0 clear-wifi | reboot | factory-reset
    sudo ./bridgectl.py -i eth0 ota build/benchpod-wifi-bridge.bin

Linux only (AF_PACKET), needs root or CAP_NET_RAW. The codec has no such requirement and is
unit-tested in test_bridgectl.py.
"""
import argparse
import socket
import struct
import sys
import time

ETHERTYPE = 0x88B5
MAGIC = b"BW"
VERSION = 1
RESPONSE = 0x80

HELLO, STATUS, SCAN, SET_WIFI, CLEAR_WIFI = 0x01, 0x02, 0x03, 0x04, 0x05
OTA_BEGIN, OTA_DATA, OTA_END, REBOOT, FACTORY_RESET, OTA_ABORT = 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B

T_RESULT, T_MODEL, T_FW_VERSION, T_HW_REV, T_STATE, T_BRIDGE_MAC, T_POD_MAC = 1, 2, 3, 4, 5, 6, 7
T_SSID, T_PASSPHRASE, T_CHANNEL, T_RSSI, T_BSSID, T_AUTHMODE, T_AP = 8, 9, 10, 11, 12, 13, 14
T_IMAGE_SIZE, T_OFFSET, T_DATA, T_UPTIME, T_REASON, T_COUNTERS, T_MESSAGE, T_FLAGS = 15, 16, 17, 18, 19, 20, 21, 22

STATES = ["unconfigured", "wait-pod", "joining", "connected", "auth-failed", "no-ap"]
RESULTS = ["ok", "bad-request", "unsupported", "busy", "storage", "ota-not-started", "ota-bad-offset",
           "ota-verify", "ota-too-large", "internal", "ota-wrong-image"]
AUTH = ["open", "wep", "wpa", "wpa2", "wpa/wpa2", "wpa2-ent", "wpa3", "wpa2/wpa3"]
COUNTERS = ["eth_rx", "wifi_tx", "wifi_tx_err", "wifi_rx", "eth_tx", "eth_tx_err", "dropped"]
BROADCAST = b"\xff" * 6


def tlv(tag, value):
    if isinstance(value, str):
        value = value.encode()
    return struct.pack(">BH", tag, len(value)) + value


def encode(dst, src, mtype, seq, tlvs=b""):
    frame = dst + src + struct.pack(">H", ETHERTYPE) + MAGIC + struct.pack(">BBHH", VERSION, mtype, seq, len(tlvs))
    frame += tlvs
    return frame.ljust(60, b"\0")


def parse_tlvs(body):
    """List of (tag, value). Raises ValueError on a truncated TLV."""
    out, i = [], 0
    while i < len(body):
        if len(body) - i < 3:
            raise ValueError("truncated TLV header")
        tag, n = struct.unpack_from(">BH", body, i)
        if i + 3 + n > len(body):
            raise ValueError("TLV runs past the body")
        out.append((tag, body[i + 3:i + 3 + n]))
        i += 3 + n
    return out


def decode(frame):
    """Returns a dict, or None if the frame is not a BWP v1 frame."""
    if len(frame) < 22 or struct.unpack_from(">H", frame, 12)[0] != ETHERTYPE or frame[14:16] != MAGIC:
        return None
    version, mtype, seq, blen = struct.unpack_from(">BBHH", frame, 16)
    if version != VERSION or 22 + blen > len(frame):
        return None
    return {"dst": frame[0:6], "src": frame[6:12], "type": mtype, "seq": seq, "tlvs": parse_tlvs(frame[22:22 + blen])}


def first(tlvs, tag, default=None):
    return next((v for t, v in tlvs if t == tag), default)


def mac_str(b):
    return ":".join(f"{x:02x}" for x in b)


def describe(msg):
    """Human-readable lines for a HELLO, STATUS or SCAN_RESULT."""
    t = msg["tlvs"]
    lines = []
    for tag, v in t:
        if tag == T_RESULT:
            lines.append(f"result: {RESULTS[v[0]] if v[0] < len(RESULTS) else v[0]}")
        elif tag in (T_MODEL, T_FW_VERSION, T_HW_REV, T_SSID, T_MESSAGE):
            name = {T_MODEL: "model", T_FW_VERSION: "fw", T_HW_REV: "hw", T_SSID: "ssid", T_MESSAGE: "message"}[tag]
            lines.append(f"{name}: {v.decode(errors='replace')}")
        elif tag == T_STATE:
            lines.append(f"state: {STATES[v[0]] if v[0] < len(STATES) else v[0]}")
        elif tag in (T_BRIDGE_MAC, T_POD_MAC, T_BSSID):
            lines.append(f"{ {T_BRIDGE_MAC: 'bridge-mac', T_POD_MAC: 'pod-mac', T_BSSID: 'bssid'}[tag]}: {mac_str(v)}")
        elif tag == T_CHANNEL:
            lines.append(f"channel: {v[0]}")
        elif tag == T_RSSI:
            lines.append(f"rssi: {struct.unpack('b', v)[0]}")
        elif tag == T_UPTIME:
            lines.append(f"uptime: {struct.unpack('>I', v)[0]} s")
        elif tag == T_REASON:
            lines.append(f"last-disconnect-reason: {v[0]}")
        elif tag == T_FLAGS:
            lines.append(f"flags: 0x{v[0]:02x}")
        elif tag == T_COUNTERS:
            vals = struct.unpack(">7I", v)
            lines.append("counters: " + " ".join(f"{n}={x}" for n, x in zip(COUNTERS, vals)))
        elif tag == T_AP:
            a = parse_tlvs(v)
            auth = first(a, T_AUTHMODE, b"\xff")[0]
            lines.append(f"{struct.unpack('b', first(a, T_RSSI))[0]:4d} {first(a, T_CHANNEL)[0]:2d} "
                         f"{AUTH[auth] if auth < len(AUTH) else 'other':9s} {mac_str(first(a, T_BSSID))} "
                         f"{first(a, T_SSID).decode(errors='replace')}")
    return lines


class Link:
    def __init__(self, ifname):
        self.sock = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(ETHERTYPE))
        self.sock.bind((ifname, 0))
        self.mac = self.sock.getsockname()[4]
        self.bridge = None
        self.seq = 0

    def recv(self, timeout):
        end = time.monotonic() + timeout
        while (left := end - time.monotonic()) > 0:
            self.sock.settimeout(left)
            try:
                msg = decode(self.sock.recv(2048))
            except socket.timeout:
                return None
            if msg:
                if msg["type"] == HELLO:
                    self.bridge = msg["src"]
                return msg
        return None

    def hello(self, timeout=5.0):
        end = time.monotonic() + timeout
        while (left := end - time.monotonic()) > 0:
            msg = self.recv(left)
            if msg and msg["type"] == HELLO:
                return msg
        raise SystemExit("no HELLO from a bridge")

    def request(self, mtype, tlvs=b"", timeout=1.0, retries=3):
        dst = self.bridge or BROADCAST
        self.seq = (self.seq % 0xFFFF) + 1
        frame = encode(dst, self.mac, mtype, self.seq, tlvs)
        for _ in range(retries):
            self.sock.send(frame)
            end = time.monotonic() + timeout
            while (left := end - time.monotonic()) > 0:
                msg = self.recv(left)
                if msg and msg["type"] == mtype | RESPONSE and msg["seq"] == self.seq:
                    return msg
        raise SystemExit(f"no response to request 0x{mtype:02x}")


def result_of(msg):
    r = first(msg["tlvs"], T_RESULT, b"\x09")[0]
    m = first(msg["tlvs"], T_MESSAGE)
    return r, (m.decode() if m else "")


def check(msg):
    r, m = result_of(msg)
    if r:
        raise SystemExit(f"error: {RESULTS[r] if r < len(RESULTS) else r} {m}")
    print("ok")


def ota(link, path):
    image = open(path, "rb").read()
    check(link.request(OTA_BEGIN, tlv(T_IMAGE_SIZE, struct.pack(">I", len(image))), timeout=30))
    off = 0
    while off < len(image):
        chunk = image[off:off + 1024]
        msg = link.request(OTA_DATA, tlv(T_OFFSET, struct.pack(">I", off)) + tlv(T_DATA, chunk))
        r, m = result_of(msg)
        nxt = struct.unpack(">I", first(msg["tlvs"], T_OFFSET, b"\0\0\0\0"))[0]
        if r not in (0, 6):  # 6: bad offset, resume from the bridge's offset
            raise SystemExit(f"error at {off}: {RESULTS[r] if r < len(RESULTS) else r} {m}")
        off = nxt
        print(f"\r{off}/{len(image)}", end="", flush=True)
    print()
    check(link.request(OTA_END, timeout=30))
    check(link.request(REBOOT))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-i", "--interface", required=True)
    ap.add_argument("command", choices=["hello", "status", "scan", "set-wifi", "clear-wifi", "reboot",
                                        "factory-reset", "ota"])
    ap.add_argument("args", nargs="*")
    a = ap.parse_args()
    link = Link(a.interface)
    hello = link.hello()
    if a.command == "hello":
        print("\n".join(describe(hello)))
    elif a.command == "status":
        print("\n".join(describe(link.request(STATUS))))
    elif a.command == "scan":
        print("\n".join(describe(link.request(SCAN, timeout=10, retries=1))))
    elif a.command == "set-wifi":
        if len(a.args) not in (1, 2):
            raise SystemExit("set-wifi <ssid> [passphrase]")
        body = tlv(T_SSID, a.args[0]) + (tlv(T_PASSPHRASE, a.args[1]) if len(a.args) == 2 else b"")
        check(link.request(SET_WIFI, body))
    elif a.command == "clear-wifi":
        check(link.request(CLEAR_WIFI))
    elif a.command == "reboot":
        check(link.request(REBOOT))
    elif a.command == "factory-reset":
        check(link.request(FACTORY_RESET))
    elif a.command == "ota":
        if len(a.args) != 1:
            raise SystemExit("ota <image.bin>")
        ota(link, a.args[0])


if __name__ == "__main__":
    sys.exit(main())
