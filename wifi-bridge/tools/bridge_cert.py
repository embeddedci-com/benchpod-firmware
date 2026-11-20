#!/usr/bin/env python3
"""Provision a Wi-Fi bridge's identity certificate through the pod it is plugged into.

Pods from firmware 3.10.7 send the Wi-Fi password only to a bridge that shows a certificate for its
identity key (wifi-bridge/docs/SECURITY.md, "Bridge identity"). The steps, with the bridge on the
pod's Ethernet port and the pod on USB:

    python3 bridge_cert.py identity --host 10.237.8.85
        prints the bridge's identity key (64 hex digits)
    sign it: Actions > "Sign bridge identity" with that key (release key, needs approval), or for a
        development pod: python3 tools/fwsign.py sign-bridge-id --key ~/.config/benchpod/fw-signing-dev.key \\
            --id <key> --release 3.10.7 -o bridge-id.sig
    python3 bridge_cert.py install --host 10.237.8.85 bridge-id.sig
        sends it in four parts (JSON wifi_bridge action cert), then reads wifi_status until the pod
        reports the bridge as genuine

Firmware 4.0.0 and later: talks JSON to the pod over its USB network adapter (the pod's address on
the USB link is `status.usb.ip`, 10.237.x.y; the certificate is accepted over the USB link only).
Firmware up to 3.10.13 did this with the console's `bridge cert`. The pod forwards the certificate
over the signed control channel; the bridge checks it against its own key and the trusted keys
before it keeps it.
"""
import argparse
import json
import re
import socket
import sys
import time

CLIENT_API = 2   # features/features.toml api


def command(host, cmd, timeout=10.0):
    """One JSON command over plain TCP 8080 (the USB link); returns the reply's data or exits."""
    cmd = dict(cmd, client_api=CLIENT_API)
    with socket.create_connection((host, 8080), timeout=timeout) as s:
        s.sendall((json.dumps(cmd) + "\n").encode())
        buf = b""
        while b"\n" not in buf:
            chunk = s.recv(4096)
            if not chunk:
                break
            buf += chunk
    reply = json.loads(buf.split(b"\n", 1)[0])
    if reply.get("status") != "ok":
        sys.exit(f"{cmd['cmd']}: {reply.get('code') or ''} {reply.get('args') or reply.get('message') or reply}")
    return reply.get("data")


def bridge(host):
    data = command(host, {"cmd": "wifi_status"})
    b = (data or {}).get("bridge")
    if not b:
        sys.exit("no Wi-Fi bridge in use on the pod's Ethernet port (or built-in Wi-Fi is configured)")
    return b


def read_cert(path):
    data = open(path, "rb").read()
    if len(data) != 128:
        text = data.decode(errors="replace").strip()
        if re.fullmatch(r"[0-9a-fA-F]{256}", text):
            data = bytes.fromhex(text)
    if len(data) != 128:
        sys.exit(f"{path}: a certificate is 128 bytes (bridge-id.sig) or 256 hex digits (bridge-id.hex)")
    if data[:4] != b"BPSG" or data[6] != 8:
        sys.exit(f"{path}: not a bridge identity certificate (target bridge-id)")
    return data


def cmd_identity(a):
    b = bridge(a.host)
    if not b.get("identity"):
        sys.exit("no identity reported: bridge firmware before 0.1.3")
    print(b["identity"])
    print(f"genuine: {'yes' if b.get('genuine') else 'no'}", file=sys.stderr)


def cmd_install(a):
    hexs = read_cert(a.cert).hex()
    for part in range(4):
        data = command(a.host, {"cmd": "wifi_bridge", "action": "cert", "part": part + 1,
                                "hex": hexs[64 * part:64 * (part + 1)]})
    if not (data or {}).get("stored"):
        sys.exit(f"the pod did not send the certificate to the bridge: {data}")
    print("certificate stored on the bridge")
    for _ in range(15):
        time.sleep(2)
        b = bridge(a.host)
        if b.get("genuine"):
            print(f"bridge {b.get('identity')} is genuine")
            return 0
    sys.exit("the pod does not report the bridge as genuine yet: check wifi_status bridge.genuine")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", choices=["identity", "install"])
    ap.add_argument("cert", nargs="?", help="install: bridge-id.sig (or .hex)")
    ap.add_argument("--host", required=True, help="the pod's address on the USB link (status.usb.ip), e.g. 10.237.8.85")
    a = ap.parse_args()
    if a.command == "install" and not a.cert:
        ap.error("install needs the certificate file")
    return cmd_identity(a) if a.command == "identity" else cmd_install(a)


if __name__ == "__main__":
    sys.exit(main())
