#!/usr/bin/env python3
"""Provision a Wi-Fi bridge's identity certificate through the pod it is plugged into.

Pods from firmware 3.10.7 send the Wi-Fi password only to a bridge that shows a certificate for its
identity key (wifi-bridge/docs/SECURITY.md, "Bridge identity"). The steps, with the bridge on the
pod's Ethernet port and the pod on USB:

    python3 bridge_cert.py identity --port /dev/cu.usbmodemXXXX
        prints the bridge's identity key (64 hex digits)
    sign it: Actions > "Sign bridge identity" with that key (release key, needs approval), or for a
        development pod: python3 tools/fwsign.py sign-bridge-id --key ~/.config/benchpod/fw-signing-dev.key \\
            --id <key> --release 3.10.7 -o bridge-id.sig
    python3 bridge_cert.py install --port /dev/cu.usbmodemXXXX bridge-id.sig
        sends it in four parts (`bridge cert <1-4> <hex>`), then shows `bridge` until the pod
        reports the bridge as genuine

Talks to the pod's USB console (pyserial). The pod forwards the certificate over the signed
control channel; the bridge checks it against its own key and the trusted keys before it keeps it.
"""
import argparse
import re
import sys
import time

try:
    import serial
except ImportError:  # pragma: no cover
    sys.exit("pip install pyserial")


def console(port, line, wait=3.0):
    """One console line; returns what the pod printed up to its next prompt."""
    with serial.Serial(port, 115200, timeout=0.2) as s:
        s.reset_input_buffer()
        s.write((line + "\r\n").encode())
        out, deadline = b"", time.time() + wait
        while time.time() < deadline:
            out += s.read(4096)
            if out.rstrip().endswith(b">") and line.encode() in out:
                break
        return out.decode(errors="replace")


def bridge_lines(port):
    text = console(port, "bridge")
    ident = re.search(r"identity: ([0-9a-f]{64})", text)
    genuine = re.search(r"genuine: (yes|no)", text)
    return text, ident.group(1) if ident else None, genuine.group(1) if genuine else None


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
    text, ident, genuine = bridge_lines(a.port)
    if not ident:
        sys.exit("no identity reported: no bridge on the pod's Ethernet port, or bridge firmware before 0.1.3\n"
                 + text)
    print(ident)
    print(f"genuine: {genuine}", file=sys.stderr)


def cmd_install(a):
    cert = read_cert(a.cert)
    hexs = cert.hex()
    for part in range(4):
        text = console(a.port, f"bridge cert {part + 1} {hexs[64 * part:64 * (part + 1)]}", wait=5.0)
        if "error" in text or ("stored" not in text):
            sys.exit(f"part {part + 1}: {text.strip()}")
    print(text.strip().splitlines()[-2] if "\n" in text.strip() else text.strip())
    for _ in range(15):
        time.sleep(2)
        _, ident, genuine = bridge_lines(a.port)
        if genuine == "yes":
            print(f"bridge {ident} is genuine")
            return 0
    sys.exit("the pod does not report the bridge as genuine yet: check `bridge` on its console")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", choices=["identity", "install"])
    ap.add_argument("cert", nargs="?", help="install: bridge-id.sig (or .hex)")
    ap.add_argument("--port", required=True, help="the pod's USB console, e.g. /dev/cu.usbmodemXXXX")
    a = ap.parse_args()
    if a.command == "install" and not a.cert:
        ap.error("install needs the certificate file")
    return cmd_identity(a) if a.command == "identity" else cmd_install(a)


if __name__ == "__main__":
    sys.exit(main())
