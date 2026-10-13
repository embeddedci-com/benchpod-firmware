#!/usr/bin/env python3
"""Pin the USB console replies that host tools parse (docs/usb-serial-interface.md, "Stable replies").

benchpod CLI, the embeddedci SDK and the web app's setup wizard (WebSerial) read these console lines,
and older releases of them stay in use, so a change to one of these formats breaks them. This fails
when a pinned format string is no longer in console.c. Changing one on purpose means a new line
next to the old one (keep the old) and updating the tools first. Run from anywhere:
`python3 stm32h563/test/check_console_contract.py`.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
CONSOLE = os.path.join(HERE, "..", "src", "console.c")

# (reader, the exact C format literal in console.c)
PINNED = [
    ("pod detection (CLI, SDK, wizard)", r'"  device : benchpod\r\n"'),
    ("status ip (CLI show-network, SDK LAN fallback)", r'"  ip     : %s\r\n"'),
    ("status usb (CLI, SDK USB network)", r'"  usb    : net %s pod %s host %s link %s rx %lu (drop %lu) tx %lu (full %lu)\r\n"'),
    ("status board (CLI firmware version, nrst_pin)", r'"  board  : %s  fw v%s  rev %s  nrst_pin=%s  usb_cc=%s\r\n"'),
    ("ping (SDK)", r'"  PING ok  version=0x%02x\r\n"'),
    ("identity (CLI, SDK usb:<name>, wizard)", r'"identity %s %s\r\n"'),
    ("identity unknown", r'"identity unknown %s\r\n"'),
    ("identity-wipe", r'"identity-wipe ok %s\r\n"'),
    ("sig-policy", r'"sig-policy %s %u\r\n"'),
    ("lan-policy", r'"lan-policy %s\r\n"'),
    ("cloud CA list", r'"ca %s %.64s\r\n"'),
    ("cloud CA none", r'"ca none\r\n"'),
    ("cloud proxy", r'"proxy %s:%u %s\r\n"'),
    ("cloud proxy none", r'"proxy none\r\n"'),
    ("wifi-show ssid (CLI, wizard)", r'"  ssid: %s\r\n"'),
    ("wifi-show state", r'"  state: %s\r\n"'),
    ("wifi-show ip", r'"  ip: %s\r\n"'),
    ("blobs (install-blobs on older CLIs)", r'"blob %s %s %lu %lu %s\r\n"'),
    ("upload-begin", r'"upload-begin ok\r\n"'),
    ("upload-data", r'"upload-data ok %lu\r\n> "'),
    ("upload-end", r'"upload-end ok\r\n"'),
    ("upload-commit firmware", r'"upload-commit resetting\r\n"'),
    ("upload-commit blob", r'"upload-commit ok\r\n"'),
    ("usb-reattach", r'"usb-reattach ok\r\n"'),
    ("dfu", r'"  entering DFU (USB bootloader)'),
]


def main():
    with open(CONSOLE, encoding="utf-8") as f:
        src = f.read()
    missing = [(who, lit) for who, lit in PINNED if lit not in src]
    if missing:
        print("console contract: FAIL")
        for who, lit in missing:
            print(f"  {lit} ({who}) is no longer in console.c")
        print("These replies are read by host tools (docs/usb-serial-interface.md, Stable replies).")
        return 1
    print(f"console contract: PASS ({len(PINNED)} pinned replies)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
