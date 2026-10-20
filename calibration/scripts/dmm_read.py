#!/usr/bin/env python3
"""Read the bench DMM (34401A over the NI GPIB-USB-HS) from the command line.

A thin CLI over dmm.py, so a calibration run (or a Claude session) reads the DMM itself instead
of someone typing readings in. DC volts only.

    dmm_read.py idn                      # *IDN?: checks the adapter, the cable and the address
    dmm_read.py read                     # one reading, in volts
    dmm_read.py read -n 10 --json        # mean of 10 readings with min/max/std, as JSON
    dmm_read.py read --range 100         # the 100 V range (the +/-12 V output)
    dmm_read.py watch --interval 0.5     # a reading every 0.5 s until Ctrl-C (or --count N)

Ranges: 0.1, 1, 10 (default), 100, 1000 or auto. Fixed ranges are steadier for calibration;
the 10 V range covers 0 to 5 V, use 100 for the 12 V output. --nplc sets the integration time
in power-line cycles (10 = 0.2 s a reading, low noise; 1 = fast). --mock reads a fake DMM.

Needs libusb and pyusb on the Mac (calibration/scripts/requirements-host.txt):
    brew install libusb
    python3 -m venv calibration/.venv
    calibration/.venv/bin/pip install -r calibration/scripts/requirements-host.txt
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from dmm import Agilent34401A, DMMError, MockDMM  # noqa: E402

RANGES = ["0.1", "1", "10", "100", "1000", "auto"]


def open_dmm(args):
    if args.mock:
        return MockDMM()
    auto = args.range == "auto"
    dmm = Agilent34401A(gpib_addr=args.addr, nplc=args.nplc, vrange="DEF" if auto else args.range)
    if auto:
        dmm._write("VOLT:DC:RANG:AUTO ON")
    return dmm


def stats(values):
    n = len(values)
    mean = sum(values) / n
    std = math.sqrt(sum((v - mean) ** 2 for v in values) / (n - 1)) if n > 1 else 0.0
    return {"volts": mean, "n": n, "min": min(values), "max": max(values), "std": std}


def cmd_read(dmm, args):
    values = [dmm.read_dc() for _ in range(max(1, args.samples))]
    s = stats(values)
    if args.json:
        print(json.dumps({**s, "range": args.range, "nplc": args.nplc}))
    elif s["n"] == 1:
        print(f"{s['volts']:.7f} V")
    else:
        print(f"{s['volts']:.7f} V  (n={s['n']}, min {s['min']:.7f}, max {s['max']:.7f}, "
              f"std {s['std'] * 1e6:.1f} uV)")


def cmd_watch(dmm, args):
    k = 0
    try:
        while args.count == 0 or k < args.count:
            v = dmm.read_dc()
            stamp = time.strftime("%H:%M:%S")
            print(json.dumps({"t": stamp, "volts": v}) if args.json else f"{stamp}  {v:.7f} V", flush=True)
            k += 1
            if args.interval > 0:
                time.sleep(args.interval)
    except KeyboardInterrupt:
        pass


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--addr", type=int, default=22, help="GPIB primary address of the DMM (default 22)")
    ap.add_argument("--range", choices=RANGES, default="10", help="DC volts range (default 10)")
    ap.add_argument("--nplc", type=float, default=10, help="integration time in power-line cycles (default 10)")
    ap.add_argument("--mock", action="store_true", help="a fake DMM, no hardware")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("idn", help="identify the DMM")
    rd = sub.add_parser("read", help="read DC volts")
    rd.add_argument("-n", "--samples", type=int, default=1, help="readings to average (default 1)")
    rd.add_argument("--json", action="store_true")
    w = sub.add_parser("watch", help="read continuously")
    w.add_argument("--interval", type=float, default=0.5, help="seconds between readings (default 0.5)")
    w.add_argument("--count", type=int, default=0, help="stop after N readings (default: until Ctrl-C)")
    w.add_argument("--json", action="store_true")
    args = ap.parse_args(argv)
    try:
        dmm = open_dmm(args)
    except DMMError as e:
        sys.exit(f"dmm: {e}")
    except Exception as e:  # GPIB errors from ni_gpib_usb (a timeout: the DMM does not answer)
        sys.exit(f"dmm: the DMM does not answer on GPIB address {args.addr} ({e}). Is it on, is the "
                 "GPIB cable seated, and is its HP-IB address set to that (Shift > I/O MENU)?")
    try:
        if args.cmd == "idn":
            print(dmm.idn().strip())
        elif args.cmd == "read":
            cmd_read(dmm, args)
        else:
            cmd_watch(dmm, args)
    except DMMError as e:
        sys.exit(f"dmm: {e}")
    except Exception as e:
        sys.exit(f"dmm: GPIB error while reading ({e})")
    finally:
        dmm.close()


if __name__ == "__main__":
    main()
