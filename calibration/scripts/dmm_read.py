#!/usr/bin/env python3
"""Read the bench DMM from the command line.

A thin CLI over dmm.py, so a calibration run (or a Claude session) reads the DMM itself instead
of someone typing readings in. DC volts only. The meter is found on its own: an HP/Agilent
34401A on the NI GPIB-USB-HS, or an OWON XDM1041/XDM2041 on its USB serial port.

    dmm_read.py idn                      # *IDN?: which meter answered (checks cable, adapter, port)
    dmm_read.py read                     # one reading, in volts
    dmm_read.py read -n 10 --json        # mean of 10 readings with min/max/std, as JSON
    dmm_read.py --range 5.5 read         # the smallest range that reads up to 5.5 V
    dmm_read.py watch --interval 0.5     # a reading every 0.5 s until Ctrl-C (or --count N)
    dmm_read.py --dmm owon --port /dev/cu.usbserial-1410 idn   # that meter, on that port

--range is the largest voltage you expect, in volts, or auto (default 10). Each meter picks
its smallest range that reads that far: 5.5 is the 34401A's 10 V range and the OWON's 5 V
range, 10 is the 34401A's 10 V range and the OWON's 50 V range. Fixed ranges are steadier for
calibration. --nplc sets the integration time in power-line cycles on the 34401A (10 = 0.2 s a
reading, low noise; 1 = fast); the OWON has slow, medium and fast instead: 5 and above is slow.
--dmm picks the meter (auto, 34401a, owon or mock; default auto, or $DMM); --port is the
OWON's serial port (default $DMM_PORT, else the USB serial port that answers as an OWON).

Needs libusb, pyusb and pyserial on the Mac (calibration/scripts/requirements-host.txt):
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
from dmm import KINDS, DMMError, open_dmm  # noqa: E402


def range_arg(s):
    """argparse type for --range: a positive number of volts, or auto."""
    if s.strip().lower() == "auto":
        return "auto"
    try:
        v = float(s)
    except ValueError:
        raise argparse.ArgumentTypeError(f"{s!r} is not a number of volts or auto") from None
    if v <= 0:
        raise argparse.ArgumentTypeError("the range must be above 0 V")
    if v > 1000:
        raise argparse.ArgumentTypeError("the meters read up to 1000 V")
    return s


def stats(values):
    n = len(values)
    mean = sum(values) / n
    std = math.sqrt(sum((v - mean) ** 2 for v in values) / (n - 1)) if n > 1 else 0.0
    return {"volts": mean, "n": n, "min": min(values), "max": max(values), "std": std}


def cmd_read(dmm, args):
    values = [dmm.read_dc() for _ in range(max(1, args.samples))]
    s = stats(values)
    if args.json:
        print(json.dumps({**s, "range": args.range, "nplc": args.nplc, "meter": dmm.name,
                          "meter_range": dmm.range_label}))
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
    ap.add_argument("--dmm", choices=KINDS, default=None,
                    help="which meter (default auto: the first one that answers; or $DMM)")
    ap.add_argument("--port", default=None,
                    help="the OWON's serial port (default $DMM_PORT, else auto-detected)")
    ap.add_argument("--addr", type=int, default=22, help="GPIB primary address of the 34401A (default 22)")
    ap.add_argument("--range", type=range_arg, default="10",
                    help="largest voltage expected, in volts, or auto (default 10)")
    ap.add_argument("--nplc", type=float, default=10,
                    help="integration time in power-line cycles (default 10; OWON: 5+ = slow)")
    ap.add_argument("--mock", action="store_true", help="a fake DMM, no hardware (= --dmm mock)")
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
        dmm = open_dmm("mock" if args.mock else args.dmm, vrange=args.range, nplc=args.nplc,
                       gpib_addr=args.addr, port=args.port)
    except DMMError as e:
        sys.exit(f"dmm: {e}. Is the meter on and its USB cable in? A 34401A needs its HP-IB "
                 f"address set to {args.addr} (Shift > I/O MENU); an OWON may need --port.")
    try:
        if args.cmd == "idn":
            print(dmm.idn().strip())
        elif args.cmd == "read":
            cmd_read(dmm, args)
        else:
            cmd_watch(dmm, args)
    except DMMError as e:
        sys.exit(f"dmm: {e}")
    except Exception as e:  # GPIB or serial errors from the transport
        sys.exit(f"dmm: {dmm.name} error while reading ({e})")
    finally:
        dmm.close()


if __name__ == "__main__":
    main()
