#!/usr/bin/env python3
"""Watch a BenchPod running the EMI test build over Ethernet, with no IP address.

The EMI test build (make EMI_TEST=1) sends two kinds of raw frames on EtherType 0x88B6:

  BPEMI  the traffic stream (default 20 Mbit/s): "BPEMI", a 32-bit sequence number and the
         pod's uptime in ms (little-endian), then filler
  BPMON  every 5 s, broadcast: one line of text, "BPMON run 12, reset power-on, uptime 61234 ms,
         seq 98765, emi: running, led green, ..." (the USB console's monitor line)

This script captures them (tcpdump, so it needs capture rights: sudo, or Wireshark's ChmodBPF on
macOS), prints every BPMON line and flags what a lab needs to know:

  LOST      sequence gaps in the stream (frames the laptop did not receive)
  STOPPED   no stream frames for --stall seconds (the pod still sends BPMON: the loop stopped)
  SILENT    no frames at all for --silent seconds (pod off, hung, or the cable is out)
  REBOOT    the run number changed, or the uptime or the sequence went back
  LED       the LED left green: blue (recovering) while a function fails, blue once it runs again,
            red when it failed for over 60 s or ran out of recovery steps (a fail)

Everything printed also goes to a log file (default emi-watch-<date>-<time>.log).

  sudo python3 emi_watch.py -i en7                 # live, on the interface the pod is cabled to
  python3 emi_watch.py -r capture.pcap             # a capture made earlier (tcpdump -w)

The LED is the official pass/fail check; this is a convenience for the operator outside the
chamber. Wireshark shows the same frames: filter `eth.type == 0x88b6`.
"""
import argparse
import datetime
import os
import re
import select
import struct
import subprocess
import sys
import time

ETHERTYPE = 0x88B6
SNAPLEN = 512   # covers a BPMON line; a BPEMI frame only needs its first 27 bytes


class PcapStream:
    """Parse a pcap byte stream (classic format, any endianness, us or ns stamps) incrementally."""

    def __init__(self):
        self.buf = bytearray()
        self.endian = None
        self.nano = False

    def feed(self, data):
        self.buf += data
        out = []
        if self.endian is None:
            if len(self.buf) < 24:
                return out
            magic = self.buf[:4]
            if magic in (b"\xd4\xc3\xb2\xa1", b"\x4d\x3c\xb2\xa1"):
                self.endian = "<"
            elif magic in (b"\xa1\xb2\xc3\xd4", b"\xa1\xb2\x3c\x4d"):
                self.endian = ">"
            else:
                raise ValueError("not a pcap stream (pcapng is not supported: use tcpdump -w)")
            self.nano = magic in (b"\x4d\x3c\xb2\xa1", b"\xa1\xb2\x3c\x4d")
            del self.buf[:24]
        while len(self.buf) >= 16:
            sec, frac, incl, _orig = struct.unpack(self.endian + "IIII", self.buf[:16])
            if len(self.buf) < 16 + incl:
                break
            ts = sec + frac / (1e9 if self.nano else 1e6)
            out.append((ts, bytes(self.buf[16:16 + incl])))
            del self.buf[:16 + incl]
        return out


_RUN = re.compile(r"run (\d+)")
_RESET = re.compile(r"reset ([\w-]+)")
_UPTIME = re.compile(r"uptime (\d+) ms")
_LED = re.compile(r"led (green \(starting\)|green|red|blue \(recovering\)|blue|off)")


class Watcher:
    """The analysis, independent of where the frames come from (live tcpdump or a file)."""

    def __init__(self, out, stall=2.0, silent=12.0):
        self.out = out
        self.stall = stall
        self.silent = silent
        self.pods = {}            # source MAC -> state
        self.alerts = []

    def say(self, ts, text, alert=None):
        stamp = datetime.datetime.fromtimestamp(ts).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        line = f"{stamp} {('[' + alert + '] ') if alert else ''}{text}"
        self.out(line)
        if alert:
            self.alerts.append((ts, alert, text))

    def pod(self, mac):
        p = self.pods.get(mac)
        if p is None:
            p = self.pods[mac] = {
                "seq": None, "uptime": None, "frames": 0, "lost": 0, "t_stream": None,
                "t_any": None, "stalled": False, "silent": False, "run": None, "mon_uptime": None,
                "led": None, "reboots": 0, "mon": 0,
            }
        return p

    def frame(self, ts, data):
        if len(data) < 19 or struct.unpack(">H", data[12:14])[0] != ETHERTYPE:
            return
        mac = ":".join(f"{b:02x}" for b in data[6:12])
        p = self.pod(mac)
        if p["silent"]:
            self.say(ts, f"{mac}: frames again after {ts - p['t_any']:.1f} s of silence", "BACK")
            p["silent"] = False
        p["t_any"] = ts
        tag = data[14:19]
        if tag == b"BPEMI" and len(data) >= 27:
            self.stream(ts, mac, p, *struct.unpack("<II", data[19:27]))
        elif tag == b"BPMON":
            self.monitor(ts, mac, p, data[14:].split(b"\0", 1)[0].decode("ascii", "replace"))

    def stream(self, ts, mac, p, seq, uptime):
        if p["stalled"]:
            self.say(ts, f"{mac}: stream running again after {ts - p['t_stream']:.1f} s", "RESUMED")
            p["stalled"] = False
        if p["seq"] is not None:
            if seq < p["seq"] or (p["uptime"] is not None and uptime + 1000 < p["uptime"]):
                p["reboots"] += 1
                self.say(ts, f"{mac}: stream restarted (seq {p['seq']} -> {seq}, uptime "
                             f"{p['uptime']} -> {uptime} ms): the pod rebooted", "REBOOT")
            elif seq > p["seq"] + 1:
                n = seq - p["seq"] - 1
                p["lost"] += n
                self.say(ts, f"{mac}: {n} stream frames lost (seq {p['seq']} -> {seq}), "
                             f"{p['lost']} in all", "LOST")
        p["seq"] = seq
        p["uptime"] = uptime
        p["frames"] += 1
        p["t_stream"] = ts

    def monitor(self, ts, mac, p, text):
        p["mon"] += 1
        self.say(ts, f"{mac}: {text}")
        m = _RUN.search(text)
        run = int(m.group(1)) if m else None
        m = _UPTIME.search(text)
        up = int(m.group(1)) if m else None
        reset = (_RESET.search(text) or [None, "?"])[1]
        if p["run"] is not None and run is not None and run != p["run"]:
            p["reboots"] += 1
            self.say(ts, f"{mac}: run {p['run']} -> {run} (reset cause {reset}): the pod rebooted",
                     "REBOOT")
        elif p["mon_uptime"] is not None and up is not None and up < p["mon_uptime"]:
            p["reboots"] += 1
            self.say(ts, f"{mac}: uptime went back ({p['mon_uptime']} -> {up} ms, reset cause "
                         f"{reset}): the pod rebooted", "REBOOT")
        if run is not None:
            p["run"] = run
        if up is not None:
            p["mon_uptime"] = up
        m = _LED.search(text)
        if m:
            led = m.group(1)
            if led != p["led"] and led in ("red", "blue (recovering)", "blue", "off"):
                meaning = {"red": "FAILED: failing over 60 s or out of recovery steps",
                           "blue (recovering)": "failing now, recovering",
                           "blue": "failed earlier, recovered", "off": "not running"}[led]
                self.say(ts, f"{mac}: LED {led} ({meaning})", "LED")
            elif led != p["led"] and p["led"] in ("red", "blue (recovering)", "blue", "off"):
                self.say(ts, f"{mac}: LED {led} again", "LED")
            p["led"] = led

    def tick(self, now):
        for mac, p in self.pods.items():
            if p["t_stream"] is not None and not p["stalled"] and now - p["t_stream"] > self.stall:
                p["stalled"] = True
                self.say(now, f"{mac}: no stream frames for {now - p['t_stream']:.1f} s", "STOPPED")
            if p["t_any"] is not None and not p["silent"] and now - p["t_any"] > self.silent:
                p["silent"] = True
                self.say(now, f"{mac}: no frames at all for {now - p['t_any']:.1f} s "
                              "(pod off or hung, or no link)", "SILENT")

    def summary(self):
        lines = []
        for mac, p in self.pods.items():
            total = p["frames"] + p["lost"]
            pct = 100.0 * p["lost"] / total if total else 0.0
            lines.append(f"{mac}: {p['frames']} stream frames, {p['lost']} lost ({pct:.3f} %), "
                         f"{p['mon']} status lines, {p['reboots']} reboots, last LED {p['led']}, "
                         f"run {p['run']}")
        if not self.pods:
            lines.append("no BenchPod frames seen")
        lines.append(f"{len(self.alerts)} alerts")
        return lines


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("-i", "--iface", help="capture live on this interface (needs capture rights)")
    src.add_argument("-r", "--read", help="read a pcap file instead")
    ap.add_argument("--log", help="log file (default emi-watch-<date>-<time>.log; '-' for none)")
    ap.add_argument("--stall", type=float, default=2.0, help="seconds without stream frames (2)")
    ap.add_argument("--silent", type=float, default=12.0, help="seconds without any frame (12)")
    ap.add_argument("--quiet", action="store_true", help="print only alerts (the log has all)")
    args = ap.parse_args(argv)

    log_path = args.log or datetime.datetime.now().strftime("emi-watch-%Y%m%d-%H%M%S.log")
    log = None if log_path == "-" else open(log_path, "a", buffering=1, encoding="utf-8")

    def out(line):
        if log:
            log.write(line + "\n")
        if not args.quiet or line.split(" ", 2)[2].startswith("["):
            print(line, flush=True)

    w = Watcher(out, stall=args.stall, silent=args.silent)
    ps = PcapStream()
    if log:
        print(f"logging to {os.path.abspath(log_path)}", file=sys.stderr)
    try:
        if args.read:
            with open(args.read, "rb") as f:
                for ts, data in ps.feed(f.read()):
                    w.tick(ts)
                    w.frame(ts, data)
        else:
            cmd = ["tcpdump", "-i", args.iface, "-U", "-s", str(SNAPLEN), "-w", "-",
                   f"ether proto 0x{ETHERTYPE:04x}"]
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE)
            fd = proc.stdout.fileno()
            print(f"capturing on {args.iface} ({' '.join(cmd)}); Ctrl-C to stop", file=sys.stderr)
            while True:
                r, _, _ = select.select([fd], [], [], 0.5)
                if r:
                    data = os.read(fd, 1 << 16)
                    if not data:
                        sys.exit(f"tcpdump exited ({proc.wait()}): capture rights? try sudo")
                    for ts, frame in ps.feed(data):
                        w.frame(ts, frame)
                w.tick(time.time())
    except KeyboardInterrupt:
        pass
    for line in w.summary():
        out(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S.000} summary: {line}")
    if log:
        log.close()
    return 1 if w.alerts else 0


if __name__ == "__main__":
    sys.exit(main())
