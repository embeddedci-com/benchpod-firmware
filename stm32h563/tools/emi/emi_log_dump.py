#!/usr/bin/env python3
"""Download and decode the run log of a BenchPod running the EMI test build.

The EMI test build keeps every run (one power-on) in the W25Q: a RUN_START record at boot, an
EVENT on every LED or link change, a CHECKPOINT with the counters (when a value that shows trouble
changed, at most every 10 s, else once a minute; src/emi_log.h). This reads the
whole ring with the JSON command emi_log, a chunk at a time, and prints it oldest first, then a
summary per run.

  python3 emi_log_dump.py 192.168.1.150            # the pod's LAN address (raw API, port 8080)
  python3 emi_log_dump.py usb:benchpod-30a9f1      # any SDK connection, when embeddedci is installed
  python3 emi_log_dump.py HOST --raw log.bin       # also keep the raw slots
  python3 emi_log_dump.py --file log.bin           # decode a raw dump made earlier
  python3 emi_log_dump.py HOST --runs 5            # only the last 5 runs

While the loop runs, each chunk read stops the DAC for a moment (the W25Q shares the PSRAM bus);
the pod holds its next capture back between chunks so the download is not refused for long.
"""
import argparse
import base64
import json
import socket
import struct
import sys
import time
import zlib

REC = 64
MAGIC = 0x4C45
TYPES = {1: "START", 2: "EVENT", 3: "CHECK"}
LEDS = {1: "starting", 2: "green", 3: "red", 4: "blue", 5: "recovering"}
DACS = ["sine", "dc", "current", "off", "all"]
EVENTS = {1: "led", 2: "ethernet link lost", 3: "ethernet link back", 4: "activity started",
          5: "activity stopped", 6: "recovery",
          7: "ESP32-C3 restart", 8: "health"}


def _str(b):
    return b.split(b"\0", 1)[0].decode("ascii", "replace")


def decode(slot):
    """A 64-byte slot -> dict, or None when it is empty or not a valid record."""
    if len(slot) != REC or slot == b"\xff" * REC:
        return None
    magic, typ, led, seq, run, up = struct.unpack_from("<HBBIII", slot, 0)
    if magic != MAGIC or typ not in TYPES:
        return None
    if struct.unpack_from("<I", slot, 60)[0] != zlib.crc32(slot[:60]):
        return None
    r = {"type": TYPES[typ], "led": LEDS.get(led, "off"), "seq": seq, "run": run, "uptime_ms": up}
    if typ == 1:
        core, dac, mbit, gw = struct.unpack_from("<HBBI", slot, 48)
        r.update(fw=_str(slot[16:32]), reset=_str(slot[32:48]), core_mhz=core,
                 dac=DACS[dac] if dac < 5 else "?", net_mbit=mbit, gw_version=gw)
    elif typ == 2:
        kind, frm, to = slot[16], slot[17], slot[18]
        caps, errs, sent = struct.unpack_from("<III", slot, 20)
        r.update(kind=EVENTS.get(kind, f"event {kind}"), led_from=LEDS.get(frm, "off"),
                 led_to=LEDS.get(to, "off"), is_led=kind == 1, captures=caps, errors=errs,
                 net_sent=sent, text=_str(slot[32:60]))
    else:
        vals = struct.unpack_from("<IIIIIII", slot, 16)
        dac, mbit, running, link = slot[44], slot[45], slot[46], slot[47]
        r.update(zip(("captures", "errors", "net_sent", "net_drops", "link_losses", "rec_dropped",
                      "glitch_us_max"), vals))
        r.update(dac=DACS[dac] if dac < 5 else "?", net_mbit=mbit, running=bool(running),
                 link=bool(link), text=_str(slot[48:60]))
    return r


def encode(r):
    """dict -> 64 bytes (the host tests use it; the pod has its own encoder in C)."""
    typ = {v: k for k, v in TYPES.items()}[r["type"]]
    led = {v: k for k, v in LEDS.items()}.get(r.get("led"), 0)
    b = bytearray(REC)
    struct.pack_into("<HBBIII", b, 0, MAGIC, typ, led, r.get("seq", 0), r.get("run", 0),
                     r.get("uptime_ms", 0))
    if typ == 1:
        b[16:32] = r.get("fw", "").encode()[:16].ljust(16, b"\0")
        b[32:48] = r.get("reset", "").encode()[:16].ljust(16, b"\0")
        struct.pack_into("<HBBI", b, 48, r.get("core_mhz", 0), DACS.index(r.get("dac", "sine")),
                         r.get("net_mbit", 0), r.get("gw_version", 0))
    elif typ == 2:
        b[16] = {v: k for k, v in EVENTS.items()}.get(r.get("kind"), 1)
        b[17] = {v: k for k, v in LEDS.items()}.get(r.get("led_from"), 0)
        b[18] = {v: k for k, v in LEDS.items()}.get(r.get("led_to"), 0)
        struct.pack_into("<III", b, 20, r.get("captures", 0), r.get("errors", 0), r.get("net_sent", 0))
        b[32:60] = r.get("text", "").encode()[:27].ljust(28, b"\0")
    else:
        struct.pack_into("<IIIIIII", b, 16, *(r.get(k, 0) for k in (
            "captures", "errors", "net_sent", "net_drops", "link_losses", "rec_dropped",
            "glitch_us_max")))
        b[44] = DACS.index(r.get("dac", "sine"))
        b[45], b[46], b[47] = r.get("net_mbit", 0), int(r.get("running", 0)), int(r.get("link", 0))
        b[48:60] = r.get("text", "").encode()[:11].ljust(12, b"\0")
    struct.pack_into("<I", b, 60, zlib.crc32(bytes(b[:60])))
    return bytes(b)


def records(raw):
    """All valid records in a raw dump, oldest first (by sequence number)."""
    out = [r for r in (decode(raw[i:i + REC]) for i in range(0, len(raw) - REC + 1, REC)) if r]
    return sorted(out, key=lambda r: r["seq"])


def fmt(r):
    """The same line the pod prints for `emi log`."""
    up = r["uptime_ms"]
    head = f"#{r['seq']} run {r['run']} +{up // 1000}.{up % 1000 // 100}s "
    if r["type"] == "START":
        return head + (f"START fw {r['fw']} reset {r['reset']} core {r['core_mhz']} MHz dac {r['dac']} "
                       f"net {r['net_mbit']} Mbit/s gw {r['gw_version']}")
    if r["type"] == "EVENT":
        if r["kind"] == "recovery":
            return head + f"EVENT recovery: {r['text']}, captures {r['captures']}, errors {r['errors']}"
        what = f"led {r['led_from']} -> {r['led_to']}" if r["is_led"] else r["kind"]
        last = f", last: {r['text']}" if r["text"] else ""
        return head + (f"EVENT {what}, captures {r['captures']}, errors {r['errors']}, "
                       f"sent {r['net_sent']}{last}")
    last = f", last: {r['text']}" if r["text"] else ""
    return head + (f"CHECK led {r['led']}, {'running' if r['running'] else 'stopped'}, dac {r['dac']}, "
                   f"captures {r['captures']}, errors {r['errors']}, sent {r['net_sent']}, "
                   f"drops {r['net_drops']}, link {'up' if r['link'] else 'down'}, link losses "
                   f"{r['link_losses']}, glitch max {r['glitch_us_max']} us, lost records "
                   f"{r['rec_dropped']}{last}")


def summarize(recs):
    """One line per run: when it started, how long it lasted, what went wrong."""
    runs = {}
    for r in recs:
        s = runs.setdefault(r["run"], {"run": r["run"], "start": None, "last_ms": 0, "leds": set(),
                                       "events": 0, "last": None, "records": 0})
        s["records"] += 1
        s["last_ms"] = max(s["last_ms"], r["uptime_ms"])
        s["leds"].add(r["led"])
        if r["type"] == "START":
            s["start"] = r
        elif r["type"] == "EVENT":
            s["events"] += 1
            if r["is_led"]:
                s["leds"].add(r["led_to"])
        else:
            s["last"] = r
    lines = []
    for n in sorted(runs):
        s = runs[n]
        st = s["start"]
        head = (f"run {n}: fw {st['fw']}, reset {st['reset']}" if st else
                f"run {n}: (start record overwritten)")
        verdict = ("FAILED (failing over 60 s, or out of recovery steps)" if "red" in s["leds"]
                   else "FAILED, RECOVERED" if ("blue" in s["leds"] or "recovering" in s["leds"])
                   else "no failure")
        cp = s["last"]
        tail = (f", last checkpoint: captures {cp['captures']}, errors {cp['errors']}, drops "
                f"{cp['net_drops']}, link losses {cp['link_losses']}, glitch max {cp['glitch_us_max']} us"
                if cp else "")
        lines.append(f"{head}, ran {s['last_ms'] / 1000:.0f} s or more, {s['records']} records, "
                     f"{s['events']} events, LED: {verdict}{tail}")
    return lines


class RawTcp:
    """The pod's raw JSON API (port 8080): one JSON line out, one back, per connection."""

    def __init__(self, target, timeout=10.0):
        host, _, port = target.partition(":")
        self.addr = (host, int(port or 8080))
        self.timeout = timeout

    def command(self, req):
        with socket.create_connection(self.addr, self.timeout) as s:
            s.sendall((json.dumps(req) + "\n").encode())
            buf = b""
            while b"\n" not in buf:
                chunk = s.recv(65536)
                if not chunk:
                    break
                buf += chunk
        reply = json.loads(buf.split(b"\n", 1)[0])
        if reply.get("status") != "ok":
            raise RuntimeError(reply.get("message", "error"))
        return reply.get("data")


def connect(target):
    """An SDK connection when the embeddedci package is installed and the target needs it."""
    if target.startswith(("usb", "embeddedci:", "/dev/")) or target == "":
        from embeddedci.benchpod import BenchPod   # pip install embeddedci
        return BenchPod(target or None)
    return RawTcp(target)


def download(pod, progress=print):
    info = pod.command({"cmd": "emi_log"})
    slots, step = info["slots"], info.get("max_count", 16)
    progress(f"run log: {info['valid']} records in {slots} slots, this run {info['run']}; "
             f"reading {step} slots at a time")
    if "session_us_max" in info:
        progress(f"write errors {info.get('write_errors', 0)}, W25Q session last "
                 f"{info['session_us_last']} us, max {info['session_us_max']} us, DAC gap max "
                 f"{info.get('glitch_us_max', 0)} us ({info.get('dac_stops', 0)} sessions stopped it)")
    raw = bytearray()
    off = 0
    while off < slots:
        for attempt in range(100):
            try:
                d = pod.command({"cmd": "emi_log", "offset": off, "count": min(step, slots - off)})
                break
            except Exception as e:   # the SDK's PodBusyError, or RawTcp's RuntimeError
                if not str(e).startswith("busy") and "busy" not in type(e).__name__.lower():
                    raise
                time.sleep(0.2)
        else:
            raise RuntimeError("the pod stayed busy")
        data = d.get("data", "")
        raw += base64.urlsafe_b64decode(data + "=" * (-len(data) % 4))
        off += d["count"]
    return bytes(raw), info


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("target", nargs="?", help="pod address (host[:port]) or an SDK connection")
    ap.add_argument("--file", help="decode a raw dump (--raw) instead of reading a pod")
    ap.add_argument("--raw", help="also write the raw slots to this file")
    ap.add_argument("--runs", type=int, help="only the last N runs")
    ap.add_argument("--json", action="store_true", help="print the records as JSON lines")
    args = ap.parse_args(argv)
    if args.file:
        with open(args.file, "rb") as f:
            raw = f.read()
    elif args.target is not None:
        raw, _info = download(connect(args.target), progress=lambda s: print(s, file=sys.stderr))
        if args.raw:
            with open(args.raw, "wb") as f:
                f.write(raw)
    else:
        ap.error("give a pod address or --file")
    recs = records(raw)
    if args.runs:
        keep = sorted({r["run"] for r in recs})[-args.runs:]
        recs = [r for r in recs if r["run"] in keep]
    for r in recs:
        print(json.dumps(r) if args.json else fmt(r))
    if not args.json:
        print()
        for line in summarize(recs):
            print(line)
    return 0


if __name__ == "__main__":
    sys.exit(main())
