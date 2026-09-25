#!/usr/bin/env python3
"""Hardware-verified iCE40 images for firmware releases.

A release must embed the gateware images that were tested on hardware, not a fresh CI build:
the CI toolchain (OSS CAD Suite, Linux) places the design differently from a local build, so
the same sources give a different bitstream with different clk48 margin (v40: loop 56.95 MHz
locally, 48.84 MHz in CI).  So the tested images are committed in ice40/release/ with a
MANIFEST, and the release job embeds those.

  promote  copy the current ice40 build (make -C ice40 images) into ice40/release/ and write the
           MANIFEST.  Run it AFTER that exact build passed on hardware; --hw-verified says how.
  check    fail unless ice40/release/ is complete, untampered (sha256) and still matches the
           gateware sources (a source hash + GATEWARE_VERSION).  A gateware change without a
           new promote fails here: build, test on hardware, promote.
  install  check, then put the promoted images where the STM32 build embeds them
           (ice40/bench_pod_fpga_vbench_pod_{loop,deep}.bin + .gwversion).

The source hash covers what determines the bitstream's logic: ice40/src/*.v and *.vh, the pin
constraints and clocks.py.  It does not cover the Makefile (seeds, synthesis flags): a seed
change alone does not invalidate a promoted image, which stays the tested one.
"""
import argparse
import datetime
import hashlib
import pathlib
import re
import shutil
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
ICE40 = ROOT / "ice40"
REL = ICE40 / "release"
MANIFEST = REL / "MANIFEST"
IMAGES = ("loop", "deep")


def image_path(kind, base=ICE40):
    return base / f"bench_pod_fpga_vbench_pod_{kind}.bin"


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def source_files():
    files = sorted((ICE40 / "src").glob("*.v")) + sorted((ICE40 / "src").glob("*.vh"))
    files += [ICE40 / "vbench_pod.pcf", ICE40 / "clocks.py"]
    return files


def source_hash():
    h = hashlib.sha256()
    for f in source_files():
        h.update(f.relative_to(ICE40).as_posix().encode() + b"\0")
        h.update(f.read_bytes().replace(b"\r\n", b"\n") + b"\0")
    return h.hexdigest()


def source_gw_version():
    m = re.search(r"GATEWARE_VERSION\(8'd(\d+)\)", (ICE40 / "src" / "top_v2.v").read_text())
    if not m:
        sys.exit("ice40_release: cannot find GATEWARE_VERSION in ice40/src/top_v2.v")
    return m.group(1)


def read_manifest():
    if not MANIFEST.exists():
        return None
    out = {}
    for line in MANIFEST.read_text().splitlines():
        if line and not line.startswith("#") and "=" in line:
            k, v = line.split("=", 1)
            out[k.strip()] = v.strip()
    return out


def tool_version(cmd):
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, check=True)
        return (r.stdout.strip() or r.stderr.strip()).splitlines()[0]   # nextpnr prints to stderr
    except Exception:
        return "unknown"


def clk48_fmax(kind):
    log = ICE40 / f"bench_pod_fpga_vbench_pod_{kind}.asc.pnr.log"
    if not log.exists():
        return "unknown"
    found = re.findall(r"Max frequency for clock 'clk48[^:]*: *([0-9.]+) MHz", log.read_text())
    return found[-1] if found else "unknown"


def git_commit():
    try:
        rev = subprocess.run(["git", "-C", str(ROOT), "rev-parse", "--short", "HEAD"],
                             capture_output=True, text=True, check=True).stdout.strip()
        dirty = subprocess.run(["git", "-C", str(ROOT), "status", "--porcelain", "--", "ice40/src",
                                "ice40/vbench_pod.pcf", "ice40/clocks.py"],
                               capture_output=True, text=True, check=True).stdout.strip()
        return rev + (" (gateware sources modified)" if dirty else "")
    except Exception:
        return "unknown"


def cmd_promote(args):
    if not args.hw_verified.strip():
        sys.exit("ice40_release promote: --hw-verified is required (what hardware test this exact build passed)")
    gw = source_gw_version()
    for kind in IMAGES:
        img = image_path(kind)
        if not img.exists():
            sys.exit(f"ice40_release promote: {img.relative_to(ROOT)} missing; run `make -C ice40 images` first")
        tag = (img.parent / (img.name + ".gwversion"))
        if tag.exists() and tag.read_text().strip() != gw:
            sys.exit(f"ice40_release promote: {img.name} was built as gateware v{tag.read_text().strip()}, "
                     f"but the sources say v{gw}; rebuild with `make -C ice40 images`")
    REL.mkdir(exist_ok=True)
    lines = [
        "# Hardware-verified iCE40 images embedded by firmware releases (tools/ice40_release.py).",
        "# Written by `make -C ice40 promote`; checked by `make -C ice40 check-release`.",
        f"gateware_version={gw}",
        f"source_sha256={source_hash()}",
        f"built_from={git_commit()}",
        f"promoted={datetime.date.today().isoformat()}",
        f"yosys={tool_version(['yosys', '-V'])}",
        f"nextpnr={tool_version(['nextpnr-ice40', '--version'])}",
        f"hw_verified={args.hw_verified.strip()}",
    ]
    for kind in IMAGES:
        dst = image_path(kind, REL)
        shutil.copyfile(image_path(kind), dst)
        seed = args.seed_loop if kind == "loop" else args.seed_deep
        lines += [f"{kind}.sha256={sha256(dst)}", f"{kind}.seed={seed}",
                  f"{kind}.clk48_mhz={clk48_fmax(kind)}"]
    MANIFEST.write_text("\n".join(lines) + "\n")
    print(f"promoted gateware v{gw} to {REL.relative_to(ROOT)}/ (commit it with the sources)")


def check():
    """Return a list of problems (empty = the promoted images are good to embed)."""
    m = read_manifest()
    if m is None:
        return [f"{MANIFEST.relative_to(ROOT)} missing: no hardware-verified images promoted"]
    problems = []
    gw = source_gw_version()
    if m.get("gateware_version") != gw:
        problems.append(f"promoted images are gateware v{m.get('gateware_version')}, the sources are v{gw}")
    if m.get("source_sha256") != source_hash():
        problems.append("the gateware sources changed since the images were promoted")
    for kind in IMAGES:
        img = image_path(kind, REL)
        if not img.exists():
            problems.append(f"{img.relative_to(ROOT)} missing")
        elif sha256(img) != m.get(f"{kind}.sha256"):
            problems.append(f"{img.relative_to(ROOT)} does not match its MANIFEST sha256")
    if not m.get("hw_verified"):
        problems.append("MANIFEST has no hw_verified record")
    return problems


def cmd_check(args):
    problems = check()
    if problems:
        for p in problems:
            print(f"ice40_release: {p}", file=sys.stderr)
        print("ice40_release: build (`make -C ice40 images`), test that build on hardware, then "
              "`make -C ice40 promote HW_VERIFIED=\"...\"` and commit ice40/release/.", file=sys.stderr)
        sys.exit(1)
    m = read_manifest()
    print(f"ice40_release: gateware v{m['gateware_version']} images match the sources "
          f"(hardware: {m['hw_verified']})")


def cmd_install(args):
    cmd_check(args)
    gw = read_manifest()["gateware_version"]
    for kind in IMAGES:
        dst = image_path(kind)
        shutil.copyfile(image_path(kind, REL), dst)
        (dst.parent / (dst.name + ".gwversion")).write_text(gw + "\n")
    print(f"ice40_release: installed the promoted v{gw} images for the firmware build")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("promote")
    p.add_argument("--hw-verified", required=True)
    p.add_argument("--seed-loop", default="unknown")
    p.add_argument("--seed-deep", default="unknown")
    p.set_defaults(fn=cmd_promote)
    sub.add_parser("check").set_defaults(fn=cmd_check)
    sub.add_parser("install").set_defaults(fn=cmd_install)
    args = ap.parse_args()
    args.fn(args)


if __name__ == "__main__":
    main()
