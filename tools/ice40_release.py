#!/usr/bin/env python3
"""Hardware-verified iCE40 images for firmware releases.

A release must embed the gateware images that were tested on hardware, not a fresh CI build:
the CI toolchain (OSS CAD Suite, Linux) places the design differently from a local build, so
the same sources give a different bitstream with different clk48 margin (v40: loop 56.95 MHz
locally, 48.84 MHz in CI).  So the tested images are committed in ice40/release/ with a
MANIFEST, and the release job embeds those.

  promote  copy the current ice40 build (make -C ice40 images) into ice40/release/ and write the
           MANIFEST.  Run it AFTER that exact build passed on hardware; --hw-verified says how.
           Refuses an image older than any gateware source (a build from before the last edit).
           The MANIFEST records the toolchain (yosys, the nextpnr revision) and the synthesis and
           place-and-route flags the Makefile passes in, so a promoted image can be rebuilt.
  check    fail unless ice40/release/ is complete, untampered (sha256) and still matches the
           gateware sources (a source hash + GATEWARE_VERSION).  A gateware change without a
           new promote fails here: build, test on hardware, promote.
  install  check, then put the promoted images where the STM32 build embeds them
           (ice40/bench_pod_fpga_vbench_pod_{loop,deep}.bin + .gwversion).

The source hash covers what determines the bitstream's logic: ice40/src/*.v and *.vh, the pin
constraints and clocks.py.  It does not cover the Makefile (seeds, synthesis flags): a seed
change alone does not invalidate a promoted image, which stays the tested one.  `check` only
notes when the Makefile's flags no longer match the ones the promoted images were built with.
"""
import argparse
import datetime
import hashlib
import os
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


def image_path(kind, base=None):
    return (base or ICE40) / f"bench_pod_fpga_vbench_pod_{kind}.bin"


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def source_files():
    files = sorted((ICE40 / "src").glob("*.v")) + sorted((ICE40 / "src").glob("*.vh"))
    files += [ICE40 / "vbench_pod.pcf", ICE40 / "clocks.py"]
    return files


def build_inputs():
    """Everything an image's .bin is built from: the hashed sources plus the synthesis helpers."""
    return source_files() + sorted((ICE40 / "synth").glob("*.v")) + [ICE40 / "synth" / "check_dsp.py"]


def stale_against_sources(img):
    """The newest build input that is newer than img, or None when img is newer than all of them."""
    built = img.stat().st_mtime
    newer = [f for f in build_inputs() if f.exists() and f.stat().st_mtime > built]
    return max(newer, key=lambda f: f.stat().st_mtime) if newer else None


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


def nextpnr_revision(exe="nextpnr-ice40"):
    """The nextpnr revision: the version its --version prints, or, for a build that prints an
    empty one (Homebrew's prints "(Version )"), the package version from the install path.  The
    binary's sha256 is appended so two builds that report the same version stay distinguishable."""
    path = shutil.which(exe)
    if not path:
        return "unknown"
    rev = ""
    m = re.search(r"\(Version ([^)]*)\)", tool_version([path, "--version"]))
    if m:
        rev = m.group(1).strip()
    if not rev:
        real = pathlib.Path(os.path.realpath(path))
        parts = real.parts
        if "Cellar" in parts and parts.index("Cellar") + 2 < len(parts):
            i = parts.index("Cellar")
            rev = f"homebrew {parts[i + 1]} {parts[i + 2]}"
        else:
            rev = "unknown"
    try:
        rev += f" (sha256 {sha256(pathlib.Path(os.path.realpath(path)))[:16]})"
    except OSError:
        pass
    return rev


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
        newer = stale_against_sources(img)
        if newer is not None:
            sys.exit(f"ice40_release promote: {img.relative_to(ROOT)} is older than {newer.relative_to(ROOT)}, "
                     f"so it was not built from the current sources; rebuild with `make -C ice40 images`, "
                     f"test that build on hardware, then promote")
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
        f"nextpnr_revision={nextpnr_revision()}",
        f"synth_flags={args.synth_flags.strip() or 'unknown'}",
        f"pnr_flags={args.pnr_flags.strip() or 'unknown'}",
        f"hw_verified={args.hw_verified.strip()}",
    ]
    for kind in IMAGES:
        dst = image_path(kind, REL)
        shutil.copyfile(image_path(kind), dst)
        seed = args.seed_loop if kind == "loop" else args.seed_deep
        defines = args.defines_loop if kind == "loop" else args.defines_deep
        lines += [f"{kind}.sha256={sha256(dst)}", f"{kind}.seed={seed}",
                  f"{kind}.defines={defines.strip() or 'unknown'}",
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
    for note in flag_notes(m, args):
        print(f"ice40_release: note: {note}")


def flag_notes(m, args):
    """Where the Makefile's current flags differ from the ones the promoted images were built
    with.  Informational only: the promoted images stay the hardware-tested ones (see above)."""
    current = {"synth_flags": getattr(args, "synth_flags", ""), "pnr_flags": getattr(args, "pnr_flags", ""),
               "loop.defines": getattr(args, "defines_loop", ""),
               "deep.defines": getattr(args, "defines_deep", "")}
    notes = []
    for key, now in current.items():
        now = (now or "").strip()
        if not now:
            continue
        if key not in m:
            notes.append(f"the MANIFEST predates recording {key}")
        elif m[key] != now:
            notes.append(f"{key} is now '{now}', the promoted images were built with '{m[key]}'")
    return notes


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
    c = sub.add_parser("check")
    i = sub.add_parser("install")
    for q in (p, c, i):
        q.add_argument("--synth-flags", default="", help="yosys synth_ice40 flags (the Makefile's SYNTH_FLAGS)")
        q.add_argument("--pnr-flags", default="", help="nextpnr-ice40 flags without the seed")
        q.add_argument("--defines-loop", default="", help="Verilog defines of the loop image")
        q.add_argument("--defines-deep", default="", help="Verilog defines of the deep image")
    p.set_defaults(fn=cmd_promote)
    c.set_defaults(fn=cmd_check)
    i.set_defaults(fn=cmd_install)
    args = ap.parse_args()
    args.fn(args)


if __name__ == "__main__":
    main()
