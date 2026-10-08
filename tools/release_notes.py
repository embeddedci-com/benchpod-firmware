#!/usr/bin/env python3
"""Draft firmware release notes from the pull requests merged since the previous release.

    tools/release_notes.py --tag v3.7.0                    # after tagging stm32-v3.7.0
    tools/release_notes.py --tag v3.7.0 --ref origin/main  # draft before tagging
    tools/release_notes.py --tag v3.7.0 --previous v3.5.1  # compare with another release

Reads only the local git history (fetch the tags first). The changes are the pull requests
merged into main between the previous stm32-v* tag and the release: GitHub's merge commits
("Merge pull request #N from ..." with the PR title as the body) and squash merges ("Title (#N)"),
leaving out a pull request whose branch the previous release was already built from.
Also prints the firmware version (stm32h563/src/version.h) and the embedded gateware version
(ice40/release/MANIFEST) at that commit. The output is a draft: review it, add the hardware test
line, then `gh release edit vX.Y.Z --notes-file notes.md`. The release job can call it later.
"""

import argparse
import re
import subprocess
import sys

REPO_URL = "https://github.com/embeddedci-com/benchpod-firmware"
MERGE_RE = re.compile(r"^Merge pull request #(\d+) from \S+")
SQUASH_RE = re.compile(r"^(.*\S)\s+\(#(\d+)\)$")


def git(*args, check=True):
    r = subprocess.run(["git", *args], capture_output=True, text=True)
    if check and r.returncode != 0:
        raise SystemExit(f"release_notes: git {' '.join(args)}: {r.stderr.strip()}")
    return r.stdout if r.returncode == 0 else None


def version_key(tag):
    return tuple(int(x) for x in re.findall(r"\d+", re.sub(r"^(stm32-)?v", "", tag))[:3])


def resolve_ref(version, ref):
    """The commit the release is built from: --ref, else stm32-v<version>, else v<version>."""
    for candidate in ([ref] if ref else [f"stm32-v{version}", f"v{version}"]):
        if git("rev-parse", "--verify", "--quiet", candidate + "^{commit}", check=False):
            return candidate
    raise SystemExit(f"release_notes: no tag stm32-v{version} or v{version}; pass --ref")


def previous_tag(version, ref):
    """The newest stm32-v* tag older than version that is an ancestor of ref."""
    tags = (git("tag", "--list", "stm32-v*") or "").split()
    older = sorted((t for t in tags if version_key(t) < version_key(version)), key=version_key,
                   reverse=True)
    for t in older:
        if git("merge-base", "--is-ancestor", t, ref, check=False) is not None:
            return t
    return None


def pull_requests(base, ref):
    """[(number, title)] merged into the first-parent line of ref since base, oldest first."""
    rng = f"{base}..{ref}" if base else ref
    out = git("log", "--first-parent", "--reverse", "--format=%P%x00%s%x00%b%x1e", rng)
    prs = []
    for rec in out.split("\x1e"):
        rec = rec.strip("\n")
        if not rec:
            continue
        parents, subject, body = (rec.split("\x00") + ["", ""])[:3]
        m = MERGE_RE.match(subject)
        if m:
            # A release can be tagged on a pull request's branch before main merges it; that
            # PR's work is already in the previous release.
            head = parents.split()[-1]
            if base and git("merge-base", "--is-ancestor", head, base, check=False) is not None:
                continue
            title = next((line.strip() for line in body.splitlines() if line.strip()), "")
            prs.append((int(m.group(1)), title or subject))
            continue
        m = SQUASH_RE.match(subject)
        if m:
            prs.append((int(m.group(2)), m.group(1)))
    return prs


def file_at(ref, path):
    return git("show", f"{ref}:{path}", check=False)


def render(version, ref, base, prs):
    fw = re.search(r'#define\s+FIRMWARE_VERSION\s+"([^"]+)"',
                   file_at(ref, "stm32h563/src/version.h") or "")
    gw = re.search(r"^gateware_version=(\d+)", file_at(ref, "ice40/release/MANIFEST") or "", re.M)
    lines = [f"BenchPod firmware {version}", ""]
    if fw and fw.group(1) != version:
        lines += [f"> Warning: stm32h563/src/version.h at {ref} says {fw.group(1)}, not {version}.", ""]
    lines += [f"- Firmware: {fw.group(1) if fw else '?'}",
              f"- Embedded gateware: v{gw.group(1) if gw else '?'} (updated at boot)",
              ""]
    lines.append("## Changes")
    lines.append("")
    if prs:
        for number, title in prs:
            lines.append(f"- {title} ([#{number}]({REPO_URL}/pull/{number}))")
    else:
        lines.append("- No pull requests merged since the previous release.")
    lines.append("")
    lines.append("## Tested")
    lines.append("")
    lines.append("- TODO: the bench pod, the suites and their results for the published build.")
    lines.append("")
    if base:
        prev = "v" + base.removeprefix("stm32-v")
        lines.append(f"Full diff: {REPO_URL}/compare/{base}...stm32-v{version}  (previous release {prev})")
    return "\n".join(lines) + "\n"


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--tag", required=True, help="release version, v3.7.0 or stm32-v3.7.0")
    ap.add_argument("--ref", help="commit to describe (default: the release's tag)")
    ap.add_argument("--previous", help="previous release to compare with (default: the newest older stm32-v* tag)")
    args = ap.parse_args(argv)
    version = re.sub(r"^(stm32-)?v", "", args.tag)
    if not re.fullmatch(r"\d+\.\d+\.\d+", version):
        ap.error(f"--tag {args.tag}: want vX.Y.Z")
    ref = resolve_ref(version, args.ref)
    if args.previous:
        base = "stm32-v" + re.sub(r"^(stm32-)?v", "", args.previous)
    else:
        base = previous_tag(version, ref)
    sys.stdout.write(render(version, ref, base, pull_requests(base, ref)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
