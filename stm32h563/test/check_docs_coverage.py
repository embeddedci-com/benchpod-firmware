#!/usr/bin/env python3
"""Fail when a command the firmware accepts is missing from the public docs.

Three surfaces, each read from the code:

  JSON verbs       src/cmd_tier.c (k_tiers)         -> a row in docs/API.md's Commands index
  SCPI commands    src/scpi_server.c (.pattern)     -> docs/API.md's SCPI reference
  Console commands src/console.c (argv[0] / cmd)    -> docs/usb-serial-interface.md

A command that is internal on purpose goes in the matching *_UNDOCUMENTED set below, with a
reason. Run from anywhere: `python3 stm32h563/test/check_docs_coverage.py`. Exits 1 on a gap.
"""
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
SRC = os.path.join(ROOT, "stm32h563", "src")
DOCS = os.path.join(ROOT, "docs")

# Commands that are deliberately not in the public docs. Keep each one justified.
JSON_UNDOCUMENTED = set()
SCPI_UNDOCUMENTED = set()
CONSOLE_UNDOCUMENTED = set()

# Sanity floors: if a regex stops matching after a refactor, fail instead of passing vacuously.
MIN_JSON, MIN_SCPI, MIN_CONSOLE = 80, 40, 60


def read(*parts):
    with open(os.path.join(*parts), encoding="utf-8") as f:
        return f.read()


def section(text, start, end):
    """The text from the line `start` up to the next line starting with `end`."""
    i = text.find(start)
    if i < 0:
        return ""
    j = text.find(end, i + len(start))
    return text[i:j if j >= 0 else len(text)]


def json_verbs():
    # Matches both { "verb", CMD_TIER_Tn } and { { "verb", CMD_TIER_Tn }, flags, fn }.
    return re.findall(r'\{\s*"([a-z0-9_]+)"\s*,\s*CMD_TIER_T\d', read(SRC, "cmd_tier.c"))


def scpi_patterns():
    return re.findall(r'\.pattern\s*=\s*"([^"]+)"', read(SRC, "scpi_server.c"))


def console_verbs():
    src = read(SRC, "console.c")
    verbs = set(re.findall(r'strcmp\(argv\[0\],\s*"([a-z0-9-]+)"\)', src))
    verbs |= set(re.findall(r'strncmp\((?:cmd|line),\s*"([a-z0-9-]+) ?"', src))
    # Alias tables ({ "new-name", "old-name" }) dispatch through argv[0] rewriting.
    for block in re.findall(r'aliases\[\]\[2\]\s*=\s*\{(.*?)\};', src, re.S):
        verbs |= set(re.findall(r'\{\s*"([a-z0-9-]+)"\s*,\s*"[a-z0-9-]+"\s*\}', block))
    return sorted(verbs)


def main():
    api = read(DOCS, "API.md")
    usb = read(DOCS, "usb-serial-interface.md")
    failures = []

    # JSON: a row "| `verb` |" in the Commands index table.
    index = section(api, "\n## Commands\n", "\n### ")
    verbs = json_verbs()
    if len(verbs) < MIN_JSON:
        failures.append(f"found only {len(verbs)} JSON verbs in cmd_tier.c; did the table format change?")
    for v in verbs:
        if v in JSON_UNDOCUMENTED:
            continue
        if not re.search(r"^\| `%s` \|" % re.escape(v), index, re.M):
            failures.append(f"JSON command `{v}` (cmd_tier.c) has no row in docs/API.md's Commands index")

    # SCPI: the pattern in the SCPI reference ('#' is written <n>; a query whose setter is
    # documented may be shown as "`SETTER ...` / `?`").
    scpi_doc = section(api, "\n## SCPI reference\n", "\n## ").replace("<n>", "#")
    scpi_lines = scpi_doc.splitlines()
    pats = scpi_patterns()
    if len(pats) < MIN_SCPI:
        failures.append(f"found only {len(pats)} SCPI patterns in scpi_server.c; did the table format change?")
    pat_set = set(pats)
    for p in pats:
        if p in SCPI_UNDOCUMENTED:
            continue
        base = p[:-1] if p.endswith("?") else p
        if p in scpi_doc:
            ok = True
        elif p.endswith("?") and base in pat_set:
            ok = any(base in line and "/ `?`" in line for line in scpi_lines)
        else:
            ok = False
        if not ok:
            failures.append(f"SCPI command `{p}` (scpi_server.c) is not in docs/API.md's SCPI reference")

    # Console: the verb at the start of a code span in usb-serial-interface.md.
    cverbs = console_verbs()
    if len(cverbs) < MIN_CONSOLE:
        failures.append(f"found only {len(cverbs)} console commands in console.c; did the dispatch change?")
    for v in cverbs:
        if v in CONSOLE_UNDOCUMENTED:
            continue
        if not re.search(r"`%s[` ]" % re.escape(v), usb):
            failures.append(f"console command `{v}` (console.c) is not in docs/usb-serial-interface.md")

    if failures:
        print("docs coverage: FAIL")
        for f in failures:
            print("  " + f)
        print("Document it, or add it to the *_UNDOCUMENTED sets in this script with a reason.")
        return 1
    print(f"docs coverage: PASS ({len(verbs)} JSON, {len(pats)} SCPI, {len(cverbs)} console commands)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
