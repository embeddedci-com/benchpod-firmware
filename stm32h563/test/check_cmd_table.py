#!/usr/bin/env python3
"""Cross-check the JSON command table (src/cmd_tier.c k_tiers) against the files kept by hand.

  test/vectors/cmd_tier_vectors.json  its "table" section is k_tiers: every verb of a normal
                                      build, in table order, with its row tier. --write
                                      regenerates the section from k_tiers (the rest of the file,
                                      the cases, stays as it is).
  docs/API.md                         the Commands index has a row for every verb and no other,
                                      and its Tier column is the row tier, "Tn/T2" for a verb
                                      whose tier depends on its arguments (a row with a write
                                      test). check_docs_coverage.py checks only the first half.
                                      Its "Commands that pass a lease:" line names exactly the
                                      rows with CMD_F_ADMIN.
  features/features.toml              with --features: every `docs` anchor resolves to a heading
                                      or an explicit anchor in its file. Not run by `make test`.

The vectors are mirrored in embeddedci-server (testdata/shared/cmd_tier_vectors.json, checked
by its api/shared_vectors_test.go): after --write, copy the file there byte for byte.

Run from anywhere: `python3 stm32h563/test/check_cmd_table.py [--write] [--features]`.
Exits 1 on a mismatch.
"""
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
FW = os.path.normpath(os.path.join(HERE, ".."))
ROOT = os.path.normpath(os.path.join(FW, ".."))
TABLE_SRC = os.path.join(FW, "src", "cmd_tier.c")
VECTORS = os.path.join(HERE, "vectors", "cmd_tier_vectors.json")
API = os.path.join(ROOT, "docs", "API.md")
FEATURES = os.path.join(ROOT, "features", "features.toml")

ROW = re.compile(r'\{ \{ "([a-z0-9_]+)", CMD_TIER_(T\d) \}, [^\n]*?, (NULL|\w+) \},')
# A row's verb and its flags (the field after the {verb, tier} pair).
ROW_FLAGS = re.compile(r'\{ \{ "([a-z0-9_]+)", CMD_TIER_T\d \}, ([^,]+),')
LEASE_LINE = "Commands that pass a lease:"
# Rows only some builds have (the EMI test build's emi_log): not in the vectors, which are the
# normal build's table.
BUILD_ONLY = re.compile(r"^#if defined\(BENCHPOD_EMI_TEST\)\n.*?^#endif\n", re.S | re.M)


def read(path):
    with open(path, encoding="utf-8") as f:
        return f.read()


def table_rows(normal_build_only):
    """[(verb, tier, mixed)] in table order."""
    src = read(TABLE_SRC)
    if normal_build_only:
        src = BUILD_ONLY.sub("", src)
    return [(v, t, w != "NULL") for v, t, w in ROW.findall(src)]


def check_vectors(write):
    raw = read(VECTORS)
    doc = json.loads(raw)
    want = {v: t for v, t, _ in table_rows(True)}
    if json.dumps(doc, indent=2, ensure_ascii=False) + "\n" != raw:
        return ["test/vectors/cmd_tier_vectors.json is not in its canonical layout "
                "(json.dumps indent 2); --write would reformat it, so the mirror would differ"]
    if list(doc["table"].items()) == list(want.items()):
        return []
    if write:
        doc["table"] = want
        with open(VECTORS, "w", encoding="utf-8") as f:
            f.write(json.dumps(doc, indent=2, ensure_ascii=False) + "\n")
        print("cmd table: rewrote the vectors' table from k_tiers; copy the file to "
              "embeddedci-server testdata/shared/cmd_tier_vectors.json")
        return []
    have = doc["table"]
    out = []
    for v, t in want.items():
        if v not in have:
            out.append(f"vectors: k_tiers verb `{v}` ({t}) is missing from the table")
        elif have[v] != t:
            out.append(f"vectors: `{v}` is {have[v]} in the table, {t} in k_tiers")
    for v in have:
        if v not in want:
            out.append(f"vectors: `{v}` is in the table but not in k_tiers")
    if not out:
        out.append("vectors: the table's order differs from k_tiers")
    out.append("run `python3 stm32h563/test/check_cmd_table.py --write`, then copy the file to "
               "embeddedci-server")
    return out


def check_api():
    api = read(API)
    i = api.find("\n## Commands\n")
    j = api.find("\n### ", i + 1)
    index = api[i:j if j >= 0 else len(api)]
    docs = {v: t.strip() for v, t in re.findall(r"^\| `([a-z0-9_]+)` \| ([^|]+)\|", index, re.M)}
    docs.pop("cmd", None)   # the header row
    out = []
    if len(docs) < 80:
        out.append(f"API.md: found only {len(docs)} rows in the Commands index; did its format change?")
    rows = table_rows(False)
    verbs = {v for v, _, _ in rows}
    for v, t, mixed in rows:
        want = f"{t}/T2" if mixed else t
        if v in docs and docs[v] != want:
            out.append(f"API.md: `{v}` is {docs[v]} in the Commands index, {want} in k_tiers")
    for v in docs:
        if v not in verbs:
            out.append(f"API.md: the Commands index lists `{v}`, which is not in k_tiers")
    return out


def check_admin():
    """API.md's "Commands that pass a lease:" line lists the CMD_F_ADMIN rows, no more, no less."""
    want = {v for v, f in ROW_FLAGS.findall(read(TABLE_SRC)) if "CMD_F_ADMIN" in f}
    lines = [l for l in read(API).split("\n") if l.startswith(LEASE_LINE)]
    if len(lines) != 1:
        return [f"API.md: expected one line starting with `{LEASE_LINE}`, found {len(lines)}"]
    have = set(re.findall(r"`([a-z0-9_]+)`", lines[0]))
    out = []
    for v in sorted(want - have):
        out.append(f"API.md: `{v}` has CMD_F_ADMIN but is not on the `{LEASE_LINE}` line")
    for v in sorted(have - want):
        out.append(f"API.md: `{v}` is on the `{LEASE_LINE}` line but has no CMD_F_ADMIN")
    if not want:
        out.append("cmd_tier.c: no row has CMD_F_ADMIN; did the table format change?")
    return out


def slugs(text):
    """GitHub's heading anchors in a markdown file, plus explicit <a id/name> anchors."""
    out, seen = set(), {}
    in_code = False
    for line in text.split("\n"):
        if line.startswith("```"):
            in_code = not in_code
            continue
        m = None if in_code else re.match(r"^#{1,6}\s+(.*?)\s*#*\s*$", line)
        if not m:
            continue
        h = re.sub(r"<[^>]+>", "", m.group(1))
        h = re.sub(r"\[([^\]]*)\]\([^)]*\)", r"\1", h)
        s = re.sub(r"[^\w\- ]", "", h.strip().lower()).replace(" ", "-")
        n = seen.get(s, 0)
        seen[s] = n + 1
        out.add(s if n == 0 else f"{s}-{n}")
    out |= set(re.findall(r'<a\s+(?:id|name)="([^"]+)"', text))
    return out


def check_features():
    out = []
    cache = {}
    for ref in re.findall(r'^docs = "([^"#]+)#([^"]+)"', read(FEATURES), re.M):
        f, anchor = ref
        if f.startswith("http"):
            continue
        if f not in cache:
            cache[f] = slugs(read(os.path.join(ROOT, "docs", f)))
        if anchor not in cache[f]:
            out.append(f"features.toml: docs = \"{f}#{anchor}\" is not a heading in docs/{f}")
    return out


def main():
    write = "--write" in sys.argv[1:]
    failures = check_vectors(write) + check_api() + check_admin()
    if "--features" in sys.argv[1:]:
        failures += check_features()
    if failures:
        print("cmd table: FAIL")
        for f in failures:
            print("  " + f)
        return 1
    print(f"cmd table: PASS ({len(table_rows(False))} verbs: vectors table, API.md index, tiers and lease-free commands)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
