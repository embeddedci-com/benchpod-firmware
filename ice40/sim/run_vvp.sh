#!/bin/sh
# Run a compiled iverilog bench and turn the verdict it PRINTS into the exit status.
#
# vvp exits 0 whatever a bench prints, so before this every `make <bench>` — and `make test` in
# CI — passed as long as the bench compiled, even when it printed FAIL.  A bench passes here when
# it prints a line starting with "PASS" (or "ALL TESTS PASSED") and no line starting with "FAIL"
# (or containing "TEST(S) FAILED").  Indented report lines ("  ... -> FAIL" in tb_adc_timing's
# expected-failure row) don't count.
#
# usage: sh sim/run_vvp.sh <bench.vvp> [vvp args...]
out=$(vvp "$@")
rc=$?
printf '%s\n' "$out"
if [ "$rc" -ne 0 ]; then
    echo "*** vvp $*: exited $rc"
    exit "$rc"
fi
if printf '%s\n' "$out" | grep -Eq '^FAIL|TEST\(S\) FAILED'; then
    echo "*** $1: the bench reported FAIL"
    exit 1
fi
if ! printf '%s\n' "$out" | grep -Eq '^(PASS|ALL TESTS PASSED)'; then
    echo "*** $1: no PASS line — the bench did not report success"
    exit 1
fi
