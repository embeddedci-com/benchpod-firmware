#!/bin/sh
# Place-and-route one synthesized image on a range of seeds and print a table:
#   seed  result  clk48_MHz  clk_MHz  x48_ns  x24_ns
# (x48_ns: nextpnr's max delay of the clk -> clk48 paths.  clocks.py declares the two unrelated,
#  so nextpnr does not time these gearbox paths; they must fit in one clk48 period minus how far
#  clk lags clk48 through its divider flop and global buffer.)
# (x24_ns, v51: the same for clk48 -> clk, the 48 MS/s LA pairs; MAX_X24_NS in the Makefile.)
# sorted by clk48 margin.  result is "ok", "slow" (closes @48 / @24 but below MIN_CLK48_MHZ or
# MIN_CLK_MHZ), "xdomain" (x48_ns over MAX_X48_NS or x24_ns over MAX_X24_NS), "timing" (nextpnr's
# own 48 MHz / 24 MHz check failed) or "noplace".  Only "ok" passes the Makefile's pnr macro with
# PNR_STRICT=1.  The limits come from the environment (`make seed-sweep` passes the Makefile's);
# the defaults here are the Makefile's defaults.
#
#   synth/seed_sweep.sh <image.json> [first last [jobs]]
#
# The seeds run in a rolling pool (a new seed starts as soon as one finishes).  CI splits a sweep
# into chunks on parallel runners: RAW=1 prints only the header comment and the unsorted result
# lines of a chunk, and `synth/seed_sweep.sh --merge first last <raw file>...` turns the raw files
# of all chunks into the one table above (header, sorted rows, ok count).
#
# Run it on the FINAL netlist (after the GATEWARE_VERSION bump, which re-rolls placement), in each
# toolchain that builds the image (make -C ice40 seed-sweep; CI: the ice40-seed-sweep workflow).
# Same nextpnr flags as the Makefile's pnr macro, so a seed's result here is its result there.
set -eu
self=$(cd "$(dirname "$0")" && pwd)/$(basename "$0")

if [ "${1:-}" = --merge ]; then
  first=$2
  last=$3
  shift 3
  grep -h '^#' "$@" | head -n 1 | sed -E "s/seeds [0-9]+\.\.[0-9]+/seeds $first..$last/"
  echo "seed result clk48_MHz clk_MHz x48_ns x24_ns"
  grep -hv '^#' "$@" | sort -k3,3 -g -r
  echo "# ok (pass every gate): $(grep -hv '^#' "$@" | awk '$2=="ok"' | wc -l | tr -d ' ') of $((last - first + 1))"
  exit 0
fi

if [ "${1:-}" = --one ]; then
  # one seed, run by the pool below: json, limits and the output dir come from its environment
  json=$SWEEP_JSON out=$SWEEP_OUT min48=$MIN_CLK48_MHZ minclk=$MIN_CLK_MHZ max48=$MAX_X48_NS max24=$MAX_X24_NS
  s=$2
  log=$out/$s.log
  if nextpnr-ice40 --up5k --package sg48 --pcf vbench_pod.pcf --pcf-allow-unconstrained \
       --pre-pack clocks.py --threads 1 --seed "$s" --json "$json" --asc "$out/$s.asc" \
       >"$log" 2>&1; then r=ok; else r=fail; fi
  f48=$(grep "Max frequency for clock 'clk48" "$log" | tail -n 1 | sed -E 's/.*: *([0-9.]+) MHz.*/\1/')
  fclk=$(grep -E "Max frequency for clock +'clk'" "$log" | tail -n 1 | sed -E 's/.*: *([0-9.]+) MHz.*/\1/')
  x48=$(grep -E "Max delay posedge clk +-> posedge clk48" "$log" | tail -n 1 | sed -E 's/.*: *([0-9.]+) ns.*/\1/')
  x24=$(grep -E "Max delay posedge clk48[^ ]* +-> posedge clk +:" "$log" | tail -n 1 | sed -E 's/.*: *([0-9.]+) ns.*/\1/')
  if [ $r = fail ]; then
    if grep -q "legal placement" "$log"; then r=noplace; else r=timing; fi
  elif awk -v x="${x48:-999}" -v y="${x24:-999}" -v a="$max48" -v b="$max24" \
       'BEGIN { exit !(x > a || y > b) }'; then
    r=xdomain
  elif awk -v f="${f48:-0}" -v c="${fclk:-0}" -v m="$min48" -v n="$minclk" \
       'BEGIN { exit !(f < m || c < n) }'; then
    r=slow
  fi
  printf '%s %s %s %s %s %s\n' "$s" "$r" "${f48:--}" "${fclk:--}" "${x48:--}" "${x24:--}" >"$out/$s.res"
  rm -f "$out/$s.asc"
  exit 0
fi

json=$1
first=${2:-1}
last=${3:-24}
jobs=${4:-$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 4)}
export MIN_CLK48_MHZ=${MIN_CLK48_MHZ:-50}
export MIN_CLK_MHZ=${MIN_CLK_MHZ:-25.5}
export MAX_X48_NS=${MAX_X48_NS:-15}
export MAX_X24_NS=${MAX_X24_NS:-15}
cd "$(dirname "$0")/.."
out=$(mktemp -d "${TMPDIR:-/tmp}/seed_sweep.XXXXXX")
trap 'rm -rf "$out"' EXIT
export SWEEP_JSON=$json SWEEP_OUT=$out

# rolling pool: at most $jobs nextpnr runs at a time, a new seed starts as soon as one finishes
seq "$first" "$last" | xargs -P "$jobs" -I{} sh "$self" --one {}

echo "# $json: seeds $first..$last, MIN_CLK48_MHZ=$MIN_CLK48_MHZ MIN_CLK_MHZ=$MIN_CLK_MHZ MAX_X48_NS=$MAX_X48_NS MAX_X24_NS=$MAX_X24_NS ($(yosys -V 2>/dev/null | cut -d' ' -f1-2))"
if [ -n "${RAW:-}" ]; then
  cat "$out"/*.res
  exit 0
fi
echo "seed result clk48_MHz clk_MHz x48_ns x24_ns"
cat "$out"/*.res | sort -k3,3 -g -r
echo "# ok (pass every gate): $(cat "$out"/*.res | awk '$2=="ok"' | wc -l | tr -d ' ') of $((last - first + 1))"
