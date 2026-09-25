#!/bin/sh
# Place-and-route one synthesized image on a range of seeds and print a table:
#   seed  result  clk48_MHz  clk_MHz  x48_ns
# (x48_ns: nextpnr's max delay of the clk -> clk48 paths.  clocks.py declares the two unrelated,
#  so nextpnr does not time these gearbox paths; they must fit in one clk48 period minus how far
#  clk lags clk48 through its divider flop and global buffer.)
# sorted by clk48 margin.  result is "ok", "slow" (closes @48 but below MIN_CLK48_MHZ),
# "timing" (nextpnr's own 48 MHz / 24 MHz check failed) or "noplace".
#
#   synth/seed_sweep.sh <image.json> [first last [jobs]]
#
# Run it on the FINAL netlist (after the GATEWARE_VERSION bump, which re-rolls placement), in each
# toolchain that builds the image (make -C ice40 seed-sweep; CI: the ice40-seed-sweep workflow).
# Same nextpnr flags as the Makefile's pnr macro, so a seed's result here is its result there.
set -eu
json=$1
first=${2:-1}
last=${3:-24}
jobs=${4:-$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 4)}
min48=${MIN_CLK48_MHZ:-50}
cd "$(dirname "$0")/.."
out=$(mktemp -d "${TMPDIR:-/tmp}/seed_sweep.XXXXXX")
trap 'rm -rf "$out"' EXIT

one() {
  s=$1
  log=$out/$s.log
  if nextpnr-ice40 --up5k --package sg48 --pcf vbench_pod.pcf --pcf-allow-unconstrained \
       --pre-pack clocks.py --threads 1 --seed "$s" --json "$json" --asc "$out/$s.asc" \
       >"$log" 2>&1; then r=ok; else r=fail; fi
  f48=$(grep "Max frequency for clock 'clk48" "$log" | tail -n 1 | sed -E 's/.*: *([0-9.]+) MHz.*/\1/')
  fclk=$(grep -E "Max frequency for clock +'clk'" "$log" | tail -n 1 | sed -E 's/.*: *([0-9.]+) MHz.*/\1/')
  x48=$(grep -E "Max delay posedge clk +-> posedge clk48" "$log" | tail -n 1 | sed -E 's/.*: *([0-9.]+) ns.*/\1/')
  if [ $r = fail ]; then
    if grep -q "legal placement" "$log"; then r=noplace; else r=timing; fi
  elif awk -v f="${f48:-0}" -v m="$min48" 'BEGIN { exit !(f < m) }'; then
    r=slow
  fi
  printf '%s %s %s %s %s\n' "$s" "$r" "${f48:--}" "${fclk:--}" "${x48:--}" >"$out/$s.res"
  rm -f "$out/$s.asc"
}

# plain sh job pool: at most $jobs nextpnr runs at a time
running=0
s=$first
while [ "$s" -le "$last" ]; do
  one "$s" &
  running=$((running + 1))
  if [ "$running" -ge "$jobs" ]; then wait; running=0; fi
  s=$((s + 1))
done
wait

echo "# $json: seeds $first..$last, MIN_CLK48_MHZ=$min48 ($(yosys -V 2>/dev/null | cut -d' ' -f1-2))"
echo "seed result clk48_MHz clk_MHz x48_ns"
cat "$out"/*.res | sort -k3,3 -g -r
echo "# reach MIN_CLK48_MHZ: $(cat "$out"/*.res | awk '$2=="ok"' | wc -l | tr -d ' ') of $((last - first + 1))"
