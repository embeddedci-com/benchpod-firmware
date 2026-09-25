# Makefile for the bench-pod iCE40 gateware (iCE40UP5K-SG48).
#
# Uses the open-source IceStorm toolchain.  Install:
#   macOS:  brew install icestorm yosys nextpnr-ice40
#   Linux:  apt install yosys nextpnr-ice40 fpga-icestorm
#
# BOARD selects the pin-constraint file. Only the STM32 "vbench_pod" carrier
# remains (the RP2350 upduino/vbench_pod_b boards were retired 2026-07-09), so
# this is effectively fixed; the variable is kept for artifact naming.
#
# Targets:
#   make [BOARD=...]            build .bin bitstream for that board
#   make flash [BOARD=...]      build + program SPI flash via onboard FTDI (iceprog)
#   make flash-ram [BOARD=...]  build + load bitstream directly into FPGA SRAM
#   make prog                   alias for `make flash`
#   make sim                    quick syntax check via iverilog (optional)
#   make clean                  remove generated artefacts (all boards)
#   make help                   this list

PROJECT = bench_pod_fpga

# Gateware for the STM32 "vbench-pod" board: 16-bit serial ADC (MCP33131D-10),
# APS6404L PSRAM, async ADC capture written to PSRAM (the MCU reads the result
# over memory-mapped XSPI). (The legacy v1 parallel-ADC RP2350 gateware and its
# firmware were retired 2026-07-09; this is the only generation now.)
BOARD  ?= vbench_pod
# Unified single-clock (24 MHz) capture datapath: two producers -> two spram_ring16
# rings -> psram_dual_writer -> two PSRAM regions (docs/unified-adc-la-capture-redesign.md).
# The old correlated capture_merge/cdc/la_drain_ctrl modules were removed.
SRC = src/top_v2.v            \
      src/engine_block.v      \
      src/spi_slave.v         \
      src/cmd_dispatch.v      \
      src/sample_buf.v        \
      src/adc_mcp33131.v      \
      src/psram_dual_writer.v \
      src/spram_ring16.v      \
      src/dac8551_engine.v    \
      src/cdc_pulse_payload.v \
      src/dac_loop.v          \
      src/dac_psram_reader.v  \
      src/psram_bus_arbiter.v \
      src/la_bank.v           \
      src/stepper_engine.v    \
      src/swd_engine.v        \
      src/i2c_regfile.v       \
      src/i2c_target.v        \
      src/la_psram_capture.v  \
      src/uart_engine.v
DEFINES = -DVERSION_V2
# Per-clock timing constraints for the two v2 clock domains (clk48 / clk).
PREPACK = --pre-pack clocks.py
# Pinned PnR seed.  TIMING is not the constraint: after the single-clock collapse
# (docs/adc-capture-cdc-review.md) the whole design runs on the single 24 MHz `clk`,
# which closes with wide margin (~30-37 MHz depending on seed) vs a 24 MHz target;
# clk48 only clocks the ÷2 + POR (~94 MHz).
#
# PLACEMENT was formerly tight (~89% LCs => routability-seed-fragile, only seeds
# 4/6 placed).  The LUT-reduction pass in docs/ice40-lut-optimization.md brought the
# v2 design down to ~79% LCs (4198/5280), and it now places on EVERY seed 1..10
# (clk 34-39 MHz).  So this pin is no longer load-bearing for routability — it is
# kept only for build determinism (seed 6 places at ~37 MHz).  --threads 1 keeps
# PnR reproducible.  If a future netlist change tightens placement again, sweep
# `--seed N` and re-pin one that places (and reclaim more LCs per that doc).
# v16 DEEP ADC widened cap_count/adc_left 16->24-bit (and OP_CAPTURE 8->10 bytes),
# nudging util to 80% LC (4241/5280).  v17 DEEP DAC REPLAY (dac_psram_reader + FIFO)
# then pushed util to 89% LC (4722/5280) and made clk48 the binding, seed-fragile
# constraint (only ~3 of 9 swept seeds closed clk48 @48 MHz).
# LUT-REDUCTION PASS (docs): la_bank driver mux rewritten to one-hot decode + cmd_dispatch
# fixed-arg collector folded into the uniform S_COLLECT path.  That brought util to 87%
# LC (4622/5280).
# CONCURRENT-PSRAM PASS (docs/psram-concurrent-arbiter): added psram_bus_arbiter so deep
# DAC replay and ADC/LA capture time-multiplex the one quad bus (bus_req/gnt/busy/urgent
# handshakes on dac_psram_reader + psram_dual_writer).  Util is now 88% LC (4664/5280).
# In THIS netlist clk48 has no timed setup path (it only feeds SB_IO output regs + the ÷2
# divider) — nextpnr reports no clk48 Fmax — so the binding constraint is PLACEMENT/routability
# at 87% fill, not clk48 timing.
# v18 CONCURRENT-PSRAM (docs/psram-concurrent-arbiter): the arbiter's cross-chip req/gnt/busy
# nets between the reader FSM, writer FSM and arbiter first pushed util to 4642 LC and — more
# than raw fill — created a routability hotspot that placed on NO seed (1..20 all failed).  The
# fix was an LC/congestion-reduction pass: HALVE the psram_dual_writer staging FIFOs (AW 6->5,
# 64->32 B; CHUNK 32->16) — those FIFOs have an ASYNC read so on the LUT-RAM-less up5k they are
# FF+mux logic, and halving them reclaimed the LC AND the routing.  Now 4619 LC (BELOW the v17
# 4622 baseline) and it places on seed 9 (the v17 pin) at clk ~32 MHz (PASS @24).  --threads 1
# keeps PnR reproducible.  If a future change tightens placement, re-sweep `--seed N`.
# v19 LC-REDUCTION PASS (ADC deep-ring dropped, dead code purged): util drops to 4414 LC (83%).
# clk48 IS a timed domain here (the DAC8551 sequencer + psram_dual_writer serializer run on it),
# and closing it @48 MHz under clocks.py is seed-fragile: a fresh 1..12 sweep on this netlist
# passed clk48@48 only on seeds 6 (49.1), 10 (48.5) and 12 (50.4) — seed 9 now closes clk48 at
# only 47.55 (FAIL @48), so re-pinned to SEED 12 (best clk48 margin; clk ~34.6 MHz PASS @24).
# NOTE: a seed change can HW-regress deep DAC PSRAM replay even when static timing passes
# (docs/yosys-0.66-abc9-placement, docs/deep-dac-psram-replay) — re-verify hwe2e DAC replay on HW.
# v21 CAPTURE-TIED DAC AUTO-STOP: the OP_SET_DAC_STOP_AFTER threshold reg + a down-counter in
# top_v2 pushed util 4414 -> 4536 LC (83% -> 85%), and at that fill seed 12 no longer closes
# clk48 (47.7 FAIL @48).  A fresh 1..20 sweep on the v21 netlist passed clk48@48 only on seeds
# 1 (49.25), 7 (49.00) and 11 (49.00) — re-pinned to SEED 1 (best clk48 margin; clk 33.83 MHz
# PASS @24).  RE-VERIFY hwe2e deep DAC replay on HW after this seed change.
# v24 LC/STABILITY PASS (docs/ice40-lut-optimization.md): removed the dead SB_WARMBOOT block +
# OP_WARMBOOT decode, narrowed cmd_dispatch arg_count 16->12, uart_engine bit-period counters
# 24->18, and made dac_loop's tick timer a registered down-counter (dropping the last wide clk48
# magnitude compare).  Loop image drops 4572 -> 4513 LC (86% -> 85%).  A fresh 1..12 sweep on the
# v24 loop netlist: seed 9 (the old pin) no longer closes clk48 @48; seeds 4,5,6,7,8,10,11 do.
# Re-pinned to SEED 11 (best+reproducible clk48 margin: 50.9 MHz worst path, clk 34.7 MHz; A/B
# builds bit-identical).  SEED_DEEP 3 unchanged (clk48 55.0 MHz).
# v29 SELECTABLE LOOP INPUT SOURCE (dac_loop src mux + sweep accumulator): loop image 4301 ->
# 4483 LC (81% -> 84%).  At that fill seed 11 still closes clk48 but its margin fell 53.75 ->
# 50.51 MHz, so the loop netlist was re-swept 1..12: seed 1 FAILS @48, everything else passes,
# and seed 2 is the best (clk48 54.37 MHz, clk 34.48).  Re-pinned to SEED 2.  The binding path
# is NOT in the loop — it is the pre-existing psram_run_48 -> dac dz -> wave_addr CEN chain, so
# this is placement luck, which is exactly what the pin is for.  SEED_DEEP is untouched (the
# deep netlist barely moved: 50.98 MHz at seed 4).
# v30 INPUT CONDITIONER (affine map onto the curve index + over-range trip): loop image
# 4483 -> 4592 LC (84% -> 86%).  This fill is the tightest the loop image has run: of seeds
# 1..12 only 4,6,7,8,10 place at all (1,3,5,9,11,12 hit "unable to find legal placement"),
# and of those seed 2's placement closes only 46.21 MHz.  Re-swept and RE-PINNED to SEED 8
# (clk48 53.33 MHz, clk 32.86) — back to v29's margin.  Two features were CUT during this
# pass to get here, both measured, not guessed: a programmable per-tick slew bound (~50 carry
# cells for two 17-bit variable compares + a negate) and the non-modular subtract mode; see
# dac_loop.v.  If anything else is added to this image, expect to re-sweep again — and note
# that 6/12 seeds failing to place is the real signal that this image is out of room.
# v33+: the default single image IS the loop netlist (identical DEFINES), so it no longer carries
# a pin of its own.  That pin had drifted — seed 8 did not even place on v33, and CI's release
# job built this image — while SEED_LOOP was the one re-swept and HW-verified.  Both now follow
# SEED_LOOP / SEED_LOOP_FALLBACK (the history above is the loop image's).
SEED = $(SEED_LOOP)
# SEED FALLBACK (2026-09-04).  A single pinned seed is NOT portable across
# toolchain BUILDS: the CI Linux OSS-CAD-Suite yosys 0.65 and a macOS Homebrew
# yosys 0.65 map this design slightly differently (CI 4587 LC / local 4592 LC,
# different promoted globals), and at 86% fill that is enough to flip a seed from
# "places" to "unable to find legal placement".  Seed 8 places locally but FAILED
# in CI on the v31 netlist.  So each PnR now tries the pinned seed FIRST and falls
# back down this list until one both PLACES and CLOSES timing (nextpnr errors on a
# timing miss, so a fallback build is never a timing-failing build).  Where the
# pinned seed places, the bitstream is bit-identical to before — the fallback only
# ever kicks in on a toolchain/netlist where the pin would have hard-failed, and it
# prints a loud warning because that placement is NOT the HW-verified one.
# Order is by measured clk48 margin on the local 1..16 sweep of the v31 loop netlist;
# the trailing group are the seeds that do NOT place on the LOCAL netlist — they are
# still worth a last-resort try because a different toolchain build maps the design
# differently (that is the whole reason this list exists), and they cost nothing unless
# every earlier seed has already failed.
SEED_FALLBACK ?= $(SEED_LOOP_FALLBACK)

PCF     = $(BOARD).pcf
OUT     = $(PROJECT)_$(BOARD)

DEVICE  = up5k
PACKAGE = sg48

# Common nextpnr-ice40 invocation.  --threads 1 keeps PnR reproducible.
NEXTPNR = nextpnr-ice40 --$(DEVICE) --package $(PACKAGE) \
                        --pcf $(PCF) --pcf-allow-unconstrained $(PREPACK) --threads 1

# ---- PnR policy: placement/margin gates ----
# PNR_STRICT=0 (default) keeps the historical behaviour: a fallback seed, or a clk48 Fmax below
#   MIN_CLK48_MHZ, prints a loud WARNING and the bitstream is kept, so a toolchain build that
#   maps the design differently (CI vs local — see SEED_FALLBACK) still completes.
# PNR_STRICT=1 makes either one FAIL the build (the .asc is removed so a re-run can't pick it
#   up) and tries no fallback seed.  Use it for anything that ships.  CI passes it through from
#   the ICE40_PNR_STRICT repository variable (.github/workflows) — off until CI's toolchain is
#   confirmed to place the pinned seeds.
# MIN_CLK48_MHZ: nextpnr only fails clk48 below 48.00 MHz, but placements closing at
#   48.2-48.8 MHz have silently HW-regressed deep DAC replay (the v28/v31 notes), and seeds are
#   pinned for >= 50.  Checked against the final post-route Fmax.
PNR_STRICT    ?= 0
MIN_CLK48_MHZ ?= 50

# $(call pnr,<in.json>,<out.asc>,<pinned seed>,<fallback seeds>)
#
# Place & route with the PINNED seed first; only if that seed fails (no legal
# placement, or a timing miss — nextpnr errors on both) walk the fallback list
# (not with PNR_STRICT=1).  See the SEED_FALLBACK comment for why a single pin is not
# portable across toolchain builds.  A fallback build is loudly flagged: it is NOT the
# placement that was verified on hardware.  The routed clk48 Fmax is then checked
# against MIN_CLK48_MHZ.
define pnr
	@set -e; \
	log=$(2).pnr.log; \
	for s in $(3) $(if $(filter 1,$(PNR_STRICT)),,$(4)); do \
	  echo "nextpnr: $(2): trying --seed $$s"; \
	  if $(NEXTPNR) --seed $$s --json $(1) --asc $(2).tmp > $$log 2>&1; then \
	    mv -f $(2).tmp $(2); \
	    grep 'Max frequency' $$log || true; \
	    bad=; \
	    if [ "$$s" = "$(3)" ]; then \
	      echo "nextpnr: $(2): placed on the pinned seed $$s"; \
	    else \
	      echo "*** WARNING: pinned seed $(3) did not place/close for $(2);"; \
	      echo "***          fell back to seed $$s.  This bitstream is NOT the"; \
	      echo "***          HW-verified placement — re-verify on hardware (deep"; \
	      echo "***          DAC PSRAM replay especially) and re-pin the seed."; \
	      bad="fallback seed $$s"; \
	    fi; \
	    f48=$$(grep "Max frequency for clock 'clk48" $$log | tail -n 1 | sed -E 's/.*: *([0-9.]+) MHz.*/\1/'); \
	    if [ -n "$$f48" ] && awk -v f="$$f48" -v m="$(MIN_CLK48_MHZ)" 'BEGIN { exit !(f < m) }'; then \
	      echo "*** WARNING: $(2): clk48 closes at only $$f48 MHz (MIN_CLK48_MHZ=$(MIN_CLK48_MHZ))."; \
	      echo "***          Static timing passes, but margin this thin has HW-regressed deep"; \
	      echo "***          DAC replay before — re-sweep and re-pin the seed."; \
	      bad="$${bad:+$$bad, }clk48 $$f48 MHz"; \
	    fi; \
	    if [ -n "$$bad" ] && [ "$(PNR_STRICT)" = "1" ]; then \
	      rm -f $(2); \
	      echo "ERROR: PNR_STRICT=1: $(2) failed the placement gate ($$bad)."; \
	      exit 1; \
	    fi; \
	    exit 0; \
	  fi; \
	  rm -f $(2).tmp; \
	  echo "nextpnr: $(2): seed $$s failed: $$(grep -m1 ERROR $$log)"; \
	done; \
	echo "ERROR: no seed in '$(3) $(if $(filter 1,$(PNR_STRICT)),(PNR_STRICT=1: no fallbacks),$(4))' both placed and closed timing for $(2)."; \
	echo "       Last nextpnr log follows; the design is likely out of room —"; \
	echo "       do an LC-reduction pass (docs/ice40-lut-optimization.md) or sweep more seeds."; \
	cat $$log; \
	exit 1
endef

# iCE40 primitive sim models (SB_IO / SB_GB / SB_RGBA_DRV), shipped with yosys —
# needed to simulate a whole top through its pads (tb_top_v2 / topsmoketest).
CELLS_SIM = $(shell yosys-config --datdir)/ice40/cells_sim.v

# Every bench runs through sim/run_vvp.sh, which turns the verdict a bench PRINTS into the exit
# status: vvp exits 0 whatever a bench prints, so a FAILing bench used to pass `make test` and CI.
VVP = sh sim/run_vvp.sh

.PHONY: all images flash flash-ram prog sim simtest uarttest cdctest dactest daclooptest dacpsramrdtest dacpsramskewtest dacpsramreplaytest adctest adctimingtest dualwrtest arbtest tritest trimodetest looptest loopframetest multiboot lapsramtest measuretest argtest steppertest topsmoketest topcapturetest check-protocol test clean help

all: $(OUT).bin

# -abc9: timing-driven LUT mapping (ABC9 flow).  Free win, no RTL change.
#   DO NOT drop to -noabc9 to chase placement on newer yosys: it costs ~8 MHz of
#   clk margin (37.6 -> 29.3 MHz) and — though nextpnr still reports clk PASS at
#   24 MHz — that HW-REGRESSES deep DAC PSRAM replay (the dac_psram_reader QPI +
#   dual-clock FIFO path fails; DAC stuck ~full-scale, cal1 spread 1 mV vs 2808 mV
#   working).  Proven on the bench 2026-07-10, both gateware builds flashed and
#   tested.  See docs/yosys-0.66-abc9-placement.md.  NOTE: yosys 0.66's abc9
#   netlist does NOT place at this ~87% fill (0/15 seeds); staying on yosys 0.65
#   until the design frees ~2% LC (then 0.66 -abc9 should place — do NOT use
#   -noabc9).  Toolchain is pinned to 0.65 for this reason.
# -spram: infer SB_SPRAM256KA for the 256Kbit correlated-capture ring (small
#         RAMs still go to BRAM — SPRAM is only cheaper for the big one).
$(OUT).json: $(SRC)
	yosys -p "read_verilog -Isrc $(DEFINES) $(SRC); synth_ice40 -abc9 -dsp -spram -top top -json $@"

# --pcf-allow-unconstrained: a board .pcf may intentionally leave some ports
# (e.g. DAC/ADC on a board whose analog wiring isn't pinned down yet)
# unconstrained — nextpnr auto-places those on free pins instead of erroring.
$(OUT).asc: $(OUT).json $(PCF)
	$(call pnr,$(OUT).json,$(OUT).asc,$(SEED),$(SEED_FALLBACK))

$(OUT).bin: $(OUT).asc
	icepack $(OUT).asc $(OUT).bin

# ---- Multi-image WARMBOOT build (SB_WARMBOOT + icemulti) ----
# Packs TWO gateware images into one config-flash image: slot 0 = the closed-loop image
# (power-on default), slot 1 = deep-DAC-PSRAM-replay.  The firmware SB_WARMBOOTs between
# them at runtime (OP_WARMBOOT / {"cmd":"fpga_image"}) with NO reflash.  Each image is a
# different netlist (USE_DEEP_REPLAY), so each needs its OWN placing+clk48 seed.
# v30: clk48 53.33 MHz (re-swept after the input conditioner; only 5 of 12 seeds both place and close).
# NOTE: keep these assignments free of trailing comments — the value is compared against the
# seed actually used, and make would keep the trailing whitespace in it.
# v32 EXACT CAPTURE DIVIDERS (constants only in la_psram_capture/adc_mcp33131; 4569 LC, fewer than
# v31) re-rolled placement: seed 8 no longer places.  Swept 1..16: 5/8/11/12/15 do not place; the
# rest close, best is 13 at clk48 55.61 MHz (clk 33.85).  Re-pinned SEED_LOOP 8 -> 13.
# v33 DAC RESTART PERIOD FIX (+1 clk48 stage on the DAC start/stop CDC; 4599 LC) re-rolled it
# again: seed 13 no longer places.  Swept 1..16: 2/4/5/8/12/13/14/15 do not place; best is 11 at
# clk48 51.75 MHz (clk 35.26), then 3 (50.41) and 7 (50.17).  Re-pinned SEED_LOOP 13 -> 11.
# v34 ROBUSTNESS PASS: the cmd_dispatch config-wire LC reduction took the loop image 4599 -> 4407 LC
# (87% -> 83%), and the DAC engine's frame-end enable cone (the clk48 critical path the interim v34
# sweeps found) now tests a registered flag.  Swept 1..16: ALL 16 place and close (v33: 8 did not
# place); best is 9 at clk48 54.29 MHz (clk 33.32), then 11 (52.92) and 12 (52.74).  Re-pinned
# SEED_LOOP 11 -> 9.
# v35 CAPTURE TRIGGER + GPIO_GET (4407 -> 4442 LC, 83% -> 84%): the gated-start logic re-rolled
# placement.  Swept 1..16: ALL 16 still place and close; seed 9 drops to clk48 52.73 MHz and the
# best is 13 at 56.44 MHz (clk 32.86), then 1 (55.94) and 6 (55.04).  Re-pinned SEED_LOOP 9 -> 13.
# RE-VERIFY deep DAC replay + a capture on HW after this seed change (see AGENTS.md).
# v37 LC PASS (4442 -> 4351 LC, 84% -> 82%; clk-domain only, see src/top_v2.v).  Swept 1..16 on
# the final v37 netlist: all place, 14/16 close @48 (16 and 5 miss).  Best is 14 at clk48
# 57.05 MHz (clk 30.64), then 9 (53.68) and 1 (53.53).  Re-pinned SEED_LOOP 13 -> 14.  The loop's
# binding clk48 path is still dac_loop's clamp, which this pass left alone.
# v38 CLOSED-LOOP CROSSING (4351 -> 4379 LC, 82%): the loop params became clk48 registers and
# the clamp compare got its own state.  Swept 1..24: all place, 23/24 close AND reach
# MIN_CLK48_MHZ (v37: 14/16 closed @48); seed 14 fell to 52.82, best is 5 at clk48 56.69 MHz
# (clk 31.07), then 23 and 21 (55.04).  Re-pinned SEED_LOOP 14 -> 5.
SEED_LOOP ?= 5
# Ordered by the v38 sweep's clk48 margin (every seed places; only the last misses 50 MHz).
SEED_LOOP_FALLBACK ?= 23 21 12 2 17 20 13 24 10 16 14 7 1 8 9 22 11 4 19 3 6 15
# v27 CO-TRIGGER + PIPELINED FIFO-EMPTY: the co-trigger topology re-rolled placement and seed 5 (at
# clk48 51.26 MHz static) HW-REGRESSED deep replay — the binding clk48 path was the pre-existing
# 6-LUT reader<->DAC FIFO-empty chain (rgray==wgray_r2 -> fetch/RCLKE/CE), whose scattered placement
# was the whole seed-fragility.  Pipelining that compare into a registered `rd_empty`
# (dac_psram_reader.v, bit-exact, no added latency) took the reader OFF the critical path: the deep
# image now closes clk48 on ALL seeds (49.35..54.59) instead of 5/8, so it is no longer seed-fragile.
# Re-swept 1..10: seed 9 is best (clk48 54.59 MHz, clk 35.25 MHz).
# v31 DAC SOURCE-SELECT FIX: the two extra decode terms in cmd_dispatch re-rolled placement, and on
# seed 4 the deep image dropped to clk48 48.77 MHz — still green at 48.00 but only 0.77 MHz of margin
# on the exact clock+image where deep replay has historically died SILENTLY with green STA (see the
# v28 note above).  Re-swept 1..8 on the new netlist: 8 is best at clk48 51.13 MHz (A/B builds
# bit-identical), i.e. MORE margin than the pre-fix 49.75.  Re-pinned SEED_DEEP 4 -> 8.
# v31: clk48 51.13 MHz, clk 34.69 MHz (re-swept after the source-select fix).
# v32 EXACT CAPTURE DIVIDERS re-rolled placement again: seed 8 fell to clk48 48.19 MHz (0.19 MHz
# of margin).  Swept 1..16: 11 is best at clk48 50.44 MHz (clk 35.64), then 1 at 49.75; 4/5/6 now
# MISS 48 MHz, and most others sit at 48.19.  Re-pinned SEED_DEEP 8 -> 11 — re-verify deep replay
# on hardware (TestHW_V2_DeepReplay) after any change here.
# v33 DAC RESTART PERIOD FIX: seed 11 still closes but at clk48 48.77 MHz (0.77 MHz of margin).
# Swept 1..16: 13 is best at clk48 51.13 MHz (clk 34.76), then 4 (49.49) and 16 (49.35); 1/3/8/14
# MISS 48 MHz.  Re-pinned SEED_DEEP 11 -> 13 — re-verify deep replay on hardware.
# v34 ROBUSTNESS PASS (4155 LC): the interim v34 netlists fell to a best of 49.80 / 48.74 MHz — the
# DAC engine's frame-end enable cone — until that cone was pipelined (registered `last`, CDC pulse
# off a flop).  Final sweep 1..16: 13/16 close, 6 and 15 best at clk48 51.13 MHz; 15 has the better
# clk (32.47 vs 31.23).  Re-pinned SEED_DEEP 13 -> 15 — re-verify deep replay on hardware.
# v35 CAPTURE TRIGGER + GPIO_GET (4155 -> 4170 LC): the deep netlist's clk48 is the tightest thing
# in this design, and the trigger re-rolled its placement — seed 15 now MISSES 48 MHz (46.39).
# Swept 1..16: 9/16 close (v34: 13/16); 13 and 2 are both best at clk48 51.13 MHz — the same margin
# as v34 — and 13 has much the better clk (34.53 vs 31.26), then 7 (49.75) and 14 (49.27).
# Re-pinned SEED_DEEP 15 -> 13 — RE-VERIFY deep replay on hardware (TestHW_V2_DeepReplay).
# NB an earlier v35 draft also kept an LA PRE-TRIGGER window in the SPRAM ring (4280 LC): at that
# fill only 7/16 seeds closed and the BEST was 49.75 MHz, under MIN_CLK48_MHZ.  That is why the
# pre-trigger was dropped rather than shipped (see the v35 note in src/top_v2.v).
# v36 LA13/LA14 READBACK (src/top_v2.v): widening la_sample/la_levels + la_psram_capture's N
# 12->14 re-rolled placement and seed 13 now MISSES 48 MHz (47.48).  Swept 1..16: only 9/16 close
# (v35: 13/16); 14 is best at clk48 51.13 MHz (clk 34.05), then 5 (49.36) and 8 (49.35).
# Re-pinned SEED_DEEP 13 -> 14 — RE-VERIFY deep replay on hardware (TestHW_V2_DeepReplay).
# v37 LC PASS (4187 -> 4088 LC, 79% -> 77%; no clk48 logic changed): the re-rolled placement
# is WORSE for clk48 here.  Swept 1..40 on the final v37 netlist: all place, only 18/40 close @48
# and only seed 12 reaches MIN_CLK48_MHZ (50.07 MHz, clk 33.65), then 18 (49.80) and 11 (49.49).
# v36 was 9/16 closing, best 51.13.  Re-pinned SEED_DEEP 14 -> 12 — RE-VERIFY deep replay on
# hardware.  The deep image's binding clk48 path is dac_psram_reader's FIFO read pointer; that,
# not more seeds, is the next thing to fix here.
# v38 (cmd_dispatch lost the loop registers; no deep logic changed) re-rolled it again, the
# other way: swept 1..40, all place, 32/40 close @48 and 13 reach MIN_CLK48_MHZ (v37: 1/40).
# Seed 12 fell to 48.77; twelve seeds tie at clk48 51.13 MHz, and 3 has the best clk (32.69).
# Re-pinned SEED_DEEP 12 -> 3 — RE-VERIFY deep replay on hardware.
SEED_DEEP ?= 3
# Ordered by the v37 sweep's clk48 margin.  The deep netlist no longer closes on every seed, but
# a timing miss makes nextpnr error, so a failing seed in this list is only a wasted attempt; the
# list exists for the same cross-toolchain reason as SEED_FALLBACK above.
SEED_DEEP_FALLBACK ?= 38 1 14 9 29 16 35 18 10 20 32 39 24 34 28
MULTI = $(OUT)_multi.bin
# The GATEWARE_VERSION the images are built with, read from the source they are built from and
# written next to each .bin (<image>.bin.gwversion). The firmware embeds it so a pod can tell
# that the gateware in its iCE40 config flash is not the one its firmware shipped with.
GW_VERSION := $(shell sed -n "s/.*GATEWARE_VERSION(8'd\([0-9]*\)).*/\1/p" src/top_v2.v | head -1)

$(OUT)_loop.json: $(SRC)
	yosys -p "read_verilog -Isrc $(DEFINES) $(SRC); synth_ice40 -abc9 -dsp -spram -top top -json $@"
$(OUT)_deep.json: $(SRC)
	yosys -p "read_verilog -Isrc $(DEFINES) -DUSE_DEEP_REPLAY $(SRC); synth_ice40 -abc9 -dsp -spram -top top -json $@"
$(OUT)_loop.bin: $(OUT)_loop.json $(PCF)
	$(call pnr,$<,$(OUT)_loop.asc,$(SEED_LOOP),$(SEED_LOOP_FALLBACK))
	icepack $(OUT)_loop.asc $@
	echo $(GW_VERSION) > $@.gwversion
$(OUT)_deep.bin: $(OUT)_deep.json $(PCF)
	$(call pnr,$<,$(OUT)_deep.asc,$(SEED_DEEP),$(SEED_DEEP_FALLBACK))
	icepack $(OUT)_deep.asc $@
	echo $(GW_VERSION) > $@.gwversion
# -p0: power-on image = slot 0 (loop).  -a16: align images at 64 KB (SB_WARMBOOT jumps here).
multiboot: $(OUT)_loop.bin $(OUT)_deep.bin
	icemulti -p0 -a16 -o $(MULTI) $(OUT)_loop.bin $(OUT)_deep.bin
	@echo "multiboot image -> $(MULTI): slot0=closed-loop image, slot1=deep-DAC-replay"

# The two runtime images the firmware embeds (stm32h563/Makefile: image 0 = loop, 1 = deep) —
# what pods actually run and what a release must build.  `make` (all) builds only one image.
images: $(OUT)_loop.bin $(OUT)_deep.bin

# Persistent flash: writes bitstream to the board's SPI flash chip via iceprog.
# The FPGA reloads from this flash on every power-up.
flash: $(OUT).bin
	iceprog $(OUT).bin

# Volatile load: SRAM-only.  Faster for iterative development but the
# bitstream is lost on power-cycle.  Re-run after every reset.
flash-ram: $(OUT).bin
	iceprog -S $(OUT).bin

# IceStorm convention alias.
prog: flash

# Optional: lint/syntax via iverilog
sim:
	iverilog -g2012 -I src -Wall -o /tmp/$(PROJECT).sim $(SRC) && rm -f /tmp/$(PROJECT).sim
	@echo "Syntax OK"

# Self-checking simulation of the generic I2C target (needs iverilog/vvp).
simtest:
	iverilog -g2012 -o /tmp/$(PROJECT)_i2c.vvp \
	         sim/tb_i2c_target.v src/i2c_target.v src/i2c_regfile.v
	$(VVP) /tmp/$(PROJECT)_i2c.vvp
	@rm -f /tmp/$(PROJECT)_i2c.vvp

# Self-checking loopback simulation of the soft UART (needs iverilog/vvp).
uarttest:
	iverilog -g2012 -o /tmp/$(PROJECT)_uart.vvp \
	         sim/tb_uart_engine.v src/uart_engine.v
	$(VVP) /tmp/$(PROJECT)_uart.vvp
	@rm -f /tmp/$(PROJECT)_uart.vvp

# The clk->clk48 pulse+value crossing (cdc_pulse_payload): every pulse arrives once and its
# consumer sees that pulse's value — the v33 DAC restart bug class — at a sync and an async ratio.
cdctest:
	iverilog -g2012 -o /tmp/$(PROJECT)_cdc.vvp \
	         sim/tb_cdc_pulse_payload.v src/cdc_pulse_payload.v
	$(VVP) /tmp/$(PROJECT)_cdc.vvp
	@rm -f /tmp/$(PROJECT)_cdc.vvp

# Self-checking simulation of the v2 DAC8551 serial waveform engine: frame content plus the
# DAC8551 datasheet pin timing (t5-t9) and the clk/sample the firmware's rate model assumes.
dactest:
	iverilog -g2012 -o /tmp/$(PROJECT)_dac.vvp \
	         sim/tb_dac8551.v src/dac8551_engine.v
	$(VVP) /tmp/$(PROJECT)_dac.vvp
	@rm -f /tmp/$(PROJECT)_dac.vvp

# Self-checking simulation of the DAC8551 waveform-loop terminal (sleft counter).
daclooptest:
	iverilog -g2012 -o /tmp/$(PROJECT)_dacloop.vvp \
	         sim/tb_dac8551_loop.v src/dac8551_engine.v
	$(VVP) /tmp/$(PROJECT)_dacloop.vvp
	@rm -f /tmp/$(PROJECT)_dacloop.vvp

# Self-checking simulation of the QPI PSRAM streaming READ master (deep DAC replay):
# a modelled APS6404L answers 0xEB reads and the test checks the FIFO byte stream
# matches PSRAM contents in order, across the loop wrap.
dacpsramrdtest:
	iverilog -g2012 -I src -o /tmp/$(PROJECT)_dacpsramrd.vvp \
	         sim/tb_dac_psram_reader.v src/dac_psram_reader.v
	$(VVP) /tmp/$(PROJECT)_dacpsramrd.vvp
	@rm -f /tmp/$(PROJECT)_dacpsramrd.vvp

# REGRESSION guard for the deep-replay/PSRAM curse: models the clk<->clk48 SB_GB skew
# AND a realistic PSRAM read round-trip, and checks the reader delivers every byte across
# the sweep.  The v28 single-clk48-domain reader passes all skews; a clk-domain PSRAM
# sampler (the pre-v28 bug, or any regression to it) fails at the larger skews.
dacpsramskewtest:
	iverilog -g2012 -I src -o /tmp/$(PROJECT)_dacskew.vvp \
	         sim/tb_dac_psram_skew.v src/dac_psram_reader.v
	$(VVP) /tmp/$(PROJECT)_dacskew.vvp
	@rm -f /tmp/$(PROJECT)_dacskew.vvp

# Self-checking simulation of the streaming DAC engine popping a PSRAM waveform out
# of the reader FIFO (reader -> dac8551_engine in psram_mode) to the DAC8551 frames.
dacpsramreplaytest:
	iverilog -g2012 -I src -o /tmp/$(PROJECT)_dacpsramrep.vvp \
	         sim/tb_dac_psram_replay.v src/dac_psram_reader.v src/dac8551_engine.v
	$(VVP) /tmp/$(PROJECT)_dacpsramrep.vvp
	@rm -f /tmp/$(PROJECT)_dacpsramrep.vvp

# (ilatest removed in v24 with the on-FPGA I2C-LA sampler — i2c_la_capture.v is gone.)

# Self-checking simulation of the v2 MCP33131D-10 serial ADC capture engine.
adctest:
	iverilog -g2012 -o /tmp/$(PROJECT)_adc.vvp \
	         sim/tb_adc_mcp33131.v src/adc_mcp33131.v
	$(VVP) /tmp/$(PROJECT)_adc.vvp
	@rm -f /tmp/$(PROJECT)_adc.vvp

# Timing + throughput sign-off of the PRODUCTION adc_mcp33131 at 48 MHz against the
# analog-accurate mcp33131_model (real tDO + pad round-trip).  Proves the shipping
# SCLK_DIV=2 config reads correctly, reports the achieved sample rate, and shows
# SCLK_DIV=1 (24 MHz SCLK) as the failing edge.  See docs/adc-clock-analysis.md.
adctimingtest:
	iverilog -g2012 -o /tmp/$(PROJECT)_adct.vvp \
	         sim/tb_adc_timing.v sim/mcp33131_model.v src/adc_mcp33131.v
	$(VVP) /tmp/$(PROJECT)_adct.vvp
	@rm -f /tmp/$(PROJECT)_adct.vvp

# Self-checking test of the DUAL-region QPI writer: two streams (ADC + raw LA)
# into two contiguous PSRAM regions off one trigger, plus tCEM/setup/bus_own.
# Instantiates the REAL SB_IO pads (DDR SCLK + comb data on clk48, exactly like
# top_v2) so the 48 MHz DDR drain's mode-0 pad timing is exercised, not just the
# FSM — hence it needs yosys' cells_sim.v.
dualwrtest:
	iverilog -g2012 -I src -o /tmp/$(PROJECT)_dualwr.vvp \
	         sim/tb_psram_dual_writer.v src/psram_dual_writer.v $(CELLS_SIM)
	$(VVP) /tmp/$(PROJECT)_dualwr.vvp
	@rm -f /tmp/$(PROJECT)_dualwr.vvp

# Self-checking test of CONCURRENT DAC replay + ADC/LA capture over ONE shared quad
# bus, time-multiplexed by psram_bus_arbiter (burst-granular ownership).  Runs the
# reader (0xEB, streaming out of the top region) and the dual writer (0x38, draining
# into the bottom regions) simultaneously through top_v2's real pad mux + one
# APS6404L model; checks no bus collision, no DAC underrun, correct data both ways.
arbtest:
	iverilog -g2012 -I src -o /tmp/$(PROJECT)_arb.vvp \
	         sim/tb_psram_arbiter.v src/dac_psram_reader.v src/psram_dual_writer.v \
	         src/psram_bus_arbiter.v $(CELLS_SIM)
	$(VVP) /tmp/$(PROJECT)_arb.vvp
	@rm -f /tmp/$(PROJECT)_arb.vvp

# Self-checking test of the UNIFIED tri-capture master: DAC replay (read) + ADC + LA
# (write) concurrently through ONE psram_tri_master + one shared pad set (reuses the
# combined APS6404L model).  Needs cells_sim.v for the SB_IO pads.
tritest:
	iverilog -g2012 -I src -o /tmp/$(PROJECT)_tri.vvp \
	         sim/tb_psram_tri_master.v src/psram_tri_master.v $(CELLS_SIM)
	$(VVP) /tmp/$(PROJECT)_tri.vvp
	@rm -f /tmp/$(PROJECT)_tri.vvp

# Closed-loop DAC control engine: convergence + clamp + strm + determinism.
looptest:
	iverilog -g2012 -I src -o /tmp/$(PROJECT)_loop.vvp \
	         sim/tb_dac_loop.v src/dac_loop.v
	$(VVP) /tmp/$(PROJECT)_loop.vvp
	@rm -f /tmp/$(PROJECT)_loop.vvp

# The real dac_loop + dac8551_engine: every DAC frame is a whole value of the loop's `v`,
# never {new high byte, old low byte} (v38; 176 of 1111 frames were torn before).
loopframetest:
	iverilog -g2012 -I src -o /tmp/$(PROJECT)_loopframe.vvp \
	         sim/tb_dac_loop_frames.v src/dac_loop.v src/dac8551_engine.v
	$(VVP) /tmp/$(PROJECT)_loopframe.vvp
	@rm -f /tmp/$(PROJECT)_loopframe.vvp

# Subset (DAC-disabled) + region-backstop coverage for the tri-master.
trimodetest:
	iverilog -g2012 -I src -o /tmp/$(PROJECT)_trimode.vvp \
	         sim/tb_psram_tri_modes.v src/psram_tri_master.v $(CELLS_SIM)
	$(VVP) /tmp/$(PROJECT)_trimode.vvp
	@rm -f /tmp/$(PROJECT)_trimode.vvp

# Self-checking simulation of the deep LA→PSRAM sampler (byte packing + framing).
lapsramtest:
	iverilog -g2012 -o /tmp/$(PROJECT)_lapsram.vvp \
	         sim/tb_la_psram_capture.v src/la_psram_capture.v
	$(VVP) /tmp/$(PROJECT)_lapsram.vvp
	@rm -f /tmp/$(PROJECT)_lapsram.vvp

# Self-checking simulation of the cmd_dispatch MEASURE decode (DAC+ADC co-start).
measuretest:
	iverilog -g2012 -I src -o /tmp/$(PROJECT)_meas.vvp \
	         sim/tb_measure.v src/cmd_dispatch.v
	$(VVP) /tmp/$(PROJECT)_meas.vvp
	@rm -f /tmp/$(PROJECT)_meas.vvp

# Self-checking simulation of the cmd_dispatch payload byte-counter (the arg_rem
# down-counter that replaced the per-state arg_count+1>=arg_len add-compare).
argtest:
	iverilog -g2012 -I src -o /tmp/$(PROJECT)_args.vvp \
	         sim/tb_dispatch_args.v src/cmd_dispatch.v
	$(VVP) /tmp/$(PROJECT)_args.vvp
	@rm -f /tmp/$(PROJECT)_args.vvp

# Self-checking simulation of the stepper engine pulse-train generator.
steppertest:
	iverilog -g2012 -o /tmp/$(PROJECT)_step.vvp \
	         sim/tb_stepper.v src/stepper_engine.v
	$(VVP) /tmp/$(PROJECT)_step.vvp
	@rm -f /tmp/$(PROJECT)_step.vvp

# Verify the generated opcode files (gateware .vh + both firmware .h + the doc)
# are in sync with the single-source table in tools/gen_protocol.py.
check-protocol:
	python3 ../tools/gen_protocol.py --check

# Top-level integration smoke test: drives real SPI transactions through the
# SB_IO/SB_GB/SB_RGBA_DRV pad primitives (needs yosys' cells_sim.v) and checks the
# instant-response commands (PING/VERSION/STATUS).  This is the only bench that
# exercises the top_v2 wiring seam — spi_slave -> cmd_dispatch -> MISO through the
# pads + clk48÷2 + power-on reset — where the last two field bugs actually lived.
# Uses $(SRC) (the v2 build set by default) so it tracks the shipping top.
topsmoketest:
	iverilog -g2012 -I src -o /tmp/$(PROJECT)_top.vvp -s tb_top_v2 \
	         sim/tb_top_v2.v $(SRC) $(CELLS_SIM)
	$(VVP) /tmp/$(PROJECT)_top.vvp
	@rm -f /tmp/$(PROJECT)_top.vvp

# Top-level end-to-end capture: SPI OP_CAPTURE -> producers -> rings -> dual writer
# -> the two PSRAM regions, with the tb acting as the PSRAM write-slave.  Catches
# datapath/arbiter/region-select bugs the isolated dualwrtest can't.  Also decodes the
# DAC8551 pins across a restart onto a shorter waveform (the v33 clk48 control-CDC fix).
topcapturetest:
	iverilog -g2012 -I src -o /tmp/$(PROJECT)_topcap.vvp -s tb_top_capture \
	         sim/tb_top_capture.v $(SRC) $(CELLS_SIM)
	$(VVP) /tmp/$(PROJECT)_topcap.vvp
	@rm -f /tmp/$(PROJECT)_topcap.vvp

# Run every self-checking testbench, then smoke-build the v1 gateware.
test: simtest uarttest cdctest dactest daclooptest dacpsramrdtest dacpsramskewtest dacpsramreplaytest adctest adctimingtest dualwrtest arbtest tritest trimodetest looptest loopframetest multiboot lapsramtest measuretest argtest steppertest topsmoketest topcapturetest check-protocol
	@echo "All gateware testbenches passed."

clean:
	# no underscore in the glob: also catches legacy non-suffixed artifacts
	# (bench_pod_fpga.json/.asc/.bin) from before the per-board naming scheme.
	rm -f $(PROJECT)*.json $(PROJECT)*.asc $(PROJECT)*.bin $(PROJECT)*.pnr.log

help:
	@echo "Usage: make [target]        (BOARD defaults to vbench_pod, the STM32 carrier)"
	@echo ""
	@echo "Targets:"
	@echo "  all          Build bitstream for BOARD (default; one image, the loop netlist)"
	@echo "  images       Build the loop + deep images the firmware embeds (what pods run)"
	@echo "               PNR_STRICT=1 fails on a fallback seed or clk48 < MIN_CLK48_MHZ (50)"
	@echo "  flash        Build + program SPI flash via iceprog (persistent)"
	@echo "  flash-ram    Build + load to FPGA SRAM only (volatile, faster)"
	@echo "  prog         Alias for 'flash'"
	@echo "  sim          Quick Verilog syntax check via iverilog"
	@echo "  clean        Remove generated artefacts"
	@echo "  help         Show this message"

# (labuftest removed in v24 with la_capture_buf.v — the on-FPGA I2C-LA trace buffer is gone.)
