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
      src/spram_ring48.v      \
      src/la_byte_pack.v      \
      src/la48_sampler.v      \
      src/la_change_capture.v \
      src/dac8551_engine.v    \
      src/cdc_pulse_payload.v \
      src/dac_loop.v          \
      src/dac_psram_reader.v  \
      src/psram_bus_arbiter.v \
      src/la_bank.v           \
      src/stepper_engine.v    \
      src/swd_engine.v        \
      src/i2c_master.v        \
      src/jtag_seq.v          \
      src/swim_engine.v       \
      src/i2c_emu.v           \
      src/emu_fifo.v          \
      src/spi_target.v        \
      src/la_psram_capture.v  \
      src/uart_engine.v       \
      src/dsp_counter.v       \
      src/dsp_counter2.v      \
      src/lc_counter.v        \
      src/psram_pads.v        \
      src/cell_gearbox48.v
DEFINES = -DVERSION_V2
# The deep-replay image is the same top with this on top of DEFINES (the loop image adds nothing).
DEEP_DEFINES = -DUSE_DEEP_REPLAY
# v54: the emulation image (the emulated sensors; no control loop, no deep replay). src/image_config.vh.
EMU_DEFINES = -DUSE_EMULATION
# Per-clock timing constraints for the two v2 clock domains (clk48 / clk).
PREPACK = --pre-pack clocks.py
# Pinned PnR seeds.  Every image has its own netlist and so its own pinned seed: placement, not
# the 24 MHz `clk` target, is what moves from seed to seed, and the clk48 margin and the untimed
# clk <-> clk48 crossings (MIN_CLK48_MHZ, MAX_X48_NS, MAX_X24_NS below) decide which seed is pinned.
# --threads 1 keeps PnR reproducible.  A netlist change re-rolls placement: re-sweep
# (make seed-sweep IMAGE=...), re-pin, and re-verify on hardware.  Why each pin was chosen,
# version by version, is in ../docs/design/ice40-seed-history.md.
# The default single image is the loop netlist (identical DEFINES), so it follows SEED_LOOP.
SEED = $(SEED_LOOP)
# Fallback seeds: a pinned seed is not portable across toolchain builds (CI's OSS CAD Suite and a
# local Homebrew yosys map the design slightly differently), so the pnr macro tries the pin first
# and then each fallback until one places and closes timing.  A fallback build prints a loud
# warning: it is not the hardware-verified placement.  PNR_STRICT=1 tries no fallback.
SEED_FALLBACK ?= $(SEED_LOOP_FALLBACK)

PCF     = $(BOARD).pcf
OUT     = $(PROJECT)_$(BOARD)

DEVICE  = up5k
PACKAGE = sg48

# Common nextpnr-ice40 invocation.  --threads 1 keeps PnR reproducible.
NEXTPNR_FLAGS = --$(DEVICE) --package $(PACKAGE) --pcf $(PCF) --pcf-allow-unconstrained $(PREPACK) --threads 1
NEXTPNR = nextpnr-ice40 $(NEXTPNR_FLAGS)

# ---- PnR policy: placement/margin gates ----
# PNR_STRICT=0 (default) keeps the historical behaviour: a fallback seed, or a clk48 Fmax below
#   MIN_CLK48_MHZ, prints a loud WARNING and the bitstream is kept, so a toolchain build that
#   maps the design differently (CI vs local — see SEED_FALLBACK) still completes.
# PNR_STRICT=1 makes either one FAIL the build (the .asc is removed so a re-run can't pick it
#   up) and tries no fallback seed.  Use it for anything that ships.  CI always runs with it
#   (.github/workflows/test.yml).
# Either way, a nextpnr log without the clk48 Fmax or the clk -> clk48 max-delay line FAILS the
#   build (v47): the gates below read those lines, and a missing one used to pass silently.
# MIN_CLK48_MHZ: nextpnr only fails clk48 below 48.00 MHz, but placements closing at
#   48.2-48.8 MHz have silently HW-regressed deep DAC replay (the v28/v31 notes), and seeds are
#   pinned for >= 50.  Checked against the final post-route Fmax.
# MAX_X48_NS: clocks.py declares clk and clk48 unrelated, so nextpnr does not time the clk -> clk48
#   paths (the PSRAM writer's and reader's gearboxes into the clk48 serializers).  They must fit in
#   one clk48 period (20.8 ns) minus how far clk lags clk48 through its divider flop and global
#   buffer (a few ns).  nextpnr still reports their max delay; hardware-verified v41 placements sit
#   at 12.1-14.5 ns.  Checked like MIN_CLK48_MHZ (warning; a failure with PNR_STRICT=1).
# MAX_X24_NS (v50): the other direction, clk48 -> clk, also untimed.  Since v50 it carries the
#   48 MS/s LA sample pairs (la48_sampler -> la_change_capture's ra/rb), latched one clk48 period
#   before clk rises, so a path gets about 20.8 ns plus clk's lag.  It must stay flop-to-flop:
#   <= 6.2 ns before v50; 32.6 ns when the encoder's compares read the pair directly.
# MIN_CLK_MHZ: the same margin gate for the 24 MHz `clk` domain, where nextpnr only fails below
#   24.00 MHz.  The emulation image is bound by clk, not clk48 (v59 pins: loop 26.75, deep 27.72,
#   emu 26.08 MHz).  Checked like MIN_CLK48_MHZ (warning; a failure with PNR_STRICT=1).
PNR_STRICT    ?= 0
MIN_CLK48_MHZ ?= 50
MIN_CLK_MHZ   ?= 25.5
MAX_X48_NS    ?= 15
MAX_X24_NS    ?= 15

# $(call pnr,<in.json>,<out.asc>,<pinned seed>,<fallback seeds>)
#
# Place & route with the PINNED seed first; only if that seed fails (no legal
# placement, or a timing miss — nextpnr errors on both) walk the fallback list
# (not with PNR_STRICT=1).  See the SEED_FALLBACK comment for why a single pin is not
# portable across toolchain builds.  A fallback build is loudly flagged: it is NOT the
# placement that was verified on hardware.  The routed clk48 Fmax is then checked
# against MIN_CLK48_MHZ and the clk Fmax against MIN_CLK_MHZ.
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
	    fclk=$$(grep -E "Max frequency for clock +'clk'" $$log | tail -n 1 | sed -E 's/.*: *([0-9.]+) MHz.*/\1/'); \
	    x48=$$(grep -E "Max delay posedge clk +-> posedge clk48" $$log | tail -n 1 | sed -E 's/.*: *([0-9.]+) ns.*/\1/'); \
	    x24=$$(grep -E "Max delay posedge clk48[^ ]* +-> posedge clk +:" $$log | tail -n 1 | sed -E 's/.*: *([0-9.]+) ns.*/\1/'); \
	    if [ -z "$$fclk" ]; then \
	      rm -f $(2); \
	      echo "ERROR: $(2): the nextpnr log ($$log) has no clk Max frequency line, so the"; \
	      echo "       MIN_CLK_MHZ gate cannot run."; \
	      exit 1; \
	    fi; \
	    if [ -z "$$x24" ]; then \
	      rm -f $(2); \
	      echo "ERROR: $(2): the nextpnr log ($$log) has no clk48 -> clk max delay line, so the"; \
	      echo "       MAX_X24_NS gate cannot run."; \
	      exit 1; \
	    fi; \
	    if [ -z "$$f48" ] || [ -z "$$x48" ]; then \
	      rm -f $(2); \
	      echo "ERROR: $(2): the nextpnr log ($$log) has no $${f48:+clk -> clk48 max delay}$${f48:-clk48 Max frequency} line,"; \
	      echo "       so the clk48 gates cannot run.  A renamed clock or a changed nextpnr report format"; \
	      echo "       used to pass here silently."; \
	      exit 1; \
	    fi; \
	    if awk -v f="$$f48" -v m="$(MIN_CLK48_MHZ)" 'BEGIN { exit !(f < m) }'; then \
	      echo "*** WARNING: $(2): clk48 closes at only $$f48 MHz (MIN_CLK48_MHZ=$(MIN_CLK48_MHZ))."; \
	      echo "***          Static timing passes, but margin this thin has HW-regressed deep"; \
	      echo "***          DAC replay before — re-sweep and re-pin the seed."; \
	      bad="$${bad:+$$bad, }clk48 $$f48 MHz"; \
	    fi; \
	    if awk -v f="$$fclk" -v m="$(MIN_CLK_MHZ)" 'BEGIN { exit !(f < m) }'; then \
	      echo "*** WARNING: $(2): clk closes at only $$fclk MHz (MIN_CLK_MHZ=$(MIN_CLK_MHZ))."; \
	      echo "***          Static timing passes at 24 MHz, but the margin is thin; re-sweep the seed."; \
	      bad="$${bad:+$$bad, }clk $$fclk MHz"; \
	    fi; \
	    echo "nextpnr: $(2): clk -> clk48 (untimed) max delay $$x48 ns (MAX_X48_NS=$(MAX_X48_NS))"; \
	    if awk -v x="$$x48" -v m="$(MAX_X48_NS)" 'BEGIN { exit !(x > m) }'; then \
	      echo "*** WARNING: $(2): the untimed clk -> clk48 paths take $$x48 ns (MAX_X48_NS=$(MAX_X48_NS))."; \
	      echo "***          They must fit in 20.8 ns minus clk's lag behind clk48; re-sweep the seed."; \
	      bad="$${bad:+$$bad, }clk->clk48 $$x48 ns"; \
	    fi; \
	    echo "nextpnr: $(2): clk48 -> clk (untimed) max delay $$x24 ns (MAX_X24_NS=$(MAX_X24_NS))"; \
	    if awk -v x="$$x24" -v m="$(MAX_X24_NS)" 'BEGIN { exit !(x > m) }'; then \
	      echo "*** WARNING: $(2): the untimed clk48 -> clk paths take $$x24 ns (MAX_X24_NS=$(MAX_X24_NS))."; \
	      echo "***          A clk48 signal feeds clk logic without a flop first, or the seed placed it far."; \
	      bad="$${bad:+$$bad, }clk48->clk $$x24 ns"; \
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

# $(call topbench,<bench module>,<images>): compile and run a whole-top bench (sim/<module>.v + $(SRC))
# once per image, `loop` (the default DEFINES) and/or `deep` (-DUSE_DEEP_REPLAY).  The two images
# are different netlists of the same top, so a top-level bench runs on both unless it tests a
# block only one image carries: a change that only one image's build exercised used to break the
# other one silently.
define topbench
	@set -e; for img in $(2); do \
	  srcs="$(SRC)"; \
	  case $$img in deep) defs="$(DEEP_DEFINES)";; emu) defs="$(EMU_DEFINES)";; *) defs=;; esac; \
	  echo "== $(1): $$img image"; \
	  iverilog -g2012 -I src $$defs -o /tmp/$(PROJECT)_$(1)_$$img.vvp -s $(1) sim/$(1).v $$srcs $(CELLS_SIM); \
	  $(VVP) /tmp/$(PROJECT)_$(1)_$$img.vvp; \
	  rm -f /tmp/$(PROJECT)_$(1)_$$img.vvp; \
	done
endef

.PHONY: i2cemutest all images seed-sweep promote check-release release-images flash flash-ram prog sim uarttest cdctest dactest daclooptest dacpsramrdtest dacpsramskewtest dacpsramreplaytest adctest adctimingtest dualwrtest arbtest looptest loopframetest dsptest multiboot lapsramtest measuretest argtest steppertest topsmoketest topcapturetest topadct0test topboottest topspilinktest topdeeptest topmuxtest spitest swdqtest uart2test topemutest ringtest lachangetest toplachangetest jtagtest swimtest i2cmastertest check-protocol releasetooltest test clean help

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
#
# $(call synth,<extra -D flags>,<out.json>): synth_ice40 in three segments so the explicit SB_MAC16
# cells (dsp_counter, v41) sit out the `coarse` step as SB_MAC16_KEEP: yosys 0.65's ice40_dsp
# (run there by -dsp) rewrites any SB_MAC16 as a multiplier, which killed the counters in the
# netlist while every RTL bench passed (synth/sb_mac16_keep.v).  The control loop's multipliers
# are still $mul in that step, so -dsp infers them exactly as before.  synth/check_dsp.py then
# fails the build (and removes the netlist) unless every dsp_counter cell came through intact.
SYNTH_FLAGS = -abc9 -dsp -spram -top top
SYNTH_AUX   = synth/sb_mac16_keep.v synth/check_dsp.py
define synth
	yosys -p "read_verilog -lib synth/sb_mac16_keep.v; read_verilog -Isrc $(DEFINES) $(1) $(or $(3),$(SRC)); \
	  synth_ice40 $(SYNTH_FLAGS) -run :coarse; chtype -set SB_MAC16_KEEP t:SB_MAC16; \
	  synth_ice40 $(SYNTH_FLAGS) -run coarse:map_ram; chtype -set SB_MAC16 t:SB_MAC16_KEEP; \
	  synth_ice40 $(SYNTH_FLAGS) -run map_ram: -json $(2)"
	python3 synth/check_dsp.py $(2) src || { rm -f $(2); exit 1; }
endef
$(OUT).json: $(SRC) $(SYNTH_AUX)
	$(call synth,,$@)

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
# NOTE: keep these assignments free of trailing comments — the value is compared against the
# seed actually used, and make would keep the trailing whitespace in it.
# v63 seeds: local sweep 1-64 (Yosys 0.65, nextpnr 0.10) and CI sweep 1-64 (run 45406106727,
# Yosys 0.65+7).  Each pin is the seed with the best worst relative margin over the four gates
# (clk48 >= 50 MHz, clk >= 25.5 MHz, clk -> clk48 <= 15 ns, clk48 -> clk <= 15 ns) in BOTH toolchains.
# Loop image: seed 11 since v63 (local 57 ok, CI 51 ok, 44 in both).  Local: clk48 56.69 MHz,
# clk 27.93 MHz, clk -> clk48 10.62 ns, clk48 -> clk 8.50 ns.  CI: clk48 55.29 MHz, clk 29.24 MHz,
# clk -> clk48 10.19 ns, clk48 -> clk 6.66 ns.  Seed 38 had the best margin but FAILED on hardware:
# the LA sampled at 0.495x its reported rate (SDK e2e test_la_sample_rate_matches_the_host_clock,
# 30a9f1, 5 of 5 runs; seed 11 and the v62 image pass on the same pod and firmware).  The four
# gates do not catch that placement fault, so a new loop pin needs that test on hardware.
# (v61: seed 20; v62: seed 34.)
SEED_LOOP ?= 11
# The next seeds that pass both toolchains, by worst relative margin (38 is excluded, see above).
SEED_LOOP_FALLBACK ?= 13 51 49 45
# Deep image: seed 31 since v63 (local 16 ok, CI 22 ok, only 31 and 63 in both; v62's seed 35
# fails the local x48 gate at 19.05 ns).  Local: clk48 54.63 MHz, clk 26.24 MHz, clk -> clk48
# 14.56 ns, clk48 -> clk 4.39 ns.  CI: clk48 52.55 MHz, clk 27.55 MHz, clk -> clk48 12.80 ns,
# clk48 -> clk 4.39 ns.  Passes the LA sample-rate test on hardware.  (v60: seed 24; v62: seed 35.)
SEED_DEEP ?= 31
# 63 passes both toolchains (thinner on clk48 locally: 50.98 MHz); 51, 4 and 1 pass CI only.
SEED_DEEP_FALLBACK ?= 63 51 4 1
# Emulation image (USE_EMULATION): seed 41 since v63 (local 56 ok, CI 38 ok, 28 in both).
# Local: clk48 65.32 MHz, clk 27.69 MHz, clk -> clk48 13.22 ns, clk48 -> clk 7.62 ns.  CI: clk48
# 54.76 MHz, clk 28.83 MHz, clk -> clk48 13.86 ns, clk48 -> clk 7.75 ns.  Seeds 52 and 6 had more
# margin but halved the LA sample rate on hardware, like loop seed 38 (see above); 41 and the v62
# image pass.  It is the fullest image (5202/5280 LC, 98.5%).  Loop 4339 LC (82%), deep 4655 LC
# (88%).  (v60: seed 60; v62: seed 30.)
SEED_EMU ?= 41
# The next seeds that pass both toolchains, by worst relative margin (52 and 6 are excluded).
SEED_EMU_FALLBACK ?= 63 57 27 47
MULTI = $(OUT)_multi.bin
# The GATEWARE_VERSION the images are built with, read from the source they are built from and
# written next to each .bin (<image>.bin.gwversion). The firmware embeds it so a pod can tell
# that the gateware in its iCE40 config flash is not the one its firmware shipped with.
GW_VERSION := $(shell sed -n "s/.*GATEWARE_VERSION(8'd\([0-9]*\)).*/\1/p" src/top_v2.v | head -1)

$(OUT)_loop.json: $(SRC) $(SYNTH_AUX)
	$(call synth,,$@)
$(OUT)_deep.json: $(SRC) $(SYNTH_AUX)
	$(call synth,$(DEEP_DEFINES),$@)
$(OUT)_emu.json: $(SRC) $(SYNTH_AUX)
	$(call synth,$(EMU_DEFINES),$@)
$(OUT)_loop.bin: $(OUT)_loop.json $(PCF)
	$(call pnr,$<,$(OUT)_loop.asc,$(SEED_LOOP),$(SEED_LOOP_FALLBACK))
	icepack $(OUT)_loop.asc $@
	echo $(GW_VERSION) > $@.gwversion
$(OUT)_deep.bin: $(OUT)_deep.json $(PCF)
	$(call pnr,$<,$(OUT)_deep.asc,$(SEED_DEEP),$(SEED_DEEP_FALLBACK))
	icepack $(OUT)_deep.asc $@
	echo $(GW_VERSION) > $@.gwversion
$(OUT)_emu.bin: $(OUT)_emu.json $(PCF)
	$(call pnr,$<,$(OUT)_emu.asc,$(SEED_EMU),$(SEED_EMU_FALLBACK))
	icepack $(OUT)_emu.asc $@
	echo $(GW_VERSION) > $@.gwversion
# -p0: power-on image = slot 0 (loop).  -a16: align images at 64 KB (SB_WARMBOOT jumps here).
multiboot: $(OUT)_loop.bin $(OUT)_deep.bin
	icemulti -p0 -a16 -o $(MULTI) $(OUT)_loop.bin $(OUT)_deep.bin
	@echo "multiboot image -> $(MULTI): slot0=closed-loop image, slot1=deep-DAC-replay"

# The three runtime images (loop, deep, emu) that stm32h563/Makefile installs as config-flash
# blobs: what pods actually run and what a release must build.  `make` (all) builds only one image.
images: $(OUT)_loop.bin $(OUT)_deep.bin $(OUT)_emu.bin

# Sweep PnR seeds for one image on its current netlist: make seed-sweep IMAGE=deep [SWEEP="1 32"].
# Pin the seed with the best clk48 margin that is also good in CI's toolchain (the
# ice40-seed-sweep workflow runs this same script there).  The sweep classifies each seed with
# the same gates as the pnr macro (MIN_CLK48_MHZ, MIN_CLK_MHZ, MAX_X48_NS, MAX_X24_NS).
IMAGE ?= loop
SWEEP ?= 1 24
seed-sweep: $(OUT)_$(IMAGE).json $(PCF)
	MIN_CLK48_MHZ=$(MIN_CLK48_MHZ) MIN_CLK_MHZ=$(MIN_CLK_MHZ) MAX_X48_NS=$(MAX_X48_NS) MAX_X24_NS=$(MAX_X24_NS) \
	  sh synth/seed_sweep.sh $(OUT)_$(IMAGE).json $(SWEEP)

# ---- hardware-verified images for firmware releases (tools/ice40_release.py) ----
# The release job ships ice40/release/, NOT a fresh CI build: CI's toolchain places the same
# sources differently (v40: loop clk48 56.95 MHz locally, 48.84 MHz in CI).  After an `images`
# build has passed on hardware:  make promote HW_VERIFIED="e2e 132 passed on benchpod-baea06"
# and commit ice40/release/ with the sources.  check-release fails once the sources move on.
# promote refuses an image older than any gateware source, and records the toolchain (yosys, the
# nextpnr revision) and the flags below in the MANIFEST; check-release notes when they have moved.
RELEASE_TOOL  = python3 ../tools/ice40_release.py
# (--opt=value: the values start with a dash, which argparse would take for an option)
RELEASE_FLAGS = --synth-flags="$(SYNTH_FLAGS)" --pnr-flags="$(NEXTPNR_FLAGS)" \
                --defines-loop="$(DEFINES)" --defines-deep="$(DEFINES) $(DEEP_DEFINES)" \
                --defines-emu="$(DEFINES) $(EMU_DEFINES)"
promote:
	@test -n "$(HW_VERIFIED)" || { echo 'usage: make promote HW_VERIFIED="<which hardware test this build passed>"'; exit 2; }
	$(RELEASE_TOOL) promote --hw-verified "$(HW_VERIFIED)" --seed-loop $(SEED_LOOP) --seed-deep $(SEED_DEEP) --seed-emu $(SEED_EMU) $(RELEASE_FLAGS)
check-release:
	$(RELEASE_TOOL) check $(RELEASE_FLAGS)
# Put the promoted images where stm32h563/Makefile picks them up as blobs (what the release job runs).
release-images:
	$(RELEASE_TOOL) install $(RELEASE_FLAGS)

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

# (simtest, the v24..v53 single I2C target, was replaced in v54 by i2cemutest.)

# v54: the time-multiplexed I2C target engine (4 targets) against an I2C master model.
i2cemutest:
	iverilog -g2012 -o /tmp/$(PROJECT)_i2cemu.vvp -s tb_i2c_emu \
	         sim/tb_i2c_emu.v src/i2c_emu.v $(CELLS_SIM)
	$(VVP) /tmp/$(PROJECT)_i2cemu.vvp
	@rm -f /tmp/$(PROJECT)_i2cemu.vvp

# Self-checking loopback simulation of the soft UART (needs iverilog/vvp).
uarttest:
	iverilog -g2012 -o /tmp/$(PROJECT)_uart.vvp \
	         sim/tb_uart_engine.v src/uart_engine.v src/lc_counter.v $(CELLS_SIM)
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

# DAC8554 support (v49): the stream control byte and the one-shot static frame (OP_DAC_CTRL).
dac8554test:
	iverilog -g2012 -o /tmp/$(PROJECT)_dac8554.vvp \
	         sim/tb_dac8554.v src/dac8551_engine.v
	$(VVP) /tmp/$(PROJECT)_dac8554.vvp
	@rm -f /tmp/$(PROJECT)_dac8554.vvp

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
	         sim/tb_dac_psram_reader.v src/dac_psram_reader.v src/cell_gearbox48.v src/psram_pads.v $(CELLS_SIM)
	$(VVP) /tmp/$(PROJECT)_dacpsramrd.vvp
	@rm -f /tmp/$(PROJECT)_dacpsramrd.vvp

# REGRESSION guard for the deep-replay/PSRAM curse: models the clk<->clk48 SB_GB skew
# AND a realistic PSRAM read round-trip, and checks the reader delivers every byte across
# the sweep.  The v28 single-clk48-domain reader passes all skews; a clk-domain PSRAM
# sampler (the pre-v28 bug, or any regression to it) fails at the larger skews.
dacpsramskewtest:
	iverilog -g2012 -I src -o /tmp/$(PROJECT)_dacskew.vvp \
	         sim/tb_dac_psram_skew.v src/dac_psram_reader.v src/cell_gearbox48.v src/psram_pads.v $(CELLS_SIM)
	$(VVP) /tmp/$(PROJECT)_dacskew.vvp
	@rm -f /tmp/$(PROJECT)_dacskew.vvp

# Self-checking simulation of the streaming DAC engine popping a PSRAM waveform out
# of the reader FIFO (reader -> dac8551_engine in psram_mode) to the DAC8551 frames.
dacpsramreplaytest:
	iverilog -g2012 -I src -o /tmp/$(PROJECT)_dacpsramrep.vvp \
	         sim/tb_dac_psram_replay.v src/dac_psram_reader.v src/cell_gearbox48.v src/dac8551_engine.v src/psram_pads.v $(CELLS_SIM)
	$(VVP) /tmp/$(PROJECT)_dacpsramrep.vvp
	@rm -f /tmp/$(PROJECT)_dacpsramrep.vvp

# (ilatest removed in v24 with the on-FPGA I2C-LA sampler — i2c_la_capture.v is gone.)

# Self-checking simulation of the v2 MCP33131D-10 serial ADC capture engine.
adctest:
	iverilog -g2012 -o /tmp/$(PROJECT)_adc.vvp \
	         sim/tb_adc_mcp33131.v src/adc_mcp33131.v $(CELLS_SIM)
	$(VVP) /tmp/$(PROJECT)_adc.vvp
	@rm -f /tmp/$(PROJECT)_adc.vvp

# Timing + throughput sign-off of the PRODUCTION adc_mcp33131 (24 MHz clk, 12 MHz SCLK, its SDO
# SB_IO from the cells model) against the analog-accurate mcp33131_model (tEN/tDO + pad and cable
# round trip).  Measures setup and hold at the falling-edge sample (v63) from zero delay to a
# 45.5 ns round trip, and shows a round trip over 1.5 clk as the failing edge.
adctimingtest:
	iverilog -g2012 -o /tmp/$(PROJECT)_adct.vvp \
	         sim/tb_adc_timing.v sim/mcp33131_model.v src/adc_mcp33131.v $(CELLS_SIM)
	$(VVP) /tmp/$(PROJECT)_adct.vvp
	@rm -f /tmp/$(PROJECT)_adct.vvp

# Self-checking test of the DUAL-region QPI writer: two streams (ADC + raw LA)
# into two contiguous PSRAM regions off one trigger, plus tCEM/setup/bus_own.
# Instantiates the REAL SB_IO pads (DDR SCLK + comb data on clk48, exactly like
# top_v2) so the 48 MHz DDR drain's mode-0 pad timing is exercised, not just the
# FSM — hence it needs yosys' cells_sim.v.
dualwrtest:
	iverilog -g2012 -I src -o /tmp/$(PROJECT)_dualwr.vvp \
	         sim/tb_psram_dual_writer.v src/psram_dual_writer.v src/cell_gearbox48.v src/dsp_counter.v $(CELLS_SIM)
	$(VVP) /tmp/$(PROJECT)_dualwr.vvp
	@rm -f /tmp/$(PROJECT)_dualwr.vvp

# Self-checking test of CONCURRENT DAC replay + ADC/LA capture over ONE shared quad
# bus, time-multiplexed by psram_bus_arbiter (burst-granular ownership).  Runs the
# reader (0xEB, streaming out of the top region) and the dual writer (0x38, draining
# into the bottom regions) simultaneously through top_v2's real pad mux + one
# APS6404L model; checks no bus collision, no DAC underrun, correct data both ways.
arbtest:
	iverilog -g2012 -I src -o /tmp/$(PROJECT)_arb.vvp \
	         sim/tb_psram_arbiter.v src/dac_psram_reader.v src/psram_dual_writer.v src/cell_gearbox48.v src/dsp_counter.v src/psram_pads.v \
	         src/psram_bus_arbiter.v $(CELLS_SIM)
	$(VVP) /tmp/$(PROJECT)_arb.vvp
	@rm -f /tmp/$(PROJECT)_arb.vvp

# dsp_counter (an SB_MAC16 accumulator) == the fabric counter it replaces, cycle for cycle (v41).
dsptest:
	iverilog -g2012 -I src -o /tmp/$(PROJECT)_dsp.vvp \
	         sim/tb_dsp_counter.v src/dsp_counter.v src/dsp_counter2.v $(CELLS_SIM)
	$(VVP) /tmp/$(PROJECT)_dsp.vvp
	@rm -f /tmp/$(PROJECT)_dsp.vvp

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

# Self-checking simulation of the deep LA→PSRAM sampler (byte packing + framing).
lapsramtest:
	iverilog -g2012 -o /tmp/$(PROJECT)_lapsram.vvp \
	         sim/tb_la_psram_capture.v src/la_psram_capture.v src/dsp_counter.v $(CELLS_SIM)
	$(VVP) /tmp/$(PROJECT)_lapsram.vvp
	@rm -f /tmp/$(PROJECT)_lapsram.vvp

# Self-checking simulation of the cmd_dispatch MEASURE decode (DAC+ADC co-start).
# The deep image's decoder: START_DAC_PSRAM is decoded only there (v60).
measuretest:
	iverilog -g2012 -I src $(DEEP_DEFINES) -o /tmp/$(PROJECT)_meas.vvp \
	         sim/tb_measure.v src/cmd_dispatch.v
	$(VVP) /tmp/$(PROJECT)_meas.vvp
	@rm -f /tmp/$(PROJECT)_meas.vvp

# Self-checking simulation of the cmd_dispatch payload byte-counter (the arg_rem
# down-counter that replaced the per-state arg_count+1>=arg_len add-compare).
# v54: once as the loop image's decoder, once as the emulation image's (the I2C_* commands).
argtest:
	@set -e; for d in "" "$(DEEP_DEFINES)" "$(EMU_DEFINES)"; do \
	  echo "== tb_dispatch_args $$d"; \
	  iverilog -g2012 -I src $$d -o /tmp/$(PROJECT)_args.vvp sim/tb_dispatch_args.v src/cmd_dispatch.v; \
	  $(VVP) /tmp/$(PROJECT)_args.vvp; \
	done
	@rm -f /tmp/$(PROJECT)_args.vvp

# Self-checking simulation of the stepper engine pulse-train generator.
steppertest:
	iverilog -g2012 -o /tmp/$(PROJECT)_step.vvp \
	         sim/tb_stepper.v src/stepper_engine.v sim/stepper_engine_ref.v src/dsp_counter2.v $(CELLS_SIM)
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
# Uses $(SRC) so it tracks the shipping top; like every whole-top bench below, it runs on both
# images (topbench).
topsmoketest:
	$(call topbench,tb_top_v2,loop deep emu)

# Top-level end-to-end capture: SPI OP_CAPTURE -> producers -> rings -> dual writer
# -> the two PSRAM regions, with the tb acting as the PSRAM write-slave.  Catches
# datapath/arbiter/region-select bugs the isolated dualwrtest can't.  Also decodes the
# DAC8551 pins across a restart onto a shorter waveform (the v33 clk48 control-CDC fix), and
# OP_DAC_CTRL's control byte and static frames (v49, the DAC8554).
topcapturetest:
	$(call topbench,tb_top_capture,loop deep)

# The DEEP image's whole top (-DUSE_DEEP_REPLAY, v46): a deep replay keeps its sample order while
# the STM32 takes the bus (bus_own swept over a reader burst and the DAC engine's byte pops), a
# capture alongside the replay read back mid-replay, and an overflowed capture then a clean one.
# v52: the ADC capture's sample 0 is converted at a fixed offset from t0, the LA's sample-0 instant,
# whatever the arm's phase against the free-running engine, the previous divider or a trigger.
topadct0test:
	$(call topbench,tb_top_adc_t0,loop deep)

# v52: the PSRAM pads idle (CS high, SCLK low, IO released) from configuration on, both images.
topboottest:
	$(call topbench,tb_top_boot,loop deep)

# v52: the STM32 -> iCE40 SPI link at 1.95, 3.9 and 5 MHz against an STM32 model (pad delays, MISO
# setup, a drifting phase): PING/VERSION/STATUS and a 256-byte register write + streamed read-back.
topspilinktest:
	$(call topbench,tb_top_spilink,loop deep emu)

topdeeptest:
	$(call topbench,tb_top_deep,deep)

# la_bank's driver priority through the whole top (P2 T-2): static, UART, I2C, stepper and the
# SPI engine armed over the SPI link onto one pad, each taking it over and handing it back, plus
# the channel >= N guard.
topmuxtest:
	$(call topbench,tb_top_la_mux,emu)

# SPI master (v44): SPI_ARM/CS/STATUS + SWD_FEED/READ through the whole top, against a
# behavioural SPI flash on the LA pads (ID, program, erase, read; modes 0 and 3; re-arm; SWD after).
spitest:
	$(call topbench,tb_top_spi,loop deep)

# UART2 (v48): the TX-only second UART next to the proxy through the whole top: both decoded
# on the pads with bit timing, re-armed on another pin and rate, then disarmed.
uart2test:
	$(call topbench,tb_top_uart2,emu)

# v54: the emulated sensors through the whole emulation image: I2C targets, the FIFO, data ready
# and the SPI target, driven over the SPI link, with bus models on the LA pads.
topemutest:
	$(call topbench,tb_top_emu,emu)

# SWD transfer queue (v45): SWD_QFEED/QCONFIG/QSTATUS through the whole top against a SW-DP
# model (mixed reads/writes, a queue bigger than the link delivers, WAIT/parity/no-target stops).
swdqtest:
	$(call topbench,tb_top_swdq,loop deep)

# v55 I2C master (open drain, repeated START, NACK, clock stretching, arbitration) and SPI
# modes 0-3, on swd_engine alone against behavioural targets.
i2cmastertest:
	iverilog -g2012 -o /tmp/tb_i2c_master.vvp -I src sim/tb_i2c_master.v src/swd_engine.v src/i2c_master.v src/jtag_seq.v src/swim_engine.v
	vvp -n /tmp/tb_i2c_master.vvp | tee /dev/stderr | grep -q '^PASS'

# v61 JTAG master: DAP_JTAG_Sequence sequences against a 2-TAP chain model (IDCODE, IR/BYPASS,
# 64-cycle sequences, a queue that runs dry), at half periods 2, 3 and 13, on swd_engine alone.
jtagtest:
	iverilog -g2012 -o /tmp/tb_jtag.vvp -I src sim/tb_jtag.v src/swd_engine.v src/i2c_master.v src/jtag_seq.v src/swim_engine.v
	vvp -n /tmp/tb_jtag.vvp | tee /dev/stderr | grep -q '^PASS'

# v62 SWIM master: entry + sync at two SWIM clock rates, SRST / ROTF / WOTF, NACK and retry, a chip
# that never answers, a slow-rising line, high speed, on swd_engine alone against a behavioural STM8.
swimtest:
	iverilog -g2012 -o /tmp/tb_swim.vvp -I src sim/tb_swim.v src/swd_engine.v src/i2c_master.v src/jtag_seq.v src/swim_engine.v
	vvp -n /tmp/tb_swim.vvp | tee /dev/stderr | grep -q '^PASS'

# spram_ring16 (v45 wrap-bit pointers) against the v44 ring, cycle for cycle, at 8 words deep.
ringtest:
	iverilog -g2012 -I src -o /tmp/$(PROJECT)_ring.vvp -s tb_spram_ring16 \
	         sim/tb_spram_ring16.v src/spram_ring16.v sim/spram_ring16_ref.v
	$(VVP) /tmp/$(PROJECT)_ring.vvp
	@rm -f /tmp/$(PROJECT)_ring.vvp

# The v50 LA change capture, sampler to bytes (tb_la_change.v), with the full ring and with a
# 16-word ring that must overflow under a max-rate burst.
LACHANGE_SRC = src/la48_sampler.v src/la_change_capture.v src/spram_ring48.v src/la_byte_pack.v src/lc_counter.v $(CELLS_SIM)
lachangetest:
	@set -e; for d in "" -DSMALL_RING; do \
	  echo "== tb_la_change $$d"; \
	  iverilog -g2012 -I src $$d -o /tmp/$(PROJECT)_lachange.vvp -s tb_la_change sim/tb_la_change.v $(LACHANGE_SRC); \
	  $(VVP) /tmp/$(PROJECT)_lachange.vvp; \
	done
	@rm -f /tmp/$(PROJECT)_lachange.vvp

# The v50 change capture through the whole top (SPI arm -> PSRAM), on both images.
toplachangetest:
	$(call topbench,tb_top_lachange,loop deep)

# Unit tests of tools/ice40_release.py (promote's stale-image refusal, the MANIFEST fields).
releasetooltest:
	python3 ../tools/test_ice40_release.py

# Run every self-checking testbench, then smoke-build the v1 gateware.
test: i2cemutest uarttest cdctest dactest dac8554test daclooptest dacpsramrdtest dacpsramskewtest dacpsramreplaytest adctest adctimingtest dualwrtest arbtest looptest loopframetest dsptest multiboot lapsramtest measuretest argtest steppertest topsmoketest topcapturetest topadct0test topboottest topspilinktest topdeeptest topmuxtest spitest swdqtest uart2test topemutest ringtest lachangetest toplachangetest jtagtest swimtest i2cmastertest check-protocol releasetooltest
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
	@echo "               PNR_STRICT=1 fails on a fallback seed, clk48 < MIN_CLK48_MHZ (50) or clk < MIN_CLK_MHZ (25.5)"
	@echo "  promote      Copy a HARDWARE-TESTED images build to release/ (HW_VERIFIED=\"...\")"
	@echo "               (refuses images older than the sources; records toolchain + flags)"
	@echo "  check-release  Fail unless release/ still matches the gateware sources"
	@echo "  release-images Install release/ for the firmware build (the release job)"
	@echo "  flash        Build + program SPI flash via iceprog (persistent)"
	@echo "  flash-ram    Build + load to FPGA SRAM only (volatile, faster)"
	@echo "  prog         Alias for 'flash'"
	@echo "  sim          Quick Verilog syntax check via iverilog"
	@echo "  clean        Remove generated artefacts"
	@echo "  help         Show this message"

# (labuftest removed in v24 with la_capture_buf.v — the on-FPGA I2C-LA trace buffer is gone.)

# The gateware source list (scripts: make -s print-src).
print-src:
	@echo $(SRC)
.PHONY: print-src topemutest
