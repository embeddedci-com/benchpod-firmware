# iCE40 seed history

This file keeps the per-version notes on the place-and-route seeds of the three gateware
images: loop (closed-loop DAC), deep (deep DAC replay) and emu (sensor emulation). They used to
live as comments in `ice40/Makefile` next to `SEED_LOOP`, `SEED_DEEP` and `SEED_EMU`. The
Makefile now keeps only a short note on the current pins; the notes below are moved verbatim
(with the leading `#` removed) and are not updated any more. They record why each pin was chosen
at the time, so some of their statements (fill, fallback order, "CI sweep not run yet",
"not hardware-tested") are no longer true. The current state is in the Makefile and in
`ice40/release/MANIFEST`.

When you re-pin a seed, add a short entry for the new version at the end of the matching
section here and update the current note in the Makefile.

## Single image and loop image, up to v33

Formerly the comment above `SEED = $(SEED_LOOP)`.

```text
Pinned PnR seed.  TIMING is not the constraint: after the single-clock collapse
(docs/adc-capture-cdc-review.md) the whole design runs on the single 24 MHz `clk`,
which closes with wide margin (~30-37 MHz depending on seed) vs a 24 MHz target;
clk48 only clocks the ÷2 + POR (~94 MHz).

PLACEMENT was formerly tight (~89% LCs => routability-seed-fragile, only seeds
4/6 placed).  The LUT-reduction pass in docs/ice40-lut-optimization.md brought the
v2 design down to ~79% LCs (4198/5280), and it now places on EVERY seed 1..10
(clk 34-39 MHz).  So this pin is no longer load-bearing for routability — it is
kept only for build determinism (seed 6 places at ~37 MHz).  --threads 1 keeps
PnR reproducible.  If a future netlist change tightens placement again, sweep
`--seed N` and re-pin one that places (and reclaim more LCs per that doc).
v16 DEEP ADC widened cap_count/adc_left 16->24-bit (and OP_CAPTURE 8->10 bytes),
nudging util to 80% LC (4241/5280).  v17 DEEP DAC REPLAY (dac_psram_reader + FIFO)
then pushed util to 89% LC (4722/5280) and made clk48 the binding, seed-fragile
constraint (only ~3 of 9 swept seeds closed clk48 @48 MHz).
LUT-REDUCTION PASS (docs): la_bank driver mux rewritten to one-hot decode + cmd_dispatch
fixed-arg collector folded into the uniform S_COLLECT path.  That brought util to 87%
LC (4622/5280).
CONCURRENT-PSRAM PASS (docs/psram-concurrent-arbiter): added psram_bus_arbiter so deep
DAC replay and ADC/LA capture time-multiplex the one quad bus (bus_req/gnt/busy/urgent
handshakes on dac_psram_reader + psram_dual_writer).  Util is now 88% LC (4664/5280).
In THIS netlist clk48 has no timed setup path (it only feeds SB_IO output regs + the ÷2
divider) — nextpnr reports no clk48 Fmax — so the binding constraint is PLACEMENT/routability
at 87% fill, not clk48 timing.
v18 CONCURRENT-PSRAM (docs/psram-concurrent-arbiter): the arbiter's cross-chip req/gnt/busy
nets between the reader FSM, writer FSM and arbiter first pushed util to 4642 LC and — more
than raw fill — created a routability hotspot that placed on NO seed (1..20 all failed).  The
fix was an LC/congestion-reduction pass: HALVE the psram_dual_writer staging FIFOs (AW 6->5,
64->32 B; CHUNK 32->16) — those FIFOs have an ASYNC read so on the LUT-RAM-less up5k they are
FF+mux logic, and halving them reclaimed the LC AND the routing.  Now 4619 LC (BELOW the v17
4622 baseline) and it places on seed 9 (the v17 pin) at clk ~32 MHz (PASS @24).  --threads 1
keeps PnR reproducible.  If a future change tightens placement, re-sweep `--seed N`.
v19 LC-REDUCTION PASS (ADC deep-ring dropped, dead code purged): util drops to 4414 LC (83%).
clk48 IS a timed domain here (the single-channel DAC sequencer + psram_dual_writer serializer run on it),
and closing it @48 MHz under clocks.py is seed-fragile: a fresh 1..12 sweep on this netlist
passed clk48@48 only on seeds 6 (49.1), 10 (48.5) and 12 (50.4) — seed 9 now closes clk48 at
only 47.55 (FAIL @48), so re-pinned to SEED 12 (best clk48 margin; clk ~34.6 MHz PASS @24).
NOTE: a seed change can HW-regress deep DAC PSRAM replay even when static timing passes
(docs/yosys-0.66-abc9-placement, docs/deep-dac-psram-replay) — re-verify hwe2e DAC replay on HW.
v21 CAPTURE-TIED DAC AUTO-STOP: the OP_SET_DAC_STOP_AFTER threshold reg + a down-counter in
top_v2 pushed util 4414 -> 4536 LC (83% -> 85%), and at that fill seed 12 no longer closes
clk48 (47.7 FAIL @48).  A fresh 1..20 sweep on the v21 netlist passed clk48@48 only on seeds
1 (49.25), 7 (49.00) and 11 (49.00) — re-pinned to SEED 1 (best clk48 margin; clk 33.83 MHz
PASS @24).  RE-VERIFY hwe2e deep DAC replay on HW after this seed change.
v24 LC/STABILITY PASS (docs/ice40-lut-optimization.md): removed the dead SB_WARMBOOT block +
OP_WARMBOOT decode, narrowed cmd_dispatch arg_count 16->12, uart_engine bit-period counters
24->18, and made dac_loop's tick timer a registered down-counter (dropping the last wide clk48
magnitude compare).  Loop image drops 4572 -> 4513 LC (86% -> 85%).  A fresh 1..12 sweep on the
v24 loop netlist: seed 9 (the old pin) no longer closes clk48 @48; seeds 4,5,6,7,8,10,11 do.
Re-pinned to SEED 11 (best+reproducible clk48 margin: 50.9 MHz worst path, clk 34.7 MHz; A/B
builds bit-identical).  SEED_DEEP 3 unchanged (clk48 55.0 MHz).
v29 SELECTABLE LOOP INPUT SOURCE (dac_loop src mux + sweep accumulator): loop image 4301 ->
4483 LC (81% -> 84%).  At that fill seed 11 still closes clk48 but its margin fell 53.75 ->
50.51 MHz, so the loop netlist was re-swept 1..12: seed 1 FAILS @48, everything else passes,
and seed 2 is the best (clk48 54.37 MHz, clk 34.48).  Re-pinned to SEED 2.  The binding path
is NOT in the loop — it is the pre-existing psram_run_48 -> dac dz -> wave_addr CEN chain, so
this is placement luck, which is exactly what the pin is for.  SEED_DEEP is untouched (the
deep netlist barely moved: 50.98 MHz at seed 4).
v30 INPUT CONDITIONER (affine map onto the curve index + over-range trip): loop image
4483 -> 4592 LC (84% -> 86%).  This fill is the tightest the loop image has run: of seeds
1..12 only 4,6,7,8,10 place at all (1,3,5,9,11,12 hit "unable to find legal placement"),
and of those seed 2's placement closes only 46.21 MHz.  Re-swept and RE-PINNED to SEED 8
(clk48 53.33 MHz, clk 32.86) — back to v29's margin.  Two features were CUT during this
pass to get here, both measured, not guessed: a programmable per-tick slew bound (~50 carry
cells for two 17-bit variable compares + a negate) and the non-modular subtract mode; see
dac_loop.v.  If anything else is added to this image, expect to re-sweep again — and note
that 6/12 seeds failing to place is the real signal that this image is out of room.
v33+: the default single image IS the loop netlist (identical DEFINES), so it no longer carries
a pin of its own.  That pin had drifted — seed 8 did not even place on v33, and CI's release
job built this image — while SEED_LOOP was the one re-swept and HW-verified.  Both now follow
SEED_LOOP / SEED_LOOP_FALLBACK (the history above is the loop image's).
```

## Why each image has a fallback list (2026-09-04)

Formerly the comment above `SEED_FALLBACK`.

```text
SEED FALLBACK (2026-09-04).  A single pinned seed is NOT portable across
toolchain BUILDS: the CI Linux OSS-CAD-Suite yosys 0.65 and a macOS Homebrew
yosys 0.65 map this design slightly differently (CI 4587 LC / local 4592 LC,
different promoted globals), and at 86% fill that is enough to flip a seed from
"places" to "unable to find legal placement".  Seed 8 places locally but FAILED
in CI on the v31 netlist.  So each PnR now tries the pinned seed FIRST and falls
back down this list until one both PLACES and CLOSES timing (nextpnr errors on a
timing miss, so a fallback build is never a timing-failing build).  Where the
pinned seed places, the bitstream is bit-identical to before — the fallback only
ever kicks in on a toolchain/netlist where the pin would have hard-failed, and it
prints a loud warning because that placement is NOT the HW-verified one.
Order is by measured clk48 margin on the local 1..16 sweep of the v31 loop netlist;
the trailing group are the seeds that do NOT place on the LOCAL netlist — they are
still worth a last-resort try because a different toolchain build maps the design
differently (that is the whole reason this list exists), and they cost nothing unless
every earlier seed has already failed.
```

## Loop image (`SEED_LOOP`), v30 to v56

```text
v30: clk48 53.33 MHz (re-swept after the input conditioner; only 5 of 12 seeds both place and close).
v32 EXACT CAPTURE DIVIDERS (constants only in la_psram_capture/adc_mcp33131; 4569 LC, fewer than
v31) re-rolled placement: seed 8 no longer places.  Swept 1..16: 5/8/11/12/15 do not place; the
rest close, best is 13 at clk48 55.61 MHz (clk 33.85).  Re-pinned SEED_LOOP 8 -> 13.
v33 DAC RESTART PERIOD FIX (+1 clk48 stage on the DAC start/stop CDC; 4599 LC) re-rolled it
again: seed 13 no longer places.  Swept 1..16: 2/4/5/8/12/13/14/15 do not place; best is 11 at
clk48 51.75 MHz (clk 35.26), then 3 (50.41) and 7 (50.17).  Re-pinned SEED_LOOP 13 -> 11.
v34 ROBUSTNESS PASS: the cmd_dispatch config-wire LC reduction took the loop image 4599 -> 4407 LC
(87% -> 83%), and the DAC engine's frame-end enable cone (the clk48 critical path the interim v34
sweeps found) now tests a registered flag.  Swept 1..16: ALL 16 place and close (v33: 8 did not
place); best is 9 at clk48 54.29 MHz (clk 33.32), then 11 (52.92) and 12 (52.74).  Re-pinned
SEED_LOOP 11 -> 9.
v35 CAPTURE TRIGGER + GPIO_GET (4407 -> 4442 LC, 83% -> 84%): the gated-start logic re-rolled
placement.  Swept 1..16: ALL 16 still place and close; seed 9 drops to clk48 52.73 MHz and the
best is 13 at 56.44 MHz (clk 32.86), then 1 (55.94) and 6 (55.04).  Re-pinned SEED_LOOP 9 -> 13.
RE-VERIFY deep DAC replay + a capture on HW after this seed change (see AGENTS.md).
v37 LC PASS (4442 -> 4351 LC, 84% -> 82%; clk-domain only, see src/top_v2.v).  Swept 1..16 on
the final v37 netlist: all place, 14/16 close @48 (16 and 5 miss).  Best is 14 at clk48
57.05 MHz (clk 30.64), then 9 (53.68) and 1 (53.53).  Re-pinned SEED_LOOP 13 -> 14.  The loop's
binding clk48 path is still dac_loop's clamp, which this pass left alone.
v38 CLOSED-LOOP CROSSING (4351 -> 4379 LC, 82%): the loop params became clk48 registers and
the clamp compare got its own state.  Swept 1..24: all place, 23/24 close AND reach
MIN_CLK48_MHZ (v37: 14/16 closed @48); seed 14 fell to 52.82, best is 5 at clk48 56.69 MHz
(clk 31.07), then 23 and 21 (55.04).  Re-pinned SEED_LOOP 14 -> 5.
v39 LOOP TELEMETRY CROSSING (4379 -> 4405 LC, 83%): seed 5 fell to clk48 50.93 MHz.  Swept
1..24: all place and all reach MIN_CLK48_MHZ; best is 8 at 57.87 MHz (clk 30.03), then 24
(57.74) and 20 (57.03).  Re-pinned SEED_LOOP 5 -> 8.
v40 LC PASS TIER 2 (4405 -> 4197 LC, 83% -> 79%): swept 1..24, all place and close, 23/24 reach
MIN_CLK48_MHZ.  Seed 8 stays: it ties the best (56.95 MHz, clk 33.53).
v41 LC PASS TIER 3 (4197 -> 3909 LC, 79% -> 74%; five counters in SB_MAC16 DSPs) + the
psram_pads retime.  Swept 1..24 on the final netlist: all place, 21/24 close @48, 12 reach
MIN_CLK48_MHZ; seed 8 no longer closes.  Best is 23 at clk48 55.69 MHz (clk 32.76).
Re-pinned SEED_LOOP 8 -> 23.
v42 (2026-09-25) STEPPER COUNTERS IN THE LAST DSP + DAC ENGINE CE CONES (3909 -> 3846 LC, 72%).
Moving the stepper's counters into a DSP took the loop netlist from 12/24 seeds reaching
MIN_CLK48_MHZ to 2/48: the binding path was rst48 -> the clock enables of the DAC engine's shift
register, whose ~35 datapath flops sat behind the FSM's rst/stop/start/running priority chain.
That margin was fragile, not lost: the same netlist with only the version constant changed
reached 15/24.  Fixed in the RTL (dac8551_engine.v: the frame datapath in its own block, the
`sleft <= 1` compare registered).  Swept 1..64 in BOTH toolchains (synth/seed_sweep.sh locally,
the ice40-seed-sweep workflow for CI's OSS CAD Suite): 61/64 local and 62/64 CI reach
MIN_CLK48_MHZ.  Pinned SEED_LOOP 23 -> 52, the best seed in both (clk48 57.47 local / 58.31 CI,
clk 33.30, clk -> clk48 10.19 ns), so CI can run with PNR_STRICT=1.
v43 (2026-09-25) WRITER HONOURS bus_own + LOOP TELEMETRY SNAPSHOT (3914 LC, 74%; the snapshot is
+67 LC).  The snapshot's first form put a wait-counter compare in ~35 clock enables and bound
the loop's clk48 on a third of the seeds (45/64); a registered tlm_ok flag took it to 57/64
local, 51/64 CI.  Pinned SEED_LOOP 52 -> 31, good in both (clk48 55.53 local / 56.86 CI,
clk -> clk48 10.38 / 11.47 ns).
v44 (2026-09-28) SPI MASTER in swd_engine (+~200 LC: 4108 LC, 78%).  Local sweep 1..64: 33/64
reach MIN_CLK48_MHZ (24 slow, 7 fail at 48 MHz).  The binding path on the failing seeds is the
untouched clk48 dac8551_engine div_cnt chain spread across the die, i.e. fill, not new logic.
CI sweep (OSS CAD Suite): 45/64.  51, the best local seed, misses 48 MHz in CI (47.50), so
pinned SEED_LOOP 31 -> 58, good in both (clk48 52.96 local / 52.59 CI, clk -> clk48 11.17 /
11.86 ns).
v45 (SWD transfer queue + LC pass: SPI on the SWD sequencer, cmd_dispatch, i2c_target,
uart_engine/fifo, LA ring, ADC; 3864 LC, 73%, below v43's 3914).  Swept 1..64 in both
toolchains: 61/64 local, 46/64 CI (v45 before the LC pass: 22/64 local).  Pinned SEED_LOOP
58 -> 26 (clk48 56.70 local / 55.48 CI, clk -> clk48 <= 11.38 ns).
v46 (CORR-2 ring overflow clear + version; 3895 LC, 74%): swept 1..64 locally: 57/64 reach
MIN_CLK48_MHZ (2 miss 48 MHz).  Seed 26 still closes (clk48 54.69, clk 32.69, clk -> clk48
10.19 ns) and stays pinned.  CI sweep not run yet.
v47 (GW-1, 3886 LC, 74%): seed 26 still closes (clk48 56.32, clk 31.22, clk -> clk48 11.05 ns);
kept, no sweep.
v47 + cell_gearbox48 (ARCH-GW-1: the writer's and reader's clk -> clk48 cell latch as one module;
yosys equiv proves both modules unchanged, same flip-flops, but abc9 maps the netlist slightly
differently: 3893 LC): seed 26 still closes (clk48 55.05, clk 29.73, clk -> clk48 10.38 ns).
v48 (UART2, the TX-only second UART; 4066 LC, 77%): swept 1..64 in both toolchains: 48/64 local,
55/64 CI reach MIN_CLK48_MHZ.  Pinned SEED_LOOP 26 -> 17, the best in both (clk48 54.84 local /
55.98 CI, clk -> clk48 10.73 / 9.84 ns).
v49 (quad DAC: OP_DAC_CTRL stream control byte + one-shot static frame; 4133 LC, 78%): seed 17
first fell to clk48 43.55 MHz, bound by an OR of three flags in the DAC engine's enable; with that
back to one flop (run_i) and wave_addr out of the FSM's priority chain it closes at 52.48.
Swept 1..64 locally: 44/64 reach MIN_CLK48_MHZ.  Pinned SEED_LOOP 17 -> 57 (clk48 55.31, clk
30.75, clk -> clk48 11.42 ns).  CI sweep not run yet.  RE-VERIFY on hardware.
v50 (LA change capture + pre-trigger; 4777 LC, 90%): seed 57 failed clk48 (47.77 MHz).  Swept
1..48 locally: 29/48 reach MIN_CLK48_MHZ.  Pinned SEED_LOOP 57 -> 31 (clk48 54.52, clk 29.40,
clk -> clk48 11.42 ns), the best clk48 with a clk -> clk48 well inside MAX_X48_NS.  CI sweep not
run yet.  RE-VERIFY on hardware.
CI sweep (Linux yosys) of the same netlist: seed 31 only reaches 44.99 MHz there.  Seed 40 is the
best seed good in BOTH toolchains (clk48 53.98 local / 53.98 CI, clk -> clk48 13.19 / 13.05 ns).
Pinned SEED_LOOP 31 -> 40.  RE-VERIFY on hardware.
v51 (LC / clk48 headroom pass; 4579 LC, 87%): the v50 seeds that missed 48 MHz were bound by
dac8551_engine's sidx/sleft enable cone, now one registered flag (fe).  After the version bump
abc9 also mapped dac_loop's vmax compare to a LUT ripple that capped clk48 at 49.52 MHz on every
seed; the compares are now the sign of a subtract.  Swept 1..48 in both toolchains: 36/48 local,
37/48 CI reach MIN_CLK48_MHZ (v50: 29/48 local), 29 in both.  Pinned SEED_LOOP 40 -> 7, the best
in both (clk48 54.40 local / 58.30 CI, clk 28.44, clk -> clk48 12.76 / 12.14 ns, clk48 -> clk
8.24 / 9.15 ns).  RE-VERIFY on hardware.
v52 (review fixes: ADC t0 restart, SPI slave, UART framing; 4641 LC, 88%): seed 7 fell to
52.96 MHz.  Swept 1..48 locally: 32/48 reach MIN_CLK48_MHZ.  Pinned SEED_LOOP 7 -> 35 (clk48 56.69,
clk 29.13, clk -> clk48 13.59 ns, clk48 -> clk 9.43 ns).  CI sweep not run yet.  RE-VERIFY on hardware.
v53 (ADC-urgent pick in psram_dual_writer): swept 1..64 in both toolchains (CI run 42421449089).
Pinned SEED_LOOP 35 -> 51, the seed with the highest clk48 in the slower toolchain (53.75 CI /
54.11 local MHz; clk -> clk48 13.62 / 12.46 ns).
v54 (the I2C target and UART2 moved to the emulation image; 3998 LC, 75%): swept 1..32 locally:
all 32 reach MIN_CLK48_MHZ.  Pinned SEED_LOOP 51 -> 25 (clk48 61.18, clk 28.07, clk -> clk48
10.08 ns, clk48 -> clk 5.58 ns).  CI sweep not run yet.  RE-VERIFY on hardware.
v56 (UART parity + stop bits, status flags cleared after the read; 4179 LC): seed 25 failed the
CI gate (clk -> clk48 15.20 ns). Swept 1..32 in both toolchains: re-pinned SEED_LOOP 25 -> 11
(local clk48 56.95 MHz, clk -> clk48 9.84 ns; CI 56.35 MHz, 10.12 ns). HW-verified 2026-11-16.
```

`SEED_LOOP_FALLBACK`:

```text
v54: the next local seeds by clk48, all within MAX_X48_NS (the CI sweep should re-order this).
```

## Deep image (`SEED_DEEP`), v27 to v58

```text
v27 CO-TRIGGER + PIPELINED FIFO-EMPTY: the co-trigger topology re-rolled placement and seed 5 (at
clk48 51.26 MHz static) HW-REGRESSED deep replay — the binding clk48 path was the pre-existing
6-LUT reader<->DAC FIFO-empty chain (rgray==wgray_r2 -> fetch/RCLKE/CE), whose scattered placement
was the whole seed-fragility.  Pipelining that compare into a registered `rd_empty`
(dac_psram_reader.v, bit-exact, no added latency) took the reader OFF the critical path: the deep
image now closes clk48 on ALL seeds (49.35..54.59) instead of 5/8, so it is no longer seed-fragile.
Re-swept 1..10: seed 9 is best (clk48 54.59 MHz, clk 35.25 MHz).
v31 DAC SOURCE-SELECT FIX: the two extra decode terms in cmd_dispatch re-rolled placement, and on
seed 4 the deep image dropped to clk48 48.77 MHz — still green at 48.00 but only 0.77 MHz of margin
on the exact clock+image where deep replay has historically died SILENTLY with green STA (see the
v28 note above).  Re-swept 1..8 on the new netlist: 8 is best at clk48 51.13 MHz (A/B builds
bit-identical), i.e. MORE margin than the pre-fix 49.75.  Re-pinned SEED_DEEP 4 -> 8.
v31: clk48 51.13 MHz, clk 34.69 MHz (re-swept after the source-select fix).
v32 EXACT CAPTURE DIVIDERS re-rolled placement again: seed 8 fell to clk48 48.19 MHz (0.19 MHz
of margin).  Swept 1..16: 11 is best at clk48 50.44 MHz (clk 35.64), then 1 at 49.75; 4/5/6 now
MISS 48 MHz, and most others sit at 48.19.  Re-pinned SEED_DEEP 8 -> 11 — re-verify deep replay
on hardware (TestHW_V2_DeepReplay) after any change here.
v33 DAC RESTART PERIOD FIX: seed 11 still closes but at clk48 48.77 MHz (0.77 MHz of margin).
Swept 1..16: 13 is best at clk48 51.13 MHz (clk 34.76), then 4 (49.49) and 16 (49.35); 1/3/8/14
MISS 48 MHz.  Re-pinned SEED_DEEP 11 -> 13 — re-verify deep replay on hardware.
v34 ROBUSTNESS PASS (4155 LC): the interim v34 netlists fell to a best of 49.80 / 48.74 MHz — the
DAC engine's frame-end enable cone — until that cone was pipelined (registered `last`, CDC pulse
off a flop).  Final sweep 1..16: 13/16 close, 6 and 15 best at clk48 51.13 MHz; 15 has the better
clk (32.47 vs 31.23).  Re-pinned SEED_DEEP 13 -> 15 — re-verify deep replay on hardware.
v35 CAPTURE TRIGGER + GPIO_GET (4155 -> 4170 LC): the deep netlist's clk48 is the tightest thing
in this design, and the trigger re-rolled its placement — seed 15 now MISSES 48 MHz (46.39).
Swept 1..16: 9/16 close (v34: 13/16); 13 and 2 are both best at clk48 51.13 MHz — the same margin
as v34 — and 13 has much the better clk (34.53 vs 31.26), then 7 (49.75) and 14 (49.27).
Re-pinned SEED_DEEP 15 -> 13 — RE-VERIFY deep replay on hardware (TestHW_V2_DeepReplay).
NB an earlier v35 draft also kept an LA PRE-TRIGGER window in the SPRAM ring (4280 LC): at that
fill only 7/16 seeds closed and the BEST was 49.75 MHz, under MIN_CLK48_MHZ.  That is why the
pre-trigger was dropped rather than shipped (see the v35 note in src/top_v2.v).
v36 LA13/LA14 READBACK (src/top_v2.v): widening la_sample/la_levels + la_psram_capture's N
12->14 re-rolled placement and seed 13 now MISSES 48 MHz (47.48).  Swept 1..16: only 9/16 close
(v35: 13/16); 14 is best at clk48 51.13 MHz (clk 34.05), then 5 (49.36) and 8 (49.35).
Re-pinned SEED_DEEP 13 -> 14 — RE-VERIFY deep replay on hardware (TestHW_V2_DeepReplay).
v37 LC PASS (4187 -> 4088 LC, 79% -> 77%; no clk48 logic changed): the re-rolled placement
is WORSE for clk48 here.  Swept 1..40 on the final v37 netlist: all place, only 18/40 close @48
and only seed 12 reaches MIN_CLK48_MHZ (50.07 MHz, clk 33.65), then 18 (49.80) and 11 (49.49).
v36 was 9/16 closing, best 51.13.  Re-pinned SEED_DEEP 14 -> 12 — RE-VERIFY deep replay on
hardware.  The deep image's binding clk48 path is dac_psram_reader's FIFO read pointer; that,
not more seeds, is the next thing to fix here.
v38 (cmd_dispatch lost the loop registers; no deep logic changed) re-rolled it again, the
other way: swept 1..40, all place, 32/40 close @48 and 13 reach MIN_CLK48_MHZ (v37: 1/40).
Seed 12 fell to 48.77; twelve seeds tie at clk48 51.13 MHz, and 3 has the best clk (32.69).
Re-pinned SEED_DEEP 12 -> 3 — RE-VERIFY deep replay on hardware.
v39 (only the version constant changed in the deep netlist): swept 1..24, 11/24 close @48 and
ONLY seed 3 reaches MIN_CLK48_MHZ (51.13 MHz, clk 32.96).  Kept.  The deep image's clk48 margin
is at the mercy of placement; its binding path (dac_psram_reader) is the next thing to fix.
v40 LC PASS TIER 2 (4092 -> 3872 LC, 77% -> 73%): seed 3 now MISSES 48 MHz (46.39).  Swept
1..32: all place, 26/32 close @48 (v39: 11/24), 5 reach MIN_CLK48_MHZ, all at exactly
51.13 MHz (a fixed path caps the deep image there); 23 has the best clk (32.23).
Re-pinned SEED_DEEP 3 -> 23 — RE-VERIFY deep replay on hardware.
v41: the deep image's two clk48 bottlenecks are FIXED rather than placed around: the reader
FIFO's `empty` is a registered flag (off the BRAM read-enable path) and psram_pads retimes the
PSRAM outputs so the SCLK DDR pad has no half-cycle path through the mux.  Swept 1..24 on the
final netlist (3615 LC, 68%): ALL 24 close @48 and 22/24 reach MIN_CLK48_MHZ (v40: 5/32), best
14 at 55.35 MHz (clk 35.42).  Re-pinned SEED_DEEP 23 -> 14 — RE-VERIFY deep replay on hardware.
v42: swept 1..64 in both toolchains on the final netlist (3568 LC, 68%): 62/64 local and 62/64
CI reach MIN_CLK48_MHZ.  Pinned SEED_DEEP 14 -> 34, the best seed in both (clk48 55.35 local /
53.73 CI, clk -> clk48 12.76 ns).  RE-VERIFY deep replay on hardware.
v43: swept 1..64 in both toolchains: 60/64 local, 58/64 CI.  Pinned SEED_DEEP 34 -> 7 (clk48
55.27 local / 54.04 CI, clk -> clk48 13.11 / 12.72 ns).  RE-VERIFY deep replay on hardware.
v44 (SPI master, 3787 LC, 71%): swept 1..64 in both toolchains: 60/64 local, 64/64 CI.  Pinned
SEED_DEEP 7 -> 6 (clk48 53.73 local / 51.93 CI, clk -> clk48 <= 12.72 ns in both).
v45 after the LC pass (3549 LC, 67%): 63/64 local, 57/64 CI.  Pinned SEED_DEEP 6 -> 25 (clk48
54.59 local / 55.27 CI, clk -> clk48 <= 13.29 ns).
v46 (CORR-1: reader honours bus_own with burst commit/abort, ne_r clear off the subtract;
3579 LC, 68%): swept 1..64 locally: ALL 64 reach MIN_CLK48_MHZ (v45: 63/64), eight tie at
59.26 MHz.  Pinned SEED_DEEP 25 -> 50 (clk48 59.26, clk 32.34, clk -> clk48 12.72 ns; 25 ties
on clk48 but its clk is 27.94).  CI sweep not run yet.  RE-VERIFY deep replay on hardware.
v47 (GW-1: every arm flushes the writer's staging FIFOs and the LA ring; 3590 LC, 68%): seed 50
fell to clk48 49.61 MHz.  Swept 1..64 locally: 60/64 reach MIN_CLK48_MHZ, fourteen tie at
56.95 MHz.  Pinned SEED_DEEP 50 -> 57 (clk48 56.95, clk 34.51, clk -> clk48 12.82 ns; it is in
the cross-toolchain fallback list).  CI sweep not run yet.  RE-VERIFY deep replay on hardware.
v47 + cell_gearbox48 (ARCH-GW-1, no logic change, see SEED_LOOP; 3610 LC): seed 57 still closes
(clk48 57.67, clk 32.55, clk -> clk48 14.41 ns).  A new bitstream: RE-VERIFY on hardware.
v48 (UART2; 3780 LC, 71%): seed 57 FAILED clk48 (45.50 MHz).  Swept 1..64 in both toolchains:
60/64 local, 64/64 CI reach MIN_CLK48_MHZ.  Pinned SEED_DEEP 57 -> 13 (clk48 59.54 local /
59.26 CI, clk -> clk48 12.79 / 13.91 ns).  RE-VERIFY deep replay on hardware.
v49 (quad DAC; 3842 LC, 73%): seed 13 still closes clk48 (57.33) but its clk -> clk48 paths
take 15.39 ns (MAX_X48_NS=15; the reader's gearbox, untouched).  Swept 1..64 locally: 55/64
reach MIN_CLK48_MHZ, 38 of them within MAX_X48_NS.  Pinned SEED_DEEP 13 -> 48 (clk48 59.54, clk
32.30, clk -> clk48 12.80 ns).  CI sweep not run yet.  RE-VERIFY deep replay on hardware.
v50 (LA change capture + pre-trigger; 4445 LC, 84%): seed 48 closes (clk48 55.73) but its
clk -> clk48 paths take 14.64 ns, near MAX_X48_NS.  Swept 1..32 locally.  Pinned SEED_DEEP 48 -> 31
(clk48 57.07, clk 27.65, clk -> clk48 12.80 ns).  The CI sweep closes it too (55.24 MHz, 13.44 ns).
v51 (4306 LC, 82%; no deep logic but the LA path and the DAC engine changed): seed 31 misses
MIN_CLK48_MHZ.  Swept 1..48 in both toolchains: 38/48 local, 45/48 CI, 35 in both.  Pinned
SEED_DEEP 31 -> 16 (clk48 55.27 local / 55.35 CI, clk 29.03, clk -> clk48 13.95 / 13.99 ns,
clk48 -> clk 4.39 / 5.68 ns; 9 ties but sits at 14.99 ns clk -> clk48).  RE-VERIFY deep replay.
v52 (4287 LC, 81%): seed 16 fell to 48.53 MHz.  Swept 1..48 locally: 40/48 reach MIN_CLK48_MHZ,
nine tie at 56.95.  Pinned SEED_DEEP 16 -> 41, the tie with the shortest clk -> clk48 paths
(13.92 ns; clk 27.46, clk48 -> clk 6.22 ns).  CI then failed seed 41: clk -> clk48 18.40 ns.
Swept 1..64 in both (CI run 42391635281): 51/64 local; 11 seeds keep clk48 >= 50 MHz and
clk -> clk48 <= MAX_X48_NS in both.  Pinned SEED_DEEP 41 -> 5, the one with the most
clk -> clk48 margin (13.88 CI / 14.09 local ns; clk48 57.07 CI / 55.60 local MHz).
v53: seed 5 missed timing locally (45.03 MHz) and passed MAX_X48_NS in neither. Swept 1..64 in
both (CI run 42421449089): pinned SEED_DEEP 5 -> 37 (57.61 local / 59.54 CI MHz; clk -> clk48
13.92 / 13.83 ns).
v54 (no I2C target, no UART2; 3663 LC, 69%): seed 37 fell to 51.58 MHz.  Swept 1..32 locally: all 32
reach MIN_CLK48_MHZ.  Pinned SEED_DEEP 37 -> 11 (clk48 59.54, clk 29.31, clk -> clk48 13.34 ns,
clk48 -> clk 5.58 ns; 14 ties on clk48 at 14.67 ns).  CI sweep not run yet.  RE-VERIFY deep replay.
v55 (I2C master + SPI modes 1/2 in the engine; 3839 LC): CI failed seed 11 (clk -> clk48 15.22 ns,
14.57 locally).  Swept 1..32 in both (CI run 43497781121): 30/32 local, 32/32 CI reach
MIN_CLK48_MHZ.  Pinned SEED_DEEP 11 -> 29, the most clk -> clk48 margin in the worse toolchain
(13.32 local / 13.59 CI ns; clk48 54.27 / 53.74 MHz).  RE-VERIFY deep replay on hardware.
v56: seed 29 reaches clk -> clk48 15.52 ns in CI. Re-pinned SEED_DEEP 29 -> 16 (local clk48 56.95
MHz, 12.80 ns; CI 57.33 MHz, 13.81 ns).
v58 (UART DE): seed 16 reaches clk -> clk48 15.18 ns locally (MAX_X48_NS 15). Swept 1..64
locally: 60/64 reach MIN_CLK48_MHZ. Re-pinned SEED_DEEP 16 -> 34 (clk48 55.60 MHz, clk -> clk48
12.62 ns, the most margin). CI sweep rate-limited (setup-oss-cad-suite), not compared yet.
```

`SEED_DEEP_FALLBACK`:

```text
Good in both toolchains, ordered by the lower of the two clk48 results (v51 sweeps; v48 before).  A timing
miss makes nextpnr error, so a failing seed here is only a wasted attempt; the list exists for
the same cross-toolchain reason as SEED_FALLBACK above.  v49: 14 dropped (fails locally), 13
added (the v48 pin), and the v49 local seeds within MAX_X48_NS first.
```

## Emulation image (`SEED_EMU`), v54

```text
v54: the emulation image (USE_EMULATION; 4 I2C targets, write log, SPI target, FIFO, UART2 TX+RX;
4891 LC, 92%, 30/30 block RAM, 4/4 SPRAM).  Its binding clock is clk (24 MHz), not clk48: the
image has no clk48 logic beyond what every image has.  Swept 1..42 locally: ALL 42 place, close
and reach MIN_CLK48_MHZ (clk48 54.2..66.4 MHz), clk 25.1..28.8 MHz.  Pinned SEED_EMU 27 (clk48
66.37, clk 28.05, clk -> clk48 10.73 ns, clk48 -> clk 6.22 ns).  The fallbacks are the next seeds
with clk >= 27.5 MHz.  CI sweep not run yet.  Not hardware-tested.
```

## v59

No seed changed. The images promoted on 2026-11-19 (`ice40/release/MANIFEST`) were
hardware-verified on that day with these results from the local toolchain:

| image | seed | clk48 MHz | clk MHz | clk -> clk48 ns | clk48 -> clk ns | logic cells |
|-------|------|-----------|---------|-----------------|-----------------|-------------|
| loop  | 11   | 56.07     | 26.75   | 11.17           | 6.33            | 4222/5280 (80%) |
| deep  | 34   | 59.54     | 27.72   | 14.53           | 4.39            | 3932/5280 (74%) |
| emu   | 27   | 53.26     | 26.08   | 11.82           | 6.87            | 5151/5280 (97.6%) |

The emu image also uses all block RAM (30/30) and all SPRAM (4/4).

## v60

All three seeds re-pinned on the v60 netlist (the 2026-11 audit's gateware items: UART_CONFIG2,
START_DAC_PSRAM and the control-loop opcodes per image, cmd_dispatch's per-image command groups,
one i2c_emu file). On the v59 pins the v60 netlist missed gates (emu seed 27: clk 24.76 MHz,
clk -> clk48 15.60 ns). Swept 1..64 in the local toolchain (Homebrew yosys 0.65, nextpnr 0.10) and
in CI's (OSS CAD Suite 2026-05-13, yosys 0.65+7; the ice40-seed-sweep workflow, which now sweeps
the emulation image too). Seeds passing every gate (MIN_CLK48_MHZ 50, MIN_CLK_MHZ 25.5,
MAX_X48_NS 15, MAX_X24_NS 15):

| image | local | CI |
|-------|-------|----|
| loop  | 54/64 | 53/64 |
| deep  | 46/64 | 51/64 |
| emu   | 50/64 | 28/64 |

Each pin is the seed whose worst relative margin over the four gates, in both toolchains, is the
largest; the fallbacks are the next ones in that order. Local results on the pins:

| image | seed | clk48 MHz | clk MHz | clk -> clk48 ns | clk48 -> clk ns | logic cells |
|-------|------|-----------|---------|-----------------|-----------------|-------------|
| loop  | 2    | 55.86     | 28.35   | 10.97           | 6.33            | 4217/5280 (80%) |
| deep  | 24   | 59.26     | 27.45   | 13.92           | 5.62            | 3939/5280 (75%) |
| emu   | 60   | 61.41     | 27.37   | 11.17           | 6.66            | 5155/5280 (97.6%) |

CI on the same seeds: loop 56.02 / 29.66 / 10.73 / 6.98, deep 57.67 / 28.72 / 13.27 / 6.85,
emu 56.06 / 28.36 / 11.78 / 8.17. The first emu pick, seed 57 (the best local seed), fails timing
in CI's toolchain (clk48 43.83 MHz), so emu is pinned from the seeds good in both. Hardware-verified on benchpod-30a9f1 on 2026-11-20 (`ice40/release/MANIFEST`).

## v65

The emulation image split: the emulation image drops the bus-master engine (SWD, the SPI master,
the I2C master) and the deep image drops SWIM, on top of v64's registered LA pads. Swept 1..64 in
the local toolchain (Homebrew yosys 0.65, nextpnr 0.10) and in CI's (yosys 0.65+7, run
45485242806). Seeds passing every gate:

| image | local | CI | both |
|-------|-------|----|------|
| loop  | 50/64 | 50/64 | 38 |
| deep  | 36/64 | 44/64 | 24 |
| emu   | 62/64 | 63/64 | 61 |

Each pin is the seed whose worst relative margin over the four gates, in both toolchains, is the
largest. Local results on the pins (CI in brackets):

| image | seed | clk48 MHz | clk MHz | clk -> clk48 ns | clk48 -> clk ns | logic cells |
|-------|------|-----------|---------|-----------------|-----------------|-------------|
| loop  | 12   | 54.96 (56.53) | 28.21 (29.36) | 11.31 (12.03) | 5.58 (6.85) | 4306/5280 (82%) |
| deep  | 1    | 55.27 (55.60) | 27.35 (29.36) | 13.22 (13.11) | 6.66 (4.39) | 4036/5280 (76%) |
| emu   | 26   | 68.40 (68.16) | 28.90 (29.17) | 11.92 (9.98)  | 4.39 (5.03) | 4418/5280 (84%) |

The emulation image went from 98.5% LC (v63) and the fewest good seeds to the most.

## v66

The 19.2 MHz DAC SCLK: `dac8551_engine` puts out half pairs and the new `dac_pads` drives SCLK and
DIN through DDR SB_IOs. Swept 1..64 in the local toolchain (Homebrew yosys 0.65, nextpnr 0.10)
and in CI's (yosys 0.65+7, run 45512812320). Seeds passing every gate:

| image | local | CI | both |
|-------|-------|----|------|
| loop  | 58/64 | 48/64 | 42 |
| deep  | 49/64 | 39/64 | 28 |
| emu   | 59/64 | 63/64 | 57 |

Each pin is the seed whose worst relative margin over the four gates, in both toolchains, is the
largest. Local results on the pins (CI in brackets):

| image | seed | clk48 MHz | clk MHz | clk -> clk48 ns | clk48 -> clk ns |
|-------|------|-----------|---------|-----------------|-----------------|
| loop  | 36   | 53.80 (55.87) | 28.82 (28.24) | 11.27 (12.57) | 9.31 (8.27) |
| deep  | 17   | 57.20 (54.27) | 28.00 (28.96) | 13.16 (13.27) | 6.12 (5.58) |
| emu   | 49   | 64.21 (62.05) | 29.30 (28.63) | 10.19 (10.62) | 5.68 (6.33) |

A first v66 netlist (SCLK 2 halves high / 3 low, swept the same way) is superseded: the bench
read its SCLK high at the 20 ns limit, and the final netlist has 3 halves high / 2 low.
