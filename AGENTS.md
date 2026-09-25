# Notes for agents working on this firmware

## The hardware is NOT broken — assume firmware/gateware first

The BenchPod boards have been bring-up-tested and reworked many times and are
**known-good**. When a capture / PSRAM / serial-bus symptom appears (e.g. "the ADC
reads garbage / 0x5555", "capture returns stale data"), **do not blame the hardware
or reach for a reflow first.** Rule out firmware and gateware with a *test*, not a
guess. We once spent a very long time chasing a "dead ADC / cracked joint" that was
actually a single iCE40 timing bug.

## iCE40 timing is the usual suspect

Two classes of iCE40 timing bug have bitten this project; check for both on any
gateware change:

- **Clock-domain crossings.** Multi-bit buses crossing 24↔48 MHz mis-latch on some
  placements and pass on others. The capture datapath is now collapsed onto a
  single 24 MHz clock specifically to eliminate this — keep it that way.
  For control that must cross (a start pulse plus its arguments), use
  `ice40/src/cdc_pulse_payload.v` — don't hand-roll a sync chain. The rule it
  enforces: **the pulse that latches a crossed value must not be the pulse that
  consumes it.** Nonblocking assignment hands a consumer on that same edge the
  *previous* value. That was the v33 DAC bug: START_DAC's period was copied into
  clk48 on the engine's own start pulse, so every start ran its first waveform
  pass at the previous waveform's length.
- **Serial-output setup.** Anything you clock out (SCLK + data) must present the
  data a half-clock *ahead* of the sampling edge. An SB_IO-registered data output
  that changes on the *same* edge as its clock has **zero setup** and is
  placement-fragile — it "works" on one build and silently fails on the next. This
  was the PSRAM-write root cause (`ice40/src/psram_writer.v`). The same goes for
  frame signals: the DAC8551 engine used to raise SYNC on the same edge as the 24th
  SCLK fall (datasheet t7 ≥ 0 ns, so skew decided it). Check a new serial peripheral
  against its datasheet timing table, and assert it on the pins in sim like
  `tb_dac8551` does (t5–t9).

Two more that RTL simulation cannot catch:

- **Explicit DSP primitives.** yosys 0.65's `ice40_dsp` pass (run by `synth_ice40 -dsp`)
  treats every `SB_MAC16` as a multiplier and rewrote the `dsp_counter` cells (clock tied to
  0, load inputs zeroed) while every bench passed. The Makefile hides explicit `SB_MAC16`
  cells from that step (`synth/sb_mac16_keep.v`) and `synth/check_dsp.py` checks the netlist.
  Instantiate new DSP cells through `dsp_counter`, or extend that check.
- **Half-cycle pad paths.** The SCLK pad is DDR: it captures `D_OUT_1` on the clk48 falling
  edge, so logic in front of it has half a period. That capped the deep image near 51 MHz
  until v41 (`src/psram_pads.v` retimes every PSRAM output by one clk48 and feeds the pad
  from flops; the reader samples one clk48 later, `PAD_PIPE=1`). Keep logic out of any
  `D_OUT_1` path, and run reader changes through `tb_dac_psram_skew`, which now drives the
  real pads.

Note the PnR seed in `ice40/Makefile` is a *placement* pin, not a timing crutch.
The capture datapath's single 24 MHz `clk` closes with wide margin (~33-35 MHz vs a
24 MHz target), so **a `clk` timing failure is a real bug, not a reason to try another
seed**. Two things are genuinely seed-sensitive and are handled by the Makefile, not
by you re-rolling seeds by hand:

- **Placement.** The loop image runs at ~74% LC (3909 at v41; 4197 at v40; 4405 at v39; 4442 at
  v35/v36). At the previous ~87%, roughly half of all seeds failed to place
  at all, and it will get there again if features are added without reclaiming LC.
  Each PnR therefore tries the pinned seed first and falls back down
  `SEED_FALLBACK` until one both places and closes timing, printing a loud warning
  when a fallback is used (that bitstream is *not* the placement verified on hardware).
  Fallbacks firing routinely means the image is out of room — do an LC-reduction pass,
  don't just extend the seed list.
- **`clk48`.** The DAC8551 sequencer and the PSRAM serializer run on the 48 MHz domain,
  whose margin is thin and placement-dependent; a seed change there has historically
  regressed deep DAC PSRAM replay on hardware even with green static timing.
  Re-verify on hardware after any seed change.

A pinned seed is also **not portable across toolchain builds**: the CI Linux
OSS-CAD-Suite and a local macOS Homebrew yosys 0.65 emit slightly different netlists
from identical sources, which is enough at this fill to flip a seed between "places"
and "unable to find legal placement". That divergence is why the fallback list exists.

Gates, so you don't have to read PnR logs to notice:

- `MIN_CLK48_MHZ` (default 50) warns when the routed clk48 Fmax is below it.
  nextpnr itself only fails below 48.00 MHz.
- `PNR_STRICT=1` turns that warning and a fallback seed into build failures.
  CI passes it from the `ICE40_PNR_STRICT` repo variable (off until CI's toolchain
  places the pinned seeds).
- After any gateware change, sweep seeds for both images (`_loop`, `_deep`) and
  re-pin for the best clk48 margin.
- Pods run the two images from `make images`, which the release builds. `make`
  alone builds one image, and it now shares the loop seed.

## Releases embed hardware-tested gateware, not a CI build

The CI toolchain places the design differently from a local build, so the same sources give
a different bitstream with different clk48 margin (v40: loop 56.95 MHz locally, 48.84 MHz in
CI). The firmware release job therefore never builds gateware: it embeds the images committed
in `ice40/release/` (`make -C ice40 release-images`), and fails if they no longer match the
gateware sources. After a gateware change:

1. `make -C ice40 images`, build the firmware, flash it, run the hardware suite on it.
2. `make -C ice40 promote HW_VERIFIED="<which test passed, on which pod>"`: copies that exact
   build to `ice40/release/` and writes the MANIFEST (source hash, sha256s, seeds, clk48,
   toolchain, the hardware record).
3. Commit `ice40/release/` with the sources. `make -C ice40 check-release` (a CI warning until
   then) passes again.

## Test the second run, not just the first

Both v32's divider off-by-one and v33's DAC restart bug passed benches that start once from
reset and check data. When you add or change anything that is armed with parameters (a
capture, a DAC mode, the loop), make the bench **re-arm it with different parameters** and
check the second run uses them (`tb_top_capture` DAC restart / capture re-arm / MEASURE / loop,
`tb_dac_psram_replay` re-arm). Check timing, not only values: tb_top_capture's sample-period
monitor and tb_dac8551's clk-per-sample check. On hardware, the host-clock tests
(`test_*_sample_rate_matches_the_host_clock` in embeddedci-python) are the independent reference.

This pays off. The first run of the deep-replay re-arm pass found the same bug in
`dac_psram_reader`: it reloaded its address only while `run` was low, and `run` rises on the
edge that writes the new base. So every re-arm's first burst read the previous replay's region.

Benches report through `sim/run_vvp.sh`. A bench must print a line *starting with* `PASS` (or
`ALL TESTS PASSED`) and no line starting with `FAIL`, or its make target fails. `vvp` itself
exits 0 whatever a bench prints, so before v34 `make test` in CI only caught compile errors.
Report expected-failure rows indented, as `tb_adc_timing` does.

## Built-in diagnostics — use these before touching a scope

- `psram-selftest` (console): 3 layers — STM32↔PSRAM, iCE40→PSRAM write, and
  per-line iCE40→PSRAM `/CS`/`SCLK`/`IO0-3` reach. Runs at boot; result is on the
  `status` line and in JSON `status.psram`.
- `psram_ping` (JSON) / `cap-selftest` (console): iCE40 writes a known ramp through
  the real writer→PSRAM path; the STM32 verifies it. `hwe2e TestHW_V2_PsramPing`.
- No-write sentinel detector: every `adc_capture_psram` stamps `DEAD BEEF CAFE F00D`
  and flags "iCE40 did NOT write … NOT the analog ADC" if it survives.
- `adc-spi` (console): read the ADC directly over SPI, bypassing PSRAM — proves the
  analog ADC independently of the capture path.
- `tb_psram_writer` (`make psramwrtest` in `ice40/`): now asserts write-data setup
  before the SCLK edge, so the zero-setup class fails in sim.

## Flashing

Software DFU: `dfu` console cmd → `make flash-dfu` (run in background, ~90 s). A
gateware change also needs the console `flash-ice40` to reprogram the iCE40 config
flash + reconfigure. A DFU reflash does NOT reconfigure the iCE40; if a bad firmware
left it unconfigured, run `flash-ice40` or power-cycle.
