# bench-pod iCE40 gateware (v2)

See [`PROTOCOL.md`](PROTOCOL.md) for the SPI command set.

Lattice iCE40UltraPlus (**iCE40UP5K-SG48**) gateware for the EmbeddedCI BenchPod
"vbench-pod" board. It is an **SPI slave to the STM32H563** and owns the
timing-critical analog + digital datapath: the 16-bit serial DAC (DAC8551), the
16-bit serial ADC (MCP33131D-10), the 14-channel logic-analyzer bank, and the
shared APS6404L **PSRAM** that captures stream into and deep DAC replay streams out of.

The STM32 sends high-level commands over SPI ("load this waveform", "capture N
samples at this rate", "arm the logic analyzer"); the gateware runs the
sample-rate clocks, buffers samples, and (for captures) streams them into
PSRAM so the STM32 can read the result back over its own memory-mapped XSPI.

> The legacy **v1** parallel-ADC gateware (for the retired RP2350 boards) was
> removed 2026-07-09. `top_v2.v` is the only top. It is built as two images from the
> same source: the **loop** image (`bench_pod_fpga_vbench_pod_loop.bin`, with the in-fabric
> DAC control loop) and the **deep** image (`..._deep.bin`, built with `-DUSE_DEEP_REPLAY`,
> with deep DAC replay from PSRAM). Both report the same gateware version; `FPGA_FEATURES`
> (0x18) says which one is running. The firmware carries both and swaps them by rewriting the
> iCE40 config flash (JSON `fpga_image`); SB_WARMBOOT cannot work on this board because the
> config flash shares the PSRAM bus.

## Capture datapath

```
        ┌─ adc_mcp33131 ─┐
STM32 ─ SPI ─► cmd_dispatch ─ engine_block ─┤ ADC producer ───────────────────────┐ (straight to writer FIFO)
   ▲         (opcode FSM)     (shared cores) └ LA  producer ─► spram_ring16 (AW15) ─┤
   │                                                                                ▼
   └──────── XSPI read-back ◄── PSRAM ◄──────── psram_dual_writer (48 MHz DDR drain)
                                two regions (sys_config.vh: LA @ base, ADC @ 4 MB)
```

- **Two producers, one trigger.** `OP_CAPTURE` (0x31) pulses the ADC and LA arms
  on the *same* cycle (shared t0). A count of 0 leaves that producer off, so it also
  covers ADC-only and LA-only captures. Since v40 it is the only capture arm: the older
  `OP_START_CAPTURE` (0x20), `OP_START_MEASURE` (0x30) and `OP_LA_CAPTURE` (0x69) are
  ignored.
- **Capture trigger** (v35). `OP_SET_TRIGGER` (0x33) sets a persistent
  channel/mode (rising/falling/high/low) on the synchronised LA levels. An arm
  then *loads* the producers but holds them until the condition is seen; that
  cycle is t0, so the DAC co-trigger and `SET_DAC_STOP_AFTER` count from the
  trigger, not the arm. `OP_TRIGGER_STATUS` (0x34) reports waiting/fired. The
  captured window is entirely *post*-trigger (no pre-trigger buffer: see the v35
  note in `top_v2.v`). Mode 0 is the pre-v35 untriggered capture, cycle for cycle.
  `OP_GPIO_GET` (0x43) reads the same synchronized 14 levels without arming
  anything.
- **Asymmetric buffering.** The LA producer (12 MS/s burst) packs its samples into
  a deep 16-bit-packed SPRAM ring (`spram_ring16`, AW=15) so the writer can drain
  it independently. The ADC producer (at most 400 kS/s, ~0.8 MB/s) has **no** deep
  ring: it writes straight into `psram_dual_writer`'s 32-byte ADC staging FIFO, and
  the starvation-free bus arbiter keeps that FIFO from overflowing (this dropped a
  whole `spram_ring16`, about 120 LC + an SPRAM block, in the v19 LC-reduction pass).
- **One writer.** `psram_dual_writer` arbitrates the LA ring and the ADC staging
  FIFO and streams both into two contiguous PSRAM regions over a single 48 MHz
  DDR-clocked QPI link, raising `CAP_DONE` when both drain. The default regions
  are single-sourced in `sys_config.vh` (LA at 0, ADC at 4 MB); `OP_SET_CAPTURE_BASES`
  (0x32) moves the ADC base at run time so a capture fits below a resident replay
  waveform.
- **Shared bus.** In the deep image `dac_psram_reader` streams a replay waveform out
  of the top of PSRAM while a capture streams in at the bottom; `psram_bus_arbiter`
  hands the bus to one master per burst and `psram_pads` muxes the pads. The STM32
  takes the whole bus with `bus_own` (every iCE40 pad tristates) to read captures and
  stage waveforms over XSPI.
- **Depths.** Both capture counts are 24-bit: up to 2,097,152 ADC samples in the
  4 MB ADC region, and LA up to the full 8 MB PSRAM when the ADC is off. The firmware
  enforces the region caps; nothing in the gateware bounds a stream to its region.
- **Clocks.** An external 48 MHz `clk48` feeds the PSRAM serializers and the DAC
  engine; the control plane and capture datapath run on `clk` = clk48 / 2 (24 MHz).
  The clk to clk48 hop is a synchronous 1:2 gearbox (`cell_gearbox48`). The promoted
  v47 images close `clk` at about 30 to 33 MHz and `clk48` at about 55 to 58 MHz
  (`ice40/release/MANIFEST` records the exact figures).
- **Integrity.** Before every capture the STM32 stamps a no-write sentinel at the
  ADC region base; if it survives the read-back the iCE40 never wrote (a
  datapath/timing fault, not the analog ADC). `STATUS` carries CAP_DONE / CAP_BUSY
  / CAP_OVF (bit 5) and the control-loop trip (bit 6); bit 7 is hardwired 0 so firmware
  can tell an unconfigured FPGA (floated `0xFF`) from a real status byte.

[`../docs/tri-capture-unified-psram.md`](../docs/tri-capture-unified-psram.md) describes a
single-master alternative to the writer + reader + arbiter split; it was shelved in v40 because
it came out bigger.

## DAC

`dac8551_engine` shifts 24-bit frames to the DAC8551 on `clk48` (SCLK 24 MHz). One
sample takes max(divider, 3) + 51 clk48, so the fastest update is 48 MHz / 54, about
889 kS/s. It plays from one of three sources: the 4 KB `LOAD_WAVE` BRAM (up to 2048
samples, `START_DAC`), PSRAM through `dac_psram_reader` (deep image, up to 4,194,304
samples, `START_DAC_PSRAM`), or `dac_loop` (loop image, an in-fabric transfer function
that indexes a curve with the ADC or a fixed/sweep input, `START_DAC_LOOP`; see
[`../docs/dac-control-loop.md`](../docs/dac-control-loop.md)).

## Logic-analyzer bank, SWD and SPI

The 14 bidirectional LA channels (`LA1`..`LA14`) are a shared resource: the
SWD/SPI engine, the stepper pulse generator, the I²C target, the UART proxy and
static GPIO all drive them through a priority mux
(`swd > stepper > i2c > uart > static`). `swd_engine.v` takes SWCLK/SWDIO as LA
channel indices (nRESET is the pod's own pin since v37), decodes OpenOCD
`remote_bitbang` batches, and since v45 also runs whole SWD transfers from a queue
(`SWD_QFEED`). Since v44 the same engine is an SPI master on four LA channels.

## Modules

| File | Purpose |
|---|---|
| `src/top_v2.v` | Top-level: clocks/reset/pads, the ADC/LA→PSRAM capture datapath, the DAC sources, the trigger |
| `src/spi_slave.v` | Bit-bang SPI slave with CDC for SCK/CSn |
| `src/cmd_dispatch.v` | FSM that decodes SPI opcodes and drives the engines |
| `src/engine_block.v` | Shared instrument cores wired to the version-specific top |
| `src/adc_mcp33131.v` | 16-bit serial SAR ADC (MCP33131D-10) sampler |
| `src/dac8551_engine.v` | 16-bit serial DAC (DAC8551) waveform sequencer |
| `src/la_psram_capture.v` | Packs the 14-ch LA word (2 bytes per sample) into the LA ring |
| `src/spram_ring16.v` | 16-bit-packed single-port SPRAM burst ring (one per stream) |
| `src/psram_dual_writer.v` | 48 MHz DDR QPI writer → two PSRAM regions |
| `src/dac_psram_reader.v` | QPI streaming reader that feeds the DAC from PSRAM (deep image) |
| `src/psram_bus_arbiter.v` | Burst-granular arbiter between the writer and the reader |
| `src/psram_pads.v` | Shared PSRAM pad mux, `bus_own` tristate, one-clk48 output retime |
| `src/cell_gearbox48.v` | clk → clk48 cell gearbox (receive half) shared by the PSRAM writer and reader |
| `src/cdc_pulse_payload.v` | Pulse + value crossing from clk to clk48 (DAC start/stop and their arguments) |
| `src/dac_loop.v` | In-fabric DAC control loop: input source, affine map, curve lookup, damping, trip (loop image) |
| `src/sample_buf.v` | Dual-clock BRAM (inferred → `SB_RAM40_4K`) for DAC waveforms and the loop curve |
| `src/la_bank.v` | 14 bidirectional LA channels + driver priority mux |
| `src/stepper_engine.v` | Microsecond-precise step-pulse generator |
| `src/swd_engine.v` | OpenOCD `remote_bitbang` SWD decoder, SWD transfer queue, SPI master, reply buffer |
| `src/i2c_target.v` / `src/i2c_regfile.v` | I²C sensor emulation (target + 256-byte register file) |
| `src/uart_engine.v` | Soft UART proxy |
| `src/dsp_counter.v` / `src/dsp_counter2.v` | Counters packed into SB_MAC16 DSP blocks to save logic cells |
| `src/lc_counter.v` | Loadable counter in one logic cell per bit |
| `src/cmd_opcodes.vh` | SPI opcode table (generated by `tools/gen_protocol.py`) |
| `src/sys_config.vh` | Logic clock and default PSRAM regions (generated by `tools/gen_protocol.py`) |

## Build

Open-source IceStorm toolchain (`brew install icestorm yosys nextpnr-ice40`):

```
make images          # build both runtime images: ..._loop.bin and ..._deep.bin (pinned seeds)
make                 # build bench_pod_fpga_vbench_pod.bin (one image, for flash/flash-ram)
make flash           # build + program the config flash via an FTDI programmer (iceprog)
make flash-ram       # build + load straight into FPGA SRAM (volatile)
make sim             # iverilog syntax check
make test            # all self-checking sims + the protocol and release-tool checks
make test PNR_STRICT=1   # what CI runs (also fails on a thin clk48 margin)
```

Pin constraints live in `vbench_pod.pcf`. Use yosys 0.65 (0.66 places worse with
`-abc9`); the toolchain is recorded in `release/MANIFEST`.

### Releases embed promoted images

The images are not embedded in the firmware binary. A firmware build packs
`ice40/bench_pod_fpga_vbench_pod_{loop,deep}.bin` into the blobs that go with it
(`build/blobs/blob-gw0.bin` = loop, `blob-gw1.bin` = deep, kept in the pod's W25Q) and
records their gateware version. A release build uses `ice40/release/` instead: images that passed a hardware test, promoted
with `make promote HW_VERIFIED="<which test>"` (which records the source hash, seeds,
toolchain and clk48 margins in `release/MANIFEST`) and installed with
`make release-images`. `make check-release` fails once any file in `src/`,
`vbench_pod.pcf` or `clocks.py` changes, comments included, so every gateware change
needs a hardware test and a new promote before the next firmware release. On a PR or
main the CI only warns; the release workflow fails.

At boot (and after a gateware blob install) the firmware compares the running gateware
version with the one it was built for and, when they differ, rewrites the iCE40 config
flash from the matching W25Q slot, keeping the image kind that was running. A blank or
broken iCE40 gets image 0 (loop). The USB console command `flash-ice40` does the same by
hand.

## Simulation

`sim/` holds self-checking iverilog testbenches for the individual cores
(`make dactest adctest psramwrtest dualwrtest lapsramtest measuretest argtest
steppertest …`) and whole-top benches that drive real SPI transactions through
the SB_IO pads (`make topsmoketest topcapturetest spitest swdqtest` on the loop
image, `make topdeeptest` on the deep image: replay across bus_own, a capture
read back mid-replay, overflow cleared by the next arm).
`make check-protocol` verifies the generated files (opcode table, `sys_config.vh`,
the firmware headers and [`PROTOCOL.md`](PROTOCOL.md)) are in sync with
`tools/gen_protocol.py`.
