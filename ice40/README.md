# bench-pod iCE40 gateware (v2)

Lattice iCE40UltraPlus (**iCE40UP5K-SG48**) gateware for the EmbeddedCI BenchPod
"vbench-pod" board. It is an **SPI slave to the STM32H563** and owns the
timing-critical analog + digital datapath: the 16-bit serial DAC (DAC8551), the
16-bit serial ADC (MCP33131D-10), the 12-channel logic-analyzer bank, and the
shared APS6404L **PSRAM** that captures stream into.

The STM32 sends high-level commands over SPI ("load this waveform", "capture N
samples at this rate", "arm the logic analyzer"); the gateware runs the
sample-rate clocks, buffers samples, and — for captures — streams them into
PSRAM so the STM32 can read the result back over its own memory-mapped XSPI.

> The legacy **v1** parallel-ADC gateware (for the retired RP2350 boards) was
> removed 2026-07-09. `top_v2.v` is the only generation; `make` builds it for
> `BOARD=vbench_pod`.

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
  on the *same* cycle (shared t0); `OP_START_CAPTURE` (0x20) arms ADC only and
  `OP_LA_CAPTURE` (0x69) arms LA only.
- **Capture trigger** (v35). `OP_SET_TRIGGER` (0x33) sets a persistent
  channel/mode (rising/falling/high/low) on the synchronised LA levels. An arm
  then *loads* the producers but holds them until the condition is seen; that
  cycle is t0, so the DAC co-trigger and `SET_DAC_STOP_AFTER` count from the
  trigger, not the arm. `OP_TRIGGER_STATUS` (0x34) reports waiting/fired. The
  captured window is entirely *post*-trigger (no pre-trigger buffer — see the v35
  note in `top_v2.v`). Mode 0 is the pre-v35 untriggered capture, cycle for cycle.
  `OP_GPIO_GET` (0x43) reads the same synchronised 12 levels without arming
  anything.
- **Asymmetric buffering.** The LA producer (12 MS/s burst) packs its samples into
  a deep 16-bit-packed SPRAM ring (`spram_ring16`, AW=15) so the writer can drain
  it independently. The ADC producer (~0.8 MB/s) has **no** deep ring — it writes
  straight into `psram_dual_writer`'s 32-byte ADC staging FIFO; the starvation-free
  bus arbiter keeps that FIFO from overflowing (this dropped a whole `spram_ring16`
  ≈120 LC + an SPRAM block — the v19 LC-reduction pass).
- **One writer.** `psram_dual_writer` arbitrates the LA ring and the ADC staging
  FIFO and streams both into two contiguous PSRAM regions over a single 48 MHz
  DDR-clocked QPI link, raising `CAP_DONE` when both drain. The regions are
  single-sourced in `sys_config.vh`.
- **Depths.** ADC capture count is 16-bit (up to 32768 samples); the standalone
  LA capture count is 24-bit (up to the full 8 MB PSRAM).
- **Clocks.** Main logic runs on the internal 24 MHz `clk`; the PSRAM drain runs
  on a 48 MHz `clk48` (÷2 + POR). fmax closes ~35 MHz on `clk` with wide margin.
- **Integrity.** Before every capture the STM32 stamps a no-write sentinel at the
  ADC region base; if it survives the read-back the iCE40 never wrote (a
  datapath/timing fault, not the analog ADC). `STATUS` carries CAP_DONE / CAP_BUSY
  / CAP_OVF (bit 5); bits 7:6 are hardwired 0 so firmware can tell an unconfigured
  FPGA (floated `0xFF`) from a real overflow.

See [`../docs/tri-capture-unified-psram.md`](../docs/tri-capture-unified-psram.md).

## Logic-analyzer bank + SWD

The 12 bidirectional LA channels (`LA1`..`LA12`) are a shared resource: the
stepper pulse generator, the SWD bit-banger, the I²C target/LA, and static GPIO
all drive them through a priority mux (`swd > stepper > i2c > static`). `dap_start`
selects SWCLK/SWDIO/nRESET as LA channel indices; `swd_engine.v` decodes OpenOCD
`remote_bitbang` batches and drives the edges, keeping the STM32's loop free.

## Modules

| File | Purpose |
|---|---|
| `src/top_v2.v` | Top-level: clocks/reset/pads + the ADC/LA→PSRAM capture datapath |
| `src/spi_slave.v` | Bit-bang SPI slave with CDC for SCK/CSn |
| `src/cmd_dispatch.v` | FSM that decodes SPI opcodes and drives the engines |
| `src/engine_block.v` | Shared instrument cores wired to the version-specific top |
| `src/adc_mcp33131.v` | 16-bit serial SAR ADC (MCP33131D-10) sampler |
| `src/dac8551_engine.v` | 16-bit serial DAC (DAC8551) waveform sequencer |
| `src/la_psram_capture.v` | Packs the 12-ch LA word into the LA ring |
| `src/spram_ring16.v` | 16-bit-packed single-port SPRAM burst ring (one per stream) |
| `src/psram_dual_writer.v` | 48 MHz DDR QPI writer → two PSRAM regions |
| `src/sample_buf.v` | Dual-port BRAM (inferred → `SB_RAM40_4K`) for DAC waveforms |
| `src/la_bank.v` | 12 bidirectional LA channels + driver priority mux |
| `src/stepper_engine.v` | Microsecond-precise step-pulse generator |
| `src/swd_engine.v` | OpenOCD `remote_bitbang` SWD decoder + reply buffer |
| `src/i2c_target.v` / `src/i2c_regfile.v` / `src/i2c_la_capture.v` | I²C sensor emulation + bus logic-analyzer |
| `src/uart_engine.v` | Soft UART proxy |
| `src/cmd_opcodes.vh` | SPI opcode table (generated by `tools/gen_protocol.py`) |

## Build

Open-source IceStorm toolchain (`brew install icestorm yosys nextpnr-ice40`):

```
make                 # build bench_pod_fpga_vbench_pod.bin (BOARD defaults to vbench_pod)
make flash           # build + program the config flash via onboard FTDI (iceprog)
make flash-ram       # build + load straight into FPGA SRAM (volatile)
make sim             # iverilog syntax check
make test            # run all self-checking sims (adc/dac/psram/dispatch/top-capture/…)
```

Pin constraints live in `vbench_pod.pcf`. The bitstream is embedded into the
STM32 firmware, which programs the iCE40 config flash via its `flash-ice40`
console command.

## Simulation

`sim/` holds self-checking iverilog testbenches for the individual cores
(`make dactest adctest psramwrtest dualwrtest lapsramtest measuretest argtest
steppertest ilatest …`) and two whole-top smoke tests that drive real SPI
transactions through the SB_IO pads (`make topsmoketest topcapturetest`).
`make check-protocol` verifies the opcode table is in sync with
`tools/gen_protocol.py`.
