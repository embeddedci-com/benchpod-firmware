# Motor & battery emulator

Firmware for the BenchPod motor & battery emulator board: an ESC or other BLDC/PMSM drive runs
against an emulated motor and battery in CI. Hardware and design:
`benchpod-private/pcb/motor-emulator` (`design.md`, schematic rev 0.3; the gateware targets the
rev 0.4 pin swap below).

- `ice40/`: gateware for the board's iCE40UP5K-SG48I. BenchPod loads it over slave SPI (the
  board has no configuration flash) and then drives it over the same pins
  ([PROTOCOL.md](ice40/PROTOCOL.md)).

## Gateware status

Milestones 1 (platform and safety) and 2 (motor model) are done, plus a pass for FPV-class DUTs
(fast, low-resistance, low-inductance motors, high PWM rates). BenchPod can drive the bridge duties
directly, or run the PMSM model: fixed speed or with mechanics, the common-mode loop and dead-time
compensation, halls from the model's angle.

FPV pass (30 Sep 2026):

- Common-mode shaping: when the min-max-injected legs would leave ±(vbus/2 − CM_MARGIN), all
  three shift together (line voltages kept) before any leg clips; v_cm is clipped to the bus.
- Back-EMF angle advance (ADV, half PWM ticks, default 1.5): the voltage computed at a tick is
  applied for the whole next period, several electrical degrees at FPV speeds.
- Phase-resistance subtraction (R_SUB): the emulator's own path (choke, shunt, FETs) is more
  resistance than a small FPV motor has; the model adds R_SUB × i to every leg to cancel part of
  it. Simulated in `spice/motor-emulator` (`studies.py rsub`) before going in.
- Duties by reciprocal: one 2^30/vbus per tick (31 clk, in the background), then each leg's duty
  is a DSP multiply. The model's PWM period floor is 180 clk (200 kHz).
- Sensing filters: `studies.py filt` found no change needed (sinc3 OSR 64 kept, no notch: the
  i0 ripple at the loop is 0.006-0.022 A pp across the FPV PWM range).

| Block | File | What it does |
| --- | --- | --- |
| Clock | `top.v` | PLL 12 → 36 MHz, the only clock (no clock-domain crossing); MCLK 18 MHz, 50 % duty |
| Link | `spi_link.v` | SPI slave (≤ 6 MHz), board-addressed 16-bit registers, broadcast writes, MISO only when addressed |
| Bridge PWM | `pwm3.v` | 3 legs, HS/LS per leg, centred pulses, 1-clk resolution + first-order dither, on-delay dead time, min-on snapping |
| Sensing | `sinc5.v` | 5 × sinc3 OSR 64 (phases, DUT battery, bus), one shared comb pipeline with its state in block RAM |
| Motor model | `motor_model.v`, `model.masm` | microcoded engine: 4-stage pipeline, one multiplier, 34-bit accumulator, program and data in block RAM; per PWM period (166 instructions): back-EMF at the advanced angle, R_SUB, torque, mechanics, i0 PI, dead-time compensation, min-max injection and CM shaping, duties by reciprocal, halls |
| Board id | `i2c_probe.v` | probes the EEPROM at 0x50-0x53 at start-up: stack position |
| Stack | `sync_wd.v` | board 0 sends SYNC (1 µs every 1 ms); watchdog on SYNC held low |

`tools/masm.py` assembles `src/model.masm` (the Makefile runs it). The engine has no interlocks:
the assembler spaces dependent instructions and pads with NOPs, and expands compares (MAX, MIN,
MAXN) and DTC into flag + action pairs, so no adder or comparator result steers the next
operation in the same clock.

Safety, in the gateware (the board's pull-downs cover a blank FPGA):

- Bridge gates stay low unless PWM_EN, the fault latch is armed, FAULT is high, the watchdog has
  not tripped and the PLL is locked. The gates are also gated at the pins, so a fault drops them
  within 2 clk, not at the next period. FAULT or a trip clears PWM_EN: no automatic restart.
- HS and LS of a leg are never high together, and every edge has at least DEADTIME + 1 clk of gap
  (checked on every clock in `tb_pwm3`, including 0 ↔ 100 % flips across a period boundary).
- Hot-swap enable, brake and the SYNC pulse are refused on boards other than board 0, and before
  the board id is known.
- ARM is one pulse per register write (the latch input is AC-coupled); every pad output comes
  straight from a flop (a decoded output glitched in the gate-level run).

Verification (`make test`, about 3 minutes): unit benches for each block, a bit-exact bench of the
model against an independent reference of its fixed-point spec (1275 ticks, five runs including a
re-arm and the shaping properties: legs inside the margin, line voltages kept), the whole chip over the real SPI link (both pinouts), and the same model and chip benches
on the yosys gate-level netlist. The build fails if yosys reports a signal driven from two always
blocks (that once simulated fine and synthesised to a constant).

Resources and timing (yosys 0.65, nextpnr-ice40 0.10): 4,631 of 5,280 LCs (87 %), 10 of 30 block
RAMs, 4 of 8 DSPs. Timing closes at 37.1-41.1 MHz against the 36 MHz target on every seed from 1
to 8, for both pinouts. The margin is thin: new features need an area and timing pass, and the
UP5K has about 600 LCs left.

## Pinout: rev 0.4 pin swap

The UP5K's RGB pins (39, 40, 41) are output-only in the open-source flow: nextpnr accepts only
`SB_RGBA_DRV` (a constant-current sink) on them, and the pad cannot be read. On schematic rev 0.3
SYNC sat on RGB1, so a board could send SYNC but never see another board hold it low. The design
doc (Gateware bring-up findings, 30 Sep 2026) swaps SYNC to pin 48 (normal I/O, open drain with
readback) and EE_WC_N to pin 40 (write-only, a current sink suits it). The gateware targets that
pinout (`emu.pcf`); `make REV03=1` builds for the rev 0.3 pins (`emu_rev03.pcf`, no SYNC readback).

ARM_REQ_N is a 24 mA current sink into an AC-coupled latch input (100 nF), about 14 us to pull it
down, so the ARM pulse is 100 us.

## Building

```sh
cd ice40
make test            # every bench must print PASS (sim/run_vvp.sh enforces it)
make                 # build/motor_emulator.bin (rev 0.4 pinout)
make REV03=1         # build/motor_emulator_rev03.bin (schematic rev 0.3 pins)
```

Needs yosys, nextpnr-ice40, icestorm and iverilog (`brew install yosys nextpnr-ice40 icestorm
icarus-verilog`).

## Next

In rough order:

1. Encoder emulation from the model's angle: ABZ (edge generator between ticks) and the SPI-slave
   profiles (AS5047P, AS5048A, MA730). This needs LCs: the sinc5 integrators and the PWM prep are
   the next candidates for sharing.
2. Trip recorder read (PCAL6408A), OSR-256 logging stream, brake energy budget, PV setpoint
   sigma-delta (DNP path), stack SYNC as the model's time base.
3. Bench calibration: current and bus offsets and gains, the effective dead time, the choke's
   leakage inductance.
4. BenchPod side: slave-SPI configuration loader (PSRAM-streamed) and a host driver for the
   register protocol, including the physical-unit conversion of the model parameters.
