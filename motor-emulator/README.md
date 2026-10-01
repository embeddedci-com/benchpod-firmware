# Motor & battery emulator

Firmware for the BenchPod motor & battery emulator board: an ESC or other BLDC/PMSM drive runs
against an emulated motor and battery in CI. Hardware and design:
`benchpod-private/pcb/motor-emulator` (`design.md`; the gateware targets schematic rev 0.5, see
"FPGA: ECP5 LFE5U-25F, rev 0.5" there).

- `ecp5/`: gateware for the board's ECP5 LFE5U-25F-6BG256C. BenchPod loads it over slave SPI (the
  board has no configuration flash) and then drives it over the same pins
  ([PROTOCOL.md](ecp5/PROTOCOL.md)). Until 30 Sep 2026 the board had an iCE40UP5K, which ran out
  of logic with the encoder; iCE40 support is gone (git history has it).

## Gateware status

Milestones 1 (platform and safety) and 2 (motor model) are done, the design doc's gateware list is
complete (30 Sep 2026: protection layer, trip recorder, battery model, logging stream, stack time
base, EEPROM, fault injection, back-EMF shape; registers in PROTOCOL.md), plus a pass for FPV-class DUTs
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
| Sensing | `sinc5.v` | 5 × sinc3 OSR 64 (phases, DUT battery, bus), one shared comb pipeline with its state in block RAM; a second instance at OSR 256 for logging |
| Motor model | `motor_model.v`, `model.masm` | microcoded engine: 4-stage pipeline, one multiplier, 34-bit accumulator, program and data in block RAM; per PWM period (166 instructions): back-EMF at the advanced angle, R_SUB, torque, mechanics, i0 PI, dead-time compensation, min-max injection and CM shaping, duties by reciprocal, halls |
| I²C | `i2c_ctl.v` | board id (EEPROM at 0x50-0x53), the trip-recorder expander (PCAL6408A input latch, read on FAULT), EEPROM byte read / write |
| Protection | `protect.v` | overcurrent trip per phase from the sinc samples (holds SYNC low: the whole stack stops), per-DUT bus overvoltage brake with hysteresis, brake energy budget, brake test pulse (resistor detection), hot-swap interlock |
| Battery | `battery.v` | battery model: OCV table over the charge drawn, R0 sag, an RC branch; the setpoint for BenchPod's DAC or the PV_SET sigma-delta output |
| Logging | `logbuf.v`, `sinc5.v` | OSR-256 sinc3 (or the OSR-64 samples) into a FIFO read in bursts through 0xC0-0xFF; no word lost when a burst ends early |
| Stack | `sync_wd.v`, `top.v` | board 0 sends SYNC (1 µs every 1 ms); watchdog on SYNC held low; the µs / ms time base; stack time (TIME_MS / TIME_SUB) and the PWM period locked to SYNC |
| Encoder | `encoder.v` | the rotor angle as ABZ and as an AS5047P / AS5048A / MA730 SPI slave; see below |
| Faults, shape | `top.v`, `encoder.v`, `motor_model.v` | fault injection (halls stuck or inverted, encoder frozen / ABZ dead / MISO stuck, back-EMF noise on the model duties); a writable back-EMF shape table (trapezoidal or any other shape) |

Register reads: live values come through fabric muxes, everything BenchPod writes reads back from
a block-RAM mirror of the link writes (so the model parameters are readable too).

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

Verification (`make test`, about 10 minutes): unit benches for each block against independent
references (the encoder: angle, ABZ, three SPI profiles at 8 MHz; the battery model bit-exact over
24k sample sets; the I²C controller against M24C02 and PCAL6408A models; protection timing against
the arithmetic; the sinc filters at OSR 64 and 256), a bit-exact bench of the model against an
independent reference of its fixed-point spec (1425 ticks, six runs including a re-arm, the
shaping properties and a trapezoidal back-EMF), the whole chip over the real SPI link (17 steps,
every feature through its registers), and the same model and chip benches on the yosys gate-level
ECP5 netlist (`sim/mult18x18d_sim.v` models the hard multiplier, which yosys's library has only as
a black box; memories go to LUT RAM there, since the library's block-RAM cell has no behaviour).
The build fails if yosys reports a signal driven from two always blocks (that once simulated fine
and synthesised to a constant).

Resources and timing (yosys 0.65, nextpnr-ecp5 0.10, OSS CAD Suite): 38 % of the 24,288 LUTs,
20 % of the flip-flops, 18 of 28 multipliers, 9 of 56 block RAMs; timing closes at 64.4-67.1 MHz
against the 36 MHz clock on seeds 1-8. The bitstream is 256 KB (compressed). The iCE40-era area
pass (register read-back mirror, shared sinc comb register, the model's shifter and angle advance
on multipliers, one µs / ms time base) is kept; it costs nothing here.

## Encoder emulation

`src/encoder.v`, verified in simulation (`tb_encoder`, and through the link in `tb_top`),
registers in [PROTOCOL.md](ecp5/PROTOCOL.md#encoder):

- Mechanical angle from the model's electrical angle, exact over any run: electrical revolutions
  counted modulo the pole pairs, one 22-step division per PWM period; then BenchPod's DIR and
  offset.
- ABZ: an edge generator chases the angle in 2-LSB steps spread evenly over each PWM period (one
  period of lag, up to 66k RPM mechanical); counts by an exact carry accumulator, any CPR that is a
  multiple of 4; Z at count 0 with A = B = 0, as the AS5047P and MA730.
- SPI slave, single clock (oversampled, SCK up to 8 MHz): AS5047P and AS5048A (mode 1, parity,
  answers in the next frame, error flags, writes) and MA730 (mode 0 / 3, angle every frame,
  register read / write), with a 64-entry register file BenchPod can load (DIAAGC, magnitude,
  magnet faults).

## Pinout

`ecp5/emu.lpf` is the ball map, the same table as the design doc's rev 0.5 section. The
power-stage outputs (PWM, brake, hot-swap, ARM, PV setpoint) sit on the top banks: the ECP5's
left and right banks have no hot-socket protection. Pads are plain Verilog in `top.v` (yosys maps
them to the ECP5 I/O cells); pulls, drive and slew are in the LPF. ARM drives the gate of the
N-FET that pulls ARM_REQ_N (the iCE40's 24 mA current sink is gone); LED, EE_WC_N and SYNC are
open drain. The link clock comes in on a user ball (T6) as well as CCLK, which is a dedicated
configuration pin.

## Building

```sh
cd ecp5
make test            # every bench must print PASS (sim/run_vvp.sh enforces it)
make                 # build/motor_emulator.bit
make SEED=n          # another placement seed
```

Needs the OSS CAD Suite (yosys, nextpnr-ecp5, prjtrellis' ecppack; `TOOLS=` points at its `bin`,
default `/opt/oss-cad-suite/bin`) and iverilog.

## Next

In rough order:

1. Bring-up on the rev 0.5 board: BenchPod's slave-SPI loader for the ECP5 (write-only:
   PROGRAMN, 50 ms, ISC_ENABLE, LSC_BITSTREAM_BURST + bitstream, ISC_DISABLE, then DONE), then
   the hardware checks in the design doc (DONE gating of BRAKE / SHDN at power-up).
2. Bench calibration: current and bus offsets and gains, the effective dead time, the choke's
   leakage inductance.
3. BenchPod side: slave-SPI configuration loader (PSRAM-streamed) and a host driver for the
   register protocol, including the physical-unit conversion of the model parameters and the
   encoder register-file tables per chip, the battery-model and brake-budget unit conversions, the
   log-stream reader.
