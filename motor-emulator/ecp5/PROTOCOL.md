# Motor emulator link protocol (BenchPod ↔ emulator ECP5)

BenchPod talks to the emulator boards over the stack link on header pins 1-11: SCK, MOSI, MISO, SS,
plus PROGRAMN/DONE for configuration, SYNC and FAULT. After configuration the same SPI pins carry
this register protocol. All boards in a stack share SCK, MOSI, MISO and SS, so every transaction
names its board.

## Electrical

- SPI mode 0 (CPOL 0, CPHA 0), MSB first, SS low for the whole transaction.
- SCK up to 6 MHz (the gateware oversamples SCK with its 36 MHz clock; `tb_top` runs at 6 MHz).
- MISO is driven only by the board a read addresses; otherwise it floats (each board has 470 Ω
  in series, so a mistake cannot short two drivers).

## Transactions

```
byte 0   command   [7] 1 = read, 0 = write
                   [6] broadcast (writes only: every board applies it)
                   [5:4] board id (0-3, from the EEPROM strap address 0x50-0x53)
                   [3:0] 0
byte 1   register address (auto-increments after each 16-bit word, except on a port register)

write:   MSB, LSB [, MSB, LSB ...]          one register write per LSB
read:    one turnaround byte (MISO 0x00), then MSB, LSB [, MSB, LSB ...]
```

Port registers keep the address for the whole burst, so one transaction loads a table or drains
the log: BAT_TBL_DATA (0x69), SHAPE_DATA (0x78) and the LOG_FIFO window (0xC0-0xFF).

A read captures the whole 16-bit register when its MSB goes out, so a live value (a sinc sample)
cannot tear between its two bytes. Broadcast reads are ignored (four boards would drive MISO).

Example: write 0x4000 to DUTY_A on board 2: `20 0A 40 00`. Read STATUS from board 1:
`90 03 00 xx xx` (the last two bytes return MSB, LSB).

## Registers (16 bit)

| Addr | Name | Access | Description |
| --- | --- | --- | --- |
| 0x00 | ID | R | 0x4D45 ("ME") |
| 0x01 | VERSION | R | gateware version (0x0006: protection, trip recorder, battery model, logging, stack time base, EEPROM, fault injection, back-EMF shape; 0x0007: port registers take a whole burst) |
| 0x02 | BOARD | R/W | [1:0] board id, [8] probe done, [9] probe ok (EEPROM answered). Write bit 15 = probe again |
| 0x03 | STATUS | R | see below |
| 0x04 | CONTROL | R/W | see below; FAULT, a watchdog trip or an overcurrent trip clears PWM_EN; an overcurrent or hot-swap trip clears HSWAP_EN |
| 0x05 | ARM | W | any value: one 100-101 µs pulse on ARM_REQ_N (sets the board fault latch) |
| 0x06 | WD_CLEAR | W | any value: clear the SYNC watchdog trip |
| 0x07 | SCRATCH | R/W | free, for link tests |
| 0x08 | PWM_PERIOD | R/W | clocks per bridge PWM period at 36 MHz (default 180 = 200 kHz, minimum 32) |
| 0x09 | DEADTIME | R/W | extra gap per edge in clocks (0-63); both gates are off for DEADTIME + 1 clk, on top of the MP6539's own floor |
| 0x0A-0x0C | DUTY_A/B/C | R/W | Q0.16 fraction of the period the phase node sits at the bus (default 0x8000); dithered, so the average has 16-bit resolution |
| 0x0D | MIN_ON | R/W | shortest gate pulse in clocks (default 2); shorter pulses snap to 0 or 100 % |
| 0x10-0x14 | SINC_A..E | R | signed, sinc3 OSR 64 at 18 MHz (281.25 kSPS): phase A, B, C current, DUT battery current, bus voltage. 0 = 0 V at the modulator, ±32767 = full scale (±320 mV) |
| 0x15 | SINC_COUNT | R | increments with every new sample set |
| 0x18 | BRAKE_DUTY | R/W | [7:0] manual brake PWM duty /256 at about 20 kHz (needs CONTROL.BRAKE_EN; board 0 only; the energy budget applies) |
| 0x1C | HALL | R/W | [2:0] hall levels the DUT sees (C, B, A); default 111 (used unless MODE[4]) |
| 0x20 | MODE | R/W | [1:0] 0 = direct duties (DUTY_A-C), 1 = model at fixed speed, 2 = model with mechanics; [2] common-mode loop; [3] dead-time compensation; [4] halls from the model's angle |
| 0x21-0x2E | model parameters | R/W | see Motor model below |
| 0x2F | MODEL_RESET | W | any value: angle 0, speed = W_SET, common-mode integrator 0 |
| 0x30 | THETA | R | electrical angle, top 16 bits (65536 = one revolution) |
| 0x31 | SPEED | R | w16 (speed, see below) |
| 0x32 | EMF | R | back-EMF amplitude E |
| 0x33 | I0 | R | zero-sequence current (ia + ib + ic) / 3 |
| 0x34 | VCM | R | common-mode loop output |
| 0x35 | IQ | R | current in phase with the back-EMF |
| 0x36-0x38 | MDUTY_A-C | R | the model's duties |
| 0x39 | MODEL_COUNT | R | model ticks (one per PWM period) |
| 0x3A-0x3D | I_OFS_A-C, VBUS_OFS | R/W | sinc offsets subtracted before the model (calibration) |
| 0x3E | ADV | R/W | back-EMF angle advance in half PWM periods, 0-7 (default 3 = 1.5 periods) |
| 0x3F | R_SUB | R/W | resistance subtracted from the emulated phase: u_x += R_SUB x i_x >>> 15 (see Motor model) |
| 0x40-0x48 | encoder | | see Encoder below |
| 0x50-0x5E | protection, trip recorder | | see Protection below |
| 0x60-0x6E | battery model | | see Battery model below |
| 0x70-0x73 | stack time base | | see Stack time base below |
| 0x74-0x78 | fault injection, back-EMF shape | | see Fault injection and back-EMF shape below |
| 0x79-0x7B, 0xC0-0xFF | logging stream | | see Logging stream below |
| 0x7C-0x7E | board EEPROM | | see Board EEPROM below |

Reads: the live registers (0x00-0x04, 0x10-0x15, 0x30-0x39, 0x45-0x48, 0x50, 0x59-0x5B, 0x5D,
0x67, 0x6C-0x6E, 0x71-0x73, 0x7A, 0x7B, 0x7D, 0x7E, 0xC0-0xFF) return the current value;
every other register returns the value last written to it, as written (a block-RAM mirror of the
link writes, so the gateware's own clamping and masking, e.g. PWM_PERIOD[9:0], is not shown).
Action registers (ARM, WD_CLEAR, MODEL_RESET, BRK_TEST, BAT_TBL_DATA, SHAPE_DATA) read back what
was written.

Changes to period, dead time, duties and min-on take effect at the start of the next PWM period.
In model modes the period is at least 180 clk (200 kHz): the model runs once per period, takes
about 160 clk, and the PWM reads its duties from 24 clk before the period ends.

### CONTROL (0x04)

| Bit | Name | Meaning |
| --- | --- | --- |
| 0 | PWM_EN | run the bridge; the gates also need LATCH_Q, FAULT high, no watchdog trip |
| 1 | HSWAP_EN | enable the LT4363 hot-swap (board 0 only) |
| 2 | BRAKE_EN | brake chopper (board 0 only) |
| 3 | SYNC_MASTER | send the 1 µs / 1 ms SYNC pulse (board 0 only) |
| 4 | SYNC_REQUIRED | stack mode: trip if SYNC pulses stop for 3 ms |

### STATUS (0x03)

| Bit | Meaning |
| --- | --- |
| 0 | FAULT line level (1 = no fault) |
| 1 | LATCH_Q: board fault latch armed |
| 2 | BACKSTOP: 57 V comparator braking |
| 3 | SYNC line level |
| 4 | watchdog trip (SYNC held low > 100 µs, or missing in stack mode) |
| 5 | bridge PWM running |
| 6 | a SYNC pulse arrived in the last 3 ms |
| 7 | PLL locked |

## Motor model

A PMSM model runs once per PWM period on a small microcoded engine (`src/motor_model.v`, program
`src/model.masm`). The emulator bridge is a voltage source: each leg outputs the modelled back-EMF
plus dead-time compensation plus a common-mode offset; the DUT's current through the dummy
inductors is measured and drives the torque. Units are the sinc codes: currents 2048 / A nominal
(positive = out of the emulator leg), voltages about 391 / V nominal on the bus channel. BenchPod
converts physical parameters with its calibration.

| Addr | Name | Meaning |
| --- | --- | --- |
| 0x21 | W_SET | speed in MODE 1, and the start speed after MODEL_RESET (signed w16) |
| 0x22 | WSHIFT | angle step per tick = w32 >>> (16 - WSHIFT), w32 = w16 << 16 (0-16) |
| 0x23 | KE | E = w16 x KE >>> 15 (0-32767) |
| 0x24 | KT | torque T = KT x iq >>> 15 (signed) |
| 0x25 | LOAD | load torque (signed) |
| 0x26 | DAMP | viscous term DAMP x w16 >>> 15 (0-32767) |
| 0x27 | INVJ | w32 += net x INVJ >>> JSHIFT per tick, net = T - LOAD - damping (MODE 2) |
| 0x28 | JSHIFT | see INVJ (0-31) |
| 0x29 | CM_KP | v_cm = -((CM_KP x i0 >>> 8) + (integrator >>> 16)) |
| 0x2A | CM_KI | integrator += CM_KI x i0 per tick (saturating 32 bits) |
| 0x2B | CM_MARGIN | common-mode shaping margin from each rail, Q16 of the bus (2 % = 1311; about the dead time as a fraction of the PWM period) |
| 0x2C | DT_FRAC | dead-time volts = vbus x DT_FRAC >> 16 (t_dead x f_pwm, e.g. 77 ns x 200 kHz = 1009) |
| 0x2D | DT_IDB | no dead-time compensation while abs(i) <= DT_IDB |
| 0x2E | HALL_OFS | hall angle offset (top 16 bits of the angle) |

Per tick: e_x = E sin(lk_x) at the advanced angle lk (the angle plus ADV half periods of speed:
the voltage computed at a tick is applied during the next period); iq = sum of (-i_x) sin(th_x)
at the angle itself; leg voltage u_x = e_x + (R_SUB x i_x >>> 15) + sign(i_x) x dead-time volts; v_cm = the PI output,
clipped to +-vbus/2; then min-max injection plus v_cm, and common-mode shaping: all three legs
shift together, just enough that the lowest and highest stay CM_MARGIN inside the rails (the line
voltages are kept; this is what a six-step DUT needs, see the design doc), each leg finally
clamped to +-vbus/2; duty_x = 0x8000 +- (abs(v_x) x floor(2^30 / vbus)) >> 14. Angles are 16 bits for the sines and halls
(1/3 revolution = 0x5555).

R_SUB emulates a motor with less phase resistance than the emulator's own path (about 74 mOhm:
inductor 38, shunt 20, choke, FET): R_SUB = (path - motor) in codes, R [Ohm] x 391 / 2048 x 32768,
for example 44 mOhm -> 275. Keep it below the path resistance (the total must stay positive). The
simulation (design doc, studies.py rsub) cut the error against a 30 mOhm motor by 3-5x. Below 512 codes of bus the duties park
at 0x8000. Design-doc gains in these units: CM_KP about 7815, CM_KI about 5000 (Kp 160 V/A,
Ki 80k V/(A s) at 200 kHz with 2048 codes/A and 391 codes/V). The exact fixed-point spec is in the
header of `src/motor_model.v`; `sim/tb_motor_model.v` checks the gateware against it bit for bit.

## Encoder

The rotor position as an encoder, from the model's angle (`src/encoder.v`; `sim/tb_encoder.v`
checks it).

| Addr | Name | Access | Description |
| --- | --- | --- | --- |
| 0x40 | ENC_CTRL | R/W | [0] ABZ outputs on, [1] SPI on, [3:2] SPI profile: 0 AS5047P, 1 AS5048A, 2 MA730; [4] DIR (mechanical direction) |
| 0x41 | ENC_CPR | R/W | ABZ counts (quadrature states) per mechanical revolution, multiple of 4, 4-32764 (AS5047P default 4000, MA730 4096). A write restarts the counter at 0 |
| 0x42 | ENC_POLES | R/W | motor pole pairs, 1-63 |
| 0x43 | ENC_OFS | R/W | mechanical offset added to the angle (65536 = one revolution): where the magnet sits |
| 0x44 | ENC_RF_ADDR | R/W | register-file index 0-63 |
| 0x45 | ENC_RF_DATA | R/W | register file [ENC_RF_ADDR] |
| 0x46 | ENC_ANGLE | R | the mechanical angle the DUT sees (16 bits per revolution) |
| 0x47 | ENC_COUNT | R | the ABZ count, 0 .. CPR-1 |
| 0x48 | ENC_STATUS | R | [15:8] SPI frames received, [2:0] AS504x error flags (parity, invalid command, framing) |

- Angle: mech = floor((electrical revolutions mod POLES x 65536 + THETA) / POLES), exact (no
  drift); angle = (DIR ? -mech : mech) + ENC_OFS. Updated once per PWM period. The model must not
  move more than 1/4 electrical revolution per period.
- ABZ: follows the angle with one PWM period of lag, edges evenly spread over the period (jitter
  up to one generator step, about 2 clk). Up to 72M angle LSB/s (66k RPM mechanical); faster, it
  falls behind and catches up. Z is high for the quadrature state at count 0 (A = B = 0). After
  MODEL_RESET or an ENC_CPR write it walks to the angle at full rate (up to half a revolution).
- SPI: SCK up to 8 MHz, CS low at least 150 ns before the first SCK edge. AS5047P / AS5048A: mode
  1, 16-bit frames with even parity, the answer in the next frame, ERRFL / error register at
  0x0001 (read to clear), EF sticky until then; writes (command frame, data frame) are stored.
  MA730: mode 0 or 3, the angle in every frame (14 bits, [1:0] = 0; partial reads fine),
  register read 010aaaaa_xxxxxxxx and write 100aaaaa_vvvvvvvv answered in the next frame.
- Register file: 0x00-0x1F the MA730's registers (factory values at power-up), 0x20 + a the
  AS504x registers 0x0000-0x001F (AS5047P SETTINGS1 = 0x0001), 0x3C-0x3F the AS504x registers
  0x3FFC-0x3FFF (0x3C = AS5047P DIAAGC 0x0180, 0x3D MAG / AS5048A diagnostics, 0x3E AS5048A
  magnitude; the angle registers are live). BenchPod loads the values for the chosen chip, and can
  set magnet faults there. The DUT's writes are stored and read back but change nothing: zero
  position, direction and resolution come from ENC_OFS, DIR and ENC_CPR.

## Protection

The gateware layer of the board's protection (`src/protect.v`), on top of the hardware latch and
the 57 V backstop. Values are offset-corrected sinc codes: currents about 2048 / A (I_OFS_A-C),
bus about 391 / V (VBUS_OFS).

| Addr | Name | Access | Description |
| --- | --- | --- | --- |
| 0x50 | TRIPS | R / W | R: [2:0] overcurrent on phase A, B, C, [3] hot-swap bus limit (sticky), [4] brake inhibited by the energy budget, [5] overvoltage brake on (live). W: 1s clear bits [3:0] |
| 0x51 | OC_LIM | R/W | phase current limit, codes; 0 = off |
| 0x52 | OC_COUNT | R/W | samples in a row over the limit before the trip, 1-15 (281 kSPS per phase: 1 trips within about 8 µs) |
| 0x53 | OV_ON | R/W | bus codes: the brake turns fully on above this (board 0); 0 = off |
| 0x54 | OV_OFF | R/W | bus codes: and off again below this |
| 0x55 | BRK_G | R/W | brake conductance for the budget: P [W] = bus^2 x BRK_G >> 24, so BRK_G = 2^24 / (391^2 x R): 11 for 10 Ω, 33 for 10 Ω ‖ 5 Ω |
| 0x56 | BRK_PAVG | R/W | W the resistor may take on average (the bucket drains this much per µs) |
| 0x57 | BRK_CAP | R/W | energy budget in 4096 µJ units; above it every brake (manual, overvoltage, test) is inhibited until the bucket is below half; 0 = no budget |
| 0x58 | BRK_TEST | W | brake test pulse of this many µs (board 0) |
| 0x59 / 0x5A | BRK_V0 / BRK_V1 | R | bus codes at the start and the end of the test pulse: R = Δt / (C x ln(V0 / V1)) with the bus capacitance known (hot-swap off) |
| 0x5B | BRK_ENERGY | R | the budget bucket, 4096 µJ units |
| 0x5C | HS_LIMIT | R/W | bus codes the hot-swap may run up to; 0 = the hot-swap is refused. Four bus samples in a row above it while it is on: TRIPS[3], hot-swap off |
| 0x5D | TRIP_SRC | R / W | R: [7:0] the trip expander's inputs at the last read (P0 OC_TRIP_N, P1 DRV_NFAULT, P2 NTC_TRIP_N, P3 HS_FLT_N, P4 FAULT, P5 LATCH_Q, P6 BACKSTOP; active low except P5, P6), [8] valid, [9] expander answered at start-up, [10] I²C error. W: read the expander now |
| 0x5E | BAT_I_OFS | R/W | DUT battery current offset (battery model) |

- An overcurrent trip drops every gate at once, clears PWM_EN and HSWAP_EN, and holds SYNC low,
  so every board in the stack trips its watchdog (100 µs). To recover: TRIPS clear, WD_CLEAR,
  re-arm, re-enable.
- The expander's inputs latch (PCAL6408A input latch): FAULT going low makes the gateware read
  it, so TRIP_SRC shows the source that fired even if it has recovered. Reading clears the latch.
- The hot-swap needs board 0, HSWAP_EN, HS_LIMIT set, no trip and no watchdog trip.
- PV_SET (battery model) parks on any trip, a watchdog trip or FAULT.

## Battery model

The bus voltage a pack would show, from the measured DUT battery current (`src/battery.v`). In
codes: current from the DUT battery channel minus BAT_I_OFS (positive = discharge; BAT_CTRL[1]
flips it), voltages in bus codes.

    every sample set (281 kSPS):  setpoint = clamp(OCV - (i x R0 >>> 15) - v_rc, VMIN, VMAX)
    every ms:                     charge += the ms's current sum; i_ms = sum x 233 >>> 16 (mean)
                                  v_rc += ((i_ms x R1 >>> 15) - v_rc) x ALPHA >>> 20
                                  OCV = table at p = charge >>> (QSHIFT - 8), interpolated

| Addr | Name | Access | Description |
| --- | --- | --- | --- |
| 0x60 | BAT_CTRL | R/W | [0] run, [1] invert the current, [2] drive PV_SET (board 0) |
| 0x61 | BAT_QSHIFT | R/W | charge per table entry is 2^QSHIFT code-samples (1 A for 1 h = 2048 x 281250 x 3600 = 2.07e12): pick it so 64 entries cover the pack (8-47) |
| 0x62 | BAT_R0 | R/W | series resistance: R [Ω] x 391 / 2048 x 32768 (20 mΩ = 125) |
| 0x63 | BAT_R1 | R/W | RC-branch resistance, same units |
| 0x64 | BAT_ALPHA | R/W | 1 ms / τ x 2^20 (τ from 16 ms to about 10 min) |
| 0x65 / 0x66 | BAT_VMIN / BAT_VMAX | R/W | setpoint clamp, bus codes |
| 0x67 | BAT_POS | R / W | position in the table, 1/256 entries (0 = full); a write sets the charge (state of charge) |
| 0x68 / 0x69 | BAT_TBL_ADDR / BAT_TBL_DATA | R/W, W | OCV table, 64 entries in bus codes for the whole pack, entry 0 full; each data write moves the address on (a port register: the whole table in one burst) |
| 0x6A / 0x6B | PV_OFS / PV_GAIN | R/W | PV_SET duty = clamp(PV_OFS + (setpoint x PV_GAIN >>> 15), 0, 65535): BenchPod's calibration of the PV board's (inverted) setpoint input |
| 0x6C | BAT_SET | R | the setpoint, bus codes: BenchPod's DAC path reads this (default) |
| 0x6D | BAT_I | R | mean current over the last ms |
| 0x6E | BAT_OCV | R | open-circuit voltage at the position |

PV_SET is a first-order sigma-delta (fraction of time high = duty / 65536) into the board's DNP
open-drain buffer and RC filter; high is released (the divider's park level).

## Stack time base

| Addr | Name | Access | Description |
| --- | --- | --- | --- |
| 0x70 | SYNC_CTRL | R/W | [0] lock the PWM period (the model's tick) to SYNC: each SYNC pulse measures the PWM phase and the next periods are stretched or shrunk by one clk until it is 0. Only with a period that divides 36000 clk (1 ms), e.g. 180 |
| 0x71 | TIME_MS | R | ms: SYNC pulses while SYNC is seen, the local ms otherwise. A read latches TIME_SUB, so read both in one burst |
| 0x72 | TIME_SUB | R | clk since that ms began (0-35999) |
| 0x73 | SYNC_ERR | R | PWM phase at the last SYNC pulse, clk (signed; 0 when locked) |

## Fault injection and back-EMF shape

| Addr | Name | Access | Description |
| --- | --- | --- | --- |
| 0x74 | HALL_FAULT | R/W | [2:0] hold these hall lines (C, B, A) at [5:3]; [8:6] invert these lines |
| 0x75 | ENC_FAULT | R/W | [0] angle frozen (SPI and ABZ hold: a stuck sensor), [1] ABZ outputs dead (low), [2] MISO stuck low |
| 0x76 | NOISE_AMP | R/W | uniform noise of ± NOISE_AMP (Q16 of the full duty, i.e. of the bus) on each model duty, new every clk; 0 = off |
| 0x77 / 0x78 | SHAPE_ADDR / SHAPE_DATA | R/W, W | back-EMF shape, 1024 Q15 entries per electrical revolution (a sine at power-up); each data write moves the address on (a port register: the whole table in one burst). The model uses it for the back-EMF and the torque alike, so any shape (trapezoidal, harmonics) keeps the power balance |

## Logging stream

| Addr | Name | Access | Description |
| --- | --- | --- | --- |
| 0x79 | LOG_CTRL | R/W | [4:0] channels (phase A, B, C, DUT battery, bus), [5] OSR 256 (sinc3, 70 kSPS per channel, 14.3 ENOB) instead of OSR 64 (281 kSPS), [6] on, [7] a sequence word before each set, [15] clear (write-only action) |
| 0x7A | LOG_LEVEL | R | words waiting (FIFO of 2048) |
| 0x7B | LOG_DROPS | R | sets dropped because the FIFO was full (a whole set at a time: the stream stays aligned) |
| 0xC0-0xFF | LOG_FIFO | R | each word read pops the FIFO: read a burst from 0xC0, of any length (a port register). A word counts as read only when its last bit is out, so a burst may end anywhere. Read no more than LOG_LEVEL: an empty FIFO reads 0x0000 |

The link carries about 370 k words per second at 6 MHz in long bursts: all five channels at
OSR 256 (350 k words/s) only just fit; pick the channels.

## Board EEPROM

| Addr | Name | Access | Description |
| --- | --- | --- | --- |
| 0x7C | EE_ADDR | R/W | byte address in the board's M24C02 (256 bytes) |
| 0x7D | EE_DATA | R / W | W: write this byte at EE_ADDR (WC low only during the write; the gateware polls the 5 ms write cycle). R: the byte the last read returned |
| 0x7E | EE_CMD | R / W | W: [0] read EE_ADDR into EE_DATA. R: [0] busy, [1] error (no ACK) |

## Bring-up sequence

1. Configure (slave SPI, every board at once; see the design doc's rev 0.5 section): pulse
   PROGRAMN low (at least 110 ns), wait 50 ms, then three SS-low frames: ISC_ENABLE (0xC6 + 3
   zero bytes); LSC_BITSTREAM_BURST (0x7A + 3 zero bytes) followed in the same frame by the whole
   .bit file; ISC_DISABLE (0x26 + 3 zero bytes). Nothing is read back, so every board in the stack
   loads at once. SCK up to 60 MHz for this. Then check each board's DONE.
2. Read ID (0x4D45) and BOARD from each board id you expect; BOARD bit 8 says the probe finished.
3. Set PWM_PERIOD, DEADTIME, duties (0x8000 = all legs at half the bus: no phase voltage).
4. Write ARM, check STATUS.LATCH_Q, then set CONTROL.PWM_EN. Arm before the DUT starts switching:
   waking the MP6539 charges its bootstraps with brief low-side pulses.
5. Board 0: set OC_LIM, OV_ON / OV_OFF, the brake budget and HS_LIMIT for the DUT before HSWAP_EN.
6. After any trip (FAULT, watchdog, overcurrent), PWM_EN is clear: read TRIPS and TRIP_SRC, clear
   TRIPS and the watchdog, re-arm and re-enable explicitly.
