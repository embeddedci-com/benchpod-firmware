# Motor emulator link protocol (BenchPod ↔ emulator iCE40)

BenchPod talks to the emulator boards over the stack link on header pins 1-11: SCK, MOSI, MISO, SS,
plus CRESET_B/CDONE for configuration, SYNC and FAULT. After configuration the same SPI pins carry
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
byte 1   register address (auto-increments after each 16-bit word)

write:   MSB, LSB [, MSB, LSB ...]          one register write per LSB
read:    one turnaround byte (MISO 0x00), then MSB, LSB [, MSB, LSB ...]
```

A read captures the whole 16-bit register when its MSB goes out, so a live value (a sinc sample)
cannot tear between its two bytes. Broadcast reads are ignored (four boards would drive MISO).

Example: write 0x4000 to DUTY_A on board 2: `20 0A 40 00`. Read STATUS from board 1:
`90 03 00 xx xx` (the last two bytes return MSB, LSB).

## Registers (16 bit)

| Addr | Name | Access | Description |
| --- | --- | --- | --- |
| 0x00 | ID | R | 0x4D45 ("ME") |
| 0x01 | VERSION | R | gateware version (0x0004: model with common-mode shaping, angle advance, resistance subtraction) |
| 0x02 | BOARD | R/W | [1:0] board id, [8] probe done, [9] probe ok (EEPROM answered). Write bit 15 = probe again |
| 0x03 | STATUS | R | see below |
| 0x04 | CONTROL | R/W | see below; FAULT or a watchdog trip clears PWM_EN |
| 0x05 | ARM | W | any value: one 100 µs pulse on ARM_REQ_N (sets the board fault latch) |
| 0x06 | WD_CLEAR | W | any value: clear the SYNC watchdog trip |
| 0x07 | SCRATCH | R/W | free, for link tests |
| 0x08 | PWM_PERIOD | R/W | clocks per bridge PWM period at 36 MHz (default 180 = 200 kHz, minimum 32) |
| 0x09 | DEADTIME | R/W | extra gap per edge in clocks (0-63); both gates are off for DEADTIME + 1 clk, on top of the MP6539's own floor |
| 0x0A-0x0C | DUTY_A/B/C | R/W | Q0.16 fraction of the period the phase node sits at the bus (default 0x8000); dithered, so the average has 16-bit resolution |
| 0x0D | MIN_ON | R/W | shortest gate pulse in clocks (default 2); shorter pulses snap to 0 or 100 % |
| 0x10-0x14 | SINC_A..E | R | signed, sinc3 OSR 64 at 18 MHz (281.25 kSPS): phase A, B, C current, DUT battery current, bus voltage. 0 = 0 V at the modulator, ±32767 = full scale (±320 mV) |
| 0x15 | SINC_COUNT | R | increments with every new sample set |
| 0x18 | BRAKE_DUTY | R/W | [7:0] brake PWM duty /256 at about 20 kHz (needs CONTROL.BRAKE_EN; board 0 only) |
| 0x1C | HALL | R/W | [2:0] hall levels the DUT sees (C, B, A); default 111 (used unless MODE[4]) |
| 0x20 | MODE | R/W | [1:0] 0 = direct duties (DUTY_A-C), 1 = model at fixed speed, 2 = model with mechanics; [2] common-mode loop; [3] dead-time compensation; [4] halls from the model's angle |
| 0x21-0x2E | model parameters | W | see Motor model below (write-only: they live in the model's data memory) |
| 0x2F | MODEL_RESET | W | any value: angle 0, speed = W_SET, common-mode integrator 0 |
| 0x30 | THETA | R | electrical angle, top 16 bits (65536 = one revolution) |
| 0x31 | SPEED | R | w16 (speed, see below) |
| 0x32 | EMF | R | back-EMF amplitude E |
| 0x33 | I0 | R | zero-sequence current (ia + ib + ic) / 3 |
| 0x34 | VCM | R | common-mode loop output |
| 0x35 | IQ | R | current in phase with the back-EMF |
| 0x36-0x38 | MDUTY_A-C | R | the model's duties |
| 0x39 | MODEL_COUNT | R | model ticks (one per PWM period) |
| 0x3A-0x3D | I_OFS_A-C, VBUS_OFS | W | sinc offsets subtracted before the model (calibration) |
| 0x3E | ADV | W | back-EMF angle advance in half PWM periods, 0-7 (default 3 = 1.5 periods) |
| 0x3F | R_SUB | W | resistance subtracted from the emulated phase: u_x += R_SUB x i_x >>> 15 (see Motor model) |

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
| 4 | SYNC_REQUIRED | stack mode: trip if SYNC pulses stop for 3 ms (rev 0.4 pinout; ignored with REV03) |

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

## Bring-up sequence

1. Configure: CRESET_B low with SS low, release, wait ≥ 1200 µs, send the bitstream, ≥ 49 extra
   clocks (see the design doc). Check CDONE.
2. Read ID (0x4D45) and BOARD from each board id you expect; BOARD bit 8 says the probe finished.
3. Set PWM_PERIOD, DEADTIME, duties (0x8000 = all legs at half the bus: no phase voltage).
4. Write ARM, check STATUS.LATCH_Q, then set CONTROL.PWM_EN. Arm before the DUT starts switching:
   waking the MP6539 charges its bootstraps with brief low-side pulses.
5. After any trip (FAULT, watchdog), PWM_EN is clear: re-arm and re-enable explicitly.
