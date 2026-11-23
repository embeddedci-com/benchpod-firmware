# SWIM master in the gateware (v62)

The STM8's one-wire debug interface (ST UM0470) needs timing the MCU cannot hold, so the iCE40
runs it: `ice40/src/swim_engine.v`, the SWD engine's fifth job after SPI, I2C and JTAG (the
JTAG notes are in [dap-over-tunnel.md](dap-over-tunnel.md#jtag)). The file header is the
reference; this page is the summary for firmware work. Status: gateware and simulation only
(`make -C ice40 swimtest`). There is no firmware protocol yet (it follows the move of the
`prog_flash` protocols to XIP) and no STM8 has been on the bench.

## Arming

`SPI_ARM` with flags bit 4 (`0x10`), deep image only (`FPGA_FEATURES` bit 4,
`FPGA_FEATURE_SWIM`; gateware >= `SWIM_MIN_GW`, 62). The SWIM pin is the channel in `mosi_ch`
and `miso_ch` (both the same channel); `sck_ch` and `cs_ch` are not driven. The engine only
ever pulls the line low or releases it. The line must rise through a pull-up: the pod's own
pull on that channel, or an external ~1 kohm to the chip's VDD. The chip's NRST is not the
engine's: the firmware drives it (the pod's NRST pin) around the entry sequence.

## Queue tokens

Tokens go in `SWD_FEED` like the other jobs' bytes. Bits 2:0 are the op.

| op | token | what it does | reply bytes |
|----|-------|--------------|-------------|
| 1 | `ENTRY` | SWIM low 16 us, then 8 pulses (4 at 1 kHz, 4 at 2 kHz, 50% duty), release, wait up to 0.5 ms for the chip's sync pulse and count it | 2: sync length L in clk (24 MHz), low byte then high nibble; both 0 if no sync came |
| 2 | `CMD` (bits 5:3 = command) | header, 3 command bits, parity, then the chip's ACK. SRST is command 0 | none |
| 3 | `TXB`, then the data byte | header, 8 data bits, parity, then the ACK | none |
| 4 | `RXB` | wait for a frame from the chip, check parity, ACK it (NACK on bad parity, the chip resends) | 1: the byte |
| 5 | `CFG`, then 3 bytes | byte 1: bit 7 high speed, bits 3:0 NACK retries (default 3); byte 2: timeout in 16-tick units (0 = 4096, default 32); byte 3: line-high wait in 4-tick units (0 = off when sending, default 64) | none |
| 6 | `REPORT` | | 3: ACK count, NACK count (both saturate at 255), flags |

ROTF and WOTF are composed from these tokens (a gateware macro did not fit the loop image):

- SRST: `CMD(0)`
- ROTF: `CMD(1) TXB(N) TXB(addr23:16) TXB(addr15:8) TXB(addr7:0) RXB x N` (N = 0 is 256)
- WOTF: `CMD(2) TXB(N) TXB(addr23:16) TXB(addr15:8) TXB(addr7:0) TXB(data) x N`

Queue the whole command in one feed: the chip starts a ROTF's first data frame soon after the
address is ACKed and the engine runs a token as soon as it has it, so a feed slower than the
wire could make it miss that first frame (a WOTF is host-driven and tolerates any pace).

## Timing

One tick is one SWIM clock of the chip. The engine counts the sync pulse (128 SWIM clocks low),
L clk, and a phase accumulator then ticks 128 times per L clk (so the SWIM clock in MHz is
3072 / L at 24 MHz). L is 384 (8 MHz) after arming, so a chip that is already in SWIM mode works
without a new `ENTRY`. A sync outside 192..1023 clk stops the queue.

| | 0 bit | 1 bit |
|--|-------|-------|
| low speed | 20 low + 2 high | 2 low + 20 high |
| high speed | 8 low + 2 high | 2 low + 8 high |

Device-to-host bits are sampled mid-bit, 11 ticks (5 at high speed) after the falling edge:
still low is a 0. After each low part the engine releases the line and waits until it reads
high (up to the CFG limit, default 256 ticks) before it counts the high part, so a weak pull-up
stretches the bit instead of merging two pulses. A line that does not rise stops the queue
(flag bit 3). The wait also guards the ACK: the engine only accepts a falling edge after the
line has read high.

High speed is switched on the chip by writing SWIM_CSR (0x007F80) with the HS bit; the firmware
then sends `CFG` with bit 7. The high-speed bit shapes follow UM0470 as read for this work and
need the bench to confirm (see below).

## Failures

A NACKed frame is sent (or asked for) again up to the retry count. Retries used up, no ACK or no
frame within the timeout, a line that stays low, or no sync stops the queue: the remaining tokens
stay queued, `q_done` says how far it got, and the engine appends the 3-byte `REPORT` record to
the reply buffer (`ENTRY` failures reply with the 2 sync bytes only). Re-arm to run again.

Flags (`REPORT` byte 3): bit 0 no ACK or no frame in time, bit 1 no sync, bit 2 sync out of range,
bit 3 line did not rise, bit 4 NACK retries used up, bit 5 a received frame had bad parity, bit 6
high speed on, bit 7 stopped. `SWD_QSTATUS` flags: bit 7 any of bits 3:0 (timeout), bit 6
`qptr[8]`, bit 5 NACK retries used up, bits 4:2 NACK count (saturating at 7), bit 1 stopped,
bit 0 busy.

## Simulation and what it does not prove

`ice40/sim/tb_swim.v` runs the engine against a behavioral STM8 on a wired-AND line with a
configurable rise time: entry (the model checks every pulse width within 3%), sync at 8, 6.4 and
9 MHz, SRST, ROTF and WOTF round trips, NACK retry, a bad-parity frame from the chip, retries
used up, no ACK, a stuck line, no sync, a short sync, high speed switched through SWIM_CSR, a
slow line with and without the wait, a slow feed (WOTF), a chip answering back to back. The model is
written from UM0470 as read for this work (header bits, bit shapes, the 22-clock slot, ACK as a
1 bit), not from silicon. Things the bench must confirm on an STM8S103 board: the entry waveform
(the 16 us hold before the pulses), how soon after a frame the chip sends its ACK, the gap before
a ROTF's first data frame, and high-speed margins. The values live in `swim_engine.v` (`HOLD_US`,
`P1_US`, `P2_US`, `SYNC_TMO_US`, the tick counts in `lowticks` / `hiticks` and the 11 / 5 sample
points).
