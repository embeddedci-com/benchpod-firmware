# Synchronized capture across two pods

Status: design only, nothing built (2026-10-28). Written down so it is ready when a user needs
it: recording a current loop and a voltage loop at the same time, for example, takes two pods,
because one pod has a single ADC (`ext` and `current_in` are positions of one mux). Chosen direction: **option B** below (keep each pod's own clock, lock the sample timing to a
reference from the other pod). The software method (option 0) works today and is the fallback.

## The problem

Each pod samples on its own clock. X1 (SX3M48.000B10F20TNN, 48 MHz, ±10 ppm at 25 °C, ±20 ppm
over temperature) drives iCE40 pin 44 (`IOB_3b_G6`); `clk48` and `clk` = clk48 / 2 (24 MHz) time
the ADC, DAC and LA. X1's tri-state pin is tied to +3V3 and no connector carries the clock off
the board, so two pods cannot share it. Two pods started on the same edge drift apart by up to
about 40 ppm: 40 µs per second of capture, 16 samples at 400 kS/s.

Two separate needs:

1. **Start together:** the same t = 0 on both pods.
2. **Stay together:** the same sample rate for the length of the capture.

## What the hardware already gives us (digital v3r3)

- **LA pins go straight to the iCE40** (top bank, powered from LA_VCCIO = the LA voltage), through
  a series resistor only. LA1 to LA8 also have an SN74LVC2G66 switch (pull resistors) on the line.
- **Two LA pins are global clock inputs:** LA6 = pin 35 (`IOT_46b_G0`), LA3 = pin 37
  (`IOT_45a_G1`). Any LA pin works as a plain input for a slow reference; these two can also
  feed the clock network.
- **Capture trigger** on an LA edge or level (gateware ≥ v35): both pods can start on one edge.
- **`gpio_pulse`** emits FPGA-timed pulses on an LA output: the master can send a reference.
- **`capture_correlated`** records ADC and LA in one capture on one clock, so a reference line
  lands in the same time base as the ADC samples.

## Option 0: shared trigger + marker pulses, host-side alignment (works today)

Wiring: one LA channel pod A ↔ the same or another LA channel on pod B, plus a ground wire. Same
LA voltage on both pods.

1. Pod A claims the channel as an output. Both pods arm a capture with a trigger on it.
2. Pod A sends a pulse train on it (for example every 1 ms): the first edge starts both captures.
3. Both pods use `capture_correlated`, so each capture carries the pulse edges next to its ADC
   samples.
4. The host matches the edges, fits offset and rate ratio, and resamples pod B onto pod A's time
   base.

Expected (not measured): start within one sample (2.5 µs at 400 kS/s, less if the ADC grid
restarts on the trigger; not checked), sub-µs alignment over the whole capture after the fit. No
firmware or gateware change; an SDK helper (`capture_synchronized(pods, ...)`) would hide the
30 or so lines it takes.

## Option B: lock each sample tick to a reference (chosen)

The slave keeps X1 as its clock. The master sends a slow reference on an LA pin; the slave's
gateware measures it against its own `clk` and corrects when each ADC conversion starts. The
average sample rate then follows the master exactly (no drift); the timing closure and the
clock network do not change.

### Reference

- A square wave on any LA pin, 3.3 V or 1.8 V (same LA voltage on both pods). Suggested 1 MHz:
  high enough for a fast lock, low enough for a jumper wire and the LA input path. 10 kHz to
  10 MHz should all work; a lab 10 MHz reference works the same way.
- Master side: a gateware output that divides `clk48` to the reference frequency on a chosen LA
  pin (`gpio_pulse` can stand in during development). The master then samples on its own clock
  as usual, so the master's sample ticks and the reference share one clock.

### Slave gateware

1. **Edge capture:** synchronize the reference input into `clk` (two flops), detect rising edges.
2. **Rate measurement:** count `clk` cycles over N reference periods (for example N = 1000 at
   1 MHz = 1 ms gate). The ratio to the nominal count is the frequency error, resolution
   1 / count per gate; averaging over gates gives better than 1 ppm within tens of ms.
3. **Sample tick from a phase accumulator:** today the ADC engine starts a conversion every
   `divider` `clk` cycles (60 → 400 kS/s, `adc_mcp33131.v`, `ADC_CAP_MIN_DIVIDER` in
   `signal_engine.c`). Replace the integer counter with an accumulator that adds a fractional
   increment per `clk`; a conversion starts on overflow. The increment comes from the measured
   rate, so the tick rate tracks the reference.
4. **Phase lock (optional second step):** also compare the tick phase to the reference edges and
   trim the increment, so the slave's ticks stay at a fixed phase to the master's, not only at
   the same rate. Rate lock alone already removes the drift; phase lock removes the remaining
   constant offset after the trigger.
5. **Start:** the existing LA trigger starts the capture; with phase lock the first tick lands at
   a fixed delay after the trigger edge.

### Timing quality

- The tick can only move in whole `clk` periods: **41.7 ns** (24 MHz). Corrections are rare: at a
  20 ppm difference the slave inserts or drops one `clk` about every 50,000 cycles (~2 ms), so
  roughly 1 in 800 samples at 400 kS/s is 41.7 ns early or late and the rest sit on a perfect
  grid. Over 2.5 µs that is 1.7 % of one sample period.
- On a fast signal that step is a small error: on a full-scale 10 kHz sine, 41.7 ns moves the
  reading by up to about 0.26 % of the amplitude, on the corrected sample only. If that matters,
  the gateware can flag corrected samples (one bit in the capture header or a sideband count) so
  the host can interpolate them.
- The DAC (on `clk48`, period = max(divider, 3) + 51 clocks) and the LA sampler can get the same
  accumulator if they must be locked as well; the ADC is the first need.

### Firmware, SDK, MCP, web

- Command (sketch): `{"cmd":"sync","role":"master","la":6,"freq_hz":1000000}` /
  `{"cmd":"sync","role":"slave","la":6,"freq_hz":1000000}` / `{"cmd":"sync","role":"off"}`;
  `status` reports `sync: {role, locked, ppm, la}`. The LA pin is owned by `sync` like GPIO owns
  pins (`la_pins` shows it, other uses refuse it).
- Capabilities flag `sync` from the gateware version that has it.
- SDK: `pod.sync_master(la=6)`, `pod.sync_slave(la=6)`, and a `capture_synchronized(pods, ...)`
  helper that arms both, triggers, and returns both captures on one time base.
- MCP tool and public docs in the same change (wiring: reference + ground, same LA voltage, LA6
  or LA3 lost to other uses while synced).

### Open questions before building

- **LC budget:** counter, accumulator and loop logic on an iCE40 that is close to full. Check the
  utilization of the image it would go into (loop or deep-replay, or both) first.
- Does the ADC conversion grid restart on a capture trigger today? (Decides the start offset
  without phase lock.)
- Input path of a 1 MHz square through the LA input (series resistor, LVC2G66 on LA1 to LA8) and
  a 20 to 50 cm jumper: check edge quality on the bench. Prefer LA9 to LA14 (no switch) if the
  switch shows up.
- Lock time and holdover: what the slave does when the reference disappears mid-capture (keep the
  last increment, flag it).

### Test plan

- Two analog pods, one signal generator sine into both ADC inputs (same cable length), reference
  LA master → slave, ground between the pods.
- Without sync: capture 10 s on both, measure the phase drift of the sine between pods (expect
  tens of µs).
- With rate lock: drift < 1 sample over 10 s; with phase lock: fixed offset < 100 ns.
- hwe2e test once a second analog pod is on the bench.

## Alternatives not chosen

- **A. Run the slave's whole FPGA from the master's 48 MHz on LA6 (G0).** Sample-exact, no
  correction steps. Needs placement and timing closure on a different clock pin with thin
  `clk48` margins, and the iCE40 has no glitch-free clock switch (a pulled cable stops the
  slave): it would be a separate "sync slave" gateware image. Only if sample-exact locking is
  ever required.
- **C. A 10 MHz lab reference through the iCE40 PLL to make 48 MHz.** As far as the PLL's divider
  ranges and minimum comparison frequency go, 10 MHz cannot give exactly 48 MHz. A 10 MHz
  reference still works through option B.
- **Clock in/out hardware on the digital board** (X1 enable to a GPIO, a 74LVC1G157-class mux
  in front of pin 44, buffered clock out, a U.FL or MMCX connector): about $1 to $2 a board.
  Not needed for option B; consider it for a later digital revision only if option A is.
- **A TCXO instead of X1** (48 MHz, CMOS output, 3225, for example HCI 8132H-Y48.000MK33DTL,
  ±2.5 ppm, about $1.60 more): better absolute timing (sample rates, generated frequencies, LA
  timing, baud rates), but two TCXOs still drift by µs per second. Separate decision, does not
  replace synchronization. The cheap ±0.5 ppm parts have clipped-sine outputs, not CMOS.
- **Network time (NTP/PTP):** milliseconds at best over the pod's network, not sample timing.
- **A dual simultaneous-sampling ADC on one analog module** (ADS8354 or AD7380 class, about $25
  to $70 more per module with the second input chain): the clean answer for current and voltage
  on one pod, a v4 or module-variant decision. Both ADC families can read both channels over
  one data line at a higher clock (to check per part): the module FFC has one ADC data line to
  the iCE40, and its spare line (MOD_SPARE) goes to the STM32 (PD13), not the FPGA.
