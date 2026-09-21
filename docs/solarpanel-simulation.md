# Solar-panel simulation — bench, calibration and setup

The BenchPod emulates a solar panel by running its in-fabric control loop as a transfer
function: the loop reads the current the DUT is drawing and drives the DAC to the panel
voltage that a real panel would produce at that current. See
[dac-control-loop.md](dac-control-loop.md) for the loop itself; this document covers the
**external analog stage** that turns the pod's signal-level DAC output into something that can
actually source current, and the **measured constants** that describe it.

> **These numbers describe ONE specific side-device.** They are not baked into the firmware or
> the gateware, and they must not be. See [Where the numbers live](#where-the-numbers-live).

---

## 1. Why an external stage is needed

The pod's analog output is an OPA2992-buffered signal output — tens of milliamps. A panel
emulator has to source real load current, so the DAC drives an op-amp/MOSFET stage that
provides the current while the pod keeps control of the voltage.

```
   pod DAC (5V path) ──► op-amp ──► MOSFET ──┬── [SHUNT 0.044 Ω] ──┬── [LOAD 5 Ω 10 W] ── PSU −
                            ▲                │                     │
                            │            IN+ ┴                 IN− ┴          (Kelvin taps)
                            │                    INA282, G = 50 V/V
                            │                         │
                            └──── feedback ───────────┼──► pod ADC (front SMA, `ext`)
                                 (LOAD side of the    │
                                  shunt — see §5)     │
```

* **9 V PSU** supplies the op-amp/MOSFET stage and the INA282.
* The pod runs on **its own supply**. Only **ground** is common. (§6 explains why.)
* The op-amp's feedback is taken at the **load side of the shunt**, so the shunt drop is inside
  the loop and the DUT sees a regulated panel voltage.

---

## 2. Measured constants

Measured 2026-07-29 on pod `192.168.1.215`, gateware v29, with the stage above.

> **Still valid on gateware v31 / the fixed `adc_read`** (checked 2026-07-30). The fit spans
> 40.4 → 995.4 mA, i.e. sense 82 → 2193 mV, whose counts (≈65460 → 63357) all sit on one side of
> the 16-bit wrap — so the old and new arithmetic agree exactly there and the slope/intercept do
> not move.
>
> **Re-measured 2026-07-30** on the fixed `adc_read` (32 points, 1.4 → 995.4 mA, 3 reads each,
> cal1 gate passed): slope **2.1925** mV/mA, intercept **−6.965** mV, R² **0.999986**, implied
> shunt 43.85 mΩ. That is within 0.32 % / 0.58 mV of the values above — **the recorded fit was
> never corrupted**, and the slope needs no rework (the 0.32 % is inside the uncertainty of
> assuming R_load = 5.000 Ω exactly, and the DMM shunt cross-check below backs the recorded
> slope in a way the re-measure cannot). Only the intercept was refreshed, in the saved preset.

### Sense chain — ADC volts per amp of load current

```
sense_mV = −6.39 + 2.1996 × I(mA)
```

| quantity | value |
|---|---|
| slope | **2.1996 mV/mA** |
| intercept | **−6.39 mV** |
| R² | **0.999981** (25 points, 40.4 → 995.4 mA) |
| RMS residual | 2.7 mV — 0.12 % of full scale |
| worst residual | −6.0 mV at 839 mA — 0.27 % of full scale |
| implied shunt | **43.99 mΩ** (= slope / 50) |
| full scale | **2193 mV at 1000 mA** |
| zero-current output | see the note below — **do not read the intercept off it** |

> **The zero-current reading is not the intercept.** An earlier revision recorded it as
> −3 … −8 mV and called it agreement; measured on 2026-07-30 with the fixed `adc_read` it is
> **+5.7 mV**, and the sign never mattered. At zero command the DAC still sits at ~7 mV, so the
> op-amp/MOSFET stage is barely conducting and `I = V_dac / R_load` stops describing it: that
> point misses the fitted line by **+9.6 mV**, while every point from 9 mA up sits within
> +0.1 … +2.3 mV of it. Take the intercept from the FIT over the working range, never from a
> single reading at zero drive. (The old −3 … −8 mV was doubly untrustworthy: it was also taken
> a few counts from the 16-bit wrap, in exactly the regime the pre-v31 averaging got wrong.)

Independent DMM cross-check, directly across the shunt:

| load current | shunt drop | implied R |
|---:|---:|---:|
| 200 mA | 9 mV | 45 mΩ |
| 800 mA | 36 mV | 45 mΩ |

Two methods agreeing to 2 % on the shunt, and a fit linear to a tenth of a percent.

### Load stage — current per DAC volt

```
I = V_dac / 5.00 Ω
```

Exact, because the op-amp regulates the load voltage to the commanded DAC voltage (feedback
at the load side of the shunt). Verified at 200 mA and 800 mA.

### Step response

Measured with `measure` (square 200 Hz, 2048 samples at 400 kSPS), which co-starts the DAC and
the ADC DMA so the capture is phase-locked to the edge.

| quantity | value |
|---|---|
| 10–90 % rise | **≤ 7.5 µs** — 3 samples, i.e. at the 400 kSPS resolution limit |
| overshoot | ~10 % (peaks 3292 mV, settles ~2982 mV) |
| settling | ~15 µs |
| single-pole equivalent τ | 3.4 µs |
| **f₃dB** | **≈ 47 kHz** |
| peak transient current on a full-scale step | **1.5 A** (vs 1 A steady state) |

That last row is the justification for the loop's `in_trip`: a fast step into this plant
overshoots 50 % in current before settling.

### Raw sweep data

DAC on the 5 V path, `adc_read source=ext`, 3 reads averaged per point. `spread` is
max−min of those 3 reads. The DAC 0.0 V row is omitted — it was a single corrupted sample
(see §7).

| DAC (V) | DAC cal (mV) | I (mA) | sense (mV) | ADC count | spread (mV) |
|---:|---:|---:|---:|---:|---:|
| 0.2 | 202 | 40.4 | 85.0 | 65457 | 5 |
| 0.4 | 397 | 79.4 | 169.7 | 65373 | 5 |
| 0.6 | 592 | 118.4 | 256.0 | 65287 | 2 |
| 0.8 | 806 | 161.2 | 350.3 | 65193 | 1 |
| 1.0 | 1001 | 200.2 | 435.0 | 65109 | 5 |
| 1.2 | 1196 | 239.2 | 520.0 | 65024 | 3 |
| 1.4 | 1391 | 278.2 | 607.0 | 64937 | 4 |
| 1.6 | 1606 | 321.2 | 696.7 | 64848 | 7 |
| 1.8 | 1800 | 360.0 | 784.7 | 64760 | 3 |
| 2.0 | 1995 | 399.0 | 865.3 | 64680 | 11 |
| 2.2 | 2210 | 442.0 | 962.7 | 64583 | 10 |
| 2.4 | 2405 | 481.0 | 1050.0 | 64497 | 5 |
| 2.6 | 2599 | 519.8 | 1139.7 | 64407 | 2 |
| 2.8 | 2794 | 558.8 | 1222.3 | 64324 | 4 |
| 3.0 | 3009 | 601.8 | 1312.3 | 64235 | 5 |
| 3.2 | 3204 | 640.8 | 1404.7 | 64143 | 1 |
| 3.4 | 3398 | 679.6 | 1490.3 | 64057 | 1 |
| 3.6 | 3593 | 718.6 | 1573.7 | 63974 | 3 |
| 3.8 | 3808 | 761.6 | 1669.3 | 63879 | 7 |
| 4.0 | 4003 | 800.6 | 1754.7 | 63794 | 4 |
| 4.2 | 4197 | 839.4 | 1834.0 | 63715 | 6 |
| 4.4 | 4392 | 878.4 | 1928.7 | 63620 | 2 |
| 4.6 | 4607 | 921.4 | 2020.3 | 63529 | 2 |
| 4.8 | 4802 | 960.4 | 2109.3 | 63440 | 3 |
| 5.0 | 4977 | 995.4 | 2186.7 | 63363 | 8 |

---

## 3. What to enter in the UI

Generator tab → **Control loop** → the **Loop input** field group:

| field | value |
|---|---|
| Unit | `mA` |
| mV at zero | `0` (use `-6.4` for the last 0.3 %) |
| mV per mA | **`2.2`** |
| Range min | `0` |
| Range max | `1000` |

With those set, the loop resolves the range into **2184 ADC counts**, which the gateware's
input conditioner maps across all 2047 curve entries (gain ≈ −30712 Q15) — **≈ 0.49 mA per
curve entry**, one ADC LSB, the hardware floor.

Panel curve suggestion for the simulation:

| field | value | note |
|---|---|---|
| Transfer curve | Droop with knee (I-V) | |
| Curve max output | `4.0` V | Voc |
| Curve knee | `10` | fill factor 0.715 — panel-realistic. Knee 6 gives FF 0.62, a shaded panel |
| Clamp min / max | `0` / `4.0` V | |
| Update divider | `1900` | see §4 |
| Loop response | `25` % | |

At knee 10 the maximum-power point sits at Imp = 0.787·Isc, Vmp = 0.909·Voc — so for
Voc 4.0 V and Isc 1000 mA: **787 mA at 3.64 V, Pmax 2.86 W**.

---

## 4. Loop tuning

The plant is **47 kHz** with ~10 % overshoot. A control loop must be comfortably slower than
the plant it drives.

| `tick_div` | loop bandwidth (at 25 % response) | verdict |
|---:|---|---|
| 64 (default) | ~27 kHz | **too fast** — same order as the plant |
| 1900 | ~1 kHz | **start here** |
| 380 | ~5 kHz | a decade below the plant; only after 1900 proves stable |

Start at 1900. The plant already overshoots 10 % open-loop, and wrapping a fast outer loop
around a marginally-damped plant is how you get oscillation. Speed up only once it is stable.

> An earlier estimate of 1900 was derived from the INA282's nominal 10 kHz bandwidth. The
> measured chain is far faster than that; 1900 remains the right starting point, but for the
> overshoot reason, not the bandwidth reason.

---

## 5. Build requirements

These are not optional — each was a real failure during bring-up (§6).

1. **The high-current loop must not run through a breadboard.** Shunt, load and the return go
   on perfboard or soldered wire. Breadboard contacts are 10–50 mΩ, against a 44 mΩ shunt.
2. **Kelvin taps.** IN+ and IN− solder directly to the shunt's own terminals, on their own
   wires carrying no load current. Contact resistance in the *sense* wires is harmless (the
   INA draws microamps) — it is resistance in the *current* path **between** the taps that
   gets measured as if it were shunt.
3. **The INA282 runs on 9 V, not 3.3 V.** Full-scale output is 2.2 V; on a 3.15 V rail the
   part saturates. It is rated to 18 V.
4. **Feedback at the load side of the shunt**, so the op-amp corrects the shunt drop. Tapped
   before the shunt you get `I × R_shunt` of droop — 44 mV at 1 A — landing right on the knee.
5. **The pod gets its own supply. Only ground is common.** A shared rail couples the load's
   transients into the instrument.
6. Header pins and Dupont jumpers are fine for the sense pair, feedback, INA V+ and its
   ground reference — all microamp-level. Not for the load current.

---

## 6. Bring-up fault ladder

Every one of these was live on the bench, in this order. Recorded because each produced a
*plausible-looking but wrong* reading rather than an obvious failure.

| # | fault | signature |
|---|---|---|
| 1 | Ground offset on the breadboard | 19 mV of offset with zero current, against a 44 mV full-scale signal |
| 2 | INA V+ on the 3.3 V rail | needed 3.5 V out, had 3.15 V — saturated; output peaked then fell |
| 3 | Pod sharing the load's PSU | brownout; **the iCE40 lost its configuration** |
| 4 | Feedback tapped before the shunt | loop resistance read 6.2 Ω instead of 5.0 Ω |
| 5 | **INA282 dead** | drew 50 mA (spec < 900 µA); output tracked common mode, not differential |

Fault 5 was the stubborn one. The proof was that **swapping IN+ and IN− changed nothing** — a
working differential amplifier must invert when its inputs are reversed. It didn't, so the
differential was not reaching the amplifier. Separately, the input quadrupled (9 → 36 mV)
while the output *fell*. A new part fixed it immediately.

Recovery note for fault 3: **only an `fpga_image` reflash recovers a lost iCE40
configuration.** It drives CRESET and reconfigures from flash. A software reboot — including
`psram_recover` — leaves the corrupt configuration in place, so it cannot fix this.

---

## 7. Reproducing the calibration

Scripts used are throwaway; the procedure is what matters. All of it returns the DAC to 0 V and
unroutes it on every exit path.

1. **Sweep.** DAC 0 → 5 V on the 5 V path in 0.2 V steps; at each point `adc_read source=ext`
   ×3 with ~0.35 s settle. Record mean and spread.
2. **Sanity gates.** Spread should be single-digit millivolts. `cal1` (internal DAC→ADC
   loopback) is the control that proves the ADC and its calibration good — it should read the
   DAC back to within a few mV across 0–4 V.
3. **Fit** `sense_mV = a + b·I` with `I = V_dac / R_load`. Require R² > 0.999.
4. **Cross-check the shunt** with a DMM directly across it at two currents. `R = V/I` must
   agree at both.
5. **Step response** via `measure` with a square wave, 400 kSPS.

Gotchas:

* `adc_read source=ext` does **not** disturb the DAC mux, so interleaving set/read is valid.
* `capture`'s chunked reply desyncs a plain request/response socket — don't mix it with
  `adc_read` on one connection.
* Subtracting two ~2 V DMM readings to obtain a tens-of-millivolts differential is ±10 mV and
  misleads. Measure **across** the shunt.
* ~~**`measure` leaves the single-shot ADC probe wedged**~~ — **FIXED 2026-07-30, firmware +
  gateware v31. No workaround is needed any more, and the ordering advice it carried ("do the
  step response last, or re-verify the probe after it") no longer applies.**
  The reading was never a wedged probe (`adc_read` does not use `ADC_PROBE` at all — it runs a
  real 16-sample PSRAM capture through the same datapath as `capture`). The cause was
  arithmetic: `handle_adc_read` averaged the 16 **raw** counts and applied the bipolar unwrap
  only to the **average**. Because the front end is inverting, 0 V sits at count ≈ 65538 —
  just past the 16-bit top — so this bench's zero-current point (§2: −3 … −8 mV) sits a handful
  of counts from the wrap. One sample landing on the far side moved the raw mean to mid-scale,
  which was then unwrapped as a single huge negative reading: plausible millivolts, wrong by
  tens of volts, quantised in 65536/16 = 4096-count steps — which is where the `0x5555`/`0xAAAA`
  look came from. `capture` was unaffected because it returns raw counts that the server
  unwraps *per sample*. `measure` was implicated only because it moves the analog operating
  point across that boundary. Fix: `adc_scale_burst()` averages on the 16-bit circle and
  unwraps once, afterwards, for **every** source. Verified on this pod: `adc_read` now tracks a
  per-sample-scaled `capture` to within 1 mV across the full range, including across the wrap.

* **`adc_read` can now return an error instead of a number.** A burst wider than ~1024 counts
  means the input is still moving and has no single voltage, so it is refused (the reply names
  the measured spread) rather than averaged into something plausible. Every reply also carries
  `span`, the burst's peak-to-peak in counts — a free per-point noise gate for step 2's sanity
  check. At the sweep's settled DC points this never fires; it fires if the DAC is still
  slewing, so keep the ~0.35 s settle.

---

## Where the numbers live

**Nothing in this document is compiled into the firmware or the gateware, and it must stay that
way.** The stage is a side-device, not part of the pod.

| number | owner | why |
|---|---|---|
| `mv_at_zero`, `mv_per_unit`, `min`, `max`, `trip` | **the host** — sent per-arm in `dac_control_loop` | describes the *bench*, which the pod cannot know |
| ADC front-end fit (`ADC_CAL_EXT`) | firmware `cal_data.h` | a property of the pod itself |
| `in_zero`, `in_gain`, `in_trip` (gateware registers) | **derived** by `dac_loop_inmap_derive()` | composes the two above; see below |

The firmware derives the gateware's register values because it is the only place that holds
both the host's bench description *and* the pod's own ADC calibration. That keeps a client from
carrying a copy of the calibration that could drift from the board's. The side-device constants
travel as command parameters (`in_mv_per_unit` and friends) and are stored per device in the
org-owned `wiring` blob — never in a build artifact.

The webapp's `DEFAULT_LOOP_INPUT` carries the *nominal* design value (0.04 Ω × 50 = 2.0 mV/mA),
which is what someone computes from a fresh build. The **measured** 2.2 belongs in the saved
per-device configuration, not in the default — keeping "what it should be" and "what this one
is" distinct.

### Saving it — presets

Because these constants must be re-supplied from the host on every arm, the Control loop tab
has a **Preset** bar: name a configuration, press Save, and the curve, clamp, response, tick
rate and the whole input map are stored server-side and shared with your organisation. Saving
over an existing name replaces it.

* Table `app.benchpod_loop_presets`, org-scoped for real users and owner-scoped for guests,
  same shape as the waveform library.
* `config` is an opaque JSON object owned by the UI, so the loop can gain fields without a
  server migration.
* Loading a preset writes the input map back into the device's org-shared `wiring` blob — that
  is where the loop reads it from — and the rest into the tab's own state.
* Named "preset" rather than "scenario" on purpose: `webapp/src/testScenario/` is a different
  feature (the DUT simulation runtime with variables and rules).

For this bench, save one called something like `Solar 4V/1A` with the §3 values in it, and none
of the numbers in this document need typing again.

If the stage is ever built onto the pod or shipped as a module, its calibration becomes a pod
property and can move into `cal_data.h` alongside `ADC_CAL_EXT`. Until then it stays host-side.
