# In-fabric DAC control loop

A deterministic control loop that lives entirely in the iCE40: every control tick it takes an
**input**, looks the target output up in a reloadable **curve** (a tabulated transfer function),
damps toward it, clamps, and drives the DAC8551. No host in the control path, and no PSRAM — so a
logic-analyzer capture can run alongside it.

```
   input ──► curve LUT ──► damp (Q15 k) ──► clamp [vmin,vmax] ──► DAC
             2048 entries, indexed by input >> 5
```

> For a worked example with a real external analog stage — the current-sense bench, its measured
> calibration constants and the tuning that follows from them — see
> [solarpanel-simulation.md](solarpanel-simulation.md).

A solar-panel / MPPT emulator is one curve you can load (output falls from Voc toward 0 as the
load current rises). Nothing in the loop is panel-specific: load a flat curve, a ramp, a droop
law, a lookup table measured off a real device — whatever the DUT should see.

## Where the input comes from

`DAC_LOOP_SRC` (0x1A, gateware ≥ v29) selects the input; the curve, damping and clamp are
identical in all three. The source can change while the loop runs — the next tick uses it.

| `source` | Input | Loop |
|---|---|---|
| `"adc"` (default) | the live ADC | **closed** — the output reacts to the DUT |
| `"fixed"` | a host-held constant (`input`) | **open** — one point of the curve, ADC out of the path |
| `"sweep"` | an internal accumulator, `+step` each tick (mod 65536) | **open** — a function of time over the whole curve |

The open-loop sources exist because a closed loop is the wrong place to start debugging. With the
input held by the host, the DAC and the output stage are testable on their own: hold a point, read
the SMA with a meter, compare against the curve. If that number is wrong, no amount of loop
tuning was ever going to help — and if it is right, everything that remains is loop behaviour.

## Bring-up procedure (no ADC)

1. Pick a curve. `constant` is the bluntest check; the panel preset is the solar-panel simulation's.
2. Arm with `source="fixed"`, clamp wide open (`vmin=0, vmax=65535`) so the clamp cannot be what
   produced the value you read.
3. Step the input through a few points — 0%, 50%, 100% is enough — with
   `{"cmd":"dac_loop_input","input":N}`. No re-arm and no curve re-upload, which is what makes the
   readings comparable.
4. Meter the DAC/SMA output at each point and compare with `curve[input]`. The webapp shows the
   expected value next to the slider; the Python SDK has `curve_output_at()`, and the same
   arithmetic is in `curveOutputAt()` (webapp) and `loopCurveOutputAt()` (hwe2e).
5. Only then switch to `source="adc"` and close the loop.

Measured on pod 192.168.1.215 (2026-07-27, gateware v29), a 4.0 V Voc panel curve on the 5 V path,
read back through the pod's own DAC→ADC loopback (`analog_path cal1`):

| input | expected | probe | measured |
|---|---|---|---|
| 0 % | 4.000 V | 4.000 V | 3.998 V |
| 50 % | 3.936 V | 3.936 V | 3.935 V |
| 90 % | 1.846 V | 1.846 V | 1.848 V |
| 100 % | 0.000 V | 0.000 V | 0.011 V |

## Commands

```jsonc
// arm (curve is base64url of 16-bit LE points; the firmware upsamples to the 2048-entry LUT)
{"cmd":"dac_control_loop","k":32767,"vmin":0,"vmax":65535,"tick_div":64,
 "curve":"<b64url>","source":"fixed","input":0,"step":0}
 -> {"armed":true,"k":…,"vmin":…,"vmax":…,"tick_div":…,"curve_pts":2048,
     "source":"fixed","input":0,"step":0}

// retarget a RUNNING loop (omitted fields keep their value)
{"cmd":"dac_loop_input","input":32768}      -> {"source":"fixed","input":32768,"step":0,"v":…}

// telemetry
{"cmd":"dac_loop_probe"}                    -> {"i":<ADC>,"in":<loop input>,"source":"fixed","v":<DAC>}

// stop
{"cmd":"dac_stop"}
```

`in` is what the last tick actually indexed the curve with. **Plot and assert against `in`, not
`i`**: in a fixed/sweep run the ADC is not in the path at all, so `i` is a reading of an input
nothing is using. On pre-v29 gateware `in` is absent and the ADC *is* the input by construction.

## Things that are refused rather than quietly substituted

* `vmin > vmax` — the gateware clamps against `vmin` first, so an inverted window would pin the
  output at `vmin` and silently discard the ceiling the caller asked for.
* `source:"sweep"` with `step:0` — an input that never advances is a fixed point wearing a moving
  name.
* An unknown `source` name — falling back to the ADC would run a *closed* loop under an open-loop
  request, and the operator would be metering a value the loop never produced.
* Arming on the **deep-replay image**: both gateware images report the same version, so only
  `FPGA_FEATURES` bit0 tells them apart. Without that gate the loop reports "armed" and drives
  nothing. Switch with `{"cmd":"fpga_image","image":0}`.

`k` (>32767 would wrap to a *small* gain in `k[14:0]`) and `tick_div` (<8 outruns the pipelined
tick) are **clamped** instead, and the arm reply echoes the effective values.

## Where the pieces live

| Layer | Files |
|---|---|
| Gateware | `ice40/src/dac_loop.v`, `cmd_dispatch.v` (0x15/0x1A/0x16/0x1B), `top_v2.v`; sim `sim/tb_dac_loop.v` (`make looptest`) |
| Firmware | `stm32h563/src/dac_loop_params.{c,h}` (pure, host-tested), `signal_engine.c`, `command_handler.c` |
| Server | passthrough command channel; `cap.dac_control_loop` + `cap.dac_loop_sources` in `api/benchpod_ws.go` |
| Webapp | `src/controlLoopCurve.ts`, `src/pages/GeneratorTab.tsx` (Generator → Control loop), `src/components/TransferCurveCanvas.tsx` |
| Python SDK | `embeddedci/benchpod/control_loop.py`, `client.control_loop/loop_input/loop_probe` |
| HW e2e | `embeddedci-server/hwe2e/benchpod_control_loop_hw_test.go`, `benchpod_loop_source_hw_test.go`, `cloud_control_loop_test.go` |

The loop and the deep-DAC-PSRAM replay reader are near-identical in size and together overflow the
up5k, so they ship as two gateware images and the pod is reflashed to switch
(`{"cmd":"fpga_image","image":0|1}`, ~2–3 s). The webapp does that automatically when the Control
loop action is selected.
