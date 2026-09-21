# Unified tri-capture: DAC + ADC + LA concurrent on one PSRAM master

Status: **DESIGN** (2026-07-20). Supersedes the two-master + `psram_bus_arbiter`
split (v18) for the concurrency path.

## Goal

Run **any subset** of the three deep streams at the same time, all through the one
APS6404L quad bus, sharing the 8 MB:

- **DAC** deep replay  — PSRAM → DAC8551 (a *read* stream)
- **ADC** capture      — MCP33131 → PSRAM (a *write* stream)
- **LA**  capture      — 12-ch logic → PSRAM (a *write* stream)

Capacity is shared: one active system can use (nearly) the whole 8 MB; two split it;
three split it three ways — but **not** as fixed halves/thirds. Firmware packs each
active stream a zone sized to *its own requested depth* and only requires the sum to
fit. It rejects an over-budget request **before** anything is sent to the iCE40.

## Why one path (not three)

The up5k is ~86 % full and seed-fragile (yosys 0.65 `-abc9`,
[ice40-lut-reduction](../ice40-v19-lc-reduction.md)). Three separate PSRAM datapaths
would not fit. Today there are already only **two** masters —
`psram_dual_writer` (ADC+LA, one write master, per-burst region round-robin) and
`dac_psram_reader` (DAC read) — joined by `psram_bus_arbiter`. This design collapses
those into **one** `psram_tri_master`: a single burst FSM, a single clk48 output
serializer, a single set of PSRAM pad drivers. A disabled channel is simply a job
with **length 0**, so it never wins arbitration — no separate path, no extra pads,
no top-level arbiter. Net LC target: **≤** the current reader + writer + arbiter.

## Memory model — dynamic contiguous zones

The three zones are packed contiguously into `[0, PSRAM_TOTAL)` with **runtime**
bases. This is the one hard change from v18, where LA/ADC bases were compile-time
params and only the DAC base was runtime.

```
 0 ┌──────────────┐
   │ LA   zone    │  la_base = 0,        la_len  = la_samples  * 2
   ├──────────────┤
   │ ADC  zone    │  adc_base = la_len,  adc_len = adc_samples * 2
   ├──────────────┤
   │ DAC  zone    │  dac_base = adc_base+adc_len, dac_len = dac_bytes
   ├──────────────┤
   │ (unused)     │
8M └──────────────┘
```

Order LA → ADC → DAC is a convention (write streams first, the pre-staged read
stream last); any packing order works as long as zones are disjoint. An **absent**
stream contributes length 0 and is skipped, so its neighbours abut with no gap.

**Guard:** keep a small `PSRAM_GUARD` (e.g. 4 KB) unallocated at the very top so a
final in-flight write burst can never run off the end of the chip.

### DAC must be staged first
The DAC waveform is *written by the STM32* into `dac_base` over XSPI **before** the
capture arms (the iCE40 owns the bus only during the capture). So the allocator must
run, assign `dac_base`, the MCU stages the waveform there, then the capture arms with
all three bases+lengths. `dac_base` is therefore computed by the same allocator, not
top-anchored as in v18.

## Firmware pre-flight: two independent checks

A request is **`{dac_bytes, adc_samples, adc_rate_hz, la_samples, la_rate_hz}`** with
zeros for absent streams. Reject with a specific error if either check fails.

### 1. Capacity (memory fit)
```
need = la_samples*2 + adc_samples*2 + dac_bytes
if (need > PSRAM_TOTAL - PSRAM_GUARD) -> error "capture too deep: <need> B > <avail> B"
```
Because packing is flexible this is the *only* capacity rule — there is no per-stream
cap except the whole. (The equal-split "50 % / 33 %" is just the symmetric special
case of this inequality when depths are equal.)

### 2. Bandwidth (bus can sustain the live rates)
Memory fit is necessary but **not sufficient**: the write streams are generated live,
so their *sustained* byte rate must not exceed what the shared bus can drain, or a
staging FIFO overflows → `CAP_OVF`. (The DAC is a read and self-paced by its own
prefetch FIFO; it contributes bus load but cannot "overflow" — if starved it would
underrun, which the same headroom check covers.)

```
load = adc_rate_hz*2 + la_rate_hz*2 + dac_rate_hz*2          (bytes/s on the bus)
if (load > PSRAM_SUSTAINED_BPS) -> error "combined sample rate exceeds bus bandwidth"
```

`PSRAM_SUSTAINED_BPS` is the *effective* drain after per-burst overhead
(CS lead + 0x38/0xEB command + 24-bit address + read dummy + CS-high tCEM gap), **not**
the 48 Mnib/s raw. It must be **characterised on hardware** and stored as a
conservative constant (see "Open / HW-characterise"). First-order estimate for a
CHUNK=16 B write burst: payload 32 nib / (32 + ~10 overhead) ≈ 0.75 → ~36 MB/s raw ×
margin. LA at 12 MS/s already needs 24 MB/s, so the three-way headroom is real and
this check will bite — that is the point.

## Gateware: `psram_tri_master.v`

One FSM on `clk` (24 MHz), output serialized on `clk48` (the proven
1:2 gearbox + DDR SCLK). Three **jobs**, each with a
runtime `{base, len, running_addr, remaining}` and a staging/prefetch FIFO:

| job | dir   | FIFO                    | QPI cmd | needs service when            |
|-----|-------|-------------------------|---------|-------------------------------|
| LA  | write | 32 B staging (from ring)| 0x38    | staging FIFO ≥ threshold      |
| ADC | write | 32 B staging            | 0x38    | staging FIFO ≥ threshold      |
| DAC | read  | 256 B dual-clock prefetch| 0xEB   | prefetch FIFO has room, len>0 |

**Burst loop** (identical framing for read & write — the shared structure that makes
the merge cheap): `S_IDLE → S_LEAD(CS low) → S_CMD → S_ADDR → [S_DUMMY, read only] →
S_DATA → S_CSH(CS-high tCEM gap) → S_IDLE`. Only three things vary by job:
1. command byte (0x38 vs 0xEB) and whether `S_DUMMY` is present,
2. the DATA phase drives (write) vs releases+samples (read) the IO pads,
3. the running address / FIFO port used.

**Arbitration** folds into `S_IDLE`'s pick: round-robin among jobs currently *needing
service* AND *not length-exhausted*. Length backstop: when a job's `remaining` hits 0
it stops winning (writes) or wraps to base (DAC read loops) — a hard HW guarantee
against overrun even if firmware math is wrong.

**Read vs write output rate.** Writes drive the pads and go fast (1 nibble / clk48).
Reads must sample PSRAM-driven data with setup margin, so a read nibble is *held for N
clk48 cycles* (parameter, N≈4 → ~12 MHz SCLK, matching today's robust reader) and
sampled mid-window. This keeps one serializer while preserving each direction's timing
character. **This is the main timing risk to validate in sim + on HW.**

**CDC** is unchanged in spirit: FSM + staging FIFOs on `clk`; DAC prefetch FIFO is
dual-clock (written `clk`, popped `clk48` by `dac8551_engine`); the clk→clk48 hop is
the phase-locked 1:2 gearbox, not the async class that caused the 0x5555 bug.

### What is removed
`dac_psram_reader.v`, `psram_dual_writer.v`, `psram_bus_arbiter.v` and the top-level
pad mux on `rd_busy` all collapse into `psram_tri_master` + a single pad block. The
DAC prefetch dual-clock FIFO and the DAC8551 engine are retained (the reader's FIFO
moves inside the tri-master).

## Protocol (tools/gen_protocol.py — authoritative)

- New opcode **`CAPTURE_TRI` 0x32**: `la_base(3)+la_cnt(3)+la_div(2) +
  adc_base(3)+adc_cnt(3)+adc_div(2)`. One trigger arms both write streams at shared t0
  with their runtime bases. (DAC base+len still arrive via `START_DAC_PSRAM` 0x13,
  now with the allocator-chosen base instead of top-anchored.)
- `SET_DAC_STOP_AFTER` 0x14 (capture-tied DAC auto-stop, v21) works unchanged across
  all three.
- Bump `GATEWARE_VERSION` (→ 22). `CAPTURE` 0x31 / `START_CAPTURE` 0x20 / `LA_CAPTURE`
  0x69 stay for back-compat but are re-expressed internally as tri-master jobs with the
  other two lengths = 0 (single/dual capture = the N=1 / N=2 case of the same path).
- `fpga_config.h`: `PSRAM_ADC_BASE`/`LA_BASE` stop being fixed; they become
  allocator outputs. Keep `PSRAM_TOTAL`, add `PSRAM_GUARD`, `PSRAM_SUSTAINED_BPS`.

## Read-back
After `CAP_DONE`, the STM32 takes the bus and reads each present zone from its
`{base,len}` over XSPI (already how dual capture streams back both regions). Three
zones instead of two; DAC zone is not read back (it was staged by the host).

## Surfaces
- **Python SDK / server / webapp**: a combined-capture entry point; advertise the
  allocator result (assigned zones + max depths given the current mix) and surface the
  two rejection reasons distinctly (too-deep vs too-fast).

## Open / HW-characterise
1. `PSRAM_SUSTAINED_BPS` — measure real drain at CHUNK=16 with all three contending;
   pick a conservative constant with margin. Until measured, gate the three-way case.
2. clk48 timing of the read-nibble-hold serializer on the seed-fragile up5k
   (sweep seeds; confirm ≥48 MHz closes).
3. LC delta vs reader+writer+arbiter — must be net-neutral-or-better; if not, trim.
4. Round-robin fairness with 3 contenders + LA the hog — confirm no FIFO breach in the
   arbiter testbench at the max rates the bandwidth check permits.
