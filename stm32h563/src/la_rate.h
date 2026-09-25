#ifndef LA_RATE_H
#define LA_RATE_H
#include <stdint.h>

/* Deep-LA (spram_ring16) burst-buffer rate planning — the pure, host-testable core
 * of fpga_la_capture_psram_start().  Extracted so test/test_la_rate.c can pin the
 * exact (samples, rate) -> divider mapping and catch regressions like the shipped
 * "LA stuck at 0.3 MS/s" bug (the divider was computed via the ADC's min-divider
 * floor, which the LA must NOT inherit).  See ice40/src/spram_ring16.v. */

/* Capture-divider semantics.  On the wire (START_CAPTURE / START_MEASURE cap_div /
 * CAPTURE / LA_CAPTURE) a capture divider is the sample PERIOD in 24 MHz clocks, so the
 * rate is exactly 24 MHz / divider.  Gateware >= EXACT_CAP_DIVIDER_MIN_GW implements
 * that; older gateware sampled every divider + 1 clocks (LA: the hi-byte write cycle
 * didn't count; ADC: its registered period flag added a cycle), so the real rate was
 * d/(d+1) of the reported one — 8% slow at 2 MS/s, measured on the bench against a DUT
 * UART.  Everything in firmware works in PERIODS; cap_divider_wire() is the one place
 * that encodes a period for the connected gateware. */
#define EXACT_CAP_DIVIDER_MIN_GW  32u

/* Single source of truth for the burst-buffer sizing.  Retuned 2026-07-08 for the
 * 48 MHz DDR PSRAM drain (docs/task8-48mhz-ddr-drain.md, GATEWARE_VERSION 11).  The DDR
 * writer is fast (~24 MB/s), but the SUSTAINED deep-capture rate is bounded by the
 * single-port spram_ring16, not the writer (see LA_PSRAM_DRAIN_HZ below): short bursts
 * that fit the ring run at the full 12 MS/s ceiling, while captures larger than the ring
 * are burst-capped to the fastest rate whose peak occupancy still fits. */
#define LA_PSRAM_MIN_DIV    2u          /* 24 MHz / 2 = 12 MS/s peak (la_psram_capture period >= 2) */
#define LA_RING_BYTES       65536u      /* spram_ring16 depth (ice40 la_ring_i, AW=15: 32K words -> 64 KB) */
/* SUSTAINED deep-capture drain is bounded by the single-port spram_ring16, NOT the
 * 24 MB/s DDR writer: each sample costs 1 write + a 3-cycle read on the shared SPRAM
 * port, and a read can only start in the sampler's idle gap.  At the 12 MS/s peak there
 * is no gap at all, so the ring only BUFFERS (<= LA_RING_BYTES/2 samples, drained after
 * the burst).
 *
 * All rates here are REAL.  The HW tuning (65535 samples at "12 MS/s" fits the ring
 * lossless, 100000 overflows; the TestHW_RawLA_Long depths pass) was done on gateware
 * <= v31, whose divider 2 really sampled at 8 MS/s, not 12.  In real units that bench
 * data puts the ring's sustained drain between ~4.0 and ~5.4 MS/s, so the old 4 MS/s
 * estimate — which read as "conservative" against a mislabelled ~6 MS/s — actually sat
 * at the bottom edge of that range. */
#define LA_PSRAM_DRAIN_HZ   3000000.0f  /* sustained ring-limited supply (S/s, real); >= 25% below the
     * measured ~4.0-5.4 MS/s.  It also reproduces the exact real rates the deep captures were
     * HW-verified at on v31 (65536 -> 6 MS/s, 131072 -> 4, 262144 -> 3.43), so no deep capture
     * runs faster than it has been proven lossless.  <=32K-sample bursts still run at 12 MS/s. */

typedef struct {
    uint32_t divider;   /* sample period in 24 MHz clocks (>= LA_PSRAM_MIN_DIV); encode with cap_divider_wire() */
    uint32_t rate_hz;   /* actual rate the divider yields = cap_hz / divider */
    uint8_t  capped;    /* 1 = burst budget LOWERED the rate so the capture fits the ring */
    uint8_t  floored;   /* 1 = request exceeded the 12 MS/s peak, clamped up to MIN_DIV */
} la_plan_t;

/* Pick the LA capture divider for `samples` at `req_rate_hz`, against a `cap_hz`
 * capture clock (24 MHz).  Two limits, both handled here:
 *   - burst depth: if the capture is bigger than the ring can hold at that rate, the
 *     rate is lowered so its peak occupancy fits (no dropped samples).
 *   - peak rate: divider is floored at LA_PSRAM_MIN_DIV (the 12 MS/s sampler ceiling).
 * Deliberately does NOT apply the ADC's conversion-time floor — the LA has none. */
la_plan_t la_psram_plan(uint32_t cap_hz, uint32_t samples, float req_rate_hz);

/* Encode a capture sample PERIOD (24 MHz clocks) as the wire divider for gateware
 * `gw_version`: the period itself on >= EXACT_CAP_DIVIDER_MIN_GW, period - 1 before
 * (where the engines added a clock).  Clamped to 16 bits.  ADC and LA alike; NOT the
 * DAC divider, whose sequencer overhead firmware already models (DAC_SEQ_OVERHEAD_CLK). */
uint16_t cap_divider_wire(uint32_t period, uint8_t gw_version);

/* Gateware >= RELOAD_WIRE_MIN_GW takes the RELOAD value its counter needs instead of doing
 * the compare + subtract itself (the v40 LC pass moved them into firmware): the LA divider is
 * period - 2, the DAC divider is divider - 1, the GPIO_STEP delay is half-phase us - 1.  Older
 * gateware takes the period / divider / delay as before.  All clamp to the 16-bit fields. */
#define RELOAD_WIRE_MIN_GW  40u

/* LA capture PERIOD (24 MHz clocks) -> wire divider.  v40+: period - 2, floored at 0 (the
 * sampler's 2-clock minimum).  Older: cap_divider_wire(). */
uint16_t la_divider_wire(uint32_t period, uint8_t gw_version);
/* DAC inter-sample divider (the firmware's DAC_SEQ_OVERHEAD_CLK model) -> wire.  v40+:
 * divider - 1 (0 stays 0; the gateware still floors the reload at 2, a DAC8551 t9 limit). */
uint16_t dac_divider_wire(uint32_t divider, uint8_t gw_version);
/* GPIO_STEP half-phase in us (>= 1) -> wire.  v40+: delay - 1. */
uint16_t step_delay_wire(uint32_t delay_us, uint8_t gw_version);

#endif /* LA_RATE_H */
