#include "la_rate.h"
#include <math.h>

la_plan_t la_psram_plan(uint32_t cap_hz, uint32_t samples, float req_rate_hz)
{
    la_plan_t p = {0};
    /* req_rate <= 0 means "max rate" — resolve it to the PEAK (cap_hz / MIN_DIV) up
     * front, so the burst-budget check below still lowers it for a deep capture.
     * (Leaving it 0 would skip the cap and return MIN_DIV = full rate, overflowing the
     * ring on a deep max-rate capture — the bug the full-8 MB lastress caught.) */
    float rate = (req_rate_hz > 0.0f)
               ? req_rate_hz
               : ((float)cap_hz / (float)LA_PSRAM_MIN_DIV);

    /* Burst budget: a capture stores 2 bytes/sample.  Above the writer's sustained
     * drain the ring fills at the net rate, so the capture is only lossless while its
     * PEAK occupancy fits the ring:  samples*2*(1 - drain/rate) <= LA_RING_BYTES.
     * If the whole capture can't fit even at drain rate's headroom, LOWER the rate so
     * it does (a uniform, no-dropped-sample capture at the fastest rate that fits). */
    if (2.0f * (float)samples > (float)LA_RING_BYTES) {
        float max_rate = LA_PSRAM_DRAIN_HZ /
                         (1.0f - (float)LA_RING_BYTES / (2.0f * (float)samples));
        if (rate > max_rate) { rate = max_rate; p.capped = 1; }
    }

    /* Divide the capture clock DIRECTLY — the LA has no ADC-conversion-time floor.
     * Round the divider UP so the actual rate never EXCEEDS the (possibly burst-capped)
     * request: a rounded-nearest divider could overshoot a cap and overflow the ring. */
    uint32_t d = (rate > 0.0f)
               ? (uint32_t)ceilf((float)cap_hz / rate)
               : LA_PSRAM_MIN_DIV;
    if (d < LA_PSRAM_MIN_DIV) { d = LA_PSRAM_MIN_DIV; p.floored = 1; }
    if (d > 0xFFFFu) d = 0xFFFFu;

    p.divider = d;
    p.rate_hz = cap_hz / d;
    return p;
}

uint16_t cap_divider_wire(uint32_t period, uint8_t gw_version)
{
    if (period > 0xFFFFu) period = 0xFFFFu;
    if (gw_version >= EXACT_CAP_DIVIDER_MIN_GW) return (uint16_t)period;
    return (uint16_t)(period > 1u ? period - 1u : 0u);   /* old engines: divider + 1 clocks */
}

/* x - k, floored at 0 and clamped to 16 bits (the reload encodings below). */
static uint16_t reload16(uint32_t x, uint32_t k)
{
    uint32_t r = (x > k) ? x - k : 0u;
    return (uint16_t)(r > 0xFFFFu ? 0xFFFFu : r);
}

uint16_t la_divider_wire(uint32_t period, uint8_t gw_version)
{
    if (gw_version >= RELOAD_WIRE_MIN_GW) return reload16(period, 2u);
    return cap_divider_wire(period, gw_version);
}

uint16_t dac_divider_wire(uint32_t divider, uint8_t gw_version)
{
    if (gw_version >= RELOAD_WIRE_MIN_GW) return reload16(divider, 1u);
    return (uint16_t)(divider > 0xFFFFu ? 0xFFFFu : divider);
}

uint16_t step_delay_wire(uint32_t delay_us, uint8_t gw_version)
{
    if (gw_version >= RELOAD_WIRE_MIN_GW) return reload16(delay_us, 1u);
    return (uint16_t)(delay_us > 0xFFFFu ? 0xFFFFu : delay_us);
}
