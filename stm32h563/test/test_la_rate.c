/* Host unit test for la_psram_plan() — the deep-LA capture rate/divider math — and
 * cap_divider_wire(), the capture-divider wire encoding.
 *
 * Regression-locks the behaviour verified on the v2 pod:
 *   - a requested rate maps to the RIGHT divider (6 MS/s -> 4, not the ADC's 80),
 *   - requests above the 12 MS/s sampler ceiling are clamped (floored),
 *   - captures bigger than the ring are burst-capped to a rate the ring sustains.
 * The "6 MS/s -> divider 4" case below is the exact bug that shipped and only
 * surfaced on hardware (la_capture ran at divider=80 = 0.3 MS/s for every rate).
 *
 * Retuned 2026-07-08 for the 48 MHz DDR drain (GATEWARE_VERSION 11): the sampler peak
 * is 12 MS/s (MIN_DIV 2) and the ring is 64 KB (AW=15).  Re-expressed 2026-09-11 in
 * REAL rates (GATEWARE_VERSION 32 made the divider an exact period; before, divider d
 * sampled every d+1 clocks): the sustained drain is bounded by the single-port
 * spram_ring16 (~4.0-5.4 MS/s real on HW), and LA_PSRAM_DRAIN_HZ is set CONSERVATIVELY
 * to 3 MS/s below that — so short bursts that fit the ring run at the full 12 MS/s,
 * while captures larger than the ring are burst-capped to the fastest rate whose peak
 * occupancy still fits (see la_rate.h).
 */
#include "la_rate.h"
#include <stdio.h>
#include <math.h>

#define HFOSC 24000000u   /* adc_capture_hz() on the v2 board */

static int fails = 0;
#define CHECK(cond, ...) do { if (!(cond)) { \
        printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

/* peak ring occupancy (bytes) of `samples` at the planned real rate */
static double peak_bytes(uint32_t samples, uint32_t rate_hz) {
    double peak = 2.0 * (double)samples * (1.0 - (double)LA_PSRAM_DRAIN_HZ / (double)rate_hz);
    return peak < 0.0 ? 0.0 : peak;   /* drain >= rate -> ring never accumulates */
}

int main(void) {
    /* ---- THE regression: small captures track the requested rate exactly ---- */
    la_plan_t p = la_psram_plan(HFOSC, 4096, 6.0e6f);
    CHECK(p.divider == 4, "6 MS/s x4096 divider=%u want 4 (ADC-floor 0.3 MS/s regression!)", p.divider);
    CHECK(p.rate_hz == 6000000u, "6 MS/s x4096 rate=%u want 6000000", p.rate_hz);
    CHECK(!p.capped, "small capture must not be burst-capped");

    CHECK(la_psram_plan(HFOSC, 4096, 3.0e6f).divider == 8,  "3 MS/s -> divider 8");
    CHECK(la_psram_plan(HFOSC, 4096, 1.0e6f).divider == 24, "1 MS/s -> divider 24");
    CHECK(la_psram_plan(HFOSC, 256,  6.0e6f).divider == 4,  "6 MS/s small -> divider 4");

    /* ---- 12 MS/s is the peak: divider 2, NOT floored ---- */
    la_plan_t hi = la_psram_plan(HFOSC, 256, 12.0e6f);
    CHECK(hi.divider == 2 && !hi.floored && hi.rate_hz == 12000000u,
          "12 MS/s -> divider 2 unfloored (div=%u floored=%u rate=%u)", hi.divider, hi.floored, hi.rate_hz);

    /* ---- above 12 MS/s clamps up to the divider-2 ceiling (floored) ---- */
    la_plan_t over = la_psram_plan(HFOSC, 256, 24.0e6f);
    CHECK(over.divider == 2 && over.floored, "24 MS/s clamps to divider 2, floored=%u", over.floored);

    /* ---- boundary: a capture that exactly fills the ring is NOT capped ---- */
    /* 32768 samples * 2 bytes == LA_RING_BYTES -> full 12 MS/s, divider 2 */
    la_plan_t edge = la_psram_plan(HFOSC, LA_RING_BYTES / 2, 12.0e6f);
    CHECK(edge.divider == 2 && !edge.capped, "ring-sized capture at full 12 MS/s (div=%u capped=%u)", edge.divider, edge.capped);

    /* ---- deep captures (bigger than the ring) are conservatively burst-capped to the
     *      real rates they were HW-verified lossless at on gateware v31 (whose divider d
     *      really sampled every d+1 clocks): TestHW_RawLA_Long 65536 -> 6 MS/s, 131072 ->
     *      4 MS/s, 262144 -> 3.43 MS/s; console lastress 65535 -> 6 MS/s. ---- */
    la_plan_t deep = la_psram_plan(HFOSC, 65535, 12.0e6f);
    CHECK(deep.divider == 4 && deep.capped && deep.rate_hz == 6000000u,
          "65535 @ 12 MS/s must burst-cap to 6 MS/s (div=%u capped=%u rate=%u)",
          deep.divider, deep.capped, deep.rate_hz);
    CHECK(peak_bytes(65535, deep.rate_hz) <= (double)LA_RING_BYTES + 512.0,
          "65535 capped peak %.0f must fit ring %u", peak_bytes(65535, deep.rate_hz), LA_RING_BYTES);
    {
        static const struct { uint32_t n; uint32_t max_rate_hz; } hw[] = {
            { 65536u, 6000000u }, { 131072u, 4000000u }, { 262144u, 3428572u },
        };
        for (unsigned i = 0; i < sizeof hw / sizeof hw[0]; i++) {
            la_plan_t q = la_psram_plan(HFOSC, hw[i].n, 12.0e6f);
            CHECK(q.capped && q.rate_hz <= hw[i].max_rate_hz,
                  "%u @ 12 MS/s -> %u S/s, must not exceed the HW-verified %u (capped=%u)",
                  hw[i].n, q.rate_hz, hw[i].max_rate_hz, q.capped);
        }
    }

    /* ---- safety net: an OUT-OF-RANGE-deep request is still rate-capped so it can never
     *      overflow the ring ---- */
    la_plan_t big = la_psram_plan(HFOSC, 200000, 12.0e6f);
    CHECK(big.capped, "200000-sample capture must be burst-capped");
    CHECK(big.divider > 2 && big.rate_hz < 12000000u,
          "capped capture divider=%u rate=%u must be slower than 12 MS/s", big.divider, big.rate_hz);
    /* peak ring occupancy at the (rounded-up) capped rate must ACTUALLY fit the ring */
    CHECK(peak_bytes(200000, big.rate_hz) <= (double)LA_RING_BYTES + 512.0 /*rounding*/,
          "capped peak occupancy %.0f must fit ring %u", peak_bytes(200000, big.rate_hz), LA_RING_BYTES);

    /* ---- degenerate: rate<=0 -> "max rate"; a SMALL capture runs at the ceiling ---- */
    CHECK(la_psram_plan(HFOSC, 256, 0.0f).divider == LA_PSRAM_MIN_DIV, "rate 0 (small) -> min divider");

    /* ---- rate<=0 ("max rate") on a DEEP capture must STILL be burst-capped, not run
     *      at full rate and overflow the ring (the full-8 MB lastress bug) ---- */
    la_plan_t deepmax = la_psram_plan(HFOSC, 4161536u /* full 8 MB LA region */, 0.0f);
    CHECK(deepmax.capped && deepmax.divider > LA_PSRAM_MIN_DIV,
          "4.16M @ max-rate must cap below full rate (div=%u capped=%u)", deepmax.divider, deepmax.capped);
    CHECK(peak_bytes(4161536u, deepmax.rate_hz) <= (double)LA_RING_BYTES + 512.0,
          "4.16M capped peak %.0f must fit ring %u", peak_bytes(4161536u, deepmax.rate_hz), LA_RING_BYTES);

    /* ---- cap_divider_wire: the divider on the wire is the exact period on gateware
     *      >= v32, and period-1 on older gateware (which sampled every divider+1 clocks),
     *      so the reported rate 24 MHz / period is right on both. ---- */
    CHECK(cap_divider_wire(24, EXACT_CAP_DIVIDER_MIN_GW) == 24, "v32: period 24 -> wire 24");
    CHECK(cap_divider_wire(2, 40) == 2,                         "v40: period 2 -> wire 2");
    CHECK(cap_divider_wire(60, EXACT_CAP_DIVIDER_MIN_GW) == 60, "v32: ADC floor 60 -> wire 60");
    CHECK(cap_divider_wire(24, 31) == 23,                       "v31: period 24 -> wire 23");
    CHECK(cap_divider_wire(2, 31) == 1,                         "v31: LA peak period 2 -> wire 1");
    CHECK(cap_divider_wire(61, 31) == 60,                       "v31: ADC period 61 -> wire 60 (its floor)");
    CHECK(cap_divider_wire(0, 31) == 0 && cap_divider_wire(1, 31) == 0, "v31: period 0/1 -> wire 0");
    CHECK(cap_divider_wire(70000u, EXACT_CAP_DIVIDER_MIN_GW) == 0xFFFFu, "v32: clamps to 16 bits");
    CHECK(cap_divider_wire(70000u, 31) == 0xFFFEu,              "v31: clamps to 16 bits, then -1");
    /* every LA plan encodes to a divider the v32 gateware takes as-is */
    CHECK(cap_divider_wire(la_psram_plan(HFOSC, 4096, 2.0e6f).divider, 32) == 12, "2 MS/s -> wire 12 on v32");

    /* ---- v40 reload encodings: the gateware no longer subtracts, so the firmware sends the
     *      counter reload.  Older gateware still gets the period / divider / delay. ---- */
    CHECK(la_divider_wire(6, 40) == 4,          "v40: LA period 6 -> wire 4 (period - 2)");
    CHECK(la_divider_wire(2, 40) == 0,          "v40: LA peak period 2 -> wire 0");
    CHECK(la_divider_wire(1, 40) == 0 && la_divider_wire(0, 40) == 0, "v40: LA period < 2 -> 0 (the floor)");
    CHECK(la_divider_wire(70000u, 40) == 0xFFFFu, "v40: LA clamps to 16 bits");
    CHECK(la_divider_wire(6, 39) == 6,          "v39: LA period 6 -> wire 6 (exact period)");
    CHECK(la_divider_wire(6, 31) == 5,          "v31: LA period 6 -> wire 5 (old +1 engines)");
    CHECK(dac_divider_wire(29, 40) == 28,       "v40: DAC divider 29 -> wire 28");
    CHECK(dac_divider_wire(0, 40) == 0,         "v40: DAC divider 0 -> 0 (gateware floors it)");
    CHECK(dac_divider_wire(29, 39) == 29,       "v39: DAC divider unchanged");
    CHECK(dac_divider_wire(70000u, 39) == 0xFFFFu, "v39: DAC clamps to 16 bits");
    CHECK(step_delay_wire(1, 40) == 0,          "v40: 1 us half-phase -> wire 0");
    CHECK(step_delay_wire(250, 40) == 249,      "v40: 250 us -> wire 249");
    CHECK(step_delay_wire(250, 39) == 250,      "v39: delay unchanged");

    /* ---- DAC replay divider: the rate a replay asks for is the rate it plays at.  One sample
     *      takes divider + 51 clk48 from the BRAM and divider + 52 from PSRAM (tb_dac8551,
     *      tb_dac_psram_replay); the old 48 MHz / rate divider played 80 kS/s at 73.7 kS/s. ---- */
    {
        const uint32_t clk = 48000000u;
        const float rates[] = { 1000.f, 10000.f, 44100.f, 80000.f, 100000.f, 250000.f, 400000.f, 800000.f };
        for (unsigned i = 0; i < sizeof(rates) / sizeof(rates[0]); i++) {
            for (int deep = 0; deep < 2; deep++) {
                dac_rate_plan_t q = dac_replay_plan(clk, rates[i], deep);
                double real = (double)clk / (double)dac_sample_clocks(q.divider, deep);
                double err = fabs(real - rates[i]) / rates[i];
                /* nearest divider: within half a clock of the request */
                double half = 0.5 / (double)dac_sample_clocks(q.divider, deep);
                CHECK(err <= half + 1e-9 && q.clamped == 0,
                      "replay %.0f S/s (deep=%d): divider %u plays %.1f S/s (%.3f%%)", rates[i], deep, q.divider, real, err * 100);
                CHECK(q.rate_hz == (uint32_t)(real + 0.5), "replay %.0f S/s (deep=%d): rate_hz %u want %.0f", rates[i], deep, q.rate_hz, real);
            }
        }
        dac_rate_plan_t q = dac_replay_plan(clk, 80000.f, false);
        CHECK(q.divider == 549 && q.rate_hz == 80000u, "80 kS/s RAM: divider %u (want 549 = 600 - 51), %u S/s", q.divider, q.rate_hz);
        q = dac_replay_plan(clk, 80000.f, true);
        CHECK(q.divider == 548 && q.rate_hz == 80000u, "80 kS/s PSRAM: divider %u (want 548 = 600 - 52), %u S/s", q.divider, q.rate_hz);
        q = dac_replay_plan(clk, 400000.f, false);
        CHECK(q.divider == 69 && q.rate_hz == 400000u, "400 kS/s RAM: divider %u (want 69), %u S/s", q.divider, q.rate_hz);
        /* the maximum: 0 asks for it; faster requests are clamped to it and flagged */
        q = dac_replay_plan(clk, 0.0f, false);
        CHECK(q.divider == DAC_MIN_DIVIDER && q.rate_hz == 888889u && q.clamped == 0, "max RAM: divider %u, %u S/s", q.divider, q.rate_hz);
        q = dac_replay_plan(clk, 0.0f, true);
        CHECK(q.divider == DAC_MIN_DIVIDER && q.rate_hz == 872727u && q.clamped == 0, "max PSRAM: divider %u, %u S/s", q.divider, q.rate_hz);
        q = dac_replay_plan(clk, 2.0e6f, false);
        CHECK(q.divider == DAC_MIN_DIVIDER && q.rate_hz == 888889u && q.clamped == 1, "2 MS/s RAM: clamp to the max (%u, %u S/s, flag %u)", q.divider, q.rate_hz, q.clamped);
        q = dac_replay_plan(clk, 12.0e6f, true);
        CHECK(q.divider == DAC_MIN_DIVIDER && q.clamped == 1, "12 MS/s PSRAM: clamp to the max (flag %u)", q.clamped);
        q = dac_replay_plan(clk, 885000.f, false);   /* rounds to the minimum divider: not a clamp */
        CHECK(q.divider == DAC_MIN_DIVIDER && q.clamped == 0, "885 kS/s RAM: divider %u flag %u", q.divider, q.clamped);
        q = dac_replay_plan(clk, 100.f, false);       /* below 48 MHz / (65535 + 51) ~= 732 S/s */
        CHECK(q.divider == DAC_MAX_DIVIDER && q.clamped == 2 && q.rate_hz == 732u, "100 S/s: clamp to the slowest (%u, %u S/s, flag %u)", q.divider, q.rate_hz, q.clamped);
        CHECK(dac_sample_clocks(0, false) == 54 && dac_sample_clocks(3, true) == 55, "sample clocks floor the divider at 3");
    }

    if (fails == 0) printf("PASS — all la_rate tests\n");
    else            printf("FAIL — %d la_rate test(s)\n", fails);
    return fails ? 1 : 0;
}
