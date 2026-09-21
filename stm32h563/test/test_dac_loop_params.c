/* Host unit test for dac_loop_params.c — the closed-loop DAC command's pure core.
 *
 * Two things are pinned here because only a hardware run would otherwise catch a change:
 *
 *   1. THE INDEX MAPPING.  The gateware reads curve[adc >> 5] out of a 2048-entry LUT
 *      (ice40/src/dac_loop.v), and the firmware upsamples a COMPACT client upload into
 *      that LUT.  For the webapp's 256-point upload the composition must come out at
 *      "source point = adc >> 8" — the exact relation the webapp's I-V overlay, the
 *      Python SDK and the hwe2e tracking test all predict against.  If the upsample ever
 *      drifts (rounding, off-by-one at the top), the emulated panel curve silently
 *      mis-shapes on a live bench and every client's prediction is wrong.
 *
 *   2. THE PARAMETER GUARD.  vmin > vmax is REJECTED: the gateware's S_CLAMP tests vmin
 *      first, so an inverted window pins the DAC at vmin and quietly ignores the caller's
 *      ceiling (an emulator would drive a rail the DUT was supposed to be protected from).
 *      k and tick_div are CLAMPED instead — a too-large k would wrap to a tiny one in
 *      fabric (k[14:0]) and a too-small tick would outrun the pipelined tick.
 */
#include "dac_loop_params.h"
#include <stdio.h>
#include <string.h>

static int fails = 0;
#define CHECK(cond, ...) do { if (!(cond)) { \
        printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

static uint16_t pt(const uint8_t *buf, size_t i) {
    return (uint16_t)(buf[i * 2u] | ((uint16_t)buf[i * 2u + 1u] << 8));
}
static void put(uint8_t *buf, size_t i, uint16_t v) {
    buf[i * 2u]      = (uint8_t)(v & 0xFFu);
    buf[i * 2u + 1u] = (uint8_t)(v >> 8);
}

/* The webapp's panel curve (controlLoopCurve.ts / control_loop.py build_panel_curve),
 * simplified to a strictly decreasing ramp — shape doesn't matter to the mapping test,
 * DISTINCT values do (they make a mis-index visible). */
static void build_ramp(uint8_t *buf, size_t pts, uint16_t voc) {
    for (size_t i = 0; i < pts; i++) put(buf, i, (uint16_t)(voc - (uint16_t)i));
}

int main(void) {
    static uint8_t lut[DAC_LOOP_CURVE_BYTES];
    static uint8_t in[DAC_LOOP_CURVE_BYTES * 2u];

    /* ---- 1. the webapp's 256-point upload -> source point = adc >> 8 ---- */
    const size_t IN_PTS = 256;
    build_ramp(in, IN_PTS, 52428);
    size_t n = dac_loop_upsample_curve(in, IN_PTS * 2u, lut, sizeof(lut));
    CHECK(n == DAC_LOOP_CURVE_BYTES, "256-pt upsample wrote %zu bytes, want %u",
          n, (unsigned)DAC_LOOP_CURVE_BYTES);
    CHECK(pt(lut, 0) == pt(in, 0), "LUT[0] must be the curve's open-circuit point");
    CHECK(pt(lut, DAC_LOOP_CURVE_POINTS - 1u) == pt(in, IN_PTS - 1u),
          "LUT top entry must be the curve's short-circuit point (%u vs %u)",
          pt(lut, DAC_LOOP_CURVE_POINTS - 1u), pt(in, IN_PTS - 1u));
    /* Walk the WHOLE 16-bit ADC range the way the gateware does: idx = adc >> 5, and
       assert the value equals the client's point adc >> 8.  This is the composition
       (upsample by 8) x (LUT index by 32) == (source index by 256). */
    for (uint32_t adc = 0; adc <= 0xFFFFu; adc += 37u) {   /* 37: coprime-ish, hits every bucket */
        uint16_t got  = pt(lut, adc >> 5);
        uint16_t want = pt(in, adc >> 8);
        if (got != want) {
            CHECK(0, "adc=%u: LUT[adc>>5]=%u but curve[adc>>8]=%u — index mapping drifted",
                  adc, got, want);
            break;
        }
    }
    /* Nearest-neighbour means each source point covers exactly 8 LUT entries. */
    for (size_t j = 0; j < DAC_LOOP_CURVE_POINTS; j++) {
        if (pt(lut, j) != pt(in, j / 8u)) {
            CHECK(0, "LUT[%zu]=%u, want curve[%zu]=%u (8 entries per source point)",
                  j, pt(lut, j), j / 8u, pt(in, j / 8u));
            break;
        }
    }

    /* ---- 2. a stale upper tail from a previous, longer load cannot survive ---- */
    memset(lut, 0xAB, sizeof(lut));
    put(in, 0, 1234); put(in, 1, 5678);
    n = dac_loop_upsample_curve(in, 2u * 2u, lut, sizeof(lut));
    CHECK(n == DAC_LOOP_CURVE_BYTES, "2-pt upsample wrote %zu bytes", n);
    CHECK(pt(lut, 0) == 1234 && pt(lut, DAC_LOOP_CURVE_POINTS - 1u) == 5678,
          "2-pt curve must fill the whole LUT (lo=%u hi=%u)",
          pt(lut, 0), pt(lut, DAC_LOOP_CURVE_POINTS - 1u));
    for (size_t j = 0; j < DAC_LOOP_CURVE_POINTS; j++) {
        uint16_t v = pt(lut, j);
        if (v != 1234 && v != 5678) {
            CHECK(0, "LUT[%zu]=%u is neither uploaded point — stale data survived", j, v);
            break;
        }
    }

    /* ---- 3. a single point is a legal FLAT curve (constant target, any ADC) ---- */
    put(in, 0, 40000);
    n = dac_loop_upsample_curve(in, 2u, lut, sizeof(lut));
    CHECK(n == DAC_LOOP_CURVE_BYTES, "1-pt upsample wrote %zu bytes", n);
    CHECK(pt(lut, 0) == 40000 && pt(lut, 1000) == 40000 &&
          pt(lut, DAC_LOOP_CURVE_POINTS - 1u) == 40000, "1-pt curve must fill the LUT flat");

    /* ---- 4. a full (or over-long) upload passes through, truncated to the LUT ---- */
    for (size_t i = 0; i < DAC_LOOP_CURVE_POINTS + 100u; i++) put(in, i, (uint16_t)(i * 3u));
    n = dac_loop_upsample_curve(in, (DAC_LOOP_CURVE_POINTS + 100u) * 2u, lut, sizeof(lut));
    CHECK(n == DAC_LOOP_CURVE_BYTES, "full upload wrote %zu bytes", n);
    CHECK(pt(lut, 0) == 0 && pt(lut, 5) == 15 &&
          pt(lut, DAC_LOOP_CURVE_POINTS - 1u) == (uint16_t)((DAC_LOOP_CURVE_POINTS - 1u) * 3u),
          "full upload must pass through verbatim (truncated to the LUT depth)");

    /* ---- 5. degenerate inputs are refused, never half-written ---- */
    CHECK(dac_loop_upsample_curve(in, 0, lut, sizeof(lut)) == 0, "0-byte curve must be refused");
    CHECK(dac_loop_upsample_curve(in, 1, lut, sizeof(lut)) == 0, "a half point must be refused");
    CHECK(dac_loop_upsample_curve(NULL, 512, lut, sizeof(lut)) == 0, "NULL input must be refused");
    CHECK(dac_loop_upsample_curve(in, 512, lut, DAC_LOOP_CURVE_BYTES - 1u) == 0,
          "an undersized output buffer must be refused");

    /* ---- 6. parameter validation: reject the inverted clamp, normalise the rest ---- */
    dac_loop_params_t p = { .k_q15 = 8191, .vmin = 40000, .vmax = 10000, .tick_div = 64 };
    CHECK(dac_loop_params_validate(&p) == DAC_LOOP_PARAMS_ERR_CLAMP_INVERTED,
          "vmin > vmax must be REJECTED (the gateware would silently pin the DAC at vmin)");
    CHECK(strstr(dac_loop_params_err_str(DAC_LOOP_PARAMS_ERR_CLAMP_INVERTED), "vmax") != NULL,
          "the rejection reason must name the offending field");

    p = (dac_loop_params_t){ .k_q15 = 8191, .vmin = 10000, .vmax = 10000, .tick_div = 64 };
    CHECK(dac_loop_params_validate(&p) == DAC_LOOP_PARAMS_OK,
          "vmin == vmax is legal — it is how a fixed output level is pinned");

    p = (dac_loop_params_t){ .k_q15 = 65535, .vmin = 0, .vmax = 65535, .tick_div = 0 };
    CHECK(dac_loop_params_validate(&p) == DAC_LOOP_PARAMS_OK, "clampable params must be accepted");
    CHECK(p.k_q15 == DAC_LOOP_K_MAX, "k=65535 must clamp to %u (fabric uses k[14:0] and would wrap to 32767's low bits)",
          (unsigned)DAC_LOOP_K_MAX);
    CHECK(p.tick_div == DAC_LOOP_TICK_MIN, "tick_div=0 must clamp to %u", (unsigned)DAC_LOOP_TICK_MIN);

    p = (dac_loop_params_t){ .k_q15 = 0, .vmin = 0, .vmax = 65535, .tick_div = 64 };
    CHECK(dac_loop_params_validate(&p) == DAC_LOOP_PARAMS_OK && p.k_q15 == DAC_LOOP_K_MIN,
          "k=0 (a frozen loop) must clamp up to %u, got %u", (unsigned)DAC_LOOP_K_MIN, p.k_q15);

    p = (dac_loop_params_t){ .k_q15 = 8191, .vmin = 0, .vmax = 52428, .tick_div = 64 };
    CHECK(dac_loop_params_validate(&p) == DAC_LOOP_PARAMS_OK &&
          p.k_q15 == 8191 && p.tick_div == 64 && p.vmin == 0 && p.vmax == 52428,
          "the webapp's defaults must pass through untouched");

    /* ---- 7. loop INPUT SOURCE validation (gateware >= v29) ---- */
    /* The default set (no source given anywhere) must still be the closed loop, so a client
       written before v29 keeps the exact behaviour it had. */
    p = (dac_loop_params_t){ .k_q15 = 8192, .vmin = 0, .vmax = 65535, .tick_div = 64 };
    CHECK(dac_loop_params_validate(&p) == DAC_LOOP_PARAMS_OK && p.src == DAC_LOOP_SRC_ADC,
          "a params struct with no source set must mean the live ADC (closed loop)");

    p = (dac_loop_params_t){ .k_q15 = 8192, .vmin = 0, .vmax = 65535, .tick_div = 64,
                             .src = DAC_LOOP_SRC_FIXED, .in_fixed = 32768, .sweep_step = 0 };
    CHECK(dac_loop_params_validate(&p) == DAC_LOOP_PARAMS_OK,
          "a FIXED source needs no step — that is the whole point of holding one point");

    p = (dac_loop_params_t){ .k_q15 = 8192, .vmin = 0, .vmax = 65535, .tick_div = 64,
                             .src = DAC_LOOP_SRC_SWEEP, .in_fixed = 0, .sweep_step = 0 };
    CHECK(dac_loop_params_validate(&p) == DAC_LOOP_PARAMS_ERR_SWEEP_STEP_ZERO,
          "a SWEEP with step 0 never advances — it must be REJECTED, not run as a fixed point");
    CHECK(strstr(dac_loop_params_err_str(DAC_LOOP_PARAMS_ERR_SWEEP_STEP_ZERO), "step") != NULL,
          "the rejection reason must name the offending field");

    p = (dac_loop_params_t){ .k_q15 = 8192, .vmin = 0, .vmax = 65535, .tick_div = 64,
                             .src = 7, .in_fixed = 0, .sweep_step = 16 };
    CHECK(dac_loop_params_validate(&p) == DAC_LOOP_PARAMS_ERR_BAD_SOURCE,
          "an out-of-range source must be REJECTED (the fabric would take src & 3 and run "
          "something the caller never asked for)");

    /* The wire names are part of the client contract (webapp, SDK, hwe2e all send them). */
    uint8_t src = 0xFF;
    CHECK(dac_loop_src_parse("adc", &src) && src == DAC_LOOP_SRC_ADC, "\"adc\" must parse");
    CHECK(dac_loop_src_parse("fixed", &src) && src == DAC_LOOP_SRC_FIXED, "\"fixed\" must parse");
    CHECK(dac_loop_src_parse("sweep", &src) && src == DAC_LOOP_SRC_SWEEP, "\"sweep\" must parse");
    src = DAC_LOOP_SRC_FIXED;
    CHECK(!dac_loop_src_parse("open", &src) && src == DAC_LOOP_SRC_FIXED,
          "an unknown source name must be refused and leave the caller's value alone — "
          "falling back to the ADC would run a closed loop under an open-loop request");
    CHECK(!dac_loop_src_parse("", &src) && !dac_loop_src_parse(NULL, &src),
          "an empty/NULL source name must be refused");
    CHECK(strcmp(dac_loop_src_str(DAC_LOOP_SRC_ADC), "adc") == 0 &&
          strcmp(dac_loop_src_str(DAC_LOOP_SRC_FIXED), "fixed") == 0 &&
          strcmp(dac_loop_src_str(DAC_LOOP_SRC_SWEEP), "sweep") == 0,
          "the source names must round-trip back out (the probe reports them)");

    /* ---- v30 INPUT MAP: the reference solar bench ------------------------------------
     * 0.04 ohm shunt into an INA282 (fixed 50 V/V, REF1/REF2 to GND) = 2.000 mV/mA, 0..1 A,
     * read through the front SMA (ADC_CAL_EXT). This is the case the whole feature exists
     * for, and the counts below are the ones the design was worked out against: 0 mA is
     * count 6 (it WRAPPED past the 16-bit top) and 1000 mA is count 63550, so the window
     * runs BACKWARDS across the seam. */
    {
        const dac_loop_adc_cal_t ext = { 65.788900f, -0.001003762f };
        dac_loop_input_t in = { .mv_at_zero = 0.0f, .mv_per_unit = 2.0f,
                                .range_min = 0.0f, .range_max = 1000.0f,
                                .trip = 0.0f, .trip_en = false };
        dac_loop_inmap_t m = {0};
        CHECK(dac_loop_inmap_derive(&in, ext, &m) == DAC_LOOP_PARAMS_OK,
              "the reference bench must derive a usable map");
        CHECK(m.in_zero == 6, "0 mA must land on count 6 (it wraps past the 16-bit top); got %u",
              (unsigned)m.in_zero);
        CHECK(m.in_gain < 0,
              "the front end INVERTS, so the index gain must be NEGATIVE — a positive gain "
              "is the bug this whole map exists to fix (output rising with load)");
        CHECK(m.map_en, "a derived map must arrive enabled");
        /* 1992 counts onto 2047 entries needs |gain| slightly over 1.0, which is capped:
         * one entry per ADC count is the hardware's own resolution floor. */
        CHECK(m.in_gain == -32767, "gain must cap at -1.0 Q15 for a ~1992-count window; got %d",
              (int)m.in_gain);
        CHECK(m.idx_max >= 1980 && m.idx_max <= 2047,
              "the window must span essentially the whole LUT (got idx_max %u) — landing "
              "short of it is the pre-v30 behaviour this replaces", (unsigned)m.idx_max);

        /* The map must be MONOTONE in the right direction across the whole range: this is
         * the property a raw-count index got backwards. */
        uint16_t prev = 0;
        for (int mA = 0; mA <= 1000; mA += 100) {
            float mv = (float)mA * 2.0f;
            float cnt = ((mv / 1000.0f) - ext.a) / ext.b;
            int32_t c16 = ((int32_t)(cnt + 0.5f)) & 0xFFFF;
            int32_t d = (int16_t)(c16 - (int32_t)m.in_zero);   /* the fabric's modular subtract */
            int32_t idx = (d * (int32_t)m.in_gain) >> 15;
            if (idx < 0) idx = 0;
            if (idx > 2047) idx = 2047;
            CHECK(mA == 0 || (uint16_t)idx > prev,
                  "index must RISE with current: %d mA gave %d, previous was %u",
                  mA, (int)idx, (unsigned)prev);
            prev = (uint16_t)idx;
        }

        /* A trip level becomes a curve index in the same frame of reference. */
        in.trip_en = true; in.trip = 900.0f;
        CHECK(dac_loop_inmap_derive(&in, ext, &m) == DAC_LOOP_PARAMS_OK, "trip must derive");
        CHECK(m.trip_en && m.in_trip > 1600 && m.in_trip < 1900,
              "900 mA of a 1000 mA range must trip near 90%% of the curve; got %u",
              (unsigned)m.in_trip);

        /* A window spanning half the count space cannot be indexed: the modular subtract
         * stops being a signed distance, so two readings would share one index. */
        dac_loop_input_t wide = in;
        wide.range_max = 40000.0f;            /* 0..80 V of sense */
        CHECK(dac_loop_inmap_derive(&wide, ext, &m) == DAC_LOOP_PARAMS_ERR_INPUT_SPAN,
              "a range spanning half the ADC must be REJECTED, not silently aliased");
        dac_loop_input_t bad = in; bad.mv_per_unit = 0.0f;
        CHECK(dac_loop_inmap_derive(&bad, ext, &m) == DAC_LOOP_PARAMS_ERR_INPUT_GAIN,
              "a zero sense gain must be rejected");
        bad = in; bad.range_max = bad.range_min;
        CHECK(dac_loop_inmap_derive(&bad, ext, &m) == DAC_LOOP_PARAMS_ERR_INPUT_RANGE,
              "an empty range must be rejected");
    }

    /* ---- v30 MAPPED UPSAMPLE: spans idx_max, interpolates, holds past the end ---------- */
    {
        static uint8_t src[8 * 2];        /* 8 points: 0, 1000, 2000 ... 7000 */
        for (int i = 0; i < 8; i++) {
            uint16_t v = (uint16_t)(i * 1000);
            src[i * 2] = (uint8_t)(v & 0xFF); src[i * 2 + 1] = (uint8_t)(v >> 8);
        }
        static uint8_t lut[DAC_LOOP_CURVE_BYTES];
        const uint16_t idx_max = 1991;     /* what the reference bench produces */
        CHECK(dac_loop_upsample_curve_mapped(src, sizeof src, idx_max, lut, sizeof lut)
                  == DAC_LOOP_CURVE_BYTES, "the mapped upsample must fill the whole LUT");
        #define LUT_AT(j) ((uint16_t)(lut[(j) * 2] | (lut[(j) * 2 + 1] << 8)))
        CHECK(LUT_AT(0) == 0, "entry 0 must be the curve's first point; got %u", LUT_AT(0));
        CHECK(LUT_AT(idx_max) == 7000,
              "the LAST REACHABLE entry must be the curve's last point — this is what the old "
              "upsample got wrong, ending the curve at entry 2047 the input cannot reach; got %u",
              LUT_AT(idx_max));
        CHECK(LUT_AT(2047) == 7000,
              "entries past the window must hold the last point: the fabric saturates an "
              "over-range input to 2047, so that is what a short-circuit reads; got %u",
              LUT_AT(2047));
        /* Interpolation: halfway along must be halfway up a linear curve, not a step. */
        uint16_t mid = LUT_AT(idx_max / 2);
        CHECK(mid > 3400 && mid < 3600,
              "the midpoint of a linear curve must interpolate to ~3500, not snap to a "
              "source point (nearest-neighbour would give 3000 or 4000); got %u", mid);
        /* Monotone, no backward steps anywhere. */
        int mono = 1;
        for (uint32_t j = 1; j <= idx_max; j++) if (LUT_AT(j) < LUT_AT(j - 1)) mono = 0;
        CHECK(mono, "a rising curve must upsample to a monotone table");
        CHECK(dac_loop_upsample_curve_mapped(src, sizeof src, 0, lut, sizeof lut) == 0,
              "idx_max 0 is unusable and must be refused");
        CHECK(dac_loop_upsample_curve_mapped(src, 0, idx_max, lut, sizeof lut) == 0,
              "an empty curve must be refused");
        #undef LUT_AT
    }

    if (fails == 0) printf("PASS test_dac_loop_params\n");
    else            printf("FAIL test_dac_loop_params: %d failure(s)\n", fails);
    return fails ? 1 : 0;
}
