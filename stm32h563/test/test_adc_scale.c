/* Host unit test for adc_scale_burst() — adc_read's sample-burst reduction.
 *
 * Regression-locks the 2026-07-29 silent-corruption bug: handle_adc_read averaged
 * the 16 RAW counts and unwrapped only the AVERAGE.  Every analog source's 0 V
 * point sits within a few counts of the 65535->0 wrap (MEASURED on pod
 * 192.168.1.215: ext 24..38, cal1@dac0 65524..65530, amp 0..9), so one sample
 * landing on the far side of the cut moved the raw mean to mid-scale, which then
 * got unwrapped as a single huge negative reading.  adc_read returned a
 * PLAUSIBLE number wrong by tens of volts, quantised in 65536/16 = 4096-count
 * steps, and never errored — a calibration sweep recorded it as data.
 *
 * The straddle cases below are the ones the old code got wrong; each carries the
 * value the old arithmetic produced so a regression is unmistakable.
 */
#include "adc_scale.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

/* The real board fits (cal_data.h), duplicated here so the test needs no HAL. */
#define EXT_A   65.788900f
#define EXT_B  -0.001003762f
#define CAL1_A  65.832398f
#define CAL1_B -0.001004471f

static int fails = 0;
#define CHECK(cond, ...) do { if (!(cond)) { \
        printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

/* The arithmetic adc_read used to do: mean of the RAW counts, then unwrap. */
static float legacy_volts(const uint16_t *raw, size_t n, float a, float b) {
    uint32_t sum = 0;
    for (size_t i = 0; i < n; i++) sum += raw[i];
    float cnt = (float)sum / (float)n;
    if (cnt < 32768.0f) cnt += 65536.0f;
    return a + b * cnt;
}

static void fill(uint16_t *dst, size_t n, uint16_t v) {
    for (size_t i = 0; i < n; i++) dst[i] = v;
}

int main(void) {
    uint16_t s[16];

    /* ---- 1. the ordinary case still works: `ext` at a quiet input ----------
       Measured on HW: counts 24..38 around a -20 mV node. All on one side of the
       cut, so old and new arithmetic must agree. */
    {
        const uint16_t q[16] = {28,29,27,27,28,30,31,30,31,30,30,29,28,29,28,29};
        memcpy(s, q, sizeof(q));
        adc_reading_t r = adc_scale_burst(s, 16, EXT_A, EXT_B);
        CHECK(r.valid, "quiet ext burst must be valid");
        CHECK(fabsf(r.volts * 1000.0f - (-22.0f)) < 3.0f,
              "quiet ext: %.1f mV, want ~-22 mV", r.volts * 1000.0f);
        CHECK(fabsf(r.volts - legacy_volts(s, 16, EXT_A, EXT_B)) < 0.001f,
              "non-straddling burst must match the legacy result");
        CHECK(r.span <= 8u, "quiet ext span=%u, want <=8", (unsigned)r.span);
    }

    /* ---- 2. THE BUG: a burst straddling the wrap -------------------------
       8 samples just below the top (65530) + 8 just above zero (5).  The true
       level is count ~65535.5, and `ext` puts 0 V at count 65541.6, so this is a
       quiet node at about +7 mV.  The legacy mean is 32767.5 -> unwrapped to
       98303.5 -> about -32.9 V: off by 33 volts, and it looked like a perfectly
       ordinary reading. */
    {
        for (int i = 0; i < 8; i++) { s[i] = 65530; s[i + 8] = 5; }
        adc_reading_t r = adc_scale_burst(s, 16, EXT_A, EXT_B);
        float legacy = legacy_volts(s, 16, EXT_A, EXT_B);
        CHECK(legacy < -32.0f, "legacy arithmetic should have been ~-32.9 V, got %.3f V", legacy);
        CHECK(r.valid, "an 11-count-wide straddling burst is still a settled reading");
        CHECK(fabsf(r.volts * 1000.0f - 7.0f) < 5.0f,
              "straddling burst: %.1f mV, want ~+7 mV (legacy gave %.0f mV)",
              r.volts * 1000.0f, legacy * 1000.0f);
        CHECK(r.span <= 12u, "straddle span=%u, want <=12 (it is a QUIET burst)", (unsigned)r.span);
    }

    /* ---- 3. the 4096-count ladder: every straddle mix must stay correct ----
       k high + (16-k) low samples.  The legacy mean walked in ~4096-count steps
       (the 21845/43690-style values seen on the bench); the circular mean must
       report ~0 V for all of them. */
    for (int k = 1; k < 16; k++) {
        for (int i = 0; i < 16; i++) s[i] = (i < k) ? 65533 : 3;
        adc_reading_t r = adc_scale_burst(s, 16, EXT_A, EXT_B);
        CHECK(r.valid, "k=%d: burst must be valid", k);
        CHECK(fabsf(r.volts * 1000.0f) < 40.0f,
              "k=%d: %.1f mV, want within +-40 mV of 0 (legacy: %.0f mV)",
              k, r.volts * 1000.0f, legacy_volts(s, 16, EXT_A, EXT_B) * 1000.0f);
    }

    /* ---- 4. unwrap is unconditional: `amp`/`cal1` at 0 V ------------------
       MEASURED on HW: the amp path reads counts 0..9 at a ~0 V input and the old
       code, which disabled the unwrap for anything but cal2/ext, reported
       +65828 mV (65.8 V!) for it. */
    {
        const uint16_t a[16] = {5,5,4,5,6,5,5,4,5,5,6,5,4,5,5,5};
        memcpy(s, a, sizeof(a));
        adc_reading_t r = adc_scale_burst(s, 16, CAL1_A, CAL1_B);
        CHECK(fabsf(r.volts * 1000.0f) < 40.0f,
              "amp/cal1 at 0 V: %.1f mV, want ~0 (unwrapped); legacy reported ~+65828 mV",
              r.volts * 1000.0f);
    }

    /* ---- 5. a positive in-range input must NOT be pushed around by the unwrap
       cal1 at DAC 128 measured 63048..63051 -> ~2.50 V. */
    {
        fill(s, 16, 63049);
        adc_reading_t r = adc_scale_burst(s, 16, CAL1_A, CAL1_B);
        CHECK(fabsf(r.volts - 2.501f) < 0.01f, "cal1 @2.5V: %.4f V, want ~2.501", r.volts);
        CHECK(r.count_uw == r.count, "an above-half-scale count must not be unwrapped");
        CHECK(r.span == 0u, "constant burst span=%u, want 0", (unsigned)r.span);
    }

    /* ---- 6. a MOVING input is refused, not averaged -----------------------
       This is the "or be refused with an error" half of the fix: a DAC left
       driving the node (a square wave here) has no single voltage. */
    {
        for (int i = 0; i < 16; i++) s[i] = (i & 1) ? 63049 : 11238;   /* ~2.5 V vs ~54 V swing */
        adc_reading_t r = adc_scale_burst(s, 16, CAL1_A, CAL1_B);
        CHECK(!r.valid, "a square-wave burst must be REFUSED (span=%u)", (unsigned)r.span);
        CHECK(r.span > ADC_BURST_MAX_SPAN, "moving-input span=%u, want >%u",
              (unsigned)r.span, (unsigned)ADC_BURST_MAX_SPAN);
    }
    /* ...and a burst right at the limit is still accepted (no over-eager refusal). */
    {
        fill(s, 16, 30000);
        s[7] = (uint16_t)(30000 + ADC_BURST_MAX_SPAN);
        adc_reading_t r = adc_scale_burst(s, 16, CAL1_A, CAL1_B);
        CHECK(r.valid, "span exactly at the limit must be accepted (span=%u)", (unsigned)r.span);
        s[7] = (uint16_t)(30000 + ADC_BURST_MAX_SPAN + 1u);
        CHECK(!adc_scale_burst(s, 16, CAL1_A, CAL1_B).valid, "one count over the limit must refuse");
    }

    /* ---- 7. degenerate inputs ------------------------------------------- */
    CHECK(!adc_scale_burst(NULL, 16, EXT_A, EXT_B).valid, "NULL burst must be invalid");
    CHECK(!adc_scale_burst(s, 0, EXT_A, EXT_B).valid,     "zero-length burst must be invalid");
    {
        fill(s, 1, 65530);
        adc_reading_t r = adc_scale_burst(s, 1, EXT_A, EXT_B);
        CHECK(r.valid && r.span == 0u, "single-sample burst is valid with span 0");
    }

    if (fails == 0) printf("test_adc_scale: PASS\n");
    else            printf("test_adc_scale: %d FAILURES\n", fails);
    return fails ? 1 : 0;
}
