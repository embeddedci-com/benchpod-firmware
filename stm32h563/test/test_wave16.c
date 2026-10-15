/* Host unit test for the generator's 16-bit waveforms (wave16.c): the calibrated `generate`
 * form plays volts or microamps through the per-path fits at the DAC8551's full 16 bits, where
 * the old form had 256 levels.
 */
#include "wave16.h"
#include "cal_data.h"
#include "current_out.h"
#include "board_rev.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int fails = 0;
#define CHECK(cond, ...) do { if (!(cond)) { \
        printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

static uint16_t buf[4096];

static void test_shape_names(void) {
    wave16_shape_t s;
    CHECK(wave16_shape_from_name("sine", &s) == 0 && s == WAVE16_SINE, "sine");
    CHECK(wave16_shape_from_name("square", &s) == 0 && s == WAVE16_SQUARE, "square");
    CHECK(wave16_shape_from_name("sawtooth", &s) == 0 && s == WAVE16_SAWTOOTH, "sawtooth");
    CHECK(wave16_shape_from_name("triangle", &s) != 0, "unknown shape refused");
}

/* The three shapes at their landmarks, rounded once. */
static void test_fill(void) {
    wave16_fill(WAVE16_SINE, 400, 10000.4, 30000.3, buf);
    CHECK(buf[0] == 30000, "sine starts at mid: %u", buf[0]);
    CHECK(buf[100] == 40001, "sine peak: %u", buf[100]);
    CHECK(buf[300] == 20000, "sine trough: %u", buf[300]);

    wave16_fill(WAVE16_SQUARE, 10, 5000.0, 20000.0, buf);
    CHECK(buf[0] == 25000 && buf[4] == 25000 && buf[5] == 15000 && buf[9] == 15000, "square halves");

    wave16_fill(WAVE16_SAWTOOTH, 4, 4000.0, 10000.0, buf);
    CHECK(buf[0] == 6000 && buf[1] == 8000 && buf[2] == 10000 && buf[3] == 12000, "sawtooth ramp %u %u %u %u",
          buf[0], buf[1], buf[2], buf[3]);

    /* Clamped at both ends, never wrapped. */
    wave16_fill(WAVE16_SQUARE, 2, 40000.0, 32768.0, buf);
    CHECK(buf[0] == 65535 && buf[1] == 0, "clamp %u %u", buf[0], buf[1]);

    /* A negative amplitude flips the codes, not the shape in volts. */
    wave16_fill(WAVE16_SINE, 4, -1000.0, 30000.0, buf);
    CHECK(buf[1] == 29000 && buf[3] == 31000, "negative amp %u %u", buf[1], buf[3]);
}

/* Volts -> codes on every DAC path agree with dac_out's own conversion, and are finer than the
   old 8-bit levels: 1 mV moves the code. */
static void test_volts(void) {
    for (int path = 0; path < 3; path++) {
        double lo = DAC_CAL[path].a, hi = DAC_CAL[path].a + DAC_CAL[path].b * 65535.0 / 256.0;
        double off = (lo + hi) / 2.0, amp = (hi - lo) / 4.0;
        double a = 0, m = 0;
        CHECK(wave16_codes_for_volts(path, amp, off, &a, &m) == NULL, "path %d in range", path);
        CHECK(fabs(m - (double)dac_code16_for_volts(path, (float)off)) <= 1.0,
              "path %d mid %.1f vs dac_out %u", path, m, dac_code16_for_volts(path, (float)off));
        CHECK(fabs(fabs(a) - 65535.0 / 4.0) < 2.0, "path %d amp %.1f", path, a);

        double a2 = 0, m2 = 0;
        CHECK(wave16_codes_for_volts(path, amp, off + 0.001, &a2, &m2) == NULL, "path %d +1 mV", path);
        CHECK(fabs(m2 - m) > 1.0 && fabs(m2 - m) < 256.0, "path %d: 1 mV moves the mid %.2f codes", path, m2 - m);
    }
    /* The 3.3 V path, as the web Generator sends it: a sine from 0.17 V to 3.13 V. */
    double a = 0, m = 0;
    CHECK(wave16_codes_for_volts(0, 1.48, 1.65, &a, &m) == NULL, "3v3 sine accepted");
    wave16_fill(WAVE16_SINE, 400, a, m, buf);
    float top = dac_volts_for_code16(0, buf[100]), bottom = dac_volts_for_code16(0, buf[300]);
    CHECK(fabsf(top - 3.13f) < 0.0005f && fabsf(bottom - 0.17f) < 0.0005f, "3v3 sine %.4f..%.4f V", bottom, top);
}

static void test_volts_refusals(void) {
    double a, m;
    const char *why = wave16_codes_for_volts(0, 1.0, 3.0, &a, &m);
    CHECK(why && strstr(why, "outside the 3v3 output"), "peak above the range refused: %s", why ? why : "(null)");
    CHECK(wave16_codes_for_volts(0, 0.0, 1.0, &a, &m) != NULL, "zero amplitude refused");
    CHECK(wave16_codes_for_volts(0, -0.5, 1.0, &a, &m) != NULL, "negative amplitude refused");
    CHECK(wave16_codes_for_volts(3, 0.5, 1.0, &a, &m) != NULL, "bad path refused");
    /* NaN in, refusal out. */
    CHECK(wave16_codes_for_volts(0, NAN, 1.0, &a, &m) != NULL, "NaN amplitude refused");
}

static void test_ua(void) {
    double a = 0, m = 0;
    CHECK(wave16_codes_for_ua(4000.0, 12000.0, &a, &m) == NULL, "8-16 mA accepted");
    wave16_fill(WAVE16_SINE, 400, a, m, buf);
    CHECK(labs(current_out_ua(buf[100]) - 16000) <= 1 && labs(current_out_ua(buf[300]) - 8000) <= 1,
          "sine 8..16 mA: %ld..%ld uA", current_out_ua(buf[300]), current_out_ua(buf[100]));
    /* A trough at 4 mA is accepted (code 0, the live zero), like current_out. */
    CHECK(wave16_codes_for_ua(8000.0, 12000.0, &a, &m) == NULL, "4..20 mA accepted");
    CHECK(wave16_codes_for_ua(9000.0, 12000.0, &a, &m) != NULL, "trough below 4 mA refused");
    CHECK(wave16_codes_for_ua(1000.0, 20000.0, &a, &m) != NULL, "peak above the top refused");
}

int main(void) {
    cal_data_select(BOARD_REV_V3);
    test_shape_names();
    test_fill();
    test_volts();
    test_volts_refusals();
    test_ua();
    cal_data_select(BOARD_REV_V2);
    test_volts();
    printf(fails ? "test_wave16: %d FAILED\n" : "test_wave16: all passed\n", fails);
    return fails ? 1 : 0;
}
