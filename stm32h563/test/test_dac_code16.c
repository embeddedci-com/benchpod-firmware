/* Host unit test for the 16-bit DAC conversion behind dac_out (cal_data.c).
 *
 * The per-path fits are on 8-bit codes, which drive the DAC8551 in its high byte, so the 16-bit
 * code for a voltage must land on the 8-bit code * 256 wherever an 8-bit code exists, and in
 * between everywhere else.
 */
#include "cal_data.h"
#include "board_rev.h"

#include <math.h>
#include <stdio.h>

static int fails = 0;
#define CHECK(cond, ...) do { if (!(cond)) { \
        printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

/* Every 8-bit code's own voltage gives back that code in the high byte. */
static void test_agrees_with_8bit(void) {
    for (int path = 0; path < 3; path++) {
        for (int c8 = 0; c8 <= 255; c8 += 17) {
            float v = DAC_CAL[path].a + DAC_CAL[path].b * (float)c8;
            uint16_t c16 = dac_code16_for_volts(path, v);
            CHECK(c16 == (uint16_t)(c8 << 8) || c16 == (uint16_t)((c8 << 8) - 1) || c16 == (uint16_t)((c8 << 8) + 1),
                  "path %d code8 %d: code16 %u, want %u", path, c8, c16, (unsigned)(c8 << 8));
        }
    }
}

/* One 16-bit step is 1/256 of an 8-bit one, and the round trip is within half a step. */
static void test_resolution(void) {
    for (int path = 0; path < 3; path++) {
        float step = DAC_CAL[path].b / 256.0f;
        for (float v = DAC_CAL[path].a + 0.3f; v < DAC_CAL[path].a + DAC_CAL[path].b * 250.0f; v += 0.0937f) {
            uint16_t c16 = dac_code16_for_volts(path, v);
            float got = dac_volts_for_code16(path, c16);
            CHECK(fabsf(got - v) <= step * 0.5f + 1e-6f, "path %d %.5f V -> %.5f V (step %.6f)", path, v, got, step);
        }
    }
    /* The 5v path: about 76 uV a step on v2. */
    float d = dac_volts_for_code16(1, 30001) - dac_volts_for_code16(1, 30000);
    CHECK(fabsf(d - 76.125e-6f) < 0.5e-6f, "5v step %.3f uV", d * 1e6f);
}

/* 0 V on the bipolar 12v path sits between 8-bit codes; the 16-bit code gets within a step. */
static void test_12v_zero(void) {
    cal_data_select(BOARD_REV_V3);
    uint16_t c16 = dac_code16_for_volts(2, 0.0f);
    float got = dac_volts_for_code16(2, c16);
    CHECK(fabsf(got) < 0.000185f, "12v at 0 V: code %u gives %.6f V", c16, got);
    CHECK(c16 > 128 * 256 - 256 && c16 < 128 * 256 + 256, "12v zero code %u", c16);
    cal_data_select(0);
}

static void test_clamps(void) {
    CHECK(dac_code16_for_volts(1, -1.0f) == 0, "below range");
    CHECK(dac_code16_for_volts(1, 99.0f) == 65535, "above range");
    CHECK(dac_code16_for_volts(1, NAN) == 0, "NaN");
    CHECK(dac_code16_for_volts(2, -13.0f) == 0 && dac_code16_for_volts(2, 13.0f) == 65535, "12v ends");
}

int main(void) {
    test_agrees_with_8bit();
    test_resolution();
    test_12v_zero();
    test_clamps();
    if (fails) { printf("test_dac_code16: %d FAILED\n", fails); return 1; }
    printf("test_dac_code16: all passed\n");
    return 0;
}
