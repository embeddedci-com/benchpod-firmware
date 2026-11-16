/* Host unit test for the analog v3r2 DAC8554 layer (dac_quad.c) and the v3r2 bits around it:
 * the revision strap bands (board_rev.h), the v3r2 calibration set (cal_data.c) and the 4-20 mA
 * check in dac_limits.c.
 */
#include "dac_quad.h"
#include "board_rev.h"
#include "cal_data.h"
#include "dac_limits.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static int fails = 0;
#define CHECK(cond, ...) do { if (!(cond)) { \
        printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

#define BIT(ch) (1u << (ch))

/* The DAC8554 frame's top byte: A1:A0 = 00, LD1:LD0 = 01, channel in DB18:17, PD0 = 0. */
static void test_ctrl_byte(void) {
    CHECK(dac_quad_ctrl(DAC_QUAD_A) == 0x10, "A: %02x", dac_quad_ctrl(DAC_QUAD_A));
    CHECK(dac_quad_ctrl(DAC_QUAD_B) == 0x12, "B: %02x", dac_quad_ctrl(DAC_QUAD_B));
    CHECK(dac_quad_ctrl(DAC_QUAD_C) == 0x14, "C: %02x", dac_quad_ctrl(DAC_QUAD_C));
    CHECK(dac_quad_ctrl(DAC_QUAD_D) == 0x16, "D: %02x", dac_quad_ctrl(DAC_QUAD_D));
    CHECK(dac_quad_ctrl(DAC_QUAD_NONE) == 0x00, "none: %02x", dac_quad_ctrl(DAC_QUAD_NONE));
    CHECK(dac_quad_ctrl(4) == 0x00, "out of range");
    CHECK(!strcmp(dac_quad_name(DAC_QUAD_C), "C") && !strcmp(dac_quad_name(DAC_QUAD_NONE), "none"), "names");
}

/* v3 divider ~3000 mV (~2930 with the internal pull-down left on), v3r2 2.2 k / 10 k ~2705 mV
   (~2590 with it on).  Anything outside both bands runs as v3, as before v3r2. */
static void test_strap_bands(void) {
    static const struct { int mv, rev, known; } t[] = {
        { 3400, BOARD_REV_V3, 1 }, { 3000, BOARD_REV_V3, 1 }, { 2930, BOARD_REV_V3, 1 }, { 2850, BOARD_REV_V3, 1 },
        { 2849, BOARD_REV_V3R2, 1 }, { 2705, BOARD_REV_V3R2, 1 }, { 2590, BOARD_REV_V3R2, 1 }, { 2450, BOARD_REV_V3R2, 1 },
        { 2449, BOARD_REV_V3, 0 }, { 1000, BOARD_REV_V3, 0 }, { 3401, BOARD_REV_V3, 0 }, { 3300, BOARD_REV_V3, 1 },
    };
    for (size_t i = 0; i < sizeof(t) / sizeof(t[0]); i++) {
        CHECK(board_rev_from_strap_mv(t[i].mv) == t[i].rev, "%d mV -> rev %d, want %d",
              t[i].mv, board_rev_from_strap_mv(t[i].mv), t[i].rev);
        CHECK(board_rev_strap_known(t[i].mv) == t[i].known, "%d mV known %d", t[i].mv, t[i].known);
    }
    CHECK(BOARD_REV_V3R2 > BOARD_REV_V3, "board_rev_is_v3() must hold on v3r2");
}

/* v3r2 has its own set, a copy of rev3's until it is measured. */
static void test_cal_set(void) {
    cal_data_select(BOARD_REV_V3);
    cal_set_t v3 = *cal_data();
    cal_data_select(BOARD_REV_V3R2);
    const cal_set_t *r2 = cal_data();
    CHECK(r2 != NULL && memcmp(&v3, r2, sizeof(v3)) == 0, "v3r2 set is not CAL_V3's copy");
    cal_data_select(BOARD_REV_V2);
    CHECK(memcmp(&v3, cal_data(), sizeof(v3)) != 0, "v2 set selected");
    /* The old boards' sets are untouched (spot values from cal_data.c before v3r2), and an
       unknown revision still gets v2's. */
    CHECK(cal_data()->dac[1].b == 0.019488f && cal_data()->adc_cal1.a == 65.832398f, "CAL_V2 changed");
    cal_data_select(BOARD_REV_UNKNOWN);
    CHECK(cal_data()->dac[1].b == 0.019488f, "unknown revision is not v2's set");
    cal_data_select(BOARD_REV_V3);
    CHECK(cal_data()->dac[1].b == 0.01953943f && cal_data()->current_out.a == 4025.338f, "CAL_V3 changed");
    cal_data_select(BOARD_REV_V2);
}

static void plan_is(analog_path_t path, int cur, uint8_t live,
                    bool touch, int ch, bool stop, uint8_t park) {
    dac_quad_plan_t p = dac_quad_plan(path, cur, live);
    CHECK(p.touch == touch && (!touch || (p.ch == ch && p.stop == stop && p.park == park)),
          "path %d from %s live %x: touch %d ch %s stop %d park %x, want %d %s %d %x",
          (int)path, dac_quad_name(cur), live, p.touch, dac_quad_name(p.ch), p.stop, p.park,
          touch, dac_quad_name(ch), stop, park);
}

static void test_plans(void) {
    const int A = DAC_QUAD_A, B = DAC_QUAD_B, C = DAC_QUAD_C, D = DAC_QUAD_D, N = DAC_QUAD_NONE;
    /* Paths that only move relays leave the DAC alone. */
    plan_is(ANALOG_PATH_ADC_EXT,    B, BIT(B), false, 0, false, 0);
    plan_is(ANALOG_PATH_CURRENT_IN, C, BIT(C), false, 0, false, 0);
    /* Each voltage path points at its channel; cal1 and cal2 share the 5 V / +-12 V channels. */
    plan_is(ANALOG_PATH_DAC_3V3, N, 0, true, A, false, 0);
    plan_is(ANALOG_PATH_DAC_5V,  N, 0, true, B, false, 0);
    plan_is(ANALOG_PATH_CAL1,    N, 0, true, B, false, 0);
    plan_is(ANALOG_PATH_DAC_12V, N, 0, true, C, false, 0);
    plan_is(ANALOG_PATH_CAL2,    N, 0, true, C, false, 0);
    /* Moving from one voltage output to another stops the stream and parks the one it left;
       re-selecting the same output keeps the stream. */
    plan_is(ANALOG_PATH_DAC_3V3, B, BIT(B), true, A, true, BIT(B));
    plan_is(ANALOG_PATH_DAC_5V,  B, BIT(B), true, B, false, 0);
    plan_is(ANALOG_PATH_CAL1,    B, BIT(B), true, B, false, 0);   /* cal1 keeps the 5 V output */
    plan_is(ANALOG_PATH_CAL2,    A, BIT(A) | BIT(C), true, C, true, BIT(A));
    /* One voltage output at a time: every other live one is parked, whatever the stream did. */
    plan_is(ANALOG_PATH_DAC_12V, D, BIT(A) | BIT(B), true, C, true, BIT(A) | BIT(B));
    /* Off: every live voltage output parked, a voltage stream stopped, a 4-20 mA stream kept. */
    plan_is(ANALOG_PATH_OFF, B, BIT(A) | BIT(B) | BIT(C), true, N, true, BIT(A) | BIT(B) | BIT(C));
    plan_is(ANALOG_PATH_OFF, D, BIT(B), true, D, false, BIT(B));
    plan_is(ANALOG_PATH_OFF, N, 0, true, N, false, 0);
    /* 4-20 mA (a current waveform) leaves the voltage outputs alone. */
    plan_is(ANALOG_PATH_CURRENT_OUT, B, BIT(B), true, D, true, 0);
    plan_is(ANALOG_PATH_CURRENT_OUT, N, BIT(A), true, D, false, 0);
    plan_is(ANALOG_PATH_CURRENT_OUT, D, 0, true, D, false, 0);
    /* `live` beyond A..C is ignored. */
    plan_is(ANALOG_PATH_OFF, N, 0xF8, true, N, false, 0);
}

/* 0 V on each output, or the dac_limits park level on the limited one. */
static void test_park_codes(void) {
    cal_data_select(BOARD_REV_V3R2);
    dac_limits_t none = {0};
    dac_limits_set_active(&none);
    for (int ch = DAC_QUAD_A; ch <= DAC_QUAD_C; ch++) {
        uint16_t c = dac_quad_park_code(ch);
        float v = dac_volts_for_code16(ch, c);
        /* The unipolar lines put 0 V just below code 0 (their offset is a few mV), so the park is
           code 0; the +-12 V line's 0 V is near mid-scale. */
        if (ch == DAC_QUAD_C) CHECK(fabsf(v) < 0.0002f && c > 32000 && c < 33500, "C park %u = %.6f V", c, v);
        else                  CHECK(c == 0 && v < 0.005f, "%s park %u = %.6f V", dac_quad_name(ch), c, v);
    }
    dac_limits_t lim = { .enabled = 1, .path = 1, .inverted = 1, .min_mv = 1000, .max_mv = 4000 };
    dac_limits_set_active(&lim);
    CHECK(dac_quad_park_code(DAC_QUAD_B) == (uint16_t)(dac_limits_park_code() << 8),
          "B park %u with limits on 5v, want %u", dac_quad_park_code(DAC_QUAD_B), dac_limits_park_code() << 8);
    CHECK(dac_quad_park_code(DAC_QUAD_A) == 0, "A park unchanged by limits on 5v");
    dac_limits_set_active(&none);
    cal_data_select(BOARD_REV_V2);
}

/* With limits set, current_out is refused where it shares the DAC and allowed on v3r2. */
static void test_limits_current_out(void) {
    dac_limits_t lim = { .enabled = 1, .path = 1, .inverted = 0, .min_mv = 0, .max_mv = 3000 };
    dac_limits_set_active(&lim);
    dac_quad_set_present(false);
    CHECK(dac_limits_check_current_out() != NULL, "v3: current_out allowed with limits set");
    CHECK(dac_limits_check_command("current_out", "{\"cmd\":\"current_out\",\"ua\":12000}") != NULL, "v3: JSON");
    dac_quad_set_present(true);
    CHECK(dac_limits_check_current_out() == NULL, "v3r2: current_out refused with limits set");
    CHECK(dac_limits_check_command("current_out", "{\"cmd\":\"current_out\",\"ua\":12000}") == NULL, "v3r2: JSON");
    dac_quad_set_present(false);
    dac_limits_t none = {0};
    dac_limits_set_active(&none);
}

/* The ADC low range keeps the +-12 V output (channel C) within +-5 V (dac_quad.h). */
static void test_low_range_guard(void) {
    cal_data_select(BOARD_REV_V3R2);
    const int C = DAC_QUAD_C;
    uint16_t c0 = dac_code16_for_volts(C, 0.0f), c49 = dac_code16_for_volts(C, 4.9f),
             c5 = dac_code16_for_volts(C, 5.0f), cm5 = dac_code16_for_volts(C, -5.0f),
             c6 = dac_code16_for_volts(C, 6.0f), cm6 = dac_code16_for_volts(C, -6.0f),
             c10 = dac_code16_for_volts(C, 10.0f);
    const char *why;

    /* Held levels: only C, only in the low range, and +-5 V itself still holds (the calibration
       points sit there). */
    CHECK(!dac_quad_low_check_hold(true, C, c49), "4.9 V held in low");
    CHECK(!dac_quad_low_check_hold(true, C, c5) && !dac_quad_low_check_hold(true, C, cm5), "+-5 V held in low");
    CHECK(!dac_quad_low_check_hold(true, C, c0), "0 V (the park) held in low");
    why = dac_quad_low_check_hold(true, C, c6);
    CHECK(why && strstr(why, "6000 mV") && strstr(why, "low range") && strstr(why, "\"range\":\"normal\""),
          "6 V refused in low with what to do: %s", why ? why : "(null)");
    CHECK(why && !strcmp(dac_quad_low_last(), why), "last refusal kept");
    CHECK(dac_quad_low_check_hold(true, C, cm6), "-6 V refused in low");
    CHECK(dac_quad_low_check_hold(true, C, 65535) && dac_quad_low_check_hold(true, C, 0), "full scale refused in low");
    CHECK(!dac_quad_low_check_hold(false, C, c10) && !dac_quad_low_check_hold(false, C, 65535), "normal range: no limit");
    CHECK(!dac_quad_low_check_hold(true, DAC_QUAD_B, 65535) && !dac_quad_low_check_hold(true, DAC_QUAD_D, 65535),
          "other channels: no limit (5v tops out at ~5 V, D is the 4-20 mA output)");

    /* Streams on C. */
    CHECK(dac_quad_low_check_stream(true, C), "stream on C in low refused");
    CHECK(!dac_quad_low_check_stream(false, C), "stream on C in normal allowed");
    CHECK(!dac_quad_low_check_stream(true, DAC_QUAD_B) && !dac_quad_low_check_stream(true, DAC_QUAD_NONE),
          "stream elsewhere in low allowed");

    /* Entering the low range. */
    CHECK(!dac_quad_low_check_enter(true, c0) && !dac_quad_low_check_enter(true, c5) &&
          !dac_quad_low_check_enter(true, cm5), "enter low with C within +-5 V");
    why = dac_quad_low_check_enter(true, c10);
    CHECK(why && strstr(why, "10000 mV") && strstr(why, "\"path\":\"off\""), "enter low with C at 10 V: %s",
          why ? why : "(null)");
    why = dac_quad_low_check_enter(false, c0);
    CHECK(why && strstr(why, "waveform"), "enter low after a stream on C: %s", why ? why : "(null)");

    /* Commands, as the dispatcher sees them. */
    CHECK(dac_quad_low_check_command("dac_out", "{\"cmd\":\"dac_out\",\"path\":\"12v\",\"volts\":6}", true, DAC_QUAD_NONE),
          "dac_out 12v 6 V in low");
    CHECK(dac_quad_low_check_command("dac_out", "{\"cmd\":\"dac_out\",\"path\":\"dac_12v\",\"volts\":-6.5}", true, C),
          "dac_out dac_12v -6.5 V in low");
    CHECK(!dac_quad_low_check_command("dac_out", "{\"cmd\":\"dac_out\",\"path\":\"12v\",\"volts\":4.9}", true, C),
          "dac_out 12v 4.9 V in low");
    CHECK(!dac_quad_low_check_command("dac_out", "{\"cmd\":\"dac_out\",\"path\":\"12v\",\"volts\":-5}", true, C),
          "dac_out 12v -5 V in low");
    CHECK(!dac_quad_low_check_command("dac_out", "{\"cmd\":\"dac_out\",\"path\":\"12v\",\"volts\":10}", false, C),
          "dac_out 12v 10 V in normal");
    CHECK(!dac_quad_low_check_command("dac_out", "{\"cmd\":\"dac_out\",\"path\":\"5v\",\"volts\":4.9}", true, C),
          "dac_out 5v in low");
    CHECK(!dac_quad_low_check_command("dac_out", "{\"cmd\":\"dac_out\",\"path\":\"12v\"}", true, C),
          "dac_out 12v route only in low");
    CHECK(dac_quad_low_check_command("generate",
          "{\"cmd\":\"generate\",\"waveform\":\"sine\",\"freq\":10,\"path\":\"12v\",\"amplitude_v\":1,\"offset_v\":0}",
          true, DAC_QUAD_B), "generate on 12v in low");
    CHECK(!dac_quad_low_check_command("generate",
          "{\"cmd\":\"generate\",\"waveform\":\"sine\",\"freq\":10,\"path\":\"5v\",\"amplitude_v\":1,\"offset_v\":2}",
          true, C), "generate on 5v in low (routes away from C)");
    CHECK(dac_quad_low_check_command("generate",
          "{\"cmd\":\"generate\",\"waveform\":\"sine\",\"freq\":10,\"path\":\"5v\",\"route\":false,\"amplitude_v\":1,\"offset_v\":2}",
          true, C), "generate route:false plays on C");
    CHECK(!dac_quad_low_check_command("generate",
          "{\"cmd\":\"generate\",\"waveform\":\"sine\",\"freq\":10,\"amplitude_ua\":4000,\"offset_ua\":12000}",
          true, C), "generate on the 4-20 mA output in low");
    CHECK(dac_quad_low_check_command("generate", "{\"cmd\":\"generate\",\"waveform\":\"sine\",\"freq\":10}", true, C),
          "8-bit generate on C in low");
    static const char *const streams[] = { "dac_set", "load", "load_bin", "replay", "measure", "dac_control_loop" };
    for (size_t i = 0; i < sizeof(streams) / sizeof(streams[0]); i++) {
        CHECK(dac_quad_low_check_command(streams[i], "{}", true, C), "%s on C in low", streams[i]);
        CHECK(!dac_quad_low_check_command(streams[i], "{}", true, DAC_QUAD_B), "%s on B in low", streams[i]);
        CHECK(!dac_quad_low_check_command(streams[i], "{}", false, C), "%s on C in normal", streams[i]);
    }
    CHECK(!dac_quad_low_check_command("status", "{}", true, C), "other commands untouched");
    CHECK(!dac_quad_low_check_command("calibrate", "{\"cmd\":\"calibrate\",\"adc\":\"cal1\",\"range\":\"low\"}", true, C),
          "calibrate is checked where it drives, not refused here");
    cal_data_select(BOARD_REV_V3);
}

int main(void) {
    test_ctrl_byte();
    test_strap_bands();
    test_cal_set();
    test_plans();
    test_park_codes();
    test_limits_current_out();
    test_low_range_guard();
    if (fails) { printf("test_dac_quad: %d failure(s)\n", fails); return 1; }
    printf("test_dac_quad: all passed\n");
    return 0;
}
