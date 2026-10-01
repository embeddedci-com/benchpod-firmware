/* Host unit test for current_out.c — the 4-20 mA output's transfer function (J9, XTR116).
 *
 * Nominal: I = 100 * (AREF/102k + Vbuf/25.5k), Vbuf = code/65536 * AREF. The conversion uses a
 * fit per board revision (cal_data.c): the nominal values on v2, a measured line on rev3. The
 * cases below run on the v2 set unless they say otherwise.
 */
#include "current_out.h"
#include "board_rev.h"
#include "cal_data.h"

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

static int fails = 0;
#define CHECK(cond, ...) do { if (!(cond)) { \
        printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

/* The formula, written the way the hardware test has it, in mA for an 8-bit code. */
static double hw_test_ma(int code8) {
    double vbuf = (double)code8 / 256.0 * 4.096;
    return 100.0 * (4.096 / 102e3 + vbuf / 25.5e3) * 1000.0;
}

static void test_range(void) {
    CHECK(current_out_min_ua() == 4016, "min %ld uA", current_out_min_ua());
    CHECK(current_out_max_ua() == 20078, "max %ld uA", current_out_max_ua());
    CHECK(current_out_ua(0) == current_out_min_ua() && current_out_ua(0xFFFF) == current_out_max_ua(),
          "range does not match the end codes");
}

static void test_matches_the_hardware_test(void) {
    /* dac_out's 8-bit code sits in the high byte of the 16-bit one. */
    for (int c8 = 0; c8 <= 255; c8 += 51) {
        long want = lround(hw_test_ma(c8) * 1000.0);
        long got = current_out_ua((uint16_t)(c8 << 8));
        CHECK(got == want, "8-bit code %d: %ld uA, want %ld", c8, got, want);
    }
}

static void test_code_for_current(void) {
    uint16_t code = 0xFFFF;
    /* 4 mA is below the live zero and must still work: code 0. */
    CHECK(current_out_code(4000, &code) == NULL && code == 0, "4000 uA -> code %u", code);
    /* The ends of the range are reachable as reported (a few codes round to the same uA). */
    CHECK(current_out_code(current_out_min_ua(), &code) == NULL && current_out_ua(code) == current_out_min_ua(),
          "min -> code %u", code);
    CHECK(current_out_code(current_out_max_ua(), &code) == NULL && current_out_ua(code) == current_out_max_ua(),
          "max -> code %u", code);

    /* Every round-trip lands within half a step (0.245 uA), so within 1 uA after rounding. */
    for (long ua = 4100; ua <= 20000; ua += 137) {
        CHECK(current_out_code(ua, &code) == NULL, "%ld uA refused", ua);
        long got = current_out_ua(code);
        CHECK(labs(got - ua) <= 1, "%ld uA -> code %u -> %ld uA", ua, code, got);
    }

    /* Monotonic: a higher request never gives a lower code. */
    uint16_t prev = 0;
    for (long ua = 4000; ua <= current_out_max_ua(); ua++) {
        if (current_out_code(ua, &code) != NULL) { CHECK(0, "%ld uA refused", ua); break; }
        if (code < prev) { CHECK(0, "code fell at %ld uA", ua); break; }
        prev = code;
    }

    CHECK(current_out_code(12000, &code) == NULL && code == 32576, "12 mA -> code %u", code);
    CHECK(current_out_code(12000, NULL) == NULL, "NULL code pointer refused");
}

static void test_refusals(void) {
    uint16_t code = 1234;
    const char *why = current_out_code(3999, &code);
    CHECK(why && strstr(why, "3999 uA is out of range") && strstr(why, "4016 to 20078 uA"), "3999: %s", why ? why : "(allowed)");
    CHECK(code == 1234, "a refusal wrote the code");
    CHECK(current_out_code(0, &code) != NULL, "0 uA allowed: there is no broken-wire level");
    CHECK(current_out_code(-5000, &code) != NULL, "negative allowed");
    CHECK(current_out_code(current_out_max_ua() + 1, &code) != NULL, "above the top code allowed");
    CHECK(current_out_code(21000, &code) != NULL, "21 mA allowed");
}

/* The v2 fit is the nominal transfer function: nobody measured a v2 board. */
static void test_v2_fit_is_nominal(void) {
    cal_data_select(BOARD_REV_V2);
    CHECK(fabs((double)CURRENT_OUT_CAL.a - current_out_nominal_zero_ua()) < 0.001, "v2 zero %f", (double)CURRENT_OUT_CAL.a);
    CHECK(fabs((double)CURRENT_OUT_CAL.b - current_out_nominal_ua_per_code()) < 1e-7, "v2 step %f", (double)CURRENT_OUT_CAL.b);
}

/* A rev3 pod uses its own fit: the measured 4.056 mA at 8-bit code 0 and 20.032 mA at 8-bit
   code 255. The range the pod reports, and the code for a current, move with it. */
static void test_rev3_fit(void) {
    cal_data_select(BOARD_REV_V3);
    CHECK(current_out_min_ua() == 4056, "rev3 min %ld uA", current_out_min_ua());
    CHECK(current_out_ua(255u << 8) == 20032, "rev3 8-bit code 255: %ld uA", current_out_ua(255u << 8));
    CHECK(current_out_max_ua() == 20094, "rev3 max %ld uA", current_out_max_ua());

    uint16_t code = 0xFFFF, nominal = 0;
    CHECK(current_out_code(4000, &code) == NULL && code == 0, "rev3 4000 uA -> code %u", code);
    CHECK(current_out_code(12000, &code) == NULL && labs(current_out_ua(code) - 12000) <= 1, "rev3 12 mA -> code %u", code);
    cal_data_select(BOARD_REV_V2);
    current_out_code(12000, &nominal);
    cal_data_select(BOARD_REV_V3);
    CHECK(code < nominal, "rev3 12 mA code %u is not below the nominal %u (its zero is 40 uA higher)", code, nominal);

    const char *why = current_out_code(20095, &code);
    CHECK(why && strstr(why, "4056 to 20094 uA"), "rev3 refusal: %s", why ? why : "(allowed)");

    /* A measured fit stays close to the nominal one: a table typo must not pass. */
    CHECK(fabs((double)CURRENT_OUT_CAL.a - current_out_nominal_zero_ua()) < 100.0, "rev3 zero is far from nominal");
    CHECK(fabs((double)CURRENT_OUT_CAL.b / current_out_nominal_ua_per_code() - 1.0) < 0.01, "rev3 step is far from nominal");
    cal_data_select(BOARD_REV_V2);
}

int main(void) {
    test_v2_fit_is_nominal();
    test_rev3_fit();
    test_range();
    test_matches_the_hardware_test();
    test_code_for_current();
    test_refusals();
    if (fails) { printf("test_current_out: %d FAILED\n", fails); return 1; }
    printf("test_current_out: all passed\n");
    return 0;
}
