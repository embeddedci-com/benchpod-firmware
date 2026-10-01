/* Host unit test for current_out.c — the 4-20 mA output's transfer function (J9, XTR116).
 *
 * I = 100 * (AREF/102k + Vbuf/25.5k), Vbuf = code/65536 * AREF. The same constants are in
 * embeddedci-server/hwe2e/benchpod_current_loop_hw_test.go, which checked them on a pod against
 * a resistor in the loop (4.02 mA at 8-bit code 0, 20.02 mA at 8-bit code 255).
 */
#include "current_out.h"

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

int main(void) {
    test_range();
    test_matches_the_hardware_test();
    test_code_for_current();
    test_refusals();
    if (fails) { printf("test_current_out: %d FAILED\n", fails); return 1; }
    printf("test_current_out: all passed\n");
    return 0;
}
