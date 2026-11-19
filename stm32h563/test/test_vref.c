/* Host unit test for J1 pin 21 on digital v3r4 (vref_logic.c): the setpoint, the state bands,
 * the rail-up hysteresis the pull expanders use, and the JSON fields. */
#include "vref_out.h"

#include <stdio.h>
#include <string.h>

static int fails = 0, checks = 0;
#define CHECK(cond, ...) do { checks++; if (!(cond)) { \
        printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

static void test_setpoint(void)
{
    CHECK(vref_setpoint_mv(VREF_SRC_BANK, 1800) == 1800, "1.8 V bank");
    CHECK(vref_setpoint_mv(VREF_SRC_BANK, 3300) == 3300, "3.3 V bank");
    /* Unset: the bank mux comes up on 3.3 V (its pull-up), so pin 21 does too. */
    CHECK(vref_setpoint_mv(VREF_SRC_BANK, 0) == 3300, "unset bank");
    CHECK(vref_setpoint_mv(VREF_SRC_5V, 3300) == 5000, "5 V");
}

static void test_classify(void)
{
    static const struct { vref_src_t src; int set, mv; vref_state_t want; } t[] = {
        { VREF_SRC_BANK, 3300, 3300, VREF_OK },
        { VREF_SRC_BANK, 3300, 3500, VREF_OK },     /* +200: still ok */
        { VREF_SRC_BANK, 3300, 3501, VREF_HIGH },   /* a 5 V signal lifting a 3.3 V pin 21 */
        { VREF_SRC_BANK, 3300, 4700, VREF_HIGH },
        { VREF_SRC_BANK, 3300, 3000, VREF_OK },     /* -300: still ok */
        { VREF_SRC_BANK, 3300, 2999, VREF_LOW },    /* overloaded */
        { VREF_SRC_BANK, 3300, 1649, VREF_SHORT },
        { VREF_SRC_BANK, 3300, 0,    VREF_SHORT },
        { VREF_SRC_BANK, 1800, 2001, VREF_HIGH },   /* 3.3 V signals with the pod at 1.8 V */
        { VREF_SRC_BANK, 1800, 1800, VREF_OK },
        { VREF_SRC_BANK, 1800, 1499, VREF_LOW },
        { VREF_SRC_BANK, 1800, 899,  VREF_SHORT },
        { VREF_SRC_5V,   5000, 4400, VREF_OK },     /* the USB supply: 4.4 to 5.1 V */
        { VREF_SRC_5V,   5000, 5100, VREF_OK },
        { VREF_SRC_5V,   5000, 4299, VREF_LOW },    /* 5 V did not come up */
        { VREF_SRC_5V,   5000, 5301, VREF_HIGH },
        { VREF_SRC_5V,   5000, 2499, VREF_SHORT },
        { VREF_SRC_BANK, 3300, -1,   VREF_UNKNOWN }, /* no reading */
    };
    for (unsigned i = 0; i < sizeof(t) / sizeof(t[0]); i++) {
        vref_state_t got = vref_classify(t[i].src, t[i].set, t[i].mv);
        CHECK(got == t[i].want, "%s %d mV at %d: %s, want %s", vref_src_name(t[i].src), t[i].set,
              t[i].mv, vref_state_name(got), vref_state_name(t[i].want));
    }
}

/* Up from 1500 mV, down below 1200 mV, unchanged without a reading. */
static void test_rail_up(void)
{
    CHECK(!vref_rail_up(false, 1499), "not up below 1500");
    CHECK(vref_rail_up(false, 1500), "up at 1500");
    CHECK(vref_rail_up(true, 1200), "stays up at 1200");
    CHECK(!vref_rail_up(true, 1199), "down below 1200");
    CHECK(vref_rail_up(true, -1) && !vref_rail_up(false, -1), "no reading keeps it");
    /* Every setting the pod makes is up: 1.8 V at its low band edge too. */
    CHECK(vref_rail_up(false, 1800 - VREF_LOW_MARGIN_MV), "1.8 V low edge is up");
}

static void test_emit(void)
{
    char buf[160];
    bp_emit_t e;
    bp_emit_init(&e, buf, sizeof(buf));
    vref_emit_fields(&e, VREF_SRC_5V, 5000, 4870, VREF_OK, true);
    CHECK(bp_emit_ok(&e) && !strcmp(buf, "\"vref_out\":{\"source\":\"5v\",\"set_mv\":5000,\"mv\":4870,"
                                          "\"state\":\"ok\",\"up\":true}"), "emit: %s", buf);
}

int main(void)
{
    test_setpoint();
    test_classify();
    test_rail_up();
    test_emit();
    printf("test_vref: %d checks, %s\n", checks, fails ? "FAILED" : "all passed");
    return fails ? 1 : 0;
}
