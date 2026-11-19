/* Host unit test for the analog path table (analog_path.c) and the gate that decides whether a
 * board gets the analog v3r2 DAC8554 behaviour (dac_quad_for_board).
 *
 * Backwards compatibility: the v2/v3 rows below are the U55 mux and U58 relay writes
 * analog_path_set made before v3r2 existed, copied from that code.  A change to any of them
 * changes what an existing pod does and must fail here.
 */
#include "i2c_bus.h"
#include "dac_quad.h"
#include "board_rev.h"
#include "board_caps.h"

#include <stdio.h>
#include <string.h>

static int fails = 0;
#define CHECK(cond, ...) do { if (!(cond)) { \
        printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

typedef struct {
    analog_path_t path;
    bool mux; bool e1; uint8_t s1; bool e2; uint8_t s2;
    bool relays; bool cal1, cal2, cur, cp;
} row_t;

/* Before v3r2 (i2c_bus.c analog_path_set, firmware 3.9.4). */
static const row_t V3[] = {
    { ANALOG_PATH_OFF,         true, false, 0, false, 0, true, false, false, false, false },
    { ANALOG_PATH_DAC_3V3,     true, true,  0, false, 0, true, false, false, false, false },
    { ANALOG_PATH_DAC_5V,      true, true,  1, false, 0, true, false, false, false, false },
    { ANALOG_PATH_DAC_12V,     true, true,  2, true,  0, true, false, false, false, false },
    { ANALOG_PATH_ADC_EXT,     false, false, 0, false, 0, true, false, false, false, false },
    { ANALOG_PATH_CAL1,        true, true,  1, false, 0, true, true,  false, false, true  },
    { ANALOG_PATH_CAL2,        true, true,  3, true,  1, true, false, true,  false, true  },
    { ANALOG_PATH_CURRENT_IN,  false, false, 0, false, 0, true, false, false, true,  false },
    { ANALOG_PATH_CURRENT_OUT, true, false, 0, false, 0, false, false, false, false, false },
};

static void check_row(const row_t *w, bool quad) {
    analog_switches_t sw;
    memset(&sw, 0xA5, sizeof(sw));
    int rc = analog_path_switches(w->path, quad, &sw);
    const char *n = analog_path_name(w->path);
    CHECK(rc == 0, "%s: rc %d", n, rc);
    bool mux = w->mux && !quad;
    CHECK(sw.mux == mux, "%s quad=%d: mux %d, want %d", n, quad, sw.mux, mux);
    if (mux)
        CHECK(sw.ctrl1_en == w->e1 && sw.ctrl1_sel == w->s1 && sw.ctrl2_en == w->e2 && sw.ctrl2_sel == w->s2,
              "%s: ctrl1 %d/%u ctrl2 %d/%u, want %d/%u %d/%u", n, sw.ctrl1_en, sw.ctrl1_sel,
              sw.ctrl2_en, sw.ctrl2_sel, w->e1, w->s1, w->e2, w->s2);
    CHECK(sw.relays == w->relays, "%s quad=%d: relays %d, want %d", n, quad, sw.relays, w->relays);
    if (w->relays)
        CHECK(sw.cal1 == w->cal1 && sw.cal2 == w->cal2 && sw.current_in == w->cur && sw.cal_path == w->cp,
              "%s quad=%d: relays %d%d%d%d, want %d%d%d%d", n, quad, sw.cal1, sw.cal2, sw.current_in,
              sw.cal_path, w->cal1, w->cal2, w->cur, w->cp);
}

/* v2/v3: exactly the pre-v3r2 writes.  v3r2: the same relays, no U55. */
static void test_table(void) {
    CHECK(sizeof(V3) / sizeof(V3[0]) == ANALOG_PATH__COUNT, "every path has a row");
    for (size_t i = 0; i < sizeof(V3) / sizeof(V3[0]); i++) {
        check_row(&V3[i], false);
        check_row(&V3[i], true);
    }
    analog_switches_t sw;
    CHECK(analog_path_switches(ANALOG_PATH__COUNT, false, &sw) == -1, "bad path");
    CHECK(analog_path_switches(ANALOG_PATH_OFF, false, NULL) == -1, "NULL out");
}

/* The names and aliases every client sends (dac_out path=3v3|5v|12v on every board). */
static void test_names(void) {
    static const struct { const char *n; analog_path_t p; } t[] = {
        { "off", ANALOG_PATH_OFF }, { "3v3", ANALOG_PATH_DAC_3V3 }, { "dac_3v3", ANALOG_PATH_DAC_3V3 },
        { "5v", ANALOG_PATH_DAC_5V }, { "12v", ANALOG_PATH_DAC_12V }, { "ext", ANALOG_PATH_ADC_EXT },
        { "sma", ANALOG_PATH_ADC_EXT }, { "cal1", ANALOG_PATH_CAL1 }, { "cal2", ANALOG_PATH_CAL2 },
        { "current_in", ANALOG_PATH_CURRENT_IN }, { "current_out", ANALOG_PATH_CURRENT_OUT },
    };
    for (size_t i = 0; i < sizeof(t) / sizeof(t[0]); i++) {
        analog_path_t p = ANALOG_PATH__COUNT;
        CHECK(analog_path_from_name(t[i].n, &p) == 0 && p == t[i].p, "name %s", t[i].n);
    }
    analog_path_t p;
    CHECK(analog_path_from_name("dac_8v", &p) != 0, "unknown name");
    CHECK(!strcmp(analog_path_name(ANALOG_PATH_DAC_12V), "dac_12v"), "canonical name");
}

/* Only an analog board with the v3r2 strap gets the DAC8554 behaviour. */
static void test_gate(void) {
#define QUAD(rev) (board_caps_for_rev(rev).dac_quad)
    CHECK(!dac_quad_for_board(QUAD(BOARD_REV_V2), true, false), "v2");
    CHECK(!dac_quad_for_board(QUAD(BOARD_REV_V3), true, false), "v3 (analog-v3, v3r1)");
    CHECK(!dac_quad_for_board(QUAD(BOARD_REV_UNKNOWN), true, false), "unknown");
    CHECK(!dac_quad_for_board(QUAD(BOARD_REV_V3), false, false), "digital board");
    CHECK(!dac_quad_for_board(QUAD(BOARD_REV_V3R2), false, false), "v3r2 strap without the analog section");
    CHECK(dac_quad_for_board(QUAD(BOARD_REV_V3R2), true, false), "analog v3r2");
    CHECK(!dac_quad_for_board(QUAD(BOARD_REV_V3R4), true, false), "v3r4 strap");
    /* Digital v3r3 has the v3 strap; its analog module has the DAC8554. */
    CHECK(dac_quad_for_board(QUAD(BOARD_REV_V3), true, true), "digital v3r3 + analog module");
    CHECK(!dac_quad_for_board(QUAD(BOARD_REV_V3), false, false), "digital v3r3, no module");
#undef QUAD
    CHECK(!dac_quad_present(), "present before boot sets it");
}

/* U58's byte: the pre-v3r2 relay bits, P4 (ANA14_EN) only when asked for (v3r2), P5 (K5, the
   ADC low range) only when asked for (module v3r4). */
static void test_calsw_reg(void) {
    for (unsigned m = 0; m < 16; m++) {
        bool c1 = m & 1, c2 = m & 2, cur = m & 4, cp = m & 8;
        CHECK(analog_calsw_reg(c1, c2, cur, cp, false, false) == m, "relays %x: %02x", m,
              analog_calsw_reg(c1, c2, cur, cp, false, false));
        CHECK(analog_calsw_reg(c1, c2, cur, cp, true, false) == (m | 0x10u), "relays %x + ana14", m);
        CHECK(analog_calsw_reg(c1, c2, cur, cp, false, true) == (m | 0x20u), "relays %x + low range", m);
        CHECK(analog_calsw_reg(c1, c2, cur, cp, true, true) == (m | 0x30u), "relays %x + both", m);
    }
    CHECK(U58_LOW_RANGE_BIT == 0x20u, "K5 is P5");
}

/* U58's output pins: P5 is only ever an output on a board with K5. */
static void test_u58_mask(void) {
    CHECK(analog_u58_mask(false, false) == 0x0Fu, "v2/v3: P0..P3");
    CHECK(analog_u58_mask(true, false) == 0x1Fu, "v3r2 / module: P4 too");
    CHECK(analog_u58_mask(true, true) == 0x3Fu, "module v3r4: P5 too");
    CHECK((uint8_t)~analog_u58_mask(true, false) & U58_LOW_RANGE_BIT, "module v3r3: P5 stays an input");
}

/* The range: names, and which paths keep it. */
static void test_range(void) {
    adc_range_t r;
    CHECK(adc_range_from_name("low", &r) == 0 && r == ADC_RANGE_LOW, "low");
    CHECK(adc_range_from_name("normal", &r) == 0 && r == ADC_RANGE_NORMAL, "normal");
    CHECK(adc_range_from_name("high", &r) == -1 && adc_range_from_name(NULL, &r) == -1, "unknown");
    CHECK(strcmp(adc_range_name(ADC_RANGE_LOW), "low") == 0 && strcmp(adc_range_name(ADC_RANGE__COUNT), "") == 0,
          "names");
    static const struct { analog_path_t p; bool keeps; } keep[] = {
        { ANALOG_PATH_OFF, false }, { ANALOG_PATH_CAL2, false },
        { ANALOG_PATH_ADC_EXT, true }, { ANALOG_PATH_CAL1, true }, { ANALOG_PATH_CURRENT_IN, true },
        { ANALOG_PATH_DAC_3V3, true }, { ANALOG_PATH_DAC_5V, true }, { ANALOG_PATH_DAC_12V, true },
        { ANALOG_PATH_CURRENT_OUT, true }, { ANALOG_PATH__COUNT, false },
    };
    for (unsigned i = 0; i < sizeof(keep) / sizeof(keep[0]); i++)
        CHECK(analog_path_keeps_low_range(keep[i].p) == keep[i].keeps, "path %d keeps the range: %d",
              (int)keep[i].p, keep[i].keeps);
}

/* Analog module v3r5: current_in picks the low range by itself and gives it back. */
static void test_auto_range(void) {
    const adc_range_t N = ADC_RANGE_NORMAL, L = ADC_RANGE_LOW;
    bool a = true;
    /* Modules before v3r5 (249 ohm, with or without K5): current_in leaves the range as it is. */
    CHECK(analog_path_auto_range(ANALOG_PATH_CURRENT_IN, true, false, true, N, false, &a) == N && !a, "v3r4 normal");
    CHECK(analog_path_auto_range(ANALOG_PATH_CURRENT_IN, true, false, true, L, false, &a) == L && !a, "v3r4 low kept");
    CHECK(analog_path_auto_range(ANALOG_PATH_CURRENT_IN, true, false, false, N, false, &a) == N && !a, "no K5");
    /* v3r5: low, and marked as picked. */
    CHECK(analog_path_auto_range(ANALOG_PATH_CURRENT_IN, true, true, true, N, false, &a) == L && a, "v3r5 current_in");
    CHECK(analog_path_auto_range(ANALOG_PATH_CURRENT_IN, true, true, true, L, true, &a) == L && a, "again");
    CHECK(analog_path_auto_range(ANALOG_PATH_CURRENT_IN, true, true, true, L, false, &a) == L && !a, "user's low stays the user's");
    /* The next path that moves the relays gives a picked low range back... */
    static const analog_path_t others[] = { ANALOG_PATH_ADC_EXT, ANALOG_PATH_CAL1, ANALOG_PATH_DAC_3V3,
                                            ANALOG_PATH_DAC_5V, ANALOG_PATH_DAC_12V, ANALOG_PATH_OFF, ANALOG_PATH_CAL2 };
    for (unsigned i = 0; i < sizeof(others) / sizeof(others[0]); i++) {
        CHECK(analog_path_auto_range(others[i], true, true, true, L, true, &a) == N && !a, "path %d after current_in",
              (int)others[i]);
        /* ...but a low range the user asked for stays (off and cal2 reset it, as before). */
        bool keeps = analog_path_keeps_low_range(others[i]);
        CHECK(analog_path_auto_range(others[i], true, true, true, L, false, &a) == (keeps ? L : N) && !a,
              "path %d with the user's low", (int)others[i]);
    }
    /* current_out leaves the relays (current_in stays routed), so the range stays as it is. */
    CHECK(analog_path_auto_range(ANALOG_PATH_CURRENT_OUT, false, true, true, L, true, &a) == L && a, "current_out");
    CHECK(analog_path_auto_range(ANALOG_PATH_CURRENT_IN, true, true, true, N, false, NULL) == L, "NULL auto_out");

    /* Every module before v3r5 (249 ohm) gets exactly the rule of earlier firmware, on every path,
       in both ranges, with or without K5: off and cal2 give normal, every other path leaves the
       range, and nothing is ever marked as picked (cur_auto is never true on such a module). */
    for (int p = 0; p < (int)ANALOG_PATH__COUNT; p++)
    for (int k5 = 0; k5 < 2; k5++)
    for (int cr = 0; cr < 2; cr++) {
        analog_switches_t sw;
        CHECK(analog_path_switches((analog_path_t)p, false, &sw) == 0, "path %d", p);
        adc_range_t cur = cr ? L : N;
        adc_range_t before = (sw.relays && !analog_path_keeps_low_range((analog_path_t)p)) ? N : cur;
        a = true;
        CHECK(analog_path_auto_range((analog_path_t)p, sw.relays, false, k5 != 0, cur, false, &a) == before && !a,
              "249 ohm module, path %d, K5 %d, range %d: not what earlier firmware did", p, k5, cr);
    }
}

int main(void) {
    test_auto_range();
    test_table();
    test_calsw_reg();
    test_u58_mask();
    test_range();
    test_names();
    test_gate();
    if (fails) { printf("test_analog_path: %d failure(s)\n", fails); return 1; }
    printf("test_analog_path: all passed\n");
    return 0;
}
