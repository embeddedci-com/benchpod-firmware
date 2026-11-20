/* Host unit test for the revision straps (board_rev.h board_rev_decide) and the revision gate
 * (board_caps.c): digital v3r4 needs both straps, every doubt runs as v3, and only v3r4 gets
 * the new capabilities.
 *
 * The v3r4 straps are swept over 1 % resistors, a +3V3 rail of 3.20 to 3.40 V (VREF+ is the
 * rail through R1 47 R, about 10 mV lower), a VREFINT-based VREF+ reading off by up to 2 %, and
 * +-4 LSB of ADC error.  The firmware's mV value is the raw ratio times the measured VREF+, so
 * the ratio test does not depend on the VREFINT error.  The older boards' mV bands (unchanged by
 * v3r4) are swept over the narrower 3.25..3.35 V and 1 %: at the wide corners the v3 / v3r2 split
 * at 2850 mV has no margin left, which is how it was before v3r4 too.
 */
#include "board_caps.h"
#include "board_rev.h"

#include <stdio.h>

static int fails = 0;
static int checks = 0;
#define CHECK(cond, ...) do { checks++; if (!(cond)) { \
        printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

/* The pin voltage of a divider: rh from the rail to the pin, rl from the pin to GND. */
static double divider(double rail, double rh, double rl) { return rail * rl / (rh + rl); }

/* What the firmware sees: mV (raw * measured VREF+ / 4095) and the measured VREF+. */
static void reading(double pin_v, double vref_v, double vref_err, int lsb, int *mv, int *vref_mv)
{
    double raw = pin_v / vref_v * 4095.0 + lsb;
    if (raw > 4095.0) raw = 4095.0;
    if (raw < 0.0) raw = 0.0;
    double vref_meas = vref_v * (1.0 + vref_err) * 1000.0;
    *mv = (int)(raw * vref_meas / 4095.0 + 0.5);
    *vref_mv = (int)(vref_meas + 0.5);
}

typedef struct { const char *name; double rh, rl; int has_rev2; double rh2, rl2; int want; int narrow; } board_t;

static void sweep(const board_t *b)
{
    static const double wide_rails[] = { 3.20, 3.25, 3.30, 3.35, 3.40 };
    static const double narrow_rails[] = { 3.25, 3.27, 3.30, 3.33, 3.35 };
    static const double tol[] = { -0.01, 0.0, 0.01 };
    static const double wide_verr[] = { -0.02, 0.0, 0.02 };
    static const double narrow_verr[] = { -0.01, 0.0, 0.01 };
    const double *rails = b->narrow ? narrow_rails : wide_rails;
    const double *verr = b->narrow ? narrow_verr : wide_verr;
    static const int lsbs[] = { -4, 0, 4 };
    int bad = 0;
    for (unsigned r = 0; r < 5; r++)
    for (unsigned a = 0; a < 3; a++)
    for (unsigned c = 0; c < 3; c++)
    for (unsigned v = 0; v < 3; v++)
    for (unsigned l = 0; l < 3; l++) {
        double rail = rails[r], vref = rail - 0.010;
        int mv, vmv, mv2 = -1, vmv2;
        reading(divider(rail, b->rh * (1 + tol[a]), b->rl * (1 + tol[c])), vref, verr[v], lsbs[l], &mv, &vmv);
        /* The firmware reads PC3 only when PA3 says v3r4. */
        if (board_rev_strap_says_v3r4(mv, vmv)) {
            if (b->has_rev2)
                reading(divider(rail, b->rh2 * (1 + tol[c]), b->rl2 * (1 + tol[a])), vref, verr[v], lsbs[l], &mv2, &vmv2);
            else
                mv2 = b->rl2 < 0 ? -1 : (int)b->rl2;   /* a fixed reading: missing / stuck */
        }
        int rev = board_rev_decide(mv, vmv, mv2);
        if (rev != b->want && bad++ < 3)
            printf("  %s: rail %.2f tol %+.0f%%/%+.0f%% vref %+.0f%% lsb %+d: %d mV (%d) rev2 %d mV -> %d, want %d\n",
                   b->name, rail, tol[a] * 100, tol[c] * 100, verr[v] * 100, lsbs[l], mv,
                   board_rev_permille(mv, vmv), mv2, rev, b->want);
        CHECK(rev == b->want, "%s", b->name);
        /* An older board never reads its PC3, at any corner of the wide sweep either. */
        if (b->narrow) {
            int wmv, wvmv;
            reading(divider(wide_rails[r], b->rh * (1 + tol[a]), b->rl * (1 + tol[c])),
                    wide_rails[r] - 0.010, wide_verr[v], lsbs[l], &wmv, &wvmv);
            CHECK(!board_rev_strap_says_v3r4(wmv, wvmv), "%s: PC3 would be read (%d mV)", b->name, wmv);
        }
    }
}

static void test_straps(void)
{
    static const board_t boards[] = {
        /* v3, analog-v3r1 and every digital board before v3r4: 1k over 10k */
        { "v3",              1000,   10000, 0, 0, 0,      BOARD_REV_V3, 1 },
        /* analog v3r2: 2.2k over 10k */
        { "analog-v3r2",     2200,   10000, 0, 0, 0,      BOARD_REV_V3R2, 1 },
        /* digital v3r4: 1k over 100k, PC3 10k over 10k */
        { "digital-v3r4",    1000,  100000, 1, 10000, 10000, BOARD_REV_V3R4, 0 },
        /* PA3 says v3r4 but PC3 is not what v3r4 has: run as v3 */
        { "v3r4, PC3 0 V",   1000,  100000, 0, 0, 0.0001, BOARD_REV_V3, 0 },
        { "v3r4, PC3 rail",  1000,  100000, 0, 0, 3300,   BOARD_REV_V3, 0 },
        { "v3r4, PC3 unread",1000,  100000, 0, 0, -1,     BOARD_REV_V3, 0 },
        { "v3r4, PC3 1/3",   1000,  100000, 1, 20000, 10000, BOARD_REV_V3, 0 },
        { "v3r4, PC3 2/3",   1000,  100000, 1, 10000, 20000, BOARD_REV_V3, 0 },
    };
    for (unsigned i = 0; i < sizeof(boards) / sizeof(boards[0]); i++) sweep(&boards[i]);
}

/* The edges of the pure helpers. */
static void test_edges(void)
{
    CHECK(board_rev_permille(1650, 3300) == 500, "permille");
    CHECK(board_rev_permille(100, 0) == -1 && board_rev_permille(-1, 3300) == -1, "permille bad input");
    /* The old band function is unchanged: 3.27 V is v3 by itself. */
    CHECK(board_rev_from_strap_mv(3267) == BOARD_REV_V3, "3267 mV alone is v3");
    CHECK(board_rev_strap_known(3267), "3267 mV is a known strap");
    /* The split: below 3130 mV never v3r4, even at a high ratio. */
    CHECK(!board_rev_strap_says_v3r4(3130, 3300), "3130 of 3300 mV is 94.8 %%");
    CHECK(board_rev_strap_says_v3r4(3140, 3300), "3140 of 3300 mV is 95.2 %%");
    CHECK(!board_rev_strap_says_v3r4(3300, 3300), "stuck at the rail");
    CHECK(board_rev_strap_says_v3r4(3420, 3450), "v3r4 read with a high VREF+ (above the mV band)");
    /* The ratio: a v3 board read with a VREF+ that came out low must not pass. */
    CHECK(!board_rev_strap_says_v3r4(3135, 3450), "3135 of 3450 mV is 90.9 %%");
    CHECK(board_rev_decide(3267, 3300, 1650) == BOARD_REV_V3R4, "nominal v3r4");
    CHECK(board_rev_decide(3267, 3300, -1) == BOARD_REV_V3, "PC3 not read");
    CHECK(board_rev_decide(2705, 3300, 1650) == BOARD_REV_V3R2, "v3r2 never reads as v3r4");
    CHECK(board_rev_decide(3000, 3300, 1650) == BOARD_REV_V3, "v3 with a v3r4-like PC3 stays v3");
    CHECK(BOARD_REV_V3R4 > BOARD_REV_V3, "board_rev_is_v3() holds on v3r4");
}

static int caps_none(board_caps_t c)
{
    return !c.vref_out && !c.la_5v && !c.la_clamps && !c.pull_expanders && !c.usb_sel && !c.cn1_dac_swap;
}

static int caps_v3(board_caps_t c) { return c.la_1v8 && c.nrst_pin && c.usb_cc; }
static int caps_v2(board_caps_t c) { return !c.la_1v8 && !c.nrst_pin && !c.usb_cc; }

/* The v3 gates were board_rev_is_v3() (rev >= BOARD_REV_V3); the quad DAC and the calibration
   set compared the revision directly.  The same answers for every revision, known or not. */
static void test_rev_gates(void)
{
    const board_caps_t z = *board_caps();
    CHECK(caps_v2(z) && !z.dac_quad && z.cal_set == BOARD_CAL_V2, "before init: as v2");
    static const int revs[] = { BOARD_REV_UNKNOWN, 1, BOARD_REV_V2, BOARD_REV_V3, BOARD_REV_V3R2,
                                BOARD_REV_V3R4, 6, 99, -1 };
    for (unsigned i = 0; i < sizeof(revs) / sizeof(revs[0]); i++) {
        const int r = revs[i];
        const board_caps_t c = board_caps_for_rev(r);
        CHECK(r >= BOARD_REV_V3 ? caps_v3(c) : caps_v2(c), "rev %d v3 gates", r);
        CHECK(c.dac_quad == (r == BOARD_REV_V3R2), "rev %d dac_quad", r);
        const board_cal_set_t want = r == BOARD_REV_V3R2 ? BOARD_CAL_V3R2
                                   : (r == BOARD_REV_V3 || r == BOARD_REV_V3R4) ? BOARD_CAL_V3 : BOARD_CAL_V2;
        CHECK(c.cal_set == want, "rev %d cal_set %d, want %d", r, c.cal_set, want);
    }
}

static void test_caps(void)
{
    /* Before board_caps_init(): nothing new. */
    CHECK(caps_none(*board_caps()), "caps before init");
    static const int old[] = { BOARD_REV_UNKNOWN, BOARD_REV_V2, BOARD_REV_V3, BOARD_REV_V3R2, 1, 6, 99, -1 };
    for (unsigned i = 0; i < sizeof(old) / sizeof(old[0]); i++)
        CHECK(caps_none(board_caps_for_rev(old[i])), "rev %d has no v3r4 caps", old[i]);
    board_caps_t c = board_caps_for_rev(BOARD_REV_V3R4);
    CHECK(c.vref_out && c.la_5v && c.la_clamps && c.pull_expanders && c.usb_sel && c.cn1_dac_swap, "v3r4 caps");
}

static void test_init(void)
{
    board_caps_init(BOARD_REV_V3);
    CHECK(caps_none(*board_caps()), "v3 after init");
    board_caps_init(BOARD_REV_V3R4);
    CHECK(board_caps()->pull_expanders, "v3r4 after init");
    board_caps_init(BOARD_REV_V3R2);
    CHECK(caps_none(*board_caps()), "v3r2 after init");
}

/* The analog part: analog on the board until board_variant_init() fills it (board_caps_init()
   alone leaves it so), every revision reports it unprobed, and latching a revision keeps what
   board_variant latched. */
static void test_analog(void)
{
    const board_analog_caps_t a = board_caps()->analog;
    CHECK(a.present && !a.module && !a.adc_low_range && !a.current_in_100r, "analog before boot");
    static const int revs[] = { BOARD_REV_UNKNOWN, BOARD_REV_V2, BOARD_REV_V3, BOARD_REV_V3R2, BOARD_REV_V3R4, 99 };
    for (unsigned i = 0; i < sizeof(revs) / sizeof(revs[0]); i++) {
        const board_analog_caps_t r = board_caps_for_rev(revs[i]).analog;
        CHECK(r.present && !r.module && !r.adc_low_range && !r.current_in_100r, "rev %d analog unprobed", revs[i]);
    }
    const board_analog_caps_t mod = { .present = true, .module = true, .current_in_100r = true };
    board_caps_set_analog(mod);
    CHECK(board_caps()->analog.module && board_caps()->analog.current_in_100r, "set_analog latches");
    board_caps_init(BOARD_REV_V3R4);
    CHECK(board_caps()->analog.module && board_caps()->analog.current_in_100r
          && !board_caps()->analog.adc_low_range && board_caps()->pull_expanders, "init keeps the analog part");
    board_caps_set_analog((board_analog_caps_t){ 0 });
    CHECK(!board_caps()->analog.present && board_caps()->pull_expanders, "set_analog keeps the revision part");
}

int main(void)
{
    test_rev_gates();   /* first: it checks the state before board_caps_init() */
    test_caps();
    test_init();
    test_analog();   /* last: it latches an analog part */
    test_edges();
    test_straps();
    printf("test_board_caps: %d checks, %s\n", checks, fails ? "FAILED" : "all passed");
    return fails ? 1 : 0;
}
