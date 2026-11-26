/* Host unit test for the digital v4 pull expanders (pull_exp_logic.c): the register map, the
 * netlist pin map, the write plan (no channel ever pulls two ways, at any step), suspended pull-ups, and the names. */
#include "pull_exp.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int fails = 0, checks = 0;
#define CHECK(cond, ...) do { checks++; if (!(cond)) { \
        printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

/* The digital v4 netlist (la-pulls.kicad_sch): channel -> { expander, 10k port, 2.2k port }.
   Written out as port names so a typo in the table does not hide in the same arithmetic. */
static void test_netlist_map(void)
{
    static const struct { unsigned la; int chip; const char *p10k, *p2k2; } nl[] = {
        { 1, PULL_EXP_U54, "P16", "P10" }, { 2, PULL_EXP_U54, "P17", "P15" },
        { 3, PULL_EXP_U54, "P02", "P01" }, { 4, PULL_EXP_U54, "P06", "P05" },
        { 5, PULL_EXP_U54, "P14", "P11" }, { 6, PULL_EXP_U54, "P07", "P04" },
        { 7, PULL_EXP_U54, "P03", "P00" }, { 8, PULL_EXP_U18, "P12", "P13" },
        { 9, PULL_EXP_U54, "P12", "P13" }, { 10, PULL_EXP_U18, "P10", "P11" },
        { 11, PULL_EXP_U18, "P06", "P07" }, { 12, PULL_EXP_U18, "P04", "P05" },
        { 13, PULL_EXP_U18, "P00", "P02" }, { 14, PULL_EXP_U18, "P01", "P03" },
    };
    #define PORTBIT(s) (((s)[1] - '0') * 8 + ((s)[2] - '0'))
    uint16_t used[2] = { 0, 0 };
    for (size_t i = 0; i < sizeof nl / sizeof nl[0]; i++) {
        const pull_exp_pin_t *m = &pull_exp_map[nl[i].la];
        CHECK(m->chip == nl[i].chip, "LA%u expander", nl[i].la);
        CHECK(m->bit10k == PORTBIT(nl[i].p10k), "LA%u 10k on %s", nl[i].la, nl[i].p10k);
        CHECK(m->bit2k2 == PORTBIT(nl[i].p2k2), "LA%u 2.2k on %s", nl[i].la, nl[i].p2k2);
        uint16_t bits = (uint16_t)((1u << m->bit10k) | (1u << m->bit2k2));
        CHECK((used[m->chip] & bits) == 0 && m->bit10k != m->bit2k2, "LA%u shares a port", nl[i].la);
        used[m->chip] |= bits;
    }
    CHECK(used[PULL_EXP_U54] == 0xFFFF, "U54 uses every port (%04x)", used[PULL_EXP_U54]);
    CHECK(used[PULL_EXP_U18] == 0x0FFF, "U18 uses P00-P13 (%04x)", used[PULL_EXP_U18]);
    #undef PORTBIT
}

static void test_map(void)
{
    uint8_t dir[15] = {0}, str[15] = {0};
    tca6416_regs_t a, b;   /* a = U18, b = U54 */
    pull_exp_regs(dir, str, false, &a, &b);
    CHECK(a.cfg == 0xFFFF && b.cfg == 0xFFFF && a.out == 0xFFFF && b.out == 0xFFFF, "all off = reset");

    dir[1] = PULL_EXP_UP;    str[1] = PULL_STR_2K2;    /* LA1 2.2k = U54 P10 */
    dir[8] = PULL_EXP_DOWN;  str[8] = PULL_STR_10K;    /* LA8 10k = U18 P12 */
    dir[9] = PULL_EXP_UP;    str[9] = PULL_STR_BOTH;   /* LA9 = U54 P12 + P13 */
    dir[14] = PULL_EXP_DOWN; str[14] = PULL_STR_BOTH;  /* LA14 = U18 P01 + P03 */
    pull_exp_regs(dir, str, false, &a, &b);
    CHECK(a.cfg == (uint16_t)~(0x0400 | 0x0002 | 0x0008), "U18 cfg %04x", a.cfg);
    CHECK(b.cfg == (uint16_t)~(0x0100 | 0x0400 | 0x0800), "U54 cfg %04x", b.cfg);
    CHECK(a.out == (uint16_t)~(0x0400 | 0x0002 | 0x0008), "U18 out %04x", a.out);
    CHECK(b.out == 0xFFFF, "U54 out %04x", b.out);
    /* U18 P14-P17 (bits 12-15) are never outputs */
    CHECK((a.cfg & 0xF000) == 0xF000, "U18 P14-P17 stay inputs");

    pull_str_t s;
    CHECK(pull_exp_channel(a, b, 1, &s) == PULL_EXP_UP && s == PULL_STR_2K2, "LA1 read back");
    CHECK(pull_exp_channel(a, b, 8, &s) == PULL_EXP_DOWN && s == PULL_STR_10K, "LA8 read back");
    CHECK(pull_exp_channel(a, b, 9, &s) == PULL_EXP_UP && s == PULL_STR_BOTH, "LA9 read back");
    CHECK(pull_exp_channel(a, b, 14, &s) == PULL_EXP_DOWN && s == PULL_STR_BOTH, "LA14 read back");
    CHECK(pull_exp_channel(a, b, 2, &s) == PULL_EXP_OFF && s == PULL_STR_NONE, "LA2 off");

    /* Suspended pull-ups: LA1 and LA9 off, the pull-downs stay. */
    pull_exp_regs(dir, str, true, &a, &b);
    CHECK(pull_exp_channel(a, b, 1, NULL) == PULL_EXP_OFF && pull_exp_channel(a, b, 9, NULL) == PULL_EXP_OFF,
          "suspended ups off");
    CHECK(pull_exp_channel(a, b, 8, NULL) == PULL_EXP_DOWN && pull_exp_channel(a, b, 14, NULL) == PULL_EXP_DOWN,
          "suspended: downs stay");

    /* Every channel and strength reads back as written, and touches only its own two ports. */
    for (unsigned la = 1; la <= 14; la++)
        for (unsigned st = 1; st <= 3; st++)
            for (unsigned d = 1; d <= 2; d++) {
                uint8_t dd[15] = {0}, ss[15] = {0};
                dd[la] = (uint8_t)d; ss[la] = (uint8_t)st;
                pull_exp_regs(dd, ss, false, &a, &b);
                CHECK(pull_exp_channel(a, b, la, &s) == (pull_exp_dir_t)d && s == (pull_str_t)st,
                      "LA%u str %u dir %u read back", la, st, d);
                for (unsigned o = 1; o <= 14; o++)
                    if (o != la) CHECK(pull_exp_channel(a, b, o, NULL) == PULL_EXP_OFF, "LA%u leaks to LA%u", la, o);
            }
}

/* Apply a plan one write at a time; after each, no channel may be engaged on both expanders in
   opposite directions, and no channel may pull in a direction that is neither its old nor its
   new one. */
static void apply_checked(tca6416_regs_t *a, tca6416_regs_t *b, tca6416_regs_t ta, tca6416_regs_t tb, int round)
{
    tca6416_regs_t fa = *a, fb = *b;
    pull_exp_write_t w[PULL_EXP_PLAN_MAX];
    size_t n = pull_exp_plan(*a, *b, ta, tb, w);
    CHECK(n <= PULL_EXP_PLAN_MAX, "plan size");
    for (size_t i = 0; i < n; i++) {
        tca6416_regs_t *r = w[i].addr == PULL_EXP_ADDR_U18 ? a : w[i].addr == PULL_EXP_ADDR_U54 ? b : NULL;
        CHECK(r != NULL, "address %02x", w[i].addr);
        CHECK(w[i].reg == TCA6416_REG_OUTPUT0 || w[i].reg == TCA6416_REG_CONFIG0, "register %02x", w[i].reg);
        if (!r) return;
        if (w[i].reg == TCA6416_REG_OUTPUT0) r->out = w[i].val; else r->cfg = w[i].val;
        for (unsigned la = 1; la <= 14; la++) {
            const pull_exp_pin_t *m = &pull_exp_map[la];
            const tca6416_regs_t *c = m->chip == PULL_EXP_U18 ? a : b;
            uint16_t b10 = (uint16_t)(1u << m->bit10k), b2 = (uint16_t)(1u << m->bit2k2);
            if (!(c->cfg & b10) && !(c->cfg & b2))
                CHECK(((c->out >> m->bit10k) & 1u) == ((c->out >> m->bit2k2) & 1u),
                      "round %d step %zu: LA%u pulls two ways", round, i, la);
            pull_str_t s_now, s_old, s_new;
            pull_exp_dir_t now = pull_exp_channel(*a, *b, la, &s_now);
            pull_exp_dir_t old = pull_exp_channel(fa, fb, la, &s_old);
            pull_exp_dir_t neu = pull_exp_channel(ta, tb, la, &s_new);
            /* Partial strength between writes (one resistor on before the other) is fine; a
               direction that is neither the old nor the new one is not. */
            bool ok = now == PULL_EXP_OFF || now == old || now == neu;
            CHECK(ok, "round %d step %zu: LA%u pulls a way that is neither old nor new", round, i, la);
            (void)s_now; (void)s_old; (void)s_new;
        }
    }
    CHECK(a->cfg == ta.cfg && a->out == ta.out && b->cfg == tb.cfg && b->out == tb.out,
          "round %d: plan does not reach the target", round);
    /* Nothing to do: no writes. */
    CHECK(pull_exp_plan(*a, *b, ta, tb, w) == 0, "a second plan writes again");
}

static void test_plan_random(void)
{
    srand(12345);
    uint8_t dir[15] = {0}, str[15] = {0};
    tca6416_regs_t a = pull_exp_reset_regs(), b = pull_exp_reset_regs();
    for (int round = 0; round < 3000; round++) {
        int changes = 1 + rand() % 4;
        for (int c = 0; c < changes; c++) {
            unsigned la = 1 + (unsigned)(rand() % 14);
            dir[la] = (uint8_t)(rand() % 3);
            str[la] = dir[la] ? (uint8_t)(1 + rand() % 3) : 0;
        }
        bool suspend = (rand() % 5) == 0;
        tca6416_regs_t ta, tb;
        pull_exp_regs(dir, str, suspend, &ta, &tb);
        apply_checked(&a, &b, ta, tb, round);
    }
    /* A flip up -> down on both resistors goes through off. */
    memset(dir, 0, sizeof dir); memset(str, 0, sizeof str);
    dir[5] = PULL_EXP_UP; str[5] = PULL_STR_BOTH;
    tca6416_regs_t ta, tb;
    pull_exp_regs(dir, str, false, &ta, &tb);
    a = pull_exp_reset_regs(); b = pull_exp_reset_regs();
    apply_checked(&a, &b, ta, tb, -1);
    dir[5] = PULL_EXP_DOWN;
    pull_exp_regs(dir, str, false, &ta, &tb);
    pull_exp_write_t w[PULL_EXP_PLAN_MAX];
    size_t n = pull_exp_plan(a, b, ta, tb, w);
    /* LA5's two resistors are both on U54: three writes there, nothing on U18. */
    CHECK(n == 3 && w[0].reg == TCA6416_REG_CONFIG0 && w[1].reg == TCA6416_REG_OUTPUT0 &&
          w[2].reg == TCA6416_REG_CONFIG0 && w[0].addr == PULL_EXP_ADDR_U54 &&
          w[1].addr == PULL_EXP_ADDR_U54 && w[2].addr == PULL_EXP_ADDR_U54, "flip: off, levels, on (%zu)", n);
    apply_checked(&a, &b, ta, tb, -2);
}

static void test_names(void)
{
    pull_str_t s;
    CHECK(pull_str_parse("10k", &s) && s == PULL_STR_10K, "10k");
    CHECK(pull_str_parse("2.2k", &s) && s == PULL_STR_2K2, "2.2k");
    CHECK(pull_str_parse("both", &s) && s == PULL_STR_BOTH, "both");
    CHECK(!pull_str_parse("4.7k", &s) && !pull_str_parse(NULL, &s), "bad strength");
    CHECK(!strcmp(pull_str_ohms(PULL_STR_BOTH), "1.8k") && pull_str_ohms(PULL_STR_NONE) == NULL, "ohms");
    CHECK(!strcmp(pull_str_name(PULL_STR_2K2), "2.2k") && !strcmp(pull_str_name(PULL_STR_NONE), "none"), "names");
}

/* Why asked-for pulls are off the wire (the `la` replies' "held"). */
static void test_hold_names(void)
{
    #define EQ(a, b) ((a) == NULL ? (b) == NULL : (b) != NULL && strcmp((a), (b)) == 0)
    CHECK(EQ(pull_exp_hold_name(true,  true,  false, true),  NULL), "live, not suspended");
    CHECK(EQ(pull_exp_hold_name(true,  true,  true,  false), NULL), "suspended, only downs asked");
    CHECK(EQ(pull_exp_hold_name(true,  true,  true,  true),  "dut_off"), "suspended pull-up");
    CHECK(EQ(pull_exp_hold_name(false, false, false, false), "pin21_down"), "pin 21 down");
    CHECK(EQ(pull_exp_hold_name(false, false, true,  true),  "pin21_down"), "pin 21 down wins");
    CHECK(EQ(pull_exp_hold_name(false, true,  false, true),  "fault"), "held after a failed write");
    /* What the chips hold after reset: every channel off, whatever was asked. */
    tca6416_regs_t r = pull_exp_reset_regs();
    for (unsigned la = 1; la <= PULL_EXP_CHANNELS; la++)
        CHECK(pull_exp_channel(r, r, la, NULL) == PULL_EXP_OFF, "LA%u off in reset", la);
    /* Suspended: a pull-up asked for is not engaged, a pull-down is. */
    uint8_t dir[PULL_EXP_CHANNELS + 1] = {0}, str[PULL_EXP_CHANNELS + 1] = {0};
    dir[2] = PULL_EXP_UP;   str[2] = PULL_STR_10K;
    dir[3] = PULL_EXP_DOWN; str[3] = PULL_STR_2K2;
    tca6416_regs_t a, b;
    pull_exp_regs(dir, str, true, &a, &b);
    CHECK(pull_exp_channel(a, b, 2, NULL) == PULL_EXP_OFF, "suspended up is off");
    CHECK(pull_exp_channel(a, b, 3, NULL) == PULL_EXP_DOWN, "down stays");
    #undef EQ
}

int main(void)
{
    test_netlist_map();
    test_map();
    test_hold_names();
    test_plan_random();
    test_names();
    printf("test_pull_exp: %d checks, %s\n", checks, fails ? "FAILED" : "all passed");
    return fails ? 1 : 0;
}
