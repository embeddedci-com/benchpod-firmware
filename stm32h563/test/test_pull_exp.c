/* Host unit test for the digital v3r4 pull expanders (pull_exp_logic.c): the register map, the
 * write plan (no channel ever pulls two ways, at any step), suspended pull-ups, and the names. */
#include "pull_exp.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int fails = 0, checks = 0;
#define CHECK(cond, ...) do { checks++; if (!(cond)) { \
        printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

static void test_map(void)
{
    uint8_t dir[15] = {0}, str[15] = {0};
    tca6416_regs_t a, b;
    pull_exp_regs(dir, str, false, &a, &b);
    CHECK(a.cfg == 0xFFFF && b.cfg == 0xFFFF && a.out == 0xFFFF && b.out == 0xFFFF, "all off = reset");

    dir[1] = PULL_EXP_UP;    str[1] = PULL_STR_2K2;    /* LA1 = P00 of U18 */
    dir[8] = PULL_EXP_DOWN;  str[8] = PULL_STR_10K;    /* LA8 = P07 of U54 */
    dir[9] = PULL_EXP_UP;    str[9] = PULL_STR_BOTH;   /* LA9 = P10 of both */
    dir[14] = PULL_EXP_DOWN; str[14] = PULL_STR_BOTH;  /* LA14 = P15 of both */
    pull_exp_regs(dir, str, false, &a, &b);
    CHECK(a.cfg == (uint16_t)~(0x0001 | 0x0100 | 0x2000), "U18 cfg %04x", a.cfg);
    CHECK(b.cfg == (uint16_t)~(0x0080 | 0x0100 | 0x2000), "U54 cfg %04x", b.cfg);
    CHECK(a.out == (uint16_t)~0x2000, "U18 out %04x", a.out);
    CHECK(b.out == (uint16_t)~(0x0080 | 0x2000), "U54 out %04x", b.out);
    /* P16, P17 (bits 14, 15) are never outputs */
    CHECK((a.cfg & 0xC000) == 0xC000 && (b.cfg & 0xC000) == 0xC000, "P16/P17 stay inputs");

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
        tca6416_regs_t *r = w[i].addr == PULL_EXP_ADDR_2K2 ? a : w[i].addr == PULL_EXP_ADDR_10K ? b : NULL;
        CHECK(r != NULL, "address %02x", w[i].addr);
        CHECK(w[i].reg == TCA6416_REG_OUTPUT0 || w[i].reg == TCA6416_REG_CONFIG0, "register %02x", w[i].reg);
        if (!r) return;
        if (w[i].reg == TCA6416_REG_OUTPUT0) r->out = w[i].val; else r->cfg = w[i].val;
        for (unsigned la = 1; la <= 14; la++) {
            uint16_t bit = (uint16_t)(1u << (la - 1));
            bool on_a = !(a->cfg & bit), on_b = !(b->cfg & bit);
            if (on_a && on_b)
                CHECK(((a->out ^ b->out) & bit) == 0, "round %d step %zu: LA%u pulls two ways", round, i, la);
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
    CHECK(n == 6 && w[0].reg == TCA6416_REG_CONFIG0 && w[1].reg == TCA6416_REG_CONFIG0 &&
          w[2].reg == TCA6416_REG_OUTPUT0 && w[3].reg == TCA6416_REG_OUTPUT0, "flip: off, levels, on (%zu)", n);
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

int main(void)
{
    test_map();
    test_plan_random();
    test_names();
    printf("test_pull_exp: %d checks, %s\n", checks, fails ? "FAILED" : "all passed");
    return fails ? 1 : 0;
}
