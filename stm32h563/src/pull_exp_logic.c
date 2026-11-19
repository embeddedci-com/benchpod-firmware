/* pull_exp_logic.c — the pure part of pull_exp.h (host tests). */
#include "pull_exp.h"

#include <string.h>

#define PORT_MASK 0x3FFFu   /* LA1..LA14 */

tca6416_regs_t pull_exp_reset_regs(void) { return (tca6416_regs_t){ .out = 0xFFFFu, .cfg = 0xFFFFu }; }

void pull_exp_regs(const uint8_t dir[PULL_EXP_CHANNELS + 1], const uint8_t str[PULL_EXP_CHANNELS + 1],
                   bool suspend_up, tca6416_regs_t *r2k2, tca6416_regs_t *r10k)
{
    /* An off channel's output latch stays high (the reset value), so turning a channel off
       touches only its configuration bit. */
    tca6416_regs_t a = pull_exp_reset_regs(), b = pull_exp_reset_regs();
    for (unsigned la = 1; la <= PULL_EXP_CHANNELS; la++) {
        uint16_t bit = (uint16_t)(1u << (la - 1u));
        if (dir[la] != PULL_EXP_UP && dir[la] != PULL_EXP_DOWN) continue;
        bool up = dir[la] == PULL_EXP_UP;
        if (up && suspend_up) continue;
        if (str[la] & PULL_STR_2K2) { a.cfg &= (uint16_t)~bit; if (!up) a.out &= (uint16_t)~bit; }
        if (str[la] & PULL_STR_10K) { b.cfg &= (uint16_t)~bit; if (!up) b.out &= (uint16_t)~bit; }
    }
    *r2k2 = a;
    *r10k = b;
}

static void add(pull_exp_write_t *out, size_t *n, uint8_t addr, uint8_t reg, uint16_t val)
{
    out[*n].addr = addr; out[*n].reg = reg; out[*n].val = val; (*n)++;
}

size_t pull_exp_plan(tca6416_regs_t f2, tca6416_regs_t f10, tca6416_regs_t t2, tca6416_regs_t t10,
                     pull_exp_write_t out[PULL_EXP_PLAN_MAX])
{
    size_t n = 0;
    /* A channel changes when any of its four bits differs on either expander. */
    uint16_t changed = (uint16_t)(((f2.cfg ^ t2.cfg) | (f2.out ^ t2.out) |
                                   (f10.cfg ^ t10.cfg) | (f10.out ^ t10.out)) & PORT_MASK);
    /* (1) off on both: a channel that was engaged and changes becomes an input first. */
    uint16_t off2 = (uint16_t)(f2.cfg | changed), off10 = (uint16_t)(f10.cfg | changed);
    if (off2 != f2.cfg)   add(out, &n, PULL_EXP_ADDR_2K2, TCA6416_REG_CONFIG0, off2);
    if (off10 != f10.cfg) add(out, &n, PULL_EXP_ADDR_10K, TCA6416_REG_CONFIG0, off10);
    /* (2) levels, while those channels are off */
    if (t2.out != f2.out)   add(out, &n, PULL_EXP_ADDR_2K2, TCA6416_REG_OUTPUT0, t2.out);
    if (t10.out != f10.out) add(out, &n, PULL_EXP_ADDR_10K, TCA6416_REG_OUTPUT0, t10.out);
    /* (3) on */
    if (t2.cfg != off2)   add(out, &n, PULL_EXP_ADDR_2K2, TCA6416_REG_CONFIG0, t2.cfg);
    if (t10.cfg != off10) add(out, &n, PULL_EXP_ADDR_10K, TCA6416_REG_CONFIG0, t10.cfg);
    return n;
}

pull_exp_dir_t pull_exp_channel(tca6416_regs_t r2, tca6416_regs_t r10, unsigned la, pull_str_t *str)
{
    if (str) *str = PULL_STR_NONE;
    if (la < 1u || la > PULL_EXP_CHANNELS) return PULL_EXP_OFF;
    uint16_t bit = (uint16_t)(1u << (la - 1u));
    bool on2 = !(r2.cfg & bit), on10 = !(r10.cfg & bit);
    if (!on2 && !on10) return PULL_EXP_OFF;
    bool up2 = (r2.out & bit) != 0, up10 = (r10.out & bit) != 0;
    if (on2 && on10 && up2 != up10) return PULL_EXP_OFF;   /* a divider: never written by plan */
    if (str) *str = (pull_str_t)((on2 ? PULL_STR_2K2 : 0) | (on10 ? PULL_STR_10K : 0));
    return (on2 ? up2 : up10) ? PULL_EXP_UP : PULL_EXP_DOWN;
}

const char *pull_str_ohms(pull_str_t s)
{
    switch (s) {
    case PULL_STR_10K:  return "10k";
    case PULL_STR_2K2:  return "2.2k";
    case PULL_STR_BOTH: return "1.8k";
    default:            return NULL;
    }
}

const char *pull_str_name(pull_str_t s)
{
    switch (s) {
    case PULL_STR_10K:  return "10k";
    case PULL_STR_2K2:  return "2.2k";
    case PULL_STR_BOTH: return "both";
    default:            return "none";
    }
}

bool pull_str_parse(const char *s, pull_str_t *out)
{
    if (!s) return false;
    if (!strcmp(s, "10k"))  { *out = PULL_STR_10K;  return true; }
    if (!strcmp(s, "2.2k") || !strcmp(s, "2k2")) { *out = PULL_STR_2K2; return true; }
    if (!strcmp(s, "both") || !strcmp(s, "1.8k")) { *out = PULL_STR_BOTH; return true; }
    return false;
}
