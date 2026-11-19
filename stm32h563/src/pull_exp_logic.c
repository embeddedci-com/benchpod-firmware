/* pull_exp_logic.c — the pure part of pull_exp.h (host tests). */
#include "pull_exp.h"

#include <string.h>

/* digital v3r4, from the netlist of la-pulls.kicad_sch (U54 placed at 90 deg, U18 at 270 deg):
   { expander, 10k port, 2.2k port }, port bit 0-7 = P00-P07, 8-15 = P10-P17. */
#define P0(n) (n)
#define P1(n) (8 + (n))
const pull_exp_pin_t pull_exp_map[PULL_EXP_CHANNELS + 1] = {
    [1]  = { PULL_EXP_U54, P1(6), P1(0) },   /* R38  / R210 */
    [2]  = { PULL_EXP_U54, P1(7), P1(5) },   /* R50  / R211 */
    [3]  = { PULL_EXP_U54, P0(2), P0(1) },   /* R58  / R212 */
    [4]  = { PULL_EXP_U54, P0(6), P0(5) },   /* R62  / R213 */
    [5]  = { PULL_EXP_U54, P1(4), P1(1) },   /* R51  / R214 */
    [6]  = { PULL_EXP_U54, P0(7), P0(4) },   /* R39  / R215 */
    [7]  = { PULL_EXP_U54, P0(3), P0(0) },   /* R63  / R216 */
    [8]  = { PULL_EXP_U18, P1(2), P1(3) },   /* R59  / R217 */
    [9]  = { PULL_EXP_U54, P1(2), P1(3) },   /* R40  / R218 */
    [10] = { PULL_EXP_U18, P1(0), P1(1) },   /* R41  / R219 */
    [11] = { PULL_EXP_U18, P0(6), P0(7) },   /* R52  / R220 */
    [12] = { PULL_EXP_U18, P0(4), P0(5) },   /* R53  / R221 */
    [13] = { PULL_EXP_U18, P0(0), P0(2) },   /* R60  / R222 */
    [14] = { PULL_EXP_U18, P0(1), P0(3) },   /* R61  / R223 */
};
#undef P0
#undef P1

tca6416_regs_t pull_exp_reset_regs(void) { return (tca6416_regs_t){ .out = 0xFFFFu, .cfg = 0xFFFFu }; }

static uint16_t bit_of(unsigned b) { return (uint16_t)(1u << b); }

/* Both port bits of one channel. */
static uint16_t chan_mask(unsigned la)
{
    return (uint16_t)(bit_of(pull_exp_map[la].bit10k) | bit_of(pull_exp_map[la].bit2k2));
}

void pull_exp_regs(const uint8_t dir[PULL_EXP_CHANNELS + 1], const uint8_t str[PULL_EXP_CHANNELS + 1],
                   bool suspend_up, tca6416_regs_t *r18, tca6416_regs_t *r54)
{
    /* An off resistor's output latch stays high (the reset value), so turning a channel off
       touches only its configuration bits. */
    tca6416_regs_t r[2] = { pull_exp_reset_regs(), pull_exp_reset_regs() };
    for (unsigned la = 1; la <= PULL_EXP_CHANNELS; la++) {
        if (dir[la] != PULL_EXP_UP && dir[la] != PULL_EXP_DOWN) continue;
        bool up = dir[la] == PULL_EXP_UP;
        if (up && suspend_up) continue;
        const pull_exp_pin_t *m = &pull_exp_map[la];
        uint16_t on = 0;
        if (str[la] & PULL_STR_10K) on |= bit_of(m->bit10k);
        if (str[la] & PULL_STR_2K2) on |= bit_of(m->bit2k2);
        r[m->chip].cfg &= (uint16_t)~on;
        if (!up) r[m->chip].out &= (uint16_t)~on;
    }
    *r18 = r[PULL_EXP_U18];
    *r54 = r[PULL_EXP_U54];
}

static void add(pull_exp_write_t *out, size_t *n, uint8_t addr, uint8_t reg, uint16_t val)
{
    out[*n].addr = addr; out[*n].reg = reg; out[*n].val = val; (*n)++;
}

size_t pull_exp_plan(tca6416_regs_t f18, tca6416_regs_t f54, tca6416_regs_t t18, tca6416_regs_t t54,
                     pull_exp_write_t out[PULL_EXP_PLAN_MAX])
{
    size_t n = 0;
    const tca6416_regs_t f[2] = { f18, f54 }, t[2] = { t18, t54 };
    const uint8_t addr[2] = { PULL_EXP_ADDR_U18, PULL_EXP_ADDR_U54 };
    /* A channel changes when any of its bits differs; then both its resistors go off first. */
    uint16_t off[2] = { f18.cfg, f54.cfg };
    for (unsigned la = 1; la <= PULL_EXP_CHANNELS; la++) {
        unsigned c = pull_exp_map[la].chip;
        uint16_t m = chan_mask(la);
        if (((f[c].cfg ^ t[c].cfg) | (f[c].out ^ t[c].out)) & m) off[c] |= m;
    }
    /* (1) off */
    for (unsigned c = 0; c < 2; c++)
        if (off[c] != f[c].cfg) add(out, &n, addr[c], TCA6416_REG_CONFIG0, off[c]);
    /* (2) levels, while those channels are off */
    for (unsigned c = 0; c < 2; c++)
        if (t[c].out != f[c].out) add(out, &n, addr[c], TCA6416_REG_OUTPUT0, t[c].out);
    /* (3) on */
    for (unsigned c = 0; c < 2; c++)
        if (t[c].cfg != off[c]) add(out, &n, addr[c], TCA6416_REG_CONFIG0, t[c].cfg);
    return n;
}

pull_exp_dir_t pull_exp_channel(tca6416_regs_t r18, tca6416_regs_t r54, unsigned la, pull_str_t *str)
{
    if (str) *str = PULL_STR_NONE;
    if (la < 1u || la > PULL_EXP_CHANNELS) return PULL_EXP_OFF;
    const pull_exp_pin_t *m = &pull_exp_map[la];
    tca6416_regs_t r = m->chip == PULL_EXP_U18 ? r18 : r54;
    uint16_t b10 = bit_of(m->bit10k), b2 = bit_of(m->bit2k2);
    bool on10 = !(r.cfg & b10), on2 = !(r.cfg & b2);
    if (!on2 && !on10) return PULL_EXP_OFF;
    bool up10 = (r.out & b10) != 0, up2 = (r.out & b2) != 0;
    if (on2 && on10 && up2 != up10) return PULL_EXP_OFF;   /* a divider: never written by plan */
    if (str) *str = (pull_str_t)((on2 ? PULL_STR_2K2 : 0) | (on10 ? PULL_STR_10K : 0));
    return (on2 ? up2 : up10) ? PULL_EXP_UP : PULL_EXP_DOWN;
}

const char *pull_exp_hold_name(bool live, bool rail_up, bool suspend_up, bool any_up_asked)
{
    if (!live) return rail_up ? "fault" : "pin21_down";
    if (suspend_up && any_up_asked) return "dut_off";
    return NULL;
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
