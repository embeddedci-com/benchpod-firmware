/* pull_exp.c — see pull_exp.h.  Digital v3r4 only (board_caps()->pull_expanders). */
#include "pull_exp.h"
#include "board_caps.h"
#include "board_pins.h"
#include "i2c_bus.h"
#include "vref_out.h"
#include "target_power.h"
#include "stm32h5xx_hal.h"
#include "bp_log.h"
#include "la_pull.h"
#include "la_pins.h"         /* la_pull_strength_for */
#include "la_io.h"           /* la_vccio_get_mv, la_io_get_mv */

static bool           s_present;
static bool           s_live;                      /* out of reset and written */
static uint8_t        s_dir[PULL_EXP_CHANNELS + 1]; /* what the host asked for */
static uint8_t        s_str[PULL_EXP_CHANNELS + 1];
static tca6416_regs_t s_hw2k2, s_hw10k;            /* what the chips hold now */
static bool           s_suspend_up;                /* the pod switched the DUT off */
static bool           s_rail_was_on;               /* a target rail was on at the last poll */

static void rst(bool release)
{
    HAL_GPIO_WritePin(PULL_RST_PORT, PULL_RST_PIN, release ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static int write_pair(uint8_t addr, uint8_t reg, uint16_t val)
{
    uint8_t b[3] = { reg, (uint8_t)(val & 0xFFu), (uint8_t)(val >> 8) };
    return i2c_bus_write(addr, b, sizeof(b));
}

/* Both chips in reset (every channel off); what they hold is the reset state. */
static void hold(void)
{
    rst(false);
    s_live = false;
    s_hw2k2 = s_hw10k = pull_exp_reset_regs();
}

/* Bring the chips to the asked-for state, from what they hold now. */
static int apply(void)
{
    tca6416_regs_t t2, t10;
    pull_exp_regs(s_dir, s_str, s_suspend_up, &t2, &t10);
    pull_exp_write_t w[PULL_EXP_PLAN_MAX];
    size_t n = pull_exp_plan(s_hw2k2, s_hw10k, t2, t10, w);
    for (size_t i = 0; i < n; i++) {
        if (write_pair(w[i].addr, w[i].reg, w[i].val) != 0) {
            /* Unknown state: back to reset (all off) and rewrite on the next change. */
            LOG_E("[pull] expander 0x%02x write failed; holding both in reset\n", w[i].addr);
            hold();
            return -1;
        }
        tca6416_regs_t *r = w[i].addr == PULL_EXP_ADDR_2K2 ? &s_hw2k2 : &s_hw10k;
        if (w[i].reg == TCA6416_REG_OUTPUT0) r->out = w[i].val;
        else                                 r->cfg = w[i].val;
    }
    return 0;
}

/* Out of reset and rewrite both (pin 21 is up). */
static int go_live(void)
{
    s_hw2k2 = s_hw10k = pull_exp_reset_regs();   /* what reset left */
    rst(true);
    HAL_Delay(1);                                /* tRESET is 600 ns (datasheet) */
    s_live = true;
    int rc = apply();
    if (rc == 0) log_printf("[pull] expanders out of reset (pin 21 %d mV)\r\n", vref_out_last_mv());
    return rc;
}

/* vref_out's rail-up decision changed. */
static void on_rail(bool up)
{
    if (!s_present) return;
    if (up) (void)go_live();
    else {
        hold();
        log_printf("[pull] pin 21 down (%d mV): pull resistors off, expanders held in reset\r\n",
                   vref_out_last_mv());
    }
}

void pull_exp_init(void)
{
    if (!board_caps()->pull_expanders) return;
    __HAL_RCC_GPIOG_CLK_ENABLE();
    rst(false);
    GPIO_InitTypeDef gp = {0};
    gp.Pin   = PULL_RST_PIN;
    gp.Mode  = GPIO_MODE_OUTPUT_PP;
    gp.Pull  = GPIO_NOPULL;
    gp.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PULL_RST_PORT, &gp);
    s_present = true;
    hold();
    vref_out_set_rail_hook(on_rail);
    if (vref_out_rail_up()) (void)go_live();
    else log_printf("[pull] pin 21 not up (%d mV): expanders stay in reset\r\n", vref_out_last_mv());
}

bool pull_exp_present(void) { return s_present; }
bool pull_exp_live(void)    { return s_present && s_live; }

bool pull_exp_recover(void)
{
    if (!s_present) return false;
    if (s_live) return true;
    /* Pin 21 down: on_rail brings them back when it returns. */
    if (!vref_out_rail_up()) return false;
    /* A write failed (apply): out of reset and rewrite every asked-for pull, once per call. */
    log_printf("[pull] retrying the expanders after a failed write\r\n");
    return go_live() == 0;
}

int pull_exp_set(unsigned la, pull_exp_dir_t dir, pull_str_t str)
{
    if (!s_present || la < 1u || la > PULL_EXP_CHANNELS) return -1;
    if (dir != PULL_EXP_OFF && (str & PULL_STR_BOTH) == 0) return -1;
    if (dir != PULL_EXP_OFF && !s_live) return PULL_EXP_ERR_RAIL;
    uint8_t pd = s_dir[la], ps = s_str[la];
    bool was_suspended = s_suspend_up;
    if (dir != PULL_EXP_OFF) s_suspend_up = false;   /* the host says the DUT can take it */
    s_dir[la] = (uint8_t)dir;
    s_str[la] = dir == PULL_EXP_OFF ? (uint8_t)PULL_STR_NONE : (uint8_t)str;
    if (!s_live) return 0;   /* off while held: nothing to write */
    if (apply() != 0) { s_dir[la] = pd; s_str[la] = ps; s_suspend_up = was_suspended; return -1; }
    return 0;
}

void pull_exp_all_off(void)
{
    if (!s_present) return;
    for (unsigned la = 1; la <= PULL_EXP_CHANNELS; la++) { s_dir[la] = PULL_EXP_OFF; s_str[la] = PULL_STR_NONE; }
    if (s_live) (void)apply();
}

pull_exp_dir_t pull_exp_dir(unsigned la)
{
    return (s_present && la >= 1u && la <= PULL_EXP_CHANNELS) ? (pull_exp_dir_t)s_dir[la] : PULL_EXP_OFF;
}

pull_str_t pull_exp_str(unsigned la)
{
    return (s_present && la >= 1u && la <= PULL_EXP_CHANNELS) ? (pull_str_t)s_str[la] : PULL_STR_NONE;
}

void pull_exp_poll(void)
{
    if (!s_present) return;
    bool a = false, b = false;
    (void)target_power_get_cached(1, &a, NULL);
    (void)target_power_get_cached(2, &b, NULL);
    bool on = a || b;
    if (on == s_rail_was_on) return;
    s_rail_was_on = on;
    bool suspend = !on;   /* only on a change: a DUT the pod never powered keeps its pulls */
    if (suspend == s_suspend_up) return;
    s_suspend_up = suspend;
    if (s_live) (void)apply();
    log_printf("[pull] target power %s: pull-ups %s\r\n", on ? "on" : "off", suspend ? "suspended" : "back on");
}

pull_exp_dir_t pull_exp_live_dir(unsigned la, pull_str_t *str)
{
    if (str) *str = PULL_STR_NONE;
    if (!s_present || !s_live) return PULL_EXP_OFF;
    return pull_exp_channel(s_hw2k2, s_hw10k, la, str);
}

const char *pull_exp_held(void)
{
    if (!s_present) return NULL;
    bool any_up = false, any = false;
    for (unsigned la = 1; la <= PULL_EXP_CHANNELS; la++) {
        if (s_dir[la] == PULL_EXP_UP) any_up = true;
        if (s_dir[la] != PULL_EXP_OFF) any = true;
    }
    /* Held in reset with nothing asked for: nothing is missing from the wire. */
    if (!any) return NULL;
    return pull_exp_hold_name(s_live, vref_out_rail_up(), s_suspend_up, any_up);
}

/* ---- the la_pull backend (la_pull.h) ----
   The pulls go to J1 pin 21, which follows the bank (or 5 V), so on v3r4 they work at every I/O
   voltage, but only while pin 21 is up. */

static int be_set(unsigned la, la_pull_dir_t dir, pull_str_t str)
{
    if (dir != LA_PULL_NONE) {
        if (la_vccio_get_mv() == LA_VCCIO_UNSET) return LA_PULL_ERR_VOLTAGE;
        /* After a failed write (pin 21 up), this request tries the expanders again. */
        if (!pull_exp_recover()) return vref_out_rail_up() ? LA_PULL_ERR_IO : LA_PULL_ERR_RAIL;
        if (str == PULL_STR_NONE) str = la_pull_strength_for(la, la_io_get_mv());
    }
    pull_exp_dir_t d = dir == LA_PULL_UP ? PULL_EXP_UP : dir == LA_PULL_DOWN ? PULL_EXP_DOWN : PULL_EXP_OFF;
    int rc = pull_exp_set(la, d, str);
    if (rc == PULL_EXP_ERR_RAIL) return LA_PULL_ERR_RAIL;
    return rc != 0 ? LA_PULL_ERR_IO : 0;
}

static la_pull_dir_t be_dir(pull_exp_dir_t d)
{
    return d == PULL_EXP_UP ? LA_PULL_UP : d == PULL_EXP_DOWN ? LA_PULL_DOWN : LA_PULL_NONE;
}

static la_pull_dir_t be_asked(unsigned la, pull_str_t *str)
{
    *str = pull_exp_str(la);
    return be_dir(pull_exp_dir(la));
}

static bool be_live(unsigned la) { return pull_exp_live_dir(la, NULL) != PULL_EXP_OFF; }
static bool be_available(void)   { return la_vccio_get_mv() != LA_VCCIO_UNSET && pull_exp_live(); }

const la_pull_backend_t la_pull_backend_exp = {
    .rail_pin21 = true,
    .init       = pull_exp_init,
    .poll       = pull_exp_poll,
    .set        = be_set,
    .asked      = be_asked,
    .live       = be_live,
    .available  = be_available,
    .all_off    = pull_exp_all_off,
    .held       = pull_exp_held,
};
