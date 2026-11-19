/*
 * test_pull_exp_state.c: the digital v3r4 pull expanders' state as the `la` replies report it
 * (pull_exp.c): what is on the wire and why a pull is held, through pin 21 dropping and coming
 * back, the pod switching the DUT off and on, and an expander write failing.
 *
 * pull_exp.c is built in with host stand-ins (shim_pull/) for the HAL, the I2C bus, pin 21's
 * sense and target power. The stand-in chips record each register write, so a test can make one
 * fail.
 */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "board_caps.h"
#include "board_rev.h"
#include "stm32h5xx_hal.h"   /* shim_pull */

/* ---- stand-ins ---- */

GPIO_TypeDef shim_gpiog;
static bool     s_in_reset = true;      /* PG7 low */
static unsigned s_writes;               /* I2C writes seen */
static int      s_fail_after = -1;      /* >= 0: that many more writes succeed, then they fail */
static bool     s_rail_up;              /* pin 21 */
static void   (*s_hook)(bool up);
static bool     s_power_on;             /* target rail 1 */

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint32_t pin, GPIO_PinState st) {
    (void)port; (void)pin;
    s_in_reset = (st == GPIO_PIN_RESET);
}
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init) { (void)port; (void)init; }
void HAL_Delay(uint32_t ms) { (void)ms; }

int i2c_bus_write(uint8_t addr, const uint8_t *buf, size_t len) {
    (void)addr; (void)buf; (void)len;
    if (s_in_reset) return -1;   /* a chip in reset does not answer */
    if (s_fail_after == 0) return -1;
    if (s_fail_after > 0) s_fail_after--;
    s_writes++;
    return 0;
}

bool vref_out_rail_up(void) { return s_rail_up; }
int  vref_out_last_mv(void) { return s_rail_up ? 3300 : 120; }
void vref_out_set_rail_hook(void (*hook)(bool up)) { s_hook = hook; }

int target_power_get_cached(int efuse, bool *enabled, bool *fault) {
    if (enabled) *enabled = (efuse == 1) && s_power_on;
    if (fault) *fault = false;
    return 0;
}

#include "../src/pull_exp.c"

/* ---- checks ---- */

static int checks, fails;
#define CHECK(cond, ...) do { checks++; if (!(cond)) { fails++; printf("FAIL %s:%d  ", __FILE__, __LINE__); \
    printf(__VA_ARGS__); printf("\n"); } } while (0)

static bool held_is(const char *want) {
    const char *h = pull_exp_held();
    return want == NULL ? h == NULL : (h != NULL && strcmp(h, want) == 0);
}

static void rail(bool up) { s_rail_up = up; if (s_hook) s_hook(up); }

static void test_states(void) {
    board_caps_init(BOARD_REV_V3R4);
    s_rail_up = false;
    pull_exp_init();
    CHECK(pull_exp_present() && !pull_exp_live(), "init with pin 21 down: held in reset");
    CHECK(held_is(NULL), "nothing asked: nothing held");
    CHECK(pull_exp_set(2, PULL_EXP_UP, PULL_STR_10K) == PULL_EXP_ERR_RAIL, "a pull while pin 21 is down");
    CHECK(!pull_exp_recover(), "no retry while pin 21 is down");
    CHECK(s_in_reset, "still in reset");

    /* Pin 21 comes up: out of reset, pulls as asked. */
    rail(true);
    CHECK(pull_exp_live() && !s_in_reset, "pin 21 up: live");
    CHECK(pull_exp_set(2, PULL_EXP_UP, PULL_STR_10K) == 0, "LA2 up");
    CHECK(pull_exp_set(3, PULL_EXP_DOWN, PULL_STR_2K2) == 0, "LA3 down");
    CHECK(pull_exp_live_dir(2, NULL) == PULL_EXP_UP && pull_exp_live_dir(3, NULL) == PULL_EXP_DOWN, "on the wire");
    CHECK(held_is(NULL), "nothing held");

    /* Pin 21 drops and comes back. */
    rail(false);
    CHECK(pull_exp_live_dir(2, NULL) == PULL_EXP_OFF && pull_exp_live_dir(3, NULL) == PULL_EXP_OFF, "pin 21 down: all off");
    CHECK(pull_exp_dir(2) == PULL_EXP_UP, "the asked-for state is kept");
    CHECK(held_is("pin21_down"), "held: pin21_down, got %s", pull_exp_held() ? pull_exp_held() : "NULL");
    rail(true);
    CHECK(pull_exp_live_dir(2, NULL) == PULL_EXP_UP && pull_exp_live_dir(3, NULL) == PULL_EXP_DOWN, "back on the wire");
    CHECK(held_is(NULL), "nothing held after pin 21 returns");

    /* The pod switches the DUT on, then off: pull-ups suspended, pull-downs stay. */
    s_power_on = true;  pull_exp_poll();
    CHECK(held_is(NULL), "DUT on");
    s_power_on = false; pull_exp_poll();
    CHECK(pull_exp_live_dir(2, NULL) == PULL_EXP_OFF, "DUT off: LA2 up suspended");
    CHECK(pull_exp_live_dir(3, NULL) == PULL_EXP_DOWN, "DUT off: LA3 down stays");
    CHECK(held_is("dut_off"), "held: dut_off, got %s", pull_exp_held() ? pull_exp_held() : "NULL");
    rail(false);
    CHECK(held_is("pin21_down"), "pin 21 down wins over dut_off");
    rail(true);
    CHECK(held_is("dut_off"), "still suspended after pin 21 returns");
    s_power_on = true;  pull_exp_poll();
    CHECK(pull_exp_live_dir(2, NULL) == PULL_EXP_UP, "DUT on: LA2 back");
    CHECK(held_is(NULL), "DUT on: nothing held");

    /* An expander write fails: held in reset ("fault"), the asked-for state kept. */
    s_fail_after = 0;
    CHECK(pull_exp_set(4, PULL_EXP_UP, PULL_STR_10K) == -1, "a failed write");
    CHECK(!pull_exp_live() && s_in_reset, "held in reset after the failure");
    CHECK(pull_exp_dir(4) == PULL_EXP_OFF && pull_exp_dir(2) == PULL_EXP_UP, "the failed request is undone, the rest kept");
    CHECK(pull_exp_live_dir(2, NULL) == PULL_EXP_OFF, "LA2 off on the wire");
    CHECK(held_is("fault"), "held: fault, got %s", pull_exp_held() ? pull_exp_held() : "NULL");
    /* The next pull turned on tries again (la_pull_engage calls pull_exp_recover). Still failing: */
    CHECK(!pull_exp_recover() && held_is("fault"), "a retry that fails stays held");
    /* The bus answers again: the retry rewrites every asked-for pull. */
    s_fail_after = -1;
    CHECK(pull_exp_recover() && pull_exp_live(), "the retry brings them back");
    CHECK(pull_exp_live_dir(2, NULL) == PULL_EXP_UP && pull_exp_live_dir(3, NULL) == PULL_EXP_DOWN, "every pull back");
    CHECK(held_is(NULL), "nothing held after the retry");
    CHECK(pull_exp_set(4, PULL_EXP_UP, PULL_STR_10K) == 0 && pull_exp_live_dir(4, NULL) == PULL_EXP_UP, "LA4 now");

    /* All off while held: nothing asked, nothing held. */
    rail(false);
    pull_exp_all_off();
    CHECK(held_is(NULL), "all off while pin 21 is down: nothing held");
    rail(true);
    CHECK(pull_exp_live_dir(2, NULL) == PULL_EXP_OFF, "all off stays off");
}

/* Older boards: no expanders, nothing changes. */
static void test_old_board(void) {
    board_caps_init(BOARD_REV_V3);
    /* pull_exp_init returns at once; s_present is already true from the v3r4 run, so reset it as a
       fresh boot would. */
    s_present = false;
    pull_exp_init();
    CHECK(!pull_exp_present() && !pull_exp_live() && !pull_exp_recover(), "no expanders on v3");
    CHECK(held_is(NULL) && pull_exp_live_dir(2, NULL) == PULL_EXP_OFF, "nothing held on v3");
}

int main(void) {
    test_states();
    test_old_board();
    printf("test_pull_exp_state: %d checks, %s\n", checks, fails ? "FAILED" : "all passed");
    return fails ? 1 : 0;
}
