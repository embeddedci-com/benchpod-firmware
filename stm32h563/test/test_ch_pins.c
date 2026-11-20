/*
 * test_ch_pins.c: host tests for the LA pull commands (src/command_handler_pins.c: `la` with a
 * pull, the pull-mask query, `la_voltage`) and the pull driver under them, on every board
 * revision: the TCA9554 switch per channel before digital v3r4, the two pull expanders on v3r4
 * (pull_exp.c), through the pull layer (la_pull.c).  The replies are compared whole: clients match their fields and texts.
 *
 * Real code: the handlers, la_pull.c and both backends (pull_tca9554.c, pull_exp.c), the pin table.  Fakes: the
 * TCA9554 pin writes, the I2C bus under the v3r4 expanders, pin 21, target power, the LA bank
 * voltage, the FPGA and the reply path.
 */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "command_handler_internal.h"
#include "board_caps.h"
#include "board_rev.h"
#include "i2c_bus.h"   /* I2C_ADDR_TCA9554_LA */
#include "la_pull.h"
#include "signal_engine.h"
#include "la_io.h"
#include "pull_exp.h"
#include "vref_out.h"
#include "bp_errors.h"
#include "stm32h5xx_hal.h"   /* shim_pull */

static int checks, fails;
#define CHECK(cond, ...) do { checks++; if (!(cond)) { fails++; printf("FAIL %s:%d: ", __func__, __LINE__); \
        printf(__VA_ARGS__); printf("\n"); } } while (0)

/* ---- fakes: the reply path ------------------------------------------------------------- */
static char g_reply[512];
static int  g_code;          /* the BP error code of the last coded refusal, 0 = none */
static char g_arg0[32];
void send_ok_str(int c, const char *p) { (void)c; g_code = 0; snprintf(g_reply, sizeof(g_reply), "ok:%s", p); }
void bp_error_send(int c, bp_errcode_t code, const bp_err_arg_t *args, size_t n);
void send_error(int c, const char *m) {
    bp_errcode_t code; size_t n;   /* like command_handler.c: a noted message goes with its code */
    const bp_err_arg_t *a = bp_err_noted(m, &code, &n);
    if (a) { bp_error_send(c, code, a, n); return; }
    g_code = 0;
    snprintf(g_reply, sizeof(g_reply), "err:%s", m);
}
void bp_error_send(int c, bp_errcode_t code, const bp_err_arg_t *args, size_t n) {
    (void)c;
    g_code = (int)code;
    g_arg0[0] = 0;
    if (n) {
        if (args[0].is_str) snprintf(g_arg0, sizeof(g_arg0), "%s", args[0].s);
        else                snprintf(g_arg0, sizeof(g_arg0), "%ld", (long)args[0].i);
    }
    memcpy(g_reply, "err:", 4);
    bp_err_render(g_reply + 4, sizeof(g_reply) - 4, code, args, n);
}
int  at_send_data(int c, const uint8_t *b, size_t n) {
    (void)c; size_t k = n < sizeof(g_reply) - 1 ? n : sizeof(g_reply) - 1;
    memcpy(g_reply, b, k); g_reply[k] = 0; return 0;
}
int  at_close_connection(int c) { (void)c; return 0; }
bool require_la_voltage(int c) { (void)c; return true; }
bool la_voltage_from_profile(void) { return true; }
uint64_t time_us_64(void) { return 0; }

/* ---- fakes: the FPGA -------------------------------------------------------------------- */
uint8_t signal_engine_fpga_version(void) { return 59; }
uint8_t signal_engine_fpga_features_cached(void) { return 0; }
int  fpga_la_set(unsigned ch, int mode) { (void)ch; (void)mode; return 0; }
int  fpga_la_step(unsigned ch, uint32_t steps, uint32_t delay_us) { (void)ch; (void)steps; (void)delay_us; return 0; }
bool fpga_la_step_busy(void) { return false; }
int  fpga_gpio_get(uint16_t *levels) { *levels = 0; return 0; }

/* ---- fakes: the board ------------------------------------------------------------------- */
static int  g_rev = BOARD_REV_V3;
static int  g_vccio = LA_VCCIO_3V3, g_io = LA_VCCIO_3V3, g_set_rc;
int  la_vccio_get_mv(void) { return g_vccio; }
int  la_io_get_mv(void) { return g_io; }
int  la_vccio_set_mv(int mv) { if (g_set_rc == 0) g_vccio = g_io = mv; return g_set_rc; }
int  la_vccio_status_pin(void) { return 1; }
int  la_vccio_readback_mv(void) { return 3290; }

/* TCA9554 at 0x20 (boards before v3r4): the output and direction bits the driver wrote. */
static uint8_t g_tca_out, g_tca_dir;
static bool    g_tca_fail;
int tca9554_set_dir(uint8_t addr, uint8_t pin, bool out) {
    if (addr != I2C_ADDR_TCA9554_LA || g_tca_fail) return -1;
    if (out) g_tca_dir |= (uint8_t)(1u << pin); else g_tca_dir &= (uint8_t)~(1u << pin);
    return 0;
}
int tca9554_write_pin(uint8_t addr, uint8_t pin, bool v) {
    if (addr != I2C_ADDR_TCA9554_LA || g_tca_fail) return -1;
    if (v) g_tca_out |= (uint8_t)(1u << pin); else g_tca_out &= (uint8_t)~(1u << pin);
    return 0;
}

/* Digital v3r4: the expanders' reset line, their I2C writes, pin 21, target power. */
GPIO_TypeDef shim_gpiog;
static bool g_in_reset = true, g_i2c_fail, g_rail_up = true, g_power_on = true;
static int  g_vref_mv = 3300;
static void (*g_rail_hook)(bool up);
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint32_t pin, GPIO_PinState st) { (void)port; (void)pin; g_in_reset = st == GPIO_PIN_RESET; }
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init) { (void)port; (void)init; }
void HAL_Delay(uint32_t ms) { (void)ms; }
int  i2c_bus_write(uint8_t addr, const uint8_t *buf, size_t len) {
    (void)addr; (void)buf; (void)len;
    return (g_in_reset || g_i2c_fail) ? -1 : 0;
}
bool vref_out_present(void) { return board_caps()->vref_out; }
bool vref_out_rail_up(void) { return g_rail_up; }
int  vref_out_last_mv(void) { return g_vref_mv; }
int  vref_out_measure_mv(void) { return g_vref_mv; }
void vref_out_set_rail_hook(void (*hook)(bool up)) { g_rail_hook = hook; }
void vref_out_emit(bp_emit_t *e) { bp_emit(e, "\"vref_out\":{\"mv\":%d}", g_vref_mv); }
int  target_power_get_cached(int efuse, bool *en, bool *flt) {
    if (en) *en = efuse == 1 && g_power_on;
    if (flt) *flt = false;
    return 0;
}
static void rail(bool up) { g_rail_up = up; g_vref_mv = up ? 3300 : 120; if (g_rail_hook) g_rail_hook(up); }
static void power(bool on) { g_power_on = on; la_pull_poll(); }

/* ---- helpers ---------------------------------------------------------------------------- */
static void la(const char *json) { g_reply[0] = 0; g_code = 0; handle_la(0, json); }
static void la_voltage(const char *json) { g_reply[0] = 0; g_code = 0; handle_la_voltage(0, json); }
#define REPLY(want) CHECK(strcmp(g_reply, want) == 0, "reply\n   got  %s\n   want %s", g_reply, want)

static void board(int rev) {
    g_rev = rev;
    board_caps_init(rev);
    la_pull_init();
    la_pins_reset();
    g_vccio = g_io = LA_VCCIO_3V3;
}

/* ---- boards before digital v3r4: one TCA9554 switch per channel ---------------------- */
static void test_tca9554_board(int rev, const char *name) {
    printf("  %s\n", name);
    board(rev);
    la_pull_all_off();
    g_tca_out = g_tca_dir = 0;
    CHECK(la_pull_available(), "%s: pulls available at 3.3 V", name);

    la("{\"cmd\":\"la\",\"la\":1,\"pull\":\"up\"}");
    REPLY("ok:{\"la\":1,\"pullup\":1,\"ohms\":\"4.7k\",\"pull\":\"up\",\"state\":\"up\",\"dirs\":[\"up\"],\"pullups_available\":1}");
    CHECK(g_tca_out == (1u << 2) && (g_tca_dir & (1u << 2)), "%s: LA1 is CTRL1 = TCA P2 (out %02x)", name, g_tca_out);
    CHECK((la_pull_asked(1) != LA_PULL_NONE) && la_pull_live(1), "%s: LA1 engaged", name);
    la("{\"cmd\":\"la\"}");
    REPLY("ok:{\"la_pullup_mask\":1,\"pullups_available\":1}");

    /* the older alias engages the channel's one direction: LA7 pulls down */
    la("{\"cmd\":\"la\",\"la\":7,\"pullup\":\"on\"}");
    REPLY("ok:{\"la\":7,\"pullup\":1,\"ohms\":\"10k\",\"pull\":\"down\",\"state\":\"down\",\"dirs\":[\"down\"],\"pullups_available\":1}");
    CHECK(g_tca_out == ((1u << 2) | (1u << 5)), "%s: LA7 is TCA P5 (out %02x)", name, g_tca_out);
    la("{\"cmd\":\"la\"}");
    REPLY("ok:{\"la_pullup_mask\":65,\"pullups_available\":1}");
    la("{\"cmd\":\"la\",\"la\":7,\"pullup\":\"off\"}");
    REPLY("ok:{\"la\":7,\"pullup\":0,\"ohms\":\"10k\",\"pull\":\"down\",\"state\":\"off\",\"dirs\":[\"down\"],\"pullups_available\":1}");

    /* refusals */
    la("{\"cmd\":\"la\",\"la\":1,\"pull\":\"down\"}");
    CHECK(g_code == BP_E_LA_PULL_DIRECTION && strcmp(g_arg0, "1") == 0, "%s: BP0024 (%d %s)", name, g_code, g_arg0);
    la("{\"cmd\":\"la\",\"la\":9,\"pull\":\"up\"}");
    CHECK(g_code == BP_E_LA_NO_PULL && strcmp(g_arg0, "9") == 0, "%s: BP0023 (%d %s)", name, g_code, g_arg0);
    la("{\"cmd\":\"la\",\"la\":9,\"pullup\":\"on\"}");
    CHECK(g_code == BP_E_LA_NO_PULL, "%s: alias on a channel without one: BP0023 (%d)", name, g_code);
    la("{\"cmd\":\"la\",\"la\":9,\"pull\":\"off\"}");   /* off is fine without a resistor */
    REPLY("ok:{\"la\":9,\"pullup\":0,\"ohms\":\"\",\"pull\":\"none\",\"state\":\"off\",\"dirs\":[],\"pullups_available\":1}");
    la("{\"cmd\":\"la\",\"la\":1,\"pull\":\"sideways\"}");
    REPLY("err:pull must be up, down or off");
    la("{\"cmd\":\"la\",\"la\":15}");
    REPLY("err:LA15 is not a pin (use 1..14)");
    /* "strength" is a v3r4 argument: older boards ignore it, as older firmware does */
    la("{\"cmd\":\"la\",\"la\":3,\"pull\":\"up\",\"strength\":\"5k\"}");
    REPLY("ok:{\"la\":3,\"pullup\":1,\"ohms\":\"2.2k\",\"pull\":\"up\",\"state\":\"up\",\"dirs\":[\"up\"],\"pullups_available\":1}");

    /* a pull-down under a line that idles high */
    uint8_t p = 8;
    la_pins_claim(LA_FN_UART_RX, LA_GPIO_NONE, 0, &p, 1);
    la("{\"cmd\":\"la\",\"la\":8,\"pull\":\"down\"}");
    CHECK(g_code == BP_E_LA_PULL_DOWN_CONFLICT && strcmp(g_arg0, "8") == 0, "%s: %s", name, g_reply);
    CHECK(!(la_pull_asked(8) != LA_PULL_NONE), "%s: refused before the switch", name);
    la_pins_release(8);

    /* an expander that does not answer */
    g_tca_fail = true;
    la("{\"cmd\":\"la\",\"la\":4,\"pull\":\"up\"}");
    REPLY("err:pull expander write failed");
    g_tca_fail = false;
    CHECK(!(la_pull_asked(4) != LA_PULL_NONE), "%s: a failed write is not engaged", name);

    /* 1.8 V: v3 boards pull to the bank rail; v2's pull-ups go to +3V3 and are refused */
    g_vccio = g_io = LA_VCCIO_1V8;
    if (rev == BOARD_REV_V2) {
        CHECK(!la_pull_available(), "v2 at 1.8 V: no pulls");
        la("{\"cmd\":\"la\",\"la\":2,\"pull\":\"up\"}");
        REPLY("err:this board's pull-ups go to 3.3 V; set the LA bank to 3.3 V first");
        CHECK(!(la_pull_asked(2) != LA_PULL_NONE), "v2 at 1.8 V: LA2 stays off");
        la("{\"cmd\":\"la\",\"la\":1,\"pull\":\"off\"}");   /* turning one off is always allowed */
        REPLY("ok:{\"la\":1,\"pullup\":0,\"ohms\":\"4.7k\",\"pull\":\"up\",\"state\":\"off\",\"dirs\":[\"up\"],\"pullups_available\":0}");
    } else {
        CHECK(la_pull_available(), "%s at 1.8 V: pulls available", name);
        la("{\"cmd\":\"la\",\"la\":2,\"pull\":\"up\"}");
        REPLY("ok:{\"la\":2,\"pullup\":1,\"ohms\":\"4.7k\",\"pull\":\"up\",\"state\":\"up\",\"dirs\":[\"up\"],\"pullups_available\":1}");
    }
    g_vccio = g_io = LA_VCCIO_UNSET;
    CHECK(!la_pull_available(), "%s: no pulls before an I/O voltage is set", name);
    g_vccio = g_io = LA_VCCIO_3V3;

    la_pull_all_off();
    CHECK(g_tca_out == 0, "%s: all off (out %02x)", name, g_tca_out);
    for (uint8_t i = 1; i <= 14; i++) CHECK(!(la_pull_asked(i) != LA_PULL_NONE) && !la_pull_live(i), "%s: LA%u off", name, i);
    la("{\"cmd\":\"la\"}");
    REPLY("ok:{\"la_pullup_mask\":0,\"pullups_available\":1}");
}

/* ---- digital v3r4: the two pull expanders -------------------------------------------- */
static void test_v3r4(void) {
    printf("  v3r4\n");
    g_rail_up = true; g_vref_mv = 3300; g_power_on = true;
    board(BOARD_REV_V3R4);
    la_pull_init();   /* the expanders out of reset: pin 21 is up */
    la_pull_poll();
    CHECK(la_pull_available(), "v3r4: pulls available with pin 21 up");

    la("{\"cmd\":\"la\",\"la\":12,\"pull\":\"down\",\"strength\":\"2.2k\"}");
    REPLY("ok:{\"la\":12,\"pullup\":1,\"ohms\":\"2.2k\",\"pull\":\"down\",\"state\":\"down\",\"dirs\":[\"up\",\"down\"],"
          "\"strength\":\"2.2k\",\"strengths\":[\"10k\",\"2.2k\",\"both\"],\"pullups_available\":1}");
    /* no strength: from the pin's function (a free pin at 3.3 V: 10k) */
    la("{\"cmd\":\"la\",\"la\":3,\"pull\":\"up\"}");
    REPLY("ok:{\"la\":3,\"pullup\":1,\"ohms\":\"10k\",\"pull\":\"up\",\"state\":\"up\",\"dirs\":[\"up\",\"down\"],"
          "\"strength\":\"10k\",\"strengths\":[\"10k\",\"2.2k\",\"both\"],\"pullups_available\":1}");
    la("{\"cmd\":\"la\",\"la\":3,\"pull\":\"up\",\"strength\":\"both\"}");
    REPLY("ok:{\"la\":3,\"pullup\":1,\"ohms\":\"1.8k\",\"pull\":\"up\",\"state\":\"up\",\"dirs\":[\"up\",\"down\"],"
          "\"strength\":\"both\",\"strengths\":[\"10k\",\"2.2k\",\"both\"],\"pullups_available\":1}");
    la("{\"cmd\":\"la\",\"la\":3,\"pull\":\"up\",\"strength\":\"5k\"}");
    REPLY("err:strength must be 10k, 2.2k or both");
    CHECK(la_pull_asked_str(3) == PULL_STR_BOTH, "v3r4: a refused strength changes nothing");
    /* the function picks 2.2k for an open-drain line */
    uint8_t p = 5;
    la_pins_claim(LA_FN_GPIO, LA_GPIO_OPEN_DRAIN, 1, &p, 1);
    CHECK(la_pull_set(5, LA_PULL_UP, PULL_STR_NONE) == 0 && la_pull_asked_str(5) == PULL_STR_2K2, "v3r4: open drain gets 2.2k");
    CHECK(la_pull_set(5, LA_PULL_NONE, PULL_STR_NONE) == 0 && !(la_pull_asked(5) != LA_PULL_NONE), "v3r4: LA5 off");
    la_pins_release(5);
    /* 5 V mode: a pod output gets 2.2k (it lifts a 3.3 V high to 5 V) */
    g_io = LA_VCCIO_5V0;
    la_pins_claim(LA_FN_GPIO, LA_GPIO_OUTPUT, 1, &p, 1);
    CHECK(la_pull_set(5, LA_PULL_UP, PULL_STR_NONE) == 0 && la_pull_asked_str(5) == PULL_STR_2K2, "v3r4 5 V: output gets 2.2k");
    CHECK(la_pull_set(5, LA_PULL_NONE, PULL_STR_NONE) == 0, "v3r4: LA5 off");
    la_pins_release(5);
    g_io = LA_VCCIO_3V3;

    la("{\"cmd\":\"la\"}");
    REPLY("ok:{\"la_pullup_mask\":2052,\"pullups_available\":1}");

    /* the pod switches the DUT off: pull-ups are held off, pull-downs stay */
    power(false);
    la("{\"cmd\":\"la\",\"la\":3}");
    REPLY("ok:{\"la\":3,\"pullup\":0,\"ohms\":\"1.8k\",\"pull\":\"up\",\"state\":\"off\",\"dirs\":[\"up\",\"down\"],"
          "\"strength\":\"both\",\"strengths\":[\"10k\",\"2.2k\",\"both\"],\"held\":\"dut_off\",\"pullups_available\":1}");
    la("{\"cmd\":\"la\"}");
    REPLY("ok:{\"la_pullup_mask\":2048,\"pullups_available\":1,\"held\":\"dut_off\",\"asked_mask\":2052}");
    CHECK((la_pull_asked(3) != LA_PULL_NONE) && !la_pull_live(3) && la_pull_live(12), "v3r4: LA3 asked, not live");
    power(true);
    la("{\"cmd\":\"la\"}");
    REPLY("ok:{\"la_pullup_mask\":2052,\"pullups_available\":1}");

    /* pin 21 down: everything held off, a new pull refused with BP0029 */
    rail(false);
    CHECK(!la_pull_available(), "v3r4: no pulls with pin 21 down");
    la("{\"cmd\":\"la\",\"la\":12}");
    REPLY("ok:{\"la\":12,\"pullup\":0,\"ohms\":\"2.2k\",\"pull\":\"down\",\"state\":\"off\",\"dirs\":[\"up\",\"down\"],"
          "\"strength\":\"2.2k\",\"strengths\":[\"10k\",\"2.2k\",\"both\"],\"held\":\"pin21_down\",\"pullups_available\":0}");
    la("{\"cmd\":\"la\"}");
    REPLY("ok:{\"la_pullup_mask\":0,\"pullups_available\":0,\"held\":\"pin21_down\",\"asked_mask\":2052}");
    la("{\"cmd\":\"la\",\"la\":6,\"pull\":\"up\"}");
    CHECK(g_code == BP_E_PULL_RAIL_DOWN && strcmp(g_arg0, "120") == 0, "v3r4: BP0029 (%d %s)", g_code, g_arg0);
    CHECK(la_pull_set(6, LA_PULL_UP, PULL_STR_NONE) == LA_PULL_ERR_RAIL, "v3r4: rail error");
    CHECK(la_pull_set(6, LA_PULL_NONE, PULL_STR_NONE) == 0, "v3r4: off is allowed with pin 21 down");
    rail(true);
    la("{\"cmd\":\"la\"}");
    REPLY("ok:{\"la_pullup_mask\":2052,\"pullups_available\":1}");

    /* no I/O voltage yet: BP0001 */
    g_vccio = g_io = LA_VCCIO_UNSET;
    CHECK(!la_pull_available(), "v3r4: no pulls without an I/O voltage");
    la("{\"cmd\":\"la\",\"la\":6,\"pull\":\"up\"}");
    CHECK(g_code == BP_E_LA_VOLTAGE_NOT_SET, "v3r4: BP0001 (%d)", g_code);
    g_vccio = g_io = LA_VCCIO_3V3;

    /* a write that fails: held ("fault") until the next pull turned on retries */
    g_i2c_fail = true;
    la("{\"cmd\":\"la\",\"la\":6,\"pull\":\"up\"}");
    REPLY("err:pull expander write failed");
    la("{\"cmd\":\"la\",\"la\":12}");
    CHECK(strstr(g_reply, "\"held\":\"fault\"") != NULL, "v3r4: held fault: %s", g_reply);
    g_i2c_fail = false;
    la("{\"cmd\":\"la\",\"la\":6,\"pull\":\"up\"}");
    CHECK(strncmp(g_reply, "ok:{\"la\":6,\"pullup\":1,", 22) == 0, "v3r4: the retry: %s", g_reply);
    la("{\"cmd\":\"la\"}");
    REPLY("ok:{\"la_pullup_mask\":2084,\"pullups_available\":1}");

    la_pull_all_off();
    la("{\"cmd\":\"la\"}");
    REPLY("ok:{\"la_pullup_mask\":0,\"pullups_available\":1}");
    la("{\"cmd\":\"la\",\"la\":12}");
    REPLY("ok:{\"la\":12,\"pullup\":0,\"ohms\":\"10k\",\"pull\":\"up\",\"state\":\"off\",\"dirs\":[\"up\",\"down\"],"
          "\"strength\":\"none\",\"strengths\":[\"10k\",\"2.2k\",\"both\"],\"pullups_available\":1}");
}

/* ---- the step train behind the same verb ---------------------------------------------- */
static void test_step(void) {
    printf("  step\n");
    board(BOARD_REV_V3);
    la("{\"cmd\":\"la\",\"la\":3,\"steps\":10,\"delay_us\":100}");
    REPLY("ok:{\"la\":3,\"steps\":10,\"delay_us\":100,\"status\":\"started\"}");
    CHECK(la_pins_fn(3) == LA_FN_STEP, "the step pin is claimed");
    la_pins_on_gateware_reconfigured();   /* drops the train's claims */
    la("{\"cmd\":\"la\",\"la\":3,\"steps\":10}");
    REPLY("err:missing delay_us");
    la("{\"cmd\":\"la\",\"steps\":10,\"delay_us\":100}");
    REPLY("err:missing la");
    la("{\"cmd\":\"la\",\"la\":3,\"steps\":\"1e5\",\"delay_us\":100}");
    REPLY("err:invalid steps");
}

/* ---- la_voltage ------------------------------------------------------------------------ */
static void test_la_voltage(void) {
    printf("  la_voltage\n");
    board(BOARD_REV_V3);
    la_voltage("{\"cmd\":\"la_voltage\",\"mv\":1800}");
    REPLY("ok:{\"mv\":1800,\"st\":1,\"readback_mv\":3290}");
    g_set_rc = -2;
    la_voltage("{\"cmd\":\"la_voltage\",\"mv\":1800}");
    REPLY("err:1.8 V needs a v3 pod; this board is v2 (its TPS2116 has no 1.8 V setting)");
    g_set_rc = -1;
    la_voltage("{\"cmd\":\"la_voltage\",\"mv\":2500}");
    REPLY("err:la voltage must be 1800 or 3300 (mv)");

    board(BOARD_REV_V3R4);
    la_voltage("{\"cmd\":\"la_voltage\",\"mv\":2500}");
    REPLY("err:la voltage must be 1800, 3300 or 5000 (mv)");
    /* 5 V that does not come up on pin 21: BP0028 */
    g_set_rc = -3; g_vref_mv = 4100;
    la_voltage("{\"cmd\":\"la_voltage\",\"mv\":5000}");
    CHECK(g_code == BP_E_VREF_5V_MISSING && strcmp(g_arg0, "4100") == 0, "BP0028 (%d %s)", g_code, g_arg0);
    g_set_rc = 0; g_vref_mv = 5000;
    la_voltage("{\"cmd\":\"la_voltage\",\"mv\":5000}");
    REPLY("ok:{\"mv\":5000,\"st\":1,\"readback_mv\":3290,\"vref_out\":{\"mv\":5000}}");
    /* a pin in use refuses a change, but not the voltage it already has */
    uint8_t p = 2;
    la_pins_claim(LA_FN_GPIO, LA_GPIO_OUTPUT, 0, &p, 1);
    la_voltage("{\"cmd\":\"la_voltage\",\"mv\":3300}");
    CHECK(strncmp(g_reply, "err:", 4) == 0 && strstr(g_reply, "LA2") != NULL, "busy pin: %s", g_reply);
    la_voltage("{\"cmd\":\"la_voltage\",\"mv\":5000}");
    CHECK(strncmp(g_reply, "ok:{\"mv\":5000", 13) == 0, "same voltage: %s", g_reply);
    la_pins_release(2);
    g_vref_mv = 3300;
    g_vccio = g_io = LA_VCCIO_3V3;
}

int main(void) {
    printf("test_ch_pins\n");
    test_tca9554_board(BOARD_REV_V2, "v2");
    test_tca9554_board(BOARD_REV_V3, "v3");
    test_tca9554_board(BOARD_REV_V3R2, "v3r2");
    test_step();
    test_la_voltage();
    test_v3r4();   /* last: pull_exp_init latches the expanders for the rest of the run */
    printf("test_ch_pins: %d checks, %d failed\n", checks, fails);
    return fails ? 1 : 0;
}
