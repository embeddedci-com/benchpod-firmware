/*
 * test_la_pins.c — host unit tests for the LA pin ownership table (src/la_pins.c).
 *
 * The messages checked here are a CONTRACT: the Python SDK parses the `pin conflict:` /
 * `pull conflict:` / `trigger timeout:` prefixes, and the rest is what a user reads to fix the
 * problem.  A test that fails on a message change is doing its job — update the SDK and
 * docs/API.md with it.
 */
#include "la_pins.h"
#include "bp_limits.h"   /* BP_CLOUD_REPLY_MAX: the la_pins reply must fit the cloud channel */

#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); failures++; } } while (0)
#define CHECK_STR(got, want) do { if (strcmp((got), (want)) != 0) { \
    printf("FAIL %s:%d\n  got:  %s\n  want: %s\n", __FILE__, __LINE__, (got), (want)); failures++; } } while (0)

static char err[LA_PINS_ERR_MAX];

#define PULL(la) ((uint16_t)(1u << ((la) - 1u)))

static bool claim(la_fn_t fn, la_gpio_mode_t mode, const uint8_t *las, size_t n, uint16_t free_fns,
                  uint16_t pulls) {
    err[0] = '\0';
    if (!la_pins_check_claim(fn, mode, las, n, free_fns, pulls, err, sizeof(err))) return false;
    la_pins_claim(fn, mode, 0, las, n);
    return true;
}

static bool gpio_json(const char *json, uint16_t pulls, uint16_t *changed, uint16_t *affected) {
    la_gpio_req_t req;
    err[0] = '\0';
    if (!la_gpio_req_parse(json, &req, err, sizeof(err))) return false;
    return la_pins_gpio_apply(&req, pulls, changed, affected, err, sizeof(err));
}

static void test_defaults_and_names(void) {
    la_pins_reset();
    for (unsigned la = 1; la <= 12; la++) {
        CHECK(la_pins_fn(la) == LA_FN_NONE);
        CHECK(la_pins_gpio(la) == LA_GPIO_NONE);
        CHECK(la_pins_level(la) == -1);
        CHECK(la_pins_drive(la) == 2);
    }
    CHECK(la_pins_owned_mask() == 0);
    CHECK_STR(la_fn_name(LA_FN_STEP_DIR), "step_dir");
    CHECK_STR(la_fn_name(LA_FN_SWD_DIO), "swd_dio");
    CHECK(la_pull_dir(1) == LA_PULL_UP && la_pull_dir(6) == LA_PULL_UP);
    CHECK(la_pull_dir(7) == LA_PULL_DOWN && la_pull_dir(8) == LA_PULL_DOWN);
    CHECK(la_pull_dir(9) == LA_PULL_NONE && la_pull_ohms(12) == NULL);
    CHECK_STR(la_pull_ohms(3), "2.2k");
    CHECK_STR(la_pull_ohms(7), "10k");
}

static void test_claims_and_conflicts(void) {
    la_pins_reset();
    uint8_t uart[2] = { 3, 4 };
    CHECK(claim(LA_FN_UART_RX, LA_GPIO_NONE, &uart[0], 1, 0, 0));
    CHECK(claim(LA_FN_UART_TX, LA_GPIO_NONE, &uart[1], 1, 0, 0));
    CHECK(la_pins_owned_mask() == (PULL(3) | PULL(4)));

    /* a different function on an owned pin */
    uint8_t sda[2] = { 4, 5 };
    CHECK(!claim(LA_FN_I2C_SDA, LA_GPIO_NONE, sda, 2, 0, 0));
    CHECK_STR(err, "pin conflict: LA4 is in use by uart_tx; stop the uart proxy first");
    CHECK(la_pins_fn(5) == LA_FN_NONE);   /* nothing committed */

    /* the same function again (second proxy on the proxy's own pin) is a conflict too */
    CHECK(!claim(LA_FN_UART_RX, LA_GPIO_NONE, &uart[0], 1, 0, 0));
    CHECK_STR(err, "pin conflict: LA3 is in use by uart_rx; stop the uart proxy first");

    /* first conflict wins */
    uint8_t swd[2] = { 11, 3 };
    la_pins_claim(LA_FN_GPIO, LA_GPIO_INPUT, 0, (uint8_t[]){ 11 }, 1);
    CHECK(!claim(LA_FN_SWD_CLK, LA_GPIO_NONE, swd, 2, 0, 0));
    CHECK_STR(err, "pin conflict: LA11 is in use by gpio; release it with {\"cmd\":\"gpio\",\"la\":11,\"mode\":\"off\"}");

    /* every release hint */
    la_pins_reset();
    la_pins_claim(LA_FN_SWD_DIO, LA_GPIO_NONE, 0, (uint8_t[]){ 12 }, 1);
    la_pins_claim(LA_FN_I2C_SCL, LA_GPIO_NONE, 0, (uint8_t[]){ 2 }, 1);
    la_pins_claim(LA_FN_STEP, LA_GPIO_NONE, 0, (uint8_t[]){ 6 }, 1);
    CHECK(!claim(LA_FN_GPIO, LA_GPIO_OUTPUT, (uint8_t[]){ 12 }, 1, LA_FN_BIT(LA_FN_GPIO), 0));
    CHECK_STR(err, "pin conflict: LA12 is in use by swd_dio; end the SWD session first");
    CHECK(!claim(LA_FN_GPIO, LA_GPIO_OUTPUT, (uint8_t[]){ 2 }, 1, LA_FN_BIT(LA_FN_GPIO), 0));
    CHECK_STR(err, "pin conflict: LA2 is in use by i2c_scl; stop the sensor emulation first ({\"cmd\":\"sensor_stop\"})");
    CHECK(!claim(LA_FN_GPIO, LA_GPIO_OUTPUT, (uint8_t[]){ 6 }, 1, LA_FN_BIT(LA_FN_GPIO), 0));
    CHECK_STR(err, "pin conflict: LA6 is in use by step; wait for the step train to finish");

    /* a replacing sensor_start treats the old sensor's pins as free */
    la_pins_reset();
    la_pins_claim(LA_FN_I2C_SDA, LA_GPIO_NONE, 0, (uint8_t[]){ 1 }, 1);
    la_pins_claim(LA_FN_I2C_SCL, LA_GPIO_NONE, 0, (uint8_t[]){ 2 }, 1);
    uint16_t i2c_free = LA_FN_BIT(LA_FN_I2C_SDA) | LA_FN_BIT(LA_FN_I2C_SCL);
    CHECK(la_pins_check_claim(LA_FN_I2C_SDA, LA_GPIO_NONE, (uint8_t[]){ 2 }, 1, i2c_free, 0, err, sizeof(err)));
    CHECK(la_pins_check_claim(LA_FN_I2C_SCL, LA_GPIO_NONE, (uint8_t[]){ 1 }, 1, i2c_free, 0, err, sizeof(err)));

    /* release by function */
    CHECK(la_pins_release_fn(LA_FN_I2C_SDA) == PULL(1));
    CHECK(la_pins_fn(1) == LA_FN_NONE && la_pins_fn(2) == LA_FN_I2C_SCL);
    CHECK(la_pins_release_fn(LA_FN_I2C_SCL) == PULL(2));
    CHECK(la_pins_owned_mask() == 0);
    CHECK(la_pins_release_fn(LA_FN_NONE) == 0);
}

static void test_pull_rules(void) {
    la_pins_reset();
    uint16_t down7 = PULL(7), up3 = PULL(3);

    /* pull-ups never conflict */
    CHECK(claim(LA_FN_UART_RX, LA_GPIO_NONE, (uint8_t[]){ 3 }, 1, 0, up3));
    la_pins_reset();

    /* claiming onto an engaged pull-down: the conflicting functions */
    CHECK(!claim(LA_FN_UART_RX, LA_GPIO_NONE, (uint8_t[]){ 7 }, 1, 0, down7));
    CHECK_STR(err, "pull conflict: LA7 has its 10k pull-down engaged, which uart_rx can't work with "
                   "(the line idles high); disable it with {\"cmd\":\"la\",\"la\":7,\"pullup\":\"off\"} "
                   "or use another channel");
    CHECK(!claim(LA_FN_I2C_SCL, LA_GPIO_NONE, (uint8_t[]){ 7 }, 1, 0, down7));
    CHECK(strstr(err, "which i2c_scl can't work with (an open-drain bus needs pull-ups)") != NULL);
    CHECK(!claim(LA_FN_SWD_DIO, LA_GPIO_NONE, (uint8_t[]){ 7 }, 1, 0, down7));
    CHECK(strstr(err, "which swd_dio can't work with (SWDIO is pulled up when released)") != NULL);
    CHECK(!claim(LA_FN_GPIO, LA_GPIO_OPEN_DRAIN, (uint8_t[]){ 7 }, 1, LA_FN_BIT(LA_FN_GPIO), down7));
    CHECK(strstr(err, "which gpio open_drain can't work with (a released line would read low)") != NULL);

    /* ...and the compatible ones */
    CHECK(la_pins_check_claim(LA_FN_SWD_CLK, LA_GPIO_NONE, (uint8_t[]){ 7 }, 1, 0, down7, err, sizeof(err)));
    CHECK(la_pins_check_claim(LA_FN_STEP, LA_GPIO_NONE, (uint8_t[]){ 7 }, 1, 0, down7, err, sizeof(err)));
    CHECK(la_pins_check_claim(LA_FN_STEP_DIR, LA_GPIO_NONE, (uint8_t[]){ 7 }, 1, 0, down7, err, sizeof(err)));
    CHECK(la_pins_check_claim(LA_FN_GPIO, LA_GPIO_INPUT, (uint8_t[]){ 7 }, 1, 0, down7, err, sizeof(err)));
    CHECK(la_pins_check_claim(LA_FN_GPIO, LA_GPIO_OUTPUT, (uint8_t[]){ 7 }, 1, 0, down7, err, sizeof(err)));
    /* the pull-down on LA8 does not affect a claim on LA7 */
    CHECK(la_pins_check_claim(LA_FN_UART_TX, LA_GPIO_NONE, (uint8_t[]){ 7 }, 1, 0, PULL(8), err, sizeof(err)));

    /* ownership is reported before pulls */
    la_pins_claim(LA_FN_STEP, LA_GPIO_NONE, 0, (uint8_t[]){ 3 }, 1);
    CHECK(!claim(LA_FN_UART_RX, LA_GPIO_NONE, (uint8_t[]){ 7, 3 }, 2, 0, down7));
    CHECK(strncmp(err, "pin conflict: LA3", 17) == 0);

    /* engaging a pull on an owned pin (the other direction) */
    la_pins_reset();
    la_pins_claim(LA_FN_UART_RX, LA_GPIO_NONE, 0, (uint8_t[]){ 8 }, 1);
    CHECK(!la_pins_check_pull_enable(8, err, sizeof(err)));
    CHECK_STR(err, "pull conflict: LA8 is used by uart_rx, which can't work with the 10k pull-down (the line idles high)");
    CHECK(la_pins_check_pull_enable(7, err, sizeof(err)));   /* LA7 is free */
    la_pins_claim(LA_FN_UART_RX, LA_GPIO_NONE, 0, (uint8_t[]){ 3 }, 1);
    CHECK(la_pins_check_pull_enable(3, err, sizeof(err)));   /* pull-up: fine */
    la_pins_claim(LA_FN_GPIO, LA_GPIO_OPEN_DRAIN, 1, (uint8_t[]){ 7 }, 1);
    CHECK(!la_pins_check_pull_enable(7, err, sizeof(err)));
    CHECK_STR(err, "pull conflict: LA7 is used by gpio open_drain, which can't work with the 10k pull-down (a released line would read low)");
    la_pins_claim(LA_FN_GPIO, LA_GPIO_OUTPUT, 1, (uint8_t[]){ 7 }, 1);
    CHECK(la_pins_check_pull_enable(7, err, sizeof(err)));
}

static void test_gpio_modes_and_levels(void) {
    la_pins_reset();
    uint16_t changed, affected;

    /* output defaults to level 0, open_drain to 1 (released) */
    CHECK(gpio_json("{\"cmd\":\"gpio\",\"la\":[1,2],\"mode\":\"output\"}", 0, &changed, &affected));
    CHECK(changed == (PULL(1) | PULL(2)) && affected == changed);
    CHECK(la_pins_gpio(1) == LA_GPIO_OUTPUT && la_pins_level(1) == 0 && la_pins_drive(1) == 0);
    CHECK(gpio_json("{\"cmd\":\"gpio\",\"la\":3,\"mode\":\"open_drain\"}", 0, &changed, &affected));
    CHECK(la_pins_level(3) == 1 && la_pins_drive(3) == 2);
    CHECK(gpio_json("{\"cmd\":\"gpio\",\"la\":3,\"level\":0}", 0, &changed, &affected));
    CHECK(la_pins_level(3) == 0 && la_pins_drive(3) == 0);
    CHECK(gpio_json("{\"cmd\":\"gpio\",\"la\":4,\"mode\":\"output\",\"level\":1}", 0, &changed, &affected));
    CHECK(la_pins_level(4) == 1 && la_pins_drive(4) == 1);
    CHECK(gpio_json("{\"cmd\":\"gpio\",\"la\":5,\"mode\":\"input\"}", 0, &changed, &affected));
    CHECK(la_pins_gpio(5) == LA_GPIO_INPUT && la_pins_level(5) == -1 && la_pins_drive(5) == 2);

    /* switching gpio modes on a gpio pin is allowed */
    CHECK(gpio_json("{\"cmd\":\"gpio\",\"la\":1,\"mode\":\"open_drain\"}", 0, &changed, &affected));
    CHECK(la_pins_gpio(1) == LA_GPIO_OPEN_DRAIN && la_pins_level(1) == 1);

    /* level on a non-output */
    CHECK(!gpio_json("{\"cmd\":\"gpio\",\"la\":5,\"level\":1}", 0, &changed, &affected));
    CHECK_STR(err, "LA5 is not a gpio output (function gpio, gpio input); configure it with {\"cmd\":\"gpio\",\"la\":5,\"mode\":\"output\"}");
    CHECK(!gpio_json("{\"cmd\":\"gpio\",\"la\":[2,9],\"level\":1}", 0, &changed, &affected));
    CHECK_STR(err, "LA9 is not a gpio output (function none); configure it with {\"cmd\":\"gpio\",\"la\":9,\"mode\":\"output\"}");
    CHECK(la_pins_level(2) == 0);   /* whole request validated first: LA2 untouched */

    /* whole request validated first for claims too */
    la_pins_claim(LA_FN_UART_TX, LA_GPIO_NONE, 0, (uint8_t[]){ 10 }, 1);
    CHECK(!gpio_json("{\"cmd\":\"gpio\",\"la\":[6,10],\"mode\":\"output\"}", 0, &changed, &affected));
    CHECK(strncmp(err, "pin conflict: LA10 is in use by uart_tx;", 40) == 0);
    CHECK(la_pins_fn(6) == LA_FN_NONE);

    /* off on a pin another function owns is refused; on a none pin it is a no-op */
    CHECK(!gpio_json("{\"cmd\":\"gpio\",\"la\":10,\"mode\":\"off\"}", 0, &changed, &affected));
    CHECK_STR(err, "pin conflict: LA10 is in use by uart_tx; stop the uart proxy first");
    CHECK(gpio_json("{\"cmd\":\"gpio\",\"la\":[4,11],\"mode\":\"off\"}", 0, &changed, &affected));
    CHECK(changed == PULL(4) && affected == (PULL(4) | PULL(11)));
    CHECK(la_pins_fn(4) == LA_FN_NONE && la_pins_drive(4) == 2);

    /* "all" only with off, releases gpio pins only */
    CHECK(!gpio_json("{\"cmd\":\"gpio\",\"la\":\"all\",\"mode\":\"output\"}", 0, &changed, &affected));
    CHECK_STR(err, "\"la\":\"all\" is only allowed with \"mode\":\"off\"");
    CHECK(gpio_json("{\"cmd\":\"gpio\",\"la\":\"all\",\"mode\":\"off\"}", 0, &changed, &affected));
    CHECK(changed == (PULL(1) | PULL(2) | PULL(3) | PULL(5)));
    CHECK(la_pins_owned_mask() == PULL(10));   /* uart_tx untouched */

    /* parse errors */
    la_gpio_req_t req;
    CHECK(!la_gpio_req_parse("{\"cmd\":\"gpio\",\"la\":13,\"mode\":\"input\"}", &req, err, sizeof(err)));
    CHECK_STR(err, "la must be 1..12 (or \"all\" with \"mode\":\"off\")");
    CHECK(!la_gpio_req_parse("{\"cmd\":\"gpio\",\"la\":1,\"mode\":\"push\"}", &req, err, sizeof(err)));
    CHECK_STR(err, "mode must be input, output, open_drain or off");
    CHECK(!la_gpio_req_parse("{\"cmd\":\"gpio\",\"la\":1,\"level\":2}", &req, err, sizeof(err)));
    CHECK_STR(err, "level must be 0 or 1");
    CHECK(!la_gpio_req_parse("{\"cmd\":\"gpio\",\"mode\":\"input\"}", &req, err, sizeof(err)));
    CHECK_STR(err, "missing la");
    CHECK(!la_gpio_req_parse("{\"cmd\":\"gpio\",\"la\":[],\"mode\":\"input\"}", &req, err, sizeof(err)));
    CHECK(!gpio_json("{\"cmd\":\"gpio\",\"la\":1,\"mode\":\"input\",\"level\":1}", 0, &changed, &affected));
    CHECK_STR(err, "level applies to output and open_drain pins, not input");
    /* a read, with nulls */
    CHECK(la_gpio_req_parse("{\"cmd\":\"gpio\",\"la\":null,\"mode\":null}", &req, err, sizeof(err)));
    CHECK(!req.has_mode && !req.has_level && req.n == 0 && !req.all);
    /* duplicates collapse, order kept */
    CHECK(la_gpio_req_parse("{\"cmd\":\"gpio\",\"la\":[ 7, 2,7 ],\"mode\":\"input\"}", &req, err, sizeof(err)));
    CHECK(req.n == 2 && req.las[0] == 7 && req.las[1] == 2);
}

static void test_step_plans(void) {
    la_pins_reset();
    la_step_plan_t plan;
    CHECK(la_pins_plan_step(3, 0, &plan, err, sizeof(err)));
    CHECK(plan.claim_step && !plan.claim_dir);
    CHECK(la_pins_plan_step(3, 4, &plan, err, sizeof(err)));
    CHECK(plan.claim_step && plan.claim_dir);

    /* gpio output pins are used without claiming */
    la_pins_claim(LA_FN_GPIO, LA_GPIO_OUTPUT, 1, (uint8_t[]){ 3, 4 }, 2);
    CHECK(la_pins_plan_step(3, 4, &plan, err, sizeof(err)));
    CHECK(!plan.claim_step && !plan.claim_dir);

    /* a gpio input (or open_drain) pin is not usable: conflict */
    la_pins_claim(LA_FN_GPIO, LA_GPIO_INPUT, 0, (uint8_t[]){ 5 }, 1);
    CHECK(!la_pins_plan_step(5, 0, &plan, err, sizeof(err)));
    CHECK_STR(err, "pin conflict: LA5 is in use by gpio; release it with {\"cmd\":\"gpio\",\"la\":5,\"mode\":\"off\"}");
    CHECK(!la_pins_plan_step(3, 5, &plan, err, sizeof(err)));
    CHECK(strncmp(err, "pin conflict: LA5", 17) == 0);

    /* a running train's pins */
    la_pins_claim(LA_FN_STEP, LA_GPIO_NONE, 0, (uint8_t[]){ 9 }, 1);
    la_pins_claim(LA_FN_STEP_DIR, LA_GPIO_NONE, 0, (uint8_t[]){ 10 }, 1);
    CHECK(!la_pins_plan_step(9, 0, &plan, err, sizeof(err)));
    CHECK_STR(err, "pin conflict: LA9 is in use by step; wait for the step train to finish");
    CHECK(!la_pins_plan_step(11, 10, &plan, err, sizeof(err)));
    CHECK_STR(err, "pin conflict: LA10 is in use by step_dir; wait for the step train to finish");

    CHECK(!la_pins_plan_step(0, 0, &plan, err, sizeof(err)));
    CHECK_STR(err, "invalid args");
    CHECK(!la_pins_plan_step(1, 13, &plan, err, sizeof(err)));
    CHECK_STR(err, "invalid dir_la");
    CHECK(!la_pins_plan_step(1, 1, &plan, err, sizeof(err)));
}

static void test_voltage_refusal(void) {
    la_pins_reset();
    CHECK(la_pins_check_voltage_change(3300, 1800, err, sizeof(err)));   /* nothing owned */
    la_pins_claim(LA_FN_UART_RX, LA_GPIO_NONE, 0, (uint8_t[]){ 3 }, 1);
    la_pins_claim(LA_FN_UART_TX, LA_GPIO_NONE, 0, (uint8_t[]){ 4 }, 1);
    CHECK(!la_pins_check_voltage_change(3300, 1800, err, sizeof(err)));
    CHECK_STR(err, "la voltage can't change while pins are in use: LA3 (uart_rx), LA4 (uart_tx); stop them first");
    CHECK(la_pins_check_voltage_change(3300, 3300, err, sizeof(err)));   /* re-set is allowed */
    CHECK(la_pins_check_voltage_change(3300, 5000, err, sizeof(err)));   /* invalid: caller's error */

    /* all 12 owned: the message still fits the module's buffer */
    const la_fn_t fns[12] = { LA_FN_STEP_DIR, LA_FN_STEP_DIR, LA_FN_I2C_SDA, LA_FN_I2C_SCL,
                              LA_FN_SWD_CLK, LA_FN_SWD_DIO, LA_FN_UART_RX, LA_FN_UART_TX,
                              LA_FN_STEP_DIR, LA_FN_STEP_DIR, LA_FN_STEP_DIR, LA_FN_STEP_DIR };
    for (unsigned la = 1; la <= 12; la++) la_pins_claim(fns[la - 1], LA_GPIO_NONE, 0, (uint8_t[]){ (uint8_t)la }, 1);
    CHECK(!la_pins_check_voltage_change(1800, 3300, err, sizeof(err)));
    size_t len = strlen(err);
    CHECK(len < sizeof(err) - 1);
    CHECK(len > 17 && strcmp(err + len - 17, "; stop them first") == 0);
}

static void test_json_entries(void) {
    la_pins_reset();
    char buf[1600];
    bp_emit_t e;

    bp_emit_init(&e, buf, sizeof(buf));
    la_pins_emit_entry(&e, 1, 0);
    CHECK_STR(buf, "{\"la\":1,\"function\":\"none\",\"gpio\":null,\"level\":null,\"pull\":{\"dir\":\"up\",\"ohms\":\"4.7k\",\"on\":false}}");

    la_pins_claim(LA_FN_GPIO, LA_GPIO_OPEN_DRAIN, 1, (uint8_t[]){ 8 }, 1);
    bp_emit_init(&e, buf, sizeof(buf));
    la_pins_emit_entry(&e, 8, PULL(8));
    CHECK_STR(buf, "{\"la\":8,\"function\":\"gpio\",\"gpio\":\"open_drain\",\"level\":1,\"pull\":{\"dir\":\"down\",\"ohms\":\"10k\",\"on\":true}}");

    la_pins_claim(LA_FN_SWD_DIO, LA_GPIO_NONE, 0, (uint8_t[]){ 12 }, 1);
    bp_emit_init(&e, buf, sizeof(buf));
    la_pins_emit_list(&e, PULL(8) | PULL(12), 0);
    CHECK_STR(buf, "[{\"la\":8,\"function\":\"gpio\",\"gpio\":\"open_drain\",\"level\":1,\"pull\":{\"dir\":\"down\",\"ohms\":\"10k\",\"on\":false}},"
                   "{\"la\":12,\"function\":\"swd_dio\",\"gpio\":null,\"level\":null,\"pull\":null}]");

    /* Worst case the la_pins / gpio read reply can reach: every pin a gpio open_drain with its
       pull on (the longest entry), plus the envelope.  It must fit the cloud reply capture. */
    la_pins_reset();
    for (unsigned la = 1; la <= 12; la++) la_pins_claim(LA_FN_GPIO, LA_GPIO_OPEN_DRAIN, 1, (uint8_t[]){ (uint8_t)la }, 1);
    bp_emit_init(&e, buf, sizeof(buf));
    bp_emit_raw(&e, "{\"status\":\"ok\",\"data\":{\"pins\":");
    la_pins_emit_list(&e, 0x0FFF, 0x00FF);
    bp_emit_raw(&e, ",\"levels\":4095}}\n");
    CHECK(bp_emit_ok(&e));
    printf("  la_pins worst-case reply: %u bytes (cloud reply cap %u)\n",
           (unsigned)bp_emit_len(&e), (unsigned)BP_CLOUD_REPLY_MAX);
    CHECK(bp_emit_len(&e) < BP_CLOUD_REPLY_MAX);
}

static void test_trigger_parse(void) {
    la_trigger_t t;
    CHECK(la_trigger_parse("{\"cmd\":\"la_capture\",\"samples\":100}", &t, err, sizeof(err)));
    CHECK(!t.present);
    CHECK(la_trigger_parse("{\"cmd\":\"la_capture\",\"trigger\":null}", &t, err, sizeof(err)));
    CHECK(!t.present);

    CHECK(la_trigger_parse("{\"cmd\":\"la_capture\",\"samples\":100,\"trigger\":{\"la\":9,\"edge\":\"falling\"},\"trigger_timeout_ms\":2500}",
                           &t, err, sizeof(err)));
    CHECK(t.present && t.la == 9 && t.edge == LA_EDGE_FALLING && t.timeout_ms == 2500);
    CHECK(la_trigger_parse("{\"cmd\":\"capture_dual\",\"trigger\":{\"la\":1}}", &t, err, sizeof(err)));
    CHECK(t.present && t.edge == LA_EDGE_RISING && t.timeout_ms == LA_TRIGGER_TIMEOUT_DEFAULT_MS);
    CHECK(la_trigger_parse("{\"trigger\":{\"edge\":\"low\",\"la\":12}}", &t, err, sizeof(err)));
    CHECK(t.la == 12 && t.edge == LA_EDGE_LOW);

    CHECK(!la_trigger_parse("{\"trigger\":{\"la\":13,\"edge\":\"rising\"}}", &t, err, sizeof(err)));
    CHECK_STR(err, "trigger la must be 1..12");
    CHECK(!la_trigger_parse("{\"trigger\":{\"edge\":\"rising\"}}", &t, err, sizeof(err)));
    CHECK_STR(err, "trigger la must be 1..12");
    CHECK(!la_trigger_parse("{\"trigger\":{\"la\":2,\"edge\":\"up\"}}", &t, err, sizeof(err)));
    CHECK_STR(err, "trigger edge must be rising, falling, high or low");
    CHECK(!la_trigger_parse("{\"trigger\":5}", &t, err, sizeof(err)));
    CHECK(strncmp(err, "trigger must be an object", 25) == 0);
    CHECK(!la_trigger_parse("{\"trigger\":{\"la\":2},\"trigger_timeout_ms\":600001}", &t, err, sizeof(err)));
    CHECK_STR(err, "trigger_timeout_ms must be 1..600000");
    CHECK(!la_trigger_parse("{\"trigger\":{\"la\":2},\"trigger_timeout_ms\":0}", &t, err, sizeof(err)));

    CHECK(la_trigger_parse("{\"trigger\":{\"la\":9,\"edge\":\"rising\"}}", &t, err, sizeof(err)));
    char msg[96];
    la_trigger_timeout_msg(&t, msg, sizeof(msg));
    CHECK_STR(msg, "trigger timeout: no rising edge on LA9 within 10000 ms");
    char buf[96];
    bp_emit_t e;
    bp_emit_init(&e, buf, sizeof(buf));
    la_trigger_emit(&e, &t, true);
    CHECK_STR(buf, "\"trigger\":{\"la\":9,\"edge\":\"rising\",\"fired\":true}");
}

int main(void) {
    test_defaults_and_names();
    test_claims_and_conflicts();
    test_pull_rules();
    test_gpio_modes_and_levels();
    test_step_plans();
    test_voltage_refusal();
    test_json_entries();
    test_trigger_parse();
    if (failures) { printf("test_la_pins: %d FAILURE(S)\n", failures); return 1; }
    printf("test_la_pins: all passed\n");
    return 0;
}
