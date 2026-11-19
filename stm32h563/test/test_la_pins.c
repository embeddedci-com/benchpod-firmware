/*
 * test_la_pins.c — host unit tests for the LA pin ownership table (src/la_pins.c).
 *
 * The messages checked here are a CONTRACT: the Python SDK parses the `pin conflict:` /
 * `pull conflict:` / `trigger timeout:` prefixes, and the rest is what a user reads to fix the
 * problem.  A test that fails on a message change is doing its job — update the SDK and
 * docs/API.md with it.
 */
#include "la_pins.h"
#include "board_caps.h"
#include "board_rev.h"
#include "bp_limits.h"   /* BP_CLOUD_REPLY_MAX: the la_pins reply must fit the cloud channel */
#include "reply_budget.h"

/* Sizes (with the newline) of the replies listed as over budget in budget_la_pins, when listed. */
#define KNOWN_V3R4_GPIO_LA_PINS   1788
#define KNOWN_V3R4_GPIO_GPIO      1762
#define KNOWN_V3_SENSOR_LA_PINS   2288
#define KNOWN_V3_SENSOR_GPIO      2288
#define KNOWN_V3R4_SENSOR_LA_PINS 2418
#define KNOWN_V3R4_SENSOR_GPIO    2392

#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); failures++; } } while (0)
#define CHECK_STR(got, want) do { if (strcmp((got), (want)) != 0) { \
    printf("FAIL %s:%d\n  got:  %s\n  want: %s\n", __FILE__, __LINE__, (got), (want)); failures++; } } while (0)

static char err[LA_PINS_ERR_MAX];

#define PULL(la) ((uint16_t)(1u << ((la) - 1u)))

static bool claim(la_fn_t fn, la_gpio_mode_t mode, const uint8_t *las, size_t n, uint32_t free_fns,
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
    for (unsigned la = 1; la <= 14; la++) {
        CHECK(la_pins_fn(la) == LA_FN_NONE);
        CHECK(la_pins_gpio(la) == LA_GPIO_NONE);
        CHECK(la_pins_level(la) == -1);
        CHECK(la_pins_drive(la) == 2);
    }
    CHECK(la_pins_owned_mask() == 0);
    CHECK_STR(la_fn_name(LA_FN_STEP_DIR), "step_dir");
    CHECK_STR(la_fn_name(LA_FN_SPI_SCK), "spi_sck");
    CHECK_STR(la_fn_name(LA_FN_I2CM_SDA), "i2cm_sda");
    CHECK_STR(la_fn_name(LA_FN_I2CM_SCL), "i2cm_scl");
    CHECK_STR(la_fn_name(LA_FN_SPI_CS), "spi_cs");
    CHECK_STR(la_fn_name(LA_FN_SWD_DIO), "swd_dio");
    CHECK(la_pull_dir(1) == LA_PULL_UP && la_pull_dir(6) == LA_PULL_UP);
    CHECK(la_pull_dir(7) == LA_PULL_DOWN && la_pull_dir(8) == LA_PULL_DOWN);
    CHECK(la_pull_dir(9) == LA_PULL_NONE && la_pull_ohms(12) == NULL);
    CHECK_STR(la_pull_ohms(3), "2.2k");
    CHECK_STR(la_pull_ohms(7), "10k");
    CHECK(la_pull_ohms(0) == NULL && la_pull_ohms(15) == NULL && la_pull_dir(15) == LA_PULL_NONE);
}

/* The pull table (one per board revision) and a host's pull request against it. */
static void test_pull_table_and_requests(void) {
    /* today's boards: LA1-LA6 pull up only, LA7/LA8 down only, LA9-LA14 have none */
    for (unsigned la = 1; la <= 14; la++) {
        bool up = la <= 6, down = la == 7 || la == 8;
        CHECK(la_pull_supports(la, LA_PULL_UP) == up);
        CHECK(la_pull_supports(la, LA_PULL_DOWN) == down);
        CHECK(!la_pull_supports(la, LA_PULL_NONE));
    }
    CHECK_STR(la_pull_ohms(1), "4.7k");
    CHECK_STR(la_pull_ohms(5), "10k");
    CHECK_STR(la_pull_dir_name(LA_PULL_UP), "up");
    CHECK_STR(la_pull_dir_name(LA_PULL_DOWN), "down");
    CHECK_STR(la_pull_dir_name(LA_PULL_NONE), "off");

    la_pull_dir_t d;
    CHECK(la_pull_dir_parse("up", &d) && d == LA_PULL_UP);
    CHECK(la_pull_dir_parse("down", &d) && d == LA_PULL_DOWN);
    CHECK(la_pull_dir_parse("off", &d) && d == LA_PULL_NONE);
    CHECK(la_pull_dir_parse("none", &d) && d == LA_PULL_NONE);
    CHECK(!la_pull_dir_parse("on", &d) && !la_pull_dir_parse("", &d) && !la_pull_dir_parse(NULL, &d));

    /* "pull": the direction asked for */
    d = LA_PULL_DOWN;
    CHECK(la_pull_request(1, "up", NULL, &d) == LA_PULL_REQ_OK && d == LA_PULL_UP);
    CHECK(la_pull_request(1, "off", NULL, &d) == LA_PULL_REQ_OK && d == LA_PULL_NONE);
    CHECK(la_pull_request(7, "down", NULL, &d) == LA_PULL_REQ_OK && d == LA_PULL_DOWN);
    d = LA_PULL_NONE;
    CHECK(la_pull_request(7, "up", NULL, &d) == LA_PULL_REQ_DIRECTION && d == LA_PULL_UP);   /* BP0024 want */
    CHECK(la_pull_request(3, "down", NULL, &d) == LA_PULL_REQ_DIRECTION && d == LA_PULL_DOWN);
    CHECK(la_pull_request(9, "up", NULL, &d) == LA_PULL_REQ_NO_PULL);
    CHECK(la_pull_request(14, "down", NULL, &d) == LA_PULL_REQ_NO_PULL);
    d = LA_PULL_UP;
    CHECK(la_pull_request(9, "off", NULL, &d) == LA_PULL_REQ_OK && d == LA_PULL_NONE);  /* nothing to do */
    CHECK(la_pull_request(1, "sideways", NULL, &d) == LA_PULL_REQ_BAD_VALUE);
    CHECK(la_pull_request(1, "on", NULL, &d) == LA_PULL_REQ_BAD_VALUE);   /* on/off is the alias' */

    /* "pullup": the older alias, on = the channel's own direction */
    CHECK(la_pull_request(2, NULL, "on", &d) == LA_PULL_REQ_OK && d == LA_PULL_UP);
    CHECK(la_pull_request(8, NULL, "on", &d) == LA_PULL_REQ_OK && d == LA_PULL_DOWN);
    CHECK(la_pull_request(8, NULL, "1", &d) == LA_PULL_REQ_OK && d == LA_PULL_DOWN);
    CHECK(la_pull_request(8, NULL, "off", &d) == LA_PULL_REQ_OK && d == LA_PULL_NONE);
    CHECK(la_pull_request(8, NULL, "0", &d) == LA_PULL_REQ_OK && d == LA_PULL_NONE);
    CHECK(la_pull_request(10, NULL, "on", &d) == LA_PULL_REQ_NO_PULL);
    CHECK(la_pull_request(10, NULL, "off", &d) == LA_PULL_REQ_OK && d == LA_PULL_NONE);
    /* both given: "pull" wins */
    CHECK(la_pull_request(7, "off", "on", &d) == LA_PULL_REQ_OK && d == LA_PULL_NONE);

    /* the supported directions as JSON */
    char buf[32];
    bp_emit_t e;
    bp_emit_init(&e, buf, sizeof(buf)); la_pull_emit_dirs(&e, 1);  CHECK_STR(buf, "[\"up\"]");
    bp_emit_init(&e, buf, sizeof(buf)); la_pull_emit_dirs(&e, 8);  CHECK_STR(buf, "[\"down\"]");
    bp_emit_init(&e, buf, sizeof(buf)); la_pull_emit_dirs(&e, 11); CHECK_STR(buf, "[]");

    /* a pull-up never conflicts; switching off never does */
    char err[LA_PINS_ERR_MAX];
    la_pins_reset();
    la_pins_claim(LA_FN_UART_RX, LA_GPIO_NONE, 0, (uint8_t[]){ 7 }, 1);
    CHECK(la_pins_check_pull_enable(7, LA_PULL_NONE, err, sizeof(err)));
    CHECK(!la_pins_check_pull_enable(7, LA_PULL_DOWN, err, sizeof(err)));
    la_pins_reset();
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
    CHECK_STR(err, "pin conflict: LA2 is in use by i2c_scl; stop the sensor emulation first ({\"cmd\":\"sensor_stop\",\"all\":true})");
    CHECK(!claim(LA_FN_GPIO, LA_GPIO_OUTPUT, (uint8_t[]){ 6 }, 1, LA_FN_BIT(LA_FN_GPIO), 0));
    CHECK_STR(err, "pin conflict: LA6 is in use by step; wait for the step train to finish");
    la_pins_claim(LA_FN_SPI_CS, LA_GPIO_NONE, 0, (uint8_t[]){ 9 }, 1);
    CHECK(!claim(LA_FN_UART_TX, LA_GPIO_NONE, (uint8_t[]){ 9 }, 1, 0, 0));
    CHECK_STR(err, "pin conflict: LA9 is in use by spi_cs; stop the SPI session first ({\"cmd\":\"spi_stop\"})");
    la_pins_claim(LA_FN_I2CM_SDA, LA_GPIO_NONE, 0, (uint8_t[]){ 4 }, 1);
    CHECK(!claim(LA_FN_GPIO, LA_GPIO_OUTPUT, (uint8_t[]){ 4 }, 1, LA_FN_BIT(LA_FN_GPIO), 0));
    CHECK_STR(err, "pin conflict: LA4 is in use by i2cm_sda; stop the I2C session first ({\"cmd\":\"i2c_stop\"})");

    /* a replacing sensor_start treats the old sensor's pins as free */
    la_pins_reset();
    la_pins_claim(LA_FN_I2C_SDA, LA_GPIO_NONE, 0, (uint8_t[]){ 1 }, 1);
    la_pins_claim(LA_FN_I2C_SCL, LA_GPIO_NONE, 0, (uint8_t[]){ 2 }, 1);
    uint32_t i2c_free = LA_FN_BIT(LA_FN_I2C_SDA) | LA_FN_BIT(LA_FN_I2C_SCL);
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
                   "(the line idles high); disable it with {\"cmd\":\"la\",\"la\":7,\"pull\":\"off\"} "
                   "or use another channel");
    CHECK(!claim(LA_FN_I2C_SCL, LA_GPIO_NONE, (uint8_t[]){ 7 }, 1, 0, down7));
    CHECK(strstr(err, "which i2c_scl can't work with (an open-drain bus needs pull-ups)") != NULL);
    CHECK(!claim(LA_FN_I2CM_SDA, LA_GPIO_NONE, (uint8_t[]){ 7 }, 1, 0, down7));
    CHECK(strstr(err, "which i2cm_sda can't work with (an open-drain bus needs pull-ups)") != NULL);
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
    CHECK(!la_pins_check_pull_enable(8, la_pull_dir(8), err, sizeof(err)));
    CHECK_STR(err, "pull conflict: LA8 is used by uart_rx, which can't work with the 10k pull-down (the line idles high)");
    CHECK(la_pins_check_pull_enable(7, la_pull_dir(7), err, sizeof(err)));   /* LA7 is free */
    la_pins_claim(LA_FN_UART_RX, LA_GPIO_NONE, 0, (uint8_t[]){ 3 }, 1);
    CHECK(la_pins_check_pull_enable(3, la_pull_dir(3), err, sizeof(err)));   /* pull-up: fine */
    la_pins_claim(LA_FN_GPIO, LA_GPIO_OPEN_DRAIN, 1, (uint8_t[]){ 7 }, 1);
    CHECK(!la_pins_check_pull_enable(7, la_pull_dir(7), err, sizeof(err)));
    CHECK_STR(err, "pull conflict: LA7 is used by gpio open_drain, which can't work with the 10k pull-down (a released line would read low)");
    la_pins_claim(LA_FN_GPIO, LA_GPIO_OUTPUT, 1, (uint8_t[]){ 7 }, 1);
    CHECK(la_pins_check_pull_enable(7, la_pull_dir(7), err, sizeof(err)));
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
    CHECK(!la_gpio_req_parse("{\"cmd\":\"gpio\",\"la\":15,\"mode\":\"input\"}", &req, err, sizeof(err)));
    CHECK_STR(err, "la must be 1..14 (or \"all\" with \"mode\":\"off\")");
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
    CHECK(!la_pins_plan_step(1, 15, &plan, err, sizeof(err)));
    CHECK_STR(err, "invalid dir_la");
    CHECK(!la_pins_plan_step(1, 1, &plan, err, sizeof(err)));
}

/* Step counts above the 16-bit gateware counter are refused with a message naming the range
   (they used to come back as "invalid args"); JSON numbers parse strictly. */
static void test_step_args(void) {
    CHECK(la_step_args_ok(1, 4, err, sizeof(err)));
    CHECK(la_step_args_ok(65535, 65535, err, sizeof(err)));
    CHECK(!la_step_args_ok(65536, 1000, err, sizeof(err)));
    CHECK(strstr(err, "steps must be 1..65535") == err);
    CHECK(!la_step_args_ok(0, 1000, err, sizeof(err)));
    CHECK(strstr(err, "steps must be 1..65535") == err);
    CHECK(!la_step_args_ok(10, 3, err, sizeof(err)));
    CHECK(strstr(err, "delay_us must be 4..65535 (microseconds per half-phase") == err);
    CHECK(!la_step_args_ok(10, 65536, err, sizeof(err)));
    CHECK(strstr(err, "delay_us must be 4..65535") == err);

    uint32_t v = 0;
    CHECK(la_step_parse_u32("70000", &v) && v == 70000);
    CHECK(la_step_parse_u32("4294967295", &v) && v == 4294967295u);
    CHECK(!la_step_parse_u32("4294967296", &v));
    CHECK(!la_step_parse_u32("1e5", &v));
    CHECK(!la_step_parse_u32("-1", &v));
    CHECK(!la_step_parse_u32("12x", &v));
    CHECK(!la_step_parse_u32("", &v));
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
    /* Digital v3r4 (5 V I/O): 5000 is a voltage there, so it is a change like any other. */
    board_caps_init(BOARD_REV_V3R4);
    CHECK(!la_pins_check_voltage_change(3300, 5000, err, sizeof(err)));
    CHECK(la_pins_check_voltage_change(5000, 5000, err, sizeof(err)));   /* re-set is allowed */
    board_caps_init(BOARD_REV_V3);
    CHECK(la_pins_check_voltage_change(3300, 5000, err, sizeof(err)));   /* older boards: as before */

    /* all 14 owned: the message still fits the module's buffer */
    const la_fn_t fns[14] = { LA_FN_STEP_DIR, LA_FN_STEP_DIR, LA_FN_I2C_SDA, LA_FN_I2C_SCL,
                              LA_FN_SWD_CLK, LA_FN_SWD_DIO, LA_FN_UART_RX, LA_FN_UART_TX,
                              LA_FN_STEP_DIR, LA_FN_STEP_DIR, LA_FN_STEP_DIR, LA_FN_STEP_DIR,
                              LA_FN_STEP_DIR, LA_FN_STEP_DIR };
    for (unsigned la = 1; la <= 14; la++) la_pins_claim(fns[la - 1], LA_GPIO_NONE, 0, (uint8_t[]){ (uint8_t)la }, 1);
    CHECK(!la_pins_check_voltage_change(1800, 3300, err, sizeof(err)));
    size_t len = strlen(err);
    CHECK(len < sizeof(err) - 1);
    CHECK(len > 17 && strcmp(err + len - 17, "; stop them first") == 0);
}

/* Digital v3r4: every channel pulls up or down; the strength follows the pin's function; the
   engaged direction and ohms are what the driver noted.  Older boards keep their table. */
static void test_pull_v3r4(void) {
    la_pins_reset();
    board_caps_init(BOARD_REV_V3);
    CHECK(la_pull_supports(1, LA_PULL_UP) && !la_pull_supports(1, LA_PULL_DOWN));
    CHECK(la_pull_ohms(9) == NULL);
    CHECK_STR(la_pull_ohms(1), "4.7k");
    la_pins_note_pull(1, LA_PULL_DOWN, "2.2k");   /* ignored on older boards */
    CHECK(la_pull_dir(1) == LA_PULL_UP);
    CHECK_STR(la_pull_ohms(1), "4.7k");
    la_pins_note_pull(1, LA_PULL_NONE, NULL);

    board_caps_init(BOARD_REV_V3R4);
    for (unsigned la = 1; la <= 14; la++) {
        CHECK(la_pull_supports(la, LA_PULL_UP) && la_pull_supports(la, LA_PULL_DOWN));
        CHECK(la_pull_dir(la) == LA_PULL_UP);
        CHECK_STR(la_pull_ohms(la), "10k");
    }
    la_pins_note_pull(7, LA_PULL_DOWN, "1.8k");
    CHECK(la_pull_dir(7) == LA_PULL_DOWN);
    CHECK_STR(la_pull_ohms(7), "1.8k");
    la_pins_note_pull(7, LA_PULL_NONE, NULL);
    CHECK(la_pull_dir(7) == LA_PULL_UP);
    CHECK_STR(la_pull_ohms(7), "10k");
    /* up/down/off requests: every direction is fine on every channel */
    la_pull_dir_t want;
    CHECK(la_pull_request(12, "down", NULL, &want) == LA_PULL_REQ_OK && want == LA_PULL_DOWN);
    CHECK(la_pull_request(3, NULL, "on", &want) == LA_PULL_REQ_OK && want == LA_PULL_UP);

    /* strength from the function */
    CHECK(la_pull_strength_for(1, 3300) == LA_PULL_STR_10K);                 /* nothing on it */
    la_pins_claim(LA_FN_I2C_SDA, LA_GPIO_NONE, 0, (uint8_t[]){ 1 }, 1);
    la_pins_claim(LA_FN_I2CM_SCL, LA_GPIO_NONE, 0, (uint8_t[]){ 2 }, 1);
    la_pins_claim(LA_FN_UART_RX, LA_GPIO_NONE, 0, (uint8_t[]){ 3 }, 1);
    la_pins_claim(LA_FN_SPI_SCK, LA_GPIO_NONE, 0, (uint8_t[]){ 4 }, 1);
    la_pins_claim(LA_FN_GPIO, LA_GPIO_OPEN_DRAIN, 1, (uint8_t[]){ 5 }, 1);
    la_pins_claim(LA_FN_GPIO, LA_GPIO_OUTPUT, 1, (uint8_t[]){ 6 }, 1);
    la_pins_claim(LA_FN_UART_HD, LA_GPIO_NONE, 0, (uint8_t[]){ 8 }, 1);
    CHECK(la_pull_strength_for(1, 3300) == LA_PULL_STR_2K2);    /* I2C */
    CHECK(la_pull_strength_for(2, 1800) == LA_PULL_STR_2K2);    /* I2C master */
    CHECK(la_pull_strength_for(3, 5000) == LA_PULL_STR_10K);    /* UART RX hold, also at 5 V */
    CHECK(la_pull_strength_for(4, 3300) == LA_PULL_STR_10K);    /* an output at 3.3 V */
    CHECK(la_pull_strength_for(4, 5000) == LA_PULL_STR_2K2);    /* an output at 5 V: the lift */
    CHECK(la_pull_strength_for(5, 3300) == LA_PULL_STR_2K2);    /* open drain */
    CHECK(la_pull_strength_for(6, 5000) == LA_PULL_STR_2K2);    /* gpio output at 5 V */
    CHECK(la_pull_strength_for(6, 3300) == LA_PULL_STR_10K);
    CHECK(la_pull_strength_for(8, 3300) == LA_PULL_STR_2K2);    /* one-wire UART */
    CHECK(la_pull_strength_for(0, 3300) == LA_PULL_STR_10K && la_pull_strength_for(15, 3300) == LA_PULL_STR_10K);
    board_caps_init(BOARD_REV_V3);
    la_pins_reset();
}

static const char *test_namer(unsigned slot) { return slot == 0 ? "env" : slot == 1 ? "imu" : NULL; }

/* Emulated sensors own their pins by instance: the conflict names the instance and the stop. */
static void test_sensor_owner(void) {
    la_pins_reset();
    la_pins_set_owner_namer(test_namer);
    la_pins_claim(LA_FN_I2C_SDA, LA_GPIO_NONE, 0, (uint8_t[]){ 1 }, 1);
    la_pins_set_owner(1, 0x03);
    la_pins_claim(LA_FN_SENSOR_TX, LA_GPIO_NONE, 0, (uint8_t[]){ 9 }, 1);
    la_pins_set_owner(9, 0x02);
    CHECK(la_fn_is_sensor(LA_FN_SENSOR_TX) && la_fn_is_sensor(LA_FN_I2C_SDA) && !la_fn_is_sensor(LA_FN_GPIO));
    CHECK(!claim(LA_FN_GPIO, LA_GPIO_OUTPUT, (uint8_t[]){ 9 }, 1, 0, 0));
    CHECK_STR(err, "pin conflict: LA9 is in use by sensor:imu (sensor_tx); stop the sensor emulation first ({\"cmd\":\"sensor_stop\",\"instance\":\"imu\"})");
    char buf[256];
    bp_emit_t e;
    bp_emit_init(&e, buf, sizeof(buf));
    la_pins_emit_entry(&e, 1, 0);
    CHECK_STR(buf, "{\"la\":1,\"function\":\"i2c_sda\",\"owner\":\"sensor:env+1\",\"gpio\":null,\"level\":null,\"pull\":{\"dir\":\"up\",\"ohms\":\"4.7k\",\"on\":false,\"dirs\":[\"up\"]}}");
    /* a pull-down under a UART sensor line is refused like under the proxy */
    CHECK(!claim(LA_FN_SENSOR_RX, LA_GPIO_NONE, (uint8_t[]){ 7 }, 1, 0, PULL(7)));
    la_pins_release(1);
    CHECK(la_pins_owner(1) == 0);
    la_pins_set_owner_namer(NULL);
    la_pins_reset();
}

/* ---- reply budgets (reply_budget.h) ---- */

static const char *long_namer(unsigned slot) { (void)slot; return "abcdefghijklmnop"; }   /* SM_NAME_MAX */

typedef enum { WORST_GPIO, WORST_SENSOR } worst_t;

/* Every channel at its longest entry: a gpio open_drain (WORST_GPIO), or a pin of an emulated
   sensor with a 16-character instance name (WORST_SENSOR; 4 I2C instances with data-ready, the SPI
   and the UART instance own more than 14 pins). Pull on, the longest ohms and direction name,
   both warnings. */
static void set_worst(worst_t w) {
    la_pins_reset();
    la_pins_set_owner_namer(w == WORST_SENSOR ? long_namer : NULL);
    for (unsigned la = 1; la <= 14; la++) {
        uint8_t pin = (uint8_t)la;
        if (w == WORST_GPIO) la_pins_claim(LA_FN_GPIO, LA_GPIO_OPEN_DRAIN, 1, &pin, 1);
        else { la_pins_claim(LA_FN_SENSOR_MISO, LA_GPIO_NONE, 0, &pin, 1); la_pins_set_owner(la, 0x01); }
        la_pins_set_dir(la, LA_DIR_DUT_OUTPUT);
        if (board_caps()->pull_expanders) la_pins_note_pull(la, LA_PULL_DOWN, "1.8k");
    }
    la_pins_set_target_powered(false);
}

static void clear_worst(void) {
    for (unsigned la = 1; la <= 14; la++) {
        la_pins_set_dir(la, LA_DIR_UNKNOWN);
        la_pins_set_owner(la, 0);
        la_pins_note_pull(la, LA_PULL_NONE, NULL);
    }
    la_pins_set_owner_namer(NULL);
    la_pins_set_target_powered(true);
    la_pins_reset();
}

/* The la_pins reply (la_pins_emit_reply, what handle_la_pins sends), and the gpio read reply (handle_gpio). */
static size_t emit_la_pins_of(char *buf, size_t cap, uint16_t only) {
    bp_emit_t e;
    bp_emit_init(&e, buf, cap);
    la_pins_emit_reply(&e, only, 0x3FFF, true, 0x3FFF);   /* the reply handle_la_pins sends */
    CHECK(bp_emit_ok(&e));
    return bp_emit_len(&e);
}

static size_t emit_la_pins(char *buf, size_t cap) { return emit_la_pins_of(buf, cap, 0x3FFF); }

static size_t emit_gpio_read(char *buf, size_t cap) {
    bp_emit_t e;
    bp_emit_init(&e, buf, cap);
    bp_emit_raw(&e, "{\"status\":\"ok\",\"data\":{\"levels\":16383,\"pins\":");
    la_pins_emit_list(&e, 0x3FFF, 0x3FFF);
    bp_emit_raw(&e, "}}\n");
    CHECK(bp_emit_ok(&e));
    return bp_emit_len(&e);
}

/* Replies known to be over budget (reply_budget.h), with the size measured when listed. 0 = must
   keep the margin. The v3 and v3r4 tables differ only in the pull table, so v2 and v3r2 are v3.
   Over budget: the gpio case on v3r4 (3 bytes under the limit) and the sensor case on every
   board (over the limit: "reply too large" over LAN and cloud). For those, a host reads la_pins
   in parts with the subset query ("la":[1,...,7] then [8,...,14]); budget_la_pins checks that
   each half keeps the margin. The gpio read has no subset: hosts use la_pins. */
static const struct { int rev; worst_t w; const char *what; size_t la_pins, gpio; } s_known[] = {
    { BOARD_REV_V2,   WORST_GPIO,   "v2, every pin gpio",       0,    0    },
    { BOARD_REV_V3,   WORST_GPIO,   "v3, every pin gpio",       0,    0    },
    { BOARD_REV_V3R2, WORST_GPIO,   "v3r2, every pin gpio",     0,    0    },
    { BOARD_REV_V3R4, WORST_GPIO,   "v3r4, every pin gpio",     KNOWN_V3R4_GPIO_LA_PINS, KNOWN_V3R4_GPIO_GPIO },
    { BOARD_REV_V2,   WORST_SENSOR, "v2, every pin a sensor",   KNOWN_V3_SENSOR_LA_PINS, KNOWN_V3_SENSOR_GPIO },
    { BOARD_REV_V3,   WORST_SENSOR, "v3, every pin a sensor",   KNOWN_V3_SENSOR_LA_PINS, KNOWN_V3_SENSOR_GPIO },
    { BOARD_REV_V3R2, WORST_SENSOR, "v3r2, every pin a sensor", KNOWN_V3_SENSOR_LA_PINS, KNOWN_V3_SENSOR_GPIO },
    { BOARD_REV_V3R4, WORST_SENSOR, "v3r4, every pin a sensor", KNOWN_V3R4_SENSOR_LA_PINS, KNOWN_V3R4_SENSOR_GPIO },
};

static void budget_la_pins(void) {
    static char buf[4096];
    char what[64];
    for (size_t i = 0; i < sizeof(s_known) / sizeof(s_known[0]); i++) {
        board_caps_init(s_known[i].rev);
        set_worst(s_known[i].w);
        size_t n = emit_la_pins(buf, sizeof(buf));
        snprintf(what, sizeof(what), "la_pins, %s", s_known[i].what);
        if (s_known[i].la_pins) {
            CHECK(reply_budget_known(what, n, REPLY_LIMIT_CLOUD, s_known[i].la_pins));
            printf("  reply budget: %-44s use the subset query (\"la\":[...]), checked below\n", what);
        }
        else                    CHECK(reply_budget_ok(what, n, REPLY_LIMIT_CLOUD, REPLY_BUDGET_MARGIN));
        n = emit_gpio_read(buf, sizeof(buf));
        snprintf(what, sizeof(what), "gpio read, %s", s_known[i].what);
        if (s_known[i].gpio) CHECK(reply_budget_known(what, n, REPLY_LIMIT_CLOUD, s_known[i].gpio));
        else                 CHECK(reply_budget_ok(what, n, REPLY_LIMIT_CLOUD, REPLY_BUDGET_MARGIN));
        /* The subset query: every case, over budget or not, fits in two halves. */
        static const uint16_t halves[2] = { 0x007F, 0x3F80 };
        for (int h = 0; h < 2; h++) {
            n = emit_la_pins_of(buf, sizeof(buf), halves[h]);
            snprintf(what, sizeof(what), "la_pins LA%s, %s", h ? "8-14" : "1-7", s_known[i].what);
            CHECK(reply_budget_ok(what, n, REPLY_LIMIT_CLOUD, REPLY_BUDGET_MARGIN));
        }
        clear_worst();
    }
    board_caps_init(BOARD_REV_V3);
}

/* {"cmd":"la_pins"} with and without the pin subset. */
static void test_query_mask(void) {
    uint16_t m = 0;
    char e2[LA_PINS_ERR_MAX];
    CHECK(la_pins_query_mask("{\"cmd\":\"la_pins\"}", &m, e2, sizeof(e2)) && m == 0x3FFF);
    CHECK(la_pins_query_mask("{\"cmd\":\"la_pins\",\"la\":null}", &m, e2, sizeof(e2)) && m == 0x3FFF);
    CHECK(la_pins_query_mask("{\"cmd\":\"la_pins\",\"la\":\"all\"}", &m, e2, sizeof(e2)) && m == 0x3FFF);
    CHECK(la_pins_query_mask("{\"cmd\":\"la_pins\",\"la\":3}", &m, e2, sizeof(e2)) && m == 0x0004);
    CHECK(la_pins_query_mask("{\"cmd\":\"la_pins\",\"la\":[1, 14,2,1]}", &m, e2, sizeof(e2)) && m == 0x2003);
    CHECK(la_pins_query_mask("{\"la\":[8],\"cmd\":\"la_pins\"}", &m, e2, sizeof(e2)) && m == 0x0080);
    /* Stray fields never refuse a query (la_pins always ignored them). */
    CHECK(la_pins_query_mask("{\"cmd\":\"la_pins\",\"mode\":\"bogus\",\"level\":7}", &m, e2, sizeof(e2)) && m == 0x3FFF);
    CHECK(la_pins_query_mask("{\"cmd\":\"la_pins\",\"mode\":\"bogus\",\"la\":[2]}", &m, e2, sizeof(e2)) && m == 0x0002);
    CHECK(!la_pins_query_mask("{\"cmd\":\"la_pins\",\"la\":15}", &m, e2, sizeof(e2)));
    CHECK(!la_pins_query_mask("{\"cmd\":\"la_pins\",\"la\":[]}", &m, e2, sizeof(e2)));
    CHECK(!la_pins_query_mask("{\"cmd\":\"la_pins\",\"la\":\"some\"}", &m, e2, sizeof(e2)));
    /* The full listing through the mask is the same bytes as before. */
    la_pins_reset();
    static char a[2048], b[2048];
    bp_emit_t ea, eb;
    bp_emit_init(&ea, a, sizeof(a)); la_pins_emit_list(&ea, 0x3FFFu, 0x00FF);
    la_pins_query_mask("{\"cmd\":\"la_pins\"}", &m, e2, sizeof(e2));
    bp_emit_init(&eb, b, sizeof(b)); la_pins_emit_list(&eb, m, 0x00FF);
    CHECK_STR(a, b);
}

static void test_json_entries(void) {
    la_pins_reset();
    char buf[2048];
    bp_emit_t e;

    bp_emit_init(&e, buf, sizeof(buf));
    la_pins_emit_entry(&e, 1, 0);
    CHECK_STR(buf, "{\"la\":1,\"function\":\"none\",\"gpio\":null,\"level\":null,\"pull\":{\"dir\":\"up\",\"ohms\":\"4.7k\",\"on\":false,\"dirs\":[\"up\"]}}");

    la_pins_claim(LA_FN_GPIO, LA_GPIO_OPEN_DRAIN, 1, (uint8_t[]){ 8 }, 1);
    bp_emit_init(&e, buf, sizeof(buf));
    la_pins_emit_entry(&e, 8, PULL(8));
    CHECK_STR(buf, "{\"la\":8,\"function\":\"gpio\",\"gpio\":\"open_drain\",\"level\":1,\"pull\":{\"dir\":\"down\",\"ohms\":\"10k\",\"on\":true,\"dirs\":[\"down\"]}}");

    la_pins_claim(LA_FN_SWD_DIO, LA_GPIO_NONE, 0, (uint8_t[]){ 12 }, 1);
    bp_emit_init(&e, buf, sizeof(buf));
    la_pins_emit_list(&e, PULL(8) | PULL(12), 0);
    CHECK_STR(buf, "[{\"la\":8,\"function\":\"gpio\",\"gpio\":\"open_drain\",\"level\":1,\"pull\":{\"dir\":\"down\",\"ohms\":\"10k\",\"on\":false,\"dirs\":[\"down\"]}},"
                   "{\"la\":12,\"function\":\"swd_dio\",\"gpio\":null,\"level\":null,\"pull\":null}]");

    budget_la_pins();
}

static void test_direction_and_warnings(void) {
    la_pins_reset();
    for (unsigned la = 1; la <= 14; la++) la_pins_set_dir(la, LA_DIR_UNKNOWN);
    la_dir_t d;
    CHECK(la_dir_parse("dut_output", &d) && d == LA_DIR_DUT_OUTPUT);
    CHECK(!la_dir_parse("output", &d));

    /* a push-pull driver may not claim a DUT output; open drain and inputs may */
    la_pins_set_dir(9, LA_DIR_DUT_OUTPUT);
    CHECK(!claim(LA_FN_GPIO, LA_GPIO_OUTPUT, (uint8_t[]){ 9 }, 1, LA_FN_BIT(LA_FN_GPIO), 0));
    CHECK_STR(err, "direction conflict: LA9 is wired to a DUT output and gpio output would drive "
                   "against it; use gpio open_drain, or fix the wiring if LA9 is not a DUT output");
    CHECK(!claim(LA_FN_SPI_MOSI, LA_GPIO_NONE, (uint8_t[]){ 9 }, 1, 0, 0));
    CHECK(!claim(LA_FN_UART_TX, LA_GPIO_NONE, (uint8_t[]){ 9 }, 1, 0, 0));
    CHECK(claim(LA_FN_SPI_MISO, LA_GPIO_NONE, (uint8_t[]){ 9 }, 1, 0, 0));
    la_pins_release(9);
    CHECK(claim(LA_FN_GPIO, LA_GPIO_OPEN_DRAIN, (uint8_t[]){ 9 }, 1, LA_FN_BIT(LA_FN_GPIO), 0));
    CHECK(claim(LA_FN_I2CM_SDA, LA_GPIO_NONE, (uint8_t[]){ 3 }, 1, 0, 0));
    /* switching the open-drain pin to push-pull is a claim too */
    CHECK(!claim(LA_FN_GPIO, LA_GPIO_OUTPUT, (uint8_t[]){ 9 }, 1, LA_FN_BIT(LA_FN_GPIO), 0));

    /* the other order: a driven pin can't be marked a DUT output */
    la_pins_reset();
    la_pins_set_dir(9, LA_DIR_UNKNOWN);
    CHECK(claim(LA_FN_GPIO, LA_GPIO_OUTPUT, (uint8_t[]){ 10 }, 1, LA_FN_BIT(LA_FN_GPIO), 0));
    CHECK(!la_pins_check_dir(10, LA_DIR_DUT_OUTPUT, err, sizeof(err)));
    CHECK_STR(err, "direction conflict: LA10 is driven by gpio output, so it can't be wired to a DUT output; stop that first");
    CHECK(la_pins_check_dir(10, LA_DIR_BIDIR, err, sizeof(err)));
    CHECK(la_pins_check_dir(11, LA_DIR_DUT_OUTPUT, err, sizeof(err)));

    /* warnings */
    la_pins_reset();
    la_pins_set_target_powered(true);
    la_pins_claim(LA_FN_GPIO, LA_GPIO_OUTPUT, 1, (uint8_t[]){ 3 }, 1);       /* 2.2k pull-up */
    CHECK(la_pins_warnings(3, 0) == 0);
    CHECK(la_pins_warnings(3, PULL(3)) == LA_WARN_PULL_VS_DRIVE);
    la_pins_claim(LA_FN_GPIO, LA_GPIO_OPEN_DRAIN, 1, (uint8_t[]){ 4 }, 1);
    CHECK(la_pins_warnings(4, PULL(4)) == 0);                                  /* open drain + pull-up: normal */
    la_pins_set_target_powered(false);
    CHECK(la_pins_warnings(3, 0) == LA_WARN_UNPOWERED);                          /* driven high */
    la_pins_set_level(3, 0);
    CHECK(la_pins_warnings(3, 0) == 0);                                          /* driven low: fine */
    CHECK(la_pins_warnings(4, PULL(4)) == LA_WARN_UNPOWERED);                   /* pulled up */
    CHECK(la_pins_warnings(7, PULL(7)) == 0);                                   /* a pull-down is fine */
    la_pins_claim(LA_FN_UART_TX, LA_GPIO_NONE, 0, (uint8_t[]){ 12 }, 1);
    CHECK(la_pins_warnings(12, 0) == LA_WARN_UNPOWERED);                         /* TX idles high */
    la_pins_set_target_powered(true);

    /* the listing carries dir and warn */
    char buf[256];
    bp_emit_t e;
    la_pins_set_dir(3, LA_DIR_DUT_INPUT);
    bp_emit_init(&e, buf, sizeof(buf));
    la_pins_set_level(3, 1);
    la_pins_emit_entry(&e, 3, PULL(3));
    CHECK_STR(buf, "{\"la\":3,\"function\":\"gpio\",\"gpio\":\"output\",\"level\":1,\"pull\":{\"dir\":\"up\",\"ohms\":\"2.2k\",\"on\":true,\"dirs\":[\"up\"]},\"dir\":\"dut_input\",\"warn\":1}");
    for (unsigned la = 1; la <= 14; la++) la_pins_set_dir(la, LA_DIR_UNKNOWN);
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

    CHECK(!la_trigger_parse("{\"trigger\":{\"la\":15,\"edge\":\"rising\"}}", &t, err, sizeof(err)));
    CHECK_STR(err, "trigger la must be 1..14");
    CHECK(!la_trigger_parse("{\"trigger\":{\"edge\":\"rising\"}}", &t, err, sizeof(err)));
    CHECK_STR(err, "trigger la must be 1..14");
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
    test_direction_and_warnings();
    test_defaults_and_names();
    test_claims_and_conflicts();
    test_pull_rules();
    test_pull_table_and_requests();
    test_gpio_modes_and_levels();
    test_step_plans();
    test_step_args();
    test_voltage_refusal();
    test_pull_v3r4();
    test_sensor_owner();
    test_query_mask();
    test_json_entries();
    test_trigger_parse();
    if (failures) { printf("test_la_pins: %d FAILURE(S)\n", failures); return 1; }
    printf("test_la_pins: all passed\n");
    return 0;
}
