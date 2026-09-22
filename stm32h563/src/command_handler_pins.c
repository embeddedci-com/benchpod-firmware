/*
 * command_handler_pins.c — the LA pin commands (`la_pins`, `gpio`), step-train ownership and
 * the hardware glue around the pure ownership table (la_pins.c): reading which pulls are
 * engaged, driving GPIO_SET from the table, and putting the table back in step with the fabric
 * after a gateware reconfiguration.
 *
 * Everything here runs on the hw worker task, like the rest of command_handler.
 */
#include "command_handler.h"
#include "command_handler_internal.h"
#include "la_pins.h"
#include "signal_engine.h"
#include "i2c_bus.h"
#include "at_driver.h"
#include "bp_json.h"
#include "bp_limits.h"
#include "pico/time.h"

#include <stdio.h>
#include <string.h>

/* ---- glue ---------------------------------------------------------------- */

uint16_t la_pull_mask_now(void) {
    uint16_t mask = 0;
    for (uint8_t la = 1; la <= 8; la++)
        if (pca9555_la_pullup_enabled(la)) mask |= (uint16_t)(1u << (la - 1u));
    return mask;
}

void la_pins_drive_mask(uint16_t mask) {
    for (unsigned la = 1; la <= LA_PINS_COUNT; la++)
        if (mask & (1u << (la - 1u))) (void)fpga_la_set(la, la_pins_drive(la));
}

bool la_claim_or_error(int conn_id, la_fn_t fn, const uint8_t *las, size_t n, uint16_t free_fns) {
    char err[LA_PINS_ERR_MAX];
    if (la_pins_check_claim(fn, LA_GPIO_NONE, las, n, free_fns, la_pull_mask_now(), err, sizeof(err)))
        return true;
    send_error(conn_id, err);
    return false;
}

/* Send a reply built with the emitter, or a structured error if it did not fit. */
static void send_emitted(int conn_id, bp_emit_t *e) {
    if (!bp_emit_ok(e)) { send_error(conn_id, "reply too large"); return; }
    if (at_send_data(conn_id, (const uint8_t *)e->buf, bp_emit_len(e)) != 0)
        at_close_connection(conn_id);
}

/* ---- step trains ---------------------------------------------------------- */

/* STATUS.STEP_BUSY is read back over SPI; don't trust a "not busy" in the first few ms after
   GPIO_STEP, and don't pay an SPI read on every worker pass while a long train runs. */
#define STEP_BUSY_SETTLE_US  5000
#define STEP_POLL_PERIOD_US 10000

static struct {
    bool            active;
    uint8_t         la;
    uint8_t         dir_la;         /* 0 = none */
    bool            claimed_step;   /* step pin was none and is released when the train ends */
    bool            claimed_dir;    /* dir pin was none: set high-Z and released when it ends */
    absolute_time_t next_check;
} s_step;

static void step_finish(void) {
    if (s_step.claimed_step && la_pins_fn(s_step.la) == LA_FN_STEP)
        la_pins_release(s_step.la);
    if (s_step.claimed_dir && la_pins_fn(s_step.dir_la) == LA_FN_STEP_DIR) {
        la_pins_release(s_step.dir_la);
        (void)fpga_la_set(s_step.dir_la, 2);   /* the latch we drove goes back to high-Z */
    }
    s_step.active = false;
}

void la_step_poll(void) {
    if (!s_step.active || !time_reached(s_step.next_check)) return;
    if (fpga_la_step_busy()) {
        s_step.next_check = make_timeout_time_us(STEP_POLL_PERIOD_US);
        return;
    }
    step_finish();
}

int la_step_begin(unsigned la, uint32_t steps, uint32_t delay_us, unsigned dir_la, int dir,
                  char *err, size_t cap) {
    if (s_step.active && time_reached(s_step.next_check) && !fpga_la_step_busy())
        step_finish();   /* a train that already ended but has not been polled yet */

    la_step_plan_t plan;
    if (!la_pins_plan_step(la, dir_la, &plan, err, cap))
        return strncmp(err, "pin conflict:", 13) == 0 ? -3 : -1;
    if (steps == 0 || steps > LA_STEP_MAX_STEPS ||
        delay_us < LA_STEP_MIN_DELAY_US || delay_us > LA_STEP_MAX_DELAY_US) {
        snprintf(err, cap, "invalid args");
        return -1;
    }
    if (s_step.active || fpga_la_step_busy()) { snprintf(err, cap, "busy"); return -2; }

    /* Direction first, so the first pulse already sees it. */
    int prev_level = -1;
    if (dir_la) {
        if (!plan.claim_dir) {
            prev_level = la_pins_level(dir_la);
            la_pins_set_level(dir_la, dir ? 1u : 0u);
        }
        (void)fpga_la_set(dir_la, dir ? 1 : 0);
    }
    int rc = fpga_la_step(la, steps, delay_us);
    if (rc != 0) {
        if (dir_la) {
            if (plan.claim_dir) {
                (void)fpga_la_set(dir_la, 2);
            } else {
                la_pins_set_level(dir_la, prev_level > 0 ? 1u : 0u);
                (void)fpga_la_set(dir_la, la_pins_drive(dir_la));
            }
        }
        snprintf(err, cap, rc == -2 ? "busy" : "invalid args");
        return rc == -2 ? -2 : -1;
    }
    uint8_t p = (uint8_t)la, d = (uint8_t)dir_la;
    if (plan.claim_step) la_pins_claim(LA_FN_STEP, LA_GPIO_NONE, 0, &p, 1);
    if (plan.claim_dir)  la_pins_claim(LA_FN_STEP_DIR, LA_GPIO_NONE, 0, &d, 1);
    s_step.active       = true;
    s_step.la           = p;
    s_step.dir_la       = d;
    s_step.claimed_step = plan.claim_step;
    s_step.claimed_dir  = plan.claim_dir;
    s_step.next_check   = make_timeout_time_us(STEP_BUSY_SETTLE_US);
    return 0;
}

/* ---- reconfiguration ------------------------------------------------------ */

void la_pins_on_gateware_reconfigured(void) {
    /* The fabric reset the stepper (a running train is gone) and every GPIO_SET latch.  Drop the
       step claims and re-drive the gpio pins so the wire matches the table again.  The uart /
       swd / i2c owners are cleaned up by command_handler_on_gateware_reconfigured, which owns
       their session state. */
    s_step.active = false;
    la_pins_release_fn(LA_FN_STEP);
    la_pins_release_fn(LA_FN_STEP_DIR);
    la_pins_drive_mask(la_pins_mask_fn(LA_FN_GPIO));
}

/* ---- SCPI entry points ------------------------------------------------------ */

int command_handler_dig_output(unsigned la, int level) {
    if (la_vccio_get_mv() == LA_VCCIO_UNSET) {
        printf("[scpi] DIG:OUTP refused: la voltage not set (la_voltage first)\n");
        return -2;
    }
    if (la < 1u || la > LA_PINS_COUNT) return -1;
    char err[LA_PINS_ERR_MAX];
    uint8_t p = (uint8_t)la;
    /* none or gpio (any mode) -> gpio output with that level; any other owner -> conflict. */
    if (!la_pins_check_claim(LA_FN_GPIO, LA_GPIO_OUTPUT, &p, 1, LA_FN_BIT(LA_FN_GPIO),
                             la_pull_mask_now(), err, sizeof(err))) {
        printf("[scpi] DIG:OUTP refused: %s\n", err);
        return -2;
    }
    la_pins_claim(LA_FN_GPIO, LA_GPIO_OUTPUT, level ? 1u : 0u, &p, 1);
    (void)fpga_la_set(la, la_pins_drive(la));
    return 0;
}

int command_handler_dig_step(unsigned la, uint32_t steps, uint32_t delay_us,
                             unsigned dir_la, int dir) {
    if (la_vccio_get_mv() == LA_VCCIO_UNSET) {
        printf("[scpi] DIG:STEP refused: la voltage not set (la_voltage first)\n");
        return -2;
    }
    char err[LA_PINS_ERR_MAX];
    int rc = la_step_begin(la, steps, delay_us, dir_la, dir, err, sizeof(err));
    if (rc == 0) return 0;
    printf("[scpi] DIG:STEP refused: %s\n", err);
    if (rc == -3) return -2;   /* conflict */
    if (rc == -2) return -3;   /* busy */
    return -1;
}

bool command_handler_step_busy(void) {
    la_step_poll();
    return fpga_la_step_busy();
}

/* ---- JSON commands ---------------------------------------------------------- */

/* {"cmd":"la_pins"} -> {"pins":[12 entries],"levels":M|null} */
void handle_la_pins(int conn_id) {
    uint16_t pulls  = la_pull_mask_now();
    uint16_t levels = 0;
    bool     have   = signal_engine_fpga_version() >= GPIO_GET_MIN_GW && fpga_gpio_get(&levels) == 0;

    char resp[BP_CLOUD_REPLY_MAX];   /* worst case 1135 B (test_la_pins.c) */
    bp_emit_t e;
    bp_emit_init(&e, resp, sizeof(resp));
    bp_emit_raw(&e, "{\"status\":\"ok\",\"data\":{\"pins\":");
    la_pins_emit_list(&e, 0x3FFFu, pulls);
    if (have) bp_emit(&e, ",\"levels\":%u}}\n", (unsigned)levels);
    else      bp_emit_raw(&e, ",\"levels\":null}}\n");
    send_emitted(conn_id, &e);
}

/* {"cmd":"gpio","la":N|[N,...]|"all","mode":"input"|"output"|"open_drain"|"off","level":0|1}
   {"cmd":"gpio","la":N|[N,...],"level":0|1}
   {"cmd":"gpio"}  (read: needs GPIO_GET in the gateware) */
void handle_gpio(int conn_id, const char *json) {
    if (!require_la_voltage(conn_id)) return;
    char err[LA_PINS_ERR_MAX];
    la_gpio_req_t req;
    if (!la_gpio_req_parse(json, &req, err, sizeof(err))) { send_error(conn_id, err); return; }

    char resp[BP_CLOUD_REPLY_MAX];
    bp_emit_t e;
    bp_emit_init(&e, resp, sizeof(resp));

    if (!req.has_mode && !req.has_level) {
        uint8_t ver = signal_engine_fpga_version();
        if (ver < GPIO_GET_MIN_GW) {
            snprintf(err, sizeof(err), "reading pin levels needs gateware v%u or newer (this pod runs v%u)",
                     (unsigned)GPIO_GET_MIN_GW, (unsigned)ver);
            send_error(conn_id, err);
            return;
        }
        uint16_t levels = 0;
        if (fpga_gpio_get(&levels) != 0) { send_error(conn_id, "pin level read failed (SPI)"); return; }
        bp_emit(&e, "{\"status\":\"ok\",\"data\":{\"levels\":%u,\"pins\":", (unsigned)levels);
        la_pins_emit_list(&e, 0x3FFFu, la_pull_mask_now());
        bp_emit_raw(&e, "}}\n");
        send_emitted(conn_id, &e);
        return;
    }

    uint16_t pulls = la_pull_mask_now();
    uint16_t changed = 0, affected = 0;
    if (!la_pins_gpio_apply(&req, pulls, &changed, &affected, err, sizeof(err))) {
        send_error(conn_id, err);
        return;
    }
    la_pins_drive_mask(changed);
    bp_emit_raw(&e, "{\"status\":\"ok\",\"data\":{\"pins\":");
    la_pins_emit_list(&e, affected, pulls);
    bp_emit_raw(&e, "}}\n");
    send_emitted(conn_id, &e);
}
