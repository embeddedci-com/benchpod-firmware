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
#include "bp_log.h"
#include "sensor_sim.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* JSON parsing is shared (see bp_json.h); same aliases as command_handler.c. */
#define json_get_value      bp_json_get

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
    if (!la_step_args_ok(steps, delay_us, err, cap)) return -1;
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
        log_printf("[scpi] DIG:OUTP refused: la voltage not set (la_voltage first)\n");
        return -2;
    }
    if (la < 1u || la > LA_PINS_COUNT) return -1;
    char err[LA_PINS_ERR_MAX];
    uint8_t p = (uint8_t)la;
    /* none or gpio (any mode) -> gpio output with that level; any other owner -> conflict. */
    if (!la_pins_check_claim(LA_FN_GPIO, LA_GPIO_OUTPUT, &p, 1, LA_FN_BIT(LA_FN_GPIO),
                             la_pull_mask_now(), err, sizeof(err))) {
        log_printf("[scpi] DIG:OUTP refused: %s\n", err);
        return -2;
    }
    la_pins_claim(LA_FN_GPIO, LA_GPIO_OUTPUT, level ? 1u : 0u, &p, 1);
    (void)fpga_la_set(la, la_pins_drive(la));
    return 0;
}

int command_handler_dig_step(unsigned la, uint32_t steps, uint32_t delay_us,
                             unsigned dir_la, int dir) {
    if (la_vccio_get_mv() == LA_VCCIO_UNSET) {
        log_printf("[scpi] DIG:STEP refused: la voltage not set (la_voltage first)\n");
        return -2;
    }
    char err[LA_PINS_ERR_MAX];
    int rc = la_step_begin(la, steps, delay_us, dir_la, dir, err, sizeof(err));
    if (rc == 0) return 0;
    log_printf("[scpi] DIG:STEP refused: %s\n", err);
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
void handle_la_pins(int conn_id, const char *json) {
    (void)json;
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

/* ---- `la`, `la_voltage` and the emulated I2C sensor ---------------------- */

/* Unified logic-analyzer-pin command. The action is inferred from the fields:
 *
 *   {"cmd":"la","la":N,"steps":S,"delay_us":D[,"dir_la":M,"direction":0|1]}
 *        → run a step pulse train on LA N (1..14); the FPGA runs it autonomously.
 *   {"cmd":"la","la":N,"pullup":"on"|"off"}  → switch LA N's pull-up (LA1..8).
 *   {"cmd":"la","la":N}                       → report LA N's pull-up state.
 *   {"cmd":"la"}                              → bitmask of enabled LA pull-ups.
 *
 * This replaces the old gpio_set/gpio_step/pullup/pullup_status commands. Active
 * drive (the former gpio_set) is gone: a pull-up — or its absence, which leaves
 * the board's pull-down — sets a line's idle level instead. */
void handle_la(int conn_id, const char *json) {
    if (!require_la_voltage(conn_id)) return;
    char steps_s[12] = {0};

    /* --- step pulse train: distinguished by the "steps" field --- */
    if (json_get_value(json, "steps", steps_s, sizeof(steps_s))) {
        char la_s[8] = {0}, delay_s[12] = {0}, dir_la_s[8] = {0}, dir_s[8] = {0};
        if (!json_get_value(json, "la", la_s, sizeof(la_s))) {
            send_error(conn_id, "missing la");
            return;
        }
        if (!json_get_value(json, "delay_us", delay_s, sizeof(delay_s))) {
            send_error(conn_id, "missing delay_us");
            return;
        }
        unsigned la = (unsigned)atoi(la_s);
        uint32_t steps = 0, delay_us = 0;
        if (!la_step_parse_u32(steps_s, &steps))    { send_error(conn_id, "invalid steps");    return; }
        if (!la_step_parse_u32(delay_s, &delay_us)) { send_error(conn_id, "invalid delay_us"); return; }

        /* Optional direction channel — driven before stepping (stepper dir). */
        unsigned dir_la = 0;
        bool     dir    = false;
        if (json_get_value(json, "dir_la", dir_la_s, sizeof(dir_la_s))) {
            dir_la = (unsigned)atoi(dir_la_s);
            if (json_get_value(json, "direction", dir_s, sizeof(dir_s)))
                dir = (atoi(dir_s) != 0);
        }

        /* Ownership: a free step/dir pin is CLAIMED for the train (and released when the
           fabric reports STEP_BUSY low again, from command_handler_poll); a gpio output is
           pulsed in place; anything else is a `pin conflict:`.  la_step_begin drives the
           direction pin and starts the train. */
        char err[LA_PINS_ERR_MAX];
        int rc = la_step_begin(la, steps, delay_us, dir_la, dir, err, sizeof(err));
        if (rc != 0) { send_error(conn_id, err); return; }

        /* Non-blocking: the FPGA runs the train autonomously, so we report that
           it has started rather than waiting for completion. */
        char payload[80];
        snprintf(payload, sizeof(payload),
                 "{\"la\":%u,\"steps\":%lu,\"delay_us\":%lu,\"status\":\"started\"}",
                 la, (unsigned long)steps, (unsigned long)delay_us);
        send_ok_str(conn_id, payload);
        return;
    }

    /* --- no "la" field: report the pull-up bitmask, bit (la-1)=LA<la> --- */
    char la_s[8] = {0};
    if (!json_get_value(json, "la", la_s, sizeof(la_s))) {
        unsigned mask = 0;
        for (unsigned la = 1; la <= 8; la++) {
            if (pca9555_la_pullup_enabled((uint8_t)la)) mask |= (1u << (la - 1));
        }
        /* pullups_available says whether a pull-up CAN be engaged right now (bank at
           3.3 V); the mask is always the truth about what is engaged. */
        char payload[64];
        snprintf(payload, sizeof(payload), "{\"la_pullup_mask\":%u,\"pullups_available\":%d}",
                 mask, la_pullups_available() ? 1 : 0);
        send_ok_str(conn_id, payload);
        return;
    }

    /* --- pull-up set ("pullup":"on|off") or query for one pin (LA1..8) --- */
    unsigned la = (unsigned)atoi(la_s);
    if (la < 1 || la > 8) {
        send_error(conn_id, "no pull-up on this la");   /* LA9-14 / out of range */
        return;
    }

    char state_s[8] = {0};
    if (json_get_value(json, "pullup", state_s, sizeof(state_s))) {
        char c = state_s[0];
        bool on = (c == 'o' || c == 'O') ? (state_s[1] == 'n' || state_s[1] == 'N')
                                         : (atoi(state_s) != 0);
        /* The other direction of the pull-compatibility rule: LA7/LA8's resistor pulls DOWN,
           which an open-drain bus / an idle-high UART / SWDIO cannot live with.  Refuse before
           touching the expander so the wire never briefly contradicts the function on it. */
        char pull_err[LA_PINS_ERR_MAX];
        if (on && !la_pins_check_pull_enable(la, pull_err, sizeof(pull_err))) {
            send_error(conn_id, pull_err);
            return;
        }
        int rc = pca9555_set_la_pullup((uint8_t)la, on);
        if (rc == LA_PULLUP_ERR_VOLTAGE) {
            /* Not a failure to talk to the expander: the resistors are 3V3-referenced
               and the bank is at 1.8 V, so engaging one would drive the DUT above its
               own rail. Say which, so the host can switch the bank instead of retrying. */
            send_error(conn_id, "pull-ups are 3V3-referenced; not available with the LA bank at 1.8 V");
            return;
        }
        if (rc != 0) {
            send_error(conn_id, "pca9555 write failed");
            return;
        }
    }

    /* "pull" says which way the channel's fixed resistor goes: LA1-LA6 up,
       LA7/LA8 down. Without it "ohms" is ambiguous, and a client that assumed
       every biased channel pulls up would drive an open-drain bus the wrong way. */
    char payload[128];
    snprintf(payload, sizeof(payload),
             "{\"la\":%u,\"pullup\":%d,\"ohms\":\"%s\",\"pull\":\"%s\",\"pullups_available\":%d}",
             la, pca9555_la_pullup_enabled((uint8_t)la) ? 1 : 0,
             pca9555_la_pullup_ohms((uint8_t)la),
             pca9555_la_pull_is_down((uint8_t)la) ? "down" : "up",
             la_pullups_available() ? 1 : 0);
    send_ok_str(conn_id, payload);
}

/* Set (or query) the LA I/O-bank voltage via the TPS2116 mux.
     {"cmd":"la_voltage","mv":1800|3300}  -> switch the bank (required before any
                                             LA op) and report the new state.
     {"cmd":"la_voltage"}                 -> report the current mv + status pin. */
void handle_la_voltage(int conn_id, const char *json) {
    char mv_s[8] = {0};
    if (json_get_value(json, "mv", mv_s, sizeof(mv_s))) {
        /* Switching the bank under a running UART proxy / SWD session / sensor emulation
           glitches every LA line and strands the 3V3-referenced pulls; refuse and name the
           pins instead.  Re-setting the CURRENT voltage stays a no-op and is allowed. */
        char why[LA_PINS_ERR_MAX];
        if (!la_pins_check_voltage_change(la_vccio_get_mv(), atoi(mv_s), why, sizeof(why))) {
            send_error(conn_id, why);
            return;
        }
        int rc = la_vccio_set_mv(atoi(mv_s));
        if (rc == -2) {
            send_error(conn_id, "1.8 V needs a v3 pod; this board is v2 "
                                "(its TPS2116 has no 1.8 V setting)");
            return;
        }
        if (rc != 0) {
            send_error(conn_id, "la voltage must be 1800 or 3300 (mv)");
            return;
        }
    }
    char payload[80];
    snprintf(payload, sizeof(payload),
             "{\"mv\":%d,\"st\":%d,\"readback_mv\":%d}",
             la_vccio_get_mv(), la_vccio_status_pin(), la_vccio_readback_mv());
    send_ok_str(conn_id, payload);
}

/* ---- Emulated I2C sensor ----
 * Mock an I2C sensor on two LA channels, driven by the FPGA's generic target.
 *
 *   {"cmd":"sensor_start","type":"bmp280","addr":"0x76","sda":1,"scl":2}
 *   {"cmd":"sensor_set","temperature_c":25.0,"pressure_pa":101325}   (the model's keys)
 *   {"cmd":"sensor_types"}                                       → models + their keys
 *   {"cmd":"sensor_stop"}
 *   {"cmd":"sensor_status"}
 *   {"cmd":"sensor_regs","start":"0xF7","len":6}     → register bytes
 *   {"cmd":"sensor_la","samples":1024,"sample_rate_mhz":2.0} → raw bus capture
 */
void handle_sensor_start(int conn_id, const char *json) {
    if (!require_la_voltage(conn_id)) return;
    char type[16] = {0}, addr_s[8] = {0}, sda_s[8] = {0}, scl_s[8] = {0};

    if (!json_get_value(json, "type", type, sizeof(type))) {
        send_error(conn_id, "missing type");
        return;
    }
    if (!json_get_value(json, "sda", sda_s, sizeof(sda_s)) ||
        !json_get_value(json, "scl", scl_s, sizeof(scl_s))) {
        send_error(conn_id, "missing sda/scl");
        return;
    }
    json_get_value(json, "addr", addr_s, sizeof(addr_s));   /* optional */

    uint8_t  addr7 = addr_s[0] ? (uint8_t)strtol(addr_s, NULL, 0) : 0;
    unsigned sda   = (unsigned)atoi(sda_s);
    unsigned scl   = (unsigned)atoi(scl_s);

    /* A replacing sensor_start may take over the running sensor's own pins, so its two
       functions count as free here — but any OTHER owner is a conflict, checked before
       sensor_sim_start touches the FPGA so a refused start leaves the old sensor running. */
    uint16_t i2c_fns = LA_FN_BIT(LA_FN_I2C_SDA) | LA_FN_BIT(LA_FN_I2C_SCL);
    uint8_t  sda_pin = (uint8_t)sda, scl_pin = (uint8_t)scl;
    if (!la_claim_or_error(conn_id, LA_FN_I2C_SDA, &sda_pin, 1, i2c_fns)) return;
    if (!la_claim_or_error(conn_id, LA_FN_I2C_SCL, &scl_pin, 1, i2c_fns)) return;

    int rc = sensor_sim_start(type, addr7, sda, scl);
    if (rc == -1) { send_error(conn_id, "unknown sensor type"); return; }
    if (rc != 0)  { send_error(conn_id, "sensor start failed (bad channel?)"); return; }
    la_pins_release_fn(LA_FN_I2C_SDA);   /* the replaced sensor's pins, if any */
    la_pins_release_fn(LA_FN_I2C_SCL);
    la_pins_claim(LA_FN_I2C_SDA, LA_GPIO_NONE, 0, &sda_pin, 1);
    la_pins_claim(LA_FN_I2C_SCL, LA_GPIO_NONE, 0, &scl_pin, 1);

    char payload[80];
    snprintf(payload, sizeof(payload),
             "{\"type\":\"%s\",\"addr\":%u,\"sda\":%u,\"scl\":%u}",
             sensor_sim_type(), sensor_sim_addr7(), sda, scl);
    send_ok_str(conn_id, payload);
}

/* A float as JSON with up to three decimals (newlib-nano printf has no %f). */
static void emit_milli(bp_emit_t *e, float v) {
    long m = lroundf(v * 1000.0f);
    unsigned long a = (unsigned long)(m < 0 ? -m : m);
    if (a % 1000u == 0) bp_emit(e, "%s%lu", m < 0 ? "-" : "", a / 1000u);
    else {
        char frac[4];
        snprintf(frac, sizeof(frac), "%03lu", a % 1000u);
        for (int i = 2; i > 0 && frac[i] == '0'; i--) frac[i] = '\0';
        bp_emit(e, "%s%lu.%s", m < 0 ? "-" : "", a / 1000u, frac);
    }
}

/* {"key":value,...} of the active model's parameters. */
static void emit_sensor_values(bp_emit_t *e) {
    const sensor_model_t *m = sensor_sim_model();
    bp_emit_raw(e, "{");
    for (unsigned i = 0; m && i < m->n_params; i++) {
        bp_emit(e, "%s\"%s\":", i ? "," : "", m->params[i].key);
        emit_milli(e, sensor_sim_value(i));
    }
    bp_emit_raw(e, "}");
}

/* {"cmd":"sensor_set","temperature_c":25.0,...}: any of the active model's parameter keys
   (sensor_types lists them).  Every key is checked before anything is applied, so a rejected
   value leaves the sensor as it was; the accepted ones reload the image once. */
void handle_sensor_set(int conn_id, const char *json) {
    const sensor_model_t *m = sensor_sim_model();
    if (!m) { send_error(conn_id, "no sensor active"); return; }

    float    v[SENSOR_MAX_PARAMS];
    uint16_t have = 0;
    char     err[96];
    for (unsigned i = 0; i < m->n_params; i++) {
        const sensor_param_t *p = &m->params[i];
        char val[24] = {0};
        if (!json_get_value(json, p->key, val, sizeof(val))) continue;
        char *end = NULL;
        v[i] = strtof(val, &end);
        if (end == val || !isfinite(v[i]) || v[i] < p->min || v[i] > p->max) {
            long lo = lroundf(p->min), hi = lroundf(p->max);
            snprintf(err, sizeof(err), "%s must be a number from %ld to %ld (%s)", p->key, lo, hi, p->unit);
            send_error(conn_id, err);
            return;
        }
        have |= (uint16_t)(1u << i);
    }
    if (!have) {
        snprintf(err, sizeof(err), "no %s parameter given (sensor_types lists them)", m->name);
        send_error(conn_id, err);
        return;
    }
    for (unsigned i = 0; i < m->n_params; i++)
        if (have & (1u << i)) (void)sensor_sim_set_value(m->params[i].key, v[i]);
    if (sensor_sim_apply() != 0) { send_error(conn_id, "sensor image reload failed"); return; }

    char resp[384];
    bp_emit_t e;
    bp_emit_init(&e, resp, sizeof(resp));
    bp_emit(&e, "{\"status\":\"ok\",\"data\":{\"type\":\"%s\",\"values\":", m->name);
    emit_sensor_values(&e);
    bp_emit_raw(&e, "}}\n");
    send_emitted(conn_id, &e);
}

/* {"cmd":"sensor_types"}: the models this firmware emulates, with their addresses and the
   parameters sensor_set takes (key, unit, range, the value at sensor_start). */
void handle_sensor_types(int conn_id, const char *json) {
    (void)json;
    char resp[BP_CLOUD_REPLY_MAX];
    bp_emit_t e;
    bp_emit_init(&e, resp, sizeof(resp));
    size_t n = 0;
    const sensor_model_t *const *models = sensor_sim_models(&n);
    bp_emit_raw(&e, "{\"status\":\"ok\",\"data\":{\"types\":[");
    for (size_t k = 0; k < n; k++) {
        const sensor_model_t *m = models[k];
        bp_emit(&e, "%s{\"type\":\"%s\",\"label\":", k ? "," : "", m->name);
        bp_emit_jstr(&e, m->label);
        bp_emit(&e, ",\"addr\":%u,\"alt_addr\":%u,\"params\":[",
                (unsigned)m->default_addr7, (unsigned)m->alt_addr7);
        for (unsigned i = 0; i < m->n_params; i++) {
            const sensor_param_t *p = &m->params[i];
            bp_emit(&e, "%s{\"key\":\"%s\",\"unit\":\"%s\",\"min\":", i ? "," : "", p->key, p->unit);
            emit_milli(&e, p->min);
            bp_emit_raw(&e, ",\"max\":");
            emit_milli(&e, p->max);
            bp_emit_raw(&e, ",\"default\":");
            emit_milli(&e, p->def);
            bp_emit_raw(&e, "}");
        }
        bp_emit_raw(&e, "]}");
    }
    bp_emit_raw(&e, "]}}\n");
    send_emitted(conn_id, &e);
}

void handle_sensor_stop(int conn_id, const char *json) {
    (void)json;
    sensor_sim_stop();
    la_pins_release_fn(LA_FN_I2C_SDA);
    la_pins_release_fn(LA_FN_I2C_SCL);
    send_ok_str(conn_id, "null");
}

void handle_sensor_status(int conn_id, const char *json) {
    (void)json;
    char resp[512];
    bp_emit_t e;
    bp_emit_init(&e, resp, sizeof(resp));
    if (!sensor_sim_active()) {
        bp_emit_raw(&e, "{\"status\":\"ok\",\"data\":{\"active\":false}}\n");
    } else {
        i2c_sensor_status_t st = {0};
        sensor_sim_get_status(&st);
        bp_emit(&e, "{\"status\":\"ok\",\"data\":{"
                    "\"active\":true,\"type\":\"%s\",\"addr\":%u,\"sda\":%u,\"scl\":%u,"
                    "\"transactions\":%u,\"writes\":%u,"
                    "\"last_reg\":%u,\"last_val\":%u,\"values\":",
                sensor_sim_type(), sensor_sim_addr7(), sensor_sim_sda(), sensor_sim_scl(),
                st.xfer_count, st.wr_count, st.last_wr_addr, st.last_wr_val);
        emit_sensor_values(&e);
        bp_emit_raw(&e, "}}\n");
    }
    send_emitted(conn_id, &e);
}

/* command_handler_poll's sensor pass: watch a configurable model's register writes, re-arm a
   sensor after a gateware reconfiguration (releasing its pins if that fails). */
void sensor_poll(void) {
    if (sensor_sim_poll() != 0) {
        la_pins_release_fn(LA_FN_I2C_SDA);
        la_pins_release_fn(LA_FN_I2C_SCL);
    }
}
