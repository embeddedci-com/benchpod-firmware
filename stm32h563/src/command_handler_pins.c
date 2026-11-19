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
#include "gw_caps.h"
#include "la_pins.h"
#include "signal_engine.h"
#include "bp_errors.h"
#include "at_driver.h"
#include "bp_json.h"
#include "bp_limits.h"
#include "pico/time.h"
#include "bp_log.h"
#include "target_power.h"
#include "board_caps.h"
#include "vref_out.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* JSON parsing is shared (see bp_json.h); same aliases as command_handler.c. */
#define json_get_value      bp_json_get

/* ---- glue ---------------------------------------------------------------- */

/* The pull masks (la_pull_mask_now, la_pull_mask_live) are in la_pull.c. */

void la_pins_drive_mask(uint16_t mask) {
    for (unsigned la = 1; la <= LA_PINS_COUNT; la++)
        if (mask & (1u << (la - 1u))) (void)fpga_la_set(la, la_pins_drive(la));
}

bool la_claim_or_error(int conn_id, la_fn_t fn, const uint8_t *las, size_t n, uint32_t free_fns) {
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
    if (!la_voltage_from_profile()) {
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
    if (!la_voltage_from_profile()) {
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

/* The pod's target power, for the listings' LA_WARN_UNPOWERED (RAM reads only). */
static void note_target_power(void) {
    bool a = false, b = false;
    (void)target_power_get_cached(1, &a, NULL);
    (void)target_power_get_cached(2, &b, NULL);
    la_pins_set_target_powered(a || b);
}

/* {"cmd":"la_pins"} -> {"pins":[14 entries],"levels":M|null}
   {"cmd":"la_pins","la":N|[N,...]} -> the same, with only those pins' entries (a listing too large
   for the cloud reply is read in parts; without "la" the reply is as it always was). */
void handle_la_pins(int conn_id, const char *json) {
    uint16_t only = 0x3FFFu;
    char qerr[LA_PINS_ERR_MAX];
    if (!la_pins_query_mask(json, &only, qerr, sizeof(qerr))) { send_error(conn_id, qerr); return; }
    note_target_power();
    uint16_t pulls  = la_pull_mask_live();
    uint16_t levels = 0;
    bool     have   = gw_caps()->gpio_get && fpga_gpio_get(&levels) == 0;

    char resp[BP_CLOUD_REPLY_MAX];   /* worst cases: test_la_pins.c, budget_la_pins */
    bp_emit_t e;
    bp_emit_init(&e, resp, sizeof(resp));
    la_pins_emit_reply(&e, only, pulls, have, levels);
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
    note_target_power();

    if (!req.has_mode && !req.has_level) {
        uint8_t ver = signal_engine_fpga_version();
        if (!gw_caps()->gpio_get) {
            snprintf(err, sizeof(err), "reading pin levels needs gateware v%u or newer (this pod runs v%u)",
                     (unsigned)GPIO_GET_MIN_GW, (unsigned)ver);
            send_error(conn_id, err);
            return;
        }
        uint16_t levels = 0;
        if (fpga_gpio_get(&levels) != 0) { send_error(conn_id, "pin level read failed (SPI)"); return; }
        bp_emit(&e, "{\"status\":\"ok\",\"data\":{\"levels\":%u,\"pins\":", (unsigned)levels);
        la_pins_emit_list(&e, 0x3FFFu, la_pull_mask_live());
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
    la_pins_emit_list(&e, affected, la_pull_mask_live());
    bp_emit_raw(&e, "}}\n");
    send_emitted(conn_id, &e);
}

/* {"cmd":"la_dir","la":N|[N,...]|"all","dir":"dut_input"|"dut_output"|"bidir"|"unknown"}
   {"cmd":"la_dir"}  (list)
   Which way each pin's DUT signal goes, from the host's wiring profile (RAM only).  The pod
   refuses a push-pull driver on a DUT output (la_pins.h). */
void handle_la_dir(int conn_id, const char *json) {
    char err[LA_PINS_ERR_MAX];
    la_gpio_req_t req;
    if (!la_gpio_req_parse(json, &req, err, sizeof(err))) { send_error(conn_id, err); return; }
    if (req.has_mode || req.has_level) { send_error(conn_id, "la_dir takes la and dir only"); return; }
    char resp[BP_CLOUD_REPLY_MAX];
    bp_emit_t e;
    bp_emit_init(&e, resp, sizeof(resp));
    note_target_power();
    uint16_t affected = 0x3FFFu;
    if (req.all || req.n > 0) {
        char ds[16];
        la_dir_t d;
        if (!bp_json_get(json, "dir", ds, sizeof(ds)) || !la_dir_parse(ds, &d)) {
            send_error(conn_id, "dir must be dut_input, dut_output, bidir or unknown"); return;
        }
        uint8_t las[LA_PINS_COUNT];
        size_t n = 0;
        if (req.all) for (unsigned la = 1; la <= LA_PINS_COUNT; la++) las[n++] = (uint8_t)la;
        else { memcpy(las, req.las, req.n); n = req.n; }
        for (size_t i = 0; i < n; i++)
            if (!la_pins_check_dir(las[i], d, err, sizeof(err))) { send_error(conn_id, err); return; }
        affected = 0;
        for (size_t i = 0; i < n; i++) { la_pins_set_dir(las[i], d); affected |= (uint16_t)(1u << (las[i] - 1u)); }
    }
    bp_emit_raw(&e, "{\"status\":\"ok\",\"data\":{\"pins\":");
    la_pins_emit_list(&e, affected, la_pull_mask_live());
    bp_emit_raw(&e, "}}\n");
    send_emitted(conn_id, &e);
}

/* ---- `la`, `la_voltage` and the emulated I2C sensor ---------------------- */

/* {"cmd":"la","la":N,"steps":S,"delay_us":D[,"dir_la":M,"direction":0|1]}
   A step pulse train on LA N (1..14); the FPGA runs it autonomously. */
static void handle_la_step(int conn_id, const char *json, const char *steps_s) {
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

    /* Optional direction channel, driven before stepping (stepper dir). */
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
}

/* {"cmd":"la"} -> the pull bitmask, bit (la-1) = LA<la>.
   pullups_available says whether a pull CAN be engaged right now (a bank voltage is set; on v2
   boards it must be 3.3 V); la_pullup_mask is always the truth about what is on the wire.  While
   the pod holds pulls off (digital v3r4): why, and the mask the host asked for (put back by the
   pod).  Never on other boards, where the two masks are the same. */
static void handle_la_pull_mask(int conn_id) {
    unsigned mask = la_pull_mask_live(), asked = la_pull_mask_now();
    const char *held = la_pull_held();
    char payload[112];
    if (held)
        snprintf(payload, sizeof(payload),
                 "{\"la_pullup_mask\":%u,\"pullups_available\":%d,\"held\":\"%s\",\"asked_mask\":%u}",
                 mask, la_pull_available() ? 1 : 0, held, asked);
    else
        snprintf(payload, sizeof(payload), "{\"la_pullup_mask\":%u,\"pullups_available\":%d}",
                 mask, la_pull_available() ? 1 : 0);
    send_ok_str(conn_id, payload);
}

/* {"cmd":"la","la":N} -> LA N's pull state (also the reply of a pull set).
   "state" is what the channel does now (up / down / off, from on) and "dirs" every way it can
   pull. Digital v3r4 may hold a pull the host asked for off (pin 21 down, pull-ups suspended);
   "held" then says why, and strength/ohms/pull stay the setting the pod puts back. Elsewhere live
   and asked are the same. The older fields stay for older hosts: "pullup" 1 when engaged, "pull"
   the channel's default direction ("none" without a resistor), "ohms" its value. */
static void handle_la_pull_query(int conn_id, unsigned la) {
    la_pull_caps_t caps = la_pull_caps(la);
    bool asked_on = la_pull_asked(la) != LA_PULL_NONE;
    bool on = la_pull_live(la);
    la_pull_dir_t dflt = la_pull_dir(la);
    bp_emit_t e;
    char payload[200];
    bp_emit_init(&e, payload, sizeof(payload));
    bp_emit(&e, "{\"la\":%u,\"pullup\":%d,\"ohms\":\"%s\",\"pull\":\"%s\",\"state\":\"%s\",\"dirs\":",
            la, on ? 1 : 0, la_pull_ohms(la) ? la_pull_ohms(la) : "",
            dflt == LA_PULL_NONE ? "none" : la_pull_dir_name(dflt),
            la_pull_dir_name(on ? dflt : LA_PULL_NONE));
    la_pull_emit_dirs(&e, la);
    /* v3r4: the strength engaged ("none" when off) and every strength the channel has. */
    if (caps.strengths) {
        bp_emit(&e, ",\"strength\":\"%s\",\"strengths\":[", pull_str_name(la_pull_asked_str(la)));
        const char *sep = "";
        for (int st = PULL_STR_10K; st <= PULL_STR_BOTH; st++)
            if (caps.strengths & LA_PULL_STR_BIT(st)) { bp_emit(&e, "%s\"%s\"", sep, pull_str_name((pull_str_t)st)); sep = ","; }
        bp_emit_raw(&e, "]");
    }
    if (asked_on && !on) {   /* digital v3r4 only: elsewhere asked and live are the same */
        const char *held = la_pull_held();
        bp_emit(&e, ",\"held\":\"%s\"", held ? held : "fault");
    }
    bp_emit(&e, ",\"pullups_available\":%d}", la_pull_available() ? 1 : 0);
    if (!bp_emit_ok(&e)) { send_error(conn_id, "reply too large"); return; }
    send_ok_str(conn_id, payload);
}

/* {"cmd":"la","la":N,"pull":"up"|"down"|"off"[,"strength":"10k"|"2.2k"|"both"]}
   {"cmd":"la","la":N,"pullup":"on"|"off"}   (the older form: on = the channel's direction)
   Switch LA N's pull resistor; a direction the channel's resistor can't pull is refused (BP0024),
   a channel without one too (BP0023).  false = refused (the error was sent). */
static bool handle_la_pull_set(int conn_id, const char *json, unsigned la,
                               const char *pull_s, const char *pullup_s) {
    la_pull_dir_t want = LA_PULL_NONE;
    switch (la_pull_request(la, pull_s, pullup_s, &want)) {
    case LA_PULL_REQ_OK: break;
    case LA_PULL_REQ_BAD_VALUE:
        send_error(conn_id, "pull must be up, down or off");
        return false;
    case LA_PULL_REQ_NO_PULL: {
        char m[64];
        snprintf(m, sizeof(m), "LA%u has no pull resistor on this board", la);
        bp_err_la_no_pull(conn_id, BP_ERR_TEXT(m), (int32_t)la);
        return false;
    }
    case LA_PULL_REQ_DIRECTION: {
        const char *have = la_pull_dir_name(la_pull_dir(la));
        char m[64];
        snprintf(m, sizeof(m), "LA%u supports pull-%s only on this board", la, have);
        bp_err_la_pull_direction(conn_id, BP_ERR_TEXT(m), (int32_t)la, la_pull_dir_name(want), have);
        return false;
    }
    }
    /* The other direction of the pull-compatibility rule: a pull-DOWN under an open-drain
       bus / an idle-high UART / SWDIO.  Refuse before touching the expander so the wire never
       briefly contradicts the function on it. */
    char pull_err[LA_PINS_ERR_MAX];
    if (!la_pins_check_pull_enable(la, want, pull_err, sizeof(pull_err))) {
        send_error(conn_id, pull_err);
        return false;
    }
    /* Digital v3r4: direction and strength per channel (without "strength" the pod picks from
       the pin's function).  A fixed resistor (the older boards) ignores "strength", like older
       firmware does. */
    la_pull_caps_t caps = la_pull_caps(la);
    pull_str_t str = PULL_STR_NONE;
    if (caps.strengths) {
        char str_s[8] = {0};
        if (json_get_value(json, "strength", str_s, sizeof(str_s)) &&
            (!pull_str_parse(str_s, &str) || !(caps.strengths & LA_PULL_STR_BIT(str)))) {
            send_error(conn_id, "strength must be 10k, 2.2k or both");
            return false;
        }
    }
    /* "off" on a channel without a resistor: nothing to switch. */
    int rc = caps.dirs ? la_pull_set(la, want, str) : 0;
    switch (rc) {
    case 0:
        return true;
    case LA_PULL_ERR_RAIL: {
        char m[96];
        snprintf(m, sizeof(m), "pin 21 is down (%d mV): the pull resistors are off until it is back",
                 vref_out_last_mv());
        bp_err_pull_rail_down(conn_id, BP_ERR_TEXT(m), (int32_t)vref_out_last_mv());
        return false;
    }
    case LA_PULL_ERR_VOLTAGE:
        bp_err_la_voltage_not_set(conn_id,
            BP_ERR_TEXT("la voltage not set; set la_mv (1800 or 3300) in the active wiring profile, or call la_voltage (mv 1800 or 3300) first"));
        return false;
    case LA_PULL_ERR_BANK:
        /* Not a failure to talk to the expander: a v2 board's pull-ups go to +3V3 and the bank
           is not at 3.3 V, so engaging one would drive the DUT above its own rail.  Say which,
           so the host can switch the bank instead of retrying.  (v3 pulls go to the bank rail
           and work at 1.8 V.) */
        send_error(conn_id, "this board's pull-ups go to 3.3 V; set the LA bank to 3.3 V first");
        return false;
    default:
        send_error(conn_id, "pull expander write failed");
        return false;
    }
}

/* The `la` verb: three commands, told apart by their fields.
 *   "steps"               a step train (handle_la_step)
 *   no "la"               the pull bitmask (handle_la_pull_mask)
 *   "la" with "pull" or "pullup"   switch the pull, then report it (handle_la_pull_set)
 *   "la" alone            report the pull (handle_la_pull_query)
 *
 * This replaces the old gpio_set/gpio_step/pullup/pullup_status commands. Active
 * drive (the former gpio_set) is gone: a pull-up (or its absence, which leaves
 * the board's pull-down) sets a line's idle level instead. */
void handle_la(int conn_id, const char *json) {
    if (!require_la_voltage(conn_id)) return;
    char steps_s[12] = {0};
    if (json_get_value(json, "steps", steps_s, sizeof(steps_s))) { handle_la_step(conn_id, json, steps_s); return; }

    char la_s[8] = {0};
    if (!json_get_value(json, "la", la_s, sizeof(la_s))) { handle_la_pull_mask(conn_id); return; }
    unsigned la = (unsigned)atoi(la_s);
    if (la < 1 || la > LA_PINS_COUNT) {
        char m[48];
        snprintf(m, sizeof(m), "LA%u is not a pin (use 1..%u)", la, (unsigned)LA_PINS_COUNT);
        send_error(conn_id, m);
        return;
    }
    char pull_s[8] = {0}, pullup_s[8] = {0};
    bool has_pull = json_get_value(json, "pull", pull_s, sizeof(pull_s));
    bool has_pullup = json_get_value(json, "pullup", pullup_s, sizeof(pullup_s));
    if ((has_pull || has_pullup) &&
        !handle_la_pull_set(conn_id, json, la, has_pull ? pull_s : NULL, has_pullup ? pullup_s : NULL))
        return;
    handle_la_pull_query(conn_id, la);
}

/* Set (or query) the LA I/O-bank voltage via the TPS2116 mux.
     {"cmd":"la_voltage","mv":1800|3300}  -> switch the bank (required before any
                                             LA op) and report the new state.
     {"cmd":"la_voltage"}                 -> report the current mv + status pin. */
void handle_la_voltage(int conn_id, const char *json) {
    char mv_s[8] = {0};
    if (json_get_value(json, "mv", mv_s, sizeof(mv_s))) {
        /* Switching the bank under a running UART proxy / SWD session / sensor emulation
           glitches every LA line; refuse and name the
           pins instead.  Re-setting the CURRENT voltage stays a no-op and is allowed. */
        char why[LA_PINS_ERR_MAX];
        if (!la_pins_check_voltage_change(la_io_get_mv(), atoi(mv_s), why, sizeof(why))) {
            send_error(conn_id, why);
            return;
        }
        int rc = la_vccio_set_mv(atoi(mv_s));
        if (rc == -3) {
            char m[96];
            snprintf(m, sizeof(m), "5 V did not come up on pin 21 (%d mV); check the USB supply",
                     vref_out_last_mv());
            bp_err_vref_5v_missing(conn_id, BP_ERR_TEXT(m), (int32_t)vref_out_last_mv());
            return;
        }
        if (rc == -2) {
            send_error(conn_id, "1.8 V needs a v3 pod; this board is v2 "
                                "(its TPS2116 has no 1.8 V setting)");
            return;
        }
        if (rc != 0) {
            send_error(conn_id, board_caps()->la_5v ? "la voltage must be 1800, 3300 or 5000 (mv)"
                                                    : "la voltage must be 1800 or 3300 (mv)");
            return;
        }
    }
    /* "mv" is the DUT's I/O voltage (5000 in 5 V mode).  Digital v3r4 adds pin 21 (vref_out_emit). */
    char payload[200];
    bp_emit_t e;
    bp_emit_init(&e, payload, sizeof(payload));
    bp_emit(&e, "{\"mv\":%d,\"st\":%d,\"readback_mv\":%d",
            la_io_get_mv(), la_vccio_status_pin(), la_vccio_readback_mv());
    if (vref_out_present()) {
        (void)vref_out_measure_mv();
        bp_emit_raw(&e, ",");
        vref_out_emit(&e);
    }
    bp_emit_raw(&e, "}");
    if (!bp_emit_ok(&e)) { send_error(conn_id, "reply too large"); return; }
    send_ok_str(conn_id, payload);
}
