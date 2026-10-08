#include "command_handler.h"
#include "command_handler_internal.h"   /* reply helpers + gate query shared with the CAN/OTA subsystem files */
#include "console.h"
#include "at_driver.h"
#include "signal_engine.h"
#include "stm32h5xx_hal.h"   /* NVIC_SystemReset for the psram_recover command */
#include "psram_alloc.h"
#include "psram.h"          /* deep DAC replay: stage the waveform into PSRAM */
#include "fpga_config.h"    /* FPGA_PSRAM_* region map, FPGA_DAC_BASE_FOR, caps */
#include "sensor_sim.h"
#include "wifi_manager.h"
#include "target_power.h"
#include "scpi_server.h"
#include "device_identity.h"
#include "i2c_bus.h"
#include "can_bus.h"
#include "cal_data.h"
#include "current_out.h"   /* 4-20 mA output (J9): uA <-> DAC code */
#include "adc_scale.h"     /* circular-mean + unwrap for adc_read's sample burst */
#include "adc_cal.h"       /* per-pod ADC calibration on top of cal_data.h */
#include "pico_compat.h"   /* sleep_ms (yields to FreeRTOS) */
#include "ina238.h"
#include "board_variant.h"
#include "b64url.h"
#include "cloud_config.h"
#include "cloud_client.h"
#include "net_server.h"   /* net_eth_stop/start/restart for the `eth` command */
#include "config_store.h"
#include "esp_wifi_ctrl.h"
#include "esp_hosted_spi.h"
#include "dap.h"
#include "swd_ll.h"
#include "watchdog.h"
#include "hw_worker.h"   /* hw_worker_submit_tunnel_reset (DAP send stall) */
#include "board_info.h"
#include "board_rev.h"
#include "usb_cc.h"
#include "nrst_ctrl.h"
#include "bp_json.h"     /* shared flat-JSON parser + bounds-tracked emitter */
#include "bp_limits.h"   /* coupled cloud/command buffer sizes */
#include "cloud_reply_cap.h"   /* the captured reply of a cloud command.request */
#include "dac_loop_params.h"  /* closed-loop DAC: curve upsample + parameter validation (host-tested) */
#include "fault.h"       /* reset cause + last-crash summary for status */
#include "psram_regions.h"  /* capture_read: is the capture still in PSRAM */
#include "boot_guard.h"  /* safe mode: status fields + the iCE40/PSRAM-off command gate */
#include "cmd_gate.h"    /* the checks before a command reaches its handler (host-tested) */
#include "sys_health.h"  /* heap/stack headroom for status */
#include "bp_err.h"      /* shared error vocabulary (bp_err_str) */
#include "ice40_flash.h"
#include "version.h"     /* FIRMWARE_VERSION (single source) */
#include "ota.h"         /* firmware OTA (PSRAM-staged) */
#include "fw_sign.h"
#include "adc_pool.h"      /* the RAM sample buffer, shared with SCPI and the console */
#include "cloud_caps.h"     /* the capabilities, shared with the cloud announcement */
#include "cmd_table.h"     /* the command table: tier, gate flags and handler per verb */
#include "pod_policy.h"
#include "lease_gate.h"
#include "hw_lock.h"     /* serialize the shared I2C bus (power_status vs the profile sampler) */
#include "la_pins.h"     /* LA pin ownership table + capture-trigger parsing/messages */
#include "power_profile.h"  /* INA238 rail profile sampler (poll + status) */

#include <string.h>
#include <stdlib.h>
#include "dac_limits.h"
#include <stdio.h>
#include <math.h>

#include "pico/time.h"   /* absolute_time_t, get_absolute_time, *_diff_us (UART escape guard timing) */
#include "flash_layout.h"
#include "FreeRTOS.h"   /* pvPortMalloc: per-call curve buffers */
#include "lwip/stats.h"    /* lwIP memory high-water marks in status */
#include "lwip/memp.h"

/* JSON parsing/emitting is shared (see bp_json.h).  These thin aliases keep the
   handler bodies below reading the way they did before the parser was unified. */
#define json_get_value      bp_json_get
#define json_flag           bp_json_flag
#define json_get_byte_array bp_json_byte_array

/* ---- Response helpers ---- */

/* send_error / send_ok_str / heavy_in_flight have external linkage (declared in
   command_handler_internal.h) so the extracted CAN/OTA subsystem files can use them. */

void send_ok_str(int conn_id, const char *payload_json) {
    /* 256, and it has now grown TWICE — 128 -> 192 when the control-loop arm reply started
       echoing the input source, 192 -> 256 when v30 added the derived input map.  Worst case
       MEASURED, not estimated: the full arm payload is 192 bytes and the
       {"status":"ok","data":…} wrapper takes it to 216.  It is worth sizing carefully because
       of the failure mode — "reply too large" on a command that WORKED: the loop is armed and
       driving, and the caller is told it failed.  Check any new reply field against this. */
    char resp[256];
    bp_emit_t e;
    bp_emit_init(&e, resp, sizeof(resp));
    bp_emit_raw(&e, "{\"status\":\"ok\",\"data\":");
    bp_emit_raw(&e, payload_json);   /* caller-built, already-valid JSON value */
    bp_emit_raw(&e, "}\n");
    /* Skip tracing the keepalive pong — the web UI pings on a timer, which would
       otherwise flood the console with identical ok lines (the "<-" side is
       already suppressed via cmd_is_noisy_poll). */
    if (strcmp(payload_json, "\"pong\"") != 0)
        printf("[cmd] -> ok  data=%s\n", payload_json);
    if (!bp_emit_ok(&e)) {
        /* Payload didn't fit the fixed reply buffer — send a structured error
           rather than a truncated (malformed) ok.  Handlers whose payload can be
           large build+send their reply directly instead of via send_ok_str. */
        send_error(conn_id, bp_err_str(BP_ERR_TOO_LARGE));
        return;
    }
    if (at_send_data(conn_id, (const uint8_t *)resp, bp_emit_len(&e)) != 0) {
        at_close_connection(conn_id);   /* reclaim slot if client already left */
    }
}

/* Big enough for the longest message a caller can build (LA_PINS_ERR_MAX) even if bp_emit_jstr
   had to escape every single byte into two.  The pin/pull-conflict messages carry a JSON snippet
   of the fix, so they really do escape a run of quotes: at 160 bytes they overflowed, and because
   a full emitter drops every later write the closing "}\n" went with them — leaving the client
   waiting for a newline that never came, until its socket timed out. */
#define CH_ERR_RESP_MAX  (2u * LA_PINS_ERR_MAX + 48u)
_Static_assert(CH_ERR_RESP_MAX > LA_PINS_ERR_MAX, "error reply must outgrow its longest message");

void send_error(int conn_id, const char *message) {
    char resp[CH_ERR_RESP_MAX];
    bp_emit_t e;
    bp_emit_init(&e, resp, sizeof(resp));
    bp_emit_raw(&e, "{\"status\":\"error\",\"message\":");
    bp_emit_jstr(&e, message);        /* escaped — a message with quotes stays valid JSON */
    bp_emit_raw(&e, "}\n");
    if (!bp_emit_ok(&e)) {
        /* Never send a torn line — the client blocks on the missing newline.  The full text is
           still in the device log above. */
        bp_emit_init(&e, resp, sizeof(resp));
        bp_emit_raw(&e, "{\"status\":\"error\",\"message\":\"error message too long\"}\n");
    }
    printf("[cmd] -> error: %s\n", message);
    if (at_send_data(conn_id, (const uint8_t *)resp, bp_emit_len(&e)) != 0) {
        at_close_connection(conn_id);
    }
}

/* Every LA-bank operation (la / la_capture / dap_start / uart_proxy / i2c-sensor)
   needs the LA I/O voltage chosen first — the TPS2116 mux is left "unset" at boot
   so the host must explicitly pick 1.8 V or 3.3 V for the DUT.  Returns true when
   a voltage is set; otherwise emits an error and returns false. */
bool require_la_voltage(int conn_id) {
    if (la_vccio_get_mv() == LA_VCCIO_UNSET) {
        send_error(conn_id, "la voltage not set; set it with la_voltage (mv 1800 or 3300) first");
        return false;
    }
    return true;
}

/* ---- Cloud command capture (CH_CLOUD_CONN) ----
   A command.request arriving over the cloud WebSocket is dispatched through the
   normal handlers with conn_id = CH_CLOUD_CONN; at_send_data() routes the reply
   into cloud_reply_cap instead of the TCP server so the cloud client can wrap it in
   a command.response frame (a reply that does not fit becomes a "too large" error). */
void command_handler_cloud_capture_append(const uint8_t *buf, size_t len) {
    cloud_reply_cap_append(buf, len);
}

/* Deferred reboot for the `psram_recover` command: send the ack first, then reset a beat
   later (command_handler_poll) so the reply flushes to the client before the pod reboots. */
static bool             s_recover_pending;
static absolute_time_t  s_recover_at;

/* ---- Heavy-op gate ----
   capture/stream/measure/test all share the single adc_cmd_buf and
   chunk_buf, so only one may run at a time.  A second client requesting a
   heavy op while one is in flight is rejected with "busy" rather than
   silently corrupting the in-flight transfer.  Light commands (ping,
   status, generate, gpio_*) use only local buffers and are not gated.

   ONE owner enforces the invariant: heavy_in_flight() is the single source of
   truth for "a capture/measure/LA/bulk-send is running", and heavy_begin() is
   the single entry point every heavy handler goes through (reject-if-busy then
   claim-the-gate).  This replaced the duplicated
   `if (v2cap.active || bulk.active) busy; if (!heavy_try_claim) busy;` pair that
   used to sit — subtly inconsistently — at the top of ~10 handlers. */
static int heavy_owner = -1;   /* conn_id holding adc_cmd_buf, or -1 */

bool heavy_try_claim(int conn_id) {
    if (heavy_owner != -1 && heavy_owner != conn_id) return false;
    heavy_owner = conn_id;
    return true;
}

void heavy_release(int conn_id) {
    if (heavy_owner == conn_id) heavy_owner = -1;
}

int heavy_owner_conn(void) { return heavy_owner; }

/* The single gate for a heavy handler: reject with "busy" if a heavy op is in
   flight or another connection owns the gate, otherwise claim it for conn_id and
   return true.  On false it has already sent the error. */
bool heavy_begin(int conn_id) {
    if (heavy_in_flight() || !heavy_try_claim(conn_id)) {
        send_error(conn_id, bp_err_str(BP_ERR_BUSY));
        return false;
    }
    return true;
}

/* Exposed to the SCPI handler so its blocking captures share the same single-
   ADC mutual exclusion as the JSON capture/stream/measure/test commands.  Same test as
   heavy_begin: OTA staging never claims heavy_owner, so the claim alone let a SCPI capture
   write PSRAM 0 (the LA region, where OTA stages its image) mid-OTA. */
bool command_handler_acquire_adc(int conn_id) {
    return !heavy_in_flight() && heavy_try_claim(conn_id);
}
void command_handler_release_adc(int conn_id) { heavy_release(conn_id); }

/* Single source of truth for "a heavy op is running" (see the gate section).
   OTA staging/verify also owns the PSRAM bus, so it counts as heavy — this gives
   OTA and captures mutual exclusion (a capture can't start mid-OTA and vice-versa). */
bool heavy_in_flight(void) {
    ota_state_t o = ota_get_state();
    return capture_busy() || o == OTA_RECEIVING || o == OTA_VERIFIED;
}

bool capture_or_upload_busy(void) {
    return capture_busy() || heavy_owner != -1 || load_bin_owner() >= 0;
}

bool heavy_or_claimed(void) {
    return heavy_in_flight() || heavy_owner != -1 || load_bin_owner() >= 0;
}

const char *bus_busy_reason(void) {
    return heavy_or_claimed() ? "busy: a capture, upload or update is using the PSRAM bus; "
                                "try again when it ends"
                              : NULL;
}

/* Emit one raw newline-delimited JSON line back over the (tunnel) conn. */
void cloud_send_json_line(int conn_id, const char *json) {
    char line[72];
    int n = snprintf(line, sizeof(line), "%s\n", json);
    if (n > 0 && (size_t)n < sizeof(line)) at_send_data(conn_id, (const uint8_t *)line, (size_t)n);
}

/* ---- Per-connection command reassembly ----
   ESP-AT delivers one +IPD per TCP segment; a JSON command may be split
   across segments, or two commands may share one segment.  We buffer per
   connection and dispatch on each '\n'. */
/* Sized to match the cloud command buffer (BP_CLOUD_CMD_IN_MAX) so a compact closed-loop
   curve LUT uploaded over LAN/serial reassembles in one line, same as over cloud. */
#define LINE_ASM_SIZE ((int)BP_CLOUD_CMD_IN_MAX)
typedef struct {
    char   buf[LINE_ASM_SIZE];
    size_t len;
    bool   discarding;   /* dropping an over-long line until its newline */
} line_asm_t;
/* Sized to cover the TCP conns (0..CH_MAX_CONN-1) plus the console (CH_CONSOLE_CONN), cloud command
   (CH_CLOUD_CONN) and the cloud tunnels (CH_CLOUD_TUNNEL_CONN..CH_CLOUD_TUNNEL_CONN_LAST) pseudo-
   connections, so a JSON handler that touches proto[]/line_asm[] for any of them stays in bounds. The
   tunnel conns, unlike the others, run the full buffered state machine in command_handler_process(). */
_Static_assert(CH_CONSOLE_CONN >= CH_MAX_CONN, "console conn id must sit above the TCP conns");
_Static_assert(CH_CLOUD_CONN > CH_CONSOLE_CONN, "cloud conn id must sit above the console conn");
_Static_assert(CH_CLOUD_TUNNEL_CONN > CH_CLOUD_CONN, "tunnel conn id must sit above the cloud conn");
_Static_assert(CH_CLOUD_TUNNEL_CONN_COUNT >= 1, "need at least one tunnel conn");
static line_asm_t line_asm[CH_CLOUD_TUNNEL_CONN_LAST + 1];

static proto_t proto[CH_CLOUD_TUNNEL_CONN_LAST + 1];

proto_t conn_proto(int conn_id) {
    return (conn_id >= 0 && conn_id <= CH_CLOUD_TUNNEL_CONN_LAST) ? proto[conn_id] : PROTO_UNKNOWN;
}

void conn_proto_set(int conn_id, proto_t p) {
    if (conn_id >= 0 && conn_id <= CH_CLOUD_TUNNEL_CONN_LAST) proto[conn_id] = p;
}

void command_handler_poll(void) {
    uart_rearm_poll();   /* before the UART drain below touches the new fabric */
    /* ---- deferred reboot for `psram_recover`: fire once the ack has had time to flush ---- */
    if (s_recover_pending && time_reached(s_recover_at)) {
        printf("[recover] rebooting to clear the PSRAM datapath (boot auto-reflashes the iCE40)\n");
        NVIC_SystemReset();   /* does not return */
    }

    /* ---- DAC uploads: give back a gate nobody replayed, abort a stalled load_bin ---- */
    dac_poll();

    /* ---- LA pin housekeeping: a finished step train releases its pins; an SWD session that
       hit its inactivity backstop (fpga_swd_poll was never called until now) releases its
       two.  Both are cheap RAM checks until something is actually running. ---- */
    la_step_poll();
    dap_poll();

    /* ---- power profile: one INA238 register read per pass, plus its reply pacing ---- */
    power_profile_poll();
    power_profile_service();

    /* ---- captures: trigger timeouts, completions and the paced bulk read-back ---- */
    capture_poll();

    /* ---- cloud speed-test upload: paced synthetic byte source ---- */
    speedtest_pump();

    /* ---- UART proxy: stream DUT→client and apply the +++ trailing guard ---- */
    uart_proxy_poll();

}

/* ---- Command handlers ---- */

/* Parse an optional "sample_rate_mhz" field into a sample rate in Hz.
   Returns 0.0f (= "auto-pick") when the field is absent.  Accepts decimals
   like 0.5, 1, 12.  Clamps to the achievable range; the firmware will pick
   the closest divider and log what it actually used. */
float parse_sample_rate_hz(const char *json) {
    char sr_s[16] = {0};
    if (!json_get_value(json, "sample_rate_mhz", sr_s, sizeof(sr_s))) return 0.0f;
    float mhz = (float)atof(sr_s);
    if (mhz <= 0.0f) return 0.0f;   /* invalid → auto */
    return mhz * 1.0e6f;
}

void handle_ping(int conn_id, const char *json) {
    (void)json;
    send_ok_str(conn_id, "\"pong\"");
}

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

void handle_target_power(int conn_id, const char *json) {
    char efuse_s[8]  = {0};
    char state_s[8]  = {0};
    char delay_s[12] = {0};

    if (!json_get_value(json, "efuse", efuse_s, sizeof(efuse_s))) {
        send_error(conn_id, "missing efuse");
        return;
    }
    if (!json_get_value(json, "state", state_s, sizeof(state_s))) {
        send_error(conn_id, "missing state");
        return;
    }

    int      efuse    = atoi(efuse_s);
    bool     on       = (atoi(state_s) != 0);
    uint32_t delay_ms = 0;
    if (json_get_value(json, "delay_ms", delay_s, sizeof(delay_s)))
        delay_ms = (uint32_t)strtoul(delay_s, NULL, 0);

    /* delay_ms == 0 applies immediately; > 0 schedules and fires from the main
       poll loop, so the connection can switch to e.g. UART proxy meanwhile. */
    if (target_power_schedule(efuse, on, delay_ms) != 0) {
        send_error(conn_id, "invalid efuse");
        return;
    }

    /* "enabled" echoes the requested state; with delay_ms > 0 it is the state
       the eFuse will reach once the delay elapses. */
    char payload[56];
    snprintf(payload, sizeof(payload),
             "{\"efuse\":%d,\"enabled\":%d,\"delay_ms\":%u}",
             efuse, on ? 1 : 0, (unsigned)delay_ms);
    send_ok_str(conn_id, payload);
}

void handle_target_status(int conn_id, const char *json) {
    (void)json;
    target_power_status_t e1, e2;
    target_power_get_status(1, &e1);
    target_power_get_status(2, &e2);

    /* Payload (~112 chars) + wrapper exceeds send_ok_str's 128-byte buffer,
       so build and send the response directly like handle_status. */
    char resp[224];
    snprintf(resp, sizeof(resp),
             "{\"status\":\"ok\",\"data\":{"
             "\"efuse1\":{\"enabled\":%d,\"fault\":%d,\"valid\":%d},"
             "\"efuse2\":{\"enabled\":%d,\"fault\":%d,\"valid\":%d},"
             "\"status_supported\":%s}}\n",
             e1.enabled, e1.fault, e1.valid,
             e2.enabled, e2.fault, e2.valid,
             e1.status_supported ? "true" : "false");
    if (at_send_data(conn_id, (const uint8_t *)resp, strlen(resp)) != 0) {
        at_close_connection(conn_id);
    }
}

/* `power_status` — read the on-board INA238 current monitors (0x40 = internal,
   0x44 = external supply). Reports bus voltage (mV) + current (µA) per rail;
   `ok:false` when a rail can't be read. Single-reply (cloud command channel).
   Boards with the pod's own monitor (0x41) add "pod": the pod's 5 V current (the DUT's
   internal rail bypasses that shunt) and total_ua, pod + internal rail = what the USB input
   delivers. Older boards leave "pod" out. */
void handle_power_status(int conn_id, const char *json) {
    (void)json;
    int ibus = 0, ish = 0, icur = 0;
    int ebus = 0, esh = 0, ecur = 0;
    int pbus = 0, psh = 0, pcur = 0;
    /* I2C1 is shared with the console `ina` command, the TCA9554 expanders and the
       power_profile sampler, all of which take hw_lock — this one read did not, so a
       concurrent transaction could interleave and return a torn register. */
    hw_lock();
    bool int_ok = (ina238_read(I2C_ADDR_INA238_INTERNAL, &ibus, &ish, &icur) == 0);
    hw_unlock();
    hw_lock();
    bool ext_ok = (ina238_read(I2C_ADDR_INA238_EXTERNAL, &ebus, &esh, &ecur) == 0);
    hw_unlock();
    bool pod = ina_pod_present(), pod_ok = false;
    if (pod) {
        hw_lock();
        pod_ok = (ina238_read(I2C_ADDR_INA_POD, &pbus, &psh, &pcur) == 0);
        hw_unlock();
    }

    char resp[352];
    bp_emit_t e;
    bp_emit_init(&e, resp, sizeof(resp));
    bp_emit(&e, "{\"status\":\"ok\",\"data\":{"
                "\"internal\":{\"ok\":%s,\"bus_mv\":%d,\"current_ua\":%d},"
                "\"external\":{\"ok\":%s,\"bus_mv\":%d,\"current_ua\":%d}",
            int_ok ? "true" : "false", ibus, icur,
            ext_ok ? "true" : "false", ebus, ecur);
    if (pod) {
        bp_emit(&e, ",\"pod\":{\"ok\":%s,\"bus_mv\":%d,\"current_ua\":%d",
                pod_ok ? "true" : "false", pbus, pcur);
        if (pod_ok && int_ok) bp_emit(&e, ",\"total_ua\":%ld", (long)pcur + (long)icur);
        bp_emit_raw(&e, "}");
    }
    bp_emit_raw(&e, "}}\n");
    if (at_send_data(conn_id, (const uint8_t *)resp, strlen(resp)) != 0) {
        at_close_connection(conn_id);
    }
}

void handle_status(int conn_id, const char *json) {
    (void)json;
    const char *ip = wifi_get_ip();
    const char *wstate;
    wifi_state_t st = wifi_get_state();
    switch (st) {
        case WIFI_DISCONNECTED: wstate = "disconnected"; break;
        case WIFI_CONNECTING:   wstate = "connecting";   break;
        case WIFI_CONNECTED:    wstate = "connected";    break;
        case WIFI_READY:        wstate = "ready";        break;
        default:                wstate = "unknown";      break;
    }

    /* RSSI is null if not associated; otherwise the AP signal strength in
       dBm.  Querying it costs an AT round-trip (~10s of ms) so we only do
       it when associated. */
    char rssi_field[24] = "null";
    if (st == WIFI_CONNECTED || st == WIFI_READY) {
        int rssi = 0;
        if (wifi_get_rssi(&rssi) == 0) {
            snprintf(rssi_field, sizeof(rssi_field), "%d", rssi);
        }
    }

    /* Built with the bounds-tracked emitter: the payload has grown past what a
       hand-sized buffer safely holds, and last_crash is free-form text (escaped
       via bp_emit_jstr) so it can never break the JSON. */
    /* 1024, not 768: caps[] below is now the full, dynamically-built feature set (was a
       7-name literal), which adds ~150 B on a v2 pod.  The emitter is bounds-tracked and
       last_crash is free-form, so keep real headroom rather than sizing to today's payload. */
    /* 1152: the LA-pin / trigger / power-profile names below add another ~60 B of caps[].
       1344: safe_mode + safe_reason (up to ~130 B). */
    /* 1408: gateware + gateware_embedded.
       1536: stacks{} per task (~100 B for 5 tasks), and the lwip/flash fields added since. */
    char resp[1536];
    cloud_caps_t caps;
    cloud_caps_collect(&caps);
    bp_emit_t e;
    bp_emit_init(&e, resp, sizeof(resp));
    bp_emit(&e, "{\"status\":\"ok\",\"data\":{"
                "\"device\":\"benchpod\",\"version\":\"%s\",\"board\":\"%s\",\"net\":\"%s\","
                "\"ip\":\"%s\",\"rssi_dbm\":%s,\"wifi\":\"%s\",\"cloud\":\"%s\",",
            FIRMWARE_VERSION, BOARD_NAME, wstate, ip, rssi_field,
            esp_wifi_ctrl_state_str(), cloud_client_state_str());
    bp_emit(&e, "\"rx_dropped\":%lu,\"tx_dropped\":%lu,"
                "\"adc_bits\":%d,\"adc_fullscale_mv\":%d,\"adc_channels\":%d,\"la_vccio_mv\":%d,",
            (unsigned long)console_rx_dropped(), (unsigned long)console_tx_dropped(),
            ADC_BITS, ADC_FULLSCALE_MV, ADC_CHANNELS, la_vccio_get_mv());
    /* Board revision decides which of the rev3 features below actually exist,
       so clients read it once here rather than probing each command. */
    bp_emit(&e, "\"board_rev\":\"%s\",\"board_rev_mv\":%d,\"nrst_pin\":%s,",
            board_rev_str(), board_rev_strap_mv(),
            caps.nrst_pin ? "true" : "false");
    /* Internal flash of this MCU: 2048 (ZIT6) or 1024 (ZGT6). Updaters check it before sending
       an image (flash_layout.h). */
    bp_emit(&e, "\"flash_kb\":%lu,", caps.flash_kb);
    /* Signed updates (fw_sign.h): ota_begin takes "sig"; the policy is "audit" (report only). */
    bp_emit(&e, "\"ota_sig\":true,\"sig_policy\":\"%s\",\"sig_keys\":%u,\"sig_policy_cmd\":true,"
                "\"lan_policy\":\"%s\",\"lan_policy_cmd\":true,\"tunnel_max_tier\":true,\"lease_state\":true,\"cloud_ca\":true,\"cloud_proxy\":true,",
            caps.sig_policy, (unsigned)fw_sign_key_count(), caps.lan_policy);
    {
        uint32_t left = 0;
        bool held = lease_gate_active(HAL_GetTick(), &left);
        bp_emit(&e, "\"lease\":{\"held\":%s,\"holder\":\"%s\",\"left_s\":%lu},",
                held ? "true" : "false", held ? lease_gate_holder() : "", (unsigned long)left);
    }
    /* lwIP memory high-water marks (lwipopts.h), to size MEM_SIZE and the pbuf pool from data. */
    bp_emit(&e, "\"lwip_mem_max\":%lu,\"lwip_mem_size\":%lu,\"pbuf_pool_max\":%u,\"pbuf_pool_size\":%u,",
            (unsigned long)lwip_stats.mem.max, (unsigned long)lwip_stats.mem.avail,
            (unsigned)lwip_stats.memp[MEMP_PBUF_POOL]->max, (unsigned)lwip_stats.memp[MEMP_PBUF_POOL]->avail);
    /* The gateware running in the iCE40 and the one this firmware embeds (0 = unknown). They
       differ after a firmware update until the boot-time gateware update has run. */
    {
        extern volatile uint32_t g_malloc_failures;   /* main.c: allocations that failed after boot */
        bp_emit(&e, "\"malloc_failures\":%lu,", (unsigned long)g_malloc_failures);
    }
    bp_emit(&e, "\"gateware\":%u,\"gateware_embedded\":%u,\"loop_tripped\":%s,\"uart_rx_overflow\":%s,",
            (unsigned)signal_engine_fpga_version(), (unsigned)ice40_embedded_gw_version(),
            fpga_dac_loop_tripped() ? "true" : "false", fpga_uart_rx_overflowed() ? "true" : "false");
    bp_emit(&e, "\"psram\":\"%s\",\"psram_ok\":%s,\"heap_free\":%u,\"heap_min\":%u,\"stack_min\":%u,"
                "\"reset\":\"%s\",\"last_crash\":",
            psram_selftest_str(), signal_engine_psram_operable() ? "true" : "false",
            sys_health_heap_free(), sys_health_heap_min_free(),
            sys_health_stack_min_free(), fault_last_reset_str());
    bp_emit_jstr(&e, fault_last_crash_str());
    /* Per-task stack headroom (minimum-ever free bytes), every task incl. IDLE and the timer
       task, so stack_min says which task it is. */
    {
        sys_health_task_t t[SYS_HEALTH_MAX_TASKS];
        int n = sys_health_tasks(t, SYS_HEALTH_MAX_TASKS);
        bp_emit_raw(&e, ",\"stacks\":{");
        for (int i = 0; i < n; i++) {
            if (i) bp_emit_raw(&e, ",");
            bp_emit_jstr(&e, t[i].name);
            bp_emit(&e, ":%u", t[i].stack_free);
        }
        bp_emit_raw(&e, "}");
    }
    /* Analog front end (board_variant.h): false on the digital-only board. */
    bp_emit(&e, ",\"analog\":%s", caps.analog ? "true" : "false");
    /* Safe mode (boot_guard.h): the reason says what is off and why; "" when not. */
    bp_emit(&e, ",\"safe_mode\":%s,\"safe_reason\":", caps.safe_mode ? "true" : "false");
    bp_emit_jstr(&e, caps.safe_reason);
    /* caps[]: the pod's advertised feature set, and the only capability source a client on a
       direct LAN/serial connection ever sees. The same values as the cloud `capabilities` frame
       (cloud_caps.h), so the two cannot drift. */
    cloud_caps_emit_list(&e, &caps);
    bp_emit_raw(&e, "}}\n");
    if (!bp_emit_ok(&e)) { send_error(conn_id, "status too large"); return; }
    if (at_send_data(conn_id, (const uint8_t *)resp, bp_emit_len(&e)) != 0) {
        at_close_connection(conn_id);
    }
}

/* ---- Device identity (Ed25519) ---- */

#define IDENT_NONCE_MAX 128   /* max nonce we will sign, in bytes */

void handle_identity_public(int conn_id, const char *json) {
    (void)json;
    uint8_t pub[DEVICE_ID_PUBLIC_LEN];
    if (device_identity_get_public(pub) != 0) {
        char msg[128];
        snprintf(msg, sizeof(msg), "identity not available: %s", device_identity_problem());
        send_error(conn_id, msg);
        return;
    }
    char b64[B64URL_ENCODED_LEN(DEVICE_ID_PUBLIC_LEN) + 1];
    b64url_encode(pub, sizeof(pub), b64, sizeof(b64));

    char payload[B64URL_ENCODED_LEN(DEVICE_ID_PUBLIC_LEN) + 16];
    snprintf(payload, sizeof(payload), "{\"public\":\"%s\"}", b64);
    send_ok_str(conn_id, payload);
}

void handle_identity_pop(int conn_id, const char *json) {
    char nonce_b64[B64URL_ENCODED_LEN(IDENT_NONCE_MAX) + 4];
    if (!json_get_value(json, "nonce", nonce_b64, sizeof(nonce_b64))) {
        send_error(conn_id, "missing nonce");
        return;
    }

    uint8_t nonce[IDENT_NONCE_MAX];
    size_t  nonce_len = 0;
    if (b64url_decode(nonce_b64, nonce, sizeof(nonce), &nonce_len) != 0) {
        send_error(conn_id, "invalid nonce");
        return;
    }

    uint8_t sig[DEVICE_ID_SIG_LEN];
    /* Signed under the proof-of-possession context, NOT the WS-auth context, so
       a pop signature obtained over the open command port can't be replayed as
       cloud WS authentication (see device_identity.h). */
    if (device_identity_sign_ctx(DEVICE_ID_CTX_POP, nonce, nonce_len, sig) != 0) {
        send_error(conn_id, "identity not available");
        return;
    }

    char b64[B64URL_ENCODED_LEN(DEVICE_ID_SIG_LEN) + 1];
    b64url_encode(sig, sizeof(sig), b64, sizeof(b64));

    /* {"status":"ok","data":{"signature":"<86 chars>"}}\n exceeds send_ok_str's
       128-byte buffer, so build and send directly like handle_target_status. */
    char resp[B64URL_ENCODED_LEN(DEVICE_ID_SIG_LEN) + 64];
    snprintf(resp, sizeof(resp),
             "{\"status\":\"ok\",\"data\":{\"signature\":\"%s\"}}\n", b64);
    if (at_send_data(conn_id, (const uint8_t *)resp, strlen(resp)) != 0) {
        at_close_connection(conn_id);
    }
}

/* identity_wipe only runs on the pod's USB console (console.c, physical presence); every JSON
   transport gets this refusal. */
void handle_identity_wipe(int conn_id, const char *json) {
    (void)json;
    send_error(conn_id, "identity_wipe: only on the pod's USB console (physical presence): "
                        "benchpod identity wipe --connection usb");
}

/* ---- Emulated I2C sensor ----
 * Mock an I2C sensor on two LA channels, driven by the FPGA's generic target.
 *
 *   {"cmd":"sensor_start","type":"bmp280","addr":"0x76","sda":1,"scl":2}
 *   {"cmd":"sensor_set","temperature_c":25.0,"pressure_pa":101325}
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

void handle_sensor_set(int conn_id, const char *json) {
    char t_s[16] = {0}, p_s[16] = {0};
    bool any = false;

    if (!sensor_sim_active()) { send_error(conn_id, "no sensor active"); return; }

    if (json_get_value(json, "temperature_c", t_s, sizeof(t_s))) {
        if (sensor_sim_set("temperature_c", (float)atof(t_s)) != 0) {
            send_error(conn_id, "temperature_c rejected");
            return;
        }
        any = true;
    }
    if (json_get_value(json, "pressure_pa", p_s, sizeof(p_s))) {
        if (sensor_sim_set("pressure_pa", (float)atof(p_s)) != 0) {
            send_error(conn_id, "pressure_pa rejected");
            return;
        }
        any = true;
    }
    if (!any) { send_error(conn_id, "no recognised parameters"); return; }

    char payload[64];
    snprintf(payload, sizeof(payload), "{\"type\":\"%s\"}", sensor_sim_type());
    send_ok_str(conn_id, payload);
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
    char resp[256];
    if (!sensor_sim_active()) {
        snprintf(resp, sizeof(resp),
                 "{\"status\":\"ok\",\"data\":{\"active\":false}}\n");
    } else {
        i2c_sensor_status_t st = {0};
        sensor_sim_get_status(&st);
        snprintf(resp, sizeof(resp),
                 "{\"status\":\"ok\",\"data\":{"
                 "\"active\":true,\"type\":\"%s\",\"addr\":%u,"
                 "\"transactions\":%u,\"writes\":%u,"
                 "\"last_reg\":%u,\"last_val\":%u}}\n",
                 sensor_sim_type(), sensor_sim_addr7(),
                 st.xfer_count, st.wr_count, st.last_wr_addr, st.last_wr_val);
    }
    if (at_send_data(conn_id, (const uint8_t *)resp, strlen(resp)) != 0) {
        at_close_connection(conn_id);
    }
}

/* ---- Entry point ---- */

/* Dispatch one complete, NUL-terminated JSON command line. */
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

/* Report the USB-C CC-line state (rev3+): which orientation the cable is in and
   how much current the upstream source advertises.
     {"cmd":"usb_cc"}  -> {"supported":true,"cc1_mv":..,"cc2_mv":..,
                           "orientation":"cc1"|"cc2"|"none",
                           "advertised":"none"|"default"|"1.5A"|"3.0A",
                           "advertised_ma":0|500|1500|3000}
   Report-only: nothing in the firmware gates on it. */
void handle_usb_cc(int conn_id, const char *json) {
    (void)json;
    usb_cc_t cc;
    if (usb_cc_read(&cc) != 0) {
        /* Distinguish "this board has no CC taps" from "the conversion failed",
           so a genuine ADC fault is never reported as a valid 0 mV reading. */
        send_error(conn_id, cc.supported ? "usb cc read failed"
                                         : "usb cc monitoring needs a v3 pod");
        return;
    }
    static const char *const orient[] = { "none", "cc1", "cc2" };
    char payload[176];
    snprintf(payload, sizeof(payload),
             "{\"supported\":%s,\"cc1_mv\":%d,\"cc2_mv\":%d,\"orientation\":\"%s\","
             "\"advertised\":\"%s\",\"advertised_ma\":%d}",
             cc.supported ? "true" : "false", cc.cc1_mv, cc.cc2_mv,
             orient[(cc.orientation >= 0 && cc.orientation <= 2) ? cc.orientation : 0],
             cc.advertised, cc.advertised_ma);
    send_ok_str(conn_id, payload);
}

/* Drive the dedicated target-reset line (rev3+, /NRST_CONTROL on J1 pin 22).
     {"cmd":"nrst","assert":1}      -> hold the target in reset
     {"cmd":"nrst","assert":0}      -> release (Hi-Z; the DUT's pull-up wins)
     {"cmd":"nrst","pulse_ms":50}   -> assert, wait, release
     {"cmd":"nrst"}                 -> report state
   Flashing does NOT need this: dap_start's CMSIS-DAP SWJ_PINS already drives the
   same pin, so an OpenOCD/pyOCD connect-under-reset works with no extra call.
   This is for power-on-reset style test steps that are not flashing. */
void handle_nrst(int conn_id, const char *json) {
    if (!nrst_ctrl_supported()) {
        send_error(conn_id, "nrst pin needs a v3 pod");
        return;
    }
    char s[12] = {0};
    if (json_get_value(json, "pulse_ms", s, sizeof(s))) {
        int ms = atoi(s);
        if (ms <= 0) { send_error(conn_id, "pulse_ms must be > 0"); return; }
        nrst_ctrl_pulse((uint32_t)ms);
    } else if (json_get_value(json, "assert", s, sizeof(s))) {
        nrst_ctrl_assert(json_flag(json, "assert"));
    }
    char payload[64];
    snprintf(payload, sizeof(payload), "{\"supported\":true,\"asserted\":%s}",
             nrst_ctrl_is_asserted() ? "true" : "false");
    send_ok_str(conn_id, payload);
}

/* Route the buffered DAC through the U55 TMUX muxes (U47/U48). Set a CTRLn by
   including its ``ctrlN_en``; ``ctrlN_sel`` picks the path (ctrl1: 0=3V3,1=5V,
   2=12V,3=12V_ADC; ctrl2: 0=12V_VMID,1=ADC_VMID,2=GND). Omitted CTRLn is left
   as-is; a bare command just reports state. */
void handle_dac_mux(int conn_id, const char *json) {
    char s[8] = {0};
    if (json_get_value(json, "ctrl1_en", s, sizeof(s))) {
        int sel = json_get_value(json, "ctrl1_sel", s, sizeof(s)) ? atoi(s) : 0;
        if (sel < 0 || sel > 3) { send_error(conn_id, "ctrl1_sel 0..3"); return; }
        if (dacmux_set_ctrl1(json_flag(json, "ctrl1_en"), (uint8_t)sel) != 0) {
            send_error(conn_id, "dac_mux ctrl1 failed"); return;
        }
    }
    if (json_get_value(json, "ctrl2_en", s, sizeof(s))) {
        int sel = json_get_value(json, "ctrl2_sel", s, sizeof(s)) ? atoi(s) : 0;
        if (sel < 0 || sel > 3) { send_error(conn_id, "ctrl2_sel 0..3"); return; }
        if (dacmux_set_ctrl2(json_flag(json, "ctrl2_en"), (uint8_t)sel) != 0) {
            send_error(conn_id, "dac_mux ctrl2 failed"); return;
        }
    }
    uint8_t reg = 0;
    if (dacmux_read(&reg) != 0) { send_error(conn_id, "dac_mux read failed"); return; }
    char payload[112];
    snprintf(payload, sizeof(payload),
        "{\"reg\":%u,\"ctrl1_en\":%u,\"ctrl1_sel\":%u,\"ctrl2_en\":%u,\"ctrl2_sel\":%u}",
        reg, (reg & 0x01u) ? 1u : 0u, (unsigned)((reg >> 1) & 3u),
        (reg & 0x08u) ? 1u : 0u, (unsigned)((reg >> 4) & 3u));
    send_ok_str(conn_id, payload);
}

/* Calibration relay switching via U58 (relays energised by a bit high): cal1
   (5V DAC->ADC), cal2 (12V DAC->ADC, exclusive with cal1), current_in
   (ADC<-amps terminal), cal_path (ADC<-cal path). Any field present sets the
   whole state (absent = off); a bare command just reports state. */
void handle_cal_switch(int conn_id, const char *json) {
    char s[8] = {0};
    bool any = json_get_value(json, "cal1", s, sizeof(s))
             | json_get_value(json, "cal2", s, sizeof(s))
             | json_get_value(json, "current_in", s, sizeof(s))
             | json_get_value(json, "cal_path", s, sizeof(s));
    if (any) {
        int rc = calsw_set(json_flag(json, "cal1"), json_flag(json, "cal2"),
                           json_flag(json, "current_in"), json_flag(json, "cal_path"));
        if (rc == -2) { send_error(conn_id, "cal1 and cal2 are mutually exclusive"); return; }
        if (rc != 0)  { send_error(conn_id, "cal_switch failed"); return; }
    }
    uint8_t reg = 0;
    if (calsw_read(&reg) != 0) { send_error(conn_id, "cal_switch read failed"); return; }
    char payload[112];
    snprintf(payload, sizeof(payload),
        "{\"reg\":%u,\"cal1\":%u,\"cal2\":%u,\"current_in\":%u,\"cal_path\":%u}",
        reg, (reg & 0x01u) ? 1u : 0u, (reg & 0x02u) ? 1u : 0u,
        (reg & 0x04u) ? 1u : 0u, (reg & 0x08u) ? 1u : 0u);
    send_ok_str(conn_id, payload);
}

/* analog_path — apply a named analog path (the SINGLE SOURCE OF TRUTH lives in
   i2c_bus.c::analog_path_set).  {"cmd":"analog_path","path":"cal1"} flips every
   switch the path needs and returns the resulting mux/relay registers.
   Names: off dac_3v3|3v3 dac_5v|5v dac_12v|12v adc_ext|ext|sma cal1 cal2 current_in current_out. */
void handle_analog_path(int conn_id, const char *json) {
    char name[16] = {0};
    if (!json_get_value(json, "path", name, sizeof(name))) { send_error(conn_id, "missing path"); return; }
    analog_path_t p;
    if (analog_path_from_name(name, &p) != 0) { send_error(conn_id, "unknown path"); return; }
    if (analog_path_set(p) != 0) { send_error(conn_id, "analog_path failed"); return; }
    uint8_t u55 = 0, u58 = 0; dacmux_read(&u55); calsw_read(&u58);
    char payload[80];
    snprintf(payload, sizeof(payload), "{\"path\":\"%s\",\"u55\":%u,\"u58\":%u}",
             analog_path_name(p), u55, u58);
    send_ok_str(conn_id, payload);
}

/* dac_out — route a DAC output path AND set a CALIBRATED voltage (one step, no
   manual mux flipping).  {"cmd":"dac_out","path":"5v","volts":2.5} → achieved
   mv + code.  path off just parks the output; omit volts to only route. */
void handle_dac_out(int conn_id, const char *json) {
    char name[16] = {0}, volts_s[16] = {0};
    if (!json_get_value(json, "path", name, sizeof(name))) { send_error(conn_id, "missing path"); return; }
    analog_path_t p; int idx = -1;
    if (analog_path_from_name(name, &p) != 0) { send_error(conn_id, "unknown path"); return; }
    if      (p == ANALOG_PATH_DAC_3V3) idx = 0;
    else if (p == ANALOG_PATH_DAC_5V)  idx = 1;
    else if (p == ANALOG_PATH_DAC_12V) idx = 2;
    else if (p != ANALOG_PATH_OFF) { send_error(conn_id, "path must be 3v3|5v|12v|off"); return; }
    if (analog_path_set(p) != 0) { send_error(conn_id, "route failed"); return; }
    long code = -1; int got_mv = 0;
    if (idx >= 0 && json_get_value(json, "volts", volts_s, sizeof(volts_s))) {
        float v = (float)atof(volts_s);
        float a = DAC_CAL[idx].a, b = DAC_CAL[idx].b;
        code = lroundf((v - a) / b);
        if (code < 0) code = 0;
        if (code > 255) code = 255;
        if (dac_set_constant((uint8_t)code, 240) != 0) { send_error(conn_id, "dac set failed"); return; }
        got_mv = (int)lroundf((a + b * (float)code) * 1000.0f);
    }
    char payload[80];
    snprintf(payload, sizeof(payload), "{\"path\":\"%s\",\"mv\":%d,\"code\":%ld}",
             analog_path_name(p), got_mv, code);
    send_ok_str(conn_id, payload);
}

/* current_out — hold a current on the 4-20 mA output (J9), in microamps (current_out.h).
     {"cmd":"current_out","ua":12000} -> {"ua":12000,"code":32576,"min_ua":4016,"max_ua":20078}
     {"cmd":"current_out"}            -> {"min_ua":4016,"max_ua":20078}   (the range; nothing moves)
   `ua` in the reply is the current the nearest 16-bit DAC code gives. A request from 4000 uA up
   to min_ua gives min_ua; anything else outside min_ua..max_ua is refused, there is no clamp.
   Setting a current switches the DAC voltage outputs off first (analog path current_out): they
   share the DAC and would follow it. The loop needs an external floating supply; the pod cannot see
   whether current flows. dac_stop does not return the loop to 4 mA: send 4000 uA for that. */
void handle_current_out(int conn_id, const char *json) {
    char ua_s[24] = {0}, payload[112];
    if (!json_get_value(json, "ua", ua_s, sizeof(ua_s))) {
        snprintf(payload, sizeof(payload), "{\"min_ua\":%ld,\"max_ua\":%ld}",
                 current_out_min_ua(), current_out_max_ua());
        send_ok_str(conn_id, payload);
        return;
    }
    char *end = NULL;
    double req = strtod(ua_s, &end);
    if (end == ua_s || *end != '\0') { send_error(conn_id, "current_out: ua must be a number of microamps"); return; }
    uint16_t code = 0;
    const char *why = current_out_code(lround(req), &code);
    if (why) { send_error(conn_id, why); return; }
    /* Outputs off first, then the level: the voltage outputs never see the new code. */
    if (analog_path_set(ANALOG_PATH_CURRENT_OUT) != 0) { send_error(conn_id, "route failed"); return; }
    if (dac_set_constant16(code, 240) != 0) { send_error(conn_id, "dac set failed"); return; }
    snprintf(payload, sizeof(payload), "{\"ua\":%ld,\"code\":%u,\"min_ua\":%ld,\"max_ua\":%ld}",
             current_out_ua(code), (unsigned)code, current_out_min_ua(), current_out_max_ua());
    send_ok_str(conn_id, payload);
}

/* adc_read — route an ADC source AND return a CALIBRATED reading in mV.
   {"cmd":"adc_read","source":"ext"} → {"source","mv","count","span"}.  `current_in` also
   returns "offset_mv", this pod's calibration offset that was taken out of `mv`
   (adc_cal.h; 0 when the pod was never calibrated), and "ua", the loop current in
   microamps (mv across the 249 ohm sense resistor).  `count` stays raw.  A short
   16-sample burst, averaged ON THE 16-BIT CIRCLE and then unwrapped — see
   adc_scale.h.  Averaging the RAW counts first (what this did until 2026-07-29)
   returns a plausible-looking number that is wrong by tens of volts whenever the
   burst straddles the wrap, which is exactly where every source's 0 V point sits.
   `span` is the burst's pk-pk in counts; a burst wider than ADC_BURST_MAX_SPAN has
   no single voltage (the input is still moving — e.g. a DAC left driving by a
   preceding `measure`/`generate`) and is REFUSED rather than averaged.  Quick, so
   it blocks briefly. */
void handle_adc_read(int conn_id, const char *json) {
    char name[16] = {0};
    analog_path_t p = ANALOG_PATH_ADC_EXT;
    if (json_get_value(json, "source", name, sizeof(name))) {
        if (analog_path_from_name(name, &p) != 0 ||
            (p != ANALOG_PATH_ADC_EXT && p != ANALOG_PATH_CAL1 &&
             p != ANALOG_PATH_CAL2 && p != ANALOG_PATH_CURRENT_IN)) {
            send_error(conn_id, "source must be ext|cal1|cal2|current_in"); return;
        }
    }
    if (!heavy_begin(conn_id)) return;
    if (analog_path_set(p) != 0) { heavy_release(conn_id); send_error(conn_id, "route failed"); return; }
    sleep_ms(20);    /* let the G6K relays (~4ms) + front-end RC settle (yields to FreeRTOS) */
    uint16_t s16[16] = {0};
    int rc = adc_capture_psram(s16, 16, 0.0f);
    heavy_release(conn_id);
    if (rc != 0) { send_error(conn_id, "adc read failed"); return; }
    /* Per-source affine fit.  The UNWRAP is not per-source — it belongs to the
       shared front end and adc_scale_burst always applies it (adc_scale.h). */
    cal_lin_t c = ADC_CAL_CAL1;
    if      (p == ANALOG_PATH_CAL2)    c = ADC_CAL_CAL2;
    else if (p == ANALOG_PATH_ADC_EXT) c = ADC_CAL_EXT;
    else if (p == ANALOG_PATH_CURRENT_IN)     c = adc_cal_current_in_fit();   /* cal1 fit + this pod's offset (adc_cal.h) */
    adc_reading_t rd = adc_scale_burst(s16, 16, c.a, c.b);
    if (!rd.valid) {
        /* Plausible-looking garbage is worse than an error: a sweep would record it. */
        char msg[176];   /* 112 cut the advice off the end */
        snprintf(msg, sizeof(msg),
                 "adc_read: input not settled on %s (%u counts pk-pk over the 16-sample "
                 "burst, limit %u). Stop the DAC (dac_stop) or let the node settle",
                 analog_path_name(p), (unsigned)rd.span, (unsigned)ADC_BURST_MAX_SPAN);
        send_error(conn_id, msg);
        return;
    }
    char payload[112];
    if (p == ANALOG_PATH_CURRENT_IN) {
        /* Say which per-pod offset is in `mv` (only `current_in` has one), and give the loop
           current the voltage stands for, so no client needs to know the sense resistor. */
        snprintf(payload, sizeof(payload),
                 "{\"source\":\"%s\",\"mv\":%d,\"count\":%d,\"span\":%u,\"offset_mv\":%ld,\"ua\":%ld}",
                 analog_path_name(p), (int)lroundf(rd.volts * 1000.0f),
                 adc_count_u16(rd.count), (unsigned)rd.span, (long)adc_cal_current_in_offset_mv(),
                 current_in_ua(rd.volts));
    } else {
        snprintf(payload, sizeof(payload),
                 "{\"source\":\"%s\",\"mv\":%d,\"count\":%d,\"span\":%u}",
                 analog_path_name(p), (int)lroundf(rd.volts * 1000.0f),
                 adc_count_u16(rd.count), (unsigned)rd.span);
    }
    send_ok_str(conn_id, payload);
}

/* The per-pod calibration of `current_in` as a reply. a_uv / b_nv are the fit this pod scales `current_in`
   with (volts = a + b*count), in the units of the capabilities frame's adc_cal_a_uv /
   adc_cal_b_nv, so a client can scale raw `current_in` counts the way adc_read does. */
static void calibrate_reply(int conn_id, const adc_reading_t *rd) {
    cal_lin_t fit = adc_cal_current_in_fit();
    char payload[224];
    int n = snprintf(payload, sizeof(payload),
                     "{\"source\":\"current_in\",\"calibrated\":%s,\"offset_mv\":%ld,\"offset_uv\":%ld,"
                     "\"a_uv\":%ld,\"b_nv\":%ld",
                     adc_cal_current_in_is_set() ? "true" : "false",
                     (long)adc_cal_current_in_offset_mv(), (long)adc_cal_current_in_offset_uv(),
                     lround((double)fit.a * 1000000.0), lround((double)fit.b * 1000000000.0));
    if (rd)
        n += snprintf(payload + n, sizeof(payload) - (size_t)n, ",\"count\":%d,\"span\":%u,\"samples\":%u",
                      adc_count_u16(rd->count), (unsigned)rd->span, (unsigned)ADC_CAL_SAMPLES);
    snprintf(payload + n, sizeof(payload) - (size_t)n, "}");
    send_ok_str(conn_id, payload);
}

/* `calibrate` — run, read or clear this pod's own ADC calibration (adc_cal.h). It sits on top
   of the compiled-in fits (cal_data.h) and is kept in flash.
     {"cmd":"calibrate"}                  -> the stored calibration
     {"cmd":"calibrate","source":"current_in"}   -> calibrate `current_in`: J8 must be DISCONNECTED
     {"cmd":"calibrate","clear":true}     -> back to the compiled-in fit
   `current_in` is the only source a pod can calibrate on its own: with J8 open the terminal is 0 V,
   so what it reads is its offset. A reading outside +/-50 mV means something is driving J8:
   it is refused and the old calibration stays. */
void handle_calibrate(int conn_id, const char *json) {
    char v[16] = {0};
    if (json_get_value(json, "clear", v, sizeof(v)) && strcmp(v, "true") == 0) {
        if (adc_cal_clear() != 0) { send_error(conn_id, "could not clear the calibration"); return; }
        calibrate_reply(conn_id, NULL);
        return;
    }
    if (!json_get_value(json, "source", v, sizeof(v))) { calibrate_reply(conn_id, NULL); return; }
    if (strcmp(v, "current_in") != 0) { send_error(conn_id, "only current_in can be calibrated on the pod: source must be current_in"); return; }
    if (!heavy_begin(conn_id)) return;
    adc_reading_t rd;
    int rc = adc_cal_measure_current_in(&rd);
    heavy_release(conn_id);
    if (rc != 0) { send_error(conn_id, rc == -1 ? "route failed" : "adc read failed"); return; }
    const char *why = adc_cal_current_in_store(&rd);
    if (why) { send_error(conn_id, why); return; }
    calibrate_reply(conn_id, &rd);
}

/* Put every module's mirror of the fabric back in step after a RECONFIGURATION (image swap,
   flash-ice40, boot/OTA reflash): called from ice40_reflash_image(), next to
   signal_engine_on_gateware_reconfigured. */
void command_handler_on_gateware_reconfigured(void) {
    /* The fabric's capture bases reset and the PSRAM was reset for the config read. */
    psram_regions_dirty_all("a gateware reload");
    dac_on_gateware_reconfigured();

    /* The reconfiguration also reset the UART, SWD, I2C-target and stepper engines, so every
       pin they owned is back to plain LA mode.  Tear the sessions down here (the table must
       not claim a function nothing is driving) and let la_pins_on_gateware_reconfigured()
       re-apply the gpio pins' GPIO_SET latches, which the fabric also lost. */
    swd_disarm_and_release();
    spi_on_gateware_reconfigured();
    if (sensor_sim_active()) sensor_sim_stop();
    la_pins_release_fn(LA_FN_I2C_SDA);
    la_pins_release_fn(LA_FN_I2C_SCL);
    capture_on_gateware_reconfigured();
    la_pins_on_gateware_reconfigured();
    uart_proxy_on_gateware_reconfigured();
}

/* {"cmd":"fpga_image","image":0|1} — switch the running iCE40 gateware IMAGE (0 = closed-loop,
   1 = deep-DAC-PSRAM-replay). SB_WARMBOOT can't reconfigure at runtime on this board (its
   config-SPI pins ARE the shared PSRAM bus), so fpga_warmboot() actually REFLASHES the selected
   image and does a CRESET cold-reconfig (~2-3 s). Returns the new image's {version, features};
   also asks the net task to re-announce capabilities so the server's cached caps track the swap. */
void handle_fpga_image(int conn_id, const char *json) {
    char s[8] = {0};
    if (!json_get_value(json, "image", s, sizeof(s)) || !s[0]) {
        send_error(conn_id, "image required (0=loop, 1=deep-replay)"); return;
    }
    uint8_t img = (uint8_t)strtoul(s, NULL, 10);
    if (!heavy_begin(conn_id)) return;
    int rc = fpga_warmboot(img);
    uint8_t ver = signal_engine_fpga_version();
    uint8_t feats = signal_engine_fpga_features();
    heavy_release(conn_id);
    if (rc == -1) { send_error(conn_id, "image out of range (0=loop, 1=deep-replay)"); return; }
    if (rc == -3) { send_error(conn_id, "image switch: reflash failed (CDONE never rose)"); return; }
    if (rc == -5) { signal_engine_psram_report_wedge(); send_error(conn_id, "image switch: iCE40->PSRAM write inoperable after retries (run psram_recover to reboot+reflash)"); return; }
    if (rc != 0)  { send_error(conn_id, "image switch failed"); return; }
    /* The live image (and thus dac_control_loop / dac_deep_replay / dac_replay_max_samples) just
       changed — re-announce so the cloud/webapp gate on the NEW image, not the connect-time one. */
    cloud_client_request_caps_resend();
    /* A reconfigure can glitch the DAC's SPI lines: put an output stage back on its park level. */
    if (dac_limits_get()->enabled) (void)dac_limits_park_now();
    char payload[96];
    snprintf(payload, sizeof(payload), "{\"image\":%u,\"version\":%u,\"features\":%u}", img, ver, feats);
    send_ok_str(conn_id, payload);
}

/* {"cmd":"psram_recover"} — one-click remote recovery for an inoperable PSRAM datapath: ack,
   then reboot.  The reboot resets every firmware-side mirror and peripheral, and the boot
   self-test auto-reflashes the iCE40 side (psram_boot_selftest_with_recovery), so the pod
   returns healthy WITHOUT a physical power-cycle.  (The July "only a reboot clears it" case was
   a stale firmware mirror of the capture base, since fixed: see fpga_warmboot.)  The client will not receive a
   command.response (the pod reboots first) — treat "no reply, device reconnects" as success
   and re-poll status.psram_ok. */
void handle_psram_recover(int conn_id, const char *json) {
    (void)json;
    send_ok_str(conn_id, "{\"recover\":\"rebooting\"}");
    s_recover_at = make_timeout_time_ms(400);   /* let the ack flush, then reset in the poll loop */
    s_recover_pending = true;
}

/* {"cmd":"psram_ping"[,"count":N]} → {"pass":bool,"count":N,"first_bad":i}
   Shared-bus write solidity check: the iCE40 streams a known +0x0101 ramp through
   the REAL capture datapath (writer -> shared quad bus -> PSRAM) and the STM32
   reads it back.  PASS => the iCE40 writes PSRAM and the STM32 sees it identically;
   FAIL => the iCE40->PSRAM write path is broken (NOT the analog ADC).  Cheap,
   deterministic, self-contained — used by hwe2e TestHW_V2_PsramPing. */
void handle_psram_ping(int conn_id, const char *json) {
    char v[12] = {0};
    int n = 16;
    if (json_get_value(json, "count", v, sizeof(v))) n = atoi(v);
    if (n < 4)   n = 4;
    if (n > 256) n = 256;
    if (!heavy_begin(conn_id)) return;
    int bad = -1;
    int rc = signal_engine_capture_selftest(n, &bad);
    heavy_release(conn_id);
    char payload[64];
    snprintf(payload, sizeof(payload), "{\"pass\":%s,\"count\":%d,\"first_bad\":%d}",
             rc == 0 ? "true" : "false", n, bad);
    send_ok_str(conn_id, payload);
}

/* Per cloud tunnel: the highest tier its user may use (command_handler.h). Written by the net
   task at tunnel.open, read by the worker. */
static volatile uint8_t s_tunnel_max_tier[CH_CLOUD_TUNNEL_CONN_COUNT] = { 3, 3, 3 };

void command_handler_set_tunnel_max_tier(int conn_id, int max_tier) {
    if (conn_id < CH_CLOUD_TUNNEL_CONN || conn_id > CH_CLOUD_TUNNEL_CONN_LAST) return;
    if (max_tier < 0 || max_tier > 3) max_tier = 3;
    s_tunnel_max_tier[conn_id - CH_CLOUD_TUNNEL_CONN] = (uint8_t)max_tier;
}

/* A LAN SCPI line with any non-query part, while a cloud job holds the pod (lease_gate.h). */
static bool scpi_line_blocked_by_lease(int conn_id, const char *line) {
    if (command_handler_policy_src(conn_id) != POLICY_SRC_LAN || !lease_gate_active(HAL_GetTick(), NULL))
        return false;
    if (!cmd_gate_scpi_line_writes(line)) return false;
    printf("[cmd] SCPI \"%s\" refused: a cloud job holds the pod\n", line);
    return true;
}

const char *command_handler_device_gate(const char *verb, const char *json) {
    return cmd_gate_device(verb, json, boot_guard_skip_hw(), board_has_analog());
}

static void dispatch_line(int conn_id, const char *buf) {
    char cmd[32] = {0};
    if (!json_get_value(buf, "cmd", cmd, sizeof(cmd))) {
        send_error(conn_id, "missing cmd");
        return;
    }

    /* The command table (cmd_table.h): tier, gate flags and handler. NULL = unknown verb, which
       still goes through the gates (as T3) before it is refused. */
    const cmd_desc_t *d = cmd_find(cmd);
    const cmd_tier_t tier = cmd_tier_of(d, buf);
    /* High-frequency polled reads (the web UI refreshes them on a timer) would flood the serial
       console with identical lines, so they are not traced. */
    if (!cmd_has(d, CMD_F_NOISY))
        printf("[cmd] <- \"%s\" (id=%d, %s)\n", cmd, conn_id, cmd_tier_name(tier));

    /* Tier, lease, tunnel, safe-mode and board gates (cmd_gate.h, host-tested). */
    {
        cmd_gate_ctx_t g = {
            .src = command_handler_policy_src(conn_id),
            .lan_policy = pod_policy_lan(),
            .tunnel_max_tier = -1,
            .skip_hw = boot_guard_skip_hw(),
            .has_analog = board_has_analog(),
        };
        if (g.src == POLICY_SRC_LAN) {
            g.lease_active = lease_gate_active(HAL_GetTick(), &g.lease_left_s);
            g.lease_holder = lease_gate_holder();
        }
        if (conn_id >= CH_CLOUD_TUNNEL_CONN && conn_id <= CH_CLOUD_TUNNEL_CONN_LAST)
            g.tunnel_max_tier = s_tunnel_max_tier[conn_id - CH_CLOUD_TUNNEL_CONN];
        char why[112];
        const char *refused = cmd_gate_check(cmd, buf, tier, &g, why, sizeof(why));
        if (refused) { send_error(conn_id, refused); return; }
    }

    /* DAC output limits (dac_limits.h): one check here covers every transport (LAN, cloud
       tunnel, cloud command channel). No-op on a pod without limits. */
    {
        const char *why = dac_limits_check_command(cmd, buf);
        if (why) { send_error(conn_id, why); return; }
    }

    if (d && d->fn) d->fn(conn_id, buf);
    else send_error(conn_id, "unknown cmd");
}

size_t command_handler_dispatch_cloud(const char *command_json, char *out, size_t out_cap) {
    char cmd[32] = {0};
    if (!json_get_value(command_json, "cmd", cmd, sizeof(cmd))) {
        int n = snprintf(out, out_cap, "{\"status\":\"error\",\"message\":\"missing cmd\"}");
        return (n > 0 && (size_t)n < out_cap) ? (size_t)n : 0;
    }
    /* Commands that stream chunks or switch the connection into a raw protocol (SWD/UART)
       cannot be carried over the single-reply cloud channel. */
    if (cmd_has(cmd_find(cmd), CMD_F_STREAM)) {
        int n = snprintf(out, out_cap,
                         "{\"status\":\"error\",\"message\":\"command not supported over cloud channel\"}");
        return (n > 0 && (size_t)n < out_cap) ? (size_t)n : 0;
    }

    /* Dispatch through the normal handlers; the reply is captured (see
       command_handler_cloud_capture_append) rather than sent to the TCP server. */
    cloud_reply_cap_begin();
    dispatch_line(CH_CLOUD_CONN, command_json);
    return cloud_reply_cap_end(out, out_cap);
}

void command_handler_process(int conn_id, const uint8_t *json_buf, size_t len) {
    /* conn_id outside the buffered set (console/cloud-command pseudo-conns or an unexpected id) —
       best-effort one-shot parse without per-connection buffering. The cloud tunnel IS buffered so
       its raw SWD/UART byte stream survives across segments like a real TCP conn. */
    if (!conn_runs_state_machine(conn_id)) {
        char buf[LINE_ASM_SIZE];
        size_t copy = len < sizeof(buf) - 1 ? len : sizeof(buf) - 1;
        memcpy(buf, json_buf, copy);
        buf[copy] = '\0';
        dispatch_line(conn_id, buf);
        return;
    }

    /* Accumulate into the per-connection line buffer and dispatch each
       newline-terminated command.  Handles commands split across TCP
       segments and multiple commands concatenated in one segment. */
    line_asm_t *la = &line_asm[conn_id];
    size_t i = 0;
    while (i < len) {
        /* Cloud speed-test sink (command_handler_net.c). */
        if (proto[conn_id] == PROTO_SPEEDTEST) {
            i += speedtest_receive(conn_id, json_buf + i, len - i);
            continue;
        }

        /* Raw binary waveform upload (command_handler_dac.c). */
        if (proto[conn_id] == PROTO_LOAD) {
            i += load_bin_receive(conn_id, json_buf + i, len - i);
            continue;
        }

        /* CMSIS-DAP mode (command_handler_dap.c). */
        if (proto[conn_id] == PROTO_DAP) {
            i += dap_proxy_receive(conn_id, json_buf + i, len - i);
            continue;   /* loop: process any trailing JSON, or exit at end */
        }

        /* UART transparent proxy (command_handler_uart.c). */
        if (proto[conn_id] == PROTO_UART) {
            i += uart_proxy_receive(conn_id, json_buf + i, len - i);
            continue;
        }

        char c = (char)json_buf[i++];
        if (c == '\r') continue;          /* tolerate CRLF */
        if (c == '\n') {
            if (la->discarding) {         /* end of an over-long line */
                la->discarding = false;
                la->len = 0;
            } else if (la->len > 0) {
                la->buf[la->len] = '\0';
                if (proto[conn_id] == PROTO_SCPI && scpi_line_blocked_by_lease(conn_id, la->buf)) {
                    /* SCPI setters wait while a cloud job holds the pod; queries still answer. */
                    scpi_push_execution_error();
                } else if (proto[conn_id] == PROTO_SCPI) scpi_dispatch_line(conn_id, la->buf);
                else                              dispatch_line(conn_id, la->buf);
                la->len = 0;
            }
            continue;
        }
        if (la->discarding) continue;
        /* Decide the protocol on the first non-whitespace byte of the
           connection.  Leading whitespace before that byte is skipped. */
        if (proto[conn_id] == PROTO_UNKNOWN) {
            if (c == ' ' || c == '\t') continue;
            proto[conn_id] = (c == '{') ? PROTO_JSON : PROTO_SCPI;
        }
        if (la->len < LINE_ASM_SIZE - 1) {
            la->buf[la->len++] = c;
        } else {
            /* No newline within LINE_ASM_SIZE — drop the line, then swallow the
               rest until its terminating newline.  Notify JSON clients; SCPI
               lines this long are not expected, so just discard. */
            la->len = 0;
            la->discarding = true;
            if (proto[conn_id] != PROTO_SCPI) send_error(conn_id, "command too long");
        }
    }
}

void command_handler_conn_closed(int conn_id) {
    /* A LAN client's OTA session outlives its socket: the next LAN connection adopts it (ota.h). */
    if (command_handler_policy_src(conn_id) == POLICY_SRC_LAN) ota_owner_gone(OTA_OWNER_LAN(conn_id));
    capture_conn_closed(conn_id);

    /* Abort an in-flight speed-test upload owned by this conn (the sink direction
       clears via the proto reset below). */
    speedtest_conn_closed(conn_id);

    /* A power profile keeps running (pod state), but its reply stream belongs to this conn. */
    power_profile_conn_closed(conn_id);

    /* Heavy ops complete within a single dispatch, so if this conn owns the gate,
       just free it. */
    heavy_release(conn_id);

    /* Drop any partial command buffered for this connection and reset its
       protocol so the next client on this slot is re-detected.  If it was an
       SWD session, release the wire to a safe state. Covers the cloud tunnel too
       (it runs the same state machine). */
    if (conn_runs_state_machine(conn_id)) {
        dap_conn_closed(conn_id);
        dac_conn_closed(conn_id);
        uart_proxy_conn_closed(conn_id);
        line_asm[conn_id].len = 0;
        line_asm[conn_id].discarding = false;
        proto[conn_id] = PROTO_UNKNOWN;
    }

    scpi_conn_closed(conn_id);
}

/* Reset one cloud-tunnel pseudo-connection between tunnels (tunnel.open / tunnel.close). Reuses the
   per-connection teardown so a torn-down tunnel that was mid-SWD/UART leaves the wire safe and the
   next tunnel starts in PROTO_UNKNOWN. */
void command_handler_tunnel_reset(int conn_id) {
    if (!is_tunnel_conn(conn_id)) return;
    command_handler_conn_closed(conn_id);
}
