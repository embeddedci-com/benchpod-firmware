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

void command_handler_poll(void) {
    uart_rearm_poll();   /* before the UART drain below touches the new fabric */
    /* ---- deferred reboot for `psram_recover`: fire once the ack has had time to flush ---- */
    device_poll();

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

/* Per cloud tunnel: the highest tier its user may use (command_handler.h). Written by the net
   task at tunnel.open, read by the worker. */
static volatile uint8_t s_tunnel_max_tier[CH_CLOUD_TUNNEL_CONN_COUNT] = { 3, 3, 3 };

void command_handler_set_tunnel_max_tier(int conn_id, int max_tier) {
    if (conn_id < CH_CLOUD_TUNNEL_CONN || conn_id > CH_CLOUD_TUNNEL_CONN_LAST) return;
    if (max_tier < 0 || max_tier > 3) max_tier = 3;
    s_tunnel_max_tier[conn_id - CH_CLOUD_TUNNEL_CONN] = (uint8_t)max_tier;
}

const char *command_handler_device_gate(const char *verb, const char *json) {
    return cmd_gate_device(verb, json, boot_guard_skip_hw(), board_has_analog());
}

void dispatch_line(int conn_id, const char *buf) {
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

    /* The raw modes end with the connection (an SWD session releases the wire to a safe state),
       then its partial command and protocol are dropped so the next client on this slot is
       re-detected. They read the protocol, so the transport goes last. Covers the cloud tunnel
       too (it runs the same state machine). */
    dap_conn_closed(conn_id);
    dac_conn_closed(conn_id);
    uart_proxy_conn_closed(conn_id);
    transport_conn_closed(conn_id);

    scpi_conn_closed(conn_id);
}

/* Reset one cloud-tunnel pseudo-connection between tunnels (tunnel.open / tunnel.close). Reuses the
   per-connection teardown so a torn-down tunnel that was mid-SWD/UART leaves the wire safe and the
   next tunnel starts in PROTO_UNKNOWN. */
void command_handler_tunnel_reset(int conn_id) {
    if (!is_tunnel_conn(conn_id)) return;
    command_handler_conn_closed(conn_id);
}
