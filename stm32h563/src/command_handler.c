/*
 * command_handler.c — the JSON command glue: replies, the heavy gate, dispatch and the hooks.
 *
 * Bytes arrive through command_handler_transport.c, which hands complete JSON lines to
 * dispatch_line here: the command table (cmd_tier.c) names the handler, cmd_gate decides whether
 * it may run. The handlers live in the command_handler_*.c modules, each owning its state;
 * command_handler_poll, command_handler_conn_closed and command_handler_on_gateware_reconfigured
 * call every module's hook in a fixed order. command_handler_internal.h is the seam between them.
 *
 * Everything here runs on the hw worker task (hw_worker.c).
 */
#include "command_handler.h"
#include "net_server.h"
#include "command_handler_internal.h"
#include "bp_errors.h"
#include "at_driver.h"
#include "signal_engine.h"
#include "la_io.h"   /* la_vccio_get_mv */
#include "stm32h5xx_hal.h"   /* HAL_GetTick */
#include "sensor_mgr.h"   /* the emulated sensors: poll, reconfiguration, DAC conflicts */
#include "scpi_server.h"
#include "board_variant.h"
#include "bp_json.h"     /* shared flat-JSON parser + bounds-tracked emitter */
#include "cloud_reply_cap.h"   /* the captured reply of a cloud command.request */
#include "psram_regions.h"
#include "boot_guard.h"  /* safe mode: the iCE40/PSRAM-off command gate */
#include "cmd_gate.h"    /* the checks before a command reaches its handler (host-tested) */
#include "bp_features.h" /* BP_API, BP_MIN_CLIENT_API: the client API check */
#include "ota.h"         /* firmware OTA (PSRAM-staged) */
#include "cmd_table.h"     /* the command table: tier, gate flags and handler per verb */
#include "pod_policy.h"
#include "FreeRTOS.h"     /* pvPortMalloc: send_ok_large */
#include "lease_gate.h"
#include "lease_local.h"
#include "la_pins.h"     /* LA pin ownership table */
#include "power_profile.h"  /* INA238 rail profile sampler (poll + status) */
#include "dac_limits.h"
#include "dac_link.h"       /* BP0080: the flat cable's DAC pinout */
#include "module_link.h"
#include "dac_quad.h"      /* the ADC low range keeps the +-12 V output within +-5 V */
#include "i2c_bus.h"       /* adc_range_get */
#include "esp_rom_flash.h"   /* a C3 flash counts as heavy (esp_rom_flash_busy) */
#include "prog_flash.h"      /* and a flash over a chip's programming pins (prog_flash_busy) */
#include "esp_dut_flash.h"   /* so does an ESP32 DUT flash (esp_dut_flash_busy) */
#include "xip_session.h"   /* a W25Q code session holds the bus */
#if defined(BENCHPOD_USB_HOST)
#include "usb_flash.h"
#endif
#include "status_led.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#if defined(BENCHPOD_WIFI_BRIDGE)
#include "wifi_bridge_net.h"
#endif
#include "bp_log.h"

/* JSON parsing/emitting is shared (see bp_json.h).  These thin aliases keep the
   handler bodies below reading the way they did before the parser was unified. */
#define json_get_value      bp_json_get

/* ---- Response helpers ---- */

/* send_error / send_ok_str have external linkage (command_handler_internal.h): every module
   replies through them. */

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
        LOG_D("[cmd] -> ok  data=%s\n", payload_json);
    if (!bp_emit_ok(&e)) {
        /* Payload didn't fit the fixed reply buffer — send a structured error
           rather than a truncated (malformed) ok.  Handlers whose payload can be
           large build+send their reply directly instead of via send_ok_str. */
        bp_err_reply_too_large(conn_id);
        return;
    }
    if (at_send_data(conn_id, (const uint8_t *)resp, bp_emit_len(&e)) != 0) {
        at_close_connection(conn_id);   /* reclaim slot if client already left */
    }
}

/* send_ok_str for payloads larger than its stack buffer (usb_host's device report, usb_flash's
   state with a long result): the wrapper goes around the payload in a heap buffer. Payloads up to
   BP_CLOUD_REPLY_MAX - 64 also fit the cloud's captured reply. */
void send_ok_large(int conn_id, const char *payload_json) {
    static const char head[] = "{\"status\":\"ok\",\"data\":";
    size_t n = strlen(payload_json);
    size_t total = sizeof(head) - 1u + n + 2u;
    char *resp = pvPortMalloc(total + 1u);
    if (!resp) { send_error(conn_id, "out of memory for the reply"); return; }
    memcpy(resp, head, sizeof(head) - 1u);
    memcpy(resp + sizeof(head) - 1u, payload_json, n);
    memcpy(resp + total - 2u, "}\n", 3u);
    LOG_D("[cmd] -> ok  data=%s\n", payload_json);
    if (at_send_data(conn_id, (const uint8_t *)resp, total) != 0) at_close_connection(conn_id);
    vPortFree(resp);
}

/* Big enough for the longest message a caller can build (LA_PINS_ERR_MAX) even if bp_emit_jstr
   had to escape every single byte into two.  The pin/pull-conflict messages carry a JSON snippet
   of the fix, so they really do escape a run of quotes: at 160 bytes they overflowed, and because
   a full emitter drops every later write the closing "}\n" went with them — leaving the client
   waiting for a newline that never came, until its socket timed out. */
#define CH_ERR_RESP_MAX  (2u * LA_PINS_ERR_MAX + 48u)
_Static_assert(CH_ERR_RESP_MAX > LA_PINS_ERR_MAX, "error reply must outgrow its longest message");

/* A stable error code (bp_errors.h): the same reply buffer and the same never-a-torn-line rule as
   send_error, with "code" and "args" next to the message. */
void bp_error_send(int conn_id, bp_errcode_t code, const bp_err_arg_t *args, size_t n) {
    char resp[CH_ERR_RESP_MAX];
    size_t len = bp_error_format(resp, sizeof(resp), code, args, n);
    LOG_D("[cmd] -> error %s", resp);
    if (len == 0) return;   /* cannot happen with CH_ERR_RESP_MAX; the format keeps the code */
    if (at_send_data(conn_id, (const uint8_t *)resp, len) != 0) {
        at_close_connection(conn_id);
    }
}

void send_error(int conn_id, const char *message) {
    /* A message a module built from a code (bp_note_*, bp_errors.h): send it with that code. */
    bp_errcode_t code;
    size_t n;
    const bp_err_arg_t *args = bp_err_noted(message, &code, &n);
    if (args) { bp_error_send(conn_id, code, args, n); return; }
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
    LOG_D("[cmd] -> error: %s\n", message);
    if (at_send_data(conn_id, (const uint8_t *)resp, bp_emit_len(&e)) != 0) {
        at_close_connection(conn_id);
    }
}

/* Every LA-bank operation (la / la_capture / dap_start / uart_proxy / i2c-sensor)
   needs the LA I/O voltage chosen first — the TPS2116 mux is left "unset" at boot.
   When nothing set it, the first LA operation takes it from the wiring profile
   (la_voltage_from_profile); a profile without la_mv still refuses.  Returns true
   when a voltage is set; otherwise emits an error and returns false. */
bool require_la_voltage(int conn_id) {
    if (!la_voltage_from_profile()) {
        bp_err_la_voltage_not_set(conn_id);
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
        bp_err_pod_busy(conn_id);
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
    /* A C3 flash reads its image from the W25Q block by block with the gateware's PSRAM masters
       stopped, so it holds off captures, uploads and updates like they hold off each other. */
    return capture_busy() || o == OTA_RECEIVING || o == OTA_VERIFIED || o == OTA_APPLYING ||
           esp_rom_flash_busy() || esp_dut_flash_busy() || prog_flash_busy() || xip_session_active()
#if defined(BENCHPOD_USB_HOST)
           || usb_flash_busy()   /* the esp mode reads the staged image without an XIP session */
#endif
           ;
}

bool capture_or_upload_busy(void) {
    return capture_busy() || heavy_owner != -1 || load_bin_owner() >= 0;
}

bool heavy_or_claimed(void) {
    return heavy_in_flight() || heavy_owner != -1 || load_bin_owner() >= 0;
}

const char *bus_busy_reason(void) {
    /* The console shows these to a person, so they keep their words; JSON sends BP0048 with
       what is using the bus. */
    if (esp_rom_flash_busy())
        return bp_tag_storage_bus_busy("busy: the ESP32-C3 is being flashed (about 2 minutes); try again "
                                       "when it ends", "the Wi-Fi chip is being flashed");
    if (xip_session_active())
        return bp_tag_storage_bus_busy("busy: the pod is running code from its W25Q (a flash over USB); try "
                                       "again when it ends", "a flash over USB is running");
#if defined(BENCHPOD_USB_HOST)
    if (usb_flash_busy())
        return bp_tag_storage_bus_busy("busy: a board on the USB host port is being flashed (usb_flash); "
                                       "try again when it ends", "a flash over USB is running");
#endif
    if (prog_flash_busy())
        return bp_tag_storage_bus_busy("busy: a chip is being flashed over its programming pins (prog_flash); "
                                       "try again when it ends", "a target is being flashed (prog_flash)");
    if (esp_dut_flash_busy())
        return bp_tag_storage_bus_busy("busy: an ESP32 target is being flashed (esp_flash); try again when "
                                       "it ends", "a target is being flashed (esp_flash)");
#if defined(BENCHPOD_WIFI_BRIDGE)
    if (wifi_bridge_ota_busy())
        return bp_tag_storage_bus_busy("busy: the Wi-Fi bridge is being updated (about a minute); try again "
                                       "when it ends", "the Wi-Fi bridge is being updated");
#endif
    return heavy_or_claimed()
        ? bp_tag_storage_bus_busy("busy: a capture, upload or update is using the PSRAM bus; try again "
                                  "when it ends", "a capture, upload or update is running")
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

    /* The pod's own lease: its deadline, and a lease.break posted by the cloud link. */
    lease_local_expire(HAL_GetTick());

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

    /* ---- wifi_clear: erase the ESP32-C3's NVS once the reply is out ---- */
    wifi_clear_poll();

    /* ---- UART proxy: stream DUT→client and apply the +++ trailing guard ---- */
    uart_proxy_poll();

    /* ---- emulated sensors: write logs, timers, UART2, stimulus, re-arm after a reconfiguration.
       A few RAM checks when no sensor runs. ---- */
    sm_poll();
}

/* ---- Command handlers ---- */

/* Parse an optional "sample_rate_hz" field (a sample rate in Hz; firmware up to 3.10.13 took
   "sample_rate_mhz").  Returns 0.0f (= "auto-pick") when the field is absent.  Accepts decimals
   like 80000 or 2304000.5.  Clamps to the achievable range; the firmware will pick
   the closest divider and log what it actually used. */
float parse_sample_rate_hz(const char *json) {
    char sr_s[16] = {0};
    if (!json_get_value(json, "sample_rate_hz", sr_s, sizeof(sr_s))) return 0.0f;
    float hz = (float)atof(sr_s);
    return hz > 0.0f ? hz : 0.0f;   /* invalid → auto */
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
    i2c_on_gateware_reconfigured();
    /* An emulated sensor keeps its pins, values and live image: sm_poll() re-arms it on the new
       fabric (and releases the pins if that fails, or when the emulation image is gone), as the
       UART proxy does below. */
    sm_on_gateware_reconfigured();
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
    return cmd_gate_device(verb, json, boot_guard_skip_hw(), board_caps()->analog.present);
}

/* One command, its row `d` already looked up (cmd_table.h: tier, gate flags and handler; NULL =
   unknown verb, which still goes through the gates as T3 before it is refused).  Every gate
   below reads that row: the verb is looked up once per command. */
static void dispatch_row(int conn_id, const char *cmd, const cmd_desc_t *d, const char *buf) {
    const cmd_tier_t tier = cmd_tier_of(d, buf);
    /* High-frequency polled reads (the web UI refreshes them on a timer) would flood the serial
       console with identical lines, so they are not traced. */
    if (!cmd_has(d, CMD_F_NOISY) && conn_id != CH_EMI_CONN)   /* the EMI loop prints its own */
        LOG_D("[cmd] <- \"%s\" (id=%d, %s)\n", cmd, conn_id, cmd_tier_name(tier));

    /* A client older than this firmware takes (features.toml min_client_api): refused first, on
       every JSON transport (LAN, USB, cloud command channel, cloud tunnels), so it hears this
       before any other refusal. The EMC test build's own loop is not a client. */
    if (conn_id != CH_EMI_CONN) {
        int32_t sent = 0;
        if (!cmd_gate_client_api(d, buf, BP_MIN_CLIENT_API, &sent)) {
            bp_err_client_too_old(conn_id, sent, (int32_t)BP_MIN_CLIENT_API, (int32_t)BP_API);
            return;
        }
    }

    /* Tier, lease, tunnel, safe-mode and board gates (cmd_gate.h, host-tested). */
    {
        cmd_gate_ctx_t g = {
            .src = command_handler_policy_src(conn_id),
            .lan_policy = pod_policy_lan(),
            .usb_link = net_conn_on_usb(conn_id),
            .lan_pairing = pod_policy_lan_pairing(),
            .lan_paired = net_conn_lan_paired(conn_id),
            .tunnel_max_tier = -1,
            .skip_hw = boot_guard_skip_hw(),
            .has_analog = board_caps()->analog.present,
        };
        if (g.src == POLICY_SRC_LAN) {
            g.lease_active = lease_gate_active(HAL_GetTick(), &g.lease_left_s);
            g.lease_holder = lease_gate_holder();
        }
        if (conn_id >= CH_CLOUD_TUNNEL_CONN && conn_id <= CH_CLOUD_TUNNEL_CONN_LAST)
            g.tunnel_max_tier = s_tunnel_max_tier[conn_id - CH_CLOUD_TUNNEL_CONN];
        char why[112];
        const char *refused = cmd_gate_check(d, cmd, buf, tier, &g, why, sizeof(why));
        /* Each refusal is noted with its code (BP0025, BP0040, ...): send_error sends it. */
        if (refused) { send_error(conn_id, refused); return; }
    }
    /* The pod's own lease (lease_local.h): on every transport, only its owners drive the pod. */
    if (!command_handler_lease_gate(conn_id, d, buf)) return;

    /* A digital board and an analog module that disagree on the flat cable's DAC pins
       (dac_link.h): nothing drives the DAC. signal_engine blocks the frames too. */
    if (dac_link_command_drives_dac(cmd, buf) && module_dac_pinout_mismatch()) {
        char mod[20];
        module_dac_pinout_module(mod, sizeof(mod));
        bp_err_dac_pinout_mismatch(conn_id, cmd, module_dac_pinout_digital(), mod);
        return;
    }

    /* DAC output limits (dac_limits.h): one check here covers every transport (LAN, cloud
       tunnel, cloud command channel). No-op on a pod without limits. */
    {
        const char *why = dac_limits_check_command(cmd, buf);
        if (why) { send_error(conn_id, why); return; }
    }
    /* The ADC low range keeps the +-12 V output within +-5 V (dac_quad.h). The DAC writes and the
       range switch check it too; this names the command in the refusal. */
    {
        const char *why = dac_quad_low_check_command(cmd, buf, adc_range_get() == ADC_RANGE_LOW,
                                                     dac_quad_channel());
        if (why) { send_error(conn_id, why); return; }
    }
    /* An analog sensor plays the DAC or the 4-20 mA output: nothing else may drive it. */
    {
        const char *why = sensor_mgr_command_conflict(cmd);
        if (why) { send_error(conn_id, why); return; }
    }

    /* The status LED flashes for a command that drives the pod, not for the polled reads. */
    status_led_note_command(d && !cmd_has(d, CMD_F_NOISY) && !cmd_tier_light_of(d, buf));
    if (d && d->fn) d->fn(conn_id, buf);
    else bp_err_unknown_command(conn_id, cmd);
}

void dispatch_line(int conn_id, const char *buf) {
    char cmd[32] = {0};
    if (!json_get_value(buf, "cmd", cmd, sizeof(cmd))) {
        send_error(conn_id, "missing cmd");
        return;
    }
    dispatch_row(conn_id, cmd, cmd_find(cmd), buf);
}

size_t command_handler_dispatch_cloud(const char *command_json, char *out, size_t out_cap) {
    char cmd[32] = {0};
    if (!json_get_value(command_json, "cmd", cmd, sizeof(cmd))) {
        int n = snprintf(out, out_cap, "{\"status\":\"error\",\"message\":\"missing cmd\"}");
        return (n > 0 && (size_t)n < out_cap) ? (size_t)n : 0;
    }
    /* Commands that stream chunks or switch the connection into a raw protocol (SWD/UART)
       cannot be carried over the single-reply cloud channel. */
    const cmd_desc_t *d = cmd_find(cmd);
    if (cmd_has(d, CMD_F_STREAM)) {
        int n = snprintf(out, out_cap,
                         "{\"status\":\"error\",\"message\":\"command not supported over cloud channel\"}");
        return (n > 0 && (size_t)n < out_cap) ? (size_t)n : 0;
    }

    /* Dispatch through the normal handlers; the reply is captured (see
       command_handler_cloud_capture_append) rather than sent to the TCP server. */
    cloud_reply_cap_begin();
    dispatch_row(CH_CLOUD_CONN, cmd, d, command_json);
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

    /* If this conn still owns the heavy gate (a held upload, a finished capture), free it. */
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

    /* An owner's connection is gone: it no longer owns the pod's lease (the lease stays). */
    lease_local_conn_closed(conn_id);
}

/* Reset one cloud-tunnel pseudo-connection between tunnels (tunnel.open / tunnel.close). Reuses the
   per-connection teardown so a torn-down tunnel that was mid-SWD/UART leaves the wire safe and the
   next tunnel starts in PROTO_UNKNOWN. */
void command_handler_tunnel_reset(int conn_id) {
    if (!is_tunnel_conn(conn_id)) return;
    command_handler_conn_closed(conn_id);
}
