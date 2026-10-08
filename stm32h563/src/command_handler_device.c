/*
 * command_handler_device.c — the pod itself: status, target power, identity and the iCE40.
 *
 *   status, power_status, target_power, target_status
 *   identity_public, identity_pop, identity_wipe (USB console only)
 *   usb_cc, nrst
 *   fpga_image, psram_recover, psram_ping
 *
 * Runs on the hw worker task, like the rest of command_handler.
 */
#include "command_handler.h"
#include "command_handler_internal.h"
#include "at_driver.h"
#include "bp_json.h"     /* shared flat-JSON parser + bounds-tracked emitter */
#include "bp_err.h"
#include "console.h"     /* console_rx_dropped / console_tx_dropped for status */
#include "signal_engine.h"
#include "stm32h5xx_hal.h"   /* NVIC_SystemReset for the psram_recover command, HAL_GetTick */
#include "wifi_manager.h"
#include "target_power.h"
#include "device_identity.h"
#include "i2c_bus.h"
#include "ina238.h"
#include "b64url.h"
#include "cloud_client.h"
#include "esp_wifi_ctrl.h"
#include "board_info.h"
#include "board_rev.h"
#include "usb_cc.h"
#include "nrst_ctrl.h"
#include "dac_limits.h"
#include "fault.h"       /* reset cause + last-crash summary for status */
#include "sys_health.h"  /* heap/stack headroom for status */
#include "ice40_flash.h"
#include "version.h"     /* FIRMWARE_VERSION (single source) */
#include "fw_sign.h"
#include "cloud_caps.h"     /* the capabilities, shared with the cloud announcement */
#include "lease_gate.h"
#include "hw_lock.h"     /* serialize the shared I2C bus (power_status vs the profile sampler) */
#include "la_pins.h"
#include "psram.h"       /* psram_selftest_str */
#include "lwip/stats.h"    /* lwIP memory high-water marks in status */
#include "lwip/memp.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/time.h"

/* JSON parsing is shared (see bp_json.h); same aliases as command_handler.c. */
#define json_get_value      bp_json_get
#define json_flag           bp_json_flag

/* Deferred reboot for the `psram_recover` command: send the ack first, then reset a beat
   later (command_handler_poll) so the reply flushes to the client before the pod reboots. */
static bool             s_recover_pending;
static absolute_time_t  s_recover_at;

/* command_handler_poll's device pass: the deferred `psram_recover` reboot. */
void device_poll(void) {
    if (s_recover_pending && time_reached(s_recover_at)) {
        printf("[recover] rebooting to clear the PSRAM datapath (boot auto-reflashes the iCE40)\n");
        NVIC_SystemReset();   /* does not return */
    }
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
