/*
 * command_handler_device.c — the pod itself: target power, identity and the iCE40 (status is in
 * command_handler_status.c).
 *
 *   power_status, target_power, target_status
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
#include "signal_engine.h"
#include "stm32h5xx_hal.h"   /* NVIC_SystemReset for the psram_recover command, HAL_GetTick */
#include "boot_guard.h"       /* boot_guard_planned_reset before the psram_recover reboot */
#include "wifi_manager.h"
#include "target_power.h"
#include "device_identity.h"
#include "i2c_bus.h"
#include "ina238.h"
#include "b64url.h"
#include "cloud_client.h"
#include "esp_wifi_ctrl.h"
#include "usb_device.h"
#include "usb_netif.h"
#include "board_info.h"
#include "board_rev.h"
#include "usb_cc.h"
#include "nrst_ctrl.h"
#include "dac_limits.h"
#include "ice40_flash.h"
#include "blob_store.h"
#include "sensor_mgr.h"   /* sensor_mgr_digital_names: an image switch away from emulation */
#include "version.h"     /* FIRMWARE_VERSION (single source) */
#include "cloud_caps.h"     /* the capabilities, shared with the cloud announcement */
#include "module_link.h"   /* digital v3r3: the module object */
#include "la_pins.h"
#include "psram.h"       /* psram_selftest_str */

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
        boot_guard_planned_reset(true);
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
    char resp[320];
    snprintf(resp, sizeof(resp),
             "{\"status\":\"ok\",\"data\":{"
             "\"efuse1\":{\"enabled\":%d,\"fault\":%d,\"valid\":%d,\"flt_pin\":%d,\"reverse_blocks\":%lu},"
             "\"efuse2\":{\"enabled\":%d,\"fault\":%d,\"valid\":%d,\"flt_pin\":%d,\"reverse_blocks\":%lu},"
             "\"status_supported\":%s}}\n",
             e1.enabled, e1.fault, e1.valid, e1.flt_pin, (unsigned long)e1.reverse_blocks,
             e2.enabled, e2.fault, e2.valid, e2.flt_pin, (unsigned long)e2.reverse_blocks,
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
    bool int_ok = (ina238_read(I2C_ADDR_INA238_INTERNAL, &ibus, &ish, &icur) == 0);
    bool ext_ok = (ina238_read(I2C_ADDR_INA238_EXTERNAL, &ebus, &esh, &ecur) == 0);
    bool pod = ina_pod_present(), pod_ok = false;
    if (pod) {
        pod_ok = (ina238_read(I2C_ADDR_INA_POD, &pbus, &psh, &pcur) == 0);
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

/* {"cmd":"fpga_image","image":0|1|2|"loop"|"deep"|"emulation"} — switch the running iCE40
   gateware IMAGE (0 = closed-loop, 1 = deep-DAC-PSRAM-replay, 2 = sensor emulation, gateware v54+).
   SB_WARMBOOT can't reconfigure at runtime on this board (its config-SPI pins ARE the shared
   PSRAM bus), so fpga_warmboot() actually REFLASHES the selected image and does a CRESET
   cold-reconfig (~2-3 s). Returns the new image's {image, name, version, features}; also asks the
   net task to re-announce capabilities so the server's cached caps track the swap.
   {"cmd":"fpga_image"} alone reports the running image. Leaving the emulation image while I2C,
   SPI or UART sensors run is refused (they need it): sensor_stop them first. */
void handle_fpga_image(int conn_id, const char *json) {
    char s[16] = {0};
    if (!json_get_value(json, "image", s, sizeof(s)) || !s[0]) {
        int kind = signal_engine_image_kind();
        char payload[112];
        snprintf(payload, sizeof(payload), "{\"image\":%d,\"name\":\"%s\",\"version\":%u,\"features\":%u}",
                 kind, signal_engine_image_name(kind), signal_engine_fpga_version(),
                 signal_engine_fpga_features());
        send_ok_str(conn_id, payload);
        return;
    }
    int img = -1;
    if (!strcmp(s, "0") || !strcmp(s, "loop")) img = 0;
    else if (!strcmp(s, "1") || !strcmp(s, "deep")) img = 1;
    else if (!strcmp(s, "2") || !strcmp(s, "emulation")) img = 2;
    if (img < 0) { send_error(conn_id, "image must be 0 (loop), 1 (deep) or 2 (emulation)"); return; }
    if (img == 2 && signal_engine_fpga_version() < EMULATION_MIN_GW) {
        send_error(conn_id, "the emulation image needs gateware v54 or later"); return;
    }
    if (img != 2) {
        char names[96];
        if (sensor_mgr_digital_names(names, sizeof(names))) {
            char msg[256];
            snprintf(msg, sizeof(msg), "image switch refused: the emulated sensors %s need the "
                     "emulation image; stop them first ({\"cmd\":\"sensor_stop\",\"all\":true})", names);
            send_error(conn_id, msg);
            return;
        }
    }
    if (!heavy_begin(conn_id)) return;
    int rc = fpga_warmboot((uint8_t)img);
    uint8_t ver = signal_engine_fpga_version();
    uint8_t feats = signal_engine_fpga_features();
    heavy_release(conn_id);
    if (rc == -1) { send_error(conn_id, "image must be 0 (loop), 1 (deep) or 2 (emulation)"); return; }
    if (rc == -2) {
        char msg[128];
        snprintf(msg, sizeof(msg), "image %d is not installed on the pod (W25Q slot %s is empty): "
                 "install the gateware blobs", img, blob_name((blob_id_t)ice40_image_slot(img)));
        send_error(conn_id, msg);
        return;
    }
    if (rc == -3) { send_error(conn_id, "image switch: reflash failed (CDONE never rose)"); return; }
    if (rc == -5) { signal_engine_psram_report_wedge(); send_error(conn_id, "image switch: iCE40->PSRAM write inoperable after retries (run psram_recover: the pod restarts and reloads the iCE40)"); return; }
    if (rc != 0)  { send_error(conn_id, "image switch failed"); return; }
    /* The live image (and thus dac_control_loop / dac_deep_replay / dac_replay_max_samples) just
       changed — re-announce so the cloud/webapp gate on the NEW image, not the connect-time one. */
    cloud_client_request_caps_resend();
    /* A reconfigure can glitch the DAC's SPI lines: put an output stage back on its park level. */
    if (dac_limits_get()->enabled) (void)dac_limits_park_now();
    char payload[112];
    snprintf(payload, sizeof(payload), "{\"image\":%d,\"name\":\"%s\",\"version\":%u,\"features\":%u}",
             img, signal_engine_image_name(img), ver, feats);
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
