/*
 * command_handler_recovery.c — the JSON commands that took over from the USB console in
 * firmware 4.0.0, when the console became a recovery tool only (docs/usb-serial-interface.md):
 *
 *   identity_wipe   erase the device key and make a new one (USB link only: physical presence)
 *   usb_reattach    drop off the USB bus and enumerate again (macOS and a locked screen)
 *   wifi_bridge     the external Wi-Fi bridge: update, reboot, factory_reset, cert (cert: USB link)
 *   wifi_reflash    write the Wi-Fi module's firmware again from the pod's esp slot
 *
 * The console had these as identity-wipe, usb-reattach, bridge and flash-esp32.
 */
#include "command_handler.h"
#include "command_handler_internal.h"
#include "bp_errors.h"
#include "bp_json.h"
#include "net_server.h"        /* net_conn_on_usb, net_cloud_reload_after_reply, net_wifi_hold_for_flash */
#include "device_identity.h"
#include "usb_device.h"
#include "esp_wifi_ctrl.h"
#include "esp_rom_flash.h"
#include "esp_rom_job.h"       /* esp_rom_result_str */
#include "blob_store.h"
#include "bp_log.h"
#if defined(BENCHPOD_WIFI_BRIDGE)
#include "wifi_bridge_net.h"
#endif

#include <stdio.h>
#include <string.h>

/* {"cmd":"identity_wipe","confirm":"<short id>"|"unknown"} -> {"short_id":"<new>"}: the way back
   for a pod whose identity record the firmware will not replace. Over the USB link only (a person
   at the pod, as the console was); the token names the identity being wiped (device_identity.h).
   The cloud then needs the pod registered again. */
void handle_identity_wipe(int conn_id, const char *json) {
    if (!net_conn_on_usb(conn_id)) { bp_err_setting_device_only(conn_id, "identity_wipe"); return; }
    char confirm[16] = {0};
    if (!bp_json_get(json, "confirm", confirm, sizeof(confirm)) || !confirm[0]) {
        send_error(conn_id, "identity_wipe: confirm with the pod's short id (identity_public), or unknown");
        return;
    }
    const char *why = device_identity_wipe(confirm);
    if (why) {
        char m[160];
        snprintf(m, sizeof(m), "identity_wipe: %s", why);
        send_error(conn_id, m);
        return;
    }
    char id[DEVICE_ID_SHORT_LEN + 1], p[40];
    device_identity_short_id(id);
    snprintf(p, sizeof(p), "{\"short_id\":\"%s\"}", id);
    send_ok_str(conn_id, p);
    net_cloud_reload_after_reply();   /* log in again with the new key */
}

/* {"cmd":"usb_reattach"} -> {"reattach":true}: about 0.4 s after the reply the pod drops off the
   USB bus for half a second and connects again, so the computer enumerates the console and the
   network adapter anew (macOS ignores an adapter that appeared while the screen was locked). */
void handle_usb_reattach(int conn_id, const char *json) {
    (void)json;
    send_ok_str(conn_id, "{\"reattach\":true}");
    usb_reattach_request();
}

/* ---- wifi_reflash: the Wi-Fi module's firmware, written again from the W25Q esp slot ---- */

static void wifi_reflash_done(int result) {
    if (result >= 0) LOG_REPLY("wifi_reflash: the Wi-Fi module was flashed and booted (%s)\n", esp_rom_result_str(result));
    else             LOG_REPLY("wifi_reflash: failed, %s (the module is left in reset)\n", esp_rom_result_str(result));
    net_wifi_hold_for_flash(false);   /* the Wi-Fi control starts the module again */
}

/* {"cmd":"wifi_reflash"} -> {"started":true,"bytes":N}: about 2 minutes in the background, while
   the API keeps answering; wifi_status shows the module coming back. The automatic install does
   the same when the module runs another image; this is the manual repair. */
void handle_wifi_reflash(int conn_id, const char *json) {
    (void)json;
    if (!esp_wifi_ctrl_module_present()) { send_error(conn_id, "wifi_reflash: " ESP_WIFI_NO_MODULE); return; }
    if (esp_rom_flash_busy()) { bp_err_update_in_progress(conn_id, "esp", "the pod"); return; }
    const char *busy = bus_busy_reason();   /* ~140 s on the shared bus */
    if (busy) { bp_err_storage_bus_busy(conn_id, busy); return; }
    net_wifi_hold_for_flash(true);   /* the Wi-Fi control must not drive EN/BOOT meanwhile */
    int rc = esp_rom_flash_start(wifi_reflash_done);
    if (rc != 0) {
        net_wifi_hold_for_flash(false);
        send_error(conn_id, rc == ESP_FLASH_START_NO_IMAGE ? "wifi_reflash: no Wi-Fi module image on the pod (install the esp blob)"
                          : rc == ESP_FLASH_START_BAD_SLOT ? "wifi_reflash: the esp slot does not match its checksum"
                          : "wifi_reflash: a flash is already running");
        return;
    }
    char p[48];
    snprintf(p, sizeof(p), "{\"started\":true,\"bytes\":%lu}", (unsigned long)blob_store_info(BLOB_ESP)->len);
    send_ok_str(conn_id, p);
}

/* ---- wifi_bridge: the external Wi-Fi bridge box on the Ethernet port ---------------------- */

#if defined(BENCHPOD_WIFI_BRIDGE)
static bool bridge_refused(int conn_id, int rc, const char *err) {
    char m[128];
    if (rc != 0) snprintf(m, sizeof(m), "Wi-Fi bridge: %s", err ? err : "no reply");
    else if (g_wifi_bridge_reply.result != BWP_OK)
        snprintf(m, sizeof(m), "Wi-Fi bridge: %s%s%s", wb_result_str(g_wifi_bridge_reply.result),
                 g_wifi_bridge_reply.message[0] ? ": " : "", g_wifi_bridge_reply.message);
    else return false;
    send_error(conn_id, m);
    return true;
}

/* The identity certificate is 128 bytes, more than a command line: four parts of 32 bytes (64 hex
   digits) in any order, sent to the bridge once all four are in. */
static void bridge_cert(int conn_id, const char *json) {
    if (!net_conn_on_usb(conn_id)) { bp_err_setting_device_only(conn_id, "wifi_bridge cert"); return; }
    static uint8_t t[3 + BWP_ID_CERT_LEN];   /* ID_CERT TLV */
    static uint8_t parts;                    /* bit per part received */
    char part_s[4] = {0}, hex[68] = {0};
    int part = bp_json_get(json, "part", part_s, sizeof(part_s)) ? part_s[0] - '0' : 0;
    bool good = part >= 1 && part <= 4 && part_s[1] == '\0' &&
                bp_json_get_fit(json, "hex", hex, sizeof(hex)) == 1 && strlen(hex) == 64;
    for (unsigned k = 0; good && k < 32; k++) {
        int hi = bp_hex_nibble(hex[2 * k]), lo = bp_hex_nibble(hex[2 * k + 1]);
        good = hi >= 0 && lo >= 0;
        t[3 + 32 * (part - 1) + k] = (uint8_t)(hi << 4 | lo);
    }
    if (!good) { send_error(conn_id, "wifi_bridge cert: part 1-4 and hex, 64 hex digits"); return; }
    parts |= (uint8_t)(1u << (part - 1));
    char p[48];
    if (parts != 0x0F) {
        snprintf(p, sizeof(p), "{\"part\":%d,\"parts\":%u}", part, (unsigned)parts);
        send_ok_str(conn_id, p);
        return;
    }
    parts = 0;
    t[0] = BWP_T_ID_CERT, t[1] = 0, t[2] = BWP_ID_CERT_LEN;
    const char *err = NULL;
    int rc = wifi_bridge_call(BWP_SET_ID_CERT, t, sizeof(t), 2000, &g_wifi_bridge_reply, &err);
    if (bridge_refused(conn_id, rc, err)) return;
    send_ok_str(conn_id, "{\"part\":4,\"parts\":15,\"stored\":true}");
}
#endif

/* {"cmd":"wifi_bridge","action":"update"|"reboot"|"factory_reset"|"cert"[,"part":1-4,"hex":".."]}
   The bridge's state is in wifi_status, its scan in wifi_scan. */
void handle_wifi_bridge(int conn_id, const char *json) {
#if defined(BENCHPOD_WIFI_BRIDGE)
    char action[16] = {0};
    bp_json_get(json, "action", action, sizeof(action));
    if (!wifi_bridge_present()) { send_error(conn_id, "wifi_bridge: no Wi-Fi bridge on the Ethernet port"); return; }
    const char *err = NULL;
    int rc;
    if (!strcmp(action, "update")) {
        wifi_bridge_ota_force();   /* the hw worker's loop runs it; wifi_status bridge.update follows it */
        send_ok_str(conn_id, "{\"update\":\"requested\"}");
    } else if (!strcmp(action, "reboot")) {
        rc = wifi_bridge_call(BWP_REBOOT, NULL, 0, 1000, &g_wifi_bridge_reply, &err);
        if (!bridge_refused(conn_id, rc, err)) send_ok_str(conn_id, "{\"rebooting\":true}");
    } else if (!strcmp(action, "factory_reset")) {
        rc = wifi_bridge_call(BWP_FACTORY_RESET, NULL, 0, 1000, &g_wifi_bridge_reply, &err);
        if (!bridge_refused(conn_id, rc, err)) send_ok_str(conn_id, "{\"factory_reset\":true}");
    } else if (!strcmp(action, "cert")) {
        bridge_cert(conn_id, json);
    } else {
        send_error(conn_id, "wifi_bridge: action must be update, reboot, factory_reset or cert");
    }
#else
    (void)json;
    send_error(conn_id, "wifi_bridge: this firmware has no Wi-Fi bridge support");
#endif
}
