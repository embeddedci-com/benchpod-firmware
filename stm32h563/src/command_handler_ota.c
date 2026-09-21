/*
 * command_handler_ota.c — firmware OTA (PSRAM-staged) command handlers.
 *
 * Extracted verbatim from command_handler.c. These single-reply handlers drive
 * ota.c and share only the reply helpers + the heavy-op gate query
 * (command_handler_internal.h); dispatch stays in command_handler.c. The bulk
 * image chunks normally arrive as WS ota.* frames, with base64 ota_data commands
 * for LAN testing.
 */
#include "command_handler.h"
#include "command_handler_internal.h"
#include "at_driver.h"
#include "ota.h"
#include "cloud_client.h"   /* frames_seen: wire-level vs staged, for stall triage */
#include "b64url.h"
#include "bp_json.h"
#include "bp_err.h"
#include "bp_limits.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* Flat-JSON accessor alias: same as command_handler.c so the moved bodies match. */
#define json_get_value      bp_json_get

/* ============================================================================
 * OTA (firmware update, PSRAM-staged) — single-reply commands, so they ride the
 * cloud command channel too, but the bulk chunks normally arrive as WS ota.*
 * frames (or, for LAN testing, base64 ota_data commands).  See ota.c.
 * ========================================================================== */

/* Build+send an ota status reply: {state,received,size,error}. */
static void ota_reply(int conn_id) {
    char resp[192];
    bp_emit_t e;
    bp_emit_init(&e, resp, sizeof(resp));
    bp_emit(&e, "{\"status\":\"ok\",\"data\":{\"state\":\"%s\",\"received\":%lu,\"size\":%lu,"
                "\"frames_seen\":%lu,\"error\":",
            ota_state_str(), (unsigned long)ota_received(), (unsigned long)ota_size(),
            (unsigned long)cloud_client_ota_frames_seen());
    bp_emit_jstr(&e, ota_error());
    bp_emit_raw(&e, "}}\n");
    if (!bp_emit_ok(&e)) { send_error(conn_id, bp_err_str(BP_ERR_TOO_LARGE)); return; }
    if (at_send_data(conn_id, (const uint8_t *)resp, bp_emit_len(&e)) != 0)
        at_close_connection(conn_id);
}

/* {"cmd":"ota_begin","size":N,"sha256":"<64 hex>"} */
void handle_ota_begin(int conn_id, const char *json) {
    char size_s[16] = {0}, sha_s[80] = {0};
    if (!json_get_value(json, "size", size_s, sizeof(size_s)) ||
        !json_get_value(json, "sha256", sha_s, sizeof(sha_s))) {
        send_error(conn_id, "missing size/sha256");
        return;
    }
    /* Refuse if a capture/measure/LA is in flight (shares the PSRAM bus). */
    if (heavy_in_flight()) { send_error(conn_id, bp_err_str(BP_ERR_BUSY)); return; }
    uint32_t size = (uint32_t)strtoul(size_s, NULL, 0);
    if (ota_begin(size, sha_s) != 0) { send_error(conn_id, ota_error()); return; }
    ota_reply(conn_id);
}

/* {"cmd":"ota_data","offset":O,"data":"<base64url>"} */
void handle_ota_data(int conn_id, const char *json) {
    char off_s[16] = {0};
    char data_b64[BP_CLOUD_RX_MAX];
    json_get_value(json, "offset", off_s, sizeof(off_s));
    if (!json_get_value(json, "data", data_b64, sizeof(data_b64))) {
        send_error(conn_id, "missing data");
        return;
    }
    uint32_t offset = (uint32_t)strtoul(off_s, NULL, 0);
    static uint8_t raw[B64URL_DECODED_MAX(BP_CLOUD_RX_MAX)];
    size_t rawlen = 0;
    if (b64url_decode(data_b64, raw, sizeof(raw), &rawlen) != 0) {
        send_error(conn_id, "invalid data");
        return;
    }
    if (ota_data(offset, raw, (uint32_t)rawlen) != 0) { send_error(conn_id, ota_error()); return; }
    ota_reply(conn_id);
}

/* {"cmd":"ota_end"} — verify the staged image's SHA-256. */
void handle_ota_end(int conn_id) {
    ota_end();          /* sets state to verified or error */
    ota_reply(conn_id);
}

/* {"cmd":"ota_status"} */
void handle_ota_status(int conn_id) { ota_reply(conn_id); }

/* {"cmd":"ota_abort"} */
void handle_ota_abort(int conn_id) { ota_abort(); ota_reply(conn_id); }

/* {"cmd":"ota_selftest"} — SAFE validation of the RAM-resident flash writer
   against a scratch sector (never the app).  Run + confirm PASS before ota_commit. */
void handle_ota_selftest(int conn_id) {
    if (heavy_in_flight()) { send_error(conn_id, bp_err_str(BP_ERR_BUSY)); return; }
    int rc = ota_commit_selftest();
    if (rc == 0) send_ok_str(conn_id, "{\"selftest\":\"pass\"}");
    else         send_error(conn_id, "ota selftest failed (see console log)");
}

/* {"cmd":"ota_commit"} — write the VERIFIED image to flash and reset (no return
   on success).  Reply is only reached on failure. */
void handle_ota_commit(int conn_id) {
    if (ota_get_state() != OTA_VERIFIED) {
        send_error(conn_id, "no verified image staged");
        return;
    }
    /* Best-effort ack before we disappear into the flash writer + reset. */
    send_ok_str(conn_id, "{\"committing\":true}");
    ota_commit();       /* does not return on success */
    send_error(conn_id, ota_error());   /* only reached if commit refused */
}
