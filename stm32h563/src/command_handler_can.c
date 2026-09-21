/*
 * command_handler_can.c — CAN (FDCAN1 / TCAN1044) command handlers.
 *
 * Extracted verbatim from command_handler.c to keep that file focused on the
 * coupled instrument core. These handlers only touch the can_bus.c driver plus
 * the shared reply helpers (command_handler_internal.h); they hold no dispatch or
 * capture state. Dispatch is still driven by dispatch_line() in command_handler.c.
 */
#include "command_handler.h"
#include "command_handler_internal.h"
#include "at_driver.h"
#include "can_bus.h"
#include "bp_json.h"
#include "bp_err.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* Flat-JSON accessors: same aliases command_handler.c uses, kept local so the
   moved handler bodies are byte-for-byte identical. */
#define json_get_value      bp_json_get
#define json_get_byte_array bp_json_byte_array
#define json_flag           bp_json_flag

/* ============================================================================
 * CAN (FDCAN1 / TCAN1044) — classic CAN over the on-board transceiver.
 * All single-reply (cloud-safe).  Loopback modes let a lone pod self-test.
 * ========================================================================== */

/* CAN byte-array fields ("data"/"reply_data") parse via the shared
   bp_json_byte_array (aliased to json_get_byte_array at the top of this file). */

/* can_config — bring FDCAN1 up.  {"cmd":"can_config","bitrate":500000,
   "mode":"internal|external|normal|listen","term":true|false}.  Defaults:
   bitrate 500000, mode normal, term off. */
void handle_can_config(int conn_id, const char *json) {
    char br_s[12] = {0}, mode_s[12] = {0};
    uint32_t bitrate = 500000u;
    if (json_get_value(json, "bitrate", br_s, sizeof(br_s)))
        bitrate = (uint32_t)strtoul(br_s, NULL, 0);
    can_mode_t mode = CAN_MODE_NORMAL;
    if (json_get_value(json, "mode", mode_s, sizeof(mode_s)) &&
        can_mode_from_name(mode_s, &mode) != 0) {
        send_error(conn_id, "mode must be normal|internal|external|listen");
        return;
    }
    bool term = json_flag(json, "term");
    bool fd   = json_flag(json, "fd");
    int rc = can_configure(bitrate, mode, fd, term);
    if (rc == -1) { send_error(conn_id, "unsupported bitrate (no exact bit timing)"); return; }
    if (rc != 0)  { send_error(conn_id, "can init failed"); return; }
    char payload[80];
    snprintf(payload, sizeof(payload),
             "{\"bitrate\":%lu,\"mode\":\"%s\",\"term\":%s}",
             (unsigned long)bitrate, can_mode_name(mode), term ? "true" : "false");
    send_ok_str(conn_id, payload);
}

/* can_write — queue one classic frame.
   {"cmd":"can_write","id":291,"ext":false,"rtr":false,"data":[1,2,3]} */
void handle_can_write(int conn_id, const char *json) {
    char id_s[12] = {0};
    if (!json_get_value(json, "id", id_s, sizeof(id_s))) { send_error(conn_id, "missing id"); return; }
    can_frame_t f = {0};
    f.id  = (uint32_t)strtoul(id_s, NULL, 0);
    f.ext = json_flag(json, "ext");
    f.rtr = json_flag(json, "rtr");
    int len = 0;
    uint8_t data[8] = {0};
    json_get_byte_array(json, "data", data, 8, &len);
    if (len > 8) len = 8;
    f.dlc = (uint8_t)len;
    memcpy(f.data, data, (size_t)len);
    int rc = can_tx(&f);
    if (rc == -1) { send_error(conn_id, "can not enabled"); return; }
    if (rc == -2) { send_error(conn_id, "bus off"); return; }
    if (rc != 0)  { send_error(conn_id, "tx failed (fifo full?)"); return; }
    char payload[64];
    snprintf(payload, sizeof(payload), "{\"id\":%lu,\"ext\":%s,\"dlc\":%d}",
             (unsigned long)f.id, f.ext ? "true" : "false", f.dlc);
    send_ok_str(conn_id, payload);
}

/* can_read — drain up to `max` (default 8, capped 8) received frames.
   {"cmd":"can_read","max":8} -> {"frames":[...],"overflow":N}. */
void handle_can_read(int conn_id, const char *json) {
    char max_s[8] = {0};
    int want = 8;
    if (json_get_value(json, "max", max_s, sizeof(max_s))) want = atoi(max_s);
    if (want < 1) want = 1;
    if (want > 8) want = 8;
    can_frame_t fr[8];
    int n = can_rx_pop(fr, want);
    can_status_t st; can_get_status(&st);

    char resp[1024];
    size_t off = 0;
    off += (size_t)snprintf(resp + off, sizeof(resp) - off,
                            "{\"status\":\"ok\",\"data\":{\"frames\":[");
    for (int i = 0; i < n; i++) {
        off += (size_t)snprintf(resp + off, sizeof(resp) - off,
                                "%s{\"id\":%lu,\"ext\":%s,\"rtr\":%s,\"dlc\":%d,\"data\":[",
                                i ? "," : "", (unsigned long)fr[i].id,
                                fr[i].ext ? "true" : "false",
                                fr[i].rtr ? "true" : "false", fr[i].dlc);
        for (int b = 0; b < fr[i].dlc; b++)
            off += (size_t)snprintf(resp + off, sizeof(resp) - off,
                                    "%s%u", b ? "," : "", fr[i].data[b]);
        off += (size_t)snprintf(resp + off, sizeof(resp) - off,
                                "],\"ts\":%lu}", (unsigned long)fr[i].ts);
    }
    off += (size_t)snprintf(resp + off, sizeof(resp) - off,
                            "],\"overflow\":%lu}}\n", (unsigned long)st.rx_overflow);
    if (at_send_data(conn_id, (const uint8_t *)resp, strlen(resp)) != 0) {
        at_close_connection(conn_id);
    }
}

void handle_can_status(int conn_id) {
    can_status_t s; can_get_status(&s);
    char resp[360];
    snprintf(resp, sizeof(resp),
             "{\"status\":\"ok\",\"data\":{"
             "\"enabled\":%s,\"mode\":\"%s\",\"bitrate\":%lu,\"term\":%s,"
             "\"tec\":%u,\"rec\":%u,\"bus_off\":%s,\"error_passive\":%s,"
             "\"rx_pending\":%lu,\"rx_overflow\":%lu,"
             "\"responder_rules\":%lu,\"responder_hits\":%lu}}\n",
             s.enabled ? "true" : "false", can_mode_name(s.mode),
             (unsigned long)s.bitrate, s.term ? "true" : "false",
             s.tec, s.rec, s.bus_off ? "true" : "false",
             s.error_passive ? "true" : "false",
             (unsigned long)s.rx_pending, (unsigned long)s.rx_overflow,
             (unsigned long)s.responder_rules, (unsigned long)s.responder_hits);
    if (at_send_data(conn_id, (const uint8_t *)resp, strlen(resp)) != 0) {
        at_close_connection(conn_id);
    }
}

/* can_respond — autonomous ECU simulation. Add a rule so the firmware auto-
   replies (from the RX ISR) to a matching request:
   {"cmd":"can_respond","match_id":291,"ext":false,"reply_id":292,
    "reply_ext":false,"reply_data":[0x50,0x03]}
   {"cmd":"can_respond","clear":true}  → remove all rules. */
void handle_can_respond(int conn_id, const char *json) {
    if (json_flag(json, "clear")) {
        can_responder_clear();
        send_ok_str(conn_id, "{\"rules\":0}");
        return;
    }
    char mid_s[12] = {0}, rid_s[12] = {0};
    if (!json_get_value(json, "match_id", mid_s, sizeof(mid_s))) { send_error(conn_id, "missing match_id"); return; }
    if (!json_get_value(json, "reply_id", rid_s, sizeof(rid_s))) { send_error(conn_id, "missing reply_id"); return; }
    can_frame_t reply = {0};
    reply.id  = (uint32_t)strtoul(rid_s, NULL, 0);
    reply.ext = json_flag(json, "reply_ext");
    reply.rtr = json_flag(json, "reply_rtr");
    int len = 0; uint8_t data[8] = {0};
    json_get_byte_array(json, "reply_data", data, 8, &len);
    if (len > 8) len = 8;
    reply.dlc = (uint8_t)len;
    memcpy(reply.data, data, (size_t)len);
    uint32_t match_id = (uint32_t)strtoul(mid_s, NULL, 0);
    bool match_ext = json_flag(json, "ext");
    int idx = can_responder_add(match_id, match_ext, &reply);
    if (idx < 0) { send_error(conn_id, "responder table full"); return; }
    char payload[48];
    snprintf(payload, sizeof(payload), "{\"rule\":%d,\"rules\":%d}", idx, can_responder_count());
    send_ok_str(conn_id, payload);
}

/* can_term — toggle the 120 Ω termination (works even with CAN disabled).
   {"cmd":"can_term","on":true|false}. */
void handle_can_term(int conn_id, const char *json) {
    char on_s[8] = {0};
    if (!json_get_value(json, "on", on_s, sizeof(on_s))) { send_error(conn_id, "missing on"); return; }
    bool on = (on_s[0] == '1' || on_s[0] == 't' || on_s[0] == 'T' ||
               on_s[0] == 'y' || on_s[0] == 'Y');
    can_set_term(on);
    char payload[32];
    snprintf(payload, sizeof(payload), "{\"term\":%s}", on ? "true" : "false");
    send_ok_str(conn_id, payload);
}

void handle_can_disable(int conn_id) {
    can_disable();
    send_ok_str(conn_id, "{\"enabled\":false}");
}
