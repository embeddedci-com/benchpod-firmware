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
#include "can_timing.h"
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
 * CAN (FDCAN1 / TCAN1044) — classic CAN and CAN FD over the on-board transceiver.
 * All single-reply (cloud-safe).  Loopback modes let a lone pod self-test.
 * ========================================================================== */

/* CAN byte-array fields ("data"/"reply_data") parse via the shared
   bp_json_byte_array (aliased to json_get_byte_array at the top of this file). */

/* Parse the frame fields shared by can_write and can_respond: data bytes (up
   to 64), the FD flags.  `fd` defaults to the bus (FD frames on an FD bus),
   `brs` to on.  Returns 0, or -1 with *err set. */
static int can_parse_frame(const char *json, const char *data_key, const char *fd_key,
                           const char *brs_key, can_frame_t *f, const char **err) {
    uint8_t data[CAN_FD_MAX_LEN + 1] = {0};
    int len = 0;
    json_get_byte_array(json, data_key, data, (int)sizeof(data), &len);
    can_status_t st; can_get_status(&st);
    const bool bus_fd = st.enabled && st.data_bitrate != 0u;
    char tmp[8];
    f->fd  = json_get_value(json, fd_key, tmp, sizeof(tmp)) ? json_flag(json, fd_key)
                                                            : (bus_fd && !f->rtr);
    f->brs = json_get_value(json, brs_key, tmp, sizeof(tmp)) ? json_flag(json, brs_key) : true;
    if (f->fd && f->rtr) { *err = "rtr needs a classic frame (CAN FD has no remote frames)"; return -1; }
    if (len > (int)CAN_FD_MAX_LEN) { *err = "data is longer than 64 bytes"; return -1; }
    if (len > 8 && !f->fd) {
        *err = bus_fd ? "more than 8 data bytes needs an FD frame"
                      : "more than 8 data bytes needs CAN FD (can_config with data_bitrate)";
        return -1;
    }
    f->dlc = (uint8_t)len;
    memcpy(f->data, data, (size_t)len);
    return 0;
}

/* can_config — bring FDCAN1 up.  {"cmd":"can_config","bitrate":500000,
   "data_bitrate":2000000,"mode":"internal|external|normal|listen",
   "term":true|false}.  Defaults: bitrate 500000, classic CAN (no
   data_bitrate), mode normal, term off.  A data_bitrate turns CAN FD on, with
   bit-rate switching when it is above the bitrate. */
void handle_can_config(int conn_id, const char *json) {
    char br_s[12] = {0}, dbr_s[12] = {0}, mode_s[12] = {0};
    uint32_t bitrate = 500000u, data_bitrate = 0u;
    if (json_get_value(json, "bitrate", br_s, sizeof(br_s)))
        bitrate = (uint32_t)strtoul(br_s, NULL, 0);
    if (json_get_value(json, "data_bitrate", dbr_s, sizeof(dbr_s)))
        data_bitrate = (uint32_t)strtoul(dbr_s, NULL, 0);
    can_mode_t mode = CAN_MODE_NORMAL;
    if (json_get_value(json, "mode", mode_s, sizeof(mode_s)) &&
        can_mode_from_name(mode_s, &mode) != 0) {
        send_error(conn_id, "mode must be normal|internal|external|listen");
        return;
    }
    bool term = json_flag(json, "term");
    int rc = can_configure(bitrate, data_bitrate, mode, term);
    if (rc == -1) { send_error(conn_id, "unsupported bitrate (no exact bit timing)"); return; }
    if (rc == -3) {
        send_error(conn_id, "unsupported data_bitrate (no exact bit timing at 80 MHz; "
                            "1, 2, 4, 5 and 8 Mbit/s work)");
        return;
    }
    if (rc == -4) { send_error(conn_id, "data_bitrate must be from bitrate up to 8000000"); return; }
    if (rc != 0)  { send_error(conn_id, "can init failed"); return; }
    char payload[128];
    snprintf(payload, sizeof(payload),
             "{\"bitrate\":%lu,\"data_bitrate\":%lu,\"fd\":%s,\"mode\":\"%s\",\"term\":%s}",
             (unsigned long)bitrate, (unsigned long)data_bitrate,
             data_bitrate ? "true" : "false", can_mode_name(mode), term ? "true" : "false");
    send_ok_str(conn_id, payload);
}

/* can_write — queue one frame.
   {"cmd":"can_write","id":291,"ext":false,"rtr":false,"data":[1,2,3]}
   On a CAN FD bus frames are FD with bit-rate switching unless "fd":false
   (classic frame) or "brs":false. */
void handle_can_write(int conn_id, const char *json) {
    char id_s[12] = {0};
    if (!json_get_value(json, "id", id_s, sizeof(id_s))) { send_error(conn_id, "missing id"); return; }
    can_frame_t f = {0};
    f.id  = (uint32_t)strtoul(id_s, NULL, 0);
    f.ext = json_flag(json, "ext");
    f.rtr = json_flag(json, "rtr");
    const char *err = NULL;
    if (can_parse_frame(json, "data", "fd", "brs", &f, &err) != 0) { send_error(conn_id, err); return; }
    int rc = can_tx(&f);
    if (rc == -1) { send_error(conn_id, "can not enabled"); return; }
    if (rc == -2) { send_error(conn_id, "bus off"); return; }
    if (rc == -4) { send_error(conn_id, "invalid frame"); return; }
    if (rc != 0)  { send_error(conn_id, "tx failed (fifo full?)"); return; }
    /* dlc: bytes on the bus (an FD length is padded up to 12, 16, ... 64). */
    uint8_t len = f.dlc;
    if (f.fd) {
        int code = can_len_to_dlc(len, true);
        if (code >= 0) len = can_dlc_to_len((uint32_t)code);
    }
    can_status_t st; can_get_status(&st);
    const bool brs = f.fd && f.brs && st.data_bitrate > st.bitrate;
    char payload[96];
    snprintf(payload, sizeof(payload), "{\"id\":%lu,\"ext\":%s,\"dlc\":%u,\"fd\":%s,\"brs\":%s}",
             (unsigned long)f.id, f.ext ? "true" : "false", (unsigned)len,
             f.fd ? "true" : "false", brs ? "true" : "false");
    send_ok_str(conn_id, payload);
}

/* Room for one can_read frame object: 64 bytes x "255," plus the fields. */
#define CAN_READ_FRAME_MAX  (CAN_FD_MAX_LEN * 4u + 128u)
/* The whole reply must fit a cloud command reply (BP_CLOUD_REPLY_MAX = 1792)
   with room for the header and trailer: stop adding frames before that and
   leave the rest queued ("pending" says how many). */
#define CAN_READ_REPLY_MAX  1650u

static size_t can_frame_json(char *out, size_t cap, const can_frame_t *f, bool first) {
    size_t off = 0;
    off += (size_t)snprintf(out + off, cap - off,
                            "%s{\"id\":%lu,\"ext\":%s,\"rtr\":%s,\"fd\":%s,\"brs\":%s,\"dlc\":%d,\"data\":[",
                            first ? "" : ",", (unsigned long)f->id,
                            f->ext ? "true" : "false", f->rtr ? "true" : "false",
                            f->fd ? "true" : "false", f->brs ? "true" : "false", f->dlc);
    for (int b = 0; b < f->dlc && off < cap; b++)
        off += (size_t)snprintf(out + off, cap - off, "%s%u", b ? "," : "", f->data[b]);
    if (off < cap)
        off += (size_t)snprintf(out + off, cap - off, "],\"ts\":%lu}", (unsigned long)f->ts);
    return (off < cap) ? off : cap - 1u;
}

/* can_read — drain up to `max` (default 8, capped 8) received frames.
   {"cmd":"can_read","max":8} -> {"frames":[...],"overflow":N,"pending":N}.
   Fewer frames than asked for come back when 64-byte FD frames would make the
   reply too long for a cloud reply; the rest stay queued.  The reply goes out
   one frame at a time (a cloud reply is collected by cloud_reply_cap), so no
   reply-sized buffer is needed. */
void handle_can_read(int conn_id, const char *json) {
    char max_s[8] = {0};
    int want = 8;
    if (json_get_value(json, "max", max_s, sizeof(max_s))) want = atoi(max_s);
    if (want < 1) want = 1;
    if (want > 8) want = 8;

    static const char head[] = "{\"status\":\"ok\",\"data\":{\"frames\":[";
    char one[CAN_READ_FRAME_MAX];
    if (at_send_data(conn_id, (const uint8_t *)head, sizeof(head) - 1u) != 0) {
        at_close_connection(conn_id);
        return;
    }
    size_t total = sizeof(head) - 1u;
    int n = 0;
    can_frame_t fr;
    while (n < want && can_rx_peek(&fr)) {
        size_t len = can_frame_json(one, sizeof(one), &fr, n == 0);
        if (total + len > CAN_READ_REPLY_MAX) break;
        (void)can_rx_pop(&fr, 1);
        n++;
        total += len;
        if (at_send_data(conn_id, (const uint8_t *)one, len) != 0) {
            at_close_connection(conn_id);
            return;
        }
    }
    can_status_t st; can_get_status(&st);
    int len = snprintf(one, sizeof(one), "],\"overflow\":%lu,\"pending\":%lu}}\n",
                       (unsigned long)st.rx_overflow, (unsigned long)st.rx_pending);
    if (at_send_data(conn_id, (const uint8_t *)one, (size_t)len) != 0) {
        at_close_connection(conn_id);
    }
}

void handle_can_status(int conn_id, const char *json) {
    (void)json;
    can_status_t s; can_get_status(&s);
    char resp[400];
    snprintf(resp, sizeof(resp),
             "{\"status\":\"ok\",\"data\":{"
             "\"enabled\":%s,\"mode\":\"%s\",\"bitrate\":%lu,\"data_bitrate\":%lu,\"fd\":%s,\"term\":%s,"
             "\"tec\":%u,\"rec\":%u,\"bus_off\":%s,\"error_passive\":%s,"
             "\"rx_pending\":%lu,\"rx_overflow\":%lu,"
             "\"responder_rules\":%lu,\"responder_hits\":%lu,\"bus_off_recoveries\":%lu}}\n",
             s.enabled ? "true" : "false", can_mode_name(s.mode),
             (unsigned long)s.bitrate, (unsigned long)s.data_bitrate,
             s.data_bitrate ? "true" : "false", s.term ? "true" : "false",
             s.tec, s.rec, s.bus_off ? "true" : "false",
             s.error_passive ? "true" : "false",
             (unsigned long)s.rx_pending, (unsigned long)s.rx_overflow,
             (unsigned long)s.responder_rules, (unsigned long)s.responder_hits,
             (unsigned long)s.bus_off_recoveries);
    if (at_send_data(conn_id, (const uint8_t *)resp, strlen(resp)) != 0) {
        at_close_connection(conn_id);
    }
}

/* can_respond — autonomous ECU simulation. Add a rule so the firmware auto-
   replies (from the RX ISR) to a matching request:
   {"cmd":"can_respond","match_id":291,"ext":false,"reply_id":292,
    "reply_ext":false,"reply_data":[0x50,0x03]}
   On a CAN FD bus the reply is an FD frame unless "reply_fd":false;
   "reply_brs":false turns bit-rate switching off for it.
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
    const char *err = NULL;
    if (can_parse_frame(json, "reply_data", "reply_fd", "reply_brs", &reply, &err) != 0) {
        send_error(conn_id, err);
        return;
    }
    uint32_t match_id = (uint32_t)strtoul(mid_s, NULL, 0);
    bool match_ext = json_flag(json, "ext");
    int idx = can_responder_add(match_id, match_ext, &reply);
    if (idx == -2) { send_error(conn_id, "invalid reply frame"); return; }
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

void handle_can_disable(int conn_id, const char *json) {
    (void)json;
    can_disable();
    send_ok_str(conn_id, "{\"enabled\":false}");
}
