/* command_handler_wiring.c: the wiring profiles stored on the pod (wiring_store.h).
 *
 *   {"cmd":"wiring_list"}
 *     -> {"default":"k66"|null,"profiles":[{"name":"k66","rev":3,"bytes":812},...],
 *         "max_profiles":16,"max_bytes":4096,"free_bytes":15120}
 *   {"cmd":"wiring_get","name":"k66"[,"offset":0]}
 *     -> {"name":"k66","rev":3,"total":812,"offset":0,"data_b64":"..."}   (<= 600 bytes a time)
 *   {"cmd":"wiring_put","name":"k66","offset":0,"total":812,"data_b64":"..."[,"if_revision":3]}
 *     -> {"name":"k66","received":600,"total":812} while pieces are missing, then
 *        {"name":"k66","rev":4,"default":false} once the last piece is in and saved
 *   {"cmd":"wiring_delete","name":"k66"}          -> {"name":"k66","deleted":true}
 *   {"cmd":"wiring_default"[,"name":"k66"]}       -> {"default":"k66"|null}
 *
 * A profile is opaque JSON to the pod: the hosts validate it. Pieces of at most 600 bytes keep a
 * reply and a command inside the cloud channel's limits (BP_CLOUD_REPLY_MAX, BP_CLOUD_CMD_IN_MAX).
 * if_revision on wiring_put: the revision the sender loaded (0 = the name must be new); when the
 * pod's copy moved on meanwhile the save is refused (BP0009) instead of overwriting it. Without
 * it the save always goes through. Reads never touch the bus; writes refuse while the DAC runs or
 * a capture or upload holds the bus (BP0016). One upload at a time for the whole pod: a new
 * offset-0 piece replaces an unfinished one (at most WIRING_PROFILE_MAX bytes held meanwhile).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "at_driver.h"
#include "b64url.h"
#include "bp_errors.h"
#include "bp_json.h"
#include "command_handler_internal.h"
#include "sensor_port.h"
#include "wiring_store.h"

#define WIRING_PIECE_MAX 600u

static bool get_long(const char *json, const char *key, long *out) {
    char v[16];
    if (!bp_json_get(json, key, v, sizeof(v))) return false;
    char *end;
    *out = strtol(v, &end, 10);
    return end != v && *end == 0;
}

static bool get_name(int conn_id, const char *json, char *name, size_t cap) {
    int r = bp_json_get_fit(json, "name", name, cap);
    if (r == 0) { bp_err_missing_argument(conn_id, BP_ERR_TEXT("missing name"), "name"); return false; }
    if (r < 0 || !wiring_name_ok(name)) {
        bp_err_wiring_bad_name(conn_id, BP_ERR_TEXT("bad profile name: 1-40 characters, a-z 0-9 and -"),
                               name);
        return false;
    }
    return true;
}

static void send_line(int conn_id, bp_emit_t *e) {
    if (at_send_data(conn_id, (const uint8_t *)e->buf, bp_emit_len(e)) != 0) at_close_connection(conn_id);
}

/* A wiring_store result as a reply. */
static void send_refusal(int conn_id, wiring_result_t why, const char *name, size_t bytes) {
    switch (why) {
    case WIRING_E_NAME:       bp_err_wiring_bad_name(conn_id, BP_ERR_TEXT("bad profile name"), name); break;
    case WIRING_E_NOT_FOUND:  bp_err_wiring_profile_not_found(conn_id, BP_ERR_TEXT("no such profile"), name); break;
    case WIRING_E_FULL:       bp_err_wiring_store_full(conn_id, BP_ERR_TEXT("wiring profiles full"),
                                                       (int32_t)WIRING_MAX_PROFILES); break;
    case WIRING_E_TOO_LARGE:  bp_err_wiring_profile_too_large(conn_id, BP_ERR_TEXT("profile too large"),
                                                              (int32_t)bytes, (int32_t)WIRING_PROFILE_MAX); break;
    case WIRING_E_DEFAULT:    bp_err_wiring_delete_default(conn_id, BP_ERR_TEXT("cannot delete the default profile"),
                                                           name); break;
    case WIRING_E_NOT_OBJECT: bp_err_wiring_not_json_object(conn_id, BP_ERR_TEXT("profile is not a JSON object")); break;
    case WIRING_OK:           break;
    }
}

/* Writes need the bus: never while the DAC runs or a capture or upload holds it. */
static bool storage_free(int conn_id) {
    if (heavy_or_claimed() || sensor_port_dac_running()) {
        bp_err_storage_busy(conn_id, BP_ERR_TEXT("busy: stop the DAC or wait for the capture, then save"));
        return false;
    }
    return true;
}

static bool apply(int conn_id, const wiring_op_t *op, size_t bytes) {
    wiring_result_t why;
    switch (wiring_store_apply(op, &why)) {
    case WIRING_STORE_OK:      return true;
    case WIRING_STORE_REFUSED: send_refusal(conn_id, why, op->name, bytes); return false;
    case WIRING_STORE_IO:      bp_err_wiring_storage_failed(conn_id, BP_ERR_TEXT("wiring storage failed")); return false;
    }
    return false;
}

void handle_wiring_list(int conn_id, const char *json) {
    (void)json;
    const wiring_index_t *idx = wiring_store_index();
    const size_t cap = 1600;   /* 16 x ~75 B + the frame: within BP_CLOUD_REPLY_MAX */
    char *resp = pvPortMalloc(cap);   /* heap, not static: RAM is full */
    if (!resp) { bp_err_wiring_storage_failed(conn_id, BP_ERR_TEXT("out of memory")); return; }
    bp_emit_t e;
    bp_emit_init(&e, resp, cap);
    bp_emit_raw(&e, "{\"status\":\"ok\",\"data\":{\"default\":");
    if (idx->def != WIRING_NO_DEFAULT) bp_emit(&e, "\"%s\"", idx->e[idx->def].name);
    else bp_emit_raw(&e, "null");
    bp_emit_raw(&e, ",\"profiles\":[");
    for (uint8_t i = 0; i < idx->count; i++)
        bp_emit(&e, "%s{\"name\":\"%s\",\"rev\":%lu,\"bytes\":%u}", i ? "," : "", idx->e[i].name,
                (unsigned long)idx->e[i].rev, (unsigned)idx->e[i].len);
    bp_emit(&e, "],\"max_profiles\":%u,\"max_bytes\":%u,\"free_bytes\":%lu}}\n",
            (unsigned)WIRING_MAX_PROFILES, (unsigned)WIRING_PROFILE_MAX,
            (unsigned long)(WIRING_DOC_MAX - (idx->doc_len ? idx->doc_len : WIRING_DOC_HDR)));
    send_line(conn_id, &e);
    vPortFree(resp);
}

void handle_wiring_get(int conn_id, const char *json) {
    char name[WIRING_NAME_MAX + 1];
    if (!get_name(conn_id, json, name, sizeof(name))) return;
    const wiring_index_t *idx = wiring_store_index();
    int i = wiring_index_find(idx, name);
    if (i < 0) { bp_err_wiring_profile_not_found(conn_id, BP_ERR_TEXT("no such profile"), name); return; }
    long offset = 0;
    char probe[16];
    if (bp_json_get(json, "offset", probe, sizeof(probe)) &&
        (!get_long(json, "offset", &offset) || offset < 0 || offset > (long)idx->e[i].len)) {
        bp_err_missing_argument(conn_id, BP_ERR_TEXT("offset must be 0..total"), "offset");
        return;
    }
    uint32_t n = idx->e[i].len - (uint32_t)offset;
    if (n > WIRING_PIECE_MAX) n = WIRING_PIECE_MAX;
    const size_t b64_cap = B64URL_ENCODED_LEN(WIRING_PIECE_MAX) + 4;
    const size_t cap = b64_cap + 192;
    char *resp = pvPortMalloc(cap + b64_cap);   /* heap, not static: RAM is full */
    if (!resp) { bp_err_wiring_storage_failed(conn_id, BP_ERR_TEXT("out of memory")); return; }
    char *b64 = resp + cap;
    size_t bl = b64url_encode(wiring_store_doc() + idx->e[i].off + offset, n, b64, b64_cap);
    bp_emit_t e;
    bp_emit_init(&e, resp, cap);
    bp_emit(&e, "{\"status\":\"ok\",\"data\":{\"name\":\"%s\",\"rev\":%lu,\"total\":%u,\"offset\":%ld,"
                "\"data_b64\":\"%.*s\"}}\n",
            name, (unsigned long)idx->e[i].rev, (unsigned)idx->e[i].len, offset, (int)bl, b64);
    send_line(conn_id, &e);
    vPortFree(resp);
}

/* One upload at a time, for the whole pod: a new offset-0 piece replaces an unfinished one. */
static struct {
    uint8_t *buf;
    char     name[WIRING_NAME_MAX + 1];
    uint32_t total, received;
    long     if_rev;           /* -1: no check */
    int      conn_id;
} s_up;

static void upload_drop(void) {
    if (s_up.buf) vPortFree(s_up.buf);
    memset(&s_up, 0, sizeof(s_up));
}

static void put_piece(int conn_id, const char *json, const char *name, long offset, long total, char *b64,
                      size_t b64_cap);

void handle_wiring_put(int conn_id, const char *json) {
    char name[WIRING_NAME_MAX + 1];
    if (!get_name(conn_id, json, name, sizeof(name))) return;
    long offset, total;
    if (!get_long(json, "offset", &offset) || offset < 0) {
        bp_err_missing_argument(conn_id, BP_ERR_TEXT("missing offset"), "offset"); return;
    }
    if (!get_long(json, "total", &total) || total < 2) {
        bp_err_missing_argument(conn_id, BP_ERR_TEXT("missing total"), "total"); return;
    }
    if (total > (long)WIRING_PROFILE_MAX) {
        bp_err_wiring_profile_too_large(conn_id, BP_ERR_TEXT("profile too large"), (int32_t)total,
                                        (int32_t)WIRING_PROFILE_MAX);
        return;
    }
    const size_t b64_cap = B64URL_ENCODED_LEN(WIRING_PIECE_MAX) + 8;
    char *b64 = pvPortMalloc(b64_cap);   /* heap, not static: RAM is full */
    if (!b64) { bp_err_wiring_storage_failed(conn_id, BP_ERR_TEXT("out of memory")); return; }
    put_piece(conn_id, json, name, offset, total, b64, b64_cap);
    vPortFree(b64);
}

static void put_piece(int conn_id, const char *json, const char *name, long offset, long total, char *b64,
                      size_t b64_cap) {
    if (bp_json_get_fit(json, "data_b64", b64, b64_cap) != 1) {
        bp_err_missing_argument(conn_id, BP_ERR_TEXT("missing data_b64 (at most 600 bytes a piece)"), "data_b64");
        return;
    }
    if (offset == 0) {
        upload_drop();
        s_up.buf = pvPortMalloc((size_t)total);
        if (!s_up.buf) { bp_err_wiring_storage_failed(conn_id, BP_ERR_TEXT("out of memory")); return; }
        strcpy(s_up.name, name);
        s_up.total = (uint32_t)total;
        s_up.conn_id = conn_id;
        if (!get_long(json, "if_revision", &s_up.if_rev)) s_up.if_rev = -1;
    } else if (!s_up.buf || strcmp(s_up.name, name) != 0 || s_up.total != (uint32_t)total ||
               s_up.received != (uint32_t)offset) {
        long expected = (s_up.buf && !strcmp(s_up.name, name)) ? (long)s_up.received : 0;
        upload_drop();
        bp_err_wiring_upload_out_of_order(conn_id, BP_ERR_TEXT("upload out of order: start again at offset 0"),
                                          (int32_t)expected);
        return;
    }
    size_t got = 0;
    if (b64url_decode(b64, s_up.buf + s_up.received, s_up.total - s_up.received, &got) != 0 || got == 0 ||
        got > WIRING_PIECE_MAX) {
        upload_drop();
        bp_err_missing_argument(conn_id, BP_ERR_TEXT("data_b64 is not base64url, or runs past total"), "data_b64");
        return;
    }
    s_up.received += (uint32_t)got;
    char resp[160];
    bp_emit_t e;
    bp_emit_init(&e, resp, sizeof(resp));
    if (s_up.received < s_up.total) {
        bp_emit(&e, "{\"status\":\"ok\",\"data\":{\"name\":\"%s\",\"received\":%lu,\"total\":%lu}}\n", name,
                (unsigned long)s_up.received, (unsigned long)s_up.total);
        send_line(conn_id, &e);
        return;
    }
    /* The last piece: check the revision, then save. */
    const wiring_index_t *idx = wiring_store_index();
    int at = wiring_index_find(idx, name);
    long cur = at < 0 ? 0 : (long)idx->e[at].rev;
    if (s_up.if_rev >= 0 && s_up.if_rev != cur) {
        upload_drop();
        bp_err_wiring_revision_conflict(conn_id, BP_ERR_TEXT("profile changed since it was loaded"), name,
                                        (int32_t)cur);
        return;
    }
    if (!storage_free(conn_id)) { upload_drop(); return; }
    wiring_op_t op = { WIRING_OP_PUT, name, s_up.buf, (uint16_t)s_up.total };
    bool ok = apply(conn_id, &op, s_up.total);
    upload_drop();
    if (!ok) return;
    idx = wiring_store_index();
    at = wiring_index_find(idx, name);
    bp_emit(&e, "{\"status\":\"ok\",\"data\":{\"name\":\"%s\",\"rev\":%lu,\"default\":%s}}\n", name,
            (unsigned long)idx->e[at].rev, (uint8_t)at == idx->def ? "true" : "false");
    send_line(conn_id, &e);
}

void handle_wiring_delete(int conn_id, const char *json) {
    char name[WIRING_NAME_MAX + 1];
    if (!get_name(conn_id, json, name, sizeof(name))) return;
    if (!storage_free(conn_id)) return;
    wiring_op_t op = { WIRING_OP_DELETE, name, NULL, 0 };
    if (!apply(conn_id, &op, 0)) return;
    char payload[96];
    snprintf(payload, sizeof(payload), "{\"name\":\"%s\",\"deleted\":true}", name);
    send_ok_str(conn_id, payload);
}

void handle_wiring_default(int conn_id, const char *json) {
    char name[WIRING_NAME_MAX + 1];
    if (bp_json_get_fit(json, "name", name, sizeof(name)) != 0) {
        if (!get_name(conn_id, json, name, sizeof(name))) return;
        const wiring_index_t *idx = wiring_store_index();
        int at = wiring_index_find(idx, name);
        if (at < 0) { bp_err_wiring_profile_not_found(conn_id, BP_ERR_TEXT("no such profile"), name); return; }
        if ((uint8_t)at != idx->def) {   /* already the default: nothing to write */
            if (!storage_free(conn_id)) return;
            wiring_op_t op = { WIRING_OP_DEFAULT, name, NULL, 0 };
            if (!apply(conn_id, &op, 0)) return;
        }
    }
    const wiring_index_t *idx = wiring_store_index();
    char payload[80];
    if (idx->def != WIRING_NO_DEFAULT) snprintf(payload, sizeof(payload), "{\"default\":\"%s\"}", idx->e[idx->def].name);
    else snprintf(payload, sizeof(payload), "{\"default\":null}");
    send_ok_str(conn_id, payload);
}
