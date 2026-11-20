/* bp_errors.c: the reply line for a stable error code (bp_errors.h, errors/catalog.toml).

   {"status":"error","code":"BP0031","args":{"la":4,"owner":"gpio",...}}\n

   No message: the hosts render the code's catalog message, its fix and its docs link from
   errors/errors.json, so the firmware holds no per-error text. "args" is left out when a code
   has none. One exception: BP0030 (client_too_old) also carries a "message", because the client
   it refuses may be too old to know the code and shows only that text.

   bp_err_note / bp_err_tag keep the code of a refusal a module hands its caller as text
   (pod_policy.c, sensor_mgr.c, la_pins.c and the like): send_error() asks bp_err_noted whether
   the text it is about to send came with a code, and sends the code instead when it did. The
   note matches the text itself (a hash and the length), not the buffer, so a caller that copies
   the text first still gets the code. Pure: no I/O, host-tested in test/test_bp_errors.c. */
#include "bp_errors.h"

#include <stdio.h>
#include <string.h>

#include "bp_json.h"

static void emit_args(bp_emit_t *e, const bp_err_arg_t *args, size_t n) {
    bp_emit_raw(e, ",\"args\":{");
    for (size_t i = 0; i < n; i++) {
        bp_emit(e, "%s\"%s\":", i ? "," : "", args[i].key);
        if (args[i].is_str) bp_emit_jstr(e, args[i].s ? args[i].s : "");
        else                bp_emit(e, "%ld", (long)args[i].i);
    }
    bp_emit_raw(e, "}");
}

static int32_t int_arg(const bp_err_arg_t *args, size_t n, const char *key) {
    for (size_t i = 0; i < n; i++)
        if (!args[i].is_str && !strcmp(args[i].key, key)) return args[i].i;
    return 0;
}

size_t bp_error_format(char *buf, size_t cap, bp_errcode_t code, const bp_err_arg_t *args,
                       size_t n) {
    bp_emit_t e;
    bp_emit_init(&e, buf, cap);
    bp_emit(&e, "{\"status\":\"error\",\"code\":\"BP%04u\"", (unsigned)code);
    if (n) emit_args(&e, args, n);
    if (code == BP_E_CLIENT_TOO_OLD)   /* the one text a client without codes must still see */
        bp_emit(&e, ",\"message\":\"Update the BenchPod SDK, MCP or CLI: this firmware needs client API "
                    "%ld or newer (this client sent %ld)\"",
                (long)int_arg(args, n, "min_client_api"), (long)int_arg(args, n, "client_api"));
    bp_emit_raw(&e, "}\n");
    if (bp_emit_ok(&e)) return bp_emit_len(&e);
    /* Too long (a huge string arg): drop the args, keep the code. Never a torn line: the client
       would block on the missing newline. */
    bp_emit_init(&e, buf, cap);
    bp_emit(&e, "{\"status\":\"error\",\"code\":\"BP%04u\"}\n", (unsigned)code);
    return bp_emit_ok(&e) ? bp_emit_len(&e) : 0;
}

/* ---- the short form: "BP0031 la=4 owner=gpio" ---- */

size_t bp_err_render(char *out, size_t cap, bp_errcode_t code, const bp_err_arg_t *args, size_t n) {
    if (!out || !cap) return 0;
    size_t len = (size_t)snprintf(out, cap, "BP%04u", (unsigned)code);
    for (size_t i = 0; i < n && len < cap; i++) {
        if (args[i].is_str)
            len += (size_t)snprintf(out + len, cap - len, " %s=%s", args[i].key, args[i].s ? args[i].s : "");
        else
            len += (size_t)snprintf(out + len, cap - len, " %s=%ld", args[i].key, (long)args[i].i);
    }
    return len < cap ? len : cap - 1u;   /* snprintf cut it to fit */
}

/* ---- notes ---- */

/* Two notes: a module may build one refusal, then ask another (bus_busy_reason) before its
   caller sends the first. String args are copied into the note's pool, cut to fit. */
#define NOTES      2u
#define NOTE_POOL  72u

typedef struct {
    uint32_t     hash;
    uint16_t     len;
    uint16_t     code;   /* BP_E_NONE = empty */
    uint8_t      n;
    bp_err_arg_t args[BP_ERR_ARGS_MAX];
    char         pool[NOTE_POOL];
} note_t;

static note_t   s_note[NOTES];
static unsigned s_next;

static uint32_t text_hash(const char *s, size_t *len) {
    uint32_t h = 2166136261u;   /* FNV-1a */
    size_t i = 0;
    for (; s[i]; i++) h = (h ^ (uint8_t)s[i]) * 16777619u;
    *len = i;
    return h;
}

static void remember(const char *text, bp_errcode_t code, const bp_err_arg_t *args, size_t n) {
    size_t len;
    uint32_t h = text_hash(text, &len);
    note_t *t = NULL;
    for (unsigned i = 0; i < NOTES && !t; i++)   /* the same text again: refresh its args */
        if (s_note[i].hash == h && s_note[i].len == len) t = &s_note[i];
    if (!t) {
        t = &s_note[s_next];
        s_next = (s_next + 1u) % NOTES;
    }
    memset(t, 0, sizeof(*t));
    t->hash = h;
    t->len  = (uint16_t)len;
    t->code = (uint16_t)code;
    size_t used = 0;
    for (size_t i = 0; i < n && i < BP_ERR_ARGS_MAX; i++) {
        t->args[i] = args[i];
        if (!args[i].is_str) continue;
        const char *s = args[i].s ? args[i].s : "";
        size_t l = strlen(s), room = NOTE_POOL - used;
        if (room == 0) { t->args[i].s = ""; continue; }
        if (l > room - 1u) l = room - 1u;
        memcpy(t->pool + used, s, l);
        t->pool[used + l] = 0;
        t->args[i].s = t->pool + used;
        used += l + 1u;
    }
    t->n = (uint8_t)(n < BP_ERR_ARGS_MAX ? n : BP_ERR_ARGS_MAX);
}

const char *bp_err_note(char *buf, size_t cap, bp_errcode_t code, const bp_err_arg_t *args,
                        size_t n) {
    if (!buf || !cap) return buf;
    bp_err_render(buf, cap, code, args, n);
    remember(buf, code, args, n);
    return buf;
}

const char *bp_err_tag(const char *text, bp_errcode_t code, const bp_err_arg_t *args, size_t n) {
    if (text) remember(text, code, args, n);
    return text;
}

const bp_err_arg_t *bp_err_noted(const char *text, bp_errcode_t *code, size_t *n) {
    if (!text) return NULL;
    size_t len;
    uint32_t h = text_hash(text, &len);
    for (unsigned i = 0; i < NOTES; i++) {
        const note_t *t = &s_note[i];
        if (t->code == BP_E_NONE || t->hash != h || t->len != len) continue;
        if (code) *code = (bp_errcode_t)t->code;
        if (n) *n = t->n;
        return t->args;
    }
    return NULL;
}
