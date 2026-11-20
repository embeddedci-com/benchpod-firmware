/* bp_errors.c: the reply line for a stable error code (bp_errors.h, errors/catalog.toml).

   {"status":"error","code":"BP0031","args":{"la":4,"owner":"gpio"},"message":"LA4 is in use by gpio"}\n

   "message" is the code's catalog message with the args filled in ({la} -> 4), so what the pod
   says is what the catalog says: the hosts render the same text, the fix and the docs link from
   errors/errors.json, keyed on "code". A build with -DBP_ERRORS_NO_TEXT has no templates and
   sends the compact fallback "BP0031 la=4 owner=gpio" instead. "args" is left out when a code
   has none.

   bp_err_note keeps the code of a message a module built for its caller (la_pins.c and the
   other modules that answer with a text): send_error() asks bp_err_noted whether the text it is
   about to send came from a code, and sends it with that code when it did. The note matches the
   text itself (a hash and the length), not the buffer, so a caller that copies the text first
   still gets the code. Pure: no I/O, host-tested in test/test_bp_errors.c. */
#define BP_ERRORS_DEFINE_TEMPLATES
#include "bp_errors.h"

#include <stdio.h>
#include <string.h>

#include "bp_json.h"

static const char *template_of(bp_errcode_t code) {
#ifdef BP_ERRORS_NO_TEXT
    (void)code;
    return NULL;
#else
    return ((unsigned)code <= BP_E_LAST) ? bp_err_templates[code] : NULL;
#endif
}

static const bp_err_arg_t *arg_named(const bp_err_arg_t *args, size_t n, const char *name,
                                     size_t len) {
    for (size_t i = 0; i < n; i++)
        if (args[i].key && strlen(args[i].key) == len && !strncmp(args[i].key, name, len))
            return &args[i];
    return NULL;
}

/* One rendered piece: a run of template text, or an arg's value. */
typedef void (*piece_fn)(void *ctx, const char *s, size_t len);

/* Walk template t: literal runs and {arg} values, in order, to put(). A {name} without such an
   arg renders as "?", like the hosts do. */
static void render(const char *t, const bp_err_arg_t *args, size_t n, piece_fn put, void *ctx) {
    while (*t) {
        const char *open = strchr(t, '{');
        const char *close = open ? strchr(open, '}') : NULL;
        if (!open || !close) { put(ctx, t, strlen(t)); return; }
        if (open > t) put(ctx, t, (size_t)(open - t));
        const bp_err_arg_t *a = arg_named(args, n, open + 1, (size_t)(close - open - 1));
        if (!a) {
            put(ctx, "?", 1);
        } else if (a->is_str) {
            const char *s = a->s ? a->s : "";
            put(ctx, s, strlen(s));
        } else {
            char num[16];
            int len = snprintf(num, sizeof(num), "%ld", (long)a->i);
            put(ctx, num, len > 0 ? (size_t)len : 0);
        }
        t = close + 1;
    }
}

/* "BP0031 la=4 owner=gpio": the message without a template. */
static void fallback(const bp_err_arg_t *args, size_t n, bp_errcode_t code, piece_fn put, void *ctx) {
    char head[8];
    int len = snprintf(head, sizeof(head), "BP%04u", (unsigned)code);
    put(ctx, head, len > 0 ? (size_t)len : 0);
    for (size_t i = 0; i < n; i++) {
        put(ctx, " ", 1);
        put(ctx, args[i].key, strlen(args[i].key));
        put(ctx, "=", 1);
        if (args[i].is_str) {
            const char *s = args[i].s ? args[i].s : "";
            put(ctx, s, strlen(s));
        } else {
            char num[16];
            int l = snprintf(num, sizeof(num), "%ld", (long)args[i].i);
            put(ctx, num, l > 0 ? (size_t)l : 0);
        }
    }
}

static void message(bp_errcode_t code, const bp_err_arg_t *args, size_t n, piece_fn put, void *ctx) {
    const char *t = template_of(code);
    if (t) render(t, args, n, put, ctx);
    else   fallback(args, n, code, put, ctx);
}

/* ---- into a JSON string ---- */

static void put_json(void *ctx, const char *s, size_t len) {
    bp_emit_t *e = ctx;
    char chunk[48];
    while (len) {   /* bp_emit_jstr wants a C string and adds the quotes: escape by hand */
        size_t k = 0;
        while (len && k + 7 < sizeof(chunk)) {
            unsigned char c = (unsigned char)*s++;
            len--;
            if (c == '"' || c == '\\') { chunk[k++] = '\\'; chunk[k++] = (char)c; }
            else if (c < 0x20)         k += (size_t)snprintf(chunk + k, sizeof(chunk) - k, "\\u%04x", c);
            else                       chunk[k++] = (char)c;
        }
        chunk[k] = 0;
        bp_emit_raw(e, chunk);
    }
}

static void emit_args(bp_emit_t *e, const bp_err_arg_t *args, size_t n) {
    bp_emit_raw(e, ",\"args\":{");
    for (size_t i = 0; i < n; i++) {
        bp_emit(e, "%s\"%s\":", i ? "," : "", args[i].key);
        if (args[i].is_str) bp_emit_jstr(e, args[i].s ? args[i].s : "");
        else                bp_emit(e, "%ld", (long)args[i].i);
    }
    bp_emit_raw(e, "}");
}

size_t bp_error_format(char *buf, size_t cap, bp_errcode_t code, const bp_err_arg_t *args,
                       size_t n) {
    bp_emit_t e;
    bp_emit_init(&e, buf, cap);
    bp_emit(&e, "{\"status\":\"error\",\"code\":\"BP%04u\"", (unsigned)code);
    if (n) emit_args(&e, args, n);
    bp_emit_raw(&e, ",\"message\":\"");
    message(code, args, n, put_json, &e);
    bp_emit_raw(&e, "\"}\n");
    if (bp_emit_ok(&e)) return bp_emit_len(&e);
    /* Too long (a huge string arg): drop the text and the args, keep the code. Never a torn line:
       the client would block on the missing newline. */
    bp_emit_init(&e, buf, cap);
    bp_emit(&e, "{\"status\":\"error\",\"code\":\"BP%04u\",\"message\":\"BP%04u\"}\n", (unsigned)code,
            (unsigned)code);
    return bp_emit_ok(&e) ? bp_emit_len(&e) : 0;
}

/* ---- into a plain string ---- */

typedef struct { char *out; size_t cap, len; } plain_t;

static void put_plain(void *ctx, const char *s, size_t len) {
    plain_t *p = ctx;
    if (!p->cap) return;
    size_t room = p->cap - 1u - p->len;
    if (len > room) len = room;
    memcpy(p->out + p->len, s, len);
    p->len += len;
    p->out[p->len] = 0;
}

size_t bp_err_render(char *out, size_t cap, bp_errcode_t code, const bp_err_arg_t *args, size_t n) {
    plain_t p = { out, cap, 0 };
    if (!out || !cap) return 0;
    out[0] = 0;
    message(code, args, n, put_plain, &p);
    return p.len;
}

/* ---- notes ---- */

/* Two notes: a module may build one message, then ask another (bus_busy_reason) before its
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
