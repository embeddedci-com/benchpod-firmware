#include "bp_json.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <ctype.h>

/* ---- parser --------------------------------------------------------------- */

static const char *skip_ws(const char *p) {
    while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') p++;
    return p;
}

/* Return a pointer to the first byte of `key`'s value (past the ':' and any
   whitespace), or NULL if the key is not present at a real key position.  A
   quoted string that merely equals the key text elsewhere (e.g. as a value) is
   skipped because the match must be followed by ':'. */
static const char *find_key(const char *json, const char *key) {
    char needle[64];
    int n = snprintf(needle, sizeof(needle), "\"%s\"", key);
    if (n <= 0 || (size_t)n >= sizeof(needle)) return NULL;
    size_t nlen = (size_t)n;

    const char *p = json;
    for (;;) {
        p = strstr(p, needle);
        if (!p) return NULL;
        const char *q = skip_ws(p + nlen);
        if (*q == ':') return skip_ws(q + 1);
        p += 1;   /* not a key here — resume scanning past this match */
    }
}

/* Skip one complete JSON value starting at p (string, object/array, or scalar).
   Returns the byte just past it, or NULL if the value is malformed/unterminated. */
static const char *skip_value(const char *p) {
    if (*p == '"') {
        p++;
        while (*p && *p != '"') { if (*p == '\\' && p[1]) p++; p++; }
        return *p ? p + 1 : NULL;
    }
    if (*p == '{' || *p == '[') {
        int depth = 0;
        bool in_str = false, esc = false;
        for (; *p; p++) {
            if (esc) { esc = false; continue; }
            if (in_str) {
                if (*p == '\\') esc = true;
                else if (*p == '"') in_str = false;
                continue;
            }
            if (*p == '"') { in_str = true; continue; }
            if (*p == '{' || *p == '[') depth++;
            else if (*p == '}' || *p == ']') { depth--; if (depth == 0) return p + 1; }
        }
        return NULL;
    }
    while (*p && *p != ',' && *p != '}' && *p != ']') p++;
    return p;
}

/* Like find_key, but matches ONLY a key of the OUTERMOST object — it walks the
   object one member at a time and skips each value wholesale, so a key nested
   inside a member never matches.

   This exists because find_key is a plain strstr scan: it returns the first
   "key": ANYWHERE, nesting included.  That silently broke the cloud command
   channel.  The server sends
       {"command":{"cmd":"sensor_start","type":"bmp280",...},...,"type":"command.request"}
   — Go marshals map[string]any with keys SORTED, so "command" precedes "type" on
   the wire.  Routing the frame with find_key(msg,"type") therefore read the
   command's OWN "type" ("bmp280"), matched no frame branch, and dropped the frame
   without a reply; the caller waited out the full 30 s timeout and got a 504.
   Any command carrying a top-level "type" hit this — sensor_start is simply the
   only one in the API whose schema has that key.  Envelope fields must be read
   with the _top variants so a command's payload can never shadow them. */
static const char *find_key_top(const char *json, const char *key) {
    const char *p = skip_ws(json);
    if (*p != '{') return NULL;
    p++;
    size_t klen = strlen(key);
    for (;;) {
        p = skip_ws(p);
        if (*p == ',') { p++; continue; }
        if (*p == '}' || *p == '\0') return NULL;
        if (*p != '"') return NULL;                 /* not a key position */
        const char *ks = ++p;                        /* first char inside the quotes */
        while (*p && *p != '"') { if (*p == '\\' && p[1]) p++; p++; }
        if (*p != '"') return NULL;
        size_t found = (size_t)(p - ks);
        p = skip_ws(p + 1);
        if (*p != ':') return NULL;
        p = skip_ws(p + 1);
        if (found == klen && strncmp(ks, key, klen) == 0) return p;
        p = skip_value(p);
        if (!p) return NULL;
    }
}

/* Decode one backslash escape.  *pp points just past the backslash; on return it
   points past the escape sequence.  Unknown escapes yield the literal char. */
static char unescape(const char **pp) {
    const char *p = *pp;
    char c = *p++;
    char out;
    switch (c) {
        case 'n': out = '\n'; break;
        case 'r': out = '\r'; break;
        case 't': out = '\t'; break;
        case 'b': out = '\b'; break;
        case 'f': out = '\f'; break;
        case '"': out = '"';  break;
        case '\\': out = '\\'; break;
        case '/': out = '/';  break;
        case 'u':   /* \uXXXX: skip up to 4 hex digits, emit a placeholder */
            for (int k = 0; k < 4 && isxdigit((unsigned char)*p); k++) p++;
            out = '?';
            break;
        default: out = c; break;
    }
    *pp = p;
    return out;
}

static bool json_get_at(const char *json, const char *key, char *out, size_t out_len,
                       bool top_level) {
    if (!out || out_len == 0) return false;
    out[0] = '\0';
    const char *p = top_level ? find_key_top(json, key) : find_key(json, key);
    if (!p) return false;

    size_t i = 0;
    if (*p == '"') {
        p++;
        while (*p && *p != '"' && i + 1 < out_len) {
            char c = *p++;
            if (c == '\\' && *p) c = unescape(&p);
            out[i++] = c;
        }
        out[i] = '\0';
        return true;   /* a present (even empty) string counts as found */
    }

    while (*p && *p != ',' && *p != '}' && *p != ' ' && *p != '\t' &&
           *p != '\n' && *p != '\r' && i + 1 < out_len) {
        out[i++] = *p++;
    }
    out[i] = '\0';
    return i > 0;
}

bool bp_json_get(const char *json, const char *key, char *out, size_t out_len) {
    return json_get_at(json, key, out, out_len, false);
}

bool bp_json_get_top(const char *json, const char *key, char *out, size_t out_len) {
    return json_get_at(json, key, out, out_len, true);
}

bool bp_json_flag(const char *json, const char *key) {
    char s[8] = {0};
    if (!bp_json_get(json, key, s, sizeof(s))) return false;
    return s[0] == '1' || s[0] == 't' || s[0] == 'T' || s[0] == 'y' || s[0] == 'Y';
}

static bool json_object_at(const char *json, const char *key, char *out, size_t cap,
                          bool top_level) {
    if (!out || cap == 0) return false;
    out[0] = '\0';
    const char *p = top_level ? find_key_top(json, key) : find_key(json, key);
    if (!p || *p != '{') return false;

    int depth = 0;
    bool in_str = false, esc = false;
    size_t i = 0;
    for (; *p; p++) {
        if (i + 1 >= cap) return false;
        out[i++] = *p;
        if (esc) { esc = false; continue; }
        if (in_str) {
            if (*p == '\\') esc = true;
            else if (*p == '"') in_str = false;
            continue;
        }
        if (*p == '"') in_str = true;
        else if (*p == '{') depth++;
        else if (*p == '}') { depth--; if (depth == 0) { out[i] = '\0'; return true; } }
    }
    return false;
}

bool bp_json_object(const char *json, const char *key, char *out, size_t cap) {
    return json_object_at(json, key, out, cap, false);
}

bool bp_json_object_top(const char *json, const char *key, char *out, size_t cap) {
    return json_object_at(json, key, out, cap, true);
}

int bp_json_byte_array(const char *json, const char *key,
                       uint8_t *out, int cap, int *len) {
    *len = 0;
    const char *p = find_key(json, key);
    if (!p || *p != '[') return 0;
    p++;
    int n = 0;
    while (*p && *p != ']') {
        while (*p == ' ' || *p == ',') p++;
        if (*p == ']' || *p == '\0') break;
        if (n < cap) out[n] = (uint8_t)strtoul(p, NULL, 0);
        n++;
        while (*p && *p != ',' && *p != ']') p++;
    }
    *len = (n > cap) ? cap : n;
    return 1;
}

/* ---- emitter -------------------------------------------------------------- */

static void emit_bytes(bp_emit_t *e, const char *s, size_t n) {
    if (!e->ok) return;
    if (n + 1 > e->cap - e->len) {   /* +1 for the NUL */
        e->ok = false;
        return;
    }
    memcpy(e->buf + e->len, s, n);
    e->len += n;
    e->buf[e->len] = '\0';
}

void bp_emit_init(bp_emit_t *e, char *buf, size_t cap) {
    e->buf = buf;
    e->cap = cap;
    e->len = 0;
    e->ok  = (cap > 0);
    if (cap > 0) buf[0] = '\0';
}

void bp_emit(bp_emit_t *e, const char *fmt, ...) {
    if (!e->ok) return;
    size_t space = e->cap - e->len;   /* includes the NUL slot */
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(e->buf + e->len, space, fmt, ap);
    va_end(ap);
    if (n < 0 || (size_t)n >= space) {
        e->ok = false;
        e->buf[e->len] = '\0';   /* drop the partial write */
        return;
    }
    e->len += (size_t)n;
}

void bp_emit_raw(bp_emit_t *e, const char *s) {
    emit_bytes(e, s, strlen(s));
}

void bp_emit_jstr(bp_emit_t *e, const char *s) {
    emit_bytes(e, "\"", 1);
    for (; *s && e->ok; s++) {
        unsigned char c = (unsigned char)*s;
        switch (c) {
            case '"':  emit_bytes(e, "\\\"", 2); break;
            case '\\': emit_bytes(e, "\\\\", 2); break;
            case '\n': emit_bytes(e, "\\n", 2);  break;
            case '\r': emit_bytes(e, "\\r", 2);  break;
            case '\t': emit_bytes(e, "\\t", 2);  break;
            default:
                if (c < 0x20) {
                    char u[7];
                    int n = snprintf(u, sizeof(u), "\\u%04x", c);
                    if (n > 0) emit_bytes(e, u, (size_t)n);
                } else {
                    emit_bytes(e, (const char *)&c, 1);
                }
        }
    }
    emit_bytes(e, "\"", 1);
}
