/* wiring_doc.c: see wiring_doc.h. Pure, host-tested in test/test_wiring_doc.c. */
#include "wiring_doc.h"

#include <string.h>

static const uint8_t k_magic[4] = { 'B', 'P', 'W', 'R' };

static uint32_t rd32(const uint8_t *p) {
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}
static uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | p[1] << 8); }
static void wr32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static void wr16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }

bool wiring_name_ok(const char *name) {
    if (!name) return false;
    size_t n = strlen(name);
    if (n == 0 || n > WIRING_NAME_MAX) return false;
    if (name[0] == '-') return false;
    for (size_t i = 0; i < n; i++) {
        char c = name[i];
        if (!((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-')) return false;
    }
    return true;
}

bool wiring_doc_parse(const uint8_t *doc, size_t len, wiring_index_t *idx) {
    memset(idx, 0, sizeof(*idx));
    idx->def = WIRING_NO_DEFAULT;
    if (len == 0) return true;
    if (len < WIRING_DOC_HDR || len > WIRING_DOC_MAX || memcmp(doc, k_magic, 4) != 0 || doc[4] != 1)
        goto bad;
    uint8_t count = doc[5], def = doc[6];
    if (count > WIRING_MAX_PROFILES || (def != WIRING_NO_DEFAULT && def >= count)) goto bad;
    size_t p = WIRING_DOC_HDR;
    for (uint8_t i = 0; i < count; i++) {
        if (p + 1 > len) goto bad;
        uint8_t nl = doc[p++];
        if (nl == 0 || nl > WIRING_NAME_MAX || p + nl + 6 > len) goto bad;
        wiring_entry_t *e = &idx->e[i];
        memcpy(e->name, doc + p, nl);
        e->name[nl] = 0;
        if (!wiring_name_ok(e->name)) goto bad;
        p += nl;
        e->rev = rd32(doc + p); p += 4;
        e->len = rd16(doc + p); p += 2;
        if (e->len > WIRING_PROFILE_MAX || p + e->len > len) goto bad;
        e->off = (uint32_t)p;
        p += e->len;
        for (uint8_t j = 0; j < i; j++)
            if (!strcmp(idx->e[j].name, e->name)) goto bad;   /* a name twice */
    }
    if (p != len) goto bad;
    idx->count = count;
    idx->def = def;
    idx->doc_len = (uint32_t)len;
    return true;
bad:
    memset(idx, 0, sizeof(*idx));
    idx->def = WIRING_NO_DEFAULT;
    return false;
}

int wiring_index_find(const wiring_index_t *idx, const char *name) {
    for (uint8_t i = 0; i < idx->count; i++)
        if (!strcmp(idx->e[i].name, name)) return i;
    return -1;
}

bool wiring_doc_json_object_ok(const uint8_t *t, size_t len) {
    size_t i = 0;
    while (i < len && (t[i] == ' ' || t[i] == '\n' || t[i] == '\r' || t[i] == '\t')) i++;
    if (i >= len || t[i] != '{') return false;
    char stack[32];
    int depth = 0;
    bool closed = false;
    for (; i < len; i++) {
        uint8_t c = t[i];
        if (closed) {
            if (c == ' ' || c == '\n' || c == '\r' || c == '\t') continue;
            return false;                           /* anything after the object */
        }
        if (c == '"') {
            for (i++; i < len && t[i] != '"'; i++) {
                if (t[i] < 0x20) return false;      /* raw control character in a string */
                if (t[i] == '\\') {
                    if (++i >= len) return false;
                    if (!strchr("\"\\/bfnrtu", t[i])) return false;
                }
            }
            if (i >= len) return false;             /* unterminated string */
            continue;
        }
        if (c == '{' || c == '[') {
            if (depth >= (int)sizeof(stack)) return false;
            stack[depth++] = (char)c;
        } else if (c == '}' || c == ']') {
            if (depth == 0 || stack[depth - 1] != (c == '}' ? '{' : '[')) return false;
            if (--depth == 0) closed = true;
        } else if (c < 0x20 && c != ' ' && c != '\n' && c != '\r' && c != '\t') {
            return false;
        }
    }
    return closed;
}

wiring_result_t wiring_doc_check(const wiring_index_t *idx, const wiring_op_t *op) {
    if (!wiring_name_ok(op->name)) return WIRING_E_NAME;
    int at = wiring_index_find(idx, op->name);
    switch (op->kind) {
    case WIRING_OP_PUT:
        if (op->json_len > WIRING_PROFILE_MAX) return WIRING_E_TOO_LARGE;
        if (!wiring_doc_json_object_ok(op->json, op->json_len)) return WIRING_E_NOT_OBJECT;
        if (at < 0 && idx->count >= WIRING_MAX_PROFILES) return WIRING_E_FULL;
        if (wiring_doc_size_after(idx, op) > WIRING_DOC_MAX) return WIRING_E_FULL;
        return WIRING_OK;
    case WIRING_OP_DELETE:
        if (at < 0) return WIRING_E_NOT_FOUND;
        if ((uint8_t)at == idx->def) return WIRING_E_DEFAULT;
        return WIRING_OK;
    case WIRING_OP_DEFAULT:
        return at < 0 ? WIRING_E_NOT_FOUND : WIRING_OK;
    }
    return WIRING_E_NAME;
}

static size_t rec_size(size_t name_len, size_t json_len) { return 1 + name_len + 4 + 2 + json_len; }

size_t wiring_doc_size_after(const wiring_index_t *idx, const wiring_op_t *op) {
    size_t n = WIRING_DOC_HDR;
    int at = wiring_index_find(idx, op->name);
    for (uint8_t i = 0; i < idx->count; i++) {
        if ((int)i == at && op->kind == WIRING_OP_DELETE) continue;
        size_t jl = ((int)i == at && op->kind == WIRING_OP_PUT) ? op->json_len : idx->e[i].len;
        n += rec_size(strlen(idx->e[i].name), jl);
    }
    if (at < 0 && op->kind == WIRING_OP_PUT) n += rec_size(strlen(op->name), op->json_len);
    return n;
}

static size_t put_rec(uint8_t *out, const char *name, uint32_t rev, const uint8_t *json, uint16_t jl) {
    size_t nl = strlen(name), p = 0;
    out[p++] = (uint8_t)nl;
    memcpy(out + p, name, nl); p += nl;
    wr32(out + p, rev); p += 4;
    wr16(out + p, jl); p += 2;
    memcpy(out + p, json, jl); p += jl;
    return p;
}

size_t wiring_doc_build(const uint8_t *old, const wiring_index_t *idx, const wiring_op_t *op,
                        uint8_t *out, size_t cap) {
    size_t need = wiring_doc_size_after(idx, op);
    if (need > cap) return 0;
    int at = wiring_index_find(idx, op->name);
    uint8_t count = 0, def = WIRING_NO_DEFAULT;
    size_t p = WIRING_DOC_HDR;
    for (uint8_t i = 0; i < idx->count; i++) {
        const wiring_entry_t *e = &idx->e[i];
        if ((int)i == at && op->kind == WIRING_OP_DELETE) continue;
        bool is_def = op->kind == WIRING_OP_DEFAULT ? (int)i == at : i == idx->def;
        if (is_def) def = count;
        if ((int)i == at && op->kind == WIRING_OP_PUT)
            p += put_rec(out + p, e->name, e->rev + 1u, op->json, op->json_len);
        else
            p += put_rec(out + p, e->name, e->rev, old + e->off, e->len);
        count++;
    }
    if (at < 0 && op->kind == WIRING_OP_PUT) {
        p += put_rec(out + p, op->name, 1, op->json, op->json_len);
        if (def == WIRING_NO_DEFAULT) def = count;   /* the first profile becomes the default */
        count++;
    }
    memcpy(out, k_magic, 4);
    out[4] = 1;
    out[5] = count;
    out[6] = def;
    out[7] = 0;
    return p;
}

/* Skip a JSON string starting at json[i] == '"'; returns the index after the closing quote, or len. */
static size_t skip_str(const uint8_t *json, size_t len, size_t i) {
    for (i++; i < len; i++) {
        if (json[i] == '\\') { i++; continue; }
        if (json[i] == '"') return i + 1;
    }
    return len;
}

uint16_t wiring_doc_json_la_mv(const uint8_t *json, size_t len) {
    static const char key[] = "la_mv";
    const size_t klen = sizeof(key) - 1u;
    int depth = 0;
    bool want_key = false;   /* at depth 1, the next string is a key */
    for (size_t i = 0; i < len;) {
        uint8_t c = json[i];
        if (c == '"') {
            size_t end = skip_str(json, len, i);
            if (depth == 1 && want_key) {
                want_key = false;
                if (end - i == klen + 2u && memcmp(json + i + 1, key, klen) == 0) {
                    size_t j = end;
                    while (j < len && (json[j] == ' ' || json[j] == '\t' || json[j] == '\n' || json[j] == '\r')) j++;
                    if (j >= len || json[j] != ':') return 0;
                    j++;
                    while (j < len && (json[j] == ' ' || json[j] == '\t' || json[j] == '\n' || json[j] == '\r')) j++;
                    uint32_t v = 0;
                    size_t digits = 0;
                    while (j < len && json[j] >= '0' && json[j] <= '9' && digits < 6) { v = v * 10u + (uint32_t)(json[j] - '0'); j++; digits++; }
                    if (digits == 0 || (j < len && (json[j] == '.' || json[j] == 'e' || json[j] == 'E' || (json[j] >= '0' && json[j] <= '9')))) return 0;
                    /* 5000: 5 V I/O, which only digital v4 takes (la_voltage_from_profile). */
                    return (v == 1800u || v == 3300u || v == 5000u) ? (uint16_t)v : 0;
                }
            }
            i = end;
            continue;
        }
        if (c == '{' || c == '[') { depth++; want_key = (c == '{' && depth == 1); }
        else if (c == '}' || c == ']') depth--;
        else if (c == ',' && depth == 1) want_key = true;
        i++;
    }
    return 0;
}
