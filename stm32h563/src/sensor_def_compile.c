/* ============================================================================
 * sensor_def_compile.c — sensor definition JSON to a compiled program.
 *
 * Walks the definition (format.md), checks every key (unknown keys are
 * errors), compiles each expression (sensor_def_expr.c) and lays the records
 * out in the caller's program buffer.  Finally it starts the program once at
 * its defaults, so a definition that compiles also evaluates.  Every error
 * names the JSON path and, for expressions, the character offset.
 * ========================================================================== */
#include "sensor_def_cc.h"

#include <math.h>

/* ---- errors and paths ---- */
static size_t put(char *o, size_t cap, size_t n, const char *s, size_t k) {
    for (size_t i = 0; i < k && n + 1 < cap; i++) o[n++] = s[i];
    if (cap) o[n < cap ? n : cap - 1] = '\0';
    return n;
}

void cc_fail(sd_cc_t *c, int code, uint32_t json_off, int32_t expr_off, const char *fmt, const char *arg) {
    if (c->failed) return;
    c->failed = true;
    sd_err_t *e = c->err;
    e->code = code;
    e->json_off = json_off;
    e->expr_off = expr_off;
    size_t n = 0;
    for (const char *f = fmt; *f && n + 1 < sizeof(e->msg); f++) {
        if (f[0] == '%' && f[1] == 's') {
            const char *a = arg ? arg : "";
            n = put(e->msg, sizeof(e->msg), n, a, strlen(a));
            f++;
        } else {
            e->msg[n++] = *f;
        }
    }
    e->msg[n] = '\0';
}

/* The path of the value (or key) at off: "registers[3].s16be". */
void sdj_path_at(const sdj_t *j, uint32_t root, uint32_t off, char *out, size_t cap) {
    size_t n = 0;
    uint32_t cur = sdj_ws(j, root);
    out[0] = '\0';
    for (int depth = 0; depth < SDJ_MAX_DEPTH; depth++) {
        char t = sdj_type(j, cur);
        if (t != '{' && t != '[') return;
        uint32_t it = cur, k = 0, v, idx = 0;
        bool found = false;
        while (t == '{' ? sdj_obj_next(j, &it, &k, &v) : sdj_arr_next(j, &it, &v)) {
            uint32_t vend = sdj_skip(j, v);
            bool in_key = t == '{' && off >= k && off < v;
            if (in_key || (off >= v && off < vend)) {
                if (t == '{') {
                    uint32_t len;
                    const char *r = sdj_raw(j, k, &len, NULL);
                    if (n) n = put(out, cap, n, ".", 1);
                    n = put(out, cap, n, r, len);
                } else {
                    char b[12];
                    size_t q = 0;
                    char d[10];
                    unsigned x = idx, m = 0;
                    do { d[m++] = (char)('0' + x % 10); x /= 10; } while (x);
                    b[q++] = '[';
                    while (m) b[q++] = d[--m];
                    b[q++] = ']';
                    n = put(out, cap, n, b, q);
                }
                if (in_key || off == sdj_ws(j, v)) return;
                cur = v;
                found = true;
                break;
            }
            idx++;
        }
        if (!found) return;
    }
}

size_t sd_err_format(const sd_err_t *e, char *out, size_t cap) {
    if (!e || !out || !cap) return 0;
    size_t n = 0;
    out[0] = '\0';
    if (e->path[0]) { n = put(out, cap, n, e->path, strlen(e->path)); n = put(out, cap, n, ": ", 2); }
    n = put(out, cap, n, e->msg, strlen(e->msg));
    if (e->expr_off >= 0) {
        char d[12];
        unsigned x = (unsigned)e->expr_off, m = 0;
        do { d[m++] = (char)('0' + x % 10); x /= 10; } while (x);
        n = put(out, cap, n, " at ", 4);
        while (m) { char ch = d[--m]; n = put(out, cap, n, &ch, 1); }
    }
    return n;
}

/* ---- program emission ---- */
bool cc_emit(sd_cc_t *c, const void *d, size_t n) {
    if (c->failed) return false;
    if ((size_t)c->len + n > c->cap) {
        cc_fail(c, SD_E_LIMIT, c->json_off, -1, "compiled program larger than 8 KB", NULL);
        return false;
    }
    if (n) memcpy(c->p + c->len, d, n);
    c->len += (uint32_t)n;
    return true;
}
bool cc_emit8(sd_cc_t *c, uint8_t v) { return cc_emit(c, &v, 1); }
bool cc_emit16(sd_cc_t *c, uint16_t v) { uint8_t b[2] = { (uint8_t)v, (uint8_t)(v >> 8) }; return cc_emit(c, b, 2); }

uint32_t cc_align(sd_cc_t *c) {
    static const uint8_t z[8] = { 0 };
    uint32_t pad = (8u - (c->len & 7u)) & 7u;
    cc_emit(c, z, pad);
    return c->len;
}

uint32_t cc_alloc(sd_cc_t *c, size_t n) {
    uint32_t at = cc_align(c);
    if (c->failed) return 0;
    if ((size_t)at + n > c->cap || at + n > 0xFFFF) {
        cc_fail(c, SD_E_LIMIT, c->json_off, -1, "compiled program larger than 8 KB", NULL);
        return 0;
    }
    memset(c->p + at, 0, n);
    c->len = at + (uint32_t)n;
    return at;
}

uint16_t cc_string(sd_cc_t *c, const char *s, size_t n) {
    if (n > 255) n = 255;
    uint32_t at = c->len;
    if (at > 0xFFFF || !cc_emit8(c, (uint8_t)n) || !cc_emit(c, s, n)) return 0;
    return (uint16_t)at;
}

void cc_range(sd_cc_t *c, uint32_t code_lo, uint32_t json_off) {
    if (c->nranges >= SD_CC_RANGES) return;
    sd_range_t *r = &c->ranges[c->nranges++];
    r->code_lo = (uint16_t)code_lo;
    r->code_hi = (uint16_t)c->len;
    r->json = (uint16_t)sdj_ws(&c->j, json_off);
}

/* ---- schema helpers ---- */
#define J (&c->j)

static bool check_keys(sd_cc_t *c, uint32_t obj, const char *const *allowed) {
    if (sdj_type(J, obj) != '{') { cc_fail(c, SD_E_SCHEMA, sdj_ws(J, obj), -1, "expected an object", NULL); return false; }
    uint32_t it = obj, k, v;
    while (sdj_obj_next(J, &it, &k, &v)) {
        bool ok = false;
        for (const char *const *a = allowed; *a; a++) if (sdj_key_is(J, k, *a)) { ok = true; break; }
        if (!ok) {
            char name[40];
            size_t n;
            if (!sdj_str(J, k, name, sizeof(name), &n)) memcpy(name, "?", 2);
            cc_fail(c, SD_E_SCHEMA, k, -1, "unknown key '%s'", name);
            return false;
        }
    }
    return true;
}

static bool get_num(sd_cc_t *c, uint32_t v, double lo, double hi, bool integer, double *out) {
    double d;
    if (!sdj_num(J, v, &d)) { cc_fail(c, SD_E_SCHEMA, sdj_ws(J, v), -1, "expected a number", NULL); return false; }
    if (integer && d != floor(d)) { cc_fail(c, SD_E_SCHEMA, sdj_ws(J, v), -1, "expected a whole number", NULL); return false; }
    if (!(d >= lo && d <= hi)) { cc_fail(c, SD_E_SCHEMA, sdj_ws(J, v), -1, "number out of range", NULL); return false; }
    *out = d;
    return true;
}
static bool get_int(sd_cc_t *c, uint32_t v, double lo, double hi, uint32_t *out) {
    double d;
    if (!get_num(c, v, lo, hi, true, &d)) return false;
    *out = (uint32_t)d;
    return true;
}
static bool get_bool(sd_cc_t *c, uint32_t v, bool *out) {
    char t = sdj_type(J, v);
    if (t != 't' && t != 'f') { cc_fail(c, SD_E_SCHEMA, sdj_ws(J, v), -1, "expected true or false", NULL); return false; }
    *out = t == 't';
    return true;
}
/* A string that must be one of `opts`; returns its index. */
static int get_enum(sd_cc_t *c, uint32_t v, const char *const *opts) {
    char b[24];
    size_t n;
    if (sdj_type(J, v) != '"' || !sdj_str(J, v, b, sizeof(b), &n)) { cc_fail(c, SD_E_SCHEMA, sdj_ws(J, v), -1, "expected a word", NULL); return -1; }
    for (int i = 0; opts[i]; i++) if (strcmp(opts[i], b) == 0) return i;
    cc_fail(c, SD_E_SCHEMA, sdj_ws(J, v), -1, "unknown value '%s'", b);
    return -1;
}
static bool get_text(sd_cc_t *c, uint32_t v, char *buf, size_t cap, size_t *n) {
    if (sdj_type(J, v) != '"') { cc_fail(c, SD_E_SCHEMA, sdj_ws(J, v), -1, "expected text", NULL); return false; }
    if (!sdj_str(J, v, buf, cap, n)) { cc_fail(c, SD_E_LIMIT, sdj_ws(J, v), -1, "text too long", NULL); return false; }
    return true;
}
static bool want_array(sd_cc_t *c, uint32_t v) {
    if (sdj_type(J, v) == '[') return true;
    cc_fail(c, SD_E_SCHEMA, sdj_ws(J, v), -1, "expected a list", NULL);
    return false;
}

/* ---- symbols (pass 1) ---- */
static const char *const k_reserved[] = {
    "t", "utc", "dt", "written", "value", "count", "fifo_level", "i", "input", "reg", "reg16be",
    "reg16le", "solve", "_", NULL };

static int add_sym(sd_cc_t *c, uint32_t at_json, const char *r, uint32_t len, bool esc, uint8_t kind, uint8_t index) {
    uint32_t off = (uint32_t)(r - c->j.s);
    if (esc || !cc_valid_name(c, off, len)) { cc_fail(c, SD_E_NAME, at_json, -1, "bad name (letters, digits, _; at most 32)", NULL); return -1; }
    char nb[40];
    memcpy(nb, r, len);
    nb[len] = '\0';
    for (int k = 0; k_reserved[k]; k++)
        if (strcmp(k_reserved[k], nb) == 0) { cc_fail(c, SD_E_NAME, at_json, -1, "'%s' is a reserved name", nb); return -1; }
    if (sd_builtin_find(r, len) >= 0) { cc_fail(c, SD_E_NAME, at_json, -1, "'%s' is a built-in function", nb); return -1; }
    if (cc_sym_find(c, r, len) >= 0) { cc_fail(c, SD_E_NAME, at_json, -1, "duplicate name '%s'", nb); return -1; }
    if (c->nsyms >= SD_CC_SYMS) { cc_fail(c, SD_E_LIMIT, at_json, -1, "too many names", NULL); return -1; }
    sd_sym_t *y = &c->syms[c->nsyms];
    memset(y, 0, sizeof(*y));
    y->name = (uint16_t)off;
    y->len = (uint8_t)len;
    y->kind = kind;
    y->index = index;
    return c->nsyms++;
}

/* Members of an object (by key) as symbols. */
static bool syms_from_keys(sd_cc_t *c, uint32_t obj, uint8_t kind, unsigned max, const char *what) {
    if (sdj_type(J, obj) != '{') { cc_fail(c, SD_E_SCHEMA, sdj_ws(J, obj), -1, "expected an object", NULL); return false; }
    uint32_t it = obj, k, v, n = 0;
    while (sdj_obj_next(J, &it, &k, &v)) {
        if (n >= max) { cc_fail(c, SD_E_LIMIT, k, -1, what, NULL); return false; }
        uint32_t len;
        bool esc;
        const char *r = sdj_raw(J, k, &len, &esc);
        if (add_sym(c, k, r, len, esc, kind, (uint8_t)n) < 0) return false;
        n++;
    }
    return true;
}

/* ---- register entries ---- */
static const char *const k_types[SDT_N] = {
    "u8", "s8", "u16be", "u16le", "s16be", "s16le", "u24be", "u24le", "s24be", "s24le", "u32be",
    "u32le", "s32be", "s32le", "bytes", "text" };
static const uint8_t k_type_len[SDT_BYTES] = { 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4 };

/* The image index of a register address (16-bit addressing: inside the page window). */
static bool image_index(sd_cc_t *c, uint32_t v, uint32_t *idx) {
    uint32_t a;
    const sdp_hdr_t *h = c->h;
    if (!get_int(c, v, 0, h->addr_bits == 16 ? 65535 : 255, &a)) return false;
    if (a < h->page || a - h->page > h->wrap) {
        cc_fail(c, SD_E_SCHEMA, sdj_ws(J, v), -1, "register outside the image (page / wrap)", NULL);
        return false;
    }
    *idx = a - h->page;
    return true;
}

static bool compile_entry(sd_cc_t *c, uint32_t obj, sdp_entry_t *e, bool command) {
    static const char *const keys_reg[] = { "at", "u8", "s8", "u16be", "u16le", "s16be", "s16le",
        "u24be", "u24le", "s24be", "s24le", "u32be", "u32le", "s32be", "s32le", "bytes", "text",
        "name", "reload", "writable", "description", NULL };
    static const char *const keys_cmd[] = { "command", "answer", "busy_us", "u8", "s8", "u16be",
        "u16le", "s16be", "s16le", "u24be", "u24le", "s24be", "s24le", "u32be", "u32le", "s32be",
        "s32le", "bytes", "text", "name", "reload", "writable", "description", NULL };
    if (!check_keys(c, obj, command ? keys_cmd : keys_reg)) return false;
    memset(e, 0, sizeof(*e));
    uint32_t it = obj, k, v, at_v = 0, val_v = 0;
    int type = -1, reload = -1;
    bool has_at = false, ro = false;
    while (sdj_obj_next(J, &it, &k, &v)) {
        if (sdj_key_is(J, k, command ? "command" : "at")) { at_v = v; has_at = true; continue; }
        if (sdj_key_is(J, k, "name")) {
            char b[40]; size_t n;
            if (!get_text(c, v, b, sizeof(b), &n)) return false;
            continue;
        }
        if (sdj_key_is(J, k, "description")) continue;
        if (sdj_key_is(J, k, "reload")) {
            static const char *const o[] = { "power_on", "always", NULL };
            if ((reload = get_enum(c, v, o)) < 0) return false;
            continue;
        }
        if (sdj_key_is(J, k, "writable")) { bool w; if (!get_bool(c, v, &w)) return false; ro = !w; continue; }
        if (sdj_key_is(J, k, "busy_us")) { uint32_t b; if (!get_int(c, v, 0, 10000000, &b)) return false; e->busy_us = b; continue; }
        int t = -1;
        if (sdj_key_is(J, k, "answer")) t = SDT_BYTES;
        for (int q = 0; q < SDT_N && t < 0; q++) if (sdj_key_is(J, k, k_types[q])) t = q;
        if (t < 0) continue;
        if (type >= 0) { cc_fail(c, SD_E_SCHEMA, k, -1, "one value per entry (a second field type)", NULL); return false; }
        type = t;
        val_v = v;
    }
    if (!has_at) { cc_fail(c, SD_E_SCHEMA, sdj_ws(J, obj), -1, command ? "missing 'command'" : "missing 'at'", NULL); return false; }
    if (type < 0) { cc_fail(c, SD_E_SCHEMA, sdj_ws(J, obj), -1, command ? "missing 'answer'" : "missing a value (u8, s16be, bytes, ...)", NULL); return false; }
    uint32_t idx;
    if (!image_index(c, at_v, &idx)) return false;
    e->at = (uint8_t)idx;
    e->type = (uint8_t)type;
    uint8_t deps = 0;
    if (type < SDT_BYTES) {
        e->len = k_type_len[type];
        c->deps = 0;
        c->let_hi = 0;
        e->code = cc_expr(c, val_v, CTX_LETS_BELOW);
        if (!e->code) return false;
        deps = c->deps;
    } else if (type == SDT_BYTES) {
        if (!want_array(c, val_v)) return false;
        uint32_t n = sdj_count(J, val_v);
        if (n == 0 || n > 255) { cc_fail(c, SD_E_LIMIT, sdj_ws(J, val_v), -1, "1 to 255 bytes", NULL); return false; }
        uint32_t arr = cc_alloc(c, 2u * n);
        if (!arr) return false;
        uint32_t ai = val_v, av, q = 0;
        while (sdj_arr_next(J, &ai, &av)) {
            c->deps = 0;
            c->let_hi = 0;
            uint16_t code = cc_expr(c, av, CTX_LETS_BELOW);
            if (!code) return false;
            deps |= c->deps;
            c->p[arr + 2 * q] = (uint8_t)code;
            c->p[arr + 2 * q + 1] = (uint8_t)(code >> 8);
            q++;
        }
        e->len = (uint8_t)n;
        e->code = (uint16_t)arr;
    } else {   /* text: a fixed string */
        char b[256];
        size_t n;
        if (!get_text(c, val_v, b, sizeof(b), &n)) return false;
        if (n == 0 || n > 255) { cc_fail(c, SD_E_LIMIT, sdj_ws(J, val_v), -1, "text: 1 to 255 characters", NULL); return false; }
        uint32_t at = cc_alloc(c, n);
        if (!at) return false;
        memcpy(c->p + at, b, n);
        e->type = SDT_RAW;
        e->len = (uint8_t)n;
        e->code = (uint16_t)at;
    }
    if (deps) e->flags |= SDE_DYN;
    if (reload == 0) e->flags &= (uint8_t)~SDE_DYN;
    if (reload == 1) e->flags |= SDE_DYN;
    if (ro) e->flags |= SDE_RO;
    if (command) e->flags |= SDE_CMD;
    return true;
}

static bool compile_entries(sd_cc_t *c, uint32_t regs, uint32_t cmds) {
    uint32_t n = (regs ? sdj_count(J, regs) : 0) + (cmds ? sdj_count(J, cmds) : 0);
    if ((regs && !want_array(c, regs)) || (cmds && !want_array(c, cmds))) return false;
    if (n > SD_MAX_ENTRIES) { cc_fail(c, SD_E_LIMIT, sdj_ws(J, regs ? regs : cmds), -1, "more than 255 register and command entries", NULL); return false; }
    if (n == 0) return true;
    uint32_t tab = cc_alloc(c, n * sizeof(sdp_entry_t));
    if (!tab) return false;
    c->h->entries = (uint16_t)tab;
    unsigned q = 0;
    for (int pass = 0; pass < 2; pass++) {
        uint32_t arr = pass == 0 ? regs : cmds;
        if (!arr) continue;
        uint32_t it = arr, v;
        while (sdj_arr_next(J, &it, &v)) {
            sdp_entry_t e;
            if (!compile_entry(c, v, &e, pass == 1)) return false;
            memcpy(c->p + tab + q * sizeof(sdp_entry_t), &e, sizeof(e));
            if (pass == 1) c->h->n_cmds++;
            q++;
        }
    }
    c->h->n_entries = (uint8_t)q;
    /* owner map: which entry a byte belongs to (the last one covering it wins) */
    uint32_t own = cc_alloc(c, SD_REGIMAGE);
    if (!own) return false;
    c->h->owner = (uint16_t)own;
    memset(c->p + own, 0xFF, SD_REGIMAGE);
    unsigned span = (unsigned)c->h->wrap + 1u;
    for (unsigned i = 0; i < q; i++) {
        const sdp_entry_t *e = (const sdp_entry_t *)(const void *)(c->p + tab) + i;
        for (unsigned b = 0; b < e->len; b++) c->p[own + (e->at + b) % span] = (uint8_t)i;
    }
    return true;
}

/* ---- sections ---- */
static bool compile_params(sd_cc_t *c, uint32_t arr) {
    static const char *const keys[] = { "key", "unit", "min", "max", "default", "label", "description", NULL };
    if (!want_array(c, arr)) return false;
    uint32_t n = sdj_count(J, arr);
    if (n > SD_MAX_PARAMS) { cc_fail(c, SD_E_LIMIT, sdj_ws(J, arr), -1, "more than 16 params", NULL); return false; }
    if (!n) return true;
    uint32_t tab = cc_alloc(c, n * sizeof(sdp_param_t));
    if (!tab) return false;
    c->h->params = (uint16_t)tab;
    c->h->n_params = (uint8_t)n;
    uint32_t it = arr, v, q = 0;
    while (sdj_arr_next(J, &it, &v)) {
        if (!check_keys(c, v, keys)) return false;
        sdp_param_t p;
        memset(&p, 0, sizeof(p));
        p.min = -1e300; p.max = 1e300;
        bool has_def = false, has_key = false;
        uint32_t mi = v, k, x;
        while (sdj_obj_next(J, &mi, &k, &x)) {
            char b[64];
            size_t len;
            if (sdj_key_is(J, k, "key")) {
                uint32_t rl;
                bool esc;
                if (sdj_type(J, x) != '"') { cc_fail(c, SD_E_SCHEMA, sdj_ws(J, x), -1, "expected a name", NULL); return false; }
                const char *r = sdj_raw(J, x, &rl, &esc);
                if (add_sym(c, sdj_ws(J, x), r, rl, esc, SYM_PARAM, (uint8_t)q) < 0) return false;
                if (!(p.key = cc_string(c, r, rl))) return false;
                has_key = true;
                if (rl == 9 && memcmp(r, "utc_start", 9) == 0) c->h->utc_param = (int8_t)q;
            } else if (sdj_key_is(J, k, "unit") || sdj_key_is(J, k, "label")) {
                if (!get_text(c, x, b, sizeof(b), &len)) return false;
                uint16_t s = cc_string(c, b, len);
                if (!s) return false;
                if (sdj_key_is(J, k, "unit")) p.unit = s; else p.label = s;
            } else if (sdj_key_is(J, k, "min")) { if (!get_num(c, x, -1e300, 1e300, false, &p.min)) return false; }
            else if (sdj_key_is(J, k, "max")) { if (!get_num(c, x, -1e300, 1e300, false, &p.max)) return false; }
            else if (sdj_key_is(J, k, "default")) { if (!get_num(c, x, -1e300, 1e300, false, &p.def)) return false; has_def = true; }
        }
        if (!has_key) { cc_fail(c, SD_E_SCHEMA, sdj_ws(J, v), -1, "a param needs 'key'", NULL); return false; }
        if (p.min > p.max) { cc_fail(c, SD_E_SCHEMA, sdj_ws(J, v), -1, "min is above max", NULL); return false; }
        if (!has_def) p.def = p.min > 0 ? p.min : (p.max < 0 ? p.max : 0);
        if (p.def < p.min || p.def > p.max) { cc_fail(c, SD_E_SCHEMA, sdj_ws(J, v), -1, "default outside min..max", NULL); return false; }
        memcpy(c->p + tab + q * sizeof(sdp_param_t), &p, sizeof(p));
        q++;
    }
    return true;
}

static bool compile_constants(sd_cc_t *c, uint32_t obj) {
    uint16_t first = c->nsyms;
    if (!syms_from_keys(c, obj, SYM_CONST, SD_MAX_CONSTS, "more than 64 constants")) return false;
    uint32_t it = obj, k, v;
    uint8_t q = 0;
    while (sdj_obj_next(J, &it, &k, &v)) {
        c->const_limit = q;
        if (!cc_const_value(c, v, &c->syms[first + q].cval)) return false;
        q++;
    }
    c->const_limit = 0;
    return true;
}

static bool compile_funcs(sd_cc_t *c, uint32_t obj, bool frag, uint16_t first) {
    uint32_t n = sdj_count(J, obj);
    if (!n) return true;
    uint32_t tab = cc_alloc(c, n * sizeof(sdp_func_t));
    if (!tab) return false;
    if (frag) { c->h->frags = (uint16_t)tab; c->h->n_frags = (uint8_t)n; }
    else { c->h->funcs = (uint16_t)tab; c->h->n_funcs = (uint8_t)n; }
    uint32_t it = obj, k, v, q = 0;
    while (sdj_obj_next(J, &it, &k, &v)) {
        if (sdj_type(J, v) != '{') { cc_fail(c, SD_E_SCHEMA, sdj_ws(J, v), -1, "expected an object", NULL); return false; }
        if (frag) { c->frag_limit = (uint8_t)q; c->func_limit = c->h->n_funcs; }
        else { c->func_limit = (uint8_t)q; c->frag_limit = 0; }
        sdp_func_t f;
        uint8_t lh, deps;
        if (!cc_function(c, v, &f, frag, &lh, &deps)) return false;
        memcpy(c->p + tab + q * sizeof(sdp_func_t), &f, sizeof(f));
        sd_sym_t *y = &c->syms[first + q];
        y->nargs = f.nargs;
        y->let_hi = lh;
        y->deps = deps;
        q++;
    }
    c->func_limit = 0xFF;
    c->frag_limit = 0xFF;
    return true;
}

static bool compile_states(sd_cc_t *c, uint32_t arr, uint16_t first) {
    static const char *const keys[] = { "name", "init", "update", "follow", "description", NULL };
    uint32_t n = sdj_count(J, arr);
    if (!n) return true;
    uint32_t tab = cc_alloc(c, n * sizeof(sdp_state_t));
    if (!tab) return false;
    c->h->states = (uint16_t)tab;
    c->h->n_states = (uint8_t)n;
    uint32_t it = arr, v, q = 0;
    (void)first;
    while (sdj_arr_next(J, &it, &v)) {
        if (!check_keys(c, v, keys)) return false;
        sdp_state_t s = { 0, 0, -1, 0 };
        uint32_t x;
        if (!sdj_find(J, v, "init", &x)) { cc_fail(c, SD_E_SCHEMA, sdj_ws(J, v), -1, "a state needs 'init'", NULL); return false; }
        if (!(s.init = cc_expr(c, x, CTX_NO_LETS))) return false;
        if (!sdj_find(J, v, "update", &x)) { cc_fail(c, SD_E_SCHEMA, sdj_ws(J, v), -1, "a state needs 'update'", NULL); return false; }
        if (!(s.update = cc_expr(c, x, CTX_NO_LETS))) return false;
        if (sdj_find(J, v, "follow", &x)) {
            uint32_t rl;
            bool esc;
            if (sdj_type(J, x) != '"') { cc_fail(c, SD_E_SCHEMA, sdj_ws(J, x), -1, "follow: expected a param name", NULL); return false; }
            const char *r = sdj_raw(J, x, &rl, &esc);
            int si = esc ? -1 : cc_sym_find(c, r, rl);
            if (si < 0 || c->syms[si].kind != SYM_PARAM) { cc_fail(c, SD_E_NAME, sdj_ws(J, x), -1, "follow: not a param", NULL); return false; }
            s.follow = c->syms[si].index;
        }
        memcpy(c->p + tab + q * sizeof(sdp_state_t), &s, sizeof(s));
        q++;
    }
    return true;
}

static bool compile_lets(sd_cc_t *c, uint32_t arr, uint16_t first) {
    uint32_t n = sdj_count(J, arr);
    if (!n) return true;
    uint32_t tab = cc_alloc(c, n * 2u);
    if (!tab) return false;
    c->h->lets = (uint16_t)tab;
    c->h->n_lets = (uint8_t)n;
    uint32_t it = arr, v, q = 0;
    while (sdj_arr_next(J, &it, &v)) {
        uint32_t k, x, mi = v;
        sdj_obj_next(J, &mi, &k, &x);
        c->let_limit = (uint8_t)q;
        c->deps = 0;
        c->let_hi = 0;
        uint16_t code = cc_expr(c, x, CTX_LETS_BELOW);
        if (!code) return false;
        c->syms[first + q].deps = c->deps;
        c->syms[first + q].let_hi = (uint8_t)(q + 1);
        c->p[tab + 2 * q] = (uint8_t)code;
        c->p[tab + 2 * q + 1] = (uint8_t)(code >> 8);
        q++;
    }
    c->let_limit = (uint8_t)n;
    return true;
}

static uint16_t expr_field(sd_cc_t *c, uint32_t obj, const char *key, bool required, uint8_t ctx, bool templ) {
    uint32_t v;
    if (!sdj_find(J, obj, key, &v)) {
        if (required) cc_fail(c, SD_E_SCHEMA, sdj_ws(J, obj), -1, "missing '%s'", key);
        return 0;
    }
    return templ ? cc_template(c, v, ctx) : cc_expr(c, v, ctx);
}

static bool compile_i2c_spi(sd_cc_t *c, uint32_t root, bool spi) {
    sdp_hdr_t *h = c->h;
    uint32_t v, x;
    h->addr_bits = 8;
    h->wrap = 0xFF;
    h->auto_inc = 1;
    if (!spi) {
        static const char *const keys[] = { "address", "alt_addresses", "register_address_bits", "page", "auto_increment", "wrap", NULL };
        if (!sdj_find(J, root, "i2c", &v)) { cc_fail(c, SD_E_SCHEMA, root, -1, "kind i2c needs an 'i2c' section with the address", NULL); return false; }
        if (!check_keys(c, v, keys)) return false;
        uint32_t a;
        if (!sdj_find(J, v, "address", &x)) { cc_fail(c, SD_E_SCHEMA, sdj_ws(J, v), -1, "missing 'address'", NULL); return false; }
        if (!get_int(c, x, 0x03, 0x77, &a)) return false;
        h->addr7 = (uint8_t)a;
        if (sdj_find(J, v, "alt_addresses", &x)) {
            if (!want_array(c, x)) return false;
            uint32_t ai = x, av;
            while (sdj_arr_next(J, &ai, &av)) {
                if (h->n_alt >= 3) { cc_fail(c, SD_E_LIMIT, sdj_ws(J, av), -1, "at most 3 alternative addresses", NULL); return false; }
                if (!get_int(c, av, 0x03, 0x77, &a)) return false;
                h->alt[h->n_alt++] = (uint8_t)a;
            }
        }
        if (sdj_find(J, v, "register_address_bits", &x)) {
            if (!get_int(c, x, 8, 16, &a) || (a != 8 && a != 16)) { if (!c->failed) cc_fail(c, SD_E_SCHEMA, sdj_ws(J, x), -1, "8 or 16", NULL); return false; }
            h->addr_bits = (uint8_t)a;
        }
        if (sdj_find(J, v, "page", &x)) {
            if (!get_int(c, x, 0, 65280, &a)) return false;
            if (h->addr_bits != 16 && a) { cc_fail(c, SD_E_SCHEMA, sdj_ws(J, x), -1, "page needs register_address_bits: 16", NULL); return false; }
            if (a & 0xFF) { cc_fail(c, SD_E_SCHEMA, sdj_ws(J, x), -1, "i2c.page: must be a multiple of 0x100", NULL); return false; }
            h->page = (uint16_t)a;
        }
        if (sdj_find(J, v, "auto_increment", &x)) { bool b; if (!get_bool(c, x, &b)) return false; h->auto_inc = b; }
        if (sdj_find(J, v, "wrap", &x)) { if (!get_int(c, x, 0, 255, &a)) return false; h->wrap = (uint8_t)a; }
    } else {
        static const char *const keys[] = { "mode", "read_bit", "increment_bit", "address_mask", "dummy_bytes", NULL };
        static const char *const bkeys[] = { "bit", "value", NULL };
        h->read_bit = 7; h->read_val = 1; h->address_mask = 0x7F;
        if (sdj_find(J, root, "spi", &v)) {
            if (!check_keys(c, v, keys)) return false;
            uint32_t a;
            if (sdj_find(J, v, "mode", &x)) { if (!get_int(c, x, 0, 3, &a)) return false; h->spi_mode = (uint8_t)a; }
            for (int q = 0; q < 2; q++) {
                if (!sdj_find(J, v, q ? "increment_bit" : "read_bit", &x)) continue;
                if (!check_keys(c, x, bkeys)) return false;
                uint32_t b = 7, val = 1, y;
                if (sdj_find(J, x, "bit", &y) && !get_int(c, y, 0, 7, &b)) return false;
                if (sdj_find(J, x, "value", &y) && !get_int(c, y, 0, 1, &val)) return false;
                if (q) { h->inc_bit = (uint8_t)b; h->inc_val = (uint8_t)val; h->has_inc = 1; }
                else { h->read_bit = (uint8_t)b; h->read_val = (uint8_t)val; }
            }
            if (sdj_find(J, v, "address_mask", &x)) { if (!get_int(c, x, 1, 255, &a)) return false; h->address_mask = (uint8_t)a; }
            if (sdj_find(J, v, "dummy_bytes", &x)) { if (!get_int(c, x, 0, 8, &a)) return false; h->dummy_bytes = (uint8_t)a; }
        }
    }
    if (sdj_find(J, root, "fill", &x)) { uint32_t a; if (!get_int(c, x, 0, 255, &a)) return false; h->fill = (uint8_t)a; }

    uint32_t regs = 0, cmds = 0;
    if (sdj_find(J, root, "registers", &x)) regs = x;
    if (sdj_find(J, root, "commands", &x)) {
        if (spi) { cc_fail(c, SD_E_SCHEMA, sdj_ws(J, x), -1, "commands are for kind i2c", NULL); return false; }
        cmds = x;
    }
    if (!compile_entries(c, regs, cmds)) return false;

    if (sdj_find(J, root, "conversion", &v)) {
        static const char *const keys[] = { "trigger", "time_us", "busy", "while_busy", NULL };
        static const char *const bkeys[] = { "at", "mask", NULL };
        static const char *const wb[] = { "flag", "nack", "stretch", NULL };
        if (!check_keys(c, v, keys)) return false;
        uint32_t a;
        h->has_conv = 1;
        if (!sdj_find(J, v, "trigger", &x)) { cc_fail(c, SD_E_SCHEMA, sdj_ws(J, v), -1, "missing 'trigger'", NULL); return false; }
        if (!image_index(c, x, &a)) return false;
        h->conv_trigger = (uint8_t)a;
        if (sdj_find(J, v, "time_us", &x)) { if (!get_int(c, x, 0, 10000000, &a)) return false; h->conv_us = a; }
        if (sdj_find(J, v, "busy", &x)) {
            if (!check_keys(c, x, bkeys)) return false;
            uint32_t y;
            if (!sdj_find(J, x, "at", &y) || !image_index(c, y, &a)) { if (!c->failed) cc_fail(c, SD_E_SCHEMA, sdj_ws(J, x), -1, "missing 'at'", NULL); return false; }
            h->conv_busy_at = (uint8_t)a;
            if (!sdj_find(J, x, "mask", &y) || !get_int(c, y, 1, 255, &a)) { if (!c->failed) cc_fail(c, SD_E_SCHEMA, sdj_ws(J, x), -1, "missing 'mask'", NULL); return false; }
            h->conv_busy_mask = (uint8_t)a;
        }
        if (sdj_find(J, v, "while_busy", &x)) {
            int w = get_enum(c, x, wb);
            if (w < 0) return false;
            if (spi && w) { cc_fail(c, SD_E_SCHEMA, sdj_ws(J, x), -1, "nack and stretch are I2C only", NULL); return false; }
            h->while_busy = (uint8_t)w;
        }
    }

    if (sdj_find(J, root, "on_write", &v)) {
        static const char *const keys[] = { "when", "do", "description", NULL };
        static const char *const dkeys[] = { "set", NULL };
        if (!want_array(c, v)) return false;
        uint32_t n = sdj_count(J, v);
        if (n > SD_MAX_RULES) { cc_fail(c, SD_E_LIMIT, sdj_ws(J, v), -1, "more than 32 on_write rules", NULL); return false; }
        if (n) {
            uint32_t tab = cc_alloc(c, n * sizeof(sdp_rule_t));
            if (!tab) return false;
            h->rules = (uint16_t)tab;
            h->n_rules = (uint8_t)n;
            uint32_t it = v, rv, q = 0;
            while (sdj_arr_next(J, &it, &rv)) {
                if (!check_keys(c, rv, keys)) return false;
                sdp_rule_t r;
                memset(&r, 0, sizeof(r));
                if (!(r.when = expr_field(c, rv, "when", true, CTX_LETS_BELOW, false))) return false;
                if (!sdj_find(J, rv, "do", &x)) { cc_fail(c, SD_E_SCHEMA, sdj_ws(J, rv), -1, "missing 'do'", NULL); return false; }
                if (sdj_type(J, x) == '"') {
                    static const char *const o[] = { "reset", NULL };
                    if (get_enum(c, x, o) < 0) return false;
                    r.reset = 1;
                } else {
                    uint32_t y;
                    if (!check_keys(c, x, dkeys)) return false;
                    if (!sdj_find(J, x, "set", &y)) { cc_fail(c, SD_E_SCHEMA, sdj_ws(J, x), -1, "do: 'reset' or { set: entry }", NULL); return false; }
                    if (!compile_entry(c, y, &r.set, false)) return false;
                }
                memcpy(c->p + tab + q * sizeof(sdp_rule_t), &r, sizeof(r));
                q++;
            }
        }
    }

    if (sdj_find(J, root, "data_ready", &v)) {
        static const char *const keys[] = { "active", "mode", "pulse_us", "on", "rate_hz", "enabled", "clear_on_read", NULL };
        static const char *const act[] = { "low", "high", NULL };
        static const char *const mode[] = { "pulse", "level", NULL };
        static const char *const on[] = { "conversion", "sample", "rate", NULL };
        if (!check_keys(c, v, keys)) return false;
        h->has_drdy = 1;
        h->drdy_high = 1;
        h->drdy_pulse_us = 50;
        h->drdy_clear = -1;
        int e;
        if (sdj_find(J, v, "active", &x)) { if ((e = get_enum(c, x, act)) < 0) return false; h->drdy_high = (uint8_t)e; }
        if (sdj_find(J, v, "mode", &x)) { if ((e = get_enum(c, x, mode)) < 0) return false; h->drdy_level = (uint8_t)e; }
        if (sdj_find(J, v, "on", &x)) { if ((e = get_enum(c, x, on)) < 0) return false; h->drdy_on = (uint8_t)e; }
        uint32_t a;
        if (sdj_find(J, v, "pulse_us", &x)) { if (!get_int(c, x, 1, 1000000, &a)) return false; h->drdy_pulse_us = a; }
        if (sdj_find(J, v, "clear_on_read", &x)) { if (!image_index(c, x, &a)) return false; h->drdy_clear = (int16_t)a; }
        if (sdj_find(J, v, "rate_hz", &x) && !(h->drdy_rate = cc_expr(c, x, CTX_LETS_BELOW))) return false;
        if (h->drdy_on == 2 && !h->drdy_rate) { cc_fail(c, SD_E_SCHEMA, sdj_ws(J, v), -1, "on: rate needs rate_hz", NULL); return false; }
        if (sdj_find(J, v, "enabled", &x) && !(h->drdy_enabled = cc_expr(c, x, CTX_LETS_BELOW))) return false;
    }

    if (sdj_find(J, root, "fifo", &v)) {
        static const char *const keys[] = { "at", "depth", "frame", "rate_hz", "enabled", "overflow", NULL };
        static const char *const ov[] = { "drop_oldest", "drop_newest", NULL };
        if (!check_keys(c, v, keys)) return false;
        uint32_t a;
        h->has_fifo = 1;
        if (!sdj_find(J, v, "at", &x) || !image_index(c, x, &a)) { if (!c->failed) cc_fail(c, SD_E_SCHEMA, sdj_ws(J, v), -1, "missing 'at'", NULL); return false; }
        h->fifo_at = (uint8_t)a;
        h->fifo_depth = 16;
        if (sdj_find(J, v, "depth", &x)) { if (!get_int(c, x, 1, 1024, &a)) return false; h->fifo_depth = (uint16_t)a; }
        if (sdj_find(J, v, "overflow", &x)) { int e = get_enum(c, x, ov); if (e < 0) return false; h->fifo_drop_newest = (uint8_t)e; }
        if (!sdj_find(J, v, "frame", &x)) { cc_fail(c, SD_E_SCHEMA, sdj_ws(J, v), -1, "missing 'frame'", NULL); return false; }
        if (!want_array(c, x)) return false;
        uint32_t n = sdj_count(J, x);
        if (n == 0 || n > 64) { cc_fail(c, SD_E_LIMIT, sdj_ws(J, x), -1, "a frame has 1 to 64 bytes", NULL); return false; }
        uint32_t arr = cc_alloc(c, 2u * n);
        if (!arr) return false;
        uint32_t ai = x, av, q = 0;
        while (sdj_arr_next(J, &ai, &av)) {
            uint16_t code = cc_expr(c, av, CTX_LETS_BELOW);
            if (!code) return false;
            c->p[arr + 2 * q] = (uint8_t)code;
            c->p[arr + 2 * q + 1] = (uint8_t)(code >> 8);
            q++;
        }
        h->fifo_frame = (uint16_t)arr;
        h->fifo_len = (uint8_t)n;
        if (!(h->fifo_rate = expr_field(c, v, "rate_hz", true, CTX_LETS_BELOW, false))) return false;
        if (sdj_find(J, v, "enabled", &x) && !(h->fifo_enabled = cc_expr(c, x, CTX_LETS_BELOW))) return false;
    }
    return true;
}

static int end_kind(sd_cc_t *c, uint32_t obj) {
    static const char *const ends[] = { "none", "nmea", "crlf", "lf", NULL };
    uint32_t x;
    if (!sdj_find(J, obj, "end", &x)) return SDEND_NONE;
    return get_enum(c, x, ends);
}

static bool compile_uart(sd_cc_t *c, uint32_t root) {
    sdp_hdr_t *h = c->h;
    uint32_t v, x;
    h->baud = 9600; h->data_bits = 8; h->parity = 'N'; h->stop_bits = 1;
    if (sdj_find(J, root, "uart", &v)) {
        static const char *const keys[] = { "baud", "format", NULL };
        if (!check_keys(c, v, keys)) return false;
        uint32_t a;
        if (sdj_find(J, v, "baud", &x)) { if (!get_int(c, x, 300, 3000000, &a)) return false; h->baud = a; }
        if (sdj_find(J, v, "format", &x)) {
            char b[8];
            size_t n;
            if (!get_text(c, x, b, sizeof(b), &n)) return false;
            if (n != 3 || b[0] < '5' || b[0] > '8' || !strchr("NEO", b[1]) || (b[2] != '1' && b[2] != '2')) {
                cc_fail(c, SD_E_SCHEMA, sdj_ws(J, x), -1, "format: like 8N1 (5-8 data bits, N/E/O, 1 or 2 stop bits)", NULL);
                return false;
            }
            h->data_bits = (uint8_t)(b[0] - '0'); h->parity = (uint8_t)b[1]; h->stop_bits = (uint8_t)(b[2] - '0');
        }
    }
    if (sdj_find(J, root, "messages", &v)) {
        static const char *const keys[] = { "every_ms", "enabled", "repeat", "text", "end", "description", NULL };
        if (!want_array(c, v)) return false;
        uint32_t n = sdj_count(J, v);
        if (n > SD_MAX_MSGS) { cc_fail(c, SD_E_LIMIT, sdj_ws(J, v), -1, "more than 32 messages", NULL); return false; }
        if (n) {
            uint32_t tab = cc_alloc(c, n * sizeof(sdp_msg_t));
            if (!tab) return false;
            h->msgs = (uint16_t)tab;
            h->n_msgs = (uint8_t)n;
            uint32_t it = v, mv, q = 0;
            while (sdj_arr_next(J, &it, &mv)) {
                if (!check_keys(c, mv, keys)) return false;
                sdp_msg_t m;
                memset(&m, 0, sizeof(m));
                if (!(m.every = expr_field(c, mv, "every_ms", true, CTX_LETS_BELOW, false))) return false;
                if (sdj_find(J, mv, "enabled", &x) && !(m.enabled = cc_expr(c, x, CTX_LETS_BELOW))) return false;
                if (sdj_find(J, mv, "repeat", &x) && !(m.repeat = cc_expr(c, x, CTX_LETS_BELOW))) return false;
                if (!(m.text = expr_field(c, mv, "text", true, CTX_LETS_BELOW, true))) return false;
                int e = end_kind(c, mv);
                if (e < 0) return false;
                m.end = (uint8_t)e;
                memcpy(c->p + tab + q * sizeof(sdp_msg_t), &m, sizeof(m));
                q++;
            }
        }
    }
    if (sdj_find(J, root, "requests", &v)) {
        static const char *const keys[] = { "match", "match_line", "answer", "text", "end", "delay_ms", "description", NULL };
        if (!want_array(c, v)) return false;
        uint32_t n = sdj_count(J, v);
        if (n > SD_MAX_REQS) { cc_fail(c, SD_E_LIMIT, sdj_ws(J, v), -1, "more than 32 requests", NULL); return false; }
        if (n) {
            uint32_t tab = cc_alloc(c, n * sizeof(sdp_req_t));
            if (!tab) return false;
            h->reqs = (uint16_t)tab;
            h->n_reqs = (uint8_t)n;
            uint32_t it = v, rv, q = 0;
            while (sdj_arr_next(J, &it, &rv)) {
                if (!check_keys(c, rv, keys)) return false;
                sdp_req_t r;
                memset(&r, 0, sizeof(r));
                bool hm = sdj_find(J, rv, "match", &x);
                uint32_t ml;
                bool hl = sdj_find(J, rv, "match_line", &ml);
                if (hm == hl) { cc_fail(c, SD_E_SCHEMA, sdj_ws(J, rv), -1, "a request has 'match' or 'match_line'", NULL); return false; }
                if (hm) {
                    if (!want_array(c, x)) return false;
                    uint32_t cnt = sdj_count(J, x);
                    if (cnt == 0 || cnt > SD_MAX_MATCH) { cc_fail(c, SD_E_LIMIT, sdj_ws(J, x), -1, "match: 1 to 64 bytes", NULL); return false; }
                    uint32_t rec = cc_alloc(c, cnt + (cnt + 7) / 8);
                    if (!rec) return false;
                    uint32_t ai = x, av, k = 0;
                    while (sdj_arr_next(J, &ai, &av)) {
                        if (sdj_type(J, av) == '"') {
                            char b[4];
                            size_t bn;
                            if (!sdj_str(J, av, b, sizeof(b), &bn) || bn != 1 || b[0] != '*') { cc_fail(c, SD_E_SCHEMA, sdj_ws(J, av), -1, "a byte or \"*\"", NULL); return false; }
                            c->p[rec + cnt + k / 8] |= (uint8_t)(1u << (k % 8));
                        } else {
                            uint32_t b;
                            if (!get_int(c, av, 0, 255, &b)) return false;
                            c->p[rec + k] = (uint8_t)b;
                        }
                        k++;
                    }
                    r.match = (uint16_t)rec;
                    r.mlen = (uint8_t)cnt;
                } else {
                    char b[SD_MAX_MATCH + 1];
                    size_t bn;
                    if (!get_text(c, ml, b, sizeof(b), &bn)) return false;
                    if (bn == 0 || bn > SD_MAX_MATCH || strchr(b, '\r') || strchr(b, '\n')) { cc_fail(c, SD_E_SCHEMA, sdj_ws(J, ml), -1, "match_line: 1 to 64 characters, no CR/LF", NULL); return false; }
                    uint32_t rec = cc_alloc(c, bn);
                    if (!rec) return false;
                    memcpy(c->p + rec, b, bn);
                    r.match = (uint16_t)rec;
                    r.mlen = (uint8_t)bn;
                    r.line = 1;
                }
                bool ha = sdj_find(J, rv, "answer", &x);
                uint32_t tv;
                bool ht = sdj_find(J, rv, "text", &tv);
                if (ha == ht) { cc_fail(c, SD_E_SCHEMA, sdj_ws(J, rv), -1, "a request has 'answer' (bytes) or 'text'", NULL); return false; }
                if (ha) {
                    if (!want_array(c, x)) return false;
                    uint32_t cnt = sdj_count(J, x);
                    if (cnt == 0 || cnt > 255) { cc_fail(c, SD_E_LIMIT, sdj_ws(J, x), -1, "answer: 1 to 255 bytes", NULL); return false; }
                    uint32_t arr = cc_alloc(c, 2u * cnt);
                    if (!arr) return false;
                    uint32_t ai = x, av, k = 0;
                    while (sdj_arr_next(J, &ai, &av)) {
                        uint16_t code = cc_expr(c, av, CTX_LETS_BELOW);
                        if (!code) return false;
                        c->p[arr + 2 * k] = (uint8_t)code;
                        c->p[arr + 2 * k + 1] = (uint8_t)(code >> 8);
                        k++;
                    }
                    r.answer = (uint16_t)arr;
                    r.alen = (uint8_t)cnt;
                } else {
                    if (!(r.answer = cc_template(c, tv, CTX_LETS_BELOW))) return false;
                    r.text = 1;
                }
                int e = end_kind(c, rv);
                if (e < 0) return false;
                r.end = (uint8_t)e;
                if (sdj_find(J, rv, "delay_ms", &x)) { uint32_t d; if (!get_int(c, x, 0, 60000, &d)) return false; r.delay_ms = (uint16_t)d; }
                memcpy(c->p + tab + q * sizeof(sdp_req_t), &r, sizeof(r));
                q++;
            }
        }
    }
    if (!h->n_msgs && !h->n_reqs) { cc_fail(c, SD_E_SCHEMA, root, -1, "kind uart needs 'messages' or 'requests'", NULL); return false; }
    return true;
}

static bool compile_analog(sd_cc_t *c, uint32_t root) {
    sdp_hdr_t *h = c->h;
    uint32_t v, x;
    static const char *const outs[] = { "voltage", "current", NULL };
    static const char *const exc[] = { "none", "adc", NULL };
    static const char *const rkeys[] = { "min", "max", NULL };
    int e;
    if (sdj_find(J, root, "output", &x)) { if ((e = get_enum(c, x, outs)) < 0) return false; h->current = (uint8_t)e; }
    if (sdj_find(J, root, "excitation", &x)) { if ((e = get_enum(c, x, exc)) < 0) return false; h->excitation = (uint8_t)e; }
    if (!sdj_find(J, root, "range", &v)) { cc_fail(c, SD_E_SCHEMA, root, -1, "kind analog needs 'range'", NULL); return false; }
    if (!check_keys(c, v, rkeys)) return false;
    if (!sdj_find(J, v, "min", &x) || !get_num(c, x, -1e15, 1e15, false, &h->range_min)) { if (!c->failed) cc_fail(c, SD_E_SCHEMA, sdj_ws(J, v), -1, "missing 'min'", NULL); return false; }
    if (!sdj_find(J, v, "max", &x) || !get_num(c, x, -1e15, 1e15, false, &h->range_max)) { if (!c->failed) cc_fail(c, SD_E_SCHEMA, sdj_ws(J, v), -1, "missing 'max'", NULL); return false; }
    if (h->range_min > h->range_max) { cc_fail(c, SD_E_SCHEMA, sdj_ws(J, v), -1, "min is above max", NULL); return false; }
    if (!(h->value = expr_field(c, root, "value", true, (uint8_t)(CTX_LETS_BELOW | (h->excitation ? CTX_ANALOG : 0)), false))) return false;
    return true;
}

/* ---- trial run: start the program once at its defaults ---- */
static int dry_run(sd_cc_t *c) {
    sd_inst_t *in = c->dry;
    sd_err_t e;
    int rc = sd_inst_init(in, c->p, c->len, &e);
    if (rc) { *c->err = e; return rc; }
    sd_set_time_ms(in, 0);
    switch (c->h->kind) {
    case SD_KIND_I2C: case SD_KIND_SPI: {
        uint8_t img[SD_REGIMAGE];
        sd_img_result_t r;
        rc = sd_regs_power_on(in, img, &r);
        if (!rc) rc = sd_regs_after_write(in, img, -1, -1, img, &r);
        if (!rc && c->h->has_fifo) { size_t n; uint8_t fr[64]; double hz; bool en; rc = sd_fifo_eval(in, img, &hz, &en); if (!rc) rc = sd_fifo_frame(in, img, fr, sizeof(fr), &n); }
        if (!rc && c->h->has_drdy) { double hz; bool en; rc = sd_drdy_eval(in, img, &hz, &en); }
        break;
    }
    case SD_KIND_UART: {
        char *buf = (char *)c->src;   /* the expression buffer is free now */
        size_t n;
        int64_t at;
        if (c->h->n_msgs) rc = sd_uart_epoch(in, buf, c->src_cap, &n, &at);
        /* each answer once */
        const sdp_req_t *rq = (const sdp_req_t *)(const void *)(c->p + c->h->reqs);
        for (unsigned i = 0; !rc && i < c->h->n_reqs; i++) {
            const uint8_t *m = c->p + rq[i].match;
            uint32_t d;
            for (unsigned k = 0; k < rq[i].mlen && !rc; k++) {
                int r2 = sd_uart_rx(in, 0, m[k], (uint8_t *)buf, c->src_cap, &n, &d);
                if (r2 < 0) rc = r2;
            }
            if (!rc && rq[i].line) { int r2 = sd_uart_rx(in, 0, '\n', (uint8_t *)buf, c->src_cap, &n, &d); if (r2 < 0) rc = r2; }
        }
        break;
    }
    case SD_KIND_ANALOG: {
        double out;
        rc = sd_analog_eval(in, 0.0, &out);
        break;
    }
    default: break;
    }
    if (rc) {
        *c->err = in->err;
        c->err->code = in->err.code ? in->err.code : rc;
        /* map the failing code back to the expression it came from */
        int32_t pc = in->err.expr_off;
        uint32_t best = 0, best_span = 0xFFFFFFFFu;
        for (unsigned i = 0; i < c->nranges; i++) {
            const sd_range_t *r = &c->ranges[i];
            if (pc >= (int32_t)r->code_lo && pc <= (int32_t)r->code_hi && (uint32_t)(r->code_hi - r->code_lo) < best_span) {
                best_span = (uint32_t)(r->code_hi - r->code_lo);
                best = r->json;
            }
        }
        c->err->json_off = best;
        c->err->expr_off = -1;
        c->failed = true;
    }
    return rc;
}

/* ---- entry point ---- */
int sd_compile(const char *json, size_t json_len, void *prog, size_t prog_cap,
               void *work, size_t work_cap, size_t *prog_len, sd_err_t *err) {
    sd_err_t dummy;
    if (!err) err = &dummy;
    memset(err, 0, sizeof(*err));
    err->expr_off = -1;
    if (prog_len) *prog_len = 0;
    if (!json || !prog || !work || ((uintptr_t)prog & 7u) || ((uintptr_t)work & 7u)) {
        err->code = SD_E_ARG;
        memcpy(err->msg, "bad arguments (buffers must be 8-byte aligned)", 47);
        return SD_E_ARG;
    }
    if (json_len > SD_JSON_MAX) {
        err->code = SD_E_LIMIT;
        memcpy(err->msg, "definition larger than 32 KB", 29);
        return SD_E_LIMIT;
    }
    size_t need = SD_CC_SYMS * sizeof(sd_sym_t) + SD_CC_NODES * sizeof(sd_node_t) +
                  SD_CC_RANGES * sizeof(sd_range_t) + SD_CC_SRC + 16 + sizeof(sd_inst_t) + 16;
    if (work_cap < need) {
        err->code = SD_E_SPACE;
        memcpy(err->msg, "compile scratch too small", 26);
        return SD_E_SPACE;
    }
    uint32_t joff = 0;
    const char *why = NULL;
    int rc = sdj_validate(json, json_len, &joff, &why);
    if (rc) {
        err->code = rc;
        err->json_off = joff;
        size_t l = strlen(why);
        memcpy(err->msg, why, l < sizeof(err->msg) - 1 ? l : sizeof(err->msg) - 1);
        return rc;
    }

    sd_cc_t cc;
    sd_cc_t *c = &cc;
    memset(c, 0, sizeof(cc));
    c->j.s = json;
    c->j.n = (uint32_t)json_len;
    c->root = sdj_ws(&c->j, 0);
    c->p = (uint8_t *)prog;
    c->cap = prog_cap < SD_PROG_MAX ? (uint32_t)prog_cap : SD_PROG_MAX;
    c->err = err;
    c->func_limit = c->frag_limit = 0xFF;
    uint8_t *w = (uint8_t *)work;
    size_t wo = 0;
    c->dry = (sd_inst_t *)(void *)(w + wo); wo += (sizeof(sd_inst_t) + 7u) & ~(size_t)7u;
    c->syms = (sd_sym_t *)(void *)(w + wo); wo += SD_CC_SYMS * sizeof(sd_sym_t);
    c->nodes = (sd_node_t *)(void *)(w + wo); wo += SD_CC_NODES * sizeof(sd_node_t);
    c->ranges = (sd_range_t *)(void *)(w + wo); wo += SD_CC_RANGES * sizeof(sd_range_t);
    c->src = (char *)(w + wo); c->src_cap = SD_CC_SRC;

    uint32_t root = c->root, v, x;
    if (sdj_type(J, root) != '{') { cc_fail(c, SD_E_SCHEMA, root, -1, "a definition is an object", NULL); goto out; }

    uint32_t hdr = cc_alloc(c, sizeof(sdp_hdr_t));
    if (c->failed) goto out;
    (void)hdr;
    c->h = (sdp_hdr_t *)(void *)c->p;
    c->h->magic = SDP_MAGIC;
    c->h->version = SDP_VERSION;
    c->h->utc_param = -1;

    /* header */
    {
        static const char *const kinds[] = { "", "i2c", "spi", "uart", "analog", NULL };
        uint32_t f;
        if (!sdj_find(J, root, "format", &v)) { cc_fail(c, SD_E_SCHEMA, root, -1, "missing 'format: 1'", NULL); goto out; }
        if (!get_int(c, v, 1, 1, &f)) { c->err->msg[0] = 0; c->failed = false; cc_fail(c, SD_E_SCHEMA, sdj_ws(J, v), -1, "this pod reads format 1", NULL); goto out; }
        if (!sdj_find(J, root, "kind", &v)) { cc_fail(c, SD_E_SCHEMA, root, -1, "missing 'kind'", NULL); goto out; }
        int k = get_enum(c, v, kinds);
        if (k <= 0) { if (!c->failed) cc_fail(c, SD_E_SCHEMA, sdj_ws(J, v), -1, "kind: i2c, spi, uart or analog", NULL); goto out; }
        c->h->kind = (uint8_t)k;
        static const char *const common[] = { "format", "kind", "name", "label", "description", "datasheet",
            "version", "license", "params", "constants", "functions", "let", "state", "fragments", NULL };
        static const char *const regkeys[] = { "i2c", "spi", "registers", "commands", "conversion", "on_write",
            "data_ready", "fifo", "fill", NULL };
        static const char *const uartkeys[] = { "uart", "messages", "requests", NULL };
        static const char *const analogkeys[] = { "output", "value", "range", "excitation", NULL };
        uint32_t it = root, kk, vv;
        while (sdj_obj_next(J, &it, &kk, &vv)) {
            bool ok = false;
            for (const char *const *a = common; *a && !ok; a++) ok = sdj_key_is(J, kk, *a);
            const char *const *ks = k <= SD_KIND_SPI ? regkeys : (k == SD_KIND_UART ? uartkeys : analogkeys);
            for (const char *const *a = ks; *a && !ok; a++) {
                if (!sdj_key_is(J, kk, *a)) continue;
                if ((k == SD_KIND_I2C && strcmp(*a, "spi") == 0) || (k == SD_KIND_SPI && strcmp(*a, "i2c") == 0)) break;
                ok = true;
            }
            if (!ok) {
                char nb[40];
                size_t n;
                if (!sdj_str(J, kk, nb, sizeof(nb), &n)) memcpy(nb, "?", 2);
                cc_fail(c, SD_E_SCHEMA, kk, -1, "unknown key '%s' (for this kind)", nb);
                goto out;
            }
        }
        char b[256];
        size_t n;
        if (!sdj_find(J, root, "name", &v)) { cc_fail(c, SD_E_SCHEMA, root, -1, "missing 'name'", NULL); goto out; }
        if (!get_text(c, v, b, sizeof(b), &n)) goto out;
        bool okn = n >= 1 && n <= 32;
        for (size_t q = 0; q < n && okn; q++) okn = (b[q] >= 'a' && b[q] <= 'z') || (b[q] >= '0' && b[q] <= '9') || b[q] == '_';
        if (!okn) { cc_fail(c, SD_E_SCHEMA, sdj_ws(J, v), -1, "name: [a-z0-9_], 1 to 32 characters", NULL); goto out; }
        if (!(c->h->name = cc_string(c, b, n))) goto out;
        if (sdj_find(J, root, "label", &v)) {
            if (!get_text(c, v, b, sizeof(b), &n)) goto out;
            if (!(c->h->label = cc_string(c, b, n))) goto out;
        }
        static const char *const texts[] = { "description", "datasheet", "version", "license", NULL };
        for (int q = 0; texts[q]; q++)
            if (sdj_find(J, root, texts[q], &v) && sdj_type(J, v) != '"') { cc_fail(c, SD_E_SCHEMA, sdj_ws(J, v), -1, "expected text", NULL); goto out; }
    }

    /* names first, then everything in dependency order */
    uint32_t consts = 0, funcs = 0, frags = 0, lets = 0, states = 0;
    bool hc = sdj_find(J, root, "constants", &consts), hf = sdj_find(J, root, "functions", &funcs);
    bool hg = sdj_find(J, root, "fragments", &frags), hl = sdj_find(J, root, "let", &lets);
    bool hs = sdj_find(J, root, "state", &states);
    if (sdj_find(J, root, "params", &v) && !compile_params(c, v)) goto out;
    uint16_t first_func = 0, first_frag = 0, first_let = 0, first_state = 0;
    if (hf) { first_func = c->nsyms; if (!syms_from_keys(c, funcs, SYM_FUNC, SD_MAX_FUNCS, "more than 16 functions")) goto out; }
    if (hg) { first_frag = c->nsyms; if (!syms_from_keys(c, frags, SYM_FRAG, SD_MAX_FRAGS, "more than 16 fragments")) goto out; }
    if (hl) {
        if (!want_array(c, lets)) goto out;
        if (sdj_count(J, lets) > SD_MAX_LETS) { cc_fail(c, SD_E_LIMIT, sdj_ws(J, lets), -1, "more than 64 let entries", NULL); goto out; }
        first_let = c->nsyms;
        uint32_t it = lets, lv;
        uint8_t q = 0;
        while (sdj_arr_next(J, &it, &lv)) {
            uint32_t k, val, mi = lv, len;
            bool esc;
            if (sdj_type(J, lv) != '{' || sdj_count(J, lv) != 1) { cc_fail(c, SD_E_SCHEMA, sdj_ws(J, lv), -1, "a let entry is one 'name: expression'", NULL); goto out; }
            sdj_obj_next(J, &mi, &k, &val);
            const char *r = sdj_raw(J, k, &len, &esc);
            if (add_sym(c, k, r, len, esc, SYM_LET, q++) < 0) goto out;
        }
    }
    if (hs) {
        if (!want_array(c, states)) goto out;
        if (sdj_count(J, states) > SD_MAX_STATES) { cc_fail(c, SD_E_LIMIT, sdj_ws(J, states), -1, "more than 16 states", NULL); goto out; }
        first_state = c->nsyms;
        uint32_t it = states, sv;
        uint8_t q = 0;
        while (sdj_arr_next(J, &it, &sv)) {
            uint32_t len;
            bool esc;
            if (sdj_type(J, sv) != '{' || !sdj_find(J, sv, "name", &x) || sdj_type(J, x) != '"') { cc_fail(c, SD_E_SCHEMA, sdj_ws(J, sv), -1, "a state needs 'name'", NULL); goto out; }
            const char *r = sdj_raw(J, x, &len, &esc);
            if (add_sym(c, sdj_ws(J, x), r, len, esc, SYM_STATE, q++) < 0) goto out;
        }
    }
    if (hc && !compile_constants(c, consts)) goto out;
    if (hf && !compile_funcs(c, funcs, false, first_func)) goto out;
    if (hg && !compile_funcs(c, frags, true, first_frag)) goto out;
    if (hs && !compile_states(c, states, first_state)) goto out;
    if (hl && !compile_lets(c, lets, first_let)) goto out;
    c->let_limit = c->h->n_lets;
    (void)first_state;

    switch (c->h->kind) {
    case SD_KIND_I2C: if (!compile_i2c_spi(c, root, false)) goto out; break;
    case SD_KIND_SPI: if (!compile_i2c_spi(c, root, true)) goto out; break;
    case SD_KIND_UART: if (!compile_uart(c, root)) goto out; break;
    case SD_KIND_ANALOG: if (!compile_analog(c, root)) goto out; break;
    default: break;
    }
    cc_align(c);
    if (c->failed) goto out;
    c->h->size = c->len;
    if (dry_run(c)) goto out;
    if (prog_len) *prog_len = c->len;
    return SD_OK;
out:
    if (!err->code) err->code = SD_E_SCHEMA;
    sdj_path_at(&c->j, c->root, err->json_off, err->path, sizeof(err->path));
    return err->code;
}
