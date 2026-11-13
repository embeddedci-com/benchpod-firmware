/* ============================================================================
 * sensor_def_json.c — a bounded JSON reader for sensor definitions.
 *
 * sdj_validate() checks the whole document once (RFC 8259 grammar, nesting at
 * most SDJ_MAX_DEPTH, strings without raw control characters).  The walkers
 * after it assume a valid document and move between values by offset, so the
 * reader needs no token array and no heap.
 * ========================================================================== */
#include "sensor_def_int.h"

typedef struct { const char *s; uint32_t n, o; const char *why; } sdjv_t;

static bool is_ws(char c) { return c == ' ' || c == '\t' || c == '\n' || c == '\r'; }
static void vws(sdjv_t *v) { while (v->o < v->n && is_ws(v->s[v->o])) v->o++; }
static bool hexd(char c) { return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F'); }
static bool digit(char c) { return c >= '0' && c <= '9'; }

static bool v_string(sdjv_t *v) {
    v->o++;   /* the opening quote */
    while (v->o < v->n) {
        unsigned char c = (unsigned char)v->s[v->o];
        if (c == '"') { v->o++; return true; }
        if (c < 0x20) { v->why = "control character in a string"; return false; }
        if (c == '\\') {
            if (v->o + 1 >= v->n) break;
            char e = v->s[v->o + 1];
            if (e == 'u') {
                if (v->o + 5 >= v->n) break;
                for (unsigned k = 2; k < 6; k++)
                    if (!hexd(v->s[v->o + k])) { v->why = "bad \\u escape"; return false; }
                v->o += 6;
                continue;
            }
            if (!strchr("\"\\/bfnrt", e) || e == 0) { v->why = "bad escape"; return false; }
            v->o += 2;
            continue;
        }
        v->o++;
    }
    v->why = "unterminated string";
    return false;
}

static bool v_number(sdjv_t *v) {
    uint32_t o = v->o;
    if (o < v->n && v->s[o] == '-') o++;
    if (o >= v->n) goto bad;
    if (v->s[o] == '0') o++;
    else if (digit(v->s[o])) { while (o < v->n && digit(v->s[o])) o++; }
    else goto bad;
    if (o < v->n && v->s[o] == '.') {
        o++;
        if (o >= v->n || !digit(v->s[o])) goto bad;
        while (o < v->n && digit(v->s[o])) o++;
    }
    if (o < v->n && (v->s[o] == 'e' || v->s[o] == 'E')) {
        o++;
        if (o < v->n && (v->s[o] == '+' || v->s[o] == '-')) o++;
        if (o >= v->n || !digit(v->s[o])) goto bad;
        while (o < v->n && digit(v->s[o])) o++;
    }
    v->o = o;
    return true;
bad:
    v->why = "bad number";
    return false;
}

static bool v_value(sdjv_t *v, int depth) {
    vws(v);
    if (v->o >= v->n) { v->why = "unexpected end"; return false; }
    char c = v->s[v->o];
    if (c == '{' || c == '[') {
        if (depth >= SDJ_MAX_DEPTH) { v->why = "nested too deep"; return false; }
        char close = c == '{' ? '}' : ']';
        v->o++;
        vws(v);
        if (v->o < v->n && v->s[v->o] == close) { v->o++; return true; }
        for (;;) {
            if (c == '{') {
                vws(v);
                if (v->o >= v->n || v->s[v->o] != '"') { v->why = "expected a key"; return false; }
                if (!v_string(v)) return false;
                vws(v);
                if (v->o >= v->n || v->s[v->o] != ':') { v->why = "expected ':'"; return false; }
                v->o++;
            }
            if (!v_value(v, depth + 1)) return false;
            vws(v);
            if (v->o >= v->n) { v->why = "unexpected end"; return false; }
            if (v->s[v->o] == ',') { v->o++; continue; }
            if (v->s[v->o] == close) { v->o++; return true; }
            v->why = c == '{' ? "expected ',' or '}'" : "expected ',' or ']'";
            return false;
        }
    }
    if (c == '"') return v_string(v);
    if (c == '-' || digit(c)) return v_number(v);
    static const char *const lit[] = { "true", "false", "null" };
    for (unsigned i = 0; i < 3; i++) {
        size_t l = strlen(lit[i]);
        if (v->n - v->o >= l && memcmp(v->s + v->o, lit[i], l) == 0) { v->o += (uint32_t)l; return true; }
    }
    v->why = "unexpected character";
    return false;
}

int sdj_validate(const char *s, size_t n, uint32_t *err_off, const char **why) {
    if (!s || n > SD_JSON_MAX) {
        if (err_off) *err_off = 0;
        if (why) *why = "too large";
        return SD_E_LIMIT;
    }
    sdjv_t v = { s, (uint32_t)n, 0, NULL };
    bool ok = v_value(&v, 0);
    if (ok) {
        vws(&v);
        if (v.o != v.n) { ok = false; v.why = "text after the document"; }
    }
    if (!ok) {
        if (err_off) *err_off = v.o;
        if (why) *why = v.why ? v.why : "invalid";
        return SD_E_JSON;
    }
    return SD_OK;
}

/* ---- walkers (valid documents only) ---- */
uint32_t sdj_ws(const sdj_t *j, uint32_t o) {
    while (o < j->n && is_ws(j->s[o])) o++;
    return o;
}

char sdj_type(const sdj_t *j, uint32_t o) {
    o = sdj_ws(j, o);
    if (o >= j->n) return 0;
    char c = j->s[o];
    if (c == '{' || c == '[' || c == '"') return c;
    if (c == '-' || digit(c)) return 'n';
    if (c == 't') return 't';
    if (c == 'f') return 'f';
    return 'z';
}

static uint32_t skip_string(const sdj_t *j, uint32_t o) {
    o++;
    while (o < j->n && j->s[o] != '"') o += (j->s[o] == '\\') ? 2u : 1u;
    return o + 1;
}

uint32_t sdj_skip(const sdj_t *j, uint32_t o) {
    o = sdj_ws(j, o);
    if (o >= j->n) return o;
    char c = j->s[o];
    if (c == '"') return skip_string(j, o);
    if (c == '{' || c == '[') {
        int depth = 0;
        while (o < j->n) {
            char d = j->s[o];
            if (d == '"') { o = skip_string(j, o); continue; }
            if (d == '{' || d == '[') depth++;
            else if (d == '}' || d == ']') { if (--depth == 0) return o + 1; }
            o++;
        }
        return o;
    }
    while (o < j->n && !is_ws(j->s[o]) && j->s[o] != ',' && j->s[o] != '}' && j->s[o] != ']') o++;
    return o;
}

/* *it: the container's opening offset at first, then the offset after the last member. */
static bool next_member(const sdj_t *j, uint32_t *it, bool obj, uint32_t *key, uint32_t *val) {
    uint32_t o = sdj_ws(j, *it);
    if (o >= j->n) return false;
    char c = j->s[o];
    if (c == '{' || c == '[') {
        if ((c == '{') != obj) return false;
        o = sdj_ws(j, o + 1);
    } else if (c == ',') {
        o = sdj_ws(j, o + 1);
    } else {
        return false;   /* '}' / ']' */
    }
    if (o >= j->n || j->s[o] == '}' || j->s[o] == ']') { *it = o; return false; }
    if (obj) {
        if (key) *key = o;
        o = sdj_ws(j, skip_string(j, o));
        o = sdj_ws(j, o + 1);   /* ':' */
    }
    if (val) *val = o;
    *it = sdj_ws(j, sdj_skip(j, o));
    return true;
}

bool sdj_obj_next(const sdj_t *j, uint32_t *it, uint32_t *key, uint32_t *val) {
    return next_member(j, it, true, key, val);
}
bool sdj_arr_next(const sdj_t *j, uint32_t *it, uint32_t *val) {
    return next_member(j, it, false, NULL, val);
}

uint32_t sdj_count(const sdj_t *j, uint32_t o) {
    o = sdj_ws(j, o);
    char t = sdj_type(j, o);
    uint32_t it = o, n = 0, k, v;
    if (t == '{') while (sdj_obj_next(j, &it, &k, &v)) n++;
    else if (t == '[') while (sdj_arr_next(j, &it, &v)) n++;
    return n;
}

const char *sdj_raw(const sdj_t *j, uint32_t o, uint32_t *len, bool *has_escape) {
    o = sdj_ws(j, o);
    uint32_t e = skip_string(j, o) - 1;
    bool esc = false;
    for (uint32_t i = o + 1; i < e; i++) if (j->s[i] == '\\') { esc = true; break; }
    if (len) *len = e - o - 1;
    if (has_escape) *has_escape = esc;
    return j->s + o + 1;
}

bool sdj_key_is(const sdj_t *j, uint32_t key, const char *name) {
    uint32_t len;
    bool esc;
    const char *r = sdj_raw(j, key, &len, &esc);
    if (!esc) return strlen(name) == len && memcmp(r, name, len) == 0;
    char buf[64];
    size_t n;
    if (!sdj_str(j, key, buf, sizeof(buf), &n)) return false;
    return strlen(name) == n && memcmp(buf, name, n) == 0;
}

bool sdj_find(const sdj_t *j, uint32_t obj, const char *name, uint32_t *val) {
    uint32_t it = obj, k, v;
    while (sdj_obj_next(j, &it, &k, &v))
        if (sdj_key_is(j, k, name)) { if (val) *val = v; return true; }
    return false;
}

static int hexv(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return c - 'A' + 10;
}

bool sdj_str(const sdj_t *j, uint32_t o, char *out, size_t cap, size_t *len) {
    uint32_t n;
    const char *r = sdj_raw(j, o, &n, NULL);
    size_t w = 0;
    for (uint32_t i = 0; i < n; i++) {
        char c = r[i];
        char tmp[4];
        size_t k = 1;
        if (c != '\\') {
            tmp[0] = c;
        } else {
            char e = r[++i];
            switch (e) {
            case 'b': tmp[0] = '\b'; break;
            case 'f': tmp[0] = '\f'; break;
            case 'n': tmp[0] = '\n'; break;
            case 'r': tmp[0] = '\r'; break;
            case 't': tmp[0] = '\t'; break;
            case 'u': {
                unsigned cp = 0;
                for (unsigned q = 1; q <= 4; q++) cp = cp * 16u + (unsigned)hexv(r[i + q]);
                i += 4;
                /* surrogates are kept as their own 3-byte sequences: names and templates are ASCII */
                if (cp < 0x80) tmp[0] = (char)cp;
                else if (cp < 0x800) { tmp[0] = (char)(0xC0 | (cp >> 6)); tmp[1] = (char)(0x80 | (cp & 0x3F)); k = 2; }
                else { tmp[0] = (char)(0xE0 | (cp >> 12)); tmp[1] = (char)(0x80 | ((cp >> 6) & 0x3F));
                       tmp[2] = (char)(0x80 | (cp & 0x3F)); k = 3; }
                break;
            }
            default: tmp[0] = e; break;
            }
        }
        if (w + k >= cap) return false;
        memcpy(out + w, tmp, k);
        w += k;
    }
    if (cap) out[w] = '\0';
    if (len) *len = w;
    return true;
}

bool sdj_num(const sdj_t *j, uint32_t o, double *v) {
    o = sdj_ws(j, o);
    if (sdj_type(j, o) != 'n') return false;
    uint32_t e = sdj_skip(j, o);
    size_t used = 0;
    double d = sd_parse_number(j->s + o, e - o, &used);
    if (used != e - o) return false;
    *v = d;
    return true;
}
