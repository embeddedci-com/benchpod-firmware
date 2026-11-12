/* ============================================================================
 * sensor_stim.c — stimulus files (format.md sections 7 and 8): how params
 * change over a test, and which faults happen when.
 *
 * sd_stim_compile() checks the JSON and lays it out flat in a program buffer
 * (segments in pre-order, a repeat followed by its body).  The player is a
 * pure function of time: sd_stim_value(s, i, t) is signal i at t seconds, and
 * sd_stim_step() reports every signal's value plus the faults that switched
 * on or off since the previous step.  Noise is a function of (seed, sample
 * index), so a replay gives the same values.
 * ========================================================================== */
#include "sensor_def_int.h"

#include <math.h>

#define STP_MAGIC 0x5453u   /* "ST" */
#define STIM_POINTS_MAX 1024u

enum { SH_HOLD = 1, SH_STEP, SH_RAMP, SH_SINE, SH_SAW, SH_PULSE, SH_NOISE, SH_DATA, SH_REPEAT };

typedef struct {
    uint8_t  shape;
    uint8_t  has_start;     /* ramp start / noise mean given */
    uint8_t  interp_step;   /* data: step interpolation */
    uint8_t  pad;
    uint16_t span;          /* records this one covers, itself included */
    uint16_t items;         /* repeat: top-level items in the body */
    uint16_t points;        /* data: offset of the [t, v] pairs */
    uint16_t npoints;
    uint32_t count;         /* repeat */
    uint64_t seed;
    double   a, b, c, d;    /* shape fields (see seg_value) */
    double   dur;           /* this record's whole duration (repeat: count * body) */
    double   body_dur;      /* repeat: one pass of the body */
} st_seg_t;

typedef struct {
    uint16_t name;          /* string record: "env.temperature_c" */
    uint16_t first;         /* first segment record */
    uint16_t items;         /* top-level segments */
    uint16_t pad;
    double   dur;
} st_sig_t;

typedef struct {
    uint16_t magic, version;
    uint8_t  n_signals, n_faults, repeat, pad;
    uint32_t size;
    double   rate_hz, duration;
    uint16_t signals, faults, segs, n_segs;
} st_hdr_t;

/* ---- compile ---- */
typedef struct {
    sdj_t    j;
    uint32_t root;
    uint8_t *p;
    uint32_t len, cap;
    uint16_t nseg;
    sd_err_t *err;
    bool     failed;
    uint16_t segoff[SD_STIM_SEGMENTS];   /* where each segment record went */
} st_cc_t;

static void sfail(st_cc_t *c, int code, uint32_t off, const char *msg) {
    if (c->failed) return;
    c->failed = true;
    c->err->code = code;
    c->err->json_off = sdj_ws(&c->j, off);
    c->err->expr_off = -1;
    size_t l = strlen(msg);
    if (l >= sizeof(c->err->msg)) l = sizeof(c->err->msg) - 1;
    memcpy(c->err->msg, msg, l);
    c->err->msg[l] = '\0';
}

static uint32_t salloc(st_cc_t *c, size_t n) {
    uint32_t at = (c->len + 7u) & ~7u;
    if (c->failed) return 0;
    if ((size_t)at + n > c->cap || at + n > 0xFFFF) { sfail(c, SD_E_LIMIT, c->root, "compiled stimulus larger than 8 KB"); return 0; }
    memset(c->p + c->len, 0, at + n - c->len);
    c->len = at + (uint32_t)n;
    return at;
}

static bool keys_ok(st_cc_t *c, uint32_t obj, const char *const *allowed) {
    if (sdj_type(&c->j, obj) != '{') { sfail(c, SD_E_SCHEMA, obj, "expected an object"); return false; }
    uint32_t it = obj, k, v;
    while (sdj_obj_next(&c->j, &it, &k, &v)) {
        bool ok = false;
        for (const char *const *a = allowed; *a && !ok; a++) ok = sdj_key_is(&c->j, k, *a);
        if (!ok) { sfail(c, SD_E_SCHEMA, k, "unknown key"); return false; }
    }
    return true;
}

static bool num_field(st_cc_t *c, uint32_t obj, const char *key, bool required, double lo, double hi, double *out) {
    uint32_t v;
    if (!sdj_find(&c->j, obj, key, &v)) {
        if (required) {
            char m[48] = "missing '";
            size_t l = strlen(m), k = strlen(key);
            if (k > 30) k = 30;
            memcpy(m + l, key, k); m[l + k] = '\''; m[l + k + 1] = '\0';
            sfail(c, SD_E_SCHEMA, obj, m);
        }
        return false;
    }
    double d;
    if (!sdj_num(&c->j, v, &d)) { sfail(c, SD_E_SCHEMA, v, "expected a number"); return false; }
    if (!(d >= lo && d <= hi)) { sfail(c, SD_E_SCHEMA, v, "number out of range"); return false; }
    *out = d;
    return true;
}

static bool word_field(const sdj_t *j, uint32_t obj, const char *key, char *buf, size_t cap, uint32_t *at) {
    uint32_t v;
    size_t n;
    if (!sdj_find(j, obj, key, &v) || sdj_type(j, v) != '"' || !sdj_str(j, v, buf, cap, &n)) return false;
    if (at) *at = v;
    return true;
}

static int compile_segs(st_cc_t *c, uint32_t arr, int depth, double *dur_out, uint16_t *items_out);

static bool compile_seg(st_cc_t *c, uint32_t obj, int depth, double *dur) {
    static const char *const names[] = { "", "hold", "step", "ramp", "sine", "saw", "pulse", "noise", "data", "repeat", NULL };
    static const char *const keys[][8] = {
        { NULL }, { "shape", "value", "duration_s", NULL }, { "shape", "value", NULL },
        { "shape", "start", "end", "duration_s", NULL },
        { "shape", "offset", "amplitude", "period_s", "phase_deg", "duration_s", NULL },
        { "shape", "low", "high", "period_s", "duration_s", NULL },
        { "shape", "low", "high", "period_s", "duty", "duration_s", NULL },
        { "shape", "mean", "sigma", "seed", "duration_s", NULL },
        { "shape", "points", "file", "interpolate", NULL },
        { "shape", "count", "segments", NULL } };
    if (sdj_type(&c->j, obj) != '{') { sfail(c, SD_E_SCHEMA, obj, "a segment is an object"); return false; }
    char sh[16];
    uint32_t shv;
    if (!word_field(&c->j, obj, "shape", sh, sizeof(sh), &shv)) { sfail(c, SD_E_SCHEMA, obj, "missing 'shape'"); return false; }
    int shape = 0;
    for (int k = 1; names[k]; k++) if (strcmp(names[k], sh) == 0) shape = k;
    if (!shape) { sfail(c, SD_E_SCHEMA, shv, "unknown shape"); return false; }
    if (!keys_ok(c, obj, keys[shape])) return false;
    if (c->nseg >= SD_STIM_SEGMENTS) { sfail(c, SD_E_LIMIT, obj, "more than 256 segments"); return false; }
    uint32_t at = salloc(c, sizeof(st_seg_t));
    if (!at) return false;
    uint16_t my = c->nseg++;
    c->segoff[my] = (uint16_t)at;
    st_seg_t s;
    memset(&s, 0, sizeof(s));
    s.shape = (uint8_t)shape;
    s.span = 1;
    const double BIG = 1e12;
    bool ok = true;
    switch (shape) {
    case SH_HOLD:
        ok = num_field(c, obj, "value", true, -BIG, BIG, &s.a) && num_field(c, obj, "duration_s", true, 0, BIG, &s.dur);
        break;
    case SH_STEP:
        ok = num_field(c, obj, "value", true, -BIG, BIG, &s.a);
        s.dur = 0;
        break;
    case SH_RAMP:
        s.has_start = num_field(c, obj, "start", false, -BIG, BIG, &s.a);
        ok = !c->failed && num_field(c, obj, "end", true, -BIG, BIG, &s.b) && num_field(c, obj, "duration_s", true, 0, BIG, &s.dur);
        break;
    case SH_SINE:
        ok = num_field(c, obj, "offset", true, -BIG, BIG, &s.a) && num_field(c, obj, "amplitude", true, -BIG, BIG, &s.b) &&
             num_field(c, obj, "period_s", true, 1e-6, BIG, &s.c) && num_field(c, obj, "duration_s", true, 0, BIG, &s.dur);
        if (ok) { s.d = 0; (void)num_field(c, obj, "phase_deg", false, -1e6, 1e6, &s.d); ok = !c->failed; }
        break;
    case SH_SAW: case SH_PULSE:
        ok = num_field(c, obj, "low", true, -BIG, BIG, &s.a) && num_field(c, obj, "high", true, -BIG, BIG, &s.b) &&
             num_field(c, obj, "period_s", true, 1e-6, BIG, &s.c) && num_field(c, obj, "duration_s", true, 0, BIG, &s.dur);
        if (ok && shape == SH_PULSE) ok = num_field(c, obj, "duty", true, 0, 1, &s.d);
        break;
    case SH_NOISE: {
        s.has_start = num_field(c, obj, "mean", false, -BIG, BIG, &s.a);
        ok = !c->failed && num_field(c, obj, "sigma", true, 0, BIG, &s.b) && num_field(c, obj, "duration_s", true, 0, BIG, &s.dur);
        double seed = 0;
        if (ok) { (void)num_field(c, obj, "seed", false, -9e15, 9e15, &seed); ok = !c->failed; }
        s.seed = (uint64_t)(int64_t)seed;
        break;
    }
    case SH_DATA: {
        uint32_t v;
        if (sdj_find(&c->j, obj, "file", &v)) { sfail(c, SD_E_SCHEMA, v, "file: the pod takes inline points (tools/yaml2json.py inlines the CSV)"); return false; }
        char ip[16];
        uint32_t iv;
        if (word_field(&c->j, obj, "interpolate", ip, sizeof(ip), &iv)) {
            if (strcmp(ip, "step") == 0) s.interp_step = 1;
            else if (strcmp(ip, "linear") != 0) { sfail(c, SD_E_SCHEMA, iv, "interpolate: linear or step"); return false; }
        } else if (sdj_find(&c->j, obj, "interpolate", &iv)) { sfail(c, SD_E_SCHEMA, iv, "interpolate: linear or step"); return false; }
        if (!sdj_find(&c->j, obj, "points", &v) || sdj_type(&c->j, v) != '[') { sfail(c, SD_E_SCHEMA, obj, "data needs 'points': [[t, value], ...]"); return false; }
        uint32_t n = sdj_count(&c->j, v);
        if (n == 0 || n > STIM_POINTS_MAX) { sfail(c, SD_E_LIMIT, v, "1 to 1024 points"); return false; }
        uint32_t pts = salloc(c, 16u * n);
        if (!pts) return false;
        uint32_t it = v, pv, k = 0;
        double last_t = -1e300;
        while (sdj_arr_next(&c->j, &it, &pv)) {
            uint32_t pi = pv, a, b;
            double tv, vv;
            if (sdj_type(&c->j, pv) != '[' || sdj_count(&c->j, pv) != 2 || !sdj_arr_next(&c->j, &pi, &a) ||
                !sdj_arr_next(&c->j, &pi, &b) || !sdj_num(&c->j, a, &tv) || !sdj_num(&c->j, b, &vv)) {
                sfail(c, SD_E_SCHEMA, pv, "a point is [seconds, value]"); return false;
            }
            if (tv < 0 || tv > BIG || tv < last_t || !(vv >= -BIG && vv <= BIG)) { sfail(c, SD_E_SCHEMA, pv, "point times start at 0 or later and never go back"); return false; }
            last_t = tv;
            memcpy(c->p + pts + 16u * k, &tv, 8);
            memcpy(c->p + pts + 16u * k + 8u, &vv, 8);
            k++;
        }
        s.points = (uint16_t)pts;
        s.npoints = (uint16_t)n;
        s.dur = last_t;
        break;
    }
    case SH_REPEAT: {
        double cnt;
        uint32_t v;
        if (depth >= (int)SD_STIM_DEPTH) { sfail(c, SD_E_LIMIT, obj, "repeats nested more than 4 deep"); return false; }
        if (!num_field(c, obj, "count", true, 1, 1e6, &cnt)) return false;
        if (cnt != floor(cnt)) { sfail(c, SD_E_SCHEMA, obj, "count: a whole number"); return false; }
        if (!sdj_find(&c->j, obj, "segments", &v)) { sfail(c, SD_E_SCHEMA, obj, "repeat needs 'segments'"); return false; }
        s.count = (uint32_t)cnt;
        double bd;
        uint16_t items;
        int span = compile_segs(c, v, depth + 1, &bd, &items);
        if (span < 0) return false;
        s.span = (uint16_t)(1 + span);
        s.items = items;
        s.body_dur = bd;
        s.dur = bd * cnt;
        break;
    }
    default: break;
    }
    if (!ok || c->failed) { if (!c->failed) sfail(c, SD_E_SCHEMA, obj, "bad segment"); return false; }
    memcpy(c->p + at, &s, sizeof(s));
    *dur = s.dur;
    return true;
}

/* Returns the records the list used, -1 on failure. */
static int compile_segs(st_cc_t *c, uint32_t arr, int depth, double *dur_out, uint16_t *items_out) {
    if (sdj_type(&c->j, arr) != '[') { sfail(c, SD_E_SCHEMA, arr, "expected a list of segments"); return -1; }
    uint16_t start = c->nseg;
    uint32_t it = arr, v;
    double total = 0;
    uint16_t items = 0;
    while (sdj_arr_next(&c->j, &it, &v)) {
        double d;
        if (!compile_seg(c, v, depth, &d)) return -1;
        total += d;
        items++;
    }
    if (!items) { sfail(c, SD_E_SCHEMA, arr, "empty segment list"); return -1; }
    *dur_out = total;
    *items_out = items;
    return c->nseg - start;
}

static bool name_ok(const char *s, size_t n) {
    if (n == 0 || n > 32) return false;
    for (size_t i = 0; i < n; i++)
        if (!((s[i] >= 'a' && s[i] <= 'z') || (s[i] >= 'A' && s[i] <= 'Z') || (s[i] >= '0' && s[i] <= '9') || s[i] == '_')) return false;
    return true;
}

static int parse_fault(const sdj_t *j, uint32_t obj, sd_fault_t *f, uint32_t *bad_off, const char **why) {
    static const char *const keys[] = { "type", "at_s", "duration_s", "sensor", "param", "value", "at", "mask", "rate_hz", NULL };
    memset(f, 0, sizeof(*f));
    f->duration_s = -1;
    f->at = -1;
    *bad_off = obj;
    if (sdj_type(j, obj) != '{') { *why = "a fault is an object"; return SD_E_SCHEMA; }
    uint32_t it = obj, k, v;
    while (sdj_obj_next(j, &it, &k, &v)) {
        bool ok = false;
        for (const char *const *a = keys; *a && !ok; a++) ok = sdj_key_is(j, k, *a);
        if (!ok) { *bad_off = k; *why = "unknown key"; return SD_E_SCHEMA; }
    }
    char t[16];
    uint32_t tv;
    if (!word_field(j, obj, "type", t, sizeof(t), &tv)) { *why = "missing 'type'"; return SD_E_SCHEMA; }
    for (int q = SD_FAULT_STUCK; q <= SD_FAULT_SHORT; q++) if (strcmp(sd_fault_name((sd_fault_type_t)q), t) == 0) f->type = (sd_fault_type_t)q;
    if (!f->type) { *bad_off = tv; *why = "unknown fault type"; return SD_E_SCHEMA; }
    struct { const char *key; double *dst; double lo, hi; bool *flag; } nums[] = {
        { "at_s", &f->at_s, 0, 1e12, NULL }, { "duration_s", &f->duration_s, 0, 1e12, NULL },
        { "value", &f->value, -1e12, 1e12, &f->has_value }, { "rate_hz", &f->rate_hz, 0, 1e6, NULL } };
    for (unsigned q = 0; q < sizeof(nums) / sizeof(nums[0]); q++) {
        if (!sdj_find(j, obj, nums[q].key, &v)) continue;
        double d;
        if (!sdj_num(j, v, &d) || !(d >= nums[q].lo && d <= nums[q].hi)) { *bad_off = v; *why = "number out of range"; return SD_E_SCHEMA; }
        *nums[q].dst = d;
        if (nums[q].flag) *nums[q].flag = true;
    }
    if (sdj_find(j, obj, "at", &v)) {
        double d;
        if (!sdj_num(j, v, &d) || d != floor(d) || d < 0 || d > 255) { *bad_off = v; *why = "at: a register 0..255"; return SD_E_SCHEMA; }
        f->at = (int16_t)d;
    }
    if (sdj_find(j, obj, "mask", &v)) {
        double d;
        if (!sdj_num(j, v, &d) || d != floor(d) || d < 1 || d > 255) { *bad_off = v; *why = "mask: 1..255"; return SD_E_SCHEMA; }
        f->mask = (uint8_t)d;
    }
    const char *const sk[2] = { "sensor", "param" };
    char *dst[2] = { f->sensor, f->param };
    for (int q = 0; q < 2; q++) {
        if (!sdj_find(j, obj, sk[q], &v)) continue;
        size_t n;
        if (sdj_type(j, v) != '"' || !sdj_str(j, v, dst[q], 33, &n) || !name_ok(dst[q], n)) { *bad_off = v; *why = "a name: letters, digits, _"; return SD_E_SCHEMA; }
    }
    *bad_off = obj;
    switch (f->type) {
    case SD_FAULT_OFFSET: case SD_FAULT_GAIN:
        if (!f->param[0] || !f->has_value) { *why = "offset and gain need 'param' and 'value'"; return SD_E_SCHEMA; }
        break;
    case SD_FAULT_CORRUPT:
        if (f->at < 0 || !f->mask) { *why = "corrupt needs 'at' and 'mask'"; return SD_E_SCHEMA; }
        break;
    case SD_FAULT_BUS_STUCK:
        if (f->duration_s < 0 || f->duration_s > 1) { *why = "bus_stuck lasts at most 1 s ('duration_s')"; return SD_E_SCHEMA; }
        break;
    case SD_FAULT_DROP:
        if (!f->has_value || f->value < 0 || f->value > 1) { *why = "drop needs 'value' 0..1"; return SD_E_SCHEMA; }
        break;
    case SD_FAULT_BAUD_ERROR:
        if (!f->has_value) { *why = "baud_error needs 'value' (percent)"; return SD_E_SCHEMA; }
        break;
    case SD_FAULT_GARBAGE:
        if (f->rate_hz <= 0) f->rate_hz = 100;
        break;
    default: break;
    }
    return SD_OK;
}

int sd_fault_from_json(const char *json, size_t len, sd_fault_t *out, sd_err_t *err) {
    sd_err_t dummy;
    if (!err) err = &dummy;
    memset(err, 0, sizeof(*err));
    err->expr_off = -1;
    if (!json || !out) { err->code = SD_E_ARG; return SD_E_ARG; }
    const char *why = "invalid";
    uint32_t off = 0;
    int rc = sdj_validate(json, len, &off, &why);
    if (!rc) {
        sdj_t j = { json, (uint32_t)len };
        rc = parse_fault(&j, sdj_ws(&j, 0), out, &off, &why);
        if (rc) sdj_path_at(&j, sdj_ws(&j, 0), off, err->path, sizeof(err->path));
    }
    if (rc) {
        err->code = rc;
        err->json_off = off;
        size_t l = strlen(why);
        if (l >= sizeof(err->msg)) l = sizeof(err->msg) - 1;
        memcpy(err->msg, why, l);
        err->msg[l] = '\0';
    }
    return rc;
}

int sd_stim_compile(const char *json, size_t json_len, void *prog, size_t prog_cap, size_t *prog_len, sd_err_t *err) {
    sd_err_t dummy;
    if (!err) err = &dummy;
    memset(err, 0, sizeof(*err));
    err->expr_off = -1;
    if (prog_len) *prog_len = 0;
    if (!json || !prog || ((uintptr_t)prog & 7u)) { err->code = SD_E_ARG; return SD_E_ARG; }
    const char *why = NULL;
    uint32_t off = 0;
    int rc = sdj_validate(json, json_len, &off, &why);
    if (rc) {
        err->code = rc;
        err->json_off = off;
        size_t l = strlen(why);
        memcpy(err->msg, why, l < sizeof(err->msg) - 1 ? l : sizeof(err->msg) - 1);
        return rc;
    }
    static const st_cc_t zero_cc;
    st_cc_t cc = zero_cc;
    cc.j.s = json;
    cc.j.n = (uint32_t)json_len;
    cc.p = (uint8_t *)prog;
    cc.cap = (uint32_t)(prog_cap < SD_STIM_MAX ? prog_cap : SD_STIM_MAX);
    cc.err = err;
    st_cc_t *c = &cc;
    c->root = sdj_ws(&c->j, 0);
    static const char *const keys[] = { "format", "kind", "name", "description", "repeat", "rate_hz", "signals", "faults", NULL };
    uint32_t v, hdr_at = 0;
    st_hdr_t h;
    memset(&h, 0, sizeof(h));
    if (!keys_ok(c, c->root, keys)) goto out;
    {
        double f;
        char k[16];
        uint32_t kv;
        if (!num_field(c, c->root, "format", true, 1, 1, &f)) { if (!c->failed) sfail(c, SD_E_SCHEMA, c->root, "format: 1"); goto out; }
        if (!word_field(&c->j, c->root, "kind", k, sizeof(k), &kv) || strcmp(k, "stimulus") != 0) { sfail(c, SD_E_SCHEMA, c->root, "kind: stimulus"); goto out; }
    }
    hdr_at = salloc(c, sizeof(st_hdr_t));   /* at 0 */
    if (c->failed) goto out;
    (void)hdr_at;
    h.magic = STP_MAGIC;
    h.version = 1;
    h.rate_hz = 50;
    if (sdj_find(&c->j, c->root, "rate_hz", &v)) { if (!num_field(c, c->root, "rate_hz", true, 1, 100, &h.rate_hz)) goto out; }
    if (sdj_find(&c->j, c->root, "repeat", &v)) {
        char t = sdj_type(&c->j, v);
        if (t != 't' && t != 'f') { sfail(c, SD_E_SCHEMA, v, "repeat: true or false"); goto out; }
        h.repeat = t == 't';
    }
    if (sdj_find(&c->j, c->root, "signals", &v)) {
        if (sdj_type(&c->j, v) != '{') { sfail(c, SD_E_SCHEMA, v, "signals: { name: [segments] }"); goto out; }
        uint32_t n = sdj_count(&c->j, v);
        if (n > SD_STIM_SIGNALS) { sfail(c, SD_E_LIMIT, v, "more than 16 signals"); goto out; }
        uint32_t tab = salloc(c, n * sizeof(st_sig_t));
        if (!tab && n) goto out;
        h.signals = (uint16_t)tab;
        h.n_signals = (uint8_t)n;
        /* names first (the segment records follow contiguously after) */
        uint32_t it = v, k, sv, q = 0;
        st_sig_t sig[SD_STIM_SIGNALS];
        memset(sig, 0, sizeof(sig));
        while (sdj_obj_next(&c->j, &it, &k, &sv)) {
            char nb[72];
            size_t nl;
            if (!sdj_str(&c->j, k, nb, sizeof(nb), &nl)) { sfail(c, SD_E_SCHEMA, k, "signal name too long"); goto out; }
            const char *dot = memchr(nb, '.', nl);
            bool ok = dot ? (name_ok(nb, (size_t)(dot - nb)) && name_ok(dot + 1, nl - (size_t)(dot - nb) - 1)) : name_ok(nb, nl);
            if (!ok) { sfail(c, SD_E_SCHEMA, k, "a signal is 'param' or 'instance.param'"); goto out; }
            for (uint32_t r = 0; r < q; r++) {
                uint8_t ol = c->p[sig[r].name];
                if (ol == nl && memcmp(c->p + sig[r].name + 1, nb, nl) == 0) { sfail(c, SD_E_NAME, k, "duplicate signal"); goto out; }
            }
            uint32_t at = c->len;
            if (at + 1 + nl > c->cap) { sfail(c, SD_E_LIMIT, k, "compiled stimulus larger than 8 KB"); goto out; }
            c->p[at] = (uint8_t)nl;
            memcpy(c->p + at + 1, nb, nl);
            c->len += 1u + (uint32_t)nl;
            sig[q].name = (uint16_t)at;
            q++;
        }
        it = v;
        q = 0;
        while (sdj_obj_next(&c->j, &it, &k, &sv)) {
            sig[q].first = c->nseg;
            if (compile_segs(c, sv, 0, &sig[q].dur, &sig[q].items) < 0) goto out;
            if (sig[q].dur > h.duration) h.duration = sig[q].dur;
            q++;
        }
        memcpy(c->p + tab, sig, n * sizeof(st_sig_t));
        h.n_segs = c->nseg;
    }
    if (sdj_find(&c->j, c->root, "faults", &v)) {
        if (sdj_type(&c->j, v) != '[') { sfail(c, SD_E_SCHEMA, v, "faults: a list"); goto out; }
        uint32_t n = sdj_count(&c->j, v);
        if (n > SD_STIM_FAULTS) { sfail(c, SD_E_LIMIT, v, "more than 32 faults"); goto out; }
        uint32_t tab = salloc(c, n * sizeof(sd_fault_t));
        if (!tab && n) goto out;
        h.faults = (uint16_t)tab;
        h.n_faults = (uint8_t)n;
        uint32_t it = v, fv, q = 0;
        while (sdj_arr_next(&c->j, &it, &fv)) {
            sd_fault_t f;
            uint32_t bad;
            const char *w = NULL;
            int r = parse_fault(&c->j, fv, &f, &bad, &w);
            if (r) { sfail(c, r, bad, w); goto out; }
            memcpy(c->p + tab + q * sizeof(sd_fault_t), &f, sizeof(f));
            q++;
        }
    }
    /* where each segment record is (data points sit between them) */
    if (c->nseg) {
        uint32_t idx = salloc(c, 2u * c->nseg);
        if (!idx) goto out;
        memcpy(c->p + idx, c->segoff, 2u * c->nseg);
        h.segs = (uint16_t)idx;
    }
    c->len = (c->len + 7u) & ~7u;
    h.size = c->len;
    memcpy(c->p, &h, sizeof(h));
    if (prog_len) *prog_len = c->len;
    return SD_OK;
out:
    if (!err->code) err->code = SD_E_SCHEMA;
    sdj_path_at(&c->j, c->root, err->json_off, err->path, sizeof(err->path));
    return err->code;
}

/* ---- player ---- */
static const st_hdr_t *sh(const sd_stim_t *s) { return (const st_hdr_t *)(const void *)s->prog; }
static const st_seg_t *seg(const sd_stim_t *s, unsigned i) {
    const uint16_t *idx = (const uint16_t *)(const void *)(s->prog + sh(s)->segs);
    return (const st_seg_t *)(const void *)(s->prog + idx[i]);
}

int sd_stim_init(sd_stim_t *s, const void *prog, size_t prog_len) {
    const st_hdr_t *h = (const st_hdr_t *)prog;
    if (!s || !prog || ((uintptr_t)prog & 7u) || prog_len < sizeof(st_hdr_t) || h->magic != STP_MAGIC ||
        h->version != 1 || h->size != prog_len)
        return SD_E_ARG;
    memset(s, 0, sizeof(*s));
    s->prog = (const uint8_t *)prog;
    return SD_OK;
}

unsigned sd_stim_n_signals(const sd_stim_t *s) { return sh(s)->n_signals; }
unsigned sd_stim_n_faults(const sd_stim_t *s) { return sh(s)->n_faults; }
double sd_stim_rate_hz(const sd_stim_t *s) { return sh(s)->rate_hz; }
double sd_stim_duration_s(const sd_stim_t *s) { return sh(s)->duration; }
bool sd_stim_repeats(const sd_stim_t *s) { return sh(s)->repeat != 0; }

static const st_sig_t *sig_of(const sd_stim_t *s, unsigned i) {
    return (const st_sig_t *)(const void *)(s->prog + sh(s)->signals) + i;
}

int sd_stim_signal(const sd_stim_t *s, unsigned i, const char **inst, size_t *inst_len,
                   const char **param, size_t *param_len) {
    if (i >= sh(s)->n_signals) return SD_E_ARG;
    const uint8_t *r = s->prog + sig_of(s, i)->name;
    const char *n = (const char *)r + 1;
    size_t l = r[0];
    const char *dot = memchr(n, '.', l);
    if (dot) {
        *inst = n; *inst_len = (size_t)(dot - n);
        *param = dot + 1; *param_len = l - (size_t)(dot - n) - 1;
    } else {
        *inst = n; *inst_len = 0;
        *param = n; *param_len = l;
    }
    return SD_OK;
}

int sd_stim_fault(const sd_stim_t *s, unsigned i, sd_fault_t *out) {
    if (i >= sh(s)->n_faults || !out) return SD_E_ARG;
    memcpy(out, s->prog + sh(s)->faults + i * sizeof(sd_fault_t), sizeof(*out));
    return SD_OK;
}

static double frac(double x) { return x - floor(x); }

static double gauss(uint64_t seed, int64_t k) {
    uint64_t a = seed * 0x9E3779B97F4A7C15ull + (uint64_t)k * 0xD1B54A32D192ED03ull;
    uint64_t x = a + 0x9E3779B97F4A7C15ull;
    x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull; x = (x ^ (x >> 27)) * 0x94D049BB133111EBull; x ^= x >> 31;
    uint64_t y = x + 0x9E3779B97F4A7C15ull;
    y = (y ^ (y >> 30)) * 0xBF58476D1CE4E5B9ull; y = (y ^ (y >> 27)) * 0x94D049BB133111EBull; y ^= y >> 31;
    double u1 = ((double)(x >> 11) + 1.0) / 9007199254740993.0;   /* (0, 1] */
    double u2 = (double)(y >> 11) / 9007199254740992.0;
    return sqrt(-2.0 * sd_log(u1)) * sd_cos(2.0 * SD_PI * u2);
}

static double list_value(const sd_stim_t *s, unsigned first, unsigned items, double t, double prev, bool *inside);
static double list_end(const sd_stim_t *s, unsigned first, unsigned items, double prev);

static double data_value(const sd_stim_t *s, const st_seg_t *g, double t) {
    const uint8_t *p = s->prog + g->points;
    double t0, v0;
    memcpy(&t0, p, 8); memcpy(&v0, p + 8, 8);
    if (t <= t0) return v0;
    for (unsigned k = 1; k < g->npoints; k++) {
        double t1, v1;
        memcpy(&t1, p + 16u * k, 8); memcpy(&v1, p + 16u * k + 8, 8);
        if (t < t1) {
            if (g->interp_step || t1 == t0) return v0;
            return v0 + (v1 - v0) * (t - t0) / (t1 - t0);
        }
        t0 = t1; v0 = v1;
    }
    return v0;
}

/* Segment value at local time t (0 <= t < dur), given the value before it. */
static double seg_value(const sd_stim_t *s, unsigned i, double t, double prev) {
    const st_seg_t *g = seg(s, i);
    switch (g->shape) {
    case SH_HOLD: case SH_STEP: return g->a;
    case SH_RAMP: {
        double a = g->has_start ? g->a : prev;
        return g->dur > 0 ? a + (g->b - a) * (t / g->dur) : g->b;
    }
    case SH_SINE: return g->a + g->b * sd_sin(2.0 * SD_PI * t / g->c + g->d * (SD_PI / 180.0));
    case SH_SAW: return g->a + (g->b - g->a) * frac(t / g->c);
    case SH_PULSE: return frac(t / g->c) < g->d ? g->b : g->a;
    case SH_NOISE: {
        double m = g->has_start ? g->a : prev;
        return m + g->b * gauss(g->seed, (int64_t)floor(t * sh(s)->rate_hz));
    }
    case SH_DATA: return data_value(s, g, t);
    case SH_REPEAT: {
        if (g->body_dur <= 0) return list_end(s, i + 1, g->items, prev);
        double k = floor(t / g->body_dur);
        if (k >= (double)g->count) k = (double)g->count - 1;
        double p = k >= 1 ? list_end(s, i + 1, g->items, prev) : prev;
        bool inside;
        return list_value(s, i + 1, g->items, t - k * g->body_dur, p, &inside);
    }
    default: return prev;
    }
}

/* The value a segment leaves behind. */
static double seg_end(const sd_stim_t *s, unsigned i, double prev) {
    const st_seg_t *g = seg(s, i);
    switch (g->shape) {
    case SH_HOLD: case SH_STEP: return g->a;
    case SH_RAMP: return g->b;
    case SH_NOISE: return g->has_start ? g->a : prev;
    case SH_DATA: { double v; memcpy(&v, s->prog + g->points + 16u * (g->npoints - 1u) + 8u, 8); return v; }
    case SH_REPEAT: return list_end(s, i + 1, g->items, prev);
    default: return seg_value(s, i, g->dur, prev);   /* sine, saw, pulse: their value at the end */
    }
}

static double list_end(const sd_stim_t *s, unsigned first, unsigned items, double prev) {
    unsigned i = first;
    for (unsigned q = 0; q < items; q++) {
        prev = seg_end(s, i, prev);
        i += seg(s, i)->span;
    }
    return prev;
}

static double list_value(const sd_stim_t *s, unsigned first, unsigned items, double t, double prev, bool *inside) {
    unsigned i = first;
    *inside = true;
    for (unsigned q = 0; q < items; q++) {
        const st_seg_t *g = seg(s, i);
        if (t < g->dur) return seg_value(s, i, t, prev);
        t -= g->dur;
        prev = seg_end(s, i, prev);
        i += g->span;
    }
    *inside = false;
    return prev;   /* after the last segment: hold its value */
}

static double eff_time(const sd_stim_t *s, double t) {
    const st_hdr_t *h = sh(s);
    if (t < 0) t = 0;
    if (h->repeat && h->duration > 0) t = fmod(t, h->duration);
    return t;
}

double sd_stim_value(const sd_stim_t *s, unsigned i, double t) {
    if (i >= sh(s)->n_signals) return 0;
    const st_sig_t *g = sig_of(s, i);
    bool inside;
    return list_value(s, g->first, g->items, eff_time(s, t), 0.0, &inside);
}

static bool fault_active(const sd_fault_t *f, double t) {
    return t >= f->at_s && (f->duration_s < 0 || t < f->at_s + f->duration_s);
}

int sd_stim_step(sd_stim_t *s, double t, sd_stim_event_t *ev, unsigned max) {
    const st_hdr_t *h = sh(s);
    unsigned n = 0;
    for (unsigned i = 0; i < h->n_signals && n < max; i++) {
        ev[n].kind = SD_EV_VALUE;
        ev[n].index = (uint8_t)i;
        ev[n].value = sd_stim_value(s, i, t);
        n++;
    }
    double te = eff_time(s, t);
    uint32_t now = 0;
    for (unsigned i = 0; i < h->n_faults; i++) {
        sd_fault_t f;
        memcpy(&f, s->prog + h->faults + i * sizeof(sd_fault_t), sizeof(f));
        if (fault_active(&f, te)) now |= 1u << i;
    }
    uint32_t was = s->stepped ? s->active : 0;
    for (unsigned i = 0; i < h->n_faults && n < max; i++) {
        uint32_t b = 1u << i;
        if ((now & b) == (was & b)) continue;
        ev[n].kind = (now & b) ? SD_EV_FAULT_ON : SD_EV_FAULT_OFF;
        ev[n].index = (uint8_t)i;
        ev[n].value = 0;
        n++;
    }
    s->active = now;
    s->stepped = true;
    s->t_last = t;
    return (int)n;
}
