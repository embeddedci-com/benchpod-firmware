/* ============================================================================
 * sensor_cmd.c — JSON commands of the emulated sensors (sensor_cmd.h).
 * ========================================================================== */
#include "sensor_cmd.h"
#include "sensor_mgr.h"
#include "sensor_port.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* A chunk: 1280 bytes is the cloud command cap, so data_b64 is at most ~1200 characters. */
#define CHUNK_B64_MAX  1240u

static const char *const k_verbs[] = {
    "sensor_define", "sensor_start", "sensor_set", "sensor_stimulus", "sensor_stimulus_start",
    "sensor_stimulus_stop", "sensor_fault", "sensor_fault_clear", "sensor_stop", "sensor_status",
    "sensor_regs", NULL,
};

bool sensor_cmd_known(const char *verb) {
    for (const char *const *v = k_verbs; *v; v++) if (verb && !strcmp(verb, *v)) return true;
    return false;
}

/* ---- base64 ---- */
static int b64v(unsigned char c) {
    if (c >= 'A' && c <= 'Z') return c - 'A';
    if (c >= 'a' && c <= 'z') return c - 'a' + 26;
    if (c >= '0' && c <= '9') return c - '0' + 52;
    if (c == '+' || c == '-') return 62;
    if (c == '/' || c == '_') return 63;
    return -1;
}

int sensor_b64_decode(const char *in, size_t n, unsigned char *out, size_t cap, size_t *out_len) {
    while (n && in[n - 1] == '=') n--;
    if (n % 4 == 1) return -1;
    size_t o = 0;
    uint32_t acc = 0;
    unsigned bits = 0;
    for (size_t i = 0; i < n; i++) {
        int v = b64v((unsigned char)in[i]);
        if (v < 0) return -1;
        acc = (acc << 6) | (uint32_t)v;
        bits += 6;
        if (bits >= 8) {
            bits -= 8;
            if (o >= cap) return -1;
            out[o++] = (unsigned char)(acc >> bits);
        }
    }
    *out_len = o;
    return 0;
}

static void b64_emit(bp_emit_t *e, const uint8_t *d, size_t n) {
    static const char t[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    char q[5] = { 0 };
    for (size_t i = 0; i < n; i += 3) {
        uint32_t v = (uint32_t)d[i] << 16 | (i + 1 < n ? (uint32_t)d[i + 1] << 8 : 0u) | (i + 2 < n ? d[i + 2] : 0u);
        q[0] = t[(v >> 18) & 63]; q[1] = t[(v >> 12) & 63];
        q[2] = i + 1 < n ? t[(v >> 6) & 63] : '=';
        q[3] = i + 2 < n ? t[v & 63] : '=';
        bp_emit_raw(e, q);
    }
}

/* ---- small parsers ---- */

static bool get_u32(const char *json, const char *key, uint32_t *out, bool *present) {
    char v[24];
    *present = bp_json_get(json, key, v, sizeof(v));
    if (!*present) return true;
    char *end = NULL;
    unsigned long x = strtoul(v, &end, 0);
    if (end == v || *end) return false;
    *out = (uint32_t)x;
    return true;
}

/* An LA channel: 3 or "LA3" / "la3". 0 = absent, -1 = bad. */
static int pin_value(const char *obj, const char *key) {
    char v[16];
    if (!bp_json_get(obj, key, v, sizeof(v))) return 0;
    const char *p = v;
    if ((p[0] == 'L' || p[0] == 'l') && (p[1] == 'A' || p[1] == 'a')) p += 2;
    char *end = NULL;
    long n = strtol(p, &end, 10);
    if (end == p || *end || n < 1 || n > 14) return -1;
    return (int)n;
}

/* {"key":number,...} in order: up to max pairs; key text copied into kbuf. */
static int parse_values(const char *obj, char kbuf[][36], double *vals, unsigned max, unsigned *n) {
    *n = 0;
    const char *p = obj;
    while (*p && *p != '{') p++;
    if (*p != '{') return -1;
    p++;
    for (;;) {
        while (*p == ' ' || *p == ',' || *p == '\t' || *p == '\n' || *p == '\r') p++;
        if (*p == '}') return 0;
        if (*p != '"') return -1;
        const char *k = ++p;
        while (*p && *p != '"') p++;
        if (*p != '"' || (size_t)(p - k) >= 36 || *n >= max) return -1;
        memcpy(kbuf[*n], k, (size_t)(p - k));
        kbuf[*n][p - k] = '\0';
        p++;
        while (*p == ' ') p++;
        if (*p++ != ':') return -1;
        char *end = NULL;
        double v = strtod(p, &end);
        if (end == p) return -1;
        vals[(*n)++] = v;
        p = end;
    }
}

static bool get_inst(const char *json, char *out, size_t cap, char *err, size_t ecap) {
    if (!bp_json_get(json, "instance", out, cap) || !out[0]) { snprintf(err, ecap, "missing instance"); return false; }
    return true;
}

static void begin_ok(bp_emit_t *e) { bp_emit_raw(e, "{\"status\":\"ok\",\"data\":{"); }
static void end_ok(bp_emit_t *e) { bp_emit_raw(e, "}}\n"); }

/* A chunked upload command: offset, total, data_b64. */
/* One buffer for the base64 text and, decoded in place (the output never overtakes the input),
   the chunk's bytes. */
static char s_chunk[CHUNK_B64_MAX + 4];

static int chunk_args(const char *json, uint32_t *off, uint32_t *total, const unsigned char **raw, size_t *n,
                      char *err, size_t cap) {
    bool p1, p2;
    if (!get_u32(json, "offset", off, &p1) || !p1) { snprintf(err, cap, "offset required (a byte offset)"); return -1; }
    if (!get_u32(json, "total", total, &p2) || !p2) { snprintf(err, cap, "total required (bytes of JSON)"); return -1; }
    char *b64 = s_chunk;
    int fit = bp_json_get_fit(json, "data_b64", b64, sizeof(s_chunk));
    if (fit == 0) { snprintf(err, cap, "data_b64 required"); return -1; }
    if (fit < 0) { snprintf(err, cap, "data_b64 too long: at most %u characters per chunk", CHUNK_B64_MAX); return -1; }
    if (sensor_b64_decode(b64, strlen(b64), (unsigned char *)s_chunk, sizeof(s_chunk), n) != 0) {
        snprintf(err, cap, "data_b64 is not base64");
        return -1;
    }
    *raw = (const unsigned char *)s_chunk;
    return 0;
}

/* ---- the commands ---- */

static int cmd_define(const char *json, bp_emit_t *e, char *err, size_t cap) {
    char inst[40];
    if (!get_inst(json, inst, sizeof(inst), err, cap)) return -1;
    uint32_t off = 0, total = 0;
    const unsigned char *raw = NULL;
    size_t n = 0;
    if (chunk_args(json, &off, &total, &raw, &n, err, cap)) return -1;
    begin_ok(e);
    int rc = sm_define_chunk(inst, off, total, raw, n, bp_json_flag(json, "dry_run"), e, err, cap);
    if (rc < 0) return -1;
    if (rc == 0) {
        bp_emit_raw(e, "\"instance\":");
        bp_emit_jstr(e, inst);
        bp_emit(e, ",\"received\":%lu", (unsigned long)sm_upload_received());
    }
    end_ok(e);
    return 0;
}

static int cmd_stimulus(const char *json, bp_emit_t *e, char *err, size_t cap) {
    uint32_t off = 0, total = 0;
    const unsigned char *raw = NULL;
    size_t n = 0;
    if (chunk_args(json, &off, &total, &raw, &n, err, cap)) return -1;
    begin_ok(e);
    int rc = sm_stim_chunk(off, total, raw, n, bp_json_flag(json, "dry_run"), e, err, cap);
    if (rc < 0) return -1;
    if (rc == 0) bp_emit(e, "\"received\":%lu", (unsigned long)sm_upload_received());
    end_ok(e);
    return 0;
}

static int cmd_start(const char *json, bp_emit_t *e, char *err, size_t cap) {
    char inst[40];
    if (!get_inst(json, inst, sizeof(inst), err, cap)) return -1;
    sm_start_req_t r;
    memset(&r, 0, sizeof(r));
    r.address = -1;
    char pins[256];
    if (bp_json_object(json, "pins", pins, sizeof(pins))) {
        static const char *const names[] = { "sda", "scl", "drdy", "sclk", "mosi", "miso", "cs", "tx", "rx" };
        uint8_t *dst[] = { &r.pins.sda, &r.pins.scl, &r.pins.drdy, &r.pins.sclk, &r.pins.mosi, &r.pins.miso,
                           &r.pins.cs, &r.pins.tx, &r.pins.rx };
        for (unsigned i = 0; i < 9; i++) {
            int v = pin_value(pins, names[i]);
            if (v < 0) { snprintf(err, cap, "pins.%s must be an LA channel 1..14", names[i]); return -1; }
            *dst[i] = (uint8_t)v;
        }
        char o[16];
        if (bp_json_get(pins, "output", o, sizeof(o))) {
            if (!strcmp(o, "dac")) r.pins.output = SM_OUT_DAC;
            else if (!strcmp(o, "current")) r.pins.output = SM_OUT_CURRENT;
            else { snprintf(err, cap, "pins.output must be \"dac\" or \"current\""); return -1; }
        }
    }
    char a[16];
    if (bp_json_get(json, "address", a, sizeof(a))) {
        char *end = NULL;
        long v = strtol(a, &end, 0);
        if (end == a || *end || v < 0 || v > 127) { snprintf(err, cap, "address must be a 7-bit address like \"0x76\""); return -1; }
        r.address = (int)v;
    }
    bool have;
    if (!get_u32(json, "baud", &r.baud, &have)) { snprintf(err, cap, "baud must be a number"); return -1; }
    static char kbuf[SD_MAX_PARAMS][36];
    char vals[640];
    if (bp_json_object(json, "values", vals, sizeof(vals))) {
        if (parse_values(vals, kbuf, r.values, SD_MAX_PARAMS, &r.n_values) != 0) {
            snprintf(err, cap, "values must be {\"param\":number,...} (at most %u)", SD_MAX_PARAMS);
            return -1;
        }
        for (unsigned i = 0; i < r.n_values; i++) r.keys[i] = kbuf[i];
    }
    begin_ok(e);
    if (sm_start(inst, &r, e, err, cap) != 0) return -1;
    end_ok(e);
    return 0;
}

static int cmd_set(const char *json, bp_emit_t *e, char *err, size_t cap) {
    char inst[40];
    if (!get_inst(json, inst, sizeof(inst), err, cap)) return -1;
    static char kbuf[SD_MAX_PARAMS][36];
    const char *keys[SD_MAX_PARAMS];
    double v[SD_MAX_PARAMS];
    unsigned n = 0;
    char vals[640];
    if (!bp_json_object(json, "values", vals, sizeof(vals)) || parse_values(vals, kbuf, v, SD_MAX_PARAMS, &n) != 0) {
        snprintf(err, cap, "values must be {\"param\":number,...} (at most %u)", SD_MAX_PARAMS);
        return -1;
    }
    for (unsigned i = 0; i < n; i++) keys[i] = kbuf[i];
    if (sm_set(inst, n, keys, v, err, cap) != 0) return -1;
    begin_ok(e);
    bp_emit_raw(e, "\"instance\":");
    bp_emit_jstr(e, inst);
    bp_emit_raw(e, ",\"values\":");
    bp_emit_raw(e, vals);
    end_ok(e);
    return 0;
}

static int cmd_fault(const char *json, bp_emit_t *e, char *err, size_t cap) {
    char inst[40];
    if (!get_inst(json, inst, sizeof(inst), err, cap)) return -1;
    /* the fault's own fields, as the format's fault object */
    char f[320], v[48];
    bp_emit_t fe;
    bp_emit_init(&fe, f, sizeof(f));
    bp_emit_raw(&fe, "{");
    static const char *const strs[] = { "type", "param", NULL };
    static const char *const nums[] = { "duration_s", "value", "at", "mask", "rate_hz", NULL };
    bool first = true;
    for (const char *const *k = strs; *k; k++)
        if (bp_json_get(json, *k, v, sizeof(v))) {
            bp_emit(&fe, "%s\"%s\":", first ? "" : ",", *k);
            bp_emit_jstr(&fe, v);
            first = false;
        }
    for (const char *const *k = nums; *k; k++)
        if (bp_json_get(json, *k, v, sizeof(v))) {
            char *end = NULL;
            double d = strtod(v, &end);   /* "0xF4" too: strtod reads hex */
            if (end == v || *end) { snprintf(err, cap, "%s must be a number", *k); return -1; }
            bp_emit(&fe, "%s\"%s\":%s", first ? "" : ",", *k, v[0] == '0' && (v[1] == 'x' || v[1] == 'X') ? "" : v);
            if (v[0] == '0' && (v[1] == 'x' || v[1] == 'X')) bp_emit(&fe, "%ld", (long)d);
            first = false;
        }
    bp_emit_raw(&fe, "}");
    if (!bp_emit_ok(&fe)) { snprintf(err, cap, "fault fields too long"); return -1; }
    sd_fault_t ft;
    sd_err_t se;
    if (sd_fault_from_json(f, strlen(f), &ft, &se) != SD_OK) {
        char m[160];
        sd_err_format(&se, m, sizeof(m));
        snprintf(err, cap, "invalid fault: %s", m);
        return -1;
    }
    if (sm_fault(inst, &ft, err, cap) != 0) return -1;
    begin_ok(e);
    bp_emit_raw(e, "\"instance\":");
    bp_emit_jstr(e, inst);
    bp_emit(e, ",\"fault\":\"%s\"", sd_fault_name(ft.type));
    end_ok(e);
    return 0;
}

int sensor_cmd(const char *verb, const char *json, bp_emit_t *e, char *err, size_t cap) {
    err[0] = '\0';
    if (!strcmp(verb, "sensor_define")) return cmd_define(json, e, err, cap);
    if (!strcmp(verb, "sensor_stimulus")) return cmd_stimulus(json, e, err, cap);
    if (!strcmp(verb, "sensor_start")) return cmd_start(json, e, err, cap);
    if (!strcmp(verb, "sensor_set")) return cmd_set(json, e, err, cap);
    if (!strcmp(verb, "sensor_fault")) return cmd_fault(json, e, err, cap);
    char inst[40] = { 0 };
    bool has_inst = bp_json_get(json, "instance", inst, sizeof(inst)) && inst[0];
    if (!strcmp(verb, "sensor_fault_clear")) {
        if (!has_inst) { snprintf(err, cap, "missing instance"); return -1; }
        if (sm_fault_clear(inst, err, cap) != 0) return -1;
        begin_ok(e);
        bp_emit_raw(e, "\"instance\":");
        bp_emit_jstr(e, inst);
        bp_emit_raw(e, ",\"faults\":[]");
        end_ok(e);
        return 0;
    }
    if (!strcmp(verb, "sensor_stop")) {
        bool all = bp_json_flag(json, "all");
        if (!has_inst && !all) { snprintf(err, cap, "give instance, or all:true"); return -1; }
        char names[120] = "";
        if (all) {
            /* the stopped names, for the reply */
            size_t n = 0;
            for (unsigned i = 0; i < SM_MAX_INST; i++) {
                const char *nm = sm_slot_name(i);
                if (!nm) continue;
                int w = snprintf(names + n, sizeof(names) - n, "%s\"%s\"", n ? "," : "", nm);
                if (w > 0 && (size_t)w < sizeof(names) - n) n += (size_t)w;
            }
        } else {
            snprintf(names, sizeof(names), "\"%s\"", inst);
        }
        if (sm_stop(all ? NULL : inst, err, cap) != 0) return -1;
        begin_ok(e);
        bp_emit(e, "\"stopped\":[%s]", names);
        end_ok(e);
        return 0;
    }
    if (!strcmp(verb, "sensor_stimulus_start")) {
        char v[24];
        double at = 0;
        if (bp_json_get(json, "at_s", v, sizeof(v))) at = strtod(v, NULL);
        if (sm_stim_start(at, err, cap) != 0) return -1;
        begin_ok(e);
        bp_emit_raw(e, "\"running\":true");
        end_ok(e);
        return 0;
    }
    if (!strcmp(verb, "sensor_stimulus_stop")) {
        sm_stim_stop();
        begin_ok(e);
        bp_emit_raw(e, "\"running\":false");
        end_ok(e);
        return 0;
    }
    if (!strcmp(verb, "sensor_status")) {
        uint32_t first = 0;
        bool have;
        if (!get_u32(json, "first", &first, &have)) { snprintf(err, cap, "first must be a number"); return -1; }
        begin_ok(e);
        if (sm_status(has_inst ? inst : NULL, first, e, err, cap) != 0) return -1;
        end_ok(e);
        return 0;
    }
    if (!strcmp(verb, "sensor_regs")) {
        if (!has_inst) { snprintf(err, cap, "missing instance"); return -1; }
        char v[16];
        unsigned start = 0, len = 256;
        if (bp_json_get(json, "start", v, sizeof(v))) start = (unsigned)strtoul(v, NULL, 0);
        if (bp_json_get(json, "len", v, sizeof(v))) len = (unsigned)strtoul(v, NULL, 0);
        uint8_t buf[256];
        if (sm_regs(inst, start, len, buf, err, cap) != 0) return -1;
        begin_ok(e);
        bp_emit_raw(e, "\"instance\":");
        bp_emit_jstr(e, inst);
        bp_emit(e, ",\"start\":\"0x%02X\",\"len\":%u,\"data_b64\":\"", start, len);
        b64_emit(e, buf, len);
        bp_emit_raw(e, "\"");
        end_ok(e);
        return 0;
    }
    snprintf(err, cap, "unknown cmd");
    return -1;
}
