/* usb_check_core.c — see usb_check_core.h. Pure: host tests build it. */
#include "usb_check_core.h"
#include "bp_json.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *const k_tests[UC_TESTS] = { "enumerate", "power", "suspend", "reset", "replug" };

const char *uc_test_name(uc_test_t t) { return (unsigned)t < UC_TESTS ? k_tests[t] : "?"; }

int uc_test_from_name(const char *name)
{
    for (int t = 0; name && t < UC_TESTS; t++)
        if (!strcmp(name, k_tests[t])) return t;
    return -1;
}

/* ---- arguments ---------------------------------------------------------------------------- */

static int get_range(const char *json, const char *key, uint32_t dflt, uint32_t lo, uint32_t hi,
                     uint32_t *out, char *why, size_t cap)
{
    char s[16] = {0};
    *out = dflt;
    if (!bp_json_get(json, key, s, sizeof(s)) || !strcmp(s, "null")) return 0;
    char *end = NULL;
    unsigned long v = strtoul(s, &end, 10);
    if (end == s || *end || s[0] == '-' || v < lo || v > hi) {
        snprintf(why, cap, "%s must be a whole number %lu..%lu", key, (unsigned long)lo, (unsigned long)hi);
        return -1;
    }
    *out = (uint32_t)v;
    return 0;
}

/* "off_ms":[5,20,100] or "off_ms":50 -> out[]; n = entries (0 when the key is absent). */
static int get_list(const char *json, const char *key, uint32_t *out, uint8_t cap, uint8_t *n,
                    uint32_t lo, uint32_t hi, char *why, size_t wcap)
{
    char pat[24];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    *n = 0;
    const char *p = strstr(json, pat);
    if (!p) return 0;
    p += strlen(pat);
    while (*p == ' ' || *p == ':') p++;
    bool arr = *p == '[';
    if (arr) p++;
    for (;;) {
        while (*p == ' ' || *p == ',') p++;
        if (*p == ']' || *p == '\0' || *p == '}') break;
        char *end = NULL;
        unsigned long v = strtoul(p, &end, 10);
        if (end == p || *p == '-' || v < lo || v > hi || *n >= cap) {
            snprintf(why, wcap, "%s must be up to %u whole numbers %lu..%lu", key, (unsigned)cap,
                     (unsigned long)lo, (unsigned long)hi);
            return -1;
        }
        out[(*n)++] = (uint32_t)v;
        p = end;
        if (!arr) break;
    }
    if (*n == 0) { snprintf(why, wcap, "%s is empty", key); return -1; }
    return 0;
}

int uc_parse_args(const char *json, uc_args_t *a, char *why, size_t cap)
{
    memset(a, 0, sizeof(*a));
    char name[12] = {0};
    int t = bp_json_get(json, "test", name, sizeof(name)) ? uc_test_from_name(name) : -1;
    if (t < 0) { snprintf(why, cap, "test must be enumerate, power, suspend, reset or replug"); return -1; }
    a->test = (uc_test_t)t;
    uint32_t cmax = UC_COUNT_MAX;
    uint32_t cdef = t == UC_RESET ? UC_RESET_COUNT_DEFAULT : UC_ENUM_COUNT_DEFAULT;
    if (t == UC_REPLUG) { cdef = UC_REPLUG_COUNT_DEFAULT; cmax = UC_REPLUG_COUNT_MAX; }
    if (get_range(json, "count", cdef, 1, cmax, &a->count, why, cap)) return -1;
    if (get_range(json, "timeout_ms", UC_TIMEOUT_DEFAULT_MS, UC_TIMEOUT_MIN_MS, UC_TIMEOUT_MAX_MS,
                  &a->timeout_ms, why, cap)) return -1;
    if (t == UC_ENUMERATE &&
        get_range(json, "off_ms", UC_OFF_MS_DEFAULT, UC_OFF_MS_MIN, UC_OFF_MS_MAX, &a->off_ms, why, cap)) return -1;
    if (t == UC_POWER || t == UC_SUSPEND) {
        if (get_range(json, "duration_ms", t == UC_POWER ? UC_POWER_MS_DEFAULT : UC_SUSPEND_MS_DEFAULT,
                      UC_DURATION_MIN_MS, UC_DURATION_MAX_MS, &a->duration_ms, why, cap)) return -1;
    }
    if (t == UC_SUSPEND &&
        get_range(json, "settle_ms", UC_SETTLE_DEFAULT_MS, UC_SETTLE_MIN_MS, UC_SETTLE_MAX_MS,
                  &a->settle_ms, why, cap)) return -1;
    if (t == UC_REPLUG) {
        if (get_list(json, "off_ms", a->drops, UC_REPLUG_LENGTHS_MAX, &a->n_drops, UC_DROP_MS_MIN,
                     UC_DROP_MS_MAX, why, cap)) return -1;
        if (a->n_drops == 0) {
            static const uint32_t k_def[4] = { 5, 20, 100, 500 };
            memcpy(a->drops, k_def, sizeof(k_def));
            a->n_drops = 4;
        }
    }
    return 0;
}

/* ---- statistics and decisions -------------------------------------------------------------- */

void uc_stat_add(uc_stat_t *s, uint32_t v)
{
    if (s->n == 0 || v < s->min) s->min = v;
    if (s->n == 0 || v > s->max) s->max = v;
    s->sum += v;
    s->n++;
}

uint32_t uc_stat_avg(const uc_stat_t *s)
{
    return s->n ? (uint32_t)((s->sum + s->n / 2u) / s->n) : 0u;
}

const char *uc_verdict_name(uc_verdict_t v)
{
    switch (v) {
    case UC_V_OK:           return "ok";
    case UC_V_AVG_OVER:     return "avg_over";
    case UC_V_PEAK_OVER:    return "peak_over";
    case UC_V_NO_LIMIT:     return "no_limit";
    case UC_V_SKIPPED:      return "skipped";
    default:                return "not_measured";
    }
}

uc_verdict_t uc_power_verdict(int32_t avg_ua, int32_t peak_ua, uint16_t max_power_ma)
{
    if (max_power_ma == 0) return UC_V_NO_LIMIT;
    int64_t lim = (int64_t)max_power_ma * 1000;
    if (avg_ua > lim) return UC_V_AVG_OVER;
    if ((int64_t)peak_ua * UC_PEAK_ALLOW_DEN > lim * UC_PEAK_ALLOW_NUM) return UC_V_PEAK_OVER;
    return UC_V_OK;
}

uc_verdict_t uc_suspend_verdict(int32_t avg_ua, bool self_powered)
{
    if (self_powered) return UC_V_SKIPPED;
    return avg_ua > UC_SUSPEND_LIMIT_UA ? UC_V_AVG_OVER : UC_V_OK;
}

/* ---- the job ------------------------------------------------------------------------------- */

void uc_job_init(uc_job_t *j, const uc_args_t *a, const uc_ops_t *ops)
{
    memset(j, 0, sizeof(*j));
    j->a = *a;
    j->ops = *ops;
    j->r.error = "";
    j->efuse_was_on = ops->efuse_on(ops->ctx);
    j->running = true;
    j->phase = PH_START;
    j->t0 = ops->now_ms(ops->ctx);
    j->r.min_ua = INT32_MAX;
    j->r.peak_ua = INT32_MIN;
    switch (a->test) {
    case UC_REPLUG: j->total = (uint32_t)a->n_drops * a->count; break;
    case UC_POWER: case UC_SUSPEND: j->total = a->duration_ms; break;
    default: j->total = a->count; break;
    }
}

void uc_job_abort(uc_job_t *j) { j->abort_asked = true; }

const char *uc_error_str(uc_err_t e)
{
    static const char *const k[] = { "", "internal", "efuse_error", "no_device", "monitor_error", "bus_busy",
                                     "bus_timeout", "host_stopped", "aborted", "not_powered_by_efuse1" };
    return (unsigned)e < sizeof(k) / sizeof(k[0]) ? k[e] : "internal";
}

const char *uc_job_phase(const uc_job_t *j)
{
    static const char *const k[] = { "start", "power_off", "power_off", "power_on", "gap", "reset",
        "sample", "suspend", "settle", "measure", "resume", "check", "recover", "restore", "reinit", "reinit", "reinit", "done" };
    return j->phase <= PH_DONE ? k[j->phase] : "?";
}

/* ---- the reply ----------------------------------------------------------------------------- */

static void emit_stat(bp_emit_t *e, const char *key, const uc_stat_t *s)
{
    if (!s->n) { bp_emit(e, ",\"%s\":null", key); return; }
    bp_emit(e, ",\"%s\":{\"min\":%lu,\"avg\":%lu,\"max\":%lu}", key, (unsigned long)s->min,
            (unsigned long)uc_stat_avg(s), (unsigned long)s->max);
}

static void emit_current(bp_emit_t *e, const uc_result_t *r)
{
    bool any = r->samples > 0;
    int32_t avg = any ? (int32_t)(r->sum_ua / (int64_t)r->samples) : 0;
    bp_emit(e, ",\"samples\":%lu,\"avg_ua\":%ld,\"min_ua\":%ld,\"peak_ua\":%ld", (unsigned long)r->samples,
            (long)avg, (long)(any ? r->min_ua : 0), (long)(any ? r->peak_ua : 0));
}

size_t uc_reply_json(const uc_job_t *j, char *out, size_t cap)
{
    bp_emit_t e;
    bp_emit_init(&e, out, cap);
    const uc_result_t *r = &j->r;
    uint32_t ms = j->running ? j->ops.now_ms(j->ops.ctx) - j->t0 : r->ms;
    bp_emit(&e, "{\"running\":%s,\"have_result\":%s,\"test\":\"%s\",\"phase\":\"%s\",\"done\":%lu,"
                "\"total\":%lu,\"ms\":%lu", j->running ? "true" : "false", j->running ? "false" : "true",
            uc_test_name(j->a.test), uc_job_phase(j), (unsigned long)j->done, (unsigned long)j->total,
            (unsigned long)ms);
    if (!j->running) {
        bp_emit(&e, ",\"pass\":%s,\"error\":\"%s\",\"result\":{\"vid\":%u,\"pid\":%u", r->pass ? "true" : "false",
                r->error, (unsigned)r->vid, (unsigned)r->pid);
        switch (j->a.test) {
        case UC_ENUMERATE:
            bp_emit(&e, ",\"cycles\":%lu,\"ok\":%lu,\"no_connect\":%lu,\"stuck\":%lu,\"timeout\":%lu,\"off_ms\":%lu",
                    (unsigned long)j->done, (unsigned long)r->ok, (unsigned long)r->no_connect,
                    (unsigned long)r->stuck, (unsigned long)r->timeout, (unsigned long)j->a.off_ms);
            emit_stat(&e, "connect_ms", &r->connect);
            emit_stat(&e, "descriptor_ms", &r->descriptor);
            emit_stat(&e, "mount_ms", &r->mount);
            break;
        case UC_POWER:
            emit_current(&e, r);
            bp_emit(&e, ",\"limit_ma\":%u,\"verdict\":\"%s\"", (unsigned)r->limit_ma, uc_verdict_name((uc_verdict_t)r->verdict));
            break;
        case UC_SUSPEND:
            emit_current(&e, r);
            bp_emit(&e, ",\"self_powered\":%s,\"limit_ua\":%d,\"current\":\"%s\",\"suspend_ok\":%s,\"susprdy\":%s,"
                        "\"sof_stopped\":%s,\"resume\":\"%s\",\"sof_resumed\":%s,\"answers\":%s,\"recovered_by_reset\":%s,\"port_reinit\":%s,\"reinit_ok\":%s",
                    r->self_powered ? "true" : "false", UC_SUSPEND_LIMIT_UA,
                    uc_verdict_name((uc_verdict_t)r->verdict), r->suspend_ok ? "true" : "false",
                    r->susprdy ? "true" : "false", r->sof_stopped ? "true" : "false",
                    r->resume_ok ? "signaling" : "failed", r->sof_resumed ? "true" : "false",
                    r->answers ? "true" : "false", r->recovered ? "true" : "false",
                    r->port_reinit ? "true" : "false", r->reinit_ok ? "true" : "false");
            break;
        case UC_RESET:
            bp_emit(&e, ",\"cycles\":%lu,\"ok\":%lu,\"stuck\":%lu,\"timeout\":%lu,\"identity_changed\":%lu",
                    (unsigned long)j->done, (unsigned long)r->ok, (unsigned long)r->stuck,
                    (unsigned long)r->timeout, (unsigned long)r->identity_changed);
            emit_stat(&e, "reenumerate_ms", &r->mount);
            break;
        default:
            bp_emit(&e, ",\"count\":%lu,\"lengths\":[", (unsigned long)j->a.count);
            for (uint8_t n = 0; n < r->nlen; n++)
                bp_emit(&e, "%s{\"off_ms\":%lu,\"stayed\":%lu,\"reenumerated\":%lu,\"failed\":%lu}", n ? "," : "",
                        (unsigned long)r->len[n].off_ms, (unsigned long)r->len[n].stayed,
                        (unsigned long)r->len[n].reenumerated, (unsigned long)r->len[n].failed);
            bp_emit(&e, "]");
            break;
        }
        bp_emit(&e, "}");
    }
    bp_emit(&e, "}");
    return e.ok ? e.len : 0;
}
