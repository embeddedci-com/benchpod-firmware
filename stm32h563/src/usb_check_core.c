/* usb_check_core.c — see usb_check_core.h. Pure: host tests build it. */
#include "usb_check_core.h"
#include "bp_json.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum {
    PH_START = 0, PH_OFF, PH_OFF_WAIT, PH_ON_WAIT, PH_GAP, PH_RESET_WAIT, PH_SAMPLE, PH_SUSPEND,
    PH_SETTLE, PH_MEASURE, PH_RESUME, PH_CHECK, PH_RECOVER,
    PH_CLEAN, PH_REINIT_STOP, PH_REINIT_OFF, PH_REINIT_ON,   /* every exit of a test that touched the bus */
    PH_DONE
};

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

const char *uc_job_phase(const uc_job_t *j)
{
    static const char *const k[] = { "start", "power_off", "power_off", "power_on", "gap", "reset",
        "sample", "suspend", "settle", "measure", "resume", "check", "recover", "restore", "reinit", "reinit", "reinit", "done" };
    return j->phase <= PH_DONE ? k[j->phase] : "?";
}

/* Put the rail and the bus back, then close the result. Always returns 0 (finished). */
static int finish(uc_job_t *j, const char *error)
{
    uc_ops_t *o = &j->ops;
    if (error && !j->r.error[0]) j->r.error = error;
    if (j->efuse_was_on && !o->efuse_on(o->ctx)) (void)o->efuse_set(o->ctx, true);
    j->r.ms = o->now_ms(o->ctx) - j->t0;
    if (j->r.error[0]) j->r.pass = false;
    j->running = false;
    j->phase = PH_DONE;
    return 0;
}

/* The exit of a test that may have left the bus suspended (or never got its SOFs back): the error
   is kept, then PH_CLEAN puts the bus back (SUSPEN and L2RES cleared, the frame counter counting,
   all checked by the bus layer) and re-initializes the port when that fails. Without a suspended
   bus it is finish(). */
static int leave(uc_job_t *j, const char *error)
{
    if (error && !j->r.error[0]) j->r.error = error;
    if (!j->suspended) return finish(j, NULL);
    j->phase = PH_CLEAN;
    j->clean_stage = 0;
    j->tp = j->ops.now_ms(j->ops.ctx);
    return 1;
}

/* Re-initialize the port: host mode off, then on again (the role switch re-initializes the
   peripheral). From a dead port, or a bus that does not count frames. */
static int begin_reinit(uc_job_t *j, uint32_t now)
{
    uc_ops_t *o = &j->ops;
    j->suspended = false;
    if (!o->host_stop || !o->host_start) return finish(j, NULL);
    j->r.port_reinit = true;
    o->host_stop(o->ctx);
    j->tp = now;
    j->phase = PH_REINIT_OFF;
    return 1;
}

static int step_clean(uc_job_t *j, uint32_t now, const uc_host_t *h)
{
    uc_ops_t *o = &j->ops;
    uc_bus_info_t bi = {0};
    switch (j->phase) {
    case PH_CLEAN: {
        if (!h->hosting) { j->suspended = false; return finish(j, NULL); }   /* the role switch cleared it */
        int st = o->bus_state(o->ctx, &bi);
        if (j->clean_stage == 0) {                       /* wait for a running bus request, then restore */
            if (st == UC_BUS_BUSY) return now - j->tp < UC_BUS_WAIT_MS ? 1 : begin_reinit(j, now);
            if (o->bus_request(o->ctx, UC_BUS_RESUME) != 0) return begin_reinit(j, now);
            j->clean_stage = 1;
            j->tp = now;
            return 1;
        }
        if (st == UC_BUS_BUSY) return now - j->tp < UC_BUS_WAIT_MS ? 1 : begin_reinit(j, now);
        if (st == UC_BUS_OK && bi.sof_resumed) { j->suspended = false; return finish(j, NULL); }
        return begin_reinit(j, now);                     /* SOFs do not run again: the port is stuck */
    }
    case PH_REINIT_OFF:
        if (!h->hosting) {
            if (o->host_start(o->ctx) != 0) return finish(j, NULL);
            j->tp = now;
            j->phase = PH_REINIT_ON;
            return 1;
        }
        return now - j->tp < UC_BUS_WAIT_MS ? 1 : finish(j, NULL);
    case PH_REINIT_ON:
        if (h->hosting && h->attached) { j->r.reinit_ok = o->get_status(o->ctx) == 0; return finish(j, NULL); }
        return now - j->tp < UC_RECOVER_MS ? 1 : finish(j, NULL);
    default:
        return finish(j, "internal");
    }
}

static void sample_add(uc_job_t *j, int32_t ua)
{
    j->r.samples++;
    j->r.sum_ua += ua;
    if (ua < j->r.min_ua) j->r.min_ua = ua;
    if (ua > j->r.peak_ua) j->r.peak_ua = ua;
}

static int32_t avg_ua(const uc_result_t *r)
{
    return r->samples ? (int32_t)(r->sum_ua / (int64_t)r->samples) : 0;
}

/* Read the monitor once: 0 ok, 1 keep going after an error, -1 too many errors. */
static int sample_once(uc_job_t *j)
{
    int32_t ua = 0;
    if (j->ops.current_ua(j->ops.ctx, &ua) != 0) return ++j->read_errors > 20 ? -1 : 1;
    j->read_errors = 0;
    sample_add(j, ua);
    return 0;
}

static int step_enumerate(uc_job_t *j, uint32_t now, const uc_host_t *h)
{
    uc_ops_t *o = &j->ops;
    switch (j->phase) {
    case PH_START:
        j->r.cycles = j->a.count;
        j->phase = PH_OFF;
        return 1;
    case PH_OFF:
        if (o->efuse_set(o->ctx, false) != 0) return finish(j, "efuse_error");
        j->tp = now;
        j->phase = PH_OFF_WAIT;
        return 1;
    case PH_OFF_WAIT: {
        uint32_t el = now - j->tp;
        if (el < j->a.off_ms) return 1;
        if (h->connected || h->attached) {
            if (el < j->a.off_ms + UC_DROP_WAIT_MS) return 1;
            return finish(j, "not_powered_by_efuse1");
        }
        j->enum0 = h->enumerations;
        j->desc0 = h->descriptors;
        j->got_conn = j->got_desc = false;
        if (o->efuse_set(o->ctx, true) != 0) return finish(j, "efuse_error");
        j->tp = now;
        j->phase = PH_ON_WAIT;
        return 1;
    }
    default: {   /* PH_ON_WAIT */
        uint32_t el = now - j->tp;
        if (!j->got_conn && (h->connected || h->descriptors != j->desc0)) { j->got_conn = true; j->t_conn = el; }
        if (!j->got_desc && h->descriptors != j->desc0) { j->got_desc = true; j->t_desc = el; }
        bool end = true;
        if (h->attached && h->enumerations != j->enum0) {
            if (!j->got_desc) j->t_desc = el;
            uc_stat_add(&j->r.connect, j->t_conn);
            uc_stat_add(&j->r.descriptor, j->t_desc);
            uc_stat_add(&j->r.mount, el);
            j->r.ok++;
            j->r.vid = h->vid;
            j->r.pid = h->pid;
        } else if (j->got_desc && h->enumeration == UC_ENUM_STUCK) {
            j->r.stuck++;
        } else if (el >= j->a.timeout_ms) {
            if (j->got_conn) j->r.timeout++; else j->r.no_connect++;
        } else {
            end = false;
        }
        if (!end) return 1;
        j->done++;
        if (++j->i >= j->a.count) {
            j->r.pass = j->r.ok == j->a.count;
            return finish(j, NULL);
        }
        j->phase = PH_OFF;
        return 1;
    }
    }
}

static int step_power(uc_job_t *j, uint32_t now, const uc_host_t *h)
{
    if (j->phase == PH_START) {
        if (!h->attached || !h->cfg_valid) return finish(j, "no_device");
        j->r.limit_ma = h->max_power_ma;
        j->r.vid = h->vid;
        j->r.pid = h->pid;
        j->tp = now;
        j->phase = PH_SAMPLE;
        return 1;
    }
    int s = sample_once(j);
    if (s < 0) return finish(j, "monitor_error");
    uint32_t el = now - j->tp;
    j->done = el < j->total ? el : j->total;
    if (el < j->a.duration_ms) return 1;
    if (j->r.samples == 0) return finish(j, "monitor_error");
    j->r.verdict = (uint8_t)uc_power_verdict(avg_ua(&j->r), j->r.peak_ua, j->r.limit_ma);
    j->r.pass = j->r.verdict == UC_V_OK;
    return finish(j, NULL);
}

static int bus_wait(uc_job_t *j, uint32_t now, uc_bus_info_t *bi)
{
    int st = j->ops.bus_state(j->ops.ctx, bi);
    if (st == UC_BUS_BUSY && now - j->tp < UC_BUS_WAIT_MS) return UC_BUS_BUSY;
    if (st == UC_BUS_BUSY) return -1;
    return st;
}

static int start_resume(uc_job_t *j, uint32_t now)
{
    if (j->ops.bus_request(j->ops.ctx, UC_BUS_RESUME) != 0) return leave(j, "bus_busy");
    j->tp = now;
    j->phase = PH_RESUME;
    return 1;
}

static int step_suspend(uc_job_t *j, uint32_t now, const uc_host_t *h)
{
    uc_ops_t *o = &j->ops;
    uc_bus_info_t bi = {0};
    switch (j->phase) {
    case PH_START:
        if (!h->attached || !h->cfg_valid) return finish(j, "no_device");
        j->r.self_powered = (h->cfg_attrs & 0x40u) != 0;
        j->r.vid = h->vid;
        j->r.pid = h->pid;
        j->r.verdict = UC_V_NOT_MEASURED;
        if (o->bus_request(o->ctx, UC_BUS_SUSPEND) != 0) return finish(j, "bus_busy");
        j->suspended = true;   /* from now on every exit restores the bus */
        j->tp = now;
        j->phase = PH_SUSPEND;
        return 1;
    case PH_SUSPEND: {
        int st = bus_wait(j, now, &bi);
        if (st == UC_BUS_BUSY) return 1;
        if (st < 0) return leave(j, "bus_timeout");
        if (st == UC_BUS_FAIL) { j->phase = PH_CHECK; j->tp = now; return 1; }
        j->r.susprdy = bi.susprdy;
        j->r.sof_stopped = bi.sof_stopped;
        j->r.suspend_ok = bi.sof_stopped;
        if (!bi.sof_stopped) return start_resume(j, now);
        j->tp = now;
        j->phase = PH_SETTLE;
        return 1;
    }
    case PH_SETTLE:
        if (now - j->tp < j->a.settle_ms) return 1;
        j->tp = now;
        j->phase = PH_MEASURE;
        return 1;
    case PH_MEASURE: {
        int s = sample_once(j);
        if (s < 0) return leave(j, "monitor_error");
        uint32_t el = now - j->tp;
        j->done = el < j->total ? el : j->total;
        if (el < j->a.duration_ms) return 1;
        if (j->r.samples == 0) return leave(j, "monitor_error");
        j->r.verdict = (uint8_t)uc_suspend_verdict(avg_ua(&j->r), j->r.self_powered);
        return start_resume(j, now);
    }
    case PH_RESUME: {
        int st = bus_wait(j, now, &bi);
        if (st == UC_BUS_BUSY) return 1;
        if (st < 0) return leave(j, "bus_timeout");
        j->suspended = !bi.sof_resumed;   /* SOFs not counting: PH_CLEAN tries again, then re-initializes the port */
        j->r.sof_resumed = bi.sof_resumed;
        j->r.resume_ok = st == UC_BUS_OK;
        j->tp = now;
        j->phase = PH_CHECK;
        return 1;
    }
    case PH_CHECK:
        j->r.answers = o->get_status(o->ctx) == 0;
        if (j->r.answers) goto done;
        /* No answer after the resume: a bus reset re-enumerates the device. */
        j->enum0 = h->enumerations;
        if (o->bus_request(o->ctx, UC_BUS_RESET) != 0) return begin_reinit(j, now);
        j->tp = now;
        j->phase = PH_RECOVER;
        return 1;
    case PH_RECOVER:
        if (h->attached && h->enumerations != j->enum0) {
            j->r.recovered = o->get_status(o->ctx) == 0;
            if (j->r.recovered) goto done;
            return begin_reinit(j, now);
        }
        if (now - j->tp < UC_RECOVER_MS) return 1;
        return begin_reinit(j, now);        /* not even a bus reset brought it back */
    default:
        return finish(j, "internal");
    }
done:
    j->r.pass = j->r.suspend_ok && j->r.resume_ok && j->r.answers &&
                (j->r.verdict == UC_V_OK || j->r.verdict == UC_V_SKIPPED);
    return leave(j, NULL);
}

static int step_reset(uc_job_t *j, uint32_t now, const uc_host_t *h)
{
    uc_ops_t *o = &j->ops;
    switch (j->phase) {
    case PH_START:
        if (!h->attached) return finish(j, "no_device");
        j->r.cycles = j->a.count;
        j->vid0 = h->vid;
        j->pid0 = h->pid;
        j->r.vid = h->vid;
        j->r.pid = h->pid;
        j->phase = PH_OFF;
        return 1;
    case PH_OFF:
        j->enum0 = h->enumerations;
        j->desc0 = h->descriptors;
        if (o->bus_request(o->ctx, UC_BUS_RESET) != 0) return finish(j, "bus_busy");
        j->tp = now;
        j->phase = PH_RESET_WAIT;
        return 1;
    default: {
        uint32_t el = now - j->tp;
        bool end = true;
        if (h->attached && h->enumerations != j->enum0) {
            uc_stat_add(&j->r.mount, el);
            j->r.ok++;
            if (h->vid != j->vid0 || h->pid != j->pid0) j->r.identity_changed++;
        } else if (h->descriptors != j->desc0 && h->enumeration == UC_ENUM_STUCK) {
            j->r.stuck++;
        } else if (el >= j->a.timeout_ms) {
            j->r.timeout++;
        } else {
            end = false;
        }
        if (!end) return 1;
        j->done++;
        if (++j->i >= j->a.count) {
            j->r.pass = j->r.ok == j->a.count && j->r.identity_changed == 0;
            return finish(j, NULL);
        }
        j->phase = PH_OFF;
        return 1;
    }
    }
}

static int replug_next(uc_job_t *j)
{
    j->done++;
    if (++j->i >= j->a.count) {
        j->i = 0;
        if (++j->k >= j->a.n_drops) {
            j->r.pass = true;
            for (uint8_t n = 0; n < j->r.nlen; n++)
                if (j->r.len[n].failed) j->r.pass = false;
            return finish(j, NULL);
        }
        j->r.len[j->k].off_ms = j->a.drops[j->k];
    }
    j->phase = PH_GAP;
    return 1;
}

static int step_replug(uc_job_t *j, uint32_t now, const uc_host_t *h)
{
    uc_ops_t *o = &j->ops;
    switch (j->phase) {
    case PH_START:
        if (!h->attached) return finish(j, "no_device");
        j->r.nlen = j->a.n_drops;
        j->r.len[0].off_ms = j->a.drops[0];
        j->r.vid = h->vid;
        j->r.pid = h->pid;
        j->phase = PH_OFF;
        return 1;
    case PH_GAP:
        if (now - j->tp < UC_GAP_MS) return 1;
        j->phase = PH_OFF;
        return 1;
    case PH_OFF:
        j->enum0 = h->enumerations;
        j->dropped = false;
        if (o->efuse_set(o->ctx, false) != 0) return finish(j, "efuse_error");
        j->tp = now;
        j->phase = PH_OFF_WAIT;
        return 1;
    case PH_OFF_WAIT:
        if (!h->connected || !h->attached) j->dropped = true;
        if (now - j->tp < j->a.drops[j->k]) return 1;
        if (o->efuse_set(o->ctx, true) != 0) return finish(j, "efuse_error");
        j->tp = now;
        j->phase = PH_ON_WAIT;
        return 1;
    default: {   /* PH_ON_WAIT */
        uint32_t el = now - j->tp;
        if (!h->connected || !h->attached) j->dropped = true;
        if (h->attached && h->enumerations != j->enum0) {
            j->r.len[j->k].reenumerated++;
        } else if (h->enumeration == UC_ENUM_STUCK) {
            j->r.len[j->k].failed++;
        } else if (!j->dropped && el >= UC_STAY_MS) {
            /* Never left the bus as far as the host saw: make sure it still answers. */
            if (o->get_status(o->ctx) == 0) j->r.len[j->k].stayed++;
            else j->r.len[j->k].failed++;
        } else if (el >= j->a.timeout_ms) {
            j->r.len[j->k].failed++;
        } else {
            return 1;
        }
        j->tp = now;
        return replug_next(j);
    }
    }
}

int uc_job_step(uc_job_t *j)
{
    if (!j->running) return 0;
    uc_ops_t *o = &j->ops;
    uint32_t now = o->now_ms(o->ctx);
    uc_host_t h;
    o->host(o->ctx, &h);
    if (j->phase >= PH_CLEAN && j->phase < PH_DONE) return step_clean(j, now, &h);   /* not abortable */
    if (!h.hosting) { j->suspended = false; return finish(j, "host_stopped"); }   /* the role switch reset the bus */
    if (j->abort_asked) return leave(j, "aborted");
    switch (j->a.test) {
    case UC_ENUMERATE: return step_enumerate(j, now, &h);
    case UC_POWER:     return step_power(j, now, &h);
    case UC_SUSPEND:   return step_suspend(j, now, &h);
    case UC_RESET:     return step_reset(j, now, &h);
    default:           return step_replug(j, now, &h);
    }
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
