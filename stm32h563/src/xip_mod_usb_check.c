/* xip_mod_usb_check.c — the usb_check job's steps (usb_check_core.h), the part that runs. Pure like
   the rest of the core, and it runs from the W25Q (xip_image.h, the usb image): the job's state
   is in the uc_job_t its caller owns, nothing is static. usb_check.c opens a session at the start of
   a check, calls uc_job_step through the image's table once per hw worker loop, and closes the
   session when the job finished. The parser, the statistics, the verdicts, the reply and the
   names stay in usb_check_core.c (internal: the command handler calls them outside any session). */
#include "usb_check_core.h"

#include <stdint.h>
#include <string.h>

/* Put the rail and the bus back, then close the result. Always returns 0 (finished). */
static int finish(uc_job_t *j, uc_err_t error)
{
    uc_ops_t *o = &j->ops;
    if (error != UC_E_NONE && !j->r.error[0]) j->r.error = uc_error_str(error);
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
static int leave(uc_job_t *j, uc_err_t error)
{
    if (error != UC_E_NONE && !j->r.error[0]) j->r.error = uc_error_str(error);
    if (!j->suspended) return finish(j, UC_E_NONE);
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
    if (!o->host_stop || !o->host_start) return finish(j, UC_E_NONE);
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
        if (!h->hosting) { j->suspended = false; return finish(j, UC_E_NONE); }   /* the role switch cleared it */
        int st = o->bus_state(o->ctx, &bi);
        if (j->clean_stage == 0) {                       /* wait for a running bus request, then restore */
            if (st == UC_BUS_BUSY) return now - j->tp < UC_BUS_WAIT_MS ? 1 : begin_reinit(j, now);
            if (o->bus_request(o->ctx, UC_BUS_RESUME) != 0) return begin_reinit(j, now);
            j->clean_stage = 1;
            j->tp = now;
            return 1;
        }
        if (st == UC_BUS_BUSY) return now - j->tp < UC_BUS_WAIT_MS ? 1 : begin_reinit(j, now);
        if (st == UC_BUS_OK && bi.sof_resumed) { j->suspended = false; return finish(j, UC_E_NONE); }
        return begin_reinit(j, now);                     /* SOFs do not run again: the port is stuck */
    }
    case PH_REINIT_OFF:
        if (!h->hosting) {
            if (o->host_start(o->ctx) != 0) return finish(j, UC_E_NONE);
            j->tp = now;
            j->phase = PH_REINIT_ON;
            return 1;
        }
        return now - j->tp < UC_BUS_WAIT_MS ? 1 : finish(j, UC_E_NONE);
    case PH_REINIT_ON:
        if (h->hosting && h->attached) { j->r.reinit_ok = o->get_status(o->ctx) == 0; return finish(j, UC_E_NONE); }
        return now - j->tp < UC_RECOVER_MS ? 1 : finish(j, UC_E_NONE);
    default:
        return finish(j, UC_E_INTERNAL);
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
        if (o->efuse_set(o->ctx, false) != 0) return finish(j, UC_E_EFUSE);
        j->tp = now;
        j->phase = PH_OFF_WAIT;
        return 1;
    case PH_OFF_WAIT: {
        uint32_t el = now - j->tp;
        if (el < j->a.off_ms) return 1;
        if (h->connected || h->attached) {
            if (el < j->a.off_ms + UC_DROP_WAIT_MS) return 1;
            return finish(j, UC_E_NOT_POWERED);
        }
        j->enum0 = h->enumerations;
        j->desc0 = h->descriptors;
        j->got_conn = j->got_desc = false;
        if (o->efuse_set(o->ctx, true) != 0) return finish(j, UC_E_EFUSE);
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
            return finish(j, UC_E_NONE);
        }
        j->phase = PH_OFF;
        return 1;
    }
    }
}

static int step_power(uc_job_t *j, uint32_t now, const uc_host_t *h)
{
    if (j->phase == PH_START) {
        if (!h->attached || !h->cfg_valid) return finish(j, UC_E_NO_DEVICE);
        j->r.limit_ma = h->max_power_ma;
        j->r.vid = h->vid;
        j->r.pid = h->pid;
        j->tp = now;
        j->phase = PH_SAMPLE;
        return 1;
    }
    int s = sample_once(j);
    if (s < 0) return finish(j, UC_E_MONITOR);
    uint32_t el = now - j->tp;
    j->done = el < j->total ? el : j->total;
    if (el < j->a.duration_ms) return 1;
    if (j->r.samples == 0) return finish(j, UC_E_MONITOR);
    j->r.verdict = (uint8_t)uc_power_verdict(avg_ua(&j->r), j->r.peak_ua, j->r.limit_ma);
    j->r.pass = j->r.verdict == UC_V_OK;
    return finish(j, UC_E_NONE);
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
    if (j->ops.bus_request(j->ops.ctx, UC_BUS_RESUME) != 0) return leave(j, UC_E_BUS_BUSY);
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
        if (!h->attached || !h->cfg_valid) return finish(j, UC_E_NO_DEVICE);
        j->r.self_powered = (h->cfg_attrs & 0x40u) != 0;
        j->r.vid = h->vid;
        j->r.pid = h->pid;
        j->r.verdict = UC_V_NOT_MEASURED;
        if (o->bus_request(o->ctx, UC_BUS_SUSPEND) != 0) return finish(j, UC_E_BUS_BUSY);
        j->suspended = true;   /* from now on every exit restores the bus */
        j->tp = now;
        j->phase = PH_SUSPEND;
        return 1;
    case PH_SUSPEND: {
        int st = bus_wait(j, now, &bi);
        if (st == UC_BUS_BUSY) return 1;
        if (st < 0) return leave(j, UC_E_BUS_TIMEOUT);
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
        if (s < 0) return leave(j, UC_E_MONITOR);
        uint32_t el = now - j->tp;
        j->done = el < j->total ? el : j->total;
        if (el < j->a.duration_ms) return 1;
        if (j->r.samples == 0) return leave(j, UC_E_MONITOR);
        j->r.verdict = (uint8_t)uc_suspend_verdict(avg_ua(&j->r), j->r.self_powered);
        return start_resume(j, now);
    }
    case PH_RESUME: {
        int st = bus_wait(j, now, &bi);
        if (st == UC_BUS_BUSY) return 1;
        if (st < 0) return leave(j, UC_E_BUS_TIMEOUT);
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
        return finish(j, UC_E_INTERNAL);
    }
done:
    j->r.pass = j->r.suspend_ok && j->r.resume_ok && j->r.answers &&
                (j->r.verdict == UC_V_OK || j->r.verdict == UC_V_SKIPPED);
    return leave(j, UC_E_NONE);
}

static int step_reset(uc_job_t *j, uint32_t now, const uc_host_t *h)
{
    uc_ops_t *o = &j->ops;
    switch (j->phase) {
    case PH_START:
        if (!h->attached) return finish(j, UC_E_NO_DEVICE);
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
        if (o->bus_request(o->ctx, UC_BUS_RESET) != 0) return finish(j, UC_E_BUS_BUSY);
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
            return finish(j, UC_E_NONE);
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
            return finish(j, UC_E_NONE);
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
        if (!h->attached) return finish(j, UC_E_NO_DEVICE);
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
        if (o->efuse_set(o->ctx, false) != 0) return finish(j, UC_E_EFUSE);
        j->tp = now;
        j->phase = PH_OFF_WAIT;
        return 1;
    case PH_OFF_WAIT:
        if (!h->connected || !h->attached) j->dropped = true;
        if (now - j->tp < j->a.drops[j->k]) return 1;
        if (o->efuse_set(o->ctx, true) != 0) return finish(j, UC_E_EFUSE);
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
    if (!h.hosting) { j->suspended = false; return finish(j, UC_E_HOST_STOPPED); }   /* the role switch reset the bus */
    if (j->abort_asked) return leave(j, UC_E_ABORTED);
    switch (j->a.test) {
    case UC_ENUMERATE: return step_enumerate(j, now, &h);
    case UC_POWER:     return step_power(j, now, &h);
    case UC_SUSPEND:   return step_suspend(j, now, &h);
    case UC_RESET:     return step_reset(j, now, &h);
    default:           return step_replug(j, now, &h);
    }
}
