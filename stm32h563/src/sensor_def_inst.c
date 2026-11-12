/* ============================================================================
 * sensor_def_inst.c — an instance of a compiled sensor definition: its params,
 * time, states, lets and value faults.
 * ========================================================================== */
#include "sensor_def_int.h"

#include <math.h>

static const sdp_param_t *params(const sd_inst_t *in) {
    return (const sdp_param_t *)(const void *)(in->prog + sd_h(in)->params);
}

static void apply_params(sd_inst_t *in) {
    const sdp_hdr_t *h = sd_h(in);
    for (unsigned i = 0; i < h->n_params; i++) {
        double v = (in->stuck & (1u << i)) ? in->stuck_val[i] : in->raw[i] * in->gain[i] + in->offset[i];
        in->vars[SDV_PARAM + i] = sdv_num(v);
    }
}

static bool in_prog(uint32_t len, uint32_t off, uint32_t n) { return off <= len && n <= len - off; }

int sd_inst_init(sd_inst_t *in, const void *prog, size_t prog_len, sd_err_t *err) {
    const uint8_t *p = (const uint8_t *)prog;
    if (err) { memset(err, 0, sizeof(*err)); err->expr_off = -1; }
    if (!in || !p || ((uintptr_t)p & 7u) || prog_len < sizeof(sdp_hdr_t) || prog_len > SD_PROG_MAX) goto bad;
    const sdp_hdr_t *h = sdp_hdr(p);
    uint32_t L = (uint32_t)prog_len;
    if (h->magic != SDP_MAGIC || h->version != SDP_VERSION || h->size != L || h->kind < SD_KIND_I2C ||
        h->kind > SD_KIND_ANALOG)
        goto bad;
    if (h->n_params > SD_MAX_PARAMS || h->n_lets > SD_MAX_LETS || h->n_states > SD_MAX_STATES ||
        h->n_msgs > SD_MAX_MSGS || h->n_reqs > SD_MAX_REQS || h->n_rules > SD_MAX_RULES)
        goto bad;
    if (!in_prog(L, h->params, h->n_params * sizeof(sdp_param_t)) || !in_prog(L, h->lets, 2u * h->n_lets) ||
        !in_prog(L, h->states, h->n_states * sizeof(sdp_state_t)) ||
        !in_prog(L, h->funcs, h->n_funcs * sizeof(sdp_func_t)) ||
        !in_prog(L, h->frags, h->n_frags * sizeof(sdp_func_t)) ||
        !in_prog(L, h->entries, h->n_entries * sizeof(sdp_entry_t)) ||
        !in_prog(L, h->rules, h->n_rules * sizeof(sdp_rule_t)) ||
        !in_prog(L, h->msgs, h->n_msgs * sizeof(sdp_msg_t)) ||
        !in_prog(L, h->reqs, h->n_reqs * sizeof(sdp_req_t)) ||
        (h->n_entries && !in_prog(L, h->owner, SD_REGIMAGE)))
        goto bad;
    memset(in, 0, sizeof(*in));
    in->prog = p;
    in->prog_len = L;
    const sdp_param_t *pp = (const sdp_param_t *)(const void *)(p + h->params);
    for (unsigned i = 0; i < h->n_params; i++) {
        in->raw[i] = pp[i].def;
        in->gain[i] = 1.0;
    }
    for (unsigned i = 0; i < SDV_N; i++) in->vars[i] = sdv_num(0);
    apply_params(in);
    in->vars[SDV_BUILTIN + SDB_WRITTEN] = sdv_num(-1);
    in->vars[SDV_BUILTIN + SDB_VALUE] = sdv_num(-1);
    in->corrupt_at = in->corrupt_applied_at = -1;
    in->rx_last_ms = -1;
    in->err.expr_off = -1;
    return SD_OK;
bad:
    if (err) { err->code = SD_E_ARG; memcpy(err->msg, "not a compiled sensor program", 30); }
    return SD_E_ARG;
}

sd_kind_t sd_kind(const sd_inst_t *in) { return in && in->prog ? (sd_kind_t)sd_h(in)->kind : SD_KIND_NONE; }

const char *sd_name(const sd_inst_t *in, size_t *len) {
    uint8_t l;
    const char *s = sdp_str(in->prog, sd_h(in)->name, &l);
    if (len) *len = l;
    return s;
}

unsigned sd_n_params(const sd_inst_t *in) { return sd_h(in)->n_params; }

int sd_param_index(const sd_inst_t *in, const char *key) {
    size_t n = strlen(key);
    const sdp_param_t *pp = params(in);
    for (unsigned i = 0; i < sd_h(in)->n_params; i++) {
        uint8_t l;
        const char *k = sdp_str(in->prog, pp[i].key, &l);
        if (l == n && memcmp(k, key, n) == 0) return (int)i;
    }
    return -1;
}

int sd_param_info(const sd_inst_t *in, unsigned i, sd_param_info_t *out) {
    if (i >= sd_h(in)->n_params || !out) return SD_E_ARG;
    const sdp_param_t *p = &params(in)[i];
    out->key = sdp_str(in->prog, p->key, &out->key_len);
    out->unit = sdp_str(in->prog, p->unit, &out->unit_len);
    out->label = sdp_str(in->prog, p->label, &out->label_len);
    out->min = p->min;
    out->max = p->max;
    out->def = p->def;
    return SD_OK;
}

int sd_set_param(sd_inst_t *in, unsigned i, double v, bool clamp) {
    if (i >= sd_h(in)->n_params || v != v) return SD_E_ARG;
    const sdp_param_t *p = &params(in)[i];
    if (v < p->min || v > p->max) {
        if (!clamp) return SD_E_RANGE;
        v = v < p->min ? p->min : p->max;
    }
    in->raw[i] = v;
    apply_params(in);
    return SD_OK;
}

double sd_get_param(const sd_inst_t *in, unsigned i) { return i < sd_h(in)->n_params ? in->raw[i] : 0.0; }
void sd_set_time_ms(sd_inst_t *in, int64_t now_ms) { in->now_ms = now_ms; }
const sd_err_t *sd_last_error(const sd_inst_t *in) { return &in->err; }
uint32_t sd_max_steps(const sd_inst_t *in) { return in->steps > in->max_steps_seen ? in->steps : in->max_steps_seen; }

/* ---- faults ---- */
static const char *const k_fault_names[] = {
    "", "stuck", "offset", "gain", "no_response", "corrupt", "bus_stuck", "garbage", "drop",
    "baud_error", "open", "short" };

const char *sd_fault_name(sd_fault_type_t t) {
    return (unsigned)t < sizeof(k_fault_names) / sizeof(k_fault_names[0]) ? k_fault_names[t] : "";
}

bool sd_fault_allowed(sd_fault_type_t t, sd_kind_t k) {
    switch (t) {
    case SD_FAULT_STUCK: case SD_FAULT_OFFSET: case SD_FAULT_GAIN: return k != SD_KIND_NONE;
    case SD_FAULT_NO_RESPONSE: return k == SD_KIND_I2C || k == SD_KIND_SPI || k == SD_KIND_UART;
    case SD_FAULT_CORRUPT: return k == SD_KIND_I2C || k == SD_KIND_SPI;
    case SD_FAULT_BUS_STUCK: return k == SD_KIND_I2C;
    case SD_FAULT_GARBAGE: case SD_FAULT_DROP: case SD_FAULT_BAUD_ERROR: return k == SD_KIND_UART;
    case SD_FAULT_OPEN: case SD_FAULT_SHORT: return k == SD_KIND_ANALOG;
    default: return false;
    }
}

int sd_set_fault(sd_inst_t *in, const sd_fault_t *f, bool on) {
    if (!f || !sd_fault_allowed(f->type, sd_kind(in))) return SD_E_ARG;
    int pi = -1;
    if (f->param[0]) {
        pi = sd_param_index(in, f->param);
        if (pi < 0) return SD_E_ARG;
    }
    switch (f->type) {
    case SD_FAULT_STUCK:
        if (pi < 0) { in->frozen = on; break; }
        if (on) {
            in->stuck_val[pi] = f->has_value ? f->value : sdv_d(in->vars[SDV_PARAM + pi]);
            in->stuck |= (uint16_t)(1u << pi);
        } else {
            in->stuck &= (uint16_t)~(1u << pi);
        }
        break;
    case SD_FAULT_OFFSET: case SD_FAULT_GAIN:
        if (pi < 0 || !f->has_value) return SD_E_ARG;
        if (f->type == SD_FAULT_OFFSET) in->offset[pi] = on ? f->value : 0.0;
        else in->gain[pi] = on ? f->value : 1.0;
        break;
    case SD_FAULT_CORRUPT:
        if (on && (f->at < 0 || f->at > 255)) return SD_E_ARG;
        in->corrupt_at = on ? f->at : -1;
        in->corrupt_mask = on ? f->mask : 0;
        break;
    case SD_FAULT_OPEN: case SD_FAULT_SHORT:
        in->force_out = on ? (f->type == SD_FAULT_OPEN ? 1 : 2) : 0;
        in->force_has_val = f->has_value;
        in->force_val = f->value;
        break;
    default:
        break;   /* bus faults: the manager acts on them */
    }
    apply_params(in);
    return SD_OK;
}

void sd_clear_faults(sd_inst_t *in) {
    for (unsigned i = 0; i < SD_MAX_PARAMS; i++) { in->gain[i] = 1.0; in->offset[i] = 0.0; }
    in->stuck = 0;
    in->frozen = false;
    in->corrupt_at = -1;
    in->force_out = 0;
    apply_params(in);
}

/* ---- per-evaluation preparation ---- */
int sd_prepare(sd_inst_t *in) {
    const sdp_hdr_t *h = sd_h(in);
    double t = (double)in->now_ms / 1000.0;
    in->vars[SDV_BUILTIN + SDB_T] = sdv_num(t);
    double utc0 = h->utc_param >= 0 ? sdv_d(in->vars[SDV_PARAM + h->utc_param]) : 0.0;
    in->vars[SDV_BUILTIN + SDB_UTC] = sdv_num(utc0 + t);
    double dt = in->started ? (double)(in->now_ms - in->state_ms) / 1000.0 : 0.0;
    in->vars[SDV_BUILTIN + SDB_DT] = sdv_num(dt);
    const sdp_state_t *st = (const sdp_state_t *)(const void *)(in->prog + h->states);
    for (unsigned i = 0; i < h->n_states; i++) {
        bool reinit = !in->started;
        if (st[i].follow >= 0) {
            double pv = sdv_d(in->vars[SDV_PARAM + st[i].follow]);
            if (pv != in->follow_last[i]) reinit = true;
            in->follow_last[i] = pv;
        }
        double v;
        int rc = sd_vm_num(in, reinit ? st[i].init : st[i].update, &v);
        if (rc) return rc;
        in->vars[SDV_STATE + i] = sdv_num(v);
    }
    in->state_ms = in->now_ms;
    in->started = true;
    for (unsigned k = 0; k < h->n_lets; k++) {
        uint64_t r;
        int rc = sd_vm_eval(in, sdp_code_at(in->prog, h->lets, k), &r);
        if (rc) return rc;
        in->vars[SDV_LET + k] = r;
    }
    return SD_OK;
}
