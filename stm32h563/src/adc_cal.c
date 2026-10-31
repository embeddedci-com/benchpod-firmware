/*
 * adc_cal.c — per-pod ADC calibration (see adc_cal.h).
 *
 * Pure: no HAL. The store is config_store.c, so the whole file runs in the host tests. Taking
 * the measurements (route + capture) is adc_cal_hw.c.
 *
 * Two halves: the record (raw measurements, what flash holds) and the derived state (the offset,
 * the DAC lines and the gains every reader uses). derive() turns one into the other with the
 * compiled general fits; it runs at load and after every change, and refuses a record whose
 * result is out of range.
 */
#include "adc_cal.h"
#include "config_store.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

_Static_assert(sizeof(adc_cal_t) <= AB_STORE_MAX_PAYLOAD, "adc_cal_t must fit in an A/B record");
_Static_assert(sizeof(adc_cal_t) == 116, "the version-2 record is 116 bytes");
_Static_assert(sizeof(adc_cal_v1_t) == 32, "the version-1 record is 32 bytes");

static const ab_store_t s_store = {
    .tag      = "adc-cal",
    .slot_off = { ADC_CAL_SLOT_A_OFFSET, ADC_CAL_SLOT_B_OFFSET },
    .magic    = ADC_CAL_RECORD_MAGIC,
    .version  = ADC_CAL_VERSION,
};

/* The same slots as version 1: the copy firmware 3.4.0 to 3.9.3 reads. */
static const ab_store_t s_store_v1 = {
    .tag      = "adc-cal",
    .slot_off = { ADC_CAL_SLOT_A_OFFSET, ADC_CAL_SLOT_B_OFFSET },
    .magic    = ADC_CAL_RECORD_MAGIC,
    .version  = ADC_CAL_VERSION_1,
};

/* The record in use, and what derive() made of it. */
static adc_cal_t s_rec;
static struct {
    float     offset_v;                    /* general cal1 fit at the zero; 0 without one */
    cal_lin_t dac[ADC_CAL_DAC_PATHS];      /* this pod's lines (where dac_set) */
    float     gain[ADC_CAL_SRCS];          /* 1.0 where gain_set is clear */
    float     gain_low;                    /* 1.0 where low_set is clear */
} s_der = { .gain = { 1.0f, 1.0f, 1.0f }, .gain_low = 1.0f };

/* Refusal text. No %f: newlib-nano's printf has none, so volts are printed as whole mV.
   Commands are dispatched one at a time and the caller copies the text into its reply
   straight away, so one buffer is enough. */
static char s_msg[200];

static const char *const s_path_name[ADC_CAL_DAC_PATHS] = { "3v3", "5v", "12v" };
static const char *const s_src_name[ADC_CAL_SRCS] = { "cal1", "cal2", "ext" };

static bool in_range(double v) { return fabs(v) * 1e6 <= ADC_CAL_MAX_OFFSET_UV; }
static bool path_ok(int path) { return path >= 0 && path < ADC_CAL_DAC_PATHS; }
static bool src_ok(int src) { return src >= 0 && src < ADC_CAL_SRCS; }
static bool gain_ok(double g) { return isfinite(g) && fabs(g - 1.0) * 1e6 <= ADC_CAL_MAX_GAIN_PPM; }
static bool low_gain_ok(double g) { return isfinite(g) && fabs(g - 1.0) * 1e6 <= ADC_CAL_MAX_LOW_GAIN_PPM; }

const char *adc_cal_src_name(int src) { return src_ok(src) ? s_src_name[src] : "?"; }
int adc_cal_src_index(const char *name) {
    for (int i = 0; name && i < ADC_CAL_SRCS; i++)
        if (strcmp(name, s_src_name[i]) == 0) return i;
    return -1;
}

static cal_lin_t compiled_src_fit(int src) {
    return src == ADC_CAL_SRC_CAL2 ? ADC_CAL_CAL2 : src == ADC_CAL_SRC_EXT ? ADC_CAL_EXT : ADC_CAL_CAL1;
}

/* A general fit at a count, in double: counts sit near 65536, where float has 1/128 of a count. */
static double fit_at(cal_lin_t c, double count) { return (double)c.a + (double)c.b * count; }

/* What a line gives at a 16-bit code, and how far a reading may sit from what the compiled
   line gives there. */
static double line_volts(cal_lin_t c, uint16_t code16) { return (double)c.a + (double)c.b * code16 / 256.0; }
static double dac_tol_v(double nominal_v) {
    return ADC_CAL_DAC_TOL_UV / 1e6 + fabs(nominal_v) * ADC_CAL_DAC_TOL_PPM / 1e6;
}

/* The line through two DAC points (cal_data.h convention: volts = a + b*code8). */
static cal_lin_t dac_line_from(const adc_cal_dac_pts_t *p) {
    double b = ((double)p->uv[1] - (double)p->uv[0]) / 1e6 / (((double)p->code16[1] - (double)p->code16[0]) / 256.0);
    double a = (double)p->uv[0] / 1e6 - b * (double)p->code16[0] / 256.0;
    return (cal_lin_t){ (float)a, (float)b };
}

/* A DMM line is plausible for its path: gain within the limit of the compiled line and, at a
   quarter and three quarters of the range, within the reading tolerance. */
static bool dac_line_ok(int path, cal_lin_t pod) {
    cal_lin_t c = DAC_CAL_COMPILED[path];
    if (!isfinite(pod.a) || !isfinite(pod.b) || c.b == 0.0f) return false;
    if (fabs((double)pod.b / (double)c.b - 1.0) * 1e6 > ADC_CAL_MAX_GAIN_PPM) return false;
    static const uint16_t probe[2] = { 16384, 49152 };
    for (int i = 0; i < 2; i++) {
        double nom = line_volts(c, probe[i]);
        if (fabs(line_volts(pod, probe[i]) - nom) > dac_tol_v(nom)) return false;
    }
    return true;
}

/* The gain two points give a source: their slope over the general fit's slope between the same
   counts. The offset cancels, so it does not depend on it. NAN when the points cannot give one. */
static double gain_from(int src, const adc_cal_gain_pts_t *p) {
    double dc = ((double)p->count_mc[1] - (double)p->count_mc[0]) / 1000.0;
    double dr = (double)compiled_src_fit(src).b * dc;
    double dv = ((double)p->uv[1] - (double)p->uv[0]) / 1e6;
    if (fabs(dv) < 1.0 || dr == 0.0) return NAN;
    return dv / dr;
}

/* The low-range gain two cal1 points give: their slope over the low range's slope before the
   gain, ratio * g_cal1 * the compiled cal1 slope. */
static double low_gain_from(const adc_cal_gain_pts_t *p, double g_cal1) {
    double dc = ((double)p->count_mc[1] - (double)p->count_mc[0]) / 1000.0;
    double dr = (double)cal_adc_low_ratio() * g_cal1 * (double)ADC_CAL_CAL1.b * dc;
    double dv = ((double)p->uv[1] - (double)p->uv[0]) / 1e6;
    if (fabs(dv) < 1.0 || dr == 0.0) return NAN;
    return dv / dr;
}

/* The record turned into what the readers use, with the compiled fits of the running firmware.
   false = something in it is out of range (nothing is changed then). */
static bool derive(const adc_cal_t *r) {
    float offset = 0.0f;
    if (r->zero_set) {
        double off = fit_at(ADC_CAL_CAL1, (double)r->zero_count_mc / 1000.0);
        if (!in_range(off)) return false;
        offset = (float)off;
    }
    cal_lin_t dac[ADC_CAL_DAC_PATHS];
    for (int i = 0; i < ADC_CAL_DAC_PATHS; i++) {
        dac[i] = DAC_CAL_COMPILED[i];
        if (!(r->dac_set & (1u << i))) continue;
        const adc_cal_dac_pts_t *p = &r->dac[i];
        if (p->code16[0] == p->code16[1]) return false;
        dac[i] = dac_line_from(p);
        if (!dac_line_ok(i, dac[i])) return false;
    }
    float gain[ADC_CAL_SRCS];
    for (int i = 0; i < ADC_CAL_SRCS; i++) {
        gain[i] = 1.0f;
        if (!(r->gain_set & (1u << i))) continue;
        double g = gain_from(i, &r->gain[i]);
        if (!gain_ok(g)) return false;
        gain[i] = (float)g;
    }
    float gain_low = 1.0f;
    if (r->low_set) {
        double g = low_gain_from(&r->gain_low, gain[ADC_CAL_SRC_CAL1]);
        if (!low_gain_ok(g)) return false;
        gain_low = (float)g;
    }
    s_der.offset_v = offset;
    s_der.gain_low = gain_low;
    memcpy(s_der.dac, dac, sizeof(dac));
    memcpy(s_der.gain, gain, sizeof(gain));
    s_rec = *r;
    cal_data_set_pod_dac(s_der.dac, s_rec.dac_set);
    return true;
}

static bool record_empty(const adc_cal_t *r) { return !r->comp_hf_dc_e4 && !r->zero_set && !r->dac_set && !r->gain_set && !r->low_set; }

static void derive_nothing(void) {
    adc_cal_t empty;
    memset(&empty, 0, sizeof(empty));
    (void)derive(&empty);
}

/* The version-1 copy of a record: the offset in uV from its zero and the current cal1 fit. */
static adc_cal_v1_t v1_copy(const adc_cal_t *r) {
    adc_cal_v1_t v1;
    memset(&v1, 0, sizeof(v1));
    if (r->zero_set) {
        v1.current_in_set = 1;
        v1.current_in_offset_uv = (int32_t)lround(fit_at(ADC_CAL_CAL1, (double)r->zero_count_mc / 1000.0) * 1e6);
    }
    return v1;
}

/* Save a record (the version-2 record, then its version-1 copy) and use it. An empty record
   clears the store instead. The record has passed derive() already. */
static int commit(const adc_cal_t *rec) {
    if (record_empty(rec)) {
        if (ab_store_clear(&s_store) != 0) return -1;
    } else {
        adc_cal_v1_t v1 = v1_copy(rec);
        if (ab_store_save(&s_store, rec, sizeof(*rec)) != 0) return -1;
        if (ab_store_save(&s_store_v1, &v1, sizeof(v1)) != 0) return -1;
    }
    (void)derive(rec);
    return 0;
}

/* Check a changed record and save it. NULL = done, else why not (range_msg when it is out of
   range). The old calibration stays in use on a refusal. */
static const char *apply(const adc_cal_t *rec, const char *range_msg) {
    adc_cal_t keep = s_rec;
    if (!derive(rec)) return range_msg;
    (void)derive(&keep);   /* derive() made it current: back to the old one until it is saved */
    if (commit(rec) != 0) return "calibrate: could not save to flash";
    return NULL;
}

bool    adc_cal_current_in_is_set(void)    { return s_rec.zero_set != 0; }
int32_t adc_cal_current_in_offset_uv(void) { return s_rec.zero_set ? (int32_t)lroundf(s_der.offset_v * 1e6f) : 0; }
int32_t adc_cal_current_in_offset_mv(void) { return (int32_t)lroundf((float)adc_cal_current_in_offset_uv() / 1000.0f); }
int32_t adc_cal_zero_count_mc(void)        { return s_rec.zero_set ? s_rec.zero_count_mc : 0; }

cal_lin_t adc_cal_fit(cal_lin_t base) {
    base.a -= s_der.offset_v;
    return base;
}

bool  adc_cal_gain_is_set(int src) { return src_ok(src) && (s_rec.gain_set & (1u << src)); }
float adc_cal_gain(int src) { return src_ok(src) ? s_der.gain[src] : 1.0f; }

cal_lin_t adc_cal_source_fit(int src) {
    cal_lin_t f = adc_cal_fit(compiled_src_fit(src));
    float g = adc_cal_gain(src);
    f.a *= g;
    f.b *= g;
    return f;
}

cal_lin_t adc_cal_current_in_fit(void) { return adc_cal_source_fit(ADC_CAL_SRC_CAL1); }

bool  adc_cal_low_gain_is_set(void) { return s_rec.low_set != 0; }
float adc_cal_low_gain(void) { return s_der.gain_low; }

cal_lin_t adc_cal_range_fit(int src, int range) {
    cal_lin_t f = adc_cal_source_fit(src);
    if (range != 1) return f;
    float k = cal_adc_low_ratio() * s_der.gain_low;
    f.a *= k;
    f.b *= k;
    return f;
}

long adc_cal_fullscale_mv(cal_lin_t fit) { return lround(fabs((double)fit.b) * 32768.0 * 1000.0); }

const float ADC_RANGE_TEST_V[ADC_RANGE_TEST_POINTS] = { 1.0f, 3.5f };

bool adc_range_test_point_ok(float set_v, float normal_v, float low_v) {
    double tol = fabs((double)set_v) * ADC_RANGE_TEST_TOL_PPM / 1e6 + ADC_RANGE_TEST_TOL_MV / 1000.0;
    if (!isfinite(normal_v) || !isfinite(low_v)) return false;
    return fabs((double)low_v - (double)normal_v) <= tol && fabs((double)normal_v - (double)set_v) <= tol &&
           fabs((double)low_v - (double)set_v) <= tol;
}

const char *adc_cal_current_in_store(const adc_reading_t *rd) {
    if (!rd) return "calibrate: no reading";
    if (!rd->valid) {
        snprintf(s_msg, sizeof(s_msg),
                 "calibrate: input not settled on current_in (%u counts pk-pk, limit %u). "
                 "Disconnect J8 and try again",
                 (unsigned)rd->span, (unsigned)ADC_BURST_MAX_SPAN);
        return s_msg;
    }
    if (!in_range(rd->volts)) {
        snprintf(s_msg, sizeof(s_msg),
                 "calibrate: current_in reads %ld mV, it must be within +/-%d mV of zero. "
                 "Something is driving J8. Disconnect it and try again",
                 lroundf(rd->volts * 1000.0f), ADC_CAL_MAX_OFFSET_UV / 1000);
        return s_msg;
    }
    adc_cal_t rec = s_rec;   /* keep whatever else the record holds */
    rec.zero_set = 1;
    rec.zero_count_mc = (int32_t)lroundf(rd->count_uw * 1000.0f);
    return apply(&rec, "calibrate: the zero is out of range");
}

/* ---- DMM parts ------------------------------------------------------------------------- */

bool adc_cal_dac_is_set(int path) { return path_ok(path) && (s_rec.dac_set & (1u << path)); }
cal_lin_t adc_cal_dac_line(int path) {
    return adc_cal_dac_is_set(path) ? s_der.dac[path] : DAC_CAL_COMPILED[path_ok(path) ? path : 0];
}

const char *adc_cal_dac_store(int path, uint16_t code1, int32_t uv1, uint16_t code2, int32_t uv2) {
    if (!path_ok(path)) return "calibrate: dac must be 3v3, 5v or 12v";
    const char *name = s_path_name[path];
    if ((unsigned)(code1 > code2 ? code1 - code2 : code2 - code1) < ADC_CAL_DAC_MIN_SPAN) {
        snprintf(s_msg, sizeof(s_msg),
                 "calibrate: the two DAC codes (%u, %u) must be at least %u apart. Drive two set points "
                 "far apart on the %s output",
                 (unsigned)code1, (unsigned)code2, (unsigned)ADC_CAL_DAC_MIN_SPAN, name);
        return s_msg;
    }
    cal_lin_t c = DAC_CAL_COMPILED[path];
    const uint16_t code[2] = { code1, code2 };
    const int32_t uv[2] = { uv1, uv2 };
    for (int i = 0; i < 2; i++) {
        double nom = line_volts(c, code[i]);
        if (fabs(uv[i] / 1e6 - nom) > dac_tol_v(nom)) {
            snprintf(s_msg, sizeof(s_msg),
                     "calibrate: the DMM reads %ld mV where the %s output gives about %ld mV at code %u. "
                     "Is the DMM on the %s SMA, set to DC volts? Nothing was stored",
                     lround(uv[i] / 1000.0), name, lround(nom * 1000.0), (unsigned)code[i], name);
            return s_msg;
        }
    }
    adc_cal_t rec = s_rec;
    rec.dac[path] = (adc_cal_dac_pts_t){ { code1, code2 }, { uv1, uv2 } };
    rec.dac_set |= (uint8_t)(1u << path);
    cal_lin_t line = dac_line_from(&rec.dac[path]);
    snprintf(s_msg, sizeof(s_msg),
             "calibrate: the two readings give the %s output a gain %ld ppm off its built-in one "
             "(limit %d). Check both readings. Nothing was stored",
             name, lround(((double)line.b / (double)c.b - 1.0) * 1e6), ADC_CAL_MAX_GAIN_PPM);
    return apply(&rec, s_msg);
}

int adc_cal_gain_path(int src) { return src == ADC_CAL_SRC_CAL1 ? 1 : 2; }

float adc_cal_gain_point(int src, int i) {
    static const float pts5[2] = { 1.0f, 4.0f }, pts12[2] = { -5.0f, 5.0f };
    return (adc_cal_gain_path(src) == 1 ? pts5 : pts12)[i ? 1 : 0];
}

const char *adc_cal_gain_ready(int src) {
    if (!src_ok(src)) return "calibrate: adc must be cal1, cal2 or ext";
    if (!adc_cal_current_in_is_set())
        return "calibrate: calibrate the ADC offset first (J8 open, {\"cmd\":\"calibrate\",\"source\":\"current_in\"})";
    int path = adc_cal_gain_path(src);
    if (!adc_cal_dac_is_set(path)) {
        snprintf(s_msg, sizeof(s_msg),
                 "calibrate: %s is measured against the %s output: calibrate that output with the DMM first",
                 s_src_name[src], s_path_name[path]);
        return s_msg;
    }
    return NULL;
}

const char *adc_cal_gain_store(int src, adc_cal_gain_meas_t *m) {
    const char *why = adc_cal_gain_ready(src);
    if (why) return why;
    if (!m) return "calibrate: no measurement";
    adc_cal_gain_pts_t p;
    for (int i = 0; i < 2; i++) {
        p.count_mc[i] = (int32_t)lroundf(m->count[i] * 1000.0f);
        p.uv[i] = (int32_t)lroundf(m->v[i] * 1e6f);
        m->r[i] = (float)(fit_at(compiled_src_fit(src), (double)p.count_mc[i] / 1000.0) - s_der.offset_v);
    }
    double g = gain_from(src, &p);
    if (!isfinite(g)) return "calibrate: the set points are too close";
    m->gain = (float)g;
    m->residual_v = (float)(((m->v[0] - g * m->r[0]) + (m->v[1] - g * m->r[1])) / 2.0);
    if (!gain_ok(g)) {
        const char *hint = src == ADC_CAL_SRC_EXT
            ? "Is the SMA cable from the +-12 V output to the ADC input connected?"
            : src == ADC_CAL_SRC_CAL1 ? "Is something connected to the 5 V output?"
                                      : "Is something connected to the +-12 V output?";
        snprintf(s_msg, sizeof(s_msg),
                 "calibrate: %s reads %ld and %ld mV where the DAC gives %ld and %ld mV, a gain %ld ppm off "
                 "(limit %d). %s Nothing was stored",
                 s_src_name[src], lroundf(m->r[0] * 1000.0f), lroundf(m->r[1] * 1000.0f),
                 lroundf(m->v[0] * 1000.0f), lroundf(m->v[1] * 1000.0f), lround((g - 1.0) * 1e6),
                 ADC_CAL_MAX_GAIN_PPM, hint);
        return s_msg;
    }
    adc_cal_t rec = s_rec;
    rec.gain[src] = p;
    rec.gain_set |= (uint8_t)(1u << src);
    return apply(&rec, "calibrate: the gain is out of range");
}

const char *adc_cal_low_gain_store(adc_cal_gain_meas_t *m) {
    const char *why = adc_cal_gain_ready(ADC_CAL_SRC_CAL1);
    if (why) return why;
    if (!m) return "calibrate: no measurement";
    double g_cal1 = s_der.gain[ADC_CAL_SRC_CAL1], k = (double)cal_adc_low_ratio() * g_cal1;
    adc_cal_gain_pts_t p;
    for (int i = 0; i < 2; i++) {
        p.count_mc[i] = (int32_t)lroundf(m->count[i] * 1000.0f);
        p.uv[i] = (int32_t)lroundf(m->v[i] * 1e6f);
        m->r[i] = (float)(k * (fit_at(ADC_CAL_CAL1, (double)p.count_mc[i] / 1000.0) - s_der.offset_v));
    }
    double g = low_gain_from(&p, g_cal1);
    if (!isfinite(g)) return "calibrate: the set points are too close";
    m->gain = (float)g;
    m->residual_v = (float)(((m->v[0] - g * m->r[0]) + (m->v[1] - g * m->r[1])) / 2.0);
    if (!low_gain_ok(g)) {
        snprintf(s_msg, sizeof(s_msg),
                 "calibrate: the low range reads %ld and %ld mV where the DAC gives %ld and %ld mV, a gain "
                 "%ld ppm off (limit %d). Is something connected to the 5 V output? Nothing was stored",
                 lroundf(m->r[0] * 1000.0f), lroundf(m->r[1] * 1000.0f), lroundf(m->v[0] * 1000.0f),
                 lroundf(m->v[1] * 1000.0f), lround((g - 1.0) * 1e6), ADC_CAL_MAX_LOW_GAIN_PPM);
        return s_msg;
    }
    adc_cal_t rec = s_rec;
    rec.gain_low = p;
    rec.low_set = 1;
    return apply(&rec, "calibrate: the low-range gain is out of range");
}

bool adc_cal_comp_is_set(void) {
    return s_rec.comp_hf_dc_e4 >= ADC_CAL_COMP_HF_DC_MIN_E4 && s_rec.comp_hf_dc_e4 <= ADC_CAL_COMP_HF_DC_MAX_E4 &&
           s_rec.comp_c_top_ff >= ADC_CAL_COMP_C_TOP_MIN_FF && s_rec.comp_c_top_ff <= ADC_CAL_COMP_C_TOP_MAX_FF;
}
uint16_t adc_cal_comp_hf_dc_e4(void) { return adc_cal_comp_is_set() ? s_rec.comp_hf_dc_e4 : 0; }
uint16_t adc_cal_comp_c_top_ff(void) { return adc_cal_comp_is_set() ? s_rec.comp_c_top_ff : 0; }

const char *adc_cal_comp_store(long hf_dc_e4, long c_top_ff) {
    if (hf_dc_e4 < ADC_CAL_COMP_HF_DC_MIN_E4 || hf_dc_e4 > ADC_CAL_COMP_HF_DC_MAX_E4 ||
        c_top_ff < ADC_CAL_COMP_C_TOP_MIN_FF || c_top_ff > ADC_CAL_COMP_C_TOP_MAX_FF)
        return "calibrate: the sweep result is out of range";
    adc_cal_t rec = s_rec;
    rec.comp_hf_dc_e4 = (uint16_t)hf_dc_e4;
    rec.comp_c_top_ff = (uint16_t)c_top_ff;
    return commit(&rec) == 0 ? NULL : "calibrate: could not save to flash";
}

int adc_cal_clear(void) {
    derive_nothing();
    return ab_store_clear(&s_store);
}

int adc_cal_clear_part(const char *part) {
    if (!part) return adc_cal_clear();
    adc_cal_t rec = s_rec;
    if (strcmp(part, "offset") == 0) {
        rec.zero_set = 0;
        rec.zero_count_mc = 0;
    } else if (strcmp(part, "dac") == 0) {
        rec.dac_set = 0;
        memset(rec.dac, 0, sizeof(rec.dac));
    } else if (strcmp(part, "adc") == 0) {
        rec.gain_set = 0;
        memset(rec.gain, 0, sizeof(rec.gain));
        rec.low_set = 0;
        memset(&rec.gain_low, 0, sizeof(rec.gain_low));
        rec.comp_hf_dc_e4 = 0;
        rec.comp_c_top_ff = 0;
    } else {
        return -2;
    }
    return commit(&rec);   /* less than before: always in range */
}

/* The record to load: the version-2 record when it is the newest or the pair of the newest
   version-1 copy (one save behind it), else the newest version-1 record converted to a zero. */
static bool pick_record(adc_cal_t *out) {
    uint32_t b2 = 0, b1 = 0;
    adc_cal_v1_t v1;
    bool has2 = ab_store_load_version(&s_store, ADC_CAL_VERSION, out, sizeof(*out), &b2) == 0;
    bool has1 = ab_store_load_version(&s_store, ADC_CAL_VERSION_1, &v1, sizeof(v1), &b1) == 0;
    if (has2 && (b2 == 0 || (has1 && b1 == 0 && b2 == 1))) return true;
    memset(out, 0, sizeof(*out));
    if (!has1 || b1 != 0 || !v1.current_in_set) return false;
    /* A version-1 record holds the offset in uV on the cal1 fit of its day (adc_cal1_v1): the
       count that fit reads as that offset is the zero. Exact, so the pod reads as it did. */
    cal_lin_t f = cal_data()->adc_cal1_v1;
    if (f.b == 0.0f) return false;
    out->zero_set = 1;
    out->zero_count_mc = (int32_t)lround(((double)v1.current_in_offset_uv / 1e6 - (double)f.a) / (double)f.b * 1000.0);
    printf("[adc-cal] converted a version-1 record (offset %ld uV) to zero count %ld.%03ld; "
           "the next save writes version 2\n", (long)v1.current_in_offset_uv,
           (long)(out->zero_count_mc / 1000), (long)labs(out->zero_count_mc % 1000));
    return true;
}

int adc_cal_load(void) {
    adc_cal_t rec;
    if (!pick_record(&rec) || record_empty(&rec)) {
        derive_nothing();
        return -1;
    }
    if (!derive(&rec)) {
        printf("[adc-cal] stored calibration out of range, ignored\n");
        derive_nothing();
        return -1;
    }
    return 0;
}
