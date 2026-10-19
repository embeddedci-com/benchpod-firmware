/*
 * adc_cal.c — per-pod ADC calibration (see adc_cal.h).
 *
 * Pure: no HAL. The store is config_store.c, so the whole file runs in the host tests. Taking
 * the measurements (route + capture) is adc_cal_hw.c.
 */
#include "adc_cal.h"
#include "config_store.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

_Static_assert(sizeof(adc_cal_t) <= AB_STORE_MAX_PAYLOAD, "adc_cal_t must fit in an A/B record");
_Static_assert(sizeof(adc_cal_v1_t) == 32, "the version-1 record is 32 bytes");

static const ab_store_t s_store = {
    .tag      = "adc-cal",
    .slot_off = { ADC_CAL_SLOT_A_OFFSET, ADC_CAL_SLOT_B_OFFSET },
    .magic    = ADC_CAL_RECORD_MAGIC,
    .version  = ADC_CAL_VERSION,
};

/* The same slots read as version 1, for a pod that last saved on firmware 3.4.0 to 3.9.3. */
static const ab_store_t s_store_v1 = {
    .tag      = "adc-cal",
    .slot_off = { ADC_CAL_SLOT_A_OFFSET, ADC_CAL_SLOT_B_OFFSET },
    .magic    = ADC_CAL_RECORD_MAGIC,
    .version  = ADC_CAL_VERSION_1,
};

static adc_cal_t s_active;

/* Refusal text. No %f: newlib-nano's printf has none, so volts are printed as whole mV.
   Commands are dispatched one at a time and the caller copies the text into its reply
   straight away, so one buffer is enough. */
static char s_msg[200];

static const char *const s_path_name[ADC_CAL_DAC_PATHS] = { "3v3", "5v", "12v" };
static const char *const s_src_name[ADC_CAL_SRCS] = { "cal1", "cal2", "ext" };

static bool in_range(int32_t uv) { return uv >= -ADC_CAL_MAX_OFFSET_UV && uv <= ADC_CAL_MAX_OFFSET_UV; }
static bool path_ok(int path) { return path >= 0 && path < ADC_CAL_DAC_PATHS; }
static bool src_ok(int src) { return src >= 0 && src < ADC_CAL_SRCS; }
static bool gain_ok(float g) { return isfinite(g) && fabsf(g - 1.0f) * 1e6f <= (float)ADC_CAL_MAX_GAIN_PPM; }

const char *adc_cal_src_name(int src) { return src_ok(src) ? s_src_name[src] : "?"; }
int adc_cal_src_index(const char *name) {
    for (int i = 0; name && i < ADC_CAL_SRCS; i++)
        if (strcmp(name, s_src_name[i]) == 0) return i;
    return -1;
}

static cal_lin_t compiled_src_fit(int src) {
    return src == ADC_CAL_SRC_CAL2 ? ADC_CAL_CAL2 : src == ADC_CAL_SRC_EXT ? ADC_CAL_EXT : ADC_CAL_CAL1;
}

/* What a line gives at a 16-bit code, and how far a reading may sit from what the compiled
   line gives there. */
static double line_volts(cal_lin_t c, uint16_t code16) { return (double)c.a + (double)c.b * code16 / 256.0; }
static double dac_tol_v(double nominal_v) {
    return ADC_CAL_DAC_TOL_UV / 1e6 + fabs(nominal_v) * ADC_CAL_DAC_TOL_PPM / 1e6;
}

/* A DMM line is plausible for its path: gain within the limit of the compiled line and, at a
   quarter and three quarters of the range, within the reading tolerance. Also for load. */
static bool dac_line_ok(int path, cal_lin_t pod) {
    cal_lin_t c = DAC_CAL_COMPILED[path];
    if (!isfinite(pod.a) || !isfinite(pod.b) || c.b == 0.0f) return false;
    if (fabs((double)pod.b / (double)c.b - 1.0) * 1e6 > ADC_CAL_MAX_GAIN_PPM) return false;
    static const uint16_t probe[2] = { 16384, 49152 };   /* a quarter and three quarters of the range */
    for (int i = 0; i < 2; i++) {
        double nom = line_volts(c, probe[i]);
        if (fabs(line_volts(pod, probe[i]) - nom) > dac_tol_v(nom)) return false;
    }
    return true;
}

/* Every part of a record is in range; an out-of-range record is ignored as a whole. */
static bool record_ok(const adc_cal_t *r) {
    if (r->current_in_set && !in_range(r->current_in_offset_uv)) return false;
    for (int i = 0; i < ADC_CAL_DAC_PATHS; i++)
        if ((r->dac_set & (1u << i)) && !dac_line_ok(i, r->dac[i])) return false;
    for (int i = 0; i < ADC_CAL_SRCS; i++)
        if ((r->adc_gain_set & (1u << i)) && !gain_ok(r->adc_gain[i])) return false;
    return true;
}

static bool record_empty(const adc_cal_t *r) {
    return !r->current_in_set && !r->dac_set && !r->adc_gain_set;
}

/* Make a record the active one: the DAC lines go to every DAC user (cal_data.h). */
static void activate(const adc_cal_t *r) {
    s_active = *r;
    cal_data_set_pod_dac(s_active.dac, s_active.dac_set);
}

/* Save a record and make it the active one. An empty record clears the store instead. */
static int commit(const adc_cal_t *rec) {
    int rc = record_empty(rec) ? ab_store_clear(&s_store) : ab_store_save(&s_store, rec, sizeof(*rec));
    if (rc != 0) return -1;
    activate(rec);
    return 0;
}

bool    adc_cal_current_in_is_set(void)    { return s_active.current_in_set != 0; }
int32_t adc_cal_current_in_offset_uv(void) { return s_active.current_in_set ? s_active.current_in_offset_uv : 0; }
int32_t adc_cal_current_in_offset_mv(void) { return (int32_t)lroundf((float)adc_cal_current_in_offset_uv() / 1000.0f); }

cal_lin_t adc_cal_fit(cal_lin_t base) {
    base.a -= (float)adc_cal_current_in_offset_uv() / 1000000.0f;
    return base;
}

bool  adc_cal_gain_is_set(int src) { return src_ok(src) && (s_active.adc_gain_set & (1u << src)); }
float adc_cal_gain(int src) { return adc_cal_gain_is_set(src) ? s_active.adc_gain[src] : 1.0f; }

cal_lin_t adc_cal_source_fit(int src) {
    cal_lin_t f = adc_cal_fit(compiled_src_fit(src));
    float g = adc_cal_gain(src);
    f.a *= g;
    f.b *= g;
    return f;
}

cal_lin_t adc_cal_current_in_fit(void) { return adc_cal_source_fit(ADC_CAL_SRC_CAL1); }

const char *adc_cal_current_in_store(const adc_reading_t *rd) {
    if (!rd) return "calibrate: no reading";
    if (!rd->valid) {
        snprintf(s_msg, sizeof(s_msg),
                 "calibrate: input not settled on current_in (%u counts pk-pk, limit %u). "
                 "Disconnect J8 and try again",
                 (unsigned)rd->span, (unsigned)ADC_BURST_MAX_SPAN);
        return s_msg;
    }
    long uv = lroundf(rd->volts * 1000000.0f);
    if (!in_range((int32_t)uv)) {
        snprintf(s_msg, sizeof(s_msg),
                 "calibrate: current_in reads %ld mV, it must be within +/-%d mV of zero. "
                 "Something is driving J8. Disconnect it and try again",
                 lroundf(rd->volts * 1000.0f), ADC_CAL_MAX_OFFSET_UV / 1000);
        return s_msg;
    }
    adc_cal_t rec = s_active;   /* keep whatever else the record holds */
    rec.current_in_set = 1;
    rec.current_in_offset_uv = (int32_t)uv;
    if (commit(&rec) != 0) return "calibrate: could not save to flash";
    return NULL;
}

/* ---- DMM parts ------------------------------------------------------------------------- */

bool adc_cal_dac_is_set(int path) { return path_ok(path) && (s_active.dac_set & (1u << path)); }
cal_lin_t adc_cal_dac_line(int path) {
    return adc_cal_dac_is_set(path) ? s_active.dac[path] : DAC_CAL_COMPILED[path_ok(path) ? path : 0];
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
    /* volts = a + b*code8, code8 = code16 / 256 (cal_data.h). */
    double b = ((double)uv2 - (double)uv1) / 1e6 / (((double)code2 - (double)code1) / 256.0);
    double a = (double)uv1 / 1e6 - b * (double)code1 / 256.0;
    cal_lin_t line = { (float)a, (float)b };
    if (!dac_line_ok(path, line)) {
        snprintf(s_msg, sizeof(s_msg),
                 "calibrate: the two readings give the %s output a gain %ld ppm off its built-in one "
                 "(limit %d). Check both readings. Nothing was stored",
                 name, lround((b / (double)c.b - 1.0) * 1e6), ADC_CAL_MAX_GAIN_PPM);
        return s_msg;
    }
    adc_cal_t rec = s_active;
    rec.dac[path] = line;
    rec.dac_set |= (uint8_t)(1u << path);
    if (commit(&rec) != 0) return "calibrate: could not save to flash";
    return NULL;
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
    float dv = m->v[1] - m->v[0], dr = m->r[1] - m->r[0];
    if (fabsf(dv) < 1.0f || !isfinite(dr) || dr == 0.0f) return "calibrate: the set points are too close";
    float g = dv / dr;
    if (!gain_ok(g)) {
        const char *hint = src == ADC_CAL_SRC_EXT
            ? "Is the SMA cable from the +-12 V output to the ADC input connected?"
            : src == ADC_CAL_SRC_CAL1 ? "Is something connected to the 5 V output?"
                                      : "Is something connected to the +-12 V output?";
        snprintf(s_msg, sizeof(s_msg),
                 "calibrate: %s reads %ld and %ld mV where the DAC gives %ld and %ld mV, a gain %ld ppm off "
                 "(limit %d). %s Nothing was stored",
                 s_src_name[src], lroundf(m->r[0] * 1000.0f), lroundf(m->r[1] * 1000.0f),
                 lroundf(m->v[0] * 1000.0f), lroundf(m->v[1] * 1000.0f), lroundf((g - 1.0f) * 1e6f),
                 ADC_CAL_MAX_GAIN_PPM, hint);
        return s_msg;
    }
    m->gain = g;
    m->residual_v = ((m->v[0] - g * m->r[0]) + (m->v[1] - g * m->r[1])) / 2.0f;
    adc_cal_t rec = s_active;
    rec.adc_gain[src] = g;
    rec.adc_gain_set |= (uint8_t)(1u << src);
    if (commit(&rec) != 0) return "calibrate: could not save to flash";
    return NULL;
}

int adc_cal_clear(void) {
    adc_cal_t empty;
    memset(&empty, 0, sizeof(empty));
    activate(&empty);
    return ab_store_clear(&s_store);
}

int adc_cal_clear_part(const char *part) {
    if (!part) return adc_cal_clear();
    adc_cal_t rec = s_active;
    if (strcmp(part, "offset") == 0) {
        rec.current_in_set = 0;
        rec.current_in_offset_uv = 0;
    } else if (strcmp(part, "dac") == 0) {
        rec.dac_set = 0;
        memset(rec.dac, 0, sizeof(rec.dac));
    } else if (strcmp(part, "adc") == 0) {
        rec.adc_gain_set = 0;
        memset(rec.adc_gain, 0, sizeof(rec.adc_gain));
    } else {
        return -2;
    }
    return commit(&rec);
}

int adc_cal_load(void) {
    adc_cal_t rec;
    memset(&rec, 0, sizeof(rec));
    if (ab_store_load(&s_store, &rec, sizeof(rec)) != 0) {
        /* Not version 2: a pod that last saved on firmware 3.4.0 to 3.9.3 has version 1. Take its
           offset; nothing is written until the next save. */
        adc_cal_v1_t v1;
        memset(&rec, 0, sizeof(rec));
        if (ab_store_load(&s_store_v1, &v1, sizeof(v1)) == 0 && v1.current_in_set) {
            rec.current_in_set = 1;
            rec.current_in_offset_uv = v1.current_in_offset_uv;
            printf("[adc-cal] loaded a version-1 record (offset only); the next save writes version 2\n");
        }
    }
    if (record_empty(&rec) || !record_ok(&rec)) {
        if (!record_empty(&rec)) printf("[adc-cal] stored calibration out of range, ignored\n");
        adc_cal_t empty;
        memset(&empty, 0, sizeof(empty));
        activate(&empty);
        return -1;
    }
    activate(&rec);
    return 0;
}
