/*
 * dac_limits.c — DAC output limits for an external output stage (see dac_limits.h).
 *
 * Pure: no HAL. The checks read JSON with bp_json and convert codes with the baked-in DAC_CAL,
 * and the store is config_store.c, so the whole file runs in the host tests. Applying the park
 * level (route + dac_set_constant) is the caller's job (command_handler.c, main.c).
 */
#include "dac_limits.h"
#include "bp_json.h"
#include "cal_data.h"
#include "config_store.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

_Static_assert(sizeof(dac_limits_t) <= AB_STORE_MAX_PAYLOAD, "dac_limits_t must fit in an A/B record");

static const ab_store_t s_store = {
    .tag      = "dac-limits",
    .slot_off = { DAC_LIMITS_SLOT_A_OFFSET, DAC_LIMITS_SLOT_B_OFFSET },
    .magic    = DAC_LIMITS_RECORD_MAGIC,
    .version  = DAC_LIMITS_VERSION,
};

static dac_limits_t s_active;

/* Slack for comparing against the window: the page and server convert with the nominal path
   range while this uses the calibration, and dac_out rounds to an 8-bit code. */
#define DAC_LIMITS_TOL_V  0.005f

/* No %f anywhere below: newlib-nano's printf has none, so volts are printed as whole mV.

   Refusal text. Commands are dispatched one at a time and the caller copies the text into its
   reply straight away, so one buffer is enough. */
static char s_msg[240];

static const char *const s_path_names[3] = { "3v3", "5v", "12v" };

const dac_limits_t *dac_limits_get(void) { return &s_active; }

void dac_limits_set_active(const dac_limits_t *in) {
    if (in) s_active = *in;
    else memset(&s_active, 0, sizeof(s_active));
}

const char *dac_limits_path_name(int path) {
    return (path >= 0 && path < 3) ? s_path_names[path] : "?";
}

/* Firmware aliases (i2c_bus.c s_path_tbl) onto the canonical names used here. */
static const char *canon(const char *name) {
    if (!name) return "";
    if (!strcmp(name, "dac_3v3")) return "3v3";
    if (!strcmp(name, "dac_5v"))  return "5v";
    if (!strcmp(name, "dac_12v")) return "12v";
    if (!strcmp(name, "adc_ext") || !strcmp(name, "sma")) return "ext";
    return name;
}

int dac_limits_path_index(const char *name) {
    const char *c = canon(name);
    for (int i = 0; i < 3; i++) if (!strcmp(c, s_path_names[i])) return i;
    return -1;
}

/* Output range of a path from its calibration, volts (code 0 .. code 255). */
static void path_range(int path, float *lo, float *hi) {
    float a = DAC_CAL[path].a, b = DAC_CAL[path].b;
    float v0 = a, v1 = a + b * 255.0f;
    *lo = v0 < v1 ? v0 : v1;
    *hi = v0 < v1 ? v1 : v0;
}

static float min_v(void) { return (float)s_active.min_mv / 1000.0f; }
static float max_v(void) { return (float)s_active.max_mv / 1000.0f; }

float dac_limits_park_volts(void) {
    return s_active.inverted ? max_v() : min_v();
}

uint8_t dac_limits_park_code(void) {
    float a = DAC_CAL[s_active.path].a, b = DAC_CAL[s_active.path].b;
    float c = (dac_limits_park_volts() - a) / b;
    /* Round INTO the window: down from the top when inverted, up from the bottom otherwise. */
    long code = s_active.inverted ? (long)floorf(c) : (long)ceilf(c);
    if (code < 0) code = 0;
    if (code > 255) code = 255;
    return (uint8_t)code;
}

const char *dac_limits_set(const dac_limits_t *in) {
    if (!in) return "missing limits";
    if (in->path > 2) return "path must be 3v3, 5v or 12v";
    if (in->min_mv >= in->max_mv) return "min_v must be below max_v";
    float lo, hi;
    path_range(in->path, &lo, &hi);
    if ((float)in->min_mv / 1000.0f < lo - 0.05f || (float)in->max_mv / 1000.0f > hi + 0.05f) {
        snprintf(s_msg, sizeof(s_msg), "limits must sit inside the %s path's range (%ld to %ld mV)",
                 s_path_names[in->path], lroundf(lo * 1000.0f), lroundf(hi * 1000.0f));
        return s_msg;
    }
    dac_limits_t rec = *in;
    rec.enabled = 1;
    rec.reserved = 0;
    if (ab_store_save(&s_store, &rec, sizeof(rec)) != 0) return "could not save the limits to flash";
    s_active = rec;
    return NULL;
}

int dac_limits_clear(void) {
    memset(&s_active, 0, sizeof(s_active));
    return ab_store_clear(&s_store);
}

int dac_limits_load(void) {
    dac_limits_t rec;
    if (ab_store_load(&s_store, &rec, sizeof(rec)) != 0 || !rec.enabled || rec.path > 2 ||
        rec.min_mv >= rec.max_mv) {
        memset(&s_active, 0, sizeof(s_active));
        return -1;
    }
    s_active = rec;
    return 0;
}

/* ---- Checks ----------------------------------------------------------------------------- */

static const char *describe(void) {
    static char d[96];
    snprintf(d, sizeof(d), "%sDAC limits are set on the %s output (%ld to %ld mV)",
             s_active.inverted ? "inverted " : "", s_path_names[s_active.path],
             (long)s_active.min_mv, (long)s_active.max_mv);
    return d;
}

const char *dac_limits_check_raw(const char *what) {
    if (!s_active.enabled) return NULL;
    snprintf(s_msg, sizeof(s_msg),
             "refused: %s, and %s writes raw DAC codes that can't be checked against them. "
             "Use dac_out or the control loop, or clear them with dac_limits enabled:false",
             describe(), what);
    return s_msg;
}

const char *dac_limits_check_route(const char *path) {
    if (!s_active.enabled || !s_active.inverted) return NULL;   /* a normal stage's 0 V is its low end */
    const char *p = canon(path);
    const char *own = s_path_names[s_active.path];
    if (!strcmp(p, own) || !strcmp(p, "ext") || !strcmp(p, "amp")) return NULL;
    if (!strcmp(p, "cal1") && s_active.path == 1) return NULL;      /* cal1 keeps the 5V mux */
    snprintf(s_msg, sizeof(s_msg),
             "refused: %s. Routing to %s disconnects it and leaves the module input near 0 V, "
             "which is full output. dac_stop parks it at %ld mV instead",
             describe(), p, lroundf(dac_limits_park_volts() * 1000.0f));
    return s_msg;
}

const char *dac_limits_check_volts(const char *path, float volts) {
    if (!s_active.enabled) return NULL;
    if (dac_limits_path_index(path) != (int)s_active.path) return NULL;   /* the route check covers it */
    if (volts < min_v() - DAC_LIMITS_TOL_V || volts > max_v() + DAC_LIMITS_TOL_V) {
        snprintf(s_msg, sizeof(s_msg), "refused: %s; %ld mV is outside them", describe(), lroundf(volts * 1000.0f));
        return s_msg;
    }
    return NULL;
}

/* A number field; false when absent or not a number. */
static bool num(const char *json, const char *key, double *out) {
    char buf[24];
    if (!bp_json_get(json, key, buf, sizeof(buf)) || !buf[0]) return false;
    char *end = NULL;
    double v = strtod(buf, &end);
    if (end == buf) return false;
    *out = v;
    return true;
}

static const char *check_loop(const char *json) {
    if (!s_active.enabled) return NULL;
    char trip[24];
    if (s_active.inverted && bp_json_get(json, "in_trip", trip, sizeof(trip)) && trip[0]) {
        snprintf(s_msg, sizeof(s_msg),
                 "refused: %s. in_trip drives the DAC to vmin, the HIGHEST output on an inverted stage",
                 describe());
        return s_msg;
    }
    double vmin, vmax;
    float a = DAC_CAL[s_active.path].a, b = DAC_CAL[s_active.path].b;
    /* 16-bit loop code -> volts: the DAC8551 takes the 8-bit calibration's code in its high byte. */
    if (!num(json, "vmin", &vmin) || !num(json, "vmax", &vmax) ||
        a + b * (float)(vmin / 256.0) < min_v() - DAC_LIMITS_TOL_V ||
        a + b * (float)(vmax / 256.0) > max_v() + DAC_LIMITS_TOL_V) {
        long lo = lroundf((min_v() - a) / b * 256.0f), hi = (long)floorf((max_v() - a) / b * 256.0f);
        if (lo < 0) lo = 0;
        if (hi > 65535) hi = 65535;
        snprintf(s_msg, sizeof(s_msg),
                 "refused: %s. The loop clamp must stay inside them: vmin at least %ld and vmax at most %ld",
                 describe(), lo, hi);
        return s_msg;
    }
    return NULL;
}

const char *dac_limits_check_command(const char *cmd, const char *json) {
    if (!s_active.enabled || !cmd) return NULL;
    static const char *const raw[] = {
        "generate", "dac_set", "load", "load_bin", "replay", "measure", "dac_mux", "cal_switch",
    };
    for (size_t i = 0; i < sizeof(raw) / sizeof(raw[0]); i++)
        if (!strcmp(cmd, raw[i])) return dac_limits_check_raw(cmd);

    char path[16] = {0};
    if (!strcmp(cmd, "analog_path")) {
        bp_json_get(json, "path", path, sizeof(path));
        return dac_limits_check_route(path);
    }
    if (!strcmp(cmd, "adc_read")) {
        if (!bp_json_get(json, "source", path, sizeof(path)) || !path[0]) strcpy(path, "ext");
        return dac_limits_check_route(path);
    }
    if (!strcmp(cmd, "dac_out")) {
        bp_json_get(json, "path", path, sizeof(path));
        const char *why = dac_limits_check_route(path);
        double v;
        if (!why && num(json, "volts", &v)) why = dac_limits_check_volts(path, (float)v);
        return why;
    }
    if (!strcmp(cmd, "dac_control_loop")) return check_loop(json);
    return NULL;
}
