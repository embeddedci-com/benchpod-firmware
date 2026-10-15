/*
 * wave16.c — the generator's waveforms at 16 bits (see wave16.h).
 *
 * Pure: no HAL, so the whole file runs in the host tests.
 */
#include "wave16.h"
#include "cal_data.h"
#include "current_out.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/* Refusal text. Commands are dispatched one at a time and the caller copies the text into its
   reply before the next one, so one static buffer is enough. Integers only in the text:
   newlib-nano's printf has no %f. */
static char s_msg[160];

static const char *const PATH_NAMES[3] = { "3v3", "5v", "12v" };

int wave16_shape_from_name(const char *name, wave16_shape_t *shape) {
    if (!name || !shape) return -1;
    if      (strcmp(name, "sine")     == 0) *shape = WAVE16_SINE;
    else if (strcmp(name, "square")   == 0) *shape = WAVE16_SQUARE;
    else if (strcmp(name, "sawtooth") == 0) *shape = WAVE16_SAWTOOTH;
    else return -1;
    return 0;
}

static uint16_t clamp16(double v) {
    if (!(v > 0.0)) return 0;               /* also catches NaN */
    if (v >= 65535.0) return 65535;
    return (uint16_t)(v + 0.5);
}

void wave16_fill(wave16_shape_t shape, uint32_t n, double amp, double mid, uint16_t *out) {
    if (!out || n == 0) return;
    for (uint32_t i = 0; i < n; i++) {
        double v;
        switch (shape) {
        case WAVE16_SQUARE:
            v = (i < n / 2) ? mid + amp : mid - amp;
            break;
        case WAVE16_SAWTOOTH:
            v = mid - amp + 2.0 * amp * (double)i / (double)n;
            break;
        case WAVE16_SINE:
        default:
            v = mid + amp * sin(2.0 * M_PI * (double)i / (double)n);
            break;
        }
        out[i] = clamp16(v);
    }
}

/* Millivolts, rounded, for the refusal text. */
static long mv(double v) { return lround(v * 1000.0); }

const char *wave16_codes_for_volts(int path, double amplitude_v, double offset_v,
                                   double *amp, double *mid) {
    if (path < 0 || path > 2) {
        snprintf(s_msg, sizeof(s_msg), "path must be 3v3|5v|12v");
        return s_msg;
    }
    if (!(amplitude_v > 0.0)) {
        snprintf(s_msg, sizeof(s_msg), "amplitude_v must be above 0 V");
        return s_msg;
    }
    /* The fit is on 8-bit codes driving the DAC's high byte: code16 = (volts - a) / b * 256. */
    const double a = DAC_CAL[path].a, b = DAC_CAL[path].b;
    const double lo_v = a, hi_v = a + b * 65535.0 / 256.0;
    const double min_v = lo_v < hi_v ? lo_v : hi_v, max_v = lo_v < hi_v ? hi_v : lo_v;
    /* The calibrated ends sit a few mV inside the nominal range (3v3: about 0.002 to 3.305 V),
       so "0 to 3.3 V" must work: a peak up to 5% of the range past an end is clamped to it
       (wave16_fill clamps each sample), as the 8-bit form always did. Further out is refused. */
    const double slack = (max_v - min_v) * 0.05;
    const double top = offset_v + amplitude_v, bottom = offset_v - amplitude_v;
    if (bottom < min_v - slack || top > max_v + slack) {
        snprintf(s_msg, sizeof(s_msg),
                 "the peaks %ld to %ld mV are outside the %s output's %ld to %ld mV",
                 mv(bottom), mv(top), PATH_NAMES[path], mv(min_v), mv(max_v));
        return s_msg;
    }
    const double c_top = (top - a) / b * 256.0, c_bottom = (bottom - a) / b * 256.0;
    if (amp) *amp = (c_top - c_bottom) / 2.0;   /* negative on a path that falls with the code */
    if (mid) *mid = (c_top + c_bottom) / 2.0;
    return NULL;
}

const char *wave16_codes_for_ua(double amplitude_ua, double offset_ua, double *amp, double *mid) {
    if (!(amplitude_ua > 0.0)) {
        snprintf(s_msg, sizeof(s_msg), "amplitude_ua must be above 0 uA");
        return s_msg;
    }
    const double top = offset_ua + amplitude_ua, bottom = offset_ua - amplitude_ua;
    /* Like current_out: a trough from 4000 uA up to the live zero is played as code 0. */
    if (bottom < (double)CURRENT_OUT_REQUEST_MIN_UA - 0.5 || top > (double)current_out_max_ua() + 0.5) {
        snprintf(s_msg, sizeof(s_msg),
                 "the peaks %ld to %ld uA are outside the 4-20 mA output's %ld to %ld uA",
                 lround(bottom), lround(top), current_out_min_ua(), current_out_max_ua());
        return s_msg;
    }
    const double c_top = current_out_code_exact(top), c_bottom = current_out_code_exact(bottom);
    if (amp) *amp = (c_top - c_bottom) / 2.0;
    if (mid) *mid = (c_top + c_bottom) / 2.0;
    return NULL;
}
