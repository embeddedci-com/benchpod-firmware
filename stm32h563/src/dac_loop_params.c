#include "dac_loop_params.h"
#include <string.h>

dac_loop_params_err_t dac_loop_params_validate(dac_loop_params_t *p) {
    if (!p) return DAC_LOOP_PARAMS_ERR_CLAMP_INVERTED;
    if (p->vmin > p->vmax) return DAC_LOOP_PARAMS_ERR_CLAMP_INVERTED;
    if (p->src > DAC_LOOP_SRC_MAX) return DAC_LOOP_PARAMS_ERR_BAD_SOURCE;
    /* A sweep with a zero step never moves: the fabric would hold in_fixed forever, so the
       caller gets a FIXED run under a name that promises a moving input — the same class of
       silent substitution as the inverted clamp.  Refuse and say so. */
    if (p->src == DAC_LOOP_SRC_SWEEP && p->sweep_step == 0u)
        return DAC_LOOP_PARAMS_ERR_SWEEP_STEP_ZERO;
    if (p->k_q15 > DAC_LOOP_K_MAX) p->k_q15 = (uint16_t)DAC_LOOP_K_MAX;
    if (p->k_q15 < DAC_LOOP_K_MIN) p->k_q15 = (uint16_t)DAC_LOOP_K_MIN;
    if (p->tick_div < DAC_LOOP_TICK_MIN) p->tick_div = (uint16_t)DAC_LOOP_TICK_MIN;
    return DAC_LOOP_PARAMS_OK;
}

const char *dac_loop_params_err_str(dac_loop_params_err_t e) {
    switch (e) {
    case DAC_LOOP_PARAMS_OK:                 return "ok";
    case DAC_LOOP_PARAMS_ERR_CLAMP_INVERTED: return "vmin must be <= vmax (inverted output clamp)";
    case DAC_LOOP_PARAMS_ERR_BAD_SOURCE:     return "unknown loop input source (use adc, fixed or sweep)";
    case DAC_LOOP_PARAMS_ERR_SWEEP_STEP_ZERO:return "sweep source needs a non-zero step (0 never advances)";
    case DAC_LOOP_PARAMS_ERR_INPUT_GAIN:     return "input mv_per_unit must be non-zero";
    case DAC_LOOP_PARAMS_ERR_INPUT_RANGE:    return "input range max must be above min";
    case DAC_LOOP_PARAMS_ERR_INPUT_SPAN:     return "input range spans half the ADC or more: the count wraps ambiguously there, so narrow the range or lower the sense gain";
    default:                                 return "invalid control-loop parameters";
    }
}

/* ---- input map derivation ------------------------------------------------------------ */

#define DAC_LOOP_IDX_MAX   (DAC_LOOP_CURVE_POINTS - 1u)   /* 2047 */
#define DAC_LOOP_Q15       32768.0f
#define DAC_LOOP_GAIN_MAX  32767
/* Half the count space. At or beyond this the modular subtract the fabric does stops being
 * a signed distance, so two readings map to one index and the curve is indexed ambiguously. */
#define DAC_LOOP_SPAN_MAX  32768.0f

/* Unwrapped (real-valued) count for a value in the channel's unit. Deliberately NOT
 * wrapped into 16 bits: the span between two of these is what sizes the gain, and wrapping
 * first would make a window straddling the seam measure ~65536 instead of its true width. */
static float dac_loop_count_for(const dac_loop_input_t *in, dac_loop_adc_cal_t cal, float units) {
    float mv = in->mv_at_zero + units * in->mv_per_unit;
    return ((mv / 1000.0f) - cal.a) / cal.b;
}

static float dac_loop_clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

dac_loop_params_err_t dac_loop_inmap_derive(const dac_loop_input_t *in,
                                            dac_loop_adc_cal_t cal,
                                            dac_loop_inmap_t *out) {
    if (!in || !out) return DAC_LOOP_PARAMS_ERR_INPUT_GAIN;
    if (in->mv_per_unit == 0.0f || cal.b == 0.0f) return DAC_LOOP_PARAMS_ERR_INPUT_GAIN;
    if (!(in->range_max > in->range_min))         return DAC_LOOP_PARAMS_ERR_INPUT_RANGE;

    const float c_lo = dac_loop_count_for(in, cal, in->range_min);
    const float c_hi = dac_loop_count_for(in, cal, in->range_max);
    const float span = c_hi - c_lo;                       /* signed: negative when inverting */
    const float aspan = span < 0.0f ? -span : span;
    if (aspan < 1.0f || aspan >= DAC_LOOP_SPAN_MAX) return DAC_LOOP_PARAMS_ERR_INPUT_SPAN;

    /* Gain that lands range_max on the last curve entry. It carries the span's SIGN, which
     * is how an inverting front end stops running the curve backwards. Magnitude is capped
     * at 1.0: one entry per count is the ADC's own limit, and a window narrower than the
     * LUT simply uses fewer entries rather than interpolating counts that do not exist. */
    float gain = (float)DAC_LOOP_IDX_MAX * DAC_LOOP_Q15 / span;
    gain = dac_loop_clampf(gain, -(float)DAC_LOOP_GAIN_MAX, (float)DAC_LOOP_GAIN_MAX);

    out->in_zero = (uint16_t)(((int32_t)(c_lo < 0.0f ? c_lo - 0.5f : c_lo + 0.5f)) & 0xFFFF);
    out->in_gain = (int16_t)(gain < 0.0f ? gain - 0.5f : gain + 0.5f);
    out->map_en  = true;

    /* The index range_max actually reaches, which is where the curve must end. With the
     * gain capped this is |span| for a narrow window and DAC_LOOP_IDX_MAX for a wide one. */
    float idx_hi = span * (float)out->in_gain / DAC_LOOP_Q15;
    out->idx_max = (uint16_t)dac_loop_clampf(idx_hi + 0.5f, 1.0f, (float)DAC_LOOP_IDX_MAX);

    out->trip_en = in->trip_en;
    out->in_trip = DAC_LOOP_IDX_MAX;
    if (in->trip_en) {
        float d = dac_loop_count_for(in, cal, in->trip) - c_lo;
        float t = d * (float)out->in_gain / DAC_LOOP_Q15;
        out->in_trip = (uint16_t)dac_loop_clampf(t + 0.5f, 0.0f, (float)DAC_LOOP_IDX_MAX);
    }
    return DAC_LOOP_PARAMS_OK;
}

size_t dac_loop_upsample_curve_mapped(const uint8_t *in, size_t in_bytes, uint16_t idx_max,
                                      uint8_t *out, size_t out_cap) {
    if (!in || !out || out_cap < DAC_LOOP_CURVE_BYTES) return 0;
    size_t in_pts = in_bytes / 2u;
    if (in_pts == 0 || idx_max == 0) return 0;
    if (idx_max > DAC_LOOP_IDX_MAX) idx_max = DAC_LOOP_IDX_MAX;

    const uint32_t last_src = (uint32_t)(in_pts - 1u);
    for (uint32_t j = 0; j < DAC_LOOP_CURVE_POINTS; j++) {
        uint32_t s;
        uint32_t frac_num = 0, frac_den = 1;
        if (j >= idx_max || last_src == 0) {
            s = last_src;                       /* past the mapped window: hold the last point.
                                                   The fabric saturates over-range inputs to
                                                   entry 2047, so this is what they read. */
        } else {
            /* Position in source points, as a fraction: j*last_src / idx_max. */
            uint32_t num = j * last_src;
            s        = num / idx_max;
            frac_num = num - s * idx_max;
            frac_den = idx_max;
        }
        uint32_t v0 = (uint32_t)in[s * 2u] | ((uint32_t)in[s * 2u + 1u] << 8);
        uint32_t v  = v0;
        if (frac_num != 0 && s < last_src) {
            uint32_t v1 = (uint32_t)in[(s + 1u) * 2u] | ((uint32_t)in[(s + 1u) * 2u + 1u] << 8);
            /* Linear interpolation in 32-bit integers: v0 + (v1-v0)*frac. Signed difference
             * matters — a falling curve is the normal case here. */
            int32_t d = (int32_t)v1 - (int32_t)v0;
            v = (uint32_t)((int32_t)v0 + (int32_t)(((int64_t)d * frac_num + (int64_t)(frac_den / 2)) / frac_den));
        }
        if (v > 65535u) v = 65535u;
        out[j * 2u]      = (uint8_t)(v & 0xFFu);
        out[j * 2u + 1u] = (uint8_t)((v >> 8) & 0xFFu);
    }
    return DAC_LOOP_CURVE_BYTES;
}

size_t dac_loop_upsample_curve(const uint8_t *in, size_t in_bytes,
                               uint8_t *out, size_t out_cap) {
    if (!in || !out || out_cap < DAC_LOOP_CURVE_BYTES) return 0;
    size_t in_pts = in_bytes / 2u;
    if (in_pts == 0) return 0;
    if (in_pts >= DAC_LOOP_CURVE_POINTS) {
        /* Already a full (or oversized) table: take the first LUT-depth points as-is. */
        for (size_t j = 0; j < DAC_LOOP_CURVE_BYTES; j++) out[j] = in[j];
        return DAC_LOOP_CURVE_BYTES;
    }
    for (size_t j = 0; j < DAC_LOOP_CURVE_POINTS; j++) {
        size_t s = (j * in_pts) / DAC_LOOP_CURVE_POINTS;
        if (s >= in_pts) s = in_pts - 1u;           /* defensive; the division cannot exceed */
        out[j * 2u]      = in[s * 2u];
        out[j * 2u + 1u] = in[s * 2u + 1u];
    }
    return DAC_LOOP_CURVE_BYTES;
}

/* ---- JSON `source` field <-> src code -------------------------------------------------- */
bool dac_loop_src_parse(const char *name, uint8_t *out_src) {
    if (!name || !out_src || !name[0]) return false;
    if (strcmp(name, "adc")   == 0) { *out_src = DAC_LOOP_SRC_ADC;   return true; }
    if (strcmp(name, "fixed") == 0) { *out_src = DAC_LOOP_SRC_FIXED; return true; }
    if (strcmp(name, "sweep") == 0) { *out_src = DAC_LOOP_SRC_SWEEP; return true; }
    return false;
}

const char *dac_loop_src_str(uint8_t src) {
    switch (src) {
    case DAC_LOOP_SRC_ADC:   return "adc";
    case DAC_LOOP_SRC_FIXED: return "fixed";
    case DAC_LOOP_SRC_SWEEP: return "sweep";
    default:                 return "unknown";
    }
}
