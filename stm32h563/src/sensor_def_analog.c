/* ============================================================================
 * sensor_def_analog.c — analog sensors: one value, clamped to the range, out
 * of the pod's DAC (voltage) or 4-20 mA output (current, in mA).  With
 * excitation: adc the value may use `input`, the voltage the pod measures.
 * ========================================================================== */
#include "sensor_def_int.h"

static bool is_analog(const sd_inst_t *in) { return in && in->prog && sd_h(in)->kind == SD_KIND_ANALOG; }

int sd_analog_info(const sd_inst_t *in, sd_analog_info_t *out) {
    if (!is_analog(in) || !out) return SD_E_ARG;
    const sdp_hdr_t *h = sd_h(in);
    out->current = h->current;
    out->excitation = h->excitation;
    out->min = h->range_min;
    out->max = h->range_max;
    return SD_OK;
}

int sd_analog_eval(sd_inst_t *in, double input, double *out) {
    if (!is_analog(in) || !out) return SD_E_ARG;
    const sdp_hdr_t *h = sd_h(in);
    in->err.code = 0;
    sd_vm_begin(in);
    in->regsrc = NULL;
    in->vars[SDV_BUILTIN + SDB_INPUT] = sdv_num(input == input ? input : 0.0);
    double v;
    int rc = SD_OK;
    if (in->frozen && in->started) {
        v = sdv_d(in->vars[SDV_LAST_OUT]);   /* a stuck sensor keeps its last output */
    } else {
        rc = sd_prepare(in);
        if (!rc) rc = sd_vm_num(in, h->value, &v);
        if (rc) return rc;
    }
    if (!(v == v)) v = h->range_min;
    if (v < h->range_min) v = h->range_min;
    if (v > h->range_max) v = h->range_max;
    if (!(in->frozen && in->started)) in->vars[SDV_LAST_OUT] = sdv_num(v);
    if (in->force_out == 1) v = in->force_has_val ? in->force_val : h->range_min;
    else if (in->force_out == 2) v = in->force_has_val ? in->force_val : h->range_max;
    *out = v;
    return SD_OK;
}
