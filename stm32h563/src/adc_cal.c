/*
 * adc_cal.c — per-pod ADC calibration (see adc_cal.h).
 *
 * Pure: no HAL. The store is config_store.c, so the whole file runs in the host tests. Taking
 * the measurement (route + capture) is adc_cal_hw.c.
 */
#include "adc_cal.h"
#include "config_store.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

_Static_assert(sizeof(adc_cal_t) <= AB_STORE_MAX_PAYLOAD, "adc_cal_t must fit in an A/B record");

static const ab_store_t s_store = {
    .tag      = "adc-cal",
    .slot_off = { ADC_CAL_SLOT_A_OFFSET, ADC_CAL_SLOT_B_OFFSET },
    .magic    = ADC_CAL_RECORD_MAGIC,
    .version  = ADC_CAL_VERSION,
};

static adc_cal_t s_active;

/* Refusal text. No %f: newlib-nano's printf has none, so volts are printed as whole mV.
   Commands are dispatched one at a time and the caller copies the text into its reply
   straight away, so one buffer is enough. */
static char s_msg[160];

static bool in_range(int32_t uv) { return uv >= -ADC_CAL_MAX_OFFSET_UV && uv <= ADC_CAL_MAX_OFFSET_UV; }

bool    adc_cal_amp_is_set(void)    { return s_active.amp_set != 0; }
int32_t adc_cal_amp_offset_uv(void) { return s_active.amp_set ? s_active.amp_offset_uv : 0; }
int32_t adc_cal_amp_offset_mv(void) { return (int32_t)lroundf((float)adc_cal_amp_offset_uv() / 1000.0f); }

cal_lin_t adc_cal_amp_fit(void) {
    cal_lin_t c = ADC_CAL_CAL1;
    c.a -= (float)adc_cal_amp_offset_uv() / 1000000.0f;
    return c;
}

const char *adc_cal_amp_store(const adc_reading_t *rd) {
    if (!rd) return "calibrate: no reading";
    if (!rd->valid) {
        snprintf(s_msg, sizeof(s_msg),
                 "calibrate: input not settled on amp (%u counts pk-pk, limit %u). "
                 "Disconnect J8 and try again",
                 (unsigned)rd->span, (unsigned)ADC_BURST_MAX_SPAN);
        return s_msg;
    }
    long uv = lroundf(rd->volts * 1000000.0f);
    if (!in_range((int32_t)uv)) {
        snprintf(s_msg, sizeof(s_msg),
                 "calibrate: amp reads %ld mV, it must be within +/-%d mV of zero. "
                 "Something is driving J8. Disconnect it and try again",
                 lroundf(rd->volts * 1000.0f), ADC_CAL_MAX_OFFSET_UV / 1000);
        return s_msg;
    }
    adc_cal_t rec = s_active;   /* keep whatever else the record holds */
    rec.amp_set = 1;
    rec.amp_offset_uv = (int32_t)uv;
    if (ab_store_save(&s_store, &rec, sizeof(rec)) != 0) return "calibrate: could not save to flash";
    s_active = rec;
    return NULL;
}

int adc_cal_clear(void) {
    memset(&s_active, 0, sizeof(s_active));
    return ab_store_clear(&s_store);
}

int adc_cal_load(void) {
    adc_cal_t rec;
    if (ab_store_load(&s_store, &rec, sizeof(rec)) != 0 || !rec.amp_set || !in_range(rec.amp_offset_uv)) {
        memset(&s_active, 0, sizeof(s_active));
        return -1;
    }
    s_active = rec;
    return 0;
}
