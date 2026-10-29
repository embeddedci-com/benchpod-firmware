/*
 * cal_data.c — the calibration tables behind cal_data.h.  Pure: no HAL.
 */
#include "cal_data.h"
#include "board_rev.h"

/* v2: measured 2026-07-02 through the internal cal path. */
static const cal_set_t CAL_V2 = {
    .dac = {
        {  0.004235f,  0.012914f },   /* 3V3 : 0 .. 3.30 V */
        {  0.007490f,  0.019488f },   /* 5V  : 0 .. 4.98 V */
        { -12.043680f, 0.094352f },   /* 12V : -12 .. +12 V (0 V at code 128) */
    },
    .adc_cal1 = { 65.832398f, -0.001004471f },
    .adc_cal2 = { 65.920100f, -0.001005709f },
    .adc_ext  = { 65.788900f, -0.001003762f },
    /* Not measured on a v2 board: the nominal transfer function (current_out.h),
     * 100 * 4.096 V / 102 k and 100 * 4.096 V / 25.5 k / 65536. */
    .current_out = { 4015.686f, 0.24509804f },
    .adc_cal1_v1 = { 65.832398f, -0.001004471f },   /* = adc_cal1 above; frozen (cal_data.h) */
};

/* rev3: measured 2026-10-12 on pod .220 against an HP 34401A, in 16-bit DAC codes
 * (benchpod-firmware/calibration/scripts/dac16_sweep.py and adc16_sweep.py).  DAC: the DMM on
 * each output SMA, unloaded, 40 codes per path; codes below 1000 (the output floor, about 8 mV
 * on 3V3 and 12 mV on 5V) are left out.  What remains is the DAC8551's own INL: +-0.2 mV on
 * 3V3, +-0.3 mV on 5V, +-1.6 mV on 12V.  ADC: 25-point sweeps against the DMM reading of the
 * same DAC code, through the two internal loopbacks and through a cable from the +-12 V SMA to
 * the ADC SMA; within about one count.  4-20 mA output: 25 currents from 4.1 to 20 mA, the
 * DMM in series with a floating 24 V supply and 324 ohm, within 1 uA; code 0 sits on the DAC
 * floor at 4055 uA, 30 uA above the line, and is left out.  The rev3 set of 2026-10-01 (8-bit
 * codes, 4 to 6 per path, and a 4-20 mA line through code 0) was up to 27 uA off at 4 mA. */
static const cal_set_t CAL_V3 = {
    .dac = {
        {  0.0016304f,  0.01290215f },   /* 3V3 */
        {  0.0025176f,  0.01953943f },   /* 5V  */
        { -12.087242f,  0.09440379f },   /* 12V */
    },
    .adc_cal1 = { 65.712125f, -0.0010027157f },
    .adc_cal2 = { 65.812962f, -0.0010042767f },
    .adc_ext  = { 65.708158f, -0.0010026443f },
    .current_out = { 4025.338f, 0.24518720f },
    .current_out_floor_ua = 4055.0f,
    .adc_cal1_v1 = { 65.712125f, -0.0010027157f },   /* = adc_cal1 above since 3.8.1; frozen (cal_data.h) */
};

/* Analog v3r2 (DAC8554 quad DAC, one channel per output): NOT MEASURED YET, a copy of CAL_V3.
 * The output stages, the 4.096 V reference and the ADC front end are v3's, so the v3 lines are
 * the best guess until a v3r2 board is swept the same way (calibration/rev3/ scripts, one DAC
 * channel per path).  Expect the DAC lines to move by the DAC8554's own offset and gain error
 * (+-(a few) mV, about 0.1 %), and the 4-20 mA floor with channel D's zero-code output. */
static const cal_set_t CAL_V3R2 = {
    .dac = {
        {  0.0016304f,  0.01290215f },   /* 3V3 (channel A) */
        {  0.0025176f,  0.01953943f },   /* 5V  (channel B) */
        { -12.087242f,  0.09440379f },   /* 12V (channel C) */
    },
    .adc_cal1 = { 65.712125f, -0.0010027157f },
    .adc_cal2 = { 65.812962f, -0.0010042767f },
    .adc_ext  = { 65.708158f, -0.0010026443f },
    .current_out = { 4025.338f, 0.24518720f },    /* channel D */
    .current_out_floor_ua = 4055.0f,
    .adc_cal1_v1 = { 65.712125f, -0.0010027157f },   /* = CAL_V3's: no v3r2 pod ran 3.4.0..3.9.3 */
};

static const cal_set_t *s_active = &CAL_V2;

float cal_adc_low_ratio(void) {
    return s_active->adc_low_ratio > 0.0f ? s_active->adc_low_ratio : ADC_LOW_RANGE_RATIO_NOMINAL;
}

/* The DAC lines in use (cal_dac_active): the one s_dac points to is read, the other is built. */
static cal_lin_t s_dac_tbl[2][3];
static const cal_lin_t *volatile s_dac = CAL_V2.dac;

void cal_data_set_pod_dac(const cal_lin_t pod[3], uint8_t mask) {
    cal_lin_t *next = (s_dac == s_dac_tbl[0]) ? s_dac_tbl[1] : s_dac_tbl[0];
    for (int i = 0; i < 3; i++)
        next[i] = (pod && (mask & (1u << i))) ? pod[i] : s_active->dac[i];
    s_dac = next;
}

void cal_data_select(int board_rev) {
    s_active = (board_rev == BOARD_REV_V3R2) ? &CAL_V3R2
             : (board_rev == BOARD_REV_V3)   ? &CAL_V3 : &CAL_V2;
    s_dac = s_active->dac;   /* the compiled lines until adc_cal_load puts this pod's in */
}

const cal_set_t *cal_data(void) {
    return s_active;
}

const cal_lin_t *cal_dac_active(void) {
    return s_dac;
}

uint16_t dac_code16_for_volts(int path, float volts) {
    const cal_lin_t c = DAC_CAL[path];
    double code = ((double)volts - c.a) / c.b * 256.0;
    if (!(code > 0.0)) return 0;              /* also catches NaN */
    if (code >= 65535.0) return 65535;
    return (uint16_t)(code + 0.5);
}

float dac_volts_for_code16(int path, uint16_t code) {
    const cal_lin_t c = DAC_CAL[path];
    return (float)(c.a + (double)c.b * code / 256.0);
}
