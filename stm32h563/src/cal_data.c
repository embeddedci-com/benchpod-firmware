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
};

/* rev3: measured 2026-10-01.  DAC: the DMM on each output SMA, unloaded, 4 to 6 codes per
 * path.  Code 0 is left out of the 3V3 and 5V fits: the output stage cannot reach 0 V (it
 * bottoms out near 8 mV and 12 mV), so that point is not on the line.  ADC: 13-point sweeps
 * of the DAC outputs fitted above, through the two internal loopbacks and through a cable
 * from the ±12 V SMA to the ADC SMA. */
static const cal_set_t CAL_V3 = {
    .dac = {
        {  0.001514f,  0.012903f },   /* 3V3 */
        {  0.002428f,  0.019539f },   /* 5V  */
        { -12.087271f, 0.094405f },   /* 12V */
    },
    .adc_cal1 = { 65.720665f, -0.001002852f },
    .adc_cal2 = { 65.824094f, -0.001004449f },
    .adc_ext  = { 65.716013f, -0.001002767f },
};

static const cal_set_t *s_active = &CAL_V2;

void cal_data_select(int board_rev) {
    s_active = (board_rev == BOARD_REV_V3) ? &CAL_V3 : &CAL_V2;
}

const cal_set_t *cal_data(void) {
    return s_active;
}
