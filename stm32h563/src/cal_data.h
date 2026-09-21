/*
 * cal_data.h — baked-in per-board calibration (measured 2026-07-02 against an
 * HP 34401A DMM through the internal cal path).  Linear: value = a + b*x.
 * Source of truth / re-runnable: bench-pod-firmware/calibration/{dac_cal,adc_cal}.json
 * (regenerate with calibration/scripts/dac_cal_direct.py and update these consts).
 */
#ifndef CAL_DATA_H
#define CAL_DATA_H

typedef struct { float a, b; } cal_lin_t;

/* DAC output paths, indexed by dacmux CTRL1 sel: 0=3V3, 1=5V, 2=12V(bipolar).
 * volts = a + b*code ; to command a voltage: code = (volts - a) / b.
 * (The 12V path is differential and also needs CTRL2 -> TO_ADC's VMID.) */
static const cal_lin_t DAC_CAL[3] = {
    {  0.004235f,  0.012914f },   /* 3V3 : 0 .. 3.30 V */
    {  0.007490f,  0.019488f },   /* 5V  : 0 .. 4.98 V */
    { -12.043680f, 0.094352f },   /* 12V : -12 .. +12 V (0 V at code 128) */
};

/* ADC calibration — volts = a + b*count, one linear fit per ADC source.  Every
 * source shares the same OPA810 + THS4551 front end, so the coefficients agree
 * to <0.1%.  The BIPOLAR sources (cal2, ext) WRAP at the 16-bit top for negative
 * inputs — unwrap the count first:  count_uw = count + 65536 when count < 32768.
 *
 * IMPORTANT: the front-SMA input is high-Z (~1 MOhm series via R90) but is NOT
 * attenuated relative to the internal cal node.  The front-end ÷12 is common to
 * every path and is already absorbed into these fits, so do NOT apply an extra
 * divider to `ext` readings.  (An earlier ADC_FRONT_SMA_DIV did, and was wrong —
 * a ±12 V DAC→SMA loopback proved `ext` tracks the cal node 1:1.) */
static const cal_lin_t ADC_CAL_CAL1 = { 65.832398f, -0.001004471f };  /* 5V loop; no wrap over 0..5V */
static const cal_lin_t ADC_CAL_CAL2 = { 65.920100f, -0.001005709f };  /* ±12V diff loop; wraps */
static const cal_lin_t ADC_CAL_EXT  = { 65.788900f, -0.001003762f };  /* front SMA; wraps (ref: DAC 12V) */

#endif /* CAL_DATA_H */
