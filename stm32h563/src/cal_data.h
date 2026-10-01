/*
 * cal_data.h — baked-in calibration, one set per board revision.  Linear: value = a + b*x.
 *
 * Each set was measured on ONE board of that revision against an HP 34401A, and every pod of
 * the revision uses it.  cal_data_select() picks the set at boot from the revision strap
 * (board_rev.h); DAC_CAL / ADC_CAL_* below then read the selected set, so callers do not care
 * which revision they run on.
 *
 * Source of truth / re-runnable: benchpod-firmware/calibration/ ({dac_cal,adc_cal}.json for v2,
 * rev3/ for rev3).  Regenerate there and update the tables in cal_data.c.
 */
#ifndef CAL_DATA_H
#define CAL_DATA_H

typedef struct { float a, b; } cal_lin_t;

typedef struct {
    /* DAC output paths, indexed by dacmux CTRL1 sel: 0=3V3, 1=5V, 2=12V(bipolar).
     * volts = a + b*code ; to command a voltage: code = (volts - a) / b.
     * (The 12V path is differential and also needs CTRL2 -> TO_ADC's VMID.) */
    cal_lin_t dac[3];

    /* ADC calibration — volts = a + b*count, one linear fit per ADC source.  Every
     * source shares the same OPA810 + THS4551 front end, so the coefficients agree
     * to about 0.2%.  The BIPOLAR sources (cal2, ext) WRAP at the 16-bit top for negative
     * inputs — unwrap the count first:  count_uw = count + 65536 when count < 32768.
     *
     * IMPORTANT: the front-SMA input is high-Z (~1 MOhm series via R90) but is NOT
     * attenuated relative to the internal cal node.  The front-end ÷12 is common to
     * every path and is already absorbed into these fits, so do NOT apply an extra
     * divider to `ext` readings.  (An earlier ADC_FRONT_SMA_DIV did, and was wrong —
     * a ±12 V DAC→SMA loopback proved `ext` tracks the cal node 1:1.) */
    cal_lin_t adc_cal1;   /* 5V loop; no wrap over 0..5V */
    cal_lin_t adc_cal2;   /* ±12V diff loop; wraps */
    cal_lin_t adc_ext;    /* front SMA; wraps (ref: DAC 12V) */
} cal_set_t;

/* Pick the set for a board revision (BOARD_REV_V3 -> rev3, anything else -> v2).  Call once
   at boot, right after board_rev_init() and before anything converts a voltage.  Until then
   the v2 set is active. */
void cal_data_select(int board_rev);

/* The active set.  A plain RAM read, safe from any task. */
const cal_set_t *cal_data(void);

#define DAC_CAL       (cal_data()->dac)
#define ADC_CAL_CAL1  (cal_data()->adc_cal1)
#define ADC_CAL_CAL2  (cal_data()->adc_cal2)
#define ADC_CAL_EXT   (cal_data()->adc_ext)

#endif /* CAL_DATA_H */
