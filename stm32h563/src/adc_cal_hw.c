/*
 * adc_cal_hw.c — the hardware half of adc_cal.h: the `current_in` offset measurement and the ADC
 * gain measurement through a loopback. Kept out of adc_cal.c so the checks and the store stay
 * host-testable.
 */
#include "adc_cal.h"
#include "cal_data.h"
#include "i2c_bus.h"         /* analog_path_set */
#include "pico_compat.h"     /* sleep_ms */
#include "signal_engine.h"   /* adc_capture_psram_start / _poll */

static uint16_t s_buf[ADC_CAL_SAMPLES];

/* The bursts into s_buf: the same burst as adc_read (16 samples at the maximum rate), many times
   over (see adc_cal.h for why one long slow capture reads differently). 0 or -2. */
static int take_bursts(void) {
    for (uint32_t i = 0; i < ADC_CAL_BURSTS; i++) {
        uint16_t *b = &s_buf[i * ADC_CAL_BURST];
        if (adc_capture_psram_start(ADC_CAL_BURST, 0.0f) != 0) return -2;
        int r;
        while ((r = adc_capture_psram_poll(b, ADC_CAL_BURST)) == 0) { }   /* 40 us; has its own deadline */
        if (r < 0) return -2;
        sleep_ms(ADC_CAL_GAP_MS);
    }
    return 0;
}

int adc_cal_measure_current_in(adc_reading_t *out) {
    if (!out) return -1;
    if (analog_path_set(ANALOG_PATH_CURRENT_IN) != 0) return -1;
    sleep_ms(20);    /* let the G6K relays (~4ms) + front-end RC settle */
    if (take_bursts() != 0) return -2;
    /* The shared cal1 fit, without this pod's offset: this reading IS the offset. */
    *out = adc_scale_burst(s_buf, ADC_CAL_SAMPLES, ADC_CAL_CAL1.a, ADC_CAL_CAL1.b);
    return 0;
}

/* One loopback point: drive `volts` on `path` through `route` (in the low range when low), let it
   settle and take the bursts.  *rd is the reading with fit `fit`; the outputs stay as they are. */
static int loop_point(int src, bool low, int path, float volts, cal_lin_t fit, uint16_t *code_out,
                      adc_reading_t *rd) {
    /* cal1 and cal2 are internal loopbacks; ext reads the front SMA, cabled to the +-12 V output
       (the ext route leaves the DAC mux as it is, so route the output first). */
    analog_path_t route = src == ADC_CAL_SRC_CAL1 ? ANALOG_PATH_CAL1
                        : src == ADC_CAL_SRC_CAL2 ? ANALOG_PATH_CAL2 : ANALOG_PATH_DAC_12V;
    uint16_t code = dac_code16_for_volts(path, volts);   /* the DMM line */
    if (analog_path_set(route) != 0 ||
        (src == ADC_CAL_SRC_EXT && analog_path_set(ANALOG_PATH_ADC_EXT) != 0) ||
        adc_range_set(low ? ADC_RANGE_LOW : ADC_RANGE_NORMAL) != 0 ||
        dac_set_constant16(code, 240) != 0) return -1;
    sleep_ms(50);    /* relays, the DAC output stage and the front-end RC */
    if (take_bursts() != 0) return -2;
    *rd = adc_scale_burst(s_buf, ADC_CAL_SAMPLES, fit.a, fit.b);
    *code_out = code;
    return 0;
}

int adc_cal_measure_gain(int src, bool low, adc_cal_gain_meas_t *m, uint32_t *span_out) {
    if (!m || src < 0 || src >= ADC_CAL_SRCS) return -1;
    if (low && (src != ADC_CAL_SRC_CAL1 || !adc_range_supported())) return -1;
    int path = adc_cal_gain_path(src);
    /* Any fit only to check the burst settled: what is kept is its mean count. */
    cal_lin_t base = ADC_CAL_CAL1;
    int rc = 0;
    for (int i = 0; i < 2 && rc == 0; i++) {
        uint16_t code = 0;
        adc_reading_t rd;
        rc = loop_point(src, low, path, adc_cal_gain_point(src, i), base, &code, &rd);
        if (rc != 0) break;
        if (!rd.valid) {
            if (span_out) *span_out = rd.span;
            rc = -3;
            break;
        }
        m->v[i] = dac_volts_for_code16(path, code);
        m->count[i] = rd.count_uw;
    }
    analog_path_set(ANALOG_PATH_OFF);   /* nothing left driving the outputs; the range back to normal */
    return rc;
}

int adc_range_selftest(adc_range_test_pt_t pts[ADC_RANGE_TEST_POINTS], uint32_t *span_out) {
    if (!pts || !adc_range_supported()) return -1;
    const int path = adc_cal_gain_path(ADC_CAL_SRC_CAL1);   /* the 5 V output */
    int rc = 0;
    for (int i = 0; i < ADC_RANGE_TEST_POINTS && rc == 0; i++) {
        float got[2] = { 0.0f, 0.0f };
        uint16_t code = 0;
        for (int r = 0; r < 2 && rc == 0; r++) {
            adc_reading_t rd;
            rc = loop_point(ADC_CAL_SRC_CAL1, r == 1, path, ADC_RANGE_TEST_V[i],
                            adc_cal_range_fit(ADC_CAL_SRC_CAL1, r), &code, &rd);
            if (rc != 0) break;
            if (!rd.valid) {
                if (span_out) *span_out = rd.span;
                rc = -3;
                break;
            }
            got[r] = rd.volts;
        }
        if (rc != 0) break;
        pts[i].set_v = dac_volts_for_code16(path, code);
        pts[i].normal_v = got[0];
        pts[i].low_v = got[1];
        pts[i].ok = adc_range_test_point_ok(pts[i].set_v, got[0], got[1]);
    }
    analog_path_set(ANALOG_PATH_OFF);
    return rc;
}
