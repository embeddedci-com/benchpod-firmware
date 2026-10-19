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

int adc_cal_measure_gain(int src, adc_cal_gain_meas_t *m, uint32_t *span_out) {
    if (!m || src < 0 || src >= ADC_CAL_SRCS) return -1;
    int path = adc_cal_gain_path(src);
    /* cal1 and cal2 are internal loopbacks; ext reads the front SMA, cabled to the +-12 V output
       (the ext route leaves the DAC mux as it is, so route the output first). */
    analog_path_t route = src == ADC_CAL_SRC_CAL1 ? ANALOG_PATH_CAL1
                        : src == ADC_CAL_SRC_CAL2 ? ANALOG_PATH_CAL2 : ANALOG_PATH_DAC_12V;
    /* The source's compiled fit with this pod's offset, without a gain: the gain is what this
       measures. */
    cal_lin_t base = adc_cal_fit(src == ADC_CAL_SRC_CAL2 ? ADC_CAL_CAL2
                               : src == ADC_CAL_SRC_EXT ? ADC_CAL_EXT : ADC_CAL_CAL1);
    int rc = 0;
    for (int i = 0; i < 2 && rc == 0; i++) {
        uint16_t code = dac_code16_for_volts(path, adc_cal_gain_point(src, i));   /* the DMM line */
        if (analog_path_set(route) != 0 ||
            (src == ADC_CAL_SRC_EXT && analog_path_set(ANALOG_PATH_ADC_EXT) != 0) ||
            dac_set_constant16(code, 240) != 0) { rc = -1; break; }
        sleep_ms(50);    /* relays, the DAC output stage and the front-end RC */
        if (take_bursts() != 0) { rc = -2; break; }
        adc_reading_t rd = adc_scale_burst(s_buf, ADC_CAL_SAMPLES, base.a, base.b);
        if (!rd.valid) {
            if (span_out) *span_out = rd.span;
            rc = -3;
            break;
        }
        m->v[i] = dac_volts_for_code16(path, code);
        m->r[i] = rd.volts;
    }
    analog_path_set(ANALOG_PATH_OFF);   /* nothing left driving the outputs */
    return rc;
}
