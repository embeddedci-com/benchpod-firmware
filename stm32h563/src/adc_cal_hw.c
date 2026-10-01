/*
 * adc_cal_hw.c — the hardware half of adc_cal.h: the `amp` offset measurement.
 * Kept out of adc_cal.c so the checks and the store stay host-testable.
 */
#include "adc_cal.h"
#include "cal_data.h"
#include "i2c_bus.h"         /* analog_path_set */
#include "pico_compat.h"     /* sleep_ms */
#include "signal_engine.h"   /* adc_capture_psram_start / _poll */

static uint16_t s_buf[ADC_CAL_SAMPLES];

int adc_cal_measure_amp(adc_reading_t *out) {
    if (!out) return -1;
    if (analog_path_set(ANALOG_PATH_AMP) != 0) return -1;
    sleep_ms(20);    /* let the G6K relays (~4ms) + front-end RC settle */
    /* The same burst as adc_read (16 samples at the maximum rate), many times over: see
       adc_cal.h for why one long slow capture reads differently. */
    for (uint32_t i = 0; i < ADC_CAL_BURSTS; i++) {
        uint16_t *b = &s_buf[i * ADC_CAL_BURST];
        if (adc_capture_psram_start(ADC_CAL_BURST, 0.0f) != 0) return -2;
        int r;
        while ((r = adc_capture_psram_poll(b, ADC_CAL_BURST)) == 0) { }   /* 40 us; has its own deadline */
        if (r < 0) return -2;
        sleep_ms(ADC_CAL_GAP_MS);
    }
    /* The shared cal1 fit, without this pod's offset: this reading IS the offset. */
    *out = adc_scale_burst(s_buf, ADC_CAL_SAMPLES, ADC_CAL_CAL1.a, ADC_CAL_CAL1.b);
    return 0;
}
