/*
 * adc_zero_hw.c — the hardware half of adc_zero.h: the long `amp` average.
 * Kept out of adc_zero.c so the checks and the store stay host-testable.
 */
#include "adc_zero.h"
#include "cal_data.h"
#include "i2c_bus.h"         /* analog_path_set */
#include "pico_compat.h"     /* sleep_ms */
#include "signal_engine.h"   /* adc_capture_psram_start / _poll */

static uint16_t s_buf[ADC_ZERO_SAMPLES];

int adc_zero_measure_amp(adc_reading_t *out) {
    if (!out) return -1;
    if (analog_path_set(ANALOG_PATH_AMP) != 0) return -1;
    sleep_ms(20);    /* let the G6K relays (~4ms) + front-end RC settle */
    if (adc_capture_psram_start(ADC_ZERO_SAMPLES, ADC_ZERO_RATE_HZ) != 0) return -2;
    /* The capture runs for 100 ms: sleep between polls instead of spinning like
       adc_capture_psram does. The poll has its own deadline. */
    for (;;) {
        int r = adc_capture_psram_poll(s_buf, ADC_ZERO_SAMPLES);
        if (r == 1) break;
        if (r < 0) return -2;
        sleep_ms(5);
    }
    /* `amp` is scaled with the cal1 fit, as in adc_read. No zero applied here. */
    *out = adc_scale_burst(s_buf, ADC_ZERO_SAMPLES, ADC_CAL_CAL1.a, ADC_CAL_CAL1.b);
    return 0;
}
