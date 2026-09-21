/* adc_scale.c — see adc_scale.h for why the circular average matters. */

#include "adc_scale.h"

#define WRAP_SPAN      65536
#define WRAP_THRESHOLD 32768

adc_reading_t adc_scale_burst(const uint16_t *raw, size_t n, float a, float b) {
    adc_reading_t r = { 0, 0.0f, 0.0f, 0.0f, 0u };
    if (!raw || n == 0) return r;

    /* Average on the CIRCLE: sum the shortest-arc delta of every sample from a
       reference (sample 0) instead of the samples themselves.  A burst straddling
       the 65535->0 cut then averages to the point BETWEEN its samples, not to
       mid-scale.  Deltas are computed in uint16 so the wrap is implicit. */
    const uint16_t ref = raw[0];
    int32_t sum = 0, dmin = 0, dmax = 0;
    for (size_t i = 0; i < n; i++) {
        int32_t d = (int32_t)(uint16_t)(raw[i] - ref);
        if (d > WRAP_THRESHOLD) d -= WRAP_SPAN;   /* -> [-32768, 32768] */
        sum += d;
        if (d < dmin) dmin = d;
        if (d > dmax) dmax = d;
    }
    r.span = (uint32_t)(dmax - dmin);

    float c = (float)ref + (float)sum / (float)n;
    /* Fold back into 0..65535 (the reference-relative mean can leave the range at
       either end), then apply the branch cut ONCE, on the mean. */
    while (c <    0.0f)     c += (float)WRAP_SPAN;
    while (c >= 65536.0f)   c -= (float)WRAP_SPAN;
    r.count = c;
    /* Unconditional negative-input unwrap: with b < 0 an un-unwrapped count below
       half-scale would decode above +32 V, which no path on this board presents. */
    r.count_uw = (c < (float)WRAP_THRESHOLD) ? c + (float)WRAP_SPAN : c;
    r.volts    = a + b * r.count_uw;
    r.valid    = (r.span <= ADC_BURST_MAX_SPAN);
    return r;
}
