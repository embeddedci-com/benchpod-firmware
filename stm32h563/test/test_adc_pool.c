/*
 * test_adc_pool.c — host tests for the shared sample buffer's ownership (src/adc_pool.c).
 */
#include "adc_pool.h"

#include <stdio.h>

static int failures;
#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); failures++; } \
} while (0)

int main(void) {
    CHECK(!adc_pool_holds(0));
    uint32_t json = adc_pool_take();           /* a JSON capture fills the trace */
    CHECK(json != 0 && adc_pool_holds(json));
    uint32_t scpi = adc_pool_take();           /* then SCPI READ? */
    CHECK(scpi != json && adc_pool_holds(scpi) && !adc_pool_holds(json));
    CHECK(!adc_pool_holds(0));                 /* "nothing kept" is never held */

    /* the regions: the trace covers what anyone keeps, the scratch follows it, both in the pool */
    CHECK(ADC_POOL_TRACE_BYTES == 8192u);
    CHECK(adc_pool_scratch() == (uint8_t *)adc_pool + ADC_POOL_TRACE_BYTES);
    CHECK(ADC_POOL_TRACE_BYTES + ADC_POOL_SCRATCH_BYTES == sizeof(adc_pool));
    CHECK(((uintptr_t)adc_pool_scratch() & 3u) == 0);
    CHECK(ADC_POOL_SCRATCH_BYTES >= SIGNAL_BUF_SIZE);   /* the JSON sensor/I2C-LA byte buffer */

    /* a new generation is never 0 */
    for (int i = 0; i < 5; i++) CHECK(adc_pool_take() != 0);
    if (failures) { printf("test_adc_pool: %d FAILED\n", failures); return 1; }
    printf("test_adc_pool: all passed\n");
    return 0;
}
