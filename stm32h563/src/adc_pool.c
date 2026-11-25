/*
 * adc_pool.c — the shared sample buffer (see adc_pool.h).
 */
#include "adc_pool.h"

uint16_t adc_pool[ADC_POOL_SAMPLES] __attribute__((aligned(4)));

static uint32_t s_gen;   /* the generation that last filled the trace region, 0 = none yet */

uint32_t adc_pool_take(void) {
    if (++s_gen == 0) s_gen = 1;   /* 0 means "nothing kept" */
    return s_gen;
}

bool adc_pool_holds(uint32_t gen) { return gen != 0 && gen == s_gen; }

/* Overridden by command_handler.c; a build or test without it has no asynchronous pool user. */
__attribute__((weak)) bool adc_pool_users_busy(void) { return false; }

bool adc_pool_overlay_begin(void) {
    if (adc_pool_users_busy()) return false;
    (void)adc_pool_take();   /* the SCPI and JSON traces are overwritten by the image */
    return true;
}
