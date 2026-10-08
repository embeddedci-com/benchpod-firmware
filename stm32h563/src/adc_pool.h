#ifndef ADC_POOL_H
#define ADC_POOL_H

/*
 * adc_pool — the one sample buffer in RAM, shared by the JSON commands, SCPI and the console.
 *
 * Every transport used to keep its own: JSON a 64 KB read-back/upload buffer plus a 4 KB byte
 * buffer, SCPI an 8 KB trace and 1.5 KB of capture scratch, the console 1.3 KB of diagnostics
 * scratch. They are all filled under the heavy gate (command_handler_acquire_adc), and every user
 * runs on the hw worker, so one buffer does for all of them.
 *
 * Two regions:
 *   trace    the first ADC_POOL_TRACE_BYTES: a waveform kept for a later replay (the JSON
 *            capture/load -> replay, SCPI READ?/TRACe:DATA -> SOURce:FUNCtion USER). Whoever
 *            fills it takes a new generation (adc_pool_take); a kept trace is still its own only
 *            while adc_pool_holds(its generation), so a replay never plays someone else's data.
 *   scratch  the rest: transient bytes inside one command (sensor reads, I2C-LA captures, SCPI
 *            DIAGnostic:CAPture?, console diagnostics). Never kept, so no generation.
 * A shallow JSON capture may fill the whole pool (both regions); it takes a generation.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "signal_engine.h"   /* SIGNAL_BUF_SIZE */

#define ADC_POOL_SAMPLES      32768u                   /* 64 KB */
#define ADC_POOL_BYTES        (ADC_POOL_SAMPLES * 2u)
/* The longest trace anyone keeps: a JSON `load` (2 * SIGNAL_BUF_SIZE bytes) or a 4096-point SCPI
   READ?. A replay plays at most SIGNAL_BUF_SIZE bytes of it. */
#define ADC_POOL_TRACE_BYTES  (2u * SIGNAL_BUF_SIZE)
#define ADC_POOL_SCRATCH_BYTES (ADC_POOL_BYTES - ADC_POOL_TRACE_BYTES)

extern uint16_t adc_pool[ADC_POOL_SAMPLES];

/* The scratch region (ADC_POOL_SCRATCH_BYTES, 4-byte aligned). */
static inline uint8_t *adc_pool_scratch(void) { return (uint8_t *)adc_pool + ADC_POOL_TRACE_BYTES; }

/* The caller is about to fill the trace region: a new generation (never 0). Every kept trace taken
   before it stops being held. */
uint32_t adc_pool_take(void);
/* Is the trace region still what generation `gen` put there? false for 0. */
bool adc_pool_holds(uint32_t gen);

#endif /* ADC_POOL_H */
