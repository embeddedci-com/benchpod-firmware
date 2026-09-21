#ifndef MCU_ADC_H
#define MCU_ADC_H

#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * mcu_adc — the STM32's OWN 12-bit ADC (ADC1), for slow housekeeping reads.
 *
 * This is NOT the instrument ADC.  Capture/measure use the external 16-bit
 * MCP33131D sampled through the iCE40 (adc_scale.c / signal_engine.c); this
 * module only serves the two rev3 housekeeping straps that hang off MCU pins:
 *
 *   PA3  ADC1_INP15  board-revision strap            (board_rev.c)
 *   PC0  ADC1_INP10  USB-C CC1 sense                 (usb_cc.c)
 *   PC2  ADC12_INP12 USB-C CC2 sense                 (usb_cc.c)
 *
 * Conversions are single-shot and polled — a handful of microseconds each, no
 * DMA, no interrupt.  The full-scale reference is VREF+ (VDDA through R1 47 R,
 * so ~3.3 V); mcu_adc_init() measures it against the factory-calibrated VREFINT
 * rather than assuming 3300 mV, because the USB-C CC thresholds are only a few
 * hundred millivolts apart.
 *
 * Not thread-safe: ADC1 has one regular sequencer and callers share it.  All
 * current callers run from the hw worker / boot path under hw_lock.
 * ==========================================================================*/

/* Configure ADC1 (12-bit, single conversion), run the hardware calibration and
   latch VREF+ from VREFINT.  Idempotent.  Returns 0 on success, -1 on failure. */
int mcu_adc_init(void);

/* True once mcu_adc_init() has succeeded. */
bool mcu_adc_ready(void);

/* Measured VREF+ in millivolts (the ADC full-scale span).  Returns the 3300 mV
   nominal if init has not run or VREFINT could not be read. */
int mcu_adc_vref_mv(void);

/* Convert one channel (an ADC_CHANNEL_x from board_pins.h).  *out_mv is the
   pin voltage in millivolts.  Returns 0, or -1 on a conversion error/timeout. */
int mcu_adc_read_mv(uint32_t channel, int *out_mv);

/* Average `n` conversions of one channel (n clamped to 1..64).  Used for the
   CC lines, which sit behind a 10 k / 10 n filter and pick up switching noise. */
int mcu_adc_read_mv_avg(uint32_t channel, unsigned n, int *out_mv);

#endif /* MCU_ADC_H */
