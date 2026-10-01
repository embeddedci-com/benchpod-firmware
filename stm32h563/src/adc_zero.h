#ifndef ADC_ZERO_H
#define ADC_ZERO_H

#include <stdbool.h>
#include <stdint.h>

#include "adc_scale.h"

/* ---- Per-pod zero for the `amp` ADC source (J8, the 4-20 mA terminal) ----------------------
 *
 * `amp` has no fit of its own: it is scaled with ADC_CAL_CAL1 (cal_data.h), constants measured
 * on one board and compiled into every pod. The offset differs per pod (measured 2026-10-01:
 * pod .220 reads about +5 mV high, pod .215 about 0), and 5 mV across the 249 ohm sense
 * resistor is 20 uA.
 *
 * J8 pin 1 goes through R81 (249 ohm) to pod GND, so with nothing connected the terminal is
 * exactly 0 V and the pod can measure its own zero with no reference. `adc_zero` does that:
 * an average of many adc_read bursts with J8 open, stored here, subtracted from every later `amp` reading.
 *
 * Only the offset. Gain stays the shared fit. ext, cal1 and cal2 are not touched.
 *
 * Stored in its own power-loss-safe A/B store (config_store.h), sectors s116/s117
 * (0x1E8000 / 0x1EA000), carved from the top of FLASH_BLOBS. The record has spare words so
 * later per-pod ADC calibration can share these sectors instead of carving more.
 * ------------------------------------------------------------------------------------------*/

#define ADC_ZERO_SLOT_A_OFFSET  0x1EA000u
#define ADC_ZERO_SLOT_B_OFFSET  0x1E8000u
#define ADC_ZERO_RECORD_MAGIC   0x52455A41u   /* "AZER" */
#define ADC_ZERO_VERSION        1u

/* A real zero is a few mV. More than this means something is driving the terminal. */
#define ADC_ZERO_MAX_UV   50000

/* The zero measurement: 32 of the 16-sample bursts adc_read takes, spread over about 100 ms.
   It has to be the SAME burst, not one long slow capture: the front end reads lower at lower
   sample rates (measured on pod .220, J8 open: count 65535.4 at 400 kS/s, 65531.6 at
   100 kS/s, 65530.6 at 10 kS/s, about 5 mV apart), and the first samples of a burst sit
   about 1 count under the rest. A zero taken any other way is off by that much. */
#define ADC_ZERO_BURST    16u
#define ADC_ZERO_BURSTS   32u
#define ADC_ZERO_SAMPLES  (ADC_ZERO_BURST * ADC_ZERO_BURSTS)
#define ADC_ZERO_GAP_MS   3u

typedef struct {
    uint8_t  amp_set;       /* 1 = amp_uv holds a measured zero */
    uint8_t  rsvd8[3];
    int32_t  amp_uv;        /* what `amp` reads with J8 open, microvolts */
    uint32_t reserved[6];   /* room for later per-pod ADC calibration */
} adc_zero_t;

/* Load from flash into the active copy. 0 = a zero was loaded, -1 = none stored. */
int adc_zero_load(void);

/* The active `amp` zero. 0 uV when none is set. */
bool    adc_zero_amp_is_set(void);
int32_t adc_zero_amp_uv(void);
int32_t adc_zero_amp_mv(void);   /* rounded, for replies */

/* Store an UNCORRECTED `amp` reading taken with J8 open as the zero. NULL = stored, else why
   it was refused (not settled, or outside +/-ADC_ZERO_MAX_UV). A refusal keeps the old zero. */
const char *adc_zero_amp_store(const adc_reading_t *rd);

/* Remove the zero (flash and active copy). 0 on success. */
int adc_zero_clear(void);

/* An uncorrected `amp` reading in volts -> the corrected one. */
float adc_zero_amp_apply(float volts);

/* Hardware (adc_zero_hw.c, not in the host tests): route `amp`, let it settle and take the
   bursts, UNCORRECTED. 0 = *out filled (check out->valid), -1 = route failed,
   -2 = capture failed. Needs the iCE40 up; the caller owns the ADC. */
int adc_zero_measure_amp(adc_reading_t *out);

#endif /* ADC_ZERO_H */
