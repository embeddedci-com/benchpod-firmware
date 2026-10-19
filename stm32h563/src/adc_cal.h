#ifndef ADC_CAL_H
#define ADC_CAL_H

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "adc_scale.h"
#include "cal_data.h"

/* ---- Per-pod ADC calibration ---------------------------------------------------------------
 *
 * cal_data.h holds the ADC fits (volts = a + b*count) measured on one board and compiled into
 * every pod. This is the per-pod layer on top: values a pod measures on itself with the
 * `calibrate` command and keeps in flash. A source without a stored value uses cal_data.h
 * unchanged.
 *
 * Today it holds one value: the offset of the ADC front end, measured on the `current_in` source
 * (J8, the 4-20 mA terminal). It differs per pod: +4 mV on pod .220, about 0 on .215 (2026-10-01),
 * +13 mV on benchpod-bca439 and -50 mV on benchpod-445e46 (2026-10-19, DMM-checked).
 *
 * Every ADC source shares one chain: the relays (K4, K3) only choose what enters R90, and from
 * there the 910k/82.5k divider (about /12), the OPA810 buffer (U40), the THS4551 driver (U37,
 * gain 1.5, output common mode 2.05 V) and the ADC (U36) are the same for ext, current_in, cal1
 * and cal2. The offset comes from that chain: 1 % gain resistors around the driver turn part of
 * its 2.05 V common mode into a differential offset, about 8x larger referred to the input. So
 * one offset, measured where the input is a known 0 V, corrects every source.
 *
 * J8 pin 1 goes through R81 (249 ohm) to pod GND, so with nothing connected the terminal is
 * exactly 0 V and the pod needs no reference: `calibrate` reads `current_in` with J8 open and stores
 * what it reads as the offset. Every source's fit is then its cal_data.h fit with that offset
 * taken out of `a` (adc_cal_fit), and every reader uses that fit: adc_read, the capabilities
 * the server and SDK scale captures with, and the control loop's input map. Measured on
 * 445e46 against a DMM: ext and current_in exact, cal1 and cal2 within a few mV (their own
 * DAC-side stages add a little).
 *
 * Only the offset. Gain stays the shared fit (within 0.3 % on both pods): it needs a reference.
 *
 * Stored in its own power-loss-safe A/B store (config_store.h), sectors s116/s117
 * (0x1E8000 / 0x1EA000), carved from the top of FLASH_BLOBS. The record has spare words so
 * more per-pod calibration (other sources, gain) can share these sectors.
 * ------------------------------------------------------------------------------------------*/

#define ADC_CAL_SLOT_A_OFFSET  0x1EA000u
#define ADC_CAL_SLOT_B_OFFSET  0x1E8000u
#define ADC_CAL_RECORD_MAGIC   0x4C414341u   /* "ACAL" */
#define ADC_CAL_VERSION        1u

/* R81, the sense resistor from J8 pin 1 to pod GND: `current_in` reads the voltage across it. */
#define CURRENT_IN_SENSE_OHMS  249

/* Resistor tolerances put a pod's offset within tens of mV (worst case with 1 % parts is
   larger, but measured pods sit within +/-50 mV). More than this means something is driving
   the terminal. */
#define ADC_CAL_MAX_OFFSET_UV  150000

/* The measurement: 32 of the 16-sample bursts adc_read takes, spread over about 100 ms.
   It has to be the SAME burst, not one long slow capture: the front end reads lower at lower
   sample rates (measured on pod .220, J8 open: count 65535.4 at 400 kS/s, 65531.6 at
   100 kS/s, 65530.6 at 10 kS/s, about 5 mV apart), and the first samples of a burst sit
   about 1 count under the rest. An offset taken any other way is off by that much. */
#define ADC_CAL_BURST    16u
#define ADC_CAL_BURSTS   32u
#define ADC_CAL_SAMPLES  (ADC_CAL_BURST * ADC_CAL_BURSTS)
#define ADC_CAL_GAP_MS   3u

typedef struct {
    uint8_t  current_in_set;         /* 1 = current_in_offset_uv holds a measured offset */
    uint8_t  rsvd8[3];
    int32_t  current_in_offset_uv;   /* what `current_in` reads with J8 open, microvolts */
    uint32_t reserved[6];     /* room for more per-pod calibration */
} adc_cal_t;

/* Load from flash into the active copy. 0 = a calibration was loaded, -1 = none stored. */
int adc_cal_load(void);

/* The `current_in` offset of this pod. 0 uV when it was never calibrated. */
bool    adc_cal_current_in_is_set(void);
int32_t adc_cal_current_in_offset_uv(void);
int32_t adc_cal_current_in_offset_mv(void);   /* rounded, for replies */

/* A source's compiled-in fit with this pod's offset removed (unchanged when never calibrated). */
cal_lin_t adc_cal_fit(cal_lin_t base);

/* The fit to scale `current_in` counts with on this pod: ADC_CAL_CAL1, offset removed. */
cal_lin_t adc_cal_current_in_fit(void);

/* A calibrated `current_in` reading in volts as the loop current in microamps: 0.996 V is
   4000 uA. Integer, because newlib-nano's printf has no %f. */
static inline long current_in_ua(float volts) {
    return lroundf(volts * 1000000.0f / (float)CURRENT_IN_SENSE_OHMS);
}

/* Store a reading of `current_in` taken with J8 open and scaled with ADC_CAL_CAL1 (NOT with
   adc_cal_current_in_fit) as the offset. NULL = stored, else why it was refused (not settled, or
   outside +/-ADC_CAL_MAX_OFFSET_UV). A refusal keeps the old calibration. */
const char *adc_cal_current_in_store(const adc_reading_t *rd);

/* Remove the per-pod calibration (flash and active copy). 0 on success. */
int adc_cal_clear(void);

/* Hardware (adc_cal_hw.c, not in the host tests): route `current_in`, let it settle and take the
   bursts, scaled with ADC_CAL_CAL1. 0 = *out filled (check out->valid), -1 = route failed,
   -2 = capture failed. Needs the iCE40 up; the caller owns the ADC. */
int adc_cal_measure_current_in(adc_reading_t *out);

#endif /* ADC_CAL_H */
