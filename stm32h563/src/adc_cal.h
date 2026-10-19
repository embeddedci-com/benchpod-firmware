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
 * Only the offset needs nothing but an open J8. The rest is the optional DMM-guided calibration
 * (embeddedci-server/docs/benchpod-dmm-calibration.md), firmware 3.9.3:
 *   - DAC: this pod's own line per output path (3v3, 5v, 12v), from two set points the pod drove
 *     and a DMM read on that output's SMA (adc_cal_dac_store). It replaces the compiled line for
 *     every DAC user through cal_data_set_pod_dac (DAC_CAL): dac_out, the calibrated generate,
 *     the DAC output limits.
 *   - ADC gain per source (cal1, cal2, ext), measured through a loopback against a DAC path that
 *     has its DMM line (adc_cal_gain_store). `current_in` is scaled with the cal1 fit and follows
 *     its gain. With offset and gain a source reads volts = g * (a + b*count - offset): 0 V stays
 *     0 V, and the gain does not depend on the offset, so either can be redone alone.
 *
 * Stored in its own power-loss-safe A/B store (config_store.h), sectors s116/s117
 * (0x1E8000 / 0x1EA000), carved from the top of FLASH_BLOBS. Version 1 (firmware 3.4.0 to 3.9.2)
 * held only the offset; version 2 adds the DMM parts. A version-1 record still loads (its offset);
 * the next save writes version 2 to the other slot, so a power cut during that leaves version 1
 * readable. Older firmware ignores a version-2 record and runs uncalibrated.
 * ------------------------------------------------------------------------------------------*/

#define ADC_CAL_SLOT_A_OFFSET  0x1EA000u
#define ADC_CAL_SLOT_B_OFFSET  0x1E8000u
#define ADC_CAL_RECORD_MAGIC   0x4C414341u   /* "ACAL" */
#define ADC_CAL_VERSION        2u
#define ADC_CAL_VERSION_1      1u            /* the offset-only record, loaded for migration */

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

/* DMM limits. A DAC reading must lie within ADC_CAL_DAC_TOL_UV + ADC_CAL_DAC_TOL_PPM of what the
   compiled line gives at that code (measured pods: under 30 mV and 0.3 %), the two codes at least
   ADC_CAL_DAC_MIN_SPAN apart, and every gain within ADC_CAL_MAX_GAIN_PPM of the compiled one. */
#define ADC_CAL_DAC_TOL_UV      100000
#define ADC_CAL_DAC_TOL_PPM     20000
#define ADC_CAL_DAC_MIN_SPAN    16384u
#define ADC_CAL_MAX_GAIN_PPM    20000

/* DAC output paths: the DAC_CAL index (cal_data.h). */
#define ADC_CAL_DAC_PATHS  3
/* ADC sources with a gain of their own (current_in follows cal1). */
enum { ADC_CAL_SRC_CAL1 = 0, ADC_CAL_SRC_CAL2 = 1, ADC_CAL_SRC_EXT = 2, ADC_CAL_SRCS = 3 };

typedef struct {
    uint8_t   current_in_set;         /* 1 = current_in_offset_uv holds a measured offset */
    uint8_t   dac_set;                /* bit n = dac[n] is this pod's DMM line (0 3v3, 1 5v, 2 12v) */
    uint8_t   adc_gain_set;           /* bit n = adc_gain[n] is measured (ADC_CAL_SRC_*) */
    uint8_t   rsvd8;
    int32_t   current_in_offset_uv;   /* what `current_in` reads with J8 open, microvolts */
    cal_lin_t dac[ADC_CAL_DAC_PATHS]; /* volts = a + b*code8, the cal_data.h convention */
    float     adc_gain[ADC_CAL_SRCS]; /* 1.0 = the compiled gain */
    uint32_t  reserved[6];            /* room for more (the 4-20 mA output, ...) */
} adc_cal_t;

/* Version 1, as firmware 3.4.0 to 3.9.2 stored it. */
typedef struct {
    uint8_t  current_in_set;
    uint8_t  rsvd8[3];
    int32_t  current_in_offset_uv;
    uint32_t reserved[6];
} adc_cal_v1_t;

/* Load from flash into the active copy and put this pod's DAC lines in (cal_data_set_pod_dac).
   0 = a calibration was loaded, -1 = none stored (or one out of range, ignored as a whole). */
int adc_cal_load(void);

/* The `current_in` offset of this pod. 0 uV when it was never calibrated. */
bool    adc_cal_current_in_is_set(void);
int32_t adc_cal_current_in_offset_uv(void);
int32_t adc_cal_current_in_offset_mv(void);   /* rounded, for replies */

/* A compiled-in fit with this pod's offset removed, no gain (unchanged when never calibrated). */
cal_lin_t adc_cal_fit(cal_lin_t base);

/* The fit a source is scaled with on this pod: its cal_data.h fit (cal1, cal2, ext), offset
   removed and gain applied. ADC_CAL_SRC_*. */
cal_lin_t adc_cal_source_fit(int src);

/* The fit to scale `current_in` counts with on this pod: cal1's (adc_cal_source_fit). */
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

/* ---- DMM parts ---- */

/* This pod's DMM line for a DAC path, and whether it has one. */
bool      adc_cal_dac_is_set(int path);
cal_lin_t adc_cal_dac_line(int path);

/* Store this pod's line for `path` (0 3v3, 1 5v, 2 12v) from two points: the 16-bit code the pod
   drove and what the DMM read, in microvolts. NULL = stored and in use, else why it was refused. */
const char *adc_cal_dac_store(int path, uint16_t code1, int32_t uv1, uint16_t code2, int32_t uv2);

/* A source's gain correction, and whether it was measured. */
bool  adc_cal_gain_is_set(int src);
float adc_cal_gain(int src);

/* The DAC path that drives a source's loopback (cal1: 5v, cal2 and ext: 12v), and its two set
   points in volts. */
int   adc_cal_gain_path(int src);
float adc_cal_gain_point(int src, int i);

/* NULL when a gain measurement of `src` may run: the offset is calibrated and the driving path
   has its DMM line. Else why not. */
const char *adc_cal_gain_ready(int src);

/* A gain measurement: what the DAC put out at the two set points (v, volts, from its DMM line)
   and what the ADC read there, scaled with the source's compiled fit and this pod's offset but
   no gain (r). */
typedef struct {
    float v[2];
    float r[2];
    float gain;          /* filled by adc_cal_gain_store */
    float residual_v;    /* what is left at the set points after the gain, mean, volts */
} adc_cal_gain_meas_t;

/* Store the gain from a measurement. NULL = stored, else why it was refused. */
const char *adc_cal_gain_store(int src, adc_cal_gain_meas_t *m);

/* Names for replies and requests: "3v3"/"5v"/"12v" and "cal1"/"cal2"/"ext" (-1 = unknown). */
const char *adc_cal_src_name(int src);
int         adc_cal_src_index(const char *name);

/* Remove the per-pod calibration (flash and active copy), or one part of it: "offset", "dac"
   or "adc" (NULL = all). 0 on success, -1 failed, -2 unknown part. */
int adc_cal_clear(void);
int adc_cal_clear_part(const char *part);

/* Hardware (adc_cal_hw.c, not in the host tests): route `current_in`, let it settle and take the
   bursts, scaled with ADC_CAL_CAL1. 0 = *out filled (check out->valid), -1 = route failed,
   -2 = capture failed. Needs the iCE40 up; the caller owns the ADC. */
int adc_cal_measure_current_in(adc_reading_t *out);

/* Hardware: drive the source's two set points through its loopback and fill m->v and m->r.
   0 = done, -1 = route or DAC failed, -2 = capture failed, -3 = a burst was not settled
   (*span_out = its span). Leaves the outputs off. The caller owns the ADC. */
int adc_cal_measure_gain(int src, adc_cal_gain_meas_t *m, uint32_t *span_out);

#endif /* ADC_CAL_H */
