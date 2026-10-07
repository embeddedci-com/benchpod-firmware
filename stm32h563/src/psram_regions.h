/*
 * psram_regions.h — is the last capture still in PSRAM?
 *
 * `capture_read` resumes a stalled read-back from PSRAM instead of capturing again. That is only
 * right while nothing has written over the capture since: an OTA stages its image at PSRAM 0
 * (the LA region), a `load_bin` with "psram" stages a waveform from the top down, a gateware
 * reload resets the fabric's capture bases, and a SCPI or console capture writes the same
 * regions without the JSON handler knowing. Any of those used to let `capture_read` stream
 * whatever was there as if it were the capture.
 *
 * The capture handler tracks the byte ranges its result occupies; every writer reports the
 * range it is about to write (psram_write does so for all STM32 writes, the capture arms for the
 * iCE40's). Once a write overlaps a tracked range the capture is stale, and the first reason is
 * kept for the error. Pure bookkeeping, host-tested (test/test_psram_regions.c).
 */
#ifndef PSRAM_REGIONS_H
#define PSRAM_REGIONS_H

#include <stdint.h>

/* Track the last capture: up to two byte ranges (ADC, LA); a zero length is no range. Clears
   any earlier staleness: this is fresh data. */
void psram_regions_track(uint32_t base0, uint32_t len0, uint32_t base1, uint32_t len1);
/* Track nothing (no capture to resume). */
void psram_regions_forget(void);
/* [base, base+len) is about to be written by `why` (a static string, e.g. "a firmware update").
   Overlapping a tracked range makes the capture stale; the first reason sticks. */
void psram_regions_dirty(uint32_t base, uint32_t len, const char *why);
/* Everything in PSRAM is suspect (a gateware reload). */
void psram_regions_dirty_all(const char *why);
/* NULL while the tracked capture is intact (or nothing is tracked), else why it is not. */
const char *psram_regions_stale(void);

#endif /* PSRAM_REGIONS_H */
