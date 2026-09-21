#ifndef PSRAM_ALLOC_H
#define PSRAM_ALLOC_H
#include <stdint.h>

/* Tri-capture PSRAM zone allocator — the pure, host-testable core of the
 * concurrent DAC + ADC + LA feature (docs/tri-capture-unified-psram.md).
 *
 * Given a request for any SUBSET of the three deep streams, it packs each active
 * stream a disjoint contiguous zone sized to ITS OWN requested depth (flexible
 * packing — not fixed halves/thirds) and validates TWO independent limits BEFORE
 * the firmware arms the iCE40:
 *   1. capacity  — the zones must fit in PSRAM (minus a guard tail),
 *   2. bandwidth — the combined SUSTAINED live byte-rate must not exceed what the
 *                  shared quad bus can drain, or a staging FIFO overflows -> CAP_OVF.
 * Memory-fit is necessary but NOT sufficient: LA at 12 MS/s alone eats 24 MB/s, so a
 * combo can fit in 8 MB yet still overrun the bus. Both checks live here so
 * test/test_psram_alloc.c can pin them and the firmware just consumes the result.
 *
 * Kept independent of fpga_config.h (the geometry is duplicated below) so this unit
 * compiles standalone in the host test — same pattern as la_rate.[ch]. */

/* ---- PSRAM geometry (mirror of fpga_config.h; single-sourced there at integration) ---- */
#define PSRAM_TOTAL_BYTES   0x800000u   /* 8 MB APS6404L                                   */
#define PSRAM_GUARD_BYTES   0x001000u   /* 4 KB unallocated tail: a final in-flight write   */
                                        /* burst can never run off the end of the chip.     */
#define PSRAM_USABLE_BYTES  (PSRAM_TOTAL_BYTES - PSRAM_GUARD_BYTES)

/* Effective SUSTAINED drain of the shared quad bus, in bytes/s, AFTER per-burst
 * overhead (CS lead + 0x38/0xEB command + 24-bit address + read dummy + CS-high tCEM
 * gap) — NOT the 48 Mnib/s (24 MB/s) raw payload rate. This is the aggregate the three
 * jobs share. PROVISIONAL until HW-characterised at CHUNK=16 with all three contending
 * (docs/tri-capture-unified-psram.md, "Open / HW-characterise"); set CONSERVATIVELY so
 * the 3-way case is throttled below the real ceiling. Same knob role as
 * LA_PSRAM_DRAIN_HZ in la_rate.h, expressed in bytes/s for the mixed-stream sum. */
#define PSRAM_SUSTAINED_BPS 20000000u   /* ~20 MB/s aggregate; HW-tune before shipping 3-way */

typedef enum {
    PSRAM_ALLOC_OK        = 0,
    PSRAM_ALLOC_ERR_CAPACITY  = 1,   /* zones don't fit in PSRAM_USABLE_BYTES     */
    PSRAM_ALLOC_ERR_BANDWIDTH = 2,   /* combined live rate exceeds the bus drain  */
} psram_alloc_status_t;

/* A stream is ABSENT when its `samples` (DAC: `bytes`) is 0 — it gets a zero-length
 * zone and is skipped by the gateware job, so its neighbours abut with no gap. */
typedef struct {
    uint32_t la_samples;    /* 12-ch LA, 2 B/sample                                     */
    uint32_t adc_samples;   /* 16-bit ADC, 2 B/sample                                   */
    uint32_t dac_bytes;     /* DAC replay waveform length in bytes (already samples*2)  */
    /* SUSTAINED (post-plan) rates for the bandwidth check, Hz; 0 if that stream absent.
       Pass the LA rate AFTER la_psram_plan()'s burst-cap, the ADC capture rate, and the
       DAC update rate. */
    uint32_t la_rate_hz;
    uint32_t adc_rate_hz;
    uint32_t dac_rate_hz;
} psram_request_t;

typedef struct {
    psram_alloc_status_t status;
    /* Assigned zones — meaningful only when status == PSRAM_ALLOC_OK. An absent stream
       has len == 0 (base still set to its packing offset). Order: LA -> ADC -> DAC. */
    uint32_t la_base,  la_len;
    uint32_t adc_base, adc_len;
    uint32_t dac_base, dac_len;
    /* Diagnostics — always filled, for firmware error messages on failure. */
    uint32_t need_bytes;    /* total requested bytes (la_len + adc_len + dac_len)        */
    uint32_t avail_bytes;   /* PSRAM_USABLE_BYTES                                        */
    uint32_t load_bps;      /* combined sustained byte-rate on the bus                   */
    uint32_t bus_bps;       /* PSRAM_SUSTAINED_BPS                                       */
} psram_layout_t;

/* Pack the active streams into disjoint contiguous zones and validate capacity then
 * bandwidth. Pure, no side effects. On failure `status` names which limit was hit and
 * the diagnostics carry the numbers for the error string; zones are left as packed
 * (do not arm on a non-OK status). */
psram_layout_t psram_plan(const psram_request_t *req);

#endif /* PSRAM_ALLOC_H */
