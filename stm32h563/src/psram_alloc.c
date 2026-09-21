/* psram_alloc.c — tri-capture PSRAM zone allocator + fit checks.
 * See psram_alloc.h and docs/tri-capture-unified-psram.md. Pure logic, no I/O. */
#include "psram_alloc.h"

psram_layout_t psram_plan(const psram_request_t *req)
{
    psram_layout_t out;

    /* 2 bytes/sample for both capture streams; DAC length is already in bytes. */
    const uint32_t la_len  = req->la_samples  * 2u;
    const uint32_t adc_len = req->adc_samples * 2u;
    const uint32_t dac_len = req->dac_bytes;

    /* Pack contiguously LA -> ADC -> DAC. An absent stream (len 0) leaves no gap: its
       base is the running offset and the next stream starts at the same address. */
    uint32_t off = 0u;
    out.la_base  = off; out.la_len  = la_len;  off += la_len;
    out.adc_base = off; out.adc_len = adc_len; off += adc_len;
    out.dac_base = off; out.dac_len = dac_len; off += dac_len;

    /* 64-bit accumulators so a pathological request can't wrap a 32-bit sum before the
       check catches it (max legit total is 8 MB, but inputs are host-supplied). */
    const uint64_t need = (uint64_t)la_len + adc_len + dac_len;
    const uint64_t load = ((uint64_t)req->la_rate_hz + req->adc_rate_hz + req->dac_rate_hz) * 2u;

    out.need_bytes  = (need > 0xFFFFFFFFu) ? 0xFFFFFFFFu : (uint32_t)need;
    out.avail_bytes = PSRAM_USABLE_BYTES;
    out.load_bps    = (load > 0xFFFFFFFFu) ? 0xFFFFFFFFu : (uint32_t)load;
    out.bus_bps     = PSRAM_SUSTAINED_BPS;

    /* 1. capacity: everything requested must fit below the guard tail. */
    if (need > (uint64_t)PSRAM_USABLE_BYTES) {
        out.status = PSRAM_ALLOC_ERR_CAPACITY;
        return out;
    }
    /* 2. bandwidth: the combined live streams must be drainable by the shared bus. */
    if (load > (uint64_t)PSRAM_SUSTAINED_BPS) {
        out.status = PSRAM_ALLOC_ERR_BANDWIDTH;
        return out;
    }

    out.status = PSRAM_ALLOC_OK;
    return out;
}
