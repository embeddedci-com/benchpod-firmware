/* Host unit test for psram_plan() — the tri-capture PSRAM zone allocator +
 * capacity/bandwidth fit-check (docs/tri-capture-unified-psram.md).
 *
 * Pins:
 *   - flexible packing: each active stream gets a contiguous zone sized to its own
 *     depth; absent streams (len 0) leave no gap; bases pack LA -> ADC -> DAC,
 *   - capacity: total must fit in PSRAM_USABLE_BYTES (guard tail reserved),
 *   - bandwidth: combined live rate*2 must be <= PSRAM_SUSTAINED_BPS,
 *   - capacity is checked BEFORE bandwidth (a request failing both reports capacity).
 */
#include "psram_alloc.h"
#include <stdio.h>

static int fails = 0;
#define CHECK(cond, ...) do { if (!(cond)) { \
        printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

/* A rate mix well under the bus so capacity cases aren't masked by bandwidth. */
#define SLOW_HZ 100000u

int main(void)
{
    /* ---- single stream uses (nearly) the whole chip ---- */
    {
        psram_request_t r = { .la_samples = 1000, .la_rate_hz = 6000000u };
        psram_layout_t l = psram_plan(&r);
        CHECK(l.status == PSRAM_ALLOC_OK, "LA-only should be OK (status=%d)", l.status);
        CHECK(l.la_base == 0 && l.la_len == 2000u, "LA zone [0,2000) got [%u,%u)", l.la_base, l.la_len);
        CHECK(l.adc_len == 0 && l.dac_len == 0, "absent ADC/DAC must be zero-length");
        /* absent streams still get a defined base = running offset (no gap) */
        CHECK(l.adc_base == 2000u && l.dac_base == 2000u, "absent zones abut LA end (adc=%u dac=%u)", l.adc_base, l.dac_base);
    }

    /* ---- exactly-full single stream is OK; one sample over fails capacity ---- */
    {
        psram_request_t full = { .la_samples = PSRAM_USABLE_BYTES / 2u, .la_rate_hz = SLOW_HZ };
        psram_layout_t l = psram_plan(&full);
        CHECK(l.status == PSRAM_ALLOC_OK, "exactly-usable LA must fit (need=%u avail=%u)", l.need_bytes, l.avail_bytes);
        CHECK(l.la_len == PSRAM_USABLE_BYTES, "full LA len=%u want %u", l.la_len, PSRAM_USABLE_BYTES);

        psram_request_t over = { .la_samples = PSRAM_USABLE_BYTES / 2u + 1u, .la_rate_hz = SLOW_HZ };
        psram_layout_t lo = psram_plan(&over);
        CHECK(lo.status == PSRAM_ALLOC_ERR_CAPACITY, "one-over must fail capacity (status=%d)", lo.status);
        CHECK(lo.need_bytes > lo.avail_bytes, "capacity diag need>avail (%u>%u)", lo.need_bytes, lo.avail_bytes);
    }

    /* ---- two streams pack contiguously ---- */
    {
        psram_request_t r = { .la_samples = 100, .adc_samples = 50,
                              .la_rate_hz = SLOW_HZ, .adc_rate_hz = SLOW_HZ };
        psram_layout_t l = psram_plan(&r);
        CHECK(l.status == PSRAM_ALLOC_OK, "LA+ADC should be OK");
        CHECK(l.la_base == 0 && l.la_len == 200u, "LA [0,200)");
        CHECK(l.adc_base == 200u && l.adc_len == 100u, "ADC abuts LA at 200 (base=%u len=%u)", l.adc_base, l.adc_len);
        CHECK(l.dac_base == 300u && l.dac_len == 0, "DAC absent, base at 300");
    }

    /* ---- three streams pack contiguously, no gaps ---- */
    {
        psram_request_t r = { .la_samples = 100, .adc_samples = 100, .dac_bytes = 400,
                              .la_rate_hz = 1000000u, .adc_rate_hz = 500000u, .dac_rate_hz = 900000u };
        psram_layout_t l = psram_plan(&r);
        CHECK(l.status == PSRAM_ALLOC_OK, "3-way should be OK (load=%u bus=%u)", l.load_bps, l.bus_bps);
        CHECK(l.la_base == 0   && l.la_len  == 200u, "LA  [0,200)");
        CHECK(l.adc_base == 200 && l.adc_len == 200u, "ADC [200,400)");
        CHECK(l.dac_base == 400 && l.dac_len == 400u, "DAC [400,800)");
        CHECK(l.need_bytes == 800u, "need=800 got %u", l.need_bytes);
        /* zones disjoint + ordered */
        CHECK(l.la_base + l.la_len == l.adc_base, "LA end == ADC base");
        CHECK(l.adc_base + l.adc_len == l.dac_base, "ADC end == DAC base");
    }

    /* ---- absent middle stream leaves no gap ---- */
    {
        psram_request_t r = { .la_samples = 100, .adc_samples = 0, .dac_bytes = 100,
                              .la_rate_hz = SLOW_HZ, .dac_rate_hz = SLOW_HZ };
        psram_layout_t l = psram_plan(&r);
        CHECK(l.status == PSRAM_ALLOC_OK, "LA+DAC (no ADC) should be OK");
        CHECK(l.adc_len == 0 && l.adc_base == 200u, "ADC zero-length at LA end");
        CHECK(l.dac_base == 200u && l.dac_len == 100u, "DAC abuts directly (base=%u) — no gap for absent ADC", l.dac_base);
    }

    /* ---- bandwidth: fits in memory but the combined rate overruns the bus ---- */
    {
        /* tiny depths (trivially fit) but rates sum*2 = 22 MB/s > 20 MB/s */
        psram_request_t r = { .la_samples = 10, .adc_samples = 10, .dac_bytes = 10,
                              .la_rate_hz = 8000000u, .adc_rate_hz = 1000000u, .dac_rate_hz = 2000000u };
        psram_layout_t l = psram_plan(&r);
        CHECK(l.status == PSRAM_ALLOC_ERR_BANDWIDTH, "over-rate must fail bandwidth (status=%d)", l.status);
        CHECK(l.load_bps == 22000000u, "load=%u want 22000000", l.load_bps);
        CHECK(l.load_bps > l.bus_bps, "bandwidth diag load>bus (%u>%u)", l.load_bps, l.bus_bps);
    }

    /* ---- bandwidth boundary: rate*2 exactly == bus is OK; +1 Hz fails ---- */
    {
        psram_request_t at = { .la_samples = 10, .la_rate_hz = PSRAM_SUSTAINED_BPS / 2u };
        CHECK(psram_plan(&at).status == PSRAM_ALLOC_OK, "load exactly == bus must pass");
        psram_request_t over = { .la_samples = 10, .la_rate_hz = PSRAM_SUSTAINED_BPS / 2u + 1u };
        CHECK(psram_plan(&over).status == PSRAM_ALLOC_ERR_BANDWIDTH, "one-Hz-over must fail bandwidth");
    }

    /* ---- capacity is checked before bandwidth (fails both -> reports capacity) ---- */
    {
        psram_request_t r = { .la_samples = PSRAM_USABLE_BYTES, /* 2x too big */
                              .la_rate_hz = 50000000u /* also way over bus */ };
        CHECK(psram_plan(&r).status == PSRAM_ALLOC_ERR_CAPACITY,
              "fails both -> capacity takes precedence");
    }

    /* ---- empty request (all absent) is a valid no-op ---- */
    {
        psram_request_t r = { 0 };
        psram_layout_t l = psram_plan(&r);
        CHECK(l.status == PSRAM_ALLOC_OK, "empty request OK");
        CHECK(l.need_bytes == 0 && l.load_bps == 0, "empty need/load both 0");
    }

    if (fails) { printf("psram_alloc: %d CHECK(s) FAILED\n", fails); return 1; }
    printf("psram_alloc: all checks passed\n");
    return 0;
}
