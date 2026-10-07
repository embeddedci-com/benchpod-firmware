/*
 * test_psram_regions.c — host tests for the "is the last capture still in PSRAM" tracker
 * (src/psram_regions.c) that capture_read checks before resuming a read-back.
 */
#include "psram_regions.h"

#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); failures++; } } while (0)

#define LA_BASE  0x000000u
#define ADC_BASE 0x400000u

static void test_nothing_tracked_is_never_stale(void)
{
    psram_regions_forget();
    psram_regions_dirty(0, 0x800000u, "anything");
    psram_regions_dirty_all("a gateware reload");
    CHECK(psram_regions_stale() == NULL);
}

static void test_ota_staging_over_the_la_region(void)
{
    /* capture_dual: 1000 ADC samples + 5000 LA samples */
    psram_regions_track(ADC_BASE, 2000, LA_BASE, 10000);
    CHECK(psram_regions_stale() == NULL);
    psram_regions_dirty(0, 1536, "a firmware update staged into PSRAM");
    CHECK(psram_regions_stale() && strstr(psram_regions_stale(), "firmware update"));
}

static void test_first_reason_sticks(void)
{
    psram_regions_track(ADC_BASE, 2000, 0, 0);
    psram_regions_dirty(ADC_BASE + 100, 4, "a PSRAM write by the firmware");
    psram_regions_dirty(ADC_BASE, 2000, "another capture");
    CHECK(strcmp(psram_regions_stale(), "a PSRAM write by the firmware") == 0);
}

static void test_disjoint_writes_keep_it_fresh(void)
{
    /* LA-only capture at the front; a deep DAC waveform staged from the top down, below it. */
    psram_regions_track(LA_BASE, 0x10000u, 0, 0);
    psram_regions_dirty(0x800000u - 0x8000u, 0x8000u, "a load_bin psram upload");
    psram_regions_dirty(0x10000u, 16, "adjacent, not overlapping");      /* starts at the end */
    CHECK(psram_regions_stale() == NULL);
    psram_regions_dirty(0xFFFFu, 1, "the last byte");
    CHECK(psram_regions_stale() && strcmp(psram_regions_stale(), "the last byte") == 0);
}

static void test_track_clears_and_dirty_all(void)
{
    psram_regions_track(LA_BASE, 64, 0, 0);
    psram_regions_dirty_all("a gateware reload");
    CHECK(strcmp(psram_regions_stale(), "a gateware reload") == 0);
    psram_regions_track(LA_BASE, 64, ADC_BASE, 64);   /* a new capture: fresh */
    CHECK(psram_regions_stale() == NULL);
    psram_regions_dirty(0xFFFFFFF0u, 0x100u, "wraps past 4 GB");     /* no overflow false hit */
    CHECK(psram_regions_stale() == NULL);
    psram_regions_forget();
    CHECK(psram_regions_stale() == NULL);
}

int main(void)
{
    test_nothing_tracked_is_never_stale();
    test_ota_staging_over_the_la_region();
    test_first_reason_sticks();
    test_disjoint_writes_keep_it_fresh();
    test_track_clears_and_dirty_all();
    if (failures) { printf("test_psram_regions: %d FAILURE(S)\n", failures); return 1; }
    printf("test_psram_regions: all passed\n");
    return 0;
}
