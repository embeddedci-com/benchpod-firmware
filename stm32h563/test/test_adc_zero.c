/* Host unit test for adc_zero.c — the per-pod zero of the `amp` ADC source (J8).
 *
 * The numbers are the ones measured on pod 192.168.1.220 (2026-10-01): with nothing driving
 * J8 the pod read count 65529, about +10 mV, and 3995 mV at 16 mA. The zero must take that
 * offset out of every later reading, survive a reboot, and refuse to store a "zero" taken
 * while something drives the terminal.
 */
#include "adc_zero.h"
#include "adc_scale.h"
#include "cal_data.h"
#include "config_store.h"
#include "mocks/mock_flash.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int fails = 0;
#define CHECK(cond, ...) do { if (!(cond)) { \
        printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

static uint16_t buf[ADC_ZERO_SAMPLES];

/* A burst alternating between two counts, scaled the way firmware scales `amp`. */
static adc_reading_t burst(uint16_t lo, uint16_t hi) {
    for (size_t i = 0; i < ADC_ZERO_SAMPLES; i++) buf[i] = (i & 1) ? hi : lo;
    return adc_scale_burst(buf, ADC_ZERO_SAMPLES, ADC_CAL_CAL1.a, ADC_CAL_CAL1.b);
}

static long mv(float volts) { return lroundf(volts * 1000.0f); }

static void test_none_changes_nothing(void) {
    mock_flash_reset();
    CHECK(adc_zero_load() == -1, "a blank pod loaded a zero");
    CHECK(!adc_zero_amp_is_set() && adc_zero_amp_uv() == 0 && adc_zero_amp_mv() == 0, "blank pod has a zero");
    CHECK(adc_zero_amp_apply(3.995f) == 3.995f, "reading changed without a zero");
}

static void test_store_apply_persist(void) {
    mock_flash_reset();
    adc_zero_load();
    adc_reading_t open = burst(65529, 65529);
    CHECK(mv(open.volts) == 10, "open J8 at count 65529 should read 10 mV, got %ld", mv(open.volts));
    CHECK(adc_zero_amp_store(&open) == NULL, "a 10 mV zero was refused: %s", adc_zero_amp_store(&open));
    CHECK(adc_zero_amp_is_set() && adc_zero_amp_mv() == 10, "zero_mv %ld", (long)adc_zero_amp_mv());
    CHECK(labs((long)adc_zero_amp_uv() - 10418) < 20, "zero_uv %ld", (long)adc_zero_amp_uv());

    /* Open terminal now reads 0, and a real reading drops by the same offset. */
    CHECK(mv(adc_zero_amp_apply(open.volts)) == 0, "open J8 reads %ld mV after the zero", mv(adc_zero_amp_apply(open.volts)));
    CHECK(mv(adc_zero_amp_apply(3.995f)) == 3985, "16 mA reads %ld mV", mv(adc_zero_amp_apply(3.995f)));

    /* Reboot: the active copy is gone, flash is not. */
    int32_t uv = adc_zero_amp_uv();
    CHECK(adc_zero_load() == 0 && adc_zero_amp_is_set() && adc_zero_amp_uv() == uv, "zero lost over a reload");
    CHECK(mock_flash_double_programs == 0 && mock_flash_ecc_reads == 0, "flash misuse");

    /* Zeroing again replaces it. The reading passed in is always the uncorrected one. */
    adc_reading_t again = burst(65532, 65532);
    CHECK(adc_zero_amp_store(&again) == NULL && adc_zero_amp_mv() == 7, "second zero: %ld mV", (long)adc_zero_amp_mv());
    CHECK(adc_zero_load() == 0 && adc_zero_amp_mv() == 7, "second zero lost over a reload");

    CHECK(adc_zero_clear() == 0 && !adc_zero_amp_is_set() && adc_zero_amp_uv() == 0, "clear failed");
    CHECK(mv(adc_zero_amp_apply(open.volts)) == 10, "old reading did not come back after a clear");
    CHECK(adc_zero_load() == -1 && !adc_zero_amp_is_set(), "a cleared zero came back");
}

/* The 0 V point sits on the 16-bit wrap. A long average that straddles it must still land
   between its samples, and a zero below 0 V must be stored with its sign. */
static void test_zero_across_the_wrap(void) {
    mock_flash_reset();
    adc_zero_load();
    adc_reading_t straddle = burst(65535, 11);           /* mean count 65541 unwrapped: -1.6 mV */
    CHECK(straddle.valid && straddle.span == 12, "straddle burst: valid %d span %u", straddle.valid, (unsigned)straddle.span);
    CHECK(mv(straddle.volts) == -2, "straddle burst reads %ld mV", mv(straddle.volts));
    CHECK(adc_zero_amp_store(&straddle) == NULL, "a straddling zero was refused");
    CHECK(adc_zero_amp_uv() < -1000 && adc_zero_amp_uv() > -2200, "zero_uv %ld", (long)adc_zero_amp_uv());

    adc_reading_t neg = burst(30, 30);                   /* count 65566 unwrapped: -27 mV */
    CHECK(adc_zero_amp_store(&neg) == NULL && adc_zero_amp_mv() == -27, "negative zero: %ld mV", (long)adc_zero_amp_mv());
    CHECK(mv(adc_zero_amp_apply(neg.volts)) == 0, "negative zero not removed");
}

static void test_refusals_keep_the_old_zero(void) {
    mock_flash_reset();
    adc_zero_load();
    adc_reading_t open = burst(65529, 65529);
    CHECK(adc_zero_amp_store(&open) == NULL, "setup");
    int32_t uv = adc_zero_amp_uv();

    /* 4 mA through 249 ohm is 996 mV: a loop is connected. */
    adc_reading_t driven = burst(64548, 64548);
    const char *why = adc_zero_amp_store(&driven);
    CHECK(why && strstr(why, "996 mV") && strstr(why, "Disconnect"), "driven J8: %s", why ? why : "(stored)");

    /* Just past the limit on both sides. */
    adc_reading_t hi = burst(65484, 65484), lo = burst(60, 60);
    CHECK(mv(hi.volts) > 50 && adc_zero_amp_store(&hi) != NULL, "+%ld mV stored", mv(hi.volts));
    CHECK(mv(lo.volts) < -50 && adc_zero_amp_store(&lo) != NULL, "%ld mV stored", mv(lo.volts));

    /* A moving input averages to about 0 V here, and must still be refused. */
    adc_reading_t moving = burst(64500, 1050);
    CHECK(!moving.valid, "a 2 V pk-pk burst counted as settled");
    why = adc_zero_amp_store(&moving);
    CHECK(why && strstr(why, "not settled"), "moving input: %s", why ? why : "(stored)");

    CHECK(adc_zero_amp_store(NULL) != NULL, "NULL reading stored");
    CHECK(adc_zero_amp_uv() == uv, "a refusal changed the active zero");
    CHECK(adc_zero_load() == 0 && adc_zero_amp_uv() == uv, "a refusal changed the stored zero");
}

/* A record that says more than the limit (a future bug, a foreign writer) is not applied. */
static void test_load_rejects_out_of_range(void) {
    mock_flash_reset();
    adc_zero_load();
    static const ab_store_t store = {
        .tag = "adc-zero", .slot_off = { ADC_ZERO_SLOT_A_OFFSET, ADC_ZERO_SLOT_B_OFFSET },
        .magic = ADC_ZERO_RECORD_MAGIC, .version = ADC_ZERO_VERSION,
    };
    adc_zero_t rec = { .amp_set = 1, .amp_uv = 996000 };
    CHECK(ab_store_save(&store, &rec, sizeof(rec)) == 0, "setup");
    CHECK(adc_zero_load() == -1 && !adc_zero_amp_is_set() && adc_zero_amp_uv() == 0, "a 996 mV zero was loaded");
    rec.amp_uv = -ADC_ZERO_MAX_UV;
    CHECK(ab_store_save(&store, &rec, sizeof(rec)) == 0, "setup");
    CHECK(adc_zero_load() == 0 && adc_zero_amp_mv() == -50, "a zero at the limit was not loaded");
}

/* The store sits in its own two sectors, below the DAC limits. */
static void test_stays_in_its_sectors(void) {
    mock_flash_reset();
    adc_zero_load();
    adc_reading_t open = burst(65529, 65529);
    CHECK(adc_zero_amp_store(&open) == NULL && adc_zero_clear() == 0 && adc_zero_amp_store(&open) == NULL, "setup");
    CHECK(ADC_ZERO_SLOT_B_OFFSET == MOCK_FLASH_BASE_OFF && ADC_ZERO_SLOT_A_OFFSET == 0x1EA000u, "slot offsets moved");
    for (uint32_t off = 0x1EC000u; off < MOCK_FLASH_BASE_OFF + MOCK_FLASH_BYTES; off++)
        if (*mock_flash_ptr(off) != 0xFF) { CHECK(0, "wrote outside its sectors at 0x%lx", (unsigned long)off); break; }
}

int main(void) {
    test_none_changes_nothing();
    test_store_apply_persist();
    test_zero_across_the_wrap();
    test_refusals_keep_the_old_zero();
    test_load_rejects_out_of_range();
    test_stays_in_its_sectors();
    if (fails) { printf("test_adc_zero: %d FAILED\n", fails); return 1; }
    printf("test_adc_zero: all passed\n");
    return 0;
}
