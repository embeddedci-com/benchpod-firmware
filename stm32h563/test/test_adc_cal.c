/* Host unit test for adc_cal.c — the per-pod ADC calibration (the `calibrate` command).
 *
 * Today that is the offset of the `amp` source (J8). The cases use count 65529, about
 * +10 mV with the shared cal1 fit, as the pod's open-terminal reading. Calibrating must take
 * that offset out of every later reading by changing the fit `amp` is scaled with, survive a
 * reboot, and refuse a reading taken while something drives the terminal.
 */
#include "adc_cal.h"
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

static uint16_t buf[ADC_CAL_SAMPLES];

/* The calibration measurement: bursts alternating between two counts, scaled with the
   shared cal1 fit (what adc_cal_measure_amp hands to adc_cal_amp_store). */
static adc_reading_t burst(uint16_t lo, uint16_t hi) {
    for (size_t i = 0; i < ADC_CAL_SAMPLES; i++) buf[i] = (i & 1) ? hi : lo;
    return adc_scale_burst(buf, ADC_CAL_SAMPLES, ADC_CAL_CAL1.a, ADC_CAL_CAL1.b);
}

static long mv(float volts) { return lroundf(volts * 1000.0f); }

/* What adc_read reports for a steady count on `amp`: scaled with this pod's fit. */
static long amp_mv(uint16_t count) {
    uint16_t b16[16];
    for (size_t i = 0; i < 16; i++) b16[i] = count;
    cal_lin_t fit = adc_cal_amp_fit();
    return mv(adc_scale_burst(b16, 16, fit.a, fit.b).volts);
}

static void test_uncalibrated_changes_nothing(void) {
    mock_flash_reset();
    CHECK(adc_cal_load() == -1, "a blank pod loaded a calibration");
    CHECK(!adc_cal_amp_is_set() && adc_cal_amp_offset_uv() == 0 && adc_cal_amp_offset_mv() == 0, "blank pod has an offset");
    cal_lin_t fit = adc_cal_amp_fit();
    CHECK(fit.a == ADC_CAL_CAL1.a && fit.b == ADC_CAL_CAL1.b, "an uncalibrated pod must use the cal1 fit as is");
    CHECK(amp_mv(65529) == 10 && amp_mv(61562) == 3995, "readings changed without a calibration");
}

static void test_store_apply_persist(void) {
    mock_flash_reset();
    adc_cal_load();
    adc_reading_t open = burst(65529, 65529);
    CHECK(mv(open.volts) == 10, "open J8 at count 65529 should read 10 mV, got %ld", mv(open.volts));
    const char *why = adc_cal_amp_store(&open);
    CHECK(why == NULL, "a 10 mV offset was refused: %s", why);
    CHECK(adc_cal_amp_is_set() && adc_cal_amp_offset_mv() == 10, "offset_mv %ld", (long)adc_cal_amp_offset_mv());
    CHECK(labs((long)adc_cal_amp_offset_uv() - 10418) < 20, "offset_uv %ld", (long)adc_cal_amp_offset_uv());

    /* Open terminal now reads 0, and a real reading drops by the same offset. */
    CHECK(amp_mv(65529) == 0, "open J8 reads %ld mV after calibrating", amp_mv(65529));
    CHECK(amp_mv(61562) == 3985, "16 mA reads %ld mV", amp_mv(61562));
    /* It is the same model as cal_data.h: only `a` moves, by the offset. */
    cal_lin_t fit = adc_cal_amp_fit();
    CHECK(fit.b == ADC_CAL_CAL1.b, "the gain changed");
    CHECK(fabsf((ADC_CAL_CAL1.a - fit.a) * 1e6f - (float)adc_cal_amp_offset_uv()) < 20.0f, "fit a is not cal1 a minus the offset");

    /* Reboot: the active copy is gone, flash is not. */
    int32_t uv = adc_cal_amp_offset_uv();
    CHECK(adc_cal_load() == 0 && adc_cal_amp_is_set() && adc_cal_amp_offset_uv() == uv, "calibration lost over a reload");
    CHECK(mock_flash_double_programs == 0 && mock_flash_ecc_reads == 0, "flash misuse");

    /* Calibrating again replaces it. The reading passed in is always scaled with the shared fit. */
    adc_reading_t again = burst(65532, 65532);
    CHECK(adc_cal_amp_store(&again) == NULL && adc_cal_amp_offset_mv() == 7, "second offset: %ld mV", (long)adc_cal_amp_offset_mv());
    CHECK(adc_cal_load() == 0 && adc_cal_amp_offset_mv() == 7, "second offset lost over a reload");

    CHECK(adc_cal_clear() == 0 && !adc_cal_amp_is_set() && adc_cal_amp_offset_uv() == 0, "clear failed");
    CHECK(amp_mv(65529) == 10, "old reading did not come back after a clear");
    CHECK(adc_cal_load() == -1 && !adc_cal_amp_is_set(), "a cleared calibration came back");
}

/* The 0 V point sits on the 16-bit wrap. An average that straddles it must still land
   between its samples, and an offset below 0 V must be stored with its sign. */
static void test_offset_across_the_wrap(void) {
    mock_flash_reset();
    adc_cal_load();
    adc_reading_t straddle = burst(65535, 11);           /* mean count 65541 unwrapped: -1.6 mV */
    CHECK(straddle.valid && straddle.span == 12, "straddle burst: valid %d span %u", straddle.valid, (unsigned)straddle.span);
    CHECK(mv(straddle.volts) == -2, "straddle burst reads %ld mV", mv(straddle.volts));
    CHECK(adc_cal_amp_store(&straddle) == NULL, "a straddling offset was refused");
    CHECK(adc_cal_amp_offset_uv() < -1000 && adc_cal_amp_offset_uv() > -2200, "offset_uv %ld", (long)adc_cal_amp_offset_uv());

    adc_reading_t neg = burst(30, 30);                   /* count 65566 unwrapped: -27 mV */
    CHECK(adc_cal_amp_store(&neg) == NULL && adc_cal_amp_offset_mv() == -27, "negative offset: %ld mV", (long)adc_cal_amp_offset_mv());
    CHECK(amp_mv(30) == 0, "negative offset not removed");
}

static void test_refusals_keep_the_old_calibration(void) {
    mock_flash_reset();
    adc_cal_load();
    adc_reading_t open = burst(65529, 65529);
    CHECK(adc_cal_amp_store(&open) == NULL, "setup");
    int32_t uv = adc_cal_amp_offset_uv();

    /* 4 mA through 249 ohm is 996 mV: a loop is connected. */
    adc_reading_t driven = burst(64548, 64548);
    const char *why = adc_cal_amp_store(&driven);
    CHECK(why && strstr(why, "996 mV") && strstr(why, "Disconnect"), "driven J8: %s", why ? why : "(stored)");

    /* Just past the limit on both sides. */
    adc_reading_t hi = burst(65484, 65484), lo = burst(60, 60);
    CHECK(mv(hi.volts) > 50 && adc_cal_amp_store(&hi) != NULL, "+%ld mV stored", mv(hi.volts));
    CHECK(mv(lo.volts) < -50 && adc_cal_amp_store(&lo) != NULL, "%ld mV stored", mv(lo.volts));

    /* A moving input averages to about 0 V here, and must still be refused. */
    adc_reading_t moving = burst(64500, 1050);
    CHECK(!moving.valid, "a 2 V pk-pk burst counted as settled");
    why = adc_cal_amp_store(&moving);
    CHECK(why && strstr(why, "not settled"), "moving input: %s", why ? why : "(stored)");

    CHECK(adc_cal_amp_store(NULL) != NULL, "NULL reading stored");
    CHECK(adc_cal_amp_offset_uv() == uv, "a refusal changed the active offset");
    CHECK(adc_cal_load() == 0 && adc_cal_amp_offset_uv() == uv, "a refusal changed the stored offset");
}

/* A record that says more than the limit (a future bug, a foreign writer) is not applied. */
static void test_load_rejects_out_of_range(void) {
    mock_flash_reset();
    adc_cal_load();
    static const ab_store_t store = {
        .tag = "adc-cal", .slot_off = { ADC_CAL_SLOT_A_OFFSET, ADC_CAL_SLOT_B_OFFSET },
        .magic = ADC_CAL_RECORD_MAGIC, .version = ADC_CAL_VERSION,
    };
    adc_cal_t rec = { .amp_set = 1, .amp_offset_uv = 996000 };
    CHECK(ab_store_save(&store, &rec, sizeof(rec)) == 0, "setup");
    CHECK(adc_cal_load() == -1 && !adc_cal_amp_is_set() && adc_cal_amp_offset_uv() == 0, "a 996 mV offset was loaded");
    rec.amp_offset_uv = -ADC_CAL_MAX_OFFSET_UV;
    CHECK(ab_store_save(&store, &rec, sizeof(rec)) == 0, "setup");
    CHECK(adc_cal_load() == 0 && adc_cal_amp_offset_mv() == -50, "an offset at the limit was not loaded");
}

/* The store sits in its own two sectors, below the DAC limits. */
static void test_stays_in_its_sectors(void) {
    mock_flash_reset();
    adc_cal_load();
    adc_reading_t open = burst(65529, 65529);
    CHECK(adc_cal_amp_store(&open) == NULL && adc_cal_clear() == 0 && adc_cal_amp_store(&open) == NULL, "setup");
    CHECK(ADC_CAL_SLOT_B_OFFSET == MOCK_FLASH_BASE_OFF && ADC_CAL_SLOT_A_OFFSET == 0x1EA000u, "slot offsets moved");
    for (uint32_t off = 0x1EC000u; off < MOCK_FLASH_BASE_OFF + MOCK_FLASH_BYTES; off++)
        if (*mock_flash_ptr(off) != 0xFF) { CHECK(0, "wrote outside its sectors at 0x%lx", (unsigned long)off); break; }
}

int main(void) {
    test_uncalibrated_changes_nothing();
    test_store_apply_persist();
    test_offset_across_the_wrap();
    test_refusals_keep_the_old_calibration();
    test_load_rejects_out_of_range();
    test_stays_in_its_sectors();
    if (fails) { printf("test_adc_cal: %d FAILED\n", fails); return 1; }
    printf("test_adc_cal: all passed\n");
    return 0;
}
