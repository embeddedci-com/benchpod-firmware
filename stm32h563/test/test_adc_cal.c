/* Host unit test for adc_cal.c — the per-pod ADC calibration (the `calibrate` command).
 *
 * Today that is the offset of the `current_in` source (J8). The cases use count 65529, about
 * +10 mV with the shared cal1 fit, as the pod's open-terminal reading. Calibrating must take
 * that offset out of every later reading by changing the fit `current_in` is scaled with, survive a
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
   shared cal1 fit (what adc_cal_measure_current_in hands to adc_cal_current_in_store). */
static adc_reading_t burst(uint16_t lo, uint16_t hi) {
    for (size_t i = 0; i < ADC_CAL_SAMPLES; i++) buf[i] = (i & 1) ? hi : lo;
    return adc_scale_burst(buf, ADC_CAL_SAMPLES, ADC_CAL_CAL1.a, ADC_CAL_CAL1.b);
}

static long mv(float volts) { return lroundf(volts * 1000.0f); }

/* The unwrapped count, in milli-counts, where the general cal1 fit reads `volts`. */
static int32_t count_mc_for_cal1(double volts) {
    return (int32_t)lround((volts - ADC_CAL_CAL1.a) / ADC_CAL_CAL1.b * 1000.0);
}

/* What adc_read reports for a steady count on `current_in`: scaled with this pod's fit. */
static long current_in_mv(uint16_t count) {
    uint16_t b16[16];
    for (size_t i = 0; i < 16; i++) b16[i] = count;
    cal_lin_t fit = adc_cal_current_in_fit();
    return mv(adc_scale_burst(b16, 16, fit.a, fit.b).volts);
}

static void test_uncalibrated_changes_nothing(void) {
    mock_flash_reset();
    CHECK(adc_cal_load() == -1, "a blank pod loaded a calibration");
    CHECK(!adc_cal_current_in_is_set() && adc_cal_current_in_offset_uv() == 0 && adc_cal_current_in_offset_mv() == 0, "blank pod has an offset");
    cal_lin_t fit = adc_cal_current_in_fit();
    CHECK(fit.a == ADC_CAL_CAL1.a && fit.b == ADC_CAL_CAL1.b, "an uncalibrated pod must use the cal1 fit as is");
    CHECK(current_in_mv(65529) == 10 && current_in_mv(61562) == 3995, "readings changed without a calibration");
}

static void test_store_apply_persist(void) {
    mock_flash_reset();
    adc_cal_load();
    adc_reading_t open = burst(65529, 65529);
    CHECK(mv(open.volts) == 10, "open J8 at count 65529 should read 10 mV, got %ld", mv(open.volts));
    const char *why = adc_cal_current_in_store(&open);
    CHECK(why == NULL, "a 10 mV offset was refused: %s", why);
    CHECK(adc_cal_current_in_is_set() && adc_cal_current_in_offset_mv() == 10, "offset_mv %ld", (long)adc_cal_current_in_offset_mv());
    CHECK(labs((long)adc_cal_current_in_offset_uv() - 10418) < 20, "offset_uv %ld", (long)adc_cal_current_in_offset_uv());

    /* Open terminal now reads 0, and a real reading drops by the same offset. */
    CHECK(current_in_mv(65529) == 0, "open J8 reads %ld mV after calibrating", current_in_mv(65529));
    CHECK(current_in_mv(61562) == 3985, "16 mA reads %ld mV", current_in_mv(61562));
    /* It is the same model as cal_data.h: only `a` moves, by the offset. */
    cal_lin_t fit = adc_cal_current_in_fit();
    CHECK(fit.b == ADC_CAL_CAL1.b, "the gain changed");
    CHECK(fabsf((ADC_CAL_CAL1.a - fit.a) * 1e6f - (float)adc_cal_current_in_offset_uv()) < 20.0f, "fit a is not cal1 a minus the offset");

    /* Reboot: the active copy is gone, flash is not. */
    int32_t uv = adc_cal_current_in_offset_uv();
    CHECK(adc_cal_load() == 0 && adc_cal_current_in_is_set() && adc_cal_current_in_offset_uv() == uv, "calibration lost over a reload");
    CHECK(mock_flash_double_programs == 0 && mock_flash_ecc_reads == 0, "flash misuse");

    /* Calibrating again replaces it. The reading passed in is always scaled with the shared fit. */
    adc_reading_t again = burst(65532, 65532);
    CHECK(adc_cal_current_in_store(&again) == NULL && adc_cal_current_in_offset_mv() == 7, "second offset: %ld mV", (long)adc_cal_current_in_offset_mv());
    CHECK(adc_cal_load() == 0 && adc_cal_current_in_offset_mv() == 7, "second offset lost over a reload");

    CHECK(adc_cal_clear() == 0 && !adc_cal_current_in_is_set() && adc_cal_current_in_offset_uv() == 0, "clear failed");
    CHECK(current_in_mv(65529) == 10, "old reading did not come back after a clear");
    CHECK(adc_cal_load() == -1 && !adc_cal_current_in_is_set(), "a cleared calibration came back");
}

/* adc_read reports the loop current too: volts across the 249 ohm sense resistor, in uA. */
static void test_loop_current(void) {
    CHECK(current_in_ua(0.996f) == 4000, "4 mA: %ld uA", current_in_ua(0.996f));
    CHECK(current_in_ua(4.980f) == 20000, "20 mA: %ld uA", current_in_ua(4.980f));
    CHECK(current_in_ua(0.0f) == 0 && current_in_ua(-0.001f) == -4, "zero and below: %ld uA", current_in_ua(-0.001f));
}

/* The 0 V point sits on the 16-bit wrap. An average that straddles it must still land
   between its samples, and an offset below 0 V must be stored with its sign. */
static void test_offset_across_the_wrap(void) {
    mock_flash_reset();
    adc_cal_load();
    adc_reading_t straddle = burst(65535, 11);           /* mean count 65541 unwrapped: -1.6 mV */
    CHECK(straddle.valid && straddle.span == 12, "straddle burst: valid %d span %u", straddle.valid, (unsigned)straddle.span);
    CHECK(mv(straddle.volts) == -2, "straddle burst reads %ld mV", mv(straddle.volts));
    CHECK(adc_cal_current_in_store(&straddle) == NULL, "a straddling offset was refused");
    CHECK(adc_cal_current_in_offset_uv() < -1000 && adc_cal_current_in_offset_uv() > -2200, "offset_uv %ld", (long)adc_cal_current_in_offset_uv());

    adc_reading_t neg = burst(30, 30);                   /* count 65566 unwrapped: -27 mV */
    CHECK(adc_cal_current_in_store(&neg) == NULL && adc_cal_current_in_offset_mv() == -27, "negative offset: %ld mV", (long)adc_cal_current_in_offset_mv());
    CHECK(current_in_mv(30) == 0, "negative offset not removed");
}

static void test_refusals_keep_the_old_calibration(void) {
    mock_flash_reset();
    adc_cal_load();
    adc_reading_t open = burst(65529, 65529);
    CHECK(adc_cal_current_in_store(&open) == NULL, "setup");
    int32_t uv = adc_cal_current_in_offset_uv();

    /* 4 mA through 249 ohm is 996 mV: a loop is connected. */
    adc_reading_t driven = burst(64548, 64548);
    const char *why = adc_cal_current_in_store(&driven);
    CHECK(why && strstr(why, "996 mV") && strstr(why, "Disconnect"), "driven J8: %s", why ? why : "(stored)");

    /* Just past the limit on both sides. */
    adc_reading_t hi = burst(65385, 65385), lo = burst(160, 160);
    CHECK(mv(hi.volts) > 150 && adc_cal_current_in_store(&hi) != NULL, "+%ld mV stored", mv(hi.volts));
    CHECK(mv(lo.volts) < -150 && adc_cal_current_in_store(&lo) != NULL, "%ld mV stored", mv(lo.volts));

    /* A moving input averages to about 0 V here, and must still be refused. */
    adc_reading_t moving = burst(64500, 1050);
    CHECK(!moving.valid, "a 2 V pk-pk burst counted as settled");
    why = adc_cal_current_in_store(&moving);
    CHECK(why && strstr(why, "not settled"), "moving input: %s", why ? why : "(stored)");

    CHECK(adc_cal_current_in_store(NULL) != NULL, "NULL reading stored");
    CHECK(adc_cal_current_in_offset_uv() == uv, "a refusal changed the active offset");
    CHECK(adc_cal_load() == 0 && adc_cal_current_in_offset_uv() == uv, "a refusal changed the stored offset");
}

/* A record that says more than the limit (a future bug, a foreign writer) is not applied. */
static void test_load_rejects_out_of_range(void) {
    mock_flash_reset();
    adc_cal_load();
    static const ab_store_t store = {
        .tag = "adc-cal", .slot_off = { ADC_CAL_SLOT_A_OFFSET, ADC_CAL_SLOT_B_OFFSET },
        .magic = ADC_CAL_RECORD_MAGIC, .version = ADC_CAL_VERSION,
    };
    adc_cal_t rec = { .zero_set = 1, .zero_count_mc = count_mc_for_cal1(0.996) };
    CHECK(ab_store_save(&store, &rec, sizeof(rec)) == 0, "setup");
    CHECK(adc_cal_load() == -1 && !adc_cal_current_in_is_set() && adc_cal_current_in_offset_uv() == 0, "a 996 mV zero was loaded");
    rec.zero_count_mc = count_mc_for_cal1(-0.1499);
    CHECK(ab_store_save(&store, &rec, sizeof(rec)) == 0, "setup");
    CHECK(adc_cal_load() == 0 && adc_cal_current_in_offset_mv() == -150, "a zero at the limit was not loaded");
}

/* benchpod-445e46 (2026-10-19): J8 open reads count 49, -50 mV, a normal part spread for this
   front end. It is stored, and the offset comes out of EVERY source's fit: they share the chain
   from R90 to the ADC, so an open SMA (ext) and the cal loopbacks read true afterwards. */
static void test_offset_applies_to_every_source(void) {
    mock_flash_reset();
    adc_cal_load();
    cal_lin_t ext0 = adc_cal_fit(ADC_CAL_EXT), cal2_0 = adc_cal_fit(ADC_CAL_CAL2);
    CHECK(ext0.a == ADC_CAL_EXT.a && cal2_0.a == ADC_CAL_CAL2.a, "an uncalibrated pod changed a fit");
    adc_reading_t open = burst(49, 49);
    CHECK(mv(open.volts) <= -40 && mv(open.volts) >= -60, "J8 open at count 49 reads %ld mV", mv(open.volts));
    const char *why = adc_cal_current_in_store(&open);
    CHECK(why == NULL, "a -50 mV offset was refused: %s", why ? why : "");
    float off = (float)adc_cal_current_in_offset_uv() / 1e6f;
    cal_lin_t fits[3] = { ADC_CAL_EXT, ADC_CAL_CAL1, ADC_CAL_CAL2 };
    for (int i = 0; i < 3; i++) {
        cal_lin_t f = adc_cal_fit(fits[i]);
        CHECK(f.b == fits[i].b, "source %d: the gain changed", i);
        /* a is about 65.7 V (0 V sits at the 16-bit wrap), so float rounding is ~8 uV */
        CHECK(fabsf((fits[i].a - f.a) - off) < 20e-6f, "source %d: offset not taken out of a", i);
    }
    cal_lin_t ci = adc_cal_current_in_fit();
    CHECK(ci.a == adc_cal_fit(ADC_CAL_CAL1).a && ci.b == ADC_CAL_CAL1.b, "current_in is not the cal1 fit with the offset");
}

/* The store sits in its own two sectors, below the DAC limits. */
static void test_stays_in_its_sectors(void) {
    mock_flash_reset();
    adc_cal_load();
    adc_reading_t open = burst(65529, 65529);
    CHECK(adc_cal_current_in_store(&open) == NULL && adc_cal_clear() == 0 && adc_cal_current_in_store(&open) == NULL, "setup");
    CHECK(ADC_CAL_SLOT_B_OFFSET == 0x1E8000u && ADC_CAL_SLOT_A_OFFSET == 0x1EA000u, "slot offsets moved");
    for (uint32_t off = MOCK_FLASH_BASE_OFF; off < MOCK_FLASH_BASE_OFF + MOCK_FLASH_BYTES; off++) {
        if (off == ADC_CAL_SLOT_B_OFFSET) { off = 0x1EC000u - 1u; continue; }   /* its two sectors */
        if (*mock_flash_ptr(off) != 0xFF) { CHECK(0, "wrote outside its sectors at 0x%lx", (unsigned long)off); break; }
    }
}

/* The compiled-in fits are per board revision (cal_data.c). rev3 has its own, anything else
   gets v2, and `current_in` follows whichever set is selected. Readings from pod .220 (rev3):
   the 5 V output at code 251 measured 4906.6 mV on the DMM and count 60642 through cal1. */
static void test_revision_selects_the_fits(void) {
    mock_flash_reset();
    adc_cal_load();
    cal_lin_t v2_cal1 = ADC_CAL_CAL1, v2_dac5 = DAC_CAL[1];

    cal_data_select(3);
    CHECK(ADC_CAL_CAL1.a != v2_cal1.a && DAC_CAL[1].b != v2_dac5.b, "rev3 selected the v2 fits");
    CHECK(labs(mv(DAC_CAL[1].a + DAC_CAL[1].b * 251.0f) - 4907) <= 1, "rev3 5 V DAC at code 251: %ld mV, DMM read 4907", mv(DAC_CAL[1].a + DAC_CAL[1].b * 251.0f));
    CHECK(labs(mv(DAC_CAL[2].a + DAC_CAL[2].b * 6.0f) + 11520) <= 2, "rev3 12 V DAC at code 6: %ld mV, DMM read -11520", mv(DAC_CAL[2].a + DAC_CAL[2].b * 6.0f));
    CHECK(labs(current_in_mv(60642) - 4907) <= 2, "rev3 current_in at count 60642: %ld mV, want 4907", current_in_mv(60642));
    cal_lin_t fit = adc_cal_current_in_fit();
    CHECK(fit.a == ADC_CAL_CAL1.a && fit.b == ADC_CAL_CAL1.b, "current_in must follow the selected cal1 fit");

    cal_data_select(2);
    CHECK(ADC_CAL_CAL1.a == v2_cal1.a && DAC_CAL[1].b == v2_dac5.b, "v2 did not get its fits back");
    cal_data_select(0);
    CHECK(ADC_CAL_CAL1.a == v2_cal1.a, "an unknown revision must fall back to v2");
}


/* ---- The DMM parts (version 2) ------------------------------------------------------------ */

static const ab_store_t s_v1_store = {
    .tag = "adc-cal", .slot_off = { ADC_CAL_SLOT_A_OFFSET, ADC_CAL_SLOT_B_OFFSET },
    .magic = ADC_CAL_RECORD_MAGIC, .version = ADC_CAL_VERSION_1,
};

/* A rev3 pod with the J8 offset of benchpod-445e46 (count 49, about -50 mV). */
static void fresh_rev3_pod(bool with_offset) {
    mock_flash_reset();
    cal_data_select(3);
    adc_cal_load();
    if (with_offset) {
        adc_reading_t open = burst(49, 49);
        CHECK(adc_cal_current_in_store(&open) == NULL, "setup: offset");
    }
}

/* A pod that last saved on firmware 3.4.0 to 3.9.3 has a version-1 record. Its offset loads;
   nothing is written until the next save, which writes version 2 and keeps the offset. */
static void test_version1_record_migrates(void) {
    mock_flash_reset();
    cal_data_select(3);
    adc_cal_v1_t v1 = { .current_in_set = 1, .current_in_offset_uv = -50210 };
    CHECK(ab_store_save(&s_v1_store, &v1, sizeof(v1)) == 0, "setup");
    CHECK(adc_cal_load() == 0 && adc_cal_current_in_offset_uv() == -50210, "v1 offset not loaded: %ld", (long)adc_cal_current_in_offset_uv());
    adc_cal_v1_t back;
    CHECK(ab_store_load(&s_v1_store, &back, sizeof(back)) == 0 && back.current_in_offset_uv == -50210, "loading rewrote the v1 record");
    CHECK(!adc_cal_dac_is_set(0) && !adc_cal_dac_is_set(1) && !adc_cal_dac_is_set(2) && !adc_cal_gain_is_set(0), "v1 brought DMM parts");

    /* The next save is version 2, with the offset kept. */
    uint16_t c1 = dac_code16_for_volts(1, 1.0f), c2 = dac_code16_for_volts(1, 4.0f);
    CHECK(adc_cal_dac_store(1, c1, 995900, c2, 3992900) == NULL, "dac store after migration");
    /* The newest record is the version-1 copy for older firmware, with the same offset. */
    CHECK(ab_store_load(&s_v1_store, &back, sizeof(back)) == 0 && labs((long)back.current_in_offset_uv + 50210) <= 1,
          "no version-1 copy next to the version-2 record");
    CHECK(adc_cal_load() == 0 && adc_cal_current_in_offset_uv() == -50210 && adc_cal_dac_is_set(1), "v2 lost the offset or the line");

    /* A v1 record without an offset is nothing. */
    mock_flash_reset();
    adc_cal_v1_t none = {0};
    CHECK(ab_store_save(&s_v1_store, &none, sizeof(none)) == 0 && adc_cal_load() == -1, "an empty v1 record loaded");
}

/* benchpod-445e46, 5 V output: asked 1.000 / 4.000 V, the DMM read 0.9959 / 3.9929 V. After
   storing those, the pod's line goes through both readings, every DAC user converts with it,
   and asking 1.000 V again picks the code that gives 1.000 V. */
static void test_dac_line_from_two_readings(void) {
    fresh_rev3_pod(false);
    cal_lin_t comp = DAC_CAL_COMPILED[1];
    uint16_t c1 = dac_code16_for_volts(1, 1.0f), c2 = dac_code16_for_volts(1, 4.0f);
    CHECK(adc_cal_dac_store(1, c1, 995900, c2, 3992900) == NULL, "445e46 5 V readings refused");
    CHECK(adc_cal_dac_is_set(1) && !adc_cal_dac_is_set(0) && !adc_cal_dac_is_set(2), "wrong path stored");
    CHECK(labs(lroundf(dac_volts_for_code16(1, c1) * 1e6f) - 995900) <= 30 &&
          labs(lroundf(dac_volts_for_code16(1, c2) * 1e6f) - 3992900) <= 30, "the line misses the readings");
    uint16_t again = dac_code16_for_volts(1, 1.0f);
    CHECK(again > c1 && labs(lroundf(dac_volts_for_code16(1, again) * 1e6f) - 1000000) <= 80,
          "1.000 V now gives %ld uV at code %u", lroundf(dac_volts_for_code16(1, again) * 1e6f), (unsigned)again);
    double gain_ppm = ((double)DAC_CAL[1].b / comp.b - 1.0) * 1e6;
    CHECK(gain_ppm < -900 && gain_ppm > -1100, "gain %ld ppm, want about -1000", lround(gain_ppm));
    CHECK(DAC_CAL[0].a == DAC_CAL_COMPILED[0].a && DAC_CAL[2].b == DAC_CAL_COMPILED[2].b, "other paths changed");

    /* +-12 V: asked -5 / +5 V, read -5.0255 / 4.980 V. */
    uint16_t m5 = dac_code16_for_volts(2, -5.0f), p5 = dac_code16_for_volts(2, 5.0f);
    CHECK(adc_cal_dac_store(2, m5, -5025500, p5, 4980000) == NULL, "445e46 12 V readings refused");
    CHECK(labs(lroundf(dac_volts_for_code16(2, dac_code16_for_volts(2, 0.0f)) * 1e6f)) <= 400, "0 V on 12v is off");

    /* Reboot keeps both; clearing the DAC part goes back to the compiled lines only. */
    cal_lin_t five = DAC_CAL[1];
    CHECK(adc_cal_load() == 0 && DAC_CAL[1].a == five.a && DAC_CAL[1].b == five.b, "line lost over a reload");
    CHECK(adc_cal_clear_part("dac") == 0 && DAC_CAL[1].b == comp.b && !adc_cal_dac_is_set(1), "clear dac");
    CHECK(adc_cal_load() == -1, "a cleared (empty) calibration loaded");
    CHECK(adc_cal_clear_part("nonsense") == -2, "unknown part accepted");
}

static void test_dac_refusals(void) {
    fresh_rev3_pod(false);
    uint16_t c1 = dac_code16_for_volts(1, 1.0f), c2 = dac_code16_for_volts(1, 4.0f);
    const char *why = adc_cal_dac_store(1, c1, 995900, (uint16_t)(c1 + 1000), 1070000);
    CHECK(why && strstr(why, "at least"), "close codes: %s", why ? why : "(stored)");
    why = adc_cal_dac_store(1, c1, 995900, c2, 12000);                  /* DMM still on the wrong SMA */
    CHECK(why && strstr(why, "reads 12 mV") && strstr(why, "5v SMA"), "wrong SMA: %s", why ? why : "(stored)");
    why = adc_cal_dac_store(1, c1, 1090000, c2, 3910000);               /* each within tolerance, slope -6 % */
    CHECK(why && strstr(why, "gain"), "slope: %s", why ? why : "(stored)");
    why = adc_cal_dac_store(3, c1, 995900, c2, 3992900);
    CHECK(why != NULL, "path 3 stored");
    CHECK(!adc_cal_dac_is_set(1) && DAC_CAL[1].b == DAC_CAL_COMPILED[1].b, "a refusal changed the line");
    CHECK(adc_cal_load() == -1, "a refusal saved something");
}

/* The ADC gain: measured against a DMM-calibrated output, after the offset. cal1 reads 0.18 %
   low here; the gain takes it out of cal1 and current_in, and nothing else. */
static void test_adc_gain(void) {
    fresh_rev3_pod(false);
    CHECK(adc_cal_gain_ready(ADC_CAL_SRC_CAL1) && strstr(adc_cal_gain_ready(ADC_CAL_SRC_CAL1), "offset"), "gain without the offset");
    adc_reading_t open = burst(49, 49);
    CHECK(adc_cal_current_in_store(&open) == NULL, "setup");
    const char *why = adc_cal_gain_ready(ADC_CAL_SRC_CAL1);
    CHECK(why && strstr(why, "5v output"), "gain without the DAC line: %s", why ? why : "(ready)");
    CHECK(adc_cal_gain_ready(7) != NULL, "unknown source ready");
    uint16_t c1 = dac_code16_for_volts(1, 1.0f), c2 = dac_code16_for_volts(1, 4.0f);
    CHECK(adc_cal_dac_store(1, c1, 995900, c2, 3992900) == NULL, "setup: 5v line");
    CHECK(adc_cal_gain_ready(ADC_CAL_SRC_CAL1) == NULL, "cal1 not ready");
    CHECK(adc_cal_gain_ready(ADC_CAL_SRC_EXT) != NULL, "ext ready without the 12v line");

    cal_lin_t ext_before = adc_cal_source_fit(ADC_CAL_SRC_EXT), cal1_before = adc_cal_source_fit(ADC_CAL_SRC_CAL1);
    /* What cal1 must read (offset removed, no gain) for a 0.18 % low gain and 1 mV left over. */
    adc_cal_gain_meas_t m = { .v = { 0.9959f, 3.9929f } };
    float off = (float)adc_cal_current_in_offset_uv() / 1e6f;
    for (int i = 0; i < 2; i++)
        m.count[i] = (float)(count_mc_for_cal1(m.v[i] / 1.0018 + 0.001 + off) / 1000.0);
    CHECK(adc_cal_gain_store(ADC_CAL_SRC_CAL1, &m) == NULL, "a 0.18 %% gain was refused");
    CHECK(fabsf(m.gain - 1.0018f) < 2e-5f && adc_cal_gain_is_set(ADC_CAL_SRC_CAL1), "gain %f", (double)m.gain);
    CHECK(fabsf(m.residual_v + 0.001f) < 5e-5f, "residual %f V, want about -1 mV", (double)m.residual_v);
    cal_lin_t cal1 = adc_cal_source_fit(ADC_CAL_SRC_CAL1);
    CHECK(fabsf(cal1.b / cal1_before.b - 1.0018f) < 2e-5f && fabsf(cal1.a / cal1_before.a - 1.0018f) < 2e-5f, "cal1 fit not scaled");
    cal_lin_t ci = adc_cal_current_in_fit();
    CHECK(ci.a == cal1.a && ci.b == cal1.b, "current_in does not follow cal1's gain");
    cal_lin_t ext = adc_cal_source_fit(ADC_CAL_SRC_EXT);
    CHECK(ext.a == ext_before.a && ext.b == ext_before.b, "ext changed with cal1's gain");
    /* 0 V stays 0 V: the count the offset reads as zero still reads zero. */
    CHECK(labs(current_in_mv(49)) <= 1, "open J8 reads %ld mV after the gain", current_in_mv(49));

    /* Persisted; a gain 5 % off is refused and keeps the old one. */
    CHECK(adc_cal_load() == 0 && fabsf(adc_cal_gain(ADC_CAL_SRC_CAL1) - 1.0018f) < 2e-5f, "gain lost over a reload");
    adc_cal_gain_meas_t bad = { .v = { 0.9959f, 3.9929f } };
    for (int i = 0; i < 2; i++) bad.count[i] = (float)(count_mc_for_cal1(bad.v[i] / 1.05 + off) / 1000.0);
    why = adc_cal_gain_store(ADC_CAL_SRC_CAL1, &bad);
    CHECK(why && strstr(why, "5 V output"), "5 %% gain: %s", why ? why : "(stored)");
    CHECK(fabsf(adc_cal_gain(ADC_CAL_SRC_CAL1) - 1.0018f) < 2e-5f, "a refused gain replaced the old one");

    /* Clearing the gains keeps the offset and the line; clearing the offset keeps the gains. */
    CHECK(adc_cal_clear_part("offset") == 0 && !adc_cal_current_in_is_set() && adc_cal_gain_is_set(ADC_CAL_SRC_CAL1) && adc_cal_dac_is_set(1), "clear offset");
    CHECK(adc_cal_clear_part("adc") == 0 && !adc_cal_gain_is_set(ADC_CAL_SRC_CAL1) && adc_cal_dac_is_set(1), "clear adc");
    CHECK(adc_cal_load() == 0 && adc_cal_dac_is_set(1) && !adc_cal_current_in_is_set(), "parts after reload");
    CHECK(adc_cal_clear() == 0 && !adc_cal_dac_is_set(1) && DAC_CAL[1].b == DAC_CAL_COMPILED[1].b, "clear all");
}

/* A version-2 record with a DAC line or a gain out of range is ignored as a whole. */
static void test_load_rejects_bad_dmm_parts(void) {
    static const ab_store_t store = {
        .tag = "adc-cal", .slot_off = { ADC_CAL_SLOT_A_OFFSET, ADC_CAL_SLOT_B_OFFSET },
        .magic = ADC_CAL_RECORD_MAGIC, .version = ADC_CAL_VERSION,
    };
    fresh_rev3_pod(false);
    uint16_t c1 = dac_code16_for_volts(1, 1.0f), c2 = dac_code16_for_volts(1, 4.0f);
    adc_cal_t rec = { .zero_set = 1, .zero_count_mc = 65585000, .dac_set = 2 };
    rec.dac[1] = (adc_cal_dac_pts_t){ { c1, c2 }, { 1000000, 4150000 } };   /* 5 % gain */
    CHECK(ab_store_save(&store, &rec, sizeof(rec)) == 0, "setup");
    CHECK(adc_cal_load() == -1 && !adc_cal_current_in_is_set() && DAC_CAL[1].b == DAC_CAL_COMPILED[1].b, "a 5 %% DAC line loaded");
    rec.dac_set = 0;
    rec.gain_set = 1;
    rec.gain[0] = (adc_cal_gain_pts_t){ { 61000000, 61000000 }, { 1000000, 4000000 } };   /* same count twice */
    CHECK(ab_store_save(&store, &rec, sizeof(rec)) == 0 && adc_cal_load() == -1, "a gain from one count loaded");
    rec.gain[0] = (adc_cal_gain_pts_t){ { count_mc_for_cal1(1.0), count_mc_for_cal1(4.0) }, { 1001000, 4004000 } };
    CHECK(ab_store_save(&store, &rec, sizeof(rec)) == 0 && adc_cal_load() == 0 && adc_cal_gain_is_set(0) &&
          fabsf(adc_cal_gain(0) - 1.001f) < 2e-5f, "a good record did not load: gain %f", (double)adc_cal_gain(0));
}

/* The record holds the zero COUNT, not an offset on a fit: when the general fit changes (here
   rev3's set swapped for v2's), J8 open still reads 0 and the offset follows the new fit. */
static void test_new_general_fit_keeps_the_zero(void) {
    fresh_rev3_pod(true);                              /* zero at count 49 (65585 unwrapped) */
    CHECK(current_in_mv(49) == 0, "rev3: J8 open reads %ld mV", current_in_mv(49));
    long off_rev3 = (long)adc_cal_current_in_offset_uv();
    cal_data_select(2);                                /* another general fit */
    CHECK(adc_cal_load() == 0 && adc_cal_zero_count_mc() == 65585000, "zero count changed: %ld", (long)adc_cal_zero_count_mc());
    CHECK(current_in_mv(49) == 0, "v2 fit: J8 open reads %ld mV", current_in_mv(49));
    CHECK(labs((long)adc_cal_current_in_offset_uv() - off_rev3) > 1000, "the offset did not follow the new fit");
    cal_data_select(3);
    adc_cal_load();
}

/* 3.9.3 stored the offset in uV (version 1); this firmware converts it to the zero count through
   the fit it was measured with. bca439 (+12.8 mV) and 445e46 (-50.575 mV) then read every source
   exactly as 3.9.3 did: its fit minus that offset. */
static void test_v1_converts_to_the_same_readings(void) {
    static const int32_t offsets_uv[2] = { 12800, -50575 };
    static const uint16_t counts[] = { 30, 49, 65521, 64548, 61562, 40000, 20000 };
    for (int k = 0; k < 2; k++) {
        mock_flash_reset();
        cal_data_select(3);
        adc_cal_v1_t v1 = { .current_in_set = 1, .current_in_offset_uv = offsets_uv[k] };
        CHECK(ab_store_save(&s_v1_store, &v1, sizeof(v1)) == 0 && adc_cal_load() == 0, "setup %d", k);
        CHECK(labs((long)adc_cal_current_in_offset_uv() - offsets_uv[k]) <= 1, "offset %ld uV, 3.9.3 had %ld",
              (long)adc_cal_current_in_offset_uv(), (long)offsets_uv[k]);
        cal_lin_t base[3] = { ADC_CAL_CAL1, ADC_CAL_CAL2, ADC_CAL_EXT };
        for (int src = 0; src < 3; src++) {
            cal_lin_t mine = adc_cal_source_fit(src);
            for (size_t i = 0; i < sizeof(counts) / sizeof(counts[0]); i++) {
                double c = counts[i] < 32768 ? counts[i] + 65536.0 : counts[i];
                double was = base[src].a - offsets_uv[k] / 1e6 + base[src].b * c;
                double now = (double)mine.a + (double)mine.b * c;
                CHECK(fabs(now - was) < 20e-6, "pod %d source %d count %u: %.6f V, 3.9.3 read %.6f V", k, src,
                      (unsigned)counts[i], now, was);
            }
        }
    }
}

/* Every save also leaves a version-1 record that firmware 3.4.0 to 3.9.3 reads (the newest one,
   version 1 only), so a downgraded pod keeps its offset. What that firmware then writes or
   clears wins over the version-2 record. */
static void test_downgrade_keeps_the_offset(void) {
    fresh_rev3_pod(true);
    uint16_t c1 = dac_code16_for_volts(1, 1.0f), c2 = dac_code16_for_volts(1, 4.0f);
    CHECK(adc_cal_dac_store(1, c1, 995900, c2, 3992900) == NULL, "setup: 5v line");
    int32_t uv = adc_cal_current_in_offset_uv();

    /* Old firmware: ab_store_load of the newest record, version 1, 32 bytes. */
    adc_cal_v1_t old;
    CHECK(ab_store_load(&s_v1_store, &old, sizeof(old)) == 0 && old.current_in_set &&
          labs((long)old.current_in_offset_uv - uv) <= 1, "old firmware cannot read the offset");
    /* This firmware still takes the version-2 record behind it. */
    CHECK(adc_cal_load() == 0 && adc_cal_dac_is_set(1), "the version-2 record was not taken");

    /* The downgraded pod calibrates again (old firmware writes a newer version 1). */
    adc_cal_v1_t again = { .current_in_set = 1, .current_in_offset_uv = -40000 };
    CHECK(ab_store_save(&s_v1_store, &again, sizeof(again)) == 0, "setup");
    CHECK(adc_cal_load() == 0 && labs((long)adc_cal_current_in_offset_uv() + 40000) <= 1 && !adc_cal_dac_is_set(1),
          "the old firmware's calibration lost to a stale version-2 record");

    /* ... or clears it. */
    fresh_rev3_pod(true);
    CHECK(adc_cal_dac_store(1, c1, 995900, c2, 3992900) == NULL, "setup");
    static const ab_store_t old_store = {
        .tag = "adc-cal", .slot_off = { ADC_CAL_SLOT_A_OFFSET, ADC_CAL_SLOT_B_OFFSET },
        .magic = ADC_CAL_RECORD_MAGIC, .version = ADC_CAL_VERSION_1,
    };
    CHECK(ab_store_clear(&old_store) == 0 && adc_cal_load() == -1, "a clear on old firmware did not stick");

    /* A power cut between the two writes leaves the version-2 record newest: still taken. */
    fresh_rev3_pod(false);
    static const ab_store_t v2_store = {
        .tag = "adc-cal", .slot_off = { ADC_CAL_SLOT_A_OFFSET, ADC_CAL_SLOT_B_OFFSET },
        .magic = ADC_CAL_RECORD_MAGIC, .version = ADC_CAL_VERSION,
    };
    adc_cal_t only = { .zero_set = 1, .zero_count_mc = 65585000 };
    CHECK(ab_store_save(&v2_store, &only, sizeof(only)) == 0 && adc_cal_load() == 0 && adc_cal_zero_count_mc() == 65585000,
          "a lone version-2 record was not taken");
}

int main(void) {
    test_uncalibrated_changes_nothing();
    test_store_apply_persist();
    test_offset_across_the_wrap();
    test_loop_current();
    test_refusals_keep_the_old_calibration();
    test_load_rejects_out_of_range();
    test_offset_applies_to_every_source();
    test_stays_in_its_sectors();
    test_revision_selects_the_fits();
    test_version1_record_migrates();
    test_dac_line_from_two_readings();
    test_dac_refusals();
    test_adc_gain();
    test_load_rejects_bad_dmm_parts();
    test_new_general_fit_keeps_the_zero();
    test_v1_converts_to_the_same_readings();
    test_downgrade_keeps_the_offset();
    if (fails) { printf("test_adc_cal: %d FAILED\n", fails); return 1; }
    printf("test_adc_cal: all passed\n");
    return 0;
}
