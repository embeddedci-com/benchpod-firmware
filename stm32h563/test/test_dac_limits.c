/* Host unit test for dac_limits.c — the DAC output limits for an external output stage.
 *
 * The case that matters: an INVERTED solar simulator on the 5v path, where 0 V on the DAC is
 * ~45 V out and the DAC must stay inside 1.85..3.60 V. Every command that could leave its input
 * near 0 V (a raw write, a route that disconnects it, a loop clamp outside the window, a trip)
 * must be refused, whichever way it arrives; everything else must pass untouched, and nothing
 * may change on a pod without limits.
 */
#include "dac_limits.h"
#include "cal_data.h"
#include "config_store.h"
#include "mocks/mock_flash.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static int fails = 0;
#define CHECK(cond, ...) do { if (!(cond)) { \
        printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

static bool refused(const char *cmd, const char *json, const char *want) {
    const char *why = dac_limits_check_command(cmd, json);
    if (!want) return why != NULL;
    return why && strstr(why, want);
}
static bool allowed(const char *cmd, const char *json) { return dac_limits_check_command(cmd, json) == NULL; }

static const dac_limits_t SOLAR = { .enabled = 1, .path = 1, .inverted = 1, .min_mv = 1850, .max_mv = 3600 };

/* The 16-bit loop code for a DAC voltage on the 5V path, as the page computes it (nominal 0..5 V). */
static long page_code(double v) { return lround(v / 5.0 * 65535.0); }

static void test_no_limits_changes_nothing(void) {
    dac_limits_set_active(NULL);
    CHECK(allowed("generate", "{\"cmd\":\"generate\"}"), "generate refused without limits");
    CHECK(allowed("analog_path", "{\"cmd\":\"analog_path\",\"path\":\"off\"}"), "path off refused without limits");
    CHECK(allowed("dac_out", "{\"cmd\":\"dac_out\",\"path\":\"5v\",\"volts\":0}"), "dac_out 0 refused without limits");
    CHECK(allowed("dac_control_loop", "{\"cmd\":\"dac_control_loop\",\"vmin\":0,\"vmax\":65535,\"in_trip\":5}"),
          "loop refused without limits");
    CHECK(allowed("current_out", "{\"cmd\":\"current_out\",\"ua\":12000}"), "current_out refused without limits");
    CHECK(dac_limits_check_current_out() == NULL, "console current-out refused without limits");
}

static void test_inverted_solar(void) {
    dac_limits_set_active(&SOLAR);
    /* raw writers */
    const char *raw[] = { "generate", "dac_set", "load", "load_bin", "replay", "measure", "dac_mux", "cal_switch" };
    for (size_t i = 0; i < sizeof(raw) / sizeof(raw[0]); i++)
        CHECK(refused(raw[i], "{}", "raw DAC codes"), "%s not refused", raw[i]);
    CHECK(allowed("dac_stop", "{\"cmd\":\"dac_stop\"}"), "dac_stop refused");
    CHECK(allowed("status", "{\"cmd\":\"status\"}"), "status refused");
    CHECK(allowed("dac_limits", "{\"cmd\":\"dac_limits\",\"enabled\":false}"), "dac_limits refused");

    /* routes */
    CHECK(refused("analog_path", "{\"path\":\"off\"}", "near 0 V"), "path off allowed");
    CHECK(refused("analog_path", "{\"path\":\"3v3\"}", "Routing to 3v3"), "another DAC path allowed");
    CHECK(refused("analog_path", "{\"path\":\"cal2\"}", NULL), "cal2 allowed");
    CHECK(allowed("analog_path", "{\"path\":\"cal1\"}"), "cal1 refused on a 5v stage");
    CHECK(allowed("analog_path", "{\"path\":\"ext\"}"), "ext refused");
    CHECK(allowed("analog_path", "{\"path\":\"dac_5v\"}"), "own path alias refused");
    CHECK(allowed("adc_read", "{\"cmd\":\"adc_read\"}"), "adc_read default refused");
    CHECK(refused("adc_read", "{\"source\":\"cal2\"}", NULL), "adc_read cal2 allowed");

    /* held levels */
    CHECK(allowed("dac_out", "{\"path\":\"5v\",\"volts\":3.6}"), "park refused");
    CHECK(allowed("dac_out", "{\"path\":\"5v\",\"volts\":2.6}"), "in-window refused");
    CHECK(allowed("dac_out", "{\"path\":\"5v\"}"), "route-only refused");
    CHECK(refused("dac_out", "{\"path\":\"5v\",\"volts\":0}", "0 mV is outside"), "0 V allowed");
    CHECK(refused("dac_out", "{\"path\":\"5v\",\"volts\":3.9}", "outside"), "3.9 V allowed");
    CHECK(refused("dac_out", "{\"path\":\"off\"}", "near 0 V"), "dac_out off allowed");

    /* the 4-20 mA output shares the DAC: a current is refused, reading its range is not */
    CHECK(refused("current_out", "{\"cmd\":\"current_out\",\"ua\":12000}", "shares that DAC"), "current_out allowed");
    CHECK(allowed("current_out", "{\"cmd\":\"current_out\"}"), "current_out range read refused");
    CHECK(refused("analog_path", "{\"path\":\"current_out\"}", "near 0 V"), "current_out route allowed on an inverted stage");
    CHECK(dac_limits_check_current_out() != NULL, "console current-out allowed");

    /* the loop, with the clamp the page sends (nominal codes) */
    char j[160];
    snprintf(j, sizeof(j), "{\"vmin\":%ld,\"vmax\":%ld}", page_code(1.847), page_code(3.6));
    CHECK(allowed("dac_control_loop", j), "page's own clamp refused: %s", dac_limits_check_command("dac_control_loop", j));
    snprintf(j, sizeof(j), "{\"vmin\":0,\"vmax\":%ld}", page_code(3.6));
    CHECK(refused("dac_control_loop", j, "vmin at least"), "floor 0 allowed");
    snprintf(j, sizeof(j), "{\"vmin\":%ld,\"vmax\":65535}", page_code(1.9));
    CHECK(refused("dac_control_loop", j, "vmax at most"), "ceiling 65535 allowed");
    CHECK(refused("dac_control_loop", "{}", NULL), "loop without clamp allowed");
    snprintf(j, sizeof(j), "{\"vmin\":%ld,\"vmax\":%ld,\"in_trip\":900}", page_code(1.9), page_code(3.5));
    CHECK(refused("dac_control_loop", j, "HIGHEST"), "trip allowed on inverted");

    /* park rounds INTO the window */
    uint8_t code = dac_limits_park_code();
    float v = DAC_CAL[1].a + DAC_CAL[1].b * (float)code;
    CHECK(v <= 3.6f && v > 3.55f, "park code %u = %.3f V", code, (double)v);
}

static void test_normal_stage(void) {
    dac_limits_t n = SOLAR;
    n.inverted = 0; n.min_mv = 200; n.max_mv = 4000;
    dac_limits_set_active(&n);
    CHECK(allowed("analog_path", "{\"path\":\"off\"}"), "path off refused on a normal stage");
    CHECK(refused("dac_out", "{\"path\":\"5v\",\"volts\":4.5}", "outside"), "over-limit allowed");
    CHECK(refused("generate", "{}", "raw"), "generate allowed");
    CHECK(refused("current_out", "{\"ua\":4000}", "shares that DAC"), "current_out allowed on a normal stage");
    CHECK(allowed("analog_path", "{\"path\":\"current_out\"}"), "current_out route refused on a normal stage");
    char j[96];
    snprintf(j, sizeof(j), "{\"vmin\":%ld,\"vmax\":%ld,\"in_trip\":900}", page_code(0.3), page_code(3.9));
    CHECK(allowed("dac_control_loop", j), "trip refused on a normal stage");
    uint8_t code = dac_limits_park_code();
    float v = DAC_CAL[1].a + DAC_CAL[1].b * (float)code;
    CHECK(v >= 0.2f && v < 0.25f, "normal park code %u = %.3f V", code, (double)v);
}

static void test_store(void) {
    mock_flash_reset();
    dac_limits_set_active(NULL);
    CHECK(dac_limits_load() == -1 && !dac_limits_get()->enabled, "blank flash loaded limits");

    dac_limits_t bad = SOLAR;
    bad.min_mv = 4000;
    CHECK(dac_limits_set(&bad) != NULL, "min >= max accepted");
    bad = SOLAR; bad.max_mv = 6000;
    CHECK(dac_limits_set(&bad) != NULL, "max past the path's range accepted");
    bad = SOLAR; bad.path = 7;
    CHECK(dac_limits_set(&bad) != NULL, "bad path accepted");

    CHECK(dac_limits_set(&SOLAR) == NULL, "valid limits refused");
    dac_limits_set_active(NULL);
    CHECK(dac_limits_load() == 0, "saved limits did not load");
    const dac_limits_t *g = dac_limits_get();
    CHECK(g->enabled && g->path == 1 && g->inverted && g->min_mv == 1850 && g->max_mv == 3600,
          "loaded %u/%u/%u %ld..%ld", g->enabled, g->path, g->inverted, (long)g->min_mv, (long)g->max_mv);
    CHECK(mock_flash_double_programs == 0 && mock_flash_ecc_reads == 0, "flash misuse");

    CHECK(dac_limits_clear() == 0 && !dac_limits_get()->enabled, "clear failed");
    CHECK(dac_limits_load() == -1, "cleared limits came back");

    CHECK(dac_limits_path_index("dac_12v") == 2 && dac_limits_path_index("ext") == -1, "path names");
}

int main(void) {
    test_no_limits_changes_nothing();
    test_inverted_solar();
    test_normal_stage();
    test_store();
    if (fails) { printf("test_dac_limits: %d FAILED\n", fails); return 1; }
    printf("test_dac_limits: all passed\n");
    return 0;
}
