/* Host tests for c3_restarts.c: the per-cause ESP32-C3 restart counts kept through soft resets. */
#include "c3_restarts.h"
#include <stdio.h>
#include <string.h>

static int g_fail, g_run;
#define CHECK(c) do { g_run++; if (!(c)) { g_fail++; printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); } } while (0)

static void test_power_on_starts_at_zero(void) {
    c3_restarts_t r;
    memset(&r, 0xA5, sizeof(r));   /* .noinit after power-on: random */
    c3_restarts_boot(&r, true);
    CHECK(r.total == 0 && r.boots == 1);
    char j[300];
    CHECK(c3_restarts_json(&r, j, sizeof(j)) > 0);
    CHECK(strcmp(j, "{\"total\":0,\"no_response\":0,\"rebooted\":0,\"no_boot\":0,\"setup\":0,"
                    "\"assoc\":0,\"flash\":0,\"boots\":1}") == 0);
}

static void test_garbage_without_power_on_starts_over(void) {
    c3_restarts_t r;
    memset(&r, 0x5A, sizeof(r));   /* first boot after flashing new firmware: a soft reset */
    c3_restarts_boot(&r, false);
    CHECK(r.total == 0 && r.boots == 1);
}

static void test_counts_by_cause_and_keeps_them_through_a_soft_reset(void) {
    c3_restarts_t r;
    c3_restarts_boot(&r, true);
    c3_restarts_note(&r, C3_RESTART_NO_RESPONSE, "no response to RSSI requests", 100);
    c3_restarts_note(&r, C3_RESTART_REBOOTED, "it announced a new boot", 200);
    c3_restarts_note(&r, C3_RESTART_REBOOTED, "it announced a new boot", 300);
    CHECK(r.total == 3 && r.count[C3_RESTART_REBOOTED] == 2 && r.count[C3_RESTART_NO_RESPONSE] == 1);
    char j[300];
    CHECK(c3_restarts_json(&r, j, sizeof(j)) > 0);
    CHECK(strstr(j, "\"last_this_boot\":true") != NULL);
    c3_restarts_boot(&r, false);   /* watchdog, OTA, reboot */
    CHECK(r.total == 3 && r.boots == 2);
    CHECK(c3_restarts_json(&r, j, sizeof(j)) > 0);
    CHECK(strstr(j, "{\"total\":3,\"no_response\":1,\"rebooted\":2,") == j);
    CHECK(strstr(j, "\"last\":\"it announced a new boot\",\"last_cause\":\"rebooted\",\"last_uptime_s\":300,"
                    "\"last_this_boot\":false,\"boots\":2}") != NULL);
    c3_restarts_boot(&r, true);    /* power-on or the reset pin */
    CHECK(r.total == 0 && r.boots == 1);
}

static void test_a_damaged_record_starts_over(void) {
    c3_restarts_t r;
    c3_restarts_boot(&r, true);
    c3_restarts_note(&r, C3_RESTART_SETUP, "init timeout", 5);
    r.count[C3_RESTART_SETUP] = 7;  /* a bit flip, or a layout from another firmware */
    c3_restarts_boot(&r, false);
    CHECK(r.total == 0);
}

static void test_reason_is_cut_and_safe_for_json(void) {
    c3_restarts_t r;
    c3_restarts_boot(&r, true);
    c3_restarts_note(&r, C3_RESTART_NO_BOOT,
                     "no boot event from the \"ESP32-C3\" (already flashed once; a long reason)", 9);
    CHECK(strlen(r.last_reason) == C3_RESTART_REASON_LEN - 1);
    CHECK(strchr(r.last_reason, '"') == NULL);
    char d[200];
    CHECK(c3_restarts_describe(&r, d, sizeof(d)) > 0);
    CHECK(strstr(d, "C3 restarts since power-on: 1 (no boot 1), last: no boot event from the 'ESP32-C3'") == d);
    CHECK(strstr(d, "at 9 s this boot") != NULL);
}

static void test_json_too_small_is_empty(void) {
    c3_restarts_t r;
    c3_restarts_boot(&r, true);
    char j[20];
    CHECK(c3_restarts_json(&r, j, sizeof(j)) == 0 && j[0] == '\0');
}

int main(void) {
    test_power_on_starts_at_zero();
    test_garbage_without_power_on_starts_over();
    test_counts_by_cause_and_keeps_them_through_a_soft_reset();
    test_a_damaged_record_starts_over();
    test_reason_is_cut_and_safe_for_json();
    test_json_too_small_is_empty();
    printf("test_c3_restarts: %d checks, %d failed\n", g_run, g_fail);
    return g_fail ? 1 : 0;
}
