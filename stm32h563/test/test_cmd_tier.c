/*
 * test_cmd_tier.c — host tests for the command tier table (src/cmd_tier.c).
 *
 * Besides spot checks, it reads dispatch_line() out of ../src/command_handler.c and fails for
 * every verb there that the table does not list, so a new command cannot ship unclassified.
 */
#include "cmd_tier.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures;

#define CHECK(cond, ...)                                        \
    do {                                                        \
        if (!(cond)) {                                           \
            printf("FAIL %s:%d: ", __func__, __LINE__);          \
            printf(__VA_ARGS__);                                 \
            printf("\n");                                        \
            failures++;                                          \
        }                                                        \
    } while (0)

static void expect(const char *cmd, const char *json, cmd_tier_t want) {
    cmd_tier_t got = cmd_tier(cmd, json);
    CHECK(got == want, "%s %s: %s, want %s", cmd, json ? json : "(no json)", cmd_tier_name(got),
          cmd_tier_name(want));
}

static void test_spot_checks(void) {
    expect("status", NULL, CMD_TIER_T0);
    expect("capture", "{\"cmd\":\"capture\"}", CMD_TIER_T0);
    expect("generate", NULL, CMD_TIER_T1);
    expect("target_power", NULL, CMD_TIER_T1);
    expect("wifi_set", NULL, CMD_TIER_T2);
    expect("cloud_set", NULL, CMD_TIER_T2);
    expect("ota_begin", NULL, CMD_TIER_T3);
    expect("ota_selftest", NULL, CMD_TIER_T3);
    expect("no_such_cmd", NULL, CMD_TIER_T3);   /* unknown never counts as harmless */
    CHECK(!cmd_tier_known("no_such_cmd") && cmd_tier_known("ping"), "known");
}

static void test_mixed_verbs(void) {
    expect("dac_limits", "{\"cmd\":\"dac_limits\"}", CMD_TIER_T0);
    expect("dac_limits", "{\"cmd\":\"dac_limits\",\"path\":\"5v\",\"min_mv\":0,\"max_mv\":1}", CMD_TIER_T2);
    expect("dac_limits", "{\"cmd\":\"dac_limits\",\"enabled\":false}", CMD_TIER_T2);
    expect("dac_limits", "{\"cmd\":\"dac_limits\",\"enabled\":true}", CMD_TIER_T0);
    expect("calibrate", "{\"cmd\":\"calibrate\"}", CMD_TIER_T0);
    expect("calibrate", "{\"cmd\":\"calibrate\",\"source\":\"current_in\"}", CMD_TIER_T2);
    expect("calibrate", "{\"cmd\":\"calibrate\",\"clear\":true}", CMD_TIER_T2);
    expect("eth", "{\"cmd\":\"eth\",\"action\":\"stats\"}", CMD_TIER_T0);
    expect("eth", "{\"cmd\":\"eth\",\"action\":\"refclk\"}", CMD_TIER_T0);
    expect("eth", "{\"cmd\":\"eth\",\"action\":\"restart\"}", CMD_TIER_T2);
    expect("eth", "{\"cmd\":\"eth\",\"action\":\"speed\",\"mbit\":10}", CMD_TIER_T2);
    expect("eth", "{\"cmd\":\"eth\"}", CMD_TIER_T2);
    expect("sig_policy", "{\"cmd\":\"sig_policy\"}", CMD_TIER_T0);
    expect("sig_policy", "{\"cmd\":\"sig_policy\",\"set\":\"required\"}", CMD_TIER_T2);
    expect("lan_policy", "{\"cmd\":\"lan_policy\"}", CMD_TIER_T0);
    expect("lan_policy", "{\"cmd\":\"lan_policy\",\"set\":\"off\"}", CMD_TIER_T2);
}

/* Every `strcmp(cmd, "<verb>")` inside dispatch_line() must be in the table. */
static void test_dispatch_line_is_covered(void) {
    FILE *f = fopen("../src/command_handler.c", "rb");
    CHECK(f != NULL, "cannot open ../src/command_handler.c (run from stm32h563/test)");
    if (!f) return;
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    char *src = malloc((size_t)n + 1);
    fread(src, 1, (size_t)n, f);
    src[n] = '\0';
    fclose(f);

    char *start = strstr(src, "static void dispatch_line(int conn_id, const char *buf) {");
    CHECK(start != NULL, "dispatch_line() not found");
    char *end = start ? strstr(start, "\n}\n") : NULL;
    int verbs = 0;
    for (char *p = start; p && p < end; ) {
        char *m = strstr(p, "else if (strcmp(cmd, \"");
        char *m0 = strstr(p, "if      (strcmp(cmd, \"");
        if (m0 && (!m || m0 < m)) m = m0;
        if (!m || m >= end) break;
        char *q = strchr(m, '"') + 1;
        char *e = strchr(q, '"');
        char verb[40] = {0};
        size_t len = (size_t)(e - q) < sizeof(verb) - 1 ? (size_t)(e - q) : sizeof(verb) - 1;
        memcpy(verb, q, len);
        CHECK(cmd_tier_known(verb), "dispatch_line handles \"%s\" but cmd_tier.c has no tier for it", verb);
        verbs++;
        p = e;
    }
    CHECK(verbs > 60, "found only %d verbs in dispatch_line (parser out of date?)", verbs);
    free(src);
}

int main(void) {
    test_spot_checks();
    test_mixed_verbs();
    test_dispatch_line_is_covered();
    if (failures) { printf("test_cmd_tier: %d FAILED\n", failures); return 1; }
    printf("test_cmd_tier: all passed\n");
    return 0;
}
