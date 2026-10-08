/*
 * test_cmd_tier.c — host tests for the command tier table (src/cmd_tier.c).
 *
 * Besides spot checks, it runs embeddedci-server's shared tier vectors through the command table
 * (cmd_table.h), checks the table's shape, and pins the gate flags to the string lists they replaced.
 */
#include "cmd_table.h"
#include "cmd_tier.h"
#include "bp_json.h"

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
    expect("cloud_ca", "{\"cmd\":\"cloud_ca\"}", CMD_TIER_T0);
    expect("cloud_ca", "{\"cmd\":\"cloud_ca\",\"clear\":true}", CMD_TIER_T2);
    expect("cloud_proxy", "{\"cmd\":\"cloud_proxy\",\"set\":\"p:3128\"}", CMD_TIER_T2);
}

static char *slurp(const char *path) {
    FILE *f = fopen(path, "rb");
    if (!f) return NULL;
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    char *buf = malloc((size_t)n + 1);
    size_t got = fread(buf, 1, (size_t)n, f);
    buf[got] = '\0';
    fclose(f);
    return buf;
}

/* One JSON string starting at p (on the opening quote) into out, unescaping \" and \\. Returns the
   character after the closing quote. */
static const char *json_string(const char *p, char *out, size_t cap) {
    size_t n = 0;
    for (p++; *p && *p != '"'; p++) {
        if (*p == '\\' && p[1]) p++;
        if (n + 1 < cap) out[n++] = *p;
    }
    out[n] = '\0';
    return *p ? p + 1 : p;
}

static cmd_tier_t parse_tier(const char *s) {
    return (s[0] == 'T' && s[1] >= '0' && s[1] <= '3' && !s[2]) ? (cmd_tier_t)(s[1] - '0') : (cmd_tier_t)-1;
}

/* embeddedci-server's shared vectors (a byte-identical copy of its
   testdata/shared/cmd_tier_vectors.json): the "table" is exactly the firmware's command table, and
   every "cases" line gets its tier from cmd_tier() the way dispatch_line() computes it. */
static void test_shared_vectors(void) {
    char *v = slurp("vectors/cmd_tier_vectors.json");
    CHECK(v != NULL, "cannot open vectors/cmd_tier_vectors.json (run from stm32h563/test)");
    if (!v) return;

    size_t rows = 0;
    const cmd_desc_t *t = cmd_table_rows(&rows);
    int seen[256] = {0};
    CHECK(rows < 256, "table too big for the test");

    const char *p = strstr(v, "\"table\": {");
    CHECK(p != NULL, "no table");
    const char *end = p ? strchr(p, '}') : NULL;
    int verbs = 0;
    for (p = p ? strchr(p, '{') + 1 : NULL; p && p < end; ) {
        const char *q = strchr(p, '"');
        if (!q || q >= end) break;
        char verb[40], tier[8];
        q = json_string(q, verb, sizeof(verb));
        q = strchr(q, '"');
        p = json_string(q, tier, sizeof(tier));
        const cmd_desc_t *d = cmd_find(verb);
        CHECK(d != NULL, "vectors' verb \"%s\" is not in the command table", verb);
        if (d) {
            CHECK(d->key.tier == parse_tier(tier), "%s: table %s, vectors %s", verb,
                  cmd_tier_name((cmd_tier_t)d->key.tier), tier);
            seen[d - t] = 1;
        }
        verbs++;
    }
    for (size_t i = 0; i < rows && i < 256; i++)
        CHECK(seen[i], "command table verb \"%s\" is not in the vectors' table", t[i].key.name);
    CHECK(verbs > 60, "parsed only %d verbs from the vectors' table", verbs);

    int cases = 0;
    p = strstr(v, "\"cases\": [");
    CHECK(p != NULL, "no cases");
    while (p && (p = strstr(p, "\"line\": \"")) != NULL) {
        char line[512], tier[8], cmd[32] = {0};
        p = json_string(p + 8, line, sizeof(line));
        p = strstr(p, "\"tier\": \"");
        if (!p) break;
        p = json_string(p + 8, tier, sizeof(tier));
        bp_json_get(line, "cmd", cmd, sizeof(cmd));
        cmd_tier_t got = cmd_tier(cmd, line);
        CHECK(got == parse_tier(tier), "%s: %s, vectors %s", line, cmd_tier_name(got), tier);
        cases++;
    }
    CHECK(cases > 80, "parsed only %d cases", cases);
    free(v);
}

/* Each verb once, a real tier, and the flags in their places: HEAVY and MIXED describe T0 rows
   (a heavy read, the read form of a mixed verb). */
static void test_table_shape(void) {
    size_t rows = 0;
    const cmd_desc_t *t = cmd_table_rows(&rows);
    CHECK(rows > 60, "only %zu rows", rows);
    for (size_t i = 0; i < rows; i++) {
        CHECK(t[i].key.name && t[i].key.name[0], "row %zu has no name", i);
        CHECK(t[i].key.tier <= CMD_TIER_T3, "%s: tier %u", t[i].key.name, t[i].key.tier);
        CHECK(strlen(t[i].key.name) < 32, "%s: longer than dispatch_line's cmd[32]", t[i].key.name);
        CHECK(cmd_find(t[i].key.name) == &t[i], "%s is in the table twice", t[i].key.name);
        if (t[i].flags & (CMD_F_HEAVY | CMD_F_MIXED))
            CHECK(t[i].key.tier == CMD_TIER_T0, "%s: HEAVY/MIXED on a %s row", t[i].key.name,
                  cmd_tier_name((cmd_tier_t)t[i].key.tier));
    }
    CHECK(cmd_find(NULL) == NULL && cmd_find("") == NULL && cmd_find("no_such_cmd") == NULL, "unknown");
}

/* The flag sets as they were in the five string lists the table replaced (cmd_gate.c's safe-mode
   and analog lists, cmd_tier.c's heavy reads, command_handler.c's cloud streaming and noisy polls).
   A change here changes which commands a gate lets through: do it on purpose. */
static void expect_set(unsigned flag, const char *what, const char *const *want, size_t n) {
    size_t rows = 0;
    const cmd_desc_t *t = cmd_table_rows(&rows);
    for (size_t i = 0; i < rows; i++) {
        int listed = 0;
        for (size_t k = 0; k < n; k++) listed |= strcmp(t[i].key.name, want[k]) == 0;
        CHECK(!!(t[i].flags & flag) == listed, "%s: %s is %s", what, t[i].key.name,
              listed ? "missing the flag" : "flagged but was not in the list");
    }
    for (size_t k = 0; k < n; k++) CHECK(cmd_find(want[k]) != NULL, "%s: no row for %s", what, want[k]);
}
#define EXPECT_SET(flag, ...) do { static const char *const w_[] = { __VA_ARGS__ }; \
        expect_set(flag, #flag, w_, sizeof(w_) / sizeof(w_[0])); } while (0)

static void test_flags_match_the_old_lists(void) {
    EXPECT_SET(CMD_F_NO_HW, "ping", "status", "cloud_set", "cloud_status", "cloud_clear",
               "wifi_set", "wifi_status", "wifi_clear", "eth", "speedtest",
               "la_voltage", "usb_cc", "nrst", "target_power", "target_status", "power_status",
               "power_profile", "identity_public", "identity_pop", "identity_wipe", "dac_limits",
               "can_config", "can_write", "can_read", "can_status", "can_term", "can_respond",
               "can_disable");
    EXPECT_SET(CMD_F_ANALOG, "generate", "capture", "stream", "measure", "load", "load_bin",
               "replay", "dac_limits", "dac_set", "dac_mux", "cal_switch", "analog_path", "dac_out",
               "current_out", "adc_read", "calibrate", "dac_control_loop", "dac_loop_probe",
               "dac_loop_input");
    EXPECT_SET(CMD_F_HEAVY, "capture", "capture_dual", "capture_read", "stream", "la_capture",
               "sensor_regs", "sensor_la", "can_read", "psram_ping", "test");
    EXPECT_SET(CMD_F_STREAM, "capture", "stream", "capture_dual", "capture_read", "measure",
               "test", "load", "replay", "load_bin", "sensor_regs", "sensor_la", "dap_start",
               "uart_proxy_start", "la_capture", "speedtest");
    EXPECT_SET(CMD_F_NOISY, "ping", "power_status", "target_status");
    EXPECT_SET(CMD_F_MIXED, "dac_limits", "calibrate", "eth", "sig_policy", "lan_policy",
               "cloud_ca", "cloud_proxy");
}

int main(void) {
    test_spot_checks();
    test_mixed_verbs();
    test_shared_vectors();
    test_table_shape();
    test_flags_match_the_old_lists();
    if (failures) { printf("test_cmd_tier: %d FAILED\n", failures); return 1; }
    printf("test_cmd_tier: all passed\n");
    return 0;
}
