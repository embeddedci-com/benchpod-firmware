/*
 * test_lease_gate.c — host tests for the LAN-yields-to-cloud-jobs gate (src/lease_gate.c) and the
 * light-read classification it relies on (cmd_tier_light).
 */
#include "lease_gate.h"
#include "cmd_tier.h"

#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(cond, ...) do { if (!(cond)) { printf("FAIL %s:%d: ", __func__, __LINE__); \
        printf(__VA_ARGS__); printf("\n"); failures++; } } while (0)

static void test_lifecycle(void) {
    uint32_t left = 99;
    lease_gate_clear();
    CHECK(!lease_gate_active(1000, &left) && left == 0, "starts free");
    lease_gate_update(true, "CI: org/repo", 120, 1000);
    CHECK(lease_gate_active(1000, &left) && left == 120, "held: left %u", left);
    CHECK(strcmp(lease_gate_holder(), "CI: org/repo") == 0, "holder '%s'", lease_gate_holder());
    CHECK(lease_gate_active(1000 + 119500, &left) && left == 1, "rounds up: %u", left);
    /* a renew pushes the deadline */
    lease_gate_update(true, "CI: org/repo", 120, 100000);
    CHECK(lease_gate_active(200000, &left) && left == 20, "after renew %u", left);
    /* the pod's own deadline ends it without a release */
    CHECK(!lease_gate_active(100000 + 120000, &left), "expired lease still held");
    /* the check itself writes nothing (it runs on another task than the writer)... */
    CHECK(lease_gate_holder()[0] != '\0', "the read-only check cleared the holder");
    /* ...the writer's expire does, and it stays free even 2^31 ms on */
    lease_gate_expire(100000 + 120000);
    CHECK(lease_gate_holder()[0] == '\0', "holder kept after expiry");
    CHECK(!lease_gate_active(100000 + 120000 + 0x80000000u, &left), "stale deadline came back");
    lease_gate_update(true, "CI", 10, 0);
    lease_gate_expire(5000);
    CHECK(lease_gate_active(5000, &left) && left == 5, "expire ended a live lease");
    /* release and link drop */
    lease_gate_update(true, "web", 60, 0);
    lease_gate_update(false, NULL, 0, 10);
    CHECK(!lease_gate_active(20, NULL), "release ignored");
    lease_gate_update(true, "web", 60, 0);
    lease_gate_clear();
    CHECK(!lease_gate_active(20, NULL), "clear ignored");
}

static void test_caps_and_sanitizing(void) {
    uint32_t left = 0;
    lease_gate_update(true, "x", 100000, 0);              /* far beyond the server's maximum */
    CHECK(lease_gate_active(0, &left) && left == LEASE_GATE_MAX_S, "not capped: %u", left);
    lease_gate_update(true, "a\"b\\c\nd", 10, 0);
    CHECK(strcmp(lease_gate_holder(), "a?b?c?d") == 0, "not sanitized: '%s'", lease_gate_holder());
    char lng[100]; memset(lng, 'h', sizeof(lng) - 1); lng[99] = '\0';
    lease_gate_update(true, lng, 10, 0);
    CHECK(strlen(lease_gate_holder()) == LEASE_GATE_HOLDER_MAX - 1, "not truncated");
    lease_gate_update(true, "x", 0, 0);
    CHECK(!lease_gate_active(0, NULL), "a zero-second lease holds");
    /* wraparound of the millisecond clock */
    lease_gate_update(true, "x", 10, 0xFFFFF000u);
    CHECK(lease_gate_active(0x00000100u, &left), "wrap broke it");
    lease_gate_clear();
}

static void test_light_reads(void) {
    CHECK(cmd_tier_light("status", NULL) && cmd_tier_light("ping", NULL), "status/ping");
    CHECK(cmd_tier_light("lan_policy", "{\"cmd\":\"lan_policy\"}"), "policy read");
    CHECK(!cmd_tier_light("lan_policy", "{\"cmd\":\"lan_policy\",\"set\":\"open\"}"), "policy set");
    CHECK(!cmd_tier_light("capture", NULL) && !cmd_tier_light("la_capture", NULL), "captures are heavy");
    CHECK(!cmd_tier_light("generate", NULL) && !cmd_tier_light("target_power", NULL), "T1");
    CHECK(!cmd_tier_light("wifi_set", NULL) && !cmd_tier_light("ota_begin", NULL), "T2/T3");
    CHECK(!cmd_tier_light("no_such_cmd", NULL), "unknown");
    CHECK(cmd_tier_light("eth", "{\"cmd\":\"eth\",\"action\":\"stats\"}"), "eth stats");
}

int main(void) {
    test_lifecycle();
    test_caps_and_sanitizing();
    test_light_reads();
    if (failures) { printf("test_lease_gate: %d FAILED\n", failures); return 1; }
    printf("test_lease_gate: all passed\n");
    return 0;
}
