/*
 * test_lease_local.c: host tests for the pod's own lease (src/lease_local.c): acquire, refresh and
 * join with the same id, busy for another id, expiry, release, the owner flag ending with its
 * connection, light reads passing the gate, and a server break.
 */
#include "lease_local.h"
#include "cmd_tier.h"

#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(cond, ...) do { if (!(cond)) { printf("FAIL %s:%d: ", __func__, __LINE__); \
        printf(__VA_ARGS__); printf("\n"); failures++; } } while (0)

enum { A = 0, B = 1, C = 7 /* a cloud tunnel */ };

static lease_local_gate_t gate(int conn, const char *cmd, const char *json, uint32_t now) {
    lease_local_view_t v;
    char by[48];
    return lease_local_check(conn, strncmp(cmd, "lease_", 6) == 0, cmd_tier_light(cmd, json) != 0,
                             now, &v, by, sizeof(by));
}

static void test_acquire_refresh_join(void) {
    lease_local_reset();
    lease_local_view_t v;
    CHECK(!lease_local_get(A, 0, &v), "starts free");
    uint32_t g0 = lease_local_generation();
    CHECK(lease_local_acquire(A, "L-1", "benchpod ci: ed", "process-node-hil", 300, 1000, &v) ==
          LEASE_LOCAL_OK, "acquire");
    CHECK(v.held && v.mine && v.left_s == 300, "view held %d mine %d left %u", v.held, v.mine, v.left_s);
    CHECK(!strcmp(v.id, "L-1") && !strcmp(v.holder, "benchpod ci: ed") &&
          !strcmp(v.purpose, "process-node-hil"), "fields %s/%s/%s", v.id, v.holder, v.purpose);
    CHECK(lease_local_generation() != g0, "acquire did not move the generation");
    uint32_t g1 = lease_local_generation();

    /* B is not an owner yet */
    lease_local_get(B, 2000, &v);
    CHECK(v.held && !v.mine, "B sees it as not mine");
    /* same id from B: refresh, and B joins; holder and purpose stay the first acquire's */
    CHECK(lease_local_acquire(B, "L-1", "other", "other", 600, 5000, &v) == LEASE_LOCAL_OK, "join");
    CHECK(v.mine && v.left_s == 600 && !strcmp(v.holder, "benchpod ci: ed"), "joined %d %u %s",
          v.mine, v.left_s, v.holder);
    CHECK(lease_local_generation() == g1, "a join is not a change");
    lease_local_get(A, 5000, &v);
    CHECK(v.mine, "A stays an owner");
    CHECK(gate(A, "dac_set", "{\"cmd\":\"dac_set\"}", 6000) == LEASE_LOCAL_PASS, "owner A drives");
    CHECK(gate(B, "dac_set", "{\"cmd\":\"dac_set\"}", 6000) == LEASE_LOCAL_PASS, "owner B drives");

    /* ttl clamps */
    CHECK(lease_local_clamp_ttl(0) == LEASE_LOCAL_TTL_DEFAULT_S, "default ttl");
    CHECK(lease_local_clamp_ttl(1) == 10 && lease_local_clamp_ttl(99999) == 1800, "clamp");
    lease_local_acquire(A, "L-1", NULL, NULL, 5, 10000, &v);
    CHECK(v.left_s == 10, "short ttl clamped to %u", v.left_s);
    lease_local_renew(A, "L-1", 4000, 10000, &v);
    CHECK(v.left_s == 1800, "long ttl clamped to %u", v.left_s);
}

static void test_busy_for_other_id(void) {
    lease_local_reset();
    lease_local_view_t v;
    lease_local_acquire(A, "L-a", "ci run 1", "hil", 120, 0, &v);
    CHECK(lease_local_acquire(B, "L-b", "ci run 2", "hil", 120, 30000, &v) == LEASE_LOCAL_BUSY, "busy");
    CHECK(v.held && !v.mine && !strcmp(v.holder, "ci run 1") && !strcmp(v.purpose, "hil") &&
          v.left_s == 90, "busy view %s %s %u", v.holder, v.purpose, v.left_s);
    CHECK(lease_local_renew(B, "L-b", 120, 30000, &v) == LEASE_LOCAL_BUSY, "renew other id");
    CHECK(lease_local_release(B, "L-b", 30000, &v) == LEASE_LOCAL_BUSY, "release other id");
    lease_local_get(A, 30000, &v);
    CHECK(v.held, "other id's release freed it");
    CHECK(gate(B, "dac_set", "{\"cmd\":\"dac_set\"}", 30000) == LEASE_LOCAL_REFUSE_BUSY, "B refused");
    CHECK(gate(C, "gpio_write", "{\"cmd\":\"gpio_write\"}", 30000) == LEASE_LOCAL_REFUSE_BUSY,
          "a tunnel refused too");
    CHECK(gate(B, "lease_acquire", "{\"cmd\":\"lease_acquire\"}", 30000) == LEASE_LOCAL_PASS,
          "lease verbs reach the handler");
    /* bad ids */
    CHECK(lease_local_acquire(B, "", "x", "y", 60, 0, &v) == LEASE_LOCAL_BAD_ID, "empty id");
    CHECK(lease_local_acquire(B, "has space", "x", "y", 60, 0, &v) == LEASE_LOCAL_BAD_ID, "space");
    char long_id[80];
    memset(long_id, 'x', sizeof(long_id) - 1); long_id[79] = '\0';
    CHECK(lease_local_acquire(B, long_id, "x", "y", 60, 0, &v) == LEASE_LOCAL_BAD_ID, "long id");
    long_id[64] = '\0';
    lease_local_reset();
    CHECK(lease_local_acquire(B, long_id, "x", "y", 60, 0, &v) == LEASE_LOCAL_OK, "64-char id");
    /* labels: cut and made safe */
    lease_local_reset();
    lease_local_acquire(A, "L", "a\"b\\c 0123456789012345678901234567890123456789", "p\n", 60, 0, &v);
    CHECK(strlen(v.holder) == 40 && !strchr(v.holder, '"') && !strchr(v.holder, '\\'),
          "holder %s", v.holder);
    CHECK(!strcmp(v.purpose, "p?"), "purpose %s", v.purpose);
}

static void test_expiry(void) {
    lease_local_reset();
    lease_local_view_t v;
    lease_local_acquire(A, "L-e", "ci", "hil", 10, 1000, &v);
    CHECK(gate(B, "dac_set", "{\"cmd\":\"dac_set\"}", 10999) == LEASE_LOCAL_REFUSE_BUSY, "held at 9.999 s");
    lease_local_get(B, 10500, &v);
    CHECK(v.left_s == 1, "rounds up: %u", v.left_s);
    uint32_t g = lease_local_generation();
    CHECK(gate(B, "dac_set", "{\"cmd\":\"dac_set\"}", 11000) == LEASE_LOCAL_PASS, "expired at 10 s");
    CHECK(lease_local_generation() != g, "expiry did not move the generation");
    CHECK(!lease_local_get(A, 11000, &v) && !v.mine, "A no longer an owner");
    CHECK(lease_local_renew(A, "L-e", 10, 11000, &v) == LEASE_LOCAL_NOT_HELD, "renew after expiry");
    CHECK(lease_local_acquire(B, "L-f", "other", "x", 60, 12000, &v) == LEASE_LOCAL_OK, "free again");
    /* the deadline stays dead 2^31 ms later */
    lease_local_expire(12000 + 60000);
    CHECK(!lease_local_get(B, 12000 + 60000 + 0x80000000u, &v), "stale deadline came back");
    /* renew pushes the deadline */
    lease_local_reset();
    lease_local_acquire(A, "L-r", "ci", "", 20, 0, &v);
    CHECK(lease_local_renew(B, "L-r", 30, 15000, &v) == LEASE_LOCAL_OK && !v.mine,
          "renew by id from another connection works but does not make it an owner");
    CHECK(lease_local_get(A, 40000, &v) && v.left_s == 5, "renewed: %u", v.left_s);
}

static void test_release(void) {
    lease_local_reset();
    lease_local_view_t v;
    CHECK(lease_local_release(A, "L-x", 0, &v) == LEASE_LOCAL_OK, "release when free is ok");
    lease_local_acquire(A, "L-x", "ci", "hil", 300, 0, &v);
    lease_local_acquire(B, "L-x", "ci", "hil", 300, 0, &v);
    uint32_t g = lease_local_generation();
    CHECK(lease_local_release(B, "L-x", 1000, &v) == LEASE_LOCAL_OK && !v.held, "release");
    CHECK(lease_local_generation() != g, "release did not move the generation");
    CHECK(gate(C, "dac_set", "{\"cmd\":\"dac_set\"}", 1000) == LEASE_LOCAL_PASS, "free after release");
    CHECK(!lease_local_get(A, 1000, &v) && !v.mine, "owner flags cleared");
}

static void test_owner_on_close(void) {
    lease_local_reset();
    lease_local_view_t v;
    lease_local_acquire(A, "L-c", "ci", "hil", 300, 0, &v);
    lease_local_conn_closed(A);
    CHECK(lease_local_get(A, 1000, &v) && !v.mine, "closed conn is no longer an owner, lease stays");
    CHECK(gate(A, "dac_set", "{\"cmd\":\"dac_set\"}", 1000) == LEASE_LOCAL_REFUSE_BUSY,
          "a new client on the slot is not an owner");
    lease_local_acquire(A, "L-c", "ci", "hil", 300, 2000, &v);
    CHECK(v.mine && gate(A, "dac_set", "{\"cmd\":\"dac_set\"}", 2000) == LEASE_LOCAL_PASS, "rejoin");
    lease_local_conn_closed(B);   /* not an owner: nothing */
    lease_local_conn_closed(-3);  /* out of range: nothing */
    lease_local_conn_closed(99);
    CHECK(lease_local_get(A, 2000, &v) && v.mine, "unrelated closes changed nothing");
}

static void test_light_allowed(void) {
    lease_local_reset();
    lease_local_view_t v;
    lease_local_acquire(A, "L-l", "ci", "hil", 300, 0, &v);
    static const char *const light[] = { "status", "ping", "power_status", "la_pins", "wiring_list" };
    for (size_t i = 0; i < sizeof(light) / sizeof(light[0]); i++) {
        char json[64];
        snprintf(json, sizeof(json), "{\"cmd\":\"%s\"}", light[i]);
        CHECK(gate(B, light[i], json, 1000) == LEASE_LOCAL_PASS, "%s refused", light[i]);
    }
    static const char *const drive[] = { "dac_set", "target_power", "la_capture", "gpio_write",
                                         "la_dir", "uart_proxy_start" };
    for (size_t i = 0; i < sizeof(drive) / sizeof(drive[0]); i++) {
        char json[64];
        snprintf(json, sizeof(json), "{\"cmd\":\"%s\",\"x\":1}", drive[i]);
        CHECK(gate(B, drive[i], json, 1000) == LEASE_LOCAL_REFUSE_BUSY, "%s allowed", drive[i]);
    }
}

static void test_break(void) {
    lease_local_reset();
    lease_local_view_t v;
    char by[48];
    lease_local_acquire(A, "L-k", "ci", "hil", 300, 0, &v);
    lease_local_acquire(B, "L-k", "ci", "hil", 300, 0, &v);
    uint32_t g = lease_local_generation();
    lease_local_break("ward (web)");
    /* the posted break lands on the writer's next call */
    CHECK(lease_local_check(C, false, false, 1000, &v, by, sizeof(by)) == LEASE_LOCAL_PASS,
          "a non-owner just runs after the break");
    CHECK(lease_local_generation() != g, "break did not move the generation");
    CHECK(!v.held, "still held after the break");
    CHECK(lease_local_check(A, false, true, 1000, &v, by, sizeof(by)) == LEASE_LOCAL_REFUSE_LOST &&
          !strcmp(by, "ward (web)"), "A told once: %s", by);
    CHECK(lease_local_check(A, false, false, 1000, &v, by, sizeof(by)) == LEASE_LOCAL_PASS,
          "A told twice");
    /* B closes before its next command: nothing pending for the next client on the slot */
    lease_local_conn_closed(B);
    CHECK(lease_local_check(B, false, false, 1000, &v, by, sizeof(by)) == LEASE_LOCAL_PASS,
          "a closed owner's lost flag reached the next client");
    /* a break with nothing held does nothing */
    lease_local_break("nobody");
    CHECK(lease_local_check(A, false, false, 2000, &v, by, sizeof(by)) == LEASE_LOCAL_PASS, "no-op");
    /* an empty name */
    lease_local_acquire(A, "L-k2", "ci", "hil", 300, 3000, &v);
    lease_local_break("");
    CHECK(lease_local_check(A, false, false, 3000, &v, by, sizeof(by)) == LEASE_LOCAL_REFUSE_LOST &&
          !strcmp(by, "the server"), "default name: %s", by);
}

static void test_snapshot(void) {
    lease_local_reset();
    lease_local_view_t v;
    uint32_t gen = 0;
    CHECK(lease_local_snapshot(0, &v, &gen) && !v.held, "free snapshot");
    lease_local_acquire(A, "L-s", "ci", "hil", 60, 0, &v);
    CHECK(lease_local_snapshot(1000, &v, &gen) && v.held && !v.mine && v.left_s == 59 &&
          gen == lease_local_generation(), "held snapshot");
    /* past the deadline the snapshot already says free, before the writer expires it */
    CHECK(lease_local_snapshot(60000, &v, &gen) && !v.held, "expired snapshot");
}

int main(void) {
    test_acquire_refresh_join();
    test_busy_for_other_id();
    test_expiry();
    test_release();
    test_owner_on_close();
    test_light_allowed();
    test_break();
    test_snapshot();
    if (failures) { printf("test_lease_local: %d FAILED\n", failures); return 1; }
    printf("test_lease_local: all passed\n");
    return 0;
}
