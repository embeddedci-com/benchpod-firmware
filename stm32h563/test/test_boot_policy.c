/* Host unit test for boot_policy: which subsystems a safe-mode boot turns off. */
#include "boot_policy.h"
#include <stdio.h>
#include <string.h>

static int fails = 0;
#define CHECK(cond, ...) do { if (!(cond)) { \
        printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

int main(void) {
    const uint32_t N = BOOT_SUB_NET, H = BOOT_SUB_HW, A = BOOT_SUB_ALL;

    /* The crashed task names the culprit, whatever else was in progress. */
    CHECK(boot_policy_culprit(A, 0, "net") == N, "net crash -> net");
    CHECK(boot_policy_culprit(A, A, "hw") == H, "hw crash after bring-up -> hw");
    /* A hang (no crash record): the bring-up that started but never finished. */
    CHECK(boot_policy_culprit(N | H, N, "") == H, "hw bring-up hang -> hw");
    CHECK(boot_policy_culprit(N, 0, "") == N, "net bring-up hang -> net");
    /* Before either started, or a crash in another task after both finished: unknown. */
    CHECK(boot_policy_culprit(0, 0, "boot") == 0, "early crash -> none");
    CHECK(boot_policy_culprit(A, A, "console") == 0, "console crash -> none");
    CHECK(boot_policy_culprit(A, A, NULL) == 0, "NULL task -> none");

    /* First safe boot: only the culprit, or both when unknown. */
    CHECK(boot_policy_off(0, H) == H, "hw culprit -> hw off");
    CHECK(boot_policy_off(0, N) == N, "net culprit -> net off");
    CHECK(boot_policy_off(0, 0) == A, "unknown -> both off");
    /* Later safe boots keep what was off and add a new culprit. */
    CHECK(boot_policy_off(H, N) == A, "hw off + net culprit -> both");
    CHECK(boot_policy_off(H, 0) == H, "clean reboot in safe mode keeps hw off only");
    CHECK(boot_policy_off(0xF0 | H, 0) == H, "junk bits masked");

    CHECK(strcmp(boot_policy_off_str(N), "network") == 0, "str net");
    CHECK(strcmp(boot_policy_off_str(H), "iCE40/PSRAM") == 0, "str hw");
    CHECK(strcmp(boot_policy_off_str(A), "network and iCE40/PSRAM") == 0, "str both");

    /* The image's identity (gw_ident_t): a record names the slot and its sha256 + length. */
    uint8_t sha_a[32], sha_b[32];
    memset(sha_a, 0xA1, sizeof(sha_a));
    memset(sha_b, 0xB2, sizeof(sha_b));
    gw_ident_t rec;
    gw_ident_make(&rec, 0, 104090, sha_a);
    CHECK(gw_ident_valid(&rec), "ident: a made record is valid");
    gw_ident_t blank;
    memset(&blank, 0xFF, sizeof(blank));
    CHECK(!gw_ident_valid(&blank), "ident: an erased sector is not a record");
    CHECK(!gw_ident_valid(NULL), "ident: NULL is not a record");
    gw_ident_t torn = rec;
    torn.sha256[7] ^= 1;
    CHECK(!gw_ident_valid(&torn), "ident: a torn record fails its CRC");
    CHECK(gw_ident_match(&rec, 1, 0, 104090, sha_a) == GW_IDENT_SAME, "ident: same slot + sha -> same");
    CHECK(gw_ident_match(&rec, 1, 0, 104090, sha_b) == GW_IDENT_DIFFERS, "ident: new sha (dev rebuild) -> differs");
    CHECK(gw_ident_match(&rec, 1, 0, 104091, sha_a) == GW_IDENT_DIFFERS, "ident: other length -> differs");
    CHECK(gw_ident_match(&rec, 1, 8, 104090, sha_a) == GW_IDENT_DIFFERS, "ident: other slot -> differs");
    CHECK(gw_ident_match(&blank, 1, 0, 104090, sha_a) == GW_IDENT_UNKNOWN, "ident: no record -> unknown");
    CHECK(gw_ident_match(NULL, 1, 0, 104090, sha_a) == GW_IDENT_UNKNOWN, "ident: record not read -> unknown");
    CHECK(gw_ident_match(&rec, 0, 0, 104090, sha_a) == GW_IDENT_NONE, "ident: slot not what the firmware expects -> none");

    /* The plan (4.0.6+): offset 0 always holds gw0 (loop); the running kind is loaded from its
       slot. P(boot, load) checks both fields. */
#define P(r, e, k, c, id, b, l, msg) do { gw_plan_t p_ = boot_policy_gateware_plan(r, e, k, c, id); \
        CHECK(p_.boot == (b) && p_.load == (l), "plan: %s (got %d, %d)", msg, (int)p_.boot, p_.load); } while (0)
    P(54, 54, 0, 1, GW_IDENT_SAME,    GW_ACT_NONE,    -1, "loop runs, offset 0 is gw0 -> nothing");
    P(54, 54, 1, 1, GW_IDENT_SAME,    GW_ACT_NONE,    -1, "deep runs (soft reset), offset 0 is gw0 -> nothing");
    P(54, 54, 2, 1, GW_IDENT_SAME,    GW_ACT_NONE,    -1, "emulation runs, offset 0 is gw0 -> nothing");
    P(54, 54, 0, 1, GW_IDENT_DIFFERS, GW_ACT_WRITE,   -1, "offset 0 holds another image -> write gw0 once");
    P(54, 54, 1, 1, GW_IDENT_DIFFERS, GW_ACT_WRITE,    1, "older firmware left deep at offset 0 -> write gw0, load deep again");
    P(54, 54, 2, 1, GW_IDENT_UNKNOWN, GW_ACT_COMPARE,  2, "no record -> compare, then load emulation again");
    P(54, 54, 0, 1, GW_IDENT_UNKNOWN, GW_ACT_COMPARE, -1, "no record, loop -> compare only");
    P(53, 54, 0, 1, GW_IDENT_DIFFERS, GW_ACT_WRITE,   -1, "new gateware: write gw0, the boot runs it");
    P(53, 54, 1, 1, GW_IDENT_DIFFERS, GW_ACT_WRITE,    1, "new gateware on deep: write gw0, load deep from its slot");
    P(53, 54, 1, 1, GW_IDENT_SAME,    GW_ACT_NONE,     1, "offset 0 is current, running deep is old -> load deep");
    P(53, 54, 0, 1, GW_IDENT_SAME,    GW_ACT_NONE,     0, "offset 0 is current, running loop is old -> load loop");
    P(53, 54, 2, 1, GW_IDENT_NONE,    GW_ACT_NONE,     2, "gw0 slot not this firmware's: offset 0 left, emulation loaded (if its slot is)");
    P(54, 54, 7, 1, GW_IDENT_DIFFERS, GW_ACT_WRITE,   -1, "unknown kind counts as loop");
    P(0,  54, 1, 1, GW_IDENT_DIFFERS, GW_ACT_NONE,    -1, "configured but not answering -> nothing");
    P(54, 0,  1, 1, GW_IDENT_DIFFERS, GW_ACT_NONE,    -1, "embedded version unknown -> nothing");
    P(0,  54, 0, 0, GW_IDENT_SAME,    GW_ACT_WRITE,   -1, "never configured -> write gw0");
    P(0,  0,  2, 0, GW_IDENT_NONE,    GW_ACT_WRITE,   -1, "never configured, embedded unknown -> still gw0");
#undef P

    /* No storm: boots after the first never write. A pod whose offset 0 already holds gw0 with no
       record: one compare, recorded, then nothing, whatever kind runs. */
    gw_ident_t store;
    memset(&store, 0xFF, sizeof(store));
    int writes = 0, compares = 0;
    for (int boot = 0; boot < 5; boot++) {
        gw_plan_t p = boot_policy_gateware_plan(54, 54, boot % 3, 1, gw_ident_match(&store, 1, 0, 104090, sha_a));
        if (p.boot == GW_ACT_COMPARE) { compares++; gw_ident_make(&store, 0, 104090, sha_a); }   /* matched */
        if (p.boot == GW_ACT_WRITE) { writes++; gw_ident_make(&store, 0, 104090, sha_a); }
    }
    CHECK(compares == 1 && writes == 0, "storm: matching image -> 1 compare, 0 writes (got %d, %d)", compares, writes);
    /* Offset 0 recorded as deep (slot 1, an older firmware's switch): one write of gw0, then none. */
    gw_ident_make(&store, 1, 104090, sha_b);
    writes = compares = 0;
    for (int boot = 0; boot < 5; boot++) {
        gw_plan_t p = boot_policy_gateware_plan(54, 54, 1, 1, gw_ident_match(&store, 1, 0, 104090, sha_a));
        if (p.boot == GW_ACT_COMPARE) { compares++; p.boot = GW_ACT_WRITE; }
        if (p.boot == GW_ACT_WRITE) { writes++; gw_ident_make(&store, 0, 104090, sha_a); }
    }
    CHECK(compares == 0 && writes == 1, "storm: deep at offset 0 -> 1 write (got %d, %d)", compares, writes);

    if (fails) { printf("test_boot_policy: %d FAILED\n", fails); return 1; }
    printf("test_boot_policy: all passed\n");
    return 0;
}
