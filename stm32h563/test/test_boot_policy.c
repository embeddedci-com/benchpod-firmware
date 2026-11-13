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

    /* Boot-time gateware update: only when both versions are known and differ. */
    CHECK(boot_policy_gateware_image(36, 36, 0, 1) == -1, "gw: same version, no update");
    CHECK(boot_policy_gateware_image(35, 36, 0, 1) == 0, "gw: older loop image -> reload image 0");
    CHECK(boot_policy_gateware_image(35, 36, 1, 1) == 1, "gw: older deep image -> reload image 1");
    CHECK(boot_policy_gateware_image(53, 54, 2, 1) == 2, "gw: older emulation image -> reload image 2");
    CHECK(boot_policy_gateware_image(53, 54, 7, 1) == 0, "gw: unknown image kind -> image 0");
    CHECK(boot_policy_gateware_image(37, 36, 0, 1) == 0, "gw: newer gateware than the firmware -> match the firmware");
    CHECK(boot_policy_gateware_image(0, 36, 0, 1) == -1, "gw: configured but not answering -> leave it (not a blank flash)");
    CHECK(boot_policy_gateware_image(35, 0, 0, 1) == -1, "gw: embedded version unknown -> never touch it");
    /* A new board: the config flash is blank, so the iCE40 never configured and answers v0. The
       blobs arrive after the firmware (flash-self installs them once it boots), so this runs again
       when gw0 is installed: load image 0 then, not only at the next boot. */
    CHECK(boot_policy_gateware_image(0, 48, 0, 0) == 0, "gw: iCE40 never configured -> load image 0");
    CHECK(boot_policy_gateware_image(0, 0, 0, 0) == 0, "gw: never configured, embedded unknown -> still image 0");
    CHECK(boot_policy_gateware_image(48, 48, 1, 0) == 0, "gw: CDONE low wins over a stale version");

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

    /* The action: version first (as before), then the identity at the same version. */
    int img = -7;
    CHECK(boot_policy_gateware_action(53, 54, 2, 1, GW_IDENT_SAME, &img) == GW_ACT_WRITE && img == 2,
          "act: older version -> write, whatever the record says");
    img = -7;
    CHECK(boot_policy_gateware_action(54, 54, 2, 1, GW_IDENT_SAME, &img) == GW_ACT_NONE && img == -7,
          "act: same version, recorded image -> nothing");
    CHECK(boot_policy_gateware_action(54, 54, 2, 1, GW_IDENT_DIFFERS, &img) == GW_ACT_WRITE && img == 2,
          "act: same version, new emulation image -> write image 2");
    CHECK(boot_policy_gateware_action(54, 54, 1, 1, GW_IDENT_DIFFERS, &img) == GW_ACT_WRITE && img == 1,
          "act: same version, new deep image -> write image 1");
    CHECK(boot_policy_gateware_action(54, 54, 0, 1, GW_IDENT_UNKNOWN, &img) == GW_ACT_COMPARE && img == 0,
          "act: no record yet -> compare first, never a blind rewrite");
    CHECK(boot_policy_gateware_action(54, 54, 7, 1, GW_IDENT_DIFFERS, &img) == GW_ACT_WRITE && img == 0,
          "act: unknown kind -> image 0");
    img = -7;
    CHECK(boot_policy_gateware_action(54, 54, 0, 1, GW_IDENT_NONE, &img) == GW_ACT_NONE && img == -7,
          "act: slot not the firmware's blob -> nothing");
    CHECK(boot_policy_gateware_action(0, 54, 0, 1, GW_IDENT_DIFFERS, &img) == GW_ACT_NONE,
          "act: configured but not answering -> nothing");
    CHECK(boot_policy_gateware_action(54, 0, 0, 1, GW_IDENT_DIFFERS, &img) == GW_ACT_NONE,
          "act: embedded version unknown -> nothing");
    CHECK(boot_policy_gateware_action(0, 54, 0, 0, GW_IDENT_SAME, &img) == GW_ACT_WRITE && img == 0,
          "act: never configured -> write image 0");

    /* First boot after this update on a pod whose image is already right: the record is blank, so
       one compare, it matches and is recorded; the next boot sees SAME and does nothing. No write. */
    gw_ident_t store;
    memset(&store, 0xFF, sizeof(store));
    int writes = 0, compares = 0;
    for (int boot = 0; boot < 5; boot++) {
        gw_action_t a = boot_policy_gateware_action(54, 54, 0, 1, gw_ident_match(&store, 1, 0, 104090, sha_a), &img);
        if (a == GW_ACT_COMPARE) { compares++; gw_ident_make(&store, 0, 104090, sha_a); }   /* matched */
        if (a == GW_ACT_WRITE) { writes++; gw_ident_make(&store, 0, 104090, sha_a); }
    }
    CHECK(compares == 1 && writes == 0, "storm: matching image -> 1 compare, 0 writes (got %d, %d)", compares, writes);
    /* ... and when it does not match: one write, then recorded. */
    memset(&store, 0xFF, sizeof(store));
    writes = compares = 0;
    for (int boot = 0; boot < 5; boot++) {
        gw_action_t a = boot_policy_gateware_action(54, 54, 0, 1, gw_ident_match(&store, 1, 0, 104090, sha_b), &img);
        if (a == GW_ACT_COMPARE) { compares++; a = GW_ACT_WRITE; }   /* differs: write */
        if (a == GW_ACT_WRITE) { writes++; gw_ident_make(&store, 0, 104090, sha_b); }
    }
    CHECK(compares == 1 && writes == 1, "storm: other image -> 1 compare, 1 write (got %d, %d)", compares, writes);

    if (fails) { printf("test_boot_policy: %d FAILED\n", fails); return 1; }
    printf("test_boot_policy: all passed\n");
    return 0;
}
