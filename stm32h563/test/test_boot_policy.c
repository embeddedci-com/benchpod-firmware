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
    CHECK(boot_policy_gateware_image(37, 36, 0, 1) == 0, "gw: newer gateware than the firmware -> match the firmware");
    CHECK(boot_policy_gateware_image(0, 36, 0, 1) == -1, "gw: configured but not answering -> leave it (not a blank flash)");
    CHECK(boot_policy_gateware_image(35, 0, 0, 1) == -1, "gw: embedded version unknown -> never touch it");
    /* A new board: the config flash is blank, so the iCE40 never configured and answers v0. The
       blobs arrive after the firmware (flash-self installs them once it boots), so this runs again
       when gw0 is installed: load image 0 then, not only at the next boot. */
    CHECK(boot_policy_gateware_image(0, 48, 0, 0) == 0, "gw: iCE40 never configured -> load image 0");
    CHECK(boot_policy_gateware_image(0, 0, 0, 0) == 0, "gw: never configured, embedded unknown -> still image 0");
    CHECK(boot_policy_gateware_image(48, 48, 1, 0) == 0, "gw: CDONE low wins over a stale version");

    if (fails) { printf("test_boot_policy: %d FAILED\n", fails); return 1; }
    printf("test_boot_policy: all passed\n");
    return 0;
}
