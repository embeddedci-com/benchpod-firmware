/*
 * test_pod_policy.c — host tests for the persisted signature and LAN policies (src/pod_policy.c):
 * defaults, who may change what (the ratchet), persistence across a reboot, and that a power
 * cut during a save leaves the previous policy.
 */
#include "pod_policy.h"
#include "fw_sign.h"
#include "mocks/mock_flash.h"

#include <setjmp.h>
#include <stdio.h>
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

static int lan_changes;
static pod_lan_policy_t last_lan;
void pod_policy_on_lan_change(pod_lan_policy_t now) { lan_changes++; last_lan = now; }

/* A fresh boot: reset what the module holds by loading over an erased store. */
static void boot_blank(void) {
    mock_flash_reset();
    /* force the in-RAM copy back to defaults through the USB path, then wipe the flash again */
    pod_policy_set_sig(FW_SIG_POLICY_AUDIT, POLICY_SRC_USB);
    pod_policy_set_lan(POD_LAN_OPEN, POLICY_SRC_USB);
    mock_flash_reset();
    pod_policy_load();
    lan_changes = 0;
}

static void test_defaults(void) {
    boot_blank();
    CHECK(pod_policy_sig() == FW_SIG_POLICY_AUDIT, "no record: signatures %d", pod_policy_sig());
    CHECK(pod_policy_lan() == POD_LAN_OPEN, "no record: LAN %d", pod_policy_lan());
    CHECK(fw_sign_policy() == FW_SIG_POLICY_AUDIT, "fw_sign not audit");
}

static void test_sig_ratchet(void) {
    boot_blank();
    CHECK(pod_policy_set_sig(FW_SIG_POLICY_REQUIRED, POLICY_SRC_LAN) != NULL, "LAN set the signature policy");
    CHECK(pod_policy_sig() == FW_SIG_POLICY_AUDIT, "LAN refusal changed it");

    CHECK(pod_policy_set_sig(FW_SIG_POLICY_PERMISSIVE, POLICY_SRC_CLOUD) == NULL, "cloud could not tighten");
    CHECK(pod_policy_set_sig(FW_SIG_POLICY_REQUIRED, POLICY_SRC_CLOUD) == NULL, "cloud could not tighten again");
    CHECK(fw_sign_policy() == FW_SIG_POLICY_REQUIRED, "fw_sign did not follow");
    const char *why = pod_policy_set_sig(FW_SIG_POLICY_AUDIT, POLICY_SRC_CLOUD);
    CHECK(why && strstr(why, "only the USB console") && strstr(why, "now required"), "cloud loosened: %s",
          why ? why : "(null)");
    CHECK(pod_policy_sig() == FW_SIG_POLICY_REQUIRED, "cloud loosening changed it");
    CHECK(pod_policy_set_sig(FW_SIG_POLICY_REQUIRED, POLICY_SRC_CLOUD) == NULL, "same value from cloud refused");

    CHECK(pod_policy_set_sig(FW_SIG_POLICY_AUDIT, POLICY_SRC_USB) == NULL, "USB could not loosen");
    CHECK(fw_sign_policy() == FW_SIG_POLICY_AUDIT, "fw_sign did not follow USB");
    CHECK(pod_policy_set_sig((fw_sig_policy_t)7, POLICY_SRC_USB) != NULL, "unknown value accepted");
}

static void test_lan(void) {
    boot_blank();
    CHECK(pod_policy_set_lan(POD_LAN_OFF, POLICY_SRC_LAN) != NULL, "LAN set its own policy");
    CHECK(lan_changes == 0, "hook ran on a refusal");
    CHECK(pod_policy_set_lan(POD_LAN_LOCKED, POLICY_SRC_CLOUD) == NULL, "cloud lock");
    CHECK(pod_policy_set_lan(POD_LAN_OFF, POLICY_SRC_CLOUD) == NULL, "cloud off");
    CHECK(pod_policy_set_lan(POD_LAN_OPEN, POLICY_SRC_CLOUD) == NULL, "cloud open again");
    CHECK(lan_changes == 3 && last_lan == POD_LAN_OPEN, "hook %d last %d", lan_changes, last_lan);
    CHECK(pod_policy_set_lan(POD_LAN_OPEN, POLICY_SRC_USB) == NULL && lan_changes == 3, "no-op ran the hook");
    CHECK(pod_policy_set_lan((pod_lan_policy_t)9, POLICY_SRC_USB) != NULL, "unknown LAN value accepted");
}

static void test_persists(void) {
    boot_blank();
    pod_policy_set_sig(FW_SIG_POLICY_REQUIRED, POLICY_SRC_CLOUD);
    pod_policy_set_lan(POD_LAN_LOCKED, POLICY_SRC_CLOUD);
    fw_sign_set_policy(FW_SIG_POLICY_AUDIT);       /* a reboot forgets RAM */
    pod_policy_load();
    CHECK(pod_policy_sig() == FW_SIG_POLICY_REQUIRED && fw_sign_policy() == FW_SIG_POLICY_REQUIRED,
          "signatures after reboot %d", pod_policy_sig());
    CHECK(pod_policy_lan() == POD_LAN_LOCKED, "LAN after reboot %d", pod_policy_lan());
}

/* Power cut at every step of a save: afterwards the pod reads either the old or the new policy,
   never something else, and never crashes. */
static void test_power_cut(void) {
    for (int cut = 1; cut < 40; cut++) {
        boot_blank();
        pod_policy_set_lan(POD_LAN_LOCKED, POLICY_SRC_USB);   /* the policy before the cut */
        mock_flash_cut_after = cut;
        if (setjmp(mock_flash_power_jmp) == 0) {
            pod_policy_set_lan(POD_LAN_OFF, POLICY_SRC_USB);
        }
        mock_flash_cut_after = 0;
        pod_policy_load();
        pod_lan_policy_t now = pod_policy_lan();
        CHECK(now == POD_LAN_LOCKED || now == POD_LAN_OFF, "cut %d: LAN %d", cut, now);
        CHECK(pod_policy_sig() == FW_SIG_POLICY_AUDIT, "cut %d: signatures changed", cut);
    }
}

/* SEC-3: the CA and proxy are cloud-link trust settings; the LAN may never change them, whatever
   the LAN policy (open included). */
static void test_cloud_link_gate(void) {
    boot_blank();
    CHECK(pod_policy_lan() == POD_LAN_OPEN, "precondition: LAN open");
    const char *why = pod_policy_cloud_link_gate("cloud_proxy", POLICY_SRC_LAN);
    CHECK(why && !strcmp(why, "cloud_proxy: change it from the cloud or the USB console"),
          "LAN gate: %s", why ? why : "(null)");
    CHECK(pod_policy_cloud_link_gate("cloud_ca", POLICY_SRC_USB) == NULL, "USB refused");
    CHECK(pod_policy_cloud_link_gate("cloud_ca", POLICY_SRC_CLOUD) == NULL, "cloud refused");
    pod_policy_set_lan(POD_LAN_LOCKED, POLICY_SRC_USB);
    CHECK(pod_policy_cloud_link_gate("cloud_ca", POLICY_SRC_LAN) != NULL, "locked LAN allowed");
    pod_policy_set_lan(POD_LAN_OPEN, POLICY_SRC_USB);
}

static void test_names(void) {
    CHECK(pod_policy_sig_from_name("required") == FW_SIG_POLICY_REQUIRED, "required");
    CHECK(pod_policy_sig_from_name("bogus") == -1 && pod_policy_sig_from_name(NULL) == -1, "bogus sig");
    CHECK(pod_policy_lan_from_name("off") == POD_LAN_OFF && pod_policy_lan_from_name("x") == -1, "lan names");
    CHECK(!strcmp(pod_policy_lan_name(POD_LAN_LOCKED), "locked"), "lan name");
}

int main(void) {
    test_defaults();
    test_sig_ratchet();
    test_lan();
    test_persists();
    test_power_cut();
    test_cloud_link_gate();
    test_names();
    if (failures) { printf("test_pod_policy: %d FAILED\n", failures); return 1; }
    printf("test_pod_policy: all passed\n");
    return 0;
}
