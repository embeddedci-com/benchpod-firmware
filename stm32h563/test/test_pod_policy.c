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

static int usb_net_changes;
static bool last_usb_net = true;
void pod_policy_on_usb_net_change(bool on) { usb_net_changes++; last_usb_net = on; }

/* A fresh boot: reset what the module holds by loading over an erased store. */
static void boot_blank(void) {
    mock_flash_reset();
    /* force the in-RAM copy back to defaults through the USB path, then wipe the flash again */
    pod_policy_set_sig(FW_SIG_POLICY_AUDIT, POLICY_SRC_USB);
    pod_policy_set_lan(POD_LAN_OPEN, POLICY_SRC_USB);
    pod_policy_set_usb_net(true, POLICY_SRC_USB, false);
    pod_policy_set_lan_unsigned(false, POLICY_SRC_USB, false);
    pod_policy_clear_floor(POLICY_SRC_USB);
    mock_flash_reset();
    pod_policy_load();
    lan_changes = 0;
    usb_net_changes = 0;
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
    const char *why = pod_policy_cloud_link_gate("cloud_proxy", POLICY_SRC_LAN, false);
    CHECK(why && !strcmp(why, "cloud_proxy: change it from the cloud or over USB"),
          "LAN gate: %s", why ? why : "(null)");
    CHECK(pod_policy_cloud_link_gate("cloud_ca", POLICY_SRC_USB, false) == NULL, "USB refused");
    CHECK(pod_policy_cloud_link_gate("cloud_ca", POLICY_SRC_CLOUD, false) == NULL, "cloud refused");
    /* the USB network adapter is physical presence for these two settings, under every LAN policy */
    CHECK(pod_policy_cloud_link_gate("cloud_proxy", POLICY_SRC_LAN, true) == NULL, "USB network refused");
    pod_policy_set_lan(POD_LAN_LOCKED, POLICY_SRC_USB);
    CHECK(pod_policy_cloud_link_gate("cloud_ca", POLICY_SRC_LAN, false) != NULL, "locked LAN allowed");
    CHECK(pod_policy_cloud_link_gate("cloud_ca", POLICY_SRC_LAN, true) == NULL, "USB network refused under a locked LAN");
    pod_policy_set_lan(POD_LAN_OPEN, POLICY_SRC_USB);
}

static void test_names(void) {
    CHECK(pod_policy_sig_from_name("required") == FW_SIG_POLICY_REQUIRED, "required");
    CHECK(pod_policy_sig_from_name("bogus") == -1 && pod_policy_sig_from_name(NULL) == -1, "bogus sig");
    CHECK(pod_policy_lan_from_name("off") == POD_LAN_OFF && pod_policy_lan_from_name("x") == -1, "lan names");
    CHECK(!strcmp(pod_policy_lan_name(POD_LAN_LOCKED), "locked"), "lan name");
}

/* The USB network setting: on by default; only the device changes it (the console, or the USB
   link itself), never the cloud or Ethernet/Wi-Fi; it survives a reboot and is separate from
   the LAN policy. */
static void test_usb_net(void) {
    boot_blank();
    CHECK(pod_policy_usb_net(), "default must be on");
    const char *why = pod_policy_set_usb_net(false, POLICY_SRC_CLOUD, false);
    CHECK(why && strstr(why, "on the device"), "cloud allowed: %s", why ? why : "(null)");
    why = pod_policy_set_usb_net(false, POLICY_SRC_LAN, false);
    CHECK(why != NULL, "Ethernet/Wi-Fi allowed");
    CHECK(pod_policy_usb_net() && usb_net_changes == 0, "a refused change changed it");
    CHECK(pod_policy_set_usb_net(false, POLICY_SRC_LAN, true) == NULL, "the USB link refused");
    CHECK(!pod_policy_usb_net() && usb_net_changes == 1 && !last_usb_net, "off not applied");
    CHECK(pod_policy_lan() == POD_LAN_OPEN && lan_changes == 0, "the LAN policy moved");
    CHECK(pod_policy_set_usb_net(false, POLICY_SRC_USB, false) == NULL && usb_net_changes == 1,
          "same value must not re-enumerate");
    pod_policy_load();   /* reboot */
    CHECK(!pod_policy_usb_net(), "off lost on reboot");
    CHECK(pod_policy_set_usb_net(true, POLICY_SRC_USB, false) == NULL && pod_policy_usb_net() &&
          last_usb_net, "the console could not turn it back on");
}

/* Security review 2026-11 M10/L14: port 8443 has no client authentication, so neither a TLS nor
   a plaintext LAN connection may change the LAN TLS identity or HTTPS-only (either way). */
static void test_lan_tls_gate(void) {
    boot_blank();
    CHECK(pod_policy_lan_tls_gate(POLICY_SRC_LAN, false) != NULL, "external LAN may change LAN TLS");
    CHECK(pod_policy_lan_tls_gate(POLICY_SRC_LAN, true) == NULL, "USB link refused");
    CHECK(pod_policy_lan_tls_gate(POLICY_SRC_USB, false) == NULL, "USB console refused");
    CHECK(pod_policy_lan_tls_gate(POLICY_SRC_CLOUD, false) == NULL, "cloud refused");
    CHECK(pod_policy_set_https_only(true, POLICY_SRC_LAN, false) != NULL, "LAN turned HTTPS-only on");
    CHECK(!pod_policy_https_only(), "a refused change changed it");
    CHECK(pod_policy_set_https_only(true, POLICY_SRC_LAN, true) == NULL && pod_policy_https_only(), "USB link");
    CHECK(pod_policy_set_https_only(false, POLICY_SRC_LAN, false) != NULL && pod_policy_https_only(),
          "LAN turned HTTPS-only off");
    CHECK(pod_policy_set_https_only(false, POLICY_SRC_CLOUD, false) == NULL && !pod_policy_https_only(), "cloud");
}

/* Security review 2026-11 H3/M7: "LAN images any" loosens, so only the device turns it on; the
   floor only goes up from an image and is cleared on the console; both survive a reboot. */
static void test_lan_unsigned_and_floor(void) {
    boot_blank();
    CHECK(!pod_policy_lan_unsigned() && !fw_sign_lan_unsigned(), "default must be signed only");
    CHECK(pod_policy_set_lan_unsigned(true, POLICY_SRC_LAN, false) != NULL, "Ethernet/Wi-Fi turned it on");
    CHECK(pod_policy_set_lan_unsigned(true, POLICY_SRC_CLOUD, false) != NULL, "the cloud turned it on");
    CHECK(!pod_policy_lan_unsigned(), "a refused change changed it");
    CHECK(pod_policy_set_lan_unsigned(true, POLICY_SRC_LAN, true) == NULL && fw_sign_lan_unsigned(), "USB link");
    pod_policy_load();
    CHECK(pod_policy_lan_unsigned() && fw_sign_lan_unsigned(), "lost on reboot");
    CHECK(pod_policy_set_lan_unsigned(false, POLICY_SRC_CLOUD, false) == NULL && !fw_sign_lan_unsigned(),
          "the cloud could not tighten");

    CHECK(pod_policy_rollback_floor() == 0 && fw_sign_floor() == 0, "default floor");
    CHECK(pod_policy_raise_floor(0x030A07) == NULL && fw_sign_floor() == 0x030A07, "raise");
    CHECK(pod_policy_raise_floor(0x030A05) == NULL && fw_sign_floor() == 0x030A07, "the floor went down");
    pod_policy_load();
    CHECK(pod_policy_rollback_floor() == 0x030A07 && fw_sign_floor() == 0x030A07, "floor lost on reboot");
    CHECK(pod_policy_clear_floor(POLICY_SRC_CLOUD) != NULL && pod_policy_clear_floor(POLICY_SRC_LAN) != NULL,
          "cleared from off the console");
    CHECK(pod_policy_clear_floor(POLICY_SRC_USB) == NULL && fw_sign_floor() == 0, "console clear");
    CHECK(pod_policy_sig() == FW_SIG_POLICY_AUDIT && pod_policy_lan() == POD_LAN_OPEN, "other policies moved");
}

int main(void) {
    test_defaults();
    test_lan_unsigned_and_floor();
    test_lan_tls_gate();
    test_sig_ratchet();
    test_lan();
    test_persists();
    test_power_cut();
    test_cloud_link_gate();
    test_usb_net();
    test_names();
    if (failures) { printf("test_pod_policy: %d FAILED\n", failures); return 1; }
    printf("test_pod_policy: all passed\n");
    return 0;
}
