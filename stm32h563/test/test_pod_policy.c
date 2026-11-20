/*
 * test_pod_policy.c — host tests for the persisted signature and LAN policies (src/pod_policy.c):
 * defaults, who may change what (the ratchet), persistence across a reboot, and that a power
 * cut during a save leaves the previous policy.
 */
#include "pod_policy.h"
#include "fw_sign.h"
#include "config_store.h"
#include "mocks/mock_flash.h"
#include "bp_errors.h"

#include <setjmp.h>
#include <stdio.h>
#include <string.h>

static int failures;

/* The code a refusal was noted with (what send_error sends), and its first string arg. */
static const char *g_arg0;
static int code_of(const char *why) {
    bp_errcode_t c = BP_E_NONE;
    size_t n = 0;
    const bp_err_arg_t *a = why ? bp_err_noted(why, &c, &n) : NULL;
    g_arg0 = a && n && a[0].is_str ? a[0].s : "";
    return a ? (int)c : 0;
}

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
    pod_policy_set_lan(POD_LAN_OPEN, POLICY_SRC_USB);
    pod_policy_set_usb_net(true, POLICY_SRC_USB, false);
    pod_policy_set_https_only(true, POLICY_SRC_USB, false);
    pod_policy_set_scpi(false, POLICY_SRC_USB, false);
    pod_policy_clear_floor(POLICY_SRC_USB);
    pod_policy_set_bridge_pin(NULL, NULL);
    pod_policy_set_log_level(BP_LOG_INFO);
    mock_flash_reset();
    pod_policy_load();
    lan_changes = 0;
    usb_net_changes = 0;
}

static void test_defaults(void) {
    boot_blank();
    CHECK(pod_policy_sig() == FW_SIG_POLICY_REQUIRED, "no record: signatures %d", pod_policy_sig());
    CHECK(pod_policy_lan() == POD_LAN_OPEN, "no record: LAN %d", pod_policy_lan());
    CHECK(fw_sign_policy() == FW_SIG_POLICY_REQUIRED, "fw_sign not required");
}

/* Firmware 4.0.0: "required" always. Setting it again works from every source (a server pushes
   it); audit and permissive are refused from every source, the USB console included (BP0074). A
   record older firmware wrote with audit still loads as required. */
static void test_sig_always_required(void) {
    boot_blank();
    const policy_src_t srcs[] = { POLICY_SRC_USB, POLICY_SRC_CLOUD, POLICY_SRC_LAN };
    for (size_t i = 0; i < sizeof(srcs) / sizeof(srcs[0]); i++) {
        CHECK(pod_policy_set_sig(FW_SIG_POLICY_REQUIRED, srcs[i]) == NULL, "src %zu: required refused", i);
        const char *why = pod_policy_set_sig(FW_SIG_POLICY_AUDIT, srcs[i]);
        CHECK(code_of(why) == BP_E_SIGNATURES_ALWAYS_REQUIRED && !strcmp(g_arg0, "sig_policy"),
              "src %zu loosened to audit: %s", i, why ? why : "(null)");
        CHECK(code_of(pod_policy_set_sig(FW_SIG_POLICY_PERMISSIVE, srcs[i])) == BP_E_SIGNATURES_ALWAYS_REQUIRED,
              "src %zu loosened to permissive", i);
        CHECK(pod_policy_sig() == FW_SIG_POLICY_REQUIRED && fw_sign_policy() == FW_SIG_POLICY_REQUIRED,
              "src %zu: a refusal changed it", i);
    }
    CHECK(pod_policy_set_sig((fw_sig_policy_t)7, POLICY_SRC_USB) != NULL, "unknown value accepted");

    /* A 3.10.x record: audit, and "LAN images any" (byte 6). */
    static const ab_store_t v1 = { .tag = "old", .slot_off = { POD_POLICY_SLOT_A_OFFSET, POD_POLICY_SLOT_B_OFFSET },
                                   .magic = POD_POLICY_RECORD_MAGIC, .version = 1 };
    uint8_t rec[16] = {0};
    const uint32_t magic = POD_POLICY_RECORD_MAGIC;
    memcpy(rec, &magic, 4);
    rec[4] = FW_SIG_POLICY_AUDIT;
    rec[8] = 1;   /* lan_unsigned */
    mock_flash_reset();
    CHECK(ab_store_save(&v1, rec, sizeof(rec)) == 0, "old save");
    pod_policy_load();
    CHECK(pod_policy_sig() == FW_SIG_POLICY_REQUIRED && fw_sign_policy() == FW_SIG_POLICY_REQUIRED,
          "an old audit record loosened it: %d", pod_policy_sig());
    /* The next save writes required (and no lan_unsigned) for firmware rolled back to 3.10.x. */
    CHECK(pod_policy_set_lan(POD_LAN_LOCKED, POLICY_SRC_USB) == NULL, "lan");
    CHECK(ab_store_load(&v1, rec, sizeof(rec)) == 0 && rec[4] == FW_SIG_POLICY_REQUIRED && rec[8] == 0,
          "saved sig %u lan_unsigned %u", rec[4], rec[8]);
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
        CHECK(pod_policy_sig() == FW_SIG_POLICY_REQUIRED, "cut %d: signatures changed", cut);
    }
}

/* SEC-3: the CA and proxy are cloud-link trust settings; the LAN may never change them, whatever
   the LAN policy (open included). */
static void test_cloud_link_gate(void) {
    boot_blank();
    CHECK(pod_policy_lan() == POD_LAN_OPEN, "precondition: LAN open");
    const char *why = pod_policy_cloud_link_gate("cloud_proxy", POLICY_SRC_LAN, false);
    CHECK(code_of(why) == BP_E_SETTING_CLOUD_OR_USB && !strcmp(g_arg0, "cloud_proxy"),
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
    CHECK(code_of(why) == BP_E_SETTING_DEVICE_ONLY && !strcmp(g_arg0, "usb_network"), "cloud allowed: %s",
          why ? why : "(null)");
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
    /* Firmware 4.0.0: HTTPS-only on and SCPI off with no record. */
    CHECK(pod_policy_https_only() && !pod_policy_scpi(), "no record: HTTPS-only %d SCPI %d",
          pod_policy_https_only(), pod_policy_scpi());
    CHECK(pod_policy_set_https_only(false, POLICY_SRC_LAN, false) != NULL, "LAN turned plaintext on");
    CHECK(pod_policy_https_only(), "a refused change changed it");
    CHECK(pod_policy_set_https_only(false, POLICY_SRC_LAN, true) == NULL && !pod_policy_https_only(), "USB link");
    CHECK(pod_policy_set_https_only(true, POLICY_SRC_LAN, false) != NULL && !pod_policy_https_only(),
          "LAN turned HTTPS-only on");
    CHECK(pod_policy_set_https_only(true, POLICY_SRC_CLOUD, false) == NULL && pod_policy_https_only(), "cloud");
    /* SCPI: the same gate, persisted. */
    CHECK(pod_policy_set_scpi(true, POLICY_SRC_LAN, false) != NULL && !pod_policy_scpi(), "LAN turned SCPI on");
    CHECK(pod_policy_set_scpi(true, POLICY_SRC_CLOUD, false) == NULL && pod_policy_scpi(), "cloud SCPI on");
    CHECK(pod_policy_set_scpi(false, POLICY_SRC_LAN, true) == NULL && !pod_policy_scpi(), "USB link SCPI off");
    CHECK(pod_policy_set_scpi(true, POLICY_SRC_USB, false) == NULL, "console SCPI on");
    CHECK(pod_policy_set_https_only(false, POLICY_SRC_USB, false) == NULL, "console plaintext on");
    pod_policy_load();
    CHECK(pod_policy_scpi() && !pod_policy_https_only(), "lost on reboot");
    /* Older firmware reads HTTPS-only from byte 7: kept in step (0 = plaintext on). */
    static const ab_store_t v1 = { .tag = "old", .slot_off = { POD_POLICY_SLOT_A_OFFSET, POD_POLICY_SLOT_B_OFFSET },
                                   .magic = POD_POLICY_RECORD_MAGIC, .version = 1 };
    uint8_t rec[16];
    CHECK(ab_store_load(&v1, rec, sizeof(rec)) == 0 && rec[7] == 0 && rec[10] == 1 && rec[11] == 1,
          "record https_only %u lan_plain %u scpi %u", rec[7], rec[10], rec[11]);
    CHECK(pod_policy_set_https_only(true, POLICY_SRC_USB, false) == NULL, "console HTTPS-only on");
    CHECK(ab_store_load(&v1, rec, sizeof(rec)) == 0 && rec[7] == 1 && rec[10] == 0, "record after on");
    /* A 3.10.x record with HTTPS-only off (byte 7 = 0, the old default) loads as HTTPS-only on. */
    memset(rec, 0, sizeof(rec));
    const uint32_t magic = POD_POLICY_RECORD_MAGIC;
    memcpy(rec, &magic, 4);
    mock_flash_reset();
    CHECK(ab_store_save(&v1, rec, sizeof(rec)) == 0, "old save");
    pod_policy_load();
    CHECK(pod_policy_https_only() && !pod_policy_scpi(), "an old record opened plaintext or SCPI");
}

/* Security review 2026-11 M7: the floor only goes up from an image and is cleared on the console;
   it survives a reboot. */
static void test_floor(void) {
    boot_blank();
    CHECK(pod_policy_rollback_floor() == 0 && fw_sign_floor() == 0, "default floor");
    CHECK(pod_policy_raise_floor(0x030A07) == NULL && fw_sign_floor() == 0x030A07, "raise");
    CHECK(pod_policy_raise_floor(0x030A05) == NULL && fw_sign_floor() == 0x030A07, "the floor went down");
    pod_policy_load();
    CHECK(pod_policy_rollback_floor() == 0x030A07 && fw_sign_floor() == 0x030A07, "floor lost on reboot");
    CHECK(pod_policy_clear_floor(POLICY_SRC_CLOUD) != NULL && pod_policy_clear_floor(POLICY_SRC_LAN) != NULL,
          "cleared from off the console");
    CHECK(pod_policy_clear_floor(POLICY_SRC_USB) == NULL && fw_sign_floor() == 0, "console clear");
    CHECK(pod_policy_sig() == FW_SIG_POLICY_REQUIRED && pod_policy_lan() == POD_LAN_OPEN, "other policies moved");
}

/* The stored genuine Wi-Fi bridge (H4) lives in a version 2 record; a version 1 copy stays the newest so
   older firmware keeps loading the policies, and their save drops only the pin. */
static void test_bridge_pin(void) {
    boot_blank();
    uint8_t mac[6] = {0x24, 0x0a, 0xc4, 1, 2, 3}, pub[32], m2[6], p2[32];
    memset(pub, 0x77, sizeof(pub));
    CHECK(!pod_policy_bridge_pin(m2, p2), "pinned by default");
    pod_policy_set_lan(POD_LAN_LOCKED, POLICY_SRC_USB);
    CHECK(pod_policy_set_bridge_pin(mac, pub) == NULL, "pin");
    pod_policy_load();
    CHECK(pod_policy_bridge_pin(m2, p2) && !memcmp(m2, mac, 6) && !memcmp(p2, pub, 32), "pin lost on reboot");
    CHECK(pod_policy_lan() == POD_LAN_LOCKED, "policy lost next to the pin");
    /* older firmware reads the version 1 record: it must be the newest and carry the policy */
    static const ab_store_t v1 = { .tag = "old", .slot_off = { POD_POLICY_SLOT_A_OFFSET, POD_POLICY_SLOT_B_OFFSET },
                                   .magic = POD_POLICY_RECORD_MAGIC, .version = 1 };
    uint8_t rec[16];
    CHECK(ab_store_load(&v1, rec, sizeof(rec)) == 0 && rec[5] == POD_LAN_LOCKED, "older firmware cannot load it");
    /* ...and its own save (a policy change there) overwrites the version 2 record */
    rec[5] = POD_LAN_OPEN;
    CHECK(ab_store_save(&v1, rec, sizeof(rec)) == 0, "old save");
    pod_policy_load();
    CHECK(!pod_policy_bridge_pin(m2, p2) && pod_policy_lan() == POD_LAN_OPEN, "after an older firmware's save");
    CHECK(pod_policy_set_bridge_pin(mac, pub) == NULL && pod_policy_set_bridge_pin(NULL, NULL) == NULL, "clear");
    pod_policy_load();
    CHECK(!pod_policy_bridge_pin(m2, p2), "clear not kept");
}

/* The console log level: default info, any value persists across a reboot, the other policies
   are untouched, an unknown value is refused. */
static void test_log_level(void) {
    boot_blank();
    CHECK(log_level() == BP_LOG_INFO, "no record: log level %d", log_level());
    CHECK(pod_policy_set_log_level(BP_LOG_DEBUG) == NULL, "debug refused");
    CHECK(log_level() == BP_LOG_DEBUG, "debug not applied");
    log_set_level(BP_LOG_ERROR);   /* the RAM copy only: the reboot reads the record */
    pod_policy_load();
    CHECK(log_level() == BP_LOG_DEBUG, "debug lost on reboot: %d", log_level());
    CHECK(pod_policy_set_log_level(BP_LOG_WARN) == NULL && log_level() == BP_LOG_WARN, "warn");
    pod_policy_load();
    CHECK(log_level() == BP_LOG_WARN, "warn lost on reboot");
    CHECK(pod_policy_lan() == POD_LAN_OPEN && pod_policy_usb_net(), "another policy moved");
    CHECK(pod_policy_set_log_level((bp_log_level_t)7) != NULL, "an unknown level was taken");
    CHECK(pod_policy_set_log_level(BP_LOG_INFO) == NULL, "info refused");
    pod_policy_load();
    CHECK(log_level() == BP_LOG_INFO, "info lost on reboot");
    /* A policy change keeps the level. */
    CHECK(pod_policy_set_log_level(BP_LOG_DEBUG) == NULL, "debug refused");
    CHECK(pod_policy_set_lan(POD_LAN_LOCKED, POLICY_SRC_USB) == NULL, "lan");
    pod_policy_load();
    CHECK(log_level() == BP_LOG_DEBUG && pod_policy_lan() == POD_LAN_LOCKED, "level or LAN lost");
    boot_blank();
}

int main(void) {
    test_log_level();
    test_defaults();
    test_bridge_pin();
    test_floor();
    test_lan_tls_gate();
    test_sig_always_required();
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
