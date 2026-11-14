/*
 * pod_policy.c — persisted signature and LAN policies (see pod_policy.h).
 */
#include "pod_policy.h"
#include "config_store.h"
#include "bp_log.h"

#include <stdio.h>
#include <string.h>

typedef struct {
    uint32_t magic;       /* POD_POLICY_RECORD_MAGIC */
    uint8_t  sig;         /* fw_sig_policy_t */
    uint8_t  lan;         /* pod_lan_policy_t */
    uint8_t  usb_net_off; /* 1: the USB network adapter is off (0, older records: on) */
    uint8_t  https_only;  /* 1: no plaintext LAN API on Ethernet/Wi-Fi (0, older records: off) */
    /* Firmware 3.10.7+, in what were reserved bytes (zero in older records; older firmware keeps
       them as they are when it saves): */
    uint8_t  lan_unsigned;    /* 1: the external LAN may install images without a good signature */
    uint8_t  reserved[3];
    uint32_t rollback_floor;  /* packed release a signed firmware image must not be below; 0 none */
} pod_policy_rec_t;
_Static_assert(sizeof(pod_policy_rec_t) == 16, "the policy record keeps its size (older firmware loads it)");

static const ab_store_t k_store = {
    .tag = "policy",
    .slot_off = { POD_POLICY_SLOT_A_OFFSET, POD_POLICY_SLOT_B_OFFSET },
    .magic = POD_POLICY_RECORD_MAGIC,
    .version = 1,
    .legacy_off = 0,
    .legacy_magic = 0,
};

static pod_policy_rec_t s_rec = { .magic = POD_POLICY_RECORD_MAGIC, .sig = FW_SIG_POLICY_AUDIT,
                                  .lan = POD_LAN_OPEN };

__attribute__((weak)) void pod_policy_on_lan_change(pod_lan_policy_t now) { (void)now; }
__attribute__((weak)) void pod_policy_on_usb_net_change(bool on) { (void)on; }
__attribute__((weak)) void pod_policy_on_https_only_change(bool on) { (void)on; }

void pod_policy_load(void) {
    pod_policy_rec_t r;
    if (ab_store_load(&k_store, &r, sizeof(r)) == 0 && r.magic == POD_POLICY_RECORD_MAGIC) {
        /* A value from newer firmware this one does not know: take the strictest known one
           rather than silently opening up. */
        if (r.sig > FW_SIG_POLICY_REQUIRED) r.sig = FW_SIG_POLICY_REQUIRED;
        if (r.lan > POD_LAN_OFF) r.lan = POD_LAN_LOCKED;
        if (r.usb_net_off > 1) r.usb_net_off = 1;
        if (r.https_only > 1) r.https_only = 1;
        if (r.lan_unsigned > 1) r.lan_unsigned = 0;   /* unknown: the stricter meaning */
        s_rec = r;
    }
    fw_sign_set_policy((fw_sig_policy_t)s_rec.sig);
    fw_sign_set_lan_unsigned(s_rec.lan_unsigned != 0);
    fw_sign_set_floor(s_rec.rollback_floor);
    char fl[16];
    log_printf("[policy] signatures %s, LAN %s, USB network %s, HTTPS-only %s, LAN images %s, floor %s\n",
           fw_sign_policy_name((fw_sig_policy_t)s_rec.sig), pod_policy_lan_name((pod_lan_policy_t)s_rec.lan),
           s_rec.usb_net_off ? "off" : "on", s_rec.https_only ? "on" : "off",
           s_rec.lan_unsigned ? "any" : "signed", s_rec.rollback_floor ? fw_sign_release_str(s_rec.rollback_floor, fl, sizeof(fl)) : "none");
}

fw_sig_policy_t  pod_policy_sig(void) { return (fw_sig_policy_t)s_rec.sig; }
pod_lan_policy_t pod_policy_lan(void) { return (pod_lan_policy_t)s_rec.lan; }

static const char *save(const pod_policy_rec_t *r) {
    if (ab_store_save(&k_store, r, sizeof(*r)) != 0) return "could not save the policy";
    s_rec = *r;
    return NULL;
}

const char *pod_policy_set_sig(fw_sig_policy_t p, policy_src_t src) {
    if ((unsigned)p > FW_SIG_POLICY_REQUIRED) return "sig_policy: unknown policy";
    if (src == POLICY_SRC_LAN) return "sig_policy: change it from the cloud or the USB console";
    if (src == POLICY_SRC_CLOUD && p < pod_policy_sig()) {
        static char why[96];
        snprintf(why, sizeof(why), "sig_policy: only the USB console can loosen the policy (now %s)",
                 fw_sign_policy_name(pod_policy_sig()));
        return why;
    }
    if (p == pod_policy_sig()) return NULL;
    pod_policy_rec_t r = s_rec;
    r.sig = (uint8_t)p;
    const char *why = save(&r);
    if (why) return why;
    fw_sign_set_policy(p);
    log_printf("[policy] signatures now %s (from %s)\n", fw_sign_policy_name(p),
           src == POLICY_SRC_USB ? "USB" : "cloud");
    return NULL;
}

const char *pod_policy_set_lan(pod_lan_policy_t p, policy_src_t src) {
    if ((unsigned)p > POD_LAN_OFF) return "lan_policy: unknown policy";
    if (src == POLICY_SRC_LAN) return "lan_policy: change it from the cloud or the USB console";
    if (p == pod_policy_lan()) return NULL;
    pod_policy_rec_t r = s_rec;
    r.lan = (uint8_t)p;
    const char *why = save(&r);
    if (why) return why;
    log_printf("[policy] LAN now %s (from %s)\n", pod_policy_lan_name(p),
           src == POLICY_SRC_USB ? "USB" : "cloud");
    pod_policy_on_lan_change(p);
    return NULL;
}

bool pod_policy_usb_net(void) { return s_rec.usb_net_off == 0; }

const char *pod_policy_set_usb_net(bool on, policy_src_t src, bool usb_link) {
    if (src == POLICY_SRC_CLOUD || (src == POLICY_SRC_LAN && !usb_link))
        return "usb_network: change it on the device (the USB console, or over the USB link)";
    if (on == pod_policy_usb_net()) return NULL;
    pod_policy_rec_t r = s_rec;
    r.usb_net_off = on ? 0 : 1;
    const char *why = save(&r);
    if (why) return why;
    log_printf("[policy] USB network now %s (from %s)\n", on ? "on" : "off",
           src == POLICY_SRC_USB ? "the USB console" : "the USB link");
    pod_policy_on_usb_net_change(on);
    return NULL;
}

bool pod_policy_https_only(void) { return s_rec.https_only != 0; }

const char *pod_policy_lan_tls_gate(policy_src_t src, bool usb_link) {
    /* Not "or over TLS": port 8443 has no client authentication, so a TLS connection proves
       nothing about who is on the other end (security review 2026-11, M10). */
    if (src != POLICY_SRC_LAN || usb_link) return NULL;
    return "lan_tls: change it from the cloud or over USB";
}

const char *pod_policy_set_https_only(bool on, policy_src_t src, bool usb_link) {
    /* Both ways: on cuts off every plaintext client, off is what a man-in-the-middle wants. */
    const char *why = pod_policy_lan_tls_gate(src, usb_link);
    if (why) return why;
    if (on == pod_policy_https_only()) return NULL;
    pod_policy_rec_t r = s_rec;
    r.https_only = on ? 1 : 0;
    why = save(&r);
    if (why) return why;
    log_printf("[policy] HTTPS-only now %s (from %s)\n", on ? "on" : "off",
               src == POLICY_SRC_USB ? "USB" : src == POLICY_SRC_CLOUD ? "cloud" : "the USB link");
    pod_policy_on_https_only_change(on);
    return NULL;
}

bool pod_policy_lan_unsigned(void) { return s_rec.lan_unsigned != 0; }

const char *pod_policy_set_lan_unsigned(bool on, policy_src_t src, bool usb_link) {
    /* Loosening needs physical presence; the cloud may only tighten (like sig_policy). */
    if (src == POLICY_SRC_LAN && !usb_link)
        return "lan_ota: change it on the device (the USB console, or over the USB link)";
    if (on && src == POLICY_SRC_CLOUD)
        return "lan_ota: only the USB console or the USB link can allow unsigned LAN updates";
    if (on == pod_policy_lan_unsigned()) return NULL;
    pod_policy_rec_t r = s_rec;
    r.lan_unsigned = on ? 1 : 0;
    const char *why = save(&r);
    if (why) return why;
    fw_sign_set_lan_unsigned(on);
    log_printf("[policy] LAN images now %s (from %s)\n", on ? "any" : "signed only",
               src == POLICY_SRC_USB ? "the USB console" : src == POLICY_SRC_CLOUD ? "cloud" : "the USB link");
    return NULL;
}

uint32_t pod_policy_rollback_floor(void) { return s_rec.rollback_floor; }

const char *pod_policy_raise_floor(uint32_t release) {
    if (release <= s_rec.rollback_floor) return NULL;   /* never lowers */
    pod_policy_rec_t r = s_rec;
    r.rollback_floor = release;
    const char *why = save(&r);
    if (why) return why;
    fw_sign_set_floor(release);
    char fl[16];
    log_printf("[policy] rollback floor now %s\n", fw_sign_release_str(release, fl, sizeof(fl)));
    return NULL;
}

const char *pod_policy_clear_floor(policy_src_t src) {
    if (src != POLICY_SRC_USB) return "rollback_floor: clear it on the USB console";
    if (s_rec.rollback_floor == 0) return NULL;
    pod_policy_rec_t r = s_rec;
    r.rollback_floor = 0;
    const char *why = save(&r);
    if (why) return why;
    fw_sign_set_floor(0);
    log_printf("[policy] rollback floor cleared (from the USB console)\n");
    return NULL;
}

const char *pod_policy_cloud_link_gate(const char *verb, policy_src_t src, bool usb_link) {
    if (src != POLICY_SRC_LAN || usb_link) return NULL;
    static char why[80];
    snprintf(why, sizeof(why), "%s: change it from the cloud or over USB", verb);
    return why;
}

int pod_policy_sig_from_name(const char *name) {
    if (!name) return -1;
    if (!strcmp(name, "audit")) return FW_SIG_POLICY_AUDIT;
    if (!strcmp(name, "permissive")) return FW_SIG_POLICY_PERMISSIVE;
    if (!strcmp(name, "required")) return FW_SIG_POLICY_REQUIRED;
    return -1;
}

int pod_policy_lan_from_name(const char *name) {
    if (!name) return -1;
    if (!strcmp(name, "open")) return POD_LAN_OPEN;
    if (!strcmp(name, "locked")) return POD_LAN_LOCKED;
    if (!strcmp(name, "off")) return POD_LAN_OFF;
    return -1;
}

const char *pod_policy_lan_name(pod_lan_policy_t p) {
    switch (p) {
    case POD_LAN_OPEN:   return "open";
    case POD_LAN_LOCKED: return "locked";
    case POD_LAN_OFF:    return "off";
    }
    return "?";
}
