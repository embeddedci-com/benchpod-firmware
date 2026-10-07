/*
 * pod_policy.c — persisted signature and LAN policies (see pod_policy.h).
 */
#include "pod_policy.h"
#include "config_store.h"

#include <stdio.h>
#include <string.h>

typedef struct {
    uint32_t magic;       /* POD_POLICY_RECORD_MAGIC */
    uint8_t  sig;         /* fw_sig_policy_t */
    uint8_t  lan;         /* pod_lan_policy_t */
    uint8_t  reserved[10];
} pod_policy_rec_t;

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

void pod_policy_load(void) {
    pod_policy_rec_t r;
    if (ab_store_load(&k_store, &r, sizeof(r)) == 0 && r.magic == POD_POLICY_RECORD_MAGIC) {
        /* A value from newer firmware this one does not know: take the strictest known one
           rather than silently opening up. */
        if (r.sig > FW_SIG_POLICY_REQUIRED) r.sig = FW_SIG_POLICY_REQUIRED;
        if (r.lan > POD_LAN_OFF) r.lan = POD_LAN_LOCKED;
        s_rec = r;
    }
    fw_sign_set_policy((fw_sig_policy_t)s_rec.sig);
    printf("[policy] signatures %s, LAN %s\n", fw_sign_policy_name((fw_sig_policy_t)s_rec.sig),
           pod_policy_lan_name((pod_lan_policy_t)s_rec.lan));
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
    printf("[policy] signatures now %s (from %s)\n", fw_sign_policy_name(p),
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
    printf("[policy] LAN now %s (from %s)\n", pod_policy_lan_name(p),
           src == POLICY_SRC_USB ? "USB" : "cloud");
    pod_policy_on_lan_change(p);
    return NULL;
}

const char *pod_policy_cloud_link_gate(const char *verb, policy_src_t src) {
    if (src != POLICY_SRC_LAN) return NULL;
    static char why[80];
    snprintf(why, sizeof(why), "%s: change it from the cloud or the USB console", verb);
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
