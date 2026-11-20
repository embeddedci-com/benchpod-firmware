/*
 * pod_policy.c — persisted signature and LAN policies (see pod_policy.h).
 */
#include "pod_policy.h"
#include "bp_errors.h"   /* the LAN/cloud refusals are catalog codes (bp_note_*) */
#include "config_store.h"
#include "bp_log.h"

#include <stdio.h>
#include <string.h>

typedef struct {
    uint32_t magic;       /* POD_POLICY_RECORD_MAGIC */
    uint8_t  sig;         /* fw_sig_policy_t */
    uint8_t  lan;         /* pod_lan_policy_t */
    uint8_t  usb_net_off; /* 1: the USB network adapter is off (0, older records: on) */
    uint8_t  https_only;  /* 1: no plaintext LAN API on Ethernet/Wi-Fi. Firmware 4.0.0 writes it from
                             lan_plain (older firmware reads it) and ignores it on load. */
    /* Firmware 3.10.7+, in what were reserved bytes (zero in older records; older firmware keeps
       them as they are when it saves): */
    uint8_t  lan_unsigned;    /* 3.10.7 to 3.10.13: 1 = the external LAN may install images without a
                                 good signature. Firmware 4.0.0 ignores it and writes 0. */
    /* After 3.10.12, in the next reserved byte: */
    uint8_t  log_level;       /* console log level (log_level.h stored form): 0 = default (info) */
    /* Firmware 4.0.0, in the last two reserved bytes (0 in older records: both off): */
    uint8_t  lan_plain;       /* 1: the plaintext JSON API on :8080 answers on Ethernet/Wi-Fi too */
    uint8_t  scpi_on;         /* 1: SCPI answers on the pod's TCP connections */
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

/* Version 2 (firmware 3.10.7+): the version 1 record plus the last genuine Wi-Fi bridge. Every save
   writes it and then a version 1 copy as the newest record, so older firmware still loads its
   policies (adc_cal.c does the same). Older firmware that saves overwrites the version 2 record:
   the pin is gone then and the pod adopts its bridge again (wifi_bridge.h). */
typedef struct {
    pod_policy_rec_t v1;
    uint8_t  bridge_mac[6];
    uint8_t  bridge_pinned;   /* 1: bridge_mac/bridge_pub hold a bridge whose identity was verified */
    uint8_t  pad;
    uint8_t  bridge_pub[32];
    /* Firmware 4.0.1+, in the first reserved byte (0 in older records and after older firmware
       saves: pairing on, the stricter meaning): */
    uint8_t  lan_pairing_off; /* 1: LAN computers need no pairing (lan_clients.h) */
    uint8_t  reserved[7];
} pod_policy_rec2_t;
_Static_assert(sizeof(pod_policy_rec2_t) == 64, "policy record v2 layout");

static const ab_store_t k_store2 = {
    .tag = "policy",
    .slot_off = { POD_POLICY_SLOT_A_OFFSET, POD_POLICY_SLOT_B_OFFSET },
    .magic = POD_POLICY_RECORD_MAGIC,
    .version = 2,
    .legacy_off = 0,
    .legacy_magic = 0,
};

/* The pin is not kept in RAM (the bridge client holds it): read from the version 2 record. */
static bool load_pin(pod_policy_rec2_t *r2) {
    uint32_t behind;
    return ab_store_load_version(&k_store2, 2, r2, sizeof(*r2), &behind) == 0 &&
           r2->v1.magic == POD_POLICY_RECORD_MAGIC && r2->bridge_pinned == 1;
}

static pod_policy_rec_t s_rec = { .magic = POD_POLICY_RECORD_MAGIC, .sig = FW_SIG_POLICY_REQUIRED,
                                  .lan = POD_LAN_OPEN };

/* The one buffer for a formatted refusal: callers (the hw worker, one command at a time) send it
   before asking again. */
static char s_why[112];   /* the longest, a catalog message with a setting name, is about 90 */

/* Kept here, not in s_rec: it lives in the version 2 record only. */
static uint8_t s_lan_pairing_off;

__attribute__((weak)) void pod_policy_on_lan_change(pod_lan_policy_t now) { (void)now; }
__attribute__((weak)) void pod_policy_on_usb_net_change(bool on) { (void)on; }
__attribute__((weak)) void pod_policy_on_https_only_change(bool on) { (void)on; }
__attribute__((weak)) void pod_policy_on_scpi_change(bool on) { (void)on; }

void pod_policy_load(void) {
    pod_policy_rec_t r;
    pod_policy_rec2_t r2;
    uint32_t b1 = 0, b2 = 0;
    bool h1 = ab_store_load_version(&k_store, 1, &r, sizeof(r), &b1) == 0 && r.magic == POD_POLICY_RECORD_MAGIC;
    bool h2 = ab_store_load_version(&k_store2, 2, &r2, sizeof(r2), &b2) == 0 &&
              r2.v1.magic == POD_POLICY_RECORD_MAGIC;
    if (h2 && (!h1 || b2 < b1)) { r = r2.v1; h1 = true; }   /* the newer of the two */
    s_lan_pairing_off = h2 && r2.lan_pairing_off == 1 ? 1 : 0;   /* anything else: pairing on */
    if (h1) {
        /* A value from newer firmware this one does not know: take the strictest known one
           rather than silently opening up. */
        if (r.sig > FW_SIG_POLICY_REQUIRED) r.sig = FW_SIG_POLICY_REQUIRED;
        if (r.lan > POD_LAN_OFF) r.lan = POD_LAN_LOCKED;
        if (r.usb_net_off > 1) r.usb_net_off = 1;
        if (r.lan_plain > 1) r.lan_plain = 0;   /* unknown: the stricter meaning */
        if (r.scpi_on > 1) r.scpi_on = 0;
        s_rec = r;
    }
    /* Firmware 4.0.0: signatures are always required, whatever a record from older firmware says
       (audit was its default). The record gets "required" with the next save, so firmware rolled
       back to 3.10.x keeps it too. */
    s_rec.sig = FW_SIG_POLICY_REQUIRED;
    s_rec.lan_unsigned = 0;
    fw_sign_set_policy(FW_SIG_POLICY_REQUIRED);
    fw_sign_set_floor(s_rec.rollback_floor);
    log_set_level(log_level_from_stored(s_rec.log_level));
    char fl[16];
    log_printf("[policy] signatures %s, LAN %s, pairing %s, USB network %s, HTTPS-only %s, SCPI %s, floor %s, log %s\n",
           fw_sign_policy_name((fw_sig_policy_t)s_rec.sig), pod_policy_lan_name((pod_lan_policy_t)s_rec.lan),
           s_lan_pairing_off ? "off" : "on", s_rec.usb_net_off ? "off" : "on", s_rec.lan_plain ? "off" : "on", s_rec.scpi_on ? "on" : "off",
           s_rec.rollback_floor ? fw_sign_release_str(s_rec.rollback_floor, fl, sizeof(fl)) : "none",
           log_level_name(log_level()));
}


fw_sig_policy_t  pod_policy_sig(void) { return (fw_sig_policy_t)s_rec.sig; }
pod_lan_policy_t pod_policy_lan(void) { return (pod_lan_policy_t)s_rec.lan; }

/* Both records: version 2 (with the pin), then the version 1 copy for older firmware. */
static const char *save_all(const pod_policy_rec_t *r, bool pinned, const uint8_t mac[6], const uint8_t pub[32]) {
    pod_policy_rec2_t r2;
    memset(&r2, 0, sizeof(r2));
    r2.v1 = *r;
    /* Older firmware reads HTTPS-only from its own byte: keep it in step with lan_plain. */
    r2.v1.https_only = r2.v1.lan_plain ? 0 : 1;
    r2.lan_pairing_off = s_lan_pairing_off;
    if (pinned) {
        r2.bridge_pinned = 1;
        memcpy(r2.bridge_mac, mac, 6);
        memcpy(r2.bridge_pub, pub, 32);
    }
    if (ab_store_save(&k_store2, &r2, sizeof(r2)) != 0 || ab_store_save(&k_store, &r2.v1, sizeof(r2.v1)) != 0)
        return "could not save the policy";
    s_rec = r2.v1;
    return NULL;
}

static const char *save(const pod_policy_rec_t *r) {
    pod_policy_rec2_t r2;
    bool pinned = load_pin(&r2);
    return save_all(r, pinned, r2.bridge_mac, r2.bridge_pub);
}

bool pod_policy_bridge_pin(uint8_t mac[6], uint8_t pub[32]) {
    pod_policy_rec2_t r2;
    if (!load_pin(&r2)) return false;
    memcpy(mac, r2.bridge_mac, 6);
    memcpy(pub, r2.bridge_pub, 32);
    return true;
}

const char *pod_policy_set_bridge_pin(const uint8_t *mac, const uint8_t *pub) {
    static const uint8_t zero[32];
    pod_policy_rec2_t r2;
    bool pinned = load_pin(&r2);
    if (!mac && !pinned) return NULL;
    if (mac && pinned && !memcmp(mac, r2.bridge_mac, 6) && !memcmp(pub, r2.bridge_pub, 32)) return NULL;
    const char *why = save_all(&s_rec, mac != NULL, mac ? mac : zero, mac ? pub : zero);
    if (!why) log_printf("[policy] Wi-Fi bridge %s\n", mac ? "stored" : "cleared");
    return why;
}

const char *pod_policy_set_sig(fw_sig_policy_t p, policy_src_t src) {
    /* Firmware 4.0.0: "required" only, from every source (a server pushing it keeps working). There
       is no way to loosen it: self-built firmware goes in over USB DFU. */
    (void)src;
    if ((unsigned)p > FW_SIG_POLICY_REQUIRED) return "sig_policy: unknown policy";
    if (p == FW_SIG_POLICY_REQUIRED) return NULL;
    return bp_note_signatures_always_required(s_why, sizeof(s_why), "sig_policy", fw_sign_policy_name(p));
}

const char *pod_policy_set_lan(pod_lan_policy_t p, policy_src_t src) {
    if ((unsigned)p > POD_LAN_OFF) return "lan_policy: unknown policy";
    if (src == POLICY_SRC_LAN) return bp_note_policy_not_from_lan(s_why, sizeof(s_why), "lan_policy");
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

bool pod_policy_lan_pairing(void) { return s_lan_pairing_off == 0; }

const char *pod_policy_set_lan_pairing(bool on, policy_src_t src, bool usb_link) {
    /* Who may drive the pod from the LAN is decided by its USB cable or its cloud account, never
       from the LAN itself: the USB link counts as the cable. */
    if (src == POLICY_SRC_LAN && !usb_link)
        return bp_note_policy_not_from_lan(s_why, sizeof(s_why), "lan_pairing");
    if (on == pod_policy_lan_pairing()) return NULL;
    uint8_t was = s_lan_pairing_off;
    s_lan_pairing_off = on ? 0 : 1;
    const char *why = save(&s_rec);
    if (why) { s_lan_pairing_off = was; return why; }
    log_printf("[policy] LAN pairing now %s (from %s)\n", on ? "on" : "off",
               src == POLICY_SRC_USB ? "USB" : src == POLICY_SRC_CLOUD ? "cloud" : "the USB link");
    return NULL;
}

bool pod_policy_usb_net(void) { return s_rec.usb_net_off == 0; }

const char *pod_policy_set_usb_net(bool on, policy_src_t src, bool usb_link) {
    if (src == POLICY_SRC_CLOUD || (src == POLICY_SRC_LAN && !usb_link))
        return bp_note_setting_device_only(s_why, sizeof(s_why), "usb_network");
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

bool pod_policy_https_only(void) { return s_rec.lan_plain == 0; }
bool pod_policy_scpi(void) { return s_rec.scpi_on != 0; }

const char *pod_policy_lan_tls_gate(policy_src_t src, bool usb_link) {
    /* Not "or over TLS": port 8443 has no client authentication, so a TLS connection proves
       nothing about who is on the other end (security review 2026-11, M10). */
    if (src != POLICY_SRC_LAN || usb_link) return NULL;
    return bp_note_setting_cloud_or_usb(s_why, sizeof(s_why), "lan_tls");
}

const char *pod_policy_set_https_only(bool on, policy_src_t src, bool usb_link) {
    /* Both ways: on cuts off every plaintext client, off is what a man-in-the-middle wants. */
    const char *why = pod_policy_lan_tls_gate(src, usb_link);
    if (why) return why;
    if (on == pod_policy_https_only()) return NULL;
    pod_policy_rec_t r = s_rec;
    r.lan_plain = on ? 0 : 1;
    why = save(&r);
    if (why) return why;
    log_printf("[policy] HTTPS-only now %s (from %s)\n", on ? "on" : "off",
               src == POLICY_SRC_USB ? "USB" : src == POLICY_SRC_CLOUD ? "cloud" : "the USB link");
    pod_policy_on_https_only_change(on);
    return NULL;
}

const char *pod_policy_set_scpi(bool on, policy_src_t src, bool usb_link) {
    /* The same gate as HTTPS-only: SCPI is plaintext with no client authentication, so turning
       it on is for the device or the cloud, never the external LAN. */
    const char *why = pod_policy_lan_tls_gate(src, usb_link);
    if (why) return why;
    if (on == pod_policy_scpi()) return NULL;
    pod_policy_rec_t r = s_rec;
    r.scpi_on = on ? 1 : 0;
    why = save(&r);
    if (why) return why;
    log_printf("[policy] SCPI now %s (from %s)\n", on ? "on" : "off",
               src == POLICY_SRC_USB ? "USB" : src == POLICY_SRC_CLOUD ? "cloud" : "the USB link");
    pod_policy_on_scpi_change(on);
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

const char *pod_policy_clear_floor(policy_src_t src, bool usb_link) {
    if (src != POLICY_SRC_USB && !usb_link)   /* physical presence: firmware 4.0.0 takes the USB link */
        return bp_note_setting_device_only(s_why, sizeof(s_why), "rollback_floor");
    if (s_rec.rollback_floor == 0) return NULL;
    pod_policy_rec_t r = s_rec;
    r.rollback_floor = 0;
    const char *why = save(&r);
    if (why) return why;
    fw_sign_set_floor(0);
    log_printf("[policy] rollback floor cleared (over USB)\n");
    return NULL;
}

const char *pod_policy_cloud_link_gate(const char *verb, policy_src_t src, bool usb_link) {
    if (src != POLICY_SRC_LAN || usb_link) return NULL;
    return bp_note_setting_cloud_or_usb(s_why, sizeof(s_why), verb);
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

const char *pod_policy_set_log_level(bp_log_level_t lv) {
    if ((unsigned)lv > BP_LOG_DEBUG) return "log_level: set must be error, warn, info or debug";
    uint8_t v = log_level_to_stored(lv);
    if (v == s_rec.log_level || (s_rec.log_level == 0 && lv == BP_LOG_DEFAULT)) {
        log_set_level(lv);
        return NULL;
    }
    pod_policy_rec_t r = s_rec;
    r.log_level = v;
    const char *why = save(&r);
    if (why) return why;
    log_set_level(lv);
    return NULL;
}
