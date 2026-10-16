#ifndef POD_POLICY_H
#define POD_POLICY_H

/*
 * pod_policy — the pod's persisted security policies (docs/design/policy-commands.md):
 *
 *   signature policy  audit < permissive < required   (fw_sign.h; what OTA accepts)
 *   LAN policy        open | locked | off            (what the LAN TCP API may do)
 *
 * Both live in one power-loss-safe record (config_store.h A/B slots at 0x1E4000/0x1E6000).
 * No record = audit + open, which is what a pod did before this existed.
 *
 * Who may change what is decided here, from where the request came:
 *   USB console  anything (physical presence: the way back)
 *   cloud        the LAN policy freely; the signature policy only to a stricter value
 *   LAN TCP      nothing
 * A refused change returns the reason and changes nothing.
 */

#include <stdbool.h>
#include <stdint.h>
#include "fw_sign.h"

#define POD_POLICY_SLOT_A_OFFSET  0x1E6000u   /* bank2 sector 115 */
#define POD_POLICY_SLOT_B_OFFSET  0x1E4000u   /* bank2 sector 114 */
#define POD_POLICY_RECORD_MAGIC   0x4C4F5050u /* "PPOL" */

typedef enum {
    POD_LAN_OPEN = 0,   /* everything, as before */
    POD_LAN_LOCKED,     /* T0 + T1 only (cmd_tier.h) */
    POD_LAN_OFF,        /* no LAN listener, no mDNS */
} pod_lan_policy_t;

typedef enum {
    POLICY_SRC_USB = 0,
    POLICY_SRC_CLOUD,
    POLICY_SRC_LAN,
} policy_src_t;

/* Read the record (boot) and apply the signature policy to fw_sign. */
void pod_policy_load(void);

fw_sig_policy_t  pod_policy_sig(void);
pod_lan_policy_t pod_policy_lan(void);

/* Change a policy. NULL = done (and persisted), else why not (nothing changed). */
const char *pod_policy_set_sig(fw_sig_policy_t p, policy_src_t src);
const char *pod_policy_set_lan(pod_lan_policy_t p, policy_src_t src);

/* The cloud link's trust settings (company CA install or clear, HTTP proxy set or clear):
   NULL = this source may change them, else the refusal for `verb`. The LAN never may, whatever
   the LAN policy: a LAN attacker who installs their own CA and points the pod at their proxy
   could otherwise terminate its TLS and relay the host-bound login (cloud-hardening.md).
   usb_link: a LAN connection on the USB network adapter (firmware 3.9.0+). That is physical
   presence, like the USB console (anything on the computer can open the console anyway), so it
   may change these two settings; it stays LAN tier for everything else. */
const char *pod_policy_cloud_link_gate(const char *verb, policy_src_t src, bool usb_link);

/* The USB network adapter (firmware 3.9.0+), default on. Off, the pod enumerates as its console
   only: the computer sees no network adapter. Independent of the LAN policy, which covers
   Ethernet and Wi-Fi. Only the device may change it: the USB console, or a connection on the USB
   link itself (usb_link), which can turn it off but not back on (the link is gone then); never
   the cloud or Ethernet/Wi-Fi. */
bool pod_policy_usb_net(void);
const char *pod_policy_set_usb_net(bool on, policy_src_t src, bool usb_link);
/* Called after the USB network setting changed (usb_device re-enumerates). Weak no-op. */
void pod_policy_on_usb_net_change(bool on);

/* HTTPS-only (lan_tls.h, docs/design/lan-tls.md), default off. On, the external LAN (Ethernet,
   Wi-Fi) gets no plaintext :8080 connections, only TLS on :8443; the USB network adapter keeps
   plaintext. Turning it on is allowed from anywhere. Turning it off, like replacing or
   regenerating the TLS certificate (pod_policy_lan_tls_gate), needs the USB console, the USB link,
   the cloud, or a LAN connection over TLS: never a plaintext LAN connection, which an active
   man-in-the-middle could have injected. */
bool pod_policy_https_only(void);
const char *pod_policy_set_https_only(bool on, policy_src_t src, bool usb_link, bool tls_link);
/* NULL = this source may change the LAN TLS identity or turn HTTPS-only off, else the refusal. */
const char *pod_policy_lan_tls_gate(policy_src_t src, bool usb_link, bool tls_link);
/* Called after HTTPS-only changed (net_server closes plaintext LAN connections). Weak no-op. */
void pod_policy_on_https_only_change(bool on);

/* "audit"/"permissive"/"required" -> value, -1 if unknown. */
int pod_policy_sig_from_name(const char *name);
/* "open"/"locked"/"off" -> value, -1 if unknown. */
int pod_policy_lan_from_name(const char *name);
const char *pod_policy_lan_name(pod_lan_policy_t p);

/* Called after the LAN policy changed (net_server starts or stops the listener and mDNS).
   Weak no-op by default, so the host tests link without the network stack. */
void pod_policy_on_lan_change(pod_lan_policy_t now);

#endif /* POD_POLICY_H */
