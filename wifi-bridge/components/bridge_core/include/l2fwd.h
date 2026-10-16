// Layer-2 forwarding rules for the bridge: learn the pod's MAC, decide per frame.
//
// The bridge forwards between its Ethernet port and the Wi-Fi station for one MAC only, the
// pod's. The Wi-Fi station runs with that MAC, so the AP and the router see the pod itself.
// Plain C, no ESP-IDF dependencies (host-tested in test/). The caller serialises access.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    L2_DROP = 0,
    L2_FORWARD,  // to the other port
    L2_CONTROL,  // a control-channel frame for the bridge (Ethernet side only)
} l2_action_t;

typedef enum {
    L2_EV_NONE = 0,
    L2_EV_LEARNED,  // first pod MAC since boot
    L2_EV_CHANGED,  // a different device after an Ethernet link-down: restart Wi-Fi with its MAC
} l2_event_t;

typedef enum {
    L2_WHY_NONE = 0,
    L2_WHY_RUNT,
    L2_WHY_BAD_SRC,       // multicast or all-zero source
    L2_WHY_OWN_SRC,       // source is the bridge itself
    L2_WHY_OTHER_MAC,     // not the pod
    L2_WHY_NOT_FOR_POD,   // Wi-Fi frame for some other unicast address
    L2_WHY_CONTROL,       // control EtherType where it is not accepted
    L2_WHY_TO_BRIDGE,     // non-control frame addressed to the bridge
    L2_WHY_NO_POD,        // pod MAC not learned yet
    L2_WHY_WIFI_DOWN,     // Wi-Fi not connected
    L2_WHY_REFLECTED,     // our own broadcast echoed back by the AP
} l2_why_t;

typedef struct {
    uint8_t bridge_mac[6];  // the bridge's own Ethernet MAC (control channel address)
    uint8_t pod_mac[6];
    bool pod_known;
    bool relearn;           // set on Ethernet link-down: the next valid source becomes the pod
    l2_why_t last_why;      // reason for the last L2_DROP, for counters and tests
} l2fwd_t;

void l2fwd_init(l2fwd_t *s, const uint8_t bridge_mac[6]);
// The cable was unplugged (or the pod rebooted): allow a different device to take over.
void l2fwd_link_down(l2fwd_t *s);
// Ethernet -> Wi-Fi. ev reports a learned or changed pod MAC (may be NULL).
l2_action_t l2fwd_from_eth(l2fwd_t *s, const uint8_t *frame, size_t len, bool wifi_up, l2_event_t *ev);
// Wi-Fi -> Ethernet.
l2_action_t l2fwd_from_wifi(l2fwd_t *s, const uint8_t *frame, size_t len);

static inline bool l2_is_multicast(const uint8_t *mac)
{
    return mac[0] & 0x01;
}

#ifdef __cplusplus
}
#endif
