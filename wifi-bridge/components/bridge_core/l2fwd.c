// Layer-2 forwarding rules. See l2fwd.h and docs/PROTOCOL.md ("Forwarding").
#include "l2fwd.h"

#include <string.h>

#include "bwp.h"

static const uint8_t ZERO[6];

static uint16_t ethertype(const uint8_t *f)
{
    return (uint16_t)((f[12] << 8) | f[13]);
}

static l2_action_t drop(l2fwd_t *s, l2_why_t why)
{
    s->last_why = why;
    return L2_DROP;
}

void l2fwd_init(l2fwd_t *s, const uint8_t bridge_mac[6])
{
    memset(s, 0, sizeof(*s));
    memcpy(s->bridge_mac, bridge_mac, 6);
}

void l2fwd_link_down(l2fwd_t *s)
{
    s->relearn = true;
}

l2_action_t l2fwd_from_eth(l2fwd_t *s, const uint8_t *f, size_t len, bool wifi_up, l2_event_t *ev)
{
    const uint8_t *dst = f, *src = f + 6;
    if (ev) {
        *ev = L2_EV_NONE;
    }
    s->last_why = L2_WHY_NONE;
    if (len < 14) {
        return drop(s, L2_WHY_RUNT);
    }
    if (l2_is_multicast(src) || memcmp(src, ZERO, 6) == 0) {
        return drop(s, L2_WHY_BAD_SRC);
    }
    if (memcmp(src, s->bridge_mac, 6) == 0) {
        return drop(s, L2_WHY_OWN_SRC);
    }

    if (ethertype(f) == BWP_ETHERTYPE) {
        // Control frames are never forwarded. Accepted sent to us or broadcast.
        if (memcmp(dst, s->bridge_mac, 6) != 0 && !l2_is_multicast(dst)) {
            return drop(s, L2_WHY_CONTROL);
        }
        // Learn from these only: a pod speaks this protocol, the switch or router the bridge may
        // be plugged into by mistake does not, and its devices must never get onto Wi-Fi.
        if (!s->pod_known || s->relearn) {
            bool changed = s->pod_known && memcmp(s->pod_mac, src, 6) != 0;
            if (ev && (!s->pod_known || changed)) {
                *ev = s->pod_known ? L2_EV_CHANGED : L2_EV_LEARNED;
            }
            memcpy(s->pod_mac, src, 6);
            s->pod_known = true;
            s->relearn = false;
        }
        if (memcmp(src, s->pod_mac, 6) != 0) {
            return drop(s, L2_WHY_OTHER_MAC);
        }
        return L2_CONTROL;
    }
    if (!s->pod_known) {
        return drop(s, L2_WHY_NO_POD);
    }
    if (memcmp(src, s->pod_mac, 6) != 0) {
        return drop(s, L2_WHY_OTHER_MAC);
    }
    if (memcmp(dst, s->bridge_mac, 6) == 0) {
        return drop(s, L2_WHY_TO_BRIDGE);
    }
    if (!wifi_up) {
        return drop(s, L2_WHY_WIFI_DOWN);
    }
    return L2_FORWARD;
}

l2_action_t l2fwd_from_wifi(l2fwd_t *s, const uint8_t *f, size_t len)
{
    const uint8_t *dst = f, *src = f + 6;
    s->last_why = L2_WHY_NONE;
    if (len < 14) {
        return drop(s, L2_WHY_RUNT);
    }
    if (ethertype(f) == BWP_ETHERTYPE) {
        return drop(s, L2_WHY_CONTROL);  // never accept control frames from the radio side
    }
    if (!s->pod_known) {
        return drop(s, L2_WHY_NO_POD);
    }
    if (memcmp(src, s->pod_mac, 6) == 0) {
        return drop(s, L2_WHY_REFLECTED);  // e.g. the AP echoing the pod's broadcast (breaks IPv6 DAD)
    }
    if (l2_is_multicast(dst) || memcmp(dst, s->pod_mac, 6) == 0) {
        return L2_FORWARD;
    }
    return drop(s, L2_WHY_NOT_FOR_POD);
}
