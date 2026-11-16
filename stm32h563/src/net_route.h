#ifndef NET_ROUTE_H
#define NET_ROUTE_H

#include <stdint.h>
#include <stdbool.h>

/* ---- Interface choice for outbound IPv4 (host-testable) ---------------------
 *
 * lwIP's ip4_route() returns the FIRST netif whose subnet holds the destination,
 * and netif_add() prepends, so with eth and Wi-Fi on the same LAN subnet every
 * reply left over Wi-Fi, even for TCP connections that arrived on Ethernet.
 * net_server.c installs this as LWIP_HOOK_IP4_ROUTE_SRC:
 *
 *   1. src is the address of a usable interface -> that interface (a reply
 *      leaves the interface it arrived on).
 *   2. otherwise the first usable interface, in priority order (eth first),
 *      whose subnet holds dest.
 *   3. otherwise -1: lwIP falls back to its default route (eth priority too,
 *      see update_default_route()).
 *
 * Addresses are compared as-is (network byte order in lwIP); 0 = any/unset.
 * `usable` = netif up + link up + non-zero address.
 * ---------------------------------------------------------------------------*/

typedef struct {
    uint32_t ip;
    uint32_t mask;
    bool     usable;
} net_route_if_t;

/* Returns the index into ifs[] (ordered by priority), or -1 for "no opinion". */
int net_route_pick(const net_route_if_t *ifs, int n, uint32_t src, uint32_t dest);

/* The interface that owns local address `local` (a connection's local IP), as an index into
   ifs[], or -1 when none does (0, or an address no interface holds any more). Ignores `usable`:
   a connection keeps the address it was opened from even while its link flaps. The cloud client
   reports it as the "uplink" of its capabilities frame (eth or wifi). */
int net_route_owner(const net_route_if_t *ifs, int n, uint32_t local);

/* Strong-host rule for the USB network adapter's address (installed as LWIP_HOOK_IP4_INPUT).
   lwIP accepts a packet for ANY of its addresses on ANY interface (weak host), so a LAN host that
   routes the USB link's /30 through the pod's Ethernet or Wi-Fi address would otherwise reach the
   API "on the USB link", which counts as physical presence (pod_policy.h). True = drop the packet:
   it is addressed to the USB subnet (`usb_ip`/`usb_mask`, its broadcast included) but did not
   arrive on the USB interface. usb_ip 0 (no address yet) never drops. */
bool net_route_usb_ingress_drop(uint32_t dest, bool arrived_on_usb, uint32_t usb_ip, uint32_t usb_mask);

#endif /* NET_ROUTE_H */
