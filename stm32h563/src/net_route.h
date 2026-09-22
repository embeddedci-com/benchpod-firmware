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

#endif /* NET_ROUTE_H */
