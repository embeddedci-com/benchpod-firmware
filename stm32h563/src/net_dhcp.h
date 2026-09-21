#ifndef NET_DHCP_H
#define NET_DHCP_H

#include <stdint.h>
#include <stdbool.h>

/* ---- Pure DHCP acquire/retry state machine (host-testable) ------------------
 *
 * net_server.c drives one of these per interface (eth + Wi-Fi) on a ~500 ms
 * tick. The decision logic is kept free of lwIP calls so it can be unit-tested
 * (test/test_net_dhcp.c); the caller performs the returned side-effect action.
 *
 * Design notes / fixes over the previous inline version:
 *  - Link state is POLLED every tick (link_up input), not latched from a netif
 *    link callback. A missed link-up edge can no longer strand the machine at
 *    0.0.0.0 — if the link is up and we are idle, we (re)start acquisition.
 *  - Re-acquire / re-kick are explicit actions so the caller can choose a clean
 *    restart, and the whole thing is deterministic + testable.
 * ---------------------------------------------------------------------------*/

typedef enum {
    NET_DHCP_OFF = 0,   /* link down / idle                         */
    NET_DHCP_WAIT,      /* DISCOVER sent, awaiting a lease          */
    NET_DHCP_DONE,      /* bound (lwIP owns renew/rebind)           */
} net_dhcp_state_t;

typedef enum {
    NET_DHCP_DO_NOTHING = 0,
    NET_DHCP_DO_DISCOVER,   /* clear addr + dhcp_start() (fresh acquire)     */
    NET_DHCP_DO_REKICK,     /* dhcp_start() again — stalled in WAIT too long */
    NET_DHCP_DO_BOUND,      /* got a lease: latch IP + announce mDNS         */
    NET_DHCP_DO_IPCHANGE,   /* bound IP changed under us: re-latch + announce*/
    NET_DHCP_DO_LINKDOWN,   /* link dropped: clear the cached IP string      */
} net_dhcp_action_t;

typedef struct {
    uint8_t  state;        /* net_dhcp_state_t */
    uint16_t wait_ticks;   /* ticks elapsed in WAIT since the last DISCOVER  */
} net_dhcp_t;

/* Advance one tick. Inputs are sampled by the caller from lwIP:
 *   link_up    = netif_is_link_up()
 *   has_lease  = dhcp_supplied_address()
 *   ip_changed = bound and the netif IPv4 differs from the cached string
 * `rekick_ticks` = ticks to sit in WAIT with no lease before forcing a fresh
 * DISCOVER (0 disables the re-kick and leaves retransmission entirely to lwIP).
 * Returns the side-effect the caller must perform; mutates *d in place. */
net_dhcp_action_t net_dhcp_step(net_dhcp_t *d, bool link_up, bool has_lease,
                                bool ip_changed, uint16_t rekick_ticks);

#endif /* NET_DHCP_H */
