/*
 * net_dhcp.c — pure DHCP acquire/retry state machine (no lwIP dependency).
 * See net_dhcp.h. Unit-tested by test/test_net_dhcp.c.
 */
#include "net_dhcp.h"

net_dhcp_action_t net_dhcp_step(net_dhcp_t *d, bool link_up, bool has_lease,
                                bool ip_changed, uint16_t rekick_ticks)
{
    /* Link down overrides everything: drop the address and go idle. Emit the
       LINKDOWN side-effect only on the down edge (state was active). */
    if (!link_up) {
        bool was_active = (d->state != NET_DHCP_OFF);
        d->state = NET_DHCP_OFF;
        d->wait_ticks = 0;
        return was_active ? NET_DHCP_DO_LINKDOWN : NET_DHCP_DO_NOTHING;
    }

    switch ((net_dhcp_state_t)d->state) {
    case NET_DHCP_OFF:
        /* Link is up but we are idle — fresh boot, or a link-up edge was missed
           (self-heal, instead of stranding at 0.0.0.0). Start acquisition. */
        d->state = NET_DHCP_WAIT;
        d->wait_ticks = 0;
        return NET_DHCP_DO_DISCOVER;

    case NET_DHCP_WAIT:
        if (has_lease) {
            d->state = NET_DHCP_DONE;
            d->wait_ticks = 0;
            return NET_DHCP_DO_BOUND;
        }
        /* lwIP retransmits DISCOVER on its own fine timer; the re-kick is only a
           coarse backstop for a router that never answered. 0 disables it. */
        if (rekick_ticks != 0 && ++d->wait_ticks >= rekick_ticks) {
            d->wait_ticks = 0;
            return NET_DHCP_DO_REKICK;
        }
        return NET_DHCP_DO_NOTHING;

    case NET_DHCP_DONE:
        if (!has_lease) {
            /* Lease lost (NAK / expiry / router reboot) — re-acquire cleanly. */
            d->state = NET_DHCP_WAIT;
            d->wait_ticks = 0;
            return NET_DHCP_DO_DISCOVER;
        }
        if (ip_changed) return NET_DHCP_DO_IPCHANGE;
        return NET_DHCP_DO_NOTHING;

    default:
        /* Corrupt state — recover by re-acquiring. */
        d->state = NET_DHCP_OFF;
        d->wait_ticks = 0;
        return NET_DHCP_DO_NOTHING;
    }
}
