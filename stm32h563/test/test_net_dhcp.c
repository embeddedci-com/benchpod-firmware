/*
 * test_net_dhcp.c — host unit tests for the pure DHCP acquire/retry state
 * machine (src/net_dhcp.c). No lwIP / HAL. Run via test/Makefile (`make`).
 *
 * These lock in the retry/renew behavior that was previously untested and flaky:
 * fresh acquire, bind, the coarse re-kick cadence, lease-loss re-acquire, IP
 * change on renew, link down/up, and the OFF self-heal (missed link-up edge).
 */
#include "net_dhcp.h"

#include <stdint.h>
#include <stdio.h>

static int failures;
#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); failures++; } \
} while (0)

/* Convenience: one step with explicit inputs. */
static net_dhcp_action_t step(net_dhcp_t *d, bool link, bool lease, bool ipchg, uint16_t rk)
{
    return net_dhcp_step(d, link, lease, ipchg, rk);
}

#define REKICK 40   /* matches DHCP_REKICK_TICKS in net_server.c */

/* Fresh boot with the link already up → immediate DISCOVER, then WAIT. */
static void test_fresh_acquire(void)
{
    net_dhcp_t d = { NET_DHCP_OFF, 0 };
    CHECK(step(&d, true, false, false, REKICK) == NET_DHCP_DO_DISCOVER);
    CHECK(d.state == NET_DHCP_WAIT);
    CHECK(d.wait_ticks == 0);
}

/* WAIT → lease arrives → BOUND/DONE. */
static void test_bind(void)
{
    net_dhcp_t d = { NET_DHCP_WAIT, 5 };
    CHECK(step(&d, true, true, false, REKICK) == NET_DHCP_DO_BOUND);
    CHECK(d.state == NET_DHCP_DONE);
    /* Steady state: bound, no change → nothing. */
    CHECK(step(&d, true, true, false, REKICK) == NET_DHCP_DO_NOTHING);
    CHECK(d.state == NET_DHCP_DONE);
}

/* WAIT with no lease: nothing until exactly REKICK ticks, then one re-kick,
   and the counter resets so re-kicks recur on a fixed cadence (not every tick). */
static void test_rekick_cadence(void)
{
    net_dhcp_t d = { NET_DHCP_WAIT, 0 };
    for (int t = 1; t < REKICK; t++)
        CHECK(step(&d, true, false, false, REKICK) == NET_DHCP_DO_NOTHING);
    CHECK(d.wait_ticks == REKICK - 1);
    CHECK(step(&d, true, false, false, REKICK) == NET_DHCP_DO_REKICK);  /* the REKICK-th tick */
    CHECK(d.wait_ticks == 0);
    CHECK(d.state == NET_DHCP_WAIT);
    /* Next window: no re-kick until REKICK more ticks. */
    for (int t = 1; t < REKICK; t++)
        CHECK(step(&d, true, false, false, REKICK) == NET_DHCP_DO_NOTHING);
    CHECK(step(&d, true, false, false, REKICK) == NET_DHCP_DO_REKICK);
}

/* rekick_ticks == 0 disables the coarse re-kick (retransmission left to lwIP). */
static void test_rekick_disabled(void)
{
    net_dhcp_t d = { NET_DHCP_WAIT, 0 };
    for (int t = 0; t < 1000; t++)
        CHECK(step(&d, true, false, false, 0) == NET_DHCP_DO_NOTHING);
    CHECK(d.state == NET_DHCP_WAIT);
}

/* Bound, router hands out a different address on renew → IPCHANGE (stay DONE). */
static void test_ip_change_on_renew(void)
{
    net_dhcp_t d = { NET_DHCP_DONE, 0 };
    CHECK(step(&d, true, true, true, REKICK) == NET_DHCP_DO_IPCHANGE);
    CHECK(d.state == NET_DHCP_DONE);
}

/* Bound, lease lost (NAK/expiry) with link still up → re-acquire (DISCOVER). */
static void test_lease_lost_reacquire(void)
{
    net_dhcp_t d = { NET_DHCP_DONE, 0 };
    CHECK(step(&d, true, false, false, REKICK) == NET_DHCP_DO_DISCOVER);
    CHECK(d.state == NET_DHCP_WAIT);
    CHECK(d.wait_ticks == 0);
}

/* Link down from any active state → LINKDOWN once, then OFF/quiet. */
static void test_link_down(void)
{
    net_dhcp_t d = { NET_DHCP_DONE, 0 };
    CHECK(step(&d, false, false, false, REKICK) == NET_DHCP_DO_LINKDOWN);
    CHECK(d.state == NET_DHCP_OFF);
    /* Already off + still down → nothing (no repeated LINKDOWN spam). */
    CHECK(step(&d, false, false, false, REKICK) == NET_DHCP_DO_NOTHING);
    CHECK(d.state == NET_DHCP_OFF);

    /* Down while merely WAITing also parks at OFF. */
    net_dhcp_t w = { NET_DHCP_WAIT, 7 };
    CHECK(step(&w, false, false, false, REKICK) == NET_DHCP_DO_LINKDOWN);
    CHECK(w.state == NET_DHCP_OFF);
    CHECK(w.wait_ticks == 0);
}

/* Self-heal: link is up but we somehow sit at OFF (a link-up edge was missed) —
   the machine must (re)start acquisition rather than strand at 0.0.0.0. */
static void test_self_heal_off(void)
{
    net_dhcp_t d = { NET_DHCP_OFF, 0 };
    CHECK(step(&d, true, false, false, REKICK) == NET_DHCP_DO_DISCOVER);
    CHECK(d.state == NET_DHCP_WAIT);
}

/* End-to-end: boot → discover → wait a bit → bound → link drops → link back →
   fresh discover → bound. */
static void test_full_lifecycle(void)
{
    net_dhcp_t d = { NET_DHCP_OFF, 0 };
    CHECK(step(&d, true, false, false, REKICK) == NET_DHCP_DO_DISCOVER); /* boot */
    CHECK(step(&d, true, false, false, REKICK) == NET_DHCP_DO_NOTHING);  /* waiting */
    CHECK(step(&d, true, false, false, REKICK) == NET_DHCP_DO_NOTHING);
    CHECK(step(&d, true, true,  false, REKICK) == NET_DHCP_DO_BOUND);    /* got lease */
    CHECK(step(&d, true, true,  false, REKICK) == NET_DHCP_DO_NOTHING);  /* steady */
    CHECK(step(&d, false,false, false, REKICK) == NET_DHCP_DO_LINKDOWN); /* link down */
    CHECK(d.state == NET_DHCP_OFF);
    CHECK(step(&d, true, false, false, REKICK) == NET_DHCP_DO_DISCOVER); /* link back */
    CHECK(step(&d, true, true,  false, REKICK) == NET_DHCP_DO_BOUND);    /* rebind */
    CHECK(d.state == NET_DHCP_DONE);
}

/* Slow/awkward router: no lease for a long time (multiple re-kick windows), then
   it finally answers → bind. Mirrors "router finished rebooting". */
static void test_slow_router_then_bind(void)
{
    net_dhcp_t d = { NET_DHCP_OFF, 0 };
    CHECK(step(&d, true, false, false, REKICK) == NET_DHCP_DO_DISCOVER);
    int rekicks = 0;
    for (int t = 0; t < REKICK * 3; t++)
        if (step(&d, true, false, false, REKICK) == NET_DHCP_DO_REKICK) rekicks++;
    CHECK(rekicks == 3);                 /* one re-kick per REKICK-tick window */
    CHECK(step(&d, true, true, false, REKICK) == NET_DHCP_DO_BOUND);
    CHECK(d.state == NET_DHCP_DONE);
}

int main(void)
{
    test_fresh_acquire();
    test_bind();
    test_rekick_cadence();
    test_rekick_disabled();
    test_ip_change_on_renew();
    test_lease_lost_reacquire();
    test_link_down();
    test_self_heal_off();
    test_full_lifecycle();
    test_slow_router_then_bind();
    if (failures == 0) { printf("PASS — all net_dhcp tests\n"); return 0; }
    printf("FAILED — %d check(s)\n", failures);
    return 1;
}
