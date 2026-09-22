/*
 * test_net_route.c — host unit tests for the outbound interface choice
 * (src/net_route.c). No lwIP / HAL. Run via test/Makefile (`make`).
 *
 * Locks in the fix for asymmetric routing: with eth and Wi-Fi on the same /24,
 * replies went out over Wi-Fi even for connections that arrived on Ethernet.
 */
#include "net_route.h"

#include <stdint.h>
#include <stdio.h>

static int failures;
#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); failures++; } \
} while (0)

#define IP(a, b, c, d) (((uint32_t)(a) << 24) | ((uint32_t)(b) << 16) | ((uint32_t)(c) << 8) | (uint32_t)(d))
#define MASK24 IP(255, 255, 255, 0)

enum { ETH = 0, WIFI = 1 };

/* The field case: eth .220 and wifi .221 on one home /24. */
static net_route_if_t both_up[2] = {
    { IP(192, 168, 1, 220), MASK24, true },
    { IP(192, 168, 1, 221), MASK24, true },
};

static void test_reply_leaves_arrival_interface(void)
{
    uint32_t mac = IP(192, 168, 1, 50);
    CHECK(net_route_pick(both_up, 2, IP(192, 168, 1, 220), mac) == ETH);
    CHECK(net_route_pick(both_up, 2, IP(192, 168, 1, 221), mac) == WIFI);
}

static void test_unbound_source_prefers_eth(void)
{
    CHECK(net_route_pick(both_up, 2, 0, IP(192, 168, 1, 50)) == ETH);
}

static void test_off_subnet_defers_to_default_route(void)
{
    CHECK(net_route_pick(both_up, 2, 0, IP(104, 16, 0, 1)) == -1);
    /* ...unless the source pins it (cloud socket bound to the wifi address). */
    CHECK(net_route_pick(both_up, 2, IP(192, 168, 1, 221), IP(104, 16, 0, 1)) == WIFI);
}

static void test_eth_down_falls_back_to_wifi(void)
{
    net_route_if_t ifs[2] = { both_up[0], both_up[1] };
    ifs[ETH].usable = false;   /* cable unplugged */
    CHECK(net_route_pick(ifs, 2, 0, IP(192, 168, 1, 50)) == WIFI);
    /* A reply from the stale eth address must not be pinned to a dead netif. */
    CHECK(net_route_pick(ifs, 2, IP(192, 168, 1, 220), IP(192, 168, 1, 50)) == WIFI);
}

static void test_unaddressed_interface_never_matches(void)
{
    net_route_if_t ifs[2] = {
        { 0, 0, true },                          /* eth link up, no lease yet */
        { IP(192, 168, 1, 221), MASK24, true },
    };
    CHECK(net_route_pick(ifs, 2, 0, IP(192, 168, 1, 50)) == WIFI);
    CHECK(net_route_pick(ifs, 2, 0, IP(10, 0, 0, 1)) == -1);  /* mask 0 must not match all */
}

static void test_different_subnets(void)
{
    net_route_if_t ifs[2] = {
        { IP(10, 0, 0, 5), MASK24, true },
        { IP(192, 168, 1, 221), MASK24, true },
    };
    CHECK(net_route_pick(ifs, 2, 0, IP(192, 168, 1, 50)) == WIFI);
    CHECK(net_route_pick(ifs, 2, 0, IP(10, 0, 0, 9)) == ETH);
}

static void test_nothing_usable(void)
{
    net_route_if_t ifs[2] = { both_up[0], both_up[1] };
    ifs[0].usable = ifs[1].usable = false;
    CHECK(net_route_pick(ifs, 2, IP(192, 168, 1, 220), IP(192, 168, 1, 50)) == -1);
}

int main(void)
{
    test_reply_leaves_arrival_interface();
    test_unbound_source_prefers_eth();
    test_off_subnet_defers_to_default_route();
    test_eth_down_falls_back_to_wifi();
    test_unaddressed_interface_never_matches();
    test_different_subnets();
    test_nothing_usable();
    if (failures) { printf("test_net_route: %d FAILED\n", failures); return 1; }
    printf("test_net_route: all passed\n");
    return 0;
}
