/*
 * net_route.c — pure outbound interface choice (no lwIP dependency).
 * See net_route.h. Unit-tested by test/test_net_route.c.
 */
#include "net_route.h"

int net_route_pick(const net_route_if_t *ifs, int n, uint32_t src, uint32_t dest)
{
    if (src != 0) {
        for (int i = 0; i < n; i++)
            if (ifs[i].usable && ifs[i].ip == src) return i;
    }
    for (int i = 0; i < n; i++) {
        if (!ifs[i].usable || ifs[i].ip == 0) continue;
        if ((ifs[i].ip & ifs[i].mask) == (dest & ifs[i].mask)) return i;
    }
    return -1;
}
