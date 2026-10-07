/*
 * bus_owner.c — owner token and depth for the shared quad bus (see bus_owner.h).
 */
#include "bus_owner.h"

int bus_owner_take(bus_owner_t *b, const void *who)
{
    if (b->depth == 0) {
        b->owner = who;
        b->depth = 1;
        return 1;
    }
    if (b->owner != who) b->violations++;   /* two tasks on one bus: nothing serializes them */
    if (b->depth < UINT16_MAX) b->depth++;
    return 0;
}

int bus_owner_give(bus_owner_t *b, const void *who)
{
    if (b->depth == 0) {
        b->unpaired++;
        return 1;
    }
    if (b->owner != who) b->violations++;
    if (--b->depth == 0) {
        b->owner = 0;
        return 1;
    }
    return 0;
}

int bus_owner_handover(bus_owner_t *b)
{
    int dropped = b->depth;
    if (dropped) b->violations++;
    b->depth = 0;
    b->owner = 0;
    return dropped;
}
