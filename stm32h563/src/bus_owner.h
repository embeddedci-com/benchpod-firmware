/*
 * bus_owner.h — who holds the shared PSRAM/W25Q quad bus, and how deep (see psram.c).
 *
 * The STM32 and the iCE40 share one quad bus (PG0 says who drives it). On the STM32 side many
 * paths take it: OTA staging, a load_bin upload, capture read-back, every W25Q session. They
 * used to nest without knowing it: an inner w25q_open/close pair inside an outer holder released
 * the bus to the iCE40 on its close while the outer holder was still reading. This tracks an
 * owner token and a depth, so only the outermost release hands the bus back, and anything that
 * breaks the rules is counted instead of silently corrupting a transfer.
 *
 * Pure bookkeeping (no HAL), so test/test_bus_owner.c pins it. psram.c switches the pins on
 * what these return.
 */
#ifndef BUS_OWNER_H
#define BUS_OWNER_H

#include <stdint.h>

typedef struct {
    const void *owner;        /* the holder's token (the FreeRTOS task), NULL while free      */
    uint16_t    depth;        /* nested takes by the holder; 0 = free                          */
    uint32_t    violations;   /* takes/gives by a non-holder, holds dropped by a handover      */
    uint32_t    unpaired;     /* gives with nothing held (old "hand it to the iCE40" style)    */
} bus_owner_t;

/* Take the bus for `who`. 1 = it was free: the caller switches the pins to the STM32. 0 = the
   holder nested (or another owner took it: a violation, counted; the pins are already ours). */
int bus_owner_take(bus_owner_t *b, const void *who);

/* Give back one take. 1 = that was the outermost (or nothing was held): the caller hands the pins
   to the iCE40. 0 = still held by an outer take, leave the pins alone. A give by a non-holder
   is counted as a violation but still balances one take. */
int bus_owner_give(bus_owner_t *b, const void *who);

/* The bus must go to the iCE40 now (a capture or replay is about to start, or the iCE40 must
   read its config): drop every hold. Returns the depth that was dropped; anything but 0 means a
   take leaked or an outer holder lost the bus, and counts as a violation. */
int bus_owner_handover(bus_owner_t *b);

#endif /* BUS_OWNER_H */
