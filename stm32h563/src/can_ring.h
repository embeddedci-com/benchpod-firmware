#ifndef CAN_RING_H
#define CAN_RING_H

/* ============================================================================
 * can_ring — single-producer / single-consumer byte ring of variable-length
 * CAN frame records.  The FDCAN RX interrupt is the only producer, can_rx_pop /
 * can_rx_peek (task side) the only consumer.
 *
 * Why variable length: a fixed slot big enough for a 64-byte CAN FD frame is
 * 80 bytes, so 32 slots would cost 2.5 KB of a RAM that has none to spare.  A
 * record here is a 12-byte header plus the payload rounded up to 4 bytes: a
 * classic 8-byte frame takes 20 bytes, a 64-byte FD frame 76.
 *
 * A record never wraps: when it does not fit before the end of the buffer the
 * producer writes a wrap marker (0xFFFFFFFF, never a valid 29-bit id) and
 * starts again at offset 0.  Pure C, host-tested (test/test_can_timing.c).
 * ========================================================================== */

#include <stdint.h>
#include <stdbool.h>
#include "can_bus.h"

typedef struct {
    uint8_t          *buf;      /* 4-byte aligned, size a multiple of 4      */
    uint32_t          size;
    volatile uint32_t head;     /* byte offset, written by the producer only */
    volatile uint32_t tail;     /* byte offset, written by the consumer only */
    volatile uint32_t pushed;   /* frames ever pushed (producer)             */
    volatile uint32_t popped;   /* frames ever popped (consumer)             */
} can_ring_t;

void can_ring_init(can_ring_t *r, uint8_t *buf, uint32_t size);

/* Consumer-side reset (drop everything).  Call with the producer stopped. */
void can_ring_reset(can_ring_t *r);

/* Producer: append one frame (dlc bytes of data).  Returns false if full. */
bool can_ring_push(can_ring_t *r, const can_frame_t *f);

/* Consumer: copy the oldest frame without removing it.  Returns false if empty. */
bool can_ring_peek(const can_ring_t *r, can_frame_t *out);

/* Consumer: remove the oldest frame (copied to out when out != NULL).
   Returns false if empty. */
bool can_ring_pop(can_ring_t *r, can_frame_t *out);

/* Frames waiting. */
uint32_t can_ring_count(const can_ring_t *r);

#endif /* CAN_RING_H */
