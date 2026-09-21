#include "conn_tx.h"
#include "command_handler.h"   /* tunnel conn id range */
#include "bp_limits.h"

#include <string.h>

/* Ring capacity per connection.  bulk_pump paces against conn_tx_free(), so this
   only needs to hold a comfortable burst between net-task drains, not a whole
   capture. Power of two so the free-running 16-bit indices mask cleanly (65536 is
   a whole multiple of the size, so wrap is seamless). */
#define CONN_TX_RING_SIZE 2048u
#define CONN_TX_MASK      (CONN_TX_RING_SIZE - 1u)

/* Ring slots: the real TCP conns (0 .. CONN_TX_TCP_SLOTS-1) followed by the cloud
   tunnels.  CONN_TX_TCP_SLOTS mirrors net_server's NET_MAX_CONN / command_handler's
   CH_MAX_CONN (both 5). */
#define CONN_TX_TCP_SLOTS 5
#define CONN_TX_SLOTS     (CONN_TX_TCP_SLOTS + CH_CLOUD_TUNNEL_CONN_COUNT)

/* head advanced only by the producer (worker), tail only by the consumer (net);
   both free-running 16-bit counters.  used = (uint16_t)(head - tail). */
typedef struct {
    volatile uint16_t head;
    volatile uint16_t tail;
    uint8_t  buf[CONN_TX_RING_SIZE];
} ring_t;

static ring_t s_rings[CONN_TX_SLOTS];

int conn_tx_slot(int conn_id) {
    if (conn_id >= 0 && conn_id < CONN_TX_TCP_SLOTS) return conn_id;
    if (conn_id >= CH_CLOUD_TUNNEL_CONN && conn_id <= CH_CLOUD_TUNNEL_CONN_LAST)
        return CONN_TX_TCP_SLOTS + (conn_id - CH_CLOUD_TUNNEL_CONN);
    return -1;
}

static ring_t *ring_of(int conn_id) {
    int s = conn_tx_slot(conn_id);
    return (s < 0) ? NULL : &s_rings[s];
}

size_t conn_tx_used(int conn_id) {
    ring_t *r = ring_of(conn_id);
    if (!r) return 0;
    return (uint16_t)(r->head - r->tail);
}

size_t conn_tx_free(int conn_id) {
    ring_t *r = ring_of(conn_id);
    if (!r) return 0;
    return CONN_TX_MASK - (uint16_t)(r->head - r->tail);   /* keep 1 slot spare */
}

size_t conn_tx_write(int conn_id, const uint8_t *buf, size_t len) {
    ring_t *r = ring_of(conn_id);
    if (!r) return 0;
    uint16_t head = r->head;
    size_t free = CONN_TX_MASK - (uint16_t)(head - r->tail);
    if (len > free) len = free;
    for (size_t i = 0; i < len; i++)
        r->buf[(head + i) & CONN_TX_MASK] = buf[i];
    r->head = (uint16_t)(head + len);
    return len;
}

size_t conn_tx_peek(int conn_id, const uint8_t **out) {
    ring_t *r = ring_of(conn_id);
    if (!r) return 0;
    uint16_t used = (uint16_t)(r->head - r->tail);
    if (used == 0) return 0;
    uint16_t tpos    = (uint16_t)(r->tail & CONN_TX_MASK);
    uint16_t to_end  = (uint16_t)(CONN_TX_RING_SIZE - tpos);
    uint16_t contig  = used < to_end ? used : to_end;
    *out = &r->buf[tpos];
    return contig;
}

void conn_tx_advance(int conn_id, size_t n) {
    ring_t *r = ring_of(conn_id);
    if (r) r->tail = (uint16_t)(r->tail + n);
}

void conn_tx_reset(int conn_id) {
    ring_t *r = ring_of(conn_id);
    if (r) r->tail = r->head;
}
