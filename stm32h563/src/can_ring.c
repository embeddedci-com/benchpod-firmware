/*
 * can_ring.c — SPSC byte ring of variable-length CAN frame records (see
 * can_ring.h).
 */
#include "can_ring.h"

#include <string.h>

#define WRAP_MARK  0xFFFFFFFFu
#define HDR_LEN    12u

#define F_EXT  0x01u
#define F_RTR  0x02u
#define F_FD   0x04u
#define F_BRS  0x08u

/* Ordering between the payload writes and the index publish.  The ISR and the
   task share one core, so a compiler barrier is enough; the fence is that and
   also correct on the host. */
#define RING_FENCE()  __atomic_thread_fence(__ATOMIC_SEQ_CST)

static uint32_t rec_len(uint32_t dlc) { return HDR_LEN + ((dlc + 3u) & ~3u); }

static uint32_t rd32(const uint8_t *p) { uint32_t v; memcpy(&v, p, 4); return v; }
static void     wr32(uint8_t *p, uint32_t v) { memcpy(p, &v, 4); }

void can_ring_init(can_ring_t *r, uint8_t *buf, uint32_t size)
{
    r->buf  = buf;
    r->size = size & ~3u;
    can_ring_reset(r);
}

void can_ring_reset(can_ring_t *r)
{
    r->head = r->tail = 0;
    r->pushed = r->popped = 0;
}

bool can_ring_push(can_ring_t *r, const can_frame_t *f)
{
    uint32_t dlc = f->dlc > 64u ? 64u : f->dlc;
    uint32_t len = rec_len(dlc);
    uint32_t head = r->head, tail = r->tail;
    uint32_t at;

    if (head >= tail) {
        uint32_t end = r->size - head;
        if (len < end || (len == end && tail != 0u)) {
            at = head;
        } else if (len < tail) {
            at = 0;                        /* wrap: marker at head, record at 0 */
        } else {
            return false;
        }
    } else {
        if (len >= tail - head) return false;
        at = head;
    }

    uint8_t *p = &r->buf[at];
    wr32(p, f->id);
    wr32(p + 4, f->ts);
    p[8]  = (uint8_t)((f->ext ? F_EXT : 0u) | (f->rtr ? F_RTR : 0u) |
                      (f->fd ? F_FD : 0u) | (f->brs ? F_BRS : 0u));
    p[9]  = (uint8_t)dlc;
    p[10] = (uint8_t)len;
    p[11] = 0;
    memcpy(p + HDR_LEN, f->data, dlc);
    if (at != head) wr32(&r->buf[head], WRAP_MARK);
    RING_FENCE();
    uint32_t next = at + len;
    r->head = (next == r->size) ? 0u : next;
    r->pushed = r->pushed + 1u;
    return true;
}

/* Offset of the oldest record, following a wrap marker.  head != tail. */
static uint32_t first_rec(const can_ring_t *r, uint32_t tail)
{
    if (rd32(&r->buf[tail]) == WRAP_MARK) return 0;
    return tail;
}

bool can_ring_peek(const can_ring_t *r, can_frame_t *out)
{
    uint32_t tail = r->tail;
    if (tail == r->head) return false;
    RING_FENCE();
    const uint8_t *p = &r->buf[first_rec(r, tail)];
    memset(out, 0, sizeof(*out));
    out->id  = rd32(p);
    out->ts  = rd32(p + 4);
    out->ext = (p[8] & F_EXT) != 0u;
    out->rtr = (p[8] & F_RTR) != 0u;
    out->fd  = (p[8] & F_FD) != 0u;
    out->brs = (p[8] & F_BRS) != 0u;
    out->dlc = p[9];
    memcpy(out->data, p + HDR_LEN, out->dlc);
    return true;
}

bool can_ring_pop(can_ring_t *r, can_frame_t *out)
{
    uint32_t tail = r->tail;
    if (tail == r->head) return false;
    RING_FENCE();
    if (out) (void)can_ring_peek(r, out);
    uint32_t at = first_rec(r, tail);
    uint32_t next = at + r->buf[at + 10u];
    RING_FENCE();
    r->tail = (next == r->size) ? 0u : next;
    r->popped = r->popped + 1u;
    return true;
}

uint32_t can_ring_count(const can_ring_t *r)
{
    return r->pushed - r->popped;
}
