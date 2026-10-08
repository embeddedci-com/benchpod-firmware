/*
 * test_cloud_rx.c — host unit tests for the cloud client's inbound byte
 * accumulator sizing (bp_limits.h) and the append/consume/refuse operations
 * cloud_client.c runs on it (src/cloud_rx.c, linked here, not copied).
 *
 * Regression guard for the deep-DAC-replay "backoff: rx overflow" bug: a burst of
 * chunked server->device tunnel.data frames arrives faster than the net task drains
 * s_rx.  With the old single-frame-sized buffer (BP_CLOUD_RX_MAX = 2048) two frames
 * overflowed it and reset the whole cloud link; the fix decoupled the accumulator
 * (BP_CLOUD_RX_ACCUM) so it absorbs a multi-frame burst.  These tests pin both the
 * sizing floor and the ring behavior so a future shrink is caught here.
 */
#include "bp_limits.h"
#include "cloud_rx.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

static int failures;
#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); failures++; } \
} while (0)

/* The firmware's s_rx + s_rx_len + s_rx_overflow, parameterized by capacity so we can
   show the old one-frame size overflows where the accumulator does not. */
typedef struct {
    uint8_t *buf;
    size_t   cap;
    size_t   len;
    bool     overflow;
} rx_t;

static void rx_append(rx_t *r, const uint8_t *data, size_t len) {     /* cl_rx_append */
    if (!cloud_rx_append(r->buf, r->cap, &r->len, data, len)) r->overflow = true;
}

static void rx_consume(rx_t *r, size_t n) {                           /* cl_rx_consume */
    cloud_rx_consume(r->buf, &r->len, n);
}

/* One worst-case server->device frame's worth of decrypted bytes. */
#define FRAME BP_TUNNEL_OUT_FRAME_MAX

static void test_sizing_invariants(void) {
    /* The accumulator must hold at least one full frame... */
    CHECK(BP_CLOUD_RX_ACCUM >= BP_CLOUD_RX_MAX);
    CHECK(BP_CLOUD_RX_MAX   >= BP_TUNNEL_OUT_FRAME_MAX);
    /* ...and, the whole point of the fix, a real BURST of frames — not just one.
       8 frames is the documented floor that keeps a chunked deep-replay upload from
       overflowing s_rx between net-task drains. */
    CHECK(BP_CLOUD_RX_ACCUM >= 8u * FRAME);
}

static void test_accumulator_absorbs_a_burst(void) {
    uint8_t mem[BP_CLOUD_RX_ACCUM];
    rx_t r = { mem, sizeof(mem), 0, false };
    uint8_t frame[FRAME];
    memset(frame, 0xA5, sizeof(frame));

    /* A burst of 8 full frames queues up (net task hasn't drained yet) without
       overflowing — the behavior the 2048-byte buffer could not provide. */
    for (int i = 0; i < 8; i++) rx_append(&r, frame, sizeof(frame));
    CHECK(!r.overflow);
    CHECK(r.len == 8u * FRAME);
}

static void test_old_single_frame_size_overflowed(void) {
    /* Document the regression: with only a per-frame-sized buffer, the SECOND frame
       of a burst overflows — exactly the old "backoff: rx overflow". */
    uint8_t mem[BP_CLOUD_RX_MAX];
    rx_t r = { mem, sizeof(mem), 0, false };
    uint8_t frame[FRAME];
    memset(frame, 0x5A, sizeof(frame));

    rx_append(&r, frame, sizeof(frame));
    CHECK(!r.overflow);                 /* first frame fits */
    rx_append(&r, frame, sizeof(frame));
    CHECK(r.overflow);                  /* second frame overflows a one-frame buffer */
}

static void test_consume_frees_room(void) {
    uint8_t mem[BP_CLOUD_RX_ACCUM];
    rx_t r = { mem, sizeof(mem), 0, false };
    uint8_t frame[FRAME];
    memset(frame, 0x33, sizeof(frame));

    /* Fill to the last frame that fits. */
    size_t fit = BP_CLOUD_RX_ACCUM / FRAME;
    for (size_t i = 0; i < fit; i++) rx_append(&r, frame, sizeof(frame));
    CHECK(!r.overflow);

    /* Draining parsed frames (as cl_process_ws_frames -> cl_rx_consume does) makes
       room for more — a sustained stream keeps flowing without ever overflowing. */
    rx_consume(&r, 4u * FRAME);
    CHECK(r.len == (fit - 4) * FRAME);
    for (int i = 0; i < 4; i++) rx_append(&r, frame, sizeof(frame));
    CHECK(!r.overflow);

    /* Partial consume then verify the shift kept the tail bytes intact. */
    r.len = 0; r.overflow = false;
    uint8_t seq[FRAME];
    for (size_t i = 0; i < sizeof(seq); i++) seq[i] = (uint8_t)i;
    rx_append(&r, seq, sizeof(seq));
    rx_consume(&r, 10);
    CHECK(r.len == sizeof(seq) - 10);
    CHECK(mem[0] == 10 && mem[1] == 11);   /* memmove shifted the remainder down */
}

/* cl_recv_cb: refuse (backpressure) only while there is something to drain. */
static void test_refuse_is_backpressure(void) {
    const size_t cap = BP_CLOUD_RX_ACCUM;
    CHECK(!cloud_rx_refuse(cap, 0, 100));             /* room: take it */
    CHECK(!cloud_rx_refuse(cap, cap - 100, 100));     /* exactly fits */
    CHECK(cloud_rx_refuse(cap, cap - 99, 100));       /* one byte short, something to drain */
    CHECK(cloud_rx_refuse(cap, 1, cap));              /* the net task drains first */
    /* empty and still too big: take it (and overflow) rather than wait for room that never comes */
    CHECK(!cloud_rx_refuse(cap, 0, cap + 1));

    uint8_t mem[64];
    rx_t r = { mem, sizeof(mem), 0, false };
    uint8_t big[65] = {0};
    rx_append(&r, big, sizeof(big));
    CHECK(r.overflow && r.len == 0);                  /* nothing partial written */
    r.overflow = false;
    rx_append(&r, big, 64);
    CHECK(!r.overflow && r.len == 64);
    rx_append(&r, big, 0);                            /* an empty segment fits even when full */
    CHECK(!r.overflow && r.len == 64);
    rx_consume(&r, 1000);                             /* over-consume empties */
    CHECK(r.len == 0);
}

int main(void) {
    test_sizing_invariants();
    test_accumulator_absorbs_a_burst();
    test_old_single_frame_size_overflowed();
    test_consume_frees_room();
    test_refuse_is_backpressure();
    if (failures) { printf("test_cloud_rx: %d FAILURES\n", failures); return 1; }
    printf("test_cloud_rx: all passed\n");
    return 0;
}
