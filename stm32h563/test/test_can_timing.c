/* Host test for can_timing.c (classic timing unchanged, CAN FD nominal + data
   timing on the 80 MHz FD kernel clock, TDC offset, DLC <-> length) and
   can_ring.c (the variable-length RX ring). */
#include "can_timing.h"
#include "can_ring.h"

#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); failures++; } } while (0)

static uint32_t total(const can_timing_t *t) { return 1u + t->seg1 + t->seg2; }

/* The timing hits the rate exactly. */
static int exact(uint32_t fclk, uint32_t rate, const can_timing_t *t)
{
    return (uint64_t)t->presc * total(t) * rate == fclk;
}

static void test_classic_unchanged(void)
{
    /* Values the pre-FD solver produced at 25 MHz (bench-proven rates). */
    struct { uint32_t rate, presc, seg1, seg2, sjw; } v[] = {
        { 500000, 2, 20, 4, 4 },   /* 25 tq, 84 % */
        { 250000, 4, 20, 4, 4 },
        { 125000, 8, 20, 4, 4 },
        { 1000000, 1, 20, 4, 4 },
        { 10000, 100, 20, 4, 4 },
    };
    for (unsigned i = 0; i < sizeof(v) / sizeof(v[0]); i++) {
        can_timing_t t;
        CHECK(can_timing_nominal(CAN_CLASSIC_KERNEL_HZ, v[i].rate, false, &t) == 0);
        CHECK(t.presc == v[i].presc && t.seg1 == v[i].seg1 && t.seg2 == v[i].seg2 && t.sjw == v[i].sjw);
        CHECK(exact(CAN_CLASSIC_KERNEL_HZ, v[i].rate, &t));
        CHECK(t.tdco == 0);
    }
    can_timing_t t;
    CHECK(can_timing_nominal(CAN_CLASSIC_KERNEL_HZ, 0, false, &t) == -1);
    CHECK(can_timing_nominal(CAN_CLASSIC_KERNEL_HZ, 333333, false, &t) == -1);
}

static void test_fd_nominal(void)
{
    uint32_t rates[] = { 10000, 20000, 50000, 100000, 125000, 250000, 500000, 800000, 1000000 };
    for (unsigned i = 0; i < sizeof(rates) / sizeof(rates[0]); i++) {
        can_timing_t t;
        CHECK(can_timing_nominal(CAN_FD_KERNEL_HZ, rates[i], true, &t) == 0);
        CHECK(exact(CAN_FD_KERNEL_HZ, rates[i], &t));
        CHECK(t.presc >= 1 && t.presc <= 512);
        CHECK(t.seg1 >= 1 && t.seg1 <= 256 && t.seg2 >= 2 && t.seg2 <= 128);
        CHECK(t.sjw == t.seg2);
        uint32_t sp = can_timing_sample_permille(&t);
        CHECK(sp >= 790 && sp <= 810);
    }
    /* 500 kbit/s: 160 tq at 80 MHz, 80 % sample point. */
    can_timing_t t;
    CHECK(can_timing_nominal(CAN_FD_KERNEL_HZ, 500000, true, &t) == 0);
    CHECK(t.presc == 1 && t.seg1 == 127 && t.seg2 == 32);
}

static void test_fd_data(void)
{
    struct { uint32_t rate, presc, seg1, seg2, sp, tdco; } v[] = {
        { 1000000, 2, 31, 8, 800, 64 },
        { 2000000, 1, 31, 8, 800, 32 },
        { 4000000, 1, 14, 5, 750, 15 },
        { 5000000, 1, 11, 4, 750, 12 },
        { 8000000, 1, 7, 2, 800, 8 },
    };
    for (unsigned i = 0; i < sizeof(v) / sizeof(v[0]); i++) {
        can_timing_t t;
        CHECK(can_timing_data(CAN_FD_KERNEL_HZ, v[i].rate, &t) == 0);
        CHECK(exact(CAN_FD_KERNEL_HZ, v[i].rate, &t));
        CHECK(t.presc == v[i].presc && t.seg1 == v[i].seg1 && t.seg2 == v[i].seg2);
        CHECK(can_timing_sample_permille(&t) == v[i].sp);
        CHECK(t.sjw == t.seg2);
        CHECK(t.tdco == v[i].tdco);
        CHECK(t.seg1 <= 32 && t.seg2 <= 16 && t.sjw <= 16);
    }
    can_timing_t t;
    /* 2 Mbit/s needs 12.5 tq at 25 MHz: why FD runs from 80 MHz. */
    CHECK(can_timing_data(CAN_CLASSIC_KERNEL_HZ, 2000000, &t) == -1);
    CHECK(can_timing_data(CAN_CLASSIC_KERNEL_HZ, 5000000, &t) == 0);
    /* Not exact at 80 MHz: 3 Mbit/s (26.67 tq), 6 Mbit/s (13.3 tq). */
    CHECK(can_timing_data(CAN_FD_KERNEL_HZ, 3000000, &t) == -1);
    CHECK(can_timing_data(CAN_FD_KERNEL_HZ, 6000000, &t) == -1);
    /* Above the transceiver rating. */
    CHECK(can_timing_data(CAN_FD_KERNEL_HZ, 10000000, &t) == -2);
    CHECK(can_timing_data(CAN_FD_KERNEL_HZ, 0, &t) == -1);
    /* A slow data rate needs a prescaler > 2: TDC off. */
    CHECK(can_timing_data(CAN_FD_KERNEL_HZ, 500000, &t) == 0);
    CHECK(t.presc > 2 && t.tdco == 0);
}

static void test_dlc(void)
{
    for (uint32_t n = 0; n <= 8; n++) {
        CHECK(can_len_to_dlc(n, false) == (int)n);
        CHECK(can_len_to_dlc(n, true) == (int)n);
        CHECK(can_dlc_to_len(n) == n);
    }
    CHECK(can_len_to_dlc(9, false) == -1);
    CHECK(can_len_to_dlc(9, true) == 9 && can_dlc_to_len(9) == 12);
    CHECK(can_len_to_dlc(12, true) == 9);
    CHECK(can_len_to_dlc(13, true) == 10 && can_dlc_to_len(10) == 16);
    CHECK(can_len_to_dlc(20, true) == 11);
    CHECK(can_len_to_dlc(24, true) == 12);
    CHECK(can_len_to_dlc(25, true) == 13 && can_dlc_to_len(13) == 32);
    CHECK(can_len_to_dlc(33, true) == 14 && can_dlc_to_len(14) == 48);
    CHECK(can_len_to_dlc(64, true) == 15 && can_dlc_to_len(15) == 64);
    CHECK(can_len_to_dlc(65, true) == -1);
}

static can_frame_t mk(uint32_t id, uint8_t dlc, bool fd)
{
    can_frame_t f;
    memset(&f, 0, sizeof(f));
    f.id = id; f.dlc = dlc; f.fd = fd; f.brs = fd; f.ext = (id & 1u) != 0u;
    f.ts = id * 3u;
    for (uint8_t i = 0; i < dlc; i++) f.data[i] = (uint8_t)(id + i);
    return f;
}

static bool same(const can_frame_t *a, const can_frame_t *b)
{
    return a->id == b->id && a->dlc == b->dlc && a->fd == b->fd && a->brs == b->brs &&
           a->ext == b->ext && a->rtr == b->rtr && a->ts == b->ts &&
           memcmp(a->data, b->data, a->dlc) == 0;
}

static void test_ring(void)
{
    static uint8_t buf[256] __attribute__((aligned(4)));
    can_ring_t r;
    can_frame_t o;
    can_ring_init(&r, buf, sizeof(buf));
    CHECK(!can_ring_peek(&r, &o) && !can_ring_pop(&r, &o) && can_ring_count(&r) == 0);

    /* Fill with 64-byte FD frames (76 B each): 3 fit in 256 (228), not 4. */
    for (uint32_t i = 0; i < 3; i++) { can_frame_t f = mk(100 + i, 64, true); CHECK(can_ring_push(&r, &f)); }
    { can_frame_t f = mk(200, 64, true); CHECK(!can_ring_push(&r, &f)); }
    CHECK(can_ring_count(&r) == 3);
    for (uint32_t i = 0; i < 3; i++) {
        can_frame_t e = mk(100 + i, 64, true);
        CHECK(can_ring_peek(&r, &o) && same(&o, &e));
        CHECK(can_ring_pop(&r, &o) && same(&o, &e));
    }
    CHECK(can_ring_count(&r) == 0);

    /* Mixed sizes through many wraps, interleaved push/pop, FIFO order kept. */
    uint32_t next_in = 0, next_out = 0;
    for (int step = 0; step < 5000; step++) {
        uint8_t dlc = (uint8_t)((next_in * 7u) % 65u);
        can_frame_t f = mk(next_in, dlc, dlc > 8);
        bool pushed = can_ring_push(&r, &f);
        if (pushed) next_in++;
        if (!pushed || (step % 3) == 0) {
            if (can_ring_pop(&r, &o)) {
                uint8_t edlc = (uint8_t)((next_out * 7u) % 65u);
                can_frame_t e = mk(next_out, edlc, edlc > 8);
                CHECK(same(&o, &e));
                next_out++;
            }
        }
        CHECK(can_ring_count(&r) == next_in - next_out);
    }
    while (can_ring_pop(&r, &o)) {
        uint8_t edlc = (uint8_t)((next_out * 7u) % 65u);
        can_frame_t e = mk(next_out, edlc, edlc > 8);
        CHECK(same(&o, &e));
        next_out++;
    }
    CHECK(next_out == next_in && next_in > 1500);

    /* Classic 8-byte frames: 20 B each, 12 fit in 256 (a full ring keeps one gap). */
    can_ring_reset(&r);
    int n = 0;
    for (;;) { can_frame_t f = mk((uint32_t)n, 8, false); if (!can_ring_push(&r, &f)) break; n++; }
    CHECK(n == 12);
    /* The production ring: 1024 B holds 51 classic frames, 13 FD-64 frames. */
    static uint8_t big[1024] __attribute__((aligned(4)));
    can_ring_init(&r, big, sizeof(big));
    n = 0;
    for (;;) { can_frame_t f = mk((uint32_t)n, 8, false); if (!can_ring_push(&r, &f)) break; n++; }
    CHECK(n == 51);
    can_ring_reset(&r);
    n = 0;
    for (;;) { can_frame_t f = mk((uint32_t)n, 64, true); if (!can_ring_push(&r, &f)) break; n++; }
    CHECK(n == 13);
}

int main(void)
{
    test_classic_unchanged();
    test_fd_nominal();
    test_fd_data();
    test_dlc();
    test_ring();
    if (failures) { printf("test_can_timing: %d FAILURE(S)\n", failures); return 1; }
    printf("test_can_timing: all passed\n");
    return 0;
}
