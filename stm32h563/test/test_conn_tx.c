/*
 * test_conn_tx.c — host unit tests for the per-connection TX rings that decouple
 * the hw-worker task (producer) from the net task (consumer/lwIP).  Exercises the
 * free/used accounting, fill-to-capacity, and the contiguous peek across the
 * buffer wrap (the easy-to-get-wrong part of the SPSC ring).
 */
#include "conn_tx.h"
#include "command_handler.h"   /* conn id ranges */

#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); failures++; } \
} while (0)

#define CONN 0   /* a TCP conn slot */

static void drain_all(int conn) {
    const uint8_t *p;
    size_t n;
    while ((n = conn_tx_peek(conn, &p)) > 0) conn_tx_advance(conn, n);
}

static void test_basic(void) {
    conn_tx_reset(CONN);
    CHECK(conn_tx_used(CONN) == 0);
    size_t cap = conn_tx_free(CONN);
    CHECK(cap > 1000);   /* ring is a couple KB */

    uint8_t data[100];
    for (int i = 0; i < 100; i++) data[i] = (uint8_t)i;
    CHECK(conn_tx_write(CONN, data, 100) == 100);
    CHECK(conn_tx_used(CONN) == 100);
    CHECK(conn_tx_free(CONN) == cap - 100);

    const uint8_t *p;
    size_t n = conn_tx_peek(CONN, &p);
    CHECK(n == 100);
    CHECK(memcmp(p, data, 100) == 0);
    conn_tx_advance(CONN, 100);
    CHECK(conn_tx_used(CONN) == 0);
}

static void test_fill_and_short_write(void) {
    conn_tx_reset(CONN);
    size_t cap = conn_tx_free(CONN);
    static uint8_t big[4096];
    memset(big, 0xAB, sizeof(big));
    /* Writing more than capacity queues exactly `cap` bytes (short write). */
    size_t w = conn_tx_write(CONN, big, sizeof(big));
    CHECK(w == cap);
    CHECK(conn_tx_free(CONN) == 0);
    /* Another write queues nothing. */
    CHECK(conn_tx_write(CONN, big, 10) == 0);
    drain_all(CONN);
    CHECK(conn_tx_used(CONN) == 0);
}

/* Reconstruct a written buffer through the peek/advance API (which returns at most
   a contiguous run, so a wrapped payload comes back in two pieces). */
static void write_read_verify(const uint8_t *data, size_t len) {
    CHECK(conn_tx_write(CONN, data, len) == len);
    CHECK(conn_tx_used(CONN) == len);
    uint8_t out[256];
    size_t got = 0;
    const uint8_t *p;
    size_t n;
    while (got < len && (n = conn_tx_peek(CONN, &p)) > 0) {
        memcpy(out + got, p, n);
        conn_tx_advance(CONN, n);
        got += n;
    }
    CHECK(got == len);
    CHECK(memcmp(out, data, len) == 0);
    CHECK(conn_tx_used(CONN) == 0);
}

static void test_wrap(void) {
    uint8_t data[100];
    for (int i = 0; i < 100; i++) data[i] = (uint8_t)(i + 1);
    /* Sweep the head position across the whole ring (via write+drain of varying
       sizes) so the subsequent 100-byte write straddles the buffer end on some
       iterations — the reconstruction must be correct regardless. */
    conn_tx_reset(CONN);
    for (int step = 0; step < 400; step++) {
        write_read_verify(data, 100);
        /* Nudge head forward by a non-divisor amount so we exercise every wrap phase. */
        static uint8_t filler[37];
        memset(filler, 0, sizeof(filler));
        CHECK(conn_tx_write(CONN, filler, sizeof(filler)) == sizeof(filler));
        drain_all(CONN);
    }
}

static void test_no_ring(void) {
    /* The console / cloud-command pseudo-conns have no ring. */
    CHECK(conn_tx_slot(CH_CONSOLE_CONN) < 0);
    CHECK(conn_tx_slot(CH_CLOUD_CONN) < 0);
    CHECK(conn_tx_write(CH_CONSOLE_CONN, (const uint8_t *)"x", 1) == 0);
    /* A tunnel conn does have a ring. */
    CHECK(conn_tx_slot(CH_CLOUD_TUNNEL_CONN) >= 0);
}

int main(void) {
    test_basic();
    test_fill_and_short_write();
    test_wrap();
    test_no_ring();
    if (failures) { printf("FAILED — %d conn_tx checks\n", failures); return 1; }
    printf("PASS — all conn_tx tests\n");
    return 0;
}
