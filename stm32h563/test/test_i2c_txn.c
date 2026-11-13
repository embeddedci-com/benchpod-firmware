/* test_i2c_txn.c — I2C master token sequences and reply parsing (i2c_txn.c). */
#include "i2c_txn.h"

#include <stdio.h>
#include <string.h>

static int fails;
#define CHECK(c) do { if (!(c)) { fails++; printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); } } while (0)

/* the gateware tokens (signal_engine.h I2C_TOK_*, ice40/src/i2c_master.v) */
enum { S = 1, P = 2, W = 3, RA = 4, RN = 5 };

static void expect_tok(const i2c_txn_t *t, const uint8_t *want, size_t n) {
    CHECK(t->ntok == n);
    if (t->ntok == n) CHECK(memcmp(t->tok, want, n) == 0);
}

static void test_write_then_read(void) {
    i2c_txn_t t;
    const uint8_t w[1] = { 0x0F };
    CHECK(i2c_txn_xfer(&t, 0x48, w, 1, 2) == 0);
    const uint8_t want[] = { S, W, 0x90, W, 0x0F, S, W, 0x91, RA, RN, P };
    expect_tok(&t, want, sizeof(want));
    CHECK(t.nreply == 5);                       /* addr ack, data ack, addr ack, 2 bytes */
    CHECK(t.bits == 2 + 9 + 9 + 2 + 9 + 9 + 9 + 2);

    uint8_t reply[5] = { 0, 0, 0, 0x01, 0x17 };
    i2c_result_t r;
    i2c_txn_result(&t, reply, &r);
    CHECK(r.ack && r.nack_at == -1 && r.nrx == 2 && r.rx_off == 3);
    CHECK(reply[r.rx_off] == 0x01 && reply[r.rx_off + 1] == 0x17);
}

static void test_address_nack(void) {
    i2c_txn_t t;
    const uint8_t w[1] = { 0x0F };
    CHECK(i2c_txn_xfer(&t, 0x50, w, 1, 2) == 0);
    uint8_t reply[5] = { 1, 1, 1, 0xFF, 0xFF };   /* nobody there: all NACK, reads float high */
    i2c_result_t r;
    i2c_txn_result(&t, reply, &r);
    CHECK(!r.ack && r.nack_at == 0 && r.nrx == 0);
}

static void test_data_nack_and_read_addr_nack(void) {
    i2c_txn_t t;
    const uint8_t w[3] = { 1, 2, 3 };
    CHECK(i2c_txn_xfer(&t, 0x20, w, 3, 0) == 0);
    const uint8_t want[] = { S, W, 0x40, W, 1, W, 2, W, 3, P };
    expect_tok(&t, want, sizeof(want));
    uint8_t reply[4] = { 0, 0, 1, 1 };            /* the 2nd data byte is NACKed */
    i2c_result_t r;
    i2c_txn_result(&t, reply, &r);
    CHECK(!r.ack && r.nack_at == 2);

    CHECK(i2c_txn_xfer(&t, 0x20, NULL, 0, 1) == 0);   /* read only */
    const uint8_t want2[] = { S, W, 0x41, RN, P };
    expect_tok(&t, want2, sizeof(want2));
    uint8_t reply2[2] = { 1, 0xFF };
    i2c_txn_result(&t, reply2, &r);
    CHECK(!r.ack && r.nack_at == 1 && r.nrx == 0);    /* nw + 1: the read phase's address */
}

static void test_probe_and_limits(void) {
    i2c_txn_t t;
    CHECK(i2c_txn_xfer(&t, 0x3C, NULL, 0, 0) == 0);
    const uint8_t want[] = { S, W, 0x78, P };
    expect_tok(&t, want, sizeof(want));
    CHECK(t.nreply == 1);

    uint8_t big[I2C_TXN_WRITE_MAX + 1] = { 0 };
    CHECK(i2c_txn_xfer(&t, 0x80, NULL, 0, 0) == -1);
    CHECK(i2c_txn_xfer(&t, 0x10, big, I2C_TXN_WRITE_MAX + 1, 0) == -1);
    CHECK(i2c_txn_xfer(&t, 0x10, NULL, 0, I2C_TXN_READ_MAX + 1) == -1);
    /* the largest transaction fits the gateware queue, and its reply too */
    CHECK(i2c_txn_xfer(&t, 0x10, big, I2C_TXN_WRITE_MAX, I2C_TXN_READ_MAX) == 0);
    CHECK(t.ntok <= I2C_TXN_TOK_MAX);
    CHECK(t.nreply == 1 + I2C_TXN_WRITE_MAX + 1 + I2C_TXN_READ_MAX);
    CHECK(t.nreply <= I2C_TXN_TOK_MAX);
}

static void test_scan(void) {
    i2c_txn_t t;
    CHECK(i2c_txn_scan(&t, I2C_SCAN_FIRST, I2C_SCAN_LAST) == 0);
    CHECK(t.nreply == I2C_SCAN_LAST - I2C_SCAN_FIRST + 1);
    CHECK(t.ntok == 4u * t.nreply);
    CHECK(t.tok[0] == S && t.tok[1] == W && t.tok[2] == (I2C_SCAN_FIRST << 1) && t.tok[3] == P);
    CHECK(i2c_txn_scan(&t, 0, 0x7F) == 0);             /* the whole range still fits */
    CHECK(i2c_txn_scan(&t, 9, 8) == -1);
}

static void test_time(void) {
    i2c_txn_t t;
    CHECK(i2c_txn_xfer(&t, 0x3C, NULL, 0, 0) == 0);   /* 13 SCL periods */
    /* 100 kHz (half 120): 10 us per period */
    CHECK(i2c_txn_us(&t, 120) == 130);
}

int main(void) {
    test_write_then_read();
    test_address_nack();
    test_data_nack_and_read_addr_nack();
    test_probe_and_limits();
    test_scan();
    test_time();
    if (fails) { printf("test_i2c_txn: %d FAILED\n", fails); return 1; }
    printf("test_i2c_txn: OK\n");
    return 0;
}
