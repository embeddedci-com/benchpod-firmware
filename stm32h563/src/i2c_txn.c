/*
 * i2c_txn.c — see i2c_txn.h.
 */
#include "i2c_txn.h"

#include <string.h>

/* signal_engine.h's tokens, repeated here so this file stays HAL-free (the test checks they match). */
enum { T_START = 1, T_STOP = 2, T_WRITE = 3, T_READ_ACK = 4, T_READ_NACK = 5 };

static void put(i2c_txn_t *t, uint8_t b) { t->tok[t->ntok++] = b; }

static void put_write(i2c_txn_t *t, uint8_t b) {
    put(t, T_WRITE); put(t, b);
    t->nreply++; t->bits += 9u;
}

static void put_cond(i2c_txn_t *t, uint8_t c) { put(t, c); t->bits += 2u; }

int i2c_txn_xfer(i2c_txn_t *t, uint8_t addr7, const uint8_t *w, size_t nw, size_t nr) {
    if (!t || addr7 > 0x7Fu || nw > I2C_TXN_WRITE_MAX || nr > I2C_TXN_READ_MAX || (nw && !w))
        return -1;
    memset(t, 0, sizeof(*t));
    t->nw = nw; t->nr = nr;
    if (nw > 0 || nr == 0) {
        put_cond(t, T_START);
        put_write(t, (uint8_t)(addr7 << 1));
        for (size_t i = 0; i < nw; i++) put_write(t, w[i]);
    }
    if (nr > 0) {
        put_cond(t, T_START);                       /* a repeated START after a write */
        put_write(t, (uint8_t)((addr7 << 1) | 1u));
        for (size_t i = 0; i < nr; i++) {
            put(t, i + 1 < nr ? T_READ_ACK : T_READ_NACK);
            t->nreply++; t->bits += 9u;
        }
    }
    put_cond(t, T_STOP);
    return 0;
}

void i2c_txn_result(const i2c_txn_t *t, const uint8_t *reply, i2c_result_t *res) {
    res->ack = true; res->nack_at = -1; res->nrx = 0; res->rx_off = 0;
    size_t k = 0;
    if (t->nw > 0 || t->nr == 0) {
        for (size_t i = 0; i <= t->nw; i++, k++)
            if ((reply[k] & 1u) && res->nack_at < 0) { res->ack = false; res->nack_at = (int)i; }
    }
    if (t->nr > 0) {
        bool addr_nack = (reply[k] & 1u) != 0;
        k++;
        if (addr_nack && res->nack_at < 0) { res->ack = false; res->nack_at = (int)t->nw + 1; }
        /* Nothing answered the read: the bytes are the pull-ups' 0xFF, not data. */
        if (!addr_nack && !(res->nack_at == 0)) { res->rx_off = k; res->nrx = t->nr; }
    }
}

int i2c_txn_scan(i2c_txn_t *t, uint8_t first, uint8_t last) {
    if (!t || first > last || last > 0x7Fu) return -1;
    memset(t, 0, sizeof(*t));
    for (unsigned a = first; a <= last; a++) {
        if (t->ntok + 4u > I2C_TXN_TOK_MAX) return -1;
        put_cond(t, T_START);
        put_write(t, (uint8_t)(a << 1));
        put_cond(t, T_STOP);
    }
    return 0;
}

uint32_t i2c_txn_us(const i2c_txn_t *t, unsigned half) {
    /* 2 half periods per SCL period at 24 MHz, rounded up */
    uint64_t clk = (uint64_t)t->bits * 2u * half;
    return (uint32_t)((clk + 23u) / 24u);
}
