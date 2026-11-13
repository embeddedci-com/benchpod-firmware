/*
 * i2c_txn.h — I2C master transactions as gateware tokens, and their replies (pure C, host-tested
 * in test/test_i2c_txn.c).  The tokens are signal_engine.h's I2C_TOK_*; command_handler_i2c.c runs
 * them with fpga_i2c_run().
 */
#ifndef I2C_TXN_H
#define I2C_TXN_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define I2C_TXN_TOK_MAX     512u    /* = I2C_QUEUE_MAX */
#define I2C_TXN_WRITE_MAX   128u    /* data bytes written per transaction */
#define I2C_TXN_READ_MAX    248u    /* bytes read per transaction */
#define I2C_SCAN_FIRST      0x08u   /* 0x00..0x07 and 0x78..0x7F are reserved addresses */
#define I2C_SCAN_LAST       0x77u

typedef struct {
    uint8_t  tok[I2C_TXN_TOK_MAX];
    size_t   ntok;
    size_t   nreply;      /* reply bytes the tokens produce */
    size_t   nw, nr;      /* data bytes written / read */
    uint32_t bits;        /* SCL periods, for the time budget */
} i2c_txn_t;

typedef struct {
    bool    ack;          /* every address and data byte the pod wrote was acknowledged */
    int     nack_at;      /* -1 none; 0 the address (write phase, or the only phase); 1..nw a data
                             byte (1-based); nw+1 the address of the read phase */
    size_t  nrx;          /* bytes read (0 when an address was not acknowledged) */
    size_t  rx_off;       /* where they start in the reply */
} i2c_result_t;

/* START, address+W, w[0..nw), then (when nr > 0) a repeated START, address+R, nr reads (ACK all
   but the last, which is NACKed), STOP.  nw == 0 && nr > 0 reads only; nw == 0 && nr == 0 probes
   the address.  0 ok; -1 addr > 0x7F, nw > I2C_TXN_WRITE_MAX or nr > I2C_TXN_READ_MAX. */
int  i2c_txn_xfer(i2c_txn_t *t, uint8_t addr7, const uint8_t *w, size_t nw, size_t nr);

/* reply[0..t->nreply) from the run -> res; the bytes read are reply[res->rx_off..+res->nrx). */
void i2c_txn_result(const i2c_txn_t *t, const uint8_t *reply, i2c_result_t *res);

/* One START, address+W, STOP per address in first..last (each reply byte: 0 = present). */
int  i2c_txn_scan(i2c_txn_t *t, uint8_t first, uint8_t last);

/* How long the tokens take at `half` (24 MHz clk per half period) with no clock stretching. */
uint32_t i2c_txn_us(const i2c_txn_t *t, unsigned half);

#endif
