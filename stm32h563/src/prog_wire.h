#ifndef PROG_WIRE_H
#define PROG_WIRE_H

/* prog_wire — the pin layer under the pin programmers (prog_flash.h).
 *
 * Two-wire protocols (PIC ICSP, SiLabs C2, AVR TPI, XMEGA PDI) drive a clock pin and a data pin
 * through the SWD engine's remote_bitbang byte mode (fpga_swd_arm + fpga_swd_feed): every call
 * below appends bytes to a feed buffer, and prog_wire_flush() sends them. The FPGA applies the
 * bytes in order, so the timing between two calls is one feed byte (measure: plan step 0), not a
 * fixed bit period. Rules for the protocols:
 *  - Leave the clock HIGH at every flush and every delay longer than a few us: C2 resets the chip
 *    after a 20 us low, PDI after ~100 us. prog_wire_flush() refuses (returns -1) when the clock
 *    is low at the flush, except inside a reset the protocol asked for (prog_wire_delay_low_us).
 *  - Samples come back in order, one per prog_wire_sample(), after the flush.
 *
 * UPDI drives one pin through the UART engine in half duplex (open drain, 8E2): the byte calls
 * at the bottom.
 *
 * The ops struct lets host tests plug in a bit-level model of the chip (test/prog_sim.h).
 */

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

typedef struct {
    void *io;
    /* Two-wire: queue a level change, drive/release data, take a sample of data. */
    void (*set)(void *io, int clk, int data);       /* both levels at once (rbb d/e/f/g) */
    void (*data_drive)(void *io, bool drive);        /* true: pod drives data; false: released */
    void (*sample)(void *io);                        /* queue one sample of data */
    /* Send the queue; samples[] gets one 0/1 per queued sample. Returns samples read, < 0 error. */
    int  (*flush)(void *io, uint8_t *samples, size_t cap);
    /* The pod's reset pin (open drain): true = held low. */
    void (*nrst)(void *io, bool low);
    void (*delay_us)(void *io, uint32_t us);
    uint32_t (*now_ms)(void *io);
    /* UPDI: one pin, UART engine, half duplex. */
    int  (*uart_config)(void *io, uint32_t baud, bool parity_even, bool stop2);
    int  (*uart_write)(void *io, const uint8_t *buf, size_t n);
    /* Up to n bytes, waiting up to timeout_ms in all; returns bytes read. */
    size_t (*uart_read)(void *io, uint8_t *buf, size_t n, uint32_t timeout_ms);
    void (*uart_rx_clear)(void *io);
} prog_wire_ops_t;

/* Real ops on the pod (prog_wire.c): two-wire on (clk, data) LA channels, or UPDI on `data`. */
int  prog_wire_open_2w(prog_wire_ops_t *ops, unsigned clk_ch, unsigned data_ch);
int  prog_wire_open_uart(prog_wire_ops_t *ops, unsigned data_ch);
void prog_wire_close(prog_wire_ops_t *ops);

#endif /* PROG_WIRE_H */
