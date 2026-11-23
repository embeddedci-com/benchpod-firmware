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
 *    after a 20 us low, PDI after ~100 us. A long clock low is only right where the protocol means
 *    a reset (C2); the host tests check the longest low outside it.
 *  - delay_us() sends what is queued first, then waits.
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

/* Real ops on the pod (prog_wire.c): two-wire on (clk, data) LA channels (fpga_swd_arm's rc:
   -2 = the engine is in use), or UPDI on `data` (claims the pin itself; err says why not). */
#define PROG_WIRE_MEM 360u   /* bytes of state the caller provides (mem, 4-byte aligned) */
int  prog_wire_open_2w(prog_wire_ops_t *ops, void *mem, unsigned clk_ch, unsigned data_ch);
int  prog_wire_open_uart(prog_wire_ops_t *ops, void *mem, unsigned data_ch, char *err, size_t cap);
/* The chip's reset on an LA pin (claimed open drain by the caller) instead of the pod's reset pin. */
void prog_wire_reset_pin(prog_wire_ops_t *ops, unsigned la);
void prog_wire_close(prog_wire_ops_t *ops);

#endif /* PROG_WIRE_H */
