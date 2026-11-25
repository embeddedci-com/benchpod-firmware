/* prog_wire.c — the pod's pin layer for prog_flash (see prog_wire.h).
 *
 * Two-wire: the SWD engine armed on (clk, data) in its remote_bitbang byte mode. set() queues
 * d/e/f/g, data_drive() O/o, sample() c; flush() feeds the queue to the FPGA and turns the 'c'
 * replies into 0/1. A full queue flushes on its own (keeping the samples for the next flush()).
 * The data pin is held up by the pod's 10k pull while the session runs, so a released line with no
 * chip reads 1, as the protocols expect.
 * UPDI: the soft UART in half duplex on one pin (open drain, 2.2k pull-up, as uart_proxy_start
 * half_duplex does).
 * Runs on the hw worker only. The state lives in memory the caller gives (PROG_WIRE_MEM bytes):
 * prog_job.c lends it from the DAP's idle scratch, so it costs no RAM of its own. */
#include "prog_wire.h"
#include "command_handler_internal.h"   /* uart_pins_* */
#include "signal_engine.h"              /* fpga_swd_*, fpga_uart_* */
#include "la_pins.h"
#include "nrst_ctrl.h"
#include "stm32h5xx_hal.h"

#include <string.h>

#define FEED_MAX    192u
/* A set() with the clock high flushes once the queue holds this much, so the automatic flush
   (which push() falls back to at FEED_MAX) lands with the clock high, never inside a C2 strobe's
   low phase. The headroom is for the bytes between two high levels (a strobe queues < 8). */
#define FEED_SOFT   (FEED_MAX - 16u)
#define SAMPLES_MAX 128u
#define SAMPLES_SOFT (SAMPLES_MAX - 32u)   /* op_set flushes at a clock-high edge past this */

typedef struct {
    uint8_t  clk, data;          /* LA channels (2-wire) */
    bool     two_wire, uart;
    uint8_t  feed[FEED_MAX];
    uint16_t nfeed;
    uint8_t  samples[SAMPLES_MAX];   /* taken by an automatic flush, handed out by the next flush() */
    uint16_t nsampq;                 /* 'c' bytes in feed[] (samples the next feed returns) */
    uint16_t nsamples;
    int      err;                /* a feed failed (engine disarmed) */
    la_pull_token_t data_pull;
    uart_pins_t uart_pins;
    uint32_t baud;
    uint8_t  reset_ch;           /* 0 = the pod's reset pin, else an LA pin (open drain) */
} wire_t;
_Static_assert(sizeof(wire_t) <= PROG_WIRE_MEM, "PROG_WIRE_MEM too small for wire_t");

static void feed_now(wire_t *w_)
{
    wire_t *w = w_;
    size_t off = 0;
    char   rep[SAMPLES_MAX];
    while (off < w->nfeed) {
        size_t got = 0;
        bool   exit_json = false, quit = false;
        size_t room = SAMPLES_MAX - w->nsamples;
        size_t used = fpga_swd_feed_exact(w->feed + off, w->nfeed - off, rep, room, &got, &exit_json, &quit);
        for (size_t i = 0; i < got && w->nsamples < SAMPLES_MAX; i++)
            w->samples[w->nsamples++] = (rep[i] == '1') ? 1u : 0u;
        off += used;
        if (used == 0 || quit) { w->err = -1; break; }
    }
    w->nfeed = 0;
    w->nsampq = 0;
}

static void push(wire_t *w, uint8_t b)
{
    /* A 'c' needs room in samples[]: flush first when the queue could overflow either buffer. These
       are the hard limits; op_set flushes earlier, with the clock high, so they are not reached in a
       normal frame (a flush inside a clock-low phase stretches it by a whole SPI transaction). */
    if (w->nfeed >= FEED_MAX || (b == 'c' && w->nsamples + w->nsampq >= SAMPLES_MAX)) feed_now(w);
    if (b == 'c') w->nsampq++;
    w->feed[w->nfeed++] = b;
}

static void op_set(void *io, int clk, int data)
{
    wire_t *w = io;
    push(w, (uint8_t)('d' + (clk ? 2 : 0) + (data ? 1 : 0)));
    if (clk && (w->nfeed >= FEED_SOFT || w->nsamples + w->nsampq >= SAMPLES_SOFT)) feed_now(w);
}

static void op_drive(void *io, bool drive) { push((wire_t *)io, drive ? 'O' : 'o'); }
static void op_sample(void *io)            { push((wire_t *)io, 'c'); }

static int op_flush(void *io, uint8_t *samples, size_t cap)
{
    wire_t *w = io;
    feed_now(w);
    if (w->err) { w->err = 0; w->nsamples = 0; return -1; }
    size_t n = w->nsamples < cap ? w->nsamples : cap;
    if (samples && n) memcpy(samples, w->samples, n);
    w->nsamples = 0;
    return (int)n;
}

static void op_nrst(void *io, bool low)
{
    wire_t *w = io;
    if (!w->reset_ch) { nrst_ctrl_assert(low); return; }
    if (w->two_wire && w->nfeed) feed_now(w);   /* reset moves after what is queued */
    la_pins_set_level(w->reset_ch, low ? 0 : 1);  /* open drain: 0 drives low, 1 releases */
    (void)fpga_la_set(w->reset_ch, la_pins_drive(w->reset_ch));
}

void prog_wire_reset_pin(prog_wire_ops_t *ops, unsigned la)
{
    wire_t *w = ops->io;
    if (w) w->reset_ch = (uint8_t)la;
}

static void op_delay_us(void *io, uint32_t us)
{
    wire_t *w = io;
    if (w->two_wire && w->nfeed) feed_now(w);      /* the delay starts after what is queued */
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = us * (SystemCoreClock / 1000000u);
    while ((uint32_t)(DWT->CYCCNT - start) < cycles) { }
}

static uint32_t op_now_ms(void *io) { (void)io; return HAL_GetTick(); }

static int op_uart_config(void *io, uint32_t baud, bool parity_even, bool stop2)
{
    wire_t *w = io;
    uint8_t frame = UART_FRAME_HALFDUPLEX | UART_FRAME_PARITY | (parity_even ? 0 : UART_FRAME_ODD) |
                    (stop2 ? UART_FRAME_STOP2 : 0);
    fpga_uart_disable();
    if (fpga_uart_config_frame(w->uart_pins.rx, w->uart_pins.tx, baud, frame, true) != 0) return -1;
    w->baud = baud;
    return 0;
}

static int op_uart_write(void *io, const uint8_t *buf, size_t n)
{
    (void)io;
    if (fpga_uart_write(buf, n) != 0) return -1;
    fpga_uart_tx_pump();
    return 0;
}

static size_t op_uart_read(void *io, uint8_t *buf, size_t n, uint32_t timeout_ms)
{
    (void)io;
    size_t   k = 0;
    uint32_t t0 = HAL_GetTick();
    while (k < n) {
        fpga_uart_tx_pump();
        k += fpga_uart_read(buf + k, n - k);
        if (k >= n || HAL_GetTick() - t0 >= timeout_ms) break;
    }
    return k;
}

static void op_uart_rx_clear(void *io)
{
    (void)io;
    uint8_t junk[32];
    while (fpga_uart_read(junk, sizeof(junk)) > 0) { }
}

static void ops_fill(prog_wire_ops_t *ops, wire_t *w)
{
    memset(ops, 0, sizeof(*ops));
    ops->io = w;
    ops->set = op_set; ops->data_drive = op_drive; ops->sample = op_sample; ops->flush = op_flush;
    ops->nrst = op_nrst; ops->delay_us = op_delay_us; ops->now_ms = op_now_ms;
    ops->uart_config = op_uart_config; ops->uart_write = op_uart_write;
    ops->uart_read = op_uart_read; ops->uart_rx_clear = op_uart_rx_clear;
}

/* Pins are claimed by the caller (prog_flash.c checks and claims before it opens). */
int prog_wire_open_2w(prog_wire_ops_t *ops, void *mem, unsigned clk_ch, unsigned data_ch)
{
    wire_t *w = mem;
    memset(w, 0, sizeof(*w));
    w->clk = (uint8_t)clk_ch; w->data = (uint8_t)data_ch;
    int rc = fpga_swd_arm(clk_ch, data_ch);
    if (rc != 0) return rc;
    w->two_wire = true;
    (void)la_pull_hold(data_ch, LA_PULL_UP, PULL_STR_10K, &w->data_pull);
    ops_fill(ops, w);
    /* Clock high, data released: the idle state every protocol starts from. */
    op_drive(w, false);
    op_set(w, 1, 1);
    feed_now(w);
    return 0;
}

int prog_wire_open_uart(prog_wire_ops_t *ops, void *mem, unsigned data_ch, char *err, size_t cap)
{
    wire_t *w = mem;
    memset(w, 0, sizeof(*w));
    if (!uart_pins_claim(&w->uart_pins, data_ch, data_ch, 0, true, err, cap)) return -1;
    w->uart = true;
    ops_fill(ops, w);
    if (op_uart_config(w, 115200, true, true) != 0) {
        uart_pins_release(&w->uart_pins);
        w->uart = false;
        return -2;
    }
    uart_pins_hold_rx(&w->uart_pins);
    return 0;
}

void prog_wire_close(prog_wire_ops_t *ops)
{
    wire_t *w = ops ? ops->io : NULL;
    if (!w) return;
    if (w->two_wire) {
        w->nfeed = 0; w->nsampq = 0;
        op_drive(w, false);
        op_set(w, 1, 1);
        feed_now(w);
        fpga_swd_disarm();
        la_pull_release(&w->data_pull);
    }
    if (w->uart) {
        fpga_uart_disable();
        uart_pins_release(&w->uart_pins);
    }
    w->two_wire = w->uart = false;
    memset(ops, 0, sizeof(*ops));
}
