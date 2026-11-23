/* prog_c2.c — Silicon Labs C2 (EFM8, C8051F) for prog_flash (prog_flash.h, prog_c2.h).
 *
 * Pins: C2CK = clk (also the chip's /RST), C2D = data. One C2 clock cycle ("strobe") is
 * set(0,d) set(0,d) set(1,d): two set() calls for the low phase (the C2 minimum is 80 ns), and the
 * clock is high at every flush and delay. The chip drives C2D after the falling edge; the host
 * samples while the clock is low. The pod never drives C2D while the chip may drive it.
 *
 * Frames (AN124), all LSB first:
 *   START (1 cycle, data released), INS (2), then
 *   address write  INS 11: ADDR(8), STOP
 *   address read   INS 10: ADDR(8) driven by the chip (the status byte: bit0 OutReady, bit1 InBusy), STOP
 *   data write     INS 01: LEN(2)=0, DATA(8), WAIT (chip drives 0 until done, then 1), STOP
 *   data read      INS 00: LEN(2)=0, WAIT, DATA(8) driven by the chip, STOP
 * Flash programming goes through the FPI: bytes written to / read from FPDAT, with InBusy/OutReady
 * polled through address reads (AN127). Every FPI command is answered with 0x0D when accepted. */
#include "prog_flash.h"
#include "prog_c2.h"
#include <string.h>

#define AR_DEVID 0x00u
#define AR_REVID 0x01u
#define AR_FPCTL 0x02u

#define FPCTL_HALT   0x02u
#define FPCTL_RESET  0x04u
#define FPCTL_ENABLE 0x01u

#define FPI_GET_DERIV 0x02u
#define FPI_ERASE_DEV 0x03u
#define FPI_BLOCK_RD  0x06u
#define FPI_BLOCK_WR  0x07u
#define FPI_PAGE_ERASE 0x08u
#define FPI_OK        0x0Du

#define STAT_OUT_READY 0x01u
#define STAT_IN_BUSY   0x02u

#define BLOCK_MAX   256u
#define PIECE       32u      /* bytes staged on the stack at a time */
#define POLL_LONG   20000u   /* polls (about 50 us apart) before PROG_E_TIMEOUT */
#define POLL_SHORT  200u
#define WAIT_MAX    255u     /* WAIT cycles before PROG_E_TIMEOUT */
#define E_STAT      (-100)   /* internal: the FPI answered something other than 0x0D */

typedef struct {
    uint8_t  fpdat;        /* C2 address of FPDAT */
    uint8_t  ar;           /* address register selected now (0xFF: unknown) */
    uint8_t  e_init, w_init, r_init;
    uint16_t lim;          /* poll limit for FPI polls */
    uint32_t pos;          /* erase: next page; write/read: byte offset of the next block */
} c2_state_t;

_Static_assert(sizeof(c2_state_t) <= PROG_STATE_BYTES, "c2 state too big");

#define ST(ctx) ((c2_state_t *)(void *)(ctx)->state)

/* ---- one clock cycle and bit helpers ---- */

static void strobe(const prog_wire_ops_t *w, int d) {
    w->set(w->io, 0, d);
    w->set(w->io, 0, d);
    w->set(w->io, 1, d);
}

/* A cycle in which the chip drives C2D: sample while the clock is low. */
static void in_strobe(const prog_wire_ops_t *w) {
    w->set(w->io, 0, 1);
    w->set(w->io, 0, 1);
    w->sample(w->io);
    w->set(w->io, 1, 1);
}

static void put_bits(const prog_wire_ops_t *w, uint32_t v, unsigned n) {
    for (unsigned i = 0; i < n; i++) strobe(w, (int)((v >> i) & 1u));
}

/* START: data released (pulled up), then the pod drives for the host phases. */
static void frame_start(const prog_wire_ops_t *w) {
    w->data_drive(w->io, false);
    strobe(w, 1);
    w->data_drive(w->io, true);
}

/* Collect n sampled bits (LSB first) of a flushed frame. */
static int take_byte(const prog_wire_ops_t *w, uint8_t *v) {
    uint8_t s[8];
    if (w->flush(w->io, s, sizeof s) != 8) return PROG_E_PROTOCOL;
    uint8_t b = 0;
    for (unsigned i = 0; i < 8; i++) b |= (uint8_t)((s[i] & 1u) << i);
    *v = b;
    return PROG_OK;
}

/* WAIT: the chip drives 0 while busy and 1 when done. */
static int wait_done(const prog_wire_ops_t *w) {
    w->data_drive(w->io, false);
    for (unsigned i = 0; i < WAIT_MAX; i++) {
        uint8_t s;
        in_strobe(w);
        if (w->flush(w->io, &s, 1) != 1) return PROG_E_PROTOCOL;
        if (s & 1u) return PROG_OK;
        w->delay_us(w->io, 20);
    }
    return PROG_E_TIMEOUT;
}

static int ar_write(const prog_wire_ops_t *w, uint8_t a) {
    frame_start(w);
    put_bits(w, 3u, 2);
    put_bits(w, a, 8);
    w->data_drive(w->io, false);
    strobe(w, 1);                       /* STOP */
    return PROG_OK;
}

static int ar_read(const prog_wire_ops_t *w, uint8_t *v) {
    frame_start(w);
    put_bits(w, 2u, 2);
    w->data_drive(w->io, false);
    for (unsigned i = 0; i < 8; i++) in_strobe(w);
    strobe(w, 1);                       /* STOP */
    return take_byte(w, v);
}

static int dr_write(const prog_wire_ops_t *w, uint8_t b) {
    frame_start(w);
    put_bits(w, 1u, 2);
    put_bits(w, 0u, 2);                 /* LEN: one byte */
    put_bits(w, b, 8);
    int rc = wait_done(w);
    if (rc) return rc;
    strobe(w, 1);                       /* STOP */
    return PROG_OK;
}

static int dr_read(const prog_wire_ops_t *w, uint8_t *v) {
    frame_start(w);
    put_bits(w, 0u, 2);
    put_bits(w, 0u, 2);                 /* LEN: one byte */
    w->data_drive(w->io, false);
    int rc = wait_done(w);
    if (rc) return rc;
    for (unsigned i = 0; i < 8; i++) in_strobe(w);
    strobe(w, 1);                       /* STOP */
    return take_byte(w, v);
}

static int select_reg(prog_ctx_t *ctx, uint8_t a) {
    c2_state_t *st = ST(ctx);
    if (st->ar == a) return PROG_OK;
    st->ar = a;
    return ar_write(ctx->w, a);
}

/* Reset: C2CK low for >= 20 us (the one long low), then high and 2 us before the first frame. */
static void c2_reset(prog_ctx_t *ctx) {
    const prog_wire_ops_t *w = ctx->w;
    w->data_drive(w->io, false);
    w->set(w->io, 0, 1);
    w->delay_us(w->io, 25);
    w->set(w->io, 1, 1);
    w->delay_us(w->io, 2);
    ST(ctx)->ar = 0xFF;
}

/* ---- FPI: bytes through FPDAT ---- */

static int poll_status(prog_ctx_t *ctx, uint8_t mask, uint8_t want) {
    const prog_wire_ops_t *w = ctx->w;
    for (unsigned i = 0; i < ST(ctx)->lim; i++) {
        uint8_t v;
        int rc = ar_read(w, &v);
        if (rc) return rc;
        if ((v & mask) == want) return PROG_OK;
        w->delay_us(w->io, 50);
    }
    return PROG_E_TIMEOUT;
}

static int fpi_put(prog_ctx_t *ctx, uint8_t b) {
    int rc = select_reg(ctx, ST(ctx)->fpdat);
    if (!rc) rc = dr_write(ctx->w, b);
    if (!rc) rc = poll_status(ctx, STAT_IN_BUSY, 0);
    return rc;
}

static int fpi_get(prog_ctx_t *ctx, uint8_t *b) {
    int rc = poll_status(ctx, STAT_OUT_READY, STAT_OUT_READY);
    if (!rc) rc = select_reg(ctx, ST(ctx)->fpdat);
    if (!rc) rc = dr_read(ctx->w, b);
    return rc;
}

/* The answer to a command or a parameter list: 0x0D or E_STAT. */
static int fpi_status(prog_ctx_t *ctx) {
    uint8_t b;
    int rc = fpi_get(ctx, &b);
    if (rc) return rc;
    return b == FPI_OK ? PROG_OK : E_STAT;
}

/* ---- enter / leave ---- */

static int c2_enter(prog_ctx_t *ctx) {
    const prog_wire_ops_t *w = ctx->w;
    c2_state_t *st = ST(ctx);
    memset(ctx->state, 0, sizeof ctx->state);
    st->fpdat = (ctx->flags & 0xFFu) ? (uint8_t)ctx->flags : (uint8_t)PROG_C2_FPDAT_DEFAULT;
    st->lim = POLL_LONG;
    ctx->phase = "enter";
    c2_reset(ctx);

    ctx->phase = "id";
    uint8_t dev, rev;
    int rc = select_reg(ctx, AR_DEVID);
    if (!rc) rc = dr_read(w, &dev);
    if (!rc) rc = select_reg(ctx, AR_REVID);
    if (!rc) rc = dr_read(w, &rev);
    if (rc) return rc == PROG_E_PROTOCOL ? rc : PROG_E_NO_ANSWER;
    if (dev == 0xFF || dev == 0x00) return PROG_E_NO_ANSWER;   /* a silent line reads ones (or zeros) */
    ctx->id[0] = dev; ctx->id[1] = rev; ctx->id_len = 2;

    ctx->phase = "enter";
    rc = select_reg(ctx, AR_FPCTL);
    if (!rc) rc = dr_write(w, FPCTL_HALT);
    if (!rc) rc = dr_write(w, FPCTL_RESET);
    if (!rc) rc = dr_write(w, FPCTL_ENABLE);
    if (rc) return rc;
    w->delay_us(w->io, 20000);

    /* Derivative ID, where the family has the command: best effort, never an error. */
    uint8_t d;
    st->lim = POLL_SHORT;
    if (fpi_put(ctx, FPI_GET_DERIV) == PROG_OK && fpi_status(ctx) == PROG_OK && fpi_get(ctx, &d) == PROG_OK) {
        ctx->id[2] = d;
        ctx->id_len = 3;
    }
    st->lim = POLL_LONG;
    return PROG_OK;
}

static void c2_leave(prog_ctx_t *ctx, bool run) {
    const prog_wire_ops_t *w = ctx->w;
    ctx->phase = "leave";
    w->data_drive(w->io, false);
    if (run) c2_reset(ctx);              /* the program starts when the reset pulse ends */
    uint8_t s;
    w->flush(w->io, &s, 0);
}

/* ---- erase ---- */

static int range_ok(const prog_ctx_t *ctx) {
    if (ctx->mem != PROG_MEM_FLASH && ctx->mem != PROG_MEM_CONFIG) return PROG_E_ARGS;
    if (ctx->len == 0 || ctx->addr > 0x10000u || ctx->len > 0x10000u - ctx->addr) return PROG_E_ARGS;
    return PROG_OK;
}

/* Device erase needs the 0xDE 0xAD 0xA5 confirm bytes; it also unlocks a locked chip. */
static int erase_device(prog_ctx_t *ctx) {
    ctx->phase = "erase";
    ctx->total = 1; ctx->done = 0;
    int rc = fpi_put(ctx, FPI_ERASE_DEV);
    if (!rc) rc = fpi_status(ctx);
    if (!rc) rc = fpi_put(ctx, 0xDE);
    if (!rc) rc = fpi_put(ctx, 0xAD);
    if (!rc) rc = fpi_put(ctx, 0xA5);
    if (!rc) rc = fpi_status(ctx);
    if (rc == E_STAT) rc = PROG_E_PROTOCOL;
    if (rc) return rc;
    ctx->done = 1;
    return PROG_OK;
}

/* One page per call over [addr, addr+len). */
static int erase_page(prog_ctx_t *ctx) {
    c2_state_t *st = ST(ctx);
    if (ctx->page == 0) return PROG_E_ARGS;
    uint32_t first = ctx->addr / ctx->page, last = (ctx->addr + ctx->len - 1u) / ctx->page;
    if (last > 0xFFu) return PROG_E_ARGS;                 /* the page number is one byte */
    if (!st->e_init) { st->e_init = 1; st->pos = first; ctx->total = last - first + 1u; ctx->done = 0; }
    ctx->phase = "erase";
    int rc = fpi_put(ctx, FPI_PAGE_ERASE);
    if (!rc) rc = fpi_status(ctx);
    if (!rc) rc = fpi_put(ctx, (uint8_t)st->pos);
    if (!rc) rc = fpi_status(ctx);
    if (rc == E_STAT) rc = PROG_E_LOCKED;                 /* a locked chip refuses page erase */
    if (rc) return rc;
    st->pos++;
    ctx->done = st->pos - first;
    if (st->pos > last) { st->e_init = 0; return PROG_OK; }
    return PROG_RUNNING;
}

static int c2_erase(prog_ctx_t *ctx) {
    if (ctx->erase == PROG_ERASE_NONE) return PROG_OK;
    if (ctx->erase == PROG_ERASE_CHIP) return erase_device(ctx);
    int rc = range_ok(ctx);
    return rc ? rc : erase_page(ctx);
}

static int c2_unsupported(prog_ctx_t *ctx) { (void)ctx; return PROG_E_UNSUPPORTED; }

const prog_proto_t prog_proto_c2 = {
    .name = "c2", .uart = false, .uses_nrst = false,
    .mems = (1u<<PROG_MEM_FLASH)|(1u<<PROG_MEM_CONFIG),
    .enter = c2_enter, .erase = c2_erase, .write = c2_unsupported, .read = c2_unsupported,
    .leave = c2_leave,
};
