/* prog_c2.c — Silicon Labs C2 (EFM8, C8051F) for prog_flash (prog_flash.h, prog_c2.h).
 *
 * Pins: C2CK = clk (also the chip's /RST), C2D = data. One C2 clock cycle ("strobe") is
 * set(0,d) set(0,d) set(1,d): two set() calls for the low phase (the C2 minimum is 80 ns), and the
 * clock is high at every flush and delay. The chip drives C2D after the RISING edge of C2CK
 * (AN127 1.4; AN127SW: STROBE_C2CK(); then read C2D): the host samples with the clock high, at
 * least 120 ns after the edge. The feed byte time is not measured yet (prog_wire.h); this file
 * assumes >= 40 ns per byte, so an input cycle queues SAMPLE_HOLD = 3 more set(1,..) bytes between
 * the edge and the sample. The pod never drives C2D while the chip may drive it.
 *
 * Frames (AN124), all LSB first:
 *   START (1 cycle, data released), INS (2), then
 *   address write  INS 11: ADDR(8), STOP
 *   address read   INS 10: ADDR(8) driven by the chip (the status byte: bit0 OutReady, bit1 InBusy), STOP
 *   data write     INS 01: LEN(2)=0, DATA(8), WAIT (chip drives 0 until done, then 1), STOP
 *   data read      INS 00: LEN(2)=0, WAIT, DATA(8) driven by the chip, STOP
 * Flash programming goes through the FPI: bytes written to / read from FPDAT, with InBusy/OutReady
 * polled through address reads (AN127). Every FPI command is answered with 0x0D when accepted.
 * An FPI answer other than 0x0D fails the call (PROG_E_LOCKED when the first answer of an erase or
 * block command refuses, else PROG_E_PROTOCOL) and ctx->at_addr holds that raw status byte.
 * Before the first erase or write, the device-specific setup of AN127 3.8 Table 3.6 runs (VDD
 * monitor, flash timing, clock): EFM8 BB1/BB2/BB3/LB1/UB1/UB2/SB1/SB2, keyed by device ID. The IDs
 * that collide with C8051F families (0x30 F85x, 0x25 F99x, 0x16 F92x, 0x28 F38x) have the same
 * sequence. Other C8051F rows are not in the table (flash is tight): ctx->flags C2_FLAG_NO_SETUP
 * skips it. An ID that is not in the table is not refused: it gets no setup (writing SFRs of the
 * wrong family could hit unrelated registers) and FPDAT 0xB4. EFM8BB5x (BB50/51/52) is not in
 * AN127 Rev 1.4; its device ID and C2 page size are unconfirmed (SDK: 2 KB flash page, and no
 * VDM0CN at SFR 0xFF), so the host passes ctx->page until it is measured on the bench. */
#include "prog_flash.h"
#include "prog_c2.h"
#include <string.h>

#define AR_DEVID 0x00u
#define AR_REVID 0x01u
#define AR_FPCTL 0x02u

/* FPCTL key sequence 0x02, 0x04, 0x01 (AN127 2.1; AN127SW names: RESET, CORE_RESET, HALT) */
#define FPCTL_RESET      0x02u
#define FPCTL_CORE_RESET 0x04u
#define FPCTL_HALT       0x01u

#define FPI_GET_DERIV 0x02u
#define FPI_ERASE_DEV 0x03u
#define FPI_BLOCK_RD  0x06u
#define FPI_BLOCK_WR  0x07u
#define FPI_PAGE_ERASE 0x08u
#define FPI_DIRECT_WR 0x0Au
#define FPI_OK        0x0Du

#define STAT_OUT_READY 0x01u
#define STAT_IN_BUSY   0x02u

#define BLOCK_MAX   256u
#define PIECE       32u      /* bytes staged on the stack at a time */
#define POLL_LONG   20000u   /* polls (about 50 us apart) before PROG_E_TIMEOUT: ~1 s of delay plus the
                                polls themselves; the device erase takes up to 20 s (AN127SW) */
#define SAMPLE_HOLD 3u       /* set(1,..) bytes after the rising edge before a sample (>= 120 ns) */
#define POLL_SHORT  200u
#define WAIT_MAX    255u     /* WAIT cycles before PROG_E_TIMEOUT */
#define E_STAT      (-100)   /* internal: the FPI answered something other than 0x0D */

typedef struct {
    uint8_t  fpdat;        /* C2 address of FPDAT */
    uint8_t  ar;           /* address register selected now (0xFF: unknown) */
    uint8_t  e_init, w_init, r_init;
    uint8_t  setup_done;   /* the device setup ran */
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

/* A cycle in which the chip drives C2D: sample with the clock high, after the rising edge. */
static void in_strobe(const prog_wire_ops_t *w) {
    w->set(w->io, 0, 1);
    w->set(w->io, 0, 1);
    for (unsigned i = 0; i <= SAMPLE_HOLD; i++) w->set(w->io, 1, 1);   /* edge, then the hold */
    w->sample(w->io);
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

/* The answer to a command or a parameter list: 0x0D, or E_STAT with the raw byte in at_addr. */
static int fpi_status(prog_ctx_t *ctx) {
    uint8_t b;
    int rc = fpi_get(ctx, &b);
    if (rc) return rc;
    if (b == FPI_OK) return PROG_OK;
    ctx->at_addr = b;
    return E_STAT;
}

/* ---- device tables (AN127 3.8, Tables 3.5 and 3.6) ---- */

/* FPDAT: 0xAD on EFM8UB2 and 'F38x (0x28), 'F34x (0x0F), 'T62x/'T32x (0x18, 0x19; 0x1A in
   AN127SW); 0xB4 on everything else, including IDs this table does not know. A host flag wins. */
static uint8_t fpdat_for(uint8_t dev, uint32_t flags) {
    if (flags & 0xFFu) return (uint8_t)flags;
    if (dev == 0x28 || dev == 0x0F || (dev >= 0x18 && dev <= 0x1A)) return 0xADu;
    return (uint8_t)PROG_C2_FPDAT_DEFAULT;
}

/* Setup sequences: (SFR, value) pairs, SFR 0 = wait 5 us, end at SFR 0 with value 0xFF.
   Written with WriteSFR (address write, data write) or, on SB1/SB2 ("WriteDirect" in the table),
   with the FPI direct write command. */
typedef struct { uint8_t sfr, val; } c2_op_t;
#define OPS_END { 0, 0xFF }
static const c2_op_t k_seq_efm8[] = { {0xFF, 0x80}, {0, 0}, {0xEF, 0x02}, {0xA9, 0x00}, OPS_END };   /* BB1 BB2 BB3 LB1 UB1 */
static const c2_op_t k_seq_ub2[]  = { {0xB6, 0x90}, {0xFF, 0x80}, {0xEF, 0x02}, {0xA9, 0x03}, OPS_END };
static const c2_op_t k_seq_sb1[]  = { {0xB6, 0x40}, {0xFF, 0x80}, {0xEF, 0x02}, {0xA9, 0x04}, OPS_END };
static const c2_op_t k_seq_sb2[]  = { {0xA7, 0x00}, {0xB2, 0x8F}, {0xA9, 0x00}, OPS_END };

static const c2_op_t k_seq_none[] = { OPS_END };
/* Returns the sequence for a device ID; *known is false for an ID not in the table (no setup). */
static const c2_op_t *seq_for(uint8_t dev, bool *direct, bool *known) {
    *direct = false; *known = true;
    switch (dev) {
    case 0x30: case 0x32: case 0x34: return k_seq_efm8;
    case 0x28: return k_seq_ub2;
    case 0x25: *direct = true; return k_seq_sb1;
    case 0x16: *direct = true; return k_seq_sb2;
    default: *known = false; return k_seq_none;
    }
}

static int sfr_write(prog_ctx_t *ctx, uint8_t a, uint8_t v) {
    int rc = ar_write(ctx->w, a);
    if (!rc) rc = dr_write(ctx->w, v);
    ST(ctx)->ar = 0xFF;                  /* the address register no longer selects FPDAT */
    return rc;
}

static int direct_write(prog_ctx_t *ctx, uint8_t a, uint8_t v) {
    int rc = fpi_put(ctx, FPI_DIRECT_WR);
    if (!rc) rc = fpi_status(ctx);
    if (!rc) rc = fpi_put(ctx, a);
    if (!rc) rc = fpi_put(ctx, 1);
    if (!rc) rc = fpi_put(ctx, v);
    return rc;
}

/* Before the first erase or write. A failed answer is a protocol error. */
static int ensure_setup(prog_ctx_t *ctx) {
    c2_state_t *st = ST(ctx);
    if (st->setup_done || (ctx->flags & C2_FLAG_NO_SETUP)) return PROG_OK;
    bool direct, known;
    const c2_op_t *q = seq_for(ctx->id[0], &direct, &known);
    ctx->phase = known ? "setup" : "setup-none";
    for (int rc; q->sfr || q->val != 0xFF; q++) {
        if (q->sfr == 0) { ctx->w->delay_us(ctx->w->io, 5); continue; }
        rc = direct ? direct_write(ctx, q->sfr, q->val) : sfr_write(ctx, q->sfr, q->val);
        if (rc) return rc == E_STAT ? PROG_E_PROTOCOL : rc;
    }
    st->setup_done = 1;
    return PROG_OK;
}

/* ---- enter / leave ---- */

static int c2_enter(prog_ctx_t *ctx) {
    const prog_wire_ops_t *w = ctx->w;
    c2_state_t *st = ST(ctx);
    memset(ctx->state, 0, sizeof ctx->state);
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
    st->fpdat = fpdat_for(dev, ctx->flags);

    ctx->phase = "enter";
    rc = select_reg(ctx, AR_FPCTL);
    if (!rc) rc = dr_write(w, FPCTL_RESET);
    if (!rc) rc = dr_write(w, FPCTL_CORE_RESET);
    if (!rc) rc = dr_write(w, FPCTL_HALT);
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
    int rc = ensure_setup(ctx);
    if (rc) return rc;
    ctx->phase = "erase";
    ctx->total = 1; ctx->done = 0;
    rc = fpi_put(ctx, FPI_ERASE_DEV);
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
    int rc = ensure_setup(ctx);
    if (rc) return rc;
    ctx->phase = "erase";
    rc = fpi_put(ctx, FPI_PAGE_ERASE);
    if (!rc) rc = fpi_status(ctx);
    if (rc == E_STAT) return PROG_E_LOCKED;               /* a locked chip refuses page erase */
    /* page number, status, then 0x00 starts the erase and the third status ends it (AN127 3.3:
       up to 40 ms; the status poll below allows far more) */
    if (!rc) rc = fpi_put(ctx, (uint8_t)st->pos);
    if (!rc) rc = fpi_status(ctx);
    if (!rc) rc = fpi_put(ctx, 0x00);
    if (!rc) rc = fpi_status(ctx);
    if (rc == E_STAT) rc = PROG_E_PROTOCOL;
    if (rc) { st->e_init = 0; return rc; }
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

/* ---- block write (verified) and block read ---- */

/* Length of the block at image offset off: up to 256 bytes, inside one flash page. */
static uint32_t block_len(const prog_ctx_t *ctx, uint32_t off, bool paged) {
    uint32_t n = ctx->len - off;
    if (n > BLOCK_MAX) n = BLOCK_MAX;
    if (paged && ctx->page) {
        uint32_t room = ctx->page - (ctx->addr + off) % ctx->page;
        if (n > room) n = room;
    }
    return n;
}

/* Block command header: command, status, address (hi, lo), length (0 = 256), status. */
static int block_open(prog_ctx_t *ctx, uint8_t cmd, uint32_t a, uint32_t n) {
    int rc = fpi_put(ctx, cmd);
    if (!rc) rc = fpi_status(ctx);
    if (rc) return rc == E_STAT ? PROG_E_LOCKED : rc;   /* a locked chip refuses flash access */
    rc = fpi_put(ctx, (uint8_t)(a >> 8));
    if (!rc) rc = fpi_put(ctx, (uint8_t)a);
    if (!rc) rc = fpi_put(ctx, (uint8_t)n);
    if (!rc) rc = fpi_status(ctx);
    return rc == E_STAT ? PROG_E_PROTOCOL : rc;
}

/* Read n bytes of the block just opened; to dst (off) or compared against the image. */
static int block_stream_read(prog_ctx_t *ctx, uint32_t off, uint32_t n, bool verify) {
    uint8_t buf[PIECE], ref[PIECE];
    for (uint32_t i = 0; i < n; i += PIECE) {
        uint32_t k = n - i < PIECE ? n - i : PIECE;
        if (verify && ctx->src(ctx->job, off + i, ref, k) != 0) return PROG_E_SRC;
        for (uint32_t j = 0; j < k; j++) {
            int rc = fpi_get(ctx, &buf[j]);
            if (rc) return rc;
        }
        if (verify) {
            for (uint32_t j = 0; j < k; j++)
                if (buf[j] != ref[j]) { ctx->at_addr = ctx->addr + off + i + j; return PROG_E_VERIFY; }
        } else if (ctx->dst(ctx->job, off + i, buf, k) != 0) return PROG_E_SRC;
    }
    return PROG_OK;
}

static int c2_write(prog_ctx_t *ctx) {
    c2_state_t *st = ST(ctx);
    int rc = range_ok(ctx);
    if (rc) return rc;
    if (!st->w_init) {
        st->w_init = 1; st->pos = 0; ctx->done = 0; ctx->total = 0;
        for (uint32_t o = 0; o < ctx->len; o += block_len(ctx, o, true)) ctx->total++;
    }
    uint32_t off = st->pos, n = block_len(ctx, off, true), a = ctx->addr + off;
    rc = ensure_setup(ctx);
    if (rc) { st->w_init = 0; return rc; }
    ctx->phase = "write";
    rc = block_open(ctx, FPI_BLOCK_WR, a, n);
    for (uint32_t i = 0; !rc && i < n;) {
        uint8_t buf[PIECE];
        uint32_t k = n - i < PIECE ? n - i : PIECE;
        if (ctx->src(ctx->job, off + i, buf, k) != 0) { rc = PROG_E_SRC; break; }
        for (uint32_t j = 0; !rc && j < k; j++) rc = fpi_put(ctx, buf[j]);
        i += k;
    }
    if (!rc) { rc = fpi_status(ctx); if (rc == E_STAT) rc = PROG_E_PROTOCOL; }
    if (!rc && ctx->mem == PROG_MEM_FLASH) {   /* the lock byte is not read back: once set, reads are refused */
        ctx->phase = "verify";
        rc = block_open(ctx, FPI_BLOCK_RD, a, n);
        if (!rc) rc = block_stream_read(ctx, off, n, true);
    }
    if (rc) { st->w_init = 0; return rc; }
    st->pos = off + n;
    ctx->done++;
    if (st->pos >= ctx->len) { st->w_init = 0; return PROG_OK; }
    return PROG_RUNNING;
}

static int c2_read(prog_ctx_t *ctx) {
    c2_state_t *st = ST(ctx);
    int rc = range_ok(ctx);
    if (rc) return rc;
    if (!st->r_init) {
        st->r_init = 1; st->pos = 0; ctx->done = 0;
        ctx->total = (ctx->len + BLOCK_MAX - 1u) / BLOCK_MAX;
    }
    uint32_t off = st->pos, n = block_len(ctx, off, false);
    ctx->phase = "read";
    rc = block_open(ctx, FPI_BLOCK_RD, ctx->addr + off, n);
    if (!rc) rc = block_stream_read(ctx, off, n, false);
    if (rc) { st->r_init = 0; return rc; }
    st->pos = off + n;
    ctx->done++;
    if (st->pos >= ctx->len) { st->r_init = 0; return PROG_OK; }
    return PROG_RUNNING;
}


PROG_PROTO(c2, c2_enter, c2_erase, c2_write, c2_read, c2_leave, PROG_INFO_C2)
