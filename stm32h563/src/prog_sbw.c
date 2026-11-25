/* prog_sbw.c: MSP430 Spy-Bi-Wire for prog_flash (prog_flash.h, prog_sbw.h).
 *
 * Written from docs/design/spy-bi-wire.md (section numbers below); that document cites SLAU320AJ, the
 * family user's guides, the datasheets and TI's Replicator430FR source for every sequence.
 *
 * Layers:
 *  1. Bit layer (sbw_flush, sbw_cycle, sbw_pins, sbw_entry): the only code that touches prog_wire.
 *     A gateware slot engine (docs section 15) replaces exactly this part. One JTAG clock = one
 *     "cycle" = three slots (TMS, TDI, TDO) of SBWTCK, queued as bytes of prog_wire's byte mode:
 *       TMS slot: set(1,tms) set(0,tms) set(1,tms)      TDI slot: the same with tdi
 *       TDO slot: release, set(0) , sample, set(1), drive   (docs 14.1)
 *     The clock is HIGH at every flush: the layer counts the queued bytes itself and flushes at a cycle
 *     boundary, because prog_wire's own guard (nsamples + nfeed >= 128) would flush before a sample,
 *     inside a low phase (docs 14). Longest low phase: 2 byte times (TDO with read, TMSLDH).
 *  2. JTAG layer: TAP reset, IR and DR shifts (docs 7), TCLK (5.1).
 *  3. MSP430 CPUXv2 layer: take control (9), lock (10), memory access and FRAM (11), mailbox erase (10),
 *     release (12).
 * State lives in ctx->state (no .data/.bss: the file is meant for an XIP image, xip_image.h).
 */
#include "prog_flash.h"
#include "prog_sbw.h"
#include <string.h>

/* ---- constants (docs 7.2, 7.3, 8) ---- */
#define IR_CNTRL_SIG_16BIT   0x13u
#define IR_CNTRL_SIG_CAPTURE 0x14u
#define IR_CNTRL_SIG_RELEASE 0x15u
#define IR_ADDR_16BIT        0x83u
#define IR_DATA_TO_ADDR      0x85u
#define IR_DATA_16BIT        0x41u
#define IR_DATA_CAPTURE      0x42u
#define IR_COREIP_ID         0x17u
#define IR_DEVICE_ID         0x87u
#define IR_JMB_EXCHANGE      0x61u

#define JID_XV1  0x89u
#define JID_FR4  0x98u            /* FR2xx, FR4xx */
#define JID_FR5  0x99u
#define JID_FR57 0x91u

#define EMU_MASK     0x0301u      /* "Full-Emulation-State" check (docs 9 step 7) */
#define WDTCTL_FR4   0x01CCu
#define WDTCTL_OTHER 0x015Cu
#define SYSCFG0      0x0160u      /* FR2xx/FR4xx: PFWP bit 0, DFWP bit 1, password 0xA5 in the high byte */
#define MPUCTL0      0x05A0u
#define FRWP_FR59    0x0140u      /* FR59xx/FR60xx write protect, bit 0 (docs 11, unclear item 10) */
#define BLK          16u          /* words per write/read call */
#define QMAX         96u          /* queued bytes at which a cycle first flushes (docs 14) */
#define CYCLE_BYTES  11u

typedef struct {
    uint8_t  jid, tclk, locked, ready, unprot;
    uint8_t  q, nrd;
    int8_t   err;                 /* sticky: a flush failed */
    uint8_t  init;                /* a multi-call operation is running */
    uint8_t  tdo[32];             /* TDO bits collected since the last shift started */
    uint32_t devptr, pos;
} sbw_state_t;

_Static_assert(sizeof(sbw_state_t) <= PROG_STATE_BYTES, "sbw state too big");

#define ST(ctx) ((sbw_state_t *)(void *)(ctx)->state)

/* ======================================================================== */
/* 1. Bit layer                                                              */
/* ======================================================================== */

static void sbw_flush(prog_ctx_t *c)
{
    const prog_wire_ops_t *w = c->w;
    sbw_state_t *st = ST(c);
    uint8_t s[32];
    int n = w->flush(w->io, s, sizeof s);
    if (n < 0) { st->err = PROG_E_PROTOCOL; n = 0; }
    for (int i = 0; i < n && st->nrd < sizeof st->tdo; i++) st->tdo[st->nrd++] = s[i] & 1u;
    st->q = 0;
}

/* One JTAG clock. tms/tdi: the levels in the TMS and TDI slots; dh: the TMS slot is TMSLDH (SBWTDIO
   back high before the rise, docs 5.1); rd: sample TDO in the TDO slot. SBWTCK is high on entry and exit. */
static void sbw_cycle(prog_ctx_t *c, int tms, int tdi, int dh, int rd)
{
    const prog_wire_ops_t *w = c->w;
    void *io = w->io;
    sbw_state_t *st = ST(c);
    if (st->q > QMAX) sbw_flush(c);
    w->set(io, 1, tms); w->set(io, 0, tms);
    if (dh) w->set(io, 0, 1);
    w->set(io, 1, dh ? 1 : tms);
    w->set(io, 1, tdi); w->set(io, 0, tdi); w->set(io, 1, tdi);
    w->data_drive(io, false);                    /* the chip drives while SBWTCK is low */
    w->set(io, 0, tdi);
    if (rd) w->sample(io);
    w->set(io, 1, tdi);
    w->data_drive(io, true);                     /* after the rise: the chip has released */
    st->q = (uint8_t)(st->q + CYCLE_BYTES);
}

/* Entry (docs 4). RST high: the normal entry; RST low: the chip does not run user code. */
static void sbw_entry(prog_ctx_t *c, bool rst_low)
{
    const prog_wire_ops_t *w = c->w;
    void *io = w->io;
    w->data_drive(io, true);
    if (!rst_low) {
        w->set(io, 0, 1); w->delay_us(io, 4000);
        w->set(io, 1, 1); w->delay_us(io, 20000);
        w->delay_us(io, 60);
    } else {
        w->set(io, 0, 0); w->delay_us(io, 51000);     /* TEST low 1 ms, then RST low 50 ms (RST stays low) */
        w->set(io, 1, 0); w->delay_us(io, 100000);
        w->set(io, 1, 1); w->delay_us(io, 40);
    }
    w->set(io, 0, 1);                            /* the first clock: one byte time low, data high */
    w->set(io, 1, 1);
    w->delay_us(io, rst_low ? 40 : 60);
    w->delay_us(io, 5000);
    ST(c)->tclk = 1;
}

/* Leave SBW: TEST low for more than 100 us (docs 3, 12); RST released, or held low. */
static void sbw_exit(prog_ctx_t *c, bool hold_rst)
{
    const prog_wire_ops_t *w = c->w;
    if (hold_rst) { w->set(w->io, 1, 0); w->set(w->io, 0, 0); }
    else          { w->set(w->io, 0, 1); }
    w->delay_us(w->io, 15000);
    if (!hold_rst) w->data_drive(w->io, false);
}

/* ======================================================================== */
/* 2. JTAG layer                                                             */
/* ======================================================================== */

/* IR (n = 8): LSB first, returns the JTAG ID (MSB first). DR: MSB first. 20 bit: returns the value as
   written (docs 7.1: the TDO order 15..0, 19..16 is rebuilt). */
static uint32_t jt_shift(prog_ctx_t *c, bool ir, unsigned n, uint32_t data)
{
    sbw_state_t *st = ST(c);
    int t = st->tclk;
    st->nrd = 0;
    sbw_cycle(c, 1, t, 0, 0);                    /* Run-Test/Idle -> Select-DR */
    if (ir) sbw_cycle(c, 1, 1, 0, 0);            /* -> Select-IR */
    sbw_cycle(c, 0, 1, 0, 0);                    /* -> Capture */
    sbw_cycle(c, 0, 1, 0, 0);                    /* -> Shift */
    for (unsigned i = 0; i < n; i++)
        sbw_cycle(c, i == n - 1u, (int)((data >> (ir ? i : n - 1u - i)) & 1u), 0, 1);
    sbw_cycle(c, 1, 1, 0, 0);                    /* Exit1 -> Update */
    sbw_cycle(c, 0, t, 0, 0);                    /* -> Run-Test/Idle */
    sbw_flush(c);
    if (st->nrd != n) { st->err = PROG_E_PROTOCOL; return 0; }
    uint32_t v = 0;
    for (unsigned i = 0; i < n; i++) v = (v << 1) | st->tdo[i];
    if (n == 20u) v = ((v << 16) + (v >> 4)) & 0xFFFFFu;
    return v;
}

static uint32_t ir(prog_ctx_t *c, unsigned ins)             { return jt_shift(c, true, 8, ins); }
static uint32_t dr16(prog_ctx_t *c, unsigned d)             { return jt_shift(c, false, 16, d); }
static uint32_t dr20(prog_ctx_t *c, uint32_t a)             { return jt_shift(c, false, 20, a); }

static void tclk(prog_ctx_t *c, int level)
{
    sbw_state_t *st = ST(c);
    sbw_cycle(c, 0, level, st->tclk, 0);         /* TMSLDH when TCLK is high (docs 5.1) */
    st->tclk = (uint8_t)level;
}

static void tap_reset(prog_ctx_t *c)
{
    for (int i = 0; i < 6; i++) sbw_cycle(c, 1, 1, 0, 0);
    sbw_cycle(c, 0, 1, 0, 0);
    ST(c)->tclk = 1;
}

/* ======================================================================== */
/* 3. MSP430 CPUXv2                                                          */
/* ======================================================================== */

static uint16_t rd_raw(prog_ctx_t *c, uint32_t a)
{
    tclk(c, 0);
    ir(c, IR_CNTRL_SIG_16BIT); dr16(c, 0x0501);
    ir(c, IR_ADDR_16BIT);      dr20(c, a);
    ir(c, IR_DATA_TO_ADDR);
    tclk(c, 1); tclk(c, 0);
    uint16_t v = (uint16_t)dr16(c, 0);
    tclk(c, 1); tclk(c, 0); tclk(c, 1);
    return v;
}

static void wr_raw(prog_ctx_t *c, uint32_t a, unsigned d)
{
    tclk(c, 0);
    ir(c, IR_CNTRL_SIG_16BIT); dr16(c, 0x0500);
    ir(c, IR_ADDR_16BIT);      dr20(c, a);
    tclk(c, 1);
    ir(c, IR_DATA_TO_ADDR);    dr16(c, d);
    tclk(c, 0);
    ir(c, IR_CNTRL_SIG_16BIT); dr16(c, 0x0501);
    tclk(c, 1); tclk(c, 0); tclk(c, 1);
}

static bool emu_ok(prog_ctx_t *c)
{
    ir(c, IR_CNTRL_SIG_CAPTURE);
    return (dr16(c, 0) & EMU_MASK) != 0;
}

static bool jid_valid(unsigned id) { return id == JID_FR4 || id == JID_FR5 || id == JID_FR57; }

/* Docs 9 steps 5 to 7: the FRAM power-on reset, watchdog hold, Full-Emulation-State check. */
static int por(prog_ctx_t *c)
{
    static const uint8_t pat[] = { 0, 1, 0, 1 };
    tclk(c, 0); tclk(c, 1);
    ir(c, IR_CNTRL_SIG_16BIT); dr16(c, 0x0C01); dr16(c, 0x0401);
    ir(c, IR_DATA_16BIT);
    for (unsigned i = 0; i < 4; i++) tclk(c, pat[i]);
    dr16(c, 0x0004);                             /* a safe PC */
    tclk(c, 0); tclk(c, 1);
    ir(c, IR_DATA_CAPTURE);
    for (unsigned i = 0; i < 4; i++) tclk(c, pat[i]);
    ir(c, IR_CNTRL_SIG_16BIT); dr16(c, 0x0501);
    tclk(c, 0); tclk(c, 1);
    unsigned id = ir(c, IR_CNTRL_SIG_CAPTURE);
    if (!emu_ok(c)) return PROG_E_PROTOCOL;
    wr_raw(c, id == JID_FR4 ? WDTCTL_FR4 : WDTCTL_OTHER, 0x5A80);
    if (id == JID_FR57 || id == JID_FR5) { wr_raw(c, 0x06, 0x3FFF); wr_raw(c, 0x08, 0x3FFF); }
    return emu_ok(c) ? PROG_OK : PROG_E_PROTOCOL;
}

static int sync_por(prog_ctx_t *c)
{
    ir(c, IR_CNTRL_SIG_16BIT); dr16(c, 0x1501);
    if (!jid_valid(ir(c, IR_CNTRL_SIG_CAPTURE))) return PROG_E_PROTOCOL;
    unsigned i = 0;
    while (!(dr16(c, 0) & 0x0200u) && ++i < 50u) { }
    if (i >= 50u) return PROG_E_TIMEOUT;
    return por(c);
}

/* Entry (up to 4 tries), TAP reset, JTAG ID, lock check, core and device ID, sync, POR. A locked chip is
   not an error: st->locked, id_len 1. */
static int bring_up(prog_ctx_t *c, bool rst_low)
{
    sbw_state_t *st = ST(c);
    unsigned id = 0, i;
    st->ready = st->locked = st->unprot = 0;
    for (i = 0; i < 4u; i++) {
        if (i) { c->w->set(c->w->io, 0, 1); c->w->data_drive(c->w->io, false); c->w->delay_us(c->w->io, 15000); }
        sbw_entry(c, i == 0 && rst_low);
        tap_reset(c);
        id = ir(c, IR_CNTRL_SIG_CAPTURE);
        if (st->err) return PROG_E_PROTOCOL;
        if (jid_valid(id) || id == JID_XV1) break;
    }
    if (i >= 4u) return PROG_E_NO_ANSWER;
    c->id[0] = (uint8_t)id; c->id_len = 1; st->jid = (uint8_t)id;
    if (id == JID_XV1) return PROG_E_UNSUPPORTED;     /* flash parts: docs 12 */

    for (i = 0; i < 3u; i++) {                         /* docs 10: BYPASS answers 0xAAAA as 0x5555 */
        ir(c, IR_CNTRL_SIG_CAPTURE);
        if (dr16(c, 0xAAAA) == 0x5555u) { st->locked = 1; return PROG_OK; }
    }
    ir(c, IR_COREIP_ID);
    if (dr16(c, 0) == 0) return PROG_E_PROTOCOL;
    ir(c, IR_DEVICE_ID);
    st->devptr = dr20(c, 0);
    if (st->err) return PROG_E_PROTOCOL;
    int rc = sync_por(c);
    if (rc) return rc;
    uint16_t dev = rd_raw(c, st->devptr + 4u);        /* docs 8: the device ID word is at pointer + 4 */
    c->id[1] = (uint8_t)dev; c->id[2] = (uint8_t)(dev >> 8); c->id_len = 3;
    st->ready = 1;
    return st->err ? PROG_E_PROTOCOL : PROG_OK;
}

/* Docs 11: writes to FRAM are blocked until the protection is off (and are again after a BOR). */
static int unprotect(prog_ctx_t *c)
{
    sbw_state_t *st = ST(c);
    if (st->unprot) return PROG_OK;
    if (st->jid == JID_FR4) {
        wr_raw(c, SYSCFG0, (rd_raw(c, SYSCFG0) & ~0xFF03u) | 0xA500u);
        if (rd_raw(c, SYSCFG0) & 3u) return PROG_E_PROTOCOL;
    } else {
        uint16_t m = rd_raw(c, MPUCTL0);
        if (m & 1u) {
            if ((m & 3u) != 1u) return PROG_E_UNSUPPORTED;      /* MPU locked: needs a BOR */
            wr_raw(c, MPUCTL0, 0xA500);
            if (rd_raw(c, MPUCTL0) & 1u) return PROG_E_PROTOCOL;
        }
        if (st->jid == JID_FR5) {
            uint16_t p = rd_raw(c, FRWP_FR59);
            if (p & 1u) {
                wr_raw(c, FRWP_FR59, (p & ~0xFF01u) | 0xA500u);
                if (rd_raw(c, FRWP_FR59) & 1u) return PROG_E_PROTOCOL;
            }
        }
    }
    st->unprot = 1;
    return PROG_OK;
}

/* ---- protocol entry points ---- */

static int sbw_enter(prog_ctx_t *c)
{
    memset(c->state, 0, sizeof(sbw_state_t));
    c->phase = "enter";
    /* prog_wire: close leaves TEST low; no automatic flush inside a clock-low phase (docs 17). */
    if (c->w->mode) c->w->mode(c->w->io, PROG_WIRE_CLK_LOW_AT_CLOSE | PROG_WIRE_STRICT_LOW);
    return bring_up(c, (c->flags & SBW_FLAG_RST_LOW) != 0);
}

static int range_ok(const prog_ctx_t *c)
{
    if (c->mem != PROG_MEM_FLASH && c->mem != PROG_MEM_CONFIG) return PROG_E_ARGS;
    if (c->len == 0 || ((c->addr | c->len) & 1u) || c->addr > 0x100000u || c->len > 0x100000u - c->addr)
        return PROG_E_ARGS;
    return PROG_OK;
}

/* mode 0 write (verified), 1 erase range to 0xFFFF (verified), 2 read. One block of BLK words per call. */
static int xfer(prog_ctx_t *c, int mode)
{
    sbw_state_t *st = ST(c);
    int rc = range_ok(c);
    if (rc) return rc;
    if (!st->init) { st->init = 1; st->pos = 0; c->done = 0; c->total = (c->len / 2u + BLK - 1u) / BLK; }
    c->phase = mode == 2 ? "read" : mode == 1 ? "erase" : "write";
    if (st->locked) rc = PROG_E_LOCKED;
    else if (!st->ready) rc = PROG_E_PROTOCOL;
    else if (!emu_ok(c)) rc = PROG_E_PROTOCOL;
    else if (mode != 2) rc = unprotect(c);
    if (rc) { st->init = 0; return rc; }

    uint32_t off = st->pos, n = (c->len - off) / 2u, a = c->addr + off;
    if (n > BLK) n = BLK;
    uint8_t buf[2u * BLK];
    if (mode == 0 && c->src(c->job, off, buf, 2u * n) != 0) { st->init = 0; return PROG_E_SRC; }
    if (mode == 1) memset(buf, 0xFF, sizeof buf);
    for (uint32_t i = 0; i < n; i++) {
        if (mode == 2) {
            uint16_t v = rd_raw(c, a + 2u * i);
            buf[2u * i] = (uint8_t)v; buf[2u * i + 1u] = (uint8_t)(v >> 8);
        } else {
            wr_raw(c, a + 2u * i, (unsigned)buf[2u * i] | ((unsigned)buf[2u * i + 1u] << 8));
        }
    }
    if (mode != 2) {
        c->phase = "verify";
        for (uint32_t i = 0; i < n && !st->err; i++) {
            uint16_t v = rd_raw(c, a + 2u * i);
            if (v != ((unsigned)buf[2u * i] | ((unsigned)buf[2u * i + 1u] << 8))) {
                c->at_addr = a + 2u * i; st->init = 0; return PROG_E_VERIFY;
            }
        }
    } else if (!st->err && c->dst(c->job, off, buf, 2u * n) != 0) { st->init = 0; return PROG_E_SRC; }
    if (st->err) { st->init = 0; return PROG_E_PROTOCOL; }
    st->pos = off + 2u * n;
    c->done++;
    if (st->pos >= c->len) { st->init = 0; return PROG_OK; }
    return PROG_RUNNING;
}

static int jmb_write32(prog_ctx_t *c, unsigned a, unsigned b)
{
    ir(c, IR_JMB_EXCHANGE);
    unsigned i;
    for (i = 0; i < 3000u; i++) if (dr16(c, 0) & 1u) break;     /* IN0RDY */
    if (i >= 3000u) return PROG_E_TIMEOUT;
    dr16(c, 0x0011); dr16(c, a); dr16(c, b);                    /* 32-bit mode + INREQ; 0xA55A; 0x1A1A */
    return PROG_OK;
}

/* Docs 10: restart in reset, feed the boot code "stop" and "user code erase", restart, sync. */
static int erase_chip(prog_ctx_t *c)
{
    const prog_wire_ops_t *w = c->w;
    sbw_state_t *st = ST(c);
    c->phase = "erase";
    c->total = 1; c->done = 0;
    if (st->jid == 0) return PROG_E_PROTOCOL;
    w->data_drive(w->io, true);
    w->set(w->io, 0, 1); w->set(w->io, 0, 0);                  /* TEST low, then RST low: never both in one command */
    w->delay_us(w->io, 200000);
    sbw_entry(c, true);
    tap_reset(c);
    int rc = jmb_write32(c, 0xA55A, 0x1A1A);
    if (rc) return rc;
    w->set(w->io, 0, 1); w->delay_us(w->io, 260000);          /* 200 ms restart + 60 ms erase (docs 10) */
    rc = bring_up(c, false);
    if (rc) return rc;
    if (st->locked) return PROG_E_LOCKED;
    c->done = 1;
    return PROG_OK;
}

static int sbw_erase(prog_ctx_t *c)
{
    if (c->erase == PROG_ERASE_NONE) return PROG_OK;
    if (c->erase == PROG_ERASE_CHIP) return erase_chip(c);
    return xfer(c, 1);
}

static int sbw_write(prog_ctx_t *c) { return xfer(c, 0); }
static int sbw_read(prog_ctx_t *c)  { return xfer(c, 2); }

static void sbw_leave(prog_ctx_t *c, bool run)
{
    sbw_state_t *st = ST(c);
    uint8_t tmp;
    c->phase = "leave";
    if (st->ready) {                                           /* docs 12 */
        ir(c, IR_CNTRL_SIG_16BIT); dr16(c, 0x0C01); dr16(c, 0x0401);
        if (run) ir(c, IR_CNTRL_SIG_RELEASE);
    }
    sbw_exit(c, !run && st->ready);
    c->w->flush(c->w->io, &tmp, 0);
}

PROG_PROTO(sbw, sbw_enter, sbw_erase, sbw_write, sbw_read, sbw_leave, PROG_INFO_SBW)
