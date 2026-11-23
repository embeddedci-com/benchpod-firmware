/* prog_pic.c — PIC ICSP low-voltage entry (PIC16F1xxx, PIC16F18xxx, PIC18F Q) for prog_flash (prog_flash.h).
 * Conventions, flags and addressing: prog_pic.h. */
#include "prog_flash.h"
#include "prog_pic.h"

#define PIC_KEY   0x4D434850u   /* "MCHP": 8-bit command sets send it MSb first (32 clocks), the 6-bit set LSb first (33) */
#define HOLD      3             /* feed bytes per clock phase: >= 120 ns, spec minimum is 100 ns */
#define T_ENTS_US 1000          /* MCLR low before the key */
#define T_ENTH_US 500           /* key to first command (spec 250 us; PIC18 Q: 1 ms) */
#define T_ENTHQ_US 1200         /* TENTH of PIC18 Q */
#define T_PROG_US 6000          /* flash row (spec 2.8 ms) */
#define T_PROGN_US 12000        /* config, user ID, EEPROM word */
#define T_PINT_US 100           /* PIC18 Q Program Data (TPINT: 50 us Q43, 75 us Q84, 65 us Q10) */
#define T_PDFM_US 12000         /* PIC18 Q EEPROM and Q43/Q84 config bytes (TPDFM 11 ms) */
#define T_ERASE_US 30000        /* bulk and row erase (TERAB 5 to 11 ms, Q10 75 ms) */
#define T_ERASEQ10_US 80000     /* PIC18 Q10 bulk erase (TERAB 75 ms) */
#define BLOCK     64u           /* read bytes per call */

typedef struct {
    int32_t  pc;      /* current PC in PC units, -1 unknown */
    uint32_t idx;     /* unit (row, word, byte block) in progress */
    uint8_t  stage;   /* 0 idle, 1 running (write: programming pass), 2 write: verify pass */
    uint8_t  locked;
} pic_st_t;
_Static_assert(sizeof(pic_st_t) <= PROG_STATE_BYTES, "pic state");

#define ST(c) ((pic_st_t *)(c)->state)
#define TRY(x) do { int r_ = (x); if (r_) return r_; } while (0)

static bool s8(const prog_ctx_t *c)  { return c->flags & PIC_F_SET8; }
static bool byt(const prog_ctx_t *c) { return s8(c) && (c->flags & PIC_F_BYTEPC); }
static bool q10(const prog_ctx_t *c) { return byt(c) && (c->flags & PIC_F_Q10); }
/* one image byte per location: EEPROM, and the config bytes of PIC18 Q43/Q84 (user ID and Q10 config are 16-bit words) */
static bool nar(const prog_ctx_t *c) { return c->mem == PROG_MEM_EEPROM || (byt(c) && !q10(c) && c->mem == PROG_MEM_CONFIG); }
static uint32_t emask(const prog_ctx_t *c) { return nar(c) ? 0xFFu : byt(c) ? 0xFFFFu : 0x3FFFu; }
static uint32_t pcstep(const prog_ctx_t *c) { return byt(c) && !nar(c) ? 2u : 1u; }
static uint32_t pcof(const prog_ctx_t *c, uint32_t a) { return nar(c) || byt(c) ? a : a / 2u; }

static void wbit(const prog_wire_ops_t *w, int d) {
    for (int i = 0; i < HOLD; i++) w->set(w->io, 1, d);
    for (int i = 0; i < HOLD; i++) w->set(w->io, 0, d);
}
static void rbit(const prog_wire_ops_t *w) {
    for (int i = 0; i < HOLD; i++) w->set(w->io, 1, 1);
    w->sample(w->io);                       /* clock high: the chip's bit is valid (tCO 80 ns) */
    for (int i = 0; i < HOLD; i++) w->set(w->io, 0, 1);
}
static void sendv(const prog_wire_ops_t *w, uint32_t v, int n, bool msb) {
    for (int i = 0; i < n; i++) wbit(w, (int)((v >> (msb ? n - 1 - i : i)) & 1u));
}

static int fl(prog_ctx_t *c) {
    uint8_t t[1];
    return c->w->flush(c->w->io, t, sizeof t) < 0 ? PROG_E_PROTOCOL : 0;
}
static int wait(prog_ctx_t *c, uint32_t us) {
    TRY(fl(c));
    c->w->delay_us(c->w->io, us);
    return 0;
}

/* One command; mode 0: none, 1: payload out, 2: payload in (*out gets the data). */
static int xfer(prog_ctx_t *c, uint8_t cmd, int mode, uint32_t d, uint32_t *out) {
    const prog_wire_ops_t *w = c->w;
    bool b = s8(c);
    sendv(w, cmd, b ? 8 : 6, b);
    w->delay_us(w->io, 1);                  /* TDLY */
    if (mode == 0) return 0;
    int n = b ? 24 : 16;
    if (mode == 1) {
        sendv(w, d << 1, n, b);             /* start bit (set A) / stop bit, data in between */
        w->delay_us(w->io, 1);
        return 0;
    }
    uint8_t s[24];
    w->data_drive(w->io, false);
    for (int i = 0; i < n; i++) rbit(w);
    int r = w->flush(w->io, s, sizeof s);
    w->data_drive(w->io, true);
    if (r != n) return PROG_E_PROTOCOL;
    uint32_t v = 0;
    for (int i = 0; i < n; i++) v |= (uint32_t)(s[i] & 1u) << (b ? n - 1 - i : i);
    *out = v >> 1;
    w->delay_us(w->io, 1);
    return 0;
}

/* Set A commands */
#define A_LDCFG 0x00
#define A_LDPGM 0x02
#define A_LDDAT 0x03
#define A_RDPGM 0x04
#define A_RDDAT 0x05
#define A_INC   0x06
#define A_BEGIN 0x08
#define A_BULKP 0x09
#define A_BULKD 0x0B
#define A_ROWER 0x11
#define A_RESET 0x16
/* Set B commands */
#define B_LDPC  0x80
#define B_LD    0x00
#define B_LDINC 0x02
#define B_RD    0xFC
#define B_RDINC 0xFE
#define B_INC   0xF8
#define B_BEGIN 0xE0
#define B_PROGI 0xE0   /* PIC18 Q: Program Data, 24-bit payload, PC += n after */
#define B_BULK  0x18
#define B_ROWER 0xF0

static int seek(prog_ctx_t *c, uint32_t pc) {
    pic_st_t *s = ST(c);
    if (s->pc == (int32_t)pc) return 0;
    if (s8(c)) {
        TRY(xfer(c, B_LDPC, 1, pc, 0));
        s->pc = (int32_t)pc;
        return 0;
    }
    if (s->pc < 0 || (int32_t)pc < s->pc) { TRY(xfer(c, A_RESET, 0, 0, 0)); s->pc = 0; }
    if (pc >= 0x8000u && s->pc < 0x8000) { TRY(xfer(c, A_LDCFG, 1, 0, 0)); s->pc = 0x8000; }
    while (s->pc < (int32_t)pc) { TRY(xfer(c, A_INC, 0, 0, 0)); s->pc++; }
    return 0;
}

/* Read the element at PC unit `pc` (mask and width from c->mem). */
static int rdel(prog_ctx_t *c, uint32_t pc, uint32_t *v) {
    pic_st_t *s = ST(c);
    TRY(seek(c, pc));
    if (s8(c)) { TRY(xfer(c, B_RDINC, 2, 0, v)); s->pc += (int32_t)pcstep(c); }
    else       { TRY(xfer(c, nar(c) ? A_RDDAT : A_RDPGM, 2, 0, v)); }
    *v &= emask(c);
    return 0;
}

/* Source word at image offset `off`, padded with 0xFF past the end. */
static int getel(prog_ctx_t *c, uint32_t off, uint32_t *v) {
    uint8_t b[2] = { 0xFF, 0xFF };
    uint32_t es = nar(c) ? 1u : 2u;
    if (off < c->len) {
        uint32_t n = c->len - off < es ? c->len - off : es;
        if (c->src(c->job, off, b, n)) return PROG_E_SRC;
    }
    *v = ((uint32_t)b[1] << 8 | b[0]) & emask(c);
    if (nar(c)) *v = b[0];
    return 0;
}

static int args(prog_ctx_t *c, uint32_t *ub, bool rows) {
    uint32_t es = nar(c) ? 1u : 2u;
    if (c->mem >= PROG_MEM_COUNT || !c->len || c->addr % es) return PROG_E_ARGS;
    *ub = es;
    if (c->mem == PROG_MEM_FLASH && rows) {
        if (c->page < 2 || (c->page & 1) || c->page > 512 || c->addr % c->page) return PROG_E_ARGS;
        *ub = c->page;
    }
    return 0;
}

static int locked_chk(prog_ctx_t *c) {
    return ST(c)->locked && (c->mem == PROG_MEM_FLASH || c->mem == PROG_MEM_EEPROM) ? PROG_E_LOCKED : 0;
}

static int pic_enter(prog_ctx_t *c) {
    const prog_wire_ops_t *w = c->w;
    pic_st_t *s = ST(c);
    uint8_t m = c->mem;
    s->pc = -1; s->idx = 0; s->stage = 0; s->locked = 0;
    c->phase = "enter";
    w->data_drive(w->io, true);
    w->set(w->io, 0, 0);
    TRY(fl(c));                             /* clock and data low before MCLR falls (TENTS) */
    w->nrst(w->io, true);
    TRY(wait(c, T_ENTS_US));
    bool lsb = !s8(c) != !!(c->flags & PIC_F_KEY_FLIP);
    sendv(w, PIC_KEY, 32, !lsb);            /* 6-bit set: LSb first and a 33rd clock (DS41439 fig. 8-8) */
    if (lsb) wbit(w, 0);
    w->set(w->io, 0, 0);
    TRY(wait(c, byt(c) ? T_ENTHQ_US : T_ENTH_US));
    c->phase = "id";
    c->mem = PROG_MEM_FLASH;                /* the ID words are full-width words */
    uint32_t rev, dev, raw, rpc = byt(c) ? 0x3FFFFCu : 0x8005u;
    bool one = !s8(c) && !(c->flags & PIC_F_ID_SPLIT);
    int r;
    if (one) {                              /* DEV<8:0> in bits 13:5 and REV<4:0> of the word at 0x8006 */
        r = rdel(c, 0x8006u, &raw);
        dev = raw >> 5; rev = raw & 31u;
    } else {
        r = rdel(c, rpc, &dev);             /* revision first, then device ID (next word) */
        rev = dev;
        if (!r) r = rdel(c, rpc + (byt(c) ? 2u : 1u), &dev);
        raw = dev;
    }
    c->mem = m;
    TRY(r);
    if (raw == 0 || raw == (byt(c) ? 0xFFFFu : 0x3FFFu)) return PROG_E_NO_ANSWER;
    c->id[0] = (uint8_t)dev; c->id[1] = (uint8_t)(dev >> 8);
    c->id[2] = (uint8_t)rev; c->id[3] = (uint8_t)(rev >> 8);
    c->id_len = 4;
    if (c->baud) {                          /* the code-protect bit */
        uint32_t cw;
        c->mem = PROG_MEM_CONFIG;
        r = rdel(c, pcof(c, c->baud), &cw);
        c->mem = m;
        TRY(r);
        uint32_t bit = (cw >> ((c->flags >> 8) & 15u)) & 1u;
        s->locked = (c->flags & PIC_F_CP_HIGH) ? bit : !bit;
    }
    return fl(c);
}

/* Errors and the last call end a multi-call step: the next call starts afresh. */
static int fin(prog_ctx_t *c, int r) {
    if (r != PROG_RUNNING) ST(c)->stage = 0;
    return r;
}

static int er(prog_ctx_t *c) {
    pic_st_t *s = ST(c);
    uint32_t ub;
    if (c->erase == PROG_ERASE_NONE) return PROG_OK;
    c->phase = "erase";
    if (c->erase == PROG_ERASE_CHIP) {      /* from config space: also clears code protection */
        if (q10(c)) {                       /* no payload: the PC picks the regions (Table 3-2) */
            TRY(seek(c, 0x300000u));        /* flash, user ID, config */
            TRY(xfer(c, B_BULK, 0, 0, 0));
            TRY(wait(c, T_ERASEQ10_US));
            TRY(seek(c, 0x310000u));        /* data EEPROM only */
            TRY(xfer(c, B_BULK, 0, 0, 0));
            TRY(wait(c, T_ERASEQ10_US));
        } else if (byt(c)) {                /* Q43/Q84: payload bits 0..3 = EEPROM, flash, user ID, config */
            TRY(xfer(c, B_BULK, 1, 0x0Fu, 0));
            TRY(wait(c, T_ERASE_US));
        } else if (s8(c)) {
            TRY(seek(c, 0x8000u));
            TRY(xfer(c, B_BULK, 0, 0, 0));
            TRY(wait(c, T_ERASE_US));
        } else {
            TRY(seek(c, 0x8000u));
            TRY(xfer(c, A_BULKP, 0, 0, 0));
            TRY(wait(c, T_ERASE_US));
            if (c->flags & PIC_F_HAS_EE) TRY(xfer(c, A_BULKD, 0, 0, 0));
            TRY(wait(c, T_ERASE_US));
        }
        s->pc = -1; s->locked = 0;
        c->done = c->total = 1;
        return PROG_OK;
    }
    if (c->mem != PROG_MEM_FLASH) return PROG_OK;   /* words are overwritten; chip erase clears config */
    TRY(args(c, &ub, true));
    TRY(locked_chk(c));
    if (!s->stage) { s->stage = 1; s->idx = 0; c->done = 0; c->total = (c->len + ub - 1) / ub; }
    TRY(seek(c, pcof(c, c->addr + s->idx * ub)));
    TRY(xfer(c, s8(c) ? B_ROWER : A_ROWER, 0, 0, 0));
    TRY(wait(c, T_ERASE_US));
    c->done = ++s->idx;
    return s->idx == c->total ? PROG_OK : PROG_RUNNING;
}

static int wr(prog_ctx_t *c) {
    pic_st_t *s = ST(c);
    uint32_t ub, es = nar(c) ? 1u : 2u;
    TRY(args(c, &ub, true));
    uint32_t units = (c->len + ub - 1) / ub, nel = ub / es, st = pcstep(c);
    if (!s->stage) {
        TRY(locked_chk(c));
        s->stage = 1; s->idx = 0; c->done = 0; c->total = units; c->phase = "write";
    }
    uint32_t off = s->idx * ub, start = pcof(c, c->addr + off), v, x;
    if (s->stage == 1) {
        TRY(seek(c, start));
        for (uint32_t i = 0; i < nel; i++) {
            bool more = i + 1 < nel;
            if (byt(c)) {                   /* PIC18 Q: one Program Data per word, the chip times it */
                if (off + i * es >= c->len) break;
                TRY(getel(c, off + i * es, &v));
                TRY(xfer(c, B_PROGI, 1, v, 0));
                s->pc += (int32_t)st;
                TRY(wait(c, nar(c) ? T_PDFM_US : T_PINT_US));
                continue;
            }
            TRY(getel(c, off + i * es, &v));
            if (s8(c)) {
                TRY(xfer(c, more ? B_LDINC : B_LD, 1, v, 0));
            } else {
                TRY(xfer(c, nar(c) ? A_LDDAT : A_LDPGM, 1, v, 0));
                if (more) { TRY(xfer(c, A_INC, 0, 0, 0)); s->pc++; }
            }
        }
        if (!byt(c)) {
            if (s8(c)) s->pc = (int32_t)(start + (nel - 1) * st);
            TRY(xfer(c, s8(c) ? B_BEGIN : A_BEGIN, 0, 0, 0));
            TRY(wait(c, c->mem == PROG_MEM_FLASH ? T_PROG_US : T_PROGN_US));
        }
        c->done = ++s->idx;
        if (s->idx == units) { s->stage = 2; s->idx = 0; c->done = 0; c->phase = "verify"; }
        return PROG_RUNNING;
    }
    for (uint32_t i = 0; i < nel && off + i * es < c->len; i++) {
        uint32_t o = off + i * es, nb = c->len - o < es ? c->len - o : es;
        TRY(rdel(c, start + i * st, &x));
        TRY(getel(c, o, &v));
        if ((x ^ v) & (nb == 1 ? 0xFFu : emask(c))) { c->at_addr = c->addr + o; return PROG_E_VERIFY; }
    }
    c->done = ++s->idx;
    return s->idx == units ? PROG_OK : PROG_RUNNING;
}

static int rdb(prog_ctx_t *c) {
    pic_st_t *s = ST(c);
    uint32_t ub, es = nar(c) ? 1u : 2u, v;
    uint8_t buf[BLOCK];
    TRY(args(c, &ub, false));
    if (!s->stage) {
        TRY(locked_chk(c));
        s->stage = 1; s->idx = 0; c->done = 0; c->total = (c->len + BLOCK - 1) / BLOCK; c->phase = "read";
    }
    uint32_t off = s->idx * BLOCK, n = c->len - off < BLOCK ? c->len - off : BLOCK;
    for (uint32_t k = 0; k < n; k += es) {
        TRY(rdel(c, pcof(c, c->addr + off + k), &v));
        buf[k] = (uint8_t)v;
        if (es == 2 && k + 1 < n) buf[k + 1] = (uint8_t)(v >> 8);
    }
    if (c->dst(c->job, off, buf, n)) return PROG_E_SRC;
    c->done = ++s->idx;
    return s->idx == c->total ? PROG_OK : PROG_RUNNING;
}

static int pic_erase(prog_ctx_t *c) { return fin(c, er(c)); }
static int pic_write(prog_ctx_t *c) { return fin(c, wr(c)); }
static int pic_read(prog_ctx_t *c)  { return fin(c, rdb(c)); }

static void pic_leave(prog_ctx_t *c, bool run) {
    const prog_wire_ops_t *w = c->w;
    c->phase = "leave";
    w->data_drive(w->io, true);
    w->set(w->io, 0, 0);
    (void)fl(c);
    w->data_drive(w->io, false);
    if (run) {
        (void)fl(c);
        w->delay_us(w->io, 10);
        w->nrst(w->io, false);
    }
}

PROG_PROTO(pic, pic_enter, pic_erase, pic_write, pic_read, pic_leave, PROG_INFO_PIC)
