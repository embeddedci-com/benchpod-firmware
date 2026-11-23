/* prog_pic.c — PIC ICSP low-voltage entry (PIC16F1xxx, PIC16F18xxx, PIC18F Q) for prog_flash (prog_flash.h).
 * Conventions, flags and addressing: prog_pic.h. */
#include "prog_flash.h"
#include "prog_pic.h"

#define PIC_KEY   0x4D434850u   /* "MCHP", MSB first */
#define HOLD      3             /* feed bytes per clock phase: >= 120 ns, spec minimum is 100 ns */
#define T_ENTS_US 1000          /* MCLR low before the key */
#define T_ENTH_US 500           /* key to first command (spec 250 us) */
#define T_PROG_US 6000          /* flash row (spec 2.8 ms) */
#define T_PROGN_US 12000        /* config, user ID, EEPROM word */
#define T_ERASE_US 30000        /* bulk and row erase */
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
/* one image byte per location: EEPROM, and config / user ID on PIC18 */
static bool nar(const prog_ctx_t *c) { return c->mem == PROG_MEM_EEPROM || (byt(c) && c->mem != PROG_MEM_FLASH); }
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

static int args(prog_ctx_t *c, uint32_t *ub) {
    uint32_t es = nar(c) ? 1u : 2u;
    if (c->mem >= PROG_MEM_COUNT || !c->len || c->addr % es) return PROG_E_ARGS;
    *ub = es;
    if (c->mem == PROG_MEM_FLASH) {
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
    w->nrst(w->io, true);
    TRY(wait(c, T_ENTS_US));
    sendv(w, PIC_KEY, 32, true);
    w->set(w->io, 0, 0);
    TRY(wait(c, T_ENTH_US));
    c->phase = "id";
    c->mem = PROG_MEM_FLASH;                /* the ID words are full-width words */
    uint32_t rev, dev, rpc = byt(c) ? 0x3FFFFCu : 0x8005u;
    int r = rdel(c, rpc, &dev);             /* revision first, then device ID (next word) */
    rev = dev;
    if (!r) r = rdel(c, rpc + (byt(c) ? 2u : 1u), &dev);
    c->mem = m;
    TRY(r);
    if (dev == 0 || dev == (byt(c) ? 0xFFFFu : 0x3FFFu)) return PROG_E_NO_ANSWER;
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

static int pic_erase(prog_ctx_t *c) { (void)c; return PROG_E_UNSUPPORTED; }
static int pic_write(prog_ctx_t *c) { (void)c; return PROG_E_UNSUPPORTED; }
static int pic_read(prog_ctx_t *c)  { (void)c; return PROG_E_UNSUPPORTED; }

static void pic_leave(prog_ctx_t *c, bool run) {
    const prog_wire_ops_t *w = c->w;
    c->phase = "leave";
    w->data_drive(w->io, true);
    w->set(w->io, 0, 0);
    (void)fl(c);
    w->data_drive(w->io, false);
    if (run) {
        w->delay_us(w->io, 10);
        w->nrst(w->io, false);
    }
}

const prog_proto_t prog_proto_pic = {
    .name = "pic", .uart = false, .uses_nrst = true,
    .mems = (1u<<PROG_MEM_FLASH)|(1u<<PROG_MEM_EEPROM)|(1u<<PROG_MEM_CONFIG)|(1u<<PROG_MEM_USER),
    .enter = pic_enter, .erase = pic_erase, .write = pic_write, .read = pic_read, .leave = pic_leave,
};
