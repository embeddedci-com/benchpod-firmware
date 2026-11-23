/* prog_tpi.c — AVR TPI (ATtiny4/5/9/10/20/40/102/104) for prog_flash (prog_flash.h).
 *
 * TPICLK = clk, TPIDATA = data, the chip's RESET = the pod's reset pin (held low for the whole
 * session). 12-bit frames: prog_avr12.h. The pod knows no device tables: the host passes the
 * erase/write page size in ctx->page (bytes, even), addr is relative to the memory's base.
 * Flash is written one word (2 bytes, low first) at a time; every word is followed by an NVMBSY poll.
 */
#include "prog_flash.h"
#include "prog_avr12.h"
#include <string.h>

enum { OP_SLD = 0x20, OP_SLD_INC = 0x24, OP_SST = 0x60, OP_SST_INC = 0x64, OP_SSTPR = 0x68,
       OP_SLDCS = 0x80, OP_SSTCS = 0xC0, OP_SKEY = 0xE0 };
enum { REG_TPISR = 0x00, REG_TPIPCR = 0x02 };
#define TPISR_NVMEN  0x02u
#define TPIPCR_GT2   0x06u        /* guard time: 2 idle bits */
enum { NVM_NOOP = 0x00, NVM_CHIP_ERASE = 0x10, NVM_SECTION_ERASE = 0x14, NVM_WORD_WRITE = 0x1D };
#define NVMCSR       0x0032u
#define NVMCMD       0x0033u
#define NVMBSY       0x80u
#define ADDR_LOCK    0x3F00u
#define ADDR_CONFIG  0x3F40u
#define ADDR_SIG     0x3FC0u
#define ADDR_FLASH   0x4000u
#define MAX_PAGE     128u
#define BUSY_MS      100u

static const uint8_t k_nvm_key[8] = { 0xFF, 0x88, 0xD8, 0xCD, 0x45, 0xAB, 0x89, 0x12 };

typedef struct {
    uint32_t addr, len, pos;      /* the region in progress */
    uint8_t  mem, kind, active;   /* kind: 1 write, 2 read */
    uint8_t  locked;              /* the lock bits say the chip refuses reads and writes */
} tpi_st_t;
_Static_assert(sizeof(tpi_st_t) <= PROG_STATE_BYTES, "tpi state");
#define ST(c) ((tpi_st_t *)(void *)(c)->state)

static void sstpr(const prog_wire_ops_t *w, uint16_t a) {
    avr12_tx(w, OP_SSTPR);     avr12_tx(w, (uint8_t)a);
    avr12_tx(w, OP_SSTPR | 1); avr12_tx(w, (uint8_t)(a >> 8));
}

static void sst(const prog_wire_ops_t *w, uint16_t a, uint8_t v) {
    sstpr(w, a);
    avr12_tx(w, OP_SST);
    avr12_tx(w, v);
}

/* n bytes from data space address a. */
static int ld(const prog_wire_ops_t *w, uint16_t a, uint8_t *buf, unsigned n) {
    sstpr(w, a);
    for (unsigned i = 0; i < n; i++) {
        avr12_tx(w, OP_SLD_INC);
        int rc = avr12_rx(w, &buf[i]);
        if (rc) return rc;
    }
    return 0;
}

/* Wait for NVMBSY to clear. */
static int nvm_wait(const prog_wire_ops_t *w) {
    uint32_t t0 = w->now_ms(w->io);
    for (;;) {
        uint8_t v;
        int rc = ld(w, NVMCSR, &v, 1);
        if (rc) return rc;
        if (!(v & NVMBSY)) return 0;
        if (w->now_ms(w->io) - t0 > BUSY_MS) return PROG_E_TIMEOUT;
        w->delay_us(w->io, 200);
    }
}

/* Run an erase command: the command, then a dummy write to the high byte of an address in range. */
static int nvm_erase(const prog_wire_ops_t *w, uint8_t cmd, uint16_t a) {
    sst(w, NVMCMD, cmd);
    sst(w, a | 1u, 0xFF);
    int rc = avr12_commit(w);
    return rc ? rc : nvm_wait(w);
}

static int read_lock(prog_ctx_t *c) {
    uint8_t v;
    int rc = ld(c->w, ADDR_LOCK, &v, 1);
    if (rc) return rc;
    ST(c)->locked = (v & 3u) != 3u;
    return 0;
}

static int tpi_enter(prog_ctx_t *c) {
    const prog_wire_ops_t *w = c->w;
    memset(c->state, 0, sizeof(tpi_st_t));
    c->phase = "enter"; c->done = 0; c->total = 0;
    w->nrst(w->io, true);
    w->delay_us(w->io, 1000);
    avr12_idle(w, 20);
    avr12_tx(w, OP_SSTCS | REG_TPIPCR); avr12_tx(w, TPIPCR_GT2);
    avr12_tx(w, OP_SKEY);
    for (unsigned i = 0; i < 8; i++) avr12_tx(w, k_nvm_key[i]);
    avr12_tx(w, OP_SLDCS | REG_TPISR);
    uint8_t sr;
    int rc = avr12_rx(w, &sr);
    if (rc) return rc;
    if (!(sr & TPISR_NVMEN)) return PROG_E_PROTOCOL;   /* the chip did not take the key */
    c->phase = "id";
    c->id_len = 3;
    rc = ld(w, ADDR_SIG, c->id, 3);
    if (rc) return rc;
    return read_lock(c);
}

static int tpi_erase(prog_ctx_t *c) {
    if (c->erase != PROG_ERASE_CHIP) return PROG_OK;
    c->phase = "erase";
    int rc = nvm_erase(c->w, NVM_CHIP_ERASE, ADDR_FLASH);
    if (rc) return rc;
    rc = read_lock(c);
    if (rc) return rc;
    return ST(c)->locked ? PROG_E_LOCKED : PROG_OK;
}

/* Common checks and the next chunk. Returns <0 error, 0 nothing left (finished), 1 chunk set. */
static int next_chunk(prog_ctx_t *c, uint8_t kind, uint16_t *base, uint32_t *n) {
    tpi_st_t *st = ST(c);
    bool cfg = c->mem == PROG_MEM_CONFIG;
    if (c->mem != PROG_MEM_FLASH && !cfg) return PROG_E_ARGS;
    if ((c->addr & 1u) || c->page == 0 || (c->page & 1u) || c->page > MAX_PAGE) return PROG_E_ARGS;
    if (cfg ? (c->addr + c->len > 2u) : (c->addr + c->len > 0x10000u - ADDR_FLASH)) return PROG_E_ARGS;
    if (!st->active || st->mem != c->mem || st->kind != kind || st->addr != c->addr || st->len != c->len) {
        st->active = 1; st->mem = c->mem; st->kind = kind; st->addr = c->addr; st->len = c->len; st->pos = 0;
    }
    c->total = (c->len + c->page - 1u) / c->page;
    if (st->pos >= c->len) { st->active = 0; return 0; }
    if (st->locked) return PROG_E_LOCKED;
    uint32_t a = c->addr + st->pos;
    uint32_t k = c->page - a % c->page;
    if (k > c->len - st->pos) k = c->len - st->pos;
    *n = k;
    *base = (uint16_t)((cfg ? ADDR_CONFIG : ADDR_FLASH) + a);
    return 1;
}

static int tpi_write(prog_ctx_t *c) {
    tpi_st_t *st = ST(c);
    const prog_wire_ops_t *w = c->w;
    uint16_t base; uint32_t n;
    int rc = next_chunk(c, 1, &base, &n);
    if (rc <= 0) return rc < 0 ? rc : PROG_OK;
    uint8_t buf[MAX_PAGE + 1], chk[MAX_PAGE];
    c->phase = "write";
    if (c->src(c->job, st->pos, buf, n) != 0) return PROG_E_SRC;
    uint32_t nw = (n + 1u) & ~1u;
    if (nw > n) buf[n] = 0xFF;
    if (c->mem == PROG_MEM_CONFIG || c->erase == PROG_ERASE_PAGE) {
        rc = nvm_erase(w, NVM_SECTION_ERASE, base);
        if (rc) return rc;
    }
    sst(w, NVMCMD, NVM_WORD_WRITE);
    for (uint32_t i = 0; i < nw; i += 2) {
        sstpr(w, (uint16_t)(base + i));
        avr12_tx(w, OP_SST_INC); avr12_tx(w, buf[i]);
        avr12_tx(w, OP_SST_INC); avr12_tx(w, buf[i + 1]);
        rc = avr12_commit(w);
        if (!rc) rc = nvm_wait(w);
        if (rc) return rc;
    }
    c->phase = "verify";
    rc = ld(w, base, chk, n);
    if (rc) return rc;
    for (uint32_t i = 0; i < n; i++)
        if (chk[i] != buf[i]) { c->at_addr = c->addr + st->pos + i; return PROG_E_VERIFY; }
    st->pos += n;
    c->done = (st->pos + c->page - 1u) / c->page;
    return st->pos >= c->len ? (st->active = 0, PROG_OK) : PROG_RUNNING;
}

static int tpi_read(prog_ctx_t *c) {
    tpi_st_t *st = ST(c);
    uint16_t base; uint32_t n;
    int rc = next_chunk(c, 2, &base, &n);
    if (rc <= 0) return rc < 0 ? rc : PROG_OK;
    uint8_t buf[MAX_PAGE];
    c->phase = "read";
    rc = ld(c->w, base, buf, n);
    if (rc) return rc;
    if (c->dst(c->job, st->pos, buf, n) != 0) return PROG_E_SRC;
    st->pos += n;
    c->done = (st->pos + c->page - 1u) / c->page;
    return st->pos >= c->len ? (st->active = 0, PROG_OK) : PROG_RUNNING;
}

static void tpi_leave(prog_ctx_t *c, bool run) {
    const prog_wire_ops_t *w = c->w;
    (void)run;                                   /* the chip runs once RESET is released */
    c->phase = "leave";
    avr12_tx(w, OP_SSTCS | REG_TPISR); avr12_tx(w, 0);   /* NVMEN off */
    w->set(w->io, 1, 1);
    w->data_drive(w->io, false);
    (void)avr12_commit(w);
    w->nrst(w->io, false);
}

PROG_PROTO(tpi, tpi_enter, tpi_erase, tpi_write, tpi_read, tpi_leave, PROG_INFO_TPI)
