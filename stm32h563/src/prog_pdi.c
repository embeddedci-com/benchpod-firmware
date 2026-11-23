/* prog_pdi.c — XMEGA PDI (ATxmega A/B/C/D/E) for prog_flash (prog_flash.h).
 *
 * PDI_CLK is the chip's RESET pin (the pod's reset pin is not used), PDI_DATA the data pin. 12-bit
 * frames: prog_avr12.h. The clock is never held low: every frame helper returns with it high, and
 * a low longer than ~100 us would reset the chip out of PDI.
 *
 * The pod knows no device tables: the host passes the page size in ctx->page (bytes) and addr
 * relative to the memory: flash (application + boot, one address space from 0), eeprom, config
 * (fuse byte 0.. 5), user (user signature row). Pages go through the NVM controller's page buffer.
 */
#include "prog_flash.h"
#include "prog_avr12.h"
#include <string.h>

/* PDI instructions */
enum { I_LDS4 = 0x0C, I_STS4 = 0x4C, I_LD_INC = 0x24, I_ST = 0x60, I_ST_INC = 0x64, I_STPTR4 = 0x6B,
       I_LDCS = 0x80, I_STCS = 0xC0, I_REPEAT1 = 0xA0, I_KEY = 0xE0 };
enum { CS_STATUS = 0, CS_RESET = 1, CS_CTRL = 2 };
#define STATUS_NVMEN  0x02u
#define CTRL_GT2      0x06u              /* guard time: 2 idle bits */

/* PDI data space */
#define A_FLASH   0x00800000u
#define A_EEPROM  0x008C0000u
#define A_USER    0x008E0400u
#define A_FUSE    0x008F0020u
#define A_LOCK    0x008F0027u
#define A_DEVID   0x01000090u            /* MCU.DEVID0..2 */
#define A_NVM     0x010001C0u
#define NVM_ADDR0 (A_NVM + 0x00u)
#define NVM_DATA0 (A_NVM + 0x04u)
#define NVM_CMD   (A_NVM + 0x0Au)
#define NVM_CTRLA (A_NVM + 0x0Bu)
#define NVM_STATUS (A_NVM + 0x0Fu)
#define NVMBUSY   0x80u

/* NVM commands */
enum { C_READ_NVM = 0x43, C_CHIP_ERASE = 0x40, C_WRITE_FUSE = 0x4C,
       C_ERASE_FLASH_BUF = 0x26, C_LOAD_FLASH_BUF = 0x23, C_WRITE_FLASH = 0x2E, C_ERASE_WRITE_FLASH = 0x2F,
       C_ERASE_EE_BUF = 0x36, C_LOAD_EE_BUF = 0x33, C_WRITE_EE = 0x34, C_ERASE_WRITE_EE = 0x35,
       C_ERASE_USER = 0x18, C_WRITE_USER = 0x1A };

#define MAX_PAGE  1024u
#define PIECE     64u
#define BUSY_MS   500u

static const uint8_t k_nvm_key[8] = { 0xFF, 0x88, 0xD8, 0xCD, 0x45, 0xAB, 0x89, 0x12 };

typedef struct {
    uint32_t addr, len, pos;
    uint8_t  mem, kind, active, locked;
} pdi_st_t;
_Static_assert(sizeof(pdi_st_t) <= PROG_STATE_BYTES, "pdi state");
#define ST(c) ((pdi_st_t *)(void *)(c)->state)

static void tx32(const prog_wire_ops_t *w, uint32_t a) {
    for (unsigned i = 0; i < 4; i++) avr12_tx(w, (uint8_t)(a >> (8 * i)));
}

static void stptr(const prog_wire_ops_t *w, uint32_t a) { avr12_tx(w, I_STPTR4); tx32(w, a); }

static void sts(const prog_wire_ops_t *w, uint32_t a, uint8_t v) {
    avr12_tx(w, I_STS4); tx32(w, a); avr12_tx(w, v);
}

static int lds(const prog_wire_ops_t *w, uint32_t a, uint8_t *v) {
    avr12_tx(w, I_LDS4); tx32(w, a);
    return avr12_rx(w, v);
}

/* n bytes from a (data space), one answer each. */
static int ld(const prog_wire_ops_t *w, uint32_t a, uint8_t *buf, unsigned n) {
    stptr(w, a);
    for (unsigned i = 0; i < n; i++) {
        avr12_tx(w, I_LD_INC);
        int rc = avr12_rx(w, &buf[i]);
        if (rc) return rc;
    }
    return 0;
}

static int nvm_wait(const prog_wire_ops_t *w) {
    uint32_t t0 = w->now_ms(w->io);
    for (;;) {
        uint8_t v;
        int rc = lds(w, NVM_STATUS, &v);
        if (rc) return rc;
        if (!(v & NVMBUSY)) return 0;
        if (w->now_ms(w->io) - t0 > BUSY_MS) return PROG_E_TIMEOUT;
        w->delay_us(w->io, 200);
    }
}

/* A command started by CMDEX (buffer erase, chip erase, fuse write). */
static int cmd_ex(const prog_wire_ops_t *w, uint8_t cmd) {
    sts(w, NVM_CMD, cmd);
    sts(w, NVM_CTRLA, 1);
    int rc = avr12_commit(w);
    return rc ? rc : nvm_wait(w);
}

/* A command started by a dummy write into the memory at `a` (page erase/write). */
static int cmd_at(const prog_wire_ops_t *w, uint8_t cmd, uint32_t a) {
    sts(w, NVM_CMD, cmd);
    stptr(w, a);
    avr12_tx(w, I_ST); avr12_tx(w, 0);
    int rc = avr12_commit(w);
    return rc ? rc : nvm_wait(w);
}

static int read_lock(prog_ctx_t *c) {
    uint8_t v;
    int rc = lds(c->w, A_LOCK, &v);
    if (rc) return rc;
    ST(c)->locked = (v & 3u) != 3u;
    return 0;
}

static int pdi_enter(prog_ctx_t *c) {
    const prog_wire_ops_t *w = c->w;
    memset(c->state, 0, sizeof(pdi_st_t));
    c->phase = "enter"; c->done = 0; c->total = 0;
    avr12_idle(w, 20);                                   /* data high, clock running: enables PDI */
    avr12_tx(w, I_STCS | CS_CTRL); avr12_tx(w, CTRL_GT2);
    avr12_tx(w, I_KEY);
    for (unsigned i = 0; i < 8; i++) avr12_tx(w, k_nvm_key[i]);
    uint8_t sr = 0;
    int rc = 0;
    for (unsigned tries = 0; tries < 10; tries++) {
        avr12_tx(w, I_LDCS | CS_STATUS);
        rc = avr12_rx(w, &sr);
        if (rc) return rc;
        if (sr & STATUS_NVMEN) break;
        w->delay_us(w->io, 100);
    }
    if (!(sr & STATUS_NVMEN)) return PROG_E_PROTOCOL;    /* the chip did not take the key */
    c->phase = "id";
    c->id_len = 3;
    rc = ld(w, A_DEVID, c->id, 3);
    if (rc) return rc;
    return read_lock(c);
}

static int pdi_erase(prog_ctx_t *c) {
    if (c->erase != PROG_ERASE_CHIP) return PROG_OK;
    c->phase = "erase";
    int rc = cmd_ex(c->w, C_CHIP_ERASE);
    if (rc) return rc;
    rc = read_lock(c);
    if (rc) return rc;
    return ST(c)->locked ? PROG_E_LOCKED : PROG_OK;
}

/* Validate and pick the next chunk: <0 error, 0 finished, 1 chunk (*base data space, *n bytes,
   *pg the page start address for page memories). */
static int next_chunk(prog_ctx_t *c, uint8_t kind, uint32_t *base, uint32_t *n, uint32_t *pg) {
    pdi_st_t *st = ST(c);
    uint32_t mbase, mmax;
    switch (c->mem) {
    case PROG_MEM_FLASH:  mbase = A_FLASH;  mmax = 0x400000u; break;
    case PROG_MEM_EEPROM: mbase = A_EEPROM; mmax = 0x10000u;  break;
    case PROG_MEM_USER:   mbase = A_USER;   mmax = MAX_PAGE;  break;
    case PROG_MEM_CONFIG: mbase = A_FUSE;   mmax = 8;         break;
    default: return PROG_E_ARGS;
    }
    bool cfg = c->mem == PROG_MEM_CONFIG;
    if (!cfg && (c->page == 0 || c->page > MAX_PAGE)) return PROG_E_ARGS;
    if (c->addr > mmax || c->len > mmax - c->addr) return PROG_E_ARGS;
    if (!st->active || st->mem != c->mem || st->kind != kind || st->addr != c->addr || st->len != c->len) {
        st->active = 1; st->mem = c->mem; st->kind = kind; st->addr = c->addr; st->len = c->len; st->pos = 0;
    }
    uint32_t unit = cfg ? 1u : c->page;
    c->total = (c->len + unit - 1u) / unit;
    if (st->pos >= c->len) { st->active = 0; return 0; }
    if (st->locked) return PROG_E_LOCKED;
    uint32_t a = c->addr + st->pos;
    uint32_t k = unit - a % unit;
    if (k > c->len - st->pos) k = c->len - st->pos;
    *n = k;
    *base = mbase + a;
    *pg = mbase + a - a % unit;
    return 1;
}

/* Fill the page buffer with n staged bytes at data space address `base`. */
static int load_buffer(prog_ctx_t *c, uint32_t off, uint32_t base, uint32_t n) {
    const prog_wire_ops_t *w = c->w;
    uint8_t buf[PIECE];
    stptr(w, base);
    for (uint32_t done = 0; done < n;) {
        uint32_t k = n - done < PIECE ? n - done : PIECE;
        if (c->src(c->job, off + done, buf, k) != 0) return PROG_E_SRC;
        avr12_tx(w, I_REPEAT1); avr12_tx(w, (uint8_t)(k - 1u));
        avr12_tx(w, I_ST_INC);
        for (uint32_t i = 0; i < k; i++) avr12_tx(w, buf[i]);
        int rc = avr12_commit(w);
        if (rc) return rc;
        done += k;
    }
    return 0;
}

/* Read back n bytes at `base` and compare with the staged image. */
static int verify(prog_ctx_t *c, uint32_t off, uint32_t base, uint32_t n) {
    const prog_wire_ops_t *w = c->w;
    uint8_t want[PIECE], got[PIECE];
    int rc = 0;
    sts(w, NVM_CMD, C_READ_NVM);
    for (uint32_t done = 0; done < n;) {
        uint32_t k = n - done < PIECE ? n - done : PIECE;
        if (c->src(c->job, off + done, want, k) != 0) return PROG_E_SRC;
        rc = ld(w, base + done, got, k);
        if (rc) return rc;
        for (uint32_t i = 0; i < k; i++)
            if (got[i] != want[i]) { c->at_addr = c->addr + off + done + i; return PROG_E_VERIFY; }
        done += k;
    }
    return 0;
}

static uint32_t unit_done(const prog_ctx_t *c, uint32_t pos) {
    uint32_t unit = c->mem == PROG_MEM_CONFIG ? 1u : c->page;
    return (pos + unit - 1u) / unit;
}

static int pdi_write(prog_ctx_t *c) {
    pdi_st_t *st = ST(c);
    const prog_wire_ops_t *w = c->w;
    uint32_t base, n, pg;
    int rc = next_chunk(c, 1, &base, &n, &pg);
    if (rc <= 0) return rc < 0 ? rc : PROG_OK;
    c->phase = "write";
    if (c->mem == PROG_MEM_CONFIG) {
        uint8_t v;
        if (c->src(c->job, st->pos, &v, 1) != 0) return PROG_E_SRC;
        for (unsigned i = 0; i < 3; i++) sts(w, NVM_ADDR0 + i, (uint8_t)(base >> (8 * i)));
        sts(w, NVM_DATA0, v);
        rc = cmd_ex(w, C_WRITE_FUSE);
    } else {
        bool ee = c->mem == PROG_MEM_EEPROM, user = c->mem == PROG_MEM_USER;
        bool erase = c->erase == PROG_ERASE_PAGE;
        rc = cmd_ex(w, ee ? C_ERASE_EE_BUF : C_ERASE_FLASH_BUF);
        if (!rc) { sts(w, NVM_CMD, ee ? C_LOAD_EE_BUF : C_LOAD_FLASH_BUF); rc = load_buffer(c, st->pos, base, n); }
        if (!rc && user) rc = cmd_at(w, C_ERASE_USER, pg);
        if (!rc) {
            uint8_t op = user ? C_WRITE_USER
                       : ee   ? (erase ? C_ERASE_WRITE_EE : C_WRITE_EE)
                              : (erase ? C_ERASE_WRITE_FLASH : C_WRITE_FLASH);
            rc = cmd_at(w, op, pg);
        }
    }
    if (rc) return rc;
    c->phase = "verify";
    rc = verify(c, st->pos, base, n);
    if (rc) return rc;
    st->pos += n;
    c->done = unit_done(c, st->pos);
    return st->pos >= c->len ? (st->active = 0, PROG_OK) : PROG_RUNNING;
}

static int pdi_read(prog_ctx_t *c) {
    pdi_st_t *st = ST(c);
    const prog_wire_ops_t *w = c->w;
    uint32_t base, n, pg;
    int rc = next_chunk(c, 2, &base, &n, &pg);
    if (rc <= 0) return rc < 0 ? rc : PROG_OK;
    uint8_t buf[PIECE];
    c->phase = "read";
    sts(w, NVM_CMD, C_READ_NVM);
    for (uint32_t done = 0; done < n;) {
        uint32_t k = n - done < PIECE ? n - done : PIECE;
        rc = ld(w, base + done, buf, k);
        if (rc) return rc;
        if (c->dst(c->job, st->pos + done, buf, k) != 0) return PROG_E_SRC;
        done += k;
    }
    st->pos += n;
    c->done = unit_done(c, st->pos);
    return st->pos >= c->len ? (st->active = 0, PROG_OK) : PROG_RUNNING;
}

static void pdi_leave(prog_ctx_t *c, bool run) {
    const prog_wire_ops_t *w = c->w;
    (void)run;                                   /* the chip runs once RESET (the clock pin) stays high */
    c->phase = "leave";
    avr12_tx(w, I_STCS | CS_RESET); avr12_tx(w, 0);     /* any value but 0x59 releases the reset */
    w->set(w->io, 1, 1);
    w->data_drive(w->io, false);
    (void)avr12_commit(w);
    w->delay_us(w->io, 100);                     /* clock stays high */
}

PROG_PROTO(pdi, pdi_enter, pdi_erase, pdi_write, pdi_read, pdi_leave, PROG_INFO_PDI)
