/* prog_updi.c — UPDI (AVR tinyAVR 0/1/2, megaAVR 0, AVR DA/DB/DD) for prog_flash (prog_flash.h).
 * See prog_updi.h for what the host passes. One pin, UART engine, half duplex 8E2: every byte
 * sent comes back as an echo, which is read and checked before the chip's reply.
 * Sequences cross-checked against pymcuprog's serialupdi (application, link, nvmp0, nvmp2). */
#include "prog_flash.h"
#include "prog_updi.h"
#include <string.h>

#define SYNC 0x55
#define ACK  0x40

/* CS registers */
#define CS_STATUSA   0x00
#define CS_CTRLA     0x02
#define CS_CTRLB     0x03
#define ASI_KEY_STAT 0x07
#define ASI_RESET    0x08
#define ASI_SYS_STAT 0x0B
/* CTRLA: IBDLY, RSD (no ACK on stores), GTVAL = 2 cycles */
#define CTRLA_ON     0x86
#define CTRLA_NOACK  0x8E
#define CTRLB_CCDETDIS 0x08
#define CTRLB_UPDIDIS  0x04
#define RESET_ASSERT 0x59

/* NVM controller (same register layout for both versions) */
#define NVM_BASE   0x1000u
#define NVM_CTRLA  0x00
#define NVM_STATUS 0x02
#define NVM_DATA   0x06
#define NVM_ADDR   0x08
#define SIGROW     0x1100u

/* v0 commands */
#define V0_WP 0x01
#define V0_ERWP 0x03
#define V0_PBC 0x04
#define V0_CHER 0x05
#define V0_WFU 0x07
/* v2 commands */
#define V2_NOCMD 0x00
#define V2_FLWR 0x02
#define V2_FLPER 0x08
#define V2_EEERWR 0x13
#define V2_CHER 0x20

#define BUF 64u
#define TRY(x) do { int r_ = (x); if (r_) return r_; } while (0)

typedef struct {
    uint32_t baud, t0, flash_base, pos, chunk;
    uint8_t stage, es, nvm, asz;       /* asz: address bytes - 1 (1 = 16 bit, 2 = 24 bit) */
    uint8_t linked, erase_tried, erased;
    uint8_t buf[BUF], cmp[BUF];
} st_t;
_Static_assert(sizeof(st_t) <= PROG_STATE_BYTES, "updi state too big");

static st_t *S(prog_ctx_t *c) { return (st_t *)c->state; }
static uint32_t now(prog_ctx_t *c) { return c->w->now_ms(c->w->io); }
static void pause_us(prog_ctx_t *c, uint32_t us) { c->w->delay_us(c->w->io, us); }

/* Write tx, read and check its echo, then read nrx reply bytes. */
static int xfer(prog_ctx_t *c, const uint8_t *tx, size_t ntx, uint8_t *rx, size_t nrx) {
    const prog_wire_ops_t *w = c->w;
    uint32_t baud = S(c)->baud;
    if (w->uart_write(w->io, tx, ntx) < 0) return PROG_E_PROTOCOL;
    uint8_t e[16];
    for (size_t i = 0; i < ntx;) {
        size_t k = ntx - i < sizeof(e) ? ntx - i : sizeof(e);
        if (w->uart_read(w->io, e, k, 20 + (uint32_t)(k * 13000u / baud)) != k) return PROG_E_NO_ANSWER;
        if (memcmp(e, tx + i, k)) return PROG_E_PROTOCOL;
        i += k;
    }
    if (nrx && w->uart_read(w->io, rx, nrx, 20 + (uint32_t)(nrx * 13000u / baud)) != nrx) return PROG_E_NO_ANSWER;
    return PROG_OK;
}

static int ack1(prog_ctx_t *c, const uint8_t *tx, size_t n) {
    uint8_t a;
    TRY(xfer(c, tx, n, &a, 1));
    return a == ACK ? PROG_OK : PROG_E_PROTOCOL;
}

static int stcs(prog_ctx_t *c, uint8_t reg, uint8_t v) {
    uint8_t t[3] = { SYNC, (uint8_t)(0xC0 | reg), v };
    return xfer(c, t, 3, NULL, 0);
}
static int ldcs(prog_ctx_t *c, uint8_t reg, uint8_t *v) {
    uint8_t t[2] = { SYNC, (uint8_t)(0x80 | reg) };
    return xfer(c, t, 2, v, 1);
}

static unsigned put_addr(prog_ctx_t *c, uint8_t *t, uint32_t a) {
    unsigned n = S(c)->asz + 1;
    for (unsigned i = 0; i < n; i++) t[i] = (uint8_t)(a >> (8 * i));
    return n;
}
static int lds8(prog_ctx_t *c, uint32_t a, uint8_t *v) {
    uint8_t t[5] = { SYNC, (uint8_t)(S(c)->asz << 2) };
    unsigned n = put_addr(c, t + 2, a);
    return xfer(c, t, 2 + n, v, 1);
}
static int sts8(prog_ctx_t *c, uint32_t a, uint8_t v) {
    uint8_t t[5] = { SYNC, (uint8_t)(0x40 | (S(c)->asz << 2)) };
    unsigned n = put_addr(c, t + 2, a);
    TRY(ack1(c, t, 2 + n));
    return ack1(c, &v, 1);
}
static int st_ptr(prog_ctx_t *c, uint32_t a) {
    uint8_t t[5] = { SYNC, (uint8_t)(0x68 | S(c)->asz) };
    unsigned n = put_addr(c, t + 2, a);
    return ack1(c, t, 2 + n);
}

/* Key: 0xE0, then the 8 key bytes last first. */
static int key(prog_ctx_t *c, const char *k) {
    uint8_t t[10] = { SYNC, 0xE0 };
    for (int i = 0; i < 8; i++) t[2 + i] = (uint8_t)k[7 - i];
    return xfer(c, t, 10, NULL, 0);
}

/* A BREAK is one 0x00 at 300 baud (about 30 ms low); two in a row reset the UPDI state. */
static void brk(prog_ctx_t *c) {
    const prog_wire_ops_t *w = c->w;
    static const uint8_t z = 0;
    w->uart_config(w->io, 300, true, true);
    w->uart_write(w->io, &z, 1);
    pause_us(c, 50000);
    w->uart_rx_clear(w->io);
}
static void set_link(prog_ctx_t *c) {
    c->w->uart_config(c->w->io, S(c)->baud, true, true);
    c->w->uart_rx_clear(c->w->io);
}

static int link_init(prog_ctx_t *c) {
    uint8_t v = 0;
    TRY(stcs(c, CS_CTRLB, CTRLB_CCDETDIS));
    TRY(stcs(c, CS_CTRLA, CTRLA_ON));
    TRY(ldcs(c, CS_STATUSA, &v));
    return v ? PROG_OK : PROG_E_NO_ANSWER;
}

/* ---- enter ---- */

static int read_id(prog_ctx_t *c) {
    TRY(st_ptr(c, SIGROW));
    uint8_t t[5] = { SYNC, 0xA0, 2, SYNC, 0x24 };
    TRY(xfer(c, t, 5, c->id, 3));
    c->id_len = 3;
    return PROG_OK;
}

static int enter(prog_ctx_t *c) {
    st_t *s = S(c);
    uint8_t v, sib[16];
    c->phase = "enter";
    switch (s->stage) {
    case 0:
        memset(s, 0, sizeof(*s));
        s->baud = c->baud ? c->baud : 115200;
        if (s->baud < 4800 || s->baud > 225000) return PROG_E_ARGS;
        set_link(c);
        if (link_init(c)) {
            brk(c); brk(c);
            set_link(c);
            TRY(link_init(c));
        }
        s->linked = 1;
        {
            static const uint8_t t[2] = { SYNC, 0xE5 };
            TRY(xfer(c, t, 2, sib, 16));
        }
        for (int i = 0; i < 16; i++) if (sib[i] < 0x20 || sib[i] > 0x7E) return PROG_E_PROTOCOL;
        if (sib[8] != 'P' || sib[9] != ':') return PROG_E_PROTOCOL;
        s->nvm = (uint8_t)(sib[10] - '0');
        if (s->nvm == 0) { s->asz = 1; s->flash_base = 0x8000; }
        else if (s->nvm == 2) { s->asz = 2; s->flash_base = 0x800000; }
        else return PROG_E_UNSUPPORTED;
        if (c->flags) s->flash_base = c->flags;
        s->stage = 1;
        return PROG_RUNNING;
    case 1:   /* NVMProg key, reset toggle */
        TRY(stcs(c, ASI_RESET, RESET_ASSERT));
        TRY(key(c, "NVMProg "));
        TRY(ldcs(c, ASI_KEY_STAT, &v));
        if (!(v & 0x10)) return PROG_E_PROTOCOL;
        TRY(stcs(c, ASI_RESET, RESET_ASSERT));
        TRY(stcs(c, ASI_RESET, 0));
        s->t0 = now(c);
        s->stage = 2;
        return PROG_RUNNING;
    case 2:   /* wait for unlock, then read the ID */
        TRY(ldcs(c, ASI_SYS_STAT, &v));
        if (!(v & 1)) {
            if (!(v & 8)) return PROG_E_PROTOCOL;
            c->phase = "id";
            TRY(read_id(c));
            s->stage = 5;
            return PROG_OK;
        }
        if (now(c) - s->t0 < 100) { pause_us(c, 1000); return PROG_RUNNING; }
        if (c->erase != PROG_ERASE_CHIP || s->erase_tried) return PROG_E_LOCKED;
        s->stage = 3;
        return PROG_RUNNING;
    case 3:   /* locked: chip erase key unlocks */
        TRY(key(c, "NVMErase"));
        TRY(ldcs(c, ASI_KEY_STAT, &v));
        if (!(v & 0x08)) return PROG_E_PROTOCOL;
        TRY(stcs(c, ASI_RESET, RESET_ASSERT));
        TRY(stcs(c, ASI_RESET, 0));
        s->erase_tried = 1;
        s->t0 = now(c);
        s->stage = 4;
        return PROG_RUNNING;
    case 4:
        TRY(ldcs(c, ASI_SYS_STAT, &v));
        if (!(v & 1)) { s->erased = 1; s->stage = 1; return PROG_RUNNING; }
        if (now(c) - s->t0 >= 500) return PROG_E_TIMEOUT;
        pause_us(c, 1000);
        return PROG_RUNNING;
    }
    return PROG_OK;
}

static void leave(prog_ctx_t *c, bool run) {
    st_t *s = S(c);
    if (!s->linked) return;
    c->phase = "leave";
    stcs(c, ASI_RESET, RESET_ASSERT);
    if (run) {
        stcs(c, ASI_RESET, 0);
        stcs(c, CS_CTRLB, CTRLB_UPDIDIS | CTRLB_CCDETDIS);
    }
}

/* ---- NVM controller ---- */

static int nvm_cmd(prog_ctx_t *c, uint8_t cmd) { return sts8(c, NVM_BASE + NVM_CTRLA, cmd); }

/* One status read: busy flag in *busy, a write error answers PROG_E_PROTOCOL. */
static int nvm_status(prog_ctx_t *c, bool *busy) {
    uint8_t v;
    TRY(lds8(c, NVM_BASE + NVM_STATUS, &v));
    if (v & (S(c)->nvm == 0 ? 0x04 : 0x70)) return PROG_E_PROTOCOL;
    *busy = v & 3;
    return PROG_OK;
}

static int nvm_wait(prog_ctx_t *c, uint32_t tmo_ms) {
    uint32_t t0 = now(c);
    for (;;) {
        bool busy;
        TRY(nvm_status(c, &busy));
        if (!busy) return PROG_OK;
        pause_us(c, 500);
        if (now(c) - t0 >= tmo_ms) return PROG_E_TIMEOUT;
    }
}

/* ---- erase ---- */

static int erase(prog_ctx_t *c) {
    st_t *s = S(c);
    bool busy;
    c->phase = "erase";
    if (c->erase != PROG_ERASE_CHIP || s->erased) return PROG_OK;   /* page erase happens in write */
    if (s->es == 0) {
        TRY(nvm_wait(c, 100));
        TRY(nvm_cmd(c, s->nvm == 0 ? V0_CHER : V2_CHER));
        s->t0 = now(c);
        s->es = 1;
        return PROG_RUNNING;
    }
    TRY(nvm_status(c, &busy));
    if (busy) {
        if (now(c) - s->t0 >= 1000) return PROG_E_TIMEOUT;
        pause_us(c, 1000);
        return PROG_RUNNING;
    }
    if (s->nvm == 2) TRY(nvm_cmd(c, V2_NOCMD));
    s->erased = 1;
    s->es = 0;
    return PROG_OK;
}

static int stub(prog_ctx_t *ctx) { (void)ctx; return PROG_E_UNSUPPORTED; }

const prog_proto_t prog_proto_updi = {
    .name = "updi", .uart = true, .uses_nrst = false,
    .mems = (1u<<PROG_MEM_FLASH)|(1u<<PROG_MEM_EEPROM)|(1u<<PROG_MEM_CONFIG)|(1u<<PROG_MEM_USER),
    .enter = enter, .erase = erase, .write = stub, .read = stub, .leave = leave,
};
