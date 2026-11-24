/* prog_updi.c — UPDI (AVR tinyAVR 0/1/2, megaAVR 0, AVR DA/DB/DD) for prog_flash (prog_flash.h).
 * See prog_updi.h for what the host passes. One pin, UART engine, half duplex 8E2. The gateware's
 * receiver ignores the line while the pod sends, so the pod never reads back its own bytes: what
 * arrives is the chip's reply only (pymcuprog, on a plain serial adapter, reads and drops an echo).
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
#define SYS_NVMPROG   0x08   /* ASI_SYS_STATUS bit 3, in NVM programming mode */
#define SYS_ERASEFAIL 0x40   /* ASI_SYS_STATUS bit 6, chip erase key failed */
/* CTRLA: IBDLY, RSD (no ACK on stores), GTVAL = 2 cycles. RSD is only for streams the chip can take
   at line speed (flash words); EEPROM, fuse and user row bytes are ACKed, the ACK waits for the write. */
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

/* Write tx, then read nrx reply bytes. No echo to skip: the half-duplex receiver ignores the line
   while the pod sends. The timeout counts the frames still being sent (12 bits each with 2 stop
   bits) plus 15 bit times per reply byte (frame, the chip's guard time and some slack). */
static int xfer(prog_ctx_t *c, const uint8_t *tx, size_t ntx, uint8_t *rx, size_t nrx) {
    const prog_wire_ops_t *w = c->w;
    uint32_t baud = S(c)->baud;
    /* The chip only speaks when asked, so anything already waiting is stale (a late reply to a
       request that timed out): drop it, or every reply after it would be one byte off. */
    w->uart_rx_clear(w->io);
    if (w->uart_write(w->io, tx, ntx) < 0) return PROG_E_PROTOCOL;
    if (nrx && w->uart_read(w->io, rx, nrx,
                            20 + (uint32_t)((ntx * 12u + nrx * 15u) * 1000u / baud)) != nrx)
        return PROG_E_NO_ANSWER;
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

/* A BREAK is one 0x00 at 300 baud: 12 bits (start, 8 data, even parity, 2 stop) low for the first
   10, about 33 ms. Two in a row reset the UPDI state. The SYNC must follow soon after the BREAK
   (the UPDI times out on a long idle line), so wait for the frame by its length: the half-duplex
   receiver never hands the pod's own frames back. */
#define BREAK_FRAME_US  41000u   /* 12 bits at 300 baud = 40 ms, plus margin */
static void brk(prog_ctx_t *c) {
    const prog_wire_ops_t *w = c->w;
    static const uint8_t z = 0;
    w->uart_config(w->io, 300, true, true);
    w->uart_write(w->io, &z, 1);
    pause_us(c, BREAK_FRAME_US);
    w->uart_rx_clear(w->io);
}
static void set_link(prog_ctx_t *c) {
    c->w->uart_config(c->w->io, S(c)->baud, true, true);
    c->w->uart_rx_clear(c->w->io);
}

/* UPDI enable (datasheet "UPDI Enable with Fuse Override"): a short low pulse (one 0x00 at the link
   rate, as pymcuprog does), the chip pulls the line low until its clock runs, then the first SYNC
   must start after at least 200 us and at most 14 ms (TDebZ) from the release, or the UPDI disables
   itself again. 500 us is well inside both limits. */
static void enable_pulse(prog_ctx_t *c) {
    const prog_wire_ops_t *w = c->w;
    static const uint8_t z = 0;
    set_link(c);
    w->uart_write(w->io, &z, 1);
    /* the frame itself (12 bits) by its length, then the 500 us gap before the first SYNC */
    pause_us(c, 12u * 1000000u / S(c)->baud + 1u + 500u);
    w->uart_rx_clear(w->io);
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
        enable_pulse(c);
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
        if (sib[8] != 'P' || sib[9] != ':') return PROG_E_PROTOCOL;
        s->nvm = (uint8_t)(sib[10] - '0');
        /* megaAVR 0 reports the family "megaAVR" and has its flash at 0x4000; tinyAVR 0/1/2 at 0x8000 */
        if (s->nvm == 0) { s->asz = 1; s->flash_base = memcmp(sib, "megaAVR", 7) ? 0x8000 : 0x4000; }
        else if (s->nvm == 2) { s->asz = 2; s->flash_base = 0x800000; }
        else return PROG_E_UNSUPPORTED;
        if (c->flags) s->flash_base = c->flags;
        s->stage = 1;
        return PROG_RUNNING;
    case 1:   /* NVMProg key, reset toggle */
        /* Already in programming mode (an earlier job that failed left it there, or the chip kept
           it from a session that ended without a reset): the key is ignored then and KEY_STATUS
           reads 0, so go straight to the status poll, as pymcuprog does. */
        TRY(ldcs(c, ASI_SYS_STAT, &v));
        if (v & SYS_NVMPROG) { s->t0 = now(c); s->stage = 2; return PROG_RUNNING; }
        TRY(stcs(c, ASI_RESET, RESET_ASSERT));
        TRY(key(c, "NVMProg "));
        TRY(ldcs(c, ASI_KEY_STAT, &v));
        if (!(v & 0x10)) return PROG_E_PROTOCOL;
        TRY(stcs(c, ASI_RESET, RESET_ASSERT));
        TRY(stcs(c, ASI_RESET, 0));
        s->t0 = now(c);
        s->stage = 2;
        return PROG_RUNNING;
    case 2:   /* poll unlock and NVMPROG (datasheet: repeat until set), then read the ID */
        TRY(ldcs(c, ASI_SYS_STAT, &v));
        if (!(v & 1) && (v & 8)) {
            c->phase = "id";
            TRY(read_id(c));
            s->stage = 5;
            return PROG_OK;
        }
        if (now(c) - s->t0 < 100) { pause_us(c, 1000); return PROG_RUNNING; }
        if (!(v & 1)) return PROG_E_PROTOCOL;       /* unlocked, but the NVMPROG key was not accepted */
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
        if (v & SYS_ERASEFAIL) { c->phase = "erase"; return PROG_E_LOCKED; }   /* key erase refused, still locked */
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
    pause_us(c, 1000);
    if (run) {
        /* On the bench (ATtiny1607) the reset release and UPDIDIS sent back to back left the chip
           in NVM programming mode with its CPU stopped. Wait until it is out of programming mode
           (a few ms, as pymcuprog's serial latency gives it) before disabling the UPDI. */
        uint8_t v = 0;
        stcs(c, ASI_RESET, 0);
        uint32_t t0 = now(c);
        do {
            pause_us(c, 1000);
            if (ldcs(c, ASI_SYS_STAT, &v)) break;
        } while ((v & SYS_NVMPROG) && now(c) - t0 < 50);
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

/* ---- memory access ---- */

static uint32_t mbase(prog_ctx_t *c) {
    st_t *s = S(c);
    switch (c->mem) {
    case PROG_MEM_FLASH:  return s->flash_base;
    case PROG_MEM_EEPROM: return 0x1400;
    case PROG_MEM_CONFIG: return s->nvm == 0 ? 0x1280 : 0x1050;
    default:              return s->nvm == 0 ? 0x1300 : 0x1080;   /* user row */
    }
}

/* Read n bytes at data address daddr (BUF at a time, one REPEAT + LD ptr++ each). verify: compare
   with the staged bytes at src offset off (first difference: ctx->at_addr), else store to dst. */
static int blk(prog_ctx_t *c, uint32_t daddr, uint32_t off, uint32_t n, bool verify) {
    st_t *s = S(c);
    TRY(st_ptr(c, daddr));
    for (uint32_t p = 0; p < n;) {
        uint32_t k = n - p < BUF ? n - p : BUF;
        uint8_t t[5] = { SYNC, 0xA0, (uint8_t)(k - 1), SYNC, 0x24 };
        TRY(xfer(c, t, 5, s->buf, k));
        if (verify) {
            if (c->src(c->job, off + p, s->cmp, k)) return PROG_E_SRC;
            for (uint32_t i = 0; i < k; i++)
                if (s->buf[i] != s->cmp[i]) { c->at_addr = c->addr + off + p + i; return PROG_E_VERIFY; }
        } else if (c->dst(c->job, off + p, s->buf, k)) return PROG_E_SRC;
        p += k;
    }
    return PROG_OK;
}

/* Store n bytes (the first `real` come from the staged image at src offset off, the rest is 0xFF
   padding) to data address daddr with ST ptr++; words = 16-bit stores. rsd: stream without ACKs
   (flash words only); else every byte is ACKed, which waits for the chip's write (EEPROM 10 ms). */
static int store(prog_ctx_t *c, uint32_t daddr, uint32_t off, uint32_t real, uint32_t n, bool words, bool rsd) {
    st_t *s = S(c);
    for (uint32_t d = 0; d < n;) {
        uint32_t m = n - d, mx = words ? 512 : 256;
        if (m > mx) m = mx;
        TRY(st_ptr(c, daddr + d));
        if (rsd) TRY(stcs(c, CS_CTRLA, CTRLA_NOACK));
        uint8_t h[5] = { SYNC, 0xA0, (uint8_t)((words ? m / 2 : m) - 1), SYNC, (uint8_t)(0x64 | words) };
        int r = xfer(c, h, 5, NULL, 0);
        for (uint32_t p = 0; !r && p < m;) {
            uint32_t k = m - p < BUF ? m - p : BUF;
            uint32_t have = real > d + p ? real - (d + p) : 0;
            if (have > k) have = k;
            if (have && c->src(c->job, off + d + p, s->buf, have)) r = PROG_E_SRC;
            else {
                memset(s->buf + have, 0xFF, k - have);
                if (rsd) r = xfer(c, s->buf, k, NULL, 0);
                else for (uint32_t i = 0; !r && i < k; i++) r = ack1(c, s->buf + i, 1);
            }
            p += k;
        }
        int r2 = rsd ? stcs(c, CS_CTRLA, CTRLA_ON) : PROG_OK;
        if (r) return r;
        TRY(r2);
        d += m;
    }
    return PROG_OK;
}

/* geom: page size and flash alignment matter (write); a read needs only the range. */
static int check_args(prog_ctx_t *c, bool geom) {
    st_t *s = S(c);
    if (c->mem >= PROG_MEM_COUNT) return PROG_E_ARGS;
    if (geom && c->mem != PROG_MEM_CONFIG) {
        if (!c->page || c->page > 512) return PROG_E_ARGS;
        if (c->mem == PROG_MEM_FLASH && ((c->page | c->addr) & 1)) return PROG_E_ARGS;
    }
    uint32_t lim = s->asz == 1 ? 0x10000u : 0x1000000u, room = lim - mbase(c);   /* address space left */
    if (mbase(c) >= lim || c->addr > room || c->len > room - c->addr) return PROG_E_ARGS;
    return PROG_OK;
}

/* ---- write ---- */

/* The commands and stores for one chunk of n bytes at data address a (src offset s->pos). */
static int write_chunk(prog_ctx_t *c, uint32_t a, uint32_t n) {
    st_t *s = S(c);
    uint32_t off = s->pos, np = n;
    bool v0 = s->nvm == 0, fl = c->mem == PROG_MEM_FLASH, cfg = c->mem == PROG_MEM_CONFIG;
    bool page_erase = c->erase == PROG_ERASE_PAGE || (!v0 && c->mem == PROG_MEM_USER);
    if (fl) np = (n + 1) & ~1u;     /* word access to the flash buffer */
    TRY(nvm_wait(c, 100));
    if (v0) {
        if (cfg) {
            uint8_t b;
            if (c->src(c->job, off, &b, 1)) return PROG_E_SRC;
            TRY(sts8(c, NVM_BASE + NVM_ADDR, (uint8_t)a));
            TRY(sts8(c, NVM_BASE + NVM_ADDR + 1, (uint8_t)(a >> 8)));
            TRY(sts8(c, NVM_BASE + NVM_DATA, b));
            TRY(nvm_cmd(c, V0_WFU));
            return nvm_wait(c, 100);
        }
        TRY(nvm_cmd(c, V0_PBC));
        TRY(nvm_wait(c, 100));
        TRY(store(c, a, off, n, np, fl, true));
        TRY(nvm_cmd(c, (fl && !page_erase) ? V0_WP : V0_ERWP));   /* eeprom and user row always erase-write */
        return nvm_wait(c, 100);
    }
    if (fl || c->mem == PROG_MEM_USER) {
        if (page_erase) {
            TRY(nvm_cmd(c, V2_FLPER));
            TRY(sts8(c, a, 0xFF));
            TRY(nvm_wait(c, 100));
            TRY(nvm_cmd(c, V2_NOCMD));
        }
        TRY(nvm_cmd(c, V2_FLWR));
        TRY(store(c, a, off, n, np, fl, fl));       /* flash words stream, user row bytes are ACKed */
    } else {
        TRY(nvm_cmd(c, V2_EEERWR));      /* eeprom and fuses: ACKed bytes (10 ms each) */
        TRY(store(c, a, off, n, np, false, false));
    }
    TRY(nvm_wait(c, 100));
    return nvm_cmd(c, V2_NOCMD);
}

static int write(prog_ctx_t *c) {
    st_t *s = S(c);
    c->phase = "write";
    if (!c->len) return PROG_OK;
    TRY(check_args(c, true));
    uint32_t a = c->addr + s->pos, n;
    if (c->mem == PROG_MEM_CONFIG) {
        n = 1;
        c->total = c->len;
    } else {
        n = c->page - a % c->page;
        if (n > c->len - s->pos) n = c->len - s->pos;
        c->total = (c->addr + c->len - 1) / c->page - c->addr / c->page + 1;
    }
    TRY(write_chunk(c, mbase(c) + a, n));
    c->phase = "verify";
    TRY(blk(c, mbase(c) + a, s->pos, n, true));
    s->pos += n;
    c->done = ++s->chunk;
    if (s->pos < c->len) return PROG_RUNNING;
    s->pos = s->chunk = 0;
    return PROG_OK;
}

/* ---- read ---- */

static int read(prog_ctx_t *c) {
    st_t *s = S(c);
    c->phase = "read";
    if (!c->len) return PROG_OK;
    TRY(check_args(c, false));
    uint32_t n = c->len - s->pos;
    if (n > 256) n = 256;
    c->total = (c->len + 255) / 256;
    TRY(blk(c, mbase(c) + c->addr + s->pos, s->pos, n, false));
    s->pos += n;
    c->done = ++s->chunk;
    if (s->pos < c->len) return PROG_RUNNING;
    s->pos = s->chunk = 0;
    return PROG_OK;
}

PROG_PROTO(updi, enter, erase, write, read, leave, PROG_INFO_UPDI)
