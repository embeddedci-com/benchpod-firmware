/* test_prog_updi.c — host test of prog_updi.c against a model of a tinyAVR (NVM v0) and an
 * AVR DA (NVM v2) behind prog_sim's UART path. The model parses UPDI frames byte by byte,
 * keeps CS registers, keys, lock state and a small NVM controller, so a wrong sequence
 * (missing erase, byte access to the flash page buffer, no command before a store) shows up
 * as a failing read-back or a counted violation.
 *
 * Timing (simulated us from prog_sim): the UPDI starts disabled and is enabled by a 0x00 at the link
 * rate or a BREAK; the first SYNC must then start 200 us to 14 ms after the pulse ends (TDebZ), after
 * a BREAK within 5 ms of its end, else it is lost (UPDI disables itself, counted in timing_viol).
 * A v2 EEPROM/fuse byte takes 10 ms to write: a streamed (RSD, no ACK) byte that arrives while the
 * chip is busy is dropped (counted in drops), an ACKed one is held until the write is done. NVMPROG
 * can appear after a delay (prog_delay_us), and a refused key erase sets ERASEFAIL. */
#include "../src/prog_flash.h"
#include "prog_sim.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int fails, checks;
#define CHECK(c) do { checks++; if (!(c)) { fails++; printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); } } while (0)
#define CHECK_EQ(a, b) do { long _a = (long)(a), _b = (long)(b); checks++; if (_a != _b) { fails++; \
    printf("FAIL %s:%d: %s = %ld, expected %ld\n", __FILE__, __LINE__, #a, _a, _b); } } while (0)

/* ---- the chip model ---- */

enum { R_FLASH, R_EE, R_FUSE, R_USER, R_SIG, R_N };
typedef struct { uint32_t base, size; uint8_t d[0x4000]; } reg_t;

enum { P_SYNC, P_OP, P_ARGS, P_DATA };
enum { C_NONE, C_LDS, C_STS, C_STS_DATA, C_STPTR, C_STCS, C_RPT, C_KEY, C_ST };

typedef struct {
    int nvm;                       /* 0 or 2 */
    char sib[17];
    reg_t r[R_N];
    uint32_t fpage, epage, upage;
    /* link */
    int ps, cls; uint8_t op, arg[16]; unsigned need, na; uint32_t units, ptr, rpt, addr;
    uint8_t cs[16];
    bool disabled, silent, locked, prog, inreset, keynvm, keyerase;
    int breaks, need_breaks;
    /* NVM controller */
    uint8_t ctrla; int busy, busy_polls; bool stuck; uint8_t err;
    uint32_t nv_addr; uint8_t nv_data;
    uint8_t pb[512], pbd[512]; int pb_reg; uint32_t pb_page;
    /* checks */
    int bad_access, bad_frame, st_words, st_bytes, cmds[64];
    long corrupt_off;              /* flash read-back flips bit 0 of this byte, -1 = none */
    uint32_t last_baud;
    /* timing model */
    bool enabled, await_sync, erasefail;
    uint32_t t_open, t_close;      /* window for the first SYNC (sim us) */
    uint32_t busy_until;           /* v2 EEPROM/fuse byte write in progress until (sim us) */
    uint32_t prog_delay_us, prog_at;
    int timing_viol, drops, pulses;
    prog_sim_t *sim;
} chip_t;

static void chip_init(chip_t *c, int nvm) {
    memset(c, 0, sizeof(*c));
    c->nvm = nvm; c->corrupt_off = -1; c->t_close = 0xFFFFFFFFu; c->busy_polls = 2; c->need_breaks = 0;
    for (int i = 0; i < R_N; i++) { c->r[i].size = 0x4000; memset(c->r[i].d, 0xFF, sizeof(c->r[i].d)); }
    if (nvm == 0) {
        memcpy(c->sib, "tinyAVR P:0D:0-3", 16);
        c->r[R_FLASH].base = 0x8000; c->r[R_FLASH].size = 0x4000;
        c->r[R_EE].base = 0x1400;    c->r[R_EE].size = 256;
        c->r[R_FUSE].base = 0x1280;  c->r[R_FUSE].size = 10;
        c->r[R_USER].base = 0x1300;  c->r[R_USER].size = 32;
        c->fpage = 64; c->epage = 32; c->upage = 32;
        c->r[R_SIG].d[0] = 0x1E; c->r[R_SIG].d[1] = 0x94; c->r[R_SIG].d[2] = 0x21;   /* ATtiny1614 */
    } else {
        memcpy(c->sib, "AVR     P:2D:1-3", 16);
        c->r[R_FLASH].base = 0x800000; c->r[R_FLASH].size = 0x4000;
        c->r[R_EE].base = 0x1400;      c->r[R_EE].size = 512;
        c->r[R_FUSE].base = 0x1050;    c->r[R_FUSE].size = 16;
        c->r[R_USER].base = 0x1080;    c->r[R_USER].size = 32;
        c->fpage = 512; c->epage = 1; c->upage = 32;
        c->r[R_SIG].d[0] = 0x1E; c->r[R_SIG].d[1] = 0x97; c->r[R_SIG].d[2] = 0x0A;   /* AVR128DA */
    }
    c->r[R_SIG].base = 0x1100; c->r[R_SIG].size = 16;
    c->cs[0] = 0x30;
    c->ps = P_SYNC;
}

static void reply(chip_t *c, const uint8_t *b, size_t n) { prog_sim_uart_reply(c->sim, b, n); }
static void ack(chip_t *c) { uint8_t a = 0x40; reply(c, &a, 1); }

static reg_t *find(chip_t *c, uint32_t addr, uint32_t *off) {
    for (int i = 0; i < R_N; i++)
        if (addr >= c->r[i].base && addr < c->r[i].base + c->r[i].size) { *off = addr - c->r[i].base; return &c->r[i]; }
    return NULL;
}

static void nvm_busy(chip_t *c) { c->busy = c->stuck ? 1 << 30 : c->busy_polls; }

static uint8_t mrd(chip_t *c, uint32_t addr) {
    if (addr >= 0x1000 && addr < 0x1010) {
        switch (addr - 0x1000) {
        case 0: return c->ctrla;
        case 2: { uint8_t v = c->err; if (c->busy > 0) { v |= 1; c->busy--; }
                  if (c->sim->us < c->busy_until) v |= 1;
                  return v; }
        case 6: return c->nv_data;
        case 8: return (uint8_t)c->nv_addr;
        case 9: return (uint8_t)(c->nv_addr >> 8);
        }
        return 0;
    }
    uint32_t off; reg_t *r = find(c, addr, &off);
    if (!r) return 0xFF;
    uint8_t v = r->d[off];
    if (r == &c->r[R_FLASH] && c->corrupt_off == (long)off) v ^= 1;
    return v;
}

/* flash-like region index for the page-sized helpers */
static uint32_t page_of(chip_t *c, int ri) { return ri == R_FLASH ? c->fpage : ri == R_USER ? c->upage : c->epage; }
static int ridx(chip_t *c, reg_t *r) { return (int)(r - c->r); }

static void pb_clear(chip_t *c) { memset(c->pb, 0xFF, sizeof(c->pb)); memset(c->pbd, 0, sizeof(c->pbd)); }

static void erase_page(chip_t *c, int ri, uint32_t page) {
    uint32_t ps = page_of(c, ri), lim = c->r[ri].size;
    for (uint32_t i = page * ps; i < (page + 1) * ps && i < lim; i++) c->r[ri].d[i] = 0xFF;
}

static void v0_command(chip_t *c, uint8_t cmd) {
    c->cmds[cmd & 63]++;
    nvm_busy(c);
    reg_t *r = c->pb_reg >= 0 ? &c->r[c->pb_reg] : NULL;
    uint32_t ps = r ? page_of(c, c->pb_reg) : 0;
    switch (cmd) {
    case 4: pb_clear(c); c->pb_reg = -1; break;                       /* PBC */
    case 1: case 3: {                                                  /* WP, ERWP */
        if (!r) { c->bad_access++; break; }
        bool flash = c->pb_reg == R_FLASH;
        if (cmd == 3 && flash) erase_page(c, c->pb_reg, c->pb_page);
        for (uint32_t i = 0; i < ps; i++) {
            uint32_t o = c->pb_page * ps + i;
            if (o >= r->size) break;
            if (flash) r->d[o] &= c->pb[i];
            else if (c->pbd[i]) { if (cmd == 3) r->d[o] = 0xFF; r->d[o] &= c->pb[i]; }
        }
        break; }
    case 5:                                                            /* CHER */
        memset(c->r[R_FLASH].d, 0xFF, sizeof(c->r[R_FLASH].d));
        memset(c->r[R_EE].d, 0xFF, sizeof(c->r[R_EE].d));
        break;
    case 7:                                                            /* WFU */
        if (c->nv_addr >= c->r[R_FUSE].base && c->nv_addr < c->r[R_FUSE].base + c->r[R_FUSE].size)
            c->r[R_FUSE].d[c->nv_addr - c->r[R_FUSE].base] = c->nv_data;
        else c->bad_access++;
        break;
    default: if (cmd) c->bad_access++;
    }
}

static void mwr1(chip_t *c, uint32_t addr, uint8_t v, int dsz) {
    if (addr >= 0x1000 && addr < 0x1010) {
        switch (addr - 0x1000) {
        case 0:
            c->ctrla = v;
            if (c->nvm == 0) v0_command(c, v);
            else {
                c->cmds[v & 63]++;
                if (v == 0x20) {                                       /* CHER */
                    memset(c->r[R_FLASH].d, 0xFF, sizeof(c->r[R_FLASH].d));
                    memset(c->r[R_EE].d, 0xFF, sizeof(c->r[R_EE].d));
                    nvm_busy(c);
                } else if (v) nvm_busy(c);
            }
            break;
        case 6: c->nv_data = v; break;
        case 8: c->nv_addr = (c->nv_addr & ~0xFFu) | v; break;
        case 9: c->nv_addr = (c->nv_addr & ~0xFF00u) | ((uint32_t)v << 8); break;
        case 10: c->nv_addr = (c->nv_addr & 0xFFFFu) | ((uint32_t)v << 16); break;
        }
        return;
    }
    uint32_t off; reg_t *r = find(c, addr, &off);
    if (!r || r == &c->r[R_SIG]) { c->bad_access++; return; }
    int ri = ridx(c, r);
    if (c->nvm == 0) {
        if (ri == R_FUSE) { c->bad_access++; return; }                 /* fuses only via WFU */
        if (ri == R_FLASH && dsz != 2) c->bad_access++;                /* word access to the flash buffer */
        uint32_t ps = page_of(c, ri);
        if (c->pb_reg < 0) c->pb_reg = ri, c->pb_page = off / ps;
        if (c->pb_reg != ri || c->pb_page != off / ps) { c->bad_access++; return; }
        c->pb[off % ps] = v; c->pbd[off % ps] = 1;
        return;
    }
    /* NVM v2: no page buffer, the command decides */
    uint8_t cmd = c->ctrla;
    bool fl = ri == R_FLASH || ri == R_USER;
    if (fl) {
        if (cmd == 0x08) { erase_page(c, ri, off / page_of(c, ri)); return; }       /* FLPER */
        if (cmd == 0x02) { if (dsz != 2 && ri == R_FLASH) c->bad_access++; r->d[off] &= v; return; }   /* FLWR */
    } else {
        if (cmd == 0x13) { r->d[off] = v; c->busy_until = c->sim->us + 10000; return; }   /* EEERWR, 10 ms */
        if (cmd == 0x12) { r->d[off] &= v; return; }                                  /* EEWR */
    }
    c->bad_access++;
}

static void mwr(chip_t *c, uint32_t addr, const uint8_t *p, int dsz) {
    for (int i = 0; i < dsz; i++) mwr1(c, addr + i, p[i], dsz);
}

static void unlock_erase(chip_t *c) {
    memset(c->r[R_FLASH].d, 0xFF, sizeof(c->r[R_FLASH].d));
    memset(c->r[R_EE].d, 0xFF, sizeof(c->r[R_EE].d));
    memset(c->r[R_USER].d, 0xFF, sizeof(c->r[R_USER].d));
    c->locked = false; c->keyerase = false; c->cs[7] &= ~0x08;
}

static void stcs(chip_t *c, unsigned reg, uint8_t v) {
    if (reg == 8) {
        if (v == 0x59) c->inreset = true;
        else if (v == 0 && c->inreset) {
            c->inreset = false;
            if (c->keyerase && c->erasefail) c->keyerase = false;      /* refused: stays locked, ERASEFAIL */
            else if (c->keyerase) unlock_erase(c);
            else if (c->keynvm && !c->locked) { c->prog = true; c->prog_at = c->sim->us + c->prog_delay_us; }
            c->keynvm = false;
            if (!c->prog) c->cs[7] &= ~0x10;
        }
        return;
    }
    if (reg == 3 && (v & 4)) { c->disabled = true; c->prog = false; }
    c->cs[reg & 15] = v;
}

static void args_done(chip_t *c) {
    c->na = 0;
    switch (c->cls) {
    case C_LDS: {
        unsigned asz = ((c->op >> 2) & 3) + 1, dsz = (c->op & 3) + 1;
        uint32_t a = 0; for (unsigned i = 0; i < asz; i++) a |= (uint32_t)c->arg[i] << (8 * i);
        uint8_t o[4]; for (unsigned i = 0; i < dsz; i++) o[i] = mrd(c, a + i);
        reply(c, o, dsz); c->ps = P_SYNC; break; }
    case C_STS: {
        unsigned asz = ((c->op >> 2) & 3) + 1;
        c->addr = 0; for (unsigned i = 0; i < asz; i++) c->addr |= (uint32_t)c->arg[i] << (8 * i);
        ack(c);
        c->cls = C_STS_DATA; c->need = (c->op & 3) + 1; c->units = 1; c->ps = P_DATA; break; }
    case C_STPTR: {
        unsigned n = (c->op & 3) + 1;
        c->ptr = 0; for (unsigned i = 0; i < n; i++) c->ptr |= (uint32_t)c->arg[i] << (8 * i);
        ack(c); c->ps = P_SYNC; break; }
    case C_STCS: stcs(c, c->op & 15, c->arg[0]); c->ps = P_SYNC; break;
    case C_RPT: c->rpt = (uint32_t)c->arg[0] + 1; c->ps = P_SYNC; break;
    case C_KEY: {
        char k[9]; for (int i = 0; i < 8; i++) k[i] = (char)c->arg[7 - i]; k[8] = 0;
        /* bench (ATtiny1607): a chip already in programming mode ignores the key, KEY_STATUS reads 0 */
        if (!strcmp(k, "NVMProg ")) { if (!c->prog) { c->keynvm = true; c->cs[7] |= 0x10; } }
        else if (!strcmp(k, "NVMErase")) { c->keyerase = true; c->cs[7] |= 0x08; }
        c->ps = P_SYNC; break; }
    }
}

/* A store while the previous byte is still being written: with ACKs the UPDI holds the byte (and the
   ACK) until the write is done; streamed without ACKs the byte is lost. true = dropped. */
static bool store_while_busy(chip_t *c) {
    if (c->sim->us >= c->busy_until) return false;
    if (c->cs[2] & 8) { c->drops++; return true; }
    c->sim->us = c->busy_until;
    return false;
}

static void data_done(chip_t *c) {
    unsigned dsz = (c->op & 3) + 1;
    c->na = 0;
    if (c->cls == C_STS_DATA) {
        if (store_while_busy(c)) { c->ps = P_SYNC; return; }
        mwr(c, c->addr, c->arg, (int)dsz);
        if (!(c->cs[2] & 8)) ack(c);
        c->ps = P_SYNC; return;
    }
    /* ST ptr++ with data */
    if (store_while_busy(c)) { c->ptr += dsz; if (--c->units == 0) c->ps = P_SYNC; return; }
    if (dsz == 2) c->st_words++; else c->st_bytes++;
    mwr(c, c->ptr, c->arg, (int)dsz);
    c->ptr += dsz;
    if (!(c->cs[2] & 8)) ack(c);
    if (--c->units == 0) c->ps = P_SYNC;
}

static void decode_op(chip_t *c) {
    uint8_t op = c->op;
    c->ps = P_SYNC; c->na = 0;
    switch (op >> 4) {
    case 0x0: c->cls = C_LDS; c->need = ((op >> 2) & 3) + 1; c->ps = P_ARGS; break;
    case 0x4: c->cls = C_STS; c->need = ((op >> 2) & 3) + 1; c->ps = P_ARGS; break;
    case 0x2: {
        unsigned mode = (op >> 2) & 3, dsz = (op & 3) + 1;
        uint32_t units = c->rpt ? c->rpt : 1; c->rpt = 0;
        if (mode == 2) { uint8_t o[3] = { (uint8_t)c->ptr, (uint8_t)(c->ptr >> 8), (uint8_t)(c->ptr >> 16) }; reply(c, o, dsz); break; }
        if (c->inreset) break;   /* the data bus is held in reset: a memory load gets no answer (AVR128DA48 bench) */
        for (uint32_t u = 0; u < units; u++) {
            uint8_t o[4]; for (unsigned i = 0; i < dsz; i++) o[i] = mrd(c, c->ptr + i);
            reply(c, o, dsz);
            if (mode == 1) c->ptr += dsz;
        }
        break; }
    case 0x6: {
        unsigned mode = (op >> 2) & 3;
        if (mode == 2) { c->cls = C_STPTR; c->need = (op & 3) + 1; c->ps = P_ARGS; break; }
        c->cls = C_ST; c->need = (op & 3) + 1; c->units = c->rpt ? c->rpt : 1; c->rpt = 0; c->ps = P_DATA;
        break; }
    case 0x8: {
        unsigned reg = op & 15; uint8_t v = c->cs[reg];
        if (reg == 0x0B) v = (uint8_t)((c->locked ? 1 : 0) | ((c->prog && c->sim->us >= c->prog_at) ? 8 : 0) |
                                       (c->inreset ? 0x20 : 0) | (c->erasefail ? 0x40 : 0));
        reply(c, &v, 1); break; }
    case 0xC: c->cls = C_STCS; c->need = 1; c->ps = P_ARGS; break;
    case 0xA: c->cls = C_RPT; c->need = 1; c->ps = P_ARGS; break;
    case 0xE:
        if (op & 4) reply(c, (const uint8_t *)c->sib, 16);
        else { c->cls = C_KEY; c->need = 8u << (op & 3); c->ps = P_ARGS; }
        break;
    default: c->bad_frame++;
    }
}

static void feed(chip_t *c, uint8_t b) {
    switch (c->ps) {
    case P_SYNC: if (b == 0x55) c->ps = P_OP; else c->bad_frame++; break;
    case P_OP: c->op = b; decode_op(c); break;
    case P_ARGS: c->arg[c->na++] = b; if (c->na == c->need) args_done(c); break;
    case P_DATA: c->arg[c->na++] = b; if (c->na == c->need) data_done(c); break;
    }
}

static void model_rx(prog_sim_t *s, const uint8_t *b, size_t n, uint32_t baud, bool pe, bool s2) {
    chip_t *c = s->chip;
    if (baud == 300) {
        for (size_t i = 0; i < n; i++) if (b[i] == 0) {
            uint32_t end = s->us + 12u * 1000000u / 300u;   /* the frame (8E2 = 12 bits) is still going */
            c->breaks++; c->ps = P_SYNC; c->disabled = false; c->cs[2] = c->cs[3] = 0; c->rpt = 0;
            c->enabled = true; c->await_sync = true;
            c->t_open = 0; c->t_close = end + 5000u;        /* SYNC within a few ms of the BREAK's end */
        }
        return;
    }
    c->last_baud = baud;
    if (!pe || !s2) c->bad_frame++;
    if (c->silent) return;
    size_t i = 0;
    if (!c->enabled) {                  /* a low pulse (0x00) at the link rate enables the UPDI */
        if (b[0] != 0) return;
        c->enabled = true; c->await_sync = true; c->pulses++;
        uint32_t end = s->us + 12u * 1000000u / baud;
        c->t_open = end + 200; c->t_close = end + 14000;
        i = 1;
    }
    if (i < n && c->await_sync) {       /* the first SYNC: inside the window or the UPDI disables itself */
        if (s->us < c->t_open || s->us > c->t_close) {
            c->timing_viol++; c->enabled = false; c->await_sync = false; return;
        }
        c->await_sync = false;
    }
    if (c->disabled || c->breaks < c->need_breaks) return;
    for (; i < n; i++) feed(c, b[i]);
}

static const prog_sim_model_t MODEL = { .on_uart_rx = model_rx };

/* ---- harness ---- */

typedef struct { uint8_t img[0x4000]; uint8_t out[0x4000]; int src_fail; } job_t;

static int job_src(void *j, uint32_t off, uint8_t *buf, uint32_t n) {
    job_t *jb = j;
    if (jb->src_fail || off + n > sizeof(jb->img)) return -1;
    memcpy(buf, jb->img + off, n); return 0;
}
static int job_dst(void *j, uint32_t off, const uint8_t *buf, uint32_t n) {
    job_t *jb = j;
    if (off + n > sizeof(jb->out)) return -1;
    memcpy(jb->out + off, buf, n); return 0;
}

typedef struct {
    chip_t chip; prog_sim_t sim; prog_wire_ops_t ops; prog_ctx_t ctx; job_t job;
} rig_t;

static rig_t *rig_new(int nvm) {
    rig_t *r = calloc(1, sizeof(*r));
    chip_init(&r->chip, nvm);
    prog_sim_init(&r->sim, &r->ops, &MODEL, &r->chip);
    r->chip.sim = &r->sim;
    r->ctx.w = &r->ops; r->ctx.src = job_src; r->ctx.dst = job_dst; r->ctx.job = &r->job;
    r->ctx.page = nvm == 0 ? 64 : 512;
    return r;
}

typedef int (*step_fn)(prog_ctx_t *);
static int run(prog_ctx_t *c, step_fn f) {
    for (int i = 0; i < 200000; i++) { int rc = f(c); if (rc != PROG_RUNNING) return rc; }
    return 12345;   /* never finished */
}

static void fill(uint8_t *b, size_t n, uint8_t seed) { for (size_t i = 0; i < n; i++) b[i] = (uint8_t)(seed + i * 7 + (i >> 8)); }

/* ---- tests ---- */

static void test_model_raw(void) {
    /* The model alone, with raw frames: LDCS STATUSA answers, SIB reads, a silent chip stays silent. */
    rig_t *r = rig_new(0);
    prog_sim_t *s = &r->sim;
    const uint8_t ldcs[] = { 0x55, 0x80 };
    r->chip.enabled = true;
    r->ops.uart_write(s, ldcs, sizeof(ldcs));
    uint8_t rx[4]; size_t n = r->ops.uart_read(s, rx, 3, 1);
    CHECK_EQ(n, 1); CHECK_EQ(rx[0], 0x30);   /* the reply only: the pod never reads its own bytes */
    const uint8_t sib[] = { 0x55, 0xE5 };
    r->ops.uart_write(s, sib, sizeof(sib));
    uint8_t b[18]; n = r->ops.uart_read(s, b, 18, 1);
    CHECK_EQ(n, 16); CHECK(!memcmp(b, "tinyAVR P:0D:0-3", 16));
    free(r);
    r = rig_new(2); r->chip.silent = true; r->chip.enabled = true;
    r->ops.uart_write(&r->sim, ldcs, sizeof(ldcs));
    n = r->ops.uart_read(&r->sim, rx, 3, 1);
    CHECK_EQ(n, 0);   /* silent: nothing comes back */
    free(r);
}

static void check_entry(int nvm, uint8_t sig1, uint8_t sig2) {
    rig_t *r = rig_new(nvm);
    const prog_proto_t *p = &prog_proto_updi;
    CHECK_EQ(run(&r->ctx, p->enter), PROG_OK);
    CHECK_EQ(r->ctx.id_len, 3);
    CHECK_EQ(r->ctx.id[0], 0x1E); CHECK_EQ(r->ctx.id[1], sig1); CHECK_EQ(r->ctx.id[2], sig2);
    CHECK(r->chip.prog);
    CHECK_EQ(r->sim.baud, 115200); CHECK(r->sim.parity_even && r->sim.stop2);
    CHECK_EQ(r->chip.breaks, 0);
    CHECK_EQ(r->chip.bad_frame, 0);
    CHECK_EQ(r->chip.pulses, 1);                 /* one enable pulse, SYNC inside the TDebZ window */
    CHECK_EQ(r->chip.timing_viol, 0);
    p->leave(&r->ctx, true);
    CHECK(r->chip.disabled); CHECK(!r->chip.inreset);
    free(r);
    /* run = false keeps the chip in reset with UPDI still enabled */
    r = rig_new(nvm);
    CHECK_EQ(run(&r->ctx, p->enter), PROG_OK);
    p->leave(&r->ctx, false);
    CHECK(!r->chip.disabled); CHECK(r->chip.inreset);
    /* the next job finds it still in programming mode (a failed job's leave keeps it there): it
       skips the key, which the chip would ignore, and reads the ID */
    memset(&r->ctx.state, 0, sizeof(r->ctx.state)); r->ctx.id_len = 0;
    r->chip.cs[7] = 0;   /* KEY_STATUS read 0x00 on the bench in this state */
    CHECK_EQ(run(&r->ctx, p->enter), PROG_OK);
    CHECK_EQ(r->ctx.id_len, 3); CHECK_EQ(r->ctx.id[1], sig1);
    free(r);
}

static void test_entry(void) {
    check_entry(0, 0x94, 0x21);
    check_entry(2, 0x97, 0x0A);
    /* the chip needs a break first: double break at 300 baud, then the link baud again */
    rig_t *r = rig_new(0);
    r->chip.need_breaks = 1;
    r->ctx.baud = 225000;
    CHECK_EQ(run(&r->ctx, prog_proto_updi.enter), PROG_OK);
    CHECK_EQ(r->chip.breaks, 2);
    CHECK_EQ(r->chip.timing_viol, 0);            /* SYNC right after the second BREAK */
    CHECK_EQ(r->sim.baud, 225000);
    CHECK_EQ(r->chip.last_baud, 225000);
    free(r);
    /* silent chip */
    r = rig_new(0); r->chip.silent = true;
    CHECK_EQ(run(&r->ctx, prog_proto_updi.enter), PROG_E_NO_ANSWER);
    CHECK_EQ(r->chip.breaks, 2);   /* the pod tried the double break before giving up */
    prog_proto_updi.leave(&r->ctx, true);   /* never linked: sends nothing */
    free(r);
    /* locked, no chip erase asked */
    r = rig_new(2); r->chip.locked = true;
    CHECK_EQ(run(&r->ctx, prog_proto_updi.enter), PROG_E_LOCKED);
    CHECK(r->chip.locked);
    prog_proto_updi.leave(&r->ctx, false);
    free(r);
    /* locked, chip erase asked: unlocks, erases, reads the ID */
    r = rig_new(0); r->chip.locked = true; fill(r->chip.r[R_FLASH].d, 256, 3);
    r->ctx.erase = PROG_ERASE_CHIP;
    CHECK_EQ(run(&r->ctx, prog_proto_updi.enter), PROG_OK);
    CHECK(!r->chip.locked); CHECK(r->chip.prog);
    CHECK_EQ(r->chip.r[R_FLASH].d[5], 0xFF);
    CHECK_EQ(r->ctx.id[1], 0x94);
    free(r);
    /* unsupported NVM version, wrong baud */
    r = rig_new(0); memcpy(r->chip.sib, "AVR     P:3D:1-3", 16);
    CHECK_EQ(run(&r->ctx, prog_proto_updi.enter), PROG_E_UNSUPPORTED);
    free(r);
    r = rig_new(0); r->ctx.baud = 300000;
    CHECK_EQ(run(&r->ctx, prog_proto_updi.enter), PROG_E_ARGS);
    free(r);
}

static bool same(const uint8_t *a, const uint8_t *b, uint32_t n) { return !memcmp(a, b, n); }
static int do_write(rig_t *r, int mem, uint32_t addr, uint32_t len, int erase, uint16_t page);
static int do_write_fwd(rig_t *r, uint32_t len) { return do_write(r, PROG_MEM_FLASH, 0, len, PROG_ERASE_CHIP, 64); }

static void test_entry_timing(void) {
    /* NVMPROG shows up 5 ms after the reset release: the host polls instead of reading once */
    rig_t *r = rig_new(2); r->chip.prog_delay_us = 5000;
    CHECK_EQ(run(&r->ctx, prog_proto_updi.enter), PROG_OK);
    CHECK_EQ(r->ctx.id[1], 0x97);
    free(r);
    /* it never shows up (unlocked, key not accepted): protocol error after the 100 ms window */
    r = rig_new(0); r->chip.prog_delay_us = 500000;
    CHECK_EQ(run(&r->ctx, prog_proto_updi.enter), PROG_E_PROTOCOL);
    free(r);
    /* the key erase of a locked chip fails: ERASEFAIL is its own error, not a timeout */
    r = rig_new(2); r->chip.locked = true; r->chip.erasefail = true; r->ctx.erase = PROG_ERASE_CHIP;
    CHECK_EQ(run(&r->ctx, prog_proto_updi.enter), PROG_E_LOCKED);
    CHECK(!strcmp(r->ctx.phase, "erase"));
    CHECK(r->chip.locked);
    free(r);
    /* megaAVR 0: the family string selects the 0x4000 flash base, no flag needed */
    r = rig_new(0);
    memcpy(r->chip.sib, "megaAVR P:0D:0-3", 16);
    r->chip.r[R_FLASH].base = 0x4000;
    CHECK_EQ(do_write_fwd(r, 128), PROG_OK);
    CHECK(same(r->chip.r[R_FLASH].d, r->job.img, 128));
    CHECK_EQ(r->chip.bad_access, 0);
    free(r);
    /* a flag overrides the family default */
    r = rig_new(0);
    memcpy(r->chip.sib, "megaAVR P:0D:0-3", 16);
    r->chip.r[R_FLASH].base = 0x8000; r->ctx.flags = 0x8000;
    CHECK_EQ(do_write_fwd(r, 128), PROG_OK);
    CHECK(same(r->chip.r[R_FLASH].d, r->job.img, 128));
    free(r);
}

static void check_chip_erase(int nvm) {
    rig_t *r = rig_new(nvm);
    const prog_proto_t *p = &prog_proto_updi;
    fill(r->chip.r[R_FLASH].d, 0x4000, 9); fill(r->chip.r[R_EE].d, 256, 5);
    fill(r->chip.r[R_USER].d, 32, 1); fill(r->chip.r[R_FUSE].d, 8, 2);
    r->ctx.erase = PROG_ERASE_CHIP;
    CHECK_EQ(run(&r->ctx, p->enter), PROG_OK);
    CHECK_EQ(run(&r->ctx, p->erase), PROG_OK);
    for (int i = 0; i < 0x4000; i++) if (r->chip.r[R_FLASH].d[i] != 0xFF) { CHECK(0); break; }
    for (int i = 0; i < 256; i++) if (r->chip.r[R_EE].d[i] != 0xFF) { CHECK(0); break; }
    CHECK(r->chip.r[R_FUSE].d[0] != 0xFF);        /* fuses survive a chip erase */
    CHECK_EQ(r->chip.cmds[nvm == 0 ? 5 : 0x20], 1);
    if (nvm == 2) CHECK_EQ(r->chip.ctrla, 0);      /* v2: command cleared afterwards */
    CHECK_EQ(r->chip.bad_access, 0);
    free(r);
    /* erase = none: nothing is sent to the NVM controller */
    r = rig_new(nvm);
    fill(r->chip.r[R_FLASH].d, 64, 9);
    r->ctx.erase = PROG_ERASE_NONE;
    CHECK_EQ(run(&r->ctx, p->enter), PROG_OK);
    CHECK_EQ(run(&r->ctx, p->erase), PROG_OK);
    CHECK(r->chip.r[R_FLASH].d[3] != 0xFF);
    free(r);
    /* the NVM controller stays busy: timeout */
    r = rig_new(nvm); r->ctx.erase = PROG_ERASE_CHIP;
    CHECK_EQ(run(&r->ctx, p->enter), PROG_OK);
    r->chip.stuck = true;
    CHECK_EQ(run(&r->ctx, p->erase), PROG_E_TIMEOUT);
    free(r);
}

static void test_erase(void) {
    check_chip_erase(0);
    check_chip_erase(2);
    /* a chip unlocked with the erase key in enter() needs no second erase */
    rig_t *r = rig_new(2); r->chip.locked = true; r->ctx.erase = PROG_ERASE_CHIP;
    CHECK_EQ(run(&r->ctx, prog_proto_updi.enter), PROG_OK);
    CHECK_EQ(run(&r->ctx, prog_proto_updi.erase), PROG_OK);
    CHECK_EQ(r->chip.cmds[0x20], 0);
    free(r);
}

/* Enter, erase as asked, write [addr, addr+len) of `mem` from a patterned image, run to the end. */
static int do_write(rig_t *r, int mem, uint32_t addr, uint32_t len, int erase, uint16_t page) {
    const prog_proto_t *p = &prog_proto_updi;
    fill(r->job.img, sizeof(r->job.img), 0x11);
    r->ctx.mem = (uint8_t)mem; r->ctx.addr = addr; r->ctx.len = len; r->ctx.erase = (uint8_t)erase; r->ctx.page = page;
    int rc = run(&r->ctx, p->enter);
    if (rc) return rc;
    rc = run(&r->ctx, p->erase);
    if (rc) return rc;
    return run(&r->ctx, p->write);
}

static void test_write(void) {
    rig_t *r;
    /* tinyAVR: flash from a dirty chip, chip erase, 3 pages with a short last one */
    r = rig_new(0); fill(r->chip.r[R_FLASH].d, 0x4000, 77);
    CHECK_EQ(do_write(r, PROG_MEM_FLASH, 0, 150, PROG_ERASE_CHIP, 64), PROG_OK);
    CHECK(same(r->chip.r[R_FLASH].d, r->job.img, 150));
    CHECK_EQ(r->chip.r[R_FLASH].d[150], 0xFF);
    CHECK_EQ(r->ctx.total, 3); CHECK_EQ(r->ctx.done, 3);
    CHECK_EQ(r->chip.bad_access, 0); CHECK_EQ(r->chip.bad_frame, 0);
    CHECK_EQ(r->chip.st_bytes, 0);                      /* flash only by words */
    CHECK_EQ(r->chip.cmds[1], 3);                       /* WP per page */
    free(r);
    /* tinyAVR: page erase on an unaligned region: neighbours stay, written pages are erased first */
    r = rig_new(0); fill(r->chip.r[R_FLASH].d, 0x4000, 77);
    uint8_t keep[0x4000]; memcpy(keep, r->chip.r[R_FLASH].d, sizeof(keep));
    CHECK_EQ(do_write(r, PROG_MEM_FLASH, 70, 100, PROG_ERASE_PAGE, 64), PROG_OK);   /* pages 1 and 2 */
    CHECK(same(r->chip.r[R_FLASH].d + 70, r->job.img, 100));
    CHECK(same(r->chip.r[R_FLASH].d, keep, 64));                  /* page 0 untouched */
    CHECK(same(r->chip.r[R_FLASH].d + 192, keep + 192, 64));      /* page 3 untouched */
    CHECK_EQ(r->chip.r[R_FLASH].d[64], 0xFF);                     /* start of page 1, before the data */
    CHECK_EQ(r->ctx.total, 2);
    CHECK_EQ(r->chip.cmds[3], 2);                                 /* ERWP */
    free(r);
    /* tinyAVR: odd length pads the last word with 0xFF */
    r = rig_new(0);
    CHECK_EQ(do_write(r, PROG_MEM_FLASH, 0, 5, PROG_ERASE_CHIP, 64), PROG_OK);
    CHECK(same(r->chip.r[R_FLASH].d, r->job.img, 5)); CHECK_EQ(r->chip.r[R_FLASH].d[5], 0xFF);
    free(r);
    /* tinyAVR: eeprom (page 32, offset 8, 40 bytes), user row, fuses */
    r = rig_new(0); fill(r->chip.r[R_EE].d, 256, 4);
    CHECK_EQ(do_write(r, PROG_MEM_EEPROM, 8, 40, PROG_ERASE_NONE, 32), PROG_OK);
    CHECK(same(r->chip.r[R_EE].d + 8, r->job.img, 40));
    CHECK_EQ(r->chip.r[R_EE].d[7], (uint8_t)(4 + 7 * 7));      /* neighbours stay */
    CHECK_EQ(r->chip.r[R_EE].d[48], (uint8_t)(4 + 48 * 7));
    CHECK_EQ(r->chip.bad_access, 0);
    free(r);
    r = rig_new(0); fill(r->chip.r[R_USER].d, 32, 6);
    CHECK_EQ(do_write(r, PROG_MEM_USER, 0, 32, PROG_ERASE_NONE, 32), PROG_OK);
    CHECK(same(r->chip.r[R_USER].d, r->job.img, 32));
    free(r);
    r = rig_new(0);
    CHECK_EQ(do_write(r, PROG_MEM_CONFIG, 1, 3, PROG_ERASE_NONE, 0), PROG_OK);
    CHECK(same(r->chip.r[R_FUSE].d + 1, r->job.img, 3));
    CHECK_EQ(r->chip.r[R_FUSE].d[0], 0xFF);
    CHECK_EQ(r->ctx.total, 3);
    CHECK_EQ(r->chip.bad_access, 0);
    free(r);
    /* AVR DA: flash 1200 bytes in 512-byte pages (256 words per REPEAT), page erase */
    r = rig_new(2); fill(r->chip.r[R_FLASH].d, 0x4000, 77);
    CHECK_EQ(do_write(r, PROG_MEM_FLASH, 0, 1200, PROG_ERASE_PAGE, 512), PROG_OK);
    CHECK(same(r->chip.r[R_FLASH].d, r->job.img, 1200));
    CHECK_EQ(r->chip.r[R_FLASH].d[1200], 0xFF);                   /* rest of the last page is erased */
    CHECK_EQ(r->ctx.total, 3);
    CHECK_EQ(r->chip.bad_access, 0); CHECK_EQ(r->chip.st_bytes, 0);
    CHECK_EQ(r->chip.ctrla, 0);
    free(r);
    /* AVR DA: flash after a chip erase writes without a page erase */
    r = rig_new(2);
    CHECK_EQ(do_write(r, PROG_MEM_FLASH, 512, 512, PROG_ERASE_CHIP, 512), PROG_OK);
    CHECK(same(r->chip.r[R_FLASH].d + 512, r->job.img, 512));
    CHECK_EQ(r->chip.cmds[8], 0);
    free(r);
    /* AVR DA: eeprom, fuses, user row (user row is erased first) */
    r = rig_new(2); fill(r->chip.r[R_EE].d, 512, 4);
    CHECK_EQ(do_write(r, PROG_MEM_EEPROM, 10, 37, PROG_ERASE_NONE, 1), PROG_OK);
    CHECK(same(r->chip.r[R_EE].d + 10, r->job.img, 37));
    CHECK_EQ(r->ctx.total, 37);
    CHECK_EQ(r->chip.drops, 0);                  /* ACKed byte stores: none lost to the 10 ms writes */
    CHECK_EQ(r->chip.st_words, 0);               /* no word streaming */
    free(r);
    /* AVR DA: eeprom in 16-byte chunks: several 10 ms byte writes per command, must not drop bytes */
    r = rig_new(2); fill(r->chip.r[R_EE].d, 512, 4);
    CHECK_EQ(do_write(r, PROG_MEM_EEPROM, 10, 37, PROG_ERASE_NONE, 16), PROG_OK);
    CHECK(same(r->chip.r[R_EE].d + 10, r->job.img, 37));
    CHECK_EQ(r->chip.drops, 0);
    CHECK_EQ(r->chip.bad_access, 0);
    free(r);
    r = rig_new(2);
    CHECK_EQ(do_write(r, PROG_MEM_CONFIG, 0, 9, PROG_ERASE_NONE, 0), PROG_OK);
    CHECK(same(r->chip.r[R_FUSE].d, r->job.img, 9));
    CHECK_EQ(r->chip.drops, 0);
    free(r);
    r = rig_new(2); fill(r->chip.r[R_USER].d, 32, 6);
    CHECK_EQ(do_write(r, PROG_MEM_USER, 0, 32, PROG_ERASE_NONE, 32), PROG_OK);
    CHECK(same(r->chip.r[R_USER].d, r->job.img, 32));
    CHECK_EQ(r->chip.bad_access, 0);
    CHECK_EQ(r->chip.drops, 0);
    CHECK_EQ(r->chip.st_words, 0);               /* user row bytes are ACKed, not streamed */
    free(r);
    /* verify failure: the chip reads one byte back wrong */
    r = rig_new(0); r->chip.corrupt_off = 100;
    CHECK_EQ(do_write(r, PROG_MEM_FLASH, 64, 100, PROG_ERASE_PAGE, 64), PROG_E_VERIFY);
    CHECK_EQ(r->ctx.at_addr, 100);
    free(r);
    r = rig_new(2); r->chip.corrupt_off = 600;
    CHECK_EQ(do_write(r, PROG_MEM_FLASH, 0, 1000, PROG_ERASE_PAGE, 512), PROG_E_VERIFY);
    CHECK_EQ(r->ctx.at_addr, 600);
    free(r);
    /* NVM stays busy after the page write: timeout */
    r = rig_new(0); r->chip.busy_polls = 1 << 30;
    CHECK_EQ(do_write(r, PROG_MEM_FLASH, 0, 64, PROG_ERASE_CHIP, 64), PROG_E_TIMEOUT);
    free(r);
    /* NVM write error flag */
    r = rig_new(2); r->chip.err = 0x10;
    CHECK_EQ(do_write(r, PROG_MEM_FLASH, 0, 64, PROG_ERASE_NONE, 512), PROG_E_PROTOCOL);
    free(r);
    /* the staged image cannot be read */
    r = rig_new(0); r->job.src_fail = 1;
    CHECK_EQ(do_write(r, PROG_MEM_FLASH, 0, 64, PROG_ERASE_NONE, 64), PROG_E_SRC);
    free(r);
}

static void test_read(void) {
    for (int nvm = 0; nvm <= 2; nvm += 2) {
        rig_t *r = rig_new(nvm);
        const prog_proto_t *p = &prog_proto_updi;
        fill(r->chip.r[R_FLASH].d, 0x4000, 33); fill(r->chip.r[R_EE].d, 256, 44);
        fill(r->chip.r[R_FUSE].d, 8, 55); fill(r->chip.r[R_USER].d, 32, 66);
        CHECK_EQ(run(&r->ctx, p->enter), PROG_OK);
        struct { int mem; uint32_t addr, len; reg_t *reg; } t[] = {
            { PROG_MEM_FLASH, 0, 1000, &r->chip.r[R_FLASH] },
            { PROG_MEM_FLASH, 130, 700, &r->chip.r[R_FLASH] },    /* not a multiple of the block size */
            { PROG_MEM_EEPROM, 5, 200, &r->chip.r[R_EE] },
            { PROG_MEM_CONFIG, 0, 8, &r->chip.r[R_FUSE] },
            { PROG_MEM_USER, 4, 20, &r->chip.r[R_USER] },
        };
        for (unsigned i = 0; i < sizeof(t) / sizeof(t[0]); i++) {
            memset(r->job.out, 0xAA, sizeof(r->job.out));
            r->ctx.mem = (uint8_t)t[i].mem; r->ctx.addr = t[i].addr; r->ctx.len = t[i].len; r->ctx.page = 0;
            CHECK_EQ(run(&r->ctx, p->read), PROG_OK);
            CHECK(same(r->job.out, t[i].reg->d + t[i].addr, t[i].len));
            CHECK_EQ(r->job.out[t[i].len], 0xAA);
            CHECK_EQ(r->ctx.total, (t[i].len + 255) / 256); CHECK_EQ(r->ctx.done, r->ctx.total);
        }
        CHECK_EQ(r->chip.bad_frame, 0);
        free(r);
    }
    /* write then read back through the protocol */
    rig_t *r = rig_new(2);
    CHECK_EQ(do_write(r, PROG_MEM_FLASH, 0, 700, PROG_ERASE_CHIP, 512), PROG_OK);
    r->ctx.len = 700;
    CHECK_EQ(run(&r->ctx, prog_proto_updi.read), PROG_OK);
    CHECK(same(r->job.out, r->job.img, 700));
    /* the chip stops answering in the middle of a read */
    r->chip.silent = true;
    CHECK_EQ(run(&r->ctx, prog_proto_updi.read), PROG_E_NO_ANSWER);
    free(r);
    /* the read-back area cannot be written */
    r = rig_new(0);
    CHECK_EQ(run(&r->ctx, prog_proto_updi.enter), PROG_OK);
    r->ctx.mem = PROG_MEM_FLASH; r->ctx.addr = 0; r->ctx.len = 0x9000;
    CHECK_EQ(run(&r->ctx, prog_proto_updi.read), PROG_E_ARGS);   /* 0x8000 + 0x9000 does not fit 16 bits */
    r->ctx.len = 0x4800;
    CHECK_EQ(run(&r->ctx, prog_proto_updi.read), PROG_E_SRC);    /* out[] is 0x4000 */
    free(r);
}

static void test_refusals(void) {
    rig_t *r;
    r = rig_new(0);
    CHECK_EQ(do_write(r, PROG_MEM_FLASH, 0, 64, PROG_ERASE_NONE, 0), PROG_E_ARGS);      /* no page size */
    r->ctx.page = 64; r->ctx.addr = 1;
    CHECK_EQ(run(&r->ctx, prog_proto_updi.write), PROG_E_ARGS);                          /* odd flash address */
    r->ctx.addr = 0; r->ctx.page = 1024;
    CHECK_EQ(run(&r->ctx, prog_proto_updi.write), PROG_E_ARGS);                          /* page too big */
    r->ctx.page = 64; r->ctx.mem = PROG_MEM_COUNT;
    CHECK_EQ(run(&r->ctx, prog_proto_updi.write), PROG_E_ARGS);                          /* unknown memory */
    r->ctx.mem = PROG_MEM_FLASH; r->ctx.len = 0x9000;
    CHECK_EQ(run(&r->ctx, prog_proto_updi.write), PROG_E_ARGS);                          /* past the 16-bit space */
    r->ctx.len = 0;
    CHECK_EQ(run(&r->ctx, prog_proto_updi.write), PROG_OK);                              /* nothing to write */
    CHECK_EQ(r->chip.cmds[1] + r->chip.cmds[3] + r->chip.cmds[4], 0);                    /* nothing reached the NVM */
    free(r);
    /* a megaAVR 0 with 48 KB flash has it at 0x4000: the host passes that in flags */
    r = rig_new(0);
    r->chip.r[R_FLASH].base = 0x4000;
    r->ctx.flags = 0x4000;
    CHECK_EQ(do_write(r, PROG_MEM_FLASH, 0, 128, PROG_ERASE_CHIP, 64), PROG_OK);
    CHECK(same(r->chip.r[R_FLASH].d, r->job.img, 128));
    free(r);
    /* the chip disappears during a write */
    r = rig_new(2);
    r->ctx.mem = PROG_MEM_FLASH; r->ctx.len = 1024; r->ctx.page = 512;
    CHECK_EQ(run(&r->ctx, prog_proto_updi.enter), PROG_OK);
    r->chip.silent = true;
    CHECK_EQ(run(&r->ctx, prog_proto_updi.write), PROG_E_NO_ANSWER);
    prog_proto_updi.leave(&r->ctx, false);     /* must not hang */
    free(r);
}

int main(void) {
    test_model_raw();
    test_entry();
    test_entry_timing();
    test_erase();
    test_write();
    test_read();
    test_refusals();
    if (fails) { printf("%d of %d checks FAILED\n", fails, checks); return 1; }
    printf("test_prog_updi: %d checks passed\n", checks);
    return 0;
}
