/* Host test for prog_pic.c against bit-level models of PIC chips (prog_sim.h).
 *
 * The model latches data on the falling clock edge, drives read data after the rising edge, takes
 * the LVP key while MCLR is low, and checks the spec's waits (entry hold, program, erase) against
 * the simulated time. Three chips: K_A (6-bit commands, PIC16F1xxx), K_B16 (8-bit commands,
 * PIC16F18xxx) and K_B18 (8-bit commands, byte PC, PIC18 Q). */
#include "prog_flash.h"
#include "prog_pic.h"
#include "prog_sim.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static int failures;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); failures++; } } while (0)

#define KEY 0x4D434850u
#define NW  512u        /* flash words */
#define EESZ 64u

enum { K_A, K_B16, K_B18 };
enum { CELL_FL, CELL_CFG, CELL_EE, CELL_RO };

typedef struct {
    int kind; bool set8, byt;
    uint32_t key; bool deaf;
    uint16_t mask; unsigned rw;
    uint32_t cp_pc; int cp_bit;
    uint16_t fl[NW], cfgw[32], ee[EESZ], rev, dev;
    uint16_t lat[64]; uint8_t latv[64]; uint16_t lat1; bool ld_data;
    int fault_idx;                       /* flash word index read back with bit 0 flipped, -1 none */
    /* engine */
    int prev_clk; bool nrst_low, prog, first_cmd, first_rise;
    uint32_t keyreg; int nkey; uint32_t last_key;
    int st; int clen, plen, k; uint32_t sh; uint8_t cmd;
    uint32_t frame; int out;
    uint32_t pc;
    uint32_t idle_us, busy_need;
    unsigned timing_err, bad_cmds, ran, entries, bulk, rowerase, programs;
} chip_t;

enum { S_CMD, S_PW, S_PR };

static uint16_t *cell(chip_t *c, uint32_t pc, int data_sp, uint16_t *mask, int *kind) {
    *mask = c->mask; *kind = CELL_CFG;
    if (!c->byt) {
        if (c->kind == K_A && data_sp) {
            if (pc >= EESZ) return NULL;
            *mask = 0xFF; *kind = CELL_EE; return &c->ee[pc];
        }
        if (c->kind == K_B16 && pc >= 0x7000 && pc < 0x7000 + EESZ) {
            *mask = 0xFF; *kind = CELL_EE; return &c->ee[pc - 0x7000];
        }
        if (pc < 0x8000) {
            if (pc >= NW) return NULL;
            *kind = CELL_FL; return &c->fl[pc];
        }
        pc -= 0x8000;
        if (pc == 5) { *kind = CELL_RO; return &c->rev; }
        if (pc == 6) { *kind = CELL_RO; return &c->dev; }
        return pc < 32 ? &c->cfgw[pc] : NULL;
    }
    if (pc < 2 * NW) { *kind = CELL_FL; return &c->fl[pc / 2]; }
    *mask = 0xFF;
    if (pc >= 0x200000 && pc < 0x200008) return &c->cfgw[pc - 0x200000];
    if (pc >= 0x300000 && pc < 0x300010) return &c->cfgw[16 + pc - 0x300000];
    if (pc >= 0x380000 && pc < 0x380000 + EESZ) { *kind = CELL_EE; return &c->ee[pc - 0x380000]; }
    *mask = 0xFFFF;
    if (pc == 0x3FFFFC) { *kind = CELL_RO; return &c->rev; }
    if (pc == 0x3FFFFE) { *kind = CELL_RO; return &c->dev; }
    return NULL;
}

static bool locked(chip_t *c) {
    uint16_t m; int k;
    uint16_t *p = cell(c, c->cp_pc, 0, &m, &k);
    return p && !((*p >> c->cp_bit) & 1);
}

static unsigned widx(chip_t *c, uint32_t pc) { return c->byt ? pc / 2 : pc; }
static unsigned step(chip_t *c, uint32_t pc, int ds) {
    uint16_t m; int k;
    return cell(c, pc, ds, &m, &k) && m == 0xFFFF && c->byt ? 2 : 1;   /* 16-bit words step by 2 bytes */
}

static void erase_all(chip_t *c, bool cfg, bool ee) {
    for (unsigned i = 0; i < NW; i++) c->fl[i] = c->mask;
    if (cfg) for (unsigned i = 0; i < 32; i++) c->cfgw[i] = c->byt && i >= 8 ? 0xFF : (c->byt ? 0xFF : 0x3FFF);
    if (ee) for (unsigned i = 0; i < EESZ; i++) c->ee[i] = 0xFF;
}

static void do_program(chip_t *c) {
    uint16_t m; int k;
    uint16_t *p = cell(c, c->pc, c->ld_data, &m, &k);
    c->programs++;
    if (!p) return;
    if (k == CELL_FL) {
        if (locked(c)) return;
        unsigned wi = widx(c, c->pc), base = wi - wi % c->rw;
        for (unsigned j = 0; j < c->rw; j++) if (c->latv[j]) c->fl[base + j] &= c->lat[j];
        memset(c->latv, 0, sizeof c->latv);
    } else if (k != CELL_RO) {
        *p &= (uint16_t)(c->lat1 | ~m);
    }
    c->busy_need = 2800;
}

static void row_erase(chip_t *c) {
    c->rowerase++;
    unsigned wi = widx(c, c->pc), base = wi - wi % c->rw;
    if (locked(c)) return;
    for (unsigned j = 0; j < c->rw && base + j < NW; j++) c->fl[base + j] = c->mask;
    c->busy_need = 2800;
}

static void load(chip_t *c, uint32_t data, bool inc, bool dsp) {
    uint16_t m; int k;
    c->ld_data = dsp;
    uint16_t *p = cell(c, c->pc, dsp, &m, &k);
    if (p && k == CELL_FL) {
        unsigned j = widx(c, c->pc) % c->rw;
        c->lat[j] = (uint16_t)(data & m); c->latv[j] = 1;
    } else {
        c->lat1 = (uint16_t)(data & m);
    }
    if (inc) c->pc += step(c, c->pc, dsp);
}

static uint32_t rd(chip_t *c, bool dsp, bool inc) {
    uint16_t m; int k;
    uint16_t *p = cell(c, c->pc, dsp, &m, &k);
    uint32_t v = 0;
    if (p) {
        v = *p & m;
        if (k == CELL_FL) {
            if (locked(c)) v = 0;
            else if ((int)widx(c, c->pc) == c->fault_idx) v ^= 1;
        }
    }
    if (inc) c->pc += step(c, c->pc, dsp);
    return v;
}

static void bulk_erase(chip_t *c) {
    c->bulk++;
    uint32_t base = c->byt ? 0x300000u : 0x8000u;
    if (c->pc == base) erase_all(c, true, true); else erase_all(c, false, false);
    c->busy_need = 5000;
}

/* A command with no payload, or the payload arrived. */
static void exec(chip_t *c, uint32_t d) {
    if (!c->set8) {
        switch (c->cmd) {
        case 0x00: c->pc = 0x8000; break;
        case 0x02: load(c, d, false, false); break;
        case 0x03: load(c, d, false, true); break;
        case 0x06: c->pc++; break;
        case 0x16: c->pc = 0; break;
        case 0x08: do_program(c); break;
        case 0x09: bulk_erase(c); break;
        case 0x0B: for (unsigned i = 0; i < EESZ; i++) c->ee[i] = 0xFF; c->bulk++; c->busy_need = 5000; break;
        case 0x11: row_erase(c); break;
        default: c->bad_cmds++;
        }
    } else {
        switch (c->cmd) {
        case 0x80: c->pc = d; break;
        case 0x00: load(c, d, false, false); break;
        case 0x02: load(c, d, true, false); break;
        case 0xF8: c->pc += step(c, c->pc, 0); break;
        case 0xE0: do_program(c); break;
        case 0x18: bulk_erase(c); break;
        case 0xF0: row_erase(c); break;
        default: c->bad_cmds++;
        }
    }
}

static void cmd_done(chip_t *c) {
    bool w, r;
    if (!c->set8) { w = c->cmd == 0x00 || c->cmd == 0x02 || c->cmd == 0x03; r = c->cmd == 0x04 || c->cmd == 0x05; }
    else          { w = c->cmd == 0x80 || c->cmd == 0x00 || c->cmd == 0x02; r = c->cmd == 0xFC || c->cmd == 0xFE; }
    c->k = 0; c->sh = 0;
    if (w) { c->st = S_PW; return; }
    if (r) {
        bool inc = c->set8 && c->cmd == 0xFE;
        c->frame = rd(c, c->cmd == 0x05, inc) << 1;
        c->st = S_PR; return;
    }
    exec(c, 0);
    c->st = S_CMD;
}

static void on_nrst(prog_sim_t *s, bool low) {
    chip_t *c = s->chip;
    c->nrst_low = low;
    c->prog = false; c->keyreg = 0; c->nkey = 0; c->st = S_CMD; c->k = 0; c->sh = 0;
    c->idle_us = 0; c->busy_need = 0; c->out = -1; c->pc = 0;
    memset(c->latv, 0, sizeof c->latv);
    c->first_rise = low;
    if (!low) c->ran++;
}

static void on_time(prog_sim_t *s, uint32_t us) { ((chip_t *)s->chip)->idle_us += us; }

static void rise(chip_t *c) {
    if (!c->nrst_low) return;
    if (c->first_rise) { if (c->idle_us < 1) c->timing_err++; c->first_rise = false; }
    if (c->prog) {
        if (c->st == S_CMD && c->k == 0 && c->sh == 0) {
            if (c->first_cmd && c->idle_us < 250) c->timing_err++;     /* TENTH */
            if (c->idle_us < c->busy_need) c->timing_err++;             /* program / erase time */
            if (c->idle_us < 1) c->timing_err++;                        /* TDLY after a command */
            c->first_cmd = false; c->busy_need = 0;
        }
        if (c->st != S_CMD && c->k == 0 && c->idle_us < 1) c->timing_err++;   /* TDLY command to data */
        if (c->st == S_PR) c->out = (c->frame >> (c->set8 ? c->plen - 1 - c->k : c->k)) & 1;
    }
    c->idle_us = 0;
}

static void fall(chip_t *c, int bit) {
    if (!c->nrst_low) return;
    c->idle_us = 0;
    if (!c->prog) {
        c->keyreg = (c->keyreg << 1) | (uint32_t)bit;
        if (++c->nkey == 32) {
            c->last_key = c->keyreg;
            if (!c->deaf && c->keyreg == c->key) { c->prog = true; c->first_cmd = true; c->entries++; }
        }
        return;
    }
    if (c->st == S_CMD) {
        /* count the bits of the command with k, the value in sh (k = 0 and sh = 0 mark a new one) */
        if (c->set8) c->sh = (c->sh << 1) | (uint32_t)bit; else c->sh |= (uint32_t)bit << c->k;
        c->k++;
        if (c->k == c->clen) { c->cmd = (uint8_t)c->sh; cmd_done(c); }
    } else if (c->st == S_PW) {
        if (c->set8) c->sh = (c->sh << 1) | (uint32_t)bit; else c->sh |= (uint32_t)bit << c->k;
        if (++c->k == c->plen) { exec(c, c->sh >> 1); c->st = S_CMD; c->k = 0; c->sh = 0; }
    } else {
        c->out = -1;
        if (++c->k == c->plen) { c->st = S_CMD; c->k = 0; c->sh = 0; }
    }
}

static void on_pins(prog_sim_t *s, int clk, int pod) {
    chip_t *c = s->chip;
    int prev = c->prev_clk;
    c->prev_clk = clk;
    if (clk == prev) return;
    if (clk) rise(c); else fall(c, pod < 0 ? 1 : pod);
}

static int chip_data(prog_sim_t *s) {
    chip_t *c = s->chip;
    return c->prog && c->st == S_PR ? c->out : -1;
}

static const prog_sim_model_t MODEL = { .on_pins = on_pins, .chip_data = chip_data, .on_nrst = on_nrst, .on_time = on_time };

static void chip_init(chip_t *c, int kind) {
    memset(c, 0, sizeof *c);
    c->kind = kind; c->set8 = kind != K_A; c->byt = kind == K_B18;
    c->key = KEY; c->mask = c->byt ? 0xFFFF : 0x3FFF;
    c->clen = c->set8 ? 8 : 6; c->plen = c->set8 ? 24 : 16;
    c->rw = kind == K_A ? 8 : 16;
    c->prev_clk = 1; c->out = -1; c->fault_idx = -1;
    erase_all(c, true, true);
    if (kind == K_A)   { c->dev = 0x2A5; c->rev = 0x2002; c->cp_pc = 0x8007; c->cp_bit = 5; }
    if (kind == K_B16) { c->dev = 0x30B2; c->rev = 0x2001; c->cp_pc = 0x800B; c->cp_bit = 1; }
    if (kind == K_B18) { c->dev = 0x7A31; c->rev = 0x2003; c->cp_pc = 0x300004; c->cp_bit = 3; }
}

/* ---- rig: the pod's side ---- */
typedef struct {
    prog_sim_t sim; prog_wire_ops_t ops; chip_t chip; prog_ctx_t ctx;
    uint8_t img[1024]; uint8_t out[1024]; uint32_t out_n;
} rig_t;

static int rig_src(void *job, uint32_t off, uint8_t *buf, uint32_t n) {
    rig_t *r = job;
    if (off + n > sizeof r->img) return -1;
    memcpy(buf, r->img + off, n);
    return 0;
}
static int rig_dst(void *job, uint32_t off, const uint8_t *buf, uint32_t n) {
    rig_t *r = job;
    if (off + n > sizeof r->out) return -1;
    memcpy(r->out + off, buf, n);
    if (off + n > r->out_n) r->out_n = off + n;
    return 0;
}

static uint32_t flags_of(int kind) {
    if (kind == K_A)   return PIC_F_CP_BIT(5) | PIC_F_HAS_EE;
    if (kind == K_B16) return PIC_F_SET8 | PIC_F_CP_BIT(1);
    return PIC_F_SET8 | PIC_F_BYTEPC | PIC_F_CP_BIT(3);
}

static void rig_init(rig_t *r, int kind) {
    memset(r, 0, sizeof *r);
    chip_init(&r->chip, kind);
    prog_sim_init(&r->sim, &r->ops, &MODEL, &r->chip);
    r->ctx.w = &r->ops;
    r->ctx.flags = flags_of(kind);
    r->ctx.page = kind == K_A ? 16 : 32;
    r->ctx.baud = kind == K_A ? 0x8007u * 2 : kind == K_B16 ? 0x800Bu * 2 : 0x300004u;
    r->ctx.src = rig_src; r->ctx.dst = rig_dst; r->ctx.job = r;
}

typedef int (*step_fn)(prog_ctx_t *);
static int run(rig_t *r, step_fn f) {
    int rc = PROG_RUNNING;
    for (int i = 0; i < 100000 && rc == PROG_RUNNING; i++) rc = f(&r->ctx);
    return rc;
}

static const prog_proto_t *P(void) { return &prog_proto_pic; }

static void clean(rig_t *r, const char *what) {
    if (r->sim.contention || r->chip.timing_err || r->chip.bad_cmds) {
        printf("  %s: contention=%u timing_err=%u bad_cmds=%u\n", what, r->sim.contention, r->chip.timing_err, r->chip.bad_cmds);
        failures++;
    }
}

static void test_entry_id(void) {
    for (int k = K_A; k <= K_B18; k++) {
        rig_t *r = calloc(1, sizeof *r);
        rig_init(r, k);
        int rc = P()->enter(&r->ctx);
        if (rc != PROG_OK) printf("  kind %d rc %d\n", k, rc);
        CHECK(rc == PROG_OK);
        CHECK(r->chip.prog && r->chip.entries == 1 && r->chip.last_key == KEY);
        CHECK(r->ctx.id_len == 4);
        CHECK((r->ctx.id[0] | r->ctx.id[1] << 8) == r->chip.dev);
        CHECK((r->ctx.id[2] | r->ctx.id[3] << 8) == r->chip.rev);
        P()->leave(&r->ctx, true);
        CHECK(!r->sim.nrst_low && r->chip.ran == 1 && !r->chip.prog);
        clean(r, "entry");
        free(r);
    }
}

static void test_no_answer(void) {
    for (int k = K_A; k <= K_B18; k++) {
        rig_t *r = calloc(1, sizeof *r);
        rig_init(r, k);
        r->chip.key ^= 1;                       /* only the right key enters */
        CHECK(P()->enter(&r->ctx) == PROG_E_NO_ANSWER);
        CHECK(!r->chip.prog && r->chip.last_key == KEY);
        P()->leave(&r->ctx, false);
        CHECK(r->sim.nrst_low);                 /* run = false keeps the chip in reset */
        clean(r, "no answer");
        free(r);
    }
}

/* ---- operations ---- */
static int op(rig_t *r, step_fn f, int mem, int erase, uint32_t addr, uint32_t len) {
    r->ctx.mem = (uint8_t)mem; r->ctx.erase = (uint8_t)erase; r->ctx.addr = addr; r->ctx.len = len;
    return run(r, f);
}
#define OP_ENTER() P()->enter(&r->ctx)
#define OP_ERASE(mem, e, a, n)  op(r, P()->erase, mem, e, a, n)
#define OP_WRITE(mem, a, n)     op(r, P()->write, mem, PROG_ERASE_NONE, a, n)
#define OP_READ(mem, a, n)      op(r, P()->read, mem, PROG_ERASE_NONE, a, n)

/* the image: 14-bit chips only keep 14 bits of a word */
static void gen(rig_t *r, uint32_t n, bool w14, unsigned seed) {
    for (uint32_t i = 0; i < n; i++) {
        uint8_t b = (uint8_t)(i * 7 + seed * 13 + (i >> 3));
        r->img[i] = (w14 && (i & 1)) ? b & 0x3F : b;
    }
}

typedef struct { uint32_t cfg, cfg_n, user, user_n, ee; } geom_t;
static const geom_t GEOM[3] = {
    { 0x10010, 2, 0x10000, 8, 4 },
    { 0x10010, 2, 0x10000, 8, 0x7004 },
    { 0x300008, 2, 0x200000, 8, 0x380004 },
};

static bool readback(rig_t *r, int mem, uint32_t addr, uint32_t n) {
    memset(r->out, 0xAA, sizeof r->out); r->out_n = 0;
    if (OP_READ(mem, addr, n) != PROG_OK) return false;
    return r->out_n == n && !memcmp(r->out, r->img, n);
}

static void test_flow(int kind) {
    rig_t *r = calloc(1, sizeof *r);
    rig_init(r, kind);
    chip_t *c = &r->chip;
    const geom_t *g = &GEOM[kind];
    bool w14 = kind != K_B18;
    uint16_t m; int k;
    for (unsigned i = 0; i < NW; i++) c->fl[i] = 0x1234 & c->mask;
    *cell(c, c->cp_pc, 0, &m, &k) &= (uint16_t)~(1u << c->cp_bit);      /* code protected */
    CHECK(locked(c));
    CHECK(OP_ENTER() == PROG_OK);
    /* protected: flash is refused, config stays readable */
    CHECK(OP_READ(PROG_MEM_FLASH, 0, 16) == PROG_E_LOCKED);
    gen(r, 32, w14, 1);
    CHECK(OP_WRITE(PROG_MEM_FLASH, 0x40, 32) == PROG_E_LOCKED);
    CHECK(OP_ERASE(PROG_MEM_FLASH, PROG_ERASE_PAGE, 0x40, 32) == PROG_E_LOCKED);
    CHECK(c->programs == 0 && c->rowerase == 0);
    /* chip erase unlocks */
    CHECK(OP_ERASE(PROG_MEM_FLASH, PROG_ERASE_CHIP, 0, 0) == PROG_OK);
    CHECK(c->bulk >= 1 && !locked(c));
    for (unsigned i = 0; i < NW; i++) CHECK(c->fl[i] == c->mask);
    /* flash, a partial last row */
    uint32_t n = 70, a = 0x40;
    gen(r, n, w14, 2);
    CHECK(OP_WRITE(PROG_MEM_FLASH, a, n) == PROG_OK);
    CHECK(r->ctx.done == r->ctx.total && r->ctx.total == (n + r->ctx.page - 1) / r->ctx.page);
    for (uint32_t i = 0; i < n / 2; i++)
        CHECK(c->fl[a / 2 + i] == (uint16_t)((r->img[2 * i] | r->img[2 * i + 1] << 8) & c->mask));
    CHECK(c->fl[a / 2 + n / 2] == c->mask);
    CHECK(readback(r, PROG_MEM_FLASH, a, n));
    /* config word, user ID, EEPROM */
    gen(r, 16, false, 3);
    if (kind != K_B18) r->img[1] &= 0x3F;
    CHECK(OP_WRITE(PROG_MEM_CONFIG, g->cfg, g->cfg_n) == PROG_OK);
    CHECK(readback(r, PROG_MEM_CONFIG, g->cfg, g->cfg_n));
    gen(r, 16, w14, 4);
    CHECK(OP_WRITE(PROG_MEM_USER, g->user, g->user_n) == PROG_OK);
    CHECK(readback(r, PROG_MEM_USER, g->user, g->user_n));
    gen(r, 12, false, 5);
    CHECK(OP_WRITE(PROG_MEM_EEPROM, g->ee, 12) == PROG_OK);
    CHECK(readback(r, PROG_MEM_EEPROM, g->ee, 12));
    for (unsigned i = 0; i < 12; i++) {
        uint16_t *p = cell(c, g->ee + i, kind == K_A, &m, &k);
        CHECK(p && *p == r->img[i]);
    }
    /* the flash is still there after all that */
    gen(r, n, w14, 2);
    CHECK(readback(r, PROG_MEM_FLASH, a, n));
    /* page erase: the rows covering the range */
    CHECK(OP_ERASE(PROG_MEM_FLASH, PROG_ERASE_PAGE, a, n) == PROG_OK);
    unsigned rows = (n + r->ctx.page - 1) / r->ctx.page;
    CHECK(c->rowerase == rows);
    for (uint32_t i = 0; i < rows * r->ctx.page / 2; i++) CHECK(c->fl[a / 2 + i] == c->mask);
    clean(r, "flow");
    P()->leave(&r->ctx, true);
    CHECK(c->ran == 1);
    free(r);
}

static void test_verify_fail(int kind) {
    rig_t *r = calloc(1, sizeof *r);
    rig_init(r, kind);
    gen(r, 64, kind != K_B18, 6);
    r->chip.fault_idx = 0x40 / 2 + 5;
    CHECK(OP_ENTER() == PROG_OK);
    CHECK(OP_WRITE(PROG_MEM_FLASH, 0x40, 64) == PROG_E_VERIFY);
    CHECK(r->ctx.at_addr == 0x40 + 10);
    clean(r, "verify");
    free(r);
}

static void test_args(int kind) {
    rig_t *r = calloc(1, sizeof *r);
    rig_init(r, kind);
    gen(r, 64, true, 7);
    CHECK(OP_ENTER() == PROG_OK);
    CHECK(OP_WRITE(PROG_MEM_FLASH, 2, 16) == PROG_E_ARGS);          /* not row aligned */
    CHECK(OP_WRITE(PROG_MEM_FLASH, r->ctx.page, 0) == PROG_E_ARGS);
    r->ctx.page = 0;
    CHECK(OP_WRITE(PROG_MEM_FLASH, 0, 16) == PROG_E_ARGS);
    CHECK(OP_ERASE(PROG_MEM_FLASH, PROG_ERASE_PAGE, 0, 16) == PROG_E_ARGS);
    CHECK(OP_READ(PROG_MEM_FLASH, 1, 16) == PROG_E_ARGS);           /* odd flash address */
    r->ctx.page = 16;
    r->ctx.src = NULL;
    CHECK(r->chip.programs == 0 && r->chip.rowerase == 0);
    clean(r, "args");
    free(r);
}

int main(void) {
    test_entry_id();
    test_no_answer();
    for (int k = K_A; k <= K_B18; k++) { test_flow(k); test_verify_fail(k); test_args(k); }
    printf("test_prog_pic: %s\n", failures ? "FAILED" : "ok");
    return failures ? 1 : 0;
}
