/* Host test for prog_pdi.c against a bit-level model of an ATxmega32A4U (prog_sim.h). */
#include "prog_flash.h"
#include "prog_sim.h"
#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); failures++; } } while (0)

#define FLASH_SIZE 0x9000u      /* 32 KB application + 4 KB boot */
#define FPG 128u
#define EE_SIZE 1024u
#define EPG 32u
#define A_NVM 0x010001C0u

typedef struct {
    uint8_t flash[FLASH_SIZE], eeprom[EE_SIZE], user[FPG], fuse[8], lock, buf[FPG], ebuf[EPG];
    uint8_t cmd, nvm_addr[3], nvm_data;
    uint8_t cs_status, cs_ctrl, cs_reset;
    uint32_t ptr, busy_until;
    int prev_clk, idle1;
    bool enabled, nvmen;
    int rxn; uint16_t rxv; bool inframe;
    uint8_t op; int need, reps; uint8_t args[8]; int nargs;
    uint8_t out[64]; int outn, outi, cur;
    /* word latching in the flash/user page buffer */
    uint8_t lowtmp;
    /* PDI physical layer: enable timing, the 10 kHz turn-off, the bus-off time of a chip erase */
    int prev_pod;
    bool en_arm, en_ok;
    uint32_t en_t, last_rise, erase_until;
    unsigned dropouts, breaks, enable_ignore;
    bool key_flip, silent, parity_err, stuck_busy, stuck_erase, corrupt;
    uint32_t corrupt_addr;
    unsigned parity_bad;
} xm_t;

#define STATUS_NVMEN_BIT 0x02u
#define BIT_US 5u                      /* one clock bit on the model's bus: 100 kHz */

static bool locked(xm_t *t) { return (t->lock & 3u) != 3u; }

/* A chip erase switches the PDI bus off: NVMEN reads 0 and the data space does not answer. */
static bool bus_off(prog_sim_t *s, xm_t *t) { return s->us < t->erase_until; }

static uint8_t mem_read(prog_sim_t *s, xm_t *t, uint32_t a) {
    if (a >= 0x01000090u && a < 0x01000093u) { static const uint8_t id[3] = { 0x1E, 0x95, 0x41 }; return id[a - 0x01000090u]; }
    if (!t->nvmen || bus_off(s, t)) return 0;
    if (a == A_NVM + 0x0F) return (t->stuck_busy || s->us < t->busy_until) ? 0x80 : 0;
    if (a == A_NVM + 0x0A) return t->cmd;
    if (t->cmd != 0x43) return 0;                                   /* NVM space reads need READ_NVM loaded */
    if (a == 0x008F0027u) return t->lock;
    if (a >= 0x008F0020u && a < 0x008F0028u) return t->fuse[a - 0x008F0020u];
    if (locked(t)) return 0;
    if (a >= 0x00800000u && a < 0x00800000u + FLASH_SIZE) return t->flash[a - 0x00800000u];
    if (a >= 0x008C0000u && a < 0x008C0000u + EE_SIZE) return t->eeprom[a - 0x008C0000u];
    if (a >= 0x008E0400u && a < 0x008E0400u + FPG) return t->user[a - 0x008E0400u];
    return 0;
}

static void exec_ex(prog_sim_t *s, xm_t *t) {
    switch (t->cmd) {
    case 0x26: memset(t->buf, 0xFF, sizeof t->buf); break;
    case 0x36: memset(t->ebuf, 0xFF, sizeof t->ebuf); break;
    case 0x40:
        memset(t->flash, 0xFF, sizeof t->flash); memset(t->eeprom, 0xFF, sizeof t->eeprom);
        t->lock = 0xFF; t->busy_until = t->erase_until = s->us + 20000;
        if (t->stuck_erase) t->erase_until = 0xFFFFFFFFu;
        break;
    }
}

static void mem_write(prog_sim_t *s, xm_t *t, uint32_t a, uint8_t v) {
    if (a >= 0x01000000u && a < 0x01000000u + 0x200u) {                 /* I/O: the NVM controller */
        if (!t->nvmen || bus_off(s, t)) return;
        uint32_t r = a - A_NVM;
        if (r < 3) t->nvm_addr[r] = v;
        else if (r == 4) t->nvm_data = v;
        else if (r == 0x0A) t->cmd = v;
        else if (r == 0x0B && (v & 1)) exec_ex(s, t);
        return;
    }
    if (!t->nvmen || bus_off(s, t)) return;
    if (a >= 0x008F0020u && a < 0x008F0028u) {                          /* fuse write: started by this PDI write */
        if (t->cmd == 0x4C && !locked(t)) { t->fuse[a - 0x008F0020u] &= v; t->busy_until = s->us + 4000; }
        return;
    }
    bool fl = a >= 0x00800000u && a < 0x00800000u + FLASH_SIZE;
    bool ee = a >= 0x008C0000u && a < 0x008C0000u + EE_SIZE;
    bool us = a >= 0x008E0400u && a < 0x008E0400u + FPG;
    if (t->cmd == 0x23 && (fl || us)) {                                 /* word-wise buffer: the high byte latches the pair */
        uint32_t o = (a - (fl ? 0x00800000u : 0x008E0400u)) % FPG;
        if (!(o & 1u)) t->lowtmp = v; else { t->buf[o - 1] = t->lowtmp; t->buf[o] = v; }
        return;
    }
    if (t->cmd == 0x33 && ee) { t->ebuf[(a - 0x008C0000u) % EPG] = v; return; }
    if (locked(t)) return;
    if (fl && (t->cmd == 0x2E || t->cmd == 0x2F)) {
        uint32_t o = (a - 0x00800000u) / FPG * FPG;
        if (t->cmd == 0x2F) memset(&t->flash[o], 0xFF, FPG);
        for (uint32_t i = 0; i < FPG; i++) {
            uint8_t b = t->buf[i];
            if (t->corrupt && o + i == t->corrupt_addr) b ^= 1;
            t->flash[o + i] &= b;
        }
        memset(t->buf, 0xFF, sizeof t->buf); t->busy_until = s->us + 4000;
    } else if (ee && (t->cmd == 0x34 || t->cmd == 0x35)) {
        uint32_t o = (a - 0x008C0000u) / EPG * EPG;
        if (t->cmd == 0x35) memset(&t->eeprom[o], 0xFF, EPG);
        for (uint32_t i = 0; i < EPG; i++) t->eeprom[o + i] &= t->ebuf[i];
        memset(t->ebuf, 0xFF, sizeof t->ebuf); t->busy_until = s->us + 3000;
    } else if (us && t->cmd == 0x18) { memset(t->user, 0xFF, sizeof t->user); t->busy_until = s->us + 3000; }
    else if (us && t->cmd == 0x1A) {
        for (uint32_t i = 0; i < FPG; i++) t->user[i] &= t->buf[i];
        memset(t->buf, 0xFF, sizeof t->buf); t->busy_until = s->us + 3000;
    }
}

static void queue_reply(xm_t *t, uint8_t reply) {
    if (t->silent) return;
    int n = 0, ones = 0;
    t->out[n++] = 1; t->out[n++] = 1;                    /* guard time 2 */
    t->out[n++] = 0;
    for (int i = 0; i < 8; i++) { int bit = (reply >> i) & 1; ones += bit; t->out[n++] = (uint8_t)bit; }
    t->out[n++] = (uint8_t)((ones & 1) ^ (t->parity_err ? 1 : 0)); t->out[n++] = 1; t->out[n++] = 1;
    t->outn = n; t->outi = 0;
}

static void rx_byte(prog_sim_t *s, xm_t *t, uint8_t b) {
    if (t->need > 0) {
        t->args[t->nargs++] = b;
        if (--t->need) return;
        uint32_t a = t->args[0] | t->args[1] << 8 | (uint32_t)t->args[2] << 16 | (uint32_t)t->args[3] << 24;
        switch (t->op) {
        case 0x0C: queue_reply(t, mem_read(s, t, a)); break;
        case 0x4C: t->need = 1; t->op = 0x4D; t->nargs = 0; t->ptr = a; return;  /* address taken; data next */
        case 0x6B: t->ptr = a; break;
        case 0xA0: t->reps = t->args[0] + 1; break;
        case 0xE0: {
            static const uint8_t key[8] = { 0xFF, 0x88, 0xD8, 0xCD, 0x45, 0xAB, 0x89, 0x12 };
            if (!t->key_flip && t->cs_reset == 0x59 && memcmp(t->args, key, 8) == 0) { t->nvmen = true; t->cs_status |= 2; }
            break;
        }
        }
        if (t->op == 0x60 || t->op == 0x64) {
            mem_write(s, t, t->ptr, b);
            if (t->op == 0x64) t->ptr++;
        }
        if ((t->op & 0xF0) == 0xC0) {
            if ((t->op & 0xF) == 2) t->cs_ctrl = b;
            else if ((t->op & 0xF) == 1) t->cs_reset = b;
        }
        if (t->op == 0x4D) { mem_write(s, t, t->ptr, b); }
        return;
    }
    t->op = b; t->nargs = 0;
    if (b == 0x0C) t->need = 4;
    else if (b == 0x4C) t->need = 4;
    else if (b == 0x6B) t->need = 4;
    else if (b == 0xA0) t->need = 1;
    else if (b == 0xE0) t->need = 8;
    else if ((b & 0xF0) == 0xC0) t->need = 1;
    else if (b == 0x60 || b == 0x64) t->need = 1;
    else if (b == 0x24) { queue_reply(t, mem_read(s, t, t->ptr)); t->ptr++; }
    else if ((b & 0xF0) == 0x80) {
        uint8_t st = t->cs_status;
        if (bus_off(s, t)) st &= (uint8_t)~STATUS_NVMEN_BIT;
        queue_reply(t, (b & 0xF) == 0 ? st : (b & 0xF) == 2 ? t->cs_ctrl : t->cs_reset);
    }
}

/* REPEAT: the next instruction runs reps times; ST *(ptr++) takes reps data bytes. */
static void rx_frame(prog_sim_t *s, xm_t *t, uint8_t b) {
    if (t->reps > 0 && t->need == 0 && t->op == 0x64) {            /* data bytes of a repeated ST */
        mem_write(s, t, t->ptr++, b);
        if (--t->reps == 0) t->op = 0;
        return;
    }
    rx_byte(s, t, b);
    if (t->op == 0x64 && t->reps > 0) { t->need = 0; }
}

static void drop_pdi(xm_t *t) {                      /* clock slower than ~10 kHz: the PDI turns itself off */
    if (t->enabled) t->dropouts++;
    t->enabled = t->nvmen = false; t->cs_status = t->cs_reset = 0;
    t->inframe = false; t->need = 0; t->reps = 0; t->outn = t->outi = 0; t->cur = -1; t->idle1 = 0;
}

static void on_pins(prog_sim_t *s, int clk, int pod) {
    xm_t *t = s->chip;
    if (pod == 1 && t->prev_pod != 1 && !t->enabled) { t->en_arm = true; t->en_t = s->us; }   /* data forced high */
    t->prev_pod = pod;
    bool rise = t->prev_clk == 0 && clk == 1;
    if (rise) {
        if (s->us - t->last_rise > 100) drop_pdi(t);
        t->last_rise = s->us;
        if (t->en_arm) {                                          /* first clock after data went high */
            uint32_t d = s->us - t->en_t;
            t->en_ok = d >= 1 && d <= 100;                        /* high >= 1 us, clocks within 100 us */
            t->en_arm = false;
        }
    }
    if (t->prev_clk == 1 && clk == 0) {
        t->cur = t->outi < t->outn ? t->out[t->outi++] : -1;
    } else if (rise && pod >= 0) {
        if (!t->inframe) {
            if (pod == 1) { if (t->idle1 < 1000) t->idle1++; }
            else {
                t->inframe = true; t->rxn = 0; t->rxv = 0;
                if (!t->enabled && t->idle1 >= 16 && t->en_ok) {
                    if (t->enable_ignore) t->enable_ignore--; else t->enabled = true;
                }
                t->en_ok = false; t->idle1 = 0;
            }
        } else {
            t->rxv |= (uint16_t)(pod << t->rxn);
            if (++t->rxn == 11) {
                t->inframe = false;
                uint8_t d = (uint8_t)t->rxv;
                int ones = 0; for (int i = 0; i < 8; i++) ones += (d >> i) & 1;
                int par = (t->rxv >> 8) & 1, st = (t->rxv >> 9) & 3;
                if (d == 0 && par == 0 && st == 0) t->breaks++;
                if (!t->enabled) {}
                else if (par != (ones & 1) || st != 3) t->parity_bad++;
                else rx_frame(s, t, d);
            }
        }
    }
    t->prev_clk = clk;
    if (rise) s->us += BIT_US;
}

static int chip_data(prog_sim_t *s) { xm_t *t = s->chip; return t->cur; }
static void on_nrst(prog_sim_t *s, bool low) { (void)s; (void)low; }
static void on_time(prog_sim_t *s, uint32_t us) { (void)s; (void)us; }

typedef struct { uint8_t src[4096]; uint8_t dst[4096]; } job_t;
static int src_cb(void *j, uint32_t off, uint8_t *b, uint32_t n) { memcpy(b, ((job_t *)j)->src + off, n); return 0; }
static int dst_cb(void *j, uint32_t off, const uint8_t *b, uint32_t n) { memcpy(((job_t *)j)->dst + off, b, n); return 0; }

static xm_t chip; static prog_sim_t sim; static prog_wire_ops_t ops; static prog_ctx_t ctx; static job_t job;

static void setup(void) {
    memset(&chip, 0, sizeof chip);
    memset(chip.flash, 0xFF, sizeof chip.flash); memset(chip.eeprom, 0xFF, sizeof chip.eeprom);
    memset(chip.user, 0xFF, sizeof chip.user); memset(chip.fuse, 0xFF, sizeof chip.fuse);
    memset(chip.buf, 0xFF, sizeof chip.buf); memset(chip.ebuf, 0xFF, sizeof chip.ebuf);
    chip.lock = 0xFF; chip.cur = -1; chip.prev_clk = 1; chip.prev_pod = -1;
    prog_sim_model_t m = { .on_pins = on_pins, .chip_data = chip_data, .on_nrst = on_nrst, .on_time = on_time };
    prog_sim_init(&sim, &ops, &m, &chip);
    memset(&ctx, 0, sizeof ctx);
    ctx.w = &ops; ctx.page = FPG; ctx.src = src_cb; ctx.dst = dst_cb; ctx.job = &job;
    memset(&job, 0, sizeof job);
    for (unsigned i = 0; i < sizeof job.src; i++) job.src[i] = (uint8_t)(i * 5 + 1);
}

/* The job runs one bounded step per hw worker pass: between two steps the clock does not run.
   gap_us models that pause (0: none). */
static uint32_t gap_us;

static int run(int (*f)(prog_ctx_t *)) {
    int rc, guard = 0;
    while ((rc = f(&ctx)) == PROG_RUNNING && ++guard < 1000)
        if (gap_us) ops.delay_us(ops.io, gap_us);
    return rc;
}

int main(void) {
    const prog_proto_t *p = &prog_proto_pdi;

    /* entry + signature */
    setup();
    CHECK(p->enter(&ctx) == PROG_OK);
    CHECK(ctx.id_len == 3 && ctx.id[0] == 0x1E && ctx.id[1] == 0x95 && ctx.id[2] == 0x41);
    CHECK(chip.nvmen);
    p->leave(&ctx, true);
    CHECK(chip.cs_reset == 0);

    /* only the right key enables the NVM */
    setup();
    chip.key_flip = true;
    CHECK(p->enter(&ctx) == PROG_E_PROTOCOL);
    p->leave(&ctx, true);

    /* chip erase, flash write over 300 bytes (3 pages, last partial), read back */
    setup();
    memset(chip.flash, 0x00, 512);
    CHECK(p->enter(&ctx) == PROG_OK);
    ctx.mem = PROG_MEM_FLASH; ctx.erase = PROG_ERASE_CHIP;
    CHECK(p->erase(&ctx) == PROG_OK);
    CHECK(chip.flash[5] == 0xFF);
    ctx.addr = 0; ctx.len = 300;
    CHECK(run(p->write) == PROG_OK);
    CHECK(memcmp(chip.flash, job.src, 300) == 0 && chip.flash[300] == 0xFF);
    CHECK(ctx.total == 3 && ctx.done == 3);
    CHECK(run(p->read) == PROG_OK);
    CHECK(memcmp(job.dst, job.src, 300) == 0);
    p->leave(&ctx, true);

    /* page erase mode rewrites a dirty page; neighbors stay */
    setup();
    memset(chip.flash, 0x00, 512);
    CHECK(p->enter(&ctx) == PROG_OK);
    ctx.mem = PROG_MEM_FLASH; ctx.erase = PROG_ERASE_PAGE; ctx.addr = 128; ctx.len = 128;
    CHECK(run(p->write) == PROG_OK);
    CHECK(memcmp(chip.flash + 128, job.src, 128) == 0 && chip.flash[0] == 0 && chip.flash[256] == 0);
    p->leave(&ctx, true);

    /* eeprom, user row, fuses */
    setup();
    CHECK(p->enter(&ctx) == PROG_OK);
    ctx.mem = PROG_MEM_EEPROM; ctx.erase = PROG_ERASE_PAGE; ctx.page = EPG; ctx.addr = 0; ctx.len = 70;
    CHECK(run(p->write) == PROG_OK && memcmp(chip.eeprom, job.src, 70) == 0);
    ctx.mem = PROG_MEM_USER; ctx.page = FPG; ctx.len = 100;
    CHECK(run(p->write) == PROG_OK && memcmp(chip.user, job.src, 100) == 0);
    ctx.mem = PROG_MEM_CONFIG; ctx.addr = 1; ctx.len = 2;
    job.src[0] = 0xAB; job.src[1] = 0xCD;
    CHECK(run(p->write) == PROG_OK && chip.fuse[1] == 0xAB && chip.fuse[2] == 0xCD && chip.fuse[0] == 0xFF);
    CHECK(run(p->read) == PROG_OK && job.dst[0] == 0xAB && job.dst[1] == 0xCD);
    p->leave(&ctx, true);

    /* verify failure reports the address */
    setup();
    chip.corrupt = true; chip.corrupt_addr = 140;
    CHECK(p->enter(&ctx) == PROG_OK);
    ctx.mem = PROG_MEM_FLASH; ctx.erase = PROG_ERASE_NONE; ctx.addr = 0; ctx.len = 256;
    CHECK(run(p->write) == PROG_E_VERIFY && ctx.at_addr == 140);
    p->leave(&ctx, true);

    /* locked chip */
    setup();
    chip.lock = 0xFC; memset(chip.flash, 0x55, 256);
    CHECK(p->enter(&ctx) == PROG_OK);
    ctx.mem = PROG_MEM_FLASH; ctx.erase = PROG_ERASE_NONE; ctx.addr = 0; ctx.len = 128;
    CHECK(run(p->read) == PROG_E_LOCKED && run(p->write) == PROG_E_LOCKED);
    ctx.erase = PROG_ERASE_CHIP;
    CHECK(p->erase(&ctx) == PROG_OK);
    ctx.erase = PROG_ERASE_NONE;
    CHECK(run(p->write) == PROG_OK && memcmp(chip.flash, job.src, 128) == 0);
    p->leave(&ctx, true);

    /* parity error from the chip */
    setup();
    chip.parity_err = true;
    CHECK(p->enter(&ctx) == PROG_E_PROTOCOL);
    p->leave(&ctx, true);

    /* silent chip */
    setup();
    chip.silent = true;
    CHECK(p->enter(&ctx) == PROG_E_NO_ANSWER);
    p->leave(&ctx, true);

    /* stuck busy */
    setup();
    CHECK(p->enter(&ctx) == PROG_OK);
    chip.stuck_busy = true;                       /* NVM STATUS busy for good: a page write times out */
    ctx.mem = PROG_MEM_FLASH; ctx.erase = PROG_ERASE_NONE; ctx.addr = 0; ctx.len = 16;
    CHECK(run(p->write) == PROG_E_TIMEOUT);
    p->leave(&ctx, true);

    /* a chip erase that never ends (NVMEN stays 0) */
    setup();
    CHECK(p->enter(&ctx) == PROG_OK);
    chip.stuck_erase = true;
    ctx.mem = PROG_MEM_FLASH; ctx.erase = PROG_ERASE_CHIP;
    CHECK(p->erase(&ctx) == PROG_E_TIMEOUT);
    p->leave(&ctx, true);

    /* bad args */
    setup();
    CHECK(p->enter(&ctx) == PROG_OK);
    ctx.mem = PROG_MEM_FLASH; ctx.page = 0; ctx.len = 4;
    CHECK(p->write(&ctx) == PROG_E_ARGS);
    ctx.page = 128; ctx.addr = 0x400000u;
    CHECK(p->write(&ctx) == PROG_E_ARGS);
    p->leave(&ctx, true);

    /* entry: RESET register 0x59 first (the model takes the key only then), 3 tries with BREAKs */
    setup();
    CHECK(p->enter(&ctx) == PROG_OK && chip.cs_reset == 0x59 && chip.breaks == 0);
    p->leave(&ctx, true);
    setup();
    chip.enable_ignore = 1;
    CHECK(p->enter(&ctx) == PROG_OK && chip.breaks >= 2 && chip.nvmen);
    p->leave(&ctx, true);
    setup();
    chip.enable_ignore = 100;
    CHECK(p->enter(&ctx) == PROG_E_NO_ANSWER && chip.breaks >= 4);
    p->leave(&ctx, true);

    /* the flash page buffer latches words: an odd length and an odd start are padded with 0xFF */
    setup();
    CHECK(p->enter(&ctx) == PROG_OK);
    ctx.mem = PROG_MEM_FLASH; ctx.erase = PROG_ERASE_PAGE; ctx.addr = 0; ctx.len = 301;
    CHECK(run(p->write) == PROG_OK && memcmp(chip.flash, job.src, 301) == 0 && chip.flash[301] == 0xFF);
    ctx.erase = PROG_ERASE_PAGE; ctx.addr = 515; ctx.len = 10;
    CHECK(run(p->write) == PROG_OK && memcmp(chip.flash + 515, job.src, 10) == 0 && chip.flash[514] == 0xFF);
    ctx.mem = PROG_MEM_USER; ctx.addr = 0; ctx.len = 5;
    CHECK(run(p->write) == PROG_OK && memcmp(chip.user, job.src, 5) == 0 && chip.user[5] == 0xFF);
    p->leave(&ctx, true);

    /* gaps between job steps (the clock stops, the PDI turns off): every step re-enters when needed */
    setup();
    gap_us = 2000;
    memset(chip.flash, 0x00, 512);
    CHECK(p->enter(&ctx) == PROG_OK);
    ctx.mem = PROG_MEM_FLASH; ctx.erase = PROG_ERASE_CHIP; ctx.addr = 0; ctx.len = 300;
    ops.delay_us(ops.io, 2000);
    CHECK(p->erase(&ctx) == PROG_OK);
    ops.delay_us(ops.io, 2000);
    CHECK(run(p->write) == PROG_OK && memcmp(chip.flash, job.src, 300) == 0);
    ops.delay_us(ops.io, 2000);
    CHECK(run(p->read) == PROG_OK && memcmp(job.dst, job.src, 300) == 0);
    ctx.mem = PROG_MEM_CONFIG; ctx.addr = 1; ctx.len = 2; ctx.erase = PROG_ERASE_NONE;
    job.src[0] = 0xAB; job.src[1] = 0xCD;
    ops.delay_us(ops.io, 2000);
    CHECK(run(p->write) == PROG_OK && chip.fuse[1] == 0xAB && chip.fuse[2] == 0xCD);
    CHECK(chip.dropouts >= 4);
    p->leave(&ctx, true);
    gap_us = 0;

    setup();
    CHECK(p->enter(&ctx) == PROG_OK);
    ctx.mem = PROG_MEM_FLASH; ctx.erase = PROG_ERASE_CHIP; ctx.addr = 0; ctx.len = 300;
    CHECK(p->erase(&ctx) == PROG_OK && run(p->write) == PROG_OK);
    CHECK(run(p->read) == PROG_OK && memcmp(job.dst, job.src, 300) == 0);
    p->leave(&ctx, true);
    CHECK(chip.dropouts == 0);                              /* waits are clocked: without step gaps the PDI never drops */

    CHECK(sim.contention == 0);
    CHECK(sim.low_at_flush == 0);
    CHECK(sim.clk_low_max_us <= 50);
    printf("test_prog_pdi: %s\n", failures ? "FAILED" : "ok");
    return failures ? 1 : 0;
}
