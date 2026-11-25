/* Host test for prog_tpi.c against a bit-level model of an ATtiny10 (prog_sim.h). */
#include "prog_flash.h"
#include "prog_sim.h"
#include "../src/prog_avr12.h"
#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); failures++; } } while (0)

#define FLASH_BASE 0x4000u
#define FLASH_SIZE 1024u
#define PG 16u

typedef struct {
    /* the chip */
    uint8_t flash[FLASH_SIZE], config, lock, sig[3];
    uint8_t cs_tpisr, cs_pcr, nvmcmd;
    uint16_t ptr;
    bool nvmen;
    uint32_t busy_until;
    int prev_clk;
    /* receive side */
    int rxn; uint16_t rxv; bool inframe;
    uint8_t op; int need; uint8_t args[8]; int nargs;
    /* transmit side */
    uint8_t out[32]; int outn, outi, cur;
    /* the group write buffer (ATtiny20: 2 words, ATtiny40: 4 words, others 1) */
    unsigned nwords, wcnt, idle_run, frame_idle, op_idle;
    uint16_t wbase;
    uint8_t wbuf[8];
    /* test knobs */
    bool has_page_erase;
    unsigned nvmen_delay, keys, idle_missing, misaligned, bad_ident;
    bool key_flip, silent, parity_err, stuck_busy, nrst_low, corrupt;
    uint32_t corrupt_addr;
    unsigned frames_rx, parity_bad;
    uint32_t last_clk_us;
} tiny_t;

static bool locked(tiny_t *t) { return (t->lock & 3u) != 3u; }

static uint8_t mem_read(prog_sim_t *s, tiny_t *t, uint16_t a) {
    if (!t->nvmen) return 0;
    if (a == 0x32) return (t->stuck_busy || s->us < t->busy_until) ? 0x80 : 0;
    if (a == 0x33) return t->nvmcmd;
    if (a == 0x3F00) return t->lock;
    if (a >= 0x3FC0 && a < 0x3FC3) return t->sig[a - 0x3FC0];
    if (a == 0x3F40) return t->config;
    if (a >= FLASH_BASE && a < FLASH_BASE + FLASH_SIZE) return locked(t) ? 0 : t->flash[a - FLASH_BASE];
    return 0;
}

static void mem_write(prog_sim_t *s, tiny_t *t, uint16_t a, uint8_t v, bool inc_word) {
    (void)inc_word;
    if (!t->nvmen) return;
    if (a == 0x33) { t->nvmcmd = v; t->wcnt = 0; return; }
    bool fl = a >= FLASH_BASE && a < FLASH_BASE + FLASH_SIZE;
    bool cf = a >= 0x3F40 && a < 0x3F48;
    if (!fl && !cf) return;
    if (t->nvmcmd == 0x10 && fl && (a & 1u)) {              /* chip erase: dummy write to a high byte */
        memset(t->flash, 0xFF, sizeof t->flash); t->config = 0xFF; t->lock = 0xFF;
        t->busy_until = s->us + 15000;
    } else if (t->nvmcmd == 0x14 && (a & 1u)) {              /* section erase: the WHOLE section */
        if (fl) memset(t->flash, 0xFF, sizeof t->flash);
        else t->config = 0xFF;
        t->busy_until = s->us + 4000;
    } else if (t->nvmcmd == 0x18 && fl && t->has_page_erase && (a & 1u)) {   /* page erase (t102/104) */
        uint16_t b = (uint16_t)((a - FLASH_BASE) / PG * PG); memset(&t->flash[b], 0xFF, PG);
        t->busy_until = s->us + 4000;
    } else if (t->nvmcmd == 0x1D) {                          /* word write: the group latches, ANDs into the erased cells */
        if (locked(t)) return;
        unsigned gb = 2 * t->nwords;
        if (t->wcnt == 0) {
            if (a % gb) { t->misaligned++; return; }
            t->wbase = a;
        } else if (a != t->wbase + t->wcnt) { t->wcnt = 0; t->misaligned++; return; }
        if (t->wcnt && t->wcnt % 2 == 0 && t->op_idle < 12) {   /* the next word needs an IDLE character */
            t->idle_missing++; t->wcnt = 0; return;
        }
        t->wbuf[t->wcnt++] = v;
        if (t->wcnt == gb) {
            for (unsigned i = 0; i < gb; i++) {
                uint32_t o = t->wbase - FLASH_BASE + i;
                if (fl) {
                    uint8_t b = t->wbuf[i];
                    if (t->corrupt && o == t->corrupt_addr) b ^= 0x01;
                    t->flash[o] &= b;
                } else if (t->wbase + i == 0x3F40) t->config &= t->wbuf[i];
            }
            t->wcnt = 0;
            t->busy_until = s->us + 4000;
        }
    }
}

static void rx_byte(prog_sim_t *s, tiny_t *t, uint8_t b) {
    t->frames_rx++;
    if (t->need > 0) {
        t->args[t->nargs++] = b;
        if (--t->need) return;
        switch (t->op & 0xF0) {
        case 0x60:
            if ((t->op & 0xFC) == 0x68) { if (t->op & 1) t->ptr = (uint16_t)((t->ptr & 0xFF) | b << 8); else t->ptr = (uint16_t)((t->ptr & 0xFF00) | b); }
            else { mem_write(s, t, t->ptr, b, false); if (t->op & 4) t->ptr++; }
            break;
        case 0xC0:
            if ((t->op & 0xF) == 2) t->cs_pcr = b; else if ((t->op & 0xF) == 0) { t->cs_tpisr = b; if (!(b & 2)) t->nvmen = false; }
            break;
        case 0xE0: {
            static const uint8_t key[8] = { 0xFF, 0x88, 0xD8, 0xCD, 0x45, 0xAB, 0x89, 0x12 };
            t->keys++;
            if (!t->key_flip && memcmp(t->args, key, 8) == 0) { t->nvmen = true; t->cs_tpisr |= 2; }
            break;
        }
        }
        return;
    }
    t->op = b; t->nargs = 0; t->op_idle = t->frame_idle;   /* idle bits before the instruction */
    uint8_t reply = 0; bool answer = false;
    if ((b & 0xFB) == 0x20) { reply = mem_read(s, t, t->ptr); if (b & 4) t->ptr++; answer = true; }
    else if ((b & 0xFB) == 0x60) t->need = 1;
    else if ((b & 0xFE) == 0x68) t->need = 1;
    else if ((b & 0xF0) == 0x80) {
        reply = ((b & 0xF) == 0) ? t->cs_tpisr : (b & 0xF) == 2 ? t->cs_pcr : (b & 0xF) == 0xF ? (uint8_t)(0x80 ^ t->bad_ident) : 0;
        if ((b & 0xF) == 0 && (reply & 2) && t->nvmen_delay) { t->nvmen_delay--; reply &= (uint8_t)~2u; }   /* NVMEN comes up late */
        answer = true;
    }
    else if ((b & 0xF0) == 0xC0) t->need = 1;
    else if (b == 0xE0) t->need = 8;
    if (answer && !t->silent) {
        int n = 0;
        for (int g = 0; g < 2; g++) t->out[n++] = 1;           /* guard time 2 */
        t->out[n++] = 0;
        int ones = 0;
        for (int i = 0; i < 8; i++) { int bit = (reply >> i) & 1; ones += bit; t->out[n++] = (uint8_t)bit; }
        int par = ones & 1;
        if (t->parity_err) par ^= 1;
        t->out[n++] = (uint8_t)par; t->out[n++] = 1; t->out[n++] = 1;
        t->outn = n; t->outi = 0;
    }
}

static void on_pins(prog_sim_t *s, int clk, int pod) {
    tiny_t *t = s->chip;
    if (!t->nrst_low) { t->prev_clk = clk; return; }
    if (t->prev_clk == 1 && clk == 0) {                          /* falling edge: the chip changes its output */
        t->cur = t->outi < t->outn ? t->out[t->outi++] : -1;
    } else if (t->prev_clk == 0 && clk == 1 && pod >= 0) {       /* rising edge: the chip samples */
        if (!t->inframe) {
            if (pod == 0) { t->inframe = true; t->rxn = 0; t->rxv = 0; t->frame_idle = t->idle_run; t->idle_run = 0; }
            else t->idle_run++;
        }
        else {
            t->rxv |= (uint16_t)(pod << t->rxn);
            if (++t->rxn == 11) {                                /* 8 data + parity + 2 stop */
                t->inframe = false;
                uint8_t d = (uint8_t)t->rxv;
                int ones = 0; for (int i = 0; i < 8; i++) ones += (d >> i) & 1;
                int par = (t->rxv >> 8) & 1, st = (t->rxv >> 9) & 3;
                if (par != (ones & 1) || st != 3) t->parity_bad++; else rx_byte(s, t, d);
            }
        }
    }
    t->prev_clk = clk;
}

static int chip_data(prog_sim_t *s) { tiny_t *t = s->chip; return t->nrst_low ? t->cur : -1; }
static void on_nrst(prog_sim_t *s, bool low) { tiny_t *t = s->chip; t->nrst_low = low; if (low) { t->nvmen = false; t->inframe = false; t->need = 0; t->outn = t->outi = 0; t->cur = -1; t->cs_tpisr = 0; } }
static void on_time(prog_sim_t *s, uint32_t us) { (void)s; (void)us; }

typedef struct { uint8_t src[2048]; uint8_t dst[2048]; } job_t;
static int src_cb(void *j, uint32_t off, uint8_t *b, uint32_t n) { memcpy(b, ((job_t *)j)->src + off, n); return 0; }
static int dst_cb(void *j, uint32_t off, const uint8_t *b, uint32_t n) { memcpy(((job_t *)j)->dst + off, b, n); return 0; }

static tiny_t chip; static prog_sim_t sim; static prog_wire_ops_t ops; static prog_ctx_t ctx; static job_t job;

static void setup(void) {
    memset(&chip, 0, sizeof chip);
    memset(chip.flash, 0xFF, sizeof chip.flash);
    chip.config = 0xFF; chip.lock = 0xFF; chip.sig[0] = 0x1E; chip.sig[1] = 0x90; chip.sig[2] = 0x03;
    chip.cur = -1; chip.prev_clk = 1; chip.nwords = 1;
    prog_sim_model_t m = { .on_pins = on_pins, .chip_data = chip_data, .on_nrst = on_nrst, .on_time = on_time };
    prog_sim_init(&sim, &ops, &m, &chip);
    memset(&ctx, 0, sizeof ctx);
    ctx.w = &ops; ctx.page = PG; ctx.src = src_cb; ctx.dst = dst_cb; ctx.job = &job;
    memset(&job, 0, sizeof job);
    for (unsigned i = 0; i < sizeof job.src; i++) job.src[i] = (uint8_t)(i * 7 + 3);
}

static int run(int (*f)(prog_ctx_t *)) {
    int rc, guard = 0;
    while ((rc = f(&ctx)) == PROG_RUNNING && ++guard < 1000) {}
    return rc;
}

int main(void) {
    const prog_proto_t *p = &prog_proto_tpi;

    /* entry + signature */
    setup();
    CHECK(p->enter(&ctx) == PROG_OK);
    CHECK(ctx.id_len == 3 && ctx.id[0] == 0x1E && ctx.id[1] == 0x90 && ctx.id[2] == 0x03);
    CHECK(chip.nvmen && sim.nrst_low);
    p->leave(&ctx, true);
    CHECK(!chip.nvmen && !sim.nrst_low);

    /* only the right key enters */
    setup();
    chip.key_flip = true;
    CHECK(p->enter(&ctx) == PROG_E_PROTOCOL);
    CHECK(!chip.nvmen);
    p->leave(&ctx, true);

    /* erase + write + read back */
    setup();
    chip.flash[5] = 0x00;
    CHECK(p->enter(&ctx) == PROG_OK);
    ctx.mem = PROG_MEM_FLASH; ctx.erase = PROG_ERASE_CHIP;
    CHECK(p->erase(&ctx) == PROG_OK);
    CHECK(chip.flash[5] == 0xFF);
    ctx.addr = 0; ctx.len = 100;
    CHECK(run(p->write) == PROG_OK);
    CHECK(memcmp(chip.flash, job.src, 100) == 0 && chip.flash[100] == 0xFF);
    CHECK(ctx.done == ctx.total && ctx.total == 7);
    CHECK(run(p->read) == PROG_OK);
    CHECK(memcmp(job.dst, job.src, 100) == 0);
    p->leave(&ctx, true);
    CHECK(sim.contention == 0);

    /* erase "page" on these chips is one section erase before the first page (the section is the
       whole code area): a dirty flash is rewritten, and a multi-page write keeps every page */
    setup();
    memset(chip.flash, 0x00, 256);
    CHECK(p->enter(&ctx) == PROG_OK);
    ctx.mem = PROG_MEM_FLASH; ctx.erase = PROG_ERASE_PAGE; ctx.addr = 16; ctx.len = 100;
    CHECK(run(p->write) == PROG_OK);
    CHECK(memcmp(chip.flash + 16, job.src, 100) == 0 && chip.flash[0] == 0xFF && chip.flash[116] == 0xFF);
    p->leave(&ctx, true);

    /* ATtiny102/104: PAGE_ERASE (flag) erases only the pages written */
    setup();
    chip.has_page_erase = true;
    memset(chip.flash, 0x00, 256);
    CHECK(p->enter(&ctx) == PROG_OK);
    ctx.mem = PROG_MEM_FLASH; ctx.erase = PROG_ERASE_PAGE; ctx.addr = 16; ctx.len = 32; ctx.flags = TPI_F_PAGE_ERASE;
    CHECK(run(p->write) == PROG_OK);
    CHECK(memcmp(chip.flash + 16, job.src, 32) == 0 && chip.flash[0] == 0x00 && chip.flash[15] == 0x00 && chip.flash[48] == 0x00);
    p->leave(&ctx, true);

    /* ATtiny20 (2 words per write) and ATtiny40 (4 words): grouped, 0xFF padded, IDLE between words,
       odd length and a start inside a group */
    for (unsigned nw = 2; nw <= 4; nw += 2) {
        setup();
        chip.nwords = nw;
        memset(chip.flash, 0x00, 256);
        CHECK(p->enter(&ctx) == PROG_OK);
        ctx.mem = PROG_MEM_FLASH; ctx.erase = PROG_ERASE_PAGE; ctx.addr = 2; ctx.len = 99; ctx.flags = nw;
        CHECK(run(p->write) == PROG_OK);
        CHECK(memcmp(chip.flash + 2, job.src, 99) == 0 && chip.flash[0] == 0xFF && chip.flash[101] == 0xFF);
        CHECK(chip.idle_missing == 0 && chip.misaligned == 0);
        CHECK(run(p->read) == PROG_OK && memcmp(job.dst, job.src, 99) == 0);
        /* config: the word, then dummy words to fill the group */
        job.src[0] = 0xFE;
        ctx.mem = PROG_MEM_CONFIG; ctx.addr = 0; ctx.len = 1; ctx.erase = PROG_ERASE_NONE;
        CHECK(run(p->write) == PROG_OK && chip.config == 0xFE);
        CHECK(chip.idle_missing == 0 && chip.misaligned == 0);
        p->leave(&ctx, true);
    }
    /* a chip that wants 2 words and gets 1 loses the write */
    setup();
    chip.nwords = 2;
    CHECK(p->enter(&ctx) == PROG_OK);
    ctx.mem = PROG_MEM_FLASH; ctx.erase = PROG_ERASE_CHIP; ctx.addr = 0; ctx.len = 16; ctx.flags = 0;
    CHECK(p->erase(&ctx) == PROG_OK);
    CHECK(run(p->write) == PROG_E_VERIFY);
    p->leave(&ctx, true);

    /* flags: only the words and page-erase bits, words 1/2/4, page a multiple of the group */
    setup();
    CHECK(p->enter(&ctx) == PROG_OK);
    ctx.mem = PROG_MEM_FLASH; ctx.erase = PROG_ERASE_NONE; ctx.addr = 0; ctx.len = 16;
    ctx.flags = 3;  CHECK(run(p->write) == PROG_E_ARGS);
    ctx.flags = 0x10; CHECK(run(p->write) == PROG_E_ARGS);
    ctx.flags = 4; ctx.page = 6; CHECK(run(p->write) == PROG_E_ARGS);
    p->leave(&ctx, true);

    /* NVMEN is polled: it may take a few reads; never: protocol error */
    setup();
    chip.nvmen_delay = 3;
    CHECK(p->enter(&ctx) == PROG_OK && chip.nvmen);
    p->leave(&ctx, true);
    setup();
    chip.nvmen_delay = 1000;
    CHECK(p->enter(&ctx) == PROG_E_PROTOCOL);
    p->leave(&ctx, true);

    /* TPIIR (0x80) is read before the key goes out */
    setup();
    chip.bad_ident = 1;
    CHECK(p->enter(&ctx) == PROG_E_NO_ANSWER);
    CHECK(chip.keys == 0);
    p->leave(&ctx, true);
    setup();
    CHECK(p->enter(&ctx) == PROG_OK && chip.keys == 1);
    p->leave(&ctx, true);

    /* verify failure reports the address */
    setup();
    chip.corrupt = true; chip.corrupt_addr = 21;
    CHECK(p->enter(&ctx) == PROG_OK);
    ctx.mem = PROG_MEM_FLASH; ctx.erase = PROG_ERASE_NONE; ctx.addr = 0; ctx.len = 32;
    CHECK(run(p->write) == PROG_E_VERIFY);
    CHECK(ctx.at_addr == 21);
    p->leave(&ctx, true);

    /* config byte */
    setup();
    CHECK(p->enter(&ctx) == PROG_OK);
    job.src[0] = 0xFE;
    ctx.mem = PROG_MEM_CONFIG; ctx.addr = 0; ctx.len = 1; ctx.erase = PROG_ERASE_NONE;
    CHECK(run(p->write) == PROG_OK && chip.config == 0xFE);
    CHECK(run(p->read) == PROG_OK && job.dst[0] == 0xFE);
    p->leave(&ctx, true);

    /* locked chip: refused until a chip erase */
    setup();
    chip.lock = 0xFC; memset(chip.flash, 0x55, 64);
    CHECK(p->enter(&ctx) == PROG_OK);
    ctx.mem = PROG_MEM_FLASH; ctx.erase = PROG_ERASE_NONE; ctx.addr = 0; ctx.len = 16;
    CHECK(run(p->read) == PROG_E_LOCKED);
    CHECK(run(p->write) == PROG_E_LOCKED);
    ctx.erase = PROG_ERASE_CHIP;
    CHECK(p->erase(&ctx) == PROG_OK);
    ctx.erase = PROG_ERASE_NONE;
    CHECK(run(p->write) == PROG_OK && memcmp(chip.flash, job.src, 16) == 0);
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

    /* chip that stays busy */
    setup();
    CHECK(p->enter(&ctx) == PROG_OK);
    chip.stuck_busy = true;
    ctx.mem = PROG_MEM_FLASH; ctx.erase = PROG_ERASE_CHIP;
    CHECK(p->erase(&ctx) == PROG_E_TIMEOUT);
    p->leave(&ctx, true);

    /* bad args */
    setup();
    CHECK(p->enter(&ctx) == PROG_OK);
    ctx.mem = PROG_MEM_EEPROM; ctx.len = 4;
    CHECK(p->write(&ctx) == PROG_E_ARGS);
    ctx.mem = PROG_MEM_FLASH; ctx.page = 0;
    CHECK(p->write(&ctx) == PROG_E_ARGS);
    p->leave(&ctx, true);
    CHECK(sim.contention == 0);

    CHECK(sim.low_at_flush == 0);
    printf("test_prog_tpi: %s\n", failures ? "FAILED" : "ok");
    return failures ? 1 : 0;
}
