/* Host test for prog_c2.c against a bit-level model of a C2 chip (prog_sim.h).
 *
 * The model follows the C2 frames cycle by cycle: it latches host bits on the rising clock edge and
 * drives its own bits after the RISING edge too (AN127 1.4), so a host that samples while the clock
 * is low reads the previous bit. It has the registers DEVICEID/REVID/FPCTL/FPDAT, the
 * FPI (device erase with its confirm bytes, page erase, block read/write, derivative), a flash with
 * AND-only programming, a lock byte, and knobs for the failure cases. */
#include "prog_flash.h"
#include "prog_c2.h"
#include "prog_sim.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static int failures;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); failures++; } } while (0)

#define FLASH_SZ   0x2000u
#define LOCK_ADDR  0x1FFFu
#define PAGE       512u
#define AR_FPCTL   0x02u

typedef enum { S_START, S_INS, S_ARW, S_ARR, S_LEN, S_DW, S_WAIT, S_DR, S_STOP } c2st_t;
typedef enum { F_IDLE, F_CONF, F_PAGE, F_PAGE0, F_PARAMS_R, F_PARAMS_W, F_DATA_W, F_DIRECT } fpist_t;

typedef struct {
    /* configuration */
    bool     silent;             /* answers nothing: the line floats high */
    uint8_t  devid, revid, deriv;
    uint8_t  fpdat_addr;
    bool     no_deriv;           /* command 0x02 answers "invalid" */
    int      corrupt_at;         /* flash address read back with bit 0 flipped, -1: none */
    bool     hang_erase;         /* device erase never finishes */
    bool     hang_wait;          /* the WAIT field never ends */
    uint32_t wait_cycles;        /* WAIT cycles before the chip is ready */
    uint8_t  need[6][2]; unsigned nneed;   /* (SFR, value) pairs required before erase/write */
    uint8_t  refuse_stat;        /* nonzero: page erase answers this status for the 0x00 byte */

    /* memory */
    uint8_t  flash[FLASH_SZ];
    bool     locked;

    /* observation */
    unsigned resets, early_fpdat, erase_dev, erase_page, bad_confirm, fpctl_enable;
    uint32_t max_low_us;         /* longest clock-low that was not a reset */
    unsigned last_page;
    uint8_t  fpctl_log[8]; unsigned nfpctl;
    uint8_t  sfr_log[16][2]; unsigned nsfr;   /* writes to other C2 addresses and direct writes */
    unsigned unprepared;         /* erase/write commands that came before the required setup */

    /* C2 state */
    int      prev_clk;
    uint32_t low_us;
    c2st_t   st; unsigned nb; uint32_t acc; unsigned ins;
    uint8_t  ar, outv; bool is_read; int drive, last_drive; unsigned wait_left;
    unsigned fpctl_stage; bool enabled; uint32_t enable_wait_us, busy_us;
    /* FPI */
    fpist_t  fs; unsigned nconf, np; uint8_t pbuf[3]; uint32_t waddr, wremain;
    uint8_t  out[300]; unsigned nout, oh;
} c2m_t;

static void out_push(c2m_t *m, uint8_t b) { m->out[m->nout++] = b; }

static void sfr_note(c2m_t *m, uint8_t a, uint8_t v) {
    if (m->nsfr < 16) { m->sfr_log[m->nsfr][0] = a; m->sfr_log[m->nsfr][1] = v; m->nsfr++; }
}
static bool sfr_seen(const c2m_t *m, uint8_t a, uint8_t v) {
    for (unsigned i = 0; i < m->nsfr; i++) if (m->sfr_log[i][0] == a && m->sfr_log[i][1] == v) return true;
    return false;
}
static bool prepared(const c2m_t *m) {
    for (unsigned i = 0; i < m->nneed; i++) if (!sfr_seen(m, m->need[i][0], m->need[i][1])) return false;
    return true;
}

static void fpi_byte(c2m_t *m, uint8_t b) {
    switch (m->fs) {
    case F_IDLE:
        m->nout = m->oh = 0;
        switch (b) {
        case 0x02:
            if (m->no_deriv) out_push(m, 0x01); else { out_push(m, 0x0D); out_push(m, m->deriv); }
            break;
        case 0x03: if (!prepared(m)) m->unprepared++;
            out_push(m, 0x0D); m->fs = F_CONF; m->nconf = 0; m->np = 0; break;
        case 0x06: out_push(m, m->locked ? 0x80 : 0x0D); if (!m->locked) { m->fs = F_PARAMS_R; m->np = 0; } break;
        case 0x07: if (!prepared(m)) m->unprepared++;
            out_push(m, m->locked ? 0x80 : 0x0D); if (!m->locked) { m->fs = F_PARAMS_W; m->np = 0; } break;
        case 0x08: if (!prepared(m)) m->unprepared++;
            out_push(m, m->locked ? 0x80 : 0x0D); if (!m->locked) m->fs = F_PAGE; break;
        case 0x0A: out_push(m, 0x0D); m->fs = F_DIRECT; m->np = 0; break;
        default: out_push(m, 0x01); break;
        }
        break;
    case F_CONF: {
        static const uint8_t conf[3] = { 0xDE, 0xAD, 0xA5 };
        if (b != conf[m->nconf]) m->np = 1;          /* np used as the "bad" flag */
        if (++m->nconf == 3) {
            m->fs = F_IDLE;
            if (m->np) { m->bad_confirm++; out_push(m, 0x82); }
            else {
                m->erase_dev++;
                memset(m->flash, 0xFF, sizeof m->flash);
                m->locked = false;
                m->busy_us = m->hang_erase ? 0xFFFFFFFFu : 30000u;
                out_push(m, 0x0D);
            }
        }
        break; }
    case F_PAGE:                  /* the page number: accepted, the erase waits for the 0x00 byte */
        m->last_page = b;
        m->fs = F_PAGE0;
        out_push(m, 0x0D);
        break;
    case F_PAGE0:                 /* 0x00 starts the erase; the third status comes when it is done */
        m->fs = F_IDLE;
        if (m->refuse_stat) { out_push(m, m->refuse_stat); break; }
        m->erase_page++;
        if ((m->last_page + 1u) * PAGE <= FLASH_SZ) memset(&m->flash[m->last_page * PAGE], 0xFF, PAGE);
        m->busy_us = 40000;
        out_push(m, 0x0D);
        break;
    case F_DIRECT:                /* addr, length 1, data */
        m->pbuf[m->np++] = b;
        if (m->np == 3) { sfr_note(m, m->pbuf[0], m->pbuf[2]); m->fs = F_IDLE; }
        break;
    case F_PARAMS_R:
    case F_PARAMS_W:
        m->pbuf[m->np++] = b;
        if (m->np == 3) {
            uint32_t a = ((uint32_t)m->pbuf[0] << 8) | m->pbuf[1];
            uint32_t n = m->pbuf[2] ? m->pbuf[2] : 256u;
            m->nout = m->oh = 0;
            out_push(m, 0x0D);
            if (m->fs == F_PARAMS_R) {
                for (uint32_t i = 0; i < n; i++) {
                    uint8_t v = a + i < FLASH_SZ ? m->flash[a + i] : 0xFF;
                    if ((int)(a + i) == m->corrupt_at) v ^= 1u;
                    out_push(m, v);
                }
                m->fs = F_IDLE;
            } else { m->waddr = a; m->wremain = n; m->fs = F_DATA_W; }
        }
        break;
    case F_DATA_W:
        if (m->waddr < FLASH_SZ) m->flash[m->waddr] &= b;      /* programming only clears bits */
        if (m->waddr == LOCK_ADDR && m->flash[LOCK_ADDR] != 0xFF) m->locked = true;
        m->waddr++;
        m->busy_us = 30;
        if (--m->wremain == 0) { out_push(m, 0x0D); m->fs = F_IDLE; }
        break;
    }
}

static bool out_ready(c2m_t *m) { return m->busy_us == 0 && m->oh < m->nout; }

static void dr_write_done(c2m_t *m, uint8_t v) {
    if (m->ar == AR_FPCTL) {
        if (m->nfpctl < sizeof m->fpctl_log) m->fpctl_log[m->nfpctl++] = v;
        if (v == 0x02) m->fpctl_stage = 1;
        else if (v == 0x04 && m->fpctl_stage == 1) m->fpctl_stage = 2;
        else if (v == 0x01 && m->fpctl_stage >= 1) { m->enabled = true; m->fpctl_enable++; m->enable_wait_us = 20000; m->fpctl_stage = 0; }
        else m->fpctl_stage = 0;
    } else if (m->ar == m->fpdat_addr) {
        if (!m->enabled || m->enable_wait_us > 0 || m->busy_us > 0) { m->early_fpdat++; return; }
        fpi_byte(m, v);
    } else if (m->ar != 0 && m->ar != 1) {
        sfr_note(m, m->ar, v);
    }
}

static void chip_reset(c2m_t *m) {
    m->resets++;
    m->st = S_START; m->nb = 0; m->ar = 0; m->drive = -1;
    m->enabled = false; m->fpctl_stage = 0; m->fs = F_IDLE; m->nout = m->oh = 0; m->busy_us = 0;
    m->enable_wait_us = 0; m->nsfr = 0;
}

/* The chip's drive for the cycle that this rising edge starts: valid from the edge until the next
   rising edge (the falling edge changes nothing), as AN127 1.4 has it. */
static int drive_for(c2m_t *m) {
    switch (m->st) {
    case S_ARR: case S_DR: return (m->outv >> m->nb) & 1u;
    case S_WAIT:
        if (m->hang_wait) return 0;
        if (m->wait_left > 0) { m->wait_left--; return 0; }
        if (m->is_read && m->ar == m->fpdat_addr && !out_ready(m)) return 0;
        return 1;
    default: return -1;
    }
}

static void rising(c2m_t *m, int bit) {
    int d = drive_for(m);
    m->drive = d;
    switch (m->st) {
    case S_START: m->st = S_INS; m->nb = 0; m->acc = 0; break;
    case S_INS:
        m->acc |= (uint32_t)bit << m->nb;
        if (++m->nb == 2) {
            m->ins = m->acc; m->nb = 0; m->acc = 0;
            if (m->ins == 3) m->st = S_ARW;
            else if (m->ins == 2) {
                m->st = S_ARR;
                m->outv = (uint8_t)((out_ready(m) ? 1u : 0u) | (m->busy_us ? 2u : 0u));
            } else { m->is_read = (m->ins == 0); m->st = S_LEN; }
        }
        break;
    case S_ARW:
        m->acc |= (uint32_t)bit << m->nb;
        if (++m->nb == 8) { m->ar = (uint8_t)m->acc; m->st = S_STOP; }
        break;
    case S_ARR: if (++m->nb == 8) m->st = S_STOP; break;
    case S_LEN:
        if (++m->nb == 2) {
            m->nb = 0; m->acc = 0;
            if (m->is_read) { m->st = S_WAIT; m->wait_left = m->wait_cycles; }
            else m->st = S_DW;
        }
        break;
    case S_DW:
        m->acc |= (uint32_t)bit << m->nb;
        if (++m->nb == 8) { dr_write_done(m, (uint8_t)m->acc); m->st = S_WAIT; m->wait_left = m->wait_cycles; m->nb = 0; }
        break;
    case S_WAIT:
        if (d == 1) {
            m->nb = 0;
            if (m->is_read) {
                if (m->ar == 0) m->outv = m->devid;
                else if (m->ar == 1) m->outv = m->revid;
                else if (m->ar == m->fpdat_addr && m->oh < m->nout) m->outv = m->out[m->oh++];
                else m->outv = 0xFF;
                m->st = S_DR;
            } else m->st = S_STOP;
        }
        break;
    case S_DR: if (++m->nb == 8) m->st = S_STOP; break;
    case S_STOP: m->st = S_START; break;
    }
}

static void m_on_pins(prog_sim_t *s, int clk, int pod_data) {
    c2m_t *m = s->chip;
    if (m->silent) return;
    if (clk == m->prev_clk) return;
    m->prev_clk = clk;
    if (clk == 0) m->low_us = 0;
    else {
        if (m->low_us >= 20) chip_reset(m);
        else {
            if (m->low_us > m->max_low_us) m->max_low_us = m->low_us;
            rising(m, pod_data < 0 ? 1 : pod_data);
        }
    }
}

static int m_chip_data(prog_sim_t *s) {
    c2m_t *m = s->chip;
    return m->silent ? -1 : m->drive;
}

static void m_on_time(prog_sim_t *s, uint32_t us) {
    c2m_t *m = s->chip;
    if (s->clk == 0) m->low_us += us;
    if (m->enable_wait_us) m->enable_wait_us = m->enable_wait_us > us ? m->enable_wait_us - us : 0;
    if (m->busy_us && m->busy_us != 0xFFFFFFFFu) m->busy_us = m->busy_us > us ? m->busy_us - us : 0;
}

static const prog_sim_model_t k_model = { .on_pins = m_on_pins, .chip_data = m_chip_data, .on_time = m_on_time };

/* ---- harness ---- */

typedef struct {
    prog_sim_t sim; prog_wire_ops_t ops; c2m_t chip;
    prog_ctx_t ctx;
    uint8_t img[FLASH_SZ]; uint8_t rd[FLASH_SZ];
    int fail_src, fail_dst;
} rig_t;

static int rig_src(void *job, uint32_t off, uint8_t *buf, uint32_t n) {
    rig_t *r = job;
    if (r->fail_src || off + n > sizeof r->img) return -1;
    memcpy(buf, r->img + off, n);
    return 0;
}
static int rig_dst(void *job, uint32_t off, const uint8_t *buf, uint32_t n) {
    rig_t *r = job;
    if (r->fail_dst || off + n > sizeof r->rd) return -1;
    memcpy(r->rd + off, buf, n);
    return 0;
}

/* A layer over the sim's ops that watches what the protocol queues, as the feed does: the clock
   level at each sample, the set(1,..) bytes between a rising edge and the sample (each is one
   feed byte, >= 40 ns assumed, so 3 give 120 ns), and the longest run of bytes queued with the
   clock low (prog_wire.c flushes automatically only with the clock high, with 16 bytes of room). */
static struct {
    prog_wire_ops_t orig;
    int clk; unsigned since, run, max_run, samples, low_samples, min_hold;
} g_w;

static void w_set(void *io, int clk, int d) {
    if (clk) { g_w.since = g_w.clk ? g_w.since + 1 : 0; g_w.run = 0; }
    else if (++g_w.run > g_w.max_run) g_w.max_run = g_w.run;
    g_w.clk = clk;
    g_w.orig.set(io, clk, d);
}
static void w_drive(void *io, bool on) {
    if (!g_w.clk && ++g_w.run > g_w.max_run) g_w.max_run = g_w.run;
    g_w.orig.data_drive(io, on);
}
static void w_sample(void *io) {
    g_w.samples++;
    if (!g_w.clk) g_w.low_samples++;
    else if (g_w.since < g_w.min_hold) g_w.min_hold = g_w.since;
    if (!g_w.clk && ++g_w.run > g_w.max_run) g_w.max_run = g_w.run;
    g_w.orig.sample(io);
}

static void rig_init(rig_t *r) {
    memset(r, 0, sizeof *r);
    r->chip.devid = 0x32; r->chip.revid = 0x03; r->chip.deriv = 0x21;
    r->chip.fpdat_addr = PROG_C2_FPDAT_DEFAULT;
    r->chip.corrupt_at = -1;
    r->chip.prev_clk = 1; r->chip.drive = -1;
    r->chip.wait_cycles = 2;
    memset(r->chip.flash, 0xFF, sizeof r->chip.flash);
    prog_sim_init(&r->sim, &r->ops, &k_model, &r->chip);
    memset(&g_w, 0, sizeof g_w);
    g_w.orig = r->ops; g_w.clk = 1; g_w.min_hold = 1000;
    r->ops.set = w_set; r->ops.data_drive = w_drive; r->ops.sample = w_sample;
    r->ctx.w = &r->ops;
    r->ctx.page = PAGE;
    r->ctx.src = rig_src; r->ctx.dst = rig_dst; r->ctx.job = r;
    for (unsigned i = 0; i < sizeof r->img; i++) r->img[i] = (uint8_t)((i * 131u + 7u) ^ (i >> 5));
}

/* Run a step to completion like the job does. */
static int run(int (*step)(prog_ctx_t *), prog_ctx_t *ctx, unsigned *calls) {
    unsigned n = 0;
    int rc;
    do { rc = step(ctx); n++; } while (rc == PROG_RUNNING && n < 100000);
    if (calls) *calls = n;
    return rc;
}

/* The pin rules that hold for every test. */
static void check_pins(rig_t *r) {
    CHECK(r->sim.contention == 0);
    CHECK(r->sim.low_at_flush == 0);
    CHECK(r->chip.max_low_us <= 5);
    CHECK(r->sim.clk == 1);
}

#define PROTO prog_proto_c2

/* ---- tests ---- */

static void test_enter(void) {
    rig_t *r = malloc(sizeof *r); rig_init(r);
    CHECK(run(PROTO.enter, &r->ctx, NULL) == PROG_OK);
    CHECK(r->ctx.id_len == 3);
    CHECK(r->ctx.id[0] == 0x32 && r->ctx.id[1] == 0x03 && r->ctx.id[2] == 0x21);
    CHECK(r->chip.resets == 1);
    CHECK(r->sim.clk_low_max_us >= 20 && r->sim.clk_low_max_us <= 40);   /* the reset */
    CHECK(r->chip.nfpctl == 3 && r->chip.fpctl_log[0] == 0x02 && r->chip.fpctl_log[1] == 0x04 && r->chip.fpctl_log[2] == 0x01);
    CHECK(r->chip.enabled && r->chip.early_fpdat == 0);
    check_pins(r);
    free(r);

    /* no derivative command: ID is two bytes */
    r = malloc(sizeof *r); rig_init(r); r->chip.no_deriv = true;
    CHECK(run(PROTO.enter, &r->ctx, NULL) == PROG_OK);
    CHECK(r->ctx.id_len == 2 && r->ctx.id[0] == 0x32);
    CHECK(r->chip.early_fpdat == 0);
    check_pins(r);
    free(r);

    /* FPDAT at another C2 address (flags) */
    r = malloc(sizeof *r); rig_init(r); r->chip.fpdat_addr = 0xAD; r->ctx.flags = 0xAD;
    CHECK(run(PROTO.enter, &r->ctx, NULL) == PROG_OK);
    CHECK(r->ctx.id_len == 3 && r->ctx.id[2] == 0x21);
    free(r);
}

/* Reads come from the chip after the rising edge: sampled with the clock high, >= 3 feed bytes
   (120 ns at the assumed 40 ns) after the edge, and no long clock-low run for an auto flush. */
static void test_read_timing(void) {
    rig_t *r = malloc(sizeof *r); rig_init(r);
    CHECK(run(PROTO.enter, &r->ctx, NULL) == PROG_OK);
    CHECK(r->ctx.id[0] == 0x32 && r->ctx.id[1] == 0x03);   /* a sample one bit early would differ */
    r->ctx.addr = 0; r->ctx.len = 300;
    CHECK(run(PROTO.read, &r->ctx, NULL) == PROG_OK);
    CHECK(memcmp(r->rd, r->chip.flash, 300) == 0);
    CHECK(g_w.samples > 1000);
    CHECK(g_w.low_samples == 0);
    CHECK(g_w.min_hold >= 3);
    CHECK(g_w.max_run <= 8);
    check_pins(r);
    free(r);
}

static void test_no_answer(void) {
    rig_t *r = malloc(sizeof *r); rig_init(r); r->chip.silent = true;
    CHECK(run(PROTO.enter, &r->ctx, NULL) == PROG_E_NO_ANSWER);
    check_pins(r);
    free(r);

    r = malloc(sizeof *r); rig_init(r); r->chip.hang_wait = true;       /* a chip stuck in WAIT */
    CHECK(run(PROTO.enter, &r->ctx, NULL) == PROG_E_NO_ANSWER);
    check_pins(r);
    free(r);
}

static void test_leave(void) {
    rig_t *r = malloc(sizeof *r); rig_init(r);
    CHECK(run(PROTO.enter, &r->ctx, NULL) == PROG_OK);
    PROTO.leave(&r->ctx, false);
    CHECK(r->chip.resets == 1);                 /* just released */
    PROTO.leave(&r->ctx, true);
    CHECK(r->chip.resets == 2);                 /* reset pulse: the program starts */
    check_pins(r);
    free(r);
}

static void fill_chip(rig_t *r, uint8_t v) { memset(r->chip.flash, v, sizeof r->chip.flash); }

static void test_erase_device(void) {
    rig_t *r = malloc(sizeof *r); rig_init(r);
    fill_chip(r, 0x5A);
    CHECK(run(PROTO.enter, &r->ctx, NULL) == PROG_OK);
    r->ctx.erase = PROG_ERASE_CHIP;
    CHECK(run(PROTO.erase, &r->ctx, NULL) == PROG_OK);
    CHECK(r->chip.erase_dev == 1 && r->chip.bad_confirm == 0);
    for (unsigned i = 0; i < FLASH_SZ; i++) if (r->chip.flash[i] != 0xFF) { CHECK(0); break; }
    CHECK(r->ctx.done == 1 && r->ctx.total == 1);
    CHECK(r->chip.early_fpdat == 0);

    /* ERASE_NONE does nothing */
    fill_chip(r, 0x11);
    r->ctx.erase = PROG_ERASE_NONE;
    CHECK(run(PROTO.erase, &r->ctx, NULL) == PROG_OK);
    CHECK(r->chip.flash[0] == 0x11 && r->chip.erase_dev == 1);
    check_pins(r);
    free(r);

    /* the model refuses a wrong confirm sequence (so the pod's bytes are what erased the chip above) */
    c2m_t m; memset(&m, 0, sizeof m); m.corrupt_at = -1; m.fpdat_addr = PROG_C2_FPDAT_DEFAULT;
    memset(m.flash, 0x00, sizeof m.flash);
    fpi_byte(&m, 0x03); fpi_byte(&m, 0xDE); fpi_byte(&m, 0xAD); fpi_byte(&m, 0xA6);
    CHECK(m.bad_confirm == 1 && m.erase_dev == 0 && m.flash[0] == 0x00);

    /* a device erase that never ends */
    r = malloc(sizeof *r); rig_init(r); r->chip.hang_erase = true;
    CHECK(run(PROTO.enter, &r->ctx, NULL) == PROG_OK);
    r->ctx.erase = PROG_ERASE_CHIP;
    CHECK(run(PROTO.erase, &r->ctx, NULL) == PROG_E_TIMEOUT);
    check_pins(r);
    free(r);
}

static void test_erase_page(void) {
    rig_t *r = malloc(sizeof *r); rig_init(r);
    fill_chip(r, 0x00);
    CHECK(run(PROTO.enter, &r->ctx, NULL) == PROG_OK);
    r->ctx.erase = PROG_ERASE_PAGE;
    r->ctx.addr = 0x300; r->ctx.len = 0x400;        /* pages 1, 2, 3 (0x300..0x6FF) */
    unsigned calls = 0;
    CHECK(run(PROTO.erase, &r->ctx, &calls) == PROG_OK);
    CHECK(calls == 3 && r->ctx.total == 3 && r->ctx.done == 3);
    CHECK(r->chip.erase_page == 3 && r->chip.erase_dev == 0);
    CHECK(r->chip.flash[0x1FF] == 0x00 && r->chip.flash[0x200] == 0xFF);
    CHECK(r->chip.flash[0x7FF] == 0xFF && r->chip.flash[0x800] == 0x00);
    CHECK(r->chip.early_fpdat == 0);

    /* a second erase starts over */
    r->ctx.addr = 0; r->ctx.len = 1;
    CHECK(run(PROTO.erase, &r->ctx, &calls) == PROG_OK);
    CHECK(calls == 1 && r->chip.flash[0] == 0xFF && r->chip.last_page == 0);

    /* bad arguments */
    r->ctx.page = 0; r->ctx.addr = 0; r->ctx.len = 16;
    CHECK(PROTO.erase(&r->ctx) == PROG_E_ARGS);
    r->ctx.page = PAGE; r->ctx.len = 0;
    CHECK(PROTO.erase(&r->ctx) == PROG_E_ARGS);
    r->ctx.addr = 0xFFF0; r->ctx.len = 0x20;
    CHECK(PROTO.erase(&r->ctx) == PROG_E_ARGS);
    r->ctx.addr = 0; r->ctx.len = 16; r->ctx.mem = PROG_MEM_EEPROM;
    CHECK(PROTO.erase(&r->ctx) == PROG_E_ARGS);
    check_pins(r);
    free(r);
}

static void test_write_read(void) {
    rig_t *r = malloc(sizeof *r); rig_init(r);
    CHECK(run(PROTO.enter, &r->ctx, NULL) == PROG_OK);
    r->ctx.erase = PROG_ERASE_CHIP;
    CHECK(run(PROTO.erase, &r->ctx, NULL) == PROG_OK);

    /* 1000 bytes at 0x100: blocks stop at the 512 byte page ends (0x100..0x1FF, 0x200..0x3FF, 0x400..0x4E7) */
    r->ctx.mem = PROG_MEM_FLASH; r->ctx.addr = 0x100; r->ctx.len = 1000;
    unsigned calls = 0;
    CHECK(run(PROTO.write, &r->ctx, &calls) == PROG_OK);
    CHECK(calls == 4 && r->ctx.total == 4 && r->ctx.done == 4);   /* 256 + 256 + 256 + 232 */
    CHECK(memcmp(&r->chip.flash[0x100], r->img, 1000) == 0);
    CHECK(r->chip.flash[0xFF] == 0xFF && r->chip.flash[0x100 + 1000] == 0xFF);
    CHECK(r->chip.early_fpdat == 0);

    /* read it back, through dst */
    r->ctx.addr = 0x100; r->ctx.len = 1000;
    CHECK(run(PROTO.read, &r->ctx, &calls) == PROG_OK);
    CHECK(calls == 4 && r->ctx.total == 4 && r->ctx.done == 4);
    CHECK(memcmp(r->rd, &r->chip.flash[0x100], 1000) == 0);

    /* a short odd-sized range, not page aligned */
    r->ctx.addr = 0x7FD; r->ctx.len = 7;
    CHECK(run(PROTO.read, &r->ctx, &calls) == PROG_OK);
    CHECK(calls == 1 && memcmp(r->rd, &r->chip.flash[0x7FD], 7) == 0);
    check_pins(r);
    free(r);

    /* 256-byte block with length byte 0, and the top of the 16 bit space */
    r = malloc(sizeof *r); rig_init(r);
    CHECK(run(PROTO.enter, &r->ctx, NULL) == PROG_OK);
    r->ctx.addr = 0; r->ctx.len = 256; r->ctx.page = 0;     /* page 0: no page limit, one block */
    CHECK(run(PROTO.write, &r->ctx, &calls) == PROG_OK && calls == 1);
    CHECK(memcmp(r->chip.flash, r->img, 256) == 0);
    r->ctx.addr = 0xFFFF; r->ctx.len = 1;
    CHECK(run(PROTO.read, &r->ctx, NULL) == PROG_OK && r->rd[0] == 0xFF);
    r->ctx.addr = 0xFFFF; r->ctx.len = 2;
    CHECK(PROTO.read(&r->ctx) == PROG_E_ARGS);
    r->ctx.addr = 0; r->ctx.len = 0;
    CHECK(PROTO.write(&r->ctx) == PROG_E_ARGS);
    r->ctx.mem = PROG_MEM_USER; r->ctx.len = 4;
    CHECK(PROTO.write(&r->ctx) == PROG_E_ARGS);
    CHECK(PROTO.read(&r->ctx) == PROG_E_ARGS);
    check_pins(r);
    free(r);
}

static void test_verify_fail(void) {
    rig_t *r = malloc(sizeof *r); rig_init(r);
    CHECK(run(PROTO.enter, &r->ctx, NULL) == PROG_OK);
    r->chip.corrupt_at = 0x240;                       /* reads back with bit 0 flipped */
    r->ctx.addr = 0x200; r->ctx.len = 0x100;
    CHECK(run(PROTO.write, &r->ctx, NULL) == PROG_E_VERIFY);
    CHECK(r->ctx.at_addr == 0x240);
    check_pins(r);
    free(r);

    /* writing over programmed flash without an erase: bits cannot go back to 1 */
    r = malloc(sizeof *r); rig_init(r);
    r->chip.flash[0x305] = 0x00;
    CHECK(run(PROTO.enter, &r->ctx, NULL) == PROG_OK);
    r->ctx.addr = 0x300; r->ctx.len = 16;
    CHECK(run(PROTO.write, &r->ctx, NULL) == (r->img[5] == 0 ? PROG_OK : PROG_E_VERIFY));
    CHECK(r->img[5] != 0 && r->ctx.at_addr == 0x305);
    free(r);

    /* the staged image or the read-back area fails */
    r = malloc(sizeof *r); rig_init(r);
    CHECK(run(PROTO.enter, &r->ctx, NULL) == PROG_OK);
    r->ctx.addr = 0; r->ctx.len = 64;
    r->fail_src = 1;
    CHECK(run(PROTO.write, &r->ctx, NULL) == PROG_E_SRC);
    r->fail_src = 0; r->fail_dst = 1;
    PROTO.leave(&r->ctx, true);                    /* a failed job leaves: the chip drops the half-sent block */
    CHECK(run(PROTO.enter, &r->ctx, NULL) == PROG_OK);
    r->ctx.addr = 0; r->ctx.len = 64;
    CHECK(run(PROTO.read, &r->ctx, NULL) == PROG_E_SRC);
    /* a failed call leaves the state ready for a new one */
    r->fail_dst = 0;
    CHECK(run(PROTO.read, &r->ctx, NULL) == PROG_OK);
    check_pins(r);
    free(r);
}

static void test_lock(void) {
    rig_t *r = malloc(sizeof *r); rig_init(r);
    CHECK(run(PROTO.enter, &r->ctx, NULL) == PROG_OK);
    r->ctx.erase = PROG_ERASE_CHIP;
    CHECK(run(PROTO.erase, &r->ctx, NULL) == PROG_OK);
    r->ctx.addr = 0; r->ctx.len = 512;
    CHECK(run(PROTO.write, &r->ctx, NULL) == PROG_OK);
    /* lock byte last, as the config memory: written without read-back (it would be refused) */
    r->ctx.mem = PROG_MEM_CONFIG; r->ctx.addr = LOCK_ADDR; r->ctx.len = 1;
    r->img[0] = 0x00;
    CHECK(run(PROTO.write, &r->ctx, NULL) == PROG_OK);
    CHECK(r->chip.locked && r->chip.flash[LOCK_ADDR] == 0x00);
    PROTO.leave(&r->ctx, true);

    /* a new session: the ID still reads, flash access is refused */
    CHECK(run(PROTO.enter, &r->ctx, NULL) == PROG_OK && r->ctx.id[0] == 0x32);
    r->ctx.mem = PROG_MEM_FLASH; r->ctx.addr = 0; r->ctx.len = 64;
    CHECK(run(PROTO.read, &r->ctx, NULL) == PROG_E_LOCKED);
    CHECK(run(PROTO.write, &r->ctx, NULL) == PROG_E_LOCKED);
    r->ctx.erase = PROG_ERASE_PAGE;
    CHECK(run(PROTO.erase, &r->ctx, NULL) == PROG_E_LOCKED);
    CHECK(r->chip.locked);
    CHECK(r->chip.flash[0] != 0xFF);                   /* nothing was erased */

    /* only a device erase unlocks */
    r->ctx.erase = PROG_ERASE_CHIP;
    CHECK(run(PROTO.erase, &r->ctx, NULL) == PROG_OK);
    CHECK(!r->chip.locked);
    CHECK(run(PROTO.read, &r->ctx, NULL) == PROG_OK);
    CHECK(r->rd[0] == 0xFF && r->rd[63] == 0xFF);
    check_pins(r);
    free(r);
}

static void test_busy_chip(void) {
    /* a chip that keeps WAIT low a long time (but not forever) still works */
    rig_t *r = malloc(sizeof *r); rig_init(r);
    r->chip.wait_cycles = 40;
    CHECK(run(PROTO.enter, &r->ctx, NULL) == PROG_OK && r->ctx.id[0] == 0x32);
    r->ctx.addr = 0; r->ctx.len = 40;
    CHECK(run(PROTO.write, &r->ctx, NULL) == PROG_OK);
    CHECK(memcmp(r->chip.flash, r->img, 40) == 0);
    check_pins(r);
    free(r);
}

/* Page erase: page number, status, 0x00, third status after the erase (AN127 3.3). */
static void test_erase_page_third_status(void) {
    rig_t *r = malloc(sizeof *r); rig_init(r);
    fill_chip(r, 0x00);
    CHECK(run(PROTO.enter, &r->ctx, NULL) == PROG_OK);
    r->ctx.erase = PROG_ERASE_PAGE; r->ctx.addr = 0x200; r->ctx.len = 1;
    uint32_t t0 = r->sim.us;
    CHECK(run(PROTO.erase, &r->ctx, NULL) == PROG_OK);
    CHECK(r->chip.erase_page == 1 && r->chip.flash[0x200] == 0xFF);   /* only the 0x00 byte erases */
    CHECK(r->sim.us - t0 >= 40000);                                    /* waited for the third status */
    CHECK(r->chip.early_fpdat == 0);
    /* the chip refuses at the 0x00 byte: raw status reported, nothing erased */
    r->chip.refuse_stat = 0x02;
    r->ctx.addr = 0x400;
    CHECK(run(PROTO.erase, &r->ctx, NULL) == PROG_E_PROTOCOL);
    CHECK(r->ctx.at_addr == 0x02 && r->chip.flash[0x400] == 0x00);
    r->chip.refuse_stat = 0;
    CHECK(run(PROTO.erase, &r->ctx, NULL) == PROG_OK && r->chip.flash[0x400] == 0xFF);
    check_pins(r);
    free(r);
}

/* Device setup before erase/write (AN127 Table 3.6) keyed by the device ID. */
typedef struct { uint8_t id, fpdat; unsigned n; uint8_t seq[5][2]; bool known; } setup_row_t;
static const setup_row_t k_rows[] = {
    { 0x30, 0xB4, 3, {{0xFF,0x80},{0xEF,0x02},{0xA9,0x00}}, true },                  /* BB1 */
    { 0x32, 0xB4, 3, {{0xFF,0x80},{0xEF,0x02},{0xA9,0x00}}, true },                  /* BB2, UB1 */
    { 0x34, 0xB4, 3, {{0xFF,0x80},{0xEF,0x02},{0xA9,0x00}}, true },                  /* BB3, LB1 */
    { 0x28, 0xAD, 4, {{0xB6,0x90},{0xFF,0x80},{0xEF,0x02},{0xA9,0x03}}, true },      /* UB2, F38x */
    { 0x25, 0xB4, 4, {{0xB6,0x40},{0xFF,0x80},{0xEF,0x02},{0xA9,0x04}}, true },      /* SB1 (direct) */
    { 0x16, 0xB4, 3, {{0xA7,0x00},{0xB2,0x8F},{0xA9,0x00}}, true },                  /* SB2 (direct) */
    { 0x77, 0xB4, 3, {{0xFF,0x80},{0xEF,0x02},{0xA9,0x00}}, false },                 /* not in the table */
};

static void test_device_setup(void) {
    for (unsigned k = 0; k < sizeof k_rows / sizeof k_rows[0]; k++) {
        const setup_row_t *row = &k_rows[k];
        for (int op = 0; op < 3; op++) {              /* page erase, device erase, write */
            rig_t *r = malloc(sizeof *r); rig_init(r);
            r->chip.devid = row->id; r->chip.fpdat_addr = row->fpdat;
            r->chip.nneed = row->n;
            for (unsigned i = 0; i < row->n; i++) { r->chip.need[i][0] = row->seq[i][0]; r->chip.need[i][1] = row->seq[i][1]; }
            CHECK(run(PROTO.enter, &r->ctx, NULL) == PROG_OK);
            CHECK(r->chip.nsfr == 0);                  /* entering touches no SFR */
            r->ctx.addr = 0x200; r->ctx.len = 64;
            r->ctx.erase = op == 1 ? PROG_ERASE_CHIP : PROG_ERASE_PAGE;
            int rc = op == 2 ? run(PROTO.write, &r->ctx, NULL) : run(PROTO.erase, &r->ctx, NULL);
            CHECK(rc == PROG_OK);
            CHECK(r->chip.nsfr == row->n);
            for (unsigned i = 0; i < row->n && i < r->chip.nsfr; i++)
                CHECK(r->chip.sfr_log[i][0] == row->seq[i][0] && r->chip.sfr_log[i][1] == row->seq[i][1]);
            CHECK(r->chip.unprepared == 0 && r->chip.early_fpdat == 0);
            if (op == 2) CHECK(memcmp(&r->chip.flash[0x200], r->img, 64) == 0);   /* FPI works after the SFR writes */
            /* a second call does not repeat the setup */
            unsigned n = r->chip.nsfr;
            r->ctx.erase = PROG_ERASE_PAGE;
            CHECK(run(PROTO.erase, &r->ctx, NULL) == PROG_OK && r->chip.nsfr == n);
            check_pins(r);
            free(r);
        }
    }
    /* reading needs none; the skip flag skips it */
    rig_t *r = malloc(sizeof *r); rig_init(r);
    r->chip.nneed = 1; r->chip.need[0][0] = 0xFF; r->chip.need[0][1] = 0x80;
    CHECK(run(PROTO.enter, &r->ctx, NULL) == PROG_OK);
    r->ctx.addr = 0; r->ctx.len = 16;
    CHECK(run(PROTO.read, &r->ctx, NULL) == PROG_OK && r->chip.nsfr == 0);
    r->ctx.flags = C2_FLAG_NO_SETUP;
    CHECK(run(PROTO.write, &r->ctx, NULL) == PROG_OK);
    CHECK(r->chip.nsfr == 0 && r->chip.unprepared == 1);
    free(r);
    /* an unknown ID is still programmed (common EFM8 sequence, FPDAT 0xB4) and says so in the phase */
    r = malloc(sizeof *r); rig_init(r);
    r->chip.devid = 0x77;
    CHECK(run(PROTO.enter, &r->ctx, NULL) == PROG_OK);
    r->ctx.addr = 0; r->ctx.len = 16;
    CHECK(run(PROTO.write, &r->ctx, NULL) == PROG_OK);
    CHECK(r->chip.nsfr == 3);
    free(r);
}

/* FPDAT is picked from the device ID (AN127 Table 3.5); a host flag overrides. */
static void test_fpdat_from_id(void) {
    static const struct { uint8_t id, fpdat; } t[] = {
        {0x28, 0xAD}, {0x0F, 0xAD}, {0x18, 0xAD}, {0x19, 0xAD}, {0x1A, 0xAD},
        {0x32, 0xB4}, {0x30, 0xB4}, {0x04, 0xB4}, {0x77, 0xB4}, {0x2A, 0xB4},
    };
    for (unsigned i = 0; i < sizeof t / sizeof t[0]; i++) {
        rig_t *r = malloc(sizeof *r); rig_init(r);
        r->chip.devid = t[i].id; r->chip.fpdat_addr = t[i].fpdat;
        CHECK(run(PROTO.enter, &r->ctx, NULL) == PROG_OK);
        CHECK(r->ctx.id_len == 3 && r->ctx.id[2] == 0x21);       /* the derivative came through FPDAT */
        CHECK(r->chip.early_fpdat == 0);
        free(r);
    }
    rig_t *r = malloc(sizeof *r); rig_init(r);
    r->chip.devid = 0x28; r->chip.fpdat_addr = 0xB4; r->ctx.flags = 0xB4;   /* flag wins over the ID */
    CHECK(run(PROTO.enter, &r->ctx, NULL) == PROG_OK && r->ctx.id_len == 3);
    free(r);
}

/* The raw FPI status reaches the caller. */
static void test_raw_status(void) {
    rig_t *r = malloc(sizeof *r); rig_init(r);
    r->chip.locked = true;
    CHECK(run(PROTO.enter, &r->ctx, NULL) == PROG_OK);
    r->ctx.addr = 0; r->ctx.len = 16;
    CHECK(run(PROTO.read, &r->ctx, NULL) == PROG_E_LOCKED && r->ctx.at_addr == 0x80);
    r->ctx.at_addr = 0;
    r->ctx.erase = PROG_ERASE_PAGE;
    CHECK(run(PROTO.erase, &r->ctx, NULL) == PROG_E_LOCKED && r->ctx.at_addr == 0x80);
    free(r);
}

int main(void) {
    CHECK(strcmp(PROTO.name, "c2") == 0);
    test_enter();
    test_read_timing();
    test_no_answer();
    test_leave();
    test_erase_device();
    test_erase_page();
    test_write_read();
    test_verify_fail();
    test_lock();
    test_busy_chip();
    test_erase_page_third_status();
    test_device_setup();
    test_fpdat_from_id();
    test_raw_status();
    printf("test_prog_c2: %s\n", failures ? "FAILED" : "ok");
    return failures ? 1 : 0;
}
