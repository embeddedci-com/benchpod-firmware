/* Host test for ctrl_flash (xip_mod_ctrl_flash.c, ctrl_flash_names.c) against simulated loaders:
   an STM32 ROM DfuSe loader (sectors erased before they are written, the flash layout in the
   interface string, a busy state with a poll timeout, 2048-byte blocks), a plain DFU 1.1 loader,
   and the Teensy HalfKay loader (busy on the first block while it erases). */
#include "ctrl_flash.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); failures++; } } while (0)

/* ---- simulated device ---------------------------------------------------------------------- */

enum { DEV_DFUSE, DEV_DFU, DEV_TEENSY };

typedef struct {
    int      kind;
    uint8_t  flash[1024 * 1024];
    uint32_t flash_base;
    uint8_t  erased[64];          /* DfuSe sectors erased */
    uint32_t xfer;                /* the device's transfer size */
    uint8_t  alts;                /* alternate settings of the DFU interface (0 = 1) */
    const char *layout;
    /* DFU state */
    int      state, pending, busy_left, status;
    uint32_t ptr, plain_off;
    uint8_t  cmd[8];
    uint32_t cmd_len;
    uint8_t  data[4096];
    uint32_t data_len, data_block;
    bool     manifested, fail_status_at_block;
    uint32_t bad_block;
    /* Teensy */
    int      teensy_busy;
    bool     rebooted;
    uint32_t reports;
    /* the image */
    const uint8_t *img;
    uint32_t img_len;
    int      ctrl_calls;
} sim_dev_t;

static const uint32_t k_sectors[] = { 16384, 16384, 16384, 16384, 65536, 131072, 131072, 131072 };

static int sector_index(sim_dev_t *d, uint32_t addr, uint32_t *start)
{
    uint32_t a = d->flash_base;
    for (int i = 0; i < 8; i++) {
        if (addr >= a && addr < a + k_sectors[i]) { *start = a; return i; }
        a += k_sectors[i];
    }
    return -1;
}

static int put_utf16(uint8_t *out, const char *s)
{
    int n = (int)strlen(s);
    out[0] = (uint8_t)(2 + 2 * n); out[1] = 3;
    for (int i = 0; i < n; i++) { out[2 + 2 * i] = (uint8_t)s[i]; out[3 + 2 * i] = 0; }
    return 2 + 2 * n;
}

/* The configuration: one DFU interface with d->alts alternate settings (STM32 ROMs have one per
   memory: flash, option bytes, OTP, ...), then the DFU functional descriptor, which the DFU spec
   puts after all of them. */
static int config_desc(sim_dev_t *d, uint8_t *b)
{
    unsigned alts = d->alts ? d->alts : 1u, n = 9;
    for (unsigned a = 0; a < alts; a++, n += 9) {
        const uint8_t itf[] = { 9, 4, 0, (uint8_t)a, 0, 0xFE, 0x01, 0x02, (uint8_t)(4 + a) };
        memcpy(b + n, itf, 9);
    }
    const uint8_t fn[] = { 9, 0x21, 0x0B, 0xFF, 0x00, (uint8_t)d->xfer, (uint8_t)(d->xfer >> 8),
                           d->kind == DEV_DFUSE ? 0x1A : 0x10, 0x01 };
    memcpy(b + n, fn, 9);
    n += 9;
    const uint8_t cfg[] = { 9, 2, (uint8_t)n, 0, 1, 1, 0, 0x80, 50 };
    memcpy(b, cfg, 9);
    return (int)n;
}

static void dfu_do_pending(sim_dev_t *d)
{
    if (d->pending == 1) {          /* DfuSe command */
        uint32_t addr = d->cmd[1] | (d->cmd[2] << 8) | (d->cmd[3] << 16) | ((uint32_t)d->cmd[4] << 24);
        if (d->cmd[0] == 0x21) d->ptr = addr;
        else if (d->cmd[0] == 0x41) {
            uint32_t st;
            int i = sector_index(d, addr, &st);
            if (i < 0) { d->status = 0x08; d->state = 10; return; }
            memset(d->flash + (st - d->flash_base), 0xFF, k_sectors[i]);
            d->erased[i] = 1;
        }
    } else if (d->pending == 2) {   /* data */
        if (d->kind == DEV_DFUSE) {
            uint32_t addr = d->ptr + (d->data_block - 2) * d->xfer;
            for (uint32_t i = 0; i < d->data_len; i++) {
                uint32_t st;
                int s = sector_index(d, addr + i, &st);
                if (s < 0 || !d->erased[s]) { d->status = 0x03; d->state = 10; return; }   /* errWRITE */
                d->flash[addr + i - d->flash_base] &= d->data[i];
            }
        } else {
            memcpy(d->flash + d->plain_off, d->data, d->data_len);
            d->plain_off += d->data_len;
        }
    } else if (d->pending == 3) {
        d->manifested = true;
    }
    d->pending = 0;
}

static int sim_ctrl(void *io, const uint8_t s[8], uint8_t *data, uint32_t *actual)
{
    sim_dev_t *d = io;
    d->ctrl_calls++;
    uint8_t type = s[0], req = s[1];
    uint16_t value = (uint16_t)(s[2] | (s[3] << 8)), len = (uint16_t)(s[6] | (s[7] << 8));
    *actual = 0;
    if (d->kind == DEV_TEENSY) {
        if (type != 0x21 || req != 0x09 || len != 64 + 1024) return -1;
        uint32_t addr = data[0] | (data[1] << 8) | (data[2] << 16);
        if (addr == 0xFFFFFF) { d->rebooted = true; return 0; }
        if (addr == 0 && d->teensy_busy-- > 0) return -1;   /* erasing */
        memcpy(d->flash + addr, data + 64, 1024);
        d->reports++;
        *actual = len;
        return 0;
    }
    if (type == 0x80 && req == 6) {                         /* GET_DESCRIPTOR */
        uint8_t b[600];
        int n = (value >> 8) == 2 ? config_desc(d, b) : (value >> 8) == 3 ? put_utf16(b, d->layout) : 0;
        if (!n) return -1;
        *actual = (uint32_t)(n < len ? n : len);
        memcpy(data, b, *actual);
        return 0;
    }
    if (type == 0x21 && req == 1) {                         /* DNLOAD */
        if (d->state != 2 && d->state != 5) { d->state = 10; d->status = 0x0F; return -1; }
        if (len == 0) { d->pending = 3; d->state = 6; return 0; }
        if (len > d->xfer) return -1;
        if (value == 0 && d->kind == DEV_DFUSE) { memcpy(d->cmd, data, len); d->cmd_len = len; d->pending = 1; }
        else {
            memcpy(d->data, data, len); d->data_len = len; d->data_block = value; d->pending = 2;
            if (d->fail_status_at_block && value == d->bad_block) { d->status = 0x0A; d->state = 10; return 0; }
        }
        d->state = 3;
        d->busy_left = 2;
        *actual = len;
        return 0;
    }
    if (type == 0xA1 && req == 3) {                         /* GETSTATUS */
        if (len < 6) return -1;
        if (d->state == 3) d->state = 4;
        if (d->state == 4 && --d->busy_left <= 0) { dfu_do_pending(d); if (d->state != 10) d->state = 5; }
        if (d->state == 6) { dfu_do_pending(d); d->state = 8; }
        data[0] = (uint8_t)d->status; data[1] = 5; data[2] = 0; data[3] = 0; data[4] = (uint8_t)d->state; data[5] = 0;
        *actual = 6;
        return 0;
    }
    if (type == 0x21 && req == 4) { d->status = 0; d->state = 2; return 0; }   /* CLRSTATUS */
    if (type == 0x21 && req == 6) { d->state = 2; return 0; }                  /* ABORT */
    return -1;
}

static int sim_src(void *io, uint32_t off, uint8_t *buf, uint32_t n)
{
    sim_dev_t *d = io;
    if (off + n > d->img_len) return -1;
    memcpy(buf, d->img + off, n);
    return 0;
}

static void sim_delay(void *io, uint32_t ms) { (void)io; (void)ms; }

static int run(ctrl_flash_ctx_t *c)
{
    int rc, n = 0;
    while ((rc = ctrl_flash_step(c)) == CTRL_FLASH_RUNNING) if (++n > 200000) return -99;
    return rc;
}

static void ctx(ctrl_flash_ctx_t *c, sim_dev_t *d, ctrl_flash_mode_t mode, uint32_t addr, uint16_t vid, uint16_t pid)
{
    memset(c, 0, sizeof(*c));
    c->ops = (ctrl_flash_ops_t){ d, sim_ctrl, sim_src, sim_delay, vid, pid };
    c->mode = mode;
    c->len = d->img_len;
    c->address = addr;
    c->leave = true;
}

static void test_dfuse(uint32_t dev_xfer, uint32_t offset, uint32_t len, uint8_t alts)
{
    static sim_dev_t d;
    memset(&d, 0, sizeof(d));
    d.kind = DEV_DFUSE; d.flash_base = 0x08000000; d.xfer = dev_xfer; d.state = 2; d.alts = alts;
    d.layout = "@Internal Flash  /0x08000000/04*016Kg,01*064Kg,03*128Kg";
    memset(d.flash, 0x5A, sizeof(d.flash));   /* not erased */
    uint8_t *img = malloc(len);
    for (uint32_t i = 0; i < len; i++) img[i] = (uint8_t)(i * 13 + 1);
    d.img = img; d.img_len = len;
    ctrl_flash_ctx_t *c = malloc(sizeof(*c));
    ctx(c, &d, CTRL_FLASH_DFU, 0x08000000 + offset, 0x0483, 0xDF11);
    int rc = run(c);
    CHECK(rc == CTRL_FLASH_DONE);
    if (rc != CTRL_FLASH_DONE) printf("  dfuse xfer %u: %s (phase %s)\n", dev_xfer, ctrl_flash_rc_str(rc), ctrl_flash_phase_str(c->phase));
    CHECK(c->dfuse && c->nregions == 3);
    CHECK(c->addr_each == (dev_xfer > CTRL_FLASH_BUF));
    CHECK(memcmp(d.flash + offset, img, len) == 0);
    CHECK(d.manifested);
    /* Only the sectors the image covers were erased. */
    uint32_t st;
    int first = sector_index(&d, 0x08000000 + offset, &st), last = sector_index(&d, 0x08000000 + offset + len - 1, &st);
    for (int i = 0; i < 8; i++) CHECK(d.erased[i] == (i >= first && i <= last));
    free(img); free(c);
}

static void test_dfuse_refusals(void)
{
    static sim_dev_t d;
    memset(&d, 0, sizeof(d));
    d.kind = DEV_DFUSE; d.flash_base = 0x08000000; d.xfer = 2048; d.state = 2;
    d.layout = "@Internal Flash  /0x08000000/04*016Kg";   /* 64 KB */
    uint8_t img[4096] = { 0 };
    d.img = img; d.img_len = sizeof(img);
    ctrl_flash_ctx_t *c = malloc(sizeof(*c));
    ctx(c, &d, CTRL_FLASH_DFU, 0x08000000 + 62 * 1024, 0x0483, 0xDF11);   /* runs past the end */
    CHECK(run(c) == CTRL_FLASH_E_LAYOUT);
    d.layout = "@Option Bytes  /0x1FFF7800/01*040 e";                  /* not writable */
    ctx(c, &d, CTRL_FLASH_DFU, 0x1FFF7800, 0x0483, 0xDF11);
    CHECK(run(c) == CTRL_FLASH_E_LAYOUT);
    d.layout = "garbage";
    ctx(c, &d, CTRL_FLASH_DFU, 0x08000000, 0x0483, 0xDF11);
    CHECK(run(c) == CTRL_FLASH_E_LAYOUT);
    /* The loader errors on a block: reported, cleared. */
    d.layout = "@Internal Flash  /0x08000000/04*016Kg";
    d.state = 2; d.fail_status_at_block = true; d.bad_block = 3;
    ctx(c, &d, CTRL_FLASH_DFU, 0x08000000, 0x0483, 0xDF11);
    CHECK(run(c) == CTRL_FLASH_E_STATUS && c->dfu_status == 0x0A && d.state == 2);
    /* A loader left in dfuERROR is cleared first. */
    d.fail_status_at_block = false; d.state = 10; d.status = 0x0A;
    ctx(c, &d, CTRL_FLASH_DFU, 0x08000000, 0x0483, 0xDF11);
    CHECK(run(c) == CTRL_FLASH_DONE);
    free(c);
}

static void test_dfu_plain(void)
{
    static sim_dev_t d;
    memset(&d, 0, sizeof(d));
    d.kind = DEV_DFU; d.xfer = 256; d.state = 2;
    uint8_t img[3000];
    for (int i = 0; i < 3000; i++) img[i] = (uint8_t)(i ^ 0x33);
    d.img = img; d.img_len = sizeof(img);
    ctrl_flash_ctx_t *c = malloc(sizeof(*c));
    ctx(c, &d, CTRL_FLASH_DFU, 0, 0x2341, 0x0369);
    CHECK(run(c) == CTRL_FLASH_DONE);
    CHECK(!c->dfuse && c->xfer == 256 && c->done == (3000 + 255) / 256);
    CHECK(d.plain_off == 3000 && memcmp(d.flash, img, 3000) == 0 && d.manifested);
    free(c);
}

static void test_no_dfu(void)
{
    static sim_dev_t d;
    memset(&d, 0, sizeof(d));
    d.kind = DEV_DFU; d.xfer = 0; d.state = 2;   /* a functional descriptor with transfer size 0 */
    uint8_t img[16] = { 0 };
    d.img = img; d.img_len = sizeof(img);
    ctrl_flash_ctx_t *c = malloc(sizeof(*c));
    ctx(c, &d, CTRL_FLASH_DFU, 0, 1, 2);
    CHECK(run(c) == CTRL_FLASH_E_NO_DFU);
    free(c);
}

static void test_teensy(void)
{
    static sim_dev_t d;
    memset(&d, 0, sizeof(d));
    d.kind = DEV_TEENSY; d.teensy_busy = 3;
    enum { LEN = 10 * 1024 + 100 };
    static uint8_t img[LEN];
    memset(img, 0xFF, sizeof(img));
    for (int i = 0; i < 1024; i++) img[i] = (uint8_t)i;           /* block 0 */
    for (int i = 0; i < 100; i++) img[10 * 1024 + i] = (uint8_t)(i + 1);   /* the last, partial */
    d.img = img; d.img_len = LEN;
    memset(d.flash, 0xEE, sizeof(d.flash));
    ctrl_flash_ctx_t *c = malloc(sizeof(*c));
    ctx(c, &d, CTRL_FLASH_HALFKAY, 0, 0x16C0, 0x0478);
    CHECK(run(c) == CTRL_FLASH_DONE);
    CHECK(d.reports == 2);              /* block 0 and block 10; the 0xFF blocks are skipped */
    CHECK(memcmp(d.flash, img, 1024) == 0 && memcmp(d.flash + 10 * 1024, img + 10 * 1024, 100) == 0);
    CHECK(d.flash[10 * 1024 + 100] == 0xFF);   /* the partial block is padded with 0xFF */
    CHECK(d.rebooted && c->done == 11 && c->total == 11);

    ctx(c, &d, CTRL_FLASH_HALFKAY, 0, 0x2341, 0x0043);    /* not a Teensy */
    CHECK(run(c) == CTRL_FLASH_E_NOT_TEENSY);
    ctx(c, &d, CTRL_FLASH_HALFKAY, 0, 0x16C0, 0x0478);
    c->block = 256;
    CHECK(run(c) == CTRL_FLASH_E_ARGS);
    d.teensy_busy = 1000000;                              /* never answers: a timeout, not a hang */
    ctx(c, &d, CTRL_FLASH_HALFKAY, 0, 0x16C0, 0x0478);
    CHECK(run(c) == CTRL_FLASH_E_TIMEOUT);
    free(c);
}

static void test_hostile_descriptors(void)
{
    /* Random bytes as the config descriptor and string: an answer every time, never a crash. */
    static sim_dev_t d;
    srand(11);
    uint8_t img[64] = { 0 };
    ctrl_flash_ctx_t *c = malloc(sizeof(*c));
    for (int n = 0; n < 2000; n++) {
        memset(&d, 0, sizeof(d));
        d.kind = DEV_DFUSE; d.xfer = (uint32_t)(rand() % 5000); d.state = rand() % 12; d.flash_base = 0x08000000;
        char layout[80];
        for (int i = 0; i < 79; i++) layout[i] = (char)(32 + rand() % 95);
        layout[0] = '@'; layout[79] = 0;
        d.layout = layout;
        d.img = img; d.img_len = sizeof(img);
        ctx(c, &d, CTRL_FLASH_DFU, 0x08000000, 1, 2);
        int rc = run(c);
        CHECK(rc <= 0 && rc != -99);
    }
    free(c);
}

int main(void)
{
    test_dfuse(2048, 0, 100 * 1024 + 7, 0);       /* STM32 ROM: 2048-byte blocks, 7 sectors */
    test_dfuse(1024, 20 * 1024, 30 * 1024, 0);    /* a smaller transfer size, mid-flash */
    test_dfuse(4096, 0, 40 * 1024 + 1, 0);        /* larger than our buffer: address before each block */
    test_dfuse(2048, 0, 30 * 1024, 2);            /* STM32H7 ROM: 2 alternate settings, then the functional descriptor */
    test_dfuse(2048, 0, 30 * 1024, 4);            /* a ROM with 4 memories */
    test_dfuse_refusals();
    test_dfu_plain();
    test_no_dfu();
    test_teensy();
    test_hostile_descriptors();
    CHECK(strcmp(ctrl_flash_rc_str(CTRL_FLASH_E_NOT_TEENSY), "the board is not a Teensy in its loader") == 0);
    printf("test_ctrl_flash: %s\n", failures ? "FAILED" : "all passed");
    return failures ? 1 : 0;
}
