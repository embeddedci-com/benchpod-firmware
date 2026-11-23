/* Host test for the PICOBOOT mode of ctrl_flash (xip_mod_ctrl_flash.c) against a simulated RP2040:
   4 KB sectors that must be erased before they are written, 256-byte pages, the command block and
   its acknowledge, a status code for a bad command, and a flash that reads one page back wrong. */
#include "ctrl_flash.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); failures++; } } while (0)

#define FLASH_BASE 0x10000000u
#define FLASH_SIZE (256u * 1024u)
#define EP_OUT 0x03
#define EP_IN  0x84

typedef struct {
    uint8_t  flash[FLASH_SIZE];
    uint8_t  erased[FLASH_SIZE / 4096];
    bool     exclusive, xip_exited, rebooted, if_reset_seen, no_vendor_itf;
    uint32_t reboot_kind, writes, erases, reads;
    uint32_t bad_read_addr;       /* a read of this page returns a flipped byte (0 = none) */
    uint32_t fail_cmd_id;         /* this command id fails with status code 4 (invalid address) */
    uint8_t  status;
    /* the command in flight */
    uint8_t  cmd[32];
    bool     have_cmd, data_done;
    uint8_t  ack_dir;             /* 0: next transfer is the data/ack in the normal order */
    const uint8_t *img; uint32_t img_len;
} sim_t;

static uint32_t rd32(const uint8_t *p) { return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24); }

static int sim_ctrl(void *io, const uint8_t s[8], uint8_t *data, uint32_t *actual)
{
    sim_t *d = io;
    *actual = 0;
    uint16_t value = (uint16_t)(s[2] | (s[3] << 8)), len = (uint16_t)(s[6] | (s[7] << 8));
    if (s[0] == 0x80 && s[1] == 6 && (value >> 8) == 2) {
        uint8_t b[64];
        unsigned n = 0;
        const uint8_t cfg[] = { 9, 2, 0, 0, 2, 1, 0, 0x80, 250 };
        const uint8_t msc[] = { 9, 4, 0, 0, 2, 8, 6, 0x50, 0 };
        const uint8_t e1[] = { 7, 5, 0x81, 2, 64, 0, 0 }, e2[] = { 7, 5, 0x02, 2, 64, 0, 0 };
        memcpy(b + n, cfg, 9); n += 9;
        memcpy(b + n, msc, 9); n += 9;
        memcpy(b + n, e1, 7); n += 7;
        memcpy(b + n, e2, 7); n += 7;
        if (!d->no_vendor_itf) {
            const uint8_t v[] = { 9, 4, 1, 0, 2, 0xFF, 0, 0, 0 };
            const uint8_t o[] = { 7, 5, EP_OUT, 2, 64, 0, 0 }, i[] = { 7, 5, EP_IN, 2, 64, 0, 0 };
            memcpy(b + n, v, 9); n += 9;
            memcpy(b + n, o, 7); n += 7;
            memcpy(b + n, i, 7); n += 7;
        }
        b[2] = (uint8_t)n;
        *actual = n < len ? n : len;
        memcpy(data, b, *actual);
        return 0;
    }
    if (s[0] == 0x41 && s[1] == 0x41) { d->if_reset_seen = true; d->have_cmd = false; return 0; }
    if (s[0] == 0xC1 && s[1] == 0x42) {   /* the command status */
        memset(data, 0, 16);
        data[4] = d->status;
        *actual = 16;
        return 0;
    }
    return -1;
}

static void run_cmd(sim_t *d)
{
    const uint8_t id = d->cmd[8];
    const uint8_t *a = d->cmd + 16;
    if (d->fail_cmd_id && id == d->fail_cmd_id) { d->status = 4; return; }
    switch (id) {
    case 0x01: d->exclusive = a[0] != 0; break;
    case 0x06: d->xip_exited = true; break;
    case 0x03: {
        uint32_t addr = rd32(a), size = rd32(a + 4);
        if (addr < FLASH_BASE || (addr & 4095) || size != 4096 || addr - FLASH_BASE + size > FLASH_SIZE) { d->status = 5; return; }
        memset(d->flash + (addr - FLASH_BASE), 0xFF, size);
        d->erased[(addr - FLASH_BASE) / 4096] = 1;
        d->erases++;
        break;
    }
    case 0x02: d->rebooted = true; d->reboot_kind = 1; break;
    case 0x0A: d->rebooted = true; d->reboot_kind = 2; break;
    default: break;
    }
}

static int sim_bulk(void *io, uint8_t ep, uint8_t *data, uint32_t len, uint32_t *actual)
{
    sim_t *d = io;
    *actual = 0;
    if (!d->have_cmd) {
        if (ep != EP_OUT || len != 32 || rd32(data) != 0x431FD10Bu) return -1;
        memcpy(d->cmd, data, 32);
        d->have_cmd = true;
        d->data_done = rd32(d->cmd + 12) == 0;
        *actual = 32;
        if (d->data_done && !(d->cmd[8] & 0x80)) run_cmd(d);
        return 0;
    }
    const uint8_t id = d->cmd[8];
    const bool in = (id & 0x80) != 0;
    const uint32_t dlen = rd32(d->cmd + 12), addr = rd32(d->cmd + 16), size = rd32(d->cmd + 20);
    if (!d->data_done) {                       /* the data stage */
        if (dlen != size || len != dlen) return -1;
        if (in) {
            if (ep != EP_IN || id != 0x84 || addr < FLASH_BASE || addr - FLASH_BASE + size > FLASH_SIZE) { d->status = 4; return -1; }
            memcpy(data, d->flash + (addr - FLASH_BASE), size);
            if (d->bad_read_addr == addr) data[7] ^= 0x40;
            d->reads++;
        } else {
            if (ep != EP_OUT || id != 0x05 || (addr & 255) || size != 256) { d->status = 5; return -1; }
            for (uint32_t i = 0; i < size; i++) {
                uint32_t o = addr - FLASH_BASE + i;
                if (o >= FLASH_SIZE || !d->erased[o / 4096]) { d->status = 15; return -1; }   /* not erased */
                d->flash[o] &= data[i];
            }
            d->writes++;
        }
        *actual = len;
        d->data_done = true;
        return 0;
    }
    /* the acknowledge, in the other direction: a zero-length packet */
    if (d->fail_cmd_id && id == d->fail_cmd_id) { d->have_cmd = false; return -1; }
    if (in ? (ep != EP_OUT || len != 0) : (ep != EP_IN || len == 0)) return -1;
    d->have_cmd = false;
    *actual = 0;
    return 0;
}

static int sim_src(void *io, uint32_t off, uint8_t *buf, uint32_t n)
{
    sim_t *d = io;
    if (off + n > d->img_len) return -1;
    memcpy(buf, d->img + off, n);
    return 0;
}

static void sim_delay(void *io, uint32_t ms) { (void)io; (void)ms; }

static int run(ctrl_flash_ctx_t *c)
{
    int rc, n = 0;
    while ((rc = ctrl_flash_step(c)) == CTRL_FLASH_RUNNING) if (++n > 100000) return -99;
    return rc;
}

static void ctx(ctrl_flash_ctx_t *c, sim_t *d, uint32_t addr, uint16_t vid, uint16_t pid)
{
    memset(c, 0, sizeof(*c));
    c->ops = (ctrl_flash_ops_t){ d, sim_ctrl, sim_src, sim_delay, vid, pid, sim_bulk };
    c->mode = CTRL_FLASH_PICOBOOT;
    c->len = d->img_len;
    c->address = addr;
    c->leave = true;
}

static sim_t *new_sim(uint32_t len, uint8_t **img)
{
    sim_t *d = calloc(1, sizeof(*d));
    memset(d->flash, 0x5A, sizeof(d->flash));   /* not erased */
    *img = malloc(len);
    for (uint32_t i = 0; i < len; i++) (*img)[i] = (uint8_t)(i * 7 + 3);
    d->img = *img; d->img_len = len;
    return d;
}

static void test_flash(uint16_t pid, uint32_t len, uint32_t addr_off)
{
    uint8_t *img;
    sim_t *d = new_sim(len, &img);
    ctrl_flash_ctx_t *c = malloc(sizeof(*c));
    ctx(c, d, addr_off ? FLASH_BASE + addr_off : 0, 0x2E8A, pid);
    int rc = run(c);
    CHECK(rc == CTRL_FLASH_DONE);
    if (rc != CTRL_FLASH_DONE) printf("  pid %04x: %s (phase %s)\n", pid, ctrl_flash_rc_str(rc), ctrl_flash_phase_str(c->phase));
    CHECK(memcmp(d->flash + addr_off, img, len) == 0);
    /* the rest of the last page is padded with 0xFF; the sectors outside the image are untouched */
    uint32_t pages = (len + 255) / 256;
    CHECK(d->flash[addr_off + len] == (len % 256 ? 0xFF : 0x5A));
    CHECK(d->flash[addr_off + pages * 256 + 8192] == 0x5A);
    CHECK(d->exclusive && d->xip_exited && d->rebooted && d->reboot_kind == (pid == 0x000F ? 2u : 1u));
    CHECK(d->erases == (len + 4095) / 4096 && d->writes == pages && d->reads == pages);
    CHECK(c->done == c->total && c->total == d->erases + 2 * pages);
    free(c); free(d); free(img);
}

static void test_verify_error(void)
{
    uint8_t *img;
    sim_t *d = new_sim(5000, &img);
    d->bad_read_addr = FLASH_BASE + 256 * 7;
    ctrl_flash_ctx_t *c = malloc(sizeof(*c));
    ctx(c, d, 0, 0x2E8A, 0x0003);
    CHECK(run(c) == CTRL_FLASH_E_VERIFY);
    CHECK(c->fail_addr == FLASH_BASE + 256 * 7 && !d->rebooted && d->if_reset_seen);
    free(c); free(d); free(img);
}

static void test_refusals(void)
{
    uint8_t *img;
    sim_t *d = new_sim(4096, &img);
    ctrl_flash_ctx_t *c = malloc(sizeof(*c));
    ctx(c, d, 0, 0x0483, 0xDF11);                    /* an STM32 DFU loader */
    CHECK(run(c) == CTRL_FLASH_E_NOT_RP2 && d->erases == 0);
    ctx(c, d, 0, 0x2E8A, 0x000A);                    /* an RP2040 running MicroPython */
    CHECK(run(c) == CTRL_FLASH_E_NOT_RP2);
    ctx(c, d, FLASH_BASE + 256, 0x2E8A, 0x0003);     /* not sector aligned */
    CHECK(run(c) == CTRL_FLASH_E_ARGS);
    ctx(c, d, 0x08000000, 0x2E8A, 0x0003);           /* below flash */
    CHECK(run(c) == CTRL_FLASH_E_ARGS);
    ctx(c, d, FLASH_BASE + 0x00FFF000u, 0x2E8A, 0x0003);   /* runs past 16 MB */
    c->len = 8192;
    CHECK(run(c) == CTRL_FLASH_E_ARGS);
    d->no_vendor_itf = true;
    ctx(c, d, 0, 0x2E8A, 0x0003);
    CHECK(run(c) == CTRL_FLASH_E_NO_PICOBOOT);
    d->no_vendor_itf = false;
    /* The loader refuses a command: its status code is reported and the interface reset. */
    d->fail_cmd_id = 0x03;
    ctx(c, d, 0, 0x2E8A, 0x0003);
    CHECK(run(c) == CTRL_FLASH_E_STATUS && c->dfu_status == 4 && d->if_reset_seen);
    /* leave:false stays in BOOTSEL with the drive writable again. */
    d->fail_cmd_id = 0; d->status = 0; d->rebooted = false;
    ctx(c, d, 0, 0x2E8A, 0x0003);
    c->leave = false;
    CHECK(run(c) == CTRL_FLASH_DONE && !d->rebooted && !d->exclusive);
    /* A staged file that cannot be read. */
    d->img_len = 100;
    ctx(c, d, 0, 0x2E8A, 0x0003);
    c->len = 4096;
    CHECK(run(c) == CTRL_FLASH_E_SRC);
    free(c); free(d); free(img);
}

int main(void)
{
    test_flash(0x0003, 70000, 0);              /* RP2040: 18 sectors, a partial last page */
    test_flash(0x000F, 4096, 0);               /* RP2350: exactly one sector */
    test_flash(0x0003, 1, 0);                  /* one byte */
    test_flash(0x0003, 9000, 0x10000);         /* at an address inside flash */
    test_verify_error();
    test_refusals();
    CHECK(strcmp(ctrl_flash_phase_str(CTRL_PH_VERIFY), "verify") == 0);
    CHECK(strcmp(ctrl_flash_rc_str(CTRL_FLASH_E_NOT_RP2), "the board is not an RP2040 or RP2350 in BOOTSEL") == 0);
    printf("test_picoboot: %s\n", failures ? "FAILED" : "all passed");
    return failures ? 1 : 0;
}
