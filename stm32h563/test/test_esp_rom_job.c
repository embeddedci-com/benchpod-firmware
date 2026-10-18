/*
 * test_esp_rom_job.c — host tests for src/esp_rom_job.c, the non-blocking ESP32-C3 ROM-loader
 * write, against a simulated C3 ROM (SLIP in, SLIP out, a flash array, a clock we move):
 *   - a clean write lands byte for byte, padded with 0xFF, and verifies by MD5;
 *   - a lost FLASH_DATA reply is resent (once per lost reply), three in a row fail the write;
 *   - a ROM error status (bad checksum) is seen and resent, with 4 and with 2 status bytes;
 *   - a lost MD5 reply is asked again; one that never comes is "unverified", not failed;
 *   - an MD5 that differs fails the write; FLASH_END follows only a write that did not fail;
 *   - SYNC retries, and no ROM at all fails after ESP_ROM_SYNC_ATTEMPTS;
 *   - erase (no source) writes 0xFF; a source read error stops the write;
 *   - SLIP escapes both ways, noise and stale frames between replies are ignored;
 *   - a step never waits: nothing happens until the simulated reply is due.
 */
#include "esp_rom_job.h"
#include "mbedtls/md5.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); failures++; } \
} while (0)

/* ---- simulated clock + C3 ROM ------------------------------------------------ */

static uint32_t now;
static uint32_t now_ms(void) { return now; }

#define FLASH_SZ (64u * 1024u)
static uint8_t  flash[FLASH_SZ];

typedef struct { uint32_t at; uint8_t b; } rxb_t;
static rxb_t    rxq[4096];
static unsigned rx_head, rx_tail;

static struct {
    int      status_len;        /* 4 = the C3 ROM; 2 = the ESP8266-style variant          */
    int      sync_ignore;       /* SYNCs to leave unanswered                               */
    int      drop_data_seq;     /* FLASH_DATA seq whose replies are dropped ...            */
    int      drop_data_n;       /* ... this many times                                     */
    int      bad_chk_seq;       /* FLASH_DATA seq that first fails its checksum            */
    int      drop_md5_n;        /* MD5 replies to drop                                     */
    bool     corrupt;           /* flip a flash byte before the MD5                        */
    uint32_t reply_ms;          /* ROM processing time per command                         */
    uint32_t magic;
    /* What happened. */
    int      sync_seen, data_seen, md5_seen, end_seen, begin_seen;
    uint32_t begin_off, begin_size;
} rom;

static void rom_reset(void)
{
    memset(&rom, 0, sizeof(rom));
    rom.status_len = 4;
    rom.drop_data_seq = -1;
    rom.bad_chk_seq = -1;
    rom.reply_ms = 2;
    rom.magic = 0x1bC0DB6fu;    /* holds SLIP_END and SLIP_ESC: the reply needs escapes */
    memset(flash, 0x5A, sizeof(flash));
    rx_head = rx_tail = 0;
    now = 1000;
}

static void rx_push(uint32_t at, uint8_t b) { rxq[rx_tail++ % 4096] = (rxb_t){ at, b }; }

static void rx_push_slip(uint32_t at, const uint8_t *p, unsigned n)
{
    rx_push(at, 0xC0);
    for (unsigned i = 0; i < n; i++) {
        if (p[i] == 0xC0)      { rx_push(at, 0xDB); rx_push(at, 0xDC); }
        else if (p[i] == 0xDB) { rx_push(at, 0xDB); rx_push(at, 0xDD); }
        else                   rx_push(at, p[i]);
    }
    rx_push(at, 0xC0);
}

static void reply(uint32_t at, uint8_t op, uint32_t value, const uint8_t *data, unsigned dlen,
                  uint8_t status, uint8_t err)
{
    uint8_t f[128];
    unsigned size = dlen + (unsigned)rom.status_len;
    f[0] = 0x01; f[1] = op; f[2] = (uint8_t)size; f[3] = (uint8_t)(size >> 8);
    f[4] = (uint8_t)value; f[5] = (uint8_t)(value >> 8);
    f[6] = (uint8_t)(value >> 16); f[7] = (uint8_t)(value >> 24);
    if (dlen) memcpy(f + 8, data, dlen);
    memset(f + 8 + dlen, 0, (size_t)rom.status_len);
    f[8 + dlen] = status; f[8 + dlen + 1] = err;
    rx_push_slip(at, f, 8 + size);
}

static uint32_t le32(const uint8_t *p)
{ return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }

static uint8_t cmdbuf[2048];

static uint32_t io_send(const uint8_t *frame, uint32_t n)
{
    while (rx_head != rx_tail && rxq[rx_head % 4096].at <= now) rx_head++;   /* drops what arrived */
    uint32_t wire = n * 10000u / 115200u + 1;
    /* Decode one SLIP frame. */
    unsigned m = 0; bool esc = false;
    CHECK(frame[0] == 0xC0 && frame[n - 1] == 0xC0);
    for (uint32_t i = 1; i + 1 < n; i++) {
        uint8_t b = frame[i];
        CHECK(b != 0xC0);
        if (esc) { cmdbuf[m++] = (b == 0xDC) ? 0xC0 : 0xDB; esc = false; }
        else if (b == 0xDB) esc = true;
        else cmdbuf[m++] = b;
    }
    uint8_t op = cmdbuf[1];
    unsigned dlen = cmdbuf[2] | (cmdbuf[3] << 8);
    CHECK(m == 8 + dlen);
    const uint8_t *d = cmdbuf + 8;
    uint32_t at = now + wire + rom.reply_ms;
    rx_push(now + 1, 0x55);               /* line noise outside any frame */
    switch (op) {
    case 0x08:                             /* SYNC: the ROM answers several times */
        rom.sync_seen++;
        if (rom.sync_ignore > 0) { rom.sync_ignore--; break; }
        for (int k = 0; k < 3; k++) reply(at + (uint32_t)k, op, 0x20120707u, NULL, 0, 0, 0);
        break;
    case 0x0A: reply(at, op, rom.magic, NULL, 0, 0, 0); break;
    case 0x0D: reply(at, op, 0, NULL, 0, 0, 0); break;
    case 0x02:
        rom.begin_seen++;
        rom.begin_size = le32(d); rom.begin_off = le32(d + 12);
        CHECK(dlen == 20);
        reply(at + 50, op, 0, NULL, 0, 0, 0);
        break;
    case 0x03: {
        uint32_t len = le32(d), seq = le32(d + 4);
        CHECK(len == ESP_ROM_BLOCK && dlen == 16 + ESP_ROM_BLOCK);
        rom.data_seen++;
        uint32_t chk = 0xEF;
        for (uint32_t i = 0; i < len; i++) chk ^= d[16 + i];
        CHECK(chk == le32(cmdbuf + 4));
        if ((int)seq == rom.bad_chk_seq) { rom.bad_chk_seq = -1; reply(at, op, 0, NULL, 0, 1, 0xC1); break; }
        uint32_t off = rom.begin_off + seq * ESP_ROM_BLOCK;
        if (off + len <= FLASH_SZ) memcpy(flash + off, d + 16, len);
        if ((int)seq == rom.drop_data_seq && rom.drop_data_n > 0) { rom.drop_data_n--; break; }
        reply(at, op, 0, NULL, 0, 0, 0);
        break;
    }
    case 0x13: {
        rom.md5_seen++;
        uint32_t off = le32(d), len = le32(d + 4);
        if (rom.corrupt) flash[off + 7] ^= 0x01;
        rom.corrupt = false;
        uint8_t md5[16]; char hex[33];
        mbedtls_md5(flash + off, len, md5);
        for (int i = 0; i < 16; i++) snprintf(hex + i * 2, 3, "%02x", md5[i]);
        if (rom.drop_md5_n > 0) { rom.drop_md5_n--; break; }
        reply(at + 30, op, 0, (const uint8_t *)hex, 32, 0, 0);
        break;
    }
    case 0x04: rom.end_seen++; reply(at, op, 0, NULL, 0, 0, 0); break;
    default: CHECK(0 && "unexpected op");
    }
    return wire;
}

static int io_rx_byte(void)
{
    if (rx_head == rx_tail || rxq[rx_head % 4096].at > now) return -1;
    return rxq[rx_head++ % 4096].b;
}

/* ---- image source ---------------------------------------------------------------- */

static uint8_t image[5000];
static int     read_fail_at = -1;
static int     reads;

static int img_read(void *ctx, uint32_t off, uint8_t *buf, uint32_t n)
{
    (void)ctx;
    reads++;
    if (read_fail_at >= 0 && off >= (uint32_t)read_fail_at) return -1;
    memcpy(buf, image + off, n);
    return 0;
}

static esp_rom_io_t io = { io_send, io_rx_byte, now_ms, img_read, NULL };

/* Run the job to the end, 1 ms per step (the worker loop). */
static int run(esp_rom_job_t *j, uint32_t len, uint32_t off)
{
    esp_rom_job_start(j, &io, len, off);
    for (int guard = 0; guard < 2000000 && esp_rom_job_step(j); guard++) now++;
    CHECK(j->phase == ESP_ROM_DONE);
    return j->result;
}

static void fill_image(void)
{
    for (unsigned i = 0; i < sizeof(image); i++) image[i] = (uint8_t)(i * 7u + (i >> 8));
    image[10] = 0xC0; image[11] = 0xDB; image[12] = 0xC0;   /* escapes in the data */
    read_fail_at = -1; reads = 0;
}

/* ---- tests ------------------------------------------------------------------------- */

static void test_clean_write(void)
{
    rom_reset(); fill_image();
    esp_rom_job_t j;
    CHECK(run(&j, sizeof(image), 0x1000) == ESP_ROM_OK);
    CHECK(memcmp(flash + 0x1000, image, sizeof(image)) == 0);
    for (uint32_t i = sizeof(image); i < 5 * ESP_ROM_BLOCK; i++) CHECK(flash[0x1000 + i] == 0xFF);
    CHECK(j.blocks == 5 && rom.data_seen == 5 && j.resends == 0);
    CHECK(reads == 5);                     /* each block read once */
    CHECK(j.chip_magic == rom.magic);      /* escaped value decoded */
    CHECK(j.status_len == 4);
    CHECK(rom.begin_off == 0x1000 && rom.begin_size == sizeof(image));
    CHECK(rom.end_seen == 1 && rom.md5_seen == 1);
    CHECK(strcmp(j.want_md5, j.got_md5) == 0);
    CHECK(esp_rom_job_blocks_done(&j) == 5);
}

static void test_lost_data_reply(void)
{
    rom_reset(); fill_image();
    rom.drop_data_seq = 2; rom.drop_data_n = 1;
    esp_rom_job_t j;
    CHECK(run(&j, sizeof(image), 0) == ESP_ROM_OK);
    CHECK(j.resends == 1 && rom.data_seen == 6);
    CHECK(reads == 5);                     /* the resend reuses the block, not a re-read */
    CHECK(memcmp(flash, image, sizeof(image)) == 0);

    rom_reset(); fill_image();
    rom.drop_data_seq = 3; rom.drop_data_n = 3;
    CHECK(run(&j, sizeof(image), 0) == ESP_ROM_ERR_DATA);
    CHECK(rom.data_seen == 3 + ESP_ROM_DATA_ATTEMPTS && rom.end_seen == 0 && rom.md5_seen == 0);
}

static void test_rom_error_status(void)
{
    for (int sl = 2; sl <= 4; sl += 2) {
        rom_reset(); fill_image();
        rom.status_len = sl;
        rom.bad_chk_seq = 1;
        esp_rom_job_t j;
        CHECK(run(&j, sizeof(image), 0) == ESP_ROM_OK);
        CHECK(j.status_len == sl);
        CHECK(j.resends == 1 && rom.data_seen == 6);
    }
}

static void test_md5_retry_and_unverified(void)
{
    rom_reset(); fill_image();
    rom.drop_md5_n = 2;
    esp_rom_job_t j;
    CHECK(run(&j, sizeof(image), 0) == ESP_ROM_OK);
    CHECK(j.md5_retries == 2 && rom.md5_seen == 3 && rom.end_seen == 1);

    rom_reset(); fill_image();
    rom.drop_md5_n = ESP_ROM_MD5_ATTEMPTS;
    CHECK(run(&j, sizeof(image), 0) == ESP_ROM_UNVERIFIED);
    CHECK(rom.md5_seen == ESP_ROM_MD5_ATTEMPTS && rom.end_seen == 1);
}

static void test_md5_mismatch(void)
{
    rom_reset(); fill_image();
    rom.corrupt = true;
    esp_rom_job_t j;
    CHECK(run(&j, sizeof(image), 0) == ESP_ROM_ERR_MD5);
    CHECK(rom.end_seen == 0);
}

static void test_sync(void)
{
    rom_reset(); fill_image();
    rom.sync_ignore = 2;
    esp_rom_job_t j;
    CHECK(run(&j, sizeof(image), 0) == ESP_ROM_OK);
    CHECK(rom.sync_seen == 3);

    rom_reset(); fill_image();
    rom.sync_ignore = 1000;
    uint32_t t0 = now;
    CHECK(run(&j, sizeof(image), 0) == ESP_ROM_ERR_SYNC);
    CHECK(rom.sync_seen == ESP_ROM_SYNC_ATTEMPTS && rom.begin_seen == 0);
    CHECK(now - t0 < ESP_ROM_SYNC_ATTEMPTS * (ESP_ROM_SYNC_TIMEOUT_MS + 10));
}

static void test_erase_and_read_fail(void)
{
    rom_reset(); fill_image();
    esp_rom_job_t j;
    esp_rom_io_t erase = io; erase.read = NULL;
    esp_rom_job_start(&j, &erase, 0x4000, 0x9000);
    while (esp_rom_job_step(&j)) now++;
    CHECK(j.result == ESP_ROM_OK);
    for (uint32_t i = 0; i < 0x4000; i++) CHECK(flash[0x9000 + i] == 0xFF);
    CHECK(flash[0x8FFF] == 0x5A && flash[0xD000] == 0x5A);   /* nothing outside the region */
    CHECK(reads == 0);

    rom_reset(); fill_image();
    read_fail_at = 2 * ESP_ROM_BLOCK;
    CHECK(run(&j, sizeof(image), 0) == ESP_ROM_ERR_READ);
    CHECK(rom.md5_seen == 0 && rom.end_seen == 0);
}

static void test_step_never_waits(void)
{
    rom_reset(); fill_image();
    rom.reply_ms = 500;                    /* a slow ROM: nothing may move until it answers */
    esp_rom_job_t j;
    esp_rom_job_start(&j, &io, sizeof(image), 0);
    int syncs = rom.sync_seen;
    for (int i = 0; i < 100; i++) CHECK(esp_rom_job_step(&j));   /* same instant: no reply yet */
    CHECK(j.phase == ESP_ROM_SYNC && rom.sync_seen == syncs);
    while (esp_rom_job_step(&j)) now++;
    /* 500 ms per reply is past the SYNC timeout; the late answer to an earlier SYNC still
       counts, and the slower commands' timeouts cover the rest. */
    CHECK(j.result == ESP_ROM_OK);
    CHECK(memcmp(flash, image, sizeof(image)) == 0);

    rom_reset(); fill_image();
    CHECK(esp_rom_job_step(&j) == false);  /* a finished job stays finished */
    esp_rom_job_t idle; memset(&idle, 0, sizeof(idle));
    CHECK(esp_rom_job_step(&idle) == false);
}

static void test_sync_only(void)
{
    rom_reset(); fill_image();
    esp_rom_job_t j;
    esp_rom_job_start_sync(&j, &io);
    while (esp_rom_job_step(&j)) now++;
    CHECK(j.result == ESP_ROM_OK && j.chip_magic == rom.magic);
    CHECK(rom.begin_seen == 0 && rom.data_seen == 0);

    rom_reset();
    rom.sync_ignore = 1000;
    esp_rom_job_start_sync(&j, &io);
    while (esp_rom_job_step(&j)) now++;
    CHECK(j.result == ESP_ROM_ERR_SYNC);
}

static void test_args(void)
{
    rom_reset(); fill_image();
    esp_rom_job_t j;
    esp_rom_job_start(&j, &io, 0, 0);
    CHECK(j.phase == ESP_ROM_DONE && j.result == ESP_ROM_ERR_ARGS);
    CHECK(rom.sync_seen == 0);
}

int main(void)
{
    test_clean_write();
    test_lost_data_reply();
    test_rom_error_status();
    test_md5_retry_and_unverified();
    test_md5_mismatch();
    test_sync();
    test_erase_and_read_fail();
    test_step_never_waits();
    test_sync_only();
    test_args();
    if (failures) { printf("test_esp_rom_job: %d FAILED\n", failures); return 1; }
    printf("test_esp_rom_job: all passed\n");
    return 0;
}
