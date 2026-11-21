/*
 * esp_rom_job.c — the ESP32 ROM-loader write as a non-blocking state machine (see
 * esp_rom_job.h).  Portable: everything hardware comes in through esp_rom_io_t.
 */
#include "esp_rom_job.h"
#include "bp_log.h"
#include "mbedtls/md5.h"

#include <string.h>
#include <stdio.h>

/* ---- ROM protocol constants ---------------------------------------------- */
#define OP_FLASH_BEGIN     0x02
#define OP_FLASH_DATA      0x03
#define OP_FLASH_END       0x04
#define OP_SYNC            0x08
#define OP_READ_REG        0x0A
#define OP_SPI_SET_PARAMS  0x0B
#define OP_SPI_ATTACH      0x0D
#define OP_CHANGE_BAUDRATE 0x0F
#define OP_SPI_FLASH_MD5   0x13

#define CHECKSUM_MAGIC     0xEF
#define CHIP_DETECT_MAGIC_REG 0x40001000u

#define SLIP_END           0xC0
#define SLIP_ESC           0xDB
#define SLIP_ESC_END       0xDC
#define SLIP_ESC_ESC       0xDD

#define REPLY_PENDING      1

/* Static I/O scratch: one job at a time. */
static uint8_t  s_pkt[16 + ESP_ROM_BLOCK];                 /* FLASH_DATA payload, kept for resends */
static uint32_t s_pkt_chk;
static uint8_t  s_slip[2 * (8 + 16 + ESP_ROM_BLOCK) + 2];  /* worst-case encoded                  */
static uint8_t  s_resp[256];                               /* decoded reply                       */
static uint32_t s_value;                                   /* the reply header's value word       */
static const uint8_t *s_payload;
static int      s_plen;
static mbedtls_md5_context s_md5;
static int      s_pending_result;                          /* what DONE reports after FLASH_END   */

static void put_le32(uint8_t *p, uint32_t v)
{ p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24); }

static bool reached(uint32_t now, uint32_t deadline) { return (int32_t)(now - deadline) >= 0; }

/* ---- command out ------------------------------------------------------------ */

static void send_cmd(esp_rom_job_t *j, uint8_t op, const uint8_t *data, uint16_t dlen,
                     uint32_t checksum, uint32_t timeout_ms)
{
    /* SLIP-encode the 8-byte header and the data straight into s_slip (no unencoded copy). */
    uint8_t h[8];
    h[0] = 0x00; h[1] = op;
    h[2] = (uint8_t)dlen; h[3] = (uint8_t)(dlen >> 8);
    put_le32(h + 4, checksum);

    uint8_t *w = s_slip;
    *w++ = SLIP_END;
    for (uint32_t i = 0; i < 8u + dlen; i++) {
        uint8_t b = (i < 8u) ? h[i] : data[i - 8u];
        if (b == SLIP_END)      { *w++ = SLIP_ESC; *w++ = SLIP_ESC_END; }
        else if (b == SLIP_ESC) { *w++ = SLIP_ESC; *w++ = SLIP_ESC_ESC; }
        else                    *w++ = b;
    }
    *w++ = SLIP_END;

    j->op = op;
    j->in_frame = false; j->esc = false; j->rn = 0;
    uint32_t wire_ms = j->io->send(s_slip, (uint32_t)(w - s_slip));
    j->deadline = j->io->now_ms() + wire_ms + timeout_ms;
}

/* ---- reply in ---------------------------------------------------------------- */

/* One complete frame in s_resp[0..rn): REPLY_PENDING if it is not the answer to the command in
   flight (a runt, a late reply to an earlier command, the extra SYNC replies), else 0 or the
   ROM's error as -(100 + code). */
static int eval_frame(esp_rom_job_t *j)
{
    uint32_t n = j->rn;
    if (n < 8 + 2 || s_resp[0] != 0x01 || s_resp[1] != j->op) return REPLY_PENDING;
    uint16_t size = (uint16_t)(s_resp[2] | (s_resp[3] << 8));
    if (8u + size > n) return REPLY_PENDING;
    if (j->status_len == 0)   /* the SYNC reply carries only the status: 4 bytes on the C3 ROM */
        j->status_len = (size == 4) ? 4 : 2;
    if (size < j->status_len) return REPLY_PENDING;
    const uint8_t *body = s_resp + 8;
    uint8_t status = body[size - j->status_len];
    if (status != 0) return -(100 + (int)body[size - j->status_len + 1]);
    s_value   = (uint32_t)s_resp[4] | ((uint32_t)s_resp[5] << 8) |
                ((uint32_t)s_resp[6] << 16) | ((uint32_t)s_resp[7] << 24);
    s_payload = body;
    s_plen    = (int)size - j->status_len;
    return 0;
}

/* Drain what arrived: 0 / -(100+code) once the answer is in, -1 past the deadline, else
   REPLY_PENDING. */
static int poll_reply(esp_rom_job_t *j)
{
    int c;
    while ((c = j->io->rx_byte()) >= 0) {
        if (c == SLIP_END) {
            if (j->in_frame && j->rn > 0) {
                int r = eval_frame(j);
                j->rn = 0; j->esc = false;
                if (r != REPLY_PENDING) return r;
            }
            j->in_frame = true;           /* an END also opens the next frame */
            continue;
        }
        if (!j->in_frame) continue;       /* noise before the first END */
        if (j->esc) {
            c = (c == SLIP_ESC_END) ? SLIP_END : (c == SLIP_ESC_ESC) ? SLIP_ESC : c;
            j->esc = false;
        } else if (c == SLIP_ESC) {
            j->esc = true;
            continue;
        }
        if (j->rn < sizeof(s_resp)) s_resp[j->rn++] = (uint8_t)c;
    }
    return reached(j->io->now_ms(), j->deadline) ? -1 : REPLY_PENDING;
}

/* ---- steps --------------------------------------------------------------------- */

static void finish(esp_rom_job_t *j, int result)
{
    if (result < 0) j->failed_phase = j->phase;
    j->result = result;
    j->phase  = ESP_ROM_DONE;
    mbedtls_md5_free(&s_md5);
}

static void send_sync(esp_rom_job_t *j)
{
    uint8_t d[36];
    d[0] = 0x07; d[1] = 0x07; d[2] = 0x12; d[3] = 0x20;
    memset(d + 4, 0x55, 32);
    send_cmd(j, OP_SYNC, d, sizeof(d), 0, ESP_ROM_SYNC_TIMEOUT_MS);
}

static void send_pkt(esp_rom_job_t *j)
{
    send_cmd(j, OP_FLASH_DATA, s_pkt, (uint16_t)sizeof(s_pkt), s_pkt_chk, ESP_ROM_DATA_TIMEOUT_MS);
}

/* Read block j->seq once (it is hashed once), pad it and send it. false = the source failed. */
static bool send_block(esp_rom_job_t *j)
{
    uint32_t off = j->seq * ESP_ROM_BLOCK;
    uint32_t n = (j->len - off < ESP_ROM_BLOCK) ? j->len - off : ESP_ROM_BLOCK;
    put_le32(s_pkt + 0, ESP_ROM_BLOCK);   /* data length (padded) */
    put_le32(s_pkt + 4, j->seq);
    put_le32(s_pkt + 8, 0);
    put_le32(s_pkt + 12, 0);
    if (!j->io->read) {
        memset(s_pkt + 16, 0xFF, n);
    } else if (j->io->read(j->io->read_ctx, off, s_pkt + 16, n) != 0) {
        LOG_E("[espflash] reading the image at %lu failed\n", (unsigned long)off);
        return false;
    }
    mbedtls_md5_update(&s_md5, s_pkt + 16, n);
    if (n < ESP_ROM_BLOCK) memset(s_pkt + 16 + n, 0xFF, ESP_ROM_BLOCK - n);
    uint32_t chk = CHECKSUM_MAGIC;        /* covers only the block bytes (esptool convention) */
    for (uint32_t i = 0; i < ESP_ROM_BLOCK; i++) chk ^= s_pkt[16 + i];
    s_pkt_chk = chk;
    j->attempt = 1;
    send_pkt(j);
    return true;
}

static void send_md5(esp_rom_job_t *j)
{
    uint8_t m[16];
    put_le32(m + 0, j->offset); put_le32(m + 4, j->len);
    put_le32(m + 8, 0);         put_le32(m + 12, 0);
    /* esptool allows 8 s per MB for the ROM's MD5; twice that, plus the reply. */
    uint32_t timeout = 5000u + (j->len / 1024u) * 16u;
    send_cmd(j, OP_SPI_FLASH_MD5, m, sizeof(m), 0, timeout);
}

static void send_end(esp_rom_job_t *j, int result)
{
    uint8_t e[4]; put_le32(e, 1);         /* stay in the loader: the caller resets the C3 */
    s_pending_result = result;
    j->phase = ESP_ROM_END;
    send_cmd(j, OP_FLASH_END, e, sizeof(e), 0, 1000);
}

static void send_attach(esp_rom_job_t *j)
{
    /* SPI_ATTACH(0): default flash pins.  Non-fatal: the flash is usually attached already;
       FLASH_BEGIN/DATA and the MD5 are the real gates. */
    uint8_t a[8] = {0};
    j->phase = ESP_ROM_ATTACH;
    send_cmd(j, OP_SPI_ATTACH, a, sizeof(a), 0, 1000);
}

static void send_begin(esp_rom_job_t *j)
{
    /* The original ESP32's ROM takes 4 words.  Every later ROM sets
       SPI_FLASH_SUPPORTS_ENCRYPTED_FLASH, so esptool appends a 5th (encrypted=0). */
    uint8_t b[20];
    uint16_t n = (j->chip_magic == ESP_MAGIC_ESP32) ? 16 : 20;
    put_le32(b + 0, j->len);          /* erase size  */
    put_le32(b + 4, j->blocks);       /* packets     */
    put_le32(b + 8, ESP_ROM_BLOCK);   /* packet size */
    put_le32(b + 12, j->offset);
    put_le32(b + 16, 0);              /* not encrypted */
    log_printf("[espflash] FLASH_BEGIN (%lu blocks, erasing)...\n", (unsigned long)j->blocks);
    j->phase = ESP_ROM_BEGIN;
    send_cmd(j, OP_FLASH_BEGIN, b, n, 0, ESP_ROM_BEGIN_TIMEOUT_MS);
}

void esp_rom_job_start(esp_rom_job_t *j, const esp_rom_io_t *io, uint32_t len, uint32_t offset)
{
    esp_rom_job_start_ex(j, io, len, offset, NULL);
}

void esp_rom_job_start_ex(esp_rom_job_t *j, const esp_rom_io_t *io, uint32_t len, uint32_t offset,
                          const esp_rom_opts_t *opts)
{
    memset(j, 0, sizeof(*j));
    j->io = io;
    if (opts) j->opts = *opts;
    j->len = len;
    j->offset = offset;
    j->blocks = (len + ESP_ROM_BLOCK - 1) / ESP_ROM_BLOCK;
    mbedtls_md5_init(&s_md5);
    mbedtls_md5_starts(&s_md5);
    if (len == 0 || !io || !io->send || !io->rx_byte || !io->now_ms) {
        finish(j, ESP_ROM_ERR_ARGS);
        return;
    }
    log_printf("[espflash] %s %lu bytes at 0x%06lx\n", io->read ? "programming" : "erasing",
               (unsigned long)len, (unsigned long)offset);
    j->phase = ESP_ROM_SYNC;
    j->attempt = 1;
    send_sync(j);
}

void esp_rom_job_start_sync(esp_rom_job_t *j, const esp_rom_io_t *io)
{
    memset(j, 0, sizeof(*j));
    j->io = io;
    j->sync_only = true;
    mbedtls_md5_init(&s_md5);
    if (!io || !io->send || !io->rx_byte || !io->now_ms) { finish(j, ESP_ROM_ERR_ARGS); return; }
    j->phase = ESP_ROM_SYNC;
    j->attempt = 1;
    send_sync(j);
}

bool esp_rom_job_step(esp_rom_job_t *j)
{
    if (j->phase == ESP_ROM_IDLE || j->phase == ESP_ROM_DONE) return false;
    int r = poll_reply(j);
    if (r == REPLY_PENDING) return true;

    switch (j->phase) {
    case ESP_ROM_SYNC:
        if (r == 0) {
            uint8_t d[4]; put_le32(d, CHIP_DETECT_MAGIC_REG);
            j->phase = ESP_ROM_MAGIC;
            send_cmd(j, OP_READ_REG, d, sizeof(d), 0, 200);
        } else if (j->attempt < ESP_ROM_SYNC_ATTEMPTS) {
            j->attempt++;
            send_sync(j);
        } else {
            LOG_E("[espflash] SYNC failed\n");
            finish(j, ESP_ROM_ERR_SYNC);
        }
        break;

    case ESP_ROM_MAGIC: {
        if (r == 0) j->chip_magic = s_value;
        log_printf("[espflash] %s ROM synced (magic 0x%08lx)\n", esp_rom_chip_name(j->chip_magic),
                   (unsigned long)j->chip_magic);
        if (j->sync_only) { finish(j, ESP_ROM_OK); break; }
        if (r == 0 && j->chip_magic == ESP_MAGIC_ESP8266) {
            log_printf("[espflash] ESP8266: not supported\n");
            finish(j, ESP_ROM_ERR_CHIP);
            break;
        }
        if (j->opts.baud && j->io->set_baud) {
            /* CHANGE_BAUDRATE(new, 0): the second word is the old rate only for esptool's stub. */
            uint8_t d[8]; put_le32(d, j->opts.baud); put_le32(d + 4, 0);
            j->phase = ESP_ROM_BAUD;
            send_cmd(j, OP_CHANGE_BAUDRATE, d, sizeof(d), 0, 1000);
            break;
        }
        send_attach(j);
        break;
    }

    case ESP_ROM_BAUD:
        if (r == 0) {
            /* The answer came at the old rate; the ROM switches after sending it.  If our UART
               cannot follow, nothing more gets through: stop here, not 40 s later. */
            if (j->io->set_baud(j->opts.baud) != 0) {
                LOG_W("[espflash] could not switch our UART to %lu baud\n",
                      (unsigned long)j->opts.baud);
                finish(j, ESP_ROM_ERR_BAUD);
                break;
            }
            j->baud = j->opts.baud;
            log_printf("[espflash] baud rate now %lu\n", (unsigned long)j->opts.baud);
        } else {
            LOG_W("[espflash] CHANGE_BAUDRATE refused (rc %d): staying at the old rate\n", r);
        }
        send_attach(j);
        break;

    case ESP_ROM_ATTACH:
        if (r != 0) LOG_W("[espflash] SPI_ATTACH warning (continuing)\n");
        if (j->opts.flash_size) {
            /* SPI_SET_PARAMS(id, total size, 64 KB block, 4 KB sector, 256 B page, status mask):
               esptool sends it in ROM mode too, else the ROM refuses writes past its default. */
            uint8_t p[24];
            put_le32(p + 0, 0);
            put_le32(p + 4, j->opts.flash_size);
            put_le32(p + 8, 64u * 1024u);
            put_le32(p + 12, 4096u);
            put_le32(p + 16, 256u);
            put_le32(p + 20, 0xFFFFu);
            j->phase = ESP_ROM_PARAMS;
            send_cmd(j, OP_SPI_SET_PARAMS, p, sizeof(p), 0, 1000);
            break;
        }
        send_begin(j);
        break;

    case ESP_ROM_PARAMS:
        if (r != 0) LOG_W("[espflash] SPI_SET_PARAMS warning (rc %d, continuing)\n", r);
        send_begin(j);
        break;

    case ESP_ROM_BEGIN:
        if (r != 0) {
            LOG_E("[espflash] FLASH_BEGIN failed (rc %d)\n", r);
            finish(j, ESP_ROM_ERR_BEGIN);
            break;
        }
        j->phase = ESP_ROM_DATA;
        j->seq = 0;
        if (!send_block(j)) finish(j, ESP_ROM_ERR_READ);
        break;

    case ESP_ROM_DATA:
        if (r != 0) {
            LOG_W("[espflash] FLASH_DATA block %lu/%lu attempt %d failed (rc %d)\n",
                  (unsigned long)(j->seq + 1), (unsigned long)j->blocks, j->attempt, r);
            if (j->attempt >= ESP_ROM_DATA_ATTEMPTS) { finish(j, ESP_ROM_ERR_DATA); break; }
            if (j->attempt == 1) j->resends++;
            j->attempt++;
            send_pkt(j);                  /* the ROM accepts the same seq again (esptool) */
            break;
        }
        if ((j->seq & 0x3F) == 0 || j->seq == j->blocks - 1)
            log_printf("[espflash]  block %lu/%lu\n", (unsigned long)(j->seq + 1),
                       (unsigned long)j->blocks);
        if (++j->seq < j->blocks) {
            if (!send_block(j)) finish(j, ESP_ROM_ERR_READ);
            break;
        }
        {
            uint8_t want[16];
            mbedtls_md5_finish(&s_md5, want);
            for (int i = 0; i < 16; i++) snprintf(j->want_md5 + i * 2, 3, "%02x", want[i]);
        }
        j->phase = ESP_ROM_MD5;
        j->attempt = 1;
        send_md5(j);
        break;

    case ESP_ROM_MD5:
        if (r == 0 && s_plen >= 32) {
            memcpy(j->got_md5, s_payload, 32);
            j->got_md5[32] = '\0';
            if (memcmp(j->got_md5, j->want_md5, 32) != 0) {
                LOG_E("[espflash] MD5 MISMATCH: the chip does not hold the image\n");
                log_printf("[espflash]   want %s\n", j->want_md5);
                log_printf("[espflash]   got  %s\n", j->got_md5);
                finish(j, ESP_ROM_ERR_MD5);
                break;
            }
            log_printf("[espflash] MD5 verified (%s)\n", j->want_md5);
            send_end(j, ESP_ROM_OK);
            break;
        }
        LOG_W("[espflash] MD5 read attempt %d failed (rc %d)\n", j->attempt, r == 0 ? -2 : r);
        if (j->attempt < ESP_ROM_MD5_ATTEMPTS) {
            j->attempt++;
            j->md5_retries++;
            send_md5(j);
            break;
        }
        /* Every block was acknowledged, and the ROM checks each one's checksum before it writes
           it: the image is most likely there.  Report it unverified rather than failed and let
           the caller boot it; a C3 that does not come up is caught by its boot event. */
        log_printf("[espflash] MD5 never came back: every block was acknowledged, write unverified\n");
        send_end(j, ESP_ROM_UNVERIFIED);
        break;

    case ESP_ROM_END:
        finish(j, s_pending_result);      /* FLASH_END's own answer does not matter */
        break;

    default:
        finish(j, ESP_ROM_ERR_ARGS);
        break;
    }
    return j->phase != ESP_ROM_DONE;
}

const char *esp_rom_result_str(int result)
{
    switch (result) {
    case ESP_ROM_OK:         return "verified";
    case ESP_ROM_UNVERIFIED: return "written, MD5 not read back";
    case ESP_ROM_ERR_SYNC:   return "no answer from the ROM loader";
    case ESP_ROM_ERR_BEGIN:  return "FLASH_BEGIN failed";
    case ESP_ROM_ERR_READ:   return "reading the image failed";
    case ESP_ROM_ERR_DATA:   return "a block was not acknowledged";
    case ESP_ROM_ERR_MD5:    return "MD5 mismatch";
    case ESP_ROM_ERR_CHIP:   return "chip not supported (ESP8266)";
    case ESP_ROM_ERR_BAUD:   return "could not change the baud rate";
    default:                 return "bad arguments";
    }
}

const char *esp_rom_chip_name(uint32_t magic)
{
    switch (magic) {
    case ESP_MAGIC_ESP32:   return "ESP32";
    case 0x000007C6u:       return "ESP32-S2";
    case 0x00000009u:       return "ESP32-S3";
    case 0x6921506Fu: case 0x1B31506Fu: case 0x4881606Fu: case 0x4361606Fu:
                            return "ESP32-C3";
    case 0x6F51306Fu: case 0x7C41A06Fu:
                            return "ESP32-C2";
    case 0x2CE0806Fu:       return "ESP32-C6";
    case 0xD7B73E80u:       return "ESP32-H2";
    case ESP_MAGIC_ESP8266: return "ESP8266";
    default:                return "unknown";
    }
}

const char *esp_rom_phase_str(esp_rom_phase_t phase)
{
    switch (phase) {
    case ESP_ROM_IDLE:   return "idle";
    case ESP_ROM_SYNC:   return "sync";
    case ESP_ROM_MAGIC:  return "detect";
    case ESP_ROM_BAUD:   return "baud";
    case ESP_ROM_ATTACH: return "attach";
    case ESP_ROM_PARAMS: return "params";
    case ESP_ROM_BEGIN:  return "erase";
    case ESP_ROM_DATA:   return "write";
    case ESP_ROM_MD5:    return "verify";
    case ESP_ROM_END:    return "end";
    default:             return "done";
    }
}
