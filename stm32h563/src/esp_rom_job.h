#ifndef ESP_ROM_JOB_H
#define ESP_ROM_JOB_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>

/* ---- ESP32 ROM-loader write job, one step at a time -----------------------
 *
 * The esptool SLIP command set (SYNC, READ_REG, SPI_ATTACH, FLASH_BEGIN, FLASH_DATA,
 * SPI_FLASH_MD5, FLASH_END) as a state machine that never waits: esp_rom_job_step() looks at
 * the bytes that arrived, sends the next command when its answer is in, and returns.  The hw
 * worker calls it once per loop, so a ~2 minute C3 flash no longer holds the worker, and the
 * JSON API and the console keep answering in between (they used to queue behind the flash).
 *
 * Portable (no HAL): the UART, the clock and the image source come in through esp_rom_io_t,
 * which is how test_esp_rom_job.c drives it against a simulated C3 ROM.
 *
 * The state machine (esp_rom_job_begin, esp_rom_job_step) has no static state: the job's own
 * fields are in esp_rom_job_t, and its buffers (the SLIP frames, the running MD5) in the
 * esp_rom_work_t the caller provides (esp_rom_job_prepare).  That is what lets it run from the
 * usb image in the W25Q (xip_image.h), where objects may have no .data or .bss.
 *
 * Used for the pod's own ESP32-C3 (esp_rom_flash.c) and for an ESP32-family DUT on the LA pins
 * (esp_dut_flash.c).  With esp_rom_opts_t it also changes the baud rate (CHANGE_BAUDRATE) and
 * sets the flash size (SPI_SET_PARAMS); without them it runs exactly the C3 sequence.  The chip
 * magic picks the FLASH_BEGIN form: 4 words on the original ESP32, 5 (encrypted flag) on every
 * later chip.  The ESP8266 is refused.
 *
 * The chip must already be in its ROM download mode (straps) with the UART up; the job does not
 * touch EN/BOOT.  It finishes with the chip still in its loader (FLASH_END "stay").
 * -------------------------------------------------------------------------- */

#define ESP_ROM_BLOCK            0x400u   /* ROM-loader FLASH_DATA block size        */
#define ESP_ROM_DATA_ATTEMPTS    3        /* sends of one block before giving up (esptool) */
#define ESP_ROM_MD5_ATTEMPTS     3        /* MD5 requests before the write counts unverified */
#define ESP_ROM_SYNC_ATTEMPTS    10
#define ESP_ROM_SYNC_TIMEOUT_MS  120u
#define ESP_ROM_DATA_TIMEOUT_MS  3000u    /* after the block left the UART             */
#define ESP_ROM_BEGIN_TIMEOUT_MS 40000u   /* the ROM may erase the region up front     */

typedef int (*esp_rom_read_fn)(void *ctx, uint32_t off, uint8_t *buf, uint32_t n);

typedef struct {
    /* Drop whatever is buffered on RX, then start sending `n` bytes (one SLIP frame).  Returns
       how many ms the bytes take on the wire, so the reply timeout starts after them. */
    uint32_t (*send)(const uint8_t *frame, uint32_t n);
    int      (*rx_byte)(void);          /* next received byte, or -1 when none is waiting */
    uint32_t (*now_ms)(void);
    /* The image: `n` bytes at `off`.  NULL = erase (the region is written as 0xFF). */
    esp_rom_read_fn read;
    void           *read_ctx;
    /* Optional: switch our UART to `baud` (after the ROM acknowledged CHANGE_BAUDRATE); 0 when
       it did.  NULL = the job never changes the baud rate. */
    int            (*set_baud)(uint32_t baud);
} esp_rom_io_t;

typedef struct {
    uint32_t baud;        /* > 0 (and io->set_baud): CHANGE_BAUDRATE to it after the chip magic */
    uint32_t flash_size;  /* > 0: SPI_SET_PARAMS with this total flash size in bytes            */
} esp_rom_opts_t;

/* Chip magics (the word at 0x40001000, esptool's CHIP_DETECT_MAGIC_VALUE). */
#define ESP_MAGIC_ESP32    0x00F01D83u
#define ESP_MAGIC_ESP8266  0xFFF0C101u

typedef enum {
    ESP_ROM_IDLE = 0,
    ESP_ROM_SYNC, ESP_ROM_MAGIC, ESP_ROM_BAUD, ESP_ROM_ATTACH, ESP_ROM_PARAMS, ESP_ROM_BEGIN,
    ESP_ROM_DATA, ESP_ROM_MD5, ESP_ROM_END,
    ESP_ROM_DONE,                        /* finished: see result                         */
} esp_rom_phase_t;

/* esp_rom_job_t.result once phase is ESP_ROM_DONE. */
#define ESP_ROM_OK           0   /* written and the C3's MD5 of the region matched          */
#define ESP_ROM_UNVERIFIED   1   /* every block acknowledged, but the MD5 never came back    */
#define ESP_ROM_ERR_SYNC    -1   /* no answer from the ROM loader                            */
#define ESP_ROM_ERR_BEGIN   -2   /* FLASH_BEGIN refused or unanswered                        */
#define ESP_ROM_ERR_READ    -3   /* the image source failed                                  */
#define ESP_ROM_ERR_DATA    -4   /* a block was not acknowledged after ESP_ROM_DATA_ATTEMPTS */
#define ESP_ROM_ERR_MD5     -5   /* the C3's MD5 differs: what it holds is not the image     */
#define ESP_ROM_ERR_ARGS    -6
#define ESP_ROM_ERR_CHIP    -7   /* a chip this job does not write (ESP8266)                 */
#define ESP_ROM_ERR_BAUD    -8   /* the ROM changed its baud rate but our UART could not     */

/* The job's buffers, one per running job (the caller allocates it; the job points at it). */
typedef struct {
    uint8_t  pkt[16 + ESP_ROM_BLOCK];                   /* FLASH_DATA payload, kept for resends  */
    uint8_t  slip[2 * (8 + 16 + ESP_ROM_BLOCK) + 2];    /* the encoded frame, worst case         */
    uint8_t  resp[256];                                 /* the decoded reply                     */
    uint32_t pkt_chk;
    uint32_t value;                                     /* the reply header's value word         */
    const uint8_t *payload;
    int      plen;
    int      pending_result;                            /* what DONE reports after FLASH_END     */
    uint32_t md5[32];                                   /* a mbedtls_md5_context (checked in the .c) */
} esp_rom_work_t;

typedef struct {
    esp_rom_work_t     *w;
    const esp_rom_io_t *io;
    uint32_t len, offset, blocks, seq;
    esp_rom_phase_t phase;
    int      result;
    uint8_t  attempt;                    /* sends of the current command                  */
    uint8_t  status_len;                 /* ROM status bytes per reply (from the SYNC reply) */
    uint32_t chip_magic;
    bool     sync_only;                  /* esp_rom_job_prepare: stop after the chip magic */
    esp_rom_opts_t opts;
    esp_rom_phase_t failed_phase;        /* the phase a failed job stopped in (result < 0) */
    uint32_t baud;                       /* the rate the link runs at once BAUD is done (0 = unchanged) */
    uint32_t resends;                    /* FLASH_DATA blocks sent more than once          */
    uint32_t md5_retries;
    char     want_md5[33], got_md5[33];
    /* The command in flight. */
    uint8_t  op;
    uint32_t deadline;
    /* SLIP receive state. */
    bool     in_frame, esc;
    uint16_t rn;
} esp_rom_job_t;

/* Set a job up: zero it and record what to do.  Write `len` bytes from io->read (or 0xFF with
   io->read NULL) at `offset`; opts NULL = none (the C3 sequence); sync_only: only SYNC and read
   the chip magic (the wiring test, len ignored).  Plain data stores: it runs anywhere. */
static inline void esp_rom_job_prepare(esp_rom_job_t *j, esp_rom_work_t *w, const esp_rom_io_t *io,
                                       uint32_t len, uint32_t offset, const esp_rom_opts_t *opts,
                                       bool sync_only)
{
    memset(j, 0, sizeof(*j));
    j->w = w;
    j->io = io;
    if (opts) j->opts = *opts;
    j->len = len;
    j->offset = offset;
    j->sync_only = sync_only;
}

/* Start a prepared job: sends the first SYNC.  1 while it runs, 0 when it ended at once (bad
   arguments: phase is ESP_ROM_DONE). */
int esp_rom_job_begin(esp_rom_job_t *j);

/* Advance the job; never waits.  1 while it still runs, 0 once phase is ESP_ROM_DONE. */
int esp_rom_job_step(esp_rom_job_t *j);

/* "block 513/1072" style progress: blocks acknowledged so far.  A job that failed reports how far
   it got (0 when it never reached the data), not the total. */
static inline uint32_t esp_rom_job_blocks_done(const esp_rom_job_t *j) {
    if (j->phase == ESP_ROM_DATA) return j->seq;
    if (j->phase == ESP_ROM_DONE && j->result < 0 && j->seq < j->blocks) return j->seq;
    return j->phase > ESP_ROM_DATA ? j->blocks : 0;
}

const char *esp_rom_result_str(int result);

/* "ESP32-S3" style name for a chip magic, or "unknown". */
const char *esp_rom_chip_name(uint32_t magic);

/* The chip-magic phase or later and the phase name ("sync", "data", ...), for status replies. */
const char *esp_rom_phase_str(esp_rom_phase_t phase);

#endif /* ESP_ROM_JOB_H */
