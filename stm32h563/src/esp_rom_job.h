#ifndef ESP_ROM_JOB_H
#define ESP_ROM_JOB_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* ---- ESP32-C3 ROM-loader write job, one step at a time --------------------
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
 * The C3 must already be in its ROM download mode (straps) with the UART up; the job does not
 * touch EN/BOOT.  It finishes with the C3 still in its loader (FLASH_END "stay").
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
} esp_rom_io_t;

typedef enum {
    ESP_ROM_IDLE = 0,
    ESP_ROM_SYNC, ESP_ROM_MAGIC, ESP_ROM_ATTACH, ESP_ROM_BEGIN, ESP_ROM_DATA, ESP_ROM_MD5,
    ESP_ROM_END,
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

typedef struct {
    const esp_rom_io_t *io;
    uint32_t len, offset, blocks, seq;
    esp_rom_phase_t phase;
    int      result;
    uint8_t  attempt;                    /* sends of the current command                  */
    uint8_t  status_len;                 /* ROM status bytes per reply (from the SYNC reply) */
    uint32_t chip_magic;
    bool     sync_only;                  /* esp_rom_job_start_sync: stop after the chip magic */
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

/* Begin writing `len` bytes from io->read (or 0xFF with io->read NULL) at `offset`: sends the
   first SYNC.  The job owns static buffers, so one job runs at a time. */
void esp_rom_job_start(esp_rom_job_t *j, const esp_rom_io_t *io, uint32_t len, uint32_t offset);

/* Only SYNC and read the chip magic (the wiring test): ESP_ROM_OK or ESP_ROM_ERR_SYNC. */
void esp_rom_job_start_sync(esp_rom_job_t *j, const esp_rom_io_t *io);

/* Advance the job; never waits.  true while it still runs, false once phase is ESP_ROM_DONE. */
bool esp_rom_job_step(esp_rom_job_t *j);

/* "block 513/1072" style progress: blocks acknowledged so far. */
static inline uint32_t esp_rom_job_blocks_done(const esp_rom_job_t *j) {
    return j->phase == ESP_ROM_DATA ? j->seq : (j->phase > ESP_ROM_DATA ? j->blocks : 0);
}

const char *esp_rom_result_str(int result);

#endif /* ESP_ROM_JOB_H */
