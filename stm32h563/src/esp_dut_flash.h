#ifndef ESP_DUT_FLASH_H
#define ESP_DUT_FLASH_H

#include <stdint.h>
#include <stdbool.h>

/* ---- Flash an ESP32-family DUT over its UART ROM bootloader ---------------------------------
 *
 * The DUT's UART0 is on two LA pins (the FPGA soft UART, like uart_proxy_start), its boot strap
 * (IO0, or IO9 on the C3/C6) on an LA pin driven open-drain, and EN on the pod's reset pin (nrst,
 * open-drain) or on an LA pin, also open-drain.  The image is the first `len` bytes a `load_bin`
 * with "psram" staged; esp_rom_job.c writes it 1 KB at a time, read from PSRAM block by block.
 *
 * The flash runs in the background on the hw worker (esp_dut_flash_poll), so the worker keeps
 * answering commands; esp_dut_flash_busy() makes the PSRAM-bus users (captures, uploads, updates)
 * answer busy meanwhile, as during the pod's own C3 flash.
 *
 * Sequence: boot low, EN low 100 ms, EN released, 50 ms, SYNC at 115200 (the job retries); the
 * boot pin is released once the ROM answered.  Then CHANGE_BAUDRATE, SPI_ATTACH, SPI_SET_PARAMS,
 * FLASH_BEGIN (erase), FLASH_DATA, MD5, FLASH_END.  With `run`, EN is pulsed at the end (boot
 * released) and the DUT starts the new image; without, it stays in its loader.  Every pin is
 * released at the end, whatever the result.
 * ------------------------------------------------------------------------------------------ */

#define ESP_DUT_EN_NRST 0   /* en = the pod's reset pin */

typedef struct {
    uint8_t  rx, tx;        /* LA channels: pod RX (DUT TXD0), pod TX (DUT RXD0) */
    uint8_t  boot;          /* LA channel of IO0 / IO9 */
    uint8_t  en;            /* LA channel of EN, or ESP_DUT_EN_NRST */
    uint32_t offset;        /* flash address */
    uint32_t len;           /* bytes of the staged upload to write (0 = sync only) */
    uint32_t baud;          /* rate after CHANGE_BAUDRATE; 0 or 115200 = no change */
    uint32_t flash_size;    /* SPI_SET_PARAMS total size, 0 = do not send */
    bool     run;           /* reset into the new image at the end */
} esp_dut_params_t;

/* Start: claims the pins (err holds the reason on failure), checks the staged upload, puts the
   DUT into its loader and starts the job.  0 = started, < 0 = not started (err set): -1 a refusal
   (BP0084-style: pins, staging, busy), -2 the usb image in the W25Q cannot run (BP0083: install
   the blobs) or is busy.  The ROM-loader code runs from that image, in an XIP session the job holds
   from start to its end (esp_rom_run.h), so the pod's own C3 flash and usb_flash exclude it. */
int  esp_dut_flash_start(const esp_dut_params_t *p, char *err, uint32_t err_cap);

/* Advance the background flash; cheap when none runs.  hw worker loop only. */
void esp_dut_flash_poll(void);

/* Stop a running flash: pins released, DUT left in its loader. */
void esp_dut_flash_abort(void);

bool esp_dut_flash_busy(void);

typedef struct {
    bool        running;
    bool        have_result;     /* a flash ran since boot */
    bool        sync_only;
    const char *phase;           /* "enter", "sync", "write", ... "done" */
    uint32_t    done, total;     /* blocks */
    uint32_t    magic;
    const char *chip;            /* "ESP32-S3", "unknown" */
    int         result;          /* ESP_ROM_* (esp_rom_job.h) once !running */
    const char *result_str;
    char        md5[33];         /* the chip's MD5 of the region, "" when not read */
    uint32_t    resends, baud, ms, offset, len;
} esp_dut_status_t;

void esp_dut_flash_status(esp_dut_status_t *st);

#endif /* ESP_DUT_FLASH_H */
