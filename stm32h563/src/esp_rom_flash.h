#ifndef ESP_ROM_FLASH_H
#define ESP_ROM_FLASH_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* ---- ESP32-C3 ROM serial-bootloader flasher (STM32 host) -------------------
 *
 * Programs the on-board ESP32-C3 (U4) by driving its ROM download bootloader
 * over USART1 (PA9/PA10) while strapping BOOT (IO9/PF12) low and pulsing EN
 * (PF11).  There is NO programming header on the vbench-pod board — the C3's
 * UART and straps reach only the STM32 — so this is the sole non-soldering way
 * to flash it.  Mirrors flash-ice40: the image lives in a W25Q blob slot and
 * the pod flashes it itself (no host esptool needed).
 *
 * The protocol (esptool's SLIP command set, ROM-loader variant, no stub) lives in
 * esp_rom_job.c as a state machine that never waits.  USART1 runs on interrupts into a ring
 * buffer: a polled receiver lost a reply byte whenever a higher-priority task ran for more than
 * two byte times, which cost a 3 s FLASH_DATA timeout 17-26 times per flash.
 *
 * Hardware-verified (C3 auto-flash, 2026-09-21).  esp_rom_flash_sync() proves
 * the EN/BOOT/UART wiring without touching flash.
 *
 * These take over EN/BOOT/USART1; call only when the SPI Wi-Fi transport is
 * stopped (esp_hosted_spi_stop()).  On success the C3 is left running the new
 * firmware; on failure it is left held in reset (EN low, powered down).
 * All of it runs on the hw worker.
 * -------------------------------------------------------------------------- */

/* Strap the C3 into its ROM download bootloader and establish SLIP comms
 * (SYNC).  Optionally reads and returns the chip-detect magic (may be NULL).
 * Proves EN/BOOT/UART wiring without touching flash.  Returns 0 on success. */
int esp_rom_flash_sync(uint32_t *chip_magic_out);

/* Called on the hw worker when a background flash ends: result is ESP_ROM_OK (0),
   ESP_ROM_UNVERIFIED (1, every block acknowledged but no MD5 came back; the C3 was booted
   anyway) or a negative ESP_ROM_ERR_* (esp_rom_job.h; the C3 is held in reset). */
typedef void (*esp_flash_done_fn)(int result);

/* Start programming the C3 with the esp-hosted image in its W25Q slot (blob_store.h), after
 * checking the slot against its SHA-256.  Returns at once: esp_rom_flash_poll() (the hw worker
 * loop) carries the ~2 minute write and calls `done` at the end.  Meanwhile the worker keeps
 * serving commands, and esp_rom_flash_busy() makes the PSRAM-bus users (captures, uploads,
 * updates, W25Q writes) answer busy: each block is read from the W25Q on its own.
 * 0 = started; ESP_FLASH_START_* otherwise (`done` is not called then). */
#define ESP_FLASH_START_BUSY     -1   /* a flash is already running */
#define ESP_FLASH_START_NO_IMAGE -2   /* slot esp is empty          */
#define ESP_FLASH_START_BAD_SLOT -3   /* slot esp fails its SHA-256 */
int esp_rom_flash_start(esp_flash_done_fn done);

/* Advance a background flash; cheap when none runs.  hw worker loop only. */
void esp_rom_flash_poll(void);

/* A background flash owns the C3 (EN/BOOT/USART1) and reads the W25Q block by block. */
bool esp_rom_flash_busy(void);

/* Progress of the running flash: blocks acknowledged and the total.  false when none runs. */
bool esp_rom_flash_progress(uint32_t *done, uint32_t *total);

/* Erase the C3's NVS partition (found through the partition table of the esp-hosted image in
 * the W25Q slot) and verify by on-chip MD5 that it reads back blank.  That is where esp-hosted
 * firmware left on the IDF default WIFI_STORAGE_FLASH (pod firmware up to 3.7.0) kept the SSID
 * and password; NVS itself only marks an entry erased, so nothing short of erasing the sectors
 * removes them.  The application, otadata and phy_init are untouched: the C3 recreates an
 * empty NVS (and redoes its RF calibration) on its next boot.  Returns 0 on success, <0 on
 * error (no image in the slot, no NVS partition in its table, no ROM answer, no blank
 * verify, or a background flash running).  The C3 is left held in reset either way.  Takes
 * ~2 s and blocks the caller; hw worker only, with the PSRAM bus free (bus_busy_reason) and
 * the hardware up (not in safe mode). */
int esp_rom_flash_erase_nvs(void);

/* Bring-up diagnostic: passively monitor the C3's UART0 log (its console output
 * on U0TXD -> PA10) for `ms` milliseconds, printing each line to our console.
 * Does NOT touch EN/BOOT — the C3 keeps running the app. USART1 only. */
void esp_uart_monitor(uint32_t ms);


#endif /* ESP_ROM_FLASH_H */
