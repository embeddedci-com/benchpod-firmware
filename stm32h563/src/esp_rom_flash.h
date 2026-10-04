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
 * The protocol is a from-scratch implementation of the esptool SLIP command set
 * (SYNC / SPI_ATTACH / FLASH_BEGIN / FLASH_DATA / FLASH_END / SPI_FLASH_MD5),
 * ROM loader variant (no stub upload).  All progress is logged via printf to the
 * console.
 *
 * Hardware-verified (C3 auto-flash, 2026-09-21).  esp_rom_flash_sync() proves
 * the EN/BOOT/UART wiring without touching flash.
 *
 * These take over EN/BOOT/USART1; call only when the SPI Wi-Fi transport is
 * stopped (esp_hosted_spi_stop()).  On success the C3 is left running the new
 * firmware; on failure it is left held in reset (EN low, powered down).
 * -------------------------------------------------------------------------- */

/* Strap the C3 into its ROM download bootloader and establish SLIP comms
 * (SYNC).  Optionally reads and returns the chip-detect magic (may be NULL).
 * Proves EN/BOOT/UART wiring without touching flash.  Returns 0 on success. */
int esp_rom_flash_sync(uint32_t *chip_magic_out);

/* Full program: enter download mode, SYNC, erase+write `len` bytes read through `rd`
 * (1 KB blocks, each read once) at `offset` (use 0 for a merge_bin image), verify by
 * on-chip MD5, then reset the C3 into the newly-flashed application.  Returns 0 on
 * success, <0 on error (C3 left in reset on error).  Takes ~2 min for a ~1 MB image at
 * 115200 baud. */
typedef int (*esp_src_read_fn)(void *ctx, uint32_t off, uint8_t *buf, uint32_t n);
int esp_rom_flash_program_src(esp_src_read_fn rd, void *ctx, size_t len, uint32_t offset);

/* Program the C3 with the esp-hosted image in its W25Q slot (blob_store.h), after checking
 * the slot against its SHA-256.  Runs on the hw worker.  0 = flashed and booted. */
int esp_rom_flash_from_slot(void);

/* Hold the C3 in reset / powered down (EN low). */
void esp_rom_flash_power_off(void);

/* Bring-up diagnostic: passively monitor the C3's UART0 log (its console output
 * on U0TXD -> PA10) for `ms` milliseconds, printing each line to our console.
 * Does NOT touch EN/BOOT — the C3 keeps running the app. USART1 only. */
void esp_uart_monitor(uint32_t ms);


#endif /* ESP_ROM_FLASH_H */
