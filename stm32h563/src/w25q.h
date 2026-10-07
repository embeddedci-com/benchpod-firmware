/*
 * w25q.h — the iCE40 config flash (Winbond W25Q64, or the pin-compatible W25Q128) as seen from
 * the STM32: single-SPI commands over the shared OCTOSPI bus, chip-select on PE3 (a GPIO).
 *
 * The iCE40 boots its gateware from offset 0. The rest of the chip holds the blob slots
 * (blob_store.h). Two ways in:
 *   - ice40_flash.c holds the iCE40 in reset (CRESET) and rewrites offset 0;
 *   - w25q_open()/w25q_close() reach the flash while the iCE40 keeps running: the gateware
 *     tristates the shared bus while the STM32 owns it (PG0), and never selects the flash after
 *     configuration. The caller must have quiesced the gateware's PSRAM masters first
 *     (signal_engine_quiesce_psram_masters), as for any other bus grab.
 *
 * Every call below except open/close assumes the STM32 owns the bus.
 */
#ifndef W25Q_H
#define W25Q_H

#include <stdint.h>
#include <stddef.h>

#define W25Q_SECTOR   4096u
#define W25Q_BLOCK    65536u
#define W25Q_PAGE     256u

/* Take the bus, drive the flash chip-select, wake the flash. Leaves CRESET alone. 0 = ok. */
int  w25q_open(void);
/* Deselect the flash, put its chip-select back to Hi-Z and release the bus. */
void w25q_close(void);

/* A W25Q session for code that runs while the gateware may be live (OTA blob installs, the
   company CA and proxy, the ESP32-C3 reflash): quiesce the gateware's PSRAM masters, then
   w25q_open. Every runtime W25Q writer goes through this pair, so none can grab the bus in the
   middle of a DAC replay burst. The caller must also make sure no capture or upload owns the bus
   (heavy_or_claimed): quiescing stops a DAC, it does not wait for a capture to be read back. */
int  w25q_session_open(void);
void w25q_session_close(void);

/* Chip-select pin as a driven output, high (deselected). Used by w25q_open and ice40_flash. */
void w25q_cs_output(void);
/* Release deep power-down (0xAB). */
void w25q_wake(void);

/* JEDEC ID (Winbond: EF 40 17 = 8 MB, EF 40 18 = 16 MB). 0 = read ok. */
int  w25q_read_id(uint8_t id[3]);
/* Capacity in bytes from a JEDEC ID, 0 if it is not a W25Q this board can carry. */
uint32_t w25q_capacity(const uint8_t id[3]);

int  w25q_read(uint32_t addr, uint8_t *buf, uint32_t len);
int  w25q_erase_sector(uint32_t addr);   /* 4 KB, addr aligned */
int  w25q_erase_block(uint32_t addr);    /* 64 KB, addr aligned */
/* Program `len` bytes at `addr`, split at page boundaries. The range must be erased. */
int  w25q_program(uint32_t addr, const uint8_t *buf, uint32_t len);

#endif /* W25Q_H */
