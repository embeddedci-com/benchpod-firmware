#ifndef ICE40_FLASH_H
#define ICE40_FLASH_H

#include <stdint.h>
#include <stddef.h>

/* Reprogram the iCE40 config flash (Winbond W25Q64) over the shared OCTOSPI bus.
 * The flash shares the quad data/clock lines with the PSRAM; its chip-select is
 * PE3 (a GPIO).  Requires psram_init() to have configured OCTOSPI1.  The iCE40
 * is held in reset (CRESET=PF13) while the STM32 owns the bus (PG0), then
 * released to reconfigure (CDONE=PF14). */

/* Read the JEDEC ID (3 bytes; W25Q64 = EF 40 17).  Returns 0 on success. */
int ice40_flash_read_id(uint8_t id[3]);

/* Erase + program the whole bitstream from offset 0, verify, release the iCE40,
   and wait for CDONE.  Returns 0 on success, <0 on error. */
int ice40_flash_program(const uint8_t *data, size_t len);

/* iCE40 self-reconfiguration (SB_WARMBOOT) support: hand it the config-flash bus, then
   wait for CDONE after it reboots into the selected image. */
void ice40_prepare_reconfig(void);
int  ice40_wait_cdone(uint32_t timeout_ms);

/* Runtime gateware IMAGE SWITCH: reprogram the config flash with image n (0=closed-loop,
   1=deep-DAC-replay) and reconfigure.  Defined in console.c (holds the embedded
   images).  ~2 s.  Returns 0 ok, -1 fail. */
int  ice40_reflash_image(int n);

#endif /* ICE40_FLASH_H */
