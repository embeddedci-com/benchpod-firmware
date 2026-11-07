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

/* The same, reading the bitstream through `rd` (n <= 256 bytes at offset `off`), so the source
   can be a blob slot in this same flash: it is read page by page with the bus already held. */
typedef int (*ice40_src_read_fn)(void *ctx, uint32_t off, uint8_t *buf, uint32_t n);
int ice40_flash_program_src(ice40_src_read_fn rd, void *ctx, size_t len);

/* Reload the iCE40 from its config flash as it is (reset + configure, no erase, no write).
   0 ok (CDONE high), <0 fail. The caller re-inits the PSRAM bus and the register mirrors
   (ice40_reload does both). */
int ice40_flash_boot(void);
/* 1 when the config flash from offset 0 equals `len` bytes of the source, 0 when not, <0 when
   it could not be read. The iCE40 reconfigures from the flash afterwards. */
int ice40_flash_matches_src(ice40_src_read_fn rd, void *ctx, size_t len);
/* ice40_flash_boot plus what every reconfiguration needs around it (PSRAM to SPI and back to
   QPI, the bus back to the iCE40, the firmware's register mirrors). Never writes the flash.
   Defined in console.c. 0 ok, -1 fail. */
int  ice40_reload(void);
/* The image the iCE40 boots from (offset 0) equals one of the gateware slots: 1 yes, 0 no,
   <0 unknown (no slot, or a read failed). Defined in console.c. */
int  ice40_boot_image_intact(void);

/* CDONE right now: 1 = the iCE40 loaded a bitstream, 0 = it never configured (e.g. a new
   board whose config flash is blank).  Configures PF14 as an input first. */
int  ice40_is_configured(void);
/* Hold the iCE40 in reset (all its pins Hi-Z) with the config flash deselected: for an iCE40 that
   did not configure, which otherwise keeps driving the shared bus hunting for a bitstream.  The
   next ice40_flash_program releases it. */
void ice40_hold_off_bus(void);

/* Runtime gateware IMAGE SWITCH: reprogram the config flash with image n (0=closed-loop,
   1=deep-DAC-replay) and reconfigure, from W25Q blob slot gw0/gw1.  Defined in console.c.
   ~2 s.  Returns 0 ok, -1 fail. */
int  ice40_reflash_image(int n);
/* The GATEWARE_VERSION of the images this firmware embeds (0 = unknown at build time). */
uint8_t ice40_embedded_gw_version(void);

#endif /* ICE40_FLASH_H */
