/*
 * fw_info.h — a small block every firmware image carries at a fixed offset, so whoever is about
 * to install an image (OTA here, `make flash-dfu`, `benchpod flash-self`, the server) can tell
 * whether it fits this pod before writing a byte.
 *
 * The block sits at FW_INFO_OFFSET from the start of the image, right after the vector table
 * (config/STM32H563ZITX_FLASH.ld). Images from before it existed have no block there; they were
 * all built for the 2 MB part.
 */
#ifndef FW_INFO_H
#define FW_INFO_H

#include <stdint.h>
#include <stddef.h>

#define FW_INFO_OFFSET   0x400u
#define FW_INFO_MAGIC    0x57465042u   /* "BPFW" little-endian */
#define FW_INFO_LAYOUT   2u            /* bumped when the flash layout changes (2: blobs in the W25Q) */

/* The smallest flash this build fits: the linker keeps the code below the 1 MB part's
   persistence sectors (config/STM32H563ZITX_FLASH.ld). */
#define FW_INFO_MIN_FLASH_KB  1024u

typedef struct {
    uint32_t magic;          /* FW_INFO_MAGIC */
    uint16_t layout;         /* FW_INFO_LAYOUT */
    uint16_t min_flash_kb;   /* smallest internal flash the image fits, in KB */
    uint32_t reserved[2];    /* 0 */
} fw_info_t;

/* The flash in KB an image needs, from the `n` bytes of the image found at FW_INFO_OFFSET:
   its min_flash_kb, or 2048 for an image without a block (every image from before it). */
uint32_t fw_info_required_kb(const uint8_t *at_offset, size_t n);

#endif /* FW_INFO_H */
