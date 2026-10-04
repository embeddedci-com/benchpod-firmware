/* Pico SDK <hardware/flash.h> shim -> STM32H5 internal-flash backend
   (see src/flash_compat.c).  offset is relative to XIP_BASE (= FLASH_BASE) and
   written for the 2 MB part: persistence offsets are mapped to the top of the
   actual flash (src/flash_layout.h).
   STM32H5 erases in 8 KB sectors; programs in 16-byte quad-words (handled
   internally — callers may still pass 256-byte pages). */
#ifndef HW_FLASH_SHIM_H
#define HW_FLASH_SHIM_H
#include <stdint.h>
#include <stddef.h>

#define FLASH_PAGE_SIZE     256u
#define FLASH_SHIM_SECTOR_SIZE 8192u   /* STM32H5 sector size (not FLASH_SECTOR_SIZE: CMSIS defines that) */

void flash_range_erase(uint32_t offset, size_t count);
void flash_range_program(uint32_t offset, const uint8_t *data, size_t count);

#endif
