/*
 * flash_compat.c — STM32H5 internal-flash backend for the Pico-style
 * flash_range_erase / flash_range_program API used by config_store.c and
 * device_identity.c.
 *
 * STM32H5: 2 MB flash, 2 banks x 1 MB, 8 KB sectors (128/bank); programmed in
 * 16-byte quad-words.  `offset` is relative to FLASH_BASE (0x08000000).  The
 * persistence records live in the last two sectors (well clear of the ~120 KB
 * firmware), so erasing them never touches code.
 */
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "stm32h5xx_hal.h"
#include <string.h>

#define FLASH_BANK_SIZE   0x00100000u   /* 1 MB per bank */
#define H5_SECTOR_SIZE    0x2000u        /* 8 KB */

uint32_t save_and_disable_interrupts(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}
void restore_interrupts(uint32_t status) { __set_PRIMASK(status); }

static void addr_to_bank_sector(uint32_t abs, uint32_t *bank, uint32_t *sector)
{
    if (abs < (FLASH_BASE + FLASH_BANK_SIZE)) {
        *bank = FLASH_BANK_1;
        *sector = (abs - FLASH_BASE) / H5_SECTOR_SIZE;
    } else {
        *bank = FLASH_BANK_2;
        *sector = (abs - (FLASH_BASE + FLASH_BANK_SIZE)) / H5_SECTOR_SIZE;
    }
}

void flash_range_erase(uint32_t offset, size_t count)
{
    uint32_t abs = FLASH_BASE + offset;
    uint32_t nsectors = (uint32_t)((count + H5_SECTOR_SIZE - 1) / H5_SECTOR_SIZE);
    uint32_t bank, sector, err = 0;
    addr_to_bank_sector(abs, &bank, &sector);

    FLASH_EraseInitTypeDef e = {0};
    e.TypeErase = FLASH_TYPEERASE_SECTORS;
    e.Banks = bank;
    e.Sector = sector;
    e.NbSectors = nsectors;

    HAL_FLASH_Unlock();
    HAL_FLASHEx_Erase(&e, &err);
    HAL_FLASH_Lock();
}

void flash_range_program(uint32_t offset, const uint8_t *data, size_t count)
{
    uint32_t abs = FLASH_BASE + offset;
    HAL_FLASH_Unlock();
    for (size_t i = 0; i < count; i += 16) {
        /* Quad-word program reads 16 bytes from a word-aligned source. */
        __attribute__((aligned(16))) uint8_t qw[16];
        size_t n = (count - i < 16) ? (count - i) : 16;
        memset(qw, 0xFF, sizeof(qw));
        memcpy(qw, data + i, n);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, abs + i, (uint32_t)(uintptr_t)qw);
    }
    HAL_FLASH_Lock();
}
