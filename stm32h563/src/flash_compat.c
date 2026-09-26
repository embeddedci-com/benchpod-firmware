/*
 * flash_compat.c — STM32H5 internal-flash backend for the Pico-style
 * flash_range_erase / flash_range_program API used by config_store.c and
 * device_identity.c.
 *
 * STM32H5: 2 MB flash, 2 banks x 1 MB, 8 KB sectors (128/bank); programmed in
 * 16-byte quad-words.  `offset` is relative to FLASH_BASE (0x08000000).  The
 * persistence records live in bank-2 sectors 121..127 (0x081F2000..0x081FFFFF),
 * above the linker's FLASH_BLOBS end (0x081F0000) and the OTA self-test scratch
 * sector (0x081F0000), so erasing them never touches code.
 */
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "flash_compat.h"
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

/* ---- ECC-safe reads (see flash_compat.h) ------------------------------------
 *
 * While s_probe is set, an ECC double-error NMI is ours: flash_ecc_nmi_absorb()
 * clears ECCD and records the hit, and the NMI returns into the copy loop, which
 * stops. The probe runs with FAULTMASK set and CCR.BFHFNMIGN on, so should the H5
 * ALSO report the torn read as a precise bus fault (it does for some flash areas,
 * e.g. a byte read of the UID page), that fault is ignored and seen in CFSR
 * instead of escalating. NMI is not masked by FAULTMASK, so the absorb still runs.
 * Nothing else can execute during the probe, so any ECCD raised is our read's. */

static volatile uint32_t s_probe;
static volatile uint32_t s_probe_hit;

int flash_ecc_nmi_absorb(void)
{
    if (!s_probe) return 0;
    if ((FLASH->ECCDETR & FLASH_ECCR_ECCD) == 0u) return 0;
    FLASH->ECCDETR = FLASH_ECCR_ECCD;   /* write-1-to-clear */
    s_probe_hit = 1u;
    return 1;
}

int flash_read_checked(uint32_t offset, void *dst, size_t n)
{
    const volatile uint8_t *src = (const volatile uint8_t *)(FLASH_BASE + offset);
    uint8_t *d = (uint8_t *)dst;
    const uint32_t bus_err = SCB_CFSR_PRECISERR_Msk | SCB_CFSR_BFARVALID_Msk;

    uint32_t fm = __get_FAULTMASK();
    __set_FAULTMASK(1u);
    uint32_t ccr = SCB->CCR;
    SCB->CCR = ccr | SCB_CCR_BFHFNMIGN_Msk;
    SCB->CFSR = bus_err;                   /* clear stale bus-fault status (W1C) */
    if (FLASH->ECCDETR & FLASH_ECCR_ECCD)  /* a stale flag would fail this read */
        FLASH->ECCDETR = FLASH_ECCR_ECCD;
    s_probe_hit = 0u;
    s_probe = 1u;
    __DSB(); __ISB();

    for (size_t i = 0; i < n && !s_probe_hit; i++) {
        d[i] = src[i];
        __DSB();                           /* take a pending NMI before the next byte */
    }
    __ISB();

    int bad = s_probe_hit != 0u;
    if (FLASH->ECCDETR & FLASH_ECCR_ECCD) { FLASH->ECCDETR = FLASH_ECCR_ECCD; bad = 1; }
    if (SCB->CFSR & bus_err)              { SCB->CFSR = bus_err;              bad = 1; }
    s_probe = 0u;

    SCB->CCR = ccr;
    __DSB(); __ISB();
    __set_FAULTMASK(fm);
    return bad ? -1 : 0;
}
