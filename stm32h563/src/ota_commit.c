/*
 * ota_commit.c — RAM-resident PSRAM -> internal-flash writer for hostless OTA.
 *
 * The full image (~1.6 MB, app + embedded ESP/FPGA blobs) is larger than one
 * 1 MB flash bank, so the running firmware must be erased and rewritten IN PLACE.
 * Code that erases the flash it runs from must execute from SRAM, and so must the
 * routine that reads the source image back from PSRAM (psram_read() goes through
 * the OCTOSPI HAL, which lives in the flash being erased).  So the erase/read/
 * program primitives below are placed in .RamFunc (copied to SRAM at boot by the
 * .data startup, see the linker script) and touch ONLY registers + a RAM buffer —
 * no HAL, no printf, no libcalls (division/memcpy) — once interrupts are off.
 *
 * SAFETY: ota_commit_selftest() runs the SAME primitives against a scratch flash
 * sector in the free region (never the app), so the register-level code can be
 * validated on the bench WITHOUT any brick risk.  ota_commit() (the real, in-place
 * rewrite) must only be trusted after the self-test passes.
 */
#include "ota.h"
#include "psram.h"
#include "signal_engine.h"   /* signal_engine_quiesce_psram_masters() before the bus grab */
#include "board_pins.h"
#include "stm32h5xx_hal.h"

#include <stdio.h>
#include <string.h>

/* Standard STM32 flash unlock keys (not in the CMSIS device header). */
#define OTA_FLASH_KEY1 0x45670123u
#define OTA_FLASH_KEY2 0xCDEF89ABu

#define FLASH_BASE_ADDR   0x08000000u
#define FLASH_BANK_BYTES  0x00100000u   /* 1 MB per bank */
#define FLASH_SECTOR_BYTES 0x2000u      /* 8 KB sectors  */
#define FLASH_SECTOR_SHIFT 13u          /* 1<<13 = 8 KB (shift instead of divide) */

/* Scratch sector for the self-test: in the free flash between the image tail
   (~0x08189b28) and the persistence sectors (config 0x081FA000). Bank 2. */
#define OTA_SCRATCH_ADDR  0x081F0000u

/* OTA image staging base in PSRAM (must match ota.c's OTA_PSRAM_BASE). */
#define OTA_PSRAM_BASE    0u

/* PSRAM read burst per CS-low window — kept short so a CPU-paced read stays under
   the APS6404L tCEM.  Validated by the self-test; reduce if it ever fails. */
#define OTA_RD_BURST      128u

/* CCR value for the APS6404L 0xEB quad read: 4-line instruction, 4-line 24-bit
   address, 4-line data (8-bit instruction, no alt bytes).  Same field constants
   the XSPI HAL uses, so it matches psram.c's configured transfer. */
#define OTA_CCR_QREAD ( XSPI_CCR_IMODE_0  | XSPI_CCR_IMODE_1  \
                      | XSPI_CCR_ADMODE_0 | XSPI_CCR_ADMODE_1 | XSPI_CCR_ADSIZE_1 \
                      | XSPI_CCR_DMODE_0  | XSPI_CCR_DMODE_1 )
#define OTA_QREAD_DUMMY   6u

#define RAMFUNC __attribute__((section(".RamFunc"), noinline, used))

/* ---- RAM-resident primitives (no flash access, no libcalls) --------------- */

/* Tight PSRAM quad-read of `n` (<= OTA_RD_BURST) bytes at `addr` into `buf`,
   framed by PE4 (PSRAM CS) as a GPIO.  The OCTOSPI is already configured (QPI,
   50 MHz) by psram_init(); this reuses that state.  Returns nothing (the
   self-test validates correctness). */
RAMFUNC static void ram_psram_read_burst(uint32_t addr, uint8_t *buf, uint32_t n) {
    /* CS low (PE4): BSRR reset = upper half. */
    PSRAM_CS_PORT->BSRR = (uint32_t)PSRAM_CS_PIN << 16;

    while (OCTOSPI1->SR & OCTOSPI_SR_BUSY) { }
    OCTOSPI1->FCR = OCTOSPI_FCR_CTCF;
    OCTOSPI1->DLR = n - 1u;
    OCTOSPI1->CR  = (OCTOSPI1->CR & ~OCTOSPI_CR_FMODE) | (1u << OCTOSPI_CR_FMODE_Pos); /* indirect read */
    OCTOSPI1->TCR = (OCTOSPI1->TCR & ~OCTOSPI_TCR_DCYC) | (OTA_QREAD_DUMMY << OCTOSPI_TCR_DCYC_Pos);
    OCTOSPI1->CCR = OTA_CCR_QREAD;
    OCTOSPI1->IR  = 0xEBu;
    OCTOSPI1->AR  = addr;                 /* writing AR triggers the transaction */

    for (uint32_t i = 0; i < n; i++) {
        while (!(OCTOSPI1->SR & (OCTOSPI_SR_FTF | OCTOSPI_SR_TCF))) { }
        buf[i] = *(volatile uint8_t *)&OCTOSPI1->DR;
    }
    while (!(OCTOSPI1->SR & OCTOSPI_SR_TCF)) { }
    OCTOSPI1->FCR = OCTOSPI_FCR_CTCF;

    /* CS high (PE4): BSRR set = lower half. */
    PSRAM_CS_PORT->BSRR = (uint32_t)PSRAM_CS_PIN;
}

RAMFUNC static void ram_flash_unlock(void) {
    if (FLASH->NSCR & FLASH_CR_LOCK) {
        FLASH->NSKEYR = OTA_FLASH_KEY1;
        FLASH->NSKEYR = OTA_FLASH_KEY2;
    }
}

RAMFUNC static void ram_flash_wait(void) {
    while (FLASH->NSSR & FLASH_SR_BSY) { }
}

/* Erase one 8 KB sector at absolute address `abs` (must be sector-aligned). */
RAMFUNC static void ram_flash_erase(uint32_t abs) {
    uint32_t off = abs - FLASH_BASE_ADDR;
    uint32_t sector, bksel;
    if (off < FLASH_BANK_BYTES) {
        sector = off >> FLASH_SECTOR_SHIFT;
        bksel  = 0;
    } else {
        sector = (off - FLASH_BANK_BYTES) >> FLASH_SECTOR_SHIFT;
        bksel  = FLASH_CR_BKSEL;
    }
    ram_flash_wait();
    FLASH->NSCR &= ~(FLASH_CR_SNB | FLASH_CR_BKSEL);
    FLASH->NSCR |= (FLASH_CR_SER | bksel | (sector << FLASH_CR_SNB_Pos) | FLASH_CR_START);
    ram_flash_wait();
    FLASH->NSCR &= ~FLASH_CR_SER;
}

/* Program `len` bytes (rounded up to a 16-byte quad-word) from `src` into flash
   at `abs`.  `abs` is 16-byte aligned (sector-aligned); the caller pre-fills the
   tail of `src` past `len` with 0xFF so a short final sector programs erased
   bytes.  Each quad-word is written with IRQs masked (the flash stalls code fetch
   while BSY) — nested-safe via PRIMASK, so this is correct whether the caller
   already had IRQs off (real commit) or on (self-test). */
RAMFUNC static void ram_flash_program(uint32_t abs, const uint8_t *src, uint32_t len) {
    volatile uint32_t *dst = (volatile uint32_t *)abs;
    const uint32_t *s = (const uint32_t *)src;   /* src buffer is 4-byte aligned   */
    uint32_t nbytes = (len + 15u) & ~15u;        /* round up to whole quad-words    */
    uint32_t words  = nbytes >> 2;
    for (uint32_t w = 0; w < words; w += 4) {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        ram_flash_wait();
        FLASH->NSCR |= FLASH_CR_PG;
        dst[w + 0] = s[w + 0];
        dst[w + 1] = s[w + 1];
        dst[w + 2] = s[w + 2];
        dst[w + 3] = s[w + 3];
        ram_flash_wait();
        FLASH->NSCR &= ~FLASH_CR_PG;
        __set_PRIMASK(primask);
    }
}

RAMFUNC static void ram_iwdg_kick(void) {
    IWDG->KR = 0x0000AAAAu;
}

/* One sector: read from PSRAM into `buf` (in OTA_RD_BURST windows), erase the
   flash sector, program it.  `len` <= FLASH_SECTOR_BYTES.  Source buffer bytes
   beyond `len` are left as-is (the caller pre-fills the tail with 0xFF). */
RAMFUNC static void ram_write_one_sector(uint32_t flash_abs, uint32_t psram_addr,
                                         uint8_t *buf, uint32_t len) {
    for (uint32_t off = 0; off < len; off += OTA_RD_BURST) {
        uint32_t n = len - off;
        if (n > OTA_RD_BURST) n = OTA_RD_BURST;
        ram_psram_read_burst(psram_addr + off, buf + off, n);
    }
    ram_flash_erase(flash_abs);
    ram_flash_program(flash_abs, buf, len);
    ram_iwdg_kick();
}

/* The real in-place commit: erase + rewrite every sector of the image from PSRAM,
   with IRQs off (the ISR vectors/code are being erased), then reset.  Never
   returns.  `nsectors` is precomputed by the caller (no division here). */
RAMFUNC static void ram_commit_all(uint32_t nsectors, uint32_t total, uint8_t *buf) {
    __disable_irq();
    ram_flash_unlock();
    for (uint32_t s = 0; s < nsectors; s++) {
        uint32_t base = s << FLASH_SECTOR_SHIFT;
        uint32_t len  = total - base;
        if (len > FLASH_SECTOR_BYTES) len = FLASH_SECTOR_BYTES;
        /* Pre-fill the sector buffer with 0xFF so a short final sector programs
           erased bytes in its tail quad-word. */
        for (uint32_t i = len; i < FLASH_SECTOR_BYTES; i++) buf[i] = 0xFF;
        ram_write_one_sector(FLASH_BASE_ADDR + base, OTA_PSRAM_BASE + base, buf, len);
    }
    /* SYSRESETREQ. */
    __DSB();
    SCB->AIRCR = (0x5FAu << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk;
    for (;;) { }
}

/* ---- entry points --------------------------------------------------------- */

/* Sector buffer (RAM/.bss).  One 8 KB sector staged at a time. */
static uint8_t s_sector[FLASH_SECTOR_BYTES] __attribute__((aligned(4)));

int ota_commit(void) {
    if (ota_get_state() != OTA_VERIFIED) return -1;
    uint32_t total    = ota_size();
    uint32_t nsectors = (total + (FLASH_SECTOR_BYTES - 1u)) / FLASH_SECTOR_BYTES;

    printf("[ota] committing %lu bytes (%lu sectors) — do NOT power off\n",
           (unsigned long)total, (unsigned long)nsectors);
    /* Stop every gateware PSRAM master and drain to idle BEFORE grabbing the bus: the commit
       READS the staged image from PSRAM to write it into internal flash, so a live iCE40
       reader mid-burst contending on the shared bus could corrupt those reads and flash a bad
       image (a brick — worse than the swap wedge).  Must happen while still flash-resident. */
    signal_engine_quiesce_psram_masters();
    /* Grab the shared PSRAM bus (HAL GPIO) BEFORE going RAM-resident. */
    psram_bus_acquire();
    ram_commit_all(nsectors, total, s_sector);   /* does not return */
    return -1;                                    /* unreachable */
}

int ota_commit_selftest(void) {
    /* Build a known 8 KB pattern and stage it in PSRAM at a scratch offset via the
       normal HAL path (safe, flash-resident). */
    static uint8_t pat[FLASH_SECTOR_BYTES];
    for (uint32_t i = 0; i < FLASH_SECTOR_BYTES; i++)
        pat[i] = (uint8_t)((i * 197u + 31u) & 0xFFu);

    const uint32_t scratch_psram = OTA_PSRAM_BASE;   /* reuse offset 0 (OTA idle) */
    psram_bus_acquire();
    if (psram_write(scratch_psram, pat, FLASH_SECTOR_BYTES) != 0) {
        psram_bus_release();
        printf("[ota-selftest] PSRAM stage write failed\n");
        return -1;
    }

    /* Run the RAM-resident primitives against the SCRATCH sector (not the app).
       IRQs off only during the flash program quad-words (inside ram_flash_program);
       the app code stays intact, so we don't disable IRQs for the whole thing. */
    ram_flash_unlock();
    ram_write_one_sector(OTA_SCRATCH_ADDR, scratch_psram, s_sector, FLASH_SECTOR_BYTES);
    psram_bus_release();

    /* Verify via a direct memory-mapped flash read. */
    const uint8_t *flash = (const uint8_t *)OTA_SCRATCH_ADDR;
    int bad = -1;
    for (uint32_t i = 0; i < FLASH_SECTOR_BYTES; i++) {
        if (flash[i] != pat[i]) { bad = (int)i; break; }
    }
    if (bad >= 0) {
        printf("[ota-selftest] FAIL: scratch mismatch at byte %d (flash=0x%02x want=0x%02x)\n",
               bad, flash[bad], pat[bad]);
        return -1;
    }
    printf("[ota-selftest] PASS: RAM-resident PSRAM-read + flash erase/program verified "
           "on scratch sector 0x%08lx (%u B)\n",
           (unsigned long)OTA_SCRATCH_ADDR, (unsigned)FLASH_SECTOR_BYTES);
    return 0;
}
