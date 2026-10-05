/*
 * ota_commit.c — RAM-resident PSRAM -> internal-flash writer for hostless OTA.
 *
 * There is no second bank to install into (the 928 KB image area does not fit one bank of
 * the 1 MB part, and there is no bootloader), so the running firmware is erased and
 * rewritten IN PLACE. See docs/design/ota-fallback.md for the options to add a fallback.
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
#include "flash_layout.h"
#include "stm32h5xx_hal.h"

#include <stdio.h>
#include <string.h>

/* Standard STM32 flash unlock keys (not in the CMSIS device header). */
#define OTA_FLASH_KEY1 0x45670123u
#define OTA_FLASH_KEY2 0xCDEF89ABu

#define FLASH_BASE_ADDR   0x08000000u
#define FLASH_SECTOR_BYTES 0x2000u      /* 8 KB sectors  */
#define FLASH_SECTOR_SHIFT 13u          /* 1<<13 = 8 KB (shift instead of divide) */

/* Scratch sector for the self-test: a sector of the persistence area kept free for
   it (config_store.h), as a 2 MB-reference offset mapped onto this chip. */
#define OTA_SCRATCH_REF_OFF  0x1F0000u

/* Bank size of this chip, set before any RAM-resident code runs (that code cannot
   call into flash while flash is being erased). */
static uint32_t s_bank_bytes;

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

/* no-tree-loop-distribute-patterns: GCC must not turn a fill/compare loop here into a memset or
   memcmp call, which lives in the flash being erased (fine at -Og today, a brick at -O2). */
#define RAMFUNC __attribute__((section(".RamFunc"), noinline, used, \
                               optimize("no-tree-loop-distribute-patterns")))

/* Hardening of the in-place commit (see docs/design/ota-fallback.md, option 1):
   - every sector of the image gets a CRC32, computed by the CRC unit over bytes read through the
     HAL PSRAM path right after the SHA-256 reverify (so the table describes the hashed image);
   - before anything is erased, every sector is read again through the raw register path the
     commit uses and checked against its CRC: a mismatch refuses the commit with flash untouched;
   - each sector is erased, programmed, checked for flash error flags and read back, and retried
     up to OTA_SECTOR_TRIES times. */
#define OTA_MAX_SECTORS   128u           /* 1 MB / 8 KB: covers the 928 KB image area */
#define OTA_READ_TRIES    3u
#define OTA_SECTOR_TRIES  3u
#define OTA_FLASH_ERRS    (FLASH_SR_WRPERR | FLASH_SR_PGSERR | FLASH_SR_STRBERR | FLASH_SR_INCERR)
#define OTA_FLASH_CLR     (FLASH_CCR_CLR_WRPERR | FLASH_CCR_CLR_PGSERR | FLASH_CCR_CLR_STRBERR | \
                           FLASH_CCR_CLR_INCERR | FLASH_CCR_CLR_EOP)
static uint32_t s_sector_crc[OTA_MAX_SECTORS];

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
    if (off < s_bank_bytes) {
        sector = off >> FLASH_SECTOR_SHIFT;
        bksel  = 0;
    } else {
        sector = (off - s_bank_bytes) >> FLASH_SECTOR_SHIFT;
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

/* CRC32 (CRC unit defaults: poly 0x04C11DB7, init 0xFFFFFFFF) of `len` bytes, len a multiple
   of 4. Register-only, so it runs while flash is erased. */
RAMFUNC static uint32_t ram_crc32(const uint8_t *buf, uint32_t len) {
    CRC->CR = CRC_CR_RESET;
    const uint32_t *w = (const uint32_t *)buf;
    for (uint32_t i = 0; i < (len >> 2); i++) CRC->DR = w[i];
    return CRC->DR;
}

/* Error flags of the last flash operation, cleared. 0 = it went through. */
RAMFUNC static uint32_t ram_flash_take_errors(void) {
    uint32_t e = FLASH->NSSR & OTA_FLASH_ERRS;
    FLASH->NSCCR = OTA_FLASH_CLR;
    return e;
}

/* Read one sector of the staged image through the raw register path into `buf` (tail past
   `len` filled with 0xFF) and check it against its CRC, retrying the read. 0 = matches. */
RAMFUNC static int ram_read_checked(uint32_t psram_addr, uint8_t *buf, uint32_t len,
                                    uint32_t crc) {
    for (uint32_t t = 0; t < OTA_READ_TRIES; t++) {
        for (uint32_t i = len; i < FLASH_SECTOR_BYTES; i++) buf[i] = 0xFF;
        for (uint32_t off = 0; off < len; off += OTA_RD_BURST) {
            uint32_t n = len - off;
            if (n > OTA_RD_BURST) n = OTA_RD_BURST;
            ram_psram_read_burst(psram_addr + off, buf + off, n);
        }
        if (ram_crc32(buf, FLASH_SECTOR_BYTES) == crc) return 0;
    }
    return -1;
}

/* Erase + program one sector from `buf`, then check the error flags and read it back;
   retried. 0 = the sector now holds exactly `buf`. */
RAMFUNC static int ram_flash_sector(uint32_t flash_abs, const uint8_t *buf) {
    for (uint32_t t = 0; t < OTA_SECTOR_TRIES; t++) {
        (void)ram_flash_take_errors();
        ram_flash_erase(flash_abs);
        uint32_t err = ram_flash_take_errors();
        ram_flash_program(flash_abs, buf, FLASH_SECTOR_BYTES);
        err |= ram_flash_take_errors();
        ram_iwdg_kick();
        if (err) continue;
        const volatile uint32_t *f = (const volatile uint32_t *)flash_abs;
        const uint32_t *b = (const uint32_t *)buf;
        uint32_t i = 0;
        while (i < (FLASH_SECTOR_BYTES >> 2) && f[i] == b[i]) i++;
        if (i == (FLASH_SECTOR_BYTES >> 2)) return 0;
    }
    return -1;
}

/* The real in-place commit, from RAM with IRQs off (the vectors and code are being rewritten).
   A pre-pass reads every sector through the raw path and checks it against its CRC BEFORE the
   first erase: a mismatch returns -1 with flash untouched (the caller re-enables IRQs and
   refuses). Then every sector is read (checked again), erased, programmed, checked and read
   back, and the chip resets. A sector that still fails after its retries is left as it is: the
   pod needs USB DFU then, exactly as after a power cut (no bootloader yet). */
RAMFUNC static int ram_commit_all(uint32_t nsectors, uint32_t total, uint8_t *buf) {
    __disable_irq();
    for (uint32_t s = 0; s < nsectors; s++) {
        uint32_t base = s << FLASH_SECTOR_SHIFT;
        uint32_t len  = total - base;
        if (len > FLASH_SECTOR_BYTES) len = FLASH_SECTOR_BYTES;
        if (ram_read_checked(OTA_PSRAM_BASE + base, buf, len, s_sector_crc[s]) != 0) {
            __enable_irq();
            return -1;
        }
        ram_iwdg_kick();
    }
    ram_flash_unlock();
    for (uint32_t s = 0; s < nsectors; s++) {
        uint32_t base = s << FLASH_SECTOR_SHIFT;
        uint32_t len  = total - base;
        if (len > FLASH_SECTOR_BYTES) len = FLASH_SECTOR_BYTES;
        (void)ram_read_checked(OTA_PSRAM_BASE + base, buf, len, s_sector_crc[s]);   /* passed above */
        (void)ram_flash_sector(FLASH_BASE_ADDR + base, buf);
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
    if (ota_target() != OTA_TARGET_FIRMWARE) return ota_install_blob();
    uint32_t total    = ota_size();
    uint32_t nsectors = (total + (FLASH_SECTOR_BYTES - 1u)) / FLASH_SECTOR_BYTES;

    printf("[ota] committing %lu bytes (%lu sectors) — do NOT power off\n",
           (unsigned long)total, (unsigned long)nsectors);
    /* A copy that survives a power cut, for the bootloader planned next. Before the bus grab
       below: it takes the bus itself (w25q_open). Never blocks the install. */
    (void)ota_store_fw_copy();
    /* Stop every gateware PSRAM master and drain to idle BEFORE grabbing the bus: the commit
       READS the staged image from PSRAM to write it into internal flash, so a live iCE40
       reader mid-burst contending on the shared bus could corrupt those reads and flash a bad
       image (a brick — worse than the swap wedge).  Must happen while still flash-resident. */
    signal_engine_quiesce_psram_masters();
    /* Grab the shared PSRAM bus (HAL GPIO) BEFORE going RAM-resident. */
    psram_bus_acquire();
    /* ota_end hashed the image, but PSRAM 0 is also the LA capture region, and any master that
       wrote there since (an orphaned capture, a capture that slipped past the gate) would be
       flashed: a brick.  Hash it again now, with the bus held until the reset, so what is
       flashed is exactly what was verified. */
    if (ota_reverify_held() != 0) {
        psram_bus_release();
        printf("[ota] commit refused: %s\n", ota_error());
        return -1;
    }
    if (nsectors > OTA_MAX_SECTORS) {
        psram_bus_release();
        printf("[ota] commit refused: %lu sectors, more than %u\n",
               (unsigned long)nsectors, (unsigned)OTA_MAX_SECTORS);
        return -1;
    }
    /* Per-sector CRCs of the hashed image, read through the HAL path (bus still held). */
    __HAL_RCC_CRC_CLK_ENABLE();
    for (uint32_t sct = 0; sct < nsectors; sct++) {
        uint32_t base = sct * FLASH_SECTOR_BYTES;
        uint32_t len  = total - base;
        if (len > FLASH_SECTOR_BYTES) len = FLASH_SECTOR_BYTES;
        memset(s_sector, 0xFF, sizeof(s_sector));
        if (psram_read(OTA_PSRAM_BASE + base, s_sector, len) != 0) {
            psram_bus_release();
            printf("[ota] commit refused: PSRAM read failed at sector %lu\n", (unsigned long)sct);
            return -1;
        }
        s_sector_crc[sct] = ram_crc32(s_sector, FLASH_SECTOR_BYTES);
    }
    s_bank_bytes = flash_layout_bank_size();
    if (ram_commit_all(nsectors, total, s_sector) != 0) {   /* returns only if refused */
        psram_bus_release();
        printf("[ota] commit refused: the raw PSRAM read does not match the verified image; "
               "nothing was erased\n");
        return -1;
    }
    return -1;                                    /* unreachable */
}

int ota_commit_selftest(void) {
    /* Build a known 8 KB pattern and stage it in PSRAM at a scratch offset via the
       normal HAL path (safe, flash-resident). */
    /* The pattern is a formula, so it is built in the commit's sector buffer and recomputed for
       the verify instead of being kept in a second 8 KB buffer. */
#define SELFTEST_PAT(i) ((uint8_t)(((i) * 197u + 31u) & 0xFFu))
    for (uint32_t i = 0; i < FLASH_SECTOR_BYTES; i++)
        s_sector[i] = SELFTEST_PAT(i);

    const uint32_t scratch_psram = OTA_PSRAM_BASE;   /* reuse offset 0 (OTA idle) */
    const uint32_t scratch_addr  = FLASH_BASE_ADDR + flash_layout_store_off(OTA_SCRATCH_REF_OFF);
    s_bank_bytes = flash_layout_bank_size();
    psram_bus_acquire();
    if (psram_write(scratch_psram, s_sector, FLASH_SECTOR_BYTES) != 0) {
        psram_bus_release();
        printf("[ota-selftest] PSRAM stage write failed\n");
        return -1;
    }

    /* Run the RAM-resident primitives against the SCRATCH sector (not the app).
       IRQs off only during the flash program quad-words (inside ram_flash_program);
       the app code stays intact, so we don't disable IRQs for the whole thing. */
    __HAL_RCC_CRC_CLK_ENABLE();
    uint32_t crc = ram_crc32(s_sector, FLASH_SECTOR_BYTES);
    ram_flash_unlock();
    /* The same checked read + checked sector write the commit uses. */
    int rc = ram_read_checked(scratch_psram, s_sector, FLASH_SECTOR_BYTES, crc);
    if (rc == 0) rc = ram_flash_sector(scratch_addr, s_sector);
    psram_bus_release();
    if (rc != 0) {
        printf("[ota-selftest] FAIL: %s\n", "checked PSRAM read or checked sector write");
        return -1;
    }

    /* Verify via a direct memory-mapped flash read. */
    const uint8_t *flash = (const uint8_t *)scratch_addr;
    int bad = -1;
    for (uint32_t i = 0; i < FLASH_SECTOR_BYTES; i++) {
        if (flash[i] != SELFTEST_PAT(i)) { bad = (int)i; break; }
    }
    if (bad >= 0) {
        printf("[ota-selftest] FAIL: scratch mismatch at byte %d (flash=0x%02x want=0x%02x)\n",
               bad, flash[bad], SELFTEST_PAT((uint32_t)bad));
        return -1;
    }
    printf("[ota-selftest] PASS: RAM-resident PSRAM-read + flash erase/program verified "
           "on scratch sector 0x%08lx (%u B)\n",
           (unsigned long)scratch_addr, (unsigned)FLASH_SECTOR_BYTES);
    return 0;
}
