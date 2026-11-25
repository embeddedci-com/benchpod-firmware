/*
 * ice40_flash.c — reprogram the iCE40 config flash (W25Q64) from the STM32 over
 * the shared OCTOSPI bus (single-SPI, GPIO chip-select: PE3, PB6 on digital v3r4).  Ported from the
 * RP2350 ice40_flash.c; the SPI transport is now the H5 OCTOSPI (reusing the
 * handle from psram.c) and the chip-select / iCE40 control are GPIOs.
 *
 * ⚠ Hardware-verify: the bus-ownership handoff (PG0), the CRESET/CDONE timing,
 * and the W25Q64 command timings (erase/program WIP, tHP) need bench validation.
 */
#include "ice40_flash.h"
#include <stdbool.h>
#include "psram.h"
#include "w25q.h"
#include "board_pins.h"
#include "pico_compat.h"
#include "stm32h5xx_hal.h"
#include "bp_log.h"
#include "ice40_switch.h"   /* erase count in status.fpga_switch */
#include <stdio.h>
#include <string.h>
#include "mbedtls/sha256.h"   /* the slave load hashes what it clocks out */

#define CDONE_TIMEOUT_MS  500u

#define FCS_HIGH()  HAL_GPIO_WritePin(ICE_FLASH_CS_PORT, ICE_FLASH_CS_PIN, GPIO_PIN_SET)

/* The flash CS (qspi_cs.h) + CRESET (PF13, drive low to hold iCE40 in reset) + CDONE
   (PF14, input).  PG0/PSRAM-CS/OCTOSPI pins are handled by psram_bus_acquire. */
static void flash_pins(void)
{
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOF_CLK_ENABLE();
    g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_LOW;

    w25q_cs_output();

    HAL_GPIO_WritePin(ICE_CRESET_PORT, ICE_CRESET_PIN, GPIO_PIN_RESET);  /* hold reset */
    g.Mode = GPIO_MODE_OUTPUT_PP; g.Pin = ICE_CRESET_PIN; HAL_GPIO_Init(ICE_CRESET_PORT, &g);
    HAL_GPIO_WritePin(ICE_CRESET_PORT, ICE_CRESET_PIN, GPIO_PIN_RESET);

    g.Mode = GPIO_MODE_INPUT; g.Pin = ICE_CDONE_PIN;
    HAL_GPIO_Init(ICE_CDONE_PORT, &g);
}

int ice40_flash_read_id(uint8_t id[3])
{
    psram_bus_acquire();        /* PG0 high, OCTOSPI pins -> AF, PSRAM CS high */
    flash_pins();
    HAL_GPIO_WritePin(ICE_CRESET_PORT, ICE_CRESET_PIN, GPIO_PIN_RESET);
    sleep_ms(1);
    w25q_wake();
    int rc = w25q_read_id(id);
    /* leave the iCE40 in reset is bad — release it so it reconfigures */
    HAL_GPIO_WritePin(ICE_CRESET_PORT, ICE_CRESET_PIN, GPIO_PIN_SET);
    psram_bus_release();
    return rc;
}

/* Configuration failed, or the config flash no longer holds a whole image: keep the iCE40 in
   reset (every iCE40 pin Hi-Z) and deselect the flash.  Released from reset it keeps searching
   the flash for a bitstream, driving SCLK and the flash /CS while the flash drives SO (= PSRAM
   IO1), and the psram_init() that follows drives the same lines: two drivers.  The next reflash
   (boot recovery, a retried swap, flash-ice40) releases it. */
void ice40_hold_off_bus(void)
{
    GPIO_InitTypeDef g = {0};
    g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_LOW; g.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_WritePin(ICE_CRESET_PORT, ICE_CRESET_PIN, GPIO_PIN_RESET);
    g.Pin = ICE_CRESET_PIN; HAL_GPIO_Init(ICE_CRESET_PORT, &g);
    FCS_HIGH();
    g.Pin = ICE_FLASH_CS_PIN; HAL_GPIO_Init(ICE_FLASH_CS_PORT, &g);
    FCS_HIGH();
    LOG_I("[ice40] held in reset (off the shared bus) until the next reflash\n");
}

static int mem_read(void *ctx, uint32_t off, uint8_t *buf, uint32_t n)
{
    memcpy(buf, (const uint8_t *)ctx + off, n);
    return 0;
}

/* Hand the bus to the iCE40, release it from reset and wait for CDONE: it reads its
   configuration from the flash. The bus is the STM32's on entry (psram_bus_acquire, flash_pins). */
static int release_and_wait_cdone(void)
{
    /* Hand the config-flash bus FULLY to the iCE40 before releasing it from reset (previously
       CRESET was released while the STM32 still owned the bus, so the iCE40 came out of reset
       into a flash it couldn't reach and CDONE never asserted).  Handover, not a plain release:
       an outer hold left open here would keep PG0 high through the config read. */
    psram_bus_release();
    psram_bus_handover();
    /* ...BUT hold the PSRAM /CS DRIVEN HIGH (not the Hi-Z psram_bus_release leaves) for the whole
       config read: the shared SCLK/SO/SI edges the iCE40 clocks out can capacitively drag a Hi-Z
       (pulled-up) /CS low, glitch-selecting the PSRAM so it drives IO0(=SO) and corrupts the
       bitstream — an intermittent "CDONE never rose / iCE40 mute after reflash".  A driven high
       resists it.  Cleaned up by the psram_init()/psram_bus_acquire() ice40_reflash_image() runs. */
    psram_cs_park_high();
    {
        GPIO_InitTypeDef gz = {0};
        gz.Mode = GPIO_MODE_ANALOG;          /* flash /CS -> Hi-Z (iCE40 drives it in config) */
        gz.Pin  = ICE_FLASH_CS_PIN;
        HAL_GPIO_Init(ICE_FLASH_CS_PORT, &gz);
    }
    HAL_GPIO_WritePin(ICE_CRESET_PORT, ICE_CRESET_PIN, GPIO_PIN_SET);  /* release iCE40 */
    {
        uint32_t t0 = HAL_GetTick();
        while (HAL_GPIO_ReadPin(ICE_CDONE_PORT, ICE_CDONE_PIN) == GPIO_PIN_RESET) {
            if (HAL_GetTick() - t0 > CDONE_TIMEOUT_MS) {
                LOG_E("[ice40] CDONE never went high — configuration failed\n");
                ice40_hold_off_bus();
                return -1;
            }
        }
    }
    LOG_BOOT("[ice40] reconfigured OK (CDONE high)\n");
    return 0;
}

/* Page buffers shared by the write, the compare and the slave load (one runs at a time, all on
   the hw worker): RAM is tight. */
static uint8_t s_pg[W25Q_PAGE], s_rb[W25Q_PAGE];

/* The identity record of the image at offset 0 (boot_policy.h), cached. */
static gw_ident_t s_ident;
static bool s_ident_ok;

const gw_ident_t *ice40_ident(void) { return s_ident_ok ? &s_ident : NULL; }

void ice40_ident_load(void)
{
    s_ident_ok = false;
    if (w25q_open() != 0) return;
    gw_ident_t r;
    if (w25q_read(GW_IDENT_OFF, (uint8_t *)&r, sizeof(r)) == 0 && gw_ident_valid(&r)) {
        s_ident = r;
        s_ident_ok = true;
    }
    w25q_close();
    if (s_ident_ok)
        LOG_I("[ice40] config flash image: from slot %u, %lu bytes, sha256 %02x%02x%02x%02x...\n",
              (unsigned)s_ident.slot, (unsigned long)s_ident.len, s_ident.sha256[0], s_ident.sha256[1],
              s_ident.sha256[2], s_ident.sha256[3]);
    else
        LOG_I("[ice40] config flash image: not recorded yet\n");
}

/* Write `ident` at GW_IDENT_OFF and read it back. Bus held, sector erased. 0 = ok. */
static int ident_write(const gw_ident_t *ident)
{
    gw_ident_t rb;
    if (w25q_program(GW_IDENT_OFF, (const uint8_t *)ident, sizeof(*ident)) != 0 ||
        w25q_read(GW_IDENT_OFF, (uint8_t *)&rb, sizeof(rb)) != 0 || memcmp(&rb, ident, sizeof(rb)) != 0) {
        LOG_W("[ice40] could not record the image identity (the next boot compares again)\n");
        return -1;
    }
    s_ident = *ident;
    s_ident_ok = true;
    return 0;
}

int ice40_flash_program(const uint8_t *data, size_t len)
{
    if (!data) return -1;
    return ice40_flash_program_src(mem_read, (void *)data, len);
}

int ice40_flash_program_src(ice40_src_read_fn rd, void *ctx, size_t len)
{
    return ice40_flash_program_src_ident(rd, ctx, len, NULL);
}

int ice40_flash_program_src_ident(ice40_src_read_fn rd, void *ctx, size_t len, const gw_ident_t *ident)
{
    if (!rd || len == 0 || len > GW_IDENT_OFF) return -1;
    bool erased = false;   /* from the first erase on, the flash holds no whole image */
    uint8_t *pg = s_pg, *rb = s_rb;   /* source page, read-back page */

    psram_bus_acquire();
    flash_pins();
    HAL_GPIO_WritePin(ICE_CRESET_PORT, ICE_CRESET_PIN, GPIO_PIN_RESET);  /* iCE40 in reset */
    sleep_ms(2);
    w25q_wake();
    sleep_ms(1);

    uint8_t id[3] = {0};
    if (w25q_read_id(id) != 0 || id[0] == 0x00 || id[0] == 0xFF) {
        LOG_W("[ice40] flash not responding (ID %02x %02x %02x)\n", id[0], id[1], id[2]);
        goto fail;
    }
    LOG_BOOT("[ice40] flash ID %02x %02x %02x, programming %u bytes\n",
             id[0], id[1], id[2], (unsigned)len);

    /* The record names the image about to be replaced: drop it first, so a write that fails or is
       cut leaves no record (the next boot compares) rather than a wrong one. */
    s_ident_ok = false;
    ice40_switch_note_erases(1);
    if (w25q_erase_sector(GW_IDENT_OFF) != 0) goto fail;
    /* Erase enough 4 KB sectors to hold the bitstream. */
    for (uint32_t a = 0; a < len; a += W25Q_SECTOR) {
        erased = true;
        ice40_switch_note_erases(1);
        if (w25q_erase_sector(a) != 0) goto fail;
    }
    /* Program in 256-byte pages. */
    for (uint32_t a = 0; a < len; a += W25Q_PAGE) {
        uint32_t n = (len - a < W25Q_PAGE) ? (uint32_t)(len - a) : W25Q_PAGE;
        if (rd(ctx, a, pg, n) != 0) goto fail;
        if (w25q_program(a, pg, n) != 0) goto fail;
    }

    /* Read-back verify: prove the flash content matches the bitstream, so a
       CDONE failure can be pinned on the config/handoff path rather than a bad
       write.  Still on the STM32-owned bus here (normal read, 0x03, no dummy). */
    for (uint32_t a = 0; a < len; a += W25Q_PAGE) {
        uint32_t n = (len - a < W25Q_PAGE) ? (uint32_t)(len - a) : W25Q_PAGE;
        if (rd(ctx, a, pg, n) != 0) goto fail;
        if (w25q_read(a, rb, n) != 0) goto fail;
        if (memcmp(rb, pg, n) != 0) {
            uint32_t k = 0; while (k < n && rb[k] == pg[k]) k++;
            LOG_E("[ice40] verify FAILED @0x%06lx: wrote %02x read %02x\n",
                  (unsigned long)(a + k), pg[k], rb[k]);
            goto fail;
        }
    }
    LOG_BOOT("[ice40] verify OK (%u bytes)\n", (unsigned)len);
    if (ident) (void)ident_write(ident);   /* a failed record only costs a compare at next boot */

    return release_and_wait_cdone();

fail:
    if (erased) {
        ice40_hold_off_bus();   /* a partial image: never let it configure from that */
    } else {
        HAL_GPIO_WritePin(ICE_CRESET_PORT, ICE_CRESET_PIN, GPIO_PIN_SET);   /* old image intact */
    }
    psram_bus_release();
    return -1;
}

/* Reload the iCE40 from what its config flash holds now: reset it and let it read its bitstream,
   with no erase and no write. For a fabric that hung or lost its configuration (ESD), and the
   first step of every recovery; rewriting the flash is only for an image that is damaged. */
int ice40_flash_boot(void)
{
    psram_bus_acquire();
    flash_pins();
    HAL_GPIO_WritePin(ICE_CRESET_PORT, ICE_CRESET_PIN, GPIO_PIN_RESET);  /* iCE40 in reset */
    sleep_ms(2);
    w25q_wake();
    sleep_ms(1);
    uint8_t id[3] = {0};
    if (w25q_read_id(id) != 0 || id[0] == 0x00 || id[0] == 0xFF) {
        LOG_W("[ice40] flash not responding (ID %02x %02x %02x): not reloaded\n", id[0], id[1], id[2]);
        HAL_GPIO_WritePin(ICE_CRESET_PORT, ICE_CRESET_PIN, GPIO_PIN_SET);
        psram_bus_release();
        return -1;
    }
    return release_and_wait_cdone();
}

/* Compare the config flash from offset 0 with `len` bytes of a source (a blob slot): 1 = equal,
   0 = differs, <0 = could not read. Holds the bus and the iCE40 in reset while reading, then lets
   the iCE40 reconfigure from the flash as it was. */
static uint32_t s_mismatch_at = 0xFFFFFFFFu;
uint32_t ice40_flash_last_mismatch(void) { return s_mismatch_at; }

int ice40_flash_matches_src(ice40_src_read_fn rd, void *ctx, size_t len)
{
    return ice40_flash_matches_src_ident(rd, ctx, len, NULL);
}

int ice40_flash_matches_src_ident(ice40_src_read_fn rd, void *ctx, size_t len, const gw_ident_t *ident)
{
    uint8_t *pg = s_pg, *rb = s_rb;
    psram_bus_acquire();
    flash_pins();
    HAL_GPIO_WritePin(ICE_CRESET_PORT, ICE_CRESET_PIN, GPIO_PIN_RESET);
    sleep_ms(2);
    w25q_wake();
    sleep_ms(1);
    int same = 1;
    for (uint32_t a = 0; a < len; a += W25Q_PAGE) {
        uint32_t n = (len - a < W25Q_PAGE) ? (uint32_t)(len - a) : W25Q_PAGE;
        if (rd(ctx, a, pg, n) != 0 || w25q_read(a, rb, n) != 0) { same = -1; break; }
        if (memcmp(pg, rb, n) != 0) {
            uint32_t k = 0; while (k < n && pg[k] == rb[k]) k++;
            LOG_W("[ice40] flash image differs at 0x%06lx: slot %02x, flash %02x\n",
                  (unsigned long)(a + k), pg[k], rb[k]);
            s_mismatch_at = a + k;
            same = 0;
            break;
        }
    }
    if (same == 1 && ident && len <= GW_IDENT_OFF) {
        s_ident_ok = false;
        ice40_switch_note_erases(1);
        if (w25q_erase_sector(GW_IDENT_OFF) == 0) (void)ident_write(ident);
    }
    (void)release_and_wait_cdone();
    return same;
}

int ice40_is_configured(void) {
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOF_CLK_ENABLE();
    g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_NOPULL; g.Pin = ICE_CDONE_PIN;
    HAL_GPIO_Init(ICE_CDONE_PORT, &g);   /* out of reset it is analog, which reads 0 */
    return HAL_GPIO_ReadPin(ICE_CDONE_PORT, ICE_CDONE_PIN) == GPIO_PIN_SET;
}


/* ---- SPI slave configuration straight from a W25Q slot (no erase, no write) ----------------
   The iCE40's SPI_SS is the flash /CS, and the STM32 drives it. With SS low when CRESET rises,
   the iCE40 comes up as an SPI slave (Lattice FPGA-TN-02001, section 13) and takes its bitstream
   on SPI_SI, which is the flash's DO line. So the STM32 starts an ordinary read (0x03) at the
   slot and just provides the clock: the flash streams the image into the iCE40 while the STM32
   reads the same bits (and hashes them). Sequence:
     1. bus to the STM32 (PG0), CRESET low (every iCE40 pin Hi-Z), flash /CS = SS low, send
        0x03 + the address on IO0;
     2. IO0 to an input: in slave mode the iCE40 drives its SPI_SO pin, which is IO0, so the
        STM32 never drives it again during the load (the flash ignores DI during a read);
     3. CRESET high with SS low: slave mode. 1.2 ms (more here) with no clock: memory clear;
     4. clock the image out (data-only reads, 1 line, at most 25 MHz, the slave-mode limit);
     5. SS high, at least 49 more clocks (we send 104), CDONE high.
   Lattice's sequence also toggles SS high for 8 clocks before the image; that would end the
   flash read, so we skip it: the image's own leading bytes come before its sync word. */
#define SLAVE_PRESCALER      9u      /* 250 MHz / 10 = 25 MHz */
#define SLAVE_CLEAR_US       1500u   /* >= 1200 us configuration memory clear */
#define SLAVE_TAIL_BYTES     13u     /* 104 clocks after the image, SS high (>= 49) */
#define SLAVE_CDONE_MS       20u
#define SLAVE_CHUNK          W25Q_PAGE   /* s_rb */

static void io0_input(void)
{
    GPIO_InitTypeDef g = {0};
    g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_NOPULL; g.Pin = XSPI_IO0_PIN;
    HAL_GPIO_Init(XSPI_IO0_PORT, &g);
}

static uint32_t s_slave_skew;   /* ice40_slave_test_fail: bytes to start early (a bad load) */
static uint32_t s_slave_fail_n;
void ice40_slave_test_fail(uint32_t n) { s_slave_fail_n = n; }

int ice40_slave_load(uint32_t addr, uint32_t len, const uint8_t sha256[32])
{
    uint8_t *buf = s_rb;
    int rc = 0;
    s_slave_skew = 0;
    if (s_slave_fail_n) { s_slave_fail_n--; s_slave_skew = 64u; }   /* no sync word where expected */

    psram_bus_acquire();
    flash_pins();                       /* flash /CS out high, CRESET low, CDONE in */
    HAL_GPIO_WritePin(ICE_CRESET_PORT, ICE_CRESET_PIN, GPIO_PIN_RESET);
    sleep_ms(1);
    w25q_wake();
    uint8_t id[3] = {0};
    if (w25q_read_id(id) != 0 || id[0] == 0x00 || id[0] == 0xFF) {
        LOG_W("[ice40] slave load: flash not responding (ID %02x %02x %02x)\n", id[0], id[1], id[2]);
        rc = -1;
        goto out_reset;
    }
    (void)psram_set_prescaler(SLAVE_PRESCALER);
    const uint64_t t0 = time_us_64();
    if (w25q_stream_begin(addr - s_slave_skew) != 0) { rc = -1; goto out_reset; }
    io0_input();
    HAL_GPIO_WritePin(ICE_CRESET_PORT, ICE_CRESET_PIN, GPIO_PIN_SET);   /* SS low: slave mode */
    sleep_us(SLAVE_CLEAR_US);

    mbedtls_sha256_context h;
    mbedtls_sha256_init(&h);
    mbedtls_sha256_starts(&h, 0);
    for (uint32_t off = 0; off < len; off += SLAVE_CHUNK) {
        uint32_t n = len - off < SLAVE_CHUNK ? len - off : SLAVE_CHUNK;
        if (w25q_stream_read(buf, n) != 0) { rc = -1; break; }
        mbedtls_sha256_update(&h, buf, n);
    }
    uint8_t got[32];
    mbedtls_sha256_finish(&h, got);
    mbedtls_sha256_free(&h);
    w25q_stream_end();                  /* SS high: the flash stops driving DO */
    if (rc == 0) (void)w25q_stream_read(buf, SLAVE_TAIL_BYTES);   /* the clocks that start it */
    if (rc == 0) {
        uint32_t t0 = HAL_GetTick();
        while (HAL_GPIO_ReadPin(ICE_CDONE_PORT, ICE_CDONE_PIN) == GPIO_PIN_RESET) {
            if (HAL_GetTick() - t0 > SLAVE_CDONE_MS) { rc = -2; break; }
        }
    }
    if (rc == 0 && sha256 && memcmp(got, sha256, 32) != 0 && !s_slave_skew) {
        /* The iCE40 took the same bits: a configured part from a bad read is not trusted. */
        LOG_W("[ice40] slave load: the image read back does not match the slot's sha256\n");
        rc = -3;
    }
    if (rc != 0) {
        LOG_W("[ice40] slave load from 0x%06lx failed (%s)\n", (unsigned long)addr,
              rc == -2 ? "CDONE stayed low" : rc == -3 ? "checksum" : "bus error");
        goto out_reset;
    }
    psram_bus_park();                   /* IO0 back to the OCTOSPI (PG0 still high) */
    {
        GPIO_InitTypeDef gz = {0};
        gz.Mode = GPIO_MODE_ANALOG;     /* flash /CS Hi-Z (pulled up), as after a master-mode boot */
        gz.Pin  = ICE_FLASH_CS_PIN;
        HAL_GPIO_Init(ICE_FLASH_CS_PORT, &gz);
    }
    psram_bus_release();
    psram_bus_handover();
    LOG_D("[ice40] slave load from 0x%06lx: CDONE high after %lu us (read start to CDONE)\n",
          (unsigned long)addr, (unsigned long)(time_us_64() - t0));
    return 0;

out_reset:
    /* Never leave a half-configured part on the bus: hold it in reset, deselect the flash. The
       caller reloads (another slave load, or the flash write path). */
    w25q_stream_end();
    psram_bus_park();
    ice40_hold_off_bus();
    psram_bus_release();
    psram_bus_handover();
    return rc;
}
