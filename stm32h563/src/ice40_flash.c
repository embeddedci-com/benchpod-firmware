/*
 * ice40_flash.c — reprogram the iCE40 config flash (W25Q64) from the STM32 over
 * the shared OCTOSPI bus (single-SPI, GPIO chip-select PE3).  Ported from the
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
#include <stdio.h>
#include <string.h>

#define CDONE_TIMEOUT_MS  500u

#define FCS_HIGH()  HAL_GPIO_WritePin(ICE_FLASH_CS_PORT, ICE_FLASH_CS_PIN, GPIO_PIN_SET)

/* PE3 (flash CS) + CRESET (PF13, drive low to hold iCE40 in reset) + CDONE
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
    printf("[ice40] held in reset (off the shared bus) until the next reflash\n");
}

static int mem_read(void *ctx, uint32_t off, uint8_t *buf, uint32_t n)
{
    memcpy(buf, (const uint8_t *)ctx + off, n);
    return 0;
}

int ice40_flash_program(const uint8_t *data, size_t len)
{
    if (!data) return -1;
    return ice40_flash_program_src(mem_read, (void *)data, len);
}

int ice40_flash_program_src(ice40_src_read_fn rd, void *ctx, size_t len)
{
    if (!rd || len == 0) return -1;
    bool erased = false;   /* from the first erase on, the flash holds no whole image */
    static uint8_t pg[W25Q_PAGE];   /* source page */
    static uint8_t rb[W25Q_PAGE];   /* read-back page */

    psram_bus_acquire();
    flash_pins();
    HAL_GPIO_WritePin(ICE_CRESET_PORT, ICE_CRESET_PIN, GPIO_PIN_RESET);  /* iCE40 in reset */
    sleep_ms(2);
    w25q_wake();
    sleep_ms(1);

    uint8_t id[3] = {0};
    if (w25q_read_id(id) != 0 || id[0] == 0x00 || id[0] == 0xFF) {
        printf("[ice40] flash not responding (ID %02x %02x %02x)\n", id[0], id[1], id[2]);
        goto fail;
    }
    printf("[ice40] flash ID %02x %02x %02x, programming %u bytes\n",
           id[0], id[1], id[2], (unsigned)len);

    /* Erase enough 4 KB sectors to hold the bitstream. */
    for (uint32_t a = 0; a < len; a += W25Q_SECTOR) {
        erased = true;
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
            printf("[ice40] verify FAILED @0x%06lx: wrote %02x read %02x\n",
                   (unsigned long)(a + k), pg[k], rb[k]);
            goto fail;
        }
    }
    printf("[ice40] verify OK (%u bytes)\n", (unsigned)len);

    /* Hand the config-flash bus FULLY to the iCE40 before releasing it from reset (previously
       CRESET was released while the STM32 still owned the bus, so the iCE40 came out of reset
       into a flash it couldn't reach and CDONE never asserted). */
    psram_bus_release();
    /* ...BUT hold the PSRAM /CS DRIVEN HIGH (not the Hi-Z psram_bus_release leaves) for the whole
       config read: the shared SCLK/SO/SI edges the iCE40 clocks out can capacitively drag a Hi-Z
       (pulled-up) /CS low, glitch-selecting the PSRAM so it drives IO0(=SO) and corrupts the
       bitstream — an intermittent "CDONE never rose / iCE40 mute after reflash".  A driven high
       resists it.  Cleaned up by the psram_init()/psram_bus_acquire() ice40_reflash_image() runs. */
    psram_cs_park_high();
    {
        GPIO_InitTypeDef gz = {0};
        gz.Mode = GPIO_MODE_ANALOG;          /* PE3 (flash /CS) -> Hi-Z (iCE40 drives it in config) */
        gz.Pin  = ICE_FLASH_CS_PIN;
        HAL_GPIO_Init(ICE_FLASH_CS_PORT, &gz);
    }
    HAL_GPIO_WritePin(ICE_CRESET_PORT, ICE_CRESET_PIN, GPIO_PIN_SET);  /* release iCE40 */
    {
        uint32_t t0 = HAL_GetTick();
        while (HAL_GPIO_ReadPin(ICE_CDONE_PORT, ICE_CDONE_PIN) == GPIO_PIN_RESET) {
            if (HAL_GetTick() - t0 > CDONE_TIMEOUT_MS) {
                printf("[ice40] CDONE never went high — configuration failed\n");
                ice40_hold_off_bus();
                return -1;
            }
        }
    }
    printf("[ice40] reconfigured OK (CDONE high)\n");
    return 0;

fail:
    if (erased) {
        ice40_hold_off_bus();   /* a partial image: never let it configure from that */
    } else {
        HAL_GPIO_WritePin(ICE_CRESET_PORT, ICE_CRESET_PIN, GPIO_PIN_SET);   /* old image intact */
    }
    psram_bus_release();
    return -1;
}

int ice40_is_configured(void) {
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOF_CLK_ENABLE();
    g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_NOPULL; g.Pin = ICE_CDONE_PIN;
    HAL_GPIO_Init(ICE_CDONE_PORT, &g);   /* out of reset it is analog, which reads 0 */
    return HAL_GPIO_ReadPin(ICE_CDONE_PORT, ICE_CDONE_PIN) == GPIO_PIN_SET;
}

