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
#include "psram.h"
#include "board_pins.h"
#include "pico_compat.h"
#include "stm32h5xx_hal.h"
#include <stdio.h>
#include <string.h>

#define FLASH_SECTOR  4096u
#define FLASH_PAGE    256u
#define WIP_TIMEOUT_MS    2000u
#define CDONE_TIMEOUT_MS  500u

#define FCS_LOW()   HAL_GPIO_WritePin(ICE_FLASH_CS_PORT, ICE_FLASH_CS_PIN, GPIO_PIN_RESET)
#define FCS_HIGH()  HAL_GPIO_WritePin(ICE_FLASH_CS_PORT, ICE_FLASH_CS_PIN, GPIO_PIN_SET)

/* Single-SPI command on the flash, framed by PE3.  Reuses the OCTOSPI handle
   psram_init() set up. */
static int fxfer(uint8_t instr, int has_addr, uint32_t addr,
                 uint8_t *buf, uint32_t len, int is_read)
{
    XSPI_HandleTypeDef *h = (XSPI_HandleTypeDef *)psram_xspi();
    XSPI_RegularCmdTypeDef c = {0};
    c.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    c.Instruction = instr;
    c.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
    c.InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS;
    c.AddressMode = has_addr ? HAL_XSPI_ADDRESS_1_LINE : HAL_XSPI_ADDRESS_NONE;
    c.Address = addr;
    c.AddressWidth = HAL_XSPI_ADDRESS_24_BITS;
    c.DataMode = len ? HAL_XSPI_DATA_1_LINE : HAL_XSPI_DATA_NONE;
    c.DataLength = len;
    c.DummyCycles = 0;

    FCS_LOW();
    HAL_StatusTypeDef s = HAL_XSPI_Command(h, &c, HAL_XSPI_TIMEOUT_DEFAULT_VALUE);
    if (s == HAL_OK && len) {
        s = is_read ? HAL_XSPI_Receive(h, buf, HAL_XSPI_TIMEOUT_DEFAULT_VALUE)
                    : HAL_XSPI_Transmit(h, buf, HAL_XSPI_TIMEOUT_DEFAULT_VALUE);
    }
    FCS_HIGH();
    return (s == HAL_OK) ? 0 : -1;
}

static int flash_wait_wip(void)
{
    uint32_t t0 = HAL_GetTick();
    for (;;) {
        uint8_t sr = 0xFF;
        if (fxfer(0x05, 0, 0, &sr, 1, 1) != 0) return -1;   /* RDSR */
        if ((sr & 0x01u) == 0) return 0;                    /* WIP clear */
        if (HAL_GetTick() - t0 > WIP_TIMEOUT_MS) return -1;
    }
}

/* PE3 (flash CS) + CRESET (PF13, drive low to hold iCE40 in reset) + CDONE
   (PF14, input).  PG0/PSRAM-CS/OCTOSPI pins are handled by psram_bus_acquire. */
static void flash_pins(void)
{
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_LOW;

    FCS_HIGH();
    g.Mode = GPIO_MODE_OUTPUT_PP; g.Pin = ICE_FLASH_CS_PIN;
    HAL_GPIO_Init(ICE_FLASH_CS_PORT, &g);
    FCS_HIGH();

    HAL_GPIO_WritePin(ICE_CRESET_PORT, ICE_CRESET_PIN, GPIO_PIN_RESET);  /* hold reset */
    g.Pin = ICE_CRESET_PIN; HAL_GPIO_Init(ICE_CRESET_PORT, &g);
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
    fxfer(0xAB, 0, 0, NULL, 0, 0);   /* release deep power-down */
    sleep_ms(1);
    int rc = fxfer(0x9F, 0, 0, id, 3, 1);
    /* leave the iCE40 in reset is bad — release it so it reconfigures */
    HAL_GPIO_WritePin(ICE_CRESET_PORT, ICE_CRESET_PIN, GPIO_PIN_SET);
    psram_bus_release();
    return rc;
}

int ice40_flash_program(const uint8_t *data, size_t len)
{
    if (!data || len == 0) return -1;

    psram_bus_acquire();
    flash_pins();
    HAL_GPIO_WritePin(ICE_CRESET_PORT, ICE_CRESET_PIN, GPIO_PIN_RESET);  /* iCE40 in reset */
    sleep_ms(2);
    fxfer(0xAB, 0, 0, NULL, 0, 0);   /* wake flash */
    sleep_ms(1);

    uint8_t id[3] = {0};
    if (fxfer(0x9F, 0, 0, id, 3, 1) != 0 || id[0] == 0x00 || id[0] == 0xFF) {
        printf("[ice40] flash not responding (ID %02x %02x %02x)\n", id[0], id[1], id[2]);
        goto fail;
    }
    printf("[ice40] flash ID %02x %02x %02x, programming %u bytes\n",
           id[0], id[1], id[2], (unsigned)len);

    /* Erase enough 4 KB sectors to hold the bitstream. */
    for (uint32_t a = 0; a < len; a += FLASH_SECTOR) {
        if (fxfer(0x06, 0, 0, NULL, 0, 0) != 0) goto fail;        /* WREN */
        if (fxfer(0x20, 1, a, NULL, 0, 0) != 0) goto fail;        /* sector erase */
        if (flash_wait_wip() != 0) goto fail;
    }
    /* Program in 256-byte pages. */
    for (uint32_t a = 0; a < len; a += FLASH_PAGE) {
        uint32_t n = (len - a < FLASH_PAGE) ? (uint32_t)(len - a) : FLASH_PAGE;
        if (fxfer(0x06, 0, 0, NULL, 0, 0) != 0) goto fail;        /* WREN */
        if (fxfer(0x02, 1, a, (uint8_t *)(data + a), n, 0) != 0) goto fail; /* page program */
        if (flash_wait_wip() != 0) goto fail;
    }

    /* Read-back verify: prove the flash content matches the bitstream, so a
       CDONE failure can be pinned on the config/handoff path rather than a bad
       write.  Still on the STM32-owned bus here (normal read, 0x03, no dummy). */
    {
        static uint8_t rb[FLASH_PAGE];
        for (uint32_t a = 0; a < len; a += FLASH_PAGE) {
            uint32_t n = (len - a < FLASH_PAGE) ? (uint32_t)(len - a) : FLASH_PAGE;
            if (fxfer(0x03, 1, a, rb, n, 1) != 0) goto fail;
            if (memcmp(rb, data + a, n) != 0) {
                uint32_t k = 0; while (k < n && rb[k] == data[a + k]) k++;
                printf("[ice40] verify FAILED @0x%06lx: wrote %02x read %02x\n",
                       (unsigned long)(a + k), data[a + k], rb[k]);
                goto fail;
            }
        }
        printf("[ice40] verify OK (%u bytes)\n", (unsigned)len);
    }

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
                return -1;
            }
        }
    }
    printf("[ice40] reconfigured OK (CDONE high)\n");
    return 0;

fail:
    HAL_GPIO_WritePin(ICE_CRESET_PORT, ICE_CRESET_PIN, GPIO_PIN_SET);
    psram_bus_release();
    return -1;
}

/* Prepare for an iCE40 SELF-reconfiguration (SB_WARMBOOT / OP_WARMBOOT): hand the config-
   flash bus to the iCE40 so it can re-read the selected image (same handoff as the
   flash-done path, but WITHOUT touching CRESET — SB_WARMBOOT reboots the fabric itself). */
void ice40_prepare_reconfig(void) {
    psram_bus_release();                 /* SCLK/IO -> Hi-Z (iCE40 drives them for the read) */
    psram_cs_park_high();                /* but HOLD PSRAM /CS high so the PSRAM stays
                                            deselected and doesn't answer the config-flash
                                            0x03 read on the SHARED SCK/SO/SI pins */
    GPIO_InitTypeDef gz = {0};
    gz.Mode = GPIO_MODE_ANALOG;          /* PE3 (flash /CS) -> Hi-Z so the iCE40 drives it */
    gz.Pin  = ICE_FLASH_CS_PIN;
    HAL_GPIO_Init(ICE_FLASH_CS_PORT, &gz);
}

/* Wait for CDONE to (re)assert after a warmboot.  0 = configured, -1 = timeout. */
int ice40_wait_cdone(uint32_t timeout_ms) {
    uint32_t t0 = HAL_GetTick();
    while (HAL_GPIO_ReadPin(ICE_CDONE_PORT, ICE_CDONE_PIN) == GPIO_PIN_RESET) {
        if (HAL_GetTick() - t0 > timeout_ms) return -1;
    }
    return 0;
}
