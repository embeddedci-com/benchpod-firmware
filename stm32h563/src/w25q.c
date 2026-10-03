/*
 * w25q.c — single-SPI access to the iCE40 config flash (see w25q.h).
 *
 * Commands go out on the PSRAM's OCTOSPI1 handle (psram.c set it up), one line wide, framed by
 * PE3 as a GPIO chip-select. Moved here from ice40_flash.c, which now uses these calls.
 */
#include "w25q.h"
#include "psram.h"
#include "board_pins.h"
#include "pico_compat.h"
#include "stm32h5xx_hal.h"

#define WIP_TIMEOUT_MS      2000u     /* page program, 4 KB sector erase */
#define WIP_BLOCK_TIMEOUT_MS 3000u    /* 64 KB block erase (2 s max on the W25Q64JV) */

#define FCS_LOW()   HAL_GPIO_WritePin(ICE_FLASH_CS_PORT, ICE_FLASH_CS_PIN, GPIO_PIN_RESET)
#define FCS_HIGH()  HAL_GPIO_WritePin(ICE_FLASH_CS_PORT, ICE_FLASH_CS_PIN, GPIO_PIN_SET)

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

static int wait_wip(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    for (;;) {
        uint8_t sr = 0xFF;
        if (fxfer(0x05, 0, 0, &sr, 1, 1) != 0) return -1;   /* RDSR */
        if ((sr & 0x01u) == 0) return 0;                    /* WIP clear */
        if (HAL_GetTick() - t0 > timeout_ms) return -1;
    }
}

void w25q_cs_output(void)
{
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOE_CLK_ENABLE();
    g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_LOW;
    FCS_HIGH();
    g.Mode = GPIO_MODE_OUTPUT_PP; g.Pin = ICE_FLASH_CS_PIN;
    HAL_GPIO_Init(ICE_FLASH_CS_PORT, &g);
    FCS_HIGH();
}

void w25q_wake(void)
{
    fxfer(0xAB, 0, 0, NULL, 0, 0);   /* release deep power-down */
    sleep_us(30);                    /* tRES1 = 3 us */
}

int w25q_open(void)
{
    psram_bus_acquire();             /* PG0 high: the gateware tristates the bus; PSRAM CS high */
    w25q_cs_output();
    w25q_wake();
    return 0;
}

void w25q_close(void)
{
    FCS_HIGH();
    GPIO_InitTypeDef gz = {0};
    gz.Mode = GPIO_MODE_ANALOG;      /* back to Hi-Z, as after configuration (pulled up on board) */
    gz.Pin  = ICE_FLASH_CS_PIN;
    HAL_GPIO_Init(ICE_FLASH_CS_PORT, &gz);
    psram_bus_release();
}

int w25q_read_id(uint8_t id[3])
{
    return fxfer(0x9F, 0, 0, id, 3, 1);
}

uint32_t w25q_capacity(const uint8_t id[3])
{
    if (id[0] != 0xEF) return 0;                       /* Winbond */
    if (id[2] == 0x17) return 8u * 1024u * 1024u;      /* W25Q64 */
    if (id[2] == 0x18) return 16u * 1024u * 1024u;     /* W25Q128 */
    return 0;
}

int w25q_read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    if (len == 0) return 0;
    return fxfer(0x03, 1, addr, buf, len, 1);           /* normal read, no dummy */
}

static int erase(uint8_t op, uint32_t addr, uint32_t timeout_ms)
{
    if (fxfer(0x06, 0, 0, NULL, 0, 0) != 0) return -1;  /* WREN */
    if (fxfer(op, 1, addr, NULL, 0, 0) != 0) return -1;
    return wait_wip(timeout_ms);
}

int w25q_erase_sector(uint32_t addr) { return erase(0x20, addr, WIP_TIMEOUT_MS); }
int w25q_erase_block(uint32_t addr)  { return erase(0xD8, addr, WIP_BLOCK_TIMEOUT_MS); }

int w25q_program(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    while (len) {
        uint32_t room = W25Q_PAGE - (addr % W25Q_PAGE);
        uint32_t n = len < room ? len : room;
        if (fxfer(0x06, 0, 0, NULL, 0, 0) != 0) return -1;                 /* WREN */
        if (fxfer(0x02, 1, addr, (uint8_t *)buf, n, 0) != 0) return -1;     /* page program */
        if (wait_wip(WIP_TIMEOUT_MS) != 0) return -1;
        addr += n; buf += n; len -= n;
    }
    return 0;
}
