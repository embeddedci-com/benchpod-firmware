/* xip_session.c — see xip_session.h. Internal flash. */
#include "xip_session.h"
#include "xip_image.h"
#include "xspi_guard.h"
#include "blob_store.h"
#include "w25q.h"
#include "psram.h"
#include "board_pins.h"
#include "bp_log.h"

#include "mbedtls/sha256.h"
#include "stm32h5xx_hal.h"

#include <string.h>

#define XIP_SLOT_DATA_ADDR   (BLOB_HDR_SIZE)   /* image offset inside the slot */
#define XIP_W25Q_READ        0x0Bu             /* fast read, 1-1-1, 8 dummy clocks */
#define XIP_W25Q_DUMMY       8u

static bool     s_active;
#if !defined(BENCHPOD_XIP_RAM)
static bool     s_mapped;
#endif
static uint32_t s_begin_us;

xip_mode_t xip_mode(void)
{
#if defined(BENCHPOD_XIP_RAM)
    return XIP_MODE_RAM;
#else
    return xip_supported() ? XIP_MODE_MAPPED : XIP_MODE_NONE;
#endif
}

const char *xip_mode_name(xip_mode_t m)
{
    return m == XIP_MODE_MAPPED ? "mapped" : m == XIP_MODE_RAM ? "ram" : "none";
}

const char *xip_err_text(xip_err_t e)
{
    switch (e) {
    case XIP_OK:              return "ok";
    case XIP_ERR_UNSUPPORTED: return "this board cannot run code from its W25Q";
    case XIP_ERR_NO_IMAGE:    return "this firmware was built without a W25Q image";
    case XIP_ERR_SLOT:        return "the xip blob slot does not hold this firmware's image";
    case XIP_ERR_BUS:         return "the W25Q could not be opened";
    case XIP_ERR_MAP:         return "the W25Q could not be memory-mapped";
    case XIP_ERR_HASH:        return "the W25Q image does not match its SHA-256";
    case XIP_ERR_TABLE:       return "the W25Q image has a bad table";
    case XIP_ERR_ACTIVE:      return "a W25Q code session is already running";
    }
    return "?";
}

bool     xip_session_active(void)        { return s_active; }
uint32_t xip_session_last_begin_us(void) { return s_begin_us; }

static uint32_t now_us(void)
{
    /* The DWT cycle counter (enabled at boot, stm32h5xx_hal_timebase_tim.c): wraps after ~17 s at
       250 MHz, plenty for a begin. */
    return DWT->CYCCNT / (SystemCoreClock / 1000000u);
}

/* ---- memory-mapped mode (the guard's hooks) ------------------------------------------------ */
#if !defined(BENCHPOD_XIP_RAM)

static void flash_cs_gpio_high(void)
{
    GPIO_InitTypeDef g = {0};
    HAL_GPIO_WritePin(ICE_FLASH_CS_PORT, ICE_FLASH_CS_PIN, GPIO_PIN_SET);
    g.Pin = ICE_FLASH_CS_PIN; g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(ICE_FLASH_CS_PORT, &g);
}

static int map_w25q(void)
{
    XSPI_HandleTypeDef *h = (XSPI_HandleTypeDef *)psram_xspi();
    /* The PSRAM stays deselected on its GPIO chip-select; the W25Q's is the controller's NCS. */
    HAL_GPIO_WritePin(PSRAM_CS_PORT, PSRAM_CS_PIN, GPIO_PIN_SET);
    GPIO_InitTypeDef g = {0};
    g.Pin = ICE_FLASH_CS_PIN; g.Mode = GPIO_MODE_AF_PP; g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH; g.Alternate = GPIO_AF10_OCTOSPI1;   /* PB6: OCTOSPI1_NCS */
    HAL_GPIO_Init(ICE_FLASH_CS_PORT, &g);

    XSPI_RegularCmdTypeDef c = {0};
    c.OperationType      = HAL_XSPI_OPTYPE_READ_CFG;
    c.Instruction        = XIP_W25Q_READ;
    c.InstructionMode    = HAL_XSPI_INSTRUCTION_1_LINE;
    c.InstructionWidth   = HAL_XSPI_INSTRUCTION_8_BITS;
    c.AddressMode        = HAL_XSPI_ADDRESS_1_LINE;
    c.AddressWidth       = HAL_XSPI_ADDRESS_24_BITS;
    c.DataMode           = HAL_XSPI_DATA_1_LINE;
    c.DummyCycles        = XIP_W25Q_DUMMY;
    c.DQSMode            = HAL_XSPI_DQS_DISABLE;
    c.SIOOMode           = HAL_XSPI_SIOO_INST_EVERY_CMD;
    if (HAL_XSPI_Command(h, &c, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return -1;
    XSPI_MemoryMappedTypeDef mm = {0};
    mm.TimeOutActivation  = HAL_XSPI_TIMEOUT_COUNTER_ENABLE;   /* release NCS when the CPU idles */
    mm.TimeoutPeriodClock = 0x20u;
    if (HAL_XSPI_MemoryMapped(h, &mm) != HAL_OK) return -1;
    return 0;
}

static int unmap_w25q(void)
{
    XSPI_HandleTypeDef *h = (XSPI_HandleTypeDef *)psram_xspi();
    int rc = HAL_XSPI_Abort(h) == HAL_OK ? 0 : -1;
    flash_cs_gpio_high();   /* indirect commands frame the W25Q with the GPIO again (w25q.c) */
    return rc;
}

static const xspi_guard_hooks_t k_hooks = { unmap_w25q, map_w25q };
#endif /* !BENCHPOD_XIP_RAM */

/* ---- the session --------------------------------------------------------------------------- */

#if defined(BENCHPOD_XIP_RAM)
/* Copy the slot's image to where it was linked (a RAM region the linker reserved). */
static xip_err_t load_into_ram(uint32_t len)
{
    uint8_t *dst = (uint8_t *)(uintptr_t)__xip_start;
    if ((uint32_t)(__xip_region_end - __xip_start) < len) return XIP_ERR_SLOT;
    for (uint32_t off = 0; off < len; off += 256u) {
        uint32_t n = len - off < 256u ? len - off : 256u;
        if (blob_store_read((void *)(uintptr_t)BLOB_XIP, off, dst + off, n) != 0) return XIP_ERR_BUS;
    }
    return XIP_OK;
}
#endif

static xip_err_t check_image(const uint8_t *img, uint32_t len, const xip_hash_t *want)
{
    uint8_t sha[32];
    if (mbedtls_sha256(img, len, sha, 0) != 0 || memcmp(sha, want->sha256, 32) != 0) return XIP_ERR_HASH;
    const xip_table_t *t = (const xip_table_t *)(uintptr_t)&xip_table;
    if (t->magic != XIP_TABLE_MAGIC || t->abi != XIP_ABI || t->count != XIP_FN_COUNT) return XIP_ERR_TABLE;
    return XIP_OK;
}

xip_err_t xip_session_begin(void)
{
    const uint32_t t0 = now_us();
    if (s_active) return XIP_ERR_ACTIVE;
    const xip_mode_t mode = xip_mode();
    if (mode == XIP_MODE_NONE || !blob_listed(BLOB_XIP)) return XIP_ERR_UNSUPPORTED;
    const xip_hash_t *want = xip_image_expected();
    if (want->magic != XIP_HASH_MAGIC || want->len == 0) return XIP_ERR_NO_IMAGE;
    const blob_info_t *have = blob_store_info(BLOB_XIP);
    if (!have || !have->present || have->len != want->len || memcmp(have->sha256, want->sha256, 32) != 0)
        return XIP_ERR_SLOT;

    if (w25q_session_open() != 0) { w25q_session_close(); return XIP_ERR_BUS; }
    s_active = true;
    xip_err_t rc = XIP_OK;
#if defined(BENCHPOD_XIP_RAM)
    rc = load_into_ram(want->len);
    w25q_session_close();   /* the copy runs without the bus; the session keeps others off it */
#else
    if (map_w25q() != 0) {
        (void)unmap_w25q();
        rc = XIP_ERR_MAP;
    } else {
        s_mapped = true;
        xspi_guard_set_mapped(&k_hooks);
    }
#endif
    if (rc == XIP_OK) rc = check_image((const uint8_t *)(uintptr_t)__xip_start, want->len, want);
    if (rc != XIP_OK) {
        LOG_W("[xip] session refused: %s\n", xip_err_text(rc));
        xip_session_end();
        return rc;
    }
    s_begin_us = now_us() - t0;
    return XIP_OK;
}

uint32_t xip_session_call_u32(unsigned fn, uint32_t arg)
{
    const xip_table_t *t = (const xip_table_t *)(uintptr_t)&xip_table;
    if (!s_active || fn >= t->count) return 0;
    return ((xip_fn_u32_t)t->fn[fn])(arg);
}

int xip_session_call_ctx(unsigned fn, void *ctx)
{
    const xip_table_t *t = (const xip_table_t *)(uintptr_t)&xip_table;
    if (!s_active || fn >= t->count) return -1;
    return ((xip_fn_ctx_t)t->fn[fn])(ctx);
}

void xip_session_end(void)
{
    if (!s_active) return;
#if !defined(BENCHPOD_XIP_RAM)
    if (s_mapped) {
        xspi_guard_set_mapped(NULL);
        (void)unmap_w25q();
        s_mapped = false;
    }
    w25q_session_close();
#endif
    s_active = false;
}
