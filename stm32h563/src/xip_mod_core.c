/* xip_mod_core.c — runs from the W25Q (xip_image.h): the usb image's table and the self-test.
 *
 * The self-test proves a session end to end: the table is read from the mapped image, the code
 * runs from it, its constants are read from it, and it calls back into internal flash (a long
 * branch through a linker veneer). */
#include "xip_image.h"
#include "msc_flash.h"
#include "ctrl_flash.h"
#include "ser_flash.h"
#include "esp_rom_job.h"
#include "usb_check_core.h"

#include <stdint.h>

static const char k_probe[] = "BenchPod XIP self-test: code and constants from the W25Q";

/* Internal flash (the veneer test): FNV-1a over a byte. */
uint32_t xip_core_fnv_step(uint32_t h, uint8_t b);

static uint32_t selftest(uint32_t seed)
{
    uint32_t h = 2166136261u ^ seed;
    for (unsigned i = 0; i < sizeof(k_probe) - 1u; i++) h = xip_core_fnv_step(h, (uint8_t)k_probe[i]);
    return h;
}

__attribute__((section(".xip.table"), used))
const xip_table_t xip_table = {
    XIP_TABLE_MAGIC, XIP_ABI, XIP_FN_COUNT,
    { [XIP_FN_SELFTEST] = (const void *)selftest,
      [XIP_FN_MSC_FLASH_STEP] = (const void *)msc_flash_step,
      [XIP_FN_CTRL_FLASH_STEP] = (const void *)ctrl_flash_step,
      [XIP_FN_SER_FLASH_STEP] = (const void *)ser_flash_step,
      [XIP_FN_ESP_ROM_BEGIN] = (const void *)esp_rom_job_begin,
      [XIP_FN_ESP_ROM_STEP] = (const void *)esp_rom_job_step,
#if defined(BENCHPOD_USB_HOST)
      [XIP_FN_USB_CHECK_STEP] = (const void *)uc_job_step,
#endif
    },
};
