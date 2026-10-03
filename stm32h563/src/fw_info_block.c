/*
 * fw_info_block.c — this image's own fw_info block, placed at FW_INFO_OFFSET by the linker.
 */
#include "fw_info.h"

__attribute__((section(".fw_info"), used))
const fw_info_t g_fw_info = {
    .magic        = FW_INFO_MAGIC,
    .layout       = FW_INFO_LAYOUT,
    .min_flash_kb = FW_INFO_MIN_FLASH_KB,
    .reserved     = { 0u, 0u },
};
