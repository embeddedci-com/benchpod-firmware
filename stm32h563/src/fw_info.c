/*
 * fw_info.c — read the fw_info block of an image (see fw_info.h).
 */
#include "fw_info.h"
#include <string.h>

uint32_t fw_info_required_kb(const uint8_t *at_offset, size_t n)
{
    fw_info_t info;
    if (at_offset == NULL || n < sizeof(info)) return 2048u;
    memcpy(&info, at_offset, sizeof(info));
    if (info.magic != FW_INFO_MAGIC || info.min_flash_kb == 0u) return 2048u;
    return info.min_flash_kb;
}

uint32_t fw_info_flags(const uint8_t *at_offset, size_t n)
{
    fw_info_t info;
    if (at_offset == NULL || n < sizeof(info)) return 0u;
    memcpy(&info, at_offset, sizeof(info));
    return info.magic == FW_INFO_MAGIC ? info.reserved[0] : 0u;
}
