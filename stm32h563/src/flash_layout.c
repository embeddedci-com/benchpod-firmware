/*
 * flash_layout.c — read the internal flash size once (see flash_layout.h).
 */
#include "flash_layout.h"
#include "stm32h5xx.h"

uint32_t flash_layout_size(void)
{
    static uint32_t s_size;
    if (s_size == 0u) {
        /* Word read: the register is the low half-word of this aligned word. */
        uint32_t kb = *(volatile const uint32_t *)FLASHSIZE_BASE & 0xFFFFu;
        /* The H563 comes with 1 MB or 2 MB. Anything else (an erased 0xFFFF, a 0) is
           treated as the larger part, the layout every pod before the 1 MB variant had. */
        s_size = (kb == 1024u) ? FLASH_LAYOUT_SIZE_1MB : FLASH_LAYOUT_SIZE_2MB;
    }
    return s_size;
}
