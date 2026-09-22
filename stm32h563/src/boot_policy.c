/* boot_policy.c: see boot_policy.h. */
#include "boot_policy.h"
#include <string.h>

uint32_t boot_policy_culprit(uint32_t started, uint32_t done, const char *crash_task) {
    if (crash_task && strcmp(crash_task, "net") == 0) return BOOT_SUB_NET;
    if (crash_task && strcmp(crash_task, "hw") == 0)  return BOOT_SUB_HW;
    return started & ~done & BOOT_SUB_ALL;
}

uint32_t boot_policy_off(uint32_t prev_off, uint32_t culprit) {
    prev_off &= BOOT_SUB_ALL;
    if (culprit) return prev_off | culprit;
    return prev_off ? prev_off : BOOT_SUB_ALL;
}

const char *boot_policy_off_str(uint32_t off) {
    switch (off & BOOT_SUB_ALL) {
        case BOOT_SUB_NET: return "network";
        case BOOT_SUB_HW:  return "iCE40/PSRAM";
        default:           return "network and iCE40/PSRAM";
    }
}

int boot_policy_gateware_image(uint8_t running, uint8_t embedded, int running_is_deep)
{
    if (running == 0 || embedded == 0 || running == embedded) return -1;
    return running_is_deep ? 1 : 0;
}
