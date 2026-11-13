/* boot_policy.c: see boot_policy.h. */
#include "boot_policy.h"
#include <stddef.h>
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

int boot_policy_gateware_image(uint8_t running, uint8_t embedded, int running_kind, int configured)
{
    if (!configured) return 0;
    if (running == 0 || embedded == 0 || running == embedded) return -1;
    return (running_kind == 1 || running_kind == 2) ? running_kind : 0;
}

/* CRC-32 (IEEE, reflected), bitwise: a 44-byte record, at boot and after a gateware write. */
static uint32_t ident_crc(const void *data, size_t n)
{
    const uint8_t *p = (const uint8_t *)data;
    uint32_t c = 0xFFFFFFFFu;
    while (n--) {
        c ^= *p++;
        for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
    }
    return ~c;
}

void gw_ident_make(gw_ident_t *r, uint16_t slot, uint32_t len, const uint8_t sha256[32])
{
    memset(r, 0, sizeof(*r));
    r->magic = GW_IDENT_MAGIC;
    r->version = GW_IDENT_VERSION;
    r->slot = slot;
    r->len = len;
    memcpy(r->sha256, sha256, sizeof(r->sha256));
    r->crc = ident_crc(r, offsetof(gw_ident_t, crc));
}

int gw_ident_valid(const gw_ident_t *r)
{
    return r && r->magic == GW_IDENT_MAGIC && r->version == GW_IDENT_VERSION &&
           r->crc == ident_crc(r, offsetof(gw_ident_t, crc));
}

gw_ident_match_t gw_ident_match(const gw_ident_t *rec, int slot_ok, uint16_t slot, uint32_t len,
                                const uint8_t sha256[32])
{
    if (!slot_ok) return GW_IDENT_NONE;
    if (!gw_ident_valid(rec)) return GW_IDENT_UNKNOWN;
    return (rec->slot == slot && rec->len == len && memcmp(rec->sha256, sha256, 32) == 0)
               ? GW_IDENT_SAME : GW_IDENT_DIFFERS;
}

gw_action_t boot_policy_gateware_action(uint8_t running, uint8_t embedded, int running_kind,
                                        int configured, gw_ident_match_t ident, int *img)
{
    int v = boot_policy_gateware_image(running, embedded, running_kind, configured);
    if (v >= 0) { *img = v; return GW_ACT_WRITE; }
    /* Same version (or unknown): only a configured iCE40 that answers, on a known build. */
    if (!configured || running == 0 || embedded == 0 || running != embedded) return GW_ACT_NONE;
    int k = (running_kind == 1 || running_kind == 2) ? running_kind : 0;
    switch (ident) {
    case GW_IDENT_DIFFERS: *img = k; return GW_ACT_WRITE;
    case GW_IDENT_UNKNOWN: *img = k; return GW_ACT_COMPARE;
    default:               return GW_ACT_NONE;
    }
}
