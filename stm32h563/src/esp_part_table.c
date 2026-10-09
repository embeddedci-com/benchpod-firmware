/*
 * esp_part_table.c — find the NVS partition in a merged ESP32 flash image.
 * See esp_part_table.h.
 *
 * Table layout (esp_partition_info_t, 32 bytes per entry, little-endian), up to
 * 95 entries from ESP_PART_TABLE_OFFSET:
 *   u16 magic (0x50AA)  u8 type  u8 subtype  u32 offset  u32 size  char label[16]  u32 flags
 * The entries end at an MD5 entry (magic 0xEBEB) or erased flash (0xFFFF).
 */
#include "esp_part_table.h"

#include <string.h>

#define PART_ENTRY_SIZE     32u
#define PART_MAX_ENTRIES    95u
#define PART_MAGIC          0x50AAu
#define PART_TYPE_DATA      0x01u
#define PART_SUBTYPE_NVS    0x02u
#define PART_TABLE_SIZE     0x1000u
#define ESP_FLASH_SIZE_MAX  0x400000u    /* ESP32-C3-MINI-1: 4 MB */

static uint32_t le32(const uint8_t *p)
{ return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }

bool esp_part_find_nvs_rd(esp_part_read_fn rd, void *ctx, uint32_t *offset, uint32_t *size)
{
    if (!rd) return false;
    for (uint32_t i = 0; i < PART_MAX_ENTRIES; i++) {
        uint8_t e[PART_ENTRY_SIZE];
        if (rd(ctx, ESP_PART_TABLE_OFFSET + i * PART_ENTRY_SIZE, e, PART_ENTRY_SIZE) != 0)
            return false;                                    /* the image ends mid-table */
        if (((uint32_t)e[0] | ((uint32_t)e[1] << 8)) != PART_MAGIC) return false;   /* end of table */
        if (e[2] != PART_TYPE_DATA || e[3] != PART_SUBTYPE_NVS) continue;
        if (memcmp(e + 12, "nvs", 4) != 0) continue;        /* the default NVS partition, NUL included */

        uint32_t off = le32(e + 4), sz = le32(e + 8);
        if (sz == 0 || (off % ESP_FLASH_SECTOR_SIZE) || (sz % ESP_FLASH_SECTOR_SIZE)) return false;
        if (off < ESP_PART_TABLE_OFFSET + PART_TABLE_SIZE) return false;   /* bootloader / table */
        if (off > ESP_FLASH_SIZE_MAX || sz > ESP_FLASH_SIZE_MAX - off) return false;
        if (offset) *offset = off;
        if (size)   *size   = sz;
        return true;
    }
    return false;
}

typedef struct { const uint8_t *img; size_t len; } mem_src_t;

static int mem_read(void *ctx, uint32_t off, uint8_t *buf, uint32_t n)
{
    const mem_src_t *m = ctx;
    if (off > m->len || n > m->len - off) return -1;
    memcpy(buf, m->img + off, n);
    return 0;
}

bool esp_part_find_nvs(const uint8_t *img, size_t len, uint32_t *offset, uint32_t *size)
{
    if (!img) return false;
    mem_src_t m = { img, len };
    return esp_part_find_nvs_rd(mem_read, &m, offset, size);
}
