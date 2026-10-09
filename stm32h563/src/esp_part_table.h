#ifndef ESP_PART_TABLE_H
#define ESP_PART_TABLE_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* ---- ESP-IDF partition table lookup -----------------------------------------
 *
 * Finds the NVS data partition in a merged ESP32 flash image (the esp-hosted slave
 * image in the W25Q slot), so the pod can erase exactly that region of the C3's
 * flash (wifi-clear) without hard-coding an offset that a slave rebuild with a
 * different partitions.csv would silently turn into "erase otadata".
 *
 * Pure: no hardware, host-tested (test/test_esp_part_table.c).
 * ---------------------------------------------------------------------------*/

#define ESP_PART_TABLE_OFFSET  0x8000u   /* CONFIG_PARTITION_TABLE_OFFSET (IDF default) */
#define ESP_FLASH_SECTOR_SIZE  0x1000u

/* Reads `n` bytes at image offset `off` into `buf`; 0 = ok, nonzero = out of range or failed.
 * Same shape as esp_src_read_fn and blob_store_read, so a W25Q slot can be read directly. */
typedef int (*esp_part_read_fn)(void *ctx, uint32_t off, uint8_t *buf, uint32_t n);

/* Look up the first partition of type data (1), subtype nvs (2) named "nvs" in the
 * partition table of the merged image `img` (an image of the flash from offset 0).
 * Returns true and fills *offset / *size (both multiples of the 4 KB flash sector,
 * lying after the table and inside a 4 MB flash) when found; false if the image holds
 * no table, no such partition, or one with a geometry that must not be erased. */
bool esp_part_find_nvs(const uint8_t *img, size_t len, uint32_t *offset, uint32_t *size);

/* The same lookup on an image read through `rd` (one 32-byte entry at a time). */
bool esp_part_find_nvs_rd(esp_part_read_fn rd, void *ctx, uint32_t *offset, uint32_t *size);

#endif /* ESP_PART_TABLE_H */
