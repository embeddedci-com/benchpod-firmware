/*
 * test_esp_part_table.c — host tests for src/esp_part_table.c: where wifi-clear erases on
 * the ESP32-C3.  A wrong answer here erases the wrong part of the C3's flash, so beyond
 * "finds the NVS partition" these pin down everything it must refuse:
 *   - the table of the esp-hosted slave image (nvs at 0x9000, 16 KB);
 *   - nvs found wherever it sits in the table, by type + subtype + label;
 *   - other data partitions, and NVS partitions with another label, are not it;
 *   - no table, a truncated image, or a table without nvs: not found;
 *   - an entry that is unaligned, empty, overlaps the bootloader / table, or runs past
 *     the flash: refused;
 *   - a reader that fails (the W25Q slot read) is "not found", never a guess;
 *   - the real prebuilt image, when it is present in the checkout.
 */
#include "esp_part_table.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

static int failures;
#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); failures++; } \
} while (0)

#define IMG_LEN 0x9000u
static uint8_t img[IMG_LEN];
static unsigned nent;

static void table_reset(void) { memset(img, 0xFF, sizeof(img)); nent = 0; }

static void put_entry(uint8_t type, uint8_t subtype, uint32_t off, uint32_t size, const char *label) {
    uint8_t *e = img + ESP_PART_TABLE_OFFSET + 32u * nent++;
    memset(e, 0, 32);
    e[0] = 0xAA; e[1] = 0x50; e[2] = type; e[3] = subtype;
    e[4] = (uint8_t)off;  e[5] = (uint8_t)(off >> 8);  e[6] = (uint8_t)(off >> 16);  e[7] = (uint8_t)(off >> 24);
    e[8] = (uint8_t)size; e[9] = (uint8_t)(size >> 8); e[10] = (uint8_t)(size >> 16); e[11] = (uint8_t)(size >> 24);
    strncpy((char *)e + 12, label, 16);
}
static void put_md5_entry(void) {
    uint8_t *e = img + ESP_PART_TABLE_OFFSET + 32u * nent++;
    memset(e, 0xFF, 32); e[0] = 0xEB; e[1] = 0xEB;
}

/* partitions.esp32c3.csv of the esp-hosted slave, as partition-table.bin encodes it. */
static void slave_table(void) {
    table_reset();
    put_entry(1, 0x02, 0x009000, 0x004000, "nvs");
    put_entry(1, 0x00, 0x00D000, 0x002000, "otadata");
    put_entry(1, 0x01, 0x00F000, 0x001000, "phy_init");
    put_entry(0, 0x10, 0x010000, 0x180000, "ota_0");
    put_entry(0, 0x11, 0x190000, 0x180000, "ota_1");
    put_md5_entry();
}

static bool find(uint32_t *off, uint32_t *size) {
    *off = 0xDEAD; *size = 0xDEAD;
    return esp_part_find_nvs(img, sizeof(img), off, size);
}

static void test_slave_table(void) {
    uint32_t off, size;
    slave_table();
    CHECK(find(&off, &size));
    CHECK(off == 0x9000 && size == 0x4000);
    CHECK(esp_part_find_nvs(img, sizeof(img), NULL, NULL));    /* outputs are optional */
}

static void test_nvs_not_first(void) {
    uint32_t off, size;
    table_reset();
    put_entry(1, 0x00, 0x009000, 0x002000, "otadata");
    put_entry(1, 0x01, 0x00B000, 0x001000, "phy_init");
    put_entry(0, 0x10, 0x010000, 0x100000, "ota_0");
    put_entry(1, 0x02, 0x110000, 0x006000, "nvs");
    put_md5_entry();
    CHECK(find(&off, &size));
    CHECK(off == 0x110000 && size == 0x6000);
}

static void test_only_the_default_nvs(void) {
    uint32_t off, size;
    table_reset();                                   /* no NVS partition at all */
    put_entry(1, 0x00, 0x009000, 0x002000, "otadata");
    put_entry(0, 0x10, 0x010000, 0x100000, "nvs");   /* an app that happens to be called nvs */
    put_md5_entry();
    CHECK(!find(&off, &size));

    table_reset();                                   /* NVS partitions under other names */
    put_entry(1, 0x02, 0x009000, 0x004000, "fctry");
    put_entry(1, 0x02, 0x00D000, 0x004000, "nvs_keys");
    put_entry(1, 0x04, 0x011000, 0x001000, "nvs");   /* subtype nvs_keys, not nvs */
    put_md5_entry();
    CHECK(!find(&off, &size));

    put_entry(1, 0x02, 0x012000, 0x004000, "nvs");   /* after the MD5 entry: not part of the table */
    CHECK(!find(&off, &size));
}

static void test_no_table(void) {
    uint32_t off, size;
    table_reset();                                   /* erased flash where the table should be */
    CHECK(!find(&off, &size));
    memset(img, 0, sizeof(img));
    CHECK(!find(&off, &size));

    slave_table();
    CHECK(!esp_part_find_nvs(NULL, sizeof(img), &off, &size));
    CHECK(!esp_part_find_nvs(img, 0, &off, &size));
    CHECK(!esp_part_find_nvs(img, ESP_PART_TABLE_OFFSET, &off, &size));        /* ends at the table */
    CHECK(!esp_part_find_nvs(img, ESP_PART_TABLE_OFFSET + 31, &off, &size));   /* mid-entry */
    CHECK(esp_part_find_nvs(img, ESP_PART_TABLE_OFFSET + 32, &off, &size));    /* the entry is whole */
}

static void one_nvs(uint32_t off, uint32_t size) {
    table_reset();
    put_entry(1, 0x02, off, size, "nvs");
    put_md5_entry();
}

static void test_refuses_bad_geometry(void) {
    uint32_t off, size;
    one_nvs(0x9000, 0x4000);  CHECK(find(&off, &size));
    one_nvs(0x9000, 0);       CHECK(!find(&off, &size));     /* empty */
    one_nvs(0x9100, 0x4000);  CHECK(!find(&off, &size));     /* offset not on a sector */
    one_nvs(0x9000, 0x4100);  CHECK(!find(&off, &size));     /* size not whole sectors */
    one_nvs(0x0000, 0x4000);  CHECK(!find(&off, &size));     /* the bootloader */
    one_nvs(0x8000, 0x4000);  CHECK(!find(&off, &size));     /* the partition table itself */
    one_nvs(0x3FC000, 0x4000); CHECK(find(&off, &size));     /* the last 16 KB of 4 MB */
    one_nvs(0x3FD000, 0x4000); CHECK(!find(&off, &size));    /* runs past the flash */
    one_nvs(0x9000, 0xFFFFF000u); CHECK(!find(&off, &size)); /* offset + size wraps */
    one_nvs(0xFFFFF000u, 0x1000); CHECK(!find(&off, &size));
}

/* The image that goes in the W25Q's esp slot (git-ignored: built or fetched, see `make fetch`). */
static void test_prebuilt_image(void) {
    const char *path = "../../esp32-hosted-slave/prebuilt/esp32c3-hosted-slave-merged.bin";
    FILE *f = fopen(path, "rb");
    if (!f) { printf("test_esp_part_table: no prebuilt slave image (make fetch), real-image case not run\n"); return; }
    static uint8_t real[0x10000];
    size_t n = fread(real, 1, sizeof(real), f);
    fclose(f);
    uint32_t off = 0, size = 0;
    CHECK(esp_part_find_nvs(real, n, &off, &size));
    CHECK(off == 0x9000 && size == 0x4000);
    /* The image ships a blank NVS: flash-esp32 has always left this region erased. */
    bool blank = n >= off + size;
    for (uint32_t i = 0; blank && i < size; i++) blank = (real[off + i] == 0xFF);
    CHECK(blank);
}

/* The firmware reads the table out of the W25Q slot one entry at a time. */
static int g_reads, g_fail_at;
static int counting_read(void *ctx, uint32_t off, uint8_t *buf, uint32_t n) {
    (void)ctx;
    if (++g_reads == g_fail_at) return -1;
    if (off + n > sizeof(img)) return -1;
    memcpy(buf, img + off, n);
    return 0;
}

static void test_reader(void) {
    uint32_t off, size;
    slave_table();
    g_reads = 0; g_fail_at = 0;
    CHECK(esp_part_find_nvs_rd(counting_read, NULL, &off, &size));
    CHECK(off == 0x9000 && size == 0x4000 && g_reads == 1);       /* nvs is the first entry */

    table_reset();
    put_entry(1, 0x00, 0x009000, 0x002000, "otadata");
    put_entry(1, 0x02, 0x00B000, 0x004000, "nvs");
    put_md5_entry();
    g_reads = 0; g_fail_at = 2;                                     /* the nvs entry's read fails */
    CHECK(!esp_part_find_nvs_rd(counting_read, NULL, &off, &size));
    g_reads = 0; g_fail_at = 0;
    CHECK(esp_part_find_nvs_rd(counting_read, NULL, &off, &size) && off == 0xB000);
    CHECK(!esp_part_find_nvs_rd(NULL, NULL, &off, &size));
}

int main(void) {
    test_slave_table();
    test_nvs_not_first();
    test_only_the_default_nvs();
    test_no_table();
    test_refuses_bad_geometry();
    test_reader();
    test_prebuilt_image();
    if (failures) { printf("FAIL test_esp_part_table: %d failure(s)\n", failures); return 1; }
    printf("PASS test_esp_part_table\n");
    return 0;
}
