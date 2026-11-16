/*
 * c3_image.c — which esp-hosted image the ESP32-C3 runs (see c3_image.h).
 */
#include "c3_image.h"
#include "w25q.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    uint32_t magic;          /* C3_IMAGE_REC_MAGIC */
    uint16_t rec_version;    /* C3_IMAGE_REC_VER */
    uint16_t flags;          /* C3_REC_NO_IMAGE */
    uint32_t len;
    uint32_t version;
    uint8_t  sha256[32];
    uint32_t nvs_off;
    uint32_t nvs_size;       /* 0 = not known */
    uint32_t crc;            /* over everything above */
} c3_rec_t;

#define C3_REC_NO_IMAGE 0x0001u   /* the C3 holds no usable image */

static c3_rec_t    s_rec;
static bool        s_rec_valid;
static bool        s_missing;
static blob_info_t s_info;

/* CRC-32 (IEEE, reflected), bitwise: one 64-byte record, not worth a table. */
static uint32_t crc32_of(const void *data, size_t n)
{
    const uint8_t *p = data;
    uint32_t c = 0xFFFFFFFFu;
    while (n--) {
        c ^= *p++;
        for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
    }
    return ~c;
}

static bool rec_ok(const c3_rec_t *r)
{
    return r->magic == C3_IMAGE_REC_MAGIC && r->rec_version == C3_IMAGE_REC_VER &&
           r->crc == crc32_of(r, offsetof(c3_rec_t, crc));
}

void c3_image_load(void)
{
    c3_rec_t r;
    s_rec_valid = w25q_read(C3_IMAGE_REC_ADDR, (uint8_t *)&r, sizeof(r)) == 0 && rec_ok(&r);
    if (s_rec_valid) s_rec = r;
}

const blob_info_t *c3_image_info(void)
{
    memset(&s_info, 0, sizeof(s_info));
    if (s_missing) return &s_info;
    if (s_rec_valid) {
        if (!(s_rec.flags & C3_REC_NO_IMAGE)) {
            s_info.present = true;
            s_info.len     = s_rec.len;
            s_info.version = s_rec.version;
            memcpy(s_info.sha256, s_rec.sha256, 32);
        }
        return &s_info;
    }
    return blob_store_info(BLOB_ESP);   /* older firmware flashed the C3 from this copy */
}

bool c3_image_nvs(uint32_t *off, uint32_t *size)
{
    if (!s_rec_valid || (s_rec.flags & C3_REC_NO_IMAGE) || s_rec.nvs_size == 0) return false;
    if (off)  *off  = s_rec.nvs_off;
    if (size) *size = s_rec.nvs_size;
    return true;
}

static int write_rec(const c3_rec_t *r)
{
    c3_rec_t w = *r;
    w.crc = crc32_of(&w, offsetof(c3_rec_t, crc));
    if (w25q_erase_sector(C3_IMAGE_REC_ADDR) != 0 ||
        w25q_program(C3_IMAGE_REC_ADDR, (const uint8_t *)&w, sizeof(w)) != 0) {
        s_rec_valid = false;   /* half written reads as no record: back to the legacy view */
        printf("[c3] could not record the ESP32-C3's image\n");
        return -1;
    }
    s_rec = w;
    s_rec_valid = true;
    return 0;
}

int c3_image_save(uint32_t len, uint32_t version, const uint8_t sha256[32],
                  uint32_t nvs_off, uint32_t nvs_size)
{
    c3_rec_t r;
    memset(&r, 0, sizeof(r));
    r.magic = C3_IMAGE_REC_MAGIC;
    r.rec_version = C3_IMAGE_REC_VER;
    r.len = len;
    r.version = version;
    memcpy(r.sha256, sha256, 32);
    r.nvs_off = nvs_off;
    r.nvs_size = nvs_size;
    s_missing = false;
    return write_rec(&r);
}

int c3_image_forget(void)
{
    c3_rec_t r;
    memset(&r, 0, sizeof(r));
    r.magic = C3_IMAGE_REC_MAGIC;
    r.rec_version = C3_IMAGE_REC_VER;
    r.flags = C3_REC_NO_IMAGE;
    return write_rec(&r);
}

void c3_image_set_missing(bool missing) { s_missing = missing; }

const blob_info_t *blob_report_info(blob_id_t id)
{
    return id == BLOB_ESP ? c3_image_info() : blob_store_info(id);
}

blob_state_t blob_report_state(blob_id_t id)
{
    if (id != BLOB_ESP) return blob_state(id);
    return blob_state_of(c3_image_info(), blob_manifest(id));
}
