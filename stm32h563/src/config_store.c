/*
 * config_store.c — power-loss-safe A/B record store (see config_store.h for the
 * layout and the reasoning) and the Wi-Fi config built on it.
 *
 * The store keeps no RAM state: every call re-reads flash, so it is correct after
 * any reset. Flash is only ever read through flash_read_checked().
 */
#include "config_store.h"
#include "flash_compat.h"

#include "hardware/flash.h"
#include "hardware/sync.h"

#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#define AB_HDR_SIZE   16u
#define AB_QW         16u
#define AB_CHUNK      32u

_Static_assert(AB_HDR_SIZE + AB_STORE_MAX_PAYLOAD <= FLASH_PAGE_SIZE,
               "an A/B record must fit in one 256 B program buffer");
_Static_assert(sizeof(config_t) <= AB_STORE_MAX_PAYLOAD,
               "config_t must fit in an A/B record");

typedef struct {
    uint32_t magic;
    uint32_t seq;
    uint32_t ver_len;   /* version << 16 | payload length */
    uint32_t crc;
} ab_hdr_t;
_Static_assert(sizeof(ab_hdr_t) == AB_HDR_SIZE, "header is one quad-word");

uint32_t ab_crc32(uint32_t crc, const void *data, size_t n)
{
    const uint8_t *p = (const uint8_t *)data;
    crc = ~crc;
    while (n--) {
        crc ^= *p++;
        for (int k = 0; k < 8; k++)
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return ~crc;
}

static uint32_t hdr_len(const ab_hdr_t *h) { return h->ver_len & 0xFFFFu; }
static uint32_t hdr_ver(const ab_hdr_t *h) { return h->ver_len >> 16; }

/* Is slot i a complete record? Fills *h. When out != NULL, also copies the
   payload (at most cap bytes). Reads in small chunks to keep the stack small. */
static bool slot_read(const ab_store_t *s, int i, ab_hdr_t *h, void *out, size_t cap)
{
    uint32_t off = s->slot_off[i];
    if (flash_read_checked(off, h, sizeof(*h)) != 0) {
        printf("[%s] slot %c: ECC error in the header (torn write), ignored\n", s->tag, 'A' + i);
        return false;
    }
    if (h->magic != s->magic) return false;          /* blank or foreign */
    uint32_t len = hdr_len(h);
    if (len > AB_STORE_MAX_PAYLOAD) return false;

    uint32_t crc = ab_crc32(0, &h->seq, sizeof(h->seq));
    crc = ab_crc32(crc, &h->ver_len, sizeof(h->ver_len));
    uint8_t buf[AB_CHUNK];
    for (uint32_t pos = 0; pos < len; ) {
        uint32_t n = len - pos < AB_CHUNK ? len - pos : AB_CHUNK;
        if (flash_read_checked(off + AB_HDR_SIZE + pos, buf, n) != 0) {
            printf("[%s] slot %c: ECC error in the payload, ignored\n", s->tag, 'A' + i);
            return false;
        }
        crc = ab_crc32(crc, buf, n);
        if (out && pos < cap)
            memcpy((uint8_t *)out + pos, buf, (cap - pos < n) ? cap - pos : n);
        pos += n;
    }
    if (crc != h->crc) {
        printf("[%s] slot %c: CRC mismatch, ignored\n", s->tag, 'A' + i);
        return false;
    }
    return true;
}

/* Index of the valid slot with the highest seq, or -1. */
static int newest_slot(const bool v[2], const ab_hdr_t h[2])
{
    if (v[0] && v[1]) return ((int32_t)(h[1].seq - h[0].seq) > 0) ? 1 : 0;
    if (v[0]) return 0;
    if (v[1]) return 1;
    return -1;
}

static int scan(const ab_store_t *s, bool v[2], ab_hdr_t h[2])
{
    for (int i = 0; i < 2; i++) v[i] = slot_read(s, i, &h[i], NULL, 0);
    return newest_slot(v, h);
}

/* Does slot i's payload equal data? (The slot is already known valid.) */
static bool slot_payload_equals(const ab_store_t *s, int i, const void *data, size_t len)
{
    uint8_t buf[AB_CHUNK];
    for (size_t pos = 0; pos < len; ) {
        size_t n = len - pos < AB_CHUNK ? len - pos : AB_CHUNK;
        if (flash_read_checked(s->slot_off[i] + AB_HDR_SIZE + (uint32_t)pos, buf, n) != 0) return false;
        if (memcmp(buf, (const uint8_t *)data + pos, n) != 0) return false;
        pos += n;
    }
    return true;
}

/* Erase slot `target` and write one record into it: payload first, header last. */
static int write_record(const ab_store_t *s, int target, uint32_t seq, const void *data, size_t len)
{
    uint8_t rec[AB_HDR_SIZE + AB_STORE_MAX_PAYLOAD] __attribute__((aligned(16)));
    ab_hdr_t h;
    h.magic   = s->magic;
    h.seq     = seq;
    h.ver_len = ((uint32_t)s->version << 16) | (uint32_t)len;
    h.crc     = ab_crc32(ab_crc32(ab_crc32(0, &h.seq, 4), &h.ver_len, 4), data, len);

    size_t body = (len + AB_QW - 1u) & ~(size_t)(AB_QW - 1u);
    memset(rec, 0xFF, sizeof(rec));
    memcpy(rec, &h, sizeof(h));
    if (len) memcpy(rec + AB_HDR_SIZE, data, len);

    uint32_t off = s->slot_off[target];
    printf("[%s] writing slot %c (0x%06lx) seq %lu, %u B\n", s->tag, 'A' + target,
           (unsigned long)off, (unsigned long)seq, (unsigned)len);

    /* Interrupts off, as before: bank-2 erase/program while bank 1 keeps running. */
    uint32_t irqs = save_and_disable_interrupts();
    flash_range_erase(off, FLASH_SECTOR_SIZE);
    if (body) flash_range_program(off + AB_HDR_SIZE, rec + AB_HDR_SIZE, body);
    flash_range_program(off, rec, AB_HDR_SIZE);         /* the commit point */
    restore_interrupts(irqs);

    ab_hdr_t rh;
    if (!slot_read(s, target, &rh, NULL, 0) || rh.seq != seq || rh.crc != h.crc ||
        !slot_payload_equals(s, target, data, len)) {
        printf("[%s] ERROR: write verification failed\n", s->tag);
        return -1;
    }
    return 0;
}

int ab_store_load(const ab_store_t *s, void *out, size_t len)
{
    bool v[2]; ab_hdr_t h[2];
    int n = scan(s, v, h);
    if (n >= 0) {
        if (hdr_len(&h[n]) == 0) return -1;                  /* tombstone: cleared */
        if (hdr_ver(&h[n]) != s->version || hdr_len(&h[n]) != len) {
            printf("[%s] record schema mismatch (version %lu len %lu, expected %u/%u), ignoring\n",
                   s->tag, (unsigned long)hdr_ver(&h[n]), (unsigned long)hdr_len(&h[n]),
                   (unsigned)s->version, (unsigned)len);
            return -1;
        }
        /* Re-read into out, re-checking the CRC (the flash could differ from the scan). */
        if (!slot_read(s, n, &h[n], out, len)) return -1;
        return 0;
    }

    /* No record in either slot: a pod still on the legacy single-sector layout. */
    if (s->legacy_off == 0) return -1;
    uint32_t magic;
    if (flash_read_checked(s->legacy_off, &magic, sizeof(magic)) != 0) {
        printf("[%s] legacy sector: ECC error, ignored\n", s->tag);
        return -1;
    }
    if (magic != s->legacy_magic) return -1;
    if (flash_read_checked(s->legacy_off, out, len) != 0) {
        printf("[%s] legacy sector: ECC error, ignored\n", s->tag);
        return -1;
    }
    printf("[%s] loaded the legacy layout (0x%06lx); the next save migrates it\n",
           s->tag, (unsigned long)s->legacy_off);
    return 0;
}

int ab_store_save(const ab_store_t *s, const void *data, size_t len)
{
    if (!data || len == 0 || len > AB_STORE_MAX_PAYLOAD) return -1;
    bool v[2]; ab_hdr_t h[2];
    int n = scan(s, v, h);
    if (n >= 0 && hdr_ver(&h[n]) == s->version && hdr_len(&h[n]) == len &&
        slot_payload_equals(s, n, data, len)) {
        printf("[%s] unchanged, not rewritten\n", s->tag);
        return 0;
    }
    int target = (n < 0) ? 0 : 1 - n;
    uint32_t seq = (n < 0) ? 1u : h[n].seq + 1u;
    if (write_record(s, target, seq, data, len) != 0) return -1;
    printf("[%s] saved OK\n", s->tag);
    return 0;
}

int ab_store_clear(const ab_store_t *s)
{
    bool v[2]; ab_hdr_t h[2];
    int n = scan(s, v, h);
    int target = (n < 0) ? 0 : 1 - n;
    uint32_t seq = (n < 0) ? 1u : h[n].seq + 1u;
    int rc = write_record(s, target, seq, "", 0);

    /* The tombstone now outranks everything; wipe the rest so the old secrets are
       gone. If the tombstone failed, erasing everything is the fallback clear. */
    uint32_t irqs = save_and_disable_interrupts();
    if (rc != 0) flash_range_erase(s->slot_off[target], FLASH_SECTOR_SIZE);
    flash_range_erase(s->slot_off[1 - target], FLASH_SECTOR_SIZE);
    if (s->legacy_off) flash_range_erase(s->legacy_off, FLASH_SECTOR_SIZE);
    restore_interrupts(irqs);
    printf("[%s] cleared\n", s->tag);
    return rc;
}

/* ---- Wi-Fi config ----------------------------------------------------------*/

static const ab_store_t s_wifi_store = {
    .tag          = "cfg",
    .slot_off     = { CONFIG_SLOT_A_OFFSET, CONFIG_SLOT_B_OFFSET },
    .magic        = CONFIG_RECORD_MAGIC,
    .version      = CONFIG_VERSION,
    .legacy_off   = CONFIG_FLASH_OFFSET,
    .legacy_magic = CONFIG_MAGIC,
};

int config_load(config_t *out)
{
    config_t tmp;
    if (ab_store_load(&s_wifi_store, &tmp, sizeof(tmp)) != 0) return -1;
    if (tmp.magic != CONFIG_MAGIC || tmp.version != CONFIG_VERSION) {
        printf("[cfg] config schema mismatch (version=%lu, expected %lu), ignoring\n",
               (unsigned long)tmp.version, (unsigned long)CONFIG_VERSION);
        return -1;
    }
    if (out) memcpy(out, &tmp, sizeof(tmp));
    return 0;
}

int config_save(const config_t *cfg)
{
    if (!cfg) return -1;
    config_t tmp;
    memcpy(&tmp, cfg, sizeof(tmp));
    tmp.magic   = CONFIG_MAGIC;
    tmp.version = CONFIG_VERSION;
    return ab_store_save(&s_wifi_store, &tmp, sizeof(tmp));
}

void config_clear(void)
{
    ab_store_clear(&s_wifi_store);
}
