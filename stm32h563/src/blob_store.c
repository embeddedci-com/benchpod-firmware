/*
 * blob_store.c — blob slots in the W25Q (see blob_store.h).
 *
 * Talks to the flash only through w25q.h, which the host tests replace with a RAM model.
 */
#include "blob_store.h"
#include "w25q.h"
#include "mbedtls/sha256.h"

#include <stdio.h>
#include <string.h>

static const struct {
    const char *name;
    uint32_t    base;
    uint32_t    size;         /* header + data */
} k_slots[BLOB_SLOT_COUNT] = {
    [BLOB_GW0] = { "gw0", 0x100000u, 0x040000u },
    [BLOB_GW1] = { "gw1", 0x140000u, 0x040000u },
    [BLOB_ESP] = { "esp", 0x200000u, 0x400000u },
    [BLOB_FW]  = { "fw",  0x600000u, 0x100000u },
    [BLOB_CA]  = { "ca",  0x180000u, 0x010000u },
    [BLOB_PROXY] = { "proxy", 0x190000u, 0x002000u },
};

static blob_info_t s_info[BLOB_SLOT_COUNT];
static uint32_t    s_capacity;        /* W25Q bytes, from the last blob_store_load */

/* A slot exists when its id is known and it fits the chip (the FW slot needs >= 7 MB). */
static bool slot_ok(blob_id_t id)
{
    return id < BLOB_SLOT_COUNT && (id < BLOB_COUNT || k_slots[id].base + k_slots[id].size <= s_capacity);
}

uint32_t blob_slot_base(blob_id_t id)     { return id < BLOB_SLOT_COUNT ? k_slots[id].base : 0u; }
uint32_t blob_slot_capacity(blob_id_t id) { return id < BLOB_SLOT_COUNT ? k_slots[id].size - BLOB_HDR_SIZE : 0u; }
const char *blob_name(blob_id_t id)       { return id < BLOB_SLOT_COUNT ? k_slots[id].name : "?"; }

int blob_from_name(const char *name)
{
    if (!name) return -1;
    for (int i = 0; i < BLOB_COUNT; i++)
        if (strcmp(name, k_slots[i].name) == 0) return i;
    return -1;
}

const blob_info_t *blob_store_info(blob_id_t id) { return id < BLOB_SLOT_COUNT ? &s_info[id] : NULL; }
bool blob_store_present(blob_id_t id)            { return id < BLOB_SLOT_COUNT && s_info[id].present; }

/* Parse one slot's header sector start into s_info[id]. */
static void load_slot(blob_id_t id)
{
    blob_hdr_t h;
    uint32_t commit = 0;
    memset(&s_info[id], 0, sizeof(s_info[id]));
    if (w25q_read(k_slots[id].base, (uint8_t *)&h, sizeof(h)) != 0) return;
    if (w25q_read(k_slots[id].base + BLOB_COMMIT_OFF, (uint8_t *)&commit, sizeof(commit)) != 0) return;
    if (h.magic != BLOB_HDR_MAGIC || h.hdr_version != BLOB_HDR_VERSION || h.id != (uint16_t)id ||
        commit != BLOB_COMMIT_MAGIC || h.len == 0 || h.len > blob_slot_capacity(id))
        return;
    s_info[id].present = true;
    s_info[id].len     = h.len;
    s_info[id].version = h.version;
    memcpy(s_info[id].sha256, h.sha256, 32);
}

int blob_store_load(void)
{
    uint8_t id[3] = {0};
    memset(s_info, 0, sizeof(s_info));
    s_capacity = 0;
    if (w25q_read_id(id) != 0 || w25q_capacity(id) < 0x600000u) {
        printf("[blob] no usable W25Q (ID %02x %02x %02x)\n", id[0], id[1], id[2]);
        return -1;
    }
    s_capacity = w25q_capacity(id);
    int n = 0;
    for (int i = 0; i < BLOB_SLOT_COUNT; i++) {
        if (!slot_ok((blob_id_t)i)) continue;
        load_slot((blob_id_t)i);
        if (s_info[i].present) n++;
    }
    return n;
}

int blob_store_init(void)
{
    w25q_open();
    int n = blob_store_load();
    w25q_close();
    for (int i = 0; i < BLOB_SLOT_COUNT; i++) {
        if (!slot_ok((blob_id_t)i)) continue;
        if (s_info[i].present)
            printf("[blob] %s: %lu bytes, version %lu\n", k_slots[i].name,
                   (unsigned long)s_info[i].len, (unsigned long)s_info[i].version);
        else
            printf("[blob] %s: empty\n", k_slots[i].name);
    }
    return n;
}

/* Erase [addr, addr+len) rounded out to 4 KB sectors, with 64 KB blocks where they fit. */
static int erase_range(uint32_t addr, uint32_t len)
{
    uint32_t end = (addr + len + W25Q_SECTOR - 1u) & ~(W25Q_SECTOR - 1u);
    addr &= ~(W25Q_SECTOR - 1u);
    while (addr < end) {
        if ((addr % W25Q_BLOCK) == 0u && end - addr >= W25Q_BLOCK) {
            if (w25q_erase_block(addr) != 0) return -1;
            addr += W25Q_BLOCK;
        } else {
            if (w25q_erase_sector(addr) != 0) return -1;
            addr += W25Q_SECTOR;
        }
    }
    return 0;
}

/* SHA-256 of `len` bytes of flash at `addr`. */
static int hash_flash(uint32_t addr, uint32_t len, uint8_t out[32])
{
    static uint8_t buf[W25Q_PAGE];
    mbedtls_sha256_context ctx;
    mbedtls_sha256_init(&ctx);
    mbedtls_sha256_starts(&ctx, 0);
    int rc = 0;
    for (uint32_t off = 0; off < len; off += W25Q_PAGE) {
        uint32_t n = len - off < W25Q_PAGE ? len - off : W25Q_PAGE;
        if (w25q_read(addr + off, buf, n) != 0) { rc = -1; break; }
        mbedtls_sha256_update(&ctx, buf, n);
    }
    if (rc == 0) mbedtls_sha256_finish(&ctx, out);
    mbedtls_sha256_free(&ctx);
    return rc;
}

int blob_store_write(blob_id_t id, uint32_t len, uint32_t version, const uint8_t sha256[32],
                     blob_src_fn src, void *ctx)
{
    if (!slot_ok(id) || !src || !sha256 || len == 0 || len > blob_slot_capacity(id)) return -1;
    const uint32_t base = k_slots[id].base;
    const uint32_t data = base + BLOB_HDR_SIZE;
    static uint8_t page[W25Q_PAGE];

    memset(&s_info[id], 0, sizeof(s_info[id]));         /* empty from the first erase on */
    if (erase_range(base, BLOB_HDR_SIZE + len) != 0) {
        printf("[blob] %s: erase failed\n", k_slots[id].name);
        return -1;
    }
    for (uint32_t off = 0; off < len; off += W25Q_PAGE) {
        uint32_t n = len - off < W25Q_PAGE ? len - off : W25Q_PAGE;
        if (src(ctx, off, page, n) != 0 || w25q_program(data + off, page, n) != 0) {
            printf("[blob] %s: program failed at %lu\n", k_slots[id].name, (unsigned long)off);
            return -1;
        }
    }
    uint8_t got[32];
    if (hash_flash(data, len, got) != 0 || memcmp(got, sha256, 32) != 0) {
        printf("[blob] %s: written data does not hash back\n", k_slots[id].name);
        return -1;
    }

    blob_hdr_t h;
    memset(&h, 0, sizeof(h));
    h.magic       = BLOB_HDR_MAGIC;
    h.hdr_version = BLOB_HDR_VERSION;
    h.id          = (uint16_t)id;
    h.len         = len;
    h.version     = version;
    memcpy(h.sha256, sha256, 32);
    const uint32_t commit = BLOB_COMMIT_MAGIC;
    if (w25q_program(base, (const uint8_t *)&h, sizeof(h)) != 0 ||
        w25q_program(base + BLOB_COMMIT_OFF, (const uint8_t *)&commit, sizeof(commit)) != 0) {
        printf("[blob] %s: header write failed\n", k_slots[id].name);
        return -1;
    }
    load_slot(id);
    if (!s_info[id].present) return -1;
    printf("[blob] %s: stored %lu bytes, version %lu\n", k_slots[id].name,
           (unsigned long)len, (unsigned long)version);
    return 0;
}

int blob_store_read(void *ctx, uint32_t off, uint8_t *buf, uint32_t n)
{
    blob_id_t id = (blob_id_t)(uintptr_t)ctx;
    if (id >= BLOB_SLOT_COUNT || !s_info[id].present || off > s_info[id].len || n > s_info[id].len - off)
        return -1;
    return w25q_read(k_slots[id].base + BLOB_HDR_SIZE + off, buf, n);
}

int blob_store_verify(blob_id_t id)
{
    if (!blob_store_present(id)) return -1;
    uint8_t got[32];
    if (hash_flash(k_slots[id].base + BLOB_HDR_SIZE, s_info[id].len, got) != 0) return -1;
    return memcmp(got, s_info[id].sha256, 32) == 0 ? 0 : -1;
}

blob_state_t blob_state_of(const blob_info_t *have, const blob_manifest_t *want)
{
    if (!have || !have->present) return (want && want->len) ? BLOB_STATE_MISSING : BLOB_STATE_UNKNOWN;
    if (!want || want->len == 0) return BLOB_STATE_UNKNOWN;
    if (have->len == want->len && memcmp(have->sha256, want->sha256, 32) == 0) return BLOB_STATE_OK;
    return BLOB_STATE_OUTDATED;
}

blob_state_t blob_state(blob_id_t id)
{
    if (id >= BLOB_COUNT) return BLOB_STATE_UNKNOWN;
    return blob_state_of(&s_info[id], blob_manifest(id));
}

const char *blob_state_str(blob_state_t s)
{
    switch (s) {
        case BLOB_STATE_OK:       return "ok";
        case BLOB_STATE_OUTDATED: return "outdated";
        case BLOB_STATE_MISSING:  return "missing";
        default:                  return "unknown";
    }
}
