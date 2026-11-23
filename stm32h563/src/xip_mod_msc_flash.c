/* xip_mod_msc_flash.c — flashing a board that shows up as a USB drive (msc_flash.h).
 *
 * Runs from the W25Q (xip_image.h): no static data, no callbacks; the ops in the context are
 * internal functions. Also built into the host tests. */
#include "msc_flash.h"

#include <string.h>

#define UF2_MAGIC0  0x0A324655u
#define UF2_MAGIC1  0x9E5D5157u
#define UF2_MAGIC2  0x0AB16F30u
#define STEP_SECTORS 8u          /* sectors written per step (UF2, data) */
#define STEP_ENTRIES 2048u       /* FAT entries scanned per step */
#define NONE        0xFFFFFFFFu
#define STEP32      1024u        /* FAT32: entries scanned per step (a 4 KB cluster map per step) */
#define STEP_FREE   256u         /* FAT32: chain entries freed per step (random reads are slow) */
#define EOC32       0x0FFFFFF8u  /* FAT32 entries from here on end a chain */
#define BAD32       0x0FFFFFF7u

static uint32_t rd16(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8); }
static uint32_t rd32(const uint8_t *p) { return rd16(p) | (rd16(p + 2) << 16); }
static void wr16(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void wr32(uint8_t *p, uint32_t v) { wr16(p, v); wr16(p + 2, v >> 16); }

static int finish(msc_flash_ctx_t *c, int rc)
{
    c->result = rc;
    c->phase = MSC_PH_DONE;
    return rc;
}

/* A FAT32 boot sector (s is c->buf; the values the caller took from it come in as arguments).
   The FAT type is decided by the cluster count, as the Microsoft spec says. */
static int geometry32(msc_flash_ctx_t *c, const uint8_t *s, uint32_t spc, uint32_t rsv,
                      uint32_t nfats, uint32_t tot)
{
    uint32_t fatsz = rd32(s + 36), ext = rd16(s + 40), ver = rd16(s + 42);
    uint32_t root_clus = rd32(s + 44), fsi = rd16(s + 48);
    if (spc == 0 || (spc & (spc - 1)) != 0 || rsv == 0 || nfats == 0 || nfats > 4 ||
        fatsz == 0 || tot == 0 || ver != 0)
        return MSC_FLASH_E_FS;
    uint64_t meta = (uint64_t)rsv + (uint64_t)nfats * fatsz;
    if (meta >= tot || (uint64_t)c->vol_start + tot > c->ops.block_count) return MSC_FLASH_E_FS;
    uint32_t clusters = (uint32_t)((tot - meta) / spc);
    if (clusters < 65525u || clusters > 0x0FFFFFF4u) return MSC_FLASH_E_FS;
    if ((uint64_t)(clusters + 2u) * 4u > (uint64_t)fatsz * MSC_FLASH_SECTOR) return MSC_FLASH_E_FS;
    if (root_clus < 2u || root_clus >= clusters + 2u) return MSC_FLASH_E_FS;
    bool mirror = (ext & 0x80u) == 0;
    uint32_t active = ext & 0x0Fu;
    if (!mirror && active >= nfats) return MSC_FLASH_E_FS;
    c->fat_bits = 32;
    c->spc = (uint8_t)spc;
    c->nfats = (uint8_t)nfats;
    c->fat_start = c->vol_start + rsv;
    c->fat_secs = fatsz;
    c->root_start = 0;
    c->root_secs = 0;
    c->data_start = c->fat_start + nfats * fatsz;
    c->clusters = clusters;
    c->root_clus = root_clus;
    c->fat_rd = (uint8_t)(mirror ? 0u : active);
    c->fat_wr_end = (uint8_t)(mirror ? nfats : active + 1u);
    c->fsi_ok = 0;
    c->hint = 0;
    c->fsi_free = NONE;
    if (c->mode == MSC_FLASH_FILE && fsi != 0 && fsi != 0xFFFFu && fsi < rsv) {
        c->fsi_lba = c->vol_start + fsi;
        /* An unreadable FSInfo sector only means no hint and no update. */
        if (c->ops.read(c->ops.io, c->fsi_lba, c->buf, 1) == 0 && rd32(c->buf) == 0x41615252u &&
            rd32(c->buf + 484) == 0x61417272u && rd32(c->buf + 508) == 0xAA550000u) {
            c->fsi_ok = 1;
            c->fsi_free = rd32(c->buf + 488);
            c->hint = rd32(c->buf + 492);
        }
    }
    return 0;
}

/* Sector 0 (or the first MBR partition's): the FAT geometry. 0 = a FAT12/16/32 volume. */
static int read_geometry(msc_flash_ctx_t *c)
{
    uint8_t *s = c->buf;
    if (c->ops.read(c->ops.io, 0, s, 1) != 0) return MSC_FLASH_E_IO;
    c->vol_start = 0;
    if (s[510] == 0x55 && s[511] == 0xAA && s[0] != 0xEB && s[0] != 0xE9) {
        /* No jump instruction: an MBR. Take the first partition. */
        uint32_t lba = rd32(s + 446 + 8);
        uint8_t type = s[446 + 4];
        if (type == 0 || lba == 0 || lba >= c->ops.block_count) return MSC_FLASH_E_FS;
        c->vol_start = lba;
        if (c->ops.read(c->ops.io, lba, s, 1) != 0) return MSC_FLASH_E_IO;
    }
    if (memcmp(s + 3, "EXFAT   ", 8) == 0) return MSC_FLASH_E_EXFAT;
    uint32_t bps = rd16(s + 11), spc = s[13], rsv = rd16(s + 14), nfats = s[16];
    uint32_t root_ent = rd16(s + 17), tot = rd16(s + 19), fatsz = rd16(s + 22);
    if (tot == 0) tot = rd32(s + 32);
    if (bps != MSC_FLASH_SECTOR) return MSC_FLASH_E_SECTOR;
    if (fatsz == 0 && root_ent == 0) return geometry32(c, s, spc, rsv, nfats, tot);
    if (spc == 0 || (spc & (spc - 1)) != 0 || rsv == 0 || nfats == 0 || nfats > 4 ||
        root_ent == 0 || fatsz == 0 || tot == 0)
        return MSC_FLASH_E_FS;   /* FAT32 has no fixed root directory (root_ent 0, fatsz16 0) */
    uint32_t root_secs = (root_ent * 32u + MSC_FLASH_SECTOR - 1u) / MSC_FLASH_SECTOR;
    uint32_t meta = rsv + nfats * fatsz + root_secs;
    if (meta >= tot || c->vol_start + tot > c->ops.block_count) return MSC_FLASH_E_FS;
    uint32_t clusters = (tot - meta) / spc;
    if (clusters >= 65525u) return MSC_FLASH_E_FS;
    c->fat_bits = clusters < 4085u ? 12 : 16;
    if (((clusters + 2u) * c->fat_bits + 7u) / 8u > fatsz * MSC_FLASH_SECTOR) return MSC_FLASH_E_FS;
    c->spc = (uint8_t)spc;
    c->nfats = (uint8_t)nfats;
    c->fat_start = c->vol_start + rsv;
    c->fat_secs = fatsz;
    c->root_start = c->fat_start + nfats * fatsz;
    c->root_secs = root_secs;
    c->data_start = c->root_start + root_secs;
    c->clusters = clusters;
    c->fat_rd = 0;
    c->fat_wr_end = (uint8_t)nfats;
    return 0;
}

/* ---- UF2 ----------------------------------------------------------------------------------- */

static int uf2_check(const uint8_t *b)
{
    if (rd32(b) != UF2_MAGIC0 || rd32(b + 4) != UF2_MAGIC1 || rd32(b + 508) != UF2_MAGIC2) return -1;
    uint32_t payload = rd32(b + 16), no = rd32(b + 20), num = rd32(b + 24);
    if (payload > 476u || num == 0 || no >= num) return -1;
    return 0;
}

static int uf2_step(msc_flash_ctx_t *c)
{
    for (uint32_t i = 0; i < STEP_SECTORS && c->pos < c->total; i++, c->pos++) {
        if (c->ops.src(c->ops.io, c->pos * MSC_FLASH_SECTOR, c->buf, MSC_FLASH_SECTOR) != 0)
            return finish(c, MSC_FLASH_E_SRC);
        if (uf2_check(c->buf) != 0) return finish(c, MSC_FLASH_E_NOT_UF2);
        if (c->ops.write(c->ops.io, c->data_base + c->pos, c->buf, 1) != 0) return finish(c, MSC_FLASH_E_IO);
        c->done = c->pos + 1;
    }
    if (c->pos < c->total) return MSC_FLASH_RUNNING;
    return finish(c, MSC_FLASH_DONE);
}

/* ---- FAT ----------------------------------------------------------------------------------- */

/* Make buf hold the FAT bytes at byte_off (and the next) of FAT copy `copy`. 0 = ok. */
static int fat_window(msc_flash_ctx_t *c, uint32_t copy, uint32_t byte_off)
{
    uint32_t sec = byte_off / MSC_FLASH_SECTOR;
    if (c->fat_bits == 32) sec &= ~1u;      /* aligned pairs: a sequential scan reads each sector once */
    uint32_t lba = c->fat_start + copy * c->fat_secs + sec;
    uint32_t n = sec + 1u < c->fat_secs ? 2u : 1u;
    if (c->fat_cached == lba) return 0;
    if (c->fat_dirty) {
        uint32_t cs = c->fat_cached - c->fat_start - copy * c->fat_secs;
        uint32_t cn = cs + 1u < c->fat_secs ? 2u : 1u;
        if (c->ops.write(c->ops.io, c->fat_cached, c->buf, cn) != 0) return -1;
        c->fat_dirty = false;
    }
    if (c->ops.read(c->ops.io, lba, c->buf, n) != 0) return -1;
    c->fat_cached = lba;
    return 0;
}

static int fat_flush(msc_flash_ctx_t *c, uint32_t copy)
{
    if (!c->fat_dirty) return 0;
    uint32_t cs = c->fat_cached - c->fat_start - copy * c->fat_secs;
    uint32_t cn = cs + 1u < c->fat_secs ? 2u : 1u;
    if (c->ops.write(c->ops.io, c->fat_cached, c->buf, cn) != 0) return -1;
    c->fat_dirty = false;
    return 0;
}

/* Entry n of FAT copy `copy`: get (set == false) or put `val`. -1 = I/O error. */
static int32_t fat_entry(msc_flash_ctx_t *c, uint32_t copy, uint32_t n, bool set, uint32_t val)
{
    uint32_t off = c->fat_bits == 12 ? n + n / 2u : n * 2u;
    if (c->fat_bits == 32) off = n * 4u;
    if (fat_window(c, copy, off) != 0) return -1;
    uint32_t wsec = off / MSC_FLASH_SECTOR;
    if (c->fat_bits == 32) wsec &= ~1u;
    uint8_t *p = c->buf + (off - wsec * MSC_FLASH_SECTOR);
    if (c->fat_bits == 32) {
        uint32_t w = rd32(p);
        if (!set) return (int32_t)(w & 0x0FFFFFFFu);
        wr32(p, (w & 0xF0000000u) | (val & 0x0FFFFFFFu));   /* the top 4 bits are reserved: keep */
        c->fat_dirty = true;
        return 0;
    }
    uint32_t v = rd16(p);
    if (c->fat_bits == 12) {
        if (!set) return (int32_t)((n & 1u) ? v >> 4 : v & 0x0FFFu);
        v = (n & 1u) ? ((v & 0x000Fu) | (val << 4)) : ((v & 0xF000u) | (val & 0x0FFFu));
    } else if (!set) {
        return (int32_t)v;
    } else {
        v = val & 0xFFFFu;
    }
    wr16(p, v);
    c->fat_dirty = true;
    return 0;
}

static bool name_matches(const uint8_t *e, const char *name83)
{
    return memcmp(e, name83, 11) == 0 && (e[11] & 0x18u) == 0;   /* not a volume label or directory */
}

static int dir32_scan_step(msc_flash_ctx_t *c);

static int dir_scan_step(msc_flash_ctx_t *c)
{
    if (c->fat_bits == 32) return dir32_scan_step(c);
    if (c->pos >= c->root_secs) {
        if (c->dir_lba == NONE) return finish(c, MSC_FLASH_E_FULL);
        c->phase = MSC_PH_FAT_SCAN;
        c->pos = 2;
        c->run_len = 0;
        c->fat_cached = NONE;
        return MSC_FLASH_RUNNING;
    }
    uint32_t lba = c->root_start + c->pos;
    if (c->ops.read(c->ops.io, lba, c->buf, 1) != 0) return finish(c, MSC_FLASH_E_IO);
    for (uint32_t off = 0; off < MSC_FLASH_SECTOR; off += 32u) {
        const uint8_t *e = c->buf + off;
        if (name_matches(e, c->name83)) {       /* the same name: take its entry over */
            c->dir_lba = lba;
            c->dir_off = off;
            c->pos = c->root_secs;
            return MSC_FLASH_RUNNING;
        }
        if ((e[0] == 0x00 || e[0] == 0xE5) && c->dir_lba == NONE) {
            c->dir_lba = lba;
            c->dir_off = off;
        }
        if (e[0] == 0x00) {                       /* end of the directory */
            c->pos = c->root_secs;
            return MSC_FLASH_RUNNING;
        }
    }
    c->pos++;
    return MSC_FLASH_RUNNING;
}

static int fat32_scan_step(msc_flash_ctx_t *c);

static int fat_scan_step(msc_flash_ctx_t *c)
{
    if (c->fat_bits == 32) return fat32_scan_step(c);
    const uint32_t end = c->clusters + 2u;
    for (uint32_t i = 0; i < STEP_ENTRIES && c->pos < end; i++, c->pos++) {
        int32_t v = fat_entry(c, 0, c->pos, false, 0);
        if (v < 0) return finish(c, MSC_FLASH_E_IO);
        if (v == 0) {
            if (c->run_len++ == 0) c->run_start = c->pos;
            if (c->run_len == c->need_clusters) {
                c->phase = MSC_PH_DATA;
                c->pos = 0;
                return MSC_FLASH_RUNNING;
            }
        } else {
            c->run_len = 0;
        }
    }
    if (c->pos >= end) return finish(c, MSC_FLASH_E_FULL);
    return MSC_FLASH_RUNNING;
}

static int data_step(msc_flash_ctx_t *c)
{
    const uint32_t first = c->data_start + (c->run_start + c->dir_extend - 2u) * c->spc;
    for (uint32_t i = 0; i < STEP_SECTORS && c->pos < c->total; i++, c->pos++) {
        uint32_t off = c->pos * MSC_FLASH_SECTOR;
        uint32_t n = c->len - off < MSC_FLASH_SECTOR ? c->len - off : MSC_FLASH_SECTOR;
        memset(c->buf, 0, MSC_FLASH_SECTOR);
        if (c->ops.src(c->ops.io, off, c->buf, n) != 0) return finish(c, MSC_FLASH_E_SRC);
        if (c->ops.write(c->ops.io, first + c->pos, c->buf, 1) != 0) return finish(c, MSC_FLASH_E_IO);
        c->done = c->pos + 1;
    }
    if (c->pos < c->total) return MSC_FLASH_RUNNING;
    c->phase = MSC_PH_FAT;
    c->pos = 0;
    c->fat_copy = c->fat_rd;
    c->fat_cached = NONE;
    c->fat_dirty = false;
    return MSC_FLASH_RUNNING;
}

static int fat_step(msc_flash_ctx_t *c)
{
    const uint32_t eoc = c->fat_bits == 12 ? 0x0FFFu : c->fat_bits == 16 ? 0xFFFFu : 0x0FFFFFFFu;
    for (uint32_t i = 0; i < STEP_ENTRIES && c->pos < c->need_clusters; i++, c->pos++) {
        uint32_t n = c->run_start + c->dir_extend + c->pos;
        uint32_t next = c->pos + 1u < c->need_clusters ? n + 1u : eoc;
        if (fat_entry(c, c->fat_copy, n, true, next) != 0) return finish(c, MSC_FLASH_E_IO);
    }
    if (c->pos < c->need_clusters) return MSC_FLASH_RUNNING;
    if (c->dir_extend) {   /* FAT32: the new directory cluster ends its chain and the old end links to it */
        if (fat_entry(c, c->fat_copy, c->run_start, true, 0x0FFFFFFFu) != 0 ||
            fat_entry(c, c->fat_copy, c->dir_last, true, c->run_start) != 0)
            return finish(c, MSC_FLASH_E_IO);
    }
    if (fat_flush(c, c->fat_copy) != 0) return finish(c, MSC_FLASH_E_IO);
    c->fat_cached = NONE;
    if (++c->fat_copy < c->fat_wr_end) {
        c->pos = 0;
        return MSC_FLASH_RUNNING;
    }
    c->phase = MSC_PH_DIR;
    return MSC_FLASH_RUNNING;
}

static int dir_step(msc_flash_ctx_t *c)
{
    if (c->ops.read(c->ops.io, c->dir_lba, c->buf, 1) != 0) return finish(c, MSC_FLASH_E_IO);
    uint8_t *e = c->buf + c->dir_off;
    memset(e, 0, 32);
    memcpy(e, c->name83, 11);
    e[11] = 0x20;                                            /* archive */
    const uint32_t date = ((2026u - 1980u) << 9) | (1u << 5) | 1u;   /* 2026-01-01 */
    wr16(e + 16, date);                                      /* created */
    wr16(e + 18, date);                                      /* accessed */
    wr16(e + 24, date);                                      /* written */
    wr16(e + 20, (c->run_start + c->dir_extend) >> 16);      /* first cluster, high half (FAT32) */
    wr16(e + 26, c->run_start + c->dir_extend);
    wr32(e + 28, c->len);
    if (c->ops.write(c->ops.io, c->dir_lba, c->buf, 1) != 0) return finish(c, MSC_FLASH_E_IO);
    if (c->fat_bits != 32) return finish(c, MSC_FLASH_DONE);
    if (c->old_first != 0) {                                 /* a replaced file: free its clusters */
        c->phase = MSC_PH_FREE_OLD;
        c->free_cur = c->old_first;
        c->free_n = 0;
        c->fat_copy = c->fat_rd;
        c->fat_cached = NONE;
        c->fat_dirty = false;
    } else {
        c->phase = MSC_PH_FSINFO;
    }
    return MSC_FLASH_RUNNING;
}

/* ---- FAT32 ---------------------------------------------------------------------------------- */

/* The root directory is a cluster chain: find a free entry or the same name, remember the chain's
   last cluster. One sector per step. */
static int dir32_scan_step(msc_flash_ctx_t *c)
{
    if (c->scan_end) {
        const uint32_t end = c->clusters + 2u;
        if (c->dir_lba == NONE) c->dir_extend = 1;          /* full: the run gets a cluster for it */
        c->phase = MSC_PH_FAT_SCAN;
        c->run_len = 0;
        c->fat_cached = NONE;
        c->pos = (c->fsi_ok && c->hint >= 2u && c->hint < end) ? c->hint : 2u;
        c->wrapped = c->pos == 2u;                           /* from 2 on there is no second pass */
        c->hint = c->wrapped ? end : c->pos;
        return MSC_FLASH_RUNNING;
    }
    uint32_t lba = c->data_start + (c->dir_clus - 2u) * c->spc + c->pos;
    if (c->ops.read(c->ops.io, lba, c->buf, 1) != 0) return finish(c, MSC_FLASH_E_IO);
    c->fat_cached = NONE;
    for (uint32_t off = 0; off < MSC_FLASH_SECTOR; off += 32u) {
        const uint8_t *e = c->buf + off;
        if (name_matches(e, c->name83)) {
            c->dir_lba = lba;
            c->dir_off = off;
            c->old_first = (rd16(e + 20) << 16) | rd16(e + 26);
            c->scan_end = 1;
            return MSC_FLASH_RUNNING;
        }
        if ((e[0] == 0x00 || e[0] == 0xE5) && c->dir_lba == NONE) {
            c->dir_lba = lba;
            c->dir_off = off;
        }
        if (e[0] == 0x00) {
            c->scan_end = 1;
            return MSC_FLASH_RUNNING;
        }
    }
    if (++c->pos < c->spc) return MSC_FLASH_RUNNING;
    c->pos = 0;
    int32_t nx = fat_entry(c, c->fat_rd, c->dir_clus, false, 0);
    c->fat_cached = NONE;
    if (nx < 0) return finish(c, MSC_FLASH_E_IO);
    if ((uint32_t)nx >= EOC32) {
        c->dir_last = c->dir_clus;
        c->scan_end = 1;
        return MSC_FLASH_RUNNING;
    }
    /* A directory holds at most 65536 entries (4096 sectors): a longer chain is broken or loops. */
    if ((uint32_t)nx < 2u || (uint32_t)nx >= c->clusters + 2u || ++c->dir_n * c->spc > 4096u)
        return finish(c, MSC_FLASH_E_FS);
    c->dir_clus = (uint32_t)nx;
    return MSC_FLASH_RUNNING;
}

/* A run of free clusters, from FSInfo's next-free hint to the end, then from 2 up to the hint
   (a run that starts before the hint may go on past it). STEP32 entries per step. */
static int fat32_scan_step(msc_flash_ctx_t *c)
{
    const uint32_t end = c->clusters + 2u, want = c->need_clusters + c->dir_extend;
    for (uint32_t i = 0; i < STEP32; i++) {
        if (c->pos >= end) {
            if (c->wrapped) return finish(c, MSC_FLASH_E_FULL);
            c->wrapped = 1;
            c->pos = 2;
            c->run_len = 0;
            continue;
        }
        if (c->wrapped && c->pos >= c->hint && c->run_len == 0) return finish(c, MSC_FLASH_E_FULL);
        int32_t v = fat_entry(c, c->fat_rd, c->pos, false, 0);
        if (v < 0) return finish(c, MSC_FLASH_E_IO);
        if (v == 0) {
            if (c->run_len++ == 0) c->run_start = c->pos;
            if (c->run_len == want) {
                if (c->dir_extend) {                         /* the entry goes in the new cluster */
                    c->dir_lba = c->data_start + (c->run_start - 2u) * c->spc;
                    c->dir_off = 0;
                }
                c->phase = c->dir_extend ? MSC_PH_DIR_ZERO : MSC_PH_DATA;
                c->pos = 0;
                return MSC_FLASH_RUNNING;
            }
        } else {
            c->run_len = 0;
        }
        c->pos++;
    }
    return MSC_FLASH_RUNNING;
}

/* The new directory cluster must be all zero before it is linked in. */
static int dir_zero_step(msc_flash_ctx_t *c)
{
    const uint32_t first = c->data_start + (c->run_start - 2u) * c->spc;
    memset(c->buf, 0, MSC_FLASH_SECTOR);
    for (uint32_t i = 0; i < STEP_SECTORS && c->pos < c->spc; i++, c->pos++)
        if (c->ops.write(c->ops.io, first + c->pos, c->buf, 1) != 0) return finish(c, MSC_FLASH_E_IO);
    if (c->pos < c->spc) return MSC_FLASH_RUNNING;
    c->phase = MSC_PH_DATA;
    c->pos = 0;
    return MSC_FLASH_RUNNING;
}

/* Free the chain of the file that was replaced, in every FAT copy written. */
static int free_old_step(msc_flash_ctx_t *c)
{
    for (uint32_t i = 0; i < STEP_FREE; i++) {
        uint32_t cur = c->free_cur;
        bool stop = true;
        if (cur >= 2u && cur < c->clusters + 2u && c->free_n < c->clusters) {
            int32_t nx = fat_entry(c, c->fat_copy, cur, false, 0);
            if (nx < 0) return finish(c, MSC_FLASH_E_IO);
            if (nx != 0 && (uint32_t)nx != BAD32) {          /* a free or bad cluster ends the walk */
                if (fat_entry(c, c->fat_copy, cur, true, 0) != 0) return finish(c, MSC_FLASH_E_IO);
                if (c->fat_copy == c->fat_rd) c->freed++;
                c->free_n++;
                c->free_cur = (uint32_t)nx;
                stop = (uint32_t)nx >= EOC32;
            }
        }
        if (!stop) continue;
        if (fat_flush(c, c->fat_copy) != 0) return finish(c, MSC_FLASH_E_IO);
        c->fat_cached = NONE;
        if (++c->fat_copy < c->fat_wr_end) {
            c->free_cur = c->old_first;
            c->free_n = 0;
            return MSC_FLASH_RUNNING;
        }
        c->phase = MSC_PH_FSINFO;
        return MSC_FLASH_RUNNING;
    }
    return MSC_FLASH_RUNNING;
}

/* FSInfo: the free count and the next-free hint, only when the sector is valid. */
static int fsinfo_step(msc_flash_ctx_t *c)
{
    if (!c->fsi_ok) return finish(c, MSC_FLASH_DONE);
    if (c->ops.read(c->ops.io, c->fsi_lba, c->buf, 1) != 0) return finish(c, MSC_FLASH_E_IO);
    if (rd32(c->buf) != 0x41615252u || rd32(c->buf + 484) != 0x61417272u ||
        rd32(c->buf + 508) != 0xAA550000u)
        return finish(c, MSC_FLASH_DONE);
    uint32_t used = c->need_clusters + c->dir_extend;
    uint32_t cnt = c->fsi_free;
    if (cnt != NONE) {   /* what the sector said, less what we took, plus what we freed */
        uint64_t n = (uint64_t)cnt + c->freed;
        cnt = (n >= used && n - used <= c->clusters) ? (uint32_t)(n - used) : NONE;
    }
    wr32(c->buf + 488, cnt);
    wr32(c->buf + 492, c->run_start + used - 1u);
    if (c->ops.write(c->ops.io, c->fsi_lba, c->buf, 1) != 0) return finish(c, MSC_FLASH_E_IO);
    return finish(c, MSC_FLASH_DONE);
}

/* ---- entry ---------------------------------------------------------------------------------- */

static int start(msc_flash_ctx_t *c)
{
    c->result = MSC_FLASH_RUNNING;
    c->done = 0;
    c->pos = 0;
    c->dir_lba = NONE;
    c->fat_cached = NONE;
    c->fat_dirty = false;
    c->old_first = c->freed = c->dir_n = 0;
    c->dir_extend = c->scan_end = c->wrapped = 0;
    if (c->ops.block_size != MSC_FLASH_SECTOR) return finish(c, MSC_FLASH_E_SECTOR);
    if (c->len == 0 || c->ops.block_count == 0) return finish(c, MSC_FLASH_E_ARGS);
    c->total = (c->len + MSC_FLASH_SECTOR - 1u) / MSC_FLASH_SECTOR;
    if (c->mode == MSC_FLASH_UF2) {
        if (c->len % MSC_FLASH_SECTOR != 0) return finish(c, MSC_FLASH_E_NOT_UF2);
        /* The data area of the drive's FAT, when it has one (a UF2 bootloader takes the blocks
           anywhere; the data area keeps the boot sector and FAT intact). */
        c->data_base = read_geometry(c) == 0 ? c->data_start : 0u;
        if (c->data_base + c->total > c->ops.block_count) return finish(c, MSC_FLASH_E_FULL);
        c->phase = MSC_PH_UF2;
        return MSC_FLASH_RUNNING;
    }
    if (c->mode != MSC_FLASH_FILE || c->name83[0] == ' ' || c->name83[0] == 0) return finish(c, MSC_FLASH_E_ARGS);
    int rc = read_geometry(c);
    if (rc != 0) return finish(c, rc);
    uint32_t csize = (uint32_t)c->spc * MSC_FLASH_SECTOR;
    c->need_clusters = (c->len + csize - 1u) / csize;
    if (c->need_clusters > c->clusters) return finish(c, MSC_FLASH_E_FULL);
    c->phase = MSC_PH_DIR_SCAN;
    c->dir_clus = c->root_clus;                              /* FAT32: the root chain starts here */
    c->dir_last = NONE;
    return MSC_FLASH_RUNNING;
}

int msc_flash_step(msc_flash_ctx_t *c)
{
    switch (c->phase) {
    case MSC_PH_START:    return start(c);
    case MSC_PH_UF2:      return uf2_step(c);
    case MSC_PH_DIR_SCAN: return dir_scan_step(c);
    case MSC_PH_FAT_SCAN: return fat_scan_step(c);
    case MSC_PH_DATA:     return data_step(c);
    case MSC_PH_FAT:      return fat_step(c);
    case MSC_PH_DIR:      return dir_step(c);
    case MSC_PH_DIR_ZERO: return dir_zero_step(c);
    case MSC_PH_FREE_OLD: return free_old_step(c);
    case MSC_PH_FSINFO:   return fsinfo_step(c);
    default:              return c->result;
    }
}
