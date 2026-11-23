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
/* The entry's date and time from the context: FAT date and time words, and the exFAT timestamp.
   Unknown (0) or before 2020: 2026-01-01 00:00:00. Pure arithmetic: no tables, no static data. */
static void stamp(const msc_flash_ctx_t *c, uint32_t *fat_date, uint32_t *fat_time, uint32_t *xts)
{
    uint32_t t = c->unix_time;
    if (t < 1577836800u) t = 1767225600u;                       /* 2020-01-01; 2026-01-01 00:00:00 */
    const uint32_t days = t / 86400u, sod = t % 86400u;
    /* civil from days (Howard Hinnant), days since 1970-01-01 */
    const uint32_t z = days + 719468u, era = z / 146097u, doe = z - era * 146097u;
    const uint32_t yoe = (doe - doe / 1460u + doe / 36524u - doe / 146096u) / 365u;
    const uint32_t doy = doe - (365u * yoe + yoe / 4u - yoe / 100u);
    const uint32_t mp = (5u * doy + 2u) / 153u;
    const uint32_t d = doy - (153u * mp + 2u) / 5u + 1u;
    const uint32_t mo = mp < 10u ? mp + 3u : mp - 9u;
    uint32_t y = yoe + era * 400u + (mo <= 2u ? 1u : 0u);
    if (y > 2107u) y = 2107u;
    const uint32_t hh = sod / 3600u, mi = (sod % 3600u) / 60u, ss = sod % 60u;
    *fat_date = ((y - 1980u) << 9) | (mo << 5) | d;
    *fat_time = (hh << 11) | (mi << 5) | (ss / 2u);
    *xts = ((y - 1980u) << 25) | (mo << 21) | (d << 16) | (hh << 11) | (mi << 5) | (ss / 2u);
}

static void wr16(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void wr32(uint8_t *p, uint32_t v) { wr16(p, v); wr16(p + 2, v >> 16); }

static uint32_t clus_lba(const msc_flash_ctx_t *c, uint32_t clus, uint32_t sec)
{
    return c->data_start + (clus - 2u) * c->spc + sec;
}

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
static int geometry_exfat(msc_flash_ctx_t *c, const uint8_t *s);

static int read_geometry(msc_flash_ctx_t *c)
{
    uint8_t *s = c->buf;
    c->is_exfat = 0;
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
    if (memcmp(s + 3, "EXFAT   ", 8) == 0) return geometry_exfat(c, s);
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
static int xdir_step(msc_flash_ctx_t *c);
static int xfind_step(msc_flash_ctx_t *c);

static int dir_scan_step(msc_flash_ctx_t *c)
{
    if (c->is_exfat) return xdir_step(c);
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
    if (c->is_exfat) return xfind_step(c);
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
    c->phase = c->is_exfat ? MSC_PH_XBITMAP : MSC_PH_FAT;   /* exFAT: a contiguous file needs no FAT */
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
    uint32_t date, tm, xts;
    stamp(c, &date, &tm, &xts);
    wr16(e + 14, tm);                                        /* created: time */
    wr16(e + 22, tm);                                        /* written: time */
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

/* ---- READ and VERIFY ------------------------------------------------------------------------- */


/* The root directory entry `name83`: one sector per step (FAT12/16 root area, or the FAT32 chain). */
static int find_step(msc_flash_ctx_t *c)
{
    const bool f32 = c->fat_bits == 32;
    bool end = f32 ? c->scan_end != 0 : c->pos >= c->root_secs;
    if (!end) {
        uint32_t lba = f32 ? clus_lba(c, c->dir_clus, c->pos) : c->root_start + c->pos;
        if (c->ops.read(c->ops.io, lba, c->buf, 1) != 0) return finish(c, MSC_FLASH_E_IO);
        c->fat_cached = NONE;
        for (uint32_t off = 0; off < MSC_FLASH_SECTOR; off += 32u) {
            const uint8_t *e = c->buf + off;
            if (name_matches(e, c->name83)) {
                c->f_clus = f32 ? (rd16(e + 20) << 16) | rd16(e + 26) : rd16(e + 26);
                c->f_size = rd32(e + 28);
                c->found = 1;
                c->pos = 0;
                c->phase = c->mode == MSC_FLASH_READ ? MSC_PH_READ : MSC_PH_VERIFY;
                return MSC_FLASH_RUNNING;
            }
            if (e[0] == 0x00) { end = true; break; }
        }
        if (!end) {
            if (!f32) { c->pos++; return MSC_FLASH_RUNNING; }
            if (++c->pos < c->spc) return MSC_FLASH_RUNNING;
            c->pos = 0;
            int32_t nx = fat_entry(c, c->fat_rd, c->dir_clus, false, 0);
            c->fat_cached = NONE;
            if (nx < 0) return finish(c, MSC_FLASH_E_IO);
            if ((uint32_t)nx >= EOC32) c->scan_end = 1;
            else if ((uint32_t)nx < 2u || (uint32_t)nx >= c->clusters + 2u || ++c->dir_n * c->spc > 4096u)
                return finish(c, MSC_FLASH_E_FS);
            else c->dir_clus = (uint32_t)nx;
            return MSC_FLASH_RUNNING;
        }
    }
    return finish(c, c->mode == MSC_FLASH_READ ? MSC_FLASH_DONE : MSC_FLASH_E_NOFILE);
}

static int read_step(msc_flash_ctx_t *c)
{
    uint32_t n = c->f_size < sizeof(c->out) ? c->f_size : (uint32_t)sizeof(c->out);
    c->out_len = 0;
    if (n != 0 && c->f_clus >= 2u && c->f_clus < c->clusters + 2u) {
        if (c->ops.read(c->ops.io, clus_lba(c, c->f_clus, 0), c->buf, 1) != 0) return finish(c, MSC_FLASH_E_IO);
        memcpy(c->out, c->buf, n);
        c->out_len = (uint16_t)n;
    }
    return finish(c, MSC_FLASH_DONE);
}

static int verify_step(msc_flash_ctx_t *c)
{
    const uint32_t file_secs = c->f_size / MSC_FLASH_SECTOR;
    if (c->f_clus < 2u || c->f_clus >= c->clusters + 2u || file_secs == 0) return finish(c, MSC_FLASH_E_NOFILE);
    switch (c->vf_stage) {
    case 0:   /* the file's first block gives its base address and payload size */
        if (c->ops.read(c->ops.io, clus_lba(c, c->f_clus, 0), c->buf, 1) != 0) return finish(c, MSC_FLASH_E_IO);
        if (uf2_check(c->buf) != 0 || rd32(c->buf + 16) == 0) return finish(c, MSC_FLASH_E_NOFILE);
        c->vf_base = rd32(c->buf + 12);
        c->vf_plen = rd32(c->buf + 16);
        c->vf_clus = c->f_clus;
        c->vf_idx = 0;
        c->vf_stage = 1;
        return MSC_FLASH_RUNNING;
    case 1: { /* our next block: which sector of the file holds that address */
        if (c->pos >= c->total) return finish(c, MSC_FLASH_DONE);
        if (c->ops.src(c->ops.io, c->pos * MSC_FLASH_SECTOR, c->buf, MSC_FLASH_SECTOR) != 0)
            return finish(c, MSC_FLASH_E_SRC);
        if (uf2_check(c->buf) != 0) return finish(c, MSC_FLASH_E_NOT_UF2);
        uint32_t addr = rd32(c->buf + 12), plen = rd32(c->buf + 16);
        c->fat_cached = NONE;
        if (plen != c->vf_plen || addr < c->vf_base || (addr - c->vf_base) % plen != 0)
            return finish(c, MSC_FLASH_E_VERIFY);
        c->vf_sec = (addr - c->vf_base) / plen;
        if (c->vf_sec >= file_secs) return finish(c, MSC_FLASH_E_VERIFY);
        if (c->vf_sec / c->spc < c->vf_idx) { c->vf_clus = c->f_clus; c->vf_idx = 0; }
        c->vf_stage = 2;
        return MSC_FLASH_RUNNING;
    }
    case 2:   /* walk the file's cluster chain forward, a bounded number of entries per step */
        for (uint32_t i = 0; i < STEP_ENTRIES && c->vf_idx < c->vf_sec / c->spc; i++) {
            int32_t nx = fat_entry(c, c->fat_rd, c->vf_clus, false, 0);
            if (nx < 0) return finish(c, MSC_FLASH_E_IO);
            if ((uint32_t)nx < 2u || (uint32_t)nx >= c->clusters + 2u) return finish(c, MSC_FLASH_E_VERIFY);
            c->vf_clus = (uint32_t)nx;
            c->vf_idx++;
        }
        if (c->vf_idx == c->vf_sec / c->spc) c->vf_stage = 3;
        return MSC_FLASH_RUNNING;
    default: {   /* compare: our block in buf, the drive's in buf + 512 */
        if (c->ops.src(c->ops.io, c->pos * MSC_FLASH_SECTOR, c->buf, MSC_FLASH_SECTOR) != 0)
            return finish(c, MSC_FLASH_E_SRC);
        if (c->ops.read(c->ops.io, clus_lba(c, c->vf_clus, c->vf_sec % c->spc), c->buf + MSC_FLASH_SECTOR, 1) != 0)
            return finish(c, MSC_FLASH_E_IO);
        const uint8_t *a = c->buf, *b = c->buf + MSC_FLASH_SECTOR;
        if (uf2_check(b) != 0 || rd32(b + 12) != rd32(a + 12) || rd32(b + 16) != rd32(a + 16) ||
            memcmp(a + 32, b + 32, rd32(a + 16)) != 0)
            return finish(c, MSC_FLASH_E_VERIFY);
        c->fat_cached = NONE;
        c->pos++;
        c->done = c->pos;
        c->vf_stage = 1;
        return MSC_FLASH_RUNNING;
    }
    }
}

/* ---- exFAT ---------------------------------------------------------------------------------- */

#define X_EOC   0xFFFFFFF8u               /* FAT entries from here on end a chain */
#define X_BITS  (MSC_FLASH_SECTOR * 8u)   /* clusters one bitmap sector covers */
#define X_FAT   (c->buf + MSC_FLASH_SECTOR)

/* The boot sector (s is c->buf). The writer takes 512-byte sectors only and honors ActiveFat. */
static int geometry_exfat(msc_flash_ctx_t *c, const uint8_t *s)
{
    if (s[510] != 0x55 || s[511] != 0xAA) return MSC_FLASH_E_FS;
    for (uint32_t i = 11; i < 64; i++)
        if (s[i] != 0) return MSC_FLASH_E_FS;               /* MustBeZero */
    uint32_t sh = s[108], spcs = s[109], nfats = s[110], flags = rd16(s + 106);
    if (sh < 9 || sh > 12) return MSC_FLASH_E_FS;
    if (sh != 9) return MSC_FLASH_E_SECTOR;
    if ((rd16(s + 104) >> 8) != 1 || spcs > 15 || nfats < 1 || nfats > 2) return MSC_FLASH_E_FS;
    uint64_t vol = rd32(s + 72) | ((uint64_t)rd32(s + 76) << 32);
    uint32_t fatoff = rd32(s + 80), fatlen = rd32(s + 84), heap = rd32(s + 88), ncl = rd32(s + 92);
    uint32_t root = rd32(s + 96), spc = 1u << spcs, active = flags & 1u;
    if (vol < 2048u || (uint64_t)c->vol_start + vol > c->ops.block_count) return MSC_FLASH_E_FS;
    if (fatoff < 24u || fatlen == 0 || ncl == 0 || ncl > 0xFFFFFFF5u) return MSC_FLASH_E_FS;
    if ((uint64_t)fatoff + (uint64_t)fatlen * nfats > heap) return MSC_FLASH_E_FS;
    if ((uint64_t)heap + (uint64_t)ncl * spc > vol) return MSC_FLASH_E_FS;
    if ((uint64_t)(ncl + 2u) * 4u > (uint64_t)fatlen * MSC_FLASH_SECTOR) return MSC_FLASH_E_FS;
    if (root < 2u || root >= ncl + 2u || active >= nfats) return MSC_FLASH_E_FS;
    c->is_exfat = 1;
    c->fat_bits = 0;
    c->x_active = (uint8_t)active;
    c->spc = (uint16_t)spc;
    c->nfats = (uint8_t)nfats;
    c->fat_start = c->vol_start + fatoff + active * fatlen;   /* the active FAT */
    c->fat_secs = fatlen;
    c->data_start = c->vol_start + heap;
    c->clusters = ncl;
    c->root_clus = root;
    return 0;
}

/* FAT entry n, in the window in the second half of buf. NULL = I/O error or out of range. */
static uint8_t *xfat_at(msc_flash_ctx_t *c, uint32_t n)
{
    uint32_t lba = c->fat_start + n / 128u;
    if (n >= c->clusters + 2u) return NULL;
    if (c->fat_cached != lba) {
        if (c->fat_dirty) {
            if (c->ops.write(c->ops.io, c->fat_cached, X_FAT, 1) != 0) return NULL;
            c->fat_dirty = false;
        }
        if (c->ops.read(c->ops.io, lba, X_FAT, 1) != 0) return NULL;
        c->fat_cached = lba;
    }
    return X_FAT + (n % 128u) * 4u;
}

static int xfat_get(msc_flash_ctx_t *c, uint32_t n, uint32_t *v)
{
    const uint8_t *p = xfat_at(c, n);
    if (!p) return -1;
    *v = rd32(p);
    return 0;
}

static int xfat_flush(msc_flash_ctx_t *c)
{
    if (!c->fat_dirty) return 0;
    if (c->ops.write(c->ops.io, c->fat_cached, X_FAT, 1) != 0) return -1;
    c->fat_dirty = false;
    return 0;
}

static int bm_flush(msc_flash_ctx_t *c)
{
    if (!c->bm_dirty) return 0;
    if (c->ops.write(c->ops.io, c->bm_lba, c->buf, 1) != 0) return -1;
    c->bm_dirty = 0;
    return 0;
}

/* Sector i of the allocation bitmap into the first half of buf. 1 = there, 0 = call again (a long
   chain is walked a bounded number of entries per step), -1 = I/O error, -2 = broken bitmap. */
static int bm_load(msc_flash_ctx_t *c, uint32_t i)
{
    if (c->bm_lba != NONE && c->bm_i == i) return 1;
    if (bm_flush(c) != 0) return -1;
    if (i >= (c->bm_len + MSC_FLASH_SECTOR - 1u) / MSC_FLASH_SECTOR) return -2;
    uint32_t ci = i / c->spc;
    if (ci < c->bm_ci) { c->bm_ci = 0; c->bm_clus = c->bm_first; }
    for (uint32_t n = 0; c->bm_ci < ci; n++) {
        uint32_t nx;
        if (n >= STEP_ENTRIES) return 0;
        if (xfat_get(c, c->bm_clus, &nx) != 0) return -1;
        if (nx < 2u || nx >= c->clusters + 2u) return -2;
        c->bm_clus = nx;
        c->bm_ci++;
    }
    c->bm_lba = clus_lba(c, c->bm_clus, i % c->spc);
    c->bm_i = i;
    if (c->ops.read(c->ops.io, c->bm_lba, c->buf, 1) != 0) { c->bm_lba = NONE; return -1; }
    return 1;
}

static int bm_rc(int r) { return r == -1 ? MSC_FLASH_E_IO : MSC_FLASH_E_FS; }

/* "FIRMWARE.BIN" from the 8.3 name; returns its length (at most 12). */
static uint32_t xname(const msc_flash_ctx_t *c, char *out)
{
    uint32_t b = 8, e = 3, n;
    while (b > 0 && c->name83[b - 1] == ' ') b--;
    while (e > 0 && c->name83[8 + e - 1] == ' ') e--;
    memcpy(out, c->name83, b);
    n = b;
    if (e) { out[n++] = '.'; memcpy(out + n, c->name83 + 8, e); n += e; }
    return n;
}

static uint32_t xset_len(const msc_flash_ctx_t *c) { return 2u + (c->x_nlen + 14u) / 15u; }

/* Scan the root directory (a FAT chain), one sector per step: the allocation bitmap, a file of the
   same name, and a place for the new entry set (n free entries in one sector). */
static int xdir_plan(msc_flash_ctx_t *c);

static void xsecondary(msc_flash_ctx_t *c, const uint8_t *e)
{
    const uint8_t t = e[0];
    if (c->x_idx == 0) {
        c->x_ok = 0;
        c->x_pos = 0;
        if (t == 0xC0 && !c->found && c->x_file && e[3] == c->x_nlen && rd16(e + 4) == c->x_hash) {
            c->x_ok = 1;
            c->old_first = rd32(e + 20);
            c->old_flags = e[1];
            c->old_len = rd32(e + 28) != 0 ? NONE : rd32(e + 24);
        }
    } else if (t == 0xC1 && c->x_ok) {
        char nm[13];
        xname(c, nm);
        for (uint32_t k = 0; k < 15u && c->x_pos < c->x_nlen; k++, c->x_pos++) {
            uint32_t ch = rd16(e + 2 + 2 * k);
            if (ch < 0x80u) {                                /* ASCII: case-insensitive */
                if (ch >= 'a' && ch <= 'z') ch -= 32u;
                if (ch != (uint8_t)nm[c->x_pos]) c->x_ok = 0;
            }                                                /* else: the name hash matched */
        }
    }
    c->x_idx++;
    if (--c->x_rem == 0 && c->x_ok && c->x_pos == c->x_nlen && c->x_idx >= 2u) {
        c->found = 1;
        c->old_clus = c->cs_clus;
        c->old_sec = c->cs_sec;
        c->old_off = c->cs_off;
        c->old_cnt = (uint8_t)(1u + c->x_cnt);
    }
}

static int xdir_step(msc_flash_ctx_t *c)
{
    if (c->scan_end) return xdir_plan(c);
    const uint32_t lba = clus_lba(c, c->dir_clus, c->pos), need = xset_len(c);
    if (c->ops.read(c->ops.io, lba, c->buf, 1) != 0) return finish(c, MSC_FLASH_E_IO);
    uint32_t run = 0, run_off = 0;
    for (uint32_t off = 0; off < MSC_FLASH_SECTOR; off += 32u) {
        const uint8_t *e = c->buf + off;
        const uint8_t t = e[0];
        if (t == 0x00) {                                     /* end of the directory */
            c->x_mk = 1;
            c->mk_lba = lba;
            c->mk_off = off;
            c->mk_sec = c->pos;
            c->scan_end = 1;
            return MSC_FLASH_RUNNING;
        }
        if ((t & 0x80u) == 0) {                              /* an unused entry */
            c->x_rem = 0;
            if (c->dir_lba == NONE) {
                if (run++ == 0) run_off = off;
                if (run == need) { c->dir_lba = lba; c->dir_off = run_off; }
            }
            continue;
        }
        run = 0;
        if (c->x_rem && (t & 0x40u)) { xsecondary(c, e); continue; }
        c->x_rem = 0;
        if (t == 0x81) {                                     /* allocation bitmap */
            if ((e[1] & 1u) == c->x_active && rd32(e + 28) == 0) {
                c->bm_first = rd32(e + 20);
                c->bm_len = rd32(e + 24);
            }
        } else if (t == 0x85 && e[1] >= 2u && e[1] <= 18u) { /* a file or directory */
            c->x_rem = c->x_cnt = e[1];
            c->x_idx = 0;
            c->x_ok = 0;
            c->x_file = (rd16(e + 4) & 0x10u) == 0;
            c->cs_clus = c->dir_clus;
            c->cs_sec = c->pos;
            c->cs_off = off;
        }
    }
    if (++c->pos < c->spc) return MSC_FLASH_RUNNING;
    c->pos = 0;
    uint32_t nx;
    if (xfat_get(c, c->dir_clus, &nx) != 0) return finish(c, MSC_FLASH_E_IO);
    if (nx >= X_EOC) {
        c->dir_last = c->dir_clus;
        c->scan_end = 1;
        return MSC_FLASH_RUNNING;
    }
    /* A directory is at most 256 MB: a longer chain is broken or loops. */
    if (nx < 2u || nx >= c->clusters + 2u || ++c->dir_n * c->spc > 524288u) return finish(c, MSC_FLASH_E_FS);
    c->dir_clus = nx;
    return MSC_FLASH_RUNNING;
}

/* Where the entry set goes. A set must come right after the entries in use, so the end marker's
   place is taken; when the sector has no room for the set and a marker behind it, the rest of the
   sector is filled with unused entries and the set starts in the next one (zeroed). */
static int xdir_plan(msc_flash_ctx_t *c)
{
    const uint32_t n = xset_len(c);
    if (c->bm_first < 2u || c->bm_first >= c->clusters + 2u || c->bm_len < (c->clusters + 7u) / 8u)
        return finish(c, MSC_FLASH_E_FS);
    c->pad_lba = NONE;
    c->x_zero = 0;
    if (c->dir_lba == NONE) {
        if (c->x_mk) {
            if (16u - c->mk_off / 32u >= n + 1u) {
                c->dir_lba = c->mk_lba;
                c->dir_off = c->mk_off;
                c->x_zero = 1;
            } else {
                c->pad_lba = c->mk_lba;
                c->pad_off = c->mk_off;
                if (c->mk_sec + 1u < c->spc) {
                    c->dir_lba = c->mk_lba + 1u;
                    c->dir_off = 0;
                    c->x_zero = 2;
                } else {
                    uint32_t nx;
                    if (xfat_get(c, c->dir_clus, &nx) != 0) return finish(c, MSC_FLASH_E_IO);
                    if (nx < X_EOC) return finish(c, MSC_FLASH_E_FS);   /* clusters behind the marker */
                    c->dir_last = c->dir_clus;
                    c->dir_extend = 1;
                }
            }
        } else {
            c->dir_extend = 1;                               /* the chain is full */
        }
    }
    c->phase = MSC_PH_FAT_SCAN;
    c->pos = 0;
    c->run_len = 0;
    c->bm_lba = NONE;
    c->bm_ci = 0;
    c->bm_clus = c->bm_first;
    return MSC_FLASH_RUNNING;
}

/* A run of free clusters in the allocation bitmap, from the start: one pass, 8 sectors per step. */
static int xfind_step(msc_flash_ctx_t *c)
{
    const uint32_t want = c->need_clusters + c->dir_extend;
    for (uint32_t io = 0; io < STEP_SECTORS; io++) {
        if (c->pos >= c->clusters) return finish(c, MSC_FLASH_E_FULL);
        int r = bm_load(c, c->pos / X_BITS);
        if (r == 0) return MSC_FLASH_RUNNING;
        if (r < 0) return finish(c, bm_rc(r));
        uint32_t stop = (c->pos / X_BITS + 1u) * X_BITS;
        if (stop > c->clusters) stop = c->clusters;
        while (c->pos < stop) {
            const uint32_t b = c->pos % X_BITS;
            const uint8_t v = c->buf[b >> 3];
            if ((b & 7u) == 0 && v == 0xFFu && c->pos + 8u <= stop) { c->run_len = 0; c->pos += 8u; continue; }
            if (v & (1u << (b & 7u))) {
                c->run_len = 0;
            } else {
                if (c->run_len++ == 0) c->run_start = c->pos + 2u;
                if (c->run_len == want) {
                    if (c->dir_extend) {                     /* the entry goes in the new cluster */
                        c->dir_lba = clus_lba(c, c->run_start, 0);
                        c->dir_off = 0;
                        c->x_zero = 2;
                    }
                    c->bm_lba = NONE;
                    c->phase = c->dir_extend ? MSC_PH_DIR_ZERO : MSC_PH_DATA;
                    c->pos = 0;
                    return MSC_FLASH_RUNNING;
                }
            }
            c->pos++;
        }
    }
    return MSC_FLASH_RUNNING;
}

/* Mark the run (and the new directory cluster) used in the bitmap. */
static int xbitmap_step(msc_flash_ctx_t *c)
{
    const uint32_t want = c->need_clusters + c->dir_extend;
    for (uint32_t io = 0; io < 4u; io++) {
        if (c->pos >= want) {
            if (bm_flush(c) != 0) return finish(c, MSC_FLASH_E_IO);
            c->bm_lba = NONE;
            c->pos = 0;
            c->phase = c->dir_extend ? MSC_PH_XLINK : c->found ? MSC_PH_XUNUSE : MSC_PH_XSET;
            return MSC_FLASH_RUNNING;
        }
        const uint32_t bit = c->run_start - 2u + c->pos;
        int r = bm_load(c, bit / X_BITS);
        if (r == 0) return MSC_FLASH_RUNNING;
        if (r < 0) return finish(c, bm_rc(r));
        uint32_t k = want - c->pos, b = bit % X_BITS;
        if (k > X_BITS - b) k = X_BITS - b;
        for (uint32_t i = 0; i < k; i++, b++) c->buf[b >> 3] |= (uint8_t)(1u << (b & 7u));
        c->bm_dirty = 1;
        c->pos += k;
    }
    return MSC_FLASH_RUNNING;
}

/* The root directory grows: the old last cluster links to the new one, which ends the chain. */
static int xlink_step(msc_flash_ctx_t *c)
{
    uint8_t *p = xfat_at(c, c->run_start);
    if (!p) return finish(c, MSC_FLASH_E_IO);
    wr32(p, 0xFFFFFFFFu);
    c->fat_dirty = true;
    p = xfat_at(c, c->dir_last);
    if (!p) return finish(c, MSC_FLASH_E_IO);
    wr32(p, c->run_start);
    c->fat_dirty = true;
    if (xfat_flush(c) != 0) return finish(c, MSC_FLASH_E_IO);
    c->phase = c->found ? MSC_PH_XUNUSE : MSC_PH_XSET;
    c->pos = 0;
    return MSC_FLASH_RUNNING;
}

/* The replaced file's entry set: clear the InUse bit of each of its entries (it may span sectors). */
static int xunuse_step(msc_flash_ctx_t *c)
{
    const uint32_t lba = clus_lba(c, c->old_clus, c->old_sec);
    if (c->ops.read(c->ops.io, lba, c->buf, 1) != 0) return finish(c, MSC_FLASH_E_IO);
    uint32_t off = c->old_off;
    for (; off < MSC_FLASH_SECTOR && c->old_cnt; off += 32u, c->old_cnt--) c->buf[off] &= 0x7Fu;
    if (c->ops.write(c->ops.io, lba, c->buf, 1) != 0) return finish(c, MSC_FLASH_E_IO);
    if (c->old_cnt == 0) { c->phase = MSC_PH_XSET; c->pos = 0; return MSC_FLASH_RUNNING; }
    c->old_off = 0;
    if (++c->old_sec < c->spc) return MSC_FLASH_RUNNING;
    uint32_t nx;
    if (xfat_get(c, c->old_clus, &nx) != 0) return finish(c, MSC_FLASH_E_IO);
    if (nx < 2u || nx >= c->clusters + 2u) return finish(c, MSC_FLASH_E_FS);
    c->old_clus = nx;
    c->old_sec = 0;
    return MSC_FLASH_RUNNING;
}

/* The new entry set: File (0x85), Stream Extension (0xC0), File Name (0xC1) entries; one sector. */
static uint32_t xbuild(const msc_flash_ctx_t *c, uint8_t *set)
{
    char nm[13];
    const uint32_t n = xset_len(c), nn = n - 2u;
    uint32_t fd, ft, ts;
    stamp(c, &fd, &ft, &ts);
    memset(set, 0, n * 32u);
    set[0] = 0x85; set[1] = (uint8_t)(1u + nn);
    wr16(set + 4, 0x20);                                         /* archive */
    wr32(set + 8, ts); wr32(set + 12, ts); wr32(set + 16, ts);
    set[32] = 0xC0; set[33] = 0x03;                              /* allocation possible, no FAT chain */
    set[35] = (uint8_t)c->x_nlen;
    wr16(set + 36, c->x_hash);
    wr32(set + 40, c->len);                                      /* valid data length */
    wr32(set + 52, c->run_start + c->dir_extend);
    wr32(set + 56, c->len);                                      /* data length */
    uint32_t len = xname(c, nm);
    for (uint32_t i = 0; i < len; i++) {
        set[64 + (i / 15u) * 32u + 0] = 0xC1;
        wr16(set + 64 + (i / 15u) * 32u + 2 + (i % 15u) * 2u, (uint8_t)nm[i]);
    }
    uint32_t sum = 0;
    for (uint32_t i = 0; i < n * 32u; i++) {
        if (i == 2 || i == 3) continue;
        sum = (((sum & 1u) ? 0x8000u : 0u) + (sum >> 1) + set[i]) & 0xFFFFu;
    }
    wr16(set + 2, sum);
    return n;
}

static uint32_t xhash(const char *nm, uint32_t len)
{
    uint32_t h = 0;
    for (uint32_t i = 0; i < len; i++) {
        h = (((h & 1u) ? 0x8000u : 0u) + (h >> 1) + (uint8_t)nm[i]) & 0xFFFFu;
        h = (((h & 1u) ? 0x8000u : 0u) + (h >> 1)) & 0xFFFFu;   /* the high byte is 0 */
    }
    return h;
}

static int xset_step(msc_flash_ctx_t *c)
{
    if (c->pos == 0) {
        uint8_t set[96];
        const uint32_t n = xbuild(c, set);
        if (c->x_zero == 2) memset(c->buf, 0, MSC_FLASH_SECTOR);
        else if (c->ops.read(c->ops.io, c->dir_lba, c->buf, 1) != 0) return finish(c, MSC_FLASH_E_IO);
        memcpy(c->buf + c->dir_off, set, n * 32u);
        if (c->x_zero == 1) memset(c->buf + c->dir_off + n * 32u, 0, MSC_FLASH_SECTOR - c->dir_off - n * 32u);
        if (c->ops.write(c->ops.io, c->dir_lba, c->buf, 1) != 0) return finish(c, MSC_FLASH_E_IO);
        if (c->pad_lba != NONE) { c->pos = 1; return MSC_FLASH_RUNNING; }
    } else {   /* fill the end of the old last sector with unused entries: this makes the set reachable */
        if (c->ops.read(c->ops.io, c->pad_lba, c->buf, 1) != 0) return finish(c, MSC_FLASH_E_IO);
        for (uint32_t off = c->pad_off; off < MSC_FLASH_SECTOR; off += 32u) {
            memset(c->buf + off, 0, 32);
            c->buf[off] = 0x05;
        }
        if (c->ops.write(c->ops.io, c->pad_lba, c->buf, 1) != 0) return finish(c, MSC_FLASH_E_IO);
    }
    if (!c->found || (c->old_flags & 1u) == 0 || c->old_first < 2u || c->old_first >= c->clusters + 2u)
        return finish(c, MSC_FLASH_DONE);
    const uint32_t csize = (uint32_t)c->spc * MSC_FLASH_SECTOR;
    c->free_cur = c->old_first;
    c->free_n = 0;
    c->old_len = c->old_len == NONE ? c->clusters : (c->old_len + csize - 1u) / csize;   /* now: clusters */
    c->phase = MSC_PH_XFREE;
    c->pos = 0;
    c->bm_lba = NONE;
    c->bm_ci = 0;
    c->bm_clus = c->bm_first;
    return MSC_FLASH_RUNNING;
}

/* Free the replaced file's clusters: their bitmap bits, and the FAT chain unless NoFatChain. */
static int xfree_step(msc_flash_ctx_t *c)
{
    const bool contig = (c->old_flags & 2u) != 0;
    uint32_t io = 0;
    for (uint32_t i = 0; i < STEP_FREE; i++) {
        bool end = c->free_n >= c->clusters || c->free_cur < 2u || c->free_cur >= c->clusters + 2u ||
                   (contig && c->free_n >= c->old_len);
        if (end) {
            if (bm_flush(c) != 0 || xfat_flush(c) != 0) return finish(c, MSC_FLASH_E_IO);
            return finish(c, MSC_FLASH_DONE);
        }
        const uint32_t bit = c->free_cur - 2u;
        if (c->bm_lba == NONE || c->bm_i != bit / X_BITS) {
            if (++io > 3u) return MSC_FLASH_RUNNING;
            int r = bm_load(c, bit / X_BITS);
            if (r == 0) return MSC_FLASH_RUNNING;
            if (r < 0) return finish(c, bm_rc(r));
        }
        c->buf[(bit % X_BITS) >> 3] &= (uint8_t)~(1u << (bit & 7u));
        c->bm_dirty = 1;
        uint32_t nx = c->free_cur + 1u;
        if (!contig) {
            uint8_t *p = xfat_at(c, c->free_cur);
            if (!p) return finish(c, MSC_FLASH_E_IO);
            nx = rd32(p);
            wr32(p, 0);
            c->fat_dirty = true;
            if (nx >= X_EOC || nx == 0) nx = 0;               /* the end: a free cluster ends the walk too */
        }
        c->free_cur = nx;
        c->free_n++;
    }
    return MSC_FLASH_RUNNING;
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
    c->found = c->vf_stage = 0;
    c->x_rem = c->x_mk = c->x_zero = c->bm_dirty = 0;
    c->bm_lba = NONE;
    c->bm_ci = c->bm_first = c->bm_len = 0;
    c->out_len = 0;
    if (c->ops.block_size != MSC_FLASH_SECTOR) return finish(c, MSC_FLASH_E_SECTOR);
    if (c->mode == MSC_FLASH_READ || c->mode == MSC_FLASH_VERIFY) {
        if (c->name83[0] == ' ' || c->name83[0] == 0 || c->ops.block_count == 0) return finish(c, MSC_FLASH_E_ARGS);
        if (c->mode == MSC_FLASH_VERIFY) {
            if (c->len == 0 || c->len % MSC_FLASH_SECTOR != 0) return finish(c, MSC_FLASH_E_NOT_UF2);
            c->total = c->len / MSC_FLASH_SECTOR;
        }
        int g = read_geometry(c);
        if (g != 0) return finish(c, g);
        if (c->is_exfat) return finish(c, MSC_FLASH_E_EXFAT);   /* no read-back on exFAT */
        c->phase = MSC_PH_FIND;
        c->dir_clus = c->root_clus;
        return MSC_FLASH_RUNNING;
    }
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
    c->dir_clus = c->root_clus;                              /* FAT32, exFAT: the root chain starts here */
    if (c->is_exfat) {
        char nm[13];
        c->x_nlen = (uint16_t)xname(c, nm);
        c->x_hash = (uint16_t)xhash(nm, c->x_nlen);
        c->bm_clus = 0;
    }
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
    case MSC_PH_FIND:     return find_step(c);
    case MSC_PH_READ:     return read_step(c);
    case MSC_PH_VERIFY:   return verify_step(c);
    case MSC_PH_XBITMAP:  return xbitmap_step(c);
    case MSC_PH_XLINK:    return xlink_step(c);
    case MSC_PH_XUNUSE:   return xunuse_step(c);
    case MSC_PH_XSET:     return xset_step(c);
    case MSC_PH_XFREE:    return xfree_step(c);
    default:              return c->result;
    }
}
