/* Host test for msc_flash (xip_mod_msc_flash.c, msc_flash_names.c): UF2 blocks reach a simulated
   UF2 bootloader whole and in order; a file lands on FAT12, FAT16 and FAT32 drives (with and
   without an MBR) next to an existing file and reads back; FAT32 also: a full or extended root
   directory chain, a replaced file, fragmented free space, the FSInfo hint and counts; exFAT volumes (bitmap, entry sets, checksums, replace, root growth, TexFAT);
   hostile drives and files are refused. */
#include "msc_flash.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures, checks;
#define CHECK(c) do { checks++; if (!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); failures++; } } while (0)

typedef struct {
    uint8_t *disk;
    uint32_t sectors;
    const uint8_t *file;
    uint32_t file_len;
    int fail_write_at;     /* nth write fails (0 = never) */
    int writes;
    uint32_t wlog[4096];   /* the LBA of each write, in order */
} rig_t;

static int rd(void *io, uint32_t lba, uint8_t *buf, uint32_t n)
{
    rig_t *r = io;
    if (lba + n > r->sectors) return -1;
    memcpy(buf, r->disk + (size_t)lba * 512, (size_t)n * 512);
    return 0;
}
static int wr(void *io, uint32_t lba, const uint8_t *buf, uint32_t n)
{
    rig_t *r = io;
    if (r->writes < 4096) r->wlog[r->writes] = lba;
    if (++r->writes == r->fail_write_at || lba + n > r->sectors) return -1;
    memcpy(r->disk + (size_t)lba * 512, buf, (size_t)n * 512);
    return 0;
}
static int src(void *io, uint32_t off, uint8_t *buf, uint32_t n)
{
    rig_t *r = io;
    if (off + n > r->file_len) return -1;
    memcpy(buf, r->file + off, n);
    return 0;
}

static void put16(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put32(uint8_t *p, uint32_t v) { put16(p, v); put16(p + 2, v >> 16); }
static uint32_t get16(const uint8_t *p) { return p[0] | (p[1] << 8); }
static uint32_t get32(const uint8_t *p) { return get16(p) | (get16(p + 2) << 16); }

typedef struct { uint32_t vol, spc, rsv, nfats, root_ent, fatsz, tot, bits; } geo_t;

/* A blank FAT volume at `vol` (an MBR in front when vol > 0), with INFO.TXT in clusters 2-3. */
static void mkfs(rig_t *r, geo_t g)
{
    memset(r->disk, 0, (size_t)r->sectors * 512);
    if (g.vol) {
        uint8_t *m = r->disk;
        m[446 + 4] = g.bits == 12 ? 0x01 : 0x06;
        put32(m + 446 + 8, g.vol);
        put32(m + 446 + 12, g.tot);
        m[510] = 0x55; m[511] = 0xAA;
    }
    uint8_t *b = r->disk + (size_t)g.vol * 512;
    b[0] = 0xEB; b[1] = 0x3C; b[2] = 0x90;
    put16(b + 11, 512); b[13] = (uint8_t)g.spc; put16(b + 14, g.rsv); b[16] = (uint8_t)g.nfats;
    put16(b + 17, g.root_ent);
    if (g.tot < 65536) put16(b + 19, g.tot); else put32(b + 32, g.tot);
    b[21] = 0xF8; put16(b + 22, g.fatsz);
    b[510] = 0x55; b[511] = 0xAA;
    for (uint32_t f = 0; f < g.nfats; f++) {
        uint8_t *fat = r->disk + (size_t)(g.vol + g.rsv + f * g.fatsz) * 512;
        if (g.bits == 12) { fat[0] = 0xF8; fat[1] = 0xFF; fat[2] = 0xFF;
                            /* cluster 2 -> 3, 3 -> EOC */ fat[3] = 0x03; fat[4] = 0xF0; fat[5] = 0xFF; }
        else { put16(fat, 0xFFF8); put16(fat + 2, 0xFFFF); put16(fat + 4, 3); put16(fat + 6, 0xFFFF); }
    }
    uint8_t *root = r->disk + (size_t)(g.vol + g.rsv + g.nfats * g.fatsz) * 512;
    memcpy(root, "BENCHPODVOL", 11); root[11] = 0x08;                 /* volume label */
    memcpy(root + 32, "INFO    TXT", 11); root[32 + 11] = 0x20;
    put16(root + 32 + 26, 2); put32(root + 32 + 28, g.spc * 512 + 100);   /* two clusters */
}

static uint32_t fat_get(const rig_t *r, geo_t g, uint32_t f, uint32_t n)
{
    const uint8_t *fat = r->disk + (size_t)(g.vol + g.rsv + f * g.fatsz) * 512;
    if (g.bits == 16) return get16(fat + n * 2);
    uint32_t v = get16(fat + n + n / 2);
    return (n & 1) ? v >> 4 : v & 0xFFF;
}

/* Read a file back through its directory entry and FAT chain (copy f). Returns its length, -1 if
   absent or broken; content into out. */
static long read_back(const rig_t *r, geo_t g, const char *name83, uint8_t *out, uint32_t f)
{
    uint32_t root_secs = (g.root_ent * 32 + 511) / 512;
    const uint8_t *root = r->disk + (size_t)(g.vol + g.rsv + g.nfats * g.fatsz) * 512;
    uint32_t data = g.vol + g.rsv + g.nfats * g.fatsz + root_secs;
    for (uint32_t i = 0; i < g.root_ent; i++) {
        const uint8_t *e = root + i * 32;
        if (memcmp(e, name83, 11) != 0) continue;
        uint32_t len = get32(e + 28), c = get16(e + 26), got = 0;
        uint32_t eoc = g.bits == 12 ? 0xFF8 : 0xFFF8;
        while (got < len) {
            if (c < 2 || c >= eoc) return -1;
            for (uint32_t s = 0; s < g.spc && got < len; s++) {
                uint32_t n = len - got < 512 ? len - got : 512;
                memcpy(out + got, r->disk + (size_t)(data + (c - 2) * g.spc + s) * 512, n);
                got += n;
            }
            c = fat_get(r, g, f, c);
        }
        return c >= eoc ? (long)len : -1;
    }
    return -1;
}

static int run(msc_flash_ctx_t *c, int *steps)
{
    int rc, n = 0;
    while ((rc = msc_flash_step(c)) == MSC_FLASH_RUNNING) if (++n > 100000) return -99;
    if (steps) *steps = n;
    return rc;
}

static void ctx_init(msc_flash_ctx_t *c, rig_t *r, msc_flash_mode_t mode, const char *name, uint32_t len)
{
    memset(c, 0, sizeof(*c));
    c->ops = (msc_flash_ops_t){ r, rd, wr, src, r->sectors, 512 };
    c->mode = mode;
    c->len = len;
    if (name) CHECK(msc_flash_name83(name, c->name83));
}

static void test_file(geo_t g, uint32_t len)
{
    rig_t r = { calloc(g.vol + g.tot + 8, 512), g.vol + g.tot + 8, NULL, len, 0, 0 };
    uint8_t *file = malloc(len), *back = malloc(len + 512);
    for (uint32_t i = 0; i < len; i++) file[i] = (uint8_t)(i * 7 + (i >> 9));
    r.file = file;
    mkfs(&r, g);
    msc_flash_ctx_t c;
    ctx_init(&c, &r, MSC_FLASH_FILE, "firmware.bin", len);
    int steps = 0;
    int rc = run(&c, &steps);
    CHECK(rc == MSC_FLASH_DONE);
    if (rc != MSC_FLASH_DONE) printf("  FAT%u vol %u: %s\n", g.bits, g.vol, msc_flash_rc_str(rc));
    CHECK(c.fat_bits == g.bits);
    CHECK(c.done == (len + 511) / 512 && c.total == c.done);
    for (uint32_t f = 0; f < g.nfats; f++) {
        CHECK(read_back(&r, g, "FIRMWAREBIN", back, f) == (long)len);
        CHECK(memcmp(back, file, len) == 0);
    }
    uint8_t info[2 * 128 * 512];
    CHECK(read_back(&r, g, "INFO    TXT", info, 0) == (long)(g.spc * 512 + 100));   /* untouched */
    CHECK(c.run_start >= 4);                                     /* not over INFO.TXT's clusters */
    /* The FAT chain and the directory entry are written before the first data sector (a debugger
       drive such as Microchip's nEDBG drops data no directory entry points to yet). */
    {
        uint32_t root = g.vol + g.rsv + g.nfats * g.fatsz, data = root + (g.root_ent * 32 + 511) / 512;
        int dir_at = -1, data_at = -1, fat_at = -1;
        for (int i = 0; i < r.writes && i < 4096; i++) {
            uint32_t lba = r.wlog[i];
            if (lba >= data && data_at < 0) data_at = i;
            if (lba >= root && lba < data) dir_at = i;
            if (lba >= g.vol + g.rsv && lba < root && fat_at < 0) fat_at = i;
        }
        CHECK(fat_at >= 0 && dir_at > fat_at && data_at > dir_at);
    }

    /* Again with the same name: it takes the entry over and the file reads back again. */
    for (uint32_t i = 0; i < len; i++) file[i] ^= 0x5A;
    ctx_init(&c, &r, MSC_FLASH_FILE, "FIRMWARE.BIN", len);
    CHECK(run(&c, NULL) == MSC_FLASH_DONE);
    CHECK(read_back(&r, g, "FIRMWAREBIN", back, 0) == (long)len && memcmp(back, file, len) == 0);
    free(r.disk); free(file); free(back);
}

static void make_uf2(uint8_t *f, uint32_t nblocks, uint32_t base)
{
    for (uint32_t i = 0; i < nblocks; i++) {
        uint8_t *b = f + i * 512;
        memset(b, 0, 512);
        put32(b, 0x0A324655); put32(b + 4, 0x9E5D5157); put32(b + 8, 0x2000);
        put32(b + 12, base + i * 256); put32(b + 16, 256); put32(b + 20, i); put32(b + 24, nblocks);
        put32(b + 28, 0xE48BFF56);   /* RP2040 family */
        for (int k = 0; k < 256; k++) b[32 + k] = (uint8_t)(i + k);
        put32(b + 508, 0x0AB16F30);
    }
}

static void test_uf2(void)
{
    geo_t g = { 0, 8, 1, 2, 64, 2, 2048, 12 };   /* a small FAT12 drive like the Pico's */
    enum { nb = 40 };
    rig_t r = { calloc(4096, 512), 4096, NULL, nb * 512, 0, 0 };
    uint8_t *f = malloc(nb * 512);
    make_uf2(f, nb, 0x10000000);
    r.file = f;
    mkfs(&r, g);
    msc_flash_ctx_t c;
    ctx_init(&c, &r, MSC_FLASH_UF2, NULL, nb * 512);
    CHECK(run(&c, NULL) == MSC_FLASH_DONE);
    CHECK(c.data_base == 1 + 2 * 2 + 4);   /* the FAT data area: reserved + 2 FATs + root */
    /* The simulated bootloader: every UF2 block in any sector, payload placed by address. */
    uint8_t flash[nb * 256];
    memset(flash, 0, sizeof(flash));
    uint32_t seen = 0;
    for (uint32_t s = 0; s < r.sectors; s++) {
        const uint8_t *b = r.disk + (size_t)s * 512;
        if (get32(b) != 0x0A324655 || get32(b + 508) != 0x0AB16F30) continue;
        memcpy(flash + (get32(b + 12) - 0x10000000), b + 32, get32(b + 16));
        seen++;
    }
    CHECK(seen == nb);
    for (uint32_t i = 0; i < nb; i++) CHECK(flash[i * 256 + 7] == (uint8_t)(i + 7));
    CHECK(get16(r.disk + 510) == 0xAA55);   /* the boot sector is still there */

    /* A drive with no file system at all: blocks go from sector 0. */
    memset(r.disk, 0, (size_t)r.sectors * 512);
    ctx_init(&c, &r, MSC_FLASH_UF2, NULL, nb * 512);
    CHECK(run(&c, NULL) == MSC_FLASH_DONE && c.data_base == 0);

    /* A bad block in the middle: refused there, earlier blocks written. */
    f[20 * 512 + 3] ^= 0xFF;
    ctx_init(&c, &r, MSC_FLASH_UF2, NULL, nb * 512);
    CHECK(run(&c, NULL) == MSC_FLASH_E_NOT_UF2 && c.done == 20);
    /* Not whole blocks. */
    ctx_init(&c, &r, MSC_FLASH_UF2, NULL, nb * 512 - 1);
    CHECK(run(&c, NULL) == MSC_FLASH_E_NOT_UF2);
    free(r.disk); free(f);
}

static void test_refusals(void)
{
    geo_t g = { 0, 1, 1, 2, 16, 1, 256, 12 };
    rig_t r = { calloc(512, 512), 512, NULL, 0, 0, 0 };
    uint8_t file[4096] = { 1 };
    r.file = file; r.file_len = sizeof(file);
    msc_flash_ctx_t c;

    mkfs(&r, g);
    ctx_init(&c, &r, MSC_FLASH_FILE, "A.BIN", sizeof(file));
    c.ops.block_size = 4096;
    CHECK(run(&c, NULL) == MSC_FLASH_E_SECTOR);

    put16(r.disk + 11, 1024);                       /* 1 KB sectors in the BPB */
    ctx_init(&c, &r, MSC_FLASH_FILE, "A.BIN", sizeof(file));
    CHECK(run(&c, NULL) == MSC_FLASH_E_SECTOR);

    mkfs(&r, g);
    put16(r.disk + 17, 0); put16(r.disk + 22, 0);   /* FAT32-shaped: no fixed root, fatsz16 0 */
    ctx_init(&c, &r, MSC_FLASH_FILE, "A.BIN", sizeof(file));
    CHECK(run(&c, NULL) == MSC_FLASH_E_FS);

    mkfs(&r, g);
    put16(r.disk + 19, 60000);                      /* more sectors than the drive has */
    ctx_init(&c, &r, MSC_FLASH_FILE, "A.BIN", sizeof(file));
    CHECK(run(&c, NULL) == MSC_FLASH_E_FS);

    mkfs(&r, g);                                    /* a full root directory */
    uint8_t *root = r.disk + (size_t)(1 + 2) * 512;
    for (int i = 2; i < 16; i++) { memcpy(root + i * 32, "FULL    TXT", 11); root[i * 32 + 11] = 0x20; }
    ctx_init(&c, &r, MSC_FLASH_FILE, "A.BIN", sizeof(file));
    CHECK(run(&c, NULL) == MSC_FLASH_E_FULL);

    mkfs(&r, g);                                    /* larger than the free space */
    ctx_init(&c, &r, MSC_FLASH_FILE, "A.BIN", 300 * 512);
    r.file_len = 300 * 512;
    CHECK(run(&c, NULL) == MSC_FLASH_E_FULL);
    r.file_len = sizeof(file);

    mkfs(&r, g);                                    /* a write fails half way */
    r.writes = 0; r.fail_write_at = 3;
    ctx_init(&c, &r, MSC_FLASH_FILE, "A.BIN", sizeof(file));
    CHECK(run(&c, NULL) == MSC_FLASH_E_IO);
    r.fail_write_at = 0;

    ctx_init(&c, &r, MSC_FLASH_FILE, "A.BIN", 0);
    CHECK(run(&c, NULL) == MSC_FLASH_E_ARGS);

    /* Random boot sectors: no crash, an answer every time. */
    srand(7);
    for (int n = 0; n < 3000; n++) {
        for (int i = 0; i < 512; i++) r.disk[i] = (uint8_t)rand();
        if (n & 1) { r.disk[510] = 0x55; r.disk[511] = 0xAA; }
        if (n & 2) { put16(r.disk + 11, 512); r.disk[13] = 1 << (rand() % 8); }
        ctx_init(&c, &r, (n & 4) ? MSC_FLASH_UF2 : MSC_FLASH_FILE, "A.BIN", (n & 4) ? 512 : sizeof(file));
        int rc = run(&c, NULL);
        CHECK(rc <= 0 && rc != -99);
    }
    free(r.disk);
}

/* ---- FAT32 --------------------------------------------------------------------------------- */

typedef struct {
    uint32_t vol, spc, tot, nfats, ext;     /* in: volume start (MBR when > 0), cluster sectors, size */
    uint32_t hint, fsi;                     /* in: FSInfo next free (0 = 2), 0 = no FSInfo sector */
    uint32_t rsv, fatsz, clusters, data;    /* computed */
} g32_t;

typedef struct { rig_t r; g32_t g; uint8_t *file, *back; uint32_t len; } t32_t;

static uint8_t *f32p(const t32_t *t, uint32_t f, uint32_t n)
{
    return t->r.disk + (size_t)(t->g.vol + t->g.rsv + f * t->g.fatsz) * 512 + n * 4;
}
static uint32_t f32get(const t32_t *t, uint32_t f, uint32_t n) { return get32(f32p(t, f, n)) & 0x0FFFFFFF; }
static void f32set(t32_t *t, uint32_t n, uint32_t v)
{
    for (uint32_t f = 0; f < t->g.nfats; f++) put32(f32p(t, f, n), v);
}
static uint8_t *clus32(const t32_t *t, uint32_t c)
{
    return t->r.disk + (size_t)(t->g.data + (c - 2) * t->g.spc) * 512;
}

/* FSInfo = what the FAT says (free count) and the given hint. */
static void fsinfo_sync(t32_t *t)
{
    if (!t->g.fsi) return;
    uint32_t zero = 0;
    for (uint32_t n = 2; n < t->g.clusters + 2; n++) zero += f32get(t, 0, n) == 0;
    uint8_t *i = t->r.disk + (size_t)(t->g.vol + t->g.fsi) * 512;
    put32(i + 488, zero);
    put32(i + 492, t->g.hint);
}

static void mkfs32(t32_t *t, g32_t g, uint32_t len)
{
    memset(t, 0, sizeof(*t));
    g.rsv = 32;
    g.fatsz = 1;
    for (int k = 0; k < 4; k++) {
        g.clusters = (g.tot - g.rsv - g.nfats * g.fatsz) / g.spc;
        g.fatsz = ((g.clusters + 2) * 4 + 511) / 512;
    }
    g.clusters = (g.tot - g.rsv - g.nfats * g.fatsz) / g.spc;
    g.data = g.vol + g.rsv + g.nfats * g.fatsz;
    t->g = g;
    t->len = len;
    t->r.sectors = g.vol + g.tot + 8;
    t->r.disk = calloc(t->r.sectors, 512);
    t->r.file_len = len;
    t->file = malloc(len + 1);
    t->back = malloc(len + 4096);
    for (uint32_t i = 0; i < len; i++) t->file[i] = (uint8_t)(i * 13 + (i >> 8));
    t->r.file = t->file;
    if (g.vol) {
        uint8_t *m = t->r.disk;
        m[446 + 4] = 0x0C;
        put32(m + 446 + 8, g.vol);
        put32(m + 446 + 12, g.tot);
        m[510] = 0x55; m[511] = 0xAA;
    }
    uint8_t *b = t->r.disk + (size_t)g.vol * 512;
    b[0] = 0xEB; b[1] = 0x58; b[2] = 0x90;
    memcpy(b + 3, "MSDOS5.0", 8);
    put16(b + 11, 512); b[13] = (uint8_t)g.spc; put16(b + 14, g.rsv); b[16] = (uint8_t)g.nfats;
    b[21] = 0xF8;
    put32(b + 32, g.tot); put32(b + 36, g.fatsz);
    put16(b + 40, g.ext); put32(b + 44, 2); put16(b + 48, g.fsi ? g.fsi : 0xFFFF);
    b[510] = 0x55; b[511] = 0xAA;
    if (g.fsi) {
        uint8_t *i = t->r.disk + (size_t)(g.vol + g.fsi) * 512;
        put32(i, 0x41615252); put32(i + 484, 0x61417272); put32(i + 508, 0xAA550000);
    }
    f32set(t, 0, 0x0FFFFFF8);
    f32set(t, 1, 0x0FFFFFFF);
    f32set(t, 2, 0x0FFFFFFF);                  /* the root directory: one cluster */
    f32set(t, 3, 4);                           /* INFO.TXT: clusters 3 and 4 */
    f32set(t, 4, 0x0FFFFFFF);
    uint8_t *root = clus32(t, 2);
    memcpy(root, "BENCHPODVOL", 11); root[11] = 0x08;
    memcpy(root + 32, "INFO    TXT", 11); root[32 + 11] = 0x20;
    put16(root + 32 + 26, 3); put32(root + 32 + 28, g.spc * 512 + 100);
    fsinfo_sync(t);
}

static void free32(t32_t *t) { free(t->r.disk); free(t->file); free(t->back); }

static uint32_t last_cluster, first_cluster, root_chain_len;
static uint32_t v_first, v_chain;   /* what verify32 saw for the file */

/* The file `name83` through the root chain and FAT copy f. Its length, -1 if absent or broken. */
static long read_back32(t32_t *t, const char *name83, uint8_t *out, uint32_t f)
{
    const g32_t *g = &t->g;
    root_chain_len = 0;
    uint32_t dc = 2;
    for (uint32_t guard = 0; dc >= 2 && dc < 0x0FFFFFF8 && guard < 100; guard++) {
        root_chain_len++;
        for (uint32_t i = 0; i < g->spc * 16; i++) {
            const uint8_t *e = clus32(t, dc) + i * 32;
            if (e[0] == 0) break;
            if (memcmp(e, name83, 11) != 0 || (e[11] & 0x18)) continue;
            uint32_t len = get32(e + 28), c = (get16(e + 20) << 16) | get16(e + 26), got = 0;
            first_cluster = c;
            last_cluster = c;
            while (got < len) {
                if (c < 2 || c >= g->clusters + 2) return -1;
                last_cluster = c;
                for (uint32_t s = 0; s < g->spc && got < len; s++) {
                    uint32_t n = len - got < 512 ? len - got : 512;
                    memcpy(out + got, t->r.disk + (size_t)(g->data + (c - 2) * g->spc + s) * 512, n);
                    got += n;
                }
                c = f32get(t, f, c);
            }
            return c >= 0x0FFFFFF8 ? (long)len : -1;
        }
        dc = f32get(t, f, dc);
    }
    return -1;
}

static int run32(t32_t *t, const char *name, uint32_t len, msc_flash_ctx_t *c)
{
    t->r.file_len = len;
    ctx_init(c, &t->r, MSC_FLASH_FILE, name, len);
    return run(c, NULL);
}

/* After a good write: the file, INFO.TXT, FAT copies, FSInfo. */
static void verify32(t32_t *t, const char *name83, uint32_t len)
{
    uint32_t first = t->g.ext & 0x80 ? t->g.ext & 0xF : 0, end = t->g.ext & 0x80 ? first + 1 : t->g.nfats;
    for (uint32_t f = first; f < end; f++) {
        CHECK(read_back32(t, name83, t->back, f) == (long)len);
        CHECK(memcmp(t->back, t->file, len) == 0);
    }
    uint32_t last = last_cluster;
    v_first = first_cluster;
    v_chain = root_chain_len;
    uint8_t info[2 * 8 * 512];
    CHECK(read_back32(t, "INFO    TXT", info, first) == (long)(t->g.spc * 512 + 100));
    for (uint32_t f = 1; f < t->g.nfats && !(t->g.ext & 0x80); f++)
        CHECK(memcmp(f32p(t, 0, 0), f32p(t, f, 0), (size_t)t->g.fatsz * 512) == 0);
    if (t->g.fsi) {
        const uint8_t *i = t->r.disk + (size_t)(t->g.vol + t->g.fsi) * 512;
        uint32_t zero = 0;
        for (uint32_t n = 2; n < t->g.clusters + 2; n++) zero += f32get(t, first, n) == 0;
        CHECK(get32(i + 488) == zero);
        CHECK(get32(i + 492) == last);
        CHECK(get32(i) == 0x41615252 && get32(i + 508) == 0xAA550000);
    }
}

static void test_fat32_basic(g32_t g, uint32_t len)
{
    t32_t t;
    mkfs32(&t, g, len);
    msc_flash_ctx_t c;
    int rc = run32(&t, "firmware.bin", len, &c);
    CHECK(rc == MSC_FLASH_DONE);
    if (rc != MSC_FLASH_DONE) printf("  FAT32 spc %u vol %u: %s\n", g.spc, g.vol, msc_flash_rc_str(rc));
    CHECK(c.fat_bits == 32 && c.clusters == t.g.clusters && c.data_start == t.g.data);
    CHECK(c.done == (len + 511) / 512 && c.total == c.done);
    verify32(&t, "FIRMWAREBIN", len);
    CHECK(v_first >= 5);
    CHECK(v_chain == 1);

    /* Again, same name, other size and content: replaced, the old clusters are free again. */
    uint32_t len2 = len / 3 + 1;
    for (uint32_t i = 0; i < len2; i++) t.file[i] ^= 0x5A;
    CHECK(run32(&t, "FIRMWARE.BIN", len2, &c) == MSC_FLASH_DONE);
    verify32(&t, "FIRMWAREBIN", len2);
    /* A third time, bigger than the first: it must fit where the old file was. */
    uint32_t len3 = len + 5000;
    free(t.back); t.back = malloc(len3 + 4096);
    t.file = realloc(t.file, len3); t.r.file = t.file;
    for (uint32_t i = 0; i < len3; i++) t.file[i] = (uint8_t)(i * 3);
    CHECK(run32(&t, "FIRMWARE.BIN", len3, &c) == MSC_FLASH_DONE);
    verify32(&t, "FIRMWAREBIN", len3);
    free32(&t);
}

/* Every slot of the root directory chain taken (files, LFN pieces): the chain grows by a cluster. */
static void fill_dir(t32_t *t, uint32_t clus, uint32_t from)
{
    uint8_t *d = clus32(t, clus);
    for (uint32_t i = from; i < t->g.spc * 16; i++) {
        uint8_t *e = d + i * 32;
        if (i % 3 == 0) { memset(e, 0, 32); e[0] = 0x41; e[11] = 0x0F; memcpy(e + 1, "a\0b\0c\0", 6); }
        else { memcpy(e, "FILL0000TXT", 11); e[4] = (uint8_t)('0' + i / 100 % 10); e[5] = (uint8_t)('0' + i / 10 % 10);
               e[6] = (uint8_t)('0' + i % 10); e[11] = 0x20; put16(e + 26, 3); put32(e + 28, 10); }
    }
}

static void test_fat32_dir(g32_t g)
{
    uint32_t len = 20000;
    t32_t t;
    mkfs32(&t, g, len);
    fill_dir(&t, 2, 2);
    msc_flash_ctx_t c;
    CHECK(run32(&t, "a.bin", len, &c) == MSC_FLASH_DONE);
    CHECK(c.dir_extend == 1);
    verify32(&t, "A       BIN", len);
    CHECK(v_chain == 2);
    uint32_t nc = f32get(&t, 0, 2);
    CHECK(nc >= 5 && f32get(&t, 0, nc) == 0x0FFFFFFF);
    const uint8_t *d = clus32(&t, nc);                       /* only the new entry in it */
    CHECK(memcmp(d, "A       BIN", 11) == 0);
    int stray = 0;
    for (size_t i = 32; i < (size_t)g.spc * 512; i++) stray += d[i] != 0;
    CHECK(stray == 0);

    /* Fill the second cluster too (it has the one entry): extend again, to three clusters. */
    fill_dir(&t, nc, 1);
    CHECK(run32(&t, "b.bin", len, &c) == MSC_FLASH_DONE);
    verify32(&t, "B       BIN", len);
    CHECK(v_chain == 3);
    CHECK(read_back32(&t, "A       BIN", t.back, 0) == (long)len);   /* the first file is still there */

    /* A free slot (a deleted entry) in the first cluster is taken over, no growth. */
    clus32(&t, 2)[32 * 5] = 0xE5;
    CHECK(run32(&t, "c.bin", len, &c) == MSC_FLASH_DONE && c.dir_extend == 0);
    CHECK(memcmp(clus32(&t, 2) + 32 * 5, "C       BIN", 11) == 0);
    free32(&t);

    /* A root chain that loops, full: refused, not endless. */
    mkfs32(&t, g, len);
    fill_dir(&t, 2, 2);
    f32set(&t, 2, 2);
    CHECK(run32(&t, "a.bin", len, &c) == MSC_FLASH_E_FS);
    free32(&t);
}

static void test_fat32_alloc(void)
{
    g32_t g = { 0, 1, 70000, 2, 0, 0, 1, 0, 0, 0, 0 };
    msc_flash_ctx_t c;
    t32_t t;
    uint32_t len = 8 * 512;

    /* A fragmented FAT: every other cluster taken for a long way, the hint at the start. */
    mkfs32(&t, g, len);
    for (uint32_t n = 5; n < 3000; n += 2) f32set(&t, n, 0x0FFFFFFF);
    fsinfo_sync(&t);
    CHECK(run32(&t, "f.bin", len, &c) == MSC_FLASH_DONE);
    CHECK(c.run_start >= 3000 - 1);
    verify32(&t, "F       BIN", len);
    free32(&t);

    /* The hint in the middle of the fragments: the run is found after it. */
    g.hint = 1000;
    mkfs32(&t, g, len);
    for (uint32_t n = 5; n < 3000; n += 2) f32set(&t, n, 0x0FFFFFFF);
    fsinfo_sync(&t);
    CHECK(run32(&t, "f.bin", len, &c) == MSC_FLASH_DONE);
    CHECK(c.run_start >= 2999);
    verify32(&t, "F       BIN", len);
    free32(&t);

    /* Free space only before the hint: the search wraps. */
    g.hint = 50000;
    mkfs32(&t, g, len);
    for (uint32_t n = 50000; n < t.g.clusters + 2; n++) f32set(&t, n, 0x0FFFFFFF);
    fsinfo_sync(&t);
    CHECK(run32(&t, "f.bin", len, &c) == MSC_FLASH_DONE);
    CHECK(c.run_start == 5 && c.wrapped);
    verify32(&t, "F       BIN", len);
    free32(&t);

    /* A free run that starts just before the hint and goes on past it is found. */
    g.hint = 20000;
    mkfs32(&t, g, len);
    for (uint32_t n = 5; n < t.g.clusters + 2; n++)
        if (n < 19997 || n >= 20005) f32set(&t, n, 0x0FFFFFFF);
    fsinfo_sync(&t);
    CHECK(run32(&t, "f.bin", len, &c) == MSC_FLASH_DONE);
    CHECK(c.run_start == 19997);
    verify32(&t, "F       BIN", len);
    free32(&t);

    /* Hints that make no sense are ignored. */
    uint32_t bad[] = { 0, 1, 0x0FFFFFFF, 0xFFFFFFFF, 70000 };
    for (unsigned k = 0; k < sizeof(bad) / sizeof(bad[0]); k++) {
        g.hint = 1;
        mkfs32(&t, g, len);
        put32(t.r.disk + (size_t)(t.g.vol + t.g.fsi) * 512 + 492, bad[k]);
        CHECK(run32(&t, "f.bin", len, &c) == MSC_FLASH_DONE && c.run_start == 5);
        free32(&t);
    }

    /* The search is bounded per step. */
    g.hint = 1;
    mkfs32(&t, g, 4 * 512);
    for (uint32_t n = 5; n < t.g.clusters + 2; n++) if (n % 2) f32set(&t, n, 0x0FFFFFFF);
    for (uint32_t n = 6; n < t.g.clusters + 2; n += 2) f32set(&t, n, 0);
    fsinfo_sync(&t);
    int steps = 0, rc;
    t.r.file_len = 4 * 512;
    ctx_init(&c, &t.r, MSC_FLASH_FILE, "f.bin", 4 * 512);
    rc = 0;
    while ((rc = msc_flash_step(&c)) == MSC_FLASH_RUNNING) steps++;
    CHECK(rc == MSC_FLASH_E_FULL);
    CHECK(steps > 40 && steps < 200);       /* 65 K entries at 1024 per step */
    free32(&t);

    /* A full drive. */
    g.hint = 0;
    mkfs32(&t, g, len);
    for (uint32_t n = 5; n < t.g.clusters + 2; n++) f32set(&t, n, 0x0FFFFFFF);
    fsinfo_sync(&t);
    CHECK(run32(&t, "f.bin", len, &c) == MSC_FLASH_E_FULL);
    CHECK(read_back32(&t, "F       BIN", t.back, 0) == -1);
    /* One cluster too few. */
    f32set(&t, 100, 0); f32set(&t, 101, 0);
    CHECK(run32(&t, "f.bin", 3 * 512, &c) == MSC_FLASH_E_FULL);
    CHECK(run32(&t, "f.bin", 2 * 512, &c) == MSC_FLASH_DONE && c.run_start == 100);
    free32(&t);

    /* Larger than the whole volume. */
    mkfs32(&t, g, 40 * 1024 * 1024);
    CHECK(run32(&t, "f.bin", 40 * 1024 * 1024, &c) == MSC_FLASH_E_FULL);
    free32(&t);

    /* The root directory is full and the drive has no cluster left for it. */
    mkfs32(&t, g, len);
    fill_dir(&t, 2, 2);
    for (uint32_t n = 5; n < t.g.clusters + 2; n++) f32set(&t, n, 0x0FFFFFFF);
    f32set(&t, 100, 0);
    CHECK(run32(&t, "f.bin", 512, &c) == MSC_FLASH_E_FULL);   /* needs 2: the file and the directory */
    free32(&t);
}

static void test_fat32_fsinfo(void)
{
    g32_t g = { 0, 1, 70000, 2, 0, 0, 1, 0, 0, 0, 0 };
    msc_flash_ctx_t c;
    t32_t t;
    uint32_t len = 30000;

    /* Each signature wrong in turn: the file is written, FSInfo is not touched. */
    uint32_t offs[] = { 0, 484, 510 };
    for (unsigned k = 0; k < 3; k++) {
        mkfs32(&t, g, len);
        uint8_t *i = t.r.disk + (size_t)(t.g.vol + t.g.fsi) * 512;
        i[offs[k]] ^= 0x55;
        uint8_t before[512];
        memcpy(before, i, 512);
        CHECK(run32(&t, "f.bin", len, &c) == MSC_FLASH_DONE);
        CHECK(memcmp(before, i, 512) == 0);
        CHECK(read_back32(&t, "F       BIN", t.back, 0) == (long)len && memcmp(t.back, t.file, len) == 0);
        free32(&t);
    }
    /* No FSInfo sector in the boot sector. */
    g.fsi = 0;
    mkfs32(&t, g, len);
    CHECK(run32(&t, "f.bin", len, &c) == MSC_FLASH_DONE);
    verify32(&t, "F       BIN", len);
    free32(&t);

    /* An unknown free count stays unknown. */
    g.fsi = 1;
    mkfs32(&t, g, len);
    put32(t.r.disk + (size_t)(t.g.vol + t.g.fsi) * 512 + 488, 0xFFFFFFFF);
    CHECK(run32(&t, "f.bin", len, &c) == MSC_FLASH_DONE);
    CHECK(get32(t.r.disk + (size_t)(t.g.vol + t.g.fsi) * 512 + 488) == 0xFFFFFFFF);
    free32(&t);

    /* Top 4 bits of FAT entries are kept. */
    mkfs32(&t, g, len);
    for (uint32_t n = 5; n < 200; n++) put32(f32p(&t, 0, n), 0xA0000000);
    for (uint32_t n = 5; n < 200; n++) put32(f32p(&t, 1, n), 0xA0000000);
    CHECK(run32(&t, "f.bin", len, &c) == MSC_FLASH_DONE);
    CHECK(c.run_start == 5);
    CHECK((get32(f32p(&t, 0, 5)) >> 28) == 0xA && (get32(f32p(&t, 1, 5)) >> 28) == 0xA);
    CHECK(read_back32(&t, "F       BIN", t.back, 1) == (long)len);
    free32(&t);

    /* One active FAT (mirroring off): only that copy is written. */
    g.ext = 0x81;
    mkfs32(&t, g, len);
    CHECK(run32(&t, "f.bin", len, &c) == MSC_FLASH_DONE);
    verify32(&t, "F       BIN", len);
    CHECK(f32get(&t, 0, 5) == 0);
    free32(&t);
    g.ext = 0;

    /* A write failing anywhere before the entry leaves no entry. */
    mkfs32(&t, g, len);
    CHECK(run32(&t, "f.bin", len, &c) == MSC_FLASH_DONE);
    int total = t.r.writes;
    free32(&t);
    CHECK(total > 10);
    for (int n = 1; n <= total; n++) {
        mkfs32(&t, g, len);
        t.r.fail_write_at = n;
        int rc = run32(&t, "f.bin", len, &c);
        CHECK(rc == MSC_FLASH_E_IO);
        if (n < total - 1) CHECK(read_back32(&t, "F       BIN", t.back, 0) == -1);   /* the entry is next to last */
        else CHECK(read_back32(&t, "F       BIN", t.back, 0) != -1 || n == total - 1);
        free32(&t);
    }

    /* Replacing a file whose chain is cut short or loops: no hang, no crash. */
    mkfs32(&t, g, len);
    CHECK(run32(&t, "f.bin", len, &c) == MSC_FLASH_DONE);
    f32set(&t, first_cluster + 1, first_cluster);        /* a loop inside the old chain */
    CHECK(run32(&t, "f.bin", len, &c) <= 0);
    free32(&t);
}

static void test_fat32_other(void)
{
    g32_t g = { 0, 1, 70000, 2, 0, 0, 1, 0, 0, 0, 0 };
    msc_flash_ctx_t c;
    t32_t t;

    g.vol = 0;

    /* A "FAT32" volume with too few clusters is not FAT32. */
    g32_t small = { 0, 1, 40000, 2, 0, 0, 1, 0, 0, 0, 0 };
    mkfs32(&t, small, 512);
    CHECK(run32(&t, "f.bin", 512, &c) == MSC_FLASH_E_FS);
    free32(&t);

    /* Broken fields: root cluster, FAT size, version. */
    uint32_t off[] = { 44, 44, 36, 42, 16 };
    uint32_t val[] = { 0, 0x0FFFFFF0, 1, 1, 0 };
    for (unsigned k = 0; k < 5; k++) {
        mkfs32(&t, g, 512);
        uint8_t *b = t.r.disk;
        if (off[k] == 16) b[16] = 0; else put32(b + off[k], val[k]);
        CHECK(run32(&t, "f.bin", 512, &c) == MSC_FLASH_E_FS);
        free32(&t);
    }

    /* UF2 on a FAT32 drive: the data area, boot sector and FATs stay. */
    for (int mbr = 0; mbr < 2; mbr++) {
        g.vol = mbr ? 63 : 0;
        enum { nb = 40 };
        mkfs32(&t, g, nb * 512);
        make_uf2(t.file, nb, 0x10000000);
        t.r.file_len = nb * 512;
        ctx_init(&c, &t.r, MSC_FLASH_UF2, NULL, nb * 512);
        CHECK(run(&c, NULL) == MSC_FLASH_DONE);
        CHECK(c.data_base == g.vol + 32 + 2 * t.g.fatsz && c.data_base == t.g.data);
        for (uint32_t i = 0; i < nb; i++)
            CHECK(memcmp(t.r.disk + (size_t)(c.data_base + i) * 512, t.file + i * 512, 512) == 0);
        CHECK(get16(t.r.disk + (size_t)g.vol * 512 + 510) == 0xAA55);
        CHECK(f32get(&t, 0, 4) == 0x0FFFFFFF);
        free32(&t);
    }
    g.vol = 0;

    /* Hostile FAT32 volumes: a few thousand random changes to a good one. */
    srand(11);
    for (int n = 0; n < 4000; n++) {
        mkfs32(&t, g, 5000);
        for (int k = 0; k < 4; k++) {
            uint32_t at = (uint32_t)(rand() % 60);
            t.r.disk[at] = (uint8_t)rand();
        }
        if (n & 1) put32(f32p(&t, 0, 2), (uint32_t)rand());
        if (n & 2) fill_dir(&t, 2, 2);
        int rc = run32(&t, "f.bin", 5000, &c);
        CHECK(rc <= 0 && rc != -99);
        free32(&t);
    }
}

static void test_fat32(void)
{
    test_fat32_basic((g32_t){ 0, 1, 70000, 2, 0, 0, 1, 0, 0, 0, 0 }, 70 * 1024 + 17);       /* 512 B clusters */
    test_fat32_basic((g32_t){ 2048, 1, 70000, 2, 0, 3000, 1, 0, 0, 0, 0 }, 100 * 1024 + 3);  /* behind an MBR, a hint */
    test_fat32_basic((g32_t){ 0, 8, 540000, 2, 0, 0, 1, 0, 0, 0, 0 }, 300 * 1024 + 3);        /* 4 KB clusters */
    test_fat32_basic((g32_t){ 63, 8, 540000, 1, 0, 0, 1, 0, 0, 0, 0 }, 9000);                /* one FAT, an MBR */
    test_fat32_dir((g32_t){ 0, 1, 70000, 2, 0, 0, 1, 0, 0, 0, 0 });
    test_fat32_dir((g32_t){ 0, 8, 540000, 2, 0, 0, 1, 0, 0, 0, 0 });
    test_fat32_alloc();
    test_fat32_fsinfo();
    test_fat32_other();
}

/* ---- READ and VERIFY (after a flash, on the drive that came back) ----------------------------- */

/* A file in the root directory of a FAT16 volume (FAT32 when f32 != NULL), clusters from `first`,
   in order or (frag) every other one first. Returns nothing; the chain goes in every FAT copy. */
static uint32_t next_free_cluster;
static void put_file(rig_t *r, geo_t g, t32_t *f32, const char *name83, const uint8_t *data, uint32_t len,
                     int frag, uint32_t *first_out)
{
    uint32_t spc = f32 ? f32->g.spc : g.spc, csize = spc * 512, ncl = (len + csize - 1) / csize;
    uint32_t base = next_free_cluster, order[4096];
    for (uint32_t i = 0; i < ncl; i++) order[i] = base + i;
    if (frag)   /* swap neighbours: the chain no longer ascends */
        for (uint32_t i = 0; i + 1 < ncl; i += 2) { uint32_t t = order[i]; order[i] = order[i + 1]; order[i + 1] = t; }
    next_free_cluster += ncl;
    for (uint32_t i = 0; i < ncl; i++) {
        uint32_t nx = i + 1 < ncl ? order[i + 1] : (f32 ? 0x0FFFFFFF : 0xFFFF);
        if (f32) f32set(f32, order[i], nx);
        else for (uint32_t k = 0; k < g.nfats; k++)
            put16(r->disk + (size_t)(g.vol + g.rsv + k * g.fatsz) * 512 + order[i] * 2, nx);
        uint8_t *dst = f32 ? clus32(f32, order[i])
                           : r->disk + (size_t)(g.vol + g.rsv + g.nfats * g.fatsz + (g.root_ent * 32 + 511) / 512 +
                                                (order[i] - 2) * g.spc) * 512;
        uint32_t off = i * csize, n = len - off < csize ? len - off : csize;
        memset(dst, 0, csize);
        memcpy(dst, data + off, n);
    }
    uint8_t *dir = f32 ? clus32(f32, 2)
                       : r->disk + (size_t)(g.vol + g.rsv + g.nfats * g.fatsz) * 512;
    uint32_t slots = f32 ? spc * 16 : g.root_ent;
    for (uint32_t i = 0; i < slots; i++) {
        uint8_t *e = dir + i * 32;
        if (e[0] != 0 && e[0] != 0xE5) continue;
        memset(e, 0, 32);
        memcpy(e, name83, 11); e[11] = 0x20;
        put16(e + 26, order[0]); put16(e + 20, f32 ? order[0] >> 16 : 0); put32(e + 28, len);
        break;
    }
    if (first_out) *first_out = order[0];
}

static void test_read_file(void)
{
    const char *text = "error: The transfer timed out\nsecond line\n";
    geo_t g16 = { 63, 4, 4, 2, 512, 40, 40000, 16 };
    for (int variant = 0; variant < 4; variant++) {
        rig_t r = { calloc(g16.vol + g16.tot + 8, 512), g16.vol + g16.tot + 8, NULL, 0, 0, 0 };
        t32_t t;
        uint8_t src_buf[1024] = {0};
        msc_flash_ctx_t c;
        rig_t *rr = &r;
        if (variant == 3) { g32_t g = { 2048, 1, 70000, 2, 0, 0, 1, 0, 0, 0, 0 }; mkfs32(&t, g, 512); rr = &t.r; }
        else mkfs(&r, g16);
        next_free_cluster = variant == 3 ? 5 : 4;
        uint8_t big[600]; memset(big, 'x', sizeof(big)); memcpy(big, text, strlen(text));
        if (variant == 1) {                       /* behind many other root entries */
            uint8_t *root = r.disk + (size_t)(g16.vol + g16.rsv + g16.nfats * g16.fatsz) * 512;
            for (int i = 2; i < 300; i++) { memcpy(root + i * 32, "OTHER   TXT", 11); root[i * 32 + 11] = 0x20; }
        }
        if (variant == 2) put_file(&r, g16, NULL, "FAIL    TXT", (const uint8_t *)text, 0, 0, NULL);   /* empty */
        else if (variant == 3) put_file(rr, g16, &t, "FAIL    TXT", big, sizeof(big), 0, NULL);
        else put_file(&r, g16, NULL, "FAIL    TXT", big, sizeof(big), 0, NULL);
        rr->file = src_buf;
        memset(&c, 0, sizeof(c));
        c.ops = (msc_flash_ops_t){ rr, rd, wr, src, rr->sectors, 512 };
        c.mode = MSC_FLASH_READ;
        CHECK(msc_flash_name83("fail.txt", c.name83));
        CHECK(run(&c, NULL) == MSC_FLASH_DONE);
        CHECK(c.found == 1);
        if (variant == 2) CHECK(c.out_len == 0);
        else { CHECK(c.out_len == 128); CHECK(memcmp(c.out, text, strlen(text)) == 0); }
        /* No such file. */
        memset(&c, 0, sizeof(c));
        c.ops = (msc_flash_ops_t){ rr, rd, wr, src, rr->sectors, 512 };
        c.mode = MSC_FLASH_READ;
        CHECK(msc_flash_name83("nothere.txt", c.name83));
        CHECK(run(&c, NULL) == MSC_FLASH_DONE && c.found == 0 && c.out_len == 0);
        /* A read-only mode never writes. */
        CHECK(rr->writes == 0);
        free(r.disk);                     /* the FAT16 rig is allocated in every variant */
        if (variant == 3) free32(&t);
    }

    /* The whole DAPLink story: a file write, the loader then adds FAIL.TXT (and the drive "comes back"). */
    uint32_t len = 30000;
    rig_t r = { calloc(g16.vol + g16.tot + 8, 512), g16.vol + g16.tot + 8, NULL, len, 0, 0 };
    uint8_t *file = calloc(len, 1);
    r.file = file;
    mkfs(&r, g16);
    msc_flash_ctx_t c;
    ctx_init(&c, &r, MSC_FLASH_FILE, "firmware.bin", len);
    CHECK(run(&c, NULL) == MSC_FLASH_DONE);
    next_free_cluster = 4 + (len + 2047) / 2048 + 2;
    put_file(&r, g16, NULL, "FAIL    TXT", (const uint8_t *)text, (uint32_t)strlen(text), 0, NULL);
    ctx_init(&c, &r, MSC_FLASH_READ, "fail.txt", 0);
    CHECK(run(&c, NULL) == MSC_FLASH_DONE && c.found && c.out_len == strlen(text));
    CHECK(memcmp(c.out, text, c.out_len) == 0);
    free(r.disk); free(file);
}

/* CURRENT.UF2 of a simulated RP2 loader: `nflash` blocks of 256 bytes from 0x10000000. */
static void make_current(uint8_t *f, uint32_t nflash)
{
    make_uf2(f, nflash, 0x10000000);
}

static void test_verify(void)
{
    enum { nflash = 400, nmine = 120 };
    for (int variant = 0; variant < 4; variant++) {
        uint8_t *cur = malloc(nflash * 512), *mine = malloc(nmine * 512);
        make_current(cur, nflash);
        /* Our file: blocks 50..169 of the flash (addresses from the same base), same payload as the
           drive holds, but its own block numbering. */
        make_uf2(mine, nmine, 0x10000000 + 50 * 256);
        for (uint32_t i = 0; i < nmine; i++) memcpy(mine + i * 512 + 32, cur + (50 + i) * 512 + 32, 256);
        geo_t g16 = { 63, 4, 4, 2, 512, 40, 40000, 16 };
        rig_t r = { calloc(g16.vol + g16.tot + 8, 512), g16.vol + g16.tot + 8, mine, nmine * 512, 0, 0 };
        t32_t t;
        rig_t *rr = &r;
        if (variant == 3) { g32_t g = { 0, 1, 70000, 2, 0, 0, 1, 0, 0, 0, 0 }; mkfs32(&t, g, nmine * 512);
                            rr = &t.r; rr->file = mine; }
        else mkfs(&r, g16);
        next_free_cluster = variant == 3 ? 5 : 4;
        put_file(rr, g16, variant == 3 ? &t : NULL, "CURRENTUF2", cur, nflash * 512, variant == 1, NULL);
        msc_flash_ctx_t c;
        #define VCTX() do { memset(&c, 0, sizeof(c)); c.ops = (msc_flash_ops_t){ rr, rd, wr, src, rr->sectors, 512 }; \
                            c.mode = MSC_FLASH_VERIFY; c.len = nmine * 512; memcpy(c.name83, "CURRENTUF2", 11); } while (0)
        VCTX();
        int steps = 0, rc = run(&c, &steps);
        CHECK(rc == MSC_FLASH_DONE && c.done == nmine);
        CHECK(rr->writes == 0);
        /* The same blocks in reverse order: the chain walk starts over. */
        uint8_t *rev = malloc(nmine * 512);
        for (uint32_t i = 0; i < nmine; i++) memcpy(rev + i * 512, mine + (nmine - 1 - i) * 512, 512);
        rr->file = rev;
        VCTX();
        CHECK(run(&c, NULL) == MSC_FLASH_DONE);
        rr->file = mine;
        /* One payload byte differs, in the middle and at the end of a block. */
        for (int where = 0; where < 2; where++) {
            mine[77 * 512 + 32 + (where ? 255 : 0)] ^= 0x01;
            VCTX();
            CHECK(run(&c, NULL) == MSC_FLASH_E_VERIFY && c.pos == 77);
            mine[77 * 512 + 32 + (where ? 255 : 0)] ^= 0x01;
        }
        /* A block the drive does not hold. */
        put32(mine + 5 * 512 + 12, 0x10000000 + 1000 * 256);
        VCTX();
        CHECK(run(&c, NULL) == MSC_FLASH_E_VERIFY);
        put32(mine + 5 * 512 + 12, 0x10000000 + 55 * 256);
        put32(mine + 6 * 512 + 12, 0x0F000000);                      /* below the file's base */
        VCTX();
        CHECK(run(&c, NULL) == MSC_FLASH_E_VERIFY);
        put32(mine + 6 * 512 + 12, 0x10000000 + 56 * 256);
        /* Not whole blocks, or not UF2 blocks. */
        VCTX(); c.len = nmine * 512 - 1;
        CHECK(run(&c, NULL) == MSC_FLASH_E_NOT_UF2);
        mine[9 * 512] ^= 0xFF;
        VCTX();
        CHECK(run(&c, NULL) == MSC_FLASH_E_NOT_UF2);
        mine[9 * 512] ^= 0xFF;
        /* No CURRENT.UF2 (the board left its loader): its own result. */
        VCTX(); memcpy(c.name83, "OTHER   UF2", 11);
        CHECK(run(&c, NULL) == MSC_FLASH_E_NOFILE);
        /* A file of that name that is no UF2 file. */
        memset(cur, 0x55, 512);
        put_file(rr, g16, variant == 3 ? &t : NULL, "NOTUF2  UF2", cur, 512, 0, NULL);
        VCTX(); memcpy(c.name83, "NOTUF2  UF2", 11);
        CHECK(run(&c, NULL) == MSC_FLASH_E_NOFILE);
        /* A write to the drive anywhere would be a bug. */
        CHECK(rr->writes == 0);
        free(cur); free(mine); free(rev);
        free(r.disk);                     /* the FAT16 rig is allocated in every variant */
        if (variant == 3) free32(&t);     /* t.file is mkfs32's own buffer (rr->file points at mine) */
    }
}

/* ---- exFAT --------------------------------------------------------------------------------- */

typedef struct {
    rig_t r;
    uint32_t vol, shift, tot, nfats, active;     /* in: volume start (MBR when > 0), cluster shift, sectors */
    uint32_t spc, fatoff, fatlen, heap, ncl;     /* computed */
    uint32_t bm1, bm_clus, root, first_free_entry;                 /* bitmap first cluster and cluster count, root cluster */
    uint8_t *file, *back;
} tx_t;

static uint8_t *xsec(tx_t *t, uint32_t s) { return t->r.disk + (size_t)(t->vol + s) * 512; }
static uint8_t *xclus(tx_t *t, uint32_t c, uint32_t sec) { return xsec(t, t->heap + (c - 2) * t->spc + sec); }
static uint32_t xfatget(tx_t *t, uint32_t n) { return get32(xsec(t, t->fatoff + t->active * t->fatlen) + n * 4); }
static void xfatset(tx_t *t, uint32_t n, uint32_t v)
{
    for (uint32_t f = 0; f < t->nfats; f++) put32(xsec(t, t->fatoff + f * t->fatlen) + n * 4, v);
}
static void xbit(tx_t *t, uint32_t c, int v)
{
    uint8_t *b = xclus(t, t->bm1, 0) + (c - 2) / 8;   /* the bitmap is contiguous in the tests */
    if (v) *b |= (uint8_t)(1u << ((c - 2) % 8)); else *b &= (uint8_t)~(1u << ((c - 2) % 8));
}
static int xbitget(tx_t *t, uint32_t c) { return (xclus(t, t->bm1, 0)[(c - 2) / 8] >> ((c - 2) % 8)) & 1; }

static uint32_t xsum(const uint8_t *set, uint32_t n)
{
    uint32_t s = 0;
    for (uint32_t i = 0; i < n * 32; i++) if (i != 2 && i != 3) s = (((s & 1) ? 0x8000 : 0) + (s >> 1) + set[i]) & 0xFFFF;
    return s;
}
static uint32_t xhashw(const uint16_t *nm, uint32_t n)
{
    uint32_t h = 0;
    for (uint32_t i = 0; i < n; i++) {
        uint16_t ch = nm[i] >= 'a' && nm[i] <= 'z' ? nm[i] - 32 : nm[i];
        h = (((h & 1) ? 0x8000 : 0) + (h >> 1) + (ch & 0xFF)) & 0xFFFF;
        h = (((h & 1) ? 0x8000 : 0) + (h >> 1) + (ch >> 8)) & 0xFFFF;
    }
    return h;
}

/* Put an entry set (name ASCII) at entry `idx` of the root cluster chain's first cluster... into the
   root at byte offset `at` (the caller keeps track). contig: NoFatChain. */
static uint32_t xput(uint8_t *at, const char *name, uint32_t first, uint32_t len, int nofat)
{
    uint16_t u[64];
    uint32_t nl = (uint32_t)strlen(name), nn = (nl + 14) / 15;
    for (uint32_t i = 0; i < nl; i++) u[i] = (uint8_t)name[i];
    memset(at, 0, (2 + nn) * 32);
    at[0] = 0x85; at[1] = (uint8_t)(1 + nn); put16(at + 4, 0x20);
    at[32] = 0xC0; at[33] = (uint8_t)(1 | (nofat ? 2 : 0)); at[35] = (uint8_t)nl;
    put16(at + 36, xhashw(u, nl)); put32(at + 40, len); put32(at + 52, first); put32(at + 56, len);
    for (uint32_t i = 0; i < nl; i++) { at[64 + (i / 15) * 32] = 0xC1; put16(at + 64 + (i / 15) * 32 + 2 + (i % 15) * 2, u[i]); }
    put16(at + 2, xsum(at, 2 + nn));
    return 2 + nn;
}

/* A blank exFAT volume: boot sector, FAT, bitmap (cluster 2, contiguous), up-case table, root, and
   INFO.TXT (two clusters, FAT chain). */
static void mkexfat(tx_t *t)
{
    memset(t->r.disk, 0, (size_t)t->r.sectors * 512);
    t->spc = 1u << t->shift;
    uint32_t vol = t->tot;
    t->fatoff = 24;
    uint32_t ncl_guess = vol / t->spc;
    t->fatlen = ((ncl_guess + 2) * 4 + 511) / 512;
    uint32_t heap = 24 + t->fatlen * t->nfats;
    t->heap = (heap + t->spc - 1) / t->spc * t->spc;
    t->ncl = (vol - t->heap) / t->spc;
    uint32_t bmbytes = (t->ncl + 7) / 8;
    t->bm_clus = (bmbytes + t->spc * 512 - 1) / (t->spc * 512);
    uint32_t bm0 = 2, bm_n = t->nfats;          /* TexFAT: a second bitmap */
    t->bm1 = bm0 + (t->active ? t->bm_clus : 0);   /* the active one */
    uint32_t up = 2 + t->bm_clus * bm_n, root = up + 1;
    t->root = root;
    if (t->vol) {
        uint8_t *m = t->r.disk;
        m[446 + 4] = 0x07; put32(m + 446 + 8, t->vol); put32(m + 446 + 12, vol);
        m[510] = 0x55; m[511] = 0xAA;
    }
    uint8_t *b = xsec(t, 0);
    b[0] = 0xEB; b[1] = 0x76; b[2] = 0x90; memcpy(b + 3, "EXFAT   ", 8);
    put32(b + 64, t->vol); put32(b + 72, vol);
    put32(b + 80, t->fatoff); put32(b + 84, t->fatlen); put32(b + 88, t->heap); put32(b + 92, t->ncl);
    put32(b + 96, root); put16(b + 104, 0x0100); put16(b + 106, t->active);
    b[108] = 9; b[109] = (uint8_t)t->shift; b[110] = (uint8_t)t->nfats; b[112] = 0xFF;
    b[510] = 0x55; b[511] = 0xAA;
    for (uint32_t f = 0; f < t->nfats; f++) {
        put32(xsec(t, t->fatoff + f * t->fatlen), 0xFFFFFFF8);
        put32(xsec(t, t->fatoff + f * t->fatlen) + 4, 0xFFFFFFFF);
    }
    for (uint32_t k = 0; k < bm_n * t->bm_clus; k++)
        xfatset(t, 2 + k, (k + 1) % t->bm_clus ? 3 + k : 0xFFFFFFFF);
    xfatset(t, up, 0xFFFFFFFF);
    xfatset(t, root, 0xFFFFFFFF);
    uint32_t info = root + 1;                   /* INFO.TXT: clusters info, info+1 (a FAT chain) */
    xfatset(t, info, info + 1);
    xfatset(t, info + 1, 0xFFFFFFFF);
    for (uint32_t c = 2; c <= info + 1; c++) xbit(t, c, 1);
    uint8_t *r = xclus(t, root, 0);
    uint32_t e = 0;
    r[0] = 0x83; r[1] = 1; put16(r + 2, 'T'); e++;
    for (uint32_t k = 0; k < bm_n; k++, e++) {
        r[e * 32] = 0x81; r[e * 32 + 1] = (uint8_t)k;
        put32(r + e * 32 + 20, 2 + k * t->bm_clus); put32(r + e * 32 + 24, bmbytes);
    }
    r[e * 32] = 0x82; put32(r + e * 32 + 20, up); put32(r + e * 32 + 24, 128); e++;
    e += xput(r + e * 32, "INFO.TXT", info, t->spc * 512 + 100, 0);
    t->first_free_entry = e;
    for (uint32_t i = 0; i < t->spc * 512 + 100; i++) xclus(t, info, 0)[i] = (uint8_t)(i * 5);
}

/* The file `name` through the root chain: sets its entries' checksum and hash, the data, the bitmap.
   Length, -1 absent, -2 a bad set. */
static long xread(tx_t *t, const char *name, uint8_t *out)
{
    uint32_t dc = t->root, nl = (uint32_t)strlen(name);
    for (uint32_t guard = 0; dc >= 2 && dc < 0xFFFFFFF8 && guard < 1000; guard++) {
        for (uint32_t i = 0; i < t->spc * 16; i++) {
            const uint8_t *e = xclus(t, dc, 0) + i * 32;
            if (e[0] == 0) return -1;
            if (e[0] != 0x85) continue;
            uint32_t cnt = e[1];
            /* the set may sit in one cluster only in the tests (spc*16 entries): check the bound */
            if (i + 1 + cnt > t->spc * 16) return -2;
            if (xsum(e, 1 + cnt) != get16(e + 2)) return -2;
            const uint8_t *st = e + 32;
            if (st[0] != 0xC0 || st[3] != nl) continue;
            uint16_t u[256];
            for (uint32_t k = 0; k < st[3]; k++) u[k] = (uint16_t)get16(e + 64 + (k / 15) * 32 + 2 + (k % 15) * 2);
            if (xhashw(u, st[3]) != get16(st + 4)) return -2;
            int same = 1;
            for (uint32_t k = 0; k < nl; k++) same &= (u[k] | 0x20) == (((uint8_t)name[k]) | 0x20);
            if (!same) continue;
            uint32_t len = get32(st + 24), c = get32(st + 20), got = 0;
            int nofat = (st[1] & 2) != 0;
            if (get32(st + 8) != len || get32(st + 28) || get32(st + 12)) return -2;
            while (got < len) {
                if (c < 2 || c >= t->ncl + 2 || !xbitget(t, c)) return -2;
                for (uint32_t s = 0; s < t->spc && got < len; s++) {
                    uint32_t n = len - got < 512 ? len - got : 512;
                    memcpy(out + got, xclus(t, c, s), n);
                    got += n;
                }
                c = nofat ? c + 1 : xfatget(t, c);
            }
            return (long)len;
        }
        dc = xfatget(t, dc);
    }
    return -1;
}

static int runx(tx_t *t, const char *name, uint32_t len, msc_flash_ctx_t *c)
{
    t->r.file_len = len;
    ctx_init(c, &t->r, MSC_FLASH_FILE, name, len);
    return run(c, NULL);
}

static uint32_t xused(tx_t *t) { uint32_t n = 0; for (uint32_t c = 2; c < t->ncl + 2; c++) n += xbitget(t, c); return n; }

static void verifyx(tx_t *t, const char *name, uint32_t len)
{
    CHECK(xread(t, name, t->back) == (long)len);
    CHECK(memcmp(t->back, t->file, len) == 0);
    uint8_t *info = malloc(70000);
    CHECK(xread(t, "INFO.TXT", info) == (long)(t->spc * 512 + 100));
    for (uint32_t i = 0; i < t->spc * 512 + 100; i++) if (info[i] != (uint8_t)(i * 5)) { CHECK(0); break; }
    free(info);
}

static void newx(tx_t *t, uint32_t vol, uint32_t shift, uint32_t tot, uint32_t nfats, uint32_t active, uint32_t maxlen)
{
    memset(t, 0, sizeof(*t));
    t->vol = vol; t->shift = shift; t->tot = tot; t->nfats = nfats; t->active = active;
    t->r.sectors = vol + tot + 8;
    t->r.disk = calloc(t->r.sectors, 512);
    t->file = malloc(maxlen);
    t->back = malloc(maxlen + 4096);
    for (uint32_t i = 0; i < maxlen; i++) t->file[i] = (uint8_t)(i * 7 + (i >> 9) + 1);
    t->r.file = t->file;
    mkexfat(t);
}
static void freex(tx_t *t) { free(t->r.disk); free(t->file); free(t->back); }

static void test_exfat_basic(uint32_t vol, uint32_t shift, uint32_t tot, uint32_t len)
{
    tx_t t;
    msc_flash_ctx_t c;
    newx(&t, vol, shift, tot, 1, 0, len + 40000);
    uint32_t used0 = xused(&t);
    int rc = runx(&t, "firmware.bin", len, &c);
    CHECK(rc == MSC_FLASH_DONE);
    if (rc != MSC_FLASH_DONE) printf("  exFAT shift %u vol %u: %s\n", shift, vol, msc_flash_rc_str(rc));
    CHECK(c.is_exfat && c.clusters == t.ncl && c.data_start == t.vol + t.heap && c.spc == t.spc);
    verifyx(&t, "FIRMWARE.BIN", len);
    uint32_t need = (len + t.spc * 512 - 1) / (t.spc * 512);
    CHECK(xused(&t) == used0 + need);
    /* the data starts low: right after INFO.TXT */
    CHECK(xbitget(&t, t.root + 3));
    /* the boot sector, FAT of untouched clusters */
    CHECK(get32(xsec(&t, 0) + 64) == t.vol && xsec(&t, 0)[112] == 0xFF);
    CHECK(xfatget(&t, t.root + 3) == 0);

    /* Replace: other size and content; the old clusters are free again. */
    uint32_t len2 = len / 3 + 1;
    for (uint32_t i = 0; i < len2; i++) t.file[i] ^= 0x5A;
    CHECK(runx(&t, "FIRMWARE.BIN", len2, &c) == MSC_FLASH_DONE);
    verifyx(&t, "FIRMWARE.BIN", len2);
    CHECK(xused(&t) == used0 + (len2 + t.spc * 512 - 1) / (t.spc * 512));
    /* A lower-case name replaces the same file; another name adds one. */
    CHECK(runx(&t, "firmware.bin", len2, &c) == MSC_FLASH_DONE);
    verifyx(&t, "FIRMWARE.BIN", len2);
    CHECK(runx(&t, "app.hex", len2, &c) == MSC_FLASH_DONE);
    verifyx(&t, "APP.HEX", len2);
    CHECK(xread(&t, "FIRMWARE.BIN", t.back) == (long)len2);
    /* Bigger than the first. */
    uint32_t len3 = len + 30000;
    for (uint32_t i = 0; i < len3; i++) t.file[i] = (uint8_t)(i * 3);
    CHECK(runx(&t, "FIRMWARE.BIN", len3, &c) == MSC_FLASH_DONE);
    verifyx(&t, "FIRMWARE.BIN", len3);
    CHECK(xread(&t, "APP.HEX", t.back) == (long)len2);   /* (content differs now, length holds) */
    freex(&t);
}

/* A write that fails: the entry set is the last write, so the old state stays readable. */
static void test_exfat_fail(void)
{
    tx_t t0;
    msc_flash_ctx_t c;
    newx(&t0, 0, 3, 40000, 1, 0, 30000);
    CHECK(runx(&t0, "f.bin", 30000, &c) == MSC_FLASH_DONE);
    int total = t0.r.writes;
    freex(&t0);
    for (int k = 1; k <= total; k++) {
        tx_t t;
        newx(&t, 0, 3, 40000, 1, 0, 30000);
        uint32_t used0 = xused(&t);
        t.r.fail_write_at = k;
        int rc = runx(&t, "f.bin", 30000, &c);
        CHECK(rc == MSC_FLASH_E_IO);
        long n = xread(&t, "F.BIN", t.back);
        CHECK(n == -1 || n == 30000);
        CHECK(xread(&t, "INFO.TXT", t.back) == (long)(t.spc * 512 + 100));
        (void)used0;
        freex(&t);
    }
    /* Replace with a failing write: the old file is whole or the new one is. */
    tx_t t1;
    newx(&t1, 0, 3, 40000, 1, 0, 30000);
    CHECK(runx(&t1, "f.bin", 20000, &c) == MSC_FLASH_DONE);
    t1.r.writes = 0;
    CHECK(runx(&t1, "f.bin", 30000, &c) == MSC_FLASH_DONE);
    int total2 = t1.r.writes;
    freex(&t1);
    for (int k = 1; k <= total2; k++) {
        tx_t t;
        newx(&t, 0, 3, 40000, 1, 0, 30000);
        CHECK(runx(&t, "f.bin", 20000, &c) == MSC_FLASH_DONE);
        t.r.writes = 0;
        t.r.fail_write_at = k;
        CHECK(runx(&t, "f.bin", 30000, &c) == MSC_FLASH_E_IO);
        long n = xread(&t, "F.BIN", t.back);
        CHECK(n == -1 || n == 20000 || n == 30000);
        freex(&t);
    }
}

/* In-use entries that are not files (a volume GUID) in entries [from, to) of the root's first cluster. */
static void fill_root_x(tx_t *t, uint32_t from, uint32_t to)
{
    for (uint32_t i = from; i < to; i++) {
        uint8_t *e = xclus(t, t->root, 0) + i * 32;
        memset(e, 0, 32);
        e[0] = 0xA0;
    }
}

static void test_exfat_dir(void)
{
    msc_flash_ctx_t c;
    tx_t t;
    const uint32_t n = 4096 / 32;   /* entries in a 4 KB cluster */
    newx(&t, 0, 3, 40000, 1, 0, 70000);
    uint32_t f = t.first_free_entry;
    freex(&t);
    /* The root cluster full, no end marker: the root grows by one cluster. */
    newx(&t, 0, 3, 40000, 1, 0, 70000);
    fill_root_x(&t, f, n);
    CHECK(runx(&t, "f.bin", 70000, &c) == MSC_FLASH_DONE);
    CHECK(c.dir_extend == 1);
    verifyx(&t, "F.BIN", 70000);
    CHECK(xfatget(&t, t.root) == c.run_start && xfatget(&t, c.run_start) == 0xFFFFFFFF);
    CHECK(xbitget(&t, c.run_start) && c.run_start == t.root + 3);
    /* A second file lands in the new cluster. */
    CHECK(runx(&t, "g.bin", 3000, &c) == MSC_FLASH_DONE);
    CHECK(c.dir_extend == 0);
    verifyx(&t, "G.BIN", 3000);
    freex(&t);
    /* The end marker is the very last entry, with no room: pad and extend. */
    newx(&t, 0, 3, 40000, 1, 0, 70000);
    fill_root_x(&t, f, n - 1);
    CHECK(runx(&t, "f.bin", 70000, &c) == MSC_FLASH_DONE);
    CHECK(c.dir_extend == 1 && c.pad_lba != 0xFFFFFFFFu);
    verifyx(&t, "F.BIN", 70000);
    CHECK(xclus(&t, t.root, 7)[15 * 32] == 0x05);
    freex(&t);
    /* Deleted entries before the end marker are taken over. */
    newx(&t, 0, 3, 40000, 1, 0, 70000);
    fill_root_x(&t, f, n);
    for (uint32_t i = 20; i < 24; i++) xclus(&t, t.root, 0)[i * 32] = 0x05;
    CHECK(runx(&t, "f.bin", 5000, &c) == MSC_FLASH_DONE);
    CHECK(c.dir_extend == 0);
    CHECK(xclus(&t, t.root, 0)[20 * 32] == 0x85);
    verifyx(&t, "F.BIN", 5000);
    freex(&t);
    /* Little room before the marker in its sector: the set starts in the next sector. */
    newx(&t, 0, 3, 40000, 1, 0, 70000);
    fill_root_x(&t, f, 14);
    for (uint32_t i = 14; i < 40; i++) xclus(&t, t.root, 0)[i * 32] = 0;
    for (uint32_t i = 20; i < 40; i++) xclus(&t, t.root, 0)[i * 32] = 0xA0;   /* junk behind the marker */
    CHECK(runx(&t, "f.bin", 5000, &c) == MSC_FLASH_DONE);
    CHECK(c.pad_lba != 0xFFFFFFFFu && c.dir_lba == c.pad_lba + 1 && c.dir_extend == 0);
    verifyx(&t, "F.BIN", 5000);
    CHECK(xclus(&t, t.root, 0)[14 * 32] == 0x05 && xclus(&t, t.root, 0)[15 * 32] == 0x05);
    CHECK(xclus(&t, t.root, 1)[3 * 32] == 0);
    freex(&t);
    /* Room for the set and a marker: in place, the junk behind is cleared. */
    newx(&t, 0, 3, 40000, 1, 0, 70000);
    fill_root_x(&t, f, 12);
    for (uint32_t i = 12; i < 16; i++) xclus(&t, t.root, 0)[i * 32] = 0;
    xclus(&t, t.root, 0)[13 * 32] = 0xA0;
    xclus(&t, t.root, 0)[15 * 32] = 0xA0;
    CHECK(runx(&t, "f.bin", 5000, &c) == MSC_FLASH_DONE);
    CHECK(c.pad_lba == 0xFFFFFFFFu && c.dir_off == 12 * 32);
    verifyx(&t, "F.BIN", 5000);
    CHECK(xclus(&t, t.root, 0)[15 * 32] == 0);
    freex(&t);
    /* The old file's set sits across a sector boundary, with a FAT-chained file and a long name. */
    newx(&t, 0, 3, 40000, 1, 0, 70000);
    fill_root_x(&t, f, 15);
    /* F.BIN as a FAT chain of 3 clusters at the end of the used area, set at entries 15..17 */
    {
        uint32_t c0 = t.root + 3;
        uint8_t *e = xclus(&t, t.root, 0) + 15 * 32;
        xput(e, "F.BIN", c0, 3 * 4096 - 100, 0);
        xfatset(&t, c0, c0 + 1); xfatset(&t, c0 + 1, c0 + 2); xfatset(&t, c0 + 2, 0xFFFFFFFF);
        for (uint32_t k = 0; k < 3; k++) xbit(&t, c0 + k, 1);
        xclus(&t, t.root, 0)[18 * 32] = 0;
    }
    CHECK(runx(&t, "f.bin", 9000, &c) == MSC_FLASH_DONE);
    CHECK(c.found == 1);
    verifyx(&t, "F.BIN", 9000);
    CHECK(xclus(&t, t.root, 0)[15 * 32] == 0x05 && xclus(&t, t.root, 1)[0] == 0x40);
    CHECK(c.run_start == t.root + 6);
    for (uint32_t k = 0; k < 3; k++) CHECK(xfatget(&t, t.root + 3 + k) == 0 && !xbitget(&t, t.root + 3 + k));
    freex(&t);
}

static void test_exfat_misc(void)
{
    msc_flash_ctx_t c;
    tx_t t;
    /* Full: a file bigger than the free space, and the drive filled to the last cluster. */
    newx(&t, 0, 3, 4096, 1, 0, 3000000);
    {
        uint8_t *snap = malloc((size_t)t.r.sectors * 512);
        memcpy(snap, t.r.disk, (size_t)t.r.sectors * 512);
        CHECK(runx(&t, "f.bin", 3000000, &c) == MSC_FLASH_E_FULL);
        CHECK(memcmp(snap, t.r.disk, (size_t)t.r.sectors * 512) == 0);
        uint32_t free_cl = t.ncl + 2 - (t.root + 3);
        CHECK(runx(&t, "f.bin", free_cl * 4096 + 1, &c) == MSC_FLASH_E_FULL);
        CHECK(runx(&t, "f.bin", free_cl * 4096, &c) == MSC_FLASH_DONE);
        CHECK(runx(&t, "g.bin", 1, &c) == MSC_FLASH_E_FULL);
        free(snap);
    }
    freex(&t);
    /* Fragmented free space: the run is found past the gaps. */
    newx(&t, 0, 3, 40000, 1, 0, 70000);
    for (uint32_t k = t.root + 3; k < t.root + 60; k += 3) xbit(&t, k, 1);
    CHECK(runx(&t, "f.bin", 30000, &c) == MSC_FLASH_DONE);
    CHECK(c.run_start >= t.root + 58);
    verifyx(&t, "F.BIN", 30000);
    freex(&t);
    /* TexFAT: two FATs and bitmaps, the second active. */
    newx(&t, 0, 3, 40000, 2, 1, 70000);
    CHECK(runx(&t, "f.bin", 30000, &c) == MSC_FLASH_DONE);
    verifyx(&t, "F.BIN", 30000);
    CHECK(c.x_active == 1 && c.fat_start == t.vol + t.fatoff + t.fatlen);
    freex(&t);
    /* Refused boot sectors. */
    struct { uint32_t off, val, size; int rc; } bad[] = {
        { 11, 1, 1, MSC_FLASH_E_FS }, { 63, 7, 1, MSC_FLASH_E_FS }, { 108, 10, 1, MSC_FLASH_E_SECTOR }, { 108, 8, 1, MSC_FLASH_E_FS },
        { 108, 12, 1, MSC_FLASH_E_SECTOR }, { 109, 16, 1, MSC_FLASH_E_FS }, { 110, 0, 1, MSC_FLASH_E_FS },
        { 110, 3, 1, MSC_FLASH_E_FS }, { 80, 0, 4, MSC_FLASH_E_FS }, { 84, 0, 4, MSC_FLASH_E_FS },
        { 88, 0, 4, MSC_FLASH_E_FS }, { 88, 0x7FFFFFFF, 4, MSC_FLASH_E_FS }, { 92, 0, 4, MSC_FLASH_E_FS },
        { 92, 0xFFFFFFF6, 4, MSC_FLASH_E_FS }, { 96, 0, 4, MSC_FLASH_E_FS }, { 96, 0x7FFFFFFF, 4, MSC_FLASH_E_FS },
        { 105, 2, 1, MSC_FLASH_E_FS }, { 106, 1, 1, MSC_FLASH_E_FS }, { 72, 0xFFFFFFFF, 4, MSC_FLASH_E_FS },
        { 76, 1, 4, MSC_FLASH_E_FS }, { 510, 0, 1, MSC_FLASH_E_FS }, { 84, 1, 4, MSC_FLASH_E_FS },
    };
    for (unsigned k = 0; k < sizeof(bad) / sizeof(bad[0]); k++) {
        newx(&t, k & 1 ? 2048 : 0, 3, 40000, 1, 0, 6000);
        uint8_t *b = xsec(&t, 0);
        if (bad[k].size == 1) b[bad[k].off] = (uint8_t)bad[k].val; else put32(b + bad[k].off, bad[k].val);
        int rc = runx(&t, "f.bin", 5000, &c);
        CHECK(rc == bad[k].rc);
        if (rc != bad[k].rc) printf("  bad[%u]: %s\n", k, msc_flash_rc_str(rc));
        freex(&t);
    }
    /* No bitmap entry, a bitmap too short, a bitmap entry in the wrong place. */
    newx(&t, 0, 3, 40000, 1, 0, 6000);
    xclus(&t, t.root, 0)[32] = 0x05;
    CHECK(runx(&t, "f.bin", 5000, &c) == MSC_FLASH_E_FS);
    freex(&t);
    newx(&t, 0, 3, 40000, 1, 0, 6000);
    put32(xclus(&t, t.root, 0) + 32 + 24, 8);
    CHECK(runx(&t, "f.bin", 5000, &c) == MSC_FLASH_E_FS);
    freex(&t);
    /* A looping root chain. */
    newx(&t, 0, 3, 40000, 1, 0, 6000);
    fill_root_x(&t, t.first_free_entry, 128);
    xfatset(&t, t.root, t.root);
    t.r.file_len = 5000;
    ctx_init(&c, &t.r, MSC_FLASH_FILE, "f.bin", 5000);
    int rc, steps = 0;
    while ((rc = msc_flash_step(&c)) == MSC_FLASH_RUNNING) steps++;
    CHECK(rc == MSC_FLASH_E_FS && steps < 700000);
    freex(&t);
    /* READ and VERIFY have no exFAT support. */
    newx(&t, 0, 3, 40000, 1, 0, 40 * 512);
    ctx_init(&c, &t.r, MSC_FLASH_READ, "FAIL.TXT", 0);
    CHECK(run(&c, NULL) == MSC_FLASH_E_EXFAT);
    CHECK(strcmp(msc_flash_rc_str(MSC_FLASH_E_EXFAT), "reading back from an exFAT drive is not supported") == 0);
    /* UF2 on exFAT: the blocks go to the cluster heap. */
    enum { nb = 40 };
    make_uf2(t.file, nb, 0x10000000);
    t.r.file_len = nb * 512;
    ctx_init(&c, &t.r, MSC_FLASH_UF2, NULL, nb * 512);
    CHECK(run(&c, NULL) == MSC_FLASH_DONE);
    CHECK(c.data_base == t.vol + t.heap);
    for (uint32_t i = 0; i < nb; i++) CHECK(memcmp(t.r.disk + (size_t)(c.data_base + i) * 512, t.file + i * 512, 512) == 0);
    freex(&t);
    /* Hostile volumes: random changes to the boot sector, the FAT and the root directory. */
    srand(23);
    for (int n = 0; n < 3000; n++) {
        newx(&t, n & 1 ? 63 : 0, 3, 8000, 1, 0, 6000);
        for (int k = 0; k < 4; k++) xsec(&t, 0)[rand() % 120] = (uint8_t)rand();
        if (n & 2) xclus(&t, t.root, 0)[rand() % 200] = (uint8_t)rand();
        if (n & 4) put32(xsec(&t, t.fatoff) + 4 * (2 + rand() % 12), (uint32_t)rand());
        if (n & 8) put32(xclus(&t, t.root, 0) + 32 + 20, (uint32_t)rand());
        if (n & 16) fill_root_x(&t, t.first_free_entry, 128);
        int rc = runx(&t, "f.bin", 6000, &c);
        CHECK(rc <= 0 && rc != -99);
        freex(&t);
    }
}

static void test_exfat(void)
{
    test_exfat_basic(0, 3, 131072, 3 * 1024 * 1024 + 17);      /* 64 MB, 4 KB clusters */
    test_exfat_basic(2048, 6, 262144, 5 * 1024 * 1024 + 1);    /* 128 MB behind an MBR, 32 KB clusters */
    test_exfat_basic(0, 3, 524288, 70000);                     /* 256 MB: a two-cluster bitmap */
    test_exfat_basic(63, 0, 3000, 900);                        /* 512 B clusters */
    test_exfat_fail();
    test_exfat_dir();
    test_exfat_misc();
}

static void test_names(void)
{
    char n[11];
    CHECK(msc_flash_name83("firmware.bin", n) && memcmp(n, "FIRMWAREBIN", 11) == 0);
    CHECK(msc_flash_name83("A.HEX", n) && memcmp(n, "A       HEX", 11) == 0);
    CHECK(msc_flash_name83("NOEXT", n) && memcmp(n, "NOEXT      ", 11) == 0);
    CHECK(!msc_flash_name83("toolongname.bin", n));
    CHECK(!msc_flash_name83("a.bins", n));
    CHECK(!msc_flash_name83("sp ace.bin", n));
    CHECK(!msc_flash_name83(".bin", n));
    CHECK(!msc_flash_name83("", n));
    CHECK(strcmp(msc_flash_rc_str(MSC_FLASH_E_FS), "the drive has no FAT12, FAT16, FAT32 or exFAT file system") == 0);
    CHECK(strcmp(msc_flash_phase_str(MSC_PH_FAT), "fat") == 0);
    CHECK(strcmp(msc_flash_phase_str(MSC_PH_FSINFO), "fsinfo") == 0);
}

/* The new entry's dates: the context's unix time, else 2026-01-01. */
static void test_stamp(void)
{
    const uint32_t cases[3] = { 0, 1790000000u, 1767225599u };   /* unknown, 2026-09-21 14:13:20, 2025-12-31 23:59:59 (before 2026, after 2020) */
    const uint16_t dates[3] = { (46 << 9) | (1 << 5) | 1, 23861, (45 << 9) | (12 << 5) | 31 };
    const uint16_t times[3] = { 0, 29098, (23 << 11) | (59 << 5) | 29 };
    for (int k = 0; k < 3; k++) {
        geo_t g = { 0, 4, 1, 2, 512, 6, 8192, 12 };
        rig_t r = { calloc(g.vol + g.tot + 8, 512), g.vol + g.tot + 8, NULL, 1000, 0, 0 };
        uint8_t *file = calloc(1000, 1);
        r.file = file;
        mkfs(&r, g);
        msc_flash_ctx_t c;
        ctx_init(&c, &r, MSC_FLASH_FILE, "stamp.bin", 1000);
        c.unix_time = cases[k];
        CHECK(run(&c, NULL) == MSC_FLASH_DONE);
        const uint8_t *e = NULL;
        for (size_t o = 0; o + 32 <= (size_t)r.sectors * 512; o += 32)
            if (!memcmp(r.disk + o, "STAMP   BIN", 11)) { e = r.disk + o; break; }
        CHECK(e != NULL);
        if (e) {
            CHECK(get16(e + 16) == dates[k] && get16(e + 24) == dates[k] && get16(e + 18) == dates[k]);
            CHECK(get16(e + 14) == times[k] && get16(e + 22) == times[k]);
        }
        free(r.disk); free(file);
    }
}

int main(void)
{
    test_names();
    test_uf2();
    test_file((geo_t){ 0, 1, 1, 2, 64, 3, 1024, 12 }, 70 * 1024 + 17);      /* FAT12, 1 sector/cluster */
    test_file((geo_t){ 0, 4, 1, 2, 512, 6, 8192, 12 }, 200 * 1024);         /* FAT12, 4 sectors/cluster */
    test_file((geo_t){ 63, 4, 4, 2, 512, 40, 40000, 16 }, 300 * 1024 + 3);  /* FAT16 behind an MBR */
    test_file((geo_t){ 0, 1, 1, 1, 128, 160, 20000, 16 }, 513);             /* FAT16, one FAT copy */
    test_refusals();
    test_stamp();
    test_fat32();
    test_read_file();
    test_verify();
    test_exfat();
    printf("test_msc_flash: %s (%d checks)\n", failures ? "FAILED" : "all passed", checks);
    return failures ? 1 : 0;
}
