/* Host test for msc_flash (xip_mod_msc_flash.c, msc_flash_names.c): UF2 blocks reach a simulated
   UF2 bootloader whole and in order; a file lands on FAT12 and FAT16 drives (with and without an
   MBR) next to an existing file and reads back; hostile drives and files are refused. */
#include "msc_flash.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); failures++; } } while (0)

typedef struct {
    uint8_t *disk;
    uint32_t sectors;
    const uint8_t *file;
    uint32_t file_len;
    int fail_write_at;     /* nth write fails (0 = never) */
    int writes;
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
    CHECK(strcmp(msc_flash_rc_str(MSC_FLASH_E_FS), "the drive has no FAT12 or FAT16 file system") == 0);
    CHECK(strcmp(msc_flash_phase_str(MSC_PH_FAT), "fat") == 0);
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
    printf("test_msc_flash: %s\n", failures ? "FAILED" : "all passed");
    return failures ? 1 : 0;
}
