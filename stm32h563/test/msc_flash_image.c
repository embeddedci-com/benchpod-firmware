/* msc_flash_image: run the msc_flash writer on a raw disk image file (used by
 * test_msc_flash_macos.sh, not by the default Makefile target).
 * usage: msc_flash_image <image> <file> <NAME83> [step-limit]
 * Prints the result and the number of steps. Exit 0 = MSC_FLASH_DONE. */
#include "msc_flash.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct { FILE *img, *src; } io_t;

static int rd(void *io, uint32_t lba, uint8_t *buf, uint32_t n)
{
    io_t *x = io;
    return fseek(x->img, (long)lba * 512, SEEK_SET) || fread(buf, 512, n, x->img) != n;
}
static int wr(void *io, uint32_t lba, const uint8_t *buf, uint32_t n)
{
    io_t *x = io;
    return fseek(x->img, (long)lba * 512, SEEK_SET) || fwrite(buf, 512, n, x->img) != n || fflush(x->img);
}
static int src(void *io, uint32_t off, uint8_t *buf, uint32_t n)
{
    io_t *x = io;
    return fseek(x->src, (long)off, SEEK_SET) || fread(buf, 1, n, x->src) != n;
}

int main(int argc, char **argv)
{
    if (argc < 4) { fprintf(stderr, "usage: %s image file NAME83\n", argv[0]); return 2; }
    io_t io = { fopen(argv[1], "r+b"), fopen(argv[2], "rb") };
    if (!io.img || !io.src) { perror("open"); return 2; }
    fseek(io.img, 0, SEEK_END);
    long isz = ftell(io.img);
    fseek(io.src, 0, SEEK_END);
    long fsz = ftell(io.src);
    static msc_flash_ctx_t c;
    memset(&c, 0, sizeof(c));
    c.ops = (msc_flash_ops_t){ &io, rd, wr, src, (uint32_t)(isz / 512), 512 };
    c.mode = MSC_FLASH_FILE;
    c.len = (uint32_t)fsz;
    if (!msc_flash_name83(argv[3], c.name83)) { fprintf(stderr, "bad name\n"); return 2; }
    int rc, steps = 0;
    while ((rc = msc_flash_step(&c)) == MSC_FLASH_RUNNING) steps++;
    if (c.is_exfat) printf("exFAT");
    else printf("FAT%u", c.fat_bits);
    printf(" clusters %u first %u root-extended %u steps %d: %s\n", c.clusters, c.run_start + c.dir_extend, c.dir_extend,
           steps, msc_flash_rc_str(rc));
    return rc;
}
