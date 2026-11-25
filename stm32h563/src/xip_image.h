#ifndef XIP_IMAGE_H
#define XIP_IMAGE_H

/* xip_image — code that runs straight from the W25Q (XIP), and how the firmware finds it.
 *
 * Rarely used code is linked into `.xip*` output sections and runs from the W25Q. There are two
 * images, each with its own table, expected hash and blob slot:
 *
 *   image   section     slot       code
 *   usb     .xip        xip        the USB flash protocols and the ESP32 ROM loader (src/xip_mod_*.c)
 *   prog    .xip_prog   xip_prog   the pin programmers of prog_flash (xip_prog_mod.c, prog_<proto>.c) and
 *                                  the sensor definition compiler (sensor_def_compile.c, sensor_def_expr.c)
 *
 * Memory-mapped (XIP_RAM=0, digital v3r4): OCTOSPI1 maps the W25Q at 0x90000000. The usb image runs
 * at 0x90201000 (slot "xip", W25Q 0x200000 + the 4 KB header) and the prog image at 0x90241000
 * (slot "xip_prog", W25Q 0x240000), each at its own address, no copy.
 * RAM (XIP_RAM=1, the release default): both images are linked at the SAME RAM region (an
 * overlay), the adc_pool (adc_pool.h: the 64 KB shared sample buffer, idle during a session; the
 * Makefile's XIP_RAM_KB and the linker script's POOL/XIP/XIP_PROG regions). A session names the
 * image it needs and begin copies that image's slot into the region (after
 * adc_pool_overlay_begin() took the pool). Only one session runs at a time, so one region is
 * enough, and each image must fit the region (up to 64 KB; the link asserts it).
 *
 * The post-link step (tools/xip_finish.py) extracts each image as build/blobs/blob-<slot>.bin,
 * installed like any blob, and patches its length and SHA-256 into the image's expected-hash
 * record in internal flash, so the firmware knows the one image it can run. A session
 * (xip_session.h) checks the slot against it before running any of it.
 *
 * Rules, checked after every link (tools/check_xip_refs.py):
 *  - Internal code reaches XIP code only through the table at the start of the image
 *    (xip_table_t): it reads the entry's address at run time, inside a session. No direct call,
 *    and no function pointer to XIP code in internal code or data.
 *  - XIP code passes no pointer to its own functions anywhere (a callback would be called from
 *    another task or an ISR, outside the session); only the table holds XIP function addresses.
 *  - XIP objects (src/xip_mod_*.c, and the prog_<proto>.c and sensor_def_{compile,expr}.c files of the
 *    prog image) have no .data or .bss: their state lives in the session's context.
 *  - An image never refers to the other one (they share one RAM region, the adc_pool, in XIP_RAM=1 builds).
 * XIP code may call internal functions freely: the linker adds long-branch veneers.
 * Internal code may hold the address of an XIP table and of the image's start; that is all.
 */

#include <stdint.h>
#include <stdbool.h>
#include "blob_store.h"

typedef enum { XIP_IMG_USB = 0, XIP_IMG_PROG, XIP_IMG_COUNT } xip_img_t;

#define XIP_TABLE_MAGIC  0x50495842u   /* "BXIP" */
#define XIP_ABI          3u            /* the usb image: bump when its table's entries change meaning */
#define XIP_PROG_ABI     2u            /* the prog image */

/* The usb image's entries, by index. */
enum {
    XIP_FN_SELFTEST = 0,   /* uint32_t (*)(uint32_t seed): checksum over the image's own data */
    XIP_FN_MSC_FLASH_STEP, /* int (*)(void *ctx): msc_flash_step, a USB drive flash (msc_flash.h) */
    XIP_FN_CTRL_FLASH_STEP,/* int (*)(void *ctx): ctrl_flash_step, DFU/DfuSe and Teensy (ctrl_flash.h) */
    XIP_FN_SER_FLASH_STEP, /* int (*)(void *ctx): ser_flash_step, Arduino AVR and SAMD bootloaders (ser_flash.h) */
    XIP_FN_ESP_ROM_BEGIN,  /* int (*)(void *job): esp_rom_job_begin, the ESP32 ROM loader write (esp_rom_job.h) */
    XIP_FN_ESP_ROM_STEP,   /* int (*)(void *job): esp_rom_job_step */
    XIP_FN_COUNT
};

/* The prog image's entries. The selftest is the same function as the usb image's. */
enum {
    XIP_PROG_FN_SELFTEST = 0,  /* uint32_t (*)(uint32_t seed) */
    XIP_PROG_FN_CALL,          /* int (*)(prog_ctx_t *ctx, unsigned proto, unsigned op): prog_flash.h */
    XIP_PROG_FN_SD_COMPILE,    /* int (*)(sd_compile_args_t *): sd_compile, the sensor definition compiler (sensor_def.h) */
    XIP_PROG_FN_COUNT
};

typedef uint32_t (*xip_fn_u32_t)(uint32_t);
typedef int      (*xip_fn_ctx_t)(void *);
typedef int      (*xip_fn_ctx2_t)(void *, unsigned, unsigned);

/* An image's table, at its start: the header, then `count` entries. */
typedef struct {
    uint32_t    magic;            /* XIP_TABLE_MAGIC */
    uint32_t    abi;              /* XIP_ABI or XIP_PROG_ABI */
    uint32_t    count;            /* XIP_FN_COUNT or XIP_PROG_FN_COUNT */
    const void *fn[];
} xip_table_t;

/* In internal flash: what the post-link step found in an image. len 0 = no image (not patched). */
#define XIP_HASH_MAGIC 0x48504958u      /* "XIPH" */
typedef struct {
    uint32_t        magic;
    blob_manifest_t m;           /* len and sha256 of the image (version 0): the blob's manifest entry */
} xip_hash_t;

/* What the firmware knows about an image (xip_image.c). */
typedef struct {
    const char        *name;       /* "usb", "prog" */
    blob_id_t          blob;       /* the slot that holds it: BLOB_XIP, BLOB_XIP_PROG */
    const xip_table_t *table;      /* where it is linked: its table (also its first byte) */
    const uint8_t     *region_end; /* the most it may take */
    const xip_hash_t  *expected;
    uint32_t           abi, count; /* what the table must say */
} xip_image_t;

const xip_image_t *xip_image(xip_img_t img);
/* The expected hash of the usb image (the older call), or of any image. */
const xip_hash_t *xip_image_expected(void);
const xip_hash_t *xip_image_expected_of(xip_img_t img);

/* This board can run code from the W25Q: its chip-select is on an OCTOSPI1 NCS pin
   (board_caps()->qspi_cs_ncs_pins, digital v3r4), or the build runs the images from RAM. The xip
   and xip_prog blob slots are listed only then. */
bool xip_supported(void);

/* The tables (the first bytes of each image) and where the regions end. In RAM builds both images
   start at the same address. */
extern const xip_table_t xip_table, xip_prog_table;
extern const uint8_t __xip_start[], __xip_end[], __xip_region_end[];
extern const uint8_t __xip_prog_start[], __xip_prog_end[], __xip_prog_region_end[];

#endif /* XIP_IMAGE_H */
