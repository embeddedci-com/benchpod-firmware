#ifndef XIP_IMAGE_H
#define XIP_IMAGE_H

/* xip_image — code that runs straight from the W25Q (XIP), and how the firmware finds it.
 *
 * Rarely used, v3r4-only code (the USB flash protocols) is linked into the `.xip` output section
 * at the W25Q's memory-mapped address (OCTOSPI1 maps the W25Q at 0x90000000; the slot "xip" holds
 * the image at W25Q 0x201000, so code runs from 0x90201000). The post-link step
 * (tools/xip_finish.py) extracts it as build/blobs/blob-xip.bin, installed like any blob, and
 * patches its length and SHA-256 into xip_expected in internal flash, so the firmware knows the
 * one image it can run. A session (xip_session.h) checks the slot against it before running any of
 * it.
 *
 * Rules, checked after every link (tools/check_xip_refs.py):
 *  - Internal code reaches XIP code only through the table at the start of the image
 *    (xip_table_t): it reads the entry's address at run time, inside a session. No direct call,
 *    and no function pointer to XIP code in internal code or data.
 *  - XIP code passes no pointer to its own functions anywhere (a callback would be called from
 *    another task or an ISR, outside the session); only the table holds XIP function addresses.
 *  - XIP objects (src/xip_mod_*.c) have no .data or .bss: their state lives in the session's
 *    context.
 * XIP code may call internal functions freely: the linker adds long-branch veneers.
 */

#include <stdint.h>

#define XIP_TABLE_MAGIC  0x50495842u   /* "BXIP" */
#define XIP_ABI          2u            /* bump when the table's entries change meaning */

/* The entries, by index. */
enum {
    XIP_FN_SELFTEST = 0,   /* uint32_t (*)(uint32_t seed): checksum over the image's own data */
    XIP_FN_MSC_FLASH_STEP, /* int (*)(void *ctx): msc_flash_step, a USB drive flash (msc_flash.h) */
    XIP_FN_COUNT
};

typedef uint32_t (*xip_fn_u32_t)(uint32_t);
typedef int      (*xip_fn_ctx_t)(void *);

typedef struct {
    uint32_t    magic;            /* XIP_TABLE_MAGIC */
    uint32_t    abi;              /* XIP_ABI */
    uint32_t    count;            /* XIP_FN_COUNT */
    const void *fn[XIP_FN_COUNT];
} xip_table_t;

/* In internal flash: what the post-link step found in .xip. len 0 = no image (not patched). */
#define XIP_HASH_MAGIC 0x48504958u      /* "XIPH" */
typedef struct {
    uint32_t magic;
    uint32_t len;
    uint8_t  sha256[32];
} xip_hash_t;

const xip_hash_t *xip_image_expected(void);

/* This board can run code from the W25Q: its chip-select is on an OCTOSPI1 NCS pin
   (board_caps()->qspi_cs_ncs_pins, digital v3r4). The xip blob slot is listed only then. */
#include <stdbool.h>
bool xip_supported(void);

/* Where the image is linked (the table's address) and the most it may take. */
extern const xip_table_t xip_table;
extern const uint8_t __xip_start[], __xip_end[], __xip_region_end[];

#endif /* XIP_IMAGE_H */
