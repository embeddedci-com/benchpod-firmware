/* xip_image.c — see xip_image.h. Internal flash. */
#include "xip_image.h"
#include "board_caps.h"

/* Patched after the link by tools/xip_finish.py (objcopy --update-section .xip_hash and
   .xip_prog_hash). */
__attribute__((section(".xip_hash"), used))
const xip_hash_t xip_expected = { XIP_HASH_MAGIC, { 0u, 0u, { 0 } } };
__attribute__((section(".xip_prog_hash"), used))
const xip_hash_t xip_prog_expected = { XIP_HASH_MAGIC, { 0u, 0u, { 0 } } };

static const xip_image_t k_images[XIP_IMG_COUNT] = {
    [XIP_IMG_USB]  = { "usb",  BLOB_XIP,      &xip_table,      __xip_region_end,      &xip_expected,
                       XIP_ABI,      XIP_FN_COUNT },
    [XIP_IMG_PROG] = { "prog", BLOB_XIP_PROG, &xip_prog_table, __xip_prog_region_end, &xip_prog_expected,
                       XIP_PROG_ABI, XIP_PROG_FN_COUNT },
};

const xip_image_t *xip_image(xip_img_t img) { return img < XIP_IMG_COUNT ? &k_images[img] : 0; }

const xip_hash_t *xip_image_expected(void) { return &xip_expected; }
const xip_hash_t *xip_image_expected_of(xip_img_t img) { return img < XIP_IMG_COUNT ? k_images[img].expected : 0; }

bool xip_supported(void)
{
#if defined(BENCHPOD_XIP_RAM) || defined(BENCHPOD_XIP_BODGE_PB6)
    return true;    /* dev builds that run the image on boards without the v3r4 chip-select */
#else
    return board_caps()->qspi_cs_ncs_pins;
#endif
}

uint32_t xip_core_fnv_step(uint32_t h, uint8_t b) { return (h ^ b) * 16777619u; }
