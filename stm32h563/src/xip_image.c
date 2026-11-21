/* xip_image.c — see xip_image.h. Internal flash. */
#include "xip_image.h"
#include "board_caps.h"

/* Patched after the link by tools/xip_finish.py (objcopy --update-section .xip_hash). */
__attribute__((section(".xip_hash"), used))
const xip_hash_t xip_expected = { XIP_HASH_MAGIC, 0u, { 0 } };

const xip_hash_t *xip_image_expected(void) { return &xip_expected; }

bool xip_supported(void) { return board_caps()->qspi_cs_ncs_pins; }

uint32_t xip_core_fnv_step(uint32_t h, uint8_t b) { return (h ^ b) * 16777619u; }
