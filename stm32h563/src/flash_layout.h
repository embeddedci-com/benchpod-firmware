/*
 * flash_layout.h — one firmware for the STM32H563 with 2 MB (ZIT6) or 1 MB (ZGT6) of flash.
 *
 * The persistence offsets (pod_policy.h, adc_cal.h, dac_limits.h, cloud_config.h, config_store.h,
 * device_identity.c, the OTA self-test scratch) are written for the 2 MB part: they sit in
 * its top 112 KB, 0x1E4000..0x1FFFFF (the pod policy took 0x1E4000..0x1E7FFF, below the
 * older records, which did not move). flash_compat maps every such offset to the same distance
 * from the END of the flash this chip actually has. On a 2 MB pod the mapping is the identity,
 * so no record moves; on a 1 MB pod the same records live in 0x0E4000..0x0FFFFF.
 *
 * The size comes from the chip's flash-size register, read as a 32-bit word: a narrower read
 * of this OTP area is a precise bus fault on the H5.
 */
#ifndef FLASH_LAYOUT_H
#define FLASH_LAYOUT_H

#include <stdint.h>

#define FLASH_LAYOUT_REF_SIZE    0x200000u   /* the 2 MB part the offsets are written for */
#define FLASH_LAYOUT_STORE_BASE  0x1E4000u   /* lowest persistence offset (pod policy slot B) */
#define FLASH_LAYOUT_SIZE_1MB    0x100000u
#define FLASH_LAYOUT_SIZE_2MB    0x200000u

/* Map a 2 MB-reference offset to the offset on a part with `flash_size` bytes. Offsets below
   the persistence area are not part of the layout and pass through unchanged. */
static inline uint32_t flash_layout_map(uint32_t ref_off, uint32_t flash_size)
{
    if (ref_off < FLASH_LAYOUT_STORE_BASE) return ref_off;
    return ref_off - FLASH_LAYOUT_REF_SIZE + flash_size;
}

/* Size of the internal flash in bytes: FLASH_LAYOUT_SIZE_1MB or FLASH_LAYOUT_SIZE_2MB. */
uint32_t flash_layout_size(void);

/* Each of the two banks is half the flash: 512 KB on a 1 MB part, 1 MB on a 2 MB part. */
static inline uint32_t flash_layout_bank_size(void) { return flash_layout_size() / 2u; }

/* A 2 MB-reference offset mapped onto this chip. */
static inline uint32_t flash_layout_store_off(uint32_t ref_off)
{
    return flash_layout_map(ref_off, flash_layout_size());
}

#endif /* FLASH_LAYOUT_H */
