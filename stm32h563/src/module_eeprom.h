/*
 * module_eeprom.h — the layout of the analog module's ID + calibration EEPROM (pure, host-tested).
 *
 * The analog module of digital v3r3 carries a 24C32 (U61, 4 KB, 32-byte pages, two-byte memory
 * address) at 0x50 on the module bus, write protect tied low.  Layout version 1:
 *
 *   0x000  header, 32 bytes, little endian
 *            0  magic "BPMD"
 *            4  layout version (1)
 *            5  header length (32); a later version may grow it, and the header CRC is always
 *               its last two bytes, so this firmware still reads the fields it knows
 *            6  module type (1 = analog)
 *            7  hardware revision, major
 *            8  hardware revision, minor
 *            9  reserved (0)
 *           10  serial number, 16 bytes, NUL padded (no NUL needed when all 16 are used)
 *           26  calibration length in bytes (0 = none)
 *           28  calibration CRC-16/CCITT-FALSE
 *           30  header CRC-16/CCITT-FALSE over bytes 0..29
 *   0x020  reserved up to 0x0FF (raw writes are refused here)
 *   0x100  calibration blob, up to 3840 bytes.  The firmware stores and checks it but does not
 *          interpret it: the format is the calibration tools' to define.
 *
 * A blank chip reads all 0xFF.
 */
#ifndef MODULE_EEPROM_H
#define MODULE_EEPROM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MOD_EE_SIZE        4096u
#define MOD_EE_PAGE        32u
#define MOD_EE_HDR_LEN     32u
#define MOD_EE_CAL_ADDR    0x0100u
#define MOD_EE_CAL_MAX     (MOD_EE_SIZE - MOD_EE_CAL_ADDR)
#define MOD_EE_VERSION     1u
#define MOD_EE_SERIAL_LEN  16u

#define MOD_TYPE_ANALOG    1u

typedef struct {
    uint8_t  version;                      /* layout version found */
    uint8_t  type;                         /* MOD_TYPE_* */
    uint8_t  rev_major, rev_minor;
    char     serial[MOD_EE_SERIAL_LEN + 1];
    uint16_t cal_len;
    uint16_t cal_crc;
} module_id_t;

/* Results of module_ee_decode. */
#define MOD_EE_OK         0
#define MOD_EE_BLANK    (-1)   /* all 0xFF: never written */
#define MOD_EE_BAD_MAGIC (-2)
#define MOD_EE_BAD_CRC  (-3)
#define MOD_EE_BAD_LEN  (-4)   /* header length below 32 or past what was read */

/* CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF): pass 0xFFFF to start, the last result to go on. */
uint16_t module_ee_crc16(const uint8_t *p, size_t n, uint16_t crc);

/* Decode the header from the first `n` bytes of the chip (n >= MOD_EE_HDR_LEN). */
int  module_ee_decode(const uint8_t *buf, size_t n, module_id_t *out);
/* Encode a version-1 header (32 bytes) for `id`; id->version is ignored. */
void module_ee_encode(const module_id_t *id, uint8_t out[MOD_EE_HDR_LEN]);

/* "ok", "blank", "bad magic", "bad crc", "bad length". */
const char *module_ee_result_str(int rc);
/* "analog" for MOD_TYPE_ANALOG, else "unknown". */
const char *module_ee_type_str(uint8_t type);

/* Bytes of a write at `addr` that fit before the next page boundary (a 24C32 page write wraps
   inside its page), capped at `len`. */
size_t module_ee_chunk(uint16_t addr, size_t len);

/* A raw write may only land in the calibration area: [MOD_EE_CAL_ADDR, MOD_EE_SIZE). */
bool module_ee_raw_write_ok(uint32_t addr, size_t len);

#endif /* MODULE_EEPROM_H */
