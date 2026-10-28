/* Host unit test for module_eeprom: the analog module's ID header round-trips, a blank or
   damaged chip is told apart, a later layout with a longer header still reads, and writes are
   split at page boundaries and kept out of the header area. */
#include "module_eeprom.h"
#include <stdio.h>
#include <string.h>

static int fails = 0;
#define CHECK(cond, ...) do { if (!(cond)) { \
        printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

int main(void)
{
    uint8_t hdr[64];
    module_id_t id = { 0 }, out;

    /* CRC-16/CCITT-FALSE check value. */
    CHECK(module_ee_crc16((const uint8_t *)"123456789", 9, 0xFFFFu) == 0x29B1u, "crc check value");
    CHECK(module_ee_crc16((const uint8_t *)"56789", 5, module_ee_crc16((const uint8_t *)"1234", 4, 0xFFFFu))
          == 0x29B1u, "crc in pieces");

    /* Round trip. */
    id.type = MOD_TYPE_ANALOG; id.rev_major = 1; id.rev_minor = 2;
    strcpy(id.serial, "AM-000123");
    id.cal_len = 300; id.cal_crc = 0xBEEF;
    memset(hdr, 0xFF, sizeof(hdr));
    module_ee_encode(&id, hdr);
    CHECK(memcmp(hdr, "BPMD", 4) == 0 && hdr[4] == 1 && hdr[5] == 32, "magic, version, length");
    CHECK(module_ee_decode(hdr, 32, &out) == MOD_EE_OK, "decode");
    CHECK(out.version == 1 && out.type == MOD_TYPE_ANALOG && out.rev_major == 1 && out.rev_minor == 2,
          "fields");
    CHECK(strcmp(out.serial, "AM-000123") == 0, "serial '%s'", out.serial);
    CHECK(out.cal_len == 300 && out.cal_crc == 0xBEEF, "cal fields");
    CHECK(hdr[26] == 0x2C && hdr[27] == 0x01, "little endian cal_len");

    /* A 16-character serial has no NUL in the chip but one in the struct. */
    strcpy(id.serial, "0123456789ABCDEF");
    module_ee_encode(&id, hdr);
    CHECK(module_ee_decode(hdr, 32, &out) == MOD_EE_OK && strcmp(out.serial, "0123456789ABCDEF") == 0,
          "full-length serial");

    /* Blank, wrong magic, flipped bit, short read. */
    memset(hdr, 0xFF, sizeof(hdr));
    CHECK(module_ee_decode(hdr, 32, &out) == MOD_EE_BLANK, "blank chip");
    module_ee_encode(&id, hdr);
    hdr[0] = 'X';
    CHECK(module_ee_decode(hdr, 32, &out) == MOD_EE_BAD_MAGIC, "bad magic");
    module_ee_encode(&id, hdr);
    hdr[12] ^= 0x01;
    CHECK(module_ee_decode(hdr, 32, &out) == MOD_EE_BAD_CRC, "flipped serial bit");
    module_ee_encode(&id, hdr);
    CHECK(module_ee_decode(hdr, 31, &out) == MOD_EE_BAD_LEN, "short read");
    hdr[5] = 16;
    CHECK(module_ee_decode(hdr, 32, &out) == MOD_EE_BAD_LEN, "header length below 32");

    /* A later layout: 48-byte header, new fields after byte 29, CRC in its last two bytes. */
    module_ee_encode(&id, hdr);
    hdr[4] = 2; hdr[5] = 48;
    memset(hdr + 30, 0xA5, 16);
    uint16_t c = module_ee_crc16(hdr, 46, 0xFFFFu);
    hdr[46] = (uint8_t)c; hdr[47] = (uint8_t)(c >> 8);
    CHECK(module_ee_decode(hdr, 64, &out) == MOD_EE_OK && out.version == 2 && out.rev_minor == 2,
          "version 2 header still reads");
    CHECK(module_ee_decode(hdr, 32, &out) == MOD_EE_BAD_LEN, "longer header than was read");

    /* Page split. */
    CHECK(module_ee_chunk(0x100, 100) == 32, "page-aligned");
    CHECK(module_ee_chunk(0x10F, 100) == 17, "mid-page");
    CHECK(module_ee_chunk(0x10F, 5) == 5, "fits");
    CHECK(module_ee_chunk(0x11F, 1) == 1, "last byte of a page");

    /* Raw writes stay in the calibration area. */
    CHECK(module_ee_raw_write_ok(MOD_EE_CAL_ADDR, 32), "cal start");
    CHECK(module_ee_raw_write_ok(MOD_EE_SIZE - 1, 1), "last byte");
    CHECK(!module_ee_raw_write_ok(0, 32), "header");
    CHECK(!module_ee_raw_write_ok(MOD_EE_CAL_ADDR - 1, 2), "straddles into the header area");
    CHECK(!module_ee_raw_write_ok(MOD_EE_SIZE - 1, 2), "past the end");
    CHECK(!module_ee_raw_write_ok(MOD_EE_CAL_ADDR, 0), "empty");

    CHECK(strcmp(module_ee_result_str(MOD_EE_BLANK), "blank") == 0, "result string");
    CHECK(strcmp(module_ee_type_str(MOD_TYPE_ANALOG), "analog") == 0 &&
          strcmp(module_ee_type_str(7), "unknown") == 0, "type string");

    if (fails) { printf("test_module_eeprom: %d failure(s)\n", fails); return 1; }
    printf("test_module_eeprom: all passed\n");
    return 0;
}
