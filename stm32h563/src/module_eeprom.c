/* module_eeprom.c — the analog module's EEPROM layout. See module_eeprom.h. */
#include "module_eeprom.h"
#include <string.h>

static const uint8_t k_magic[4] = { 'B', 'P', 'M', 'D' };

uint16_t module_ee_crc16(const uint8_t *p, size_t n, uint16_t crc)
{
    while (n--) {
        crc ^= (uint16_t)(*p++ << 8);
        for (int i = 0; i < 8; i++)
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
    }
    return crc;
}

static uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static void     wr16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }

int module_ee_decode(const uint8_t *buf, size_t n, module_id_t *out)
{
    memset(out, 0, sizeof(*out));
    if (n < MOD_EE_HDR_LEN) return MOD_EE_BAD_LEN;
    bool blank = true;
    for (size_t i = 0; i < MOD_EE_HDR_LEN; i++) if (buf[i] != 0xFFu) { blank = false; break; }
    if (blank) return MOD_EE_BLANK;
    if (memcmp(buf, k_magic, sizeof(k_magic)) != 0) return MOD_EE_BAD_MAGIC;
    size_t hl = buf[5];
    if (hl < MOD_EE_HDR_LEN || hl > n) return MOD_EE_BAD_LEN;
    if (module_ee_crc16(buf, hl - 2u, 0xFFFFu) != rd16(buf + hl - 2u)) return MOD_EE_BAD_CRC;
    out->version   = buf[4];
    out->type      = buf[6];
    out->rev_major = buf[7];
    out->rev_minor = buf[8];
    memcpy(out->serial, buf + 10, MOD_EE_SERIAL_LEN);
    out->serial[MOD_EE_SERIAL_LEN] = '\0';
    out->cal_len   = rd16(buf + 26);
    out->cal_crc   = rd16(buf + 28);
    return MOD_EE_OK;
}

void module_ee_encode(const module_id_t *id, uint8_t out[MOD_EE_HDR_LEN])
{
    memset(out, 0, MOD_EE_HDR_LEN);
    memcpy(out, k_magic, sizeof(k_magic));
    out[4] = MOD_EE_VERSION;
    out[5] = MOD_EE_HDR_LEN;
    out[6] = id->type;
    out[7] = id->rev_major;
    out[8] = id->rev_minor;
    size_t sl = strnlen(id->serial, MOD_EE_SERIAL_LEN);
    memcpy(out + 10, id->serial, sl);
    wr16(out + 26, id->cal_len);
    wr16(out + 28, id->cal_crc);
    wr16(out + 30, module_ee_crc16(out, 30, 0xFFFFu));
}

const char *module_ee_result_str(int rc)
{
    switch (rc) {
    case MOD_EE_OK:        return "ok";
    case MOD_EE_BLANK:     return "blank";
    case MOD_EE_BAD_MAGIC: return "bad magic";
    case MOD_EE_BAD_CRC:   return "bad crc";
    default:               return "bad length";
    }
}

const char *module_ee_type_str(uint8_t type)
{
    return type == MOD_TYPE_ANALOG ? "analog" : "unknown";
}

size_t module_ee_chunk(uint16_t addr, size_t len)
{
    size_t room = MOD_EE_PAGE - (addr % MOD_EE_PAGE);
    return len < room ? len : room;
}

bool module_ee_raw_write_ok(uint32_t addr, size_t len)
{
    return len > 0 && addr >= MOD_EE_CAL_ADDR && addr < MOD_EE_SIZE && len <= MOD_EE_SIZE - addr;
}
