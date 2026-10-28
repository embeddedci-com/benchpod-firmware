/*
 * command_handler_module.c — the `module` and `module_power` JSON commands: digital v3r3's analog
 * module link, its EEPROM and the two rails it switches (module_link.h).
 */
#include "command_handler.h"
#include "command_handler_internal.h"
#include "at_driver.h"
#include "bp_json.h"
#include "board_variant.h"
#include "module_link.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MODULE_READ_MAX   128u   /* bytes per eeprom read: 256 hex characters in the reply */
#define MODULE_WRITE_MAX  64u    /* bytes per eeprom write: the command line stays under 384 */

static void send_emitted(int conn_id, bp_emit_t *e)
{
    if (!bp_emit_ok(e)) { send_error(conn_id, "module: reply too large"); return; }
    if (at_send_data(conn_id, (const uint8_t *)e->buf, bp_emit_len(e)) != 0)
        at_close_connection(conn_id);
}

/* The module object of `status` and `module`: {"present":..,...}.  Only on digital v3r3. */
void module_emit_state(bp_emit_t *e, bool full)
{
    module_id_t id;
    int rc = module_id(&id);
    bp_emit(e, "{\"present\":%s,\"analog\":%s,\"power\":%s,\"bridge_power\":%s,\"eeprom\":",
            module_present_now() ? "true" : "false",
            board_analog_is_module() ? "true" : "false",
            module_power_on() ? "true" : "false",
            bridge_power_on() ? "true" : "false");
    if (rc == MOD_EE_NO_CHIP) {
        bp_emit_raw(e, "\"none\"}");
        return;
    }
    bp_emit_jstr(e, module_ee_result_str(rc));
    if (rc == MOD_EE_OK && full) {
        bp_emit(e, ",\"type\":\"%s\",\"rev\":\"%u.%u\",\"serial\":", module_ee_type_str(id.type),
                id.rev_major, id.rev_minor);
        bp_emit_jstr(e, id.serial);
        bp_emit(e, ",\"layout\":%u,\"cal_len\":%u,\"cal_ok\":%s", id.version, (unsigned)id.cal_len,
                module_cal_valid() ? "true" : "false");
    }
    bp_emit_raw(e, "}");
}

static bool num_field(const char *json, const char *key, long *out)
{
    char v[16] = {0};
    if (!bp_json_get(json, key, v, sizeof(v)) || !v[0]) return false;
    char *end;
    *out = strtol(v, &end, 10);
    return *end == '\0';
}

static int hex_nibble(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

/* Hex string -> bytes; -1 on odd length, a bad digit or more than cap bytes. */
static int hex_decode(const char *s, uint8_t *out, size_t cap)
{
    size_t n = strlen(s);
    if (n % 2u || n / 2u > cap) return -1;
    for (size_t i = 0; i < n / 2u; i++) {
        int hi = hex_nibble(s[2 * i]), lo = hex_nibble(s[2 * i + 1]);
        if (hi < 0 || lo < 0) return -1;
        out[i] = (uint8_t)(hi << 4 | lo);
    }
    return (int)(n / 2u);
}

static void reply_state(int conn_id)
{
    char resp[384];
    bp_emit_t e;
    bp_emit_init(&e, resp, sizeof(resp));
    bp_emit_raw(&e, "{\"status\":\"ok\",\"data\":");
    module_emit_state(&e, true);
    bp_emit_raw(&e, "}\n");
    send_emitted(conn_id, &e);
}

/* {"cmd":"module"}                                        the state (as in status)
   {"cmd":"module","eeprom":"read","addr":N,"len":M}       raw bytes, hex (M <= 128)
   {"cmd":"module","eeprom":"write","addr":N,"data":"hex"} raw bytes into the calibration area
   {"cmd":"module","eeprom":"id","type":"analog","rev":"1.0","serial":"..."}
   {"cmd":"module","eeprom":"cal_commit","len":N}          record the calibration blob */
void handle_module(int conn_id, const char *json)
{
    if (!module_link_board()) {
        send_error(conn_id, "module: this BenchPod has no module link (digital v3r3 only)");
        return;
    }
    char op[16] = {0};
    if (!bp_json_get(json, "eeprom", op, sizeof(op))) { reply_state(conn_id); return; }
    if (!module_present_now() || !module_power_on()) {
        send_error(conn_id, "module: no module plugged in, or module power is off");
        return;
    }

    if (strcmp(op, "read") == 0) {
        long addr = 0, len = 0;
        if (!num_field(json, "addr", &addr) || !num_field(json, "len", &len) ||
            addr < 0 || len < 1 || len > (long)MODULE_READ_MAX || addr + len > (long)MOD_EE_SIZE) {
            send_error(conn_id, "module: read needs addr and len (1..128) inside the 4096-byte EEPROM");
            return;
        }
        uint8_t buf[MODULE_READ_MAX];
        if (module_eeprom_read((uint16_t)addr, buf, (size_t)len) != 0) {
            send_error(conn_id, "module: EEPROM read failed");
            return;
        }
        char resp[96 + 2 * MODULE_READ_MAX];
        bp_emit_t e;
        bp_emit_init(&e, resp, sizeof(resp));
        bp_emit(&e, "{\"status\":\"ok\",\"data\":{\"addr\":%ld,\"len\":%ld,\"data\":\"", addr, len);
        for (long i = 0; i < len; i++) bp_emit(&e, "%02x", buf[i]);
        bp_emit_raw(&e, "\"}}\n");
        send_emitted(conn_id, &e);
        return;
    }

    if (strcmp(op, "write") == 0) {
        long addr = 0;
        char hex[2 * MODULE_WRITE_MAX + 2] = {0};
        uint8_t buf[MODULE_WRITE_MAX];
        int n;
        if (!num_field(json, "addr", &addr) || !bp_json_get(json, "data", hex, sizeof(hex)) ||
            (n = hex_decode(hex, buf, sizeof(buf))) <= 0) {
            send_error(conn_id, "module: write needs addr and data (hex, 1..64 bytes)");
            return;
        }
        if (addr < 0 || !module_ee_raw_write_ok((uint32_t)addr, (size_t)n)) {
            send_error(conn_id, "module: raw writes go to the calibration area, 0x100..0xfff "
                                "(the header is written by id and cal_commit)");
            return;
        }
        if (module_eeprom_write((uint16_t)addr, buf, (size_t)n) != 0) {
            send_error(conn_id, "module: EEPROM write failed");
            return;
        }
        char data[48];
        snprintf(data, sizeof(data), "{\"addr\":%ld,\"len\":%d}", addr, n);
        send_ok_str(conn_id, data);
        return;
    }

    if (strcmp(op, "id") == 0) {
        char type[12] = {0}, rev[12] = {0}, serial[MOD_EE_SERIAL_LEN + 2] = {0};
        unsigned maj = 0, min = 0;
        if (!bp_json_get(json, "type", type, sizeof(type))) strcpy(type, "analog");
        if (strcmp(type, "analog") != 0) {
            send_error(conn_id, "module: type must be analog");
            return;
        }
        if (!bp_json_get(json, "rev", rev, sizeof(rev)) || sscanf(rev, "%u.%u", &maj, &min) != 2 ||
            maj > 255u || min > 255u) {
            send_error(conn_id, "module: id needs rev as \"major.minor\", for example \"1.0\"");
            return;
        }
        if (!bp_json_get(json, "serial", serial, sizeof(serial)) || strlen(serial) > MOD_EE_SERIAL_LEN) {
            send_error(conn_id, "module: id needs serial (up to 16 characters)");
            return;
        }
        if (module_id_write(MOD_TYPE_ANALOG, (uint8_t)maj, (uint8_t)min, serial) != 0) {
            send_error(conn_id, "module: EEPROM write failed");
            return;
        }
        reply_state(conn_id);
        return;
    }

    if (strcmp(op, "cal_commit") == 0) {
        long len = -1;
        if (!num_field(json, "len", &len) || len < 0 || len > (long)MOD_EE_CAL_MAX) {
            send_error(conn_id, "module: cal_commit needs len (0..3840)");
            return;
        }
        if (module_cal_commit((uint16_t)len) != 0) {
            send_error(conn_id, "module: EEPROM write failed");
            return;
        }
        reply_state(conn_id);
        return;
    }

    send_error(conn_id, "module: eeprom must be read, write, id or cal_commit");
}

/* {"cmd":"module_power"[,"module":true|false][,"bridge":true|false]} -> the two rails */
void handle_module_power(int conn_id, const char *json)
{
    if (!module_link_board()) {
        send_error(conn_id, "module_power: this BenchPod has no module link (digital v3r3 only)");
        return;
    }
    char v[8] = {0};
    if (bp_json_get(json, "bridge", v, sizeof(v))) {
        if (strcmp(v, "true") != 0 && strcmp(v, "false") != 0) {
            send_error(conn_id, "module_power: bridge must be true or false");
            return;
        }
        (void)bridge_power_set(strcmp(v, "true") == 0);
    }
    if (bp_json_get(json, "module", v, sizeof(v))) {
        if (strcmp(v, "true") != 0 && strcmp(v, "false") != 0) {
            send_error(conn_id, "module_power: module must be true or false");
            return;
        }
        if (module_power_set(strcmp(v, "true") == 0) == -2) {
            send_error(conn_id, "module_power: module power is on, but U58 does not answer");
            return;
        }
    }
    char data[64];
    snprintf(data, sizeof(data), "{\"module\":%s,\"bridge\":%s}",
             module_power_on() ? "true" : "false", bridge_power_on() ? "true" : "false");
    send_ok_str(conn_id, data);
}
