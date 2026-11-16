/*
 * command_handler_esp.c — `esp_flash`: flash an ESP32-family DUT over its UART ROM bootloader.
 *
 *   {"cmd":"esp_flash","op":"sync","rx":R,"tx":T,"boot":B,"en":"nrst"|N}
 *       enters the loader and reads the chip (wiring test, flash untouched); poll status
 *   {"cmd":"esp_flash","op":"start","rx":R,"tx":T,"boot":B,"en":"nrst"|N,"offset":N,"len":N,
 *    "baud":460800,"flash_size":4194304,"run":true}
 *       writes the first len bytes a `load_bin` with "psram" staged, at offset; poll status
 *   {"cmd":"esp_flash","op":"status"}
 *       {"running":..,"phase":..,"done":..,"total":..,"chip":..,"result":..,"md5":..,...}
 *   {"cmd":"esp_flash","op":"abort"}
 *
 * rx/tx are the pod's side (rx = the DUT's TXD0).  boot is IO0 (IO9 on the C3/C6), en the pod's
 * reset pin ("nrst", the default) or an LA pin; both are driven open-drain.  The work runs in the
 * background (esp_dut_flash.c), so `start` and `sync` answer at once.  Every reply fits one cloud
 * frame, so the whole flow works over the cloud command channel.
 *
 * Runs on the hw worker task, like the rest of command_handler.
 */
#include "command_handler.h"
#include "command_handler_internal.h"
#include "esp_dut_flash.h"
#include "esp_rom_job.h"
#include "la_pins.h"
#include "bp_json.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static bool get_u32(const char *json, const char *key, uint32_t *out) {
    char s[16];
    if (!bp_json_get(json, key, s, sizeof(s))) return false;
    char *end = NULL;
    unsigned long v = strtoul(s, &end, 0);
    if (!end || *end != '\0') return false;
    *out = (uint32_t)v;
    return true;
}

static bool has_key(const char *json, const char *key) {
    char t[24];
    return bp_json_get(json, key, t, sizeof(t));
}

static bool get_la(int conn_id, const char *json, const char *key, uint8_t *out) {
    uint32_t v = 0;
    if (!get_u32(json, key, &v) || v < 1 || v > LA_PINS_COUNT) {
        char m[64];
        snprintf(m, sizeof(m), "missing or invalid %s (an LA pin, 1..%u)", key, LA_PINS_COUNT);
        send_error(conn_id, m);
        return false;
    }
    *out = (uint8_t)v;
    return true;
}

static void reply_status(int conn_id) {
    esp_dut_status_t st;
    esp_dut_flash_status(&st);
    char p[384];
    const char *res = !st.have_result || st.running ? NULL
                    : st.result == ESP_ROM_OK ? "ok"
                    : st.result == ESP_ROM_UNVERIFIED ? "unverified" : st.result_str;
    int n = snprintf(p, sizeof(p),
        "{\"running\":%s,\"op\":\"%s\",\"phase\":\"%s\",\"done\":%lu,\"total\":%lu,"
        "\"chip\":\"%s\",\"magic\":\"0x%08lx\",\"offset\":%lu,\"len\":%lu,\"baud\":%lu,"
        "\"resends\":%lu,\"ms\":%lu,\"md5\":\"%s\",\"result\":",
        st.running ? "true" : "false", st.sync_only ? "sync" : "start", st.phase,
        (unsigned long)st.done, (unsigned long)st.total, st.chip, (unsigned long)st.magic,
        (unsigned long)st.offset, (unsigned long)st.len, (unsigned long)st.baud,
        (unsigned long)st.resends, (unsigned long)st.ms, st.md5);
    if (n < 0 || (size_t)n >= sizeof(p)) { send_error(conn_id, "reply too large"); return; }
    snprintf(p + n, sizeof(p) - (size_t)n, res ? "\"%s\"}" : "null}", res ? res : "");
    send_ok_str(conn_id, p);
}

void handle_esp_flash(int conn_id, const char *json) {
    char op[12] = {0};
    if (!bp_json_get(json, "op", op, sizeof(op))) { send_error(conn_id, "missing op (sync, start, status, abort)"); return; }
    if (strcmp(op, "status") == 0) { reply_status(conn_id); return; }
    if (strcmp(op, "abort") == 0)  { esp_dut_flash_abort(); reply_status(conn_id); return; }
    bool sync = strcmp(op, "sync") == 0;
    if (!sync && strcmp(op, "start") != 0) { send_error(conn_id, "op must be sync, start, status or abort"); return; }
    if (!require_la_voltage(conn_id)) return;

    esp_dut_params_t p;
    memset(&p, 0, sizeof(p));
    if (!get_la(conn_id, json, "rx", &p.rx) || !get_la(conn_id, json, "tx", &p.tx) ||
        !get_la(conn_id, json, "boot", &p.boot)) return;
    char en[12] = {0};
    if (!bp_json_get(json, "en", en, sizeof(en)) || en[0] == '\0' || strcmp(en, "nrst") == 0) {
        p.en = ESP_DUT_EN_NRST;
    } else if (!get_la(conn_id, json, "en", &p.en)) {
        return;
    }
    p.run = true;
    if (!sync) {
        if (!get_u32(json, "len", &p.len) || p.len == 0) { send_error(conn_id, "missing or invalid len"); return; }
        if (has_key(json, "offset") && !get_u32(json, "offset", &p.offset)) {
            send_error(conn_id, "invalid offset"); return;
        }
        if (p.offset & 0xFFFu) { send_error(conn_id, "offset must be a multiple of 4096 (a flash sector)"); return; }
        p.baud = 460800;
        if (has_key(json, "baud") && (!get_u32(json, "baud", &p.baud) || p.baud < 9600 || p.baud > 2000000)) {
            send_error(conn_id, "baud must be 9600..2000000"); return;
        }
        p.flash_size = 4u * 1024u * 1024u;
        if (has_key(json, "flash_size") &&
            (!get_u32(json, "flash_size", &p.flash_size) || p.flash_size < 256u * 1024u ||
             p.flash_size > 128u * 1024u * 1024u || (p.flash_size & (p.flash_size - 1u)))) {
            send_error(conn_id, "flash_size must be a power of two, 256 KB..128 MB"); return;
        }
        if ((uint64_t)p.offset + p.len > p.flash_size) { send_error(conn_id, "offset + len is past flash_size"); return; }
        if (has_key(json, "run")) p.run = bp_json_flag(json, "run");
        /* The PSRAM bus: free, or held by this connection's own upload (load_bin's hold). */
        if (!command_handler_acquire_adc(conn_id)) { send_error(conn_id, "busy"); return; }
        command_handler_release_adc(conn_id);
    }

    char err[160];
    if (esp_dut_flash_start(&p, err, sizeof(err)) != 0) { send_error(conn_id, err); return; }
    char r[48];
    snprintf(r, sizeof(r), "{\"started\":true,\"op\":\"%s\"}", sync ? "sync" : "start");
    send_ok_str(conn_id, r);
}
