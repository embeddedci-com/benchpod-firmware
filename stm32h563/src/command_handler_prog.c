/*
 * command_handler_prog.c — `prog_flash`: flash a chip over its own programming pins (prog_flash.h).
 *
 *   {"cmd":"prog_flash","op":"info","proto":"c2","clk":3,"data":4}
 *       enters programming mode and reads the chip's ID (flash untouched); poll status
 *   {"cmd":"prog_flash","op":"start","proto":"c2","clk":3,"data":4,"mem":"flash","addr":0,
 *    "len":8192,"page":512,"erase":"chip","expect_id":"0x30","baud":0,"flags":0,"run":true}
 *       writes the first len bytes a `load_bin` with "psram" staged and verifies them; poll status
 * PIC and TPI take "reset": "nrst" (the pod's reset pin, the default) or an LA pin (open drain).
 *   {"cmd":"prog_flash","op":"status"}
 *   {"cmd":"prog_flash","op":"abort"}
 *
 * UPDI takes `data` only. Every reply fits one cloud frame. Runs on the hw worker.
 */
#include "command_handler.h"
#include "command_handler_internal.h"
#include "bp_errors.h"
#include "prog_flash.h"
#include "gw_caps.h"
#include "sensor_mgr.h"   /* sensor_mgr_digital_names: the bus-master gate */
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

/* "0x1e9421" or "1e9421": up to 8 bytes, in the order the chip sends them. */
static bool parse_id(const char *s, uint8_t *out, uint8_t *n) {
    if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) s += 2;
    size_t len = strlen(s);
    if (len == 0 || len % 2 || len > 16) return false;
    for (size_t i = 0; i < len / 2; i++) {
        char b[3] = { s[2 * i], s[2 * i + 1], 0 };
        char *end = NULL;
        unsigned long v = strtoul(b, &end, 16);
        if (!end || *end) return false;
        out[i] = (uint8_t)v;
    }
    *n = (uint8_t)(len / 2);
    return true;
}

static void reply_status(int conn_id) {
    prog_status_t st;
    prog_flash_status(&st);
    char id[20] = {0};
    for (uint8_t i = 0; i < st.id_len; i++) snprintf(id + 2 * i, 3, "%02x", st.id[i]);
    char p[400];
    int n = snprintf(p, sizeof(p),
        "{\"running\":%s,\"op\":\"%s\",\"proto\":\"%s\",\"phase\":\"%s\",\"done\":%lu,\"total\":%lu,"
        "\"id\":\"%s\",\"ms\":%lu,\"protos\":[",
        st.running ? "true" : "false", st.op ? st.op : "", st.proto ? st.proto : "",
        st.phase ? st.phase : "", (unsigned long)st.done, (unsigned long)st.total, id,
        (unsigned long)st.ms);
    for (size_t i = 0; i < prog_proto_count() && n > 0 && (size_t)n < sizeof(p); i++)
        n += snprintf(p + n, sizeof(p) - (size_t)n, "%s\"%s\"", i ? "," : "", prog_proto_at(i)->name);
    if (n > 0 && (size_t)n < sizeof(p)) {
        if (st.rc == PROG_E_VERIFY)
            n += snprintf(p + n, sizeof(p) - (size_t)n, "],\"at_addr\":%lu,\"result\":", (unsigned long)st.at_addr);
        else
            n += snprintf(p + n, sizeof(p) - (size_t)n, "],\"result\":");
    }
    if (n < 0 || (size_t)n >= sizeof(p)) { bp_err_reply_too_large(conn_id); return; }
    bool res = st.have_result && !st.running && st.result;
    snprintf(p + n, sizeof(p) - (size_t)n, res ? "\"%s\"}" : "null}", res ? st.result : "");
    send_ok_str(conn_id, p);
}

static const char *const k_mems[PROG_MEM_COUNT] = { "flash", "eeprom", "config", "user" };

void handle_prog_flash(int conn_id, const char *json) {
    char op[12] = {0};
    if (!bp_json_get(json, "op", op, sizeof(op))) { send_error(conn_id, "missing op (info, start, status, abort)"); return; }
    if (strcmp(op, "status") == 0) { reply_status(conn_id); return; }
    if (strcmp(op, "abort") == 0)  { prog_flash_abort(); reply_status(conn_id); return; }
    bool info = strcmp(op, "info") == 0;
    if (!info && strcmp(op, "start") != 0) { send_error(conn_id, "op must be info, start, status or abort"); return; }
    if (!require_la_voltage(conn_id)) return;

    prog_params_t p;
    memset(&p, 0, sizeof(p));
    char proto[12] = {0};
    bp_json_get(json, "proto", proto, sizeof(proto));
    p.proto = prog_proto_find(proto);
    if (!p.proto) { send_error(conn_id, "missing or unknown proto (updi, pic, c2, tpi, pdi)"); return; }
    if (!get_la(conn_id, json, "data", &p.data)) return;
    if (!p.proto->uart && !get_la(conn_id, json, "clk", &p.clk)) return;
    char rs[8] = {0};
    if (bp_json_get(json, "reset", rs, sizeof(rs)) && rs[0] && strcmp(rs, "nrst") != 0 &&
        !get_la(conn_id, json, "reset", &p.reset)) return;
    p.op = info ? PROG_OP_INFO : PROG_OP_START;
    p.run = true;
    if (has_key(json, "run")) p.run = bp_json_flag(json, "run");
    if (has_key(json, "baud") && (!get_u32(json, "baud", &p.baud) || p.baud > 2000000)) {
        send_error(conn_id, "invalid baud"); return;
    }
    /* pic: the byte address of the config word with the code-protect bit (prog_pic.h); it travels
       in the same context field as UPDI's baud, which pic does not use. */
    if (has_key(json, "cp_addr")) {
        if (strcmp(p.proto->name, "pic") != 0) { send_error(conn_id, "cp_addr is for pic only"); return; }
        if (has_key(json, "baud")) { send_error(conn_id, "pic takes cp_addr, not baud"); return; }
        if (!get_u32(json, "cp_addr", &p.baud)) { send_error(conn_id, "invalid cp_addr"); return; }
    }
    if (has_key(json, "flags") && !get_u32(json, "flags", &p.flags)) { send_error(conn_id, "invalid flags"); return; }
    char idv[24] = {0};
    if (bp_json_get(json, "expect_id", idv, sizeof(idv)) && idv[0] &&
        !parse_id(idv, p.expect_id, &p.expect_len)) {
        send_error(conn_id, "expect_id must be hex bytes, up to 8 (\"0x1e9421\")"); return;
    }

    if (!info) {
        char mem[10];
        if (!bp_json_get(json, "mem", mem, sizeof(mem))) strcpy(mem, "flash");   /* absent: flash */
        p.mem = PROG_MEM_COUNT;
        for (uint8_t i = 0; i < PROG_MEM_COUNT; i++) if (strcmp(mem, k_mems[i]) == 0) p.mem = i;
        if (p.mem == PROG_MEM_COUNT) { send_error(conn_id, "mem must be flash, eeprom, config or user"); return; }
        if (!get_u32(json, "len", &p.len) || p.len == 0) { send_error(conn_id, "missing or invalid len"); return; }
        if (has_key(json, "addr") && !get_u32(json, "addr", &p.addr)) { send_error(conn_id, "invalid addr"); return; }
        uint32_t page = 0;
        if (!get_u32(json, "page", &page) || page == 0 || page > 4096) {
            send_error(conn_id, "missing or invalid page (the chip's write page in bytes, 1..4096)"); return;
        }
        p.page = (uint16_t)page;
        char er[8];
        if (!bp_json_get(json, "erase", er, sizeof(er))) strcpy(er, "page");      /* absent: page */
        if (strcmp(er, "chip") == 0)      p.erase = PROG_ERASE_CHIP;
        else if (strcmp(er, "page") == 0) p.erase = PROG_ERASE_PAGE;
        else if (strcmp(er, "none") == 0) p.erase = PROG_ERASE_NONE;
        else { send_error(conn_id, "erase must be chip, page or none"); return; }
        /* The PSRAM bus: free, or held by this connection's own upload (load_bin's hold). */
        if (!command_handler_acquire_adc(conn_id)) { bp_err_pod_busy(conn_id); return; }
        command_handler_release_adc(conn_id);
    }

    /* The two-wire protocols run on the bus-master engine, which gateware v65 left out of the
       emulation image (UPDI uses the UART proxy, in every image).  No automatic swap for a start:
       its data is staged in PSRAM, which a reconfiguration overwrites. */
    if (!p.proto->uart && !gw_caps()->bus_masters) {
        if (info) {
            if (!require_bus_masters(conn_id, "prog_flash")) return;
        } else {
            char names[96];
            if (sensor_mgr_digital_names(names, sizeof(names))) bp_err_fpga_image_sensors_first(conn_id, "prog_flash", "deep");
            else bp_err_fpga_image_required(conn_id, "prog_flash", "deep");
            return;
        }
    }
    char err[160] = {0};
    if (prog_flash_start(&p, err, sizeof(err)) != 0) { send_error(conn_id, err); return; }
    char r[64];
    snprintf(r, sizeof(r), "{\"started\":true,\"op\":\"%s\",\"proto\":\"%s\"}", op, p.proto->name);
    send_ok_str(conn_id, r);
}
