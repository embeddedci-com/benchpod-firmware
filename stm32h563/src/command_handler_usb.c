/*
 * command_handler_usb.c — the usb_host command: the pod as a USB host for the DUT (usb_host.h).
 *
 *   {"cmd":"usb_host"}                                         -> the state below
 *   {"cmd":"usb_host","enabled":true[,"port":"usb_a|usb_c"][,"timeout_s":N]}
 *   {"cmd":"usb_host","enabled":false}
 *     -> {"mode":"device|starting|host|stopping","port":..,"remaining_s":N,"host_ms":N,
 *         "enumerations":N,"attached":bool[,"device":{"vid","pid","speed","usb","release",
 *         "class","subclass","protocol","ep0_size","configurations","manufacturer","product",
 *         "serial","descriptor_ms","mounted_ms"[,"config":{..,"interfaces":[..]}]}]}
 *
 * "port" defaults to usb_a on a board with the USB switch; elsewhere it must be given as usb_c
 * (the USB-C adapter rig). Starting again on the same port renews the timeout (default 60 s,
 * at most 3600 s); when it runs out the pod goes back to being a USB device by itself. The
 * switch happens on the usb task right after this reply; poll to see "host" and the device.
 *
 * Only in USB_HOST=1 builds (development, not hardware-tested yet). Elsewhere the verb answers
 * like an unknown command, so clients see the same reply as from firmware without it.
 */
#include "command_handler.h"
#include "command_handler_internal.h"
#include "net_server.h"   /* net_conn_on_usb */
#include "bp_json.h"
#include "bp_errors.h"
#include "bp_limits.h"
#if defined(BENCHPOD_USB_HOST)
#include "usb_host.h"

#include "FreeRTOS.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void usb_host_reply(int conn_id)
{
    /* From the heap, not .bss (static RAM is full) and not the net task's stack. */
    const size_t cap = BP_CLOUD_REPLY_MAX - 64u;
    char *data = pvPortMalloc(cap);
    if (!data) { send_error(conn_id, "usb_host: out of memory"); return; }
    usb_host_info_t info;
    usb_host_info(&info);
    if (usb_host_reply_json(&info, data, cap) == 0) bp_err_reply_too_large(conn_id);
    else send_ok_large(conn_id, data);
    vPortFree(data);
}

#if defined(BENCHPOD_USB_HOST_LOG)
#include "usb_host_log.h"
/* {"cmd":"usb_host","log":true[,"clear":true]}: TinyUSB's host log, JSON-escaped (development builds). */
static void usb_host_log_reply(int conn_id, bool clear)
{
    const size_t cap = BP_CLOUD_REPLY_MAX - 64u;
    char *raw = pvPortMalloc(1200), *data = pvPortMalloc(cap);
    if (!raw || !data) { vPortFree(raw); vPortFree(data); send_error(conn_id, "usb_host: out of memory"); return; }
    size_t n = usb_host_log_read(raw, 1200), o = 0;
    o += (size_t)snprintf(data, cap, "{\"log\":\"");
    for (size_t i = 0; i < n && o + 8 < cap; i++) {
        unsigned char c = (unsigned char)raw[i];
        if (c == '"' || c == '\\') { data[o++] = '\\'; data[o++] = (char)c; }
        else if (c == '\n') { data[o++] = '\\'; data[o++] = 'n'; }
        else if (c < 0x20 || c > 0x7E) data[o++] = '?';
        else data[o++] = (char)c;
    }
    snprintf(data + o, cap - o, "\"}");
    if (clear) usb_host_log_clear();
    send_ok_large(conn_id, data);
    vPortFree(raw);
    vPortFree(data);
}
#endif

void handle_usb_host(int conn_id, const char *json)
{
    char v[16] = {0};
#if defined(BENCHPOD_USB_HOST_LOG)
    if (bp_json_get(json, "log", v, sizeof(v)) && !strcmp(v, "true")) {
        char c[8] = {0};
        usb_host_log_reply(conn_id, bp_json_get(json, "clear", c, sizeof(c)) && !strcmp(c, "true"));
        return;
    }
    memset(v, 0, sizeof(v));
#endif
    if (!bp_json_get(json, "enabled", v, sizeof(v))) { usb_host_reply(conn_id); return; }
    if (!strcmp(v, "false")) {
        usb_host_stop();
        usb_host_reply(conn_id);
        return;
    }
    if (strcmp(v, "true") != 0) { send_error(conn_id, "usb_host: enabled must be true or false"); return; }

    /* The pod leaves the computer's USB while it hosts: the asker would never see the reply. */
    if (net_conn_on_usb(conn_id)) {
        bp_err_usb_host_over_usb(conn_id);
        return;
    }

    usb_host_port_t port = usb_host_port_available(USB_HOST_PORT_USB_A) ? USB_HOST_PORT_USB_A
                                                                        : USB_HOST_PORT_NONE;
    char p[12] = {0};
    if (bp_json_get(json, "port", p, sizeof(p))) {
        port = usb_host_port_from_name(p);
        if (port == USB_HOST_PORT_NONE) { send_error(conn_id, "usb_host: port must be usb_a or usb_c"); return; }
    }
    if (port == USB_HOST_PORT_NONE) {
        bp_err_missing_argument(conn_id, "port");
        return;
    }

    uint32_t timeout_s = 0;
    char t[12] = {0};
    if (bp_json_get(json, "timeout_s", t, sizeof(t))) {
        char *end = NULL;
        long n = strtol(t, &end, 10);
        if (!end || *end || n < 1 || n > (long)USB_HOST_TIMEOUT_MAX_S) {
            send_error(conn_id, "usb_host: timeout_s must be 1 to 3600");
            return;
        }
        timeout_s = (uint32_t)n;
    }

    switch (usb_host_start(port, timeout_s)) {
    case USB_HOST_OK:
        usb_host_reply(conn_id);
        return;
    case USB_HOST_ERR_PORT:
        bp_err_usb_host_port_unavailable(conn_id, usb_host_port_name(port));
        return;
    case USB_HOST_ERR_CLOCK:
        bp_err_usb_host_no_crystal(conn_id);
        return;
    case USB_HOST_ERR_OTHER_PORT: {
        usb_host_info_t info;
        usb_host_info(&info);
        bp_err_usb_host_other_port(conn_id, usb_host_port_name(info.port));
        return;
    }
    default:
        send_error(conn_id, "usb_host: timeout_s must be 1 to 3600");
        return;
    }
}

/* ---- usb_flash: flash a board over the pod's USB host port (usb_flash.h) ----------------------
 *   {"cmd":"usb_flash","mode":"uf2","len":N}                       UF2 bootloaders
 *   {"cmd":"usb_flash","mode":"file","name":"FIRMWARE.BIN","len":N}  drag-and-drop (DAPLink, ...)
 *   {"cmd":"usb_flash","mode":"dfu","address":"0x08000000","len":N} USB DFU / ST DfuSe
 *   {"cmd":"usb_flash","mode":"teensy","address":0,"len":N[,"block":512]}  Teensy HalfKay
 *   {"cmd":"usb_flash","mode":"avr","len":N}                       Arduino AVR bootloaders
 *   {"cmd":"usb_flash","mode":"samd","address":"0x2000","len":N}   Arduino SAMD bootloader
 *   {"cmd":"usb_flash","mode":"esp","address":0,"len":N}           ESP32 ROM over USB serial
 *   {"cmd":"usb_flash","mode":"picoboot","address":"0x10000000","len":N}  RP2040/RP2350 PICOBOOT
 *   optional "leave":false (dfu, teensy, picoboot): stay in the loader
 *   optional "check_fail_txt":true (file): read FAIL.TXT after the drive comes back
 *   optional "verify":true (uf2): compare CURRENT.UF2 when the drive is still a loader
 *   optional "time":N (file): seconds since 1970 (UTC) for the new file's dates (default 2026-01-01)
 *   {"cmd":"usb_flash"}                                            the state
 *   {"cmd":"usb_flash","abort":true}
 * The file is the first len bytes a load_bin with "psram" staged; usb_host must be on with a
 * drive mounted. Refusals: BP0084 (usb_flash_refused), BP0083 (the W25Q code), BP0048 (bus busy).
 */
#include "usb_flash.h"
#include "msc_flash.h"
#include "usb_serial.h"
#include "usb_check.h"
#include "usb_hid.h"
#include "task.h"

static void usb_flash_reply(int conn_id)
{
    usb_flash_status_t st;
    usb_flash_status(&st);
    char data[512];
    snprintf(data, sizeof(data),
             "{\"running\":%s,\"have_result\":%s,\"mode\":\"%s\",\"name\":\"%s\",\"len\":%lu,"
             "\"address\":%lu,\"protocol\":\"%s\",\"phase\":\"%s\",\"done\":%lu,\"total\":%lu,"
             "\"result\":\"%s\",\"ok\":%s,\"ms\":%lu,\"note\":\"%s\"}",
             st.running ? "true" : "false", st.have_result ? "true" : "false", st.mode, st.name,
             (unsigned long)st.len, (unsigned long)st.address, st.protocol, st.phase, (unsigned long)st.done,
             (unsigned long)st.total, st.result,
             (!st.running && st.have_result && st.rc == 0) ? "true" : "false", (unsigned long)st.ms, st.note);
    send_ok_large(conn_id, data);
}

void handle_usb_flash(int conn_id, const char *json)
{
    if (bp_json_flag(json, "abort")) { usb_flash_abort(); usb_flash_reply(conn_id); return; }
    char mode[10] = {0};
    if (!bp_json_get(json, "mode", mode, sizeof(mode))) { usb_flash_reply(conn_id); return; }
    usb_flash_params_t p = {0};
    int m = usb_flash_mode_from_name(mode);
    if (m < 0) { bp_err_usb_flash_refused(conn_id, "mode must be uf2, file, dfu, teensy, avr, samd, esp or picoboot"); return; }
    p.mode = (uint8_t)m;
    (void)bp_json_get(json, "name", p.name, sizeof(p.name));
    char v[16] = {0};
    if (bp_json_get(json, "address", v, sizeof(v))) p.address = (uint32_t)strtoul(v, NULL, 0);
    if (bp_json_get(json, "block", v, sizeof(v))) p.block = (uint16_t)strtoul(v, NULL, 10);
    p.leave = !(bp_json_get(json, "leave", v, sizeof(v)) && !strcmp(v, "false"));
    p.check_fail_txt = bp_json_flag(json, "check_fail_txt");
    p.verify = bp_json_flag(json, "verify");
    {   /* optional "time": seconds since 1970 (UTC) for the new file's dates; the pod has no clock of its own */
        char tv[16] = {0};
        if (bp_json_get(json, "time", tv, sizeof(tv))) {
            char *end = NULL;
            unsigned long tt = strtoul(tv, &end, 10);
            if (!end || *end || tv[0] == '-' || tt < 1577836800ul || tt > 4102444799ul) {
                send_error(conn_id, "usb_flash: time must be seconds since 1970 between 2020 and 2099");
                return;
            }
            p.time = (uint32_t)tt;
        }
    }
    char n[16] = {0};
    if (!bp_json_get(json, "len", n, sizeof(n))) { bp_err_missing_argument(conn_id, "len"); return; }
    p.len = (uint32_t)strtoul(n, NULL, 10);
    if (usb_serial_con_is_open()) {
        bp_err_usb_flash_refused(conn_id, "the USB serial port is open (uart_proxy_start with port usb)");
        return;
    }
    if (!usb_flash_busy()) {
        const char *busy = bus_busy_reason();
        if (busy) { send_error(conn_id, busy); return; }
    }
    const char *why = NULL;
    int rc = usb_flash_start(&p, &why);
    if (rc == -2) { bp_err_xip_unavailable(conn_id, why); return; }
    if (rc != 0) { bp_err_usb_flash_refused(conn_id, why ? why : "refused"); return; }
    usb_flash_reply(conn_id);
}

/* ---- usb_serial: the state of the board's USB serial port (usb_serial.h) ---------------------
 *   {"cmd":"usb_serial"}  -> {"open","baud","frame","dtr","rts","bridge","rx_buffered","rx_dropped",
 *                             "tx_total","rx_total","port_present"}
 * Read only. The console itself is opened by uart_proxy_start with "port":"usb"
 * (command_handler_uart.c), which carries the bytes; "open" says whether it is open. */
#include "usb_serial.h"

void handle_usb_serial(int conn_id, const char *json)
{
    (void)json;
    usb_serial_state_t st;
    usb_serial_con_state(&st);
    char data[256];
    if (usb_serial_state_json(&st, data, sizeof(data)) == 0) { bp_err_reply_too_large(conn_id); return; }
    send_ok_str(conn_id, data);
}

/* ---- usb_check: checks of the board's own USB firmware (usb_check.h) -------------------------
 *   {"cmd":"usb_check","test":"enumerate","count":10,"off_ms":500,"timeout_ms":5000}
 *   {"cmd":"usb_check","test":"power","duration_ms":3000}
 *   {"cmd":"usb_check","test":"suspend","settle_ms":200,"duration_ms":1000}
 *   {"cmd":"usb_check","test":"reset","count":20}
 *   {"cmd":"usb_check","test":"replug","count":10,"off_ms":[5,20,100,500]}
 *   {"cmd":"usb_check"}                  the state and the last result
 *   {"cmd":"usb_check","abort":true}
 * Refusals: BP0087 (usb_check_refused), BP0088 (not powered from eFuse 1), BP0083 (the W25Q code:
 * the check's steps run from the usb image in an XIP session).
 */

static void usb_check_reply_send(int conn_id)
{
    const size_t cap = BP_CLOUD_REPLY_MAX - 64u;
    char *data = pvPortMalloc(cap);
    if (!data) { send_error(conn_id, "usb_check: out of memory"); return; }
    if (usb_check_reply(data, cap) == 0) bp_err_reply_too_large(conn_id);
    else send_ok_large(conn_id, data);
    vPortFree(data);
}

void handle_usb_check(int conn_id, const char *json)
{
    if (bp_json_flag(json, "abort")) { usb_check_abort(); usb_check_reply_send(conn_id); return; }
    char t[12] = {0};
    if (!bp_json_get(json, "test", t, sizeof(t))) { usb_check_reply_send(conn_id); return; }
    uc_args_t a;
    char why[96] = {0};
    if (uc_parse_args(json, &a, why, sizeof(why)) != 0) { bp_err_usb_check_refused(conn_id, why); return; }
    const char *w = NULL;
    int rc = usb_check_start(&a, &w);
    if (rc == -2) { bp_err_usb_check_no_efuse_power(conn_id); return; }
    if (rc == -3) { bp_err_xip_unavailable(conn_id, w ? w : "the xip blob is missing"); return; }
    if (rc != 0) { bp_err_usb_check_refused(conn_id, w ? w : "refused"); return; }
    usb_check_reply_send(conn_id);
}

/* ---- usb_hid: the HID interfaces of the hosted board and their input reports (usb_hid.h) ------
 *   {"cmd":"usb_hid"[,"instance":i][,"offset":N]}  interfaces + report descriptors (hex)
 *   {"cmd":"usb_hid","capture":true}    start catching input reports of every HID interface into
 *                                       one ring, until capture false, a detach or the end of host mode
 *   {"cmd":"usb_hid","capture":false}   stop and free the ring
 *   {"cmd":"usb_hid","read":true[,"count":N][,"timeout_ms":T][,"instance":i]}  drain the ring
 * count 1..32 (default 1), timeout_ms 1..10000 (default 2000): with an empty ring, wait up to this
 * long for a report. instance: only that interface's reports (default all). A read without a
 * running capture starts it and leaves it running. Refused while a usb_flash or usb_check runs. */
#include "usb_hid.h"
#include "task.h"
#include "usb_msc.h"

#define HID_DESC_MAX 1024u

static unsigned hid_num(const char *json, const char *key, unsigned def, unsigned lo, unsigned hi, bool *bad)
{
    char v[12] = {0};
    if (!bp_json_get(json, key, v, sizeof(v))) return def;
    char *end = NULL;
    long n = strtol(v, &end, 10);
    if (!end || *end || n < (long)lo || n > (long)hi) { *bad = true; return def; }
    return (unsigned)n;
}

void handle_usb_hid(int conn_id, const char *json)
{
    usb_host_info_t hi;
    usb_host_info(&hi);
    char v[16] = {0};
    /* capture:false works whatever the state: it frees the ring. */
    const bool has_capture = bp_json_get(json, "capture", v, sizeof(v));
    if (has_capture && !strcmp(v, "false")) {
        usb_hid_capture_stop();
        char st[64];
        if (usb_hid_capture_state(st, sizeof(st)) == 0) { bp_err_reply_too_large(conn_id); return; }
        send_ok_str(conn_id, st);
        return;
    }
    if (hi.mode != USB_HOST_MODE_HOST) { send_error(conn_id, "usb_hid: USB host mode is not on (usb_host)"); return; }
    bool bad = false;
    memset(v, 0, sizeof(v));
    const bool want_inst = bp_json_get(json, "instance", v, sizeof(v));
    const unsigned inst = hid_num(json, "instance", 0, 0, USB_HID_MAX_ITF - 1u, &bad);
    if (bad) { send_error(conn_id, "usb_hid: instance must be 0 or 1"); return; }
    usb_hid_info_t info[USB_HID_MAX_ITF];
    unsigned n = usb_hid_interfaces(info), m = 0;
    for (unsigned i = 0; i < n; i++)
        if (!want_inst || info[i].inst == inst) info[m++] = info[i];
    n = m;
    if (want_inst && n == 0) { send_error(conn_id, "usb_hid: no such HID instance"); return; }

    const size_t cap = BP_CLOUD_REPLY_MAX - 64u;
    char *data = pvPortMalloc(cap);
    if (!data) { send_error(conn_id, "usb_hid: out of memory"); return; }
    size_t len = 0;

    memset(v, 0, sizeof(v));
    const bool want_read = bp_json_get(json, "read", v, sizeof(v)) && !strcmp(v, "true");
    memset(v, 0, sizeof(v));
    const bool want_start = has_capture && bp_json_get(json, "capture", v, sizeof(v)) && !strcmp(v, "true");
    if (usb_flash_busy() || usb_check_busy()) {
        vPortFree(data);
        send_error(conn_id, "usb_hid: a usb_flash or usb_check is running");
        return;
    }
    if (want_read || want_start) {
        if (n == 0) { vPortFree(data); send_error(conn_id, "usb_hid: no HID interface is mounted"); return; }
        int rc = usb_hid_capture_start();   /* 1: it already runs */
        if (rc < 0) {
            vPortFree(data);
            send_error(conn_id, rc == -1 ? "usb_hid: out of memory" : "usb_hid: no HID interface is mounted");
            return;
        }
    }
    if (want_read) {
        unsigned count = hid_num(json, "count", 1, 1, USB_HID_RING_MAX, &bad);
        unsigned timeout = hid_num(json, "timeout_ms", 2000, 1, 10000, &bad);
        if (bad) { vPortFree(data); send_error(conn_id, "usb_hid: count must be 1 to 32 and timeout_ms 1 to 10000"); return; }
        uint8_t mask = 0;
        for (unsigned i = 0; i < n; i++) mask |= (uint8_t)(1u << info[i].inst);
        const TickType_t t0 = xTaskGetTickCount();
        while (usb_hid_capture_active() && usb_hid_capture_ready(mask) == 0 &&
               xTaskGetTickCount() - t0 < pdMS_TO_TICKS(timeout)) vTaskDelay(2);
        len = usb_hid_capture_read(mask, count, data, cap);
        if (len == 0)     /* the capture ended while waiting (the board left): say so */
            len = (size_t)snprintf(data, cap, "{\"reports\":[],\"count\":0,\"dropped\":0,\"capturing\":false}");
    } else if (want_start) {
        len = usb_hid_capture_state(data, cap);
    } else {
        usb_hid_itf_t it[USB_HID_MAX_ITF];
        uint8_t *buf = n ? pvPortMalloc(n * HID_DESC_MAX) : NULL;
        if (n && !buf) { vPortFree(data); send_error(conn_id, "usb_hid: out of memory"); return; }
        for (unsigned i = 0; i < n; i++) {
            /* GET_DESCRIPTOR(report) on the interface, over the control pipe. */
            uint8_t *d = buf + i * HID_DESC_MAX;
            const uint8_t setup[8] = { 0x81, 6, 0, 0x22, info[i].itf, 0, (uint8_t)HID_DESC_MAX, (uint8_t)(HID_DESC_MAX >> 8) };
            uint32_t got = 0;
            if (usb_ctrl_io(setup, d, &got, 1000) != 0) got = 0;
            it[i] = (usb_hid_itf_t){ info[i].inst, info[i].itf, info[i].protocol, (uint16_t)got, got ? d : NULL };
        }
        unsigned off = hid_num(json, "offset", 0, 0, HID_DESC_MAX, &bad);
        if (bad) { vPortFree(buf); vPortFree(data); send_error(conn_id, "usb_hid: offset must be 0 to 1024"); return; }
        len = usb_hid_list_json(it, n, hi.attached, off, data, cap);
        vPortFree(buf);
    }
    if (len == 0) bp_err_reply_too_large(conn_id);
    else send_ok_large(conn_id, data);
    vPortFree(data);
}

#else /* !BENCHPOD_USB_HOST */

void handle_usb_host(int conn_id, const char *json)
{
    (void)json;
    bp_err_unknown_command(conn_id, "usb_host");
}


void handle_usb_flash(int conn_id, const char *json)
{
    (void)json;
    bp_err_unknown_command(conn_id, "usb_flash");
}

void handle_usb_serial(int conn_id, const char *json)
{
    (void)json;
    bp_err_unknown_command(conn_id, "usb_serial");
}

void handle_usb_check(int conn_id, const char *json)
{
    (void)json;
    bp_err_unknown_command(conn_id, "usb_check");
}

void handle_usb_hid(int conn_id, const char *json)
{
    (void)json;
    bp_err_unknown_command(conn_id, "usb_hid");
}
#endif /* BENCHPOD_USB_HOST */
