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
 *   optional "leave":false (dfu, teensy): stay in the loader
 *   {"cmd":"usb_flash"}                                            the state
 *   {"cmd":"usb_flash","abort":true}
 * The file is the first len bytes a load_bin with "psram" staged; usb_host must be on with a
 * drive mounted. Refusals: BP0084 (usb_flash_refused), BP0083 (the W25Q code), BP0048 (bus busy).
 */
#include "usb_flash.h"
#include "msc_flash.h"
#include "usb_serial.h"
#include "task.h"

static void usb_flash_reply(int conn_id)
{
    usb_flash_status_t st;
    usb_flash_status(&st);
    char data[384];
    snprintf(data, sizeof(data),
             "{\"running\":%s,\"have_result\":%s,\"mode\":\"%s\",\"name\":\"%s\",\"len\":%lu,"
             "\"address\":%lu,\"protocol\":\"%s\",\"phase\":\"%s\",\"done\":%lu,\"total\":%lu,"
             "\"result\":\"%s\",\"ok\":%s,\"ms\":%lu}",
             st.running ? "true" : "false", st.have_result ? "true" : "false", st.mode, st.name,
             (unsigned long)st.len, (unsigned long)st.address, st.protocol, st.phase, (unsigned long)st.done,
             (unsigned long)st.total, st.result,
             (!st.running && st.have_result && st.rc == 0) ? "true" : "false", (unsigned long)st.ms);
    send_ok_large(conn_id, data);
}

void handle_usb_flash(int conn_id, const char *json)
{
    if (bp_json_flag(json, "abort")) { usb_flash_abort(); usb_flash_reply(conn_id); return; }
    char mode[8] = {0};
    if (!bp_json_get(json, "mode", mode, sizeof(mode))) { usb_flash_reply(conn_id); return; }
    usb_flash_params_t p = {0};
    int m = usb_flash_mode_from_name(mode);
    if (m < 0) { bp_err_usb_flash_refused(conn_id, "mode must be uf2, file, dfu, teensy, avr, samd or esp"); return; }
    p.mode = (uint8_t)m;
    (void)bp_json_get(json, "name", p.name, sizeof(p.name));
    char v[16] = {0};
    if (bp_json_get(json, "address", v, sizeof(v))) p.address = (uint32_t)strtoul(v, NULL, 0);
    if (bp_json_get(json, "block", v, sizeof(v))) p.block = (uint16_t)strtoul(v, NULL, 10);
    p.leave = !(bp_json_get(json, "leave", v, sizeof(v)) && !strcmp(v, "false"));
    char n[16] = {0};
    if (!bp_json_get(json, "len", n, sizeof(n))) { bp_err_missing_argument(conn_id, "len"); return; }
    p.len = (uint32_t)strtoul(n, NULL, 10);
    if (usb_serial_con_is_open()) {
        bp_err_usb_flash_refused(conn_id, "the USB serial console is open (usb_serial with open false closes it)");
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

/* ---- usb_serial: the board's USB serial port as a console (usb_serial.h) ----------------------
 *   {"cmd":"usb_serial"}                                          the state
 *   {"cmd":"usb_serial","open":true[,"baud":115200][,"dtr":true][,"rts":true]
 *                       [,"data_bits":8][,"parity":"none|even|odd"][,"stop_bits":1|2]}
 *   {"cmd":"usb_serial","open":false}
 *   {"cmd":"usb_serial","baud":9600,"dtr":false}                  change it while open
 *     -> {"open","baud","frame","dtr","rts","bridge","rx_buffered","rx_dropped","tx_total",
 *         "rx_total","port_present"}
 *   {"cmd":"usb_serial_write","data_b64":"..."[,"text":"..."][,"timeout_ms":N]}  -> {"queued":N,"eof":bool}
 *   {"cmd":"usb_serial_read"[,"max":N][,"timeout_ms":T][,"until":"text"]}      -> {"data_b64":"..","eof":bool}
 * Refusals: BP0085 (no host mode or no serial port), BP0086 (a usb_flash runs), BP0087 (not open).
 */
#include "usb_serial.h"
#include "b64url.h"

#define USB_SER_IO_MAX       1024u
#define USB_SER_TIMEOUT_MAX  5000u

static bool usb_serial_host_on(void)
{
    usb_host_info_t hi;
    usb_host_info(&hi);
    return hi.mode == USB_HOST_MODE_HOST;
}

static void usb_serial_reply(int conn_id)
{
    usb_serial_state_t st;
    usb_serial_con_state(&st);
    char data[256];
    if (usb_serial_state_json(&st, data, sizeof(data)) == 0) { bp_err_reply_too_large(conn_id); return; }
    send_ok_str(conn_id, data);
}

void handle_usb_serial(int conn_id, const char *json)
{
    usb_serial_args_t a;
    usb_serial_arg_err_t e = usb_serial_args_parse(json, &a);
    if (e != USB_SER_ARG_OK) { send_error(conn_id, usb_serial_arg_err_text(e)); return; }
    usb_serial_refusal_t r = usb_serial_plan(&a, usb_serial_con_is_open(), usb_serial_host_on(),
                                             usb_serial_ready(), usb_flash_busy());
    switch (r) {
    case USB_SER_GO: break;
    case USB_SER_NO_HOST:
    case USB_SER_NO_PORT:  bp_err_usb_serial_no_port(conn_id, usb_serial_refusal_why(r)); return;
    case USB_SER_FLASHING: bp_err_usb_serial_busy(conn_id, usb_serial_refusal_why(r)); return;
    default:               bp_err_usb_serial_not_open(conn_id); return;
    }
    if (a.has_open && !a.open) usb_serial_con_close();
    else if (a.has_open) {
        int rc = usb_serial_con_open(&a);
        if (rc == -2) { send_error(conn_id, "usb_serial: out of memory"); return; }
        if (rc != 0) { bp_err_usb_serial_no_port(conn_id, usb_serial_refusal_why(USB_SER_NO_PORT)); return; }
    } else if (!usb_serial_args_is_read(&a)) usb_serial_con_set(&a);
    usb_serial_reply(conn_id);
}

static bool usb_serial_timeout(const char *json, uint32_t def, uint32_t *out)
{
    char v[12] = {0};
    *out = def;
    if (!bp_json_get(json, "timeout_ms", v, sizeof(v))) return true;
    char *end = NULL;
    unsigned long n = strtoul(v, &end, 10);
    if (!end || *end || v[0] == '-' || n > USB_SER_TIMEOUT_MAX) return false;
    *out = (uint32_t)n;
    return true;
}

void handle_usb_serial_write(int conn_id, const char *json)
{
    if (!usb_serial_con_is_open()) { bp_err_usb_serial_not_open(conn_id); return; }
    uint32_t tmo;
    if (!usb_serial_timeout(json, 1000, &tmo)) { send_error(conn_id, "usb_serial_write: timeout_ms must be 0 to 5000"); return; }
    const size_t inmax = 1300;
    char *in = pvPortMalloc(inmax + USB_SER_IO_MAX);
    if (!in) { send_error(conn_id, "usb_serial_write: out of memory"); return; }
    uint8_t *bytes = (uint8_t *)in + inmax;
    size_t n = 0;
    const char *bad = NULL;
    if (bp_json_get(json, "data_b64", in, inmax) && in[0] &&
        usb_serial_b64_decode(in, bytes, USB_SER_IO_MAX, &n) != 0) bad = "data_b64 is not base64 or longer than 1024 bytes";
    if (!bad && bp_json_get(json, "text", in, inmax)) {
        size_t t = strlen(in);
        if (n + t > USB_SER_IO_MAX) bad = "at most 1024 bytes a call";
        else { memcpy(bytes + n, in, t); n += t; }
    }
    if (!bad && n == 0) bad = "needs data_b64 or text";
    if (bad) {
        char m[96];
        snprintf(m, sizeof(m), "usb_serial_write: %s", bad);
        vPortFree(in);
        send_error(conn_id, m);
        return;
    }
    uint32_t q = usb_serial_con_write(bytes, (uint32_t)n, tmo);
    vPortFree(in);
    char data[48];
    snprintf(data, sizeof(data), "{\"queued\":%lu,\"eof\":%s}", (unsigned long)q, usb_serial_con_present() ? "false" : "true");
    send_ok_str(conn_id, data);
}

void handle_usb_serial_read(int conn_id, const char *json)
{
    if (!usb_serial_con_is_open()) { bp_err_usb_serial_not_open(conn_id); return; }
    uint32_t tmo, max = 256;
    char v[12] = {0};
    if (!usb_serial_timeout(json, 0, &tmo)) { send_error(conn_id, "usb_serial_read: timeout_ms must be 0 to 5000"); return; }
    if (bp_json_get(json, "max", v, sizeof(v))) {
        char *end = NULL;
        unsigned long n = strtoul(v, &end, 10);
        if (!end || *end || v[0] == '-' || n < 1 || n > USB_SER_IO_MAX) { send_error(conn_id, "usb_serial_read: max must be 1 to 1024"); return; }
        max = (uint32_t)n;
    }
    char until[64] = {0};
    size_t un = bp_json_get(json, "until", until, sizeof(until)) ? strlen(until) : 0;
    const size_t reply_cap = B64URL_ENCODED_LEN(USB_SER_IO_MAX) + 64u;
    uint8_t *buf = pvPortMalloc(USB_SER_IO_MAX + reply_cap);
    if (!buf) { send_error(conn_id, "usb_serial_read: out of memory"); return; }
    char *data = (char *)buf + USB_SER_IO_MAX;
    uint32_t got = 0;
    const TickType_t t0 = xTaskGetTickCount();
    for (;;) {
        got += usb_serial_con_read(buf + got, max - got);
        if (got >= max) break;
        if (un && usb_serial_ends_with(buf, got, (const uint8_t *)until, un)) break;
        if (!usb_serial_con_present()) {   /* the port is gone: what is buffered is all there is */
            got += usb_serial_con_read(buf + got, max - got);
            break;
        }
        if (xTaskGetTickCount() - t0 >= pdMS_TO_TICKS(tmo)) break;
        vTaskDelay(1);
    }
    int o = snprintf(data, reply_cap, "{\"data_b64\":\"");
    o += (int)b64url_encode(buf, got, data + o, reply_cap - (size_t)o);
    snprintf(data + o, reply_cap - (size_t)o, "\",\"eof\":%s}", usb_serial_con_present() ? "false" : "true");
    send_ok_large(conn_id, data);
    vPortFree(buf);
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

void handle_usb_serial_write(int conn_id, const char *json)
{
    (void)json;
    bp_err_unknown_command(conn_id, "usb_serial_write");
}

void handle_usb_serial_read(int conn_id, const char *json)
{
    (void)json;
    bp_err_unknown_command(conn_id, "usb_serial_read");
}
#endif /* BENCHPOD_USB_HOST */
