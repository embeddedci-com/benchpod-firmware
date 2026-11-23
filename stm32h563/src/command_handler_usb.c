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
 *   optional "check_fail_txt":true (file): read FAIL.TXT after the drive comes back
 *   optional "verify":true (uf2): compare CURRENT.UF2 when the drive is still a loader
 *   {"cmd":"usb_flash"}                                            the state
 *   {"cmd":"usb_flash","abort":true}
 * The file is the first len bytes a load_bin with "psram" staged; usb_host must be on with a
 * drive mounted. Refusals: BP0084 (usb_flash_refused), BP0083 (the W25Q code), BP0048 (bus busy).
 */
#include "usb_flash.h"
#include "msc_flash.h"

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
    p.check_fail_txt = bp_json_flag(json, "check_fail_txt");
    p.verify = bp_json_flag(json, "verify");
    char n[16] = {0};
    if (!bp_json_get(json, "len", n, sizeof(n))) { bp_err_missing_argument(conn_id, "len"); return; }
    p.len = (uint32_t)strtoul(n, NULL, 10);
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
#endif /* BENCHPOD_USB_HOST */
