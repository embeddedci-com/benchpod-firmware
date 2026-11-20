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
    if (usb_host_reply_json(&info, data, cap) == 0) send_error(conn_id, "usb_host: reply too large");
    else send_ok_str(conn_id, data);
    vPortFree(data);
}

void handle_usb_host(int conn_id, const char *json)
{
    char v[16] = {0};
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
#else /* !BENCHPOD_USB_HOST */

void handle_usb_host(int conn_id, const char *json)
{
    (void)json;
    bp_err_unknown_command(conn_id, "usb_host");
}

#endif /* BENCHPOD_USB_HOST */
