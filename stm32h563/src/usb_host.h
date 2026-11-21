#ifndef USB_HOST_H
#define USB_HOST_H

/* usb_host — the pod as a full-speed USB host for one DUT, on demand only.
 *
 * The MCU has one USB port. Normally it is the pod's own USB device (console, USB network
 * adapter, DFU) on USB-C. usb_host_start() hands the port to the DUT for a while:
 *  - port USB_A (boards with the USB switch, board_caps.usb_sel): the switch moves the MCU's
 *    USB from USB-C to USB-A, where the DUT also gets its 5 V from the internal eFuse.
 *  - port USB_C (every board): the pod hosts on its own USB-C connector. Only for a bench rig
 *    with an adapter cable that powers the pod and carries D+/D- to the DUT; a computer on
 *    USB-C just sees the pod disappear.
 * Either way the pod is off the computer's USB while it hosts: it answers over LAN, Wi-Fi or
 * the cloud only, so a request that came in over USB is refused (the caller checks).
 *
 * The port goes back to device mode at usb_host_stop(), or by itself when the timeout runs out
 * (a client that vanished never leaves the pod off USB). Calling usb_host_start() again while
 * it hosts on the same port renews the timeout.
 *
 * All TinyUSB work happens on the usb task (usb_device.c), which polls usb_host_wanted() and
 * performs the role change; the calls below are safe from any task. */

#include <stdbool.h>
#include <stdint.h>

#include "usb_host_desc.h"

typedef enum {
    USB_HOST_PORT_NONE = 0,
    USB_HOST_PORT_USB_C,
    USB_HOST_PORT_USB_A,
} usb_host_port_t;

typedef enum {
    USB_HOST_OK = 0,
    USB_HOST_ERR_PORT,       /* the port does not exist on this board (USB-A without the switch) */
    USB_HOST_ERR_CLOCK,      /* no crystal: host mode needs the 48 MHz from the HSE PLL */
    USB_HOST_ERR_OTHER_PORT, /* already hosting on the other port: stop first */
    USB_HOST_ERR_TIMEOUT,    /* timeout_s outside 1..USB_HOST_TIMEOUT_MAX_S */
} usb_host_err_t;

#define USB_HOST_TIMEOUT_DEFAULT_S  60u
#define USB_HOST_TIMEOUT_MAX_S      3600u

typedef enum {
    USB_HOST_MODE_DEVICE = 0,  /* the pod's own USB device (normal) */
    USB_HOST_MODE_STARTING,    /* asked for, the usb task is switching */
    USB_HOST_MODE_HOST,        /* hosting the DUT */
    USB_HOST_MODE_STOPPING,
} usb_host_mode_t;

typedef struct {
    usb_host_mode_t mode;
    usb_host_port_t port;
    uint32_t remaining_s;      /* until the timeout, while starting or hosting */
    uint32_t host_ms;          /* time in host mode so far */
    uint16_t enumerations;     /* devices mounted since host mode started */
    bool     attached;         /* a device is mounted now */
    /* The mounted device (valid while attached; the last one is kept after a detach until
       the next attach, `attached` false). */
    bool     seen;
    uint8_t  speed;            /* TUSB_SPEED_FULL / _LOW */
    uint16_t vid, pid, bcd_usb, bcd_device;
    uint8_t  dev_class, dev_subclass, dev_protocol, ep0_size, num_configs;
    uint32_t descriptor_ms;    /* host start (or the previous detach) to the device descriptor */
    uint32_t mounted_ms;       /* host start (or the previous detach) to configured */
    char     manufacturer[USB_HOST_STR_MAX];
    char     product[USB_HOST_STR_MAX];
    char     serial[USB_HOST_STR_MAX];
    usb_host_config_t config;
} usb_host_info_t;

/* Ask for host mode on `port` for `timeout_s` seconds (0: the default). */
usb_host_err_t usb_host_start(usb_host_port_t port, uint32_t timeout_s);
/* Back to device mode (no-op in device mode). */
void usb_host_stop(void);
/* A consistent copy of the state. */
void usb_host_info(usb_host_info_t *out);

/* The ports this board has: USB_C always, USB_A with the switch. */
bool usb_host_port_available(usb_host_port_t port);
const char *usb_host_port_name(usb_host_port_t port);
/* "usb_c" / "usb_a" -> the port, USB_HOST_PORT_NONE otherwise. */
usb_host_port_t usb_host_port_from_name(const char *name);
const char *usb_host_mode_name(usb_host_mode_t mode);

/* The usb_host command's reply ({"mode":..,"port":..,..,"device":{..}}) into `out` (pure,
   usb_host_reply.c). Returns the length, or 0 when it does not fit `cap`. */
#include <stddef.h>
size_t usb_host_reply_json(const usb_host_info_t *info, char *out, size_t cap);

/* The mounted device's address, 0 when none (for control transfers: usb_msc.h). */
uint8_t usb_host_daddr(void);

/* ---- for the usb task (usb_device.c) ---- */
/* The port the usb task should host on now, USB_HOST_PORT_NONE for device mode (a request
   that timed out reads as NONE). */
usb_host_port_t usb_host_wanted(void);
/* The usb task reports the role change it made: host on `port`, or back to device (NONE). */
void usb_host_role_changed(usb_host_port_t port);

#endif /* USB_HOST_H */
