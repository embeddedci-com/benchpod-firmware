/* usb_host.c — USB host mode state and TinyUSB host callbacks (usb_host.h).
 *
 * Requests (any task) set s_want_port and s_deadline; the usb task reads usb_host_wanted() and
 * changes the role. The TinyUSB host callbacks run on the usb task and fill s_info; readers take
 * a copy inside a critical section, so they never see half a descriptor. */
#include "usb_host.h"
#include "board_caps.h"

#include "tusb.h"
#include "stm32h5xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

#include "host/hcd.h"   /* hcd_port_connect_status, hcd_port_speed_get */

static volatile usb_host_port_t s_want_port;   /* NONE: device mode wanted */
static volatile uint32_t        s_deadline;    /* HAL tick the request expires */
static volatile usb_host_port_t s_active_port; /* what the usb task runs: NONE = device */
static uint32_t                 s_epoch;       /* host start or the last detach (usb task) */
static uint32_t                 s_host_since;

static usb_host_info_t s_info;                 /* device fields; written on the usb task */

static bool hse_running(void) { return __HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) != 0; }

bool usb_host_port_available(usb_host_port_t port)
{
    if (port == USB_HOST_PORT_USB_C) return true;
    if (port == USB_HOST_PORT_USB_A) return board_caps()->usb_sel;
    return false;
}

usb_host_err_t usb_host_start(usb_host_port_t port, uint32_t timeout_s)
{
    if (!usb_host_port_available(port)) return USB_HOST_ERR_PORT;
    if (timeout_s == 0) timeout_s = USB_HOST_TIMEOUT_DEFAULT_S;
    if (timeout_s > USB_HOST_TIMEOUT_MAX_S) return USB_HOST_ERR_TIMEOUT;
    if (!hse_running()) return USB_HOST_ERR_CLOCK;
    taskENTER_CRITICAL();
    usb_host_port_t cur = s_want_port != USB_HOST_PORT_NONE ? s_want_port : s_active_port;
    if (cur != USB_HOST_PORT_NONE && cur != port) {
        taskEXIT_CRITICAL();
        return USB_HOST_ERR_OTHER_PORT;
    }
    uint32_t d = HAL_GetTick() + timeout_s * 1000u;
    s_deadline = d ? d : 1u;
    s_want_port = port;
    taskEXIT_CRITICAL();
    return USB_HOST_OK;
}

void usb_host_stop(void)
{
    taskENTER_CRITICAL();
    s_want_port = USB_HOST_PORT_NONE;
    taskEXIT_CRITICAL();
}

usb_host_port_t usb_host_wanted(void)
{
    usb_host_port_t p = s_want_port;
    if (p != USB_HOST_PORT_NONE && (int32_t)(HAL_GetTick() - s_deadline) >= 0) {
        taskENTER_CRITICAL();
        if (s_want_port == p) s_want_port = USB_HOST_PORT_NONE;   /* timed out */
        taskEXIT_CRITICAL();
        return USB_HOST_PORT_NONE;
    }
    return p;
}

static bool s_enum_open;        /* a device descriptor arrived, no mount yet */
static uint32_t s_enum_tick;    /* when it arrived */

void usb_host_role_changed(usb_host_port_t port)
{
    taskENTER_CRITICAL();
    s_active_port = port;
    if (port != USB_HOST_PORT_NONE) {
        s_host_since = s_epoch = HAL_GetTick();
        s_info.enumerations = 0;
        s_info.attached = false;
        s_info.seen = false;
        s_enum_open = false;
    } else {
        s_info.attached = false;
    }
    taskEXIT_CRITICAL();
}

void usb_host_info(usb_host_info_t *out)
{
    taskENTER_CRITICAL();
    *out = s_info;
    usb_host_port_t want = s_want_port, act = s_active_port;
    uint32_t now = HAL_GetTick(), deadline = s_deadline, since = s_host_since;
    taskEXIT_CRITICAL();

    if (act == USB_HOST_PORT_NONE)
        out->mode = want != USB_HOST_PORT_NONE ? USB_HOST_MODE_STARTING : USB_HOST_MODE_DEVICE;
    else
        out->mode = want == act ? USB_HOST_MODE_HOST : USB_HOST_MODE_STOPPING;
    out->port = act != USB_HOST_PORT_NONE ? act : want;
    int32_t left = (int32_t)(deadline - now);
    out->remaining_s = (want != USB_HOST_PORT_NONE && left > 0) ? ((uint32_t)left + 999u) / 1000u : 0;
    out->host_ms = act != USB_HOST_PORT_NONE ? now - since : 0;
    out->enumeration = usb_host_enum_state(out->seen, out->attached, s_enum_open, now - s_enum_tick);
    /* What the port sees on the wire, before any enumeration (one flag and one register: safe to read
       from any task while the usb task hosts). */
    out->connected = act != USB_HOST_PORT_NONE && want == act && hcd_port_connect_status(0);
    out->line_low_speed = out->connected && hcd_port_speed_get(0) == TUSB_SPEED_LOW;
}

/* ---- TinyUSB host callbacks (usb task) ---------------------------------------------- */

static uint8_t s_str_buf[2 + 2 * (USB_HOST_STR_MAX - 1)];
static uint8_t s_str_idx[3];   /* iManufacturer, iProduct, iSerialNumber */
static uint8_t s_str_next;
static uint8_t s_str_daddr;

static char *str_slot(uint8_t i)
{
    return i == 0 ? s_info.manufacturer : i == 1 ? s_info.product : s_info.serial;
}

static void str_fetch_next(void);

static void str_done(tuh_xfer_t *xfer)
{
    uint8_t i = (uint8_t)xfer->user_data;
    if (xfer->result == XFER_RESULT_SUCCESS && i < 3) {
        char tmp[USB_HOST_STR_MAX];
        usb_host_string_ascii(s_str_buf, xfer->actual_len, tmp, sizeof(tmp));
        taskENTER_CRITICAL();
        memcpy(str_slot(i), tmp, sizeof(tmp));
        taskEXIT_CRITICAL();
    }
    str_fetch_next();
}

/* One string at a time: the control pipe takes one transfer. */
static void str_fetch_next(void)
{
    while (s_str_next < 3) {
        uint8_t i = s_str_next++;
        if (s_str_idx[i] == 0) continue;
        if (tuh_descriptor_get_string(s_str_daddr, s_str_idx[i], 0x0409, s_str_buf, sizeof(s_str_buf),
                                      str_done, i))
            return;
    }
}

void tuh_enum_descriptor_device_cb(uint8_t daddr, const tusb_desc_device_t *d)
{
    (void)daddr;
    uint32_t ms = HAL_GetTick() - s_epoch;
    taskENTER_CRITICAL();
    s_info.seen = true;
    s_info.vid = d->idVendor;               s_info.pid = d->idProduct;
    s_info.bcd_usb = d->bcdUSB;             s_info.bcd_device = d->bcdDevice;
    s_info.dev_class = d->bDeviceClass;     s_info.dev_subclass = d->bDeviceSubClass;
    s_info.dev_protocol = d->bDeviceProtocol;
    s_info.ep0_size = d->bMaxPacketSize0;   s_info.num_configs = d->bNumConfigurations;
    s_info.descriptor_ms = ms;
    s_info.mounted_ms = 0;
    s_enum_open = true;
    s_enum_tick = HAL_GetTick();
    s_info.manufacturer[0] = s_info.product[0] = s_info.serial[0] = '\0';
    memset(&s_info.config, 0, sizeof(s_info.config));
    taskEXIT_CRITICAL();
    s_str_idx[0] = d->iManufacturer; s_str_idx[1] = d->iProduct; s_str_idx[2] = d->iSerialNumber;
}

bool tuh_enum_descriptor_configuration_cb(uint8_t daddr, uint8_t cfg_index,
                                          const tusb_desc_configuration_t *desc)
{
    (void)daddr; (void)cfg_index;
    /* usbh read the whole descriptor into its buffer (wTotalLength <= the buffer, checked). */
    uint16_t len = tu_le16toh(desc->wTotalLength);
    if (len > CFG_TUH_ENUMERATION_BUFSIZE) len = CFG_TUH_ENUMERATION_BUFSIZE;
    usb_host_config_t c;
    usb_host_parse_config((const uint8_t *)desc, len, &c);
    taskENTER_CRITICAL();
    s_info.config = c;
    taskEXIT_CRITICAL();
    return true;   /* the first configuration */
}

void tuh_mount_cb(uint8_t daddr)
{
    uint32_t ms = HAL_GetTick() - s_epoch;
    tusb_speed_t sp = tuh_speed_get(daddr);
    taskENTER_CRITICAL();
    s_info.attached = true;
    s_info.mounted_ms = ms;
    s_enum_open = false;
    s_info.speed = (uint8_t)sp;
    if (s_info.enumerations < UINT16_MAX) s_info.enumerations++;
    taskEXIT_CRITICAL();
    s_str_daddr = daddr;
    s_str_next = 0;
    str_fetch_next();
}

uint8_t usb_host_daddr(void) { return s_info.attached ? s_str_daddr : 0; }

void tuh_umount_cb(uint8_t daddr)
{
    (void)daddr;
    taskENTER_CRITICAL();
    s_info.attached = false;
    s_enum_open = false;
    taskEXIT_CRITICAL();
    s_str_next = 3;
    s_epoch = HAL_GetTick();   /* the next attach is timed from here */
}
