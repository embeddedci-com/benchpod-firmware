/* usb_hid.c — see usb_hid.h. USB_HOST=1 builds only. */
#include "usb_hid.h"
#include "usb_host.h"

#include "tusb.h"
#include "stm32h5xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

static struct { bool used; uint8_t inst, itf, protocol; } s_itf[USB_HID_MAX_ITF];
static usb_hid_ring_t *volatile s_ring;   /* the capture's ring, NULL = no capture; it and its slots are one block */
static uint32_t s_t0;

void tuh_hid_mount_cb(uint8_t daddr, uint8_t idx, const uint8_t *report_desc, uint16_t desc_len)
{
    (void)report_desc; (void)desc_len;
    tuh_itf_info_t info;
    if (idx >= USB_HID_MAX_ITF || !tuh_hid_itf_get_info(daddr, idx, &info)) return;
    taskENTER_CRITICAL();
    s_itf[idx].used = true;
    s_itf[idx].inst = idx;
    s_itf[idx].itf = info.desc.bInterfaceNumber;
    s_itf[idx].protocol = tuh_hid_interface_protocol(daddr, idx);
    taskEXIT_CRITICAL();
}

void tuh_hid_umount_cb(uint8_t daddr, uint8_t idx)
{
    (void)daddr;
    if (idx >= USB_HID_MAX_ITF) return;
    taskENTER_CRITICAL();
    s_itf[idx].used = false;
    bool any = false;
    for (unsigned i = 0; i < USB_HID_MAX_ITF; i++) any = any || s_itf[i].used;
    taskEXIT_CRITICAL();
    if (!any) usb_hid_capture_stop();   /* the board detached: free the ring */
}

void tuh_hid_report_received_cb(uint8_t daddr, uint8_t idx, const uint8_t *report, uint16_t len)
{
    taskENTER_CRITICAL();
    usb_hid_ring_t *r = s_ring;
    if (r) (void)usb_hid_ring_push(r, HAL_GetTick() - s_t0, idx, report, len);
    taskEXIT_CRITICAL();
    if (r) (void)tuh_hid_receive_report(daddr, idx);
}

unsigned usb_hid_interfaces(usb_hid_info_t *out)
{
    unsigned n = 0;
    usb_host_info_t hi;
    usb_host_info(&hi);
    if (!hi.attached) return 0;
    taskENTER_CRITICAL();
    for (unsigned i = 0; i < USB_HID_MAX_ITF; i++)
        if (s_itf[i].used) out[n++] = (usb_hid_info_t){ s_itf[i].inst, s_itf[i].itf, s_itf[i].protocol };
    taskEXIT_CRITICAL();
    return n;
}

bool usb_hid_capture_active(void) { return s_ring != NULL; }

int usb_hid_capture_start(void)
{
    if (s_ring) return 1;
    bool any = false;
    taskENTER_CRITICAL();
    for (unsigned i = 0; i < USB_HID_MAX_ITF; i++) any = any || s_itf[i].used;
    taskEXIT_CRITICAL();
    if (!any) return -2;
    usb_hid_ring_t *r = pvPortMalloc(sizeof(*r) + USB_HID_CAPTURE_SLOTS * sizeof(usb_hid_rep_t));
    if (!r) return -1;
    usb_hid_ring_init(r, (usb_hid_rep_t *)(r + 1), USB_HID_CAPTURE_SLOTS);
    taskENTER_CRITICAL();
    if (s_ring) { taskEXIT_CRITICAL(); vPortFree(r); return 1; }
    s_t0 = HAL_GetTick();
    s_ring = r;
    taskEXIT_CRITICAL();
    return 0;
}

void usb_hid_capture_stop(void)
{
    taskENTER_CRITICAL();
    usb_hid_ring_t *r = s_ring;
    s_ring = NULL;
    taskEXIT_CRITICAL();
    vPortFree(r);   /* NULL is fine */
}

unsigned usb_hid_capture_ready(uint8_t mask)
{
    taskENTER_CRITICAL();
    usb_hid_ring_t *r = s_ring;
    unsigned n = r ? usb_hid_ring_count_match(r, mask) : 0;
    taskEXIT_CRITICAL();
    return n;
}

size_t usb_hid_capture_read(uint8_t mask, unsigned max, char *out, size_t cap)
{
    size_t n = 0;
    taskENTER_CRITICAL();   /* the ring cannot be freed or pushed to meanwhile; the reply is small */
    if (s_ring) n = usb_hid_reports_json(s_ring, max, mask, true, out, cap);
    taskEXIT_CRITICAL();
    return n;
}

size_t usb_hid_capture_state(char *out, size_t cap)
{
    taskENTER_CRITICAL();
    usb_hid_ring_t *r = s_ring;
    unsigned buffered = r ? r->count : 0, dropped = r ? r->dropped : 0;
    taskEXIT_CRITICAL();
    return usb_hid_capture_json(r != NULL, buffered, dropped, out, cap);
}

void usb_hid_poll(void)
{
    if (!s_ring) return;
    usb_host_info_t hi;
    usb_host_info(&hi);
    if (hi.mode != USB_HOST_MODE_HOST) usb_hid_capture_stop();
}

void usb_hid_pump(void)
{
    uint8_t d = usb_host_daddr();
    if (!d || !s_ring) return;
    /* Arm what the callbacks have not (the first report, an instance that mounted later, or after a
       transient error); false while a transfer is pending or the instance is not mounted. */
    for (uint8_t i = 0; i < USB_HID_MAX_ITF; i++)
        if (s_itf[i].used) (void)tuh_hid_receive_report(d, i);
}
