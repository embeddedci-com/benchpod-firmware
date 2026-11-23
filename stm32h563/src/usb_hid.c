/* usb_hid.c — see usb_hid.h. USB_HOST=1 builds only. */
#include "usb_hid.h"
#include "usb_host.h"

#include "tusb.h"
#include "stm32h5xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

static struct { bool used; uint8_t inst, itf, protocol; } s_itf[USB_HID_MAX_ITF];
static usb_hid_ring_t *volatile s_ring;   /* the capture's ring, NULL = no capture */
static volatile uint8_t s_mask;
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
    taskEXIT_CRITICAL();
}

void tuh_hid_report_received_cb(uint8_t daddr, uint8_t idx, const uint8_t *report, uint16_t len)
{
    taskENTER_CRITICAL();
    usb_hid_ring_t *r = s_ring;
    bool on = r && (s_mask & (1u << idx));
    if (on) (void)usb_hid_ring_push(r, HAL_GetTick() - s_t0, idx, report, len);
    taskEXIT_CRITICAL();
    if (on) (void)tuh_hid_receive_report(daddr, idx);
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

bool usb_hid_capture_start(uint8_t mask, usb_hid_ring_t *ring)
{
    if (s_ring || !mask) return false;
    taskENTER_CRITICAL();
    for (unsigned i = 0; i < USB_HID_MAX_ITF; i++) {
        if ((mask & (1u << i)) && !s_itf[i].used) { taskEXIT_CRITICAL(); return false; }
    }
    s_t0 = HAL_GetTick();
    s_mask = mask;
    s_ring = ring;
    taskEXIT_CRITICAL();
    return true;
}

unsigned usb_hid_capture_count(void)
{
    taskENTER_CRITICAL();
    usb_hid_ring_t *r = s_ring;
    unsigned n = r ? r->count : 0;
    taskEXIT_CRITICAL();
    return n;
}

void usb_hid_capture_stop(void)
{
    taskENTER_CRITICAL();
    s_ring = NULL;
    s_mask = 0;
    taskEXIT_CRITICAL();
}

void usb_hid_pump(void)
{
    uint8_t d = usb_host_daddr();
    uint8_t m = s_mask;
    if (!d || !m || !s_ring) return;
    /* Arm what the callbacks have not (the first report, or after a transient error); false while
       a transfer is pending. */
    for (uint8_t i = 0; i < USB_HID_MAX_ITF; i++)
        if (m & (1u << i)) (void)tuh_hid_receive_report(d, i);
}
