/* usb_msc.c — see usb_msc.h. USB_HOST=1 builds only. */
#include "usb_msc.h"
#include "usb_host.h"

#include "tusb.h"
#include "host/usbh_pvt.h"   /* the PICOBOOT class driver below */
#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

enum { REQ_IDLE = 0, REQ_QUEUED, REQ_ISSUED, REQ_DONE };
enum { KIND_MSC = 0, KIND_CTRL, KIND_BULK };

static volatile uint8_t  s_dev;          /* the mounted drive's address, 0 = none */
static volatile uint8_t  s_state;
static volatile bool     s_ok;
static bool              s_write;
static uint32_t          s_lba, s_count;
static void             *s_buf;
static uint8_t           s_kind;
static tusb_control_request_t s_setup;
static tuh_xfer_t        s_xfer;
static volatile uint32_t s_actual;
static uint8_t           s_ep;           /* KIND_BULK: the endpoint address */

void tuh_msc_mount_cb(uint8_t dev_addr)   { s_dev = dev_addr; }
void tuh_msc_umount_cb(uint8_t dev_addr)
{
    if (s_dev != dev_addr) return;
    s_dev = 0;
    if (s_state == REQ_ISSUED || s_state == REQ_QUEUED) { s_ok = false; s_state = REQ_DONE; }
}

bool usb_msc_ready(uint32_t *blocks, uint32_t *block_size)
{
    uint8_t d = s_dev;
    if (!d || !tuh_msc_mounted(d)) return false;
    *blocks = tuh_msc_get_block_count(d, 0);
    *block_size = tuh_msc_get_block_size(d, 0);
    return *blocks > 0;
}

static bool on_done(uint8_t dev_addr, const tuh_msc_complete_data_t *cb)
{
    (void)dev_addr;
    s_ok = cb->csw && cb->csw->status == 0;
    s_state = REQ_DONE;
    return true;
}

static void on_ctrl_done(tuh_xfer_t *x)
{
    s_ok = x->result == XFER_RESULT_SUCCESS;
    s_actual = x->actual_len;
    s_state = REQ_DONE;
}

static void on_bulk_done(tuh_xfer_t *x)
{
    s_ok = x->result == XFER_RESULT_SUCCESS;
    s_actual = x->actual_len;
    s_state = REQ_DONE;
}

void usb_msc_pump(void)
{
    if (s_state != REQ_QUEUED) return;
    if (s_kind == KIND_BULK) {
        uint8_t d = usb_host_daddr();
        if (!d) { s_ok = false; s_state = REQ_DONE; return; }
        tuh_xfer_t x = { .daddr = d, .ep_addr = s_ep, .buflen = s_count, .buffer = s_buf,
                         .complete_cb = on_bulk_done, .user_data = 0 };
        if (tuh_edpt_xfer(&x)) s_state = REQ_ISSUED;
        else if (!usbh_edpt_busy(d, s_ep)) { s_ok = false; s_state = REQ_DONE; }   /* not open: no retry */
        return;   /* else busy: try again on the next pass */
    }
    if (s_kind == KIND_CTRL) {
        uint8_t d = usb_host_daddr();
        if (!d) { s_ok = false; s_state = REQ_DONE; return; }
        s_xfer = (tuh_xfer_t){ .daddr = d, .ep_addr = 0, .setup = &s_setup, .buffer = s_buf,
                               .complete_cb = on_ctrl_done, .user_data = 0 };
        if (tuh_control_xfer(&s_xfer)) s_state = REQ_ISSUED;
        return;   /* else busy: try again on the next pass */
    }
    uint8_t d = s_dev;
    bool started = d && tuh_msc_ready(d) &&
        (s_write ? tuh_msc_write10(d, 0, s_buf, s_lba, (uint16_t)s_count, on_done, 0)
                 : tuh_msc_read10(d, 0, s_buf, s_lba, (uint16_t)s_count, on_done, 0));
    if (started) s_state = REQ_ISSUED;
    else if (!d) { s_ok = false; s_state = REQ_DONE; }
    /* else busy: try again on the next pass */
}

bool usb_msc_idle(void) { return s_state == REQ_IDLE || s_state == REQ_DONE; }

int usb_msc_io(bool write, uint32_t lba, void *buf, uint32_t count, uint32_t timeout_ms)
{
    if (!s_dev || count == 0 || count > 0xFFFFu || !usb_msc_idle()) return -1;
    s_kind = KIND_MSC;
    s_write = write; s_lba = lba; s_count = count; s_buf = buf;
    s_ok = false;
    s_state = REQ_QUEUED;
    const TickType_t t0 = xTaskGetTickCount();
    while (s_state != REQ_DONE) {
        if (xTaskGetTickCount() - t0 > pdMS_TO_TICKS(timeout_ms)) {
            /* Not issued yet: take it back, so the usb task never starts it on a freed buffer.
               Issued: it stays in flight; usb_msc_idle() says when its buffer is free. */
            taskENTER_CRITICAL();
            if (s_state == REQ_QUEUED) s_state = REQ_IDLE;
            taskEXIT_CRITICAL();
            return -1;
        }
        vTaskDelay(1);
    }
    s_state = REQ_IDLE;
    return s_ok ? 0 : -1;
}

int usb_ctrl_io(const uint8_t setup[8], uint8_t *data, uint32_t *actual, uint32_t timeout_ms)
{
    *actual = 0;
    if (!usb_host_daddr()) return -2;
    if (!usb_msc_idle()) return -1;
    memcpy(&s_setup, setup, sizeof(s_setup));
    s_kind = KIND_CTRL;
    s_buf = data;
    s_actual = 0;
    s_ok = false;
    s_state = REQ_QUEUED;
    const TickType_t t0 = xTaskGetTickCount();
    while (s_state != REQ_DONE) {
        if (xTaskGetTickCount() - t0 > pdMS_TO_TICKS(timeout_ms)) {
            /* Not issued yet: take it back, so the usb task never starts it on a freed buffer.
               Issued: it stays in flight; usb_msc_idle() says when its buffer is free. */
            taskENTER_CRITICAL();
            if (s_state == REQ_QUEUED) s_state = REQ_IDLE;
            taskEXIT_CRITICAL();
            return -1;
        }
        vTaskDelay(1);
    }
    s_state = REQ_IDLE;
    *actual = s_actual;
    return s_ok ? 0 : -1;
}

/* One bulk transfer on an endpoint the PICOBOOT driver opened. len 0 sends a zero-length packet
   (OUT); an IN transfer ends on a short packet. */
int usb_bulk_io(uint8_t ep, uint8_t *data, uint32_t len, uint32_t *actual, uint32_t timeout_ms)
{
    *actual = 0;
    if (!usb_host_daddr()) return -2;
    if (!usb_msc_idle() || len > 0xFFFFu) return -1;
    s_kind = KIND_BULK;
    s_ep = ep;
    s_buf = data;
    s_count = len;
    s_actual = 0;
    s_ok = false;
    s_state = REQ_QUEUED;
    const TickType_t t0 = xTaskGetTickCount();
    while (s_state != REQ_DONE) {
        if (xTaskGetTickCount() - t0 > pdMS_TO_TICKS(timeout_ms)) {
            taskENTER_CRITICAL();
            if (s_state == REQ_QUEUED) s_state = REQ_IDLE;
            taskEXIT_CRITICAL();
            return -1;
        }
        vTaskDelay(1);
    }
    s_state = REQ_IDLE;
    *actual = s_actual;
    return s_ok ? 0 : -1;
}

/* ---- PICOBOOT: the vendor interface of an RP2040/RP2350 in BOOTSEL ---------------------------
   No TinyUSB class takes it, so a small driver opens its two bulk endpoints during enumeration
   (usbh closes them with the device). Only for those two USB IDs. */
static bool pb_init(void) { return true; }
static bool pb_deinit(void) { return true; }

static uint16_t pb_open(uint8_t rhport, uint8_t daddr, const tusb_desc_interface_t *itf, uint16_t max_len)
{
    (void)rhport;
    uint16_t vid = 0, pid = 0;
    if (!tuh_vid_pid_get(daddr, &vid, &pid) || vid != 0x2E8Au || (pid != 0x0003u && pid != 0x000Fu)) return 0;
    if (itf->bInterfaceClass != 0xFFu || itf->bNumEndpoints != 2) return 0;
    const uint16_t len = (uint16_t)(sizeof(tusb_desc_interface_t) + 2u * sizeof(tusb_desc_endpoint_t));
    if (max_len < len) return 0;
    const uint8_t *p = tu_desc_next((const uint8_t *)itf);
    for (int i = 0; i < 2; i++, p = tu_desc_next(p)) {
        const tusb_desc_endpoint_t *e = (const tusb_desc_endpoint_t *)p;
        if (e->bDescriptorType != TUSB_DESC_ENDPOINT || e->bmAttributes.xfer != TUSB_XFER_BULK) return 0;
        if (!tuh_edpt_open(daddr, e)) return 0;
    }
    return len;
}

static bool pb_set_config(uint8_t daddr, uint8_t itf_num)
{
    usbh_driver_set_config_complete(daddr, itf_num);
    return true;
}

static bool pb_xfer_cb(uint8_t daddr, uint8_t ep, xfer_result_t r, uint32_t n)
{
    (void)daddr; (void)ep; (void)r; (void)n;
    return true;   /* tuh_edpt_xfer's own callback takes the result */
}

static void pb_close(uint8_t daddr) { (void)daddr; }

usbh_class_driver_t const *usbh_app_driver_get_cb(uint8_t *count)
{
    static const usbh_class_driver_t k = { "PICOBOOT", pb_init, pb_deinit, pb_open, pb_set_config, pb_xfer_cb, pb_close };
    *count = 1;
    return &k;
}
