/* usb_msc.c — see usb_msc.h. USB_HOST=1 builds only. */
#include "usb_msc.h"

#include "tusb.h"
#include "FreeRTOS.h"
#include "task.h"

enum { REQ_IDLE = 0, REQ_QUEUED, REQ_ISSUED, REQ_DONE };

static volatile uint8_t  s_dev;          /* the mounted drive's address, 0 = none */
static volatile uint8_t  s_state;
static volatile bool     s_ok;
static bool              s_write;
static uint32_t          s_lba, s_count;
static void             *s_buf;

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

void usb_msc_pump(void)
{
    if (s_state != REQ_QUEUED) return;
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
