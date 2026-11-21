/* usb_flash.c — see usb_flash.h. USB_HOST=1 builds only. Internal flash: the job around the W25Q
   code (msc_flash_step and ctrl_flash_step run from the image, through the XIP session's table). */
#include "usb_flash.h"
#include "usb_msc.h"
#include "usb_host.h"
#include "msc_flash.h"
#include "ctrl_flash.h"
#include "xip_session.h"
#include "xip_image.h"
#include "psram.h"
#include "command_handler_internal.h"   /* command_handler_psram_stage */
#include "bp_log.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include <string.h>

#define IO_TIMEOUT_MS  3000u
#define RC_ABORTED     (-100)   /* stopped by the client, not a protocol result */

static struct {
    void    *ctx;                /* msc_flash_ctx_t or ctrl_flash_ctx_t on the heap, while a job runs
                                    (or its last request is in flight) */
    bool     running, have_result;
    int      rc;
    uint8_t  mode, phase;
    uint32_t base, done, total, len, address, t0, ms;
    char     name[16];
} s;

static bool is_ctrl(uint8_t mode) { return mode == USB_FLASH_DFU || mode == USB_FLASH_TEENSY; }

const char *usb_flash_mode_name(uint8_t mode)
{
    switch (mode) {
    case USB_FLASH_UF2:    return "uf2";
    case USB_FLASH_FILE:   return "file";
    case USB_FLASH_DFU:    return "dfu";
    case USB_FLASH_TEENSY: return "teensy";
    default:               return "?";
    }
}

int usb_flash_mode_from_name(const char *name)
{
    for (int m = USB_FLASH_UF2; m <= USB_FLASH_TEENSY; m++)
        if (name && strcmp(name, usb_flash_mode_name((uint8_t)m)) == 0) return m;
    return -1;
}

/* ---- ops (internal functions the W25Q code calls) ------------------------------------------ */

static int op_read(void *io, uint32_t lba, uint8_t *buf, uint32_t n)
{
    (void)io;
    return usb_msc_io(false, lba, buf, n, IO_TIMEOUT_MS);
}

static int op_write(void *io, uint32_t lba, const uint8_t *buf, uint32_t n)
{
    (void)io;
    return usb_msc_io(true, lba, (void *)buf, n, IO_TIMEOUT_MS);
}

static int op_ctrl(void *io, const uint8_t setup[8], uint8_t *data, uint32_t *actual)
{
    (void)io;
    return usb_ctrl_io(setup, data, actual, IO_TIMEOUT_MS);
}

static int op_src(void *io, uint32_t off, uint8_t *buf, uint32_t n)
{
    (void)io;
    psram_bus_acquire();
    int rc = psram_read(s.base + off, buf, n);
    psram_bus_release();
    return rc < 0 ? -1 : 0;
}

static void op_delay(void *io, uint32_t ms)
{
    (void)io;
    vTaskDelay(pdMS_TO_TICKS(ms ? ms : 1));
}

/* ---- the job -------------------------------------------------------------------------------- */

bool usb_flash_busy(void) { return s.running; }

static void free_ctx(void)
{
    if (s.ctx && usb_msc_idle()) { vPortFree(s.ctx); s.ctx = NULL; }
}

static void snapshot(void)
{
    if (!s.ctx) return;
    if (is_ctrl(s.mode)) {
        const ctrl_flash_ctx_t *c = s.ctx;
        s.phase = c->phase; s.done = c->done; s.total = c->total;
    } else {
        const msc_flash_ctx_t *c = s.ctx;
        s.phase = c->phase; s.done = c->done; s.total = c->total;
    }
}

static const char *rc_str(int rc) { return is_ctrl(s.mode) ? ctrl_flash_rc_str(rc) : msc_flash_rc_str(rc); }

static void finish(int rc)
{
    s.rc = rc;
    s.running = false;
    s.have_result = true;
    s.ms = (xTaskGetTickCount() - s.t0) * portTICK_PERIOD_MS;
    snapshot();
    xip_session_end();
    free_ctx();
    LOG_I("[usbflash] %s: %s (%lu of %lu, %lu ms)\n", usb_flash_mode_name(s.mode), rc_str(rc),
          (unsigned long)s.done, (unsigned long)s.total, (unsigned long)s.ms);
}

int usb_flash_start(const usb_flash_params_t *p, const char **why)
{
    *why = NULL;
    if (s.running) { *why = "a USB flash is already running"; return -1; }
    free_ctx();
    if (s.ctx) { *why = "the last USB request has not finished yet"; return -1; }
    if (p->mode > USB_FLASH_TEENSY) { *why = "mode must be uf2, file, dfu or teensy"; return -1; }
    usb_host_info_t hi;
    usb_host_info(&hi);
    if (hi.mode != USB_HOST_MODE_HOST) { *why = "USB host mode is not on (usb_host)"; return -1; }
    uint32_t blocks = 0, bsize = 0;
    if (!is_ctrl(p->mode) && !usb_msc_ready(&blocks, &bsize)) { *why = "no USB drive is mounted"; return -1; }
    if (is_ctrl(p->mode) && !hi.attached) { *why = "no USB device is attached"; return -1; }
    char n83[11];
    if (p->mode == USB_FLASH_FILE && !msc_flash_name83(p->name, n83)) { *why = "name must be an 8.3 file name"; return -1; }
    uint32_t base = 0, staged = 0;
    if (!command_handler_psram_stage(&base, &staged)) { *why = "nothing staged: load_bin with \"psram\":true first"; return -1; }
    if (p->len == 0 || p->len > staged) { *why = "len is 0 or more than the staged upload"; return -1; }

    xip_err_t xe = xip_session_begin();
    if (xe != XIP_OK) { *why = xip_err_text(xe); return -2; }
    size_t size = is_ctrl(p->mode) ? sizeof(ctrl_flash_ctx_t) : sizeof(msc_flash_ctx_t);
    s.ctx = pvPortMalloc(size);
    if (!s.ctx) { xip_session_end(); *why = "out of memory"; return -1; }
    memset(s.ctx, 0, size);
    if (is_ctrl(p->mode)) {
        ctrl_flash_ctx_t *c = s.ctx;
        c->ops = (ctrl_flash_ops_t){ &s, op_ctrl, op_src, op_delay, hi.vid, hi.pid };
        c->mode = p->mode == USB_FLASH_TEENSY ? CTRL_FLASH_HALFKAY : CTRL_FLASH_DFU;
        c->len = p->len;
        c->address = p->address;
        c->block = p->block;
        c->leave = p->leave;
    } else {
        msc_flash_ctx_t *c = s.ctx;
        c->ops = (msc_flash_ops_t){ &s, op_read, op_write, op_src, blocks, bsize };
        c->mode = p->mode == USB_FLASH_FILE ? MSC_FLASH_FILE : MSC_FLASH_UF2;
        memcpy(c->name83, n83, sizeof(n83));
        c->len = p->len;
    }
    s.base = base;
    s.mode = p->mode;
    s.len = p->len;
    s.address = p->address;
    snprintf(s.name, sizeof(s.name), "%s", p->mode == USB_FLASH_FILE ? p->name : "");
    s.phase = 0;
    s.done = s.total = 0;
    s.t0 = xTaskGetTickCount();
    s.running = true;
    return 0;
}

void usb_flash_poll(void)
{
    if (!s.running) { free_ctx(); return; }
    /* Keep host mode on while the job runs (its timeout would hand the port back). */
    usb_host_info_t hi;
    usb_host_info(&hi);
    if (hi.mode == USB_HOST_MODE_HOST && hi.remaining_s < 15u) (void)usb_host_start(hi.port, 30u);
    int rc = xip_session_call_ctx(is_ctrl(s.mode) ? XIP_FN_CTRL_FLASH_STEP : XIP_FN_MSC_FLASH_STEP, s.ctx);
    snapshot();
    if (rc != 1) finish(rc);   /* 1: MSC_FLASH_RUNNING / CTRL_FLASH_RUNNING */
}

void usb_flash_abort(void)
{
    if (s.running) finish(RC_ABORTED);
}

void usb_flash_status(usb_flash_status_t *st)
{
    st->running = s.running;
    st->have_result = s.have_result;
    st->mode = usb_flash_mode_name(s.mode);
    st->phase = is_ctrl(s.mode) ? ctrl_flash_phase_str(s.phase) : msc_flash_phase_str(s.phase);
    st->rc = s.running ? 1 : s.rc;
    st->result = s.running ? "running" : s.rc == RC_ABORTED ? "aborted" : rc_str(s.rc);
    st->done = s.done;
    st->total = s.total;
    st->len = s.len;
    st->address = s.address;
    st->ms = s.running ? (xTaskGetTickCount() - s.t0) * portTICK_PERIOD_MS : s.ms;
    memcpy(st->name, s.name, sizeof(st->name));
}
