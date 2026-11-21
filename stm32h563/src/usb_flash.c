/* usb_flash.c — see usb_flash.h. USB_HOST=1 builds only. Internal flash: the job around the
   W25Q code (msc_flash_step runs from the image, through the XIP session's table). */
#include "usb_flash.h"
#include "usb_msc.h"
#include "usb_host.h"
#include "msc_flash.h"
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

static struct {
    msc_flash_ctx_t *ctx;        /* heap, while a job runs (or its last request is in flight) */
    bool     running, have_result;
    int      rc;
    uint8_t  mode, phase;
    uint32_t base, done, total, len, t0, ms;
    char     name[16];
} s;

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

static int op_src(void *io, uint32_t off, uint8_t *buf, uint32_t n)
{
    (void)io;
    psram_bus_acquire();
    int rc = psram_read(s.base + off, buf, n);
    psram_bus_release();
    return rc < 0 ? -1 : 0;
}

bool usb_flash_busy(void) { return s.running; }

static void free_ctx(void)
{
    if (s.ctx && usb_msc_idle()) { vPortFree(s.ctx); s.ctx = NULL; }
}

static void finish(int rc)
{
    s.rc = rc;
    s.running = false;
    s.have_result = true;
    s.ms = (xTaskGetTickCount() - s.t0) * portTICK_PERIOD_MS;
    if (s.ctx) { s.phase = s.ctx->phase; s.done = s.ctx->done; s.total = s.ctx->total; }
    xip_session_end();
    free_ctx();
    LOG_I("[usbflash] %s: %s (%lu of %lu sectors, %lu ms)\n", s.mode ? "file" : "uf2",
          msc_flash_rc_str(rc), (unsigned long)s.done, (unsigned long)s.total, (unsigned long)s.ms);
}

int usb_flash_start(const usb_flash_params_t *p, const char **why)
{
    *why = NULL;
    if (s.running) { *why = "a USB flash is already running"; return -1; }
    free_ctx();
    if (s.ctx) { *why = "the last USB request has not finished yet"; return -1; }
    usb_host_info_t hi;
    usb_host_info(&hi);
    if (hi.mode != USB_HOST_MODE_HOST) { *why = "USB host mode is not on (usb_host)"; return -1; }
    uint32_t blocks = 0, bsize = 0;
    if (!usb_msc_ready(&blocks, &bsize)) { *why = "no USB drive is mounted"; return -1; }
    char n83[11];
    if (p->mode == MSC_FLASH_FILE && !msc_flash_name83(p->name, n83)) { *why = "name must be an 8.3 file name"; return -1; }
    if (p->mode != MSC_FLASH_FILE && p->mode != MSC_FLASH_UF2) { *why = "mode must be uf2 or file"; return -1; }
    uint32_t base = 0, staged = 0;
    if (!command_handler_psram_stage(&base, &staged)) { *why = "nothing staged: load_bin with \"psram\":true first"; return -1; }
    if (p->len == 0 || p->len > staged) { *why = "len is 0 or more than the staged upload"; return -1; }

    xip_err_t xe = xip_session_begin();
    if (xe != XIP_OK) { *why = xip_err_text(xe); return -2; }
    s.ctx = pvPortMalloc(sizeof(msc_flash_ctx_t));
    if (!s.ctx) { xip_session_end(); *why = "out of memory"; return -1; }
    memset(s.ctx, 0, sizeof(*s.ctx));
    s.ctx->ops = (msc_flash_ops_t){ &s, op_read, op_write, op_src, blocks, bsize };
    s.ctx->mode = p->mode;
    memcpy(s.ctx->name83, n83, sizeof(n83));
    s.ctx->len = p->len;
    s.base = base;
    s.mode = p->mode;
    s.len = p->len;
    snprintf(s.name, sizeof(s.name), "%s", p->mode == MSC_FLASH_FILE ? p->name : "");
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
    int rc = xip_session_call_ctx(XIP_FN_MSC_FLASH_STEP, s.ctx);
    s.phase = s.ctx->phase; s.done = s.ctx->done; s.total = s.ctx->total;
    if (rc != MSC_FLASH_RUNNING) finish(rc);
}

void usb_flash_abort(void)
{
    if (s.running) finish(MSC_FLASH_E_ABORTED);
}

void usb_flash_status(usb_flash_status_t *st)
{
    st->running = s.running;
    st->have_result = s.have_result;
    st->mode = s.mode == MSC_FLASH_FILE ? "file" : "uf2";
    st->phase = msc_flash_phase_str(s.phase);
    st->rc = s.running ? MSC_FLASH_RUNNING : s.rc;
    st->result = msc_flash_rc_str(st->rc);
    st->done = s.done;
    st->total = s.total;
    st->len = s.len;
    st->ms = s.running ? (xTaskGetTickCount() - s.t0) * portTICK_PERIOD_MS : s.ms;
    memcpy(st->name, s.name, sizeof(st->name));
}
