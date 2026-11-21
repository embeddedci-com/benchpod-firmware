/* usb_flash.c — see usb_flash.h. USB_HOST=1 builds only. Internal flash: the job around the W25Q
   code (msc_flash_step, ctrl_flash_step and ser_flash_step run from the image, through the XIP
   session's table), and the esp mode around the internal ESP32 flasher. */
#include "usb_flash.h"
#include "usb_msc.h"
#include "usb_serial.h"
#include "usb_host.h"
#include "msc_flash.h"
#include "ctrl_flash.h"
#include "ser_flash.h"
#include "esp_rom_job.h"
#include "esp_rom_flash.h"
#include "esp_dut_flash.h"
#include "xip_session.h"
#include "xip_image.h"
#include "psram.h"
#include "command_handler_internal.h"   /* command_handler_psram_stage */
#include "bp_log.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include <string.h>

#define IO_TIMEOUT_MS   3000u
#define LINE_MS         1000u
#define RC_ABORTED      (-100)   /* stopped by the client, not a protocol result */
#define ESP_FAST_BAUD   460800u  /* a dev kit's USB-serial chip after CHANGE_BAUDRATE */
#define ESP_BACK_MS     5000u    /* a chip with native USB enumerating again after its reset */
#define RC_ESP_UNVERIFIED 2      /* ESP_ROM_UNVERIFIED, moved off 1 ("running") */

typedef enum { FAM_MSC = 0, FAM_CTRL, FAM_SER, FAM_ESP } family_t;
typedef enum { ESP_PRE_RESET = 0, ESP_PRE_BACK, ESP_PRE_JOB } esp_pre_t;

static struct {
    void    *ctx;                /* msc/ctrl/ser context on the heap, while a job runs (or its last
                                    request is in flight) */
    bool     running, have_result, xip;
    int      rc;
    uint8_t  mode, phase, proto;
    uint32_t base, done, total, len, address, t0, ms;
    char     name[16];
    /* esp */
    bool     leave;
    uint8_t  esp_pre;
    uint32_t esp_mounts, esp_baud, esp_t;
    uint16_t vid, pid;
} s;

static esp_rom_job_t s_esp;
static esp_rom_io_t  s_esp_io;

static family_t family(uint8_t mode)
{
    switch (mode) {
    case USB_FLASH_DFU: case USB_FLASH_TEENSY: return FAM_CTRL;
    case USB_FLASH_AVR: case USB_FLASH_SAMD:   return FAM_SER;
    case USB_FLASH_ESP:                        return FAM_ESP;
    default:                                   return FAM_MSC;
    }
}

const char *usb_flash_mode_name(uint8_t mode)
{
    static const char *const k[USB_FLASH_MODES] = { "uf2", "file", "dfu", "teensy", "avr", "samd", "esp" };
    return mode < USB_FLASH_MODES ? k[mode] : "?";
}

int usb_flash_mode_from_name(const char *name)
{
    for (int m = 0; m < USB_FLASH_MODES; m++)
        if (name && strcmp(name, usb_flash_mode_name((uint8_t)m)) == 0) return m;
    return -1;
}

static uint32_t now_ms(void) { return xTaskGetTickCount() * portTICK_PERIOD_MS; }

static int read_staged(uint32_t off, uint8_t *buf, uint32_t n)
{
    psram_bus_acquire();
    int rc = psram_read(s.base + off, buf, n);
    psram_bus_release();
    return rc < 0 ? -1 : 0;
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
    return read_staged(off, buf, n);
}

static void op_delay(void *io, uint32_t ms)
{
    (void)io;
    vTaskDelay(pdMS_TO_TICKS(ms ? ms : 1));
}

static int op_ser_write(void *io, const uint8_t *buf, uint32_t n)
{
    (void)io;
    return usb_serial_write(buf, n, IO_TIMEOUT_MS);
}

static uint32_t op_ser_read(void *io, uint8_t *buf, uint32_t n, uint32_t timeout_ms)
{
    (void)io;
    return usb_serial_read(buf, n, timeout_ms);
}

static void op_ser_rx_clear(void *io) { (void)io; usb_serial_rx_clear(); }

static int op_ser_lines(void *io, bool dtr, bool rts)
{
    (void)io;
    return usb_serial_lines(dtr, rts, LINE_MS);
}

static int op_ser_baud(void *io, uint32_t baud)
{
    (void)io;
    return usb_serial_baud(baud, LINE_MS);
}

static bool op_ser_port(void *io, uint32_t *mounts, uint16_t *vid, uint16_t *pid)
{
    (void)io;
    usb_host_info_t hi;
    usb_host_info(&hi);
    *mounts = usb_serial_mounts();
    *vid = hi.vid;
    *pid = hi.pid;
    return usb_serial_ready() && hi.attached;
}

/* ---- esp: the ROM loader over the USB serial port ------------------------------------------ */

static uint32_t esp_send(const uint8_t *frame, uint32_t n)
{
    usb_serial_rx_clear();
    (void)usb_serial_write(frame, n, IO_TIMEOUT_MS);
    /* A USB-serial chip sends at the baud rate; native USB at once. */
    return usb_serial_is_bridge() ? (n * 10u * 1000u) / s.esp_baud + 2u : 2u;
}

static int esp_rx_byte(void) { return usb_serial_getc(); }

static int esp_read(void *ctx, uint32_t off, uint8_t *buf, uint32_t n)
{
    (void)ctx;
    return read_staged(off, buf, n);
}

static int esp_set_baud(uint32_t baud)
{
    if (usb_serial_baud(baud, LINE_MS) != 0) return -1;
    s.esp_baud = baud;
    return 0;
}

/* Into the ROM loader with DTR (IO0) and RTS (EN), as esptool does. */
static void esp_reset_to_loader(void)
{
    if (s.vid == 0x303A && s.pid == 0x1001) {
        /* The USB-Serial/JTAG peripheral (C3, C6, H2, S3): through (1,1), never (0,0) with EN low. */
        (void)usb_serial_lines(false, false, LINE_MS);
        vTaskDelay(pdMS_TO_TICKS(100));
        (void)usb_serial_lines(true, false, LINE_MS);
        vTaskDelay(pdMS_TO_TICKS(100));
        (void)usb_serial_lines(true, true, LINE_MS);
        (void)usb_serial_lines(false, true, LINE_MS);
        vTaskDelay(pdMS_TO_TICKS(100));
        (void)usb_serial_lines(false, false, LINE_MS);
    } else if (s.vid == 0x303A && s.pid == 0x0002) {
        /* The ROM's own USB (S2, S3) already in download mode: nothing to do. */
    } else {
        /* A dev kit's two-transistor auto-reset circuit. */
        (void)usb_serial_lines(false, true, LINE_MS);   /* EN low */
        vTaskDelay(pdMS_TO_TICKS(100));
        (void)usb_serial_lines(true, false, LINE_MS);   /* IO0 low, EN high */
        vTaskDelay(pdMS_TO_TICKS(50));
        (void)usb_serial_lines(false, false, LINE_MS);
    }
}

static void esp_hard_reset(void)
{
    (void)usb_serial_lines(false, true, LINE_MS);   /* EN low, IO0 high */
    vTaskDelay(pdMS_TO_TICKS(100));
    (void)usb_serial_lines(false, false, LINE_MS);
}

/* One pass of the esp job: 1 = running, else the result (esp_rom_job.h). */
static int esp_poll(void)
{
    switch (s.esp_pre) {
    case ESP_PRE_RESET:
        s.esp_mounts = usb_serial_mounts();
        esp_reset_to_loader();
        s.esp_t = now_ms();
        s.esp_pre = ESP_PRE_BACK;
        return 1;
    case ESP_PRE_BACK: {
        /* Native USB enumerates again after the reset; a USB-serial chip stays. */
        bool back = usb_serial_ready();
        uint32_t waited = now_ms() - s.esp_t;
        if (!back || (usb_serial_mounts() != s.esp_mounts && waited < 300u)) {
            if (waited > ESP_BACK_MS) return ESP_ROM_ERR_SYNC;
            return 1;
        }
        s.esp_baud = 115200;
        s_esp_io = (esp_rom_io_t){ esp_send, esp_rx_byte, now_ms, esp_read, NULL,
                                   usb_serial_is_bridge() ? esp_set_baud : NULL };
        esp_rom_opts_t o = { .baud = usb_serial_is_bridge() ? ESP_FAST_BAUD : 0, .flash_size = 0 };
        esp_rom_job_start_ex(&s_esp, &s_esp_io, s.len, s.address, &o);
        s.esp_pre = ESP_PRE_JOB;
        return 1;
    }
    default:
        if (esp_rom_job_step(&s_esp)) return 1;
        if (s_esp.result >= 0 && s.leave) esp_hard_reset();
        return s_esp.result == ESP_ROM_UNVERIFIED ? RC_ESP_UNVERIFIED : s_esp.result;
    }
}

/* ---- the job -------------------------------------------------------------------------------- */

bool usb_flash_busy(void) { return s.running; }

static void free_ctx(void)
{
    if (s.ctx && usb_msc_idle()) { vPortFree(s.ctx); s.ctx = NULL; }
}

static void snapshot(void)
{
    if (family(s.mode) == FAM_ESP) {
        s.phase = (uint8_t)s_esp.phase;
        s.done = esp_rom_job_blocks_done(&s_esp);
        s.total = s_esp.blocks;
        return;
    }
    if (!s.ctx) return;
    switch (family(s.mode)) {
    case FAM_CTRL: { const ctrl_flash_ctx_t *c = s.ctx; s.phase = c->phase; s.done = c->done; s.total = c->total; break; }
    case FAM_SER:  { const ser_flash_ctx_t *c = s.ctx;  s.phase = c->phase; s.done = c->done; s.total = c->total;
                     s.proto = c->proto; break; }
    default:       { const msc_flash_ctx_t *c = s.ctx;  s.phase = c->phase; s.done = c->done; s.total = c->total; break; }
    }
}

static const char *rc_str(int rc)
{
    switch (family(s.mode)) {
    case FAM_CTRL: return ctrl_flash_rc_str(rc);
    case FAM_SER:  return ser_flash_rc_str(rc);
    case FAM_ESP:  return rc == 0 ? "ok" : esp_rom_result_str(rc == RC_ESP_UNVERIFIED ? ESP_ROM_UNVERIFIED : rc);
    default:       return msc_flash_rc_str(rc);
    }
}

static void finish(int rc)
{
    s.rc = rc;
    s.running = false;
    s.have_result = true;
    s.ms = now_ms() - s.t0;
    snapshot();
    if (s.xip) { xip_session_end(); s.xip = false; }
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
    if (p->mode >= USB_FLASH_MODES) { *why = "mode must be uf2, file, dfu, teensy, avr, samd or esp"; return -1; }
    const family_t fam = family(p->mode);
    usb_host_info_t hi;
    usb_host_info(&hi);
    if (hi.mode != USB_HOST_MODE_HOST) { *why = "USB host mode is not on (usb_host)"; return -1; }
    uint32_t blocks = 0, bsize = 0;
    if (fam == FAM_MSC && !usb_msc_ready(&blocks, &bsize)) { *why = "no USB drive is mounted"; return -1; }
    if (fam == FAM_CTRL && !hi.attached) { *why = "no USB device is attached"; return -1; }
    if ((fam == FAM_SER || fam == FAM_ESP) && !(hi.attached && usb_serial_ready())) {
        *why = "no USB serial port is attached"; return -1;
    }
    if (fam == FAM_ESP && (esp_rom_flash_busy() || esp_dut_flash_busy())) {
        *why = "another ESP32 flash is running"; return -1;
    }
    char n83[11];
    if (p->mode == USB_FLASH_FILE && !msc_flash_name83(p->name, n83)) { *why = "name must be an 8.3 file name"; return -1; }
    uint32_t base = 0, staged = 0;
    if (!command_handler_psram_stage(&base, &staged)) { *why = "nothing staged: load_bin with \"psram\":true first"; return -1; }
    if (p->len == 0 || p->len > staged) { *why = "len is 0 or more than the staged upload"; return -1; }

    memset(&s, 0, sizeof(s));
    if (fam != FAM_ESP) {
        xip_err_t xe = xip_session_begin();
        if (xe != XIP_OK) { *why = xip_err_text(xe); return -2; }
        s.xip = true;
        size_t size = fam == FAM_CTRL ? sizeof(ctrl_flash_ctx_t)
                    : fam == FAM_SER  ? sizeof(ser_flash_ctx_t) : sizeof(msc_flash_ctx_t);
        s.ctx = pvPortMalloc(size);
        if (!s.ctx) { xip_session_end(); s.xip = false; *why = "out of memory"; return -1; }
        memset(s.ctx, 0, size);
    }
    switch (fam) {
    case FAM_CTRL: {
        ctrl_flash_ctx_t *c = s.ctx;
        c->ops = (ctrl_flash_ops_t){ &s, op_ctrl, op_src, op_delay, hi.vid, hi.pid };
        c->mode = p->mode == USB_FLASH_TEENSY ? CTRL_FLASH_HALFKAY : CTRL_FLASH_DFU;
        c->len = p->len;
        c->address = p->address;
        c->block = p->block;
        c->leave = p->leave;
        break;
    }
    case FAM_SER: {
        ser_flash_ctx_t *c = s.ctx;
        c->ops = (ser_flash_ops_t){ &s, op_ser_write, op_ser_read, op_ser_rx_clear, op_ser_lines,
                                    op_ser_baud, op_ser_port, op_src, op_delay };
        c->mode = p->mode == USB_FLASH_SAMD ? SER_FLASH_SAMD : SER_FLASH_AVR;
        c->len = p->len;
        c->address = p->address;
        c->leave = p->leave;
        break;
    }
    case FAM_ESP:
        memset(&s_esp, 0, sizeof(s_esp));
        s.esp_pre = ESP_PRE_RESET;
        s.vid = hi.vid;
        s.pid = hi.pid;
        break;
    default: {
        msc_flash_ctx_t *c = s.ctx;
        c->ops = (msc_flash_ops_t){ &s, op_read, op_write, op_src, blocks, bsize };
        c->mode = p->mode == USB_FLASH_FILE ? MSC_FLASH_FILE : MSC_FLASH_UF2;
        memcpy(c->name83, n83, sizeof(n83));
        c->len = p->len;
        break;
    }
    }
    s.base = base;
    s.mode = p->mode;
    s.len = p->len;
    s.address = p->address;
    s.leave = p->leave;
    snprintf(s.name, sizeof(s.name), "%s", p->mode == USB_FLASH_FILE ? p->name : "");
    s.t0 = now_ms();
    s.have_result = false;
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
    int rc;
    switch (family(s.mode)) {
    case FAM_ESP:  rc = esp_poll(); break;
    case FAM_CTRL: rc = xip_session_call_ctx(XIP_FN_CTRL_FLASH_STEP, s.ctx); break;
    case FAM_SER:  rc = xip_session_call_ctx(XIP_FN_SER_FLASH_STEP, s.ctx); break;
    default:       rc = xip_session_call_ctx(XIP_FN_MSC_FLASH_STEP, s.ctx); break;
    }
    snapshot();
    if (rc != 1) finish(rc);   /* 1: still running, in every family */
}

void usb_flash_abort(void)
{
    if (s.running) finish(RC_ABORTED);
}

void usb_flash_status(usb_flash_status_t *st)
{
    const family_t fam = family(s.mode);
    st->running = s.running;
    st->have_result = s.have_result;
    st->mode = usb_flash_mode_name(s.mode);
    if (fam == FAM_ESP && s.running && s.esp_pre != ESP_PRE_JOB) st->phase = "reset";
    else switch (fam) {
    case FAM_CTRL: st->phase = ctrl_flash_phase_str(s.phase); break;
    case FAM_SER:  st->phase = ser_flash_phase_str(s.phase); break;
    case FAM_ESP:  st->phase = esp_rom_phase_str((esp_rom_phase_t)s.phase); break;
    default:       st->phase = msc_flash_phase_str(s.phase); break;
    }
    st->protocol = fam == FAM_SER ? ser_flash_proto_str(s.proto)
                 : (fam == FAM_ESP && s_esp.chip_magic) ? esp_rom_chip_name(s_esp.chip_magic) : "";
    st->rc = s.running ? 1 : s.rc;
    st->result = s.running ? "running" : s.rc == RC_ABORTED ? "aborted" : rc_str(s.rc);
    st->done = s.done;
    st->total = s.total;
    st->len = s.len;
    st->address = s.address;
    st->ms = s.running ? now_ms() - s.t0 : s.ms;
    memcpy(st->name, s.name, sizeof(st->name));
}
