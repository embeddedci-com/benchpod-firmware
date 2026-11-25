/*
 * esp_dut_flash.c — flash an ESP32-family DUT over its UART ROM bootloader (see esp_dut_flash.h).
 *
 * The protocol is esp_rom_job.c, shared with the pod's own C3 flash; this file is the glue: the
 * FPGA soft UART on two LA pins, the EN / boot straps, the image in PSRAM, and the background
 * state machine the hw worker steps.  Runs on the hw worker only (like command_handler).
 */
#include "esp_dut_flash.h"
#include "bp_errors.h"
#include "esp_rom_job.h"
#include "esp_rom_run.h"
#include "command_handler_internal.h"   /* uart_pins_*, command_handler_psram_stage */
#include "signal_engine.h"              /* fpga_uart_*, fpga_la_set, quiesce */
#include "la_pins.h"
#include "nrst_ctrl.h"
#include "psram.h"
#include "bp_log.h"

#include "stm32h5xx_hal.h"              /* HAL_GetTick */

#include <stdio.h>
#include <string.h>

#define EN_LOW_MS       100u   /* EN held low: the RC on a devboard's EN needs the time      */
#define EN_SETTLE_MS     50u   /* EN released until the first SYNC (the ROM samples straps)   */
#define START_BAUD   115200u   /* every ROM loader starts here                               */

typedef enum { ST_IDLE, ST_EN_LOW, ST_EN_SETTLE, ST_JOB, ST_RUN_RESET } st_t;

static struct {
    st_t             st;
    esp_dut_params_t p;
    uint32_t         t_state, t0, ms;
    uint32_t         base;          /* PSRAM address of the staged upload */
    bool             boot_released;
    bool             have_result;
    int              result;
    uint32_t         uart_baud;
} s;

static esp_rom_job_t s_job;
static esp_rom_io_t  s_io;
static uart_pins_t   s_uart_pins;   /* RX / TX and RX's pull-up, the same helpers as the UART proxy */

/* ---- pins ---- */

static void od_set(uint8_t la, uint8_t level)   /* open-drain: 0 drives low, 1 releases */
{
    la_pins_set_level(la, level);
    (void)fpga_la_set(la, la_pins_drive(la));
}

static void en_set(bool low)
{
    if (s.p.en == ESP_DUT_EN_NRST) nrst_ctrl_assert(low);
    else                           od_set(s.p.en, low ? 0 : 1);
}

static void release_pins(void)
{
    fpga_uart_disable();
    uart_pins_release(&s_uart_pins);
    uint8_t gp[2] = { s.p.boot, s.p.en };
    for (int i = 0; i < 2; i++) {
        if (gp[i] == 0) continue;
        la_pins_release(gp[i]);
        (void)fpga_la_set(gp[i], la_pins_drive(gp[i]));   /* back to high-Z */
    }
    if (s.p.en == ESP_DUT_EN_NRST) nrst_ctrl_assert(false);
}

/* ---- esp_rom_io_t over the FPGA soft UART ---- */

static uint8_t  s_rx[128];      /* drained every poll; the FPGA RX FIFO holds 256 */
static uint16_t s_rx_n, s_rx_i;

static int io_rx_byte(void)
{
    if (s_rx_i >= s_rx_n) {
        fpga_uart_tx_pump();                 /* keep a long frame moving into the FPGA FIFO */
        s_rx_i = 0;
        s_rx_n = (uint16_t)fpga_uart_read(s_rx, sizeof(s_rx));
        if (s_rx_n == 0) return -1;
    }
    return s_rx[s_rx_i++];
}

static uint32_t io_send(const uint8_t *frame, uint32_t n)
{
    /* Drop stale input (late replies, the extra SYNC answers). */
    uint8_t junk[64];
    while (fpga_uart_read(junk, sizeof(junk)) > 0) {}
    s_rx_n = s_rx_i = 0;
    if (fpga_uart_write(frame, n) != 0)
        log_printf("[espdut] UART TX ring full (%lu bytes)\n", (unsigned long)n);
    fpga_uart_tx_pump();
    return n * 10u * 1000u / s.uart_baud + 2u;
}

static uint32_t io_now(void) { return HAL_GetTick(); }

static int io_set_baud(uint32_t baud)
{
    /* fpga_uart_config refuses to re-arm an armed UART (-2, "in use"): disarm it first. */
    (void)fpga_uart_config(s.p.rx, s.p.tx, baud, false);
    if (fpga_uart_config(s.p.rx, s.p.tx, baud, true) != 0) {
        LOG_W("[espdut] could not set the UART to %lu baud\n", (unsigned long)baud);
        return -1;
    }
    s_rx_n = s_rx_i = 0;
    s.uart_baud = baud;
    return 0;
}

static int io_read(void *ctx, uint32_t off, uint8_t *buf, uint32_t n)
{
    (void)ctx;
    psram_bus_acquire();
    int rc = psram_read(s.base + off, buf, n);
    psram_bus_release();
    return rc < 0 ? -1 : 0;
}

/* ---- public ---- */

bool esp_dut_flash_busy(void) { return s.st != ST_IDLE; }

int esp_dut_flash_start(const esp_dut_params_t *p, char *err, uint32_t cap)
{
    if (s.st != ST_IDLE) { bp_note_pod_busy(err, cap); return -1; }   /* an ESP32 flash is running */
    if (p->en == ESP_DUT_EN_NRST && !nrst_ctrl_supported()) {
        snprintf(err, cap, "this pod has no reset pin: wire EN to an LA pin and pass en");
        return -1;
    }
    uint32_t base = 0, staged = 0;
    if (p->len > 0) {
        if (!command_handler_psram_stage(&base, &staged)) {
            snprintf(err, cap, "nothing staged: load_bin with \"psram\":true first"); return -1;
        }
        if (p->len > staged) { snprintf(err, cap, "len is more than the staged upload"); return -1; }
    }
    if (p->rx == p->tx || p->boot == p->rx || p->boot == p->tx ||
        (p->en != ESP_DUT_EN_NRST && (p->en == p->rx || p->en == p->tx || p->en == p->boot))) {
        snprintf(err, cap, "rx, tx, boot and en must be different pins"); return -1;
    }

    /* Claim everything before anything moves; on a refusal nothing stays claimed.  The UART pins
       go through the proxy's helpers (uart_pins_claim): the same checks, and RX held high while
       the ROM's TX is not driven yet (before EN is released), like the proxy does. */
    uint8_t gp[2] = { p->boot, p->en };
    size_t ngp = (p->en == ESP_DUT_EN_NRST) ? 1 : 2;
    if (!uart_pins_claim(&s_uart_pins, p->rx, p->tx, 0, false, err, cap)) return -1;
    if (!la_pins_check_claim(LA_FN_GPIO, LA_GPIO_OPEN_DRAIN, gp, ngp, 0, la_pull_mask_now(), err, cap)) {
        uart_pins_release(&s_uart_pins);
        return -1;
    }
    int rc = fpga_uart_config(p->rx, p->tx, START_BAUD, true);
    if (rc != 0) {
        uart_pins_release(&s_uart_pins);
        if (rc == -2) bp_note_uart_session_open(err, cap);
        else          snprintf(err, cap, "invalid uart pins");
        return -1;
    }
    uart_pins_hold_rx(&s_uart_pins);   /* before EN goes low (below) and is released (the poll) */
    la_pins_claim(LA_FN_GPIO, LA_GPIO_OPEN_DRAIN, 1, gp, ngp);   /* claimed released (high) */

    if (p->len > 0) signal_engine_quiesce_psram_masters();   /* no DAC replay on the bus */

    memset(&s, 0, sizeof(s));
    memset(&s_job, 0, sizeof(s_job));
    s.p = *p;
    s.base = base;
    s.uart_baud = START_BAUD;
    s.t0 = HAL_GetTick();

    /* Into the loader: boot low, EN low; the poll releases EN after EN_LOW_MS. */
    od_set(p->boot, 0);
    en_set(true);
    s.st = ST_EN_LOW;
    s.t_state = HAL_GetTick();
    log_printf("[espdut] %s: rx LA%u tx LA%u boot LA%u en %s%u, %lu bytes at 0x%06lx\n",
               p->len ? "flash" : "sync", p->rx, p->tx, p->boot,
               p->en == ESP_DUT_EN_NRST ? "nrst" : "LA", p->en == ESP_DUT_EN_NRST ? 0u : p->en,
               (unsigned long)p->len, (unsigned long)p->offset);
    return 0;
}

static void finish(int result)
{
    s.result = result;
    s.have_result = true;
    s.ms = HAL_GetTick() - s.t0;
    log_printf("[espdut] %s: %s (%s, %lu block(s) resent, %lu ms)\n",
               s.p.len ? "flash" : "sync", esp_rom_result_str(result),
               esp_rom_chip_name(s_job.chip_magic), (unsigned long)s_job.resends,
               (unsigned long)s.ms);
    release_pins();
    s.st = ST_IDLE;
}

void esp_dut_flash_poll(void)
{
    uint32_t now = HAL_GetTick();
    switch (s.st) {
    case ST_IDLE:
        return;
    case ST_EN_LOW:
        if (now - s.t_state < EN_LOW_MS) return;
        en_set(false);
        s.st = ST_EN_SETTLE;
        s.t_state = now;
        return;
    case ST_EN_SETTLE: {
        if (now - s.t_state < EN_SETTLE_MS) return;
        s_io = (esp_rom_io_t){ io_send, io_rx_byte, io_now, s.p.len ? io_read : NULL, NULL, io_set_baud };
        if (s.p.len == 0) {
            esp_rom_job_prepare(&s_job, esp_rom_run_work(), &s_io, 0, 0, NULL, true);
            esp_rom_run_begin(&s_job);
        } else {
            esp_rom_opts_t o = { (s.p.baud && s.p.baud != START_BAUD) ? s.p.baud : 0, s.p.flash_size };
            esp_rom_job_prepare(&s_job, esp_rom_run_work(), &s_io, s.p.len, s.p.offset, &o, false);
            esp_rom_run_begin(&s_job);
        }
        s.st = ST_JOB;
        return;
    }
    case ST_JOB:
        fpga_uart_tx_pump();
        if (esp_rom_run_step(&s_job)) {
            /* The ROM answered: let go of the boot strap, the DUT may use that pin. */
            if (!s.boot_released && s_job.phase > ESP_ROM_SYNC) {
                od_set(s.p.boot, 1);
                s.boot_released = true;
            }
            return;
        }
        if (s.p.len && s.p.run && s_job.result >= 0) {
            od_set(s.p.boot, 1);
            en_set(true);                /* reset into the new image */
            s.st = ST_RUN_RESET;
            s.t_state = now;
            return;
        }
        finish(s_job.result);
        return;
    case ST_RUN_RESET:
        if (now - s.t_state < EN_LOW_MS) return;
        en_set(false);
        finish(s_job.result);
        return;
    }
}

void esp_dut_flash_abort(void)
{
    if (s.st == ST_IDLE) return;
    log_printf("[espdut] aborted\n");
    finish(ESP_ROM_ERR_ARGS);
    s.result = -100;                     /* "aborted" in the status */
}

void esp_dut_flash_status(esp_dut_status_t *st)
{
    memset(st, 0, sizeof(*st));
    st->running     = s.st != ST_IDLE;
    st->have_result = s.have_result;
    st->sync_only   = s.p.len == 0;
    st->offset      = s.p.offset;
    st->len         = s.p.len;
    st->phase = (s.st == ST_EN_LOW || s.st == ST_EN_SETTLE) ? "enter"
              : (s.st == ST_RUN_RESET) ? "reset"
              : (s.st == ST_JOB) ? esp_rom_phase_str(s_job.phase)
              : (s.have_result && s.result < 0 && s_job.failed_phase != ESP_ROM_IDLE)
                    ? esp_rom_phase_str(s_job.failed_phase) : "done";
    st->done   = esp_rom_job_blocks_done(&s_job);
    st->total  = s_job.blocks;
    st->magic  = s_job.chip_magic;
    st->chip   = s_job.chip_magic ? esp_rom_chip_name(s_job.chip_magic) : "";
    st->result = s.result;
    st->result_str = s.result == -100 ? "aborted" : esp_rom_result_str(s.result);
    if (s_job.got_md5[0]) memcpy(st->md5, s_job.got_md5, sizeof(st->md5));
    st->resends = s_job.resends;
    st->baud    = s_job.baud ? s_job.baud : START_BAUD;
    st->ms      = st->running ? HAL_GetTick() - s.t0 : s.ms;
}
