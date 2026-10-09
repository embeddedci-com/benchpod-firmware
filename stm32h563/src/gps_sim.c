/* ============================================================================
 * gps_sim.c — emulated GPS receiver over UART2.  See gps_sim.h.
 * ========================================================================== */

#include "gps_sim.h"
#include "signal_engine.h"   /* fpga_uart2_*, fpga_uart_status, UART2_STATUS_* */
#include "bp_log.h"
#include "pico/time.h"

#include <string.h>

#define STAGE_MAX 1024u      /* one epoch: ~450 B for the full set at 12 satellites */

static gps_fix_t s_fix = {
    .lat_deg = 0.0, .lon_deg = 0.0, .alt_m = 0.0f, .speed_kmh = 0.0f, .course_deg = 0.0f,
    .hdop = 0.9f, .sats = 8, .fix = 1,
    .utc_s = 1767225600u,    /* 2026-01-01T00:00:00Z */
    .utc_ms = 0,
};

static bool     s_active, s_rearm;
static unsigned s_tx_ch, s_rate_hz, s_mask;
static uint32_t s_baud;
static uint64_t s_next_us;
static uint32_t s_epochs, s_overruns, s_bytes;
static char     s_stage[STAGE_MAX];
static size_t   s_stage_len, s_stage_off;

int gps_sim_start(unsigned tx_ch, uint32_t baud, unsigned rate_hz, unsigned mask) {
    if (rate_hz == 0 || rate_hz > GPS_MAX_RATE_HZ || mask == 0 || (mask & ~GPS_SENT_ALL)) return -1;
    if (s_active) gps_sim_stop();
    int rc = fpga_uart2_config(tx_ch, baud, true);
    if (rc == -3) return -3;
    if (rc != 0) return rc == -1 ? -1 : -4;
    s_active = true;
    s_rearm = false;
    s_tx_ch = tx_ch; s_baud = baud; s_rate_hz = rate_hz; s_mask = mask;
    s_epochs = s_overruns = s_bytes = 0;
    s_stage_len = s_stage_off = 0;
    s_next_us = time_us_64();   /* the first epoch goes out on the next poll */
    log_printf("[gps] started on LA%u, %lu baud, %u Hz\n", tx_ch, (unsigned long)baud, rate_hz);
    return 0;
}

void gps_sim_stop(void) {
    if (!s_active) return;
    (void)fpga_uart2_config(s_tx_ch, s_baud, false);
    s_active = false;
    s_rearm = false;
    log_printf("[gps] stopped (%lu epochs, %lu overruns)\n", (unsigned long)s_epochs,
               (unsigned long)s_overruns);
}

const gps_fix_t *gps_sim_fix(void) { return &s_fix; }
void gps_sim_set_fix(const gps_fix_t *f) { if (f) s_fix = *f; }

void gps_sim_on_gateware_reconfigured(void) {
    if (s_active) s_rearm = true;
}

size_t gps_sim_epoch_bytes(const gps_fix_t *f, unsigned mask) {
    static char tmp[STAGE_MAX];   /* not on the caller's stack */
    return gps_nmea_epoch(f, mask, tmp, sizeof(tmp));
}

/* Hand the next piece of the staged epoch to the FIFO once it is empty. */
static void pump(void) {
    if (s_stage_off >= s_stage_len) return;
    uint8_t flags = 0;
    if (fpga_uart_status(NULL, &flags) != 0 || !(flags & UART2_STATUS_TX_EMPTY)) return;
    size_t n = s_stage_len - s_stage_off;
    if (n > FPGA_UART2_FIFO) n = FPGA_UART2_FIFO;
    fpga_uart2_fifo_write((const uint8_t *)&s_stage[s_stage_off], n);
    s_stage_off += n;
    s_bytes += (uint32_t)n;
}

int gps_sim_poll(void) {
    if (!s_active) return 0;
    if (s_rearm) {
        s_rearm = false;
        s_stage_len = s_stage_off = 0;   /* the old FIFO is gone with the fabric */
        if (fpga_uart2_config(s_tx_ch, s_baud, true) != 0) {
            log_printf("[gps] gateware reconfigured: could not re-arm UART2, stopped\n");
            s_active = false;
            return -1;
        }
        log_printf("[gps] gateware reconfigured: re-armed on LA%u\n", s_tx_ch);
    }
    uint64_t now = time_us_64();
    if (now >= s_next_us) {
        uint32_t period_us = 1000000u / s_rate_hz;
        if (s_stage_off < s_stage_len) {
            s_overruns++;                 /* still sending the last one: skip this epoch */
        } else {
            s_stage_len = gps_nmea_epoch(&s_fix, s_mask, s_stage, sizeof(s_stage));
            s_stage_off = 0;
            s_epochs++;
        }
        gps_nmea_advance(&s_fix, period_us / 1000u);
        s_next_us += period_us;
        if (s_next_us < now) s_next_us = now + period_us;   /* the worker stalled: no burst of catch-ups */
    }
    pump();
    return 0;
}

void gps_sim_status(gps_sim_status_t *out) {
    if (!out) return;
    out->active   = s_active;
    out->tx_ch    = s_active ? s_tx_ch : 0;
    out->baud     = s_baud;
    out->rate_hz  = s_rate_hz;
    out->mask     = s_mask;
    out->epochs   = s_epochs;
    out->overruns = s_overruns;
    out->bytes    = s_bytes;
}
