/* ============================================================================
 * signal_engine.c — SPI client for the iCE40 (UPduino) signal-engine FPGA.
 *
 * The RP2350 used to drive an 8-bit parallel DAC and ADC directly through
 * PIO + DMA on 18 GPIOs.  That responsibility now belongs to the iCE40 FPGA.
 * This file keeps the public signal_engine.h API identical so command_handler
 * and main don't need to change — every call here turns into one or more SPI
 * transactions to the FPGA.
 *
 * Wiring (RP2350 GPIO → FPGA):
 *                       RP2350A   RP2350B
 *   SCK                 GPIO 18   GPIO 38
 *   MOSI  (RP → FPGA)   GPIO 19   GPIO 39
 *   MISO  (FPGA → RP)   GPIO 16   GPIO 36
 *   CSn   (active low)  GPIO 17   GPIO 37
 *   BUSY  (FPGA → RP)   GPIO 22   GPIO 42
 *   FPGA pulls BUSY LOW while LOAD_WAVE / capture is in progress; the rising
 *   edge signals async completion.  Both variants land on spi0 — the SPI
 *   function of a pin is fixed by GPIO%4 (0=RX,1=CSn,2=SCK,3=TX) and the
 *   instance by GPIO/8 (even=spi0), so 16-19 and 36-39 are both spi0 groups.
 *
 * SPI command set: see docs/API.md and the plan file.
 * ========================================================================== */

#include "signal_engine.h"
#include "boot_policy.h"
#include "sensor_sim.h"   /* sensor_sim_sda/scl — I2C-LA capture reuses the deep-LA path */

#include "stm32h5xx_hal.h"
#include "board_pins.h"
#include "pico_compat.h"
#include "FreeRTOSConfig.h"
#include "psram.h"
#include "ice40_flash.h"
#include "xfer.h"
#include "dma_wait.h"
#include "hw_lock.h"   /* serialize SPI1/iCE40 access vs the net task */
#include "i2c_bus.h"   /* la_pullups_all_off — 3V3-referenced pull-ups vs the 1V8 bank */
#include "board_rev.h"  /* v2 vs v3 — the TPS2116 mux polarity differs */
#include "nrst_ctrl.h"  /* nRESET is a dedicated pin on v3, not an LA channel */

#include <string.h>
#include <math.h>
#include <stdio.h>

/* iCE40 gateware version (from CMD_VERSION at init).  >= 6 means the v2
   gateware that writes ADC captures into PSRAM (read back over XSPI).  >= 7 adds
   the P7 dedicated 48 MHz capture domain — ADC sample dividers then use
   FPGA_CAPTURE_HZ instead of the 24 MHz FPGA_HFOSC_HZ (see adc_capture_hz). */
static uint8_t s_fpga_version;   /* read + printed at boot; this firmware always
                                    runs the embedded v2 gateware, so there is no
                                    v1/v2 branching any more. */
/* FPGA_FEATURES (0x18): which optional block the RUNNING warmboot image carries —
   bit0=closed-loop DAC, bit1=deep-DAC-PSRAM-replay.  Re-read after every warmboot so
   capabilities track the live image, not the version. */
static uint8_t s_fpga_features;

/* ---- Hardware configuration (STM32H563: iCE40 operational link on SPI1) ----
 * SCK=PA5 MISO=PA6 MOSI=PB5 (AF5), CS=PB10 (GPIO, active low), BUSY=PG1 (input
 * + EXTI1 rising edge = async-op done).  See bsp/board_pins.h. */
static SPI_HandleTypeDef hspi_ice;
#define SPI_PORT          (&hspi_ice)
#define SPI_TIMEOUT_MS    100U

/* SPI1 kernel clock = PLL1Q = 250 MHz.  The usable SCK ceiling is set by the
   iCE40, NOT the STM32 master: in top_v2.v the spi_slave runs on the 24 MHz
   CONTROL-plane clock (clk = clk48/2), and its 2-flop SCK synchroniser adds
   ~2 clk = ~83 ns before MISO updates — so MISO is only valid ~100 ns after the
   SCK edge, and the master (sampling on the next edge) must give it a half-period
   ≥ that.  Measured on the v2.0.0 PCB via the console spi-diag sweep:
     /128 = 1.95 MHz → 100% clean (ping 900/900);  /64 = 3.9 MHz → marginal
     (ping ~5/100, MISO sampled before valid).
   So /128 is the reliable max — a 2x speedup over the old /256 bring-up clock,
   on top of the DMA CPU-availability win.  (spi_slave.v's "8 MHz/48 MHz" note is
   stale for v2; going faster needs the *gateware* to move spi_slave onto the
   48 MHz clk48 domain with proper CDC — not a firmware change.) */
#define SPI_ICE_PRESCALER SPI_BAUDRATEPRESCALER_128
#define SPI_CLOCK_HZ      1953125U      /* 250 MHz / 128 */
#include "fpga_config.h"               /* FPGA_HFOSC_HZ — single source: tools/gen_protocol.py */
#include "la_rate.h"                    /* la_psram_plan — deep-LA rate/divider (host-tested) */

/* ---- SPI1 DMA transfer backend ------------------------------------------
   Big capture read-backs (fpga_read_capture, up to SIGNAL_BUF_SIZE) and waveform
   loads (fpga_load_wave) now move over GPDMA with the calling task blocked on a
   completion semaphore, instead of the old byte-at-a-time HAL loop that made one
   HAL_SPI_TransmitReceive call PER BYTE and spun the CPU the whole time.  Small
   command/status transfers stay blocking (below XFER dma_min) — the xfer policy
   picks per call.  Set ICE_SPI_USE_DMA=0 to force blocking block-transfers. */
#ifndef ICE_SPI_USE_DMA
#define ICE_SPI_USE_DMA   1
#endif
#define ICE_SPI_DMA_MIN   32u   /* transfers smaller than this go blocking */

static DMA_HandleTypeDef hdma_ice_tx, hdma_ice_rx;
static dma_waiter_t      s_ice_waiter;
static bool              s_ice_dma_ok;   /* DMA channels inited OK */

/* Reads clock 0x00 on MOSI while the FPGA shifts out its response; a const
   zero buffer in flash is the DMA TX source (no RAM cost, MOSI byte-identical to
   the old per-byte loop). */
static const uint8_t s_ice_tx_zero[SIGNAL_BUF_SIZE];

static int  ice_start_txrx(void *c, const uint8_t *tx, uint8_t *rx, size_t n) {
    (void)c; dma_wait_arm(&s_ice_waiter);
    return HAL_SPI_TransmitReceive_DMA(&hspi_ice, (uint8_t *)tx, rx, (uint16_t)n) == HAL_OK ? 0 : -1;
}
static int  ice_start_tx(void *c, const uint8_t *tx, size_t n) {
    (void)c; dma_wait_arm(&s_ice_waiter);
    return HAL_SPI_Transmit_DMA(&hspi_ice, (uint8_t *)tx, (uint16_t)n) == HAL_OK ? 0 : -1;
}
static int  ice_wait(void *c, uint32_t to)  { (void)c; return dma_wait_block(&s_ice_waiter, to); }
static void ice_abort(void *c)              { (void)c; HAL_SPI_Abort(&hspi_ice); }
static int  ice_block_txrx(void *c, const uint8_t *tx, uint8_t *rx, size_t n, uint32_t to) {
    (void)c; return HAL_SPI_TransmitReceive(&hspi_ice, (uint8_t *)tx, rx, (uint16_t)n, to) == HAL_OK ? XFER_OK : XFER_ERR;
}
static int  ice_block_tx(void *c, const uint8_t *tx, size_t n, uint32_t to) {
    (void)c; return HAL_SPI_Transmit(&hspi_ice, (uint8_t *)tx, (uint16_t)n, to) == HAL_OK ? XFER_OK : XFER_ERR;
}
static bool ice_dma_ready(void *c)          { (void)c; return s_ice_dma_ok && dma_sched_ready(); }

static const xfer_backend_t s_ice_xfer = {
    .start_txrx = ice_start_txrx, .start_tx = ice_start_tx, .start_rx = NULL,
    .wait = ice_wait, .abort = ice_abort,
    .block_txrx = ice_block_txrx, .block_tx = ice_block_tx, .block_rx = NULL,
    .dma_ready = ice_dma_ready,
    .dma_min = ICE_SPI_DMA_MIN, .timeout_ms = SPI_TIMEOUT_MS,
};

/* True once the SPI1 GPDMA channels inited OK (big transfers then use DMA, not
   the blocking fallback).  Surfaced in console `status` for on-silicon proof. */
bool signal_engine_dma_active(void) { return s_ice_dma_ok; }

/* Pico-API-compatible SPI shims so the command bodies below stay verbatim.
   Return the xfer status (XFER_OK==0, <0 on DMA/blocking failure) so a timed-out
   or errored transfer propagates up instead of silently returning stale data. */
static int spi_write_blocking(SPI_HandleTypeDef *p, const uint8_t *src, size_t n)
{
    (void)p;
    return xfer_tx(&s_ice_xfer, src, n);
}
static int spi_read_blocking(SPI_HandleTypeDef *p, uint8_t tx, uint8_t *dst, size_t n)
{
    /* Every caller reads with tx==0x00; use the flash zero buffer as the TX
       source so the read is one block transfer (DMA when worthwhile). */
    if (tx == 0x00 && n <= sizeof(s_ice_tx_zero)) {
        return xfer_txrx(&s_ice_xfer, s_ice_tx_zero, dst, n);
    }
    for (size_t i = 0; i < n; i++) {   /* non-zero fill / oversized: rare path */
        uint8_t out = tx, in = 0;
        if (HAL_SPI_TransmitReceive(p, &out, &in, 1, SPI_TIMEOUT_MS) != HAL_OK)
            return XFER_ERR;
        dst[i] = in;
    }
    return XFER_OK;
}

/* BUSY input: FPGA holds it low while busy, high when idle/done.  Polled by the
   PSRAM capture flush spins; no interrupt is used. */
static inline bool busy_read(void)
{
    return HAL_GPIO_ReadPin(ICE_BUSY_PORT, ICE_BUSY_PIN) == GPIO_PIN_SET;
}

/* ---- SPI command opcodes (single source: tools/gen_protocol.py) ---- */
#include "cmd_opcodes.h"

#define PING_REPLY_MAGIC   0xA5

/* STATUS byte bit flags (returned by CMD_STATUS, must match gateware) */
#define STATUS_DAC_RUN     (1u << 0)
#define STATUS_CAP_BUSY    (1u << 1)
#define STATUS_CAP_DONE    (1u << 2)
#define STATUS_STEP_BUSY   (1u << 3)
#define STATUS_SWD_ARMED   (1u << 4)
/* v2 gateware >= 9: sticky "the last PSRAM-path capture dropped bytes (writer/
   ring overrun) or was armed over an in-flight capture" — the read-back region is
   truncated/corrupt.  Cleared by the next clean capture arm.  Older gateware
   always reports 0 here, so checking it is safe on any version. */
#define STATUS_CAP_OVF     (1u << 5)
/* v2 gateware >= 30: the control loop LATCHED its over-range trip — the input crossed
   in_trip, so the loop forced its output to vmin and holds it there until disarmed.  Older
   gateware reports 0, so reading it is safe on any version. */
#define STATUS_LOOP_TRIPPED (1u << 6)

/* ---- Internal state ---- */
static uint8_t  sample_buf[SIGNAL_BUF_SIZE];
/* v2: 16-bit waveform staged here (8-bit shape expanded into the high byte)
   and loaded to the DAC8551 as little-endian byte pairs. */
static uint16_t sample_buf16[SIGNAL_BUF_SIZE];

/* Duration timer for DAC (RP-side, identical behaviour to old impl) */
static bool            dac_timed = false;
static absolute_time_t dac_stop_at;

/* ---- Low-level SPI helpers ----------------------------------------------- */

static inline void cs_select(void)   { HAL_GPIO_WritePin(ICE_SPI_CS_PORT, ICE_SPI_CS_PIN, GPIO_PIN_RESET); }
static inline void cs_deselect(void) { HAL_GPIO_WritePin(ICE_SPI_CS_PORT, ICE_SPI_CS_PIN, GPIO_PIN_SET); }

/* Write CMD + optional args in a single CS-asserted burst.  Returns 0 on
   success, <0 if a transfer failed (CS is always released). */
static int spi_cmd_write(uint8_t cmd, const void *args, size_t args_len) {
    cs_select();
    int rc = spi_write_blocking(SPI_PORT, &cmd, 1);
    if (rc == 0 && args && args_len) {
        rc = spi_write_blocking(SPI_PORT, (const uint8_t *)args, args_len);
    }
    cs_deselect();
    return rc;
}

/* Write CMD + args, then clock out resp_len bytes (master sends 0x00 padding,
   FPGA shifts out response on MISO).  Returns 0 on success, <0 on failure — in
   which case `resp` may be incomplete and must not be trusted. */
static int spi_cmd_read(uint8_t cmd, const void *args, size_t args_len,
                        void *resp, size_t resp_len) {
    cs_select();
    int rc = spi_write_blocking(SPI_PORT, &cmd, 1);
    if (rc == 0 && args && args_len) {
        rc = spi_write_blocking(SPI_PORT, (const uint8_t *)args, args_len);
    }
    if (rc == 0 && resp && resp_len) {
        rc = spi_read_blocking(SPI_PORT, 0x00, (uint8_t *)resp, resp_len);
    }
    cs_deselect();
    return rc;
}

/* ---- SPI1 DMA IRQs -------------------------------------------------------
   The GPDMA channel IRQs finish the DMA block and arm the SPI EOT interrupt;
   SPI1_IRQHandler services that EOT and invokes the (shared, dma_wait.c)
   HAL_SPI_*CpltCallback that wakes the transfer.  All three route into the HAL. */
#if ICE_SPI_USE_DMA
void GPDMA1_Channel0_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_ice_rx); }
void GPDMA1_Channel1_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_ice_tx); }
void SPI1_IRQHandler(void)            { HAL_SPI_IRQHandler(&hspi_ice); }
#endif

/* ---- Sample-rate / divider arithmetic ------------------------------------ */

/* Extra FPGA clocks the DAC8551 sequencer spends per sample BEYOND the inter-sample
   divider gap: 3 BRAM read/latch cycles (S_RDLO + S_RDHI + S_LAT) plus a 24-bit SPI
   frame shifted at 2 clocks/bit (S_SHIFT) = 3 + 48 = 51.  See ice40/src/dac8551_
   engine.v.  So one sample actually takes (divider + DAC_SEQ_OVERHEAD_CLK) clocks and
   the real update rate is HFOSC/(divider+K), NOT HFOSC/divider.  Ignoring K made
   generate() play far below the requested frequency at small dividers (the output
   frequency "saturated": e.g. 1 kHz asked at a nominal 1 MS/s came out ~310 Hz). */
#define DAC_SEQ_OVERHEAD_CLK 51u

/* Smallest DAC divider firmware sends — and, from gateware v34, the smallest the engine honours
   (it floors lower ones to this).  The next DAC8551 frame's SYNC falls divider+3 clocks after the
   previous frame's 24th SCLK falling edge; the datasheet needs >= 100 ns there (t9), and divider
   2 gave 5 x 20.8 = 104 ns.  3 gives 125 ns for ~2% of the peak update rate (48 MHz/54). */
#define DAC_MIN_DIVIDER 3u

/* True DAC update rate (Hz) achieved by the sequencer for a given divider.  The
   DAC engine runs on DAC_CLK_HZ (48 MHz on gateware >= 13), not the 24 MHz HFOSC. */
static float dac_effective_rate_hz(uint32_t divider) {
    return (float)DAC_CLK_HZ / (float)(divider + DAC_SEQ_OVERHEAD_CLK);
}

/* Pick the DAC sample-clock divider and per-period sample count so the played
   OUTPUT frequency equals `freq`.  Key: the real sample rate is
   dac_effective_rate_hz(divider) = HFOSC/(divider+K), so
       output_freq = HFOSC / ((divider + K) * period)
   and we compute the period from that true rate (not HFOSC/divider).

   Constraint: period_samples ≤ SIGNAL_MAX_SAMPLES (the DAC waveform is 16-bit, so
   period*2 bytes must fit the SIGNAL_BUF_SIZE-byte BRAM).

   forced_sample_rate_hz:
     0    → auto-pick the smallest divider (fastest, smoothest waveform) whose
            per-period sample count still fits the buffer.
     > 0  → treat it as the desired EFFECTIVE (real) update rate and back out the
            divider that yields it (rate = HFOSC/(divider+K)).  A lower rate gives a
            coarser waveform but reduces clock feedthrough.  If the resulting period
            exceeds the buffer it is clamped and a warning is logged (the actual
            output frequency will then be higher than requested).

   divider floor is DAC_MIN_DIVIDER (DAC8551 frame timing, see above). */
static uint32_t compute_divider_and_period(float freq, float forced_sample_rate_hz,
                                           uint32_t *out_period) {
    if (freq <= 0.0f) {
        *out_period = 0;
        return 0;
    }

    uint32_t divider;
    if (forced_sample_rate_hz > 0.0f) {
        /* Effective rate = DAC_CLK/(divider+K) → divider = DAC_CLK/rate - K. */
        float d = (float)DAC_CLK_HZ / forced_sample_rate_hz - (float)DAC_SEQ_OVERHEAD_CLK;
        divider = (d < (float)DAC_MIN_DIVIDER) ? DAC_MIN_DIVIDER
                : (d > 65535.0f) ? 65535u : (uint32_t)(d + 0.5f);
    } else {
        /* Smallest divider whose per-period count fits: period = eff_rate/freq ≤ MAX
           ⇒ divider ≥ DAC_CLK/(MAX*freq) - K.  Take the ceiling, floored at DAC_MIN_DIVIDER. */
        float dmin = (float)DAC_CLK_HZ / ((float)SIGNAL_MAX_SAMPLES * freq)
                     - (float)DAC_SEQ_OVERHEAD_CLK;
        divider = (dmin < (float)DAC_MIN_DIVIDER) ? DAC_MIN_DIVIDER
                : (dmin >= 65535.0f) ? 65535u : (uint32_t)dmin + 1u;
    }

    float    eff_rate = dac_effective_rate_hz(divider);
    uint32_t period   = (uint32_t)(eff_rate / freq + 0.5f);
    if (period == 0) period = 1;
    if (period > SIGNAL_MAX_SAMPLES) {
        /* Integer units — newlib-nano printf has no %f. Waveform freq can be sub-Hz
           here (low freq -> long period), so report it in milli-Hz; the rate is
           always large, so integer S/s. */
        printf("[sig] WARN: %lu mHz needs %lu samples/period at %lu S/s — exceeds "
               "%u-sample waveform buffer; clamping (actual freq will be higher)\n",
               (unsigned long)lroundf(freq * 1000.0f), (unsigned long)period,
               (unsigned long)lroundf(eff_rate), SIGNAL_MAX_SAMPLES);
        period = SIGNAL_MAX_SAMPLES;
    }
    *out_period = period;
    return divider;
}

/* Convert a desired sample rate (Hz) into an FPGA clock divider.
   sample_rate = FPGA_HFOSC_HZ / divider, so divider = HFOSC / rate.
   0 (or negative) → divider 2, the max rate (12 MSPS) — preserves the historic
   capture/replay default.  Clamped to the achievable divider range 2..65535. */
static uint32_t divider_from_rate_hz(float sample_rate_hz) {
    if (sample_rate_hz <= 0.0f) return 2;
    uint32_t d = (uint32_t)((float)FPGA_HFOSC_HZ / sample_rate_hz + 0.5f);
    if (d < 2)     d = 2;
    if (d > 65535) d = 65535;
    return d;
}

/* Same, but against the DAC engine's clock (DAC_CLK_HZ = 48 MHz on gw >= 13) — for
   the arbitrary/replay DAC playback rate.  Floored at DAC_MIN_DIVIDER, not 2. */
static uint32_t dac_divider_from_rate_hz(float sample_rate_hz) {
    if (sample_rate_hz <= 0.0f) return DAC_MIN_DIVIDER;
    uint32_t d = (uint32_t)((float)DAC_CLK_HZ / sample_rate_hz + 0.5f);
    if (d < DAC_MIN_DIVIDER) d = DAC_MIN_DIVIDER;
    if (d > 65535) d = 65535;
    return d;
}

/* ADC capture clock.  The serial-ADC engine (adc_mcp33131) runs on the 24 MHz
   FPGA_HFOSC_HZ clock — the same domain as the DAC/UART/I2C-LA — so ADC sample-rate
   dividers are computed against 24 MHz.  (The pre-P7 belief that >=v7 captured on a
   dedicated 48 MHz clock was the bug: the engine was demoted to 24 MHz to make the
   SDO read work, but this returned FPGA_CAPTURE_HZ, so every capture ran at HALF
   the requested rate and MEASURE's ADC/DAC correlation was 2x off.  Restored to
   FPGA_HFOSC_HZ; the gateware's CAP_DIV_SHIFT is now 0 to match — see top_v2.v.) */
static uint32_t adc_capture_hz(void) {
    return FPGA_HFOSC_HZ;   /* 24 MHz — the ADC engine's actual clock (v1 and v2) */
}

/* Minimum ADC sample-period divider.  The 24 MHz serial-ADC engine is busy ~59
   clocks/sample (1 us conversion = 24 clks + a 16-bit readout at 12 MHz SCLK = 32
   clks + transitions), so the gateware physically floors the sample period at
   ADC_CAP_MIN_DIVIDER — matching adc_mcp33131.v's floor (60 -> ~400 kSPS max, the
   MCP33131D-10 is a 1 MSPS part).  Requesting faster just makes the gateware floor
   it while firmware over-reports the rate (a wrong timebase), so clamp here. */
#define ADC_CAP_MIN_DIVIDER 60u

/* Shortest real ADC sample period for the connected gateware.  Gateware older than
   EXACT_CAP_DIVIDER_MIN_GW takes divider + 1 clocks, so firmware sends period - 1
   (cap_divider_wire) — and that wire value is still floored at 60, so the fastest
   real period there is 61. */
static uint32_t adc_min_divider(void) {
    return (s_fpga_version >= EXACT_CAP_DIVIDER_MIN_GW) ? ADC_CAP_MIN_DIVIDER
                                                        : ADC_CAP_MIN_DIVIDER + 1u;
}

/* divider_from_rate_hz against the ADC capture clock (see adc_capture_hz).  Returns
   the sample PERIOD in clocks; encode it with cap_divider_wire() when sending. */
static uint32_t adc_divider_from_rate_hz(float sample_rate_hz) {
    uint32_t min = adc_min_divider();
    if (sample_rate_hz <= 0.0f) return min;   /* default = max rate */
    uint32_t d = (uint32_t)((float)adc_capture_hz() / sample_rate_hz + 0.5f);
    if (d < min)   d = min;
    if (d > 65535) d = 65535;
    return d;
}

/* ---- Waveform fill (unchanged from the old PIO-based impl) --------------- */

static void fill_sine(uint32_t n, uint8_t amplitude, uint8_t offset) {
    for (uint32_t i = 0; i < n; i++) {
        float   angle = 2.0f * 3.14159265f * (float)i / (float)n;
        int32_t v     = (int32_t)(sinf(angle) * amplitude) + offset;
        if (v < 0)   v = 0;
        if (v > 255) v = 255;
        sample_buf[i] = (uint8_t)v;
    }
}

static void fill_square(uint32_t n, uint8_t amplitude, uint8_t offset) {
    for (uint32_t i = 0; i < n; i++) {
        int32_t v = (i < n / 2) ? ((int32_t)offset + amplitude)
                                 : ((int32_t)offset - amplitude);
        if (v < 0)   v = 0;
        if (v > 255) v = 255;
        sample_buf[i] = (uint8_t)v;
    }
}

static void fill_sawtooth(uint32_t n, uint8_t amplitude, uint8_t offset) {
    for (uint32_t i = 0; i < n; i++) {
        int32_t v = (int32_t)offset - amplitude
                    + (int32_t)(2u * amplitude * i / n);
        if (v < 0)   v = 0;
        if (v > 255) v = 255;
        sample_buf[i] = (uint8_t)v;
    }
}

/* ---- Diagnostic: dump the first N samples being sent to the DAC ---------- */

/* How many samples to dump in the log when a waveform is loaded.  Set to 0
   to suppress entirely.  20 lines is concise enough not to bury the log. */
#define DAC_DUMP_SAMPLES  20

static void format_bin8(char *out10, uint8_t v) {
    for (int i = 0; i < 4; i++) out10[i]     = (v & (1u << (7 - i))) ? '1' : '0';
    out10[4] = '_';
    for (int i = 0; i < 4; i++) out10[5 + i] = (v & (1u << (3 - i))) ? '1' : '0';
    out10[9] = '\0';
}

static void log_dac_samples(uint32_t total) {
    uint32_t n = (total < DAC_DUMP_SAMPLES) ? total : DAC_DUMP_SAMPLES;
    if (n == 0) return;
    printf("[sig] first %lu of %lu DAC samples loaded to FPGA (D7..D0):\n",
           (unsigned long)n, (unsigned long)total);
    char bin[16];
    for (uint32_t i = 0; i < n; i++) {
        format_bin8(bin, sample_buf[i]);
        printf("[sig]   [%2lu] %s  (0x%02x = %3u)\n",
               (unsigned long)i, bin, sample_buf[i], sample_buf[i]);
    }
}

/* ---- FPGA command wrappers ----------------------------------------------- */

/* LOAD_WAVE:  [CMD][len_lo][len_hi][N data bytes] */
static int fpga_load_wave(const uint8_t *data, size_t len) {
    if (len == 0 || len > SIGNAL_BUF_SIZE) return -1;
    uint8_t header[3] = { CMD_LOAD_WAVE,
                          (uint8_t)(len & 0xFF),
                          (uint8_t)((len >> 8) & 0xFF) };
    cs_select();
    int rc = spi_write_blocking(SPI_PORT, header, sizeof(header));
    if (rc == 0) rc = spi_write_blocking(SPI_PORT, data, len);
    cs_deselect();
    return rc;
}

/* START_DAC: [CMD][period_lo][period_hi][div_lo][div_hi] */
static int fpga_start_dac(uint32_t period, uint32_t divider) {
    divider = dac_divider_wire(divider, s_fpga_version);   /* v40+: the reload, divider - 1 */
    uint8_t args[4] = {
        (uint8_t)(period  & 0xFF), (uint8_t)((period  >> 8) & 0xFF),
        (uint8_t)(divider & 0xFF), (uint8_t)((divider >> 8) & 0xFF),
    };
    spi_cmd_write(CMD_START_DAC, args, sizeof(args));
    return 0;
}

/* A co-triggered DAC start (DAC_ARM_ON_CAPTURE) staged for the next capture arm, and whether the
   capture in flight took it — so a trigger-timeout abort (an untriggered t0) can cancel it first. */
static bool s_cotrig_pending;
static bool s_cotrig_in_capture;

static void cotrig_consume(void) {   /* at every capture arm */
    s_cotrig_in_capture = s_cotrig_pending;
    s_cotrig_pending    = false;
}

static int fpga_stop_dac(void) {
    spi_cmd_write(CMD_STOP_DAC, NULL, 0);
    s_cotrig_pending    = false;   /* STOP_DAC cancels a staged-but-uncaptured start */
    s_cotrig_in_capture = false;
    return 0;
}

/* START_DAC_PSRAM (deep replay, gateware >= v17):
   [CMD][base(3,LE)][count(3,LE samples)][div(2,LE)].  The iCE40 streams the
   waveform straight out of PSRAM to the DAC8551, so replay depth is bounded by
   PSRAM (up to FPGA_DAC_REPLAY_MAX_SAMPLES), not the 4 KB LOAD_WAVE BRAM.  The
   waveform must already be staged in PSRAM at `base` and the STM32 must have
   released the shared bus (psram_bus_release) so the iCE40 can read it. */
static int fpga_start_dac_psram(uint32_t base, uint32_t count, uint32_t divider) {
    divider = dac_divider_wire(divider, s_fpga_version);   /* v40+: the reload, divider - 1 */
    uint8_t args[8] = {
        (uint8_t)(base    & 0xFF), (uint8_t)((base    >> 8) & 0xFF), (uint8_t)((base    >> 16) & 0xFF),
        (uint8_t)(count   & 0xFF), (uint8_t)((count   >> 8) & 0xFF), (uint8_t)((count   >> 16) & 0xFF),
        (uint8_t)(divider & 0xFF), (uint8_t)((divider >> 8) & 0xFF),
    };
    spi_cmd_write(CMD_START_DAC_PSRAM, args, sizeof(args));
    return 0;
}

/* SET_DAC_STOP_AFTER (capture-tied DAC auto-stop, gateware >= v21): [CMD][cycles(4,LE)].
   Latches a threshold in cmd_dispatch (24 MHz clk cycles); the top snapshots it at the next
   capture's t0 and cuts a concurrently-running DAC when the count expires.  0 = disarm. */
static int fpga_set_dac_stop_after(uint32_t cycles) {
    uint8_t args[4] = {
        (uint8_t)(cycles & 0xFF), (uint8_t)((cycles >> 8) & 0xFF),
        (uint8_t)((cycles >> 16) & 0xFF), (uint8_t)((cycles >> 24) & 0xFF),
    };
    spi_cmd_write(CMD_SET_DAC_STOP_AFTER, args, sizeof(args));
    return 0;
}

/* START_MEASURE (gateware >= 12): FPGA starts both the DAC sequencer (using the
   most recently loaded waveform) and the ADC capture in the same clock cycle, each
   with its OWN divider — [count(2)][dac_div(2)][cap_div(2)].  Separate dividers let
   firmware offset the DAC sequencer's per-sample overhead so the DAC and ADC step
   at the same real rate (the ADC then captures exactly one played period, aligned). */
static int fpga_start_measure(uint32_t samples, uint32_t dac_div, uint32_t cap_div) {
    cap_div = cap_divider_wire(cap_div, s_fpga_version);   /* period -> wire divider */
    uint8_t args[6] = {
        (uint8_t)(samples & 0xFF),  (uint8_t)((samples  >> 8) & 0xFF),
        (uint8_t)(dac_div & 0xFF),  (uint8_t)((dac_div  >> 8) & 0xFF),
        (uint8_t)(cap_div & 0xFF),  (uint8_t)((cap_div  >> 8) & 0xFF),
    };
    spi_cmd_write(CMD_START_MEASURE, args, sizeof(args));
    return 0;
}

/* CAPTURE (0x31): [adc_cnt(3)][adc_div(2)][la_cnt(3)][la_div(2)], counts and PERIODS in, the
   wire encodings for the connected gateware out.  A producer with count 0 is not in the capture
   (v40+ gateware then also keeps its divider: the ADC one paces the free-running ADC). */
static void fpga_capture_cmd(uint32_t adc_count, uint32_t adc_period,
                             uint32_t la_count, uint32_t la_period) {
    uint16_t adc_wire = cap_divider_wire(adc_period, s_fpga_version);
    uint16_t la_wire  = la_divider_wire(la_period,   s_fpga_version);
    uint8_t args[10] = {
        (uint8_t)adc_count, (uint8_t)(adc_count >> 8), (uint8_t)(adc_count >> 16),
        (uint8_t)adc_wire,  (uint8_t)(adc_wire  >> 8),
        (uint8_t)la_count,  (uint8_t)(la_count  >> 8), (uint8_t)(la_count  >> 16),
        (uint8_t)la_wire,   (uint8_t)(la_wire   >> 8),
    };
    spi_cmd_write(CMD_CAPTURE, args, sizeof(args));
}

/* ---- Diagnostics --------------------------------------------------------- */

int fpga_ping(int attempts, uint8_t *out_version) {
    if (attempts < 1) attempts = 1;
    for (int i = 1; i <= attempts; i++) {
        uint8_t reply = 0;
        spi_cmd_read(CMD_PING, NULL, 0, &reply, 1);
        if (reply == PING_REPLY_MAGIC) {
            uint8_t version = 0;
            spi_cmd_read(CMD_VERSION, NULL, 0, &version, 1);
            printf("[sig] FPGA OK on attempt %d/%d, gateware version %u\n",
                   i, attempts, version);
            if (out_version) *out_version = version;
            return 0;
        }
        printf("[sig] PING attempt %d/%d failed (got 0x%02x, expected 0x%02x)\n",
               i, attempts, reply, PING_REPLY_MAGIC);
        sleep_ms(50);
    }
    printf("[sig] ERROR: FPGA unreachable after %d attempts\n", attempts);
    printf("[sig]   - 0x00 → SCK or MOSI not reaching FPGA, or gateware not loaded\n");
    printf("[sig]   - 0xFF → MISO disconnected (floats high with internal pull-up)\n");
    printf("[sig]   - other → SPI mode mismatch (CPOL/CPHA) or bit-order issue\n");
    return -1;
}

int fpga_status_read(uint8_t *out_status) {
    uint8_t status = 0xFF;
    spi_cmd_read(CMD_STATUS, NULL, 0, &status, 1);
    if (out_status) *out_status = status;
    /* STATUS uses the bottom 6 bits now (DAC_RUN, CAP_BUSY, CAP_DONE, STEP_BUSY,
       SWD_ARMED, and CAP_OVF on gateware >= 9).  Only bits 6-7 are reserved, so a
       stuck-high MISO is detected on those two.  (This mask was 0xE0 before v9,
       which would have misread a legitimate CAP_OVF as a stuck bus.) */
    return (status & 0xC0) ? -1 : 0;
}

/* v9+ gateware sets STATUS_CAP_OVF when the just-completed PSRAM capture dropped
   bytes or was armed over an in-flight capture — the read-back is then truncated/
   corrupt.  Log + return true so callers fail loudly instead of returning garbage.
   On < v9 gateware the bit is always 0, so this is a no-op there. */
static bool cap_overflowed(uint8_t st, const char *what) {
    /* A CONFIGURED iCE40 always drives STATUS bits 7:6 = 0 (cmd_dispatch hardwires
       them to 2'b0).  If either is set the iCE40 isn't driving MISO at all — STATUS
       floated to 0xFF via the pull-ups because the FPGA is UNCONFIGURED/wedged (it
       loses config on a power blip / the config-window race, and a plain reboot does
       NOT reload it).  bit5 (overflow) is then a false positive, so check this FIRST
       and tell the user the real fix — reconfigure — instead of "lower the rate". */
    if ((st & 0xC0) != 0) {
        printf("[sig] ERROR: %s failed — iCE40 NOT RESPONDING (STATUS=0x%02x): the FPGA "
               "is unconfigured/wedged, not overflowing.  Reconfigure it (console "
               "`flash-ice40`); a plain reboot won't fix it.\n", what, st);
        return true;
    }
    if (st & STATUS_CAP_OVF) {
        printf("[sig] ERROR: %s OVERFLOW (STATUS bit5) — PSRAM capture dropped "
               "bytes / overlapping arm; region truncated/corrupt. Lower the rate.\n",
               what);
        return true;
    }
    return false;
}

/* ---- SPI-clock diagnostics (console spi-clk / spi-diag) ------------------
   Change the SPI1 SCK live, and loop-test read reliability at that clock, so the
   real usable SPI frequency can be swept from one flash without reflashing. */
uint32_t signal_engine_spi_set_prescaler(uint32_t div) {
    uint32_t presc;
    switch (div) {
    case 2:   presc = SPI_BAUDRATEPRESCALER_2;   break;
    case 4:   presc = SPI_BAUDRATEPRESCALER_4;   break;
    case 8:   presc = SPI_BAUDRATEPRESCALER_8;   break;
    case 16:  presc = SPI_BAUDRATEPRESCALER_16;  break;
    case 32:  presc = SPI_BAUDRATEPRESCALER_32;  break;
    case 64:  presc = SPI_BAUDRATEPRESCALER_64;  break;
    case 128: presc = SPI_BAUDRATEPRESCALER_128; break;
    case 256: presc = SPI_BAUDRATEPRESCALER_256; break;
    default:  return 0;
    }
    hspi_ice.Init.BaudRatePrescaler = presc;
    if (HAL_SPI_Init(&hspi_ice) != HAL_OK) return 0;
    return 250000000u / div;   /* SPI1 kernel = PLL1Q = 250 MHz */
}

/* Loop n rounds of {PING, VERSION, STATUS} reads and report reliability: how
   many PINGs returned the magic, whether VERSION is stable/correct, and whether
   STATUS bits flap (OR vs AND across rounds).  Clean link → ping n/n, version
   stable & >=6, STATUS or==and.  A corrupting clock shows mismatches. */
void signal_engine_spi_diag(int n) {
    if (n < 1) n = 1;
    int ping_ok = 0;
    bool ver_stable = true;
    uint8_t first_ver = 0, st_or = 0x00, st_and = 0xFF;
    for (int i = 0; i < n; i++) {
        uint8_t reply = 0, ver = 0, st = 0;
        spi_cmd_read(CMD_PING, NULL, 0, &reply, 1);
        spi_cmd_read(CMD_VERSION, NULL, 0, &ver, 1);
        spi_cmd_read(CMD_STATUS, NULL, 0, &st, 1);
        if (reply == PING_REPLY_MAGIC) ping_ok++;
        if (i == 0) first_ver = ver;
        else if (ver != first_ver) ver_stable = false;
        st_or |= st; st_and &= st;
    }
    printf("[spi-diag] %d rounds: ping_ok=%d/%d  version=0x%02x(%s)  "
           "STATUS or=0x%02x and=0x%02x  => %s\n",
           n, ping_ok, n, first_ver, ver_stable ? "stable" : "VARIES",
           st_or, st_and,
           (ping_ok == n && ver_stable && st_or == st_and) ? "CLEAN" : "CORRUPT");
}

/* Direct-over-SPI ADC read (CMD_ADC_PROBE, gateware >= v8 w/ ADC_PROBE): returns
   the LIVE adc_mcp33131 sample straight off the free-running engine, bypassing
   the capture orchestrator, the 24->48 MHz CDC, AND the PSRAM datapath.  Lets us
   tell whether a bad ADC read (e.g. the 0x5555 family) is the engine/silicon or
   the streaming path.  Console holds hw_lock (no locking here, like spi_diag). */
int signal_engine_adc_spi(uint16_t *out) {
    uint8_t resp[2] = { 0, 0 };
    int rc = spi_cmd_read(CMD_ADC_PROBE, NULL, 0, resp, sizeof(resp));
    if (rc != 0) return rc;
    if (out) *out = (uint16_t)resp[0] | ((uint16_t)resp[1] << 8);   /* LE */
    return 0;
}

/* Read n live samples directly over SPI and print them + a min/max/span summary
   (so a static 0x5555 vs a varying/driven value is obvious).  Diagnostic only. */
void signal_engine_adc_spi_diag(int n) {
    if (n < 1)  n = 1;
    if (n > 32) n = 32;
    uint16_t mn = 0xFFFF, mx = 0x0000;
    int fails = 0;
    printf("[adc-spi] %d direct reads (CMD_ADC_PROBE — no PSRAM/CDC): ", n);
    for (int i = 0; i < n; i++) {
        uint16_t v = 0;
        if (signal_engine_adc_spi(&v) != 0) { printf("FAIL "); fails++; continue; }
        if (v < mn) mn = v;
        if (v > mx) mx = v;
        printf("0x%04x%s", v, (i == n - 1) ? "" : " ");
    }
    printf("\n[adc-spi] min=0x%04x max=0x%04x span=%u fails=%d => %s\n",
           mn, mx, (unsigned)(mx - mn), fails,
           (mx - mn) < 8 ? "STATIC" : "VARYING");
}

/* ---- ADC->PSRAM capture-path self-test (CAPTURE_TEST) ---------------------
   The iCE40 substitutes a known +0x0101 ramp for the real ADC sample and streams
   it through the EXACT datapath the ADC capture uses: the 24->48 MHz sample CDC,
   the psram_writer, PSRAM, and the STM32 XSPI read-back.  If the ramp returns
   clean, that whole path (including the CDC that once corrupted stable samples
   into the 0x5555 family) is healthy; if it returns broken, it is an iCE40
   capture/timing fault — NOT the analog ADC.  This is the oracle that answers
   "is a bad capture the ADC or the path?" without guesswork. */
int signal_engine_capture_test_mode(int mode) {
    uint8_t b = (uint8_t)(mode & 0x03);   /* 0=ADC 1=ramp@24MHz(CDC) 2=ramp@48MHz(writer) */
    return spi_cmd_write(CMD_CAPTURE_TEST, &b, 1);
}

/* Capture n ramp samples in the given injection mode and return the index of the
   first sample whose step != +0x0101 (dropped/dup/corrupt), or -1 if the whole
   ramp is clean, or -2 on a transport error.  buf[] (>= n) gets the samples. */
static int capture_selftest_one(int mode, int n, uint16_t *buf) {
    int rc = signal_engine_capture_test_mode(mode);
    if (rc == 0) rc = adc_capture_psram(buf, (size_t)n, 0.0f);
    signal_engine_capture_test_mode(0);                /* always restore real ADC */
    if (rc != 0) return -2;
    for (int i = 1; i < n; i++)
        if ((uint16_t)(buf[i] - buf[i - 1]) != 0x0101u) return i;
    return -1;
}

/* Push a known +0x0101 ramp through the real ADC->PSRAM capture path (single
   24 MHz domain now — writer + PSRAM + read-back; there is no CDC any more) and
   verify it comes back clean.  Returns 0 = PASS, -1 = FAIL (first bad index in
   *bad_idx).  A FAIL is always the iCE40 capture path, never the analog ADC. */
int signal_engine_capture_selftest(int n, int *bad_idx) {
    static uint16_t buf[256];
    if (n < 4)   n = 4;
    if (n > 256) n = 256;
    if (bad_idx) *bad_idx = -1;

    int bad = capture_selftest_one(1, n, buf);
    printf("[cap-selftest] %d samples  first8: ", n);
    for (int i = 0; i < 8 && i < n; i++) printf("0x%04x ", buf[i]);
    if (bad == -1) {
        printf("\n[cap-selftest]   => PASS: ADC->PSRAM capture path is HEALTHY.\n");
        return 0;
    }
    if (bad == -2) {
        printf("\n[cap-selftest]   => FAIL: capture transport error (SPI/PSRAM).\n");
        return -1;
    }
    printf("\n[cap-selftest]   => FAIL at sample %d (step 0x%04x, want 0x0101): iCE40 "
           "capture path is corrupting KNOWN data — NOT the analog ADC.\n",
           bad, (uint16_t)(buf[bad] - buf[bad - 1]));
    if (bad_idx) *bad_idx = bad;
    return -1;
}

/* Boot self-test 3: does the iCE40 reach EVERY shared PSRAM-bus line?  Force the
   iCE40 (CMD_PSRAM_CS) to statically drive a known pattern on /CS+SCLK+IO0-3, then
   read all six back over GPIO.  *lines gets the raw reading (bit layout as
   psram_bus_read_lines); returns the count of lines that match the driven pattern
   (6 = all reach the PSRAM; a mismatch = that iCE40 pad->net is open), <0 error.
   Only drives static levels, never clocks a transfer, so it can't corrupt PSRAM. */
#define PSRAM_BUS_EXPECT  0x0Eu   /* /CS=0 SCLK=1 IO0=1 IO1=1 IO2=0 IO3=0 */
int signal_engine_psram_bus_reach(uint8_t *lines) {
    uint8_t on = 1u, off = 0u;
    if (spi_cmd_write(CMD_PSRAM_CS, &on, 1) != 0) return -1;
    psram_ce_probe_begin();
    sleep_ms(2);
    uint8_t rd = psram_bus_read_lines();
    psram_ce_probe_end();
    spi_cmd_write(CMD_PSRAM_CS, &off, 1);
    if (lines) *lines = rd;
    int match = 0;
    for (int b = 0; b < 6; b++)
        if (((rd ^ PSRAM_BUS_EXPECT) >> b & 1u) == 0) match++;
    return match;
}

/* Layered PSRAM datapath self-test (console 'psram-selftest' + once at boot):
     1) STM32<->PSRAM      — psram_test(): STM32 writes+reads a pattern.
     2) iCE40->PSRAM write — capture_selftest(): iCE40 writes a ramp, STM32 reads it.
     3) if 2 fails: does the iCE40 even REACH the /CE net? — psram_cs_reaches().
   Localises any fault to the STM32 side, the iCE40 write, or the iCE40 pad45->/CE
   joint — never mis-blaming the analog ADC. */
/* Cached result of the last psram_boot_selftest, surfaced on the console 'status'
   line (see psram_selftest_str).  Runs at boot and on 'psram-selftest'. */
static int s_psram_selftest_rc = PSRAM_ST_NOT_RUN;

int psram_selftest_result(void) { return s_psram_selftest_rc; }

const char *psram_selftest_str(void) {
    switch (s_psram_selftest_rc) {
    case PSRAM_ST_OK:              return "ok";
    case PSRAM_ST_SKIP:            return "n/a (v1)";
    case PSRAM_ST_STM32_FAIL:      return "inoperable (STM32<->PSRAM)";
    case PSRAM_ST_ICE40_WRITE_FAIL:return "inoperable (iCE40 write)";
    case PSRAM_ST_ICE40_REACH_FAIL:return "inoperable (iCE40 pad open)";
    default:                       return "not run";
    }
}

/* True while the PSRAM capture datapath is known-good (last selftest passed).  Surfaced as
   status.psram_ok so the server/UI can flag the fault and offer the reboot+reflash recovery. */
bool signal_engine_psram_operable(void) { return s_psram_selftest_rc == PSRAM_ST_OK; }

/* Mark the PSRAM datapath inoperable after a RUNTIME wedge (a capture/replay found the
   iCE40->PSRAM path dead).  Makes status.psram flip to "inoperable" without re-running the
   full selftest, so the UI sees the fault the moment it happens.  Only downgrades from
   OK/not-run; a more specific boot diagnosis is left intact.  Cleared by the next passing
   selftest (boot / psram-selftest / the boot auto-reflash). */
void signal_engine_psram_report_wedge(void) {
    if (s_psram_selftest_rc == PSRAM_ST_OK || s_psram_selftest_rc == PSRAM_ST_NOT_RUN)
        s_psram_selftest_rc = PSRAM_ST_ICE40_WRITE_FAIL;
}

/* Boot-time PSRAM selftest WITH automatic recovery: if the iCE40->PSRAM layer is faulted,
   reflash the iCE40 from its flash bitstream (the proven fix — a full CRESET reconfigure)
   and re-test.  A plain reboot therefore self-heals the iCE40 side, so the cloud
   `psram_recover` command (which just reboots) restores a wedged pod without a physical
   power-cycle.  STM32-side / pad-open faults aren't reflash-fixable, so those stay reported. */
void psram_boot_selftest_with_recovery(void) {
    /* Runs on the hw worker task (boot_deferred_hw_init), after USB is up.  A new board's
       config flash is blank: the iCE40 never configured (CDONE low) and, still hunting its
       flash for a bitstream, it also fails step 1 below, so the step-2 reflash never triggers.
       Check CDONE as well. */
    const bool configured = ice40_is_configured() != 0;
    psram_boot_selftest();
    if (!configured || psram_selftest_result() == PSRAM_ST_ICE40_WRITE_FAIL) {
        printf("[psram-selftest] %s: loading the embedded gateware (image 0)...\n",
               configured ? "iCE40->PSRAM inoperable" : "iCE40 not configured (blank config flash?)");
        if (ice40_reflash_image(0) == 0) {
            sleep_ms(5);
            signal_engine_refresh_version();   /* re-ping the iCE40 + refresh cached version */
            psram_boot_selftest();   /* re-test after the reflash */
        } else {
            printf("[psram-selftest] gateware load failed, run flash-ice40 on the console\n");
        }
    }

    /* A firmware update (flash-self, DFU) leaves the iCE40's config flash alone, so the pod
       keeps its old gateware and new gateware features stay missing until something reprograms
       it. Bring it in line with the images this firmware embeds, keeping the image kind that is
       running. The config flash is written only when the versions differ, so once per update. */
    uint8_t running = signal_engine_fpga_version();
    uint8_t embedded = ice40_embedded_gw_version();
    int deep = (running != 0) && (signal_engine_fpga_features() & FPGA_FEATURE_DEEP_REPLAY);
    int img = boot_policy_gateware_image(running, embedded, deep);
    if (img >= 0) {
        printf("[fpga] gateware v%u, this firmware embeds v%u: reprogramming image %d...\n",
               (unsigned)running, (unsigned)embedded, img);
        if (ice40_reflash_image(img) == 0) {
            sleep_ms(5);
            signal_engine_refresh_version();
            psram_boot_selftest();
            printf("[fpga] now gateware v%u\n", (unsigned)signal_engine_fpga_version());
        } else {
            printf("[fpga] gateware update failed, run flash-ice40 on the console\n");
        }
    }
}

void psram_boot_selftest(void) {
    printf("[psram-selftest] === PSRAM datapath self-test ===\n");
    /* Test 1 needs the STM32 to own the bus (psram_test does raw psram_write/read).
       At idle the bus is released to the iCE40, so acquire around it. */
    psram_bus_acquire();
    int t1 = psram_test();
    psram_bus_release();
    if (t1 != 0) {
        printf("[psram-selftest] 1) STM32<->PSRAM        : FAIL — STM32 can't reach PSRAM "
               "(OCTOSPI/PE4/U7). Stop.\n");
        s_psram_selftest_rc = PSRAM_ST_STM32_FAIL;
        return;
    }
    printf("[psram-selftest] 1) STM32<->PSRAM        : PASS\n");
    int bad = -1;
    if (signal_engine_capture_selftest(16, &bad) == 0) {
        printf("[psram-selftest] 2) iCE40->PSRAM write   : PASS\n"
               "[psram-selftest] === PASS: full PSRAM capture datapath OK ===\n");
        s_psram_selftest_rc = PSRAM_ST_OK;
        return;
    }
    printf("[psram-selftest] 2) iCE40->PSRAM write   : FAIL\n");
    uint8_t lines = 0;
    int reach = signal_engine_psram_bus_reach(&lines);
    if (reach < 0) {
        printf("[psram-selftest] 3) iCE40 bus-reach     : ERR\n");
        s_psram_selftest_rc = PSRAM_ST_ICE40_WRITE_FAIL;
        return;
    }
    static const char *nm[6] = { "/CS ", "SCLK", "IO0 ", "IO1 ", "IO2 ", "IO3 " };
    printf("[psram-selftest] 3) iCE40 bus-reach     : %d/6 lines reach the PSRAM\n", reach);
    for (int b = 0; b < 6; b++) {
        int got = (lines >> b) & 1, exp = (PSRAM_BUS_EXPECT >> b) & 1;
        printf("[psram-selftest]      %s drive=%d read=%d %s\n", nm[b], exp, got,
               got == exp ? "reaches" : "*** OPEN / not reaching ***");
    }
    if (reach == 6) {
        printf("[psram-selftest]    => all iCE40 pads reach the PSRAM; the write fault is the\n"
               "[psram-selftest]       iCE40 write timing/protocol or the PSRAM (U7). Scope during psram_ping.\n"
               "[psram-selftest]       (FIRMWARE FIRST: the board is known-good — see psram_writer.v SCLK note.)\n");
        s_psram_selftest_rc = PSRAM_ST_ICE40_WRITE_FAIL;
    } else {
        printf("[psram-selftest]    => the OPEN line(s) above don't reach the PSRAM. This is rare —\n"
               "[psram-selftest]       the board has tested good repeatedly; re-check gateware pin map\n"
               "[psram-selftest]       and bus_own/OE before reflowing the iCE40 pad. NOT the ADC.\n");
        s_psram_selftest_rc = PSRAM_ST_ICE40_REACH_FAIL;
    }
}

int dac_set_constant(uint8_t value, uint32_t divider) {
    /* Load a 1-byte waveform and start the sequencer with period=1.
       The DAC sequencer will keep clocking that same byte to the DAC chip
       continuously, giving a DC analog output for multimeter probing. */
    if (divider < DAC_MIN_DIVIDER) divider = DAC_MIN_DIVIDER;
    if (divider > 65535) divider = 65535;
    /* DAC8551: one 16-bit sample (value in the high byte), 2 bytes. */
    uint8_t one16[2] = { 0x00, value };
    if (fpga_load_wave(one16, 2) != 0) return -1;
    if (fpga_start_dac(1, divider) != 0) return -1;
    dac_timed = false;
    float dac_clk_hz = (float)DAC_CLK_HZ / (float)divider;
    /* Integer Hz — newlib-nano printf has no %f (the old %.0f printed nothing). */
    printf("[sig] DAC held at 0x%02x (%u), DAC_CLK=%lu Hz (divider=%lu)\n",
           value, value, (unsigned long)lroundf(dac_clk_hz), (unsigned long)divider);
    return 0;
}

int adc_probe_one(uint8_t *out_value) {
    if (!out_value) return -1;

    /* MCP33131D 16-bit: read a short burst through the FPGA→PSRAM path and log the
       16-bit samples. */
    uint16_t s16[16] = {0};
    if (adc_capture_psram(s16, 16, 0.0f) != 0) {
        printf("[sig] ERROR: ADC probe (PSRAM) failed\n");
        return -1;
    }
    *out_value = (uint8_t)(s16[15] >> 8);   /* high byte, for the 8-bit return */
    printf("[sig] ADC probe samples (16-bit): ");
    for (int i = 0; i < 16; i++) printf("%u%s", s16[i], i == 15 ? "\n" : ",");
    return 0;
}

/* ---- LA I/O-bank voltage (TPS2116 mux on PG3, status on PG4) -------------- */

static int s_la_vccio_mv = LA_VCCIO_UNSET;

void la_vccio_init(void) {
    /* GPIOG clock is enabled by signal_engine_init() below (same file). */
    GPIO_InitTypeDef gp = {0};

    /* PG4 = ST: TPS2116 open-drain status output (external pull-up R152 to 3V3);
       read-only input. */
    gp.Mode = GPIO_MODE_INPUT;
    gp.Pull = GPIO_NOPULL;
    gp.Pin  = LA_VCCIO_ST_PIN;
    HAL_GPIO_Init(LA_VCCIO_ST_PORT, &gp);

    /* PG3 = select (PR1): leave as a Hi-Z input at boot, so R151's 100 k pull-up
       to +3V3 decides the power-on state.  On v3 that is VIN1 = +3.3 V, the safe
       default for a 3.3 V DUT; on v2 (MODE grounded) PR1 high is the mux's
       SHUTDOWN state and the bank rail is simply unpowered.  Either way the
       software state stays UNSET, so LA-bank ops are refused until
       la_vccio_set_mv() is called — the host must choose the DUT voltage
       deliberately. */
    gp.Mode = GPIO_MODE_INPUT;
    gp.Pull = GPIO_NOPULL;
    gp.Pin  = LA_VCCIO_SW_PIN;
    HAL_GPIO_Init(LA_VCCIO_SW_PORT, &gp);

    s_la_vccio_mv = LA_VCCIO_UNSET;
}

int la_vccio_set_mv(int mv) {
    if (mv != LA_VCCIO_1V8 && mv != LA_VCCIO_3V3) return -1;

    /* 1.8 V is a rev3 feature.  On v2 the TPS2116's MODE pin is tied to GND, so
       the part has no manual 1.8 V selection at all: MODE low + PR1 low passes
       the HIGHER of the two inputs (always +3V3) and MODE low + PR1 high is
       SHUTDOWN — the bank goes high-Z, not to 1.8 V.  The old firmware called
       that state "1.8 V" and it never was; refuse it rather than silently
       cutting the DUT's I/O rail. */
    if (mv == LA_VCCIO_1V8 && !board_rev_is_v3()) return -2;

    /* The LA1-8 pull-up resistors are tied to +3V3, not to the bank rail, so any
       that are engaged must be released BEFORE the bank drops to 1.8 V — leaving
       one closed would hold the DUT line ~1.5 V above its own rail.  Doing it
       here (rather than trusting the host to remember) means the invariant holds
       for every path that switches the bank, including a mid-session change. */
    if (mv == LA_VCCIO_1V8) la_pullups_all_off();

    /* Drive the select level BEFORE switching PG3 to a push-pull output, so the
       pin never momentarily drives the wrong rail.
       v3 (U8 MODE tied to +3V3 = manual mode, datasheet truth table):
           PR1 high -> VIN1 = +3V3,  PR1 low -> VIN2 = +1V8.
       v2 (MODE tied to GND): PR1 low passes the higher input (+3V3) and PR1
       high is shutdown, so the only reachable level is +3V3 with PR1 LOW — the
       polarity is genuinely inverted between the two boards. */
    GPIO_PinState sel;
    if (board_rev_is_v3())
        sel = (mv == LA_VCCIO_3V3) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    else
        sel = GPIO_PIN_RESET;   /* v2: only +3V3 is reachable, and PR1 must be low */
    HAL_GPIO_WritePin(LA_VCCIO_SW_PORT, LA_VCCIO_SW_PIN, sel);

    GPIO_InitTypeDef gp = {0};
    gp.Mode  = GPIO_MODE_OUTPUT_PP;
    gp.Pull  = GPIO_NOPULL;
    gp.Speed = GPIO_SPEED_FREQ_LOW;
    gp.Pin   = LA_VCCIO_SW_PIN;
    HAL_GPIO_Init(LA_VCCIO_SW_PORT, &gp);

    s_la_vccio_mv = mv;
    HAL_Delay(2);   /* let the VCCIO rail settle before it is used */
    return 0;
}

int la_vccio_get_mv(void) { return s_la_vccio_mv; }

int la_vccio_status_pin(void) {
    return (HAL_GPIO_ReadPin(LA_VCCIO_ST_PORT, LA_VCCIO_ST_PIN) == GPIO_PIN_SET) ? 1 : 0;
}

int la_vccio_readback_mv(void) {
    if (!board_rev_is_v3()) return 0;   /* v2 ST is ambiguous (low = 1V8 OR shutdown) */
    return la_vccio_status_pin() ? LA_VCCIO_3V3 : LA_VCCIO_1V8;
}

/* ---- Initialisation ------------------------------------------------------ */

/* BOOT ORDER / shared-bus handshake: the iCE40 auto-configures at power-up by
   reading its config flash over the SHARED OCTOSPI bus (~100-300 ms).  main()
   must NOT let psram_init() drive that bus until the iCE40 is done, or they
   collide and the iCE40 comes up unable to write captures to PSRAM.  The SPI1
   link to the iCE40 is a SEPARATE bus, so we WAIT here by pinging over SPI1 until
   the gateware answers (== configured, running, bus_own pad-gating active) —
   holding off the shared bus for free.  ~60 * 50 ms = up to ~3 s (only if slow;
   a ready iCE40 answers on attempt 1).  main() calls psram_init() only after
   signal_engine_init() returns. */
#define INIT_PING_ATTEMPTS  60

void signal_engine_init(void) {
    GPIO_InitTypeDef gp = {0};

    /* GPIO clocks for SPI1 (PA5/PA6/PB5), CS (PB10), BUSY (PG1). */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    /* LA I/O-bank voltage mux (PG3 select, PG4 status): configure the pins and
       start in the "unset" state so LA-bank ops require an explicit voltage. */
    la_vccio_init();
    /* Select SPI1's KERNEL clock (the one that generates SCK).  Only the APB
       register clock was enabled before, and every other peripheral sets its
       source but SPI1 didn't.  SPI1SEL defaults to PLL1Q, but nothing guarantees
       the PLL1Q *output* is enabled (SystemClock_Config sets PLLQ=2 but the
       output-enable is separate), so an unclocked SPI1 emits no SCK and the
       iCE40 oversampling slave returns 0x00.  Enable PLL1Q (=250 MHz, matches
       the /256 -> ~1 MHz SCK) and point SPI1 at it explicitly. */
    __HAL_RCC_PLL1_CLKOUT_ENABLE(RCC_PLL1_DIVQ);
    {
        RCC_PeriphCLKInitTypeDef pclk = {0};
        pclk.PeriphClockSelection = RCC_PERIPHCLK_SPI1;
        pclk.Spi1ClockSelection   = RCC_SPI1CLKSOURCE_PLL1Q;
        HAL_RCCEx_PeriphCLKConfig(&pclk);
    }
    __HAL_RCC_SPI1_CLK_ENABLE();

    /* SCK/MISO/MOSI as SPI1 alternate function.  VERY_HIGH slew so SCK/MOSI edges
       stay clean as the SPI clock is raised (the FPGA slave samples those edges). */
    gp.Mode = GPIO_MODE_AF_PP;
    gp.Pull = GPIO_NOPULL;
    gp.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gp.Alternate = ICE_SPI_AF;
    gp.Pin = ICE_SPI_SCK_PIN;   HAL_GPIO_Init(ICE_SPI_SCK_PORT, &gp);
    gp.Pin = ICE_SPI_MISO_PIN;  HAL_GPIO_Init(ICE_SPI_MISO_PORT, &gp);
    gp.Pin = ICE_SPI_MOSI_PIN;  HAL_GPIO_Init(ICE_SPI_MOSI_PORT, &gp);

    /* CS as GPIO output, deasserted (high) during init. */
    HAL_GPIO_WritePin(ICE_SPI_CS_PORT, ICE_SPI_CS_PIN, GPIO_PIN_SET);
    gp.Mode = GPIO_MODE_OUTPUT_PP;
    gp.Pull = GPIO_NOPULL;
    gp.Alternate = 0;
    gp.Pin = ICE_SPI_CS_PIN;    HAL_GPIO_Init(ICE_SPI_CS_PORT, &gp);

    /* BUSY: input with rising-edge EXTI configured but the NVIC line left
       DISABLED at idle.  BUSY is not a guaranteed-stable level when idle (the
       FPGA drives it), so an always-armed edge IRQ would storm the CPU.  We
       arm the NVIC only for the duration of an async capture. */
    gp.Mode = GPIO_MODE_IT_RISING;
    gp.Pull = GPIO_PULLUP;
    gp.Pin = ICE_BUSY_PIN;      HAL_GPIO_Init(ICE_BUSY_PORT, &gp);
    HAL_NVIC_SetPriority(ICE_BUSY_EXTI_IRQn,
                         configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
    HAL_NVIC_DisableIRQ(ICE_BUSY_EXTI_IRQn);

    /* SPI1 master, mode 0 (CPOL=0/CPHA=0), 8-bit, MSB-first, software NSS,
       ~1.95 MHz (kernel /128) — the iCE40 24 MHz control-clock's reliable max. */
    hspi_ice.Instance = ICE_SPI;
    hspi_ice.Init.Mode = SPI_MODE_MASTER;
    hspi_ice.Init.Direction = SPI_DIRECTION_2LINES;
    hspi_ice.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi_ice.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi_ice.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi_ice.Init.NSS = SPI_NSS_SOFT;
    hspi_ice.Init.BaudRatePrescaler = SPI_ICE_PRESCALER;
    hspi_ice.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi_ice.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi_ice.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi_ice.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    hspi_ice.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
    if (HAL_SPI_Init(&hspi_ice) != HAL_OK) {
        printf("[sig] ERROR: SPI1 init failed\n");
        return;
    }

    printf("[sig] SPI1 ~%lu Hz  SCK=PA5 MOSI=PB5 MISO=PA6 CS=PB10  BUSY=PG1\n",
           (unsigned long)SPI_CLOCK_HZ);

#if ICE_SPI_USE_DMA
    /* GPDMA for the big capture read-backs / waveform loads.  SPI1 DMA completes
       through the SPI EOT interrupt (the GPDMA channel IRQ only arms it), so the
       SPI1 NVIC line must be enabled and routed to HAL_SPI_IRQHandler too. */
    if (dma_wait_channel_init(&hdma_ice_rx, GPDMA1_Channel0, GPDMA1_REQUEST_SPI1_RX,
                              DMA_PERIPH_TO_MEMORY, GPDMA1_Channel0_IRQn) == 0 &&
        dma_wait_channel_init(&hdma_ice_tx, GPDMA1_Channel1, GPDMA1_REQUEST_SPI1_TX,
                              DMA_MEMORY_TO_PERIPH, GPDMA1_Channel1_IRQn) == 0) {
        __HAL_LINKDMA(&hspi_ice, hdmarx, hdma_ice_rx);
        __HAL_LINKDMA(&hspi_ice, hdmatx, hdma_ice_tx);
        dma_wait_setup(&s_ice_waiter, ICE_SPI);
        HAL_NVIC_SetPriority(SPI1_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
        HAL_NVIC_EnableIRQ(SPI1_IRQn);
        s_ice_dma_ok = true;
        printf("[sig] SPI1 DMA enabled (GPDMA1 ch0=rx ch1=tx)\n");
    } else {
        printf("[sig] SPI1 DMA init failed — using blocking block transfers\n");
    }
#endif

    /* Probe the FPGA — does NOT halt on failure; DAC/ADC commands will fail
       later if the link is down, but the console/Ethernet paths still come up. */
    if (fpga_ping(INIT_PING_ATTEMPTS, &s_fpga_version) != 0) {
        printf("[sig] continuing without FPGA — DAC/ADC commands will fail.\n");
        printf("[sig] try 'spi-ping' on the console once you fix wiring.\n");
    } else {
        s_fpga_features = signal_engine_fpga_features();   /* which warmboot image booted */
        printf("[sig] FPGA gateware v%u (v2/PSRAM capture path), features 0x%02x\n",
               s_fpga_version, s_fpga_features);
    }
}

/* ---- Poll (main loop) ---------------------------------------------------- */

/* Runs in the console task.  Its SPI1 accesses (DAC-expiry stop, async-capture
   read-back) share the iCE40 bus with the net task, so they MUST hold hw_lock —
   otherwise a net-task transaction can interleave mid-transfer and corrupt the
   bus.  This matters more now that a DMA read-back yields the CPU (blocks on the
   completion semaphore) for its whole duration, guaranteeing the net task runs
   during it.  hw_lock is taken only around the actual bus ops (not the whole
   poll) to keep the hold time short; the callback just sets a flag, so it runs
   outside the lock. */
void signal_engine_poll(void) {
    /* DAC duration expiry (RP-side timer, same as old impl). */
    if (dac_timed && time_reached(dac_stop_at)) {
        dac_timed = false;
        hw_lock();
        fpga_stop_dac();
        hw_unlock();
        printf("[sig] DAC stopped (duration expired)\n");
    }
}

/* ---- DAC waveform generators --------------------------------------------- */

static void log_dac_start(const char *shape, float freq, uint8_t amplitude,
                          uint8_t offset, uint32_t duration_ms, uint32_t n_samples,
                          uint32_t divider) {
    /* Report the TRUE update rate (HFOSC/(divider+K)) and the frequency actually
       played (rate/period), so the log reflects what the DAC emits, not the naive
       HFOSC/divider that overstates it. */
    float eff_rate = dac_effective_rate_hz(divider);
    float actual_freq = (n_samples > 0) ? eff_rate / (float)n_samples : 0.0f;
    /* Integer units — newlib-nano printf has no %f. Waveform freq can be sub-Hz,
       so milli-Hz; the sample rate is always large, so integer S/s. */
    printf("[sig] DAC %s  freq=%lu mHz (actual ~%lu mHz)  amp=%u  offset=%u  %s "
           "(%lu samples/period, divider=%lu, fs=%lu S/s)\n",
           shape, (unsigned long)lroundf(freq * 1000.0f),
           (unsigned long)lroundf(actual_freq * 1000.0f), amplitude, offset,
           duration_ms > 0 ? "timed" : "continuous",
           (unsigned long)n_samples, (unsigned long)divider,
           (unsigned long)lroundf(eff_rate));
}

static int dac_generate_common(const char *shape,
                               void (*fill)(uint32_t, uint8_t, uint8_t),
                               float freq, uint8_t amplitude, uint8_t offset,
                               uint32_t duration_ms, float sample_rate_hz) {
    if (freq <= 0.0f || amplitude == 0) {
        /* Integer milli-Hz — newlib-nano printf has no %f. Signed: freq is <= 0
           here (that's why it's invalid). */
        printf("[sig] ERROR: %s invalid params (freq=%ld mHz amp=%u)\n",
               shape, (long)lroundf(freq * 1000.0f), amplitude);
        return -1;
    }
    uint32_t period  = 0;
    uint32_t divider = compute_divider_and_period(freq, sample_rate_hz, &period);
    if (divider == 0 || period == 0) return -1;

    fill(period, amplitude, offset);
    log_dac_samples(period);

    /* DAC8551 is 16-bit: expand each 8-bit waveform sample into the high byte of a
       16-bit value and load as little-endian byte pairs (2 bytes per sample; the
       dac8551_engine reads two bytes per sample). */
    for (uint32_t i = 0; i < period; i++)
        sample_buf16[i] = (uint16_t)((uint16_t)sample_buf[i] << 8);
    if (fpga_load_wave((const uint8_t *)sample_buf16, period * 2u) != 0) return -1;
    if (fpga_start_dac(period, divider) != 0) return -1;

    log_dac_start(shape, freq, amplitude, offset, duration_ms, period, divider);

    if (duration_ms > 0) {
        dac_stop_at = make_timeout_time_ms(duration_ms);
        dac_timed   = true;
    } else {
        dac_timed   = false;
    }
    return 0;
}

int dac_generate_sine(float freq, uint8_t amplitude, uint8_t offset,
                      uint32_t duration_ms, float sample_rate_hz) {
    return dac_generate_common("sine", fill_sine, freq, amplitude, offset,
                               duration_ms, sample_rate_hz);
}

int dac_generate_square(float freq, uint8_t amplitude, uint8_t offset,
                        uint32_t duration_ms, float sample_rate_hz) {
    return dac_generate_common("square", fill_square, freq, amplitude, offset,
                               duration_ms, sample_rate_hz);
}

int dac_generate_sawtooth(float freq, uint8_t amplitude, uint8_t offset,
                          uint32_t duration_ms, float sample_rate_hz) {
    return dac_generate_common("sawtooth", fill_sawtooth, freq, amplitude, offset,
                               duration_ms, sample_rate_hz);
}

int dac_generate_arbitrary_rate(const uint8_t *data, size_t len, bool loop,
                                float sample_rate_hz) {
    if (!data || len == 0 || len > SIGNAL_BUF_SIZE) {
        printf("[sig] ERROR: arbitrary invalid params (len=%u)\n", (unsigned)len);
        return -1;
    }
    /* loop=true is the normal mode for the FPGA sequencer; loop=false would
       require a one-shot variant which we don't expose yet.  For now, run
       continuously and rely on duration / dac_stop. */
    (void)loop;

    uint32_t divider = dac_divider_from_rate_hz(sample_rate_hz);
    float    actual  = (float)DAC_CLK_HZ / (float)divider;
    /* Integer S/s — newlib-nano printf has no %f (the old %.3f MS/s printed nothing). */
    printf("[sig] DAC arbitrary  len=%u  loop=%s  %lu S/s (divider=%lu)\n",
           (unsigned)len, loop ? "yes" : "no",
           (unsigned long)lroundf(actual), (unsigned long)divider);

    if (fpga_load_wave(data, len)        != 0) return -1;
    /* DAC8551: `data` is 16-bit samples as byte pairs, so the sequencer period is
       len/2 samples. */
    uint32_t period = (uint32_t)(len / 2u);
    if (fpga_start_dac(period, divider)  != 0) return -1;
    dac_timed = false;
    return 0;
}

int dac_generate_arbitrary(const uint8_t *data, size_t len, bool loop) {
    return dac_generate_arbitrary_rate(data, len, loop, 0.0f);
}

/* ---- deep-DAC PSRAM region bookkeeping (top-anchored, >=v18) ----------------
 * s_dac_ps_base is the byte base of the staged waveform (TOP - total_bytes on v18,
 * or 0 = legacy overlay on older gateware).  s_dac_ps_active marks the region as
 * OCCUPIED (a replay is armed/looping), which shrinks the concurrent LA/ADC caps so a
 * capture cannot overrun into the waveform.  Occupancy is only asserted on >=v18 (older
 * gateware has no arbiter, so capture and replay never run together anyway). */
static uint32_t s_dac_ps_base   = FPGA_PSRAM_DAC_TOP;   /* = TOP => zero-length region */
static bool     s_dac_ps_active = false;

/* Runtime PSRAM capture region bases for dynamic tri-capture zone allocation
 * (SET_CAPTURE_BASES 0x32, gateware >= v22).  Default to the legacy fixed map so an
 * un-set capture behaves exactly as before.  LA stays at 0 under flexible packing; only
 * the ADC base actually moves (see psram_alloc.c / docs/tri-capture-unified-psram.md). */
static uint32_t s_la_cap_base  = FPGA_PSRAM_LA_BASE;
static uint32_t s_adc_cap_base = FPGA_PSRAM_ADC_BASE;

/* Latch the runtime capture bases into the iCE40 (0x32) and mirror them for read-back.
 * Send BEFORE arming CAPTURE.  No-op (keeps the fixed map) on gateware < v22. */
int fpga_set_capture_bases(uint32_t la_base, uint32_t adc_base) {
    if (s_fpga_version >= CAPTURE_BASES_MIN_GW) {
        uint8_t p[6] = { (uint8_t)la_base, (uint8_t)(la_base >> 8), (uint8_t)(la_base >> 16),
                         (uint8_t)adc_base, (uint8_t)(adc_base >> 8), (uint8_t)(adc_base >> 16) };
        if (spi_cmd_write(CMD_SET_CAPTURE_BASES, p, sizeof(p)) != 0) return -1;
    }
    s_la_cap_base  = la_base;
    s_adc_cap_base = adc_base;
    return 0;
}
uint32_t signal_engine_adc_cap_base(void) { return s_adc_cap_base; }

/* Re-sync every FIRMWARE MIRROR of a gateware register to the values the fabric actually holds
 * after a RECONFIGURATION (image swap, console flash-ice40, boot/OTA reflash).  Call it from the
 * one place every reconfiguration goes through, ice40_reflash_image().
 *
 * Why this exists (2026-07-30).  Reconfiguring the iCE40 resets its registers to their power-on
 * values, but the firmware's mirrors survived — so firmware and fabric silently disagreed about
 * where a capture lives.  `capture_dual` calls fpga_set_capture_bases() with a PACKED base (ADC
 * right after LA, ~0x4000); the fabric resets that register to 0x400000.  Afterwards the STM32
 * stamped its no-write sentinel at the packed base and read back from the packed base, while the
 * iCE40 dutifully wrote at 0x400000 — the sentinel survived, and the sentinel check reports
 * exactly one thing: "iCE40 did NOT write the capture to PSRAM".  That surfaced as
 * `psram: "inoperable (iCE40 write)"` and an image swap refusing with "run psram_recover".
 *
 * So the recurring "PSRAM wedge after an image swap" was NEVER a wedge: the bus, the writer and
 * the chip were all healthy the whole time (STM32-side pattern + full 8 MB address tests pass
 * while "wedged"). It was a firmware/fabric state desync, which is why only a REBOOT cleared it
 * — that is what reset the mirror. It looked non-deterministic because it needed a capture_dual
 * (the only verb that moves the base) before the swap; a plain `capture` never triggered it. */
void signal_engine_on_gateware_reconfigured(void) {
    s_la_cap_base  = FPGA_PSRAM_LA_BASE;    /* fabric: adc_cap_base <= 24'h400000 on reset */
    s_adc_cap_base = FPGA_PSRAM_ADC_BASE;
    /* A reconfiguration also destroys any deep-DAC replay, so its top-of-PSRAM reservation is
     * stale — leaving it claimed would shrink the next capture's budget for no reason. */
    s_dac_ps_active = false;
    s_dac_ps_base   = FPGA_PSRAM_DAC_TOP;
}

/* Bytes a resident deep-DAC replay currently occupies (top-anchored), else 0 — the
 * capacity term the tri-capture allocator must reserve for the DAC. */
uint32_t signal_engine_dac_resident_bytes(void) {
    return s_dac_ps_active ? (FPGA_PSRAM_TOTAL - s_dac_ps_base) : 0u;
}

/* CLOSED-LOOP DAC (gateware >= v23): load the curve into the DAC BRAM (LOAD_WAVE, 16-bit
   LE — indexed in-fabric by adc>>5), then arm the loop.  See dac_loop.v / docs. */
int dac_control_loop_start(const uint8_t *curve, size_t curve_len,
                           uint16_t k_q15, uint16_t vmin, uint16_t vmax, uint16_t tick_div) {
    if (s_fpga_version < DAC_CONTROL_LOOP_MIN_GW) return -2;
    /* The running IMAGE must actually carry the loop engine — the deep-replay image reports
       the same gateware version but ties the loop off, so arming there would "succeed" and
       drive nothing.  Gated here (not just in the command handler) so every caller — cloud
       command, console, SDK — gets the same refusal. */
    if (!signal_engine_has_control_loop()) return -3;
    if (curve && curve_len) { if (fpga_load_wave(curve, curve_len) != 0) return -1; }
    uint8_t args[8] = {
        (uint8_t)k_q15,    (uint8_t)(k_q15    >> 8),
        (uint8_t)vmin,     (uint8_t)(vmin     >> 8),
        (uint8_t)vmax,     (uint8_t)(vmax     >> 8),
        (uint8_t)tick_div, (uint8_t)(tick_div >> 8),
    };
    spi_cmd_write(CMD_START_DAC_LOOP, args, sizeof(args));
    dac_timed = false;
    return 0;
}

uint16_t fpga_dac_loop_probe(void) {
    uint8_t b[2] = { 0, 0 };
    spi_cmd_read(CMD_DAC_PROBE, NULL, 0, b, 2);
    return (uint16_t)(b[0] | (b[1] << 8));
}

/* LOOP INPUT SOURCE (gateware >= v29): pick the ADC (closed loop), a fixed host value or the
   internal sweep.  Same image gate as arming — on the deep-replay image the loop registers
   drive nothing, so a "source set" there would be a lie the caller could not detect. */
int dac_loop_set_source(uint8_t src, uint16_t in_fixed, uint16_t sweep_step) {
    if (s_fpga_version < DAC_LOOP_SOURCE_MIN_GW) return -2;
    if (!signal_engine_has_control_loop()) return -3;
    uint8_t args[5] = {
        (uint8_t)(src & 0x03u),
        (uint8_t)in_fixed,   (uint8_t)(in_fixed   >> 8),
        (uint8_t)sweep_step, (uint8_t)(sweep_step >> 8),
    };
    if (spi_cmd_write(CMD_DAC_LOOP_SRC, args, sizeof(args)) != 0) return -1;
    return 0;
}

/* LOOP INPUT MAP (gateware >= v30): the affine map from the raw input count onto the curve
   index, plus the over-range trip.  Same image gate as arming, for the same reason — the
   deep-replay image has no loop for the map to apply to.  The registers are derived from the
   bench description in dac_loop_params.c (which is where cal_data.h lives), so callers send
   engineering units and never carry a copy of this board's ADC calibration. */
int dac_loop_set_inmap(uint16_t in_zero, int16_t in_gain, uint16_t in_trip,
                       bool map_en, bool trip_en) {
    if (s_fpga_version < DAC_LOOP_INMAP_MIN_GW) return -2;
    if (!signal_engine_has_control_loop()) return -3;
    uint16_t g = (uint16_t)in_gain;
    uint8_t args[7] = {
        (uint8_t)in_zero, (uint8_t)(in_zero >> 8),
        (uint8_t)g,       (uint8_t)(g >> 8),
        (uint8_t)in_trip, (uint8_t)(in_trip >> 8),
        (uint8_t)((map_en ? 0x01u : 0u) | (trip_en ? 0x02u : 0u)),
    };
    if (spi_cmd_write(CMD_DAC_LOOP_INMAP, args, sizeof(args)) != 0) return -1;
    return 0;
}

/* Has the control loop LATCHED its over-range trip?  STATUS bit 6 (gateware >= v30): the
   loop parks its output at vmin and stays there until disarmed, so a caller that does not
   read this sees a loop that is "running" and holding a rail for no visible reason. */
bool fpga_dac_loop_tripped(void) {
    if (s_fpga_version < DAC_LOOP_INMAP_MIN_GW) return false;
    uint8_t st = 0;
    if (fpga_status_read(&st) != 0) return false;
    return (st & STATUS_LOOP_TRIPPED) != 0u;
}

/* The input the loop's last tick used (DAC_LOOP_IN_PROBE).  Pre-v29 gateware has no such
   opcode (it would answer with whatever the default state leaves on MISO), so don't ask. */
uint16_t fpga_dac_loop_input(void) {
    if (s_fpga_version < DAC_LOOP_SOURCE_MIN_GW) return 0;
    uint8_t b[2] = { 0, 0 };
    spi_cmd_read(CMD_DAC_LOOP_IN_PROBE, NULL, 0, b, 2);
    return (uint16_t)(b[0] | (b[1] << 8));
}

/* Does the RUNNING image carry the closed-loop engine?  See signal_engine.h: the loop and
   deep-replay images share a GATEWARE_VERSION, so only the FEATURES byte distinguishes them.
   A pre-v22 gateware has no FPGA_FEATURES opcode and no loop engine either, so the cached
   0 it falls back to is the right answer. */
bool signal_engine_has_control_loop(void) {
    return (signal_engine_fpga_features() & FPGA_FEATURE_CONTROL_LOOP) != 0;
}

/* Which optional block the running warmboot image carries (FPGA_FEATURES 0x18). */
uint8_t signal_engine_fpga_features(void) {
    if (s_fpga_version >= CAPTURE_BASES_MIN_GW) {   /* >=v22 has the opcode */
        uint8_t f = 0;
        if (spi_cmd_read(CMD_FPGA_FEATURES, NULL, 0, &f, 1) == 0) { s_fpga_features = f; return f; }
    }
    return s_fpga_features;
}

/* Gateware-derived feature flags — the SINGLE place these are computed.  Both the cloud
   `capabilities` announce (cloud_client.c) and the `status` reply's caps[] (command_handler.c)
   call this, so a direct LAN/serial client and a cloud client see the same feature set.
   See signal_engine_caps_t for why that used to be two divergent lists. */
void signal_engine_caps(signal_engine_caps_t *out) {
    if (!out) return;
    uint8_t ver   = signal_engine_fpga_version();
    uint8_t feats = signal_engine_fpga_features();

    /* >=v23 carries per-image FEATURE bits (an image swap trades closed-loop for deep-replay),
       so advertise from the RUNNING image; older gateware is version-gated only. */
    out->deep_replay = (ver >= DAC_CONTROL_LOOP_MIN_GW)
                           ? ((feats & FPGA_FEATURE_DEEP_REPLAY) != 0)
                           : (ver >= DAC_DEEP_REPLAY_MIN_GW);
    out->control_loop = (ver >= DAC_CONTROL_LOOP_MIN_GW) &&
                        ((feats & FPGA_FEATURE_CONTROL_LOOP) != 0);
    /* Hardware co-triggered DAC start (>=v27): the iCE40 fires the DAC on the capture's t0, so
       DAC sample 0 is phase-locked to the capture instead of free-running. */
    out->cotrig = ver >= DAC_COTRIG_MIN_GW;
    /* loop_sources / loop_input_map are gated on the GATEWARE GENERATION, not the running
       image: both images are built from the same tree, so >=v29 / >=v30 means this pod has
       them — and a flag that survives an image swap must not blink off for the seconds between
       the swap and the pod's re-announce.  Whether the loop RUNS stays gated on control_loop. */
    out->loop_sources   = ver >= DAC_LOOP_SOURCE_MIN_GW;
    out->loop_input_map = ver >= DAC_LOOP_INMAP_MIN_GW;
    out->gpio_read       = ver >= GPIO_GET_MIN_GW;
    out->capture_trigger = ver >= CAPTURE_TRIGGER_MIN_GW;
}

/* Switch the running gateware IMAGE (0 = closed-loop, 1 = deep-DAC-replay).
   SB_WARMBOOT cannot reconfigure at runtime on this board (its config-SPI pins are the
   shared PSRAM bus, so the reconfig can't read the flash — confirmed: even a warmboot to
   the running image fails while it cold-boots fine).  So we reprogram the config flash with
   the selected image over the PROVEN ice40_flash_program + CRESET-reconfig path (~2 s), then
   re-read version + features.  The gateware SB_WARMBOOT infra stays in the tree for a future
   board where the config flash isn't on the runtime bus. */
/* Bring the gateware's PSRAM masters to a safe, IDLE boundary BEFORE an image swap grabs the
   shared bus.  ice40_reflash_image() acquires the bus (psram_reset_to_spi -> psram_bus_acquire,
   PG0/bus_own high) and RESETS the PSRAM to re-enter QPI.  The gw v26/v28 reset-gating keeps the
   deep reader+arbiter clean across that yank ONLY IF they are already idle — top_v2.v states the
   invariant plainly: "The STM only takes the bus while these masters are idle ... never mid-burst".
   heavy_begin() blocks a concurrent CAPTURE, but a fire-and-forget DAC replay / closed loop is NOT
   tracked by the heavy gate, so under load (mid-suite, post-replay) a reader can be mid-burst when
   the swap arrives.  That violates the invariant and wedges the arbiter/PSRAM until a power cycle —
   the recurring "swap wedges in either direction under load".  Stopping the DAC and draining to
   idle here closes the window in FIRMWARE, independent of which image is running or switched to.
   Shared by the image swap, the console flash-ice40 reflash, and OTA commit (all grab the bus at
   runtime and can race a live master); declared in signal_engine.h. */
void signal_engine_quiesce_psram_masters(void) {
    /* 1. Halt the deep-replay reader AND the closed loop (STOP_DAC drops dac_psram_run / the DAC
          engine; also releases the resident top-of-PSRAM deep-DAC reservation). */
    dac_stop();

    /* 2. Wait for the gateware to report no DAC running and no capture busy, and for the writer to
          have drained (BUSY high = idle) — i.e. every PSRAM master back in its CS-high S_IDLE state,
          the safe boundary the reset-gating assumes.  Bounded: a burst is ~1 us and a looping reader
          quiesces within a few sample periods of STOP_DAC, so a few ms is ample; if a genuinely stuck
          FPGA never idles we fall through to the reflash, which hard-resets it anyway. */
    uint32_t t0 = HAL_GetTick();
    for (;;) {
        uint8_t st = 0;
        bool idle = (fpga_status_read(&st) == 0) &&
                    !(st & (STATUS_DAC_RUN | STATUS_CAP_BUSY)) &&
                    busy_read();
        if (idle) break;
        if (HAL_GetTick() - t0 > 25u) {
            printf("[sig] image swap: masters not idle after 25 ms (st=0x%02x), reflashing anyway\n", st);
            break;
        }
        sleep_ms(1);
    }
}

int fpga_warmboot(uint8_t image) {
    if (image > 1) return -1;
    /* Quiesce the running gateware so the swap takes the bus at a safe (masters-idle) boundary —
       this is what makes the swap robust under load instead of wedging the PSRAM (see the helper). */
    signal_engine_quiesce_psram_masters();
    if (ice40_reflash_image((int)image) != 0) {
        printf("[sig] image switch: reflash failed\n");
        fpga_ping(INIT_PING_ATTEMPTS, &s_fpga_version);   /* refresh whatever is running */
        return -3;
    }
    sleep_ms(5);
    fpga_ping(INIT_PING_ATTEMPTS, &s_fpga_version);
    s_fpga_features = signal_engine_fpga_features();

    /* ★ fix 3: VERIFY the iCE40->PSRAM write path actually came up on the swapped-to image (a
       16-sample capture selftest: the STM32 pre-writes a sentinel, the iCE40 must overwrite it).
       With the gw v26 deep-master reset-gating this should always pass.  If it ever wedges, do
       NOT re-reflash the same image — re-running the swap re-triggers the race and COMPOUNDS the
       wedge into a persistent, power-cycle-only state.  Instead recover the bench to the immune
       LOOP image (a single, race-free reconfig) and report the failure, so the pod is left
       usable rather than dead. */
    int bad = -1;
    if (signal_engine_capture_selftest(16, &bad) == 0) {
        printf("[sig] image %u: gateware v%u, features 0x%02x (PSRAM write OK)\n",
               image, s_fpga_version, s_fpga_features);
        return 0;
    }
    printf("[sig] image %u: iCE40->PSRAM write wedged after swap\n", image);
    if (image != 0) {
        printf("[sig] recovering the bench to the loop image (image 0)\n");
        if (ice40_reflash_image(0) == 0) {
            sleep_ms(5);
            fpga_ping(INIT_PING_ATTEMPTS, &s_fpga_version);
            s_fpga_features = signal_engine_fpga_features();
        }
    }
    return -5;
}

void signal_engine_dac_psram_stage(uint32_t total_bytes) {
    s_dac_ps_base = (s_fpga_version >= CONCURRENT_PSRAM_MIN_GW)
                        ? FPGA_DAC_BASE_FOR(total_bytes)   /* top-anchored */
                        : FPGA_PSRAM_ADC_BASE;             /* legacy: overlay ADC region (0) */
}
uint32_t signal_engine_dac_psram_base(void) { return s_dac_ps_base; }
void     signal_engine_dac_psram_release(void) { s_dac_ps_active = false; }

/* LA ceiling = whichever comes first: the ADC region (dual ADC+LA), the resident DAC
   base, or the top of memory.  Samples are 2 bytes each. */
uint32_t signal_engine_la_max_samples(bool adc_in_capture) {
    uint32_t ceil = adc_in_capture ? FPGA_PSRAM_ADC_BASE : FPGA_PSRAM_TOTAL;
    if (s_dac_ps_active && s_dac_ps_base < ceil) ceil = s_dac_ps_base;
    if (ceil <= FPGA_PSRAM_LA_BASE) return 0;
    return (ceil - FPGA_PSRAM_LA_BASE) / 2u;
}
/* ADC ceiling = resident DAC base (if any) else top of memory; ADC starts at ADC_BASE. */
uint32_t signal_engine_adc_max_samples(void) {
    uint32_t ceil = FPGA_PSRAM_TOTAL;
    if (s_dac_ps_active && s_dac_ps_base < ceil) ceil = s_dac_ps_base;
    if (ceil <= FPGA_PSRAM_ADC_BASE) return 0;
    return (ceil - FPGA_PSRAM_ADC_BASE) / 2u;
}

/* Deep replay: play `count` 16-bit samples ALREADY STAGED in PSRAM at the top-anchored
   base straight out the DAC8551, looping until dac_stop.  Unlike
   dac_generate_arbitrary_rate (which loads the 4 KB FPGA BRAM, <=2048 samples),
   the iCE40 streams from PSRAM so `count` can span the whole 8 MB region.  The
   caller must have staged the bytes into PSRAM and released the shared bus
   (psram_bus_release) before calling this — the iCE40 needs to own the bus to read.
   Requires gateware >= v17 (CMD_START_DAC_PSRAM). */
int dac_replay_psram(uint32_t count, float sample_rate_hz) {
    if (count == 0 || count > FPGA_DAC_REPLAY_MAX_SAMPLES) {
        printf("[sig] ERROR: psram replay count=%lu out of range (max %lu)\n",
               (unsigned long)count, (unsigned long)FPGA_DAC_REPLAY_MAX_SAMPLES);
        return -1;
    }
    uint32_t divider = dac_divider_from_rate_hz(sample_rate_hz);
    float    actual  = (float)DAC_CLK_HZ / (float)divider;
    /* Integer S/s — newlib-nano printf has no %f (the old %.3f MS/s printed nothing). */
    printf("[sig] DAC psram replay  count=%lu  %lu S/s (divider=%lu)\n",
           (unsigned long)count, (unsigned long)lroundf(actual), (unsigned long)divider);
    if (fpga_start_dac_psram(signal_engine_dac_psram_base(), count, divider) != 0) return -1;
    /* Mark the top region occupied so a concurrent LA/ADC capture is capped below it
       (>=v18 only; older gateware serialises replay and capture). */
    s_dac_ps_active = (s_fpga_version >= CONCURRENT_PSRAM_MIN_GW);
    dac_timed = false;
    return 0;
}

/* Connected iCE40 gateware version (from CMD_VERSION at init).  Deep DAC replay
   needs >= 17; callers gate capability advertisement / clamps on this. */
uint8_t signal_engine_fpga_version(void) { return s_fpga_version; }

/* Live-read the gateware version over SPI and refresh the cached value, so `status`
   (and a re-sent capabilities frame) reflect a runtime `flash-ice40` instead of the
   boot-time snapshot.  Quiet (no [sig] diagnostics, unlike fpga_ping) — returns true
   iff the FPGA answered PING; on failure the cache is left untouched. */
bool signal_engine_refresh_version(void) {
    /* Holds hw_lock: SPI1 to the iCE40 is shared by the worker, console and net tasks
       (this is called from the console `status` path AND cl_send_capabilities on the
       net task), so the transaction must be serialised or it corrupts an in-flight
       transfer — same rule as signal_engine_poll(). */
    hw_lock();
    uint8_t ping = 0;
    spi_cmd_read(CMD_PING, NULL, 0, &ping, 1);
    bool ok = (ping == PING_REPLY_MAGIC);
    if (ok) {
        uint8_t v = 0;
        spi_cmd_read(CMD_VERSION, NULL, 0, &v, 1);
        s_fpga_version = v;
    }
    hw_unlock();
    return ok;
}

void dac_stop(void) {
    dac_timed = false;
    /* Free the top-of-PSRAM deep-DAC region so a subsequent LA/ADC capture reclaims
       the full depth (no-op if no deep replay was resident). */
    signal_engine_dac_psram_release();
    fpga_stop_dac();
    printf("[sig] DAC stopped\n");
}

void fpga_set_dac_stop_after_us(uint32_t us) {
    /* Only the >=v21 gateware decodes CMD_SET_DAC_STOP_AFTER; on older gateware it would be an
       unknown opcode, so skip it entirely (the feature is simply unavailable there).  On capable
       gateware ALWAYS send it — 0 disarms — so a stale threshold from a previous capture can't
       fire on this one.  cycles = us * 24 MHz clk; clamp to the 32-bit counter width. */
    if (s_fpga_version < DAC_CAPTURE_STOP_MIN_GW) return;
    uint64_t cycles = (uint64_t)us * (FPGA_HFOSC_HZ / 1000000u);
    if (cycles > 0xFFFFFFFFu) cycles = 0xFFFFFFFFu;
    fpga_set_dac_stop_after((uint32_t)cycles);
}

bool fpga_dac_arm_on_capture(void) {
    /* Only >=v27 decodes CMD_DAC_ARM_ON_CAPTURE; on older gateware it's an unknown opcode, so
       report "can't co-trigger" and let the caller fall back to a plain sequential start.  0 args:
       the iCE40 sets dac_pend and defers the NEXT DAC start opcode to the next capture arm. */
    if (s_fpga_version < DAC_COTRIG_MIN_GW) return false;
    spi_cmd_write(CMD_DAC_ARM_ON_CAPTURE, NULL, 0);
    s_cotrig_pending = true;
    return true;
}

/* ---- ADC capture / measure into PSRAM -----------------------------------
   The iCE40 captures `samples` 16-bit samples at `sample_rate_hz` straight into
   the shared PSRAM; the MCU then reads them back over its own XSPI.  The iCE40
   owns the quad bus while it captures (PG0 released); the MCU takes it back only
   to read out the result.  Because the capture is autonomous there is no reason
   to spin-wait — adc_capture_psram_start()/_poll() let the caller keep an event
   loop (and lwIP) alive; adc_capture_psram() is the blocking convenience wrapper
   for the console/SCPI paths. */

/* Deadline for the in-flight PSRAM capture, armed by *_start(). */
static absolute_time_t psram_cap_deadline;

/* Fill sample_buf16[0..n-1] with one period of `waveform`, 16-bit: the 8-bit
   shape goes into the high byte for the DAC8551.  Returns 0, or -1 on an
   unknown shape name. */
static int fill_wave16(const char *waveform, uint32_t n, uint8_t amplitude,
                       uint8_t offset) {
    if      (strcmp(waveform, "sine")     == 0) fill_sine(n, amplitude, offset);
    else if (strcmp(waveform, "square")   == 0) fill_square(n, amplitude, offset);
    else if (strcmp(waveform, "sawtooth") == 0) fill_sawtooth(n, amplitude, offset);
    else return -1;
    for (uint32_t i = 0; i < n; i++)
        sample_buf16[i] = (uint16_t)((uint16_t)sample_buf[i] << 8);
    return 0;
}

/* No-write sentinel: a distinctive 4-sample marker stamped into the capture
   region before every capture.  If it survives the read-back, the iCE40 wrote
   NOTHING (the STM32 is reading stale PSRAM) — a capture-datapath/timing fault,
   not the analog ADC.  Real ADC data never matches DEAD BEEF CAFE F00D, and the
   iCE40 overwrites addr 0 first when the path is healthy. See docs/adc-capture-cdc-review.md. */
static const uint16_t CAP_NOWRITE_SENTINEL[4] = { 0xDEAD, 0xBEEF, 0xCAFE, 0xF00D };

int adc_capture_psram_start(size_t samples, float sample_rate_hz) {
    /* ADC capture streams to PSRAM, so it can be much deeper than SIGNAL_BUF_SIZE
       (the FPGA-waveform/DAC bound). The MCU read-back buffer (adc_buf16) caps it. */
    if (samples == 0 || samples > ADC_CAP_MAX_SAMPLES) return -1;
    uint32_t divider = adc_divider_from_rate_hz(sample_rate_hz); /* FPGA_CAPTURE/divider (>=v7) */
    uint16_t wire    = cap_divider_wire(divider, s_fpga_version);
    /* Standard START_CAPTURE (count16, divider16); the v2 gateware streams the
       samples into PSRAM instead of its capture BRAM. */
    uint8_t args[4] = {
        (uint8_t)(samples & 0xFF), (uint8_t)((samples >> 8) & 0xFF),
        (uint8_t)(wire & 0xFF),    (uint8_t)((wire >> 8) & 0xFF),
    };
    /* Stamp the no-write sentinel at the ADC region base (bus owned), then hand the
       bus over.  START_CAPTURE arms only the ADC producer (LA off) — the v2 dual
       writer streams it to the ADC region (FPGA_PSRAM_ADC_BASE). */
    psram_bus_acquire();
    psram_write(s_adc_cap_base, (const uint8_t *)CAP_NOWRITE_SENTINEL, sizeof(CAP_NOWRITE_SENTINEL));
    psram_bus_release();   /* hand the shared bus to the iCE40 */
    if (s_fpga_version >= CAPTURE_OPCODE_ONLY_MIN_GW)
        fpga_capture_cmd((uint32_t)samples, divider, 0u, 0u);   /* ADC only: LA count 0 */
    else
        spi_cmd_write(CMD_START_CAPTURE, args, sizeof(args));
    cotrig_consume();
    psram_cap_deadline = make_timeout_time_ms(5000);
    float actual = (float)adc_capture_hz() / (float)divider;
    /* Integer S/s — newlib-nano printf has no %f (the old %.3f MS/s printed nothing). */
    printf("[sig] PSRAM capture armed (%u samples, divider=%lu, %lu S/s)\n",
           (unsigned)samples, (unsigned long)divider, (unsigned long)lroundf(actual));
    return 0;
}

int measure_psram_start(const char *waveform, float freq, uint8_t amplitude,
                        uint8_t offset, size_t samples, float sample_rate_hz) {
    /* period = samples below, and the DAC waveform is 16-bit, so the load is
       samples*2 bytes — bounded by SIGNAL_MAX_SAMPLES (= SIGNAL_BUF_SIZE/2). */
    if (samples == 0 || samples > SIGNAL_MAX_SAMPLES || freq <= 0.0f)
        return -1;

    /* The v2 gateware ties the DAC period to the capture count for MEASURE, so
       one waveform period spans the capture window: period = samples.  Choose
       the sample clock so that single period plays at `freq` (or honour an
       explicit forced sample rate). */
    uint32_t period  = (uint32_t)samples;
    float    fs      = (sample_rate_hz > 0.0f) ? sample_rate_hz
                                               : freq * (float)period;
    /* MEASURE (gateware >= 13) takes SEPARATE dividers for the DAC and the ADC.
       The DAC now runs on DAC_CLK_HZ (48 MHz) and the ADC on FPGA_HFOSC_HZ (24 MHz),
       so to make BOTH step at the same REAL rate:
         DAC_CLK/(dac_div+K) == HFOSC/cap_div,  and DAC_CLK = 2*HFOSC
         => dac_div = 2*cap_div - K.
       The ADC then captures exactly one played waveform period with no drift.
       cap_div is floored at ADC_CAP_MIN_DIVIDER (>= 60), so dac_div stays >= DAC_MIN_DIVIDER. */
    uint32_t cap_div = adc_divider_from_rate_hz(fs);
    uint32_t two_cap = 2u * cap_div;
    uint32_t dac_div = (two_cap >= DAC_SEQ_OVERHEAD_CLK + DAC_MIN_DIVIDER)
                           ? (two_cap - DAC_SEQ_OVERHEAD_CLK) : DAC_MIN_DIVIDER;

    if (fill_wave16(waveform, period, amplitude, offset) != 0) {
        printf("[sig] ERROR: measure unknown waveform \"%s\"\n", waveform);
        return -1;
    }
    log_dac_samples(period);

    /* Load the 16-bit DAC8551 waveform (2 bytes/sample), then start DAC+ADC in
       the same FPGA cycle (START_MEASURE).  Hand the bus to the iCE40 first so
       it can stream the ADC samples into PSRAM. */
    if (fpga_load_wave((const uint8_t *)sample_buf16, period * 2u) != 0) return -1;
    psram_bus_release();
    if (s_fpga_version >= CAPTURE_OPCODE_ONLY_MIN_GW) {
        /* v40: START_MEASURE is gone.  Stage the DAC start on the co-trigger, then arm an
           ADC-only capture: the DAC starts on the capture's t0, the cycle MEASURE used. */
        (void)fpga_dac_arm_on_capture();
        (void)fpga_start_dac(period, dac_div);
        fpga_capture_cmd((uint32_t)samples, cap_div, 0u, 0u);
        cotrig_consume();
    } else if (fpga_start_measure((uint32_t)samples, dac_div, cap_div) != 0) {
        psram_bus_acquire();
        return -1;
    }
    psram_cap_deadline = make_timeout_time_ms(5000);
    /* Both step at the ADC rate now (dac_div = 2*cap_div - K matches the DAC's real
       48 MHz rate to the ADC's 24 MHz rate), so report that one rate + the played
       frequency. */
    float actual = (float)FPGA_HFOSC_HZ / (float)cap_div;
    /* Integer units — newlib-nano printf has no %f (the old %f fields printed
       nothing). Rate is always large (integer S/s); the played fundamental can be
       sub-Hz (milli-Hz). */
    printf("[sig] measure(v2): %s period=%lu samples=%u dac_div=%lu cap_div=%lu "
           "(fs=%lu S/s, f=%lu mHz)\n",
           waveform, (unsigned long)period, (unsigned)samples,
           (unsigned long)dac_div, (unsigned long)cap_div,
           (unsigned long)lroundf(actual),
           (unsigned long)lroundf(actual / (float)period * 1000.0f));
    return 0;
}

/* Read a PSRAM region back with a short settle + a couple of retries.  Right after
   a capture the shared-bus hand-off from the iCE40 can make the FIRST QPI read
   transaction error even though the bus is fine a moment later; a settle + retry
   (each re-acquiring the bus) recovers it without failing the whole capture. */
static int psram_read_settled(uint32_t addr, uint8_t *buf, uint32_t len) {
    for (int attempt = 0; attempt < 3; attempt++) {
        sleep_ms(1);
        psram_bus_acquire();
        int rc = psram_read(addr, buf, len);
        psram_bus_release();
        if (rc == 0) return 0;
    }
    return -1;
}

int adc_capture_psram_poll(uint16_t *out16, size_t samples) {
    uint8_t st = 0;
    fpga_status_read(&st);
    if (st & STATUS_CAP_DONE) {
        if (cap_overflowed(st, "ADC PSRAM capture")) return -1;
        /* CAP_DONE latches the instant the last sample is captured, but the
           PSRAM writer may still be flushing its final tCEM chunk — BUSY stays
           low until it goes idle.  Drain that (µs-scale, bounded) before we
           grab the shared bus, otherwise tristating the writer mid-flush would
           drop the last samples.  This is the only spin in the capture, and it
           is the writer tail, not the (fully async) capture itself. */
        uint32_t spin = 0;
        while (!busy_read() && spin < 100000u) spin++;   /* ~µs flush; ~4 ms cap */
        /* Settle: the v2 drain controller raises CAP_DONE only once the writer has
           fully drained + idled, so BUSY is already high here and the flush spin
           above is a no-op — unlike the old single-writer path where CAP_DONE fired
           early and the spin gave the shared bus a real settle before the read-back.
           A short explicit settle lets the iCE40's last QPI write / CS-deassert land
           on the shared bus before the STM32 grabs it, else the first read-back
           transaction races the hand-off and the OCTOSPI transfer errors. */
        /* Take the bus back and read the samples out of PSRAM (QPI, GPIO-CS,
           settle + retry), then hand it back so the iCE40 owns it again at idle. */
        int rc = psram_read_settled(s_adc_cap_base, (uint8_t *)out16, (uint32_t)(samples * 2u));
        if (rc != 0) { printf("[sig] PSRAM read-back failed\n"); return -1; }
        /* No-write detector: if the sentinel survived, the iCE40 never wrote. */
        if (samples >= 4 &&
            out16[0] == CAP_NOWRITE_SENTINEL[0] && out16[1] == CAP_NOWRITE_SENTINEL[1] &&
            out16[2] == CAP_NOWRITE_SENTINEL[2] && out16[3] == CAP_NOWRITE_SENTINEL[3]) {
            printf("[sig] ERROR: iCE40 did NOT write the capture to PSRAM (sentinel "
                   "survived) — capture datapath/timing fault in the iCE40, NOT the "
                   "analog ADC. Run cap-selftest.\n");
            return -1;
        }
        return 1;
    }
    if (time_reached(psram_cap_deadline)) {
        printf("[sig] ERROR: PSRAM capture timed out (CAP_DONE never set)\n");
        return -1;
    }
    return 0;
}

int adc_capture_psram(uint16_t *out16, size_t samples, float sample_rate_hz) {
    if (!out16) return -1;
    if (adc_capture_psram_start(samples, sample_rate_hz) != 0) {
        return -1;
    }
    for (;;) {
        int r = adc_capture_psram_poll(out16, samples);
        if (r == 1) {
            printf("[sig] PSRAM capture complete (%u samples)\n", (unsigned)samples);
            return 0;
        }
        if (r < 0) return -1;
        tight_loop_contents();
    }
}

/* ---- v2: deep multi-channel LA capture into PSRAM (gateware >= 8) -----------
   The iCE40 (la_psram_capture) streams 12-channel LA samples — 2 LE bytes each,
   same layout as fpga_la_capture — into PSRAM, so the depth is bounded by PSRAM
   instead of the 4 KB on-FPGA trace buffer.  Same async bus-handoff dance as the
   ADC PSRAM capture: arm, poll for CAP_DONE, then read the region back over XSPI.
   The read-back is chunked by the caller (command_handler bulk sender), so unlike
   adc_capture_psram_poll this leaves the bus ACQUIRED on completion.

   Rate + burst budget: the deep-LA path now streams through spram_ring16 — a 32 KB
   16-bit-packed single-port SPRAM burst buffer (ice40 AW=14: 16K words) — before the
   (unchanged) PSRAM writer.  So the LA can sample FASTER than the writer drains for a
   bounded BURST (~32K samples fully at 12 MS/s); the ring
   batches the excess.  Two limits, both handled here:
     • peak rate  — LA_PSRAM_MIN_DIV=2 => 24 MHz / 2 = 12 MS/s, the ring's input rate
       (a real 12 MS/s since gateware v32 — older gateware sampled every divider + 1
       clocks; cap_divider_wire() compensates for it).
     • burst depth — above the writer's sustained drain the ring fills at the net
       rate, so a capture is only lossless while its PEAK occupancy fits the ring:
       samples*2*(1 - drain/rate) <= LA_RING_BYTES.  For a longer capture we LOWER
       the rate (raise the divider) so the whole count fits — a uniform, no-dropped-
       sample capture at the fastest rate that fits.  Captures up to LA_RING_BYTES/2
       samples always run at the full 12 MS/s.
   The (samples, rate) -> divider math lives in la_rate.c (pure, host-tested in
   test/test_la_rate.c) so this exact behaviour is regression-locked — including the
   "do NOT inherit the ADC's min-divider floor" rule that the shipped 0.3-MS/s bug
   violated.  Conservative drain estimate; HW-tuned on the bench. */

int fpga_la_capture_psram_start(size_t samples, float sample_rate_hz) {
    /* Dynamic ceiling: LA can use everything from the front up to a resident deep-DAC
       waveform (or the whole chip if none).  LA-only here (adc_in_capture=false) — the
       unified ADC+LA path caps LA to the ADC region separately. */
    uint32_t la_max = signal_engine_la_max_samples(false);
    if (samples == 0 || samples > la_max) {
        printf("[la] ERROR: LA samples=%u out of range (max %lu with %lu DAC bytes resident)\n",
               (unsigned)samples, (unsigned long)la_max,
               (unsigned long)(s_dac_ps_active ? (FPGA_PSRAM_TOTAL - s_dac_ps_base) : 0u));
        return -1;
    }

    la_plan_t plan = la_psram_plan(adc_capture_hz(), (uint32_t)samples, sample_rate_hz);
    uint32_t divider = plan.divider;
    if (plan.capped)
        printf("[la] PSRAM LA rate capped to %lu kS/s so %u samples fit the %u-byte "
               "burst buffer (no dropped samples)\n",
               (unsigned long)(plan.rate_hz / 1000u), (unsigned)samples,
               (unsigned)LA_RING_BYTES);
    if (plan.floored)
        printf("[la] PSRAM LA rate clamped to the 12 MS/s sampler ceiling (divider=%lu)\n",
               (unsigned long)divider);

    /* LA_CAPTURE [count_lo][count_mid][count_hi][div_lo][div_hi]; the count is 24-bit
       so a single capture can span the full 8 MB PSRAM.  The v2 gateware streams the
       samples into PSRAM (the MCU reads them back over XSPI). */
    uint16_t wire = cap_divider_wire(divider, s_fpga_version);
    uint8_t args[5] = {
        (uint8_t)(samples & 0xFF), (uint8_t)((samples >> 8) & 0xFF), (uint8_t)((samples >> 16) & 0xFF),
        (uint8_t)(wire & 0xFF),    (uint8_t)((wire >> 8) & 0xFF),
    };
    psram_bus_release();   /* hand the shared bus to the iCE40 */
    if (s_fpga_version >= CAPTURE_OPCODE_ONLY_MIN_GW)
        fpga_capture_cmd(0u, 0u, (uint32_t)samples, divider);   /* LA only: ADC count 0 */
    else
        spi_cmd_write(CMD_LA_CAPTURE, args, sizeof(args));
    cotrig_consume();

    /* Deadline scales with the capture window (samples * divider / capture clk) so
       deep/slow captures are not cut short. */
    uint32_t cap_hz    = adc_capture_hz();
    uint64_t window_ms = ((uint64_t)samples * divider) / (cap_hz / 1000u);
    psram_cap_deadline = make_timeout_time_ms((uint32_t)(window_ms + 3000u));
    /* integer kSPS (newlib-nano printf has no %f) */
    uint32_t ksps = (uint32_t)(((uint64_t)cap_hz / divider) / 1000u);
    printf("[la] PSRAM capture armed (%u samples, divider=%lu, %lu.%03lu MS/s, ~%lu ms window)\n",
           (unsigned)samples, (unsigned long)divider,
           (unsigned long)(ksps / 1000u), (unsigned long)(ksps % 1000u),
           (unsigned long)window_ms);
    return 0;
}

int fpga_la_capture_psram_wait(void) {
    uint8_t st = 0;
    fpga_status_read(&st);
    if (st & STATUS_CAP_DONE) {
        if (cap_overflowed(st, "LA PSRAM capture")) return -1;
        /* CAP_DONE latches at the last sample, but the writer may still be flushing
           its final tCEM chunk — BUSY stays low until idle.  Drain that before we
           grab the bus (same as adc_capture_psram_poll), then take the bus and hold
           it for the chunked read-back. */
        uint32_t spin = 0;
        while (!busy_read() && spin < 100000u) spin++;
        psram_bus_acquire();
        return 1;
    }
    if (time_reached(psram_cap_deadline)) {
        printf("[la] ERROR: PSRAM LA capture timed out (CAP_DONE never set)\n");
        return -1;
    }
    return 0;
}

int fpga_la_psram_read(uint32_t offset, uint8_t *buf, uint32_t n) {
    /* `offset` is relative to the LA region — the v2 dual writer streams the raw-LA
       capture to FPGA_PSRAM_LA_BASE, so read there (not addr 0, which is the ADC
       region). */
    return psram_read(s_la_cap_base + offset, buf, n);
}

void fpga_la_psram_release(void) {
    psram_bus_release();
}

/* ---- v2 (>=16): DEEP unified capture, chunked PSRAM read-back --------------
   The dual writer streams the ADC capture to FPGA_PSRAM_ADC_BASE and the raw-LA
   capture to FPGA_PSRAM_LA_BASE (each up to a multi-MB region).  Rather than read
   whole regions into MCU SRAM (impossible at MB scale), the command_handler bulk
   sender streams them out chunk-by-chunk: the ADC region first (global sample
   indices [0, adc_samples)), then the LA region — matching the old combined
   "ADC then LA, all 16-bit" wire format the server splits at adc_samples.  Same
   async bus-handoff dance as the LA path: wait for CAP_DONE (grabs+HOLDS the bus),
   read chunks, release. */
int fpga_capture_psram_wait(void) {
    uint8_t st = 0;
    fpga_status_read(&st);
    if (st & STATUS_CAP_DONE) {
        if (cap_overflowed(st, "capture")) return -1;
        /* CAP_DONE latches at the last sample; the v2 drain controller only raises it
           once the writer has fully drained + idled, but keep the bounded flush spin
           for parity with the LA/ADC poll paths before grabbing the shared bus. */
        uint32_t spin = 0;
        while (!busy_read() && spin < 100000u) spin++;
        psram_bus_acquire();   /* hold the bus for the whole chunked read-back */
        return 1;
    }
    if (time_reached(psram_cap_deadline)) {
        printf("[sig] ERROR: capture timed out (CAP_DONE never set)\n");
        return -1;
    }
    return 0;
}

bool fpga_capture_adc_sentinel_survived(void) {
    /* Read the first 4 words of the ADC region (bus already held) and compare to the
       no-write sentinel stamped at arm time.  Survived => the iCE40 never wrote it. */
    uint16_t head[4] = {0};
    if (psram_read(s_adc_cap_base, (uint8_t *)head, sizeof(head)) != 0)
        return false;   /* read error is reported elsewhere; don't false-positive here */
    return head[0] == CAP_NOWRITE_SENTINEL[0] && head[1] == CAP_NOWRITE_SENTINEL[1] &&
           head[2] == CAP_NOWRITE_SENTINEL[2] && head[3] == CAP_NOWRITE_SENTINEL[3];
}

int fpga_capture_psram_read16(size_t sample_index, uint16_t *out16, size_t n,
                              size_t adc_samples) {
    /* The chunk must not straddle the ADC/LA boundary (the caller clamps it).  ADC
       samples read from the ADC region at byte offset index*2; LA words read from the
       LA region at (index - adc_samples)*2.  The STM32 is little-endian, matching the
       gateware's LE sample packing, so the raw bytes ARE the uint16 values. */
    uint32_t addr;
    if (sample_index < adc_samples)
        addr = s_adc_cap_base + (uint32_t)(sample_index * 2u);
    else
        addr = s_la_cap_base  + (uint32_t)((sample_index - adc_samples) * 2u);
    return psram_read(addr, (uint8_t *)out16, (uint32_t)(n * 2u));
}

void fpga_capture_psram_release(void) {
    psram_bus_release();
}

/* Unified capture (v2): arm the ADC and the raw 12-channel LA producers TOGETHER
 * (CMD_CAPTURE / one trigger, shared t0) and read both PSRAM regions back.  The
 * gateware writes each stream to its own contiguous region (ADC -> ADC_BASE, LA ->
 * LA_BASE) with NO tags/timestamps — sample i of each stream is at t0 + i*divider,
 * and both counters start at the arm, so the two are time-aligned by construction.
 *
 * adc_count==0 => ADC not captured; la_count==0 => LA not captured (so this one
 * call serves ADC-only, LA-only, and simultaneous).  out_adc receives adc_count
 * 16-bit ADC samples; out_la receives la_count 16-bit LA words (low 12 bits =
 * LA1..LA14).  Either out pointer may be NULL if its count is 0.  Both dividers are
 * sample PERIODS in 24 MHz clk ticks (encoded for the gateware here, see
 * cap_divider_wire).  Returns 0 on success, -1 on error/timeout/overflow. */
/* Async arm: send CMD_CAPTURE (0x31) and return immediately; poll with
   fpga_dual_capture_poll().  Lets the net task keep servicing lwIP while the
   capture + read-back runs (the server /capture path streams both regions). */
int fpga_dual_capture_start(uint32_t adc_count, uint16_t adc_div,
                            uint32_t la_count, uint16_t la_div) {
    if (adc_count == 0 && la_count == 0) return -1;
    if (adc_count > ADC_CAP_MAX_SAMPLES || la_count > LA_PSRAM_MAX_SAMPLES) return -1;
    /* v16 CMD_CAPTURE payload: adc_cnt(3) + adc_div(2) + la_cnt(3) + la_div(2) — both
       counts 24-bit so a single unified capture spans the multi-MB ADC AND LA regions. */
    psram_bus_acquire();
    if (adc_count) psram_write(s_adc_cap_base, (const uint8_t *)CAP_NOWRITE_SENTINEL, sizeof(CAP_NOWRITE_SENTINEL));
    psram_bus_release();
    fpga_capture_cmd(adc_count, adc_div, la_count, la_div);
    cotrig_consume();
    /* Deadline scales with the (deep) capture window so a multi-second capture is not
       cut short; ADC dominates (slower rate), so budget on max(adc,la) sample*divider. */
    uint32_t cap_hz = adc_capture_hz();
    uint64_t adc_win = (uint64_t)adc_count * (adc_div ? adc_div : 2u);
    uint64_t la_win  = (uint64_t)la_count  * (la_div  ? la_div  : 2u);
    uint64_t win     = adc_win > la_win ? adc_win : la_win;
    uint64_t win_ms  = win / (cap_hz / 1000u);
    psram_cap_deadline = make_timeout_time_ms((uint32_t)(win_ms + 5000u));
    return 0;
}

/* Poll a started dual capture: 0 = still running, 1 = done (both regions read
   into out_adc/out_la), -1 = error/timeout/overflow. */
int fpga_dual_capture_poll(uint16_t *out_adc, uint16_t adc_count,
                           uint16_t *out_la,  uint16_t la_count) {
    uint8_t st = 0;
    fpga_status_read(&st);
    if (st & STATUS_CAP_DONE) {
        if (cap_overflowed(st, "capture")) return -1;
        uint32_t spin = 0; while (!busy_read() && spin < 100000u) spin++;   /* writer flush */
        int rc = 0;
        if (adc_count && out_adc)
            rc |= psram_read_settled(s_adc_cap_base, (uint8_t *)out_adc, (uint32_t)adc_count * 2u);
        if (la_count && out_la)
            rc |= psram_read_settled(s_la_cap_base,  (uint8_t *)out_la,  (uint32_t)la_count  * 2u);
        if (rc != 0) { printf("[sig] capture PSRAM read-back failed\n"); return -1; }
        return 1;
    }
    if (time_reached(psram_cap_deadline)) { printf("[sig] capture timed out (CAP_DONE never set)\n"); return -1; }
    return 0;
}

int fpga_dual_capture(uint16_t *out_adc, uint16_t adc_count, uint16_t adc_div,
                      uint16_t *out_la,  uint16_t la_count,  uint16_t la_div) {
    if (fpga_dual_capture_start(adc_count, adc_div, la_count, la_div) != 0) return -1;
    for (;;) {
        int r = fpga_dual_capture_poll(out_adc, adc_count, out_la, la_count);
        if (r == 1) return 0;
        if (r < 0)  return -1;
        tight_loop_contents();
    }
}

/* Rate-based async arm for the dual capture (server /capture path): converts the
   requested Hz to gateware dividers — ADC against the ADC capture clock, raw-LA
   against the 24 MHz control clock — and returns the ACHIEVED rates via
   *adc_rate_hz/*la_rate_hz so the timebase is exact.  Poll with
   fpga_dual_capture_poll(). */
int fpga_dual_capture_start_hz(uint32_t adc_samples, float adc_req_hz,
                               uint32_t la_samples, float la_req_hz,
                               float *adc_rate_hz, float *la_rate_hz) {
    uint32_t ad = adc_samples ? adc_divider_from_rate_hz(adc_req_hz) : 2u;
    uint32_t ld = la_samples  ? divider_from_rate_hz(la_req_hz)      : 2u;
    if (ad < 2u) ad = 2u; if (ad > 65535u) ad = 65535u;
    if (ld < 2u) ld = 2u; if (ld > 65535u) ld = 65535u;
    if (adc_rate_hz) *adc_rate_hz = (float)adc_capture_hz() / (float)ad;
    if (la_rate_hz)  *la_rate_hz  = (float)FPGA_HFOSC_HZ     / (float)ld;
    return fpga_dual_capture_start(adc_samples, (uint16_t)ad, la_samples, (uint16_t)ld);
}

int measure_psram(uint16_t *out16, const char *waveform, float freq,
                  uint8_t amplitude, uint8_t offset, size_t samples,
                  float sample_rate_hz) {
    if (!out16) return -1;
    if (measure_psram_start(waveform, freq, amplitude, offset, samples,
                            sample_rate_hz) != 0) return -1;
    for (;;) {
        int r = adc_capture_psram_poll(out16, samples);
        if (r == 1) {
            fpga_stop_dac();
            printf("[sig] measure(v2) complete (%u samples)\n", (unsigned)samples);
            return 0;
        }
        if (r < 0) { fpga_stop_dac(); return -1; }
        tight_loop_contents();
    }
}


/* ===========================================================================
 * Logic-Analyzer GPIO bank + SWD — FPGA-backed (replaces the RP2350 PIO)
 *
 * Host-facing LA channels are 1-based (LA1..LA14); the gateware indexes its
 * la[] bus 0-based, so we send (channel - 1) on the wire.
 * ========================================================================== */

/* SWD inactivity backstop — auto-disarm a wedged session.  Long enough that a
   legitimate idle phase during a flash (OpenOCD running a flash-erase/loader on
   the target can pause SWD bit-banging for a while) never trips it; it only
   recovers a genuinely abandoned session. */
#define SWD_INACTIVITY_MS   300000u

static bool            swd_armed_local = false;
static absolute_time_t swd_deadline;   /* refreshed on each arm/feed */

/* Validate a host LA channel (1..14) and return its 0-based wire index, or -1. */
static int la_wire_index(unsigned channel) {
    if (channel < LA_CHANNEL_MIN || channel > LA_CHANNEL_MAX) return -1;
    return (int)(channel - 1u);
}

int fpga_la_set(unsigned channel, int mode) {
    int idx = la_wire_index(channel);
    if (idx < 0)            return -1;
    if (mode < 0 || mode > 2) return -1;
    uint8_t args[2] = { (uint8_t)idx, (uint8_t)mode };
    spi_cmd_write(CMD_GPIO_SET, args, sizeof(args));
    static const char *names[3] = { "low", "high", "high-Z" };
    printf("[la] LA%u -> %s\n", channel, names[mode]);
    return 0;
}

/* Drive the iCE40 onboard status LEDs (SB_RGBA_DRV pads):
   bit0 = green, bit1 = yellow, bit2 = red.  Purely a visual indicator —
   safe to call any time SPI1 is up; if the iCE40 isn't configured the
   command is simply ignored on the wire. */
void fpga_set_led(uint8_t mask) {
    spi_cmd_write(CMD_SET_LED, &mask, 1);
}

/* ---- GPIO_GET + capture trigger (gateware >= v35) ------------------------ */

int fpga_gpio_get(uint16_t *levels) {
    if (s_fpga_version < GPIO_GET_MIN_GW) return -2;
    uint8_t b[2] = {0};
    if (spi_cmd_read(CMD_GPIO_GET, NULL, 0, b, sizeof(b)) != 0) return -1;
    if (levels) *levels = (uint16_t)((b[0] | ((uint16_t)b[1] << 8)) & 0x3FFFu);
    return 0;
}

int fpga_set_trigger(unsigned la, uint8_t mode) {
    if (s_fpga_version < CAPTURE_TRIGGER_MIN_GW) return -2;
    if (mode > 4u) return -1;
    int idx = 0;
    if (mode != 0u) {
        idx = la_wire_index(la);
        if (idx < 0) return -1;
    }
    uint8_t args[3] = { (uint8_t)idx, mode, 0u };   /* [channel][mode][flags: reserved, 0] */
    return spi_cmd_write(CMD_SET_TRIGGER, args, sizeof(args)) == 0 ? 0 : -1;
}

int fpga_trigger_status(uint8_t *status) {
    if (s_fpga_version < CAPTURE_TRIGGER_MIN_GW) return -2;
    uint8_t b = 0;
    if (spi_cmd_read(CMD_TRIGGER_STATUS, NULL, 0, &b, 1) != 0) return -1;
    if (status) *status = b;
    return 0;
}

void fpga_capture_extend_deadline_ms(uint32_t ms) {
    psram_cap_deadline += (uint64_t)ms * 1000ull;   /* absolute_time_t is microseconds */
}

void fpga_capture_abort(void) {
    if (s_cotrig_in_capture) dac_stop();
    uint8_t args[10] = {0};   /* CAPTURE with both counts 0: no producer, never waits */
    spi_cmd_write(CMD_CAPTURE, args, sizeof(args));
    s_cotrig_in_capture = false;
    (void)fpga_set_trigger(0u, 0u);
    printf("[sig] capture aborted while waiting for its trigger\n");
}

bool fpga_la_step_busy(void) {
    uint8_t st = 0;
    if (fpga_status_read(&st) != 0) return false;
    return (st & STATUS_STEP_BUSY) != 0;
}

int fpga_la_step(unsigned channel, uint32_t steps, uint32_t delay_us) {
    int idx = la_wire_index(channel);
    if (idx < 0)                                  return -1;
    if (steps == 0 || steps > LA_STEP_MAX_STEPS)  return -3;
    if (delay_us < LA_STEP_MIN_DELAY_US || delay_us > LA_STEP_MAX_DELAY_US)
        return -4;
    if (fpga_la_step_busy()) return -2;

    /* [CMD][channel][steps(2, LE)][delay(2, LE)] — 16-bit gateware fields; v40+ takes the
       half-phase minus one (step_delay_wire). */
    uint16_t dwire = step_delay_wire(delay_us, s_fpga_version);
    uint8_t args[5] = {
        (uint8_t)idx,
        (uint8_t)(steps    & 0xFF), (uint8_t)((steps    >> 8) & 0xFF),
        (uint8_t)(dwire & 0xFF),    (uint8_t)((dwire >> 8) & 0xFF),
    };
    spi_cmd_write(CMD_GPIO_STEP, args, sizeof(args));
    printf("[la] LA%u step x%lu started (delay %luus)\n",
           channel, (unsigned long)steps, (unsigned long)delay_us);
    return 0;
}

int fpga_swd_arm(unsigned swclk, unsigned swdio) {
    if (swd_armed_local) return -2;
    int clk_idx = la_wire_index(swclk);
    int dio_idx = la_wire_index(swdio);
    if (clk_idx < 0 || dio_idx < 0 || clk_idx == dio_idx) return -1;

    /* 0xFF = the gateware drives no reset line.  nRESET is the pod's own
       /NRST_CONTROL pin now (nrst_ctrl.c), not a borrowed LA channel — the
       swd_engine.v 's'/'r' opcodes are simply never sent. */
    uint8_t args[3] = { (uint8_t)clk_idx, (uint8_t)dio_idx, 0xFF };
    spi_cmd_write(CMD_SWD_ARM, args, sizeof(args));
    swd_armed_local = true;
    swd_deadline    = make_timeout_time_ms(SWD_INACTIVITY_MS);
    printf("[swd] armed  SWCLK=LA%u SWDIO=LA%u nRESET=%s\n",
           swclk, swdio, nrst_ctrl_supported() ? "PF4 (J1 pin 22)" : "unavailable");
    return 0;
}

void fpga_swd_disarm(void) {
    if (!swd_armed_local) return;
    spi_cmd_write(CMD_SWD_DISARM, NULL, 0);
    swd_armed_local = false;
    printf("[swd] disarmed\n");
}

bool fpga_swd_armed(void) { return swd_armed_local; }

void fpga_swd_poll(void) {
    if (!swd_armed_local) return;
    if (time_reached(swd_deadline)) {
        printf("[swd] inactivity timeout — disarming\n");
        fpga_swd_disarm();
    }
}

/* Send one SWD_FEED transaction: [CMD][len_lo][len_hi][N remote_bitbang bytes]. */
static void fpga_swd_feed_raw(const uint8_t *data, size_t len) {
    uint8_t hdr[3] = { CMD_SWD_FEED,
                       (uint8_t)(len & 0xFF), (uint8_t)((len >> 8) & 0xFF) };
    cs_select();
    spi_write_blocking(SPI_PORT, hdr, sizeof(hdr));
    if (len) spi_write_blocking(SPI_PORT, data, len);
    cs_deselect();
}

/* Read `count` sample reply bytes back: [CMD][len_lo][len_hi] then clock N out. */
static void fpga_swd_read_raw(uint8_t *reply, size_t count) {
    uint8_t args[2] = { (uint8_t)(count & 0xFF), (uint8_t)((count >> 8) & 0xFF) };
    spi_cmd_read(CMD_SWD_READ, args, sizeof(args), reply, count);
}

size_t fpga_swd_feed(const uint8_t *in, size_t len,
                     char *reply, size_t reply_cap, size_t *reply_len,
                     bool *exit_to_json, bool *quit) {
    *exit_to_json = false;
    *quit         = false;
    *reply_len    = 0;

    /* Per-feed wire buffer.  Bounded so it never exceeds the FPGA reply BRAM
       (512) and so each SPI burst stays modest. */
    uint8_t wire[256];
    size_t  i = 0;

    if (swd_armed_local) swd_deadline = make_timeout_time_ms(SWD_INACTIVITY_MS);

    while (i < len) {
        size_t wlen = 0;   /* wire-driving bytes in this chunk            */
        size_t cc   = 0;   /* 'c' (sample) bytes in this chunk            */
        bool   stop = false;

        while (i < len && wlen < sizeof(wire)) {
            uint8_t b = in[i];
            if (b == '{') { *exit_to_json = true;        stop = true; break; } /* don't consume */
            if (b == 'Q') { *quit = true; i++;           stop = true; break; } /* consume       */
            if (b == 'c') {
                /* Stop before a sample that would overflow the caller's reply
                   buffer, so it can flush and call us again (matches the old
                   swd_probe_feed semantics). */
                if (*reply_len + cc >= reply_cap) { stop = true; break; }
                cc++;
            }
            wire[wlen++] = b;
            i++;
        }

        if (wlen > 0) {
            fpga_swd_feed_raw(wire, wlen);
            if (cc > 0) {
                fpga_swd_read_raw((uint8_t *)reply + *reply_len, cc);
                *reply_len += cc;
            }
        }
        if (stop) break;
    }

    return i;
}

/* ===========================================================================
 * Emulated I2C sensor — FPGA-backed generic I2C target driven over SPI.
 *
 * The FPGA holds a 256-byte register image and presents it as an I2C slave on
 * two LA channels; this RP firmware (the orchestrator) fills that image and
 * configures the bus.  All sensor-specific knowledge (register maps, IDs,
 * calibration math) lives in sensor_*.c — these are just thin SPI wrappers.
 *
 * Host LA channels are 1-based (LA1..LA14); the gateware indexes la[] 0-based,
 * so SDA/SCL channels are sent as (channel - 1) like the SWD/GPIO commands.
 * ========================================================================== */

int fpga_i2c_sensor_config(uint8_t addr7, unsigned sda_ch, unsigned scl_ch,
                           bool enable, uint8_t trig_reg, uint8_t busy_reg,
                           uint8_t busy_mask, uint16_t conv_us) {
    int sda = la_wire_index(sda_ch);
    int scl = la_wire_index(scl_ch);
    if (sda < 0 || scl < 0 || sda == scl) return -1;

    /* [addr7][sda_ch][scl_ch][flags][trig_reg][busy_reg][busy_mask][conv_lo][conv_hi] */
    uint8_t args[9] = {
        (uint8_t)(addr7 & 0x7F),
        (uint8_t)sda, (uint8_t)scl,
        (uint8_t)(enable ? 0x01 : 0x00),
        trig_reg, busy_reg, busy_mask,
        (uint8_t)(conv_us & 0xFF), (uint8_t)((conv_us >> 8) & 0xFF),
    };
    spi_cmd_write(CMD_I2C_CONFIG, args, sizeof(args));
    printf("[i2c-sim] config addr=0x%02x SDA=LA%u SCL=LA%u %s trig=0x%02x "
           "busy=0x%02x/0x%02x conv=%uus\n",
           addr7, sda_ch, scl_ch, enable ? "enabled" : "disabled",
           trig_reg, busy_reg, busy_mask, conv_us);
    return 0;
}

void fpga_i2c_sensor_disable(void) {
    spi_cmd_write(CMD_I2C_DISABLE, NULL, 0);
    printf("[i2c-sim] disabled\n");
}

int fpga_i2c_load_regs(uint8_t start_addr, const uint8_t *data, size_t len) {
    if (!data || len == 0 || (size_t)start_addr + len > 256) return -1;
    /* [CMD][start_addr][len_lo][len_hi][N data bytes] */
    uint8_t hdr[4] = { CMD_I2C_LOAD_REGS, start_addr,
                       (uint8_t)(len & 0xFF), (uint8_t)((len >> 8) & 0xFF) };
    cs_select();
    spi_write_blocking(SPI_PORT, hdr, sizeof(hdr));
    spi_write_blocking(SPI_PORT, data, len);
    cs_deselect();
    return 0;
}

int fpga_i2c_read_regs(uint8_t start_addr, uint8_t *buf, size_t len) {
    if (!buf || len == 0 || (size_t)start_addr + len > 256) return -1;
    uint8_t args[3] = { start_addr,
                        (uint8_t)(len & 0xFF), (uint8_t)((len >> 8) & 0xFF) };
    spi_cmd_read(CMD_I2C_READ_REGS, args, sizeof(args), buf, len);
    return 0;
}

int fpga_i2c_sensor_status(i2c_sensor_status_t *out) {
    if (!out) return -1;
    uint8_t b[7] = {0};
    spi_cmd_read(CMD_I2C_STATUS, NULL, 0, b, sizeof(b));
    out->armed        = (b[0] & 0x01) != 0;
    out->xfer_count   = (uint16_t)(b[1] | ((uint16_t)b[2] << 8));
    out->wr_count     = (uint16_t)(b[3] | ((uint16_t)b[4] << 8));
    out->last_wr_addr = b[5];
    out->last_wr_val  = b[6];
    return 0;
}

int fpga_i2c_la_capture(uint8_t *buf, size_t bytes, float sample_rate_hz) {
    if (!buf || bytes == 0 || bytes > SIGNAL_BUF_SIZE) return -1;

    /* MIGRATED (gateware >= v24 removed the on-FPGA I2C-LA sampler + its 0x65/0x66
       opcodes to reclaim ~150 LC + an SPRAM block): source the raw {SCL,SDA} samples
       from the general DEEP-LA-into-PSRAM path (OP_LA_CAPTURE 0x69) and re-pack them
       here into the SAME legacy 4-samples/byte layout the server/python I2C decoder
       still expects — so `sensor_la` is byte-for-byte wire-compatible (no server or
       client change).  Packed byte (oldest sample in the high bits, matching
       i2c_la_capture.v): bit7=scl(s0) bit6=sda(s0) … bit1=scl(s3) bit0=sda(s3).
       The two bus channels come from the active emulated sensor's config. */
    unsigned sda_ch = sensor_sim_sda();     /* 1-based LA channels; 0 = no sensor armed */
    unsigned scl_ch = sensor_sim_scl();
    if (sda_ch == 0 || scl_ch == 0) return -1;
    uint8_t sda_bit = (uint8_t)(sda_ch - 1u);   /* deep-LA word bit index (LA1..14 -> 0..13) */
    uint8_t scl_bit = (uint8_t)(scl_ch - 1u);

    size_t samples = bytes * 4u;            /* 4 packed {SCL,SDA} samples per output byte */

    /* Arm the deep-LA capture (hands the shared PSRAM bus to the iCE40) and block until
       it completes — mirroring the old synchronous fill contract of this function. */
    if (fpga_la_capture_psram_start(samples, sample_rate_hz) != 0) return -1;
    int r;
    do { sleep_ms(1); r = fpga_la_capture_psram_wait(); } while (r == 0);  /* 1=done(bus held), -1=timeout */
    if (r < 0) return -1;

    /* Read the LA region back in chunks (2 bytes/sample, LE: byte0=LA1..8, byte1={00,LA9..14})
       and re-pack 4 samples per output byte. */
    uint8_t  chunk[256];                    /* 128 samples/chunk; no large new buffer (RAM is tight) */
    size_t   out = 0;
    uint32_t off = 0;
    uint8_t  acc = 0;
    unsigned nib = 0;
    for (size_t s = 0; s < samples; ) {
        size_t n = samples - s;
        if (n > (sizeof(chunk) / 2u)) n = sizeof(chunk) / 2u;
        if (fpga_la_psram_read(off, chunk, (uint32_t)(n * 2u)) != 0) { fpga_la_psram_release(); return -1; }
        off += (uint32_t)(n * 2u);
        for (size_t i = 0; i < n; i++) {
            uint16_t w   = (uint16_t)chunk[2u*i] | ((uint16_t)(chunk[2u*i + 1u] & 0x0Fu) << 8);
            uint8_t  scl = (uint8_t)((w >> scl_bit) & 1u);
            uint8_t  sda = (uint8_t)((w >> sda_bit) & 1u);
            acc = (uint8_t)((acc << 2) | (scl << 1) | sda);   /* oldest shifts up to bits[7:6] */
            if (++nib == 4u) { buf[out++] = acc; acc = 0; nib = 0; }
        }
        s += n;
    }
    fpga_la_psram_release();
    /* samples is a multiple of 4, so `out == bytes` and no partial byte remains. */
    printf("[i2c-sim] LA capture %u bytes (%u samples via deep-LA sda=%u scl=%u)\n",
           (unsigned)bytes, (unsigned)samples, sda_ch, scl_ch);
    return 0;
}

/* ===========================================================================
 * UART proxy — FPGA-backed soft UART on two LA channels, driven over SPI.
 *
 * The FPGA serialises/deserialises 8N1 frames on the chosen LA channels; this
 * firmware pushes TX bytes and drains RX bytes and bridges them to the console
 * or a TCP client (see console.c / command_handler.c).  Thin SPI wrappers,
 * mirroring the fpga_swd_* shape.  SDA/SCL... err, RX/TX are 1-based LA channels.
 * ========================================================================== */

/* STATUS flags byte (from CMD_UART_STATUS, must match cmd_dispatch). */
#define UART_FLAG_TX_FULL     (1u << 0)
#define UART_FLAG_TX_EMPTY    (1u << 1)
#define UART_FLAG_RX_OVERFLOW (1u << 2)
#define UART_FLAG_ARMED       (1u << 3)

/* Single-owner guard: only one UART proxy (console OR one TCP conn) at a time. */
static bool uart_armed_local = false;

int fpga_uart_config(unsigned rx_ch, unsigned tx_ch, uint32_t baud, bool enable) {
    if (enable && uart_armed_local) return -2;   /* already in use */

    int rx = la_wire_index(rx_ch);
    int tx = la_wire_index(tx_ch);
    if (rx < 0 || tx < 0 || rx == tx) return -1;
    if (baud == 0) return -1;

    /* divisor = round(HFOSC / baud), clamped to what the gateware's bit counter holds: 2 ..
       2^18-1 (~92 baud).  Gateware <= v39 clamped it the same way itself; v40 takes it as sent. */
    uint32_t div = (uint32_t)(((uint64_t)FPGA_HFOSC_HZ + baud / 2) / baud);
    if (div < 2)        div = 2;
    if (div > 0x3FFFFu) div = 0x3FFFFu;

    /* [rx_ch][tx_ch][div_lo][div_mid][div_hi][flags] */
    uint8_t args[6] = {
        (uint8_t)rx, (uint8_t)tx,
        (uint8_t)(div & 0xFF), (uint8_t)((div >> 8) & 0xFF), (uint8_t)((div >> 16) & 0xFF),
        (uint8_t)(enable ? 0x01 : 0x00),
    };
    spi_cmd_write(CMD_UART_CONFIG, args, sizeof(args));
    uart_armed_local = enable;

    float actual = (float)FPGA_HFOSC_HZ / (float)div;
    /* Integer baud — newlib-nano printf has no %f (the old %.0f printed nothing). */
    printf("[uart] config RX=LA%u TX=LA%u baud=%lu (div=%lu → %lu baud) %s\n",
           rx_ch, tx_ch, (unsigned long)baud, (unsigned long)div,
           (unsigned long)lroundf(actual), enable ? "armed" : "disabled");
    return 0;
}

void fpga_uart_disable(void) {
    spi_cmd_write(CMD_UART_DISABLE, NULL, 0);
    uart_armed_local = false;
    printf("[uart] disabled\n");
}

bool fpga_uart_active(void) { return uart_armed_local; }

int fpga_uart_write(const uint8_t *data, size_t len) {
    if (!data || len == 0) return -1;
    /* [CMD][len_lo][len_hi][N bytes] */
    uint8_t hdr[3] = { CMD_UART_WRITE,
                       (uint8_t)(len & 0xFF), (uint8_t)((len >> 8) & 0xFF) };
    cs_select();
    spi_write_blocking(SPI_PORT, hdr, sizeof(hdr));
    spi_write_blocking(SPI_PORT, data, len);
    cs_deselect();
    return 0;
}

int fpga_uart_status(uint16_t *rx_avail, uint8_t *flags) {
    uint8_t b[3] = {0};
    spi_cmd_read(CMD_UART_STATUS, NULL, 0, b, sizeof(b));
    /* rx_avail is a 9-bit count in [0, FPGA_UART_RX_FIFO]. A larger value can only be a
       bogus read — a floating MISO / unresponsive FPGA returns 0xFF for every byte, so
       avail comes back 0xFFFF.  Reject it (return -1) instead of reporting 65535 "bytes
       available", which made fpga_uart_read flood 0xFF at the client and tear the proxy
       down.  Report nothing on a bad read so the caller treats it as "no data / error". */
    uint16_t avail = (uint16_t)(b[0] | ((uint16_t)b[1] << 8));
    if (!fpga_uart_avail_ok(avail, &avail)) return -1;
    if (rx_avail) *rx_avail = avail;
    if (flags)    *flags    = b[2];
    return 0;
}

size_t fpga_uart_read(uint8_t *buf, size_t len) {
    if (!buf || len == 0) return 0;
    /* Read only what's actually available so the FIFO never underflows. A bad status
       read (unresponsive FPGA) reports no data rather than flooding garbage. */
    uint16_t avail = 0;
    if (fpga_uart_status(&avail, NULL) != 0) return 0;
    size_t n = (avail < len) ? avail : len;
    if (n == 0) return 0;
    uint8_t args[2] = { (uint8_t)(n & 0xFF), (uint8_t)((n >> 8) & 0xFF) };
    spi_cmd_read(CMD_UART_READ, args, sizeof(args), buf, n);
    return n;
}
