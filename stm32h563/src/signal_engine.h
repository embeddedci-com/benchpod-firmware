#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "fpga_config.h"   /* FPGA_PSRAM_*_BASE + derived FPGA_*_MAX_SAMPLES caps */

/* Size of the FPGA waveform BRAM in BYTES (gateware dac8551_engine.v has ADDR_W=12
 * -> 4096 byte addresses). Also bounds a few host-side scratch buffers. LOAD_WAVE
 * rejects a byte length > SIGNAL_BUF_SIZE. */
#define SIGNAL_BUF_SIZE 4096

/* Max DAC-waveform length in SAMPLES. DAC waveforms are 16-bit (DAC8551, 2
 * bytes/sample), so a single waveform is at most SIGNAL_BUF_SIZE/2 samples and
 * LOAD_WAVE then loads exactly SIGNAL_BUF_SIZE bytes. Bounding a sample count by
 * SIGNAL_BUF_SIZE directly (as if it were samples) silently asks for 2x the BRAM,
 * so the gateware rejects the load ("generate failed"); generate/measure clamp
 * their per-period / capture sample counts to SIGNAL_MAX_SAMPLES instead. */
#define SIGNAL_MAX_SAMPLES (SIGNAL_BUF_SIZE / 2u)

/* Max ADC-capture depth on v2 (gateware >= 16, DEEP ADC).  The samples stream into
 * the multi-MB PSRAM ADC region and are read back CHUNKED from PSRAM (never buffered
 * whole in MCU SRAM), so the cap is the region size, derived from the gateware map in
 * fpga_config.h — 2,097,152 samples (~5.2 s @ 0.4 MS/s) with the current 4 MB split.
 * The gateware ADC count is 24-bit to match.  (The small monolithic read-back buffer
 * used by the single-shot `read`/legacy paths is ADC_MONO_BUF_SAMPLES, decoupled from
 * this in command_handler.c.) */
#define ADC_CAP_MAX_SAMPLES FPGA_ADC_CAP_MAX_SAMPLES

typedef void (*capture_done_cb_t)(void);

void signal_engine_init(void);

/* Call from main loop: stops timed waveforms when their duration expires. */
void signal_engine_poll(void);

/* DAC waveform generators.
   freq in Hz, amplitude 0-127 (half-scale), offset 0-255 (DC bias).
   duration_ms = 0 means run continuously until dac_stop().
   sample_rate_hz selects the FPGA DAC sample clock; pass 0 to auto-pick the
   highest rate whose per-period sample count fits the buffer.  A lower
   sample rate reduces high-frequency clock feedthrough at the cost of a
   coarser (fewer-samples-per-period) waveform.
   Return 0 on success, -1 if parameters are out of range. */
int dac_generate_sine(float freq, uint8_t amplitude, uint8_t offset,
                      uint32_t duration_ms, float sample_rate_hz);
int dac_generate_square(float freq, uint8_t amplitude, uint8_t offset,
                        uint32_t duration_ms, float sample_rate_hz);
int dac_generate_sawtooth(float freq, uint8_t amplitude, uint8_t offset,
                          uint32_t duration_ms, float sample_rate_hz);

/* Play an arbitrary user-supplied waveform.
   loop=true: restart DMA from beginning each time (runs until dac_stop()).
   loop=false: play once. */
int dac_generate_arbitrary(const uint8_t *data, size_t len, bool loop);

/* As dac_generate_arbitrary(), but selects the DAC sample clock from a target
   rate in Hz.  Pass 0 to use the max rate (12 MSPS).  Use this to replay a
   captured trace at the same sample rate it was captured at, so the playback
   time-base matches the recording. */
int dac_generate_arbitrary_rate(const uint8_t *data, size_t len, bool loop,
                                float sample_rate_hz);

/* Deep replay (gateware >= v17): play `count` 16-bit samples ALREADY STAGED in
   PSRAM (top-anchored base, >=v18; overlays ADC on older gw), looping until dac_stop.  iCE40 streams the
   waveform straight from PSRAM (not the 4 KB LOAD_WAVE BRAM), so `count` can span
   the whole 8 MB region (up to FPGA_DAC_REPLAY_MAX_SAMPLES) — enough to replay a
   full stored ADC recording.  The caller must have staged the bytes into PSRAM and
   released the shared bus (psram_bus_release) so the iCE40 can read.  Pass 0 rate
   for the max clock. */
int dac_replay_psram(uint32_t count, float sample_rate_hz);

/* Connected iCE40 gateware version (CMD_VERSION, read at init).  Deep DAC replay
   (dac_replay_psram / CMD_START_DAC_PSRAM) requires >= 17. */
uint8_t signal_engine_fpga_version(void);

/* Live-read + refresh the cached gateware version over SPI (reflects a runtime
   flash-ice40). Returns true iff the FPGA answered; leaves the cache intact on
   failure. Quiet — for `status`; use fpga_ping() for the verbose bring-up path. */
bool signal_engine_refresh_version(void);

/* Minimum gateware version that supports streaming DAC replay from PSRAM. */
#define DAC_DEEP_REPLAY_MIN_GW  17u

/* Minimum gateware version with the burst-granular PSRAM bus arbiter (>=v18): deep
   DAC replay and an ADC/LA capture can run CONCURRENTLY in disjoint regions, and the
   region map is LA@front / ADC@4MB / DAC floating from the top. */
#define CONCURRENT_PSRAM_MIN_GW  18u

/* Minimum gateware version with the capture-tied DAC auto-stop (>=v21, OP_SET_DAC_STOP_AFTER
   0x14): the iCE40 cuts a concurrently-running DAC a firmware-set number of 24 MHz clk cycles
   after a capture's hardware t0, so the captured window shows the DAC switch off at a
   sample-precise point.  Older gateware does not decode the opcode. */
#define DAC_CAPTURE_STOP_MIN_GW  21u

/* Minimum gateware version with RUNTIME capture region bases (>=v22, SET_CAPTURE_BASES
   0x32): the ADC (and LA) PSRAM region base is programmable so the firmware can pack
   DAC/ADC/LA zones dynamically to fit 8 MB (docs/tri-capture-unified-psram.md). */
#define CAPTURE_BASES_MIN_GW  22u

/* Minimum gateware with the deterministic CLOSED-LOOP DAC control engine (>=v23,
   START_DAC_LOOP 0x15 + DAC_PROBE 0x16): each tick the iCE40 reads the live ADC, indexes
   the LOAD_WAVE curve as a LUT, damps + clamps, and drives the DAC — a control loop that
   depends on the ADC, e.g. a solar-panel/MPPT emulator (docs/dac-control-loop.md). */
#define DAC_CONTROL_LOOP_MIN_GW  23u

/* Arm the control loop: load `curve` (16-bit LE, up to 2048 pts) as the transfer
   function, then run the loop with Q15 damping `k`, output clamp [vmin,vmax] and control
   period `tick_div` (clk48 cycles).  STOP with fpga_stop_dac / dac_stop.  Returns -2 on
   gateware < v23, -3 when the RUNNING IMAGE has no loop engine (see below). */
int      dac_control_loop_start(const uint8_t *curve, size_t curve_len,
                                uint16_t k_q15, uint16_t vmin, uint16_t vmax, uint16_t tick_div);
/* Live loop DAC output code (DAC_PROBE), for telemetry. */
uint16_t fpga_dac_loop_probe(void);

/* Minimum gateware with a SELECTABLE LOOP INPUT SOURCE (>=v29, DAC_LOOP_SRC 0x1A +
   DAC_LOOP_IN_PROBE 0x1B): the loop's input can be the live ADC (closed loop), a host-held
   FIXED value, or an internal per-tick SWEEP — the last two drive the curve OPEN-loop, with
   the ADC and the analog input path out of the picture, which is how the DAC/output side is
   validated on its own (docs/dac-control-loop.md). */
#define DAC_LOOP_SOURCE_MIN_GW  29u
/* Input conditioner + over-range trip (DAC_LOOP_INMAP 0x1C, STATUS bit 6). */
#define DAC_LOOP_INMAP_MIN_GW   30u

/* Select where the running (or next-armed) loop takes its input from.  `src` is a
   dac_loop_src_t (0 = ADC, 1 = fixed, 2 = sweep), `in_fixed` the fixed value / sweep start,
   `sweep_step` the per-tick increment.  Persistent across arms and settable WHILE ARMED — a
   running loop picks the new source up on its next tick, which is what lets a client step
   through curve points without re-arming or re-uploading the curve.  Returns 0, -2 on
   gateware < v29, -3 when the running image has no loop engine. */
int      dac_loop_set_source(uint8_t src, uint16_t in_fixed, uint16_t sweep_step);
/* Program the loop's input map (gateware >= v30).  Derive the arguments with
   dac_loop_inmap_derive() rather than computing them at a call site — it is the one place
   that composes the bench description with this board's ADC calibration.
   Returns 0, -1 SPI, -2 gateware too old, -3 wrong FPGA image. */
int      dac_loop_set_inmap(uint16_t in_zero, int16_t in_gain, uint16_t in_trip,
                            bool map_en, bool trip_en);
/* True when the loop has latched its over-range trip and parked the output at vmin. */
bool     fpga_dac_loop_tripped(void);

/* The input value the loop's LAST TICK indexed the curve with (DAC_LOOP_IN_PROBE).  In a
   fixed/sweep run there is no ADC in the path, so this — not ADC_PROBE — is the loop's
   input.  Returns 0 on gateware < v29 (compare signal_engine_fpga_version() against
   DAC_LOOP_SOURCE_MIN_GW to tell that apart from a genuine 0 input). */
uint16_t fpga_dac_loop_input(void);

/* FPGA_FEATURES (0x18) bits — which optional block the running warmboot image carries. */
#define FPGA_FEATURE_CONTROL_LOOP  0x01u
#define FPGA_FEATURE_DEEP_REPLAY   0x02u
uint8_t  signal_engine_fpga_features(void);

/* Is the CLOSED-LOOP engine actually present in the image that is running right now?
   The two swappable images report the SAME GATEWARE_VERSION (only FEATURES differs), so a
   version test alone does NOT tell them apart: on the deep-replay image dac_loop is not
   instantiated at all (top_v2.v ties dac_loop_mode48 to 0), and START_DAC_LOOP is accepted
   into a register that drives nothing — the loop would report "armed" and silently never
   drive the DAC (DAC_PROBE reads 0 forever).  Every closed-loop entry point gates on this.
   Reads FPGA_FEATURES live over SPI, so call it from the hw worker (heavy gate held). */
bool     signal_engine_has_control_loop(void);

/* iCE40 multi-image warmboot: reconfigure the FPGA to another image in the config flash
   (0=closed-loop, 1=deep-DAC-replay) with NO reflash.  Blocks until CDONE + re-reads
   version/features.  Returns 0 ok, -1 bad image, -2 gw too old, -3 CDONE timeout. */
int      fpga_warmboot(uint8_t image);

/* Bring every gateware PSRAM master (deep-replay reader / closed loop / capture writer) to a
   safe IDLE boundary before firmware grabs the shared quad bus for a runtime reflash or OTA.
   Stops the DAC and waits (bounded) for STATUS to report no DAC/capture busy + BUSY high, so
   a bus grab never lands mid-burst and wedges the arbiter/PSRAM.  Call BEFORE psram_bus_acquire
   on any runtime path that can race a live master (image swap, flash-ice40, OTA commit). */
void     signal_engine_quiesce_psram_masters(void);

/* DAC8551 peak update rate — clk48/(3+51) ~= 889 kS/s since the DAC divider floor of 3
   (DAC_MIN_DIVIDER, DAC8551 frame timing; it was clk48/53 ~= 906 kS/s) — used as the bus-load
   term for a resident deep-DAC replay in the tri-capture bandwidth pre-check. */
#define DAC_MAX_RATE_HZ  889000u

/* Latch the runtime PSRAM capture bases into the iCE40 (0x32) for dynamic tri-capture
   zone allocation, and mirror them for read-back.  Call BEFORE arming CAPTURE; keeps the
   legacy fixed map on gateware < CAPTURE_BASES_MIN_GW.  LA is normally 0. */
int      fpga_set_capture_bases(uint32_t la_base, uint32_t adc_base);
uint32_t signal_engine_adc_cap_base(void);

/* Re-sync the firmware's mirrors of gateware registers after a RECONFIGURATION (the fabric
 * resets them; the mirrors do not).  Called by ice40_reflash_image() — the single point every
 * reconfiguration passes through.  See the definition for the desync this fixes. */
void signal_engine_on_gateware_reconfigured(void);
uint32_t signal_engine_dac_resident_bytes(void);

/* Arm (or disarm) the capture-tied DAC auto-stop for the NEXT capture: the iCE40 will cut a
   concurrently-running DAC `us` microseconds into that capture (0 = disarm / leave it running).
   Call it right BEFORE arming the capture; a no-op on gateware < DAC_CAPTURE_STOP_MIN_GW. */
void fpga_set_dac_stop_after_us(uint32_t us);

/* Minimum gateware with HARDWARE CO-TRIGGERED DAC START (>=v27, OP_DAC_ARM_ON_CAPTURE 0x19):
   defers the NEXT DAC start so it fires on the same cycle as the next capture's t0 — DAC sample 0
   == capture t0, sample-exact and jitter-free. Older gateware ignores the opcode (firmware falls
   back to the old sequential start, which the DAC free-runs at an arbitrary phase). */
#define DAC_COTRIG_MIN_GW  27u

/* Stage a co-trigger for the NEXT DAC start: after this, the following START_DAC / START_DAC_PSRAM
   latches its params (and, deep, primes its reader FIFO) but HOLDS the engine start until the next
   capture arm.  Returns true if armed (gw >= v27), false if the gateware can't co-trigger (caller
   then uses the sequential start).  Cancelled by fpga_stop_dac / dac_stop. */
bool fpga_dac_arm_on_capture(void);

/* Minimum gateware with GPIO_GET (0x43: the 12 live LA levels through a 2-flop synchroniser) and
   the CAPTURE TRIGGER (SET_TRIGGER 0x33 / TRIGGER_STATUS 0x34: an armed capture's producers wait
   in the fabric for rising/falling/high/low on one LA pin, and that cycle becomes t0 — for the
   DAC co-trigger and SET_DAC_STOP_AFTER too). */
#define GPIO_GET_MIN_GW         35u
#define CAPTURE_TRIGGER_MIN_GW  35u

/* Live LA levels, bit la-1.  0 ok, -1 SPI failure, -2 gateware < GPIO_GET_MIN_GW. */
int  fpga_gpio_get(uint16_t *levels);
/* Trigger for the NEXT capture arm (CAPTURE / LA_CAPTURE / START_CAPTURE): la 1..12, mode
   1 rising, 2 falling, 3 high, 4 low; mode 0 = off (la ignored).  It persists in the fabric until
   changed, so send mode 0 after every triggered capture.  0 ok, -1 bad args/SPI, -2 too old. */
int  fpga_set_trigger(unsigned la, uint8_t mode);
#define TRIGGER_STATUS_WAITING 0x01u   /* armed, condition not seen yet */
#define TRIGGER_STATUS_FIRED   0x02u   /* a triggered capture fired since the last arm */
int  fpga_trigger_status(uint8_t *status);   /* 0 ok, -1 SPI, -2 too old */
/* Push the in-flight capture's completion deadline out by `ms`: the trigger wait comes on top of
   the capture window.  Call right after the *_start() that set the deadline. */
void fpga_capture_extend_deadline_ms(uint32_t ms);
/* Abort a capture still waiting for its trigger: a count-0 re-arm (v35: an arm with no producer
   never waits), preceded by STOP_DAC when a co-triggered DAC start was staged for this capture
   (the abort is an untriggered t0 and would fire it), then trigger off. */
void fpga_capture_abort(void);

/* ---- gateware-derived feature set: ONE source of truth ----------------------
 * These flags are computed from the RUNNING iCE40 image (version + feature bits) and
 * were, until now, derived twice: once in cloud_client.c's `capabilities` announce and
 * once — implicitly, as a hardcoded literal — in command_handler.c's `status` reply.
 * The two drifted. `status` advertised only ["signal","gpio","power","swd","i2c_sensor",
 * "uart","la"], so a client on a DIRECT LAN/serial connection could not see deep replay,
 * the control loop or the co-trigger AT ALL, while a cloud client saw all of them from
 * the announce. That made the SDK skip hardware tests for features the pod demonstrably
 * had (hwe2e TestHW_V2_DeepReplay / TestHW_V2_DacCotriggerPhaseLock pass on the very pod
 * whose `status` denied them), silently giving the direct transport thinner coverage than
 * the cloud one.
 *
 * Both emitters now call signal_engine_caps(), so a new feature is advertised on every
 * transport or on none. */
typedef struct {
    bool deep_replay;     /* deep DAC replay streams from PSRAM (past the BRAM cap) */
    bool control_loop;    /* in-fabric DAC control loop is present in the running image */
    bool cotrig;          /* HW co-triggered DAC start: DAC sample 0 lands on capture t0 */
    bool loop_sources;    /* selectable loop INPUT source (live ADC / held / sweep) */
    bool loop_input_map;  /* affine input conditioner in front of the loop curve */
    bool gpio_read;       /* GPIO_GET: live LA pin levels (gpio read, la_pins levels) */
    bool capture_trigger; /* SET_TRIGGER / TRIGGER_STATUS: triggered captures */
} signal_engine_caps_t;

/* Fill *out with the running image's feature flags. Safe to call from any task. */
void signal_engine_caps(signal_engine_caps_t *out);

/* ---- deep-DAC PSRAM region bookkeeping (top-anchored, >=v18) ----
 * The waveform is staged at the TOP of PSRAM so an LA/ADC capture can use the space
 * below it.  Call _stage() once the total byte length is known (load_bin) to fix the
 * base; dac_replay_psram() then marks the region occupied; dac_stop() frees it. */
void     signal_engine_dac_psram_stage(uint32_t total_bytes);  /* fix base = TOP - bytes */
uint32_t signal_engine_dac_psram_base(void);                   /* byte base for stage + replay */
void     signal_engine_dac_psram_release(void);                /* region free again (dac_stop) */

/* Max LA (or ADC) samples that fit BELOW a resident deep-DAC waveform, 2 B/sample.
 * adc_in_capture=true bounds LA to the ADC region boundary (dual ADC+LA capture);
 * false lets LA use everything up to the DAC base (or the whole chip if none). */
uint32_t signal_engine_la_max_samples(bool adc_in_capture);
uint32_t signal_engine_adc_max_samples(void);

/* Stop DAC DMA output. */
void dac_stop(void);

/* PSRAM blocking capture: the iCE40 streams `samples` 16-bit samples into the
   external PSRAM, then the MCU reads them back over its own XSPI.  Spins until
   done; use the start/poll pair below from an event loop instead.  Returns 0 on
   success. */
int  adc_capture_psram(uint16_t *out16, size_t samples, float sample_rate_hz);

/* v2 unified capture: arm the ADC and raw 12-ch LA producers TOGETHER (one
   trigger, shared t0) and read both PSRAM regions back.  adc_count==0 => ADC not
   captured; la_count==0 => LA not captured.  out_adc receives adc_count 16-bit ADC
   samples; out_la receives la_count 16-bit LA words (low 14 bits = LA1..LA14);
   either out may be NULL if its count is 0.  Dividers are in 24 MHz clk ticks.
   Blocking; returns 0 on success, -1 on error/timeout/overflow. */
int  fpga_dual_capture(uint16_t *out_adc, uint16_t adc_count, uint16_t adc_div,
                       uint16_t *out_la,  uint16_t la_count,  uint16_t la_div);
/* Async form of fpga_dual_capture for the streaming server path: arm then poll.
   Counts are 24-bit (deep ADC + deep LA, gateware >= 16); dividers stay 16-bit. */
int  fpga_dual_capture_start(uint32_t adc_count, uint16_t adc_div,
                             uint32_t la_count, uint16_t la_div);
/* Small/legacy monolithic read-back into caller RAM (bounded by ADC_MONO_BUF_SAMPLES);
   deep captures use the chunked fpga_capture_psram_* path below instead. */
int  fpga_dual_capture_poll(uint16_t *out_adc, uint16_t adc_count,
                            uint16_t *out_la,  uint16_t la_count);
/* Rate-based async arm; returns the achieved ADC/LA sample rates in Hz. */
int  fpga_dual_capture_start_hz(uint32_t adc_samples, float adc_req_hz,
                                uint32_t la_samples, float la_req_hz,
                                float *adc_rate_hz, float *la_rate_hz);

/* ---- Deep unified capture: CHUNKED PSRAM read-back (no whole-capture SRAM buffer).
   After fpga_dual_capture_start[_hz](), poll fpga_capture_psram_wait() until it
   returns 1 (CAP_DONE — it drains the writer and GRABS+HOLDS the shared bus).  Then
   stream the combined result out chunk-by-chunk with fpga_capture_psram_read16():
   global sample indices [0, adc_samples) come from the ADC region, the rest from the
   LA region — matching the old combined "ADC then LA, all 16-bit" wire format the
   server splits at adc_samples.  Call fpga_capture_psram_release() when done.  This
   is the deep-ADC analogue of the fpga_la_psram_* trio. */
int  fpga_capture_psram_wait(void);
/* true = the ADC no-write sentinel survived => the iCE40 never wrote the ADC region
   (capture datapath/timing fault, NOT the analog ADC).  Bus must be held. */
bool fpga_capture_adc_sentinel_survived(void);
/* Read `n` 16-bit samples starting at global `sample_index` into out16, spanning the
   ADC region then the LA region at the `adc_samples` boundary.  The caller (bulk
   sender) must clamp a chunk so it does NOT straddle the boundary.  Bus held. */
int  fpga_capture_psram_read16(size_t sample_index, uint16_t *out16, size_t n,
                               size_t adc_samples);
/* Release the shared bus after the chunked read-back is complete. */
void fpga_capture_psram_release(void);

/* ---- v2 (PSRAM) NON-BLOCKING capture/measure --------------------------------
   The iCE40 captures autonomously into PSRAM, so the MCU need not spin while it
   runs.  *_start() arms the iCE40 (hands it the shared bus) and returns at once;
   poll repeatedly with adc_capture_psram_poll() until it reports completion.
   This keeps the network task servicing lwIP while a capture is in flight.

   adc_capture_psram_start: arm an ADC-only capture of `samples` 16-bit samples.
   measure_psram_start:     arm a phase-locked DAC waveform + ADC capture.  The
                            DAC plays one period of `waveform` across the capture
                            window (the v2 gateware ties the DAC period to the
                            capture count for MEASURE), at `freq` (or the forced
                            sample rate if sample_rate_hz > 0).
   adc_capture_psram_poll:  1 = done (out16 holds `samples` LE uint16), 0 = still
                            running, -1 = timeout / read-back error.
   Both *_start() return 0 when armed, -1 on bad params / non-v2 gateware. */
int adc_capture_psram_start(size_t samples, float sample_rate_hz);
int measure_psram_start(const char *waveform, float freq, uint8_t amplitude,
                        uint8_t offset, size_t samples, float sample_rate_hz);
int adc_capture_psram_poll(uint16_t *out16, size_t samples);

/* Blocking v2 measure (DAC waveform + ADC->PSRAM): measure_psram_start() then
   spin until done, stopping the DAC at the end.  For the synchronous SCPI path;
   the JSON path uses the start/poll pair so it never blocks the net task. */
int measure_psram(uint16_t *out16, const char *waveform, float freq,
                  uint8_t amplitude, uint8_t offset, size_t samples,
                  float sample_rate_hz);

/* True once the SPI1 GPDMA channels inited OK (big transfers use DMA). */
bool signal_engine_dma_active(void);

/* SPI-clock diagnostics: set SCK live (div = 2,4,..256; returns SCK Hz or 0),
   and loop-test read reliability at the current clock (prints CLEAN/CORRUPT). */
uint32_t signal_engine_spi_set_prescaler(uint32_t div);
void     signal_engine_spi_diag(int n);

/* Direct-over-SPI ADC read (CMD_ADC_PROBE): live sample with NO PSRAM/CDC in the
   path.  _diag reads n samples + prints min/max/span (STATIC vs VARYING). */
int      signal_engine_adc_spi(uint16_t *out);
void     signal_engine_adc_spi_diag(int n);

/* ADC->PSRAM capture-path self-test (CMD_CAPTURE_TEST): the iCE40 streams a known
   +0x0101 ramp through the real capture datapath (24->48 CDC + writer + PSRAM +
   read-back).  _selftest returns 0=PASS / -1=FAIL (first bad index in *bad_idx),
   distinguishing an iCE40 capture/timing fault from an actual ADC problem. */
int      signal_engine_capture_test_mode(int mode);   /* 0=ADC 1=ramp@24MHz 2=ramp@48MHz */
int      signal_engine_capture_selftest(int n, int *bad_idx);

/* Layered PSRAM datapath self-test: STM32<->PSRAM, iCE40->PSRAM write, and (if that
   fails) whether the iCE40 even reaches the /CE net.  Run at boot + console. */
int      signal_engine_psram_bus_reach(uint8_t *lines);  /* # of 6 lines reaching PSRAM, <0=err */
void     psram_boot_selftest(void);

/* Cached psram_boot_selftest result, surfaced on the console 'status' line. */
#define PSRAM_ST_NOT_RUN           (-1)
#define PSRAM_ST_OK                0
#define PSRAM_ST_STM32_FAIL        1
#define PSRAM_ST_ICE40_WRITE_FAIL  2
#define PSRAM_ST_ICE40_REACH_FAIL  3
#define PSRAM_ST_SKIP              4
int          psram_selftest_result(void);   /* one of PSRAM_ST_* */
const char  *psram_selftest_str(void);       /* human-readable for 'status' */
bool         signal_engine_psram_operable(void);        /* status.psram_ok — true iff last selftest passed */
void         signal_engine_psram_report_wedge(void);    /* mark inoperable after a runtime capture wedge */
void         psram_boot_selftest_with_recovery(void);   /* boot selftest + auto-reflash the iCE40 on fault */

/* ---- Diagnostics — exposed for console debug commands ---------------------*/

/* Send a SPI PING to the FPGA, retry up to attempts times.  Logs each attempt
   on UART1.  Returns 0 if any attempt got the expected 0xA5 reply, -1 if all
   attempts failed.  When 0 is returned, *out_version (if non-NULL) is filled
   in by an immediate follow-up VERSION read. */
int fpga_ping(int attempts, uint8_t *out_version);

/* Read the FPGA STATUS byte over SPI.  Returns 0 on plausibly-valid response
   (top 5 bits clear), -1 on garbage (top 5 bits set indicates miswired MISO).
   *out_status is always written.  Useful as a "is the link still alive?" probe
   from the console after init. */
int fpga_status_read(uint8_t *out_status);

/* Force the DAC to output a single byte value continuously (loaded as a
   1-sample waveform).  Useful for DC-level debugging with a multimeter on
   the analog side.  divider selects the DAC clock rate (sample_rate =
   FPGA_HFOSC_HZ / divider).  Use a large divider (e.g. 240 → 100 kHz) when
   debugging with a 24 MS/s logic analyzer that can't resolve 12 MHz cleanly.
   dac_stop() halts it. */
int dac_set_constant(uint8_t value, uint32_t divider);

/* Single-sample ADC capture.  Returns the byte the ADC pins are showing
   after one capture cycle (subject to ADC chip pipeline delay).  Useful
   paired with dac_set_constant() to verify the analog DAC→ADC path with
   no waveform sequencing involved.
   Returns 0 on success, -1 on timeout. */
int adc_probe_one(uint8_t *out_value);

/* ===========================================================================
 * Logic-Analyzer (LA) GPIO bank — bidirectional pins owned by the iCE40.
 *
 * The stepper pulse generator and the SWD bit-bang engine used to run on the
 * RP2350's PIO driving RP-native GPIOs.  They now run inside the FPGA gateware
 * (stepper_engine.v / swd_engine.v) driving the FPGA's LA bank, and the RP just
 * issues high-level commands over SPI.  The host addresses pins by 1-based LA
 * channel index (LA1..LA14); the firmware maps LA<n> → FPGA la[n-1].  See
 * docs/API.md and ice40/README.md for the channel→pin table.
 * ========================================================================== */

#define LA_CHANNEL_MIN       1u
#define LA_CHANNEL_MAX       14u

/* Stepper bounds.  steps + delay_us are 16-bit fields in the gateware
 * (cmd_dispatch GPIO_STEP: [channel][steps(2)][delay_us(2)]), so both cap at
 * 0xFFFF.  delay_us is microseconds per half-phase. */
#define LA_STEP_MIN_DELAY_US 4u
#define LA_STEP_MAX_DELAY_US 65535u
#define LA_STEP_MAX_STEPS    65535u

/* Drive an LA channel: mode 0 = low, 1 = high, 2 = high-Z.
   Returns 0 on success, -1 if channel is outside LA1..LA14. */
int fpga_la_set(unsigned channel, int mode);

/* Drive the iCE40 onboard status LEDs: bit0 = green, bit1 = yellow,
   bit2 = red.  Visual indicator only. */
void fpga_set_led(uint8_t mask);

/* Start a non-blocking step train on an LA channel: `steps` pulses, each high
   for delay_us then low for delay_us (a full step period is 2*delay_us).  The
   FPGA runs the train autonomously.
   Returns 0 on success, -1 bad channel, -2 a step train is already running,
   -3 steps == 0 or > LA_STEP_MAX_STEPS,
   -4 delay_us outside [LA_STEP_MIN_DELAY_US, LA_STEP_MAX_DELAY_US]. */
int fpga_la_step(unsigned channel, uint32_t steps, uint32_t delay_us);

/* True while a step train is running (reads the FPGA STATUS STEP_BUSY bit). */
bool fpga_la_step_busy(void);

/* ---- LA I/O-bank voltage (TPS2116 mux) -----------------------------------
   The LA bank VCCIO (iCE40 bank 0 — supplies ALL of LA1..LA14) is switched
   between 3.3 V and 1.8 V by a TPS2116 power-mux (U8): PG3 (LA_VCCIO_SWITCH)
   drives PR1 (the select), PG4 (LA_VCCIO_ST) reads the open-drain ST output.
   PR1 and ST each have a 100 k pull-up to +3V3 (R151/R152).

   The MODE pin (U8 pin 5) moved from GND to +3V3 in rev3, which changes the
   part's behaviour completely — from the datasheet truth table:

     v3, MODE >= 1 V  (manual mode)
        PR1 high  ->  ST high  ->  VOUT = VIN1 = +3.3 V
        PR1 low   ->  ST low   ->  VOUT = VIN2 = +1.8 V
     v2, MODE <= 0.35 V
        PR1 high  ->  ST low   ->  VOUT = Hi-Z (SHUTDOWN)
        PR1 low   ->            ->  VOUT = the HIGHER input = +3.3 V

   So on v2 there is no 1.8 V setting at all: what the firmware used to call
   "1.8 V" (PG3 high) actually shut the mux down and left the bank rail floating
   on its own decoupling.  la_vccio_set_mv() refuses 1800 on a v2 board, and the
   PG3 polarity for 3.3 V is inverted between the two revisions.  On v3 the Hi-Z
   power-on default (PR1 pulled high) is a genuine +3.3 V, and ST is a real
   readback of which input is live.

   The level MUST be chosen by the host before ANY LA-bank operation (la /
   la_capture / dap_start / uart_proxy / i2c-sensor); until then the bank is at
   an undefined level for the DUT and those commands are refused.  Selection is
   tracked in software (LA_VCCIO_UNSET at boot) and enforced independently of the
   hardware default so the host makes an explicit choice. */
#define LA_VCCIO_UNSET 0
#define LA_VCCIO_1V8   1800
#define LA_VCCIO_3V3   3300

/* Configure PG3 (Hi-Z -> the hardware default for this revision) and PG4
   (input); software state = unset. */
void la_vccio_init(void);
/* Select the LA bank voltage; mv must be LA_VCCIO_1V8 or LA_VCCIO_3V3.
   Drives PG3 with the revision's polarity and settles the rail.
   Returns 0 on success, -1 on an invalid millivolt value, -2 for 1800 on a v2
   board (the mux cannot produce 1.8 V there — see above). */
int  la_vccio_set_mv(int mv);
/* Current selection: LA_VCCIO_UNSET (not chosen yet), LA_VCCIO_1V8, or LA_VCCIO_3V3. */
int  la_vccio_get_mv(void);
/* Raw TPS2116 status pin (PG4/ST) level: 0 or 1. */
int  la_vccio_status_pin(void);
/* ST decoded to a rail voltage on v3 (LA_VCCIO_3V3 or LA_VCCIO_1V8); 0 on v2,
   where ST low is ambiguous between "1.8 V" and "shut down". */
int  la_vccio_readback_mv(void);

/* ---- SWD over the FPGA (OpenOCD remote_bitbang offloaded to gateware) ------
   swclk/swdio are LA channels (1..12).  nRESET is NOT an LA channel any more:
   rev3 gave the pod a dedicated /NRST_CONTROL pin (PF4 -> J1 pin 22), and
   swd_ll_nreset() drives that instead of the gateware — see nrst_ctrl.h.
   These mirror the old swd_probe_* API so command_handler/console need minimal
   changes. */
int  fpga_swd_arm(unsigned swclk, unsigned swdio);
void fpga_swd_disarm(void);
bool fpga_swd_armed(void);

/* Main-loop hook: auto-disarm a silent SWD session after an inactivity timeout
   (backstop for a wedged session whose TCP close never fired). */
void fpga_swd_poll(void);

/* Feed raw remote_bitbang bytes: forwards wire-driving bytes to the FPGA and
   reads sample replies back.  Drop-in for the old swd_probe_feed():
   stops WITHOUT consuming at '{' (sets *exit_to_json); consumes 'Q' (sets
   *quit); each 'c' sample appends an ASCII '0'/'1' to `reply` (capped at
   reply_cap so the caller can flush and resume).  Returns bytes consumed;
   *reply_len, *exit_to_json and *quit are always written. */
size_t fpga_swd_feed(const uint8_t *in, size_t len,
                     char *reply, size_t reply_cap, size_t *reply_len,
                     bool *exit_to_json, bool *quit);

/* ===========================================================================
 * Emulated I2C sensor — generic FPGA-backed I2C target driven over SPI.
 *
 * The FPGA is sensor-agnostic: it serves a 256-byte register image (filled by
 * this firmware) as an I2C slave on two LA channels, captures DUT writes, and
 * runs a configurable conversion/busy-bit handshake.  Sensor-specific models
 * (sensor_bmp280.c, …) build the register image and pick the handshake config.
 * SDA/SCL are 1-based LA channel indices (LA1..LA14).
 * ========================================================================== */

typedef struct {
    bool     armed;
    uint16_t xfer_count;     /* completed I2C transactions */
    uint16_t wr_count;       /* register bytes written by the DUT */
    uint8_t  last_wr_addr;   /* last register the DUT wrote */
    uint8_t  last_wr_val;    /* value of that last write */
} i2c_sensor_status_t;

/* Arm the I2C target: 7-bit address, SDA/SCL LA channels, and the generic
   conversion handshake (writing trig_reg sets busy_mask in busy_reg for
   conv_us microseconds).  Pass conv_us=0 / busy_mask=0 to disable the
   handshake.  Returns 0 on success, -1 on bad channel. */
int fpga_i2c_sensor_config(uint8_t addr7, unsigned sda_ch, unsigned scl_ch,
                           bool enable, uint8_t trig_reg, uint8_t busy_reg,
                           uint8_t busy_mask, uint16_t conv_us);

/* Disarm the I2C target and release its SDA channel to high-Z. */
void fpga_i2c_sensor_disable(void);

/* Load `len` bytes of the register image starting at `start_addr` (0..255). */
int fpga_i2c_load_regs(uint8_t start_addr, const uint8_t *data, size_t len);

/* Read `len` register bytes back (shows what the DUT has written too). */
int fpga_i2c_read_regs(uint8_t start_addr, uint8_t *buf, size_t len);

/* Read the I2C target status block (transaction/write counters, last write). */
int fpga_i2c_sensor_status(i2c_sensor_status_t *out);

/* Raw I2C-bus logic-analyzer capture: sample SDA/SCL into `buf` as packed
   bytes (4 samples/byte: scl,sda per sample, oldest in the high nibble).
   `bytes` packed bytes are captured at sample_rate_hz (0 = max rate); blocks
   for the capture window then reads the buffer back.  Returns 0 on success. */
int fpga_i2c_la_capture(uint8_t *buf, size_t bytes, float sample_rate_hz);

/* ---- deep multi-channel LA capture into PSRAM ----------------------------
   The iCE40 streams `samples` 14-channel LA samples (2 little-endian bytes each:
   byte 2k = LA1..LA8, byte 2k+1 = {00,LA9..LA14}) straight into the shared PSRAM, so the depth
   is bounded by PSRAM (up to LA_PSRAM_MAX_SAMPLES) rather than the 4 KB trace
   buffer.  The MCU reads the region back over its own XSPI in chunks.  Mirrors
   the async ADC PSRAM capture: arm with _start(), poll _wait() (it grabs the
   shared bus on completion), then read chunks with fpga_la_psram_read() and
   release the bus with fpga_la_psram_release() when done. */
/* 24-bit gateware sample count + streaming read-back => the LA capture can fill the
   whole LA PSRAM region.  Derived from the gateware map in fpga_config.h so it can
   never drift from the bases: (0x800000 - FPGA_PSRAM_LA_BASE) / 2 = 2,097,152 samples
   with the current 4 MB ADC / 4 MB LA split. */
#define LA_PSRAM_MAX_SAMPLES FPGA_LA_PSRAM_MAX_SAMPLES

/* Arm a deep LA→PSRAM capture of `samples` samples at sample_rate_hz (0 = max).
   The sample rate is clamped so the writer keeps up (see the .c).  Hands the
   shared bus to the iCE40.  Returns 0, or -1 (no v2>=8 gateware / bad count). */
int fpga_la_capture_psram_start(size_t samples, float sample_rate_hz);

/* Poll an armed capture.  Returns 1 = done (the shared bus has been ACQUIRED for
   the MCU; read it back then release), 0 = still capturing, -1 = timeout. */
int fpga_la_capture_psram_wait(void);

/* Read `n` bytes of the captured trace back from PSRAM at `offset` (bus must be
   held — i.e. after _wait() returned 1).  Returns 0 on success. */
int fpga_la_psram_read(uint32_t offset, uint8_t *buf, uint32_t n);

/* Release the shared bus back to the iCE40 after the read-back is complete. */
void fpga_la_psram_release(void);

/* ===========================================================================
 * UART proxy — FPGA-backed soft UART (8N1) on two LA channels.
 *
 * The FPGA serialises/deserialises frames on the chosen LA channels; this
 * firmware pushes TX bytes and drains RX bytes over SPI and bridges them to the
 * console (`uart-proxy`) or a TCP client (`uart_proxy_start`).  RX/TX are
 * 1-based LA channel indices (LA1..LA14).
 * ========================================================================== */

/* Soft-UART RX FIFO depth (uart_engine.v). rx_avail is a 9-bit count in [0, 256];
   any larger value is a bogus SPI read (floating MISO / unresponsive FPGA) and must
   never be treated as real data — see fpga_uart_status/read. */
#define FPGA_UART_RX_FIFO       256u

/* Validate a raw CMD_UART_STATUS rx_avail count. A plausible FIFO count is
   <= FPGA_UART_RX_FIFO; a floating MISO / unresponsive FPGA reads 0xFFFF, which must
   be rejected so fpga_uart_read doesn't flood 0xFF at the client. Returns true and
   (when out != NULL) writes the count when plausible. Pure — unit-tested. */
static inline bool fpga_uart_avail_ok(uint16_t raw_avail, uint16_t *out) {
    if (raw_avail > FPGA_UART_RX_FIFO) return false;
    if (out) *out = raw_avail;
    return true;
}

/* CMD_UART_STATUS flag bits (the `flags` byte from fpga_uart_status). */
#define UART_STATUS_TX_FULL     (1u << 0)
#define UART_STATUS_TX_EMPTY    (1u << 1)
#define UART_STATUS_RX_OVERFLOW (1u << 2)
#define UART_STATUS_ARMED       (1u << 3)

/* Arm the soft UART on the given RX/TX LA channels at `baud` (the firmware
   computes the FPGA bit-period divisor).  enable=false configures without
   arming.  Returns 0 on success, -1 bad channel/baud, -2 a proxy is already
   active (single-owner). */
int fpga_uart_config(unsigned rx_ch, unsigned tx_ch, uint32_t baud, bool enable);

/* Disarm the UART and release the TX channel to high-Z. */
void fpga_uart_disable(void);

/* True while a UART proxy owns the engine (console or a TCP client). */
bool fpga_uart_active(void);

/* Queue `len` bytes into the FPGA TX FIFO. Returns 0 on success. */
int fpga_uart_write(const uint8_t *data, size_t len);

/* Read the UART status: RX bytes available + flag bits (see UART_STATUS_*). */
int fpga_uart_status(uint16_t *rx_avail, uint8_t *flags);

/* Drain up to `len` bytes from the FPGA RX FIFO into `buf`; returns the count
   actually read (0 if none available). */
size_t fpga_uart_read(uint8_t *buf, size_t len);
