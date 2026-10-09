/*
 * command_handler_capture.c — captures and the paced bulk read-back.
 *
 *   capture, stream, measure        16-bit ADC into PSRAM, read back from the RAM pool
 *   capture_dual, la_capture        deep ADC/LA captures, streamed chunked out of PSRAM
 *   capture_read                    resume a stalled capture_dual / la_capture read-back
 *   test, sensor_regs, sensor_la    synthetic and I2C-sensor byte read-backs
 *
 * The heavy handlers arm the capture and return; capture_poll (from command_handler_poll) waits
 * for completion and streams the reply through the bulk sender, paced by the connection's send
 * ring. Runs on the hw worker task, like the rest of command_handler.
 */
#include "command_handler.h"
#include "command_handler_internal.h"
#include "at_driver.h"
#include "bp_json.h"
#include "bp_err.h"
#include "signal_engine.h"
#include "stm32h5xx_hal.h"  /* HAL_GetTick: bulk stall timer */
#include "psram.h"          /* psram_bus_acquire for a resumed read-back */
#include "psram_alloc.h"
#include "fpga_config.h"    /* FPGA_PSRAM_* region map, caps */
#include "psram_regions.h"  /* capture_read: is the capture still in PSRAM */
#include "adc_pool.h"       /* the RAM sample buffer, shared with SCPI and the console */
#include "b64url.h"
#include "la_pins.h"        /* capture-trigger parsing/messages */
#include "sensor_sim.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "pico/time.h"

#define CHUNK_SAMPLES    256

/* JSON parsing is shared (see bp_json.h); same aliases as command_handler.c. */
#define json_get_value      bp_json_get

/* ---- ADC buffer and chunk sender ---- */

/* The RAM sample buffer is the shared pool (adc_pool.h): the shallow `capture`/`stream`/`test`
   read-back (v2 captures are 16-bit, MCP33131) and the `load`/`load_bin` staging for `replay` use
   all of it, and the sensor/I2C-LA byte reads (adc_cmd_buf) its scratch region. DEEP captures
   (`capture_dual`) stream CHUNKED straight out of PSRAM via the bulk sender and never touch it.
   Sizing it to ADC_CAP_MAX_SAMPLES (the multi-MB PSRAM region) would ask for several MB of SRAM
   the STM32H563 does not have. SCPI and the console share the pool under the same heavy gate. */
#define ADC_MONO_BUF_SAMPLES ADC_POOL_SAMPLES
#define adc_buf16            adc_pool
#define adc_cmd_buf          adc_pool_scratch()
_Static_assert(ADC_POOL_SCRATCH_BYTES >= SIGNAL_BUF_SIZE, "adc_cmd_buf needs SIGNAL_BUF_SIZE bytes");

/* Static to avoid stack pressure; single TCP client means no reentrancy.
   Sized for a chunk of 16-bit decimal values (up to 5 digits + comma each). */
static char chunk_buf[CHUNK_SAMPLES * 7 + 48];


/* ---- v2 async PSRAM capture/measure ----
   On the v2 gateware the iCE40 captures autonomously into PSRAM, so the heavy
   handlers just arm it and return; command_handler_poll() polls for completion
   (keeping the net task servicing lwIP meanwhile), reads the samples back, and
   sends the response.  The heavy gate stays claimed across polls so a second
   client can't race the in-flight capture. */
static struct {
    bool   active;
    int    conn_id;
    size_t samples;
    bool   stop_dac;   /* measure: stop the DAC sequencer once captured */
    bool   b64;        /* reply dense samples as base64 ("enc":"b64", see bulk.b64) */
} v2cap;

/* ---- v2 async deep LA→PSRAM capture ----
   Like v2cap but for LA_CAPTURE: arm the iCE40, poll for completion in
   command_handler_poll(), then stream the trace back from PSRAM in chunks
   (bulk_begin_psram).  `bytes` = samples * 2. */
static struct {
    bool   active;
    int    conn_id;
    size_t bytes;
    bool   stop_dac;   /* HW-cut a concurrently-running DAC during this capture; free it after */
    uint32_t rate_hz;  /* ACHIEVED rate (whole divider, maybe burst-capped), reported as la_rate_hz */
} lacap;

/* ---- v2 async unified simultaneous capture (CMD_CAPTURE 0x31) ----
   Arms the ADC + raw-LA producers off ONE trigger; command_handler_poll() reads
   BOTH regions back into adc_buf16 (ADC first, then the LA words) and streams the
   combined array with bulk_begin.  The server splits it at adc_samples. */
static struct {
    bool   active;
    int    conn_id;
    size_t adc_samples;
    size_t la_samples;
    float  adc_rate_hz;   /* ACHIEVED rates (24MHz/divider), not the requested ones — */
    float  la_rate_hz;    /* the server needs these for a correct, aligned timebase.   */
    bool   stop_dac;      /* HW-cut a concurrently-running DAC during this capture; free it after */
    bool   b64;           /* reply the ADC region as base64 ("enc":"b64", see bulk.b64) */
} dualcap;

/* ---- Resume support for a stalled capture_dual read-back ----
   The captured ADC+LA data lives in PSRAM (ADC_BASE / LA_BASE) until the NEXT capture
   overwrites it, so if the read-back stalls the host can RESUME from where it left off
   (`capture_read` below) instead of re-triggering — the samples are still there, no need to
   re-capture.  We remember the last completed capture's geometry + achieved rates here.
   Invalidated whenever a new capture arms (its regions get overwritten). */
static struct {
    bool     valid;
    size_t   adc_samples;
    size_t   la_samples;
    uint32_t adc_rate_hz;
    uint32_t la_rate_hz;
} last_cap;

/* ---- Poll-driven, sndbuf-paced bulk sample sender ----
   A capture/measure/test response is a JSON array of up to 4096 samples — far
   bigger than the ~5.8 KB TCP send buffer.  Blasting every chunk in one call
   overran it: tcp_write returned ERR_MEM and the connection got aborted
   mid-response.  Instead the array is delivered chunk-by-chunk across net_poll
   cycles, each chunk gated on the free send-buffer space (at_send_avail), so
   lwIP drains acked data between polls and the buffer is never overrun.  Only
   one bulk send is in flight at a time (the heavy gate guarantees it); the gate
   is released once the final chunk has been queued. */
static struct {
    bool     active;
    int      conn_id;
    size_t   total;       /* samples (or bytes, LA-byte path) to send */
    size_t   sent;        /* samples/bytes sent so far */
    bool     is16;        /* emit 16-bit values (2 B/sample) vs 8-bit */
    bool     from_psram;  /* read each chunk from PSRAM vs RAM */
    size_t   adc_samples; /* deep unified (from_psram && is16): the ADC/LA region
                             boundary — samples [0,adc_samples) come from the ADC PSRAM
                             region, the rest from the LA region.  0 for the other paths. */
    uint32_t adc_rate_hz; /* achieved rates (integer Hz, 0 = omit) reported in the */
    uint32_t la_rate_hz;  /* first reply chunk so the server can label the timebase */
    int      la_last;     /* LA run-length carry: last word emitted in the LA region
                             (-1 = none yet) so only TRANSITIONS go on the wire */
    bool     trig_present;/* this capture was triggered: the LAST chunk carries "trigger":{...} */
    la_trigger_t trig;
    bool     trig_fired;
    bool     b64;         /* dense 16-bit chunks carry "b64":"<base64url of little-endian
                             uint16>" instead of "data":[decimal,...] (see wants_b64) */
    uint32_t progress_ms; /* HAL_GetTick() of the last chunk the connection took */
} bulk;

/* A client that stays connected but stops reading held the PSRAM bus and the busy gate for as
   long as it stayed connected (a dead tunnel's ring never drains either).  No progress for this
   long ends the send. */
#define BULK_STALL_MS 30000u

/* ---- compact dense read-back ("enc":"b64", advertised as caps[] "capture_b64") ----
   Decimal JSON costs up to 6 B per 16-bit sample ("65535,") plus a snprintf per sample, and
   that text, not the capture, was the read-back bottleneck (2M ADC samples = ~12 MB).  Base64url
   (unpadded, RFC 4648 §5) of the raw little-endian words is 8/3 B per sample, a fixed cost, and
   stays inside the same newline-delimited JSON chunk, so the LAN socket, the USB console and the
   cloud tunnel all carry it unchanged.  It is OPT-IN per request: a client that does not send
   "enc":"b64" (older SDKs, the server) keeps getting "data":[...].  Only the dense region
   changes; LA run-length frames ("la_edges") are already compact and stay as they are. */
static bool wants_b64(const char *json) {
    char enc[8] = {0};
    return json_get_value(json, "enc", enc, sizeof(enc)) && strcmp(enc, "b64") == 0;
}
/* Worst-case first-chunk header: {"status":"ok","bits":16,"adc_rate_hz":NNNNNNNNNN,
   "la_rate_hz":NNNNNNNNNN,"b64":" is 91 B. */
#define BULK_B64_HEADER_MAX 96u

/* ---- capture trigger (gateware >= CAPTURE_TRIGGER_MIN_GW) -----------------
   A triggered capture's producers load on the arm but do not sample until the fabric sees
   the condition, so the completion poll alone cannot tell "still waiting" from "slow": that
   is what trigwait is for.  It runs the trigger_timeout_ms budget and aborts the capture
   with the `trigger timeout:` error when the condition never arrives.  trig_reply carries the
   outcome from the completing capture to the bulk reply that follows it. */
static struct {
    bool            active;
    int             conn_id;
    la_trigger_t    t;
    absolute_time_t deadline;     /* trigger_timeout_ms after the arm */
    absolute_time_t next_check;   /* TRIGGER_STATUS is an SPI read: don't do it every pass */
    bool            fired;
} trigwait;
#define TRIGWAIT_POLL_US  5000

static struct { bool present; la_trigger_t t; bool fired; } trig_reply;

/* The reply footer grows by ~46 B ("trigger":{"la":12,"edge":"falling","fired":true}) on a
   triggered capture; bulk_pump reserves that on top of its usual framing budget. */
static size_t bulk_footer_reserve(void) { return bulk.trig_present ? 112u : 64u; }

static int bulk_emit_trigger(char *buf, size_t cap) {
    if (!bulk.trig_present) return 0;
    return snprintf(buf, cap, ",\"trigger\":{\"la\":%u,\"edge\":\"%s\",\"fired\":%s}",
                    (unsigned)bulk.trig.la, la_edge_name(bulk.trig.edge),
                    bulk.trig_fired ? "true" : "false");
}

/* Attach the trigger the just-finished capture ran with (if any) to this reply. */
static void bulk_take_trigger(void) {
    bulk.trig_present  = trig_reply.present;
    bulk.trig          = trig_reply.t;
    bulk.trig_fired    = trig_reply.fired;
    trig_reply.present = false;
}

/* Parse "trigger" / "trigger_timeout_ms" and, when present, gate on the gateware version and
   program SET_TRIGGER.  Call it AFTER the rest of the request validates and BEFORE the capture
   is armed — the fabric latches the trigger at the arm.  false = the error was already sent. */
static bool capture_trigger_begin(int conn_id, const char *json, la_trigger_t *out) {
    char err[LA_PINS_ERR_MAX];
    trig_reply.present = false;   /* a previous capture's outcome must never ride this reply */
    if (!la_trigger_parse(json, out, err, sizeof(err))) { send_error(conn_id, err); return false; }
    if (!out->present) return true;
    uint8_t ver = signal_engine_fpga_version();
    if (ver < CAPTURE_TRIGGER_MIN_GW) {
        snprintf(err, sizeof(err),
                 "capture triggers need gateware v%u or newer (this pod runs v%u)",
                 (unsigned)CAPTURE_TRIGGER_MIN_GW, (unsigned)ver);
        send_error(conn_id, err);
        return false;
    }
    if (fpga_set_trigger(out->la, (uint8_t)out->edge) != 0) {
        send_error(conn_id, "could not arm the capture trigger (SPI)");
        return false;
    }
    return true;
}

/* The capture armed: start the timeout budget on top of its own completion deadline. */
static void capture_trigger_armed(int conn_id, const la_trigger_t *t) {
    if (!t->present) return;
    trigwait.active     = true;
    trigwait.conn_id    = conn_id;
    trigwait.t          = *t;
    trigwait.fired      = false;
    trigwait.deadline   = make_timeout_time_ms(t->timeout_ms);
    trigwait.next_check = make_timeout_time_us(TRIGWAIT_POLL_US);
    fpga_capture_extend_deadline_ms(t->timeout_ms);
}

/* Arming failed: drop the trigger we already programmed so the next capture is untriggered. */
static void capture_trigger_cancel(const la_trigger_t *t) {
    if (t->present) (void)fpga_set_trigger(0u, 0u);
}

/* The capture ended (completed or failed): stage "trigger":{...} for the reply and turn the
   fabric trigger off, so a later untriggered capture is unaffected. */
static void capture_trigger_done(void) {
    if (!trigwait.active) return;
    uint8_t st = 0;
    if (!trigwait.fired && fpga_trigger_status(&st) == 0 && (st & TRIGGER_STATUS_FIRED))
        trigwait.fired = true;
    trig_reply.present = true;
    trig_reply.t       = trigwait.t;
    trig_reply.fired   = trigwait.fired;
    trigwait.active    = false;
    (void)fpga_set_trigger(0u, 0u);
}

/* True while any async capture path owns the shared buffers: a v2 PSRAM
   capture/measure (v2cap), a deep LA→PSRAM capture (lacap), a unified capture
   (dualcap), or a paced bulk send (bulk). */
bool capture_busy(void) {
    return v2cap.active || lacap.active || dualcap.active || bulk.active;
}

/* Scratch for one PSRAM-sourced chunk (deep LA capture read-back).  Sized to one
   bulk chunk so the readback streams straight from PSRAM without a giant RAM
   buffer; the PSRAM bus is held for the whole send (released on completion). */
static uint8_t psram_chunk[CHUNK_SAMPLES];
/* 16-bit scratch for the deep unified capture read-back (one chunk of ADC/LA words
   pulled from PSRAM before formatting).  Separate from the byte-oriented psram_chunk
   used by the deep-LA path. */
static uint16_t psram_chunk16[CHUNK_SAMPLES];

static void bulk_begin(int conn_id, size_t total, bool is16) {
    bulk.active      = true;
    bulk.progress_ms = HAL_GetTick();
    bulk.conn_id     = conn_id;
    bulk.total       = total;
    bulk.sent        = 0;
    bulk.is16        = is16;
    bulk.from_psram  = false;
    bulk.adc_samples = 0;
    bulk.adc_rate_hz = 0;   /* default: omit; a rate-aware caller sets these after */
    bulk.la_rate_hz  = 0;
    bulk.b64         = false;   /* default: decimal; an "enc":"b64" caller sets it after */
    bulk_take_trigger();
}

/* Deep unified capture: stream the combined 16-bit result straight from PSRAM —
   adc_samples ADC samples from the ADC region, then la_samples LA words from the LA
   region — chunk-by-chunk.  The shared bus must already be held (fpga_capture_psram_wait
   grabbed it); bulk_pump releases it on completion.  Wire format is identical to the old
   combined adc_buf16 stream (is16, ADC then LA), so the server split at adc_samples is
   unchanged — only the source (chunked PSRAM vs one big SRAM read) differs. */
static void bulk_begin_capture16(int conn_id, size_t adc_samples, size_t la_samples) {
    bulk.active      = true;
    bulk.progress_ms = HAL_GetTick();
    bulk.conn_id     = conn_id;
    bulk.total       = adc_samples + la_samples;
    bulk.sent        = 0;
    bulk.is16        = true;
    bulk.from_psram  = true;
    bulk.adc_samples = adc_samples;
    bulk.adc_rate_hz = 0;
    bulk.la_rate_hz  = 0;
    bulk.la_last     = -1;   /* LA region streams transitions; first LA sample is an edge */
    bulk.b64         = false;
    bulk_take_trigger();
}

/* RESUME a capture_dual read-back from `offset` (global sample index): stream the SAME
   PSRAM data from offset..end.  The shared bus must already be held.  offset>0 makes
   bulk_pump use the "chunk" framing (no metadata header) — the server already has the
   rates/geometry from the first attempt and just appends this tail. */
static void bulk_begin_capture16_resume(int conn_id, size_t offset) {
    bulk.active      = true;
    bulk.progress_ms = HAL_GetTick();
    bulk.conn_id     = conn_id;
    bulk.total       = last_cap.adc_samples + last_cap.la_samples;
    bulk.sent        = offset;
    bulk.is16        = true;
    bulk.from_psram  = true;
    bulk.adc_samples = last_cap.adc_samples;
    bulk.adc_rate_hz = last_cap.adc_rate_hz;
    bulk.la_rate_hz  = last_cap.la_rate_hz;
    bulk.la_last     = -1;   /* re-reading LA from `offset`: emit its first sample as an edge */
    bulk.trig_present = false;   /* a resume is a read-back, not a capture: no trigger outcome */
    bulk.b64         = false;
}


static void bulk_pump(void) {
    if (!bulk.active) return;
    int conn = bulk.conn_id;

    while (bulk.sent < bulk.total) {
        /* Pace against the TCP send buffer: reserve ~64 bytes for the JSON
           framing and budget up to 7 bytes per sample ("65535,").  If it is
           nearly full, stop and resume next poll once lwIP has drained acks. */
        size_t avail = at_send_avail(conn);
        if (avail < 96) {
            if (HAL_GetTick() - bulk.progress_ms > BULK_STALL_MS) {
                printf("[cmd] bulk send stalled %lu ms on conn %d at %lu/%lu: ending it\n",
                       (unsigned long)BULK_STALL_MS, conn,
                       (unsigned long)bulk.sent, (unsigned long)bulk.total);
                bulk.active = false;
                if (bulk.from_psram) fpga_capture_psram_release();
                heavy_release(conn);
                at_close_connection(conn);   /* the reply is incomplete: drop the connection */
            }
            return;
        }
        bulk.progress_ms = HAL_GetTick();

        /* ---- LA region of a deep unified capture: stream TRANSITIONS (run-length) ----
           Digital lanes are mostly static, so sending one (index,word) pair per CHANGE
           instead of one word per sample slashes the data over the tunnel.  The ADC region
           (below) stays dense.  Server reassembles via `la_edges`/`la_upto`. */
        if (bulk.is16 && bulk.from_psram && bulk.sent >= bulk.adc_samples) {
            if (avail < 200) return;   /* header + >=1 edge + footer must fit this send */
            size_t la_remaining = bulk.total - bulk.sent;
            size_t to_read = la_remaining < CHUNK_SAMPLES ? la_remaining : CHUNK_SAMPLES;
            if (fpga_capture_psram_read16(bulk.sent, psram_chunk16, to_read, bulk.adc_samples) != 0) {
                printf("[cmd] PSRAM read-back failed at %u/%u\n", (unsigned)bulk.sent, (unsigned)bulk.total);
                bulk.active = false;
                fpga_capture_psram_release();
                send_error(conn, "capture read-back failed");
                heavy_release(conn);
                return;
            }
            size_t la_base = bulk.sent - bulk.adc_samples;   /* LA-relative index of psram_chunk16[0] */
            int pos = 0;
            if (bulk.sent == 0) {
                /* adc_samples==0 (LA-only unified capture): first chunk still carries metadata. */
                pos += snprintf(chunk_buf + pos, sizeof(chunk_buf) - (size_t)pos, "{\"status\":\"ok\",\"bits\":16,");
                if (bulk.adc_rate_hz)
                    pos += snprintf(chunk_buf + pos, sizeof(chunk_buf) - (size_t)pos, "\"adc_rate_hz\":%lu,", (unsigned long)bulk.adc_rate_hz);
                if (bulk.la_rate_hz)
                    pos += snprintf(chunk_buf + pos, sizeof(chunk_buf) - (size_t)pos, "\"la_rate_hz\":%lu,", (unsigned long)bulk.la_rate_hz);
                pos += snprintf(chunk_buf + pos, sizeof(chunk_buf) - (size_t)pos, "\"la\":true,\"la_edges\":[");
            } else {
                pos += snprintf(chunk_buf + pos, sizeof(chunk_buf) - (size_t)pos, "{\"status\":\"chunk\",\"la\":true,\"la_edges\":[");
            }
            /* Cap the output at whichever of the format buffer / TCP send buffer is smaller,
               minus a reserve for the "],\"la_upto\":NNNNNNNNNN,\"more\":false}\n" footer. */
            size_t limit = sizeof(chunk_buf);
            if (avail < limit) limit = avail;
            limit -= 56 + (bulk.trig_present ? 48u : 0u);
            int    la_last = bulk.la_last;
            bool   first   = true;
            size_t covered = 0;
            for (size_t i = 0; i < to_read; i++) {
                unsigned w = psram_chunk16[i];
                if (la_last < 0 || (unsigned)la_last != w) {
                    if ((size_t)pos + 24 > limit) break;   /* edge won't fit — end frame BEFORE sample i */
                    pos += snprintf(chunk_buf + pos, sizeof(chunk_buf) - (size_t)pos,
                                    "%s[%u,%u]", first ? "" : ",", (unsigned)(la_base + i), w);
                    first = false;
                    la_last = (int)w;
                }
                covered = i + 1;
            }
            bulk.la_last = la_last;
            size_t la_upto = la_base + covered;
            bool   last    = (bulk.sent + covered >= bulk.total);
            pos += snprintf(chunk_buf + pos, sizeof(chunk_buf) - (size_t)pos,
                            "],\"la_upto\":%u", (unsigned)la_upto);
            if (last) pos += bulk_emit_trigger(chunk_buf + pos, sizeof(chunk_buf) - (size_t)pos);
            pos += snprintf(chunk_buf + pos, sizeof(chunk_buf) - (size_t)pos,
                            ",\"more\":%s}\n", last ? "false" : "true");
            if (at_send_data(conn, (const uint8_t *)chunk_buf, (size_t)pos) != 0) {
                printf("[cmd] read-back send failed at %u/%u — conn %d dropped\n", (unsigned)bulk.sent, (unsigned)bulk.total, conn);
                bulk.active = false;
                fpga_capture_psram_release();
                heavy_release(conn);
                at_close_connection(conn);
                return;
            }
            bulk.sent += covered;
            continue;
        }

        size_t reserve = bulk_footer_reserve();
        size_t room;                               /* samples that fit right now */
        if (bulk.b64 && bulk.is16) {
            /* 2 B/sample -> 8/3 chars: whole samples in (avail - framing) chars. */
            if (avail <= reserve + BULK_B64_HEADER_MAX + 8) return;
            room = (avail - reserve - BULK_B64_HEADER_MAX) * 3u / 8u;
        } else {
            if (avail <= reserve + 7) return;
            room = (avail - reserve) / 7;
        }
        if (room == 0) return;
        size_t remaining  = bulk.total - bulk.sent;
        size_t this_chunk = remaining < CHUNK_SAMPLES ? remaining : CHUNK_SAMPLES;
        if (this_chunk > room) this_chunk = room;
        /* Deep unified capture: a chunk must not straddle the ADC/LA region boundary,
           since the two live in different PSRAM regions. */
        if (bulk.from_psram && bulk.is16 &&
            bulk.sent < bulk.adc_samples && bulk.sent + this_chunk > bulk.adc_samples)
            this_chunk = bulk.adc_samples - bulk.sent;
        bool last = (bulk.sent + this_chunk >= bulk.total);

        /* Pull this chunk out of PSRAM (the bus is held for the whole send) into a
           scratch buffer before formatting.  Two PSRAM modes: the deep unified capture
           reads 16-bit samples spanning the ADC then LA regions; the deep-LA path reads
           raw bytes from the LA region. */
        if (bulk.from_psram) {
            int rc = bulk.is16
                ? fpga_capture_psram_read16(bulk.sent, psram_chunk16, this_chunk, bulk.adc_samples)
                : fpga_la_psram_read((uint32_t)bulk.sent, psram_chunk, (uint32_t)this_chunk);
            if (rc != 0) {
                printf("[cmd] PSRAM read-back failed at %u/%u\n",
                       (unsigned)bulk.sent, (unsigned)bulk.total);
                bulk.active = false;
                fpga_capture_psram_release();
                send_error(conn, bulk.is16 ? "capture read-back failed" : "la read-back failed");
                heavy_release(conn);
                return;
            }
        }

        int pos = 0;
        if (bulk.sent == 0) {
            /* First chunk carries the reply metadata: bits + the ACHIEVED sample
               rate(s). Integer Hz via %lu — nano printf has no float. */
            pos += snprintf(chunk_buf + pos, sizeof(chunk_buf) - (size_t)pos,
                            "{\"status\":\"ok\",%s", bulk.is16 ? "\"bits\":16," : "");
            if (bulk.adc_rate_hz)
                pos += snprintf(chunk_buf + pos, sizeof(chunk_buf) - (size_t)pos,
                                "\"adc_rate_hz\":%lu,", (unsigned long)bulk.adc_rate_hz);
            if (bulk.la_rate_hz)
                pos += snprintf(chunk_buf + pos, sizeof(chunk_buf) - (size_t)pos,
                                "\"la_rate_hz\":%lu,", (unsigned long)bulk.la_rate_hz);
            pos += snprintf(chunk_buf + pos, sizeof(chunk_buf) - (size_t)pos,
                            (bulk.b64 && bulk.is16) ? "\"b64\":\"" : "\"data\":[");
        } else {
            pos += snprintf(chunk_buf + pos, sizeof(chunk_buf) - (size_t)pos,
                            (bulk.b64 && bulk.is16) ? "{\"status\":\"chunk\",\"b64\":\""
                                                    : "{\"status\":\"chunk\",\"data\":[");
        }
        if (bulk.b64 && bulk.is16) {
            /* The words are already little-endian in memory (Cortex-M33), so encode them as is. */
            const uint16_t *w = bulk.from_psram ? psram_chunk16 : &adc_buf16[bulk.sent];
            pos += (int)b64url_encode((const uint8_t *)w, this_chunk * 2u,
                                      chunk_buf + pos, sizeof(chunk_buf) - (size_t)pos);
            pos += snprintf(chunk_buf + pos, sizeof(chunk_buf) - (size_t)pos, "\"");
        } else {
            for (size_t i = 0; i < this_chunk; i++) {
                unsigned v = bulk.from_psram ? (bulk.is16 ? psram_chunk16[i] : psram_chunk[i])
                           : bulk.is16       ? adc_buf16[bulk.sent + i]
                                             : adc_cmd_buf[bulk.sent + i];
                pos += snprintf(chunk_buf + pos, sizeof(chunk_buf) - (size_t)pos,
                                "%u%s", v, (i < this_chunk - 1) ? "," : "");
            }
            pos += snprintf(chunk_buf + pos, sizeof(chunk_buf) - (size_t)pos, "]");
        }
        if (last) pos += bulk_emit_trigger(chunk_buf + pos, sizeof(chunk_buf) - (size_t)pos);
        pos += snprintf(chunk_buf + pos, sizeof(chunk_buf) - (size_t)pos,
                        ",\"more\":%s}\n", last ? "false" : "true");

        if (at_send_data(conn, (const uint8_t *)chunk_buf, (size_t)pos) != 0) {
            printf("[cmd] read-back send failed at %u/%u — conn %d dropped\n",
                   (unsigned)bulk.sent, (unsigned)bulk.total, conn);
            bulk.active = false;          /* connection died — drop it */
            if (bulk.from_psram) fpga_capture_psram_release();
            heavy_release(conn);
            at_close_connection(conn);
            return;
        }
        bulk.sent += this_chunk;
    }

    printf("[cmd] read-back complete: %u %s sent over websocket\n",
           (unsigned)bulk.total, bulk.is16 ? "samples" : "bytes");
    bulk.active = false;                  /* all chunks queued */
    if (bulk.from_psram) fpga_capture_psram_release();
    heavy_release(conn);
}

/* command_handler_poll's capture pass: trigger timeouts, async completions, then the paced
   bulk read-back. */
void capture_poll(void) {
    /* ---- triggered capture: abort if the condition never arrives ----
       The producers are loaded but parked in the fabric, so the capture's own completion
       deadline says nothing; this is what turns "waiting forever" into the documented
       `trigger timeout:` error. */
    if (trigwait.active && time_reached(trigwait.next_check)) {
        trigwait.next_check = make_timeout_time_us(TRIGWAIT_POLL_US);
        uint8_t st = 0;
        if (fpga_trigger_status(&st) == 0 && (st & TRIGGER_STATUS_FIRED)) trigwait.fired = true;
        if (!trigwait.fired && time_reached(trigwait.deadline)) {
            int  conn = trigwait.conn_id;
            char msg[LA_PINS_ERR_MAX];
            la_trigger_timeout_msg(&trigwait.t, msg, sizeof(msg));
            fpga_capture_abort();              /* also turns SET_TRIGGER off */
            trigwait.active = false;
            if (v2cap.active   && v2cap.conn_id   == conn) v2cap.active   = false;
            if (lacap.active   && lacap.conn_id   == conn) lacap.active   = false;
            if (dualcap.active && dualcap.conn_id == conn) dualcap.active = false;
            send_error(conn, msg);
            heavy_release(conn);
        }
    }

    /* ---- v2 PSRAM capture/measure: poll the iCE40 for completion ---- */
    if (v2cap.active) {
        int r = adc_capture_psram_poll(adc_buf16, v2cap.samples);
        if (r != 0) {                          /* done (1) or timeout/error (-1) */
            capture_trigger_done();
            int    cid     = v2cap.conn_id;
            size_t nsamp   = v2cap.samples;
            bool   stopdac = v2cap.stop_dac;
            v2cap.active = false;
            if (stopdac) dac_stop();           /* measure: release the DAC loop */
            if (r > 0) {
                bulk_begin(cid, nsamp, true);  /* paced send; frees the gate when done */
                bulk.b64 = v2cap.b64;
            } else {
                fpga_capture_abort();          /* a timeout leaves the fabric capturing */
                trig_reply.present = false;
                send_error(cid, "capture failed");
                heavy_release(cid);
            }
        }
    }

    /* ---- v2 deep LA→PSRAM capture: poll for completion, then stream from PSRAM ---- */
    if (lacap.active) {
        int r = fpga_la_capture_psram_wait();
        if (r != 0) {                          /* done (1) or timeout (-1) */
            capture_trigger_done();
            int    cid     = lacap.conn_id;
            size_t samples = lacap.bytes / 2u;
            bool   stopdac = lacap.stop_dac;
            uint32_t la_hz = lacap.rate_hz;
            lacap.active = false;
            /* Firmware-side cleanup after the iCE40's mid-capture DAC cut (v21): free the deep-DAC
               PSRAM region + send the official stop so the depth budget + UI state stay in sync. */
            if (stopdac) dac_stop();
            if (r > 0) {
                /* _wait() took the shared bus.  Stream the trace back through the SAME
                   16-bit read-back as capture_dual but with NO ADC region (adc_samples=0),
                   so the whole thing is the LA region → run-length edges on the wire.  Also
                   remember it (last_cap) so a stalled read-back can RESUME from PSRAM
                   (capture_read) instead of re-capturing this potentially multi-MB trace. */
                last_cap.valid       = true;
                last_cap.adc_samples = 0;
                last_cap.la_samples  = samples;
                last_cap.adc_rate_hz = 0;
                last_cap.la_rate_hz  = la_hz;
                psram_regions_track(signal_engine_la_cap_base(), (uint32_t)(samples * 2u), 0, 0);
                printf("[cmd] la_capture: PSRAM data ready (%u samples), streaming back over websocket...\n",
                       (unsigned)samples);
                bulk_begin_capture16(cid, 0, samples);
                /* the ACHIEVED rate, not the request: without it the host labelled the trace
                   with the requested rate (2.304 MHz for a 2.18 MHz capture) */
                bulk.la_rate_hz = la_hz;
            } else {
                fpga_capture_abort();          /* a timeout leaves the fabric capturing */
                trig_reply.present = false;
                send_error(cid, "la capture failed");
                heavy_release(cid);
            }
        }
    }

    /* ---- v2 unified simultaneous capture: DEEP, chunked straight from PSRAM ----
       Both regions can be multi-MB, so we never buffer them in SRAM: fpga_capture_
       psram_wait() grabs+holds the shared bus on CAP_DONE, then bulk_begin_capture16
       streams the ADC region [0..adc_samples) followed by the LA region as one 16-bit
       array (the server splits it at adc_samples: ADC samples | LA 12-bit words), and
       bulk_pump releases the bus + gate when finished. ---- */
    if (dualcap.active) {
        int r = fpga_capture_psram_wait();
        if (r != 0) {
            capture_trigger_done();
            int    cid   = dualcap.conn_id;
            size_t adc_n = dualcap.adc_samples;
            size_t la_n  = dualcap.la_samples;
            uint32_t adc_hz = (uint32_t)lroundf(dualcap.adc_rate_hz);
            uint32_t la_hz  = (uint32_t)lroundf(dualcap.la_rate_hz);
            bool stopdac = dualcap.stop_dac;
            dualcap.active = false;
            /* The iCE40 already cut the DAC mid-capture (v21 auto-stop); now do the firmware-side
               cleanup — free the deep-DAC PSRAM region + send the official stop — so the capture
               depth budget and the UI's "DAC live" state reflect that the output is off. */
            if (stopdac) dac_stop();
            if (r > 0 && adc_n >= 4 && fpga_capture_adc_sentinel_survived()) {
                /* the iCE40 never wrote the ADC region — a capture datapath/timing
                   fault, NOT the analog ADC (run cap-selftest).  This is the definitive
                   runtime PSRAM wedge: flag the datapath inoperable so status.psram_ok
                   flips and the UI can offer the reboot+reflash recovery. */
                fpga_capture_psram_release();
                signal_engine_psram_report_wedge();
                trig_reply.present = false;
                send_error(cid, "capture failed (ADC region not written; run cap-selftest)");
                heavy_release(cid);
            } else if (r > 0) {
                /* Data is safely in PSRAM now — the capture itself succeeded; all that's
                   left is streaming it back.  Log it so a stall during read-back is
                   distinguishable from a capture failure. */
                printf("[cmd] capture_dual: PSRAM data ready (ADC=%u + LA=%u samples), streaming back over websocket...\n",
                       (unsigned)adc_n, (unsigned)la_n);
                /* Remember this capture so a stalled read-back can RESUME from PSRAM
                   (capture_read) instead of re-triggering — the data stays in PSRAM until
                   the next capture overwrites it. */
                last_cap.valid       = true;
                last_cap.adc_samples = adc_n;
                last_cap.la_samples  = la_n;
                last_cap.adc_rate_hz = adc_n ? adc_hz : 0;
                last_cap.la_rate_hz  = la_n  ? la_hz  : 0;
                /* capture_read refuses once anything writes over these (psram_regions.h). */
                psram_regions_track(signal_engine_adc_cap_base(), (uint32_t)(adc_n * 2u),
                                    signal_engine_la_cap_base(),  (uint32_t)(la_n  * 2u));
                bulk_begin_capture16(cid, adc_n, la_n);   /* paced send; frees bus+gate when done */
                bulk.b64 = dualcap.b64;
                /* report the ACHIEVED rates (0 for a stream with 0 samples) so the
                   server can label each lane's timebase correctly + aligned. */
                bulk.adc_rate_hz = adc_n ? adc_hz : 0;
                bulk.la_rate_hz  = la_n  ? la_hz  : 0;
            } else {
                fpga_capture_abort();          /* a timeout leaves the fabric capturing */
                trig_reply.present = false;
                send_error(cid, "capture failed");
                heavy_release(cid);
            }
        }
    }

    /* ---- bulk sample send: deliver the response paced by the TCP send buffer ---- */
    bulk_pump();
}

void capture_conn_closed(int conn_id) {
    /* Abort an in-flight PSRAM capture/measure owned by this conn, in the fabric too.  Only
       forgetting it (as before) freed the gate while the iCE40 kept writing PSRAM, so the next
       capture or upload took the bus mid-burst; and a forgotten capture_dual streamed its result
       to the dead conn when it finished, holding the bus and the gate until the slot was reused.
       A measure or a stop-after capture also leaves the DAC running: stop it. */
    bool cap_owned = false, cap_stopdac = false;
    if (v2cap.active && v2cap.conn_id == conn_id) {
        cap_owned = true; cap_stopdac |= v2cap.stop_dac; v2cap.active = false;
    }
    if (lacap.active && lacap.conn_id == conn_id) {
        cap_owned = true; cap_stopdac |= lacap.stop_dac; lacap.active = false;
    }
    if (dualcap.active && dualcap.conn_id == conn_id) {
        cap_owned = true; cap_stopdac |= dualcap.stop_dac; dualcap.active = false;
    }
    if (cap_owned) {
        fpga_capture_abort();              /* also turns SET_TRIGGER off */
        if (trigwait.active && trigwait.conn_id == conn_id) trigwait.active = false;
        if (cap_stopdac) dac_stop();
        printf("[cmd] PSRAM capture aborted: conn %d closed\n", conn_id);
    }
    /* Abort an in-flight paced bulk send owned by this conn.  If it was reading
       back from PSRAM, hand the shared bus back to the iCE40 or it stays stuck
       MCU-owned and blocks the next capture. */
    if (bulk.active && bulk.conn_id == conn_id) {
        if (bulk.from_psram) fpga_la_psram_release();
        /* A closed conn while a read-back is still streaming almost always means the
           other end (server/browser) gave up first — i.e. a capture timeout. Report
           how far we got so a truncated capture is diagnosable from the console. */
        printf("[cmd] bulk send aborted — conn %d closed at %lu/%lu samples "
               "(likely capture timeout on the server)\n",
               conn_id, (unsigned long)bulk.sent, (unsigned long)bulk.total);
        bulk.active = false;
    }
    /* A capture still parked on its trigger is abandoned with it: abort in the fabric so the
       producers are not left armed, and turn SET_TRIGGER off. */
    if (trigwait.active && trigwait.conn_id == conn_id) {
        fpga_capture_abort();
        trigwait.active = false;
        printf("[cmd] triggered capture aborted — conn %d closed before the trigger\n", conn_id);
    }
}

/* An armed capture died with the fabric. */
void capture_on_gateware_reconfigured(void) {
    trigwait.active    = false;   /* an armed capture died with the fabric */
    trig_reply.present = false;
}

void handle_capture(int conn_id, const char *json) {
    char samples_s[16] = {0};
    json_get_value(json, "samples", samples_s, sizeof(samples_s));
    size_t samples = samples_s[0] ? (size_t)atoi(samples_s) : 256;
    float  sr_hz   = parse_sample_rate_hz(json);   /* 0 = max rate */
    /* `capture`/`stream` are the SHALLOW monolithic path (read straight into adc_buf16,
       also the capture->replay staging buffer), so they are bounded by the fixed 64 KB
       buffer, NOT the multi-MB PSRAM region.  For deep multi-second ADC use capture_dual
       (adc_samples with la_samples:0), which streams chunked from PSRAM. */
    if (samples == 0 || samples > ADC_MONO_BUF_SAMPLES) {
        send_error(conn_id, "samples out of range (use capture_dual for deep captures)");
        return;
    }
    la_trigger_t trig;
    if (!capture_trigger_begin(conn_id, json, &trig)) return;
    if (!heavy_begin(conn_id)) { capture_trigger_cancel(&trig); return; }
    /* Arm the iCE40 to stream 16-bit samples into PSRAM, then return;
       command_handler_poll() reads them back and replies once it finishes, so the
       net task keeps servicing lwIP while the capture runs. */
    if (adc_capture_psram_start(samples, sr_hz) != 0) {
        heavy_release(conn_id);
        capture_trigger_cancel(&trig);
        send_error(conn_id, "capture failed");
        return;
    }
    capture_trigger_armed(conn_id, &trig);
    last_cap.valid = false;   /* this capture overwrites a PSRAM region */
    v2cap.active   = true;
    v2cap.conn_id  = conn_id;
    v2cap.samples  = samples;
    v2cap.stop_dac = false;
    v2cap.b64      = wants_b64(json);
    dac_trace_note_capture(samples);   /* the read-back lands in the pool */
    return;            /* gate stays claimed until poll completes it */
}

/* Parse a named "<key>" MHz rate field into Hz (0 = auto). */
static float parse_named_rate_hz(const char *json, const char *key) {
    char s[16] = {0};
    if (!json_get_value(json, key, s, sizeof(s)) || !s[0]) return 0.0f;
    return (float)atof(s) * 1.0e6f;
}

/* Unified simultaneous capture (v2): arm the ADC + raw 12-ch LA producers off ONE
   trigger and stream BOTH regions back as one 16-bit array — the ADC samples first,
   then the LA words — which the server splits at adc_samples.  Either count may be
   0 to capture just the other (so this one verb also serves ADC-only / LA-only).
     {"cmd":"capture_dual","adc_samples":N,"adc_rate_mhz":R,
                           "la_samples":M,"la_rate_mhz":S}
   command_handler_poll() reads back + replies, so the net task keeps running. */
void handle_capture_dual(int conn_id, const char *json) {
    char s[16] = {0};
    size_t adc_n = 0, la_n = 0;
    if (json_get_value(json, "adc_samples", s, sizeof(s)) && s[0]) adc_n = (size_t)atoi(s);
    s[0] = 0;
    if (json_get_value(json, "la_samples", s, sizeof(s)) && s[0]) la_n = (size_t)atoi(s);
    if (adc_n == 0 && la_n == 0) {
        send_error(conn_id, "adc_samples and la_samples are both 0");
        return;
    }
    /* Deep DUAL capture: ADC and LA stream to their OWN multi-MB PSRAM regions and are
       read back CHUNKED (no shared RAM buffer), so each is bounded only by its own region
       — no adc+la sum limit.  Caps are DYNAMIC (>=v18): in a DUAL capture LA stays below
       the ADC region (adc_in_capture=true), and both shrink below a resident deep-DAC
       replay at the top of PSRAM. */
    float adc_hz = parse_named_rate_hz(json, "adc_rate_mhz");
    float la_hz  = parse_named_rate_hz(json, "la_rate_mhz");
    /* Dynamic tri-capture zone allocation (gateware >= v22): pack LA (@0) + ADC + any
       resident deep-DAC into 8 MB and REJECT up front if the combined depth won't fit or
       the combined sustained rate would overrun the bus (docs/tri-capture-unified-psram.md).
       On older gateware fall back to the static per-stream region caps. */
    uint32_t adc_base = FPGA_PSRAM_ADC_BASE;   /* legacy fixed map default */
    if (signal_engine_fpga_version() >= CAPTURE_BASES_MIN_GW) {
        psram_request_t req = {
            .la_samples  = (uint32_t)la_n,  .adc_samples = (uint32_t)adc_n,
            .dac_bytes   = signal_engine_dac_resident_bytes(),
            .la_rate_hz  = (uint32_t)lroundf(la_hz),
            .adc_rate_hz = (uint32_t)lroundf(adc_hz),
            .dac_rate_hz = signal_engine_dac_resident_bytes() ? DAC_MAX_RATE_HZ : 0u,
        };
        psram_layout_t lo = psram_plan(&req);
        if (lo.status == PSRAM_ALLOC_ERR_CAPACITY) {
            send_error(conn_id, "capture too deep: LA+ADC+DAC exceed 8 MB PSRAM");
            return;
        }
        if (lo.status == PSRAM_ALLOC_ERR_BANDWIDTH) {
            send_error(conn_id, "combined sample rate exceeds PSRAM bus bandwidth");
            return;
        }
        adc_base = lo.adc_base;   /* ADC packed right after LA (LA @ base 0) */
    } else if (adc_n > signal_engine_adc_max_samples() || la_n > signal_engine_la_max_samples(true)) {
        send_error(conn_id, "samples out of range (per-stream PSRAM region limit)");
        return;
    }
    /* Optional capture-tied DAC auto-stop (>=v21): cut a concurrently-running DAC this many µs
       into the capture so the window shows it switch off.  Arm the iCE40 BEFORE the capture
       trigger so the threshold is latched by t0.  0/absent leaves the DAC running. */
    s[0] = 0;
    uint32_t stop_us = 0;
    if (json_get_value(json, "stop_dac_after_us", s, sizeof(s)) && s[0]) stop_us = (uint32_t)strtoul(s, NULL, 10);
    bool stop_dac_armed = stop_us > 0 && signal_engine_fpga_version() >= DAC_CAPTURE_STOP_MIN_GW;
    la_trigger_t trig;
    if (!capture_trigger_begin(conn_id, json, &trig)) return;
    if (!heavy_begin(conn_id)) { capture_trigger_cancel(&trig); return; }
    fpga_set_capture_bases(0u, adc_base);  /* latch runtime zones; no-op on gw < v22 */
    fpga_set_dac_stop_after_us(stop_us);   /* arm or (0) disarm; no-op on gw < v21 */
    float adc_actual = 0.0f, la_actual = 0.0f;
    if (fpga_dual_capture_start_hz((uint32_t)adc_n, adc_hz, (uint32_t)la_n, la_hz,
                                   &adc_actual, &la_actual) != 0) {
        heavy_release(conn_id);
        capture_trigger_cancel(&trig);
        send_error(conn_id, "capture failed");
        return;
    }
    capture_trigger_armed(conn_id, &trig);
    last_cap.valid = false;   /* the new capture is overwriting the PSRAM regions */
    dualcap.active      = true;
    dualcap.conn_id     = conn_id;
    dualcap.adc_samples = adc_n;
    dualcap.la_samples  = la_n;
    dualcap.adc_rate_hz = adc_actual;   /* achieved rates -> reported in the reply */
    dualcap.la_rate_hz  = la_actual;
    dualcap.stop_dac    = stop_dac_armed;   /* free the HW-cut DAC when this capture completes */
    dualcap.b64         = wants_b64(json);
    /* Log the ACHIEVED rate as integer kHz — newlib-nano's printf has no %f, so the
       old "@%.3fMS/s" silently printed nothing (the frequency vanished from the
       console). Integer kHz via %lu prints correctly. */
    printf("[cmd] capture_dual: ADC=%u @%lukHz + LA=%u @%lukHz (one trigger)\n",
           (unsigned)adc_n, (unsigned long)lroundf(adc_actual / 1000.0f),
           (unsigned)la_n,  (unsigned long)lroundf(la_actual  / 1000.0f));
}

/* {"cmd":"capture_read","offset":N}
   RESUME streaming the last capture_dual's PSRAM data from global sample index N — the
   samples are still in PSRAM, so a stalled read-back resumes here instead of re-capturing.
   Reuses the same bulk read-back path; command_handler_poll() streams + frees the bus. */
void handle_capture_read(int conn_id, const char *json) {
    if (!last_cap.valid) {
        send_error(conn_id, "no capture to resume (run capture_dual first)");
        return;
    }
    /* The samples must still be the capture's: an OTA staging, a load_bin psram, a gateware
       reload or a SCPI/console capture since then wrote over them. */
    const char *stale = psram_regions_stale();
    if (stale) {
        char why[128];
        snprintf(why, sizeof(why), "capture data was overwritten by %s; run the capture again", stale);
        send_error(conn_id, why);
        return;
    }
    char s[16] = {0};
    size_t offset = 0;
    if (json_get_value(json, "offset", s, sizeof(s)) && s[0]) offset = (size_t)atoi(s);
    size_t total = last_cap.adc_samples + last_cap.la_samples;
    if (offset > total) {
        send_error(conn_id, "offset out of range");
        return;
    }
    if (!heavy_begin(conn_id)) return;
    if (offset == total) {
        /* Nothing left — send a final empty chunk so the server closes the stream cleanly. */
        static const char empty[] = "{\"status\":\"ok\",\"data\":[],\"more\":false}\n";
        at_send_data(conn_id, (const uint8_t *)empty, sizeof(empty) - 1);
        heavy_release(conn_id);
        return;
    }
    psram_bus_acquire();                              /* hold the bus for the read-back */
    bulk_begin_capture16_resume(conn_id, offset);
    bulk.b64 = wants_b64(json);
    printf("[cmd] capture_read: resuming from %u/%u samples over websocket...\n",
           (unsigned)offset, (unsigned)total);
}

void handle_stream(int conn_id, const char *json) {
    char samples_s[16] = {0};
    json_get_value(json, "samples", samples_s, sizeof(samples_s));
    size_t samples = samples_s[0] ? (size_t)atoi(samples_s) : 256;
    float  sr_hz   = parse_sample_rate_hz(json);   /* 0 = max rate */
    /* "stream" is a one-shot PSRAM capture: the deep capture is fully async in the
       gateware, so there is nothing to chunk (the old chunked-DMA streaming model
       was v1-only).  Arm it and defer the read-back to command_handler_poll(). */
    if (samples == 0 || samples > ADC_MONO_BUF_SAMPLES) {
        send_error(conn_id, "samples out of range (use capture_dual for deep captures)");
        return;
    }
    if (!heavy_begin(conn_id)) return;

    if (adc_capture_psram_start(samples, sr_hz) != 0) {
        heavy_release(conn_id);
        send_error(conn_id, "capture failed");
        return;
    }
    last_cap.valid = false;   /* this capture overwrites a PSRAM region */
    v2cap.active   = true;
    v2cap.conn_id  = conn_id;
    v2cap.samples  = samples;
    v2cap.stop_dac = false;
    v2cap.b64      = wants_b64(json);
    dac_trace_note_capture(samples);   /* the read-back lands in the pool */
}

/* `test` — pure Pico-side diagnostic that fills a buffer with a known
   pattern (varying or constant) and returns it via the same chunked-array
   format as capture/measure.  Useful for verifying TCP, JSON serialisation,
   and the multi-chunk response path WITHOUT involving the FPGA.

   Patterns:
     "sine"    (default) — one full period across `samples`, centred at 128
                           with amplitude 100.  Obvious visual shape.
     "counter"           — 0,1,2,...,255,0,1,...  Each byte unique within a
                           256-window, so corruption is easy to spot.
     "ramp"              — linear 0..255 across the full sample count.
     "const"             — every byte = `value` (defaults to 255).

   Examples:
     {"cmd":"test"}                                → 256-sample sine
     {"cmd":"test","pattern":"counter","samples":512}
     {"cmd":"test","pattern":"const","value":42,"samples":128}
*/
void handle_test(int conn_id, const char *json) {
    char pattern[16]   = {0};
    char value_s[8]    = {0};
    char samples_s[16] = {0};

    json_get_value(json, "pattern", pattern,   sizeof(pattern));
    json_get_value(json, "value",   value_s,   sizeof(value_s));
    json_get_value(json, "samples", samples_s, sizeof(samples_s));

    size_t samples = samples_s[0] ? (size_t)atoi(samples_s) : 256;
    if (samples == 0 || samples > SIGNAL_BUF_SIZE) {
        send_error(conn_id, "samples out of range");
        return;
    }

    /* Default pattern: sine (varies visibly).  If user gave a `value` but
       no pattern, fall back to "const" so {"cmd":"test","value":N} keeps
       its old constant-fill semantics. */
    if (pattern[0] == '\0') {
        strcpy(pattern, value_s[0] ? "const" : "sine");
    }

    if (!heavy_begin(conn_id)) return;

    /* 16-bit synthetic patterns (full-scale 0..65535), matching the v2 ADC. */
    if (strcmp(pattern, "const") == 0) {
        uint16_t v = value_s[0] ? (uint16_t)atoi(value_s) : 65535u;
        for (size_t i = 0; i < samples; i++) adc_buf16[i] = v;
    } else if (strcmp(pattern, "counter") == 0) {
        for (size_t i = 0; i < samples; i++) adc_buf16[i] = (uint16_t)(i & 0xFFFF);
    } else if (strcmp(pattern, "ramp") == 0) {
        for (size_t i = 0; i < samples; i++)
            adc_buf16[i] = (uint16_t)((uint32_t)i * 65535u / (samples - 1));
    } else if (strcmp(pattern, "sine") == 0) {
        for (size_t i = 0; i < samples; i++) {
            float angle = 2.0f * 3.14159265f * (float)i / (float)samples;
            int32_t v = (int32_t)(sinf(angle) * 25000.0f) + 32768;
            if (v < 0)     v = 0;
            if (v > 65535) v = 65535;
            adc_buf16[i] = (uint16_t)v;
        }
    } else {
        heavy_release(conn_id);
        send_error(conn_id, "unknown pattern (use sine|counter|ramp|const)");
        return;
    }

    printf("[cmd] test pattern=%s samples=%u (FPGA not touched)\n",
           pattern, (unsigned)samples);

    bulk_begin(conn_id, samples, true);   /* paced send; frees the gate when done */
    bulk.b64 = wants_b64(json);
}

void handle_measure(int conn_id, const char *json) {
    char waveform[16] = {0};
    char freq_s[16]   = {0};
    char amp_s[8]     = {0};
    char off_s[8]     = {0};
    char samples_s[16]= {0};

    json_get_value(json, "waveform",  waveform,   sizeof(waveform));
    json_get_value(json, "freq",      freq_s,     sizeof(freq_s));
    json_get_value(json, "amplitude", amp_s,      sizeof(amp_s));
    json_get_value(json, "offset",    off_s,      sizeof(off_s));
    json_get_value(json, "samples",   samples_s,  sizeof(samples_s));

    if (waveform[0] == '\0') {
        send_error(conn_id, "missing waveform");
        return;
    }
    if (strcmp(waveform, "sine") != 0 && strcmp(waveform, "square") != 0 &&
        strcmp(waveform, "sawtooth") != 0) {   /* was only "measure start failed" */
        send_error(conn_id, "unknown waveform (use sine, square or sawtooth)");
        return;
    }

    float    freq      = freq_s[0]    ? (float)atof(freq_s)     : 1000.0f;
    uint8_t  amplitude = amp_s[0]     ? (uint8_t)atoi(amp_s)    : 127;
    uint8_t  offset    = off_s[0]     ? (uint8_t)atoi(off_s)    : 128;
    size_t   samples   = samples_s[0] ? (size_t)atoi(samples_s) : 256;
    float    sr_hz     = parse_sample_rate_hz(json);   /* 0 = auto */

    /* measure plays a 16-bit waveform of `samples` points (period = samples),
       so the load is samples*2 bytes -> bounded by SIGNAL_MAX_SAMPLES. */
    if (samples == 0 || samples > SIGNAL_MAX_SAMPLES) {
        send_error(conn_id, "samples out of range");
        return;
    }

    /* Integer Hz — newlib-nano printf has no %f (the old %.1f printed nothing). */
    printf("[cmd] measure waveform=%s freq=%luHz amp=%u off=%u samples=%u\n",
           waveform, (unsigned long)lroundf(freq), amplitude, offset, (unsigned)samples);

    if (!heavy_begin(conn_id)) return;

    /* Phase-locked DAC waveform + ADC->PSRAM.  Arm it and defer the read-back +
       16-bit reply (and the DAC stop) to command_handler_poll(), so the net task
       keeps servicing lwIP while the measurement runs. */
    if (measure_psram_start(waveform, freq, amplitude, offset,
                            samples, sr_hz) != 0) {
        heavy_release(conn_id);
        send_error(conn_id, "measure start failed");
        return;
    }
    last_cap.valid = false;   /* this capture overwrites a PSRAM region */
    v2cap.active   = true;
    v2cap.conn_id  = conn_id;
    v2cap.samples  = samples;
    v2cap.stop_dac = true;
    v2cap.b64      = wants_b64(json);
    dac_trace_note_capture(samples);   /* the read-back lands in the pool */
}

void handle_sensor_regs(int conn_id, const char *json) {
    char start_s[8] = {0}, len_s[8] = {0};
    json_get_value(json, "start", start_s, sizeof(start_s));
    json_get_value(json, "len",   len_s,   sizeof(len_s));

    uint8_t start = start_s[0] ? (uint8_t)strtol(start_s, NULL, 0) : 0;
    size_t  len   = len_s[0]   ? (size_t)atoi(len_s)               : 256;

    if (!sensor_sim_active()) { send_error(conn_id, "no sensor active"); return; }
    if (len == 0 || (size_t)start + len > 256) {
        send_error(conn_id, "start/len out of range");
        return;
    }
    if (!heavy_begin(conn_id)) return;
    if (sensor_sim_read_regs(start, adc_cmd_buf, len) != 0) {
        heavy_release(conn_id);
        send_error(conn_id, "read regs failed");
        return;
    }
    bulk_begin(conn_id, len, false);   /* paced send; frees the gate when done */
}

void handle_sensor_la(int conn_id, const char *json) {
    char samples_s[16] = {0};
    json_get_value(json, "samples", samples_s, sizeof(samples_s));
    /* "samples" here = packed capture bytes (4 raw {SCL,SDA} samples per byte) */
    size_t bytes = samples_s[0] ? (size_t)atoi(samples_s) : 256;
    float  sr_hz = parse_sample_rate_hz(json);   /* 0 = max rate */

    if (bytes == 0 || bytes > SIGNAL_BUF_SIZE) {
        send_error(conn_id, "samples out of range");
        return;
    }
    if (!heavy_begin(conn_id)) return;
    if (fpga_i2c_la_capture(adc_cmd_buf, bytes, sr_hz) != 0) {
        heavy_release(conn_id);
        send_error(conn_id, "la capture failed");
        return;
    }
    bulk_begin(conn_id, bytes, false);   /* paced send; frees the gate when done */
}

/* la_capture: general multi-channel logic-analyzer snapshot of ALL 14 LA
 * channels.  Unlike sensor_la (the 2 emulated-I2C pins packed 4 samples/byte),
 * this records every LA channel as TWO little-endian bytes per sample
 * (byte 2k = LA1..LA8, byte 2k+1 = {00, LA9..LA14}).  The cloud server renders
 * the raw logic traces — the retired agent only ever decoded I2C; this is the
 * new general view.
 *   {"cmd":"la_capture","samples":1024,"sample_rate_mhz":2.0}
 * "samples" is the LA sample count (2 bytes each); 2*samples must fit the buffer.  The first
 * reply frame carries "la_rate_hz", the ACHIEVED rate: 24 MHz over a whole divider, lowered
 * further for a capture deeper than the burst ring (la_rate.c), so it can differ from the
 * request. */
void handle_la_capture(int conn_id, const char *json) {
    if (!require_la_voltage(conn_id)) return;
    char samples_s[16] = {0};
    json_get_value(json, "samples", samples_s, sizeof(samples_s));
    size_t samples = samples_s[0] ? (size_t)atoi(samples_s) : 256;
    float  sr_hz   = parse_sample_rate_hz(json);   /* 0 = max rate */

    /* Stream the LA capture into PSRAM and read it back in chunks.  Asynchronous — arm
       here, then command_handler_poll() waits for completion and starts the read-back.
       Validate the count BEFORE claiming the gate so a bad request doesn't have to
       release it.  The cap is DYNAMIC (>=v18): an LA-only capture can use the whole chip,
       shrinking to the space below a resident deep-DAC replay (max_LA = (8MB-dac_bytes)/2).
       adc_in_capture=false — this is the LA-only path (capture_dual bounds its own LA). */
    uint32_t la_max = signal_engine_la_max_samples(false);
    if (samples == 0 || samples > la_max) {
        send_error(conn_id, "samples out of range");
        return;
    }
    /* Optional capture-tied DAC auto-stop (>=v21): cut a concurrently-running DAC this many µs
       into the capture.  Arm the iCE40 BEFORE the capture trigger; 0/absent leaves it running. */
    char stop_s[16] = {0};
    uint32_t stop_us = 0;
    if (json_get_value(json, "stop_dac_after_us", stop_s, sizeof(stop_s)) && stop_s[0])
        stop_us = (uint32_t)strtoul(stop_s, NULL, 10);
    bool stop_dac_armed = stop_us > 0 && signal_engine_fpga_version() >= DAC_CAPTURE_STOP_MIN_GW;
    la_trigger_t trig;
    if (!capture_trigger_begin(conn_id, json, &trig)) return;
    if (!heavy_begin(conn_id)) { capture_trigger_cancel(&trig); return; }
    fpga_set_dac_stop_after_us(stop_us);   /* arm or (0) disarm; no-op on gw < v21 */
    float la_actual = 0.0f;
    if (fpga_la_capture_psram_start(samples, sr_hz, &la_actual) != 0) {
        heavy_release(conn_id);
        capture_trigger_cancel(&trig);
        send_error(conn_id, "la capture failed");
        return;
    }
    capture_trigger_armed(conn_id, &trig);
    last_cap.valid = false;   /* this capture overwrites the LA PSRAM region */
    lacap.active   = true;
    lacap.conn_id  = conn_id;
    lacap.bytes    = samples * 2u;
    lacap.stop_dac = stop_dac_armed;
    lacap.rate_hz  = (uint32_t)lroundf(la_actual);
}
