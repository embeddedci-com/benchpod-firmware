/*
 * command_handler_dac.c — the DAC: waveform generator, uploads, replay and the in-fabric loop.
 *
 *   generate, dac_set, dac_stop, dac_limits         direct output
 *   load, load_bin (the PROTO_LOAD raw mode)         upload a trace into the RAM pool or PSRAM
 *   replay                                           play the last capture or upload
 *   dac_control_loop, dac_loop_input, dac_loop_probe the closed-loop DAC (gateware >= v23)
 *
 * Owns the replay trace bookkeeping and the load_bin upload. Runs on the hw worker task, like
 * the rest of command_handler.
 */
#include "command_handler.h"
#include "command_handler_internal.h"
#include "at_driver.h"
#include "bp_json.h"
#include "signal_engine.h"
#include "psram.h"          /* deep DAC replay: stage the waveform into PSRAM */
#include "fpga_config.h"    /* FPGA_DAC_REPLAY_MAX_SAMPLES */
#include "psram_regions.h"
#include "adc_pool.h"
#include "b64url.h"
#include "cal_data.h"       /* ADC_CAL_EXT: the loop's input map */
#include "cloud_client.h"   /* cloud_client_request_link_snapshot (load_bin stall) */
#include "dac_limits.h"
#include "dac_loop_params.h"  /* closed-loop DAC: curve upsample + parameter validation (host-tested) */
#include "FreeRTOS.h"       /* pvPortMalloc: per-call curve buffers */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/time.h"

/* JSON parsing is shared (see bp_json.h); same aliases as command_handler.c. */
#define json_get_value      bp_json_get
#define json_flag           bp_json_flag

/* The RAM sample buffer is the shared pool (adc_pool.h); same alias as command_handler.c. */
#define adc_buf16            adc_pool

/* Number of valid samples currently held in adc_cmd_buf for `replay` to play
   out the DAC.  Set by a successful `capture`/`stream` (the just-recorded
   trace) or by a `load` upload (a host-supplied trace).  0 = nothing to
   replay yet. */
static size_t replay_len = 0;
/* The pool generation (adc_pool.h) the RAM trace above was written under: once SCPI (or anyone)
   fills the pool's trace region, it is not ours to replay any more. */
static uint32_t json_trace_gen;

/* Binary-upload (`load_bin`) state while a connection is in PROTO_LOAD raw mode:
   destination buffer + capacity (adc_buf16, or PSRAM for a deep upload), total
   bytes expected, bytes received, and the arming conn (only its raw bytes count). */
static uint8_t *load_bin_dst   = NULL;
static size_t   load_bin_total = 0;
/* Bytes of the last completed PSRAM `load_bin` (at signal_engine_dac_psram_base()), for spi_stream;
   0 = nothing staged (cleared when a new upload starts). */
static uint32_t psram_stage_len = 0;
static size_t   load_bin_have  = 0;
static int      load_bin_conn  = -1;

/* Deep-replay staging: when true, load_bin streams the raw bytes STRAIGHT INTO
   PSRAM (at the top-anchored DAC base) instead of the adc_buf16 RAM buffer — for
   waveforms too big for the 4 KB DAC BRAM (up to the full 8 MB region).  The
   shared quad bus is held (psram_bus_acquire) for the whole upload and released
   at completion so the iCE40 can read it back for CMD_START_DAC_PSRAM. */
static bool     load_bin_psram = false;
/* Set at load completion: the staged trace lives in PSRAM (replay must use the
   deep CMD_START_DAC_PSRAM path, not the BRAM LOAD_WAVE). */
static bool     replay_in_psram = false;
/* A finished upload keeps the heavy gate for its `replay` (adc_buf16 must not be overwritten
   by a capture first). A client that never replays used to hold it forever: everyone else got
   "busy". After LOADED_HOLD_MS the gate is released; a RAM upload is dropped with it (a capture
   may now reuse adc_buf16), a PSRAM upload keeps its reserved region and stays replayable. */
#define LOADED_HOLD_MS 120000u
static int             loaded_hold_conn = -1;
static absolute_time_t loaded_hold_until;
/* IDLE deadline: PROTO_LOAD holds the heavy gate while waiting for `total` raw bytes.
   A host that arms load_bin then stalls (or vanishes without a clean socket close)
   would pin the gate forever, so command_handler_poll() aborts the upload if no new
   bytes arrive for LOAD_BIN_TIMEOUT_MS.  The deadline is RE-ARMED as data streams in
   (see the PROTO_LOAD receive path) so a slow-but-progressing upload over a contended
   cloud link is never killed — only a genuine stall (inbound stopped) trips it.  It is
   an idle window, not a total budget: an 8 MB upload at 2.5 KB/s takes ~55 min yet each
   chunk pushes the deadline out, so it completes as long as bytes keep coming. */
#define LOAD_BIN_TIMEOUT_MS  30000
static absolute_time_t load_bin_deadline;
/* Re-arming the idle deadline on EVERY chunk is wasteful (thousands of clock reads on a
   multi-MB upload), so re-arm only after this many new bytes have accumulated.  Kept far
   below LOAD_BIN_TIMEOUT_MS worth of the slowest tolerated rate (~2.5 KB/s => 75 KB per
   window) so even a crawling-but-live upload always re-arms well inside the window. */
#define LOAD_BIN_REARM_BYTES 8192u
static size_t load_bin_rearm_at = 0;   /* load_bin_have value at which we last re-armed */
/* Ack cadence for the load_bin sink: report cumulative bytes-received as {"ack":N} every
   LOAD_BIN_ACK_INTERVAL so the server paces the (server->device) upload to our real drain
   rate — the end-to-end backpressure the cloud path otherwise lacks (see the speedtest
   sink). Keeps a bulk upload from overrunning Cloudflare's buffer and wedging the link. */
#define LOAD_BIN_ACK_INTERVAL 8192u
static size_t load_bin_ack_at = 0;     /* load_bin_have value at the last ack sent */

/* The PSRAM region the last `load_bin` with "psram" staged (spi_stream sends it).  False when none. */
bool command_handler_psram_stage(uint32_t *base, uint32_t *len) {
    if (psram_stage_len == 0) return false;
    *base = signal_engine_dac_psram_base();
    *len  = psram_stage_len;
    return true;
}

/* The conn streaming a load_bin upload, or -1. */
int load_bin_owner(void) { return load_bin_conn; }

/* Clear all PROTO_LOAD raw-upload bookkeeping (does not touch the heavy gate or
   proto[]).  Used on completion, on the owning conn closing, and on timeout. */
static void load_bin_clear(void) {
    if (load_bin_psram) {
        /* Hand the shared quad bus back to the iCE40 (for the follow-up replay, or
           just to leave it owning the bus).  Covers both clean completion and an
           aborted/timed-out upload, so the bus is never left stuck with the STM32. */
        psram_bus_release();
        load_bin_psram = false;
    }
    load_bin_dst   = NULL;
    load_bin_total = 0;
    load_bin_have  = 0;
    load_bin_conn  = -1;
    load_bin_rearm_at = 0;
    load_bin_ack_at = 0;
}

/* A capture/stream/measure into the RAM pool is the trace `replay` plays next. */
void dac_trace_note_capture(size_t samples) {
    replay_len     = samples;
    json_trace_gen = adc_pool_take();   /* the read-back lands in the pool */
}

void dac_conn_closed(int conn_id) {
    if (!conn_runs_state_machine(conn_id)) return;
    /* A raw load_bin upload owned by this conn is abandoned — clear its state
       so a stale total/have/dst can't corrupt the next upload (the heavy gate
       is freed by command_handler_conn_closed's heavy_release). */
    if (conn_proto(conn_id) == PROTO_LOAD || load_bin_conn == conn_id)
        load_bin_clear();
}

/* command_handler_poll's DAC pass: the LOADED_HOLD_MS gate release and the load_bin stall guard. */
void dac_poll(void) {
    /* ---- a finished upload that was never replayed gives the gate back (LOADED_HOLD_MS) ---- */
    if (loaded_hold_conn >= 0 && time_reached(loaded_hold_until)) {
        int conn = loaded_hold_conn;
        loaded_hold_conn = -1;
        if (heavy_owner_conn() == conn && !heavy_in_flight()) {
            heavy_release(conn);
            if (!replay_in_psram) replay_len = 0;    /* adc_buf16 is free for captures again */
            printf("[cmd] conn %d uploaded a waveform but did not replay it within %u s: gate released%s\n",
                   conn, (unsigned)(LOADED_HOLD_MS / 1000u),
                   replay_in_psram ? " (the PSRAM upload stays replayable)" : ", upload dropped");
        }
    }

    /* ---- load_bin stall guard: abort an upload that stopped feeding bytes ----
       PROTO_LOAD holds the heavy gate while awaiting `total` raw bytes; a host that
       arms it then stalls would pin the gate forever.  The deadline is re-armed on
       every LOAD_BIN_REARM_BYTES of progress, so reaching it means NO new bytes have
       arrived for LOAD_BIN_TIMEOUT_MS (a genuine stall, not just a slow upload).  On
       stall, error out, return the conn to JSON, release the gate and clear state. */
    if (load_bin_conn >= 0 && time_reached(load_bin_deadline)) {
        int conn = load_bin_conn;
        printf("[cmd] load_bin timeout — conn %d stalled at %u/%u bytes (no data for %u ms)\n",
               conn, (unsigned)load_bin_have, (unsigned)load_bin_total,
               (unsigned)LOAD_BIN_TIMEOUT_MS);
        /* Wedge diag: snapshot the cloud link NOW (30 s in — 60 s before the idle timeout would),
           logged net-task-side.  sndbuf==0 ⇒ the pod's send path is backed up (its ACKs aren't
           leaving ⇒ server stops sending); sndbuf healthy + old ms_since_rx ⇒ the server/Cloudflare
           stopped delivering.  This is the discriminator for a silent upload stall. */
        cloud_client_request_link_snapshot("load_bin-stall");
        if (conn_runs_state_machine(conn) && conn_proto(conn) == PROTO_LOAD)
            conn_proto_set(conn, PROTO_JSON);
        heavy_release(conn);
        load_bin_clear();
        send_error(conn, "load_bin timeout");
    }
}

/* Optional float field; returns `dflt` when absent or unparseable. */
static float json_get_float(const char *json, const char *key, float dflt) {
    char buf[24] = {0};
    if (!json_get_value(json, key, buf, sizeof(buf)) || !buf[0]) return dflt;
    return (float)atof(buf);
}

/* The loop's INPUT MAP, from the flat `in_*` fields of a dac_control_loop command.
 *
 * The client describes its BENCH — what the ADC is wired to — in engineering units:
 *   in_mv_at_zero   sense output at a zero reading, mV   (0 for an INA282 with REF to GND)
 *   in_mv_per_unit  sense output per unit, mV            (0.04 ohm x 50 V/V = 2.0 mV/mA)
 *   in_min, in_max  the range the curve is authored over
 *   in_trip         optional: park the output at vmin past this level
 * and the firmware turns that into the gateware's index map, because the firmware is the
 * only place that also knows this board's ADC calibration (cal_data.h).  A client never
 * carries a copy of that calibration, so it cannot drift from the board's own.
 *
 * Absent in_mv_per_unit => no map at all (map_en=0): the pre-v30 raw-count index, which is
 * what every existing client still gets.  Returns false and sends the error itself. */
static bool parse_loop_input_map(int conn_id, const char *json,
                                 dac_loop_inmap_t *out, bool *out_present) {
    *out_present = false;
    char probe[24] = {0};
    if (!json_get_value(json, "in_mv_per_unit", probe, sizeof(probe)) || !probe[0]) return true;

    dac_loop_input_t in = {
        .mv_at_zero  = json_get_float(json, "in_mv_at_zero", 0.0f),
        .mv_per_unit = (float)atof(probe),
        .range_min   = json_get_float(json, "in_min", 0.0f),
        .range_max   = json_get_float(json, "in_max", 0.0f),
        .trip        = 0.0f,
        .trip_en     = false,
    };
    char tbuf[24] = {0};
    if (json_get_value(json, "in_trip", tbuf, sizeof(tbuf)) && tbuf[0]) {
        in.trip = (float)atof(tbuf);
        in.trip_en = true;
    }
    /* ADC_CAL_EXT: the front-SMA fit. That is where an external sense amp is wired, and the
       only path a closed loop can read a DUT through. */
    dac_loop_adc_cal_t cal = { ADC_CAL_EXT.a, ADC_CAL_EXT.b };
    dac_loop_params_err_t e = dac_loop_inmap_derive(&in, cal, out);
    if (e != DAC_LOOP_PARAMS_OK) { send_error(conn_id, dac_loop_params_err_str(e)); return false; }
    *out_present = true;
    return true;
}

void handle_generate(int conn_id, const char *json) {
    char waveform[16] = {0};
    char freq_s[16]   = {0};
    char amp_s[8]     = {0};
    char off_s[8]     = {0};
    char dur_s[16]    = {0};

    json_get_value(json, "waveform",    waveform, sizeof(waveform));
    json_get_value(json, "freq",        freq_s,   sizeof(freq_s));
    json_get_value(json, "amplitude",   amp_s,    sizeof(amp_s));
    json_get_value(json, "offset",      off_s,    sizeof(off_s));
    json_get_value(json, "duration_ms", dur_s,    sizeof(dur_s));

    float    freq      = freq_s[0] ? (float)atof(freq_s)    : 1000.0f;
    uint8_t  amplitude = amp_s[0]  ? (uint8_t)atoi(amp_s)   : 127;
    uint8_t  offset    = off_s[0]  ? (uint8_t)atoi(off_s)   : 128;
    uint32_t duration  = dur_s[0]  ? (uint32_t)atoi(dur_s)  : 0;
    float    sr_hz     = parse_sample_rate_hz(json);   /* 0 = auto */

    /* Co-trigger (v27): defer this generator's DAC start to the next capture's t0. Arm it BEFORE
       dac_generate_* sends START_DAC (the interleaved LOAD_WAVE doesn't disturb the pending arm). */
    bool want_cotrig = json_flag(json, "on_capture");
    bool cotrig = want_cotrig ? fpga_dac_arm_on_capture() : false;

    int rc = -1;
    if      (strcmp(waveform, "sine")     == 0) rc = dac_generate_sine(freq, amplitude, offset, duration, sr_hz);
    else if (strcmp(waveform, "square")   == 0) rc = dac_generate_square(freq, amplitude, offset, duration, sr_hz);
    else if (strcmp(waveform, "sawtooth") == 0) rc = dac_generate_sawtooth(freq, amplitude, offset, duration, sr_hz);
    else { send_error(conn_id, "unknown waveform"); return; }

    if (rc != 0) { send_error(conn_id, "generate failed"); return; }
    if (want_cotrig) {
        char payload[24];
        snprintf(payload, sizeof(payload), "{\"cotrig\":%s}", cotrig ? "true" : "false");
        send_ok_str(conn_id, payload);
    } else {
        send_ok_str(conn_id, "null");
    }
}

/* `load` — upload a host-supplied waveform into adc_cmd_buf, one base64url
   chunk per command, so it can later be replayed out the DAC.  A full 4096-
   sample trace won't fit in a single 256-byte command line, so the host splits
   it: send `offset:0` first (claims the shared buffer), then successive chunks
   at increasing offsets.  Each chunk's `data` is base64url of the raw sample
   bytes.  This is what lets the host push a *saved* capture (e.g. the earlier
   "normal" run after a "fault" run has overwritten the live buffer) back to the
   device for replay.

     {"cmd":"load","offset":0,"data":"<base64url>"}
     {"cmd":"load","offset":180,"data":"<base64url>"}  ... etc
   Reply: {"status":"ok","data":{"offset":O,"len":L,"total":T}} */
void handle_load(int conn_id, const char *json) {
    char offset_s[12] = {0};
    char data_b64[240] = {0};

    json_get_value(json, "offset", offset_s, sizeof(offset_s));
    if (!json_get_value(json, "data", data_b64, sizeof(data_b64))) {
        send_error(conn_id, "missing data");
        return;
    }
    size_t offset = (size_t)atoi(offset_s);   /* absent → 0 */

    /* Don't disturb the shared sample buffer while an async capture or a paced
       send is still using it. */
    if (heavy_in_flight()) { send_error(conn_id, "busy"); return; }

    /* offset 0 begins a fresh upload and claims the shared buffer; later
       chunks must come from the same connection that still holds it (custom
       same-owner continuation, so this handler doesn't use heavy_begin). */
    if (offset == 0) {
        if (!heavy_try_claim(conn_id)) { send_error(conn_id, "busy"); return; }
        replay_len = 0;
    } else if (heavy_owner_conn() != conn_id) {
        send_error(conn_id, "load not started");
        return;
    }

    /* v2 waveforms are 16-bit: upload the raw byte stream into adc_buf16 (a
       2*SIGNAL_BUF_SIZE byte buffer); replay_len is then in 16-bit samples. */
    size_t   bufcap = SIGNAL_BUF_SIZE * 2u;      /* 16-bit samples */
    uint8_t *dst    = (uint8_t *)adc_buf16;

    if (offset > bufcap) {
        send_error(conn_id, "offset out of range");
        return;
    }

    size_t dec_len = 0;
    if (b64url_decode(data_b64, dst + offset, bufcap - offset, &dec_len) != 0) {
        send_error(conn_id, "invalid data");
        return;
    }

    size_t total_bytes = offset + dec_len;
    replay_len = total_bytes / 2u;               /* 16-bit samples */
    json_trace_gen = adc_pool_take();            /* the upload is the pool's trace now */

    char payload[64];
    snprintf(payload, sizeof(payload),
             "{\"offset\":%u,\"len\":%u,\"total\":%u}",
             (unsigned)offset, (unsigned)dec_len, (unsigned)replay_len);
    send_ok_str(conn_id, payload);
    loaded_hold_conn  = conn_id;      /* the gate waits for `replay`, re-armed per chunk */
    loaded_hold_until = make_timeout_time_ms(LOADED_HOLD_MS);
}

/* Read a spread of DAC samples back out of PSRAM and print a one-line summary, so it's easy to
   confirm on the console that real, VARYING waveform data actually landed (and the DAC has
   something to play) rather than an all-zero / flat buffer.  Called right after a deep-replay
   upload completes, while the STM32 still owns the quad bus.  Cheap: just a handful of 2-byte
   reads, not the whole waveform. */
static void dac_psram_verify_log(uint32_t base, uint32_t total_bytes) {
    uint32_t nsamp = total_bytes / 2u;
    if (nsamp == 0) return;
    const int probes = 16;
    uint16_t first = 0, smin = 0xFFFF, smax = 0;
    for (int k = 0; k < probes; k++) {
        uint32_t idx = (uint32_t)((uint64_t)k * (nsamp - 1u) / (probes > 1 ? probes - 1 : 1));
        uint8_t two[2];
        if (psram_read(base + idx * 2u, two, 2) != 0) {
            printf("[dac] PSRAM verify read failed\n");
            return;
        }
        uint16_t v = (uint16_t)(two[0] | (two[1] << 8));
        if (k == 0) first = v;
        if (v < smin) smin = v;
        if (v > smax) smax = v;
    }
    printf("[dac] staged %u samples in PSRAM @0x%06lX  first=0x%04X min=0x%04X max=0x%04X  %s\n",
           (unsigned)nsamp, (unsigned long)base, first, smin, smax,
           (smin == smax) ? "FLAT — no DAC signal, check the source" : "varying (ok)");
}

/* `load_bin` — arm a raw binary upload of a host waveform.  Unlike the chunked
   base64 `load`, this switches the connection into PROTO_LOAD raw mode and the
   host streams exactly `total` raw bytes (no per-chunk JSON/base64, so not bounded
   by the 256-byte command line).  v2 waveforms are 16-bit, so the bytes land in
   adc_buf16 (or PSRAM for a deep upload) and replay_len is total/2 samples.
   Refused over the single-reply cloud command channel (needs a stream conn).
     {"cmd":"load_bin","total":N}
   Reply (immediate): {"status":"ok","data":{"ready":N}}  -> stream N bytes next.
   Reply (after N bytes): {"status":"ok","data":{"total":S}}  (S = samples). */
void handle_load_bin(int conn_id, const char *json) {
    if (!conn_runs_state_machine(conn_id)) {
        send_error(conn_id, "load_bin needs a stream connection");
        return;
    }
    char total_s[12] = {0};
    if (!json_get_value(json, "total", total_s, sizeof(total_s))) {
        send_error(conn_id, "missing total");
        return;
    }
    size_t total = (size_t)strtoul(total_s, NULL, 10);

    /* Deep replay: stream straight into PSRAM (up to the full 8 MB region) instead
       of the 4 KB DAC BRAM buffer.  The server sets "psram":1 for waveforms that
       exceed the shallow BRAM depth (e.g. a whole stored ADC recording). */
    char psram_s[8] = {0};
    /* Accept the JSON boolean true (json_get_value returns the token "true") as well
       as "1" — the server sends `"psram": true`. */
    bool to_psram = json_get_value(json, "psram", psram_s, sizeof(psram_s)) &&
                    (psram_s[0] == 't' || psram_s[0] == 'T' || psram_s[0] == '1');

    /* Shallow cap is what `replay` can actually play out the DAC BRAM
       (SIGNAL_MAX_SAMPLES 16-bit samples = SIGNAL_BUF_SIZE bytes), NOT the size of the
       adc_buf16 staging buffer (2x that) — accepting the larger size let an oversized
       upload succeed here only to fail at `replay` with "arbitrary invalid params". */
    size_t bufcap = to_psram ? (size_t)FPGA_DAC_REPLAY_MAX_SAMPLES * 2u   /* 8 MB PSRAM */
                             : (size_t)SIGNAL_MAX_SAMPLES * 2u;           /* 4 KB DAC BRAM */
    if (total == 0 || total > bufcap) {
        send_error(conn_id, "total out of range");
        return;
    }
    if (!heavy_begin(conn_id)) return;

    psram_stage_len = 0;
    if (to_psram) {
        /* Take the shared quad bus so the STM32 can write PSRAM directly over XSPI;
           released at completion (load_bin_clear) so the iCE40 can read it back for
           CMD_START_DAC_PSRAM.  The STM32 owns the bus only during the upload; once
           released a capture MAY run concurrently with the replay (>=v18 arbiter). */
        psram_bus_acquire();
        /* Fix the top-anchored DAC base now that we know the total length, so the
           staging writes and the later CMD_START_DAC_PSRAM agree on the base. */
        signal_engine_dac_psram_stage((uint32_t)total);
        psram_regions_dirty(signal_engine_dac_psram_base(), (uint32_t)total, "a load_bin psram upload");
        load_bin_psram = true;
        load_bin_dst   = NULL;                 /* bytes go to PSRAM, not a RAM buffer */
    } else {
        load_bin_psram = false;
        load_bin_dst   = (uint8_t *)adc_buf16;
        json_trace_gen = adc_pool_take();      /* the upload is the pool's trace now */
    }
    load_bin_total    = total;
    load_bin_have     = 0;
    load_bin_conn     = conn_id;
    /* IDLE window: covers the wait for the first chunk and is re-armed as each chunk lands
       (PROTO_LOAD receive path), so a large deep upload streaming for minutes over the slow,
       contended cloud TLS link never trips it — only a genuine stall does.  Replaces the old
       hard total budget (30 s + ~500 µs/byte), which could kill a slow-but-progressing
       upload; the idle re-arm is what the old comment claimed but never actually did. */
    load_bin_deadline = make_timeout_time_ms(LOAD_BIN_TIMEOUT_MS);
    load_bin_rearm_at = 0;
    replay_len        = 0;
    replay_in_psram   = false;   /* set true only on a clean psram completion */
    conn_proto_set(conn_id, PROTO_LOAD);   /* subsequent raw bytes go to the buffer/PSRAM */

    char payload[32];
    snprintf(payload, sizeof(payload), "{\"ready\":%u}", (unsigned)total);
    send_ok_str(conn_id, payload);
}

/* `replay` — play the trace currently in adc_cmd_buf out the DAC.  The buffer
   holds either the most recent `capture`/`stream` (so you can capture a motor
   current waveform and immediately replay it) or a host-uploaded trace from
   `load` (so you can replay a previously saved run).  Loops continuously until
   `dac_stop` (or the next `generate`/`measure`/`replay`).

     {"cmd":"replay","sample_rate_mhz":0.08,"samples":4096}
   `sample_rate_mhz` should match the rate the trace was captured at so the
   playback time-base matches; omit for the max 12 MSPS rate.  `samples`
   defaults to the full recorded length. */
void handle_replay(int conn_id, const char *json) {
    char samples_s[16] = {0};
    json_get_value(json, "samples", samples_s, sizeof(samples_s));
    float  sr_hz   = parse_sample_rate_hz(json);   /* 0 = max rate */
    size_t samples = samples_s[0] ? (size_t)strtoul(samples_s, NULL, 10) : replay_len;

    /* A RAM trace the pool has since given to someone else (a SCPI READ? or TRACe:DATA) is gone. */
    if (!replay_in_psram && !adc_pool_holds(json_trace_gen)) replay_len = 0;
    if (replay_len == 0) {
        send_error(conn_id, "nothing to replay");
        return;
    }
    if (samples == 0 || samples > replay_len) {
        send_error(conn_id, "samples out of range");
        return;
    }

    /* Co-trigger (v27): when the host asks to start this replay together with a capture, arm the
       iCE40 to DEFER the DAC start to the next capture's t0 (DAC sample 0 == t0).  Must be sent
       right BEFORE the DAC start opcode below.  On gateware < v27 it can't co-trigger, so we fall
       back to the immediate start (the reply's "cotrig" tells the host which happened). */
    bool cotrig = false;
    if (json_flag(json, "on_capture")) cotrig = fpga_dac_arm_on_capture();

    /* Claims the shared buffer (or no-ops if this conn already holds it from a
       preceding `load`).  Released as soon as the waveform is armed — the DAC then
       runs off the FPGA (BRAM or PSRAM), not adc_cmd_buf. */
    if (!heavy_begin(conn_id)) return;
    if (loaded_hold_conn == conn_id) loaded_hold_conn = -1;
    int rc;
    if (replay_in_psram) {
        /* Deep replay: the trace is already staged in PSRAM and the bus is released
           to the iCE40 — just arm the streaming DAC (no BRAM load, no depth cap). */
        rc = dac_replay_psram((uint32_t)samples, sr_hz);
    } else {
        /* Shallow: 16-bit trace in adc_buf16; the DAC sequencer takes byte pairs, so
           pass 2*samples bytes (<= the 4 KB LOAD_WAVE BRAM). */
        rc = dac_generate_arbitrary_rate((const uint8_t *)adc_buf16, samples * 2u, true, sr_hz);
    }
    heavy_release(conn_id);

    if (rc != 0) { send_error(conn_id, "replay failed"); return; }

    /* Trace what we just armed onto the DAC, so it's clear from the console whether the output is
       actually driving (and with real data).  For a shallow trace we can peek adc_buf16 directly;
       the deep PSRAM trace was already summarised at upload (dac_psram_verify_log). */
    const char *co = cotrig ? " [co-trigger: waiting for capture t0]" : "";
    if (replay_in_psram) {
        printf("[dac] replay armed (deep/PSRAM): %u samples @ %ld Hz%s\n",
               (unsigned)samples, (long)(sr_hz > 0.0f ? (long)sr_hz : 0), co);
    } else {
        uint16_t first = adc_buf16[0], smin = 0xFFFF, smax = 0;
        for (size_t i = 0; i < samples; i++) {
            uint16_t v = adc_buf16[i];
            if (v < smin) smin = v;
            if (v > smax) smax = v;
        }
        printf("[dac] replay armed: %u samples @ %ld Hz  first=0x%04X min=0x%04X max=0x%04X  %s%s\n",
               (unsigned)samples, (long)(sr_hz > 0.0f ? (long)sr_hz : 0), first, smin, smax,
               (smin == smax) ? "FLAT — no DAC signal, check the source" : "varying (ok)", co);
    }

    /* "cotrig" lets the host know the DAC start was DEFERRED to the next capture (it is not driving
       yet) vs already looping — so the UI waits for the capture to fire it instead of expecting
       output now. */
    char payload[64];
    snprintf(payload, sizeof(payload), "{\"samples\":%u,\"cotrig\":%s}",
             (unsigned)samples, cotrig ? "true" : "false");
    send_ok_str(conn_id, payload);
}

/* `dac_stop` — halt any running DAC output (a looped `replay`, a continuous
   `generate`, or a held `dac_set_constant`). */
void handle_dac_stop(int conn_id, const char *json) {
    (void)json;
    dac_stop();
    /* With DAC limits set (an external output stage), stopping must not leave the DAC wherever
       it was: hold it at the low-output end instead (dac_limits.h). */
    if (dac_limits_get()->enabled) {
        if (dac_limits_park_now() != 0) { send_error(conn_id, "DAC stopped, but parking it at the limits failed"); return; }
        char payload[48];
        const dac_limits_t *l = dac_limits_get();
        snprintf(payload, sizeof(payload), "{\"parked_mv\":%ld}", (long)(l->inverted ? l->max_mv : l->min_mv));
        send_ok_str(conn_id, payload);
        return;
    }
    send_ok_str(conn_id, "null");
}

/* Reply body describing the active DAC limits. */
static void dac_limits_reply(int conn_id) {
    const dac_limits_t *l = dac_limits_get();
    char payload[160];
    if (!l->enabled) {
        snprintf(payload, sizeof(payload), "{\"enabled\":false}");
    } else {
        snprintf(payload, sizeof(payload),
                 "{\"enabled\":true,\"path\":\"%s\",\"inverted\":%s,\"min_mv\":%ld,\"max_mv\":%ld,\"park_mv\":%ld}",
                 dac_limits_path_name(l->path), l->inverted ? "true" : "false",
                 (long)l->min_mv, (long)l->max_mv, (long)(l->inverted ? l->max_mv : l->min_mv));
    }
    send_ok_str(conn_id, payload);
}

/* `dac_limits` — read, set or clear the DAC output limits for an external output stage
   (dac_limits.h). Stored in flash and enforced on every DAC command until cleared.
     {"cmd":"dac_limits"}                                         -> the active limits
     {"cmd":"dac_limits","path":"5v","inverted":true,"min_mv":1850,"max_mv":3600}  -> set
     {"cmd":"dac_limits","enabled":false}                         -> clear
   Setting does not move the DAC; the next dac_stop (or boot) parks it. */
void handle_dac_limits(int conn_id, const char *json) {
    char v[16] = {0};
    if (json_get_value(json, "enabled", v, sizeof(v)) && strcmp(v, "false") == 0) {
        if (dac_limits_clear() != 0) { send_error(conn_id, "could not clear the DAC limits"); return; }
        dac_limits_reply(conn_id);
        return;
    }
    char path[16] = {0}, inv[8] = {0}, lo[16] = {0}, hi[16] = {0};
    bool has_path = json_get_value(json, "path", path, sizeof(path));
    if (!has_path) { dac_limits_reply(conn_id); return; }
    if (!json_get_value(json, "min_mv", lo, sizeof(lo)) || !json_get_value(json, "max_mv", hi, sizeof(hi))) {
        send_error(conn_id, "dac_limits needs path, min_mv and max_mv (and inverted)"); return;
    }
    int idx = dac_limits_path_index(path);
    if (idx < 0) { send_error(conn_id, "path must be 3v3, 5v or 12v"); return; }
    json_get_value(json, "inverted", inv, sizeof(inv));
    dac_limits_t l = {
        .enabled = 1, .path = (uint8_t)idx, .inverted = (uint8_t)(strcmp(inv, "true") == 0),
        .min_mv = (int32_t)atol(lo), .max_mv = (int32_t)atol(hi),
    };
    const char *why = dac_limits_set(&l);
    if (why) { send_error(conn_id, why); return; }
    dac_limits_reply(conn_id);
}

/* `dac_set` — hold the DAC at a fixed DC level (multimeter-probe / fault
   injection).  Single-reply, so it rides the cloud command channel.  On v2 the
   FPGA DAC sequencer clocks a single byte continuously; `value` is the 0..255
   level and `divider` (optional) sets DAC_CLK = HFOSC / divider.
     {"cmd":"dac_set","value":0..255,"divider":N} */
void handle_dac_set(int conn_id, const char *json) {
    char val_s[8]  = {0};
    char div_s[12] = {0};
    if (!json_get_value(json, "value", val_s, sizeof(val_s))) {
        send_error(conn_id, "missing value");
        return;
    }
    int v = atoi(val_s);
    if (v < 0 || v > 255) { send_error(conn_id, "value out of range"); return; }
    uint32_t divider = json_get_value(json, "divider", div_s, sizeof(div_s))
                           ? (uint32_t)atoi(div_s) : 0;   /* 0 -> clamped to 2 */
    if (dac_set_constant((uint8_t)v, divider) != 0) {
        send_error(conn_id, "dac set failed");
        return;
    }
    send_ok_str(conn_id, "null");
}

/* Last loop input source pushed to the gateware.  The fabric registers are write-only, so
   this mirror is what lets {"cmd":"dac_loop_input","input":N} be a one-field STEP (keep the
   source and step, move the point) instead of forcing every client to resend the whole set —
   and it is what the probe reports the source as.  Written only from the hw worker. */
static uint8_t  s_loop_src  = DAC_LOOP_SRC_ADC;
static uint16_t s_loop_in   = 0;
static uint16_t s_loop_step = 0;

/* A RECONFIGURATION resets the fabric's loop registers to their power-on values (source=ADC,
   input 0, step 0) but leaves these mirrors alone — so a `dac_loop_input` after an image swap
   would push a source the fabric is no longer in, and the probe would report the stale one.
   Same defect class as the capture-base desync (see signal_engine_on_gateware_reconfigured);
   called from the same place, ice40_reflash_image() (via command_handler_on_gateware_reconfigured). */
void dac_on_gateware_reconfigured(void) {
    s_loop_src  = DAC_LOOP_SRC_ADC;
    s_loop_in   = 0;
    s_loop_step = 0;
}

/* In-fabric DAC CONTROL LOOP (gateware >= v23): a DETERMINISTIC loop where the DAC output is
   computed every tick from an INPUT through a reloadable curve LUT — v += k*(curve[in>>5] - v),
   clamped [vmin,vmax].  Any transfer function you can tabulate: a solar-panel / EPS-MPPT
   emulator is one curve you can load, not what the feature is.  The curve goes into the DAC
   BRAM (16-bit LE, base64url).  STOP with {"cmd":"dac_stop"}.  It uses NO PSRAM, so an LA
   capture can run alongside it (optionally auto-cutting the DAC).

   `source` picks where the INPUT comes from (gateware >= v29):
     "adc"   — the live ADC: a real CLOSED loop, the output reacts to the DUT (default).
     "fixed" — a host-held constant `input`: OPEN loop, one point of the curve at a time, with
               the ADC and the analog input path out of the picture.  This is how the DAC and
               output stage get validated on their own — hold a point, meter the SMA, compare
               against curve[input] — before anyone trusts a closed loop.
     "sweep" — `input` advances by `step` every tick (mod 65536): OPEN loop, a function of TIME
               that walks the whole curve at a deterministic rate, again with no ADC.
   Change it live with {"cmd":"dac_loop_input", ...} — no re-arm, no curve re-upload.

     {"cmd":"dac_control_loop","k":Q15,"vmin":N,"vmax":N,"tick_div":N,"curve":"<b64url>",
      "source":"adc"|"fixed"|"sweep","input":N,"step":N} */
/* The curve upload buffers (16 KB): only needed while one control_loop command runs, so they
   come from the heap for that call instead of sitting in .bss. */
typedef struct {
    char    b64[SIGNAL_BUF_SIZE * 2];   /* base64url of the uploaded curve bytes */
    uint8_t curve[SIGNAL_BUF_SIZE];
    uint8_t full[SIGNAL_BUF_SIZE];      /* the full 2048-point gateware LUT       */
} loop_curve_bufs_t;

static void handle_dac_control_loop_with(int conn_id, const char *json, loop_curve_bufs_t *cb);

void handle_dac_control_loop(int conn_id, const char *json) {
    loop_curve_bufs_t *cb = pvPortMalloc(sizeof(*cb));
    if (!cb) { send_error(conn_id, "out of memory for the curve"); return; }
    handle_dac_control_loop_with(conn_id, json, cb);
    vPortFree(cb);
}

static void handle_dac_control_loop_with(int conn_id, const char *json, loop_curve_bufs_t *cb) {
    if (signal_engine_fpga_version() < DAC_CONTROL_LOOP_MIN_GW) {
        send_error(conn_id, "control loop needs gateware v23+"); return;
    }
    char s[16] = {0};
    dac_loop_params_t p = { .k_q15 = 8192, .vmin = 0, .vmax = 0xFFFF, .tick_div = 64,
                            .src = DAC_LOOP_SRC_ADC, .in_fixed = 0, .sweep_step = 0 };
    if (json_get_value(json, "k", s, sizeof(s)) && s[0])        p.k_q15    = (uint16_t)strtoul(s, NULL, 10);
    if (json_get_value(json, "vmin", s, sizeof(s)) && s[0])     p.vmin     = (uint16_t)strtoul(s, NULL, 10);
    if (json_get_value(json, "vmax", s, sizeof(s)) && s[0])     p.vmax     = (uint16_t)strtoul(s, NULL, 10);
    if (json_get_value(json, "tick_div", s, sizeof(s)) && s[0]) p.tick_div = (uint16_t)strtoul(s, NULL, 10);
    if (json_get_value(json, "input", s, sizeof(s)) && s[0])    p.in_fixed = (uint16_t)strtoul(s, NULL, 10);
    if (json_get_value(json, "step", s, sizeof(s)) && s[0])     p.sweep_step = (uint16_t)strtoul(s, NULL, 10);
    if (json_get_value(json, "source", s, sizeof(s)) && s[0]) {
        if (!dac_loop_src_parse(s, &p.src)) {
            send_error(conn_id, "unknown loop input source (use adc, fixed or sweep)"); return;
        }
        /* An OPEN-loop source on gateware that cannot honour it would silently run the CLOSED
           loop instead — the caller would meter a value the ADC produced and believe it came
           from the point they asked for.  Refuse rather than substitute. */
        if (p.src != DAC_LOOP_SRC_ADC && signal_engine_fpga_version() < DAC_LOOP_SOURCE_MIN_GW) {
            send_error(conn_id, "loop input sources need gateware v29+"); return;
        }
    }
    /* Clamp what has a safe reading (k, tick_div) and REFUSE what would silently do something
       else: an inverted output window (the gateware clamps against vmin first, so vmin>vmax
       pins the DAC at vmin and ignores the caller's ceiling) and a zero-step sweep (a "sweep"
       that never advances).  See test/test_dac_loop_params.c. */
    dac_loop_params_err_t perr = dac_loop_params_validate(&p);
    if (perr != DAC_LOOP_PARAMS_OK) { send_error(conn_id, dac_loop_params_err_str(perr)); return; }

    char    *curve_b64  = cb->b64;
    uint8_t *curve      = cb->curve;
    uint8_t *curve_full = cb->full;
    size_t curve_len = 0;
    if (json_get_value(json, "curve", curve_b64, sizeof(cb->b64)) && curve_b64[0]) {
        if (b64url_decode(curve_b64, curve, sizeof(cb->curve), &curve_len) != 0) {
            send_error(conn_id, "curve base64 decode failed"); return;
        }
        if (curve_len < 2) { send_error(conn_id, "curve needs at least one 16-bit point"); return; }
    }
    /* The gateware LUT is a fixed 2048-point table (curve[ADC>>5]); the transport caps a single
       command, so the client uploads a COMPACT curve and dac_loop_upsample_curve() expands it
       (nearest-neighbour) to fill all 2048 entries — spanning the whole ADC range with no stale
       upper entries left over from a previous load.  An already-full upload passes through.
       Omitting `curve` keeps whatever is already loaded (clamp-only arm). */
    /* The input map (v30) has to be resolved BEFORE the curve is expanded: when there is a
       map, the curve spans the client's range, which lands on indices 0..idx_max rather than
       the whole table, and it is upsampled over exactly that. */
    dac_loop_inmap_t map = {0};
    bool have_map = false;
    if (!parse_loop_input_map(conn_id, json, &map, &have_map)) return;
    if (have_map && signal_engine_fpga_version() < DAC_LOOP_INMAP_MIN_GW) {
        send_error(conn_id, "loop input map needs gateware v30+"); return;
    }
    /* A mapped SWEEP would be misleading rather than wrong: the sweep accumulator walks the
       whole 16-bit count space, but a mapped window is a few percent of it, so the output
       would track the curve for ~3% of each pass and sit saturated at one end or the other
       for the rest.  Confining the accumulator to the window needs a comparator + reload in
       fabric, and the loop image has no LC left for it (86%, and only 5 of 12 seeds place).
       So refuse the combination and name the alternative, rather than emit a sawtooth that
       looks like a broken curve. */
    if (have_map && p.src == DAC_LOOP_SRC_SWEEP) {
        send_error(conn_id, "sweep cannot be combined with an input map on this gateware: the "
                            "sweep spans the raw count range, not the mapped window. Step the "
                            "input with {\"cmd\":\"dac_loop_input\"} instead");
        return;
    }

    const uint8_t *lut = NULL;
    size_t         lut_len = 0;
    if (curve_len >= 2) {
        lut_len = have_map
            ? dac_loop_upsample_curve_mapped(curve, curve_len, map.idx_max,
                                             curve_full, sizeof(cb->full))
            : dac_loop_upsample_curve(curve, curve_len, curve_full, sizeof(cb->full));
        if (lut_len == 0) { send_error(conn_id, "curve upsample failed"); return; }
        lut = curve_full;
    }
    if (!heavy_begin(conn_id)) return;
    /* Set the input source BEFORE arming so the loop's very first tick already uses it — an
       open-loop run must never start by reading the ADC, however briefly.  Skipped entirely on
       pre-v29 gateware asked for the (default) ADC source, which is the old behaviour. */
    int rc = 0;
    if (signal_engine_fpga_version() >= DAC_LOOP_SOURCE_MIN_GW)
        rc = dac_loop_set_source(p.src, p.in_fixed, p.sweep_step);
    /* Program the map before arming too: the first tick must already index through it, or
       the loop opens by driving whatever the raw count happened to point at.
       An arm WITHOUT a map has to switch the map (and its trip) OFF: the fabric keeps the
       registers of the previous arm until a reconfiguration, so until 3.4.0 an unmapped arm
       after a mapped one silently indexed the curve through the old map (measured: fixed
       input 0 landed on curve entry 62 instead of 0). */
    if (rc == 0 && signal_engine_fpga_version() >= DAC_LOOP_INMAP_MIN_GW)
        rc = have_map ? dac_loop_set_inmap(map.in_zero, map.in_gain, map.in_trip, true, map.trip_en)
                      : dac_loop_set_inmap(0, 0, 0, false, false);
    if (rc == 0) rc = dac_control_loop_start(lut, lut_len, p.k_q15, p.vmin, p.vmax, p.tick_div);
    heavy_release(conn_id);
    if (rc == -2) { send_error(conn_id, "control loop needs gateware v23+"); return; }
    if (rc == -3) {
        /* Right image family, wrong image: the deep-replay bitstream has no loop engine, and
           both images report the same version — so this is the ONLY thing that catches it. */
        send_error(conn_id, "control loop not in the running gateware image "
                            "(switch with {\"cmd\":\"fpga_image\",\"image\":0})");
        return;
    }
    if (rc != 0)  { send_error(conn_id, "control loop arm failed"); return; }
    s_loop_src = p.src; s_loop_in = p.in_fixed; s_loop_step = p.sweep_step;
    /* Echo the DERIVED map, not the request: the caller sent millivolts-per-unit and gets
       back the counts and index the hardware will actually use, which is the only way to
       tell a map that landed where it was meant to from one that quietly saturated.
       NOTE: send_ok_str's reply buffer is 192 bytes — check any new field against it. */
    char payload[224];
    int n = snprintf(payload, sizeof(payload),
             "{\"armed\":true,\"k\":%u,\"vmin\":%u,\"vmax\":%u,\"tick_div\":%u,\"curve_pts\":%u,"
             "\"source\":\"%s\",\"input\":%u,\"step\":%u",
             p.k_q15, p.vmin, p.vmax, p.tick_div, (unsigned)(lut_len / 2u),
             dac_loop_src_str(p.src), p.in_fixed, p.sweep_step);
    if (have_map && n > 0 && (size_t)n < sizeof(payload))
        n += snprintf(payload + n, sizeof(payload) - (size_t)n,
                      ",\"in_zero\":%u,\"in_gain\":%d,\"idx_max\":%u,\"trip_idx\":%d",
                      map.in_zero, (int)map.in_gain, map.idx_max,
                      map.trip_en ? (int)map.in_trip : -1);
    if (n > 0 && (size_t)n < sizeof(payload)) snprintf(payload + n, sizeof(payload) - (size_t)n, "}");
    send_ok_str(conn_id, payload);
}

/* {"cmd":"dac_loop_input","input":N[,"source":"adc"|"fixed"|"sweep"][,"step":N]} — retarget a
   RUNNING (or next-armed) control loop's input without re-arming or re-uploading the curve.
   This is the open-loop bring-up flow: hold curve point A, meter the output, move to B, meter
   again — each step is one small command, and the curve stays exactly as it was so the two
   readings are comparable.  Omitted fields keep their current value.  Gateware >= v29. */
void handle_dac_loop_input(int conn_id, const char *json) {
    if (signal_engine_fpga_version() < DAC_LOOP_SOURCE_MIN_GW) {
        /* Cheap pre-check on the cached version; the authoritative image gate lives in
           dac_loop_set_source (a live SPI read, so it runs under the heavy lock). */
        send_error(conn_id, "loop input sources need gateware v29+"); return;
    }
    char s[16] = {0};
    /* Defaults come from what was last SET, so {"input":N} alone is a pure input step. */
    uint8_t  src  = s_loop_src;
    uint16_t in   = s_loop_in;
    uint16_t step = s_loop_step;
    if (json_get_value(json, "source", s, sizeof(s)) && s[0]) {
        if (!dac_loop_src_parse(s, &src)) {
            send_error(conn_id, "unknown loop input source (use adc, fixed or sweep)"); return;
        }
    }
    if (json_get_value(json, "input", s, sizeof(s)) && s[0]) in   = (uint16_t)strtoul(s, NULL, 10);
    if (json_get_value(json, "step",  s, sizeof(s)) && s[0]) step = (uint16_t)strtoul(s, NULL, 10);
    /* Same guards as arming: a zero-step sweep is a frozen input under a moving name. */
    dac_loop_params_t p = { .k_q15 = DAC_LOOP_K_MIN, .vmin = 0, .vmax = 0xFFFF,
                            .tick_div = DAC_LOOP_TICK_MIN,
                            .src = src, .in_fixed = in, .sweep_step = step };
    dac_loop_params_err_t perr = dac_loop_params_validate(&p);
    if (perr != DAC_LOOP_PARAMS_OK) { send_error(conn_id, dac_loop_params_err_str(perr)); return; }

    if (!heavy_begin(conn_id)) return;
    int rc = dac_loop_set_source(src, in, step);
    uint16_t v = (rc == 0) ? fpga_dac_loop_probe() : 0;
    heavy_release(conn_id);
    if (rc == -2) { send_error(conn_id, "loop input sources need gateware v29+"); return; }
    if (rc == -3) {
        send_error(conn_id, "control loop not in the running gateware image "
                            "(switch with {\"cmd\":\"fpga_image\",\"image\":0})");
        return;
    }
    if (rc != 0) { send_error(conn_id, "loop input update failed"); return; }
    s_loop_src = src; s_loop_in = in; s_loop_step = step;
    /* `v` is the output at the instant of the write — the loop damps toward the new target
       over the next ticks, so poll dac_loop_probe for the settled value. */
    char payload[96];
    snprintf(payload, sizeof(payload), "{\"source\":\"%s\",\"input\":%u,\"step\":%u,\"v\":%u}",
             dac_loop_src_str(src), in, step, v);
    send_ok_str(conn_id, payload);
}

/* {"cmd":"dac_loop_probe"} -> {"i":<adc count>,"in":<loop input>,"source":"...","v":<dac code>}
   — one live operating point for the transfer-curve view (poll this ~10-50 Hz; the webapp
   overlays it on the host-drawn curve).

   `i` is the RAW ADC count (uncalibrated — clients label it as a code, not amps/volts) read
   through ADC_PROBE, which returns the very same `adc_sample` register the loop indexes its
   curve with WHEN THE SOURCE IS THE ADC (top_v2.v feeds it to both) — so in a closed-loop run
   the pair is the loop's own operating point, up to the tick or two between the two SPI reads.
   `in` (gateware >= v29) is what the loop's last tick ACTUALLY indexed with, whatever the
   source; in a fixed/sweep run the ADC is not in the path at all and `i` is just a reading of
   an input nothing is using — plot against `in`, not `i`.  On pre-v29 gateware `in` is the ADC
   value, which is what the loop used there by construction. */
void handle_dac_loop_probe(int conn_id, const char *json) {
    (void)json;
    if (!heavy_begin(conn_id)) return;
    /* Same image gate as arming: on the deep-replay image DAC_PROBE reads a tied-off 0, which
       is indistinguishable from "the loop is driving 0 V".  Refuse instead of reporting a
       plausible-looking operating point that no loop produced. */
    if (!signal_engine_has_control_loop()) {
        heavy_release(conn_id);
        send_error(conn_id, "closed-loop DAC not in the running gateware image "
                            "(switch with {\"cmd\":\"fpga_image\",\"image\":0})");
        return;
    }
    uint16_t i_adc = 0, v = fpga_dac_loop_probe();
    signal_engine_adc_spi(&i_adc);
    bool     has_src = (signal_engine_fpga_version() >= DAC_LOOP_SOURCE_MIN_GW);
    uint16_t in      = has_src ? fpga_dac_loop_input() : i_adc;
    /* The over-range trip is latched: the output sits at vmin until disarmed.  Without this a
       tripped loop looks like one holding a rail for no reason. */
    bool     tripped = fpga_dac_loop_tripped();
    heavy_release(conn_id);
    char payload[112];
    snprintf(payload, sizeof(payload), "{\"i\":%u,\"in\":%u,\"source\":\"%s\",\"v\":%u,\"tripped\":%s}",
             i_adc, in, dac_loop_src_str(has_src ? s_loop_src : (uint8_t)DAC_LOOP_SRC_ADC), v,
             tripped ? "true" : "false");
    send_ok_str(conn_id, payload);
}

/* Raw binary waveform upload (load_bin): consume bytes straight into the
   target buffer (adc_buf16 on v2) until `total` received, then ack and
   return to JSON.  Byte-counted (raw bytes may include newlines).
   The PROTO_LOAD receive path of command_handler_process: returns the bytes
   taken from buf (what follows the upload is JSON again). */
size_t load_bin_receive(int conn_id, const uint8_t *buf, size_t len) {
    size_t need  = load_bin_total - load_bin_have;
    size_t take  = need < len ? need : len;
    if (take > 0) {
        if (load_bin_psram) {
            /* Deep replay: write the chunk straight into PSRAM at the running
               byte offset from the TOP-ANCHORED base (fixed at load_bin arm from
               `total`; the bus is held for the whole upload).  psram_write
               tCEM-chunks internally, so an arbitrary `take` is fine. */
            psram_write(signal_engine_dac_psram_base() + (uint32_t)load_bin_have,
                        buf, (uint32_t)take);
        } else if (load_bin_dst) {
            memcpy(load_bin_dst + load_bin_have, buf, take);
        }
        load_bin_have += take;
        /* Progress -> push the idle deadline out so a slow-but-live upload isn't
           killed mid-stream.  Throttled to once per LOAD_BIN_REARM_BYTES to keep the
           clock reads off the hot per-chunk path. */
        if (load_bin_have - load_bin_rearm_at >= LOAD_BIN_REARM_BYTES) {
            load_bin_deadline = make_timeout_time_ms(LOAD_BIN_TIMEOUT_MS);
            load_bin_rearm_at = load_bin_have;
        }
        /* Cloud only: ack cumulative bytes so the server paces to our drain rate
           (flow control). Gated to tunnel conns so direct-TCP clients (SDK/CLI/
           hwe2e) that don't expect interleaved ack lines are unaffected. */
        if (is_tunnel_conn(conn_id) &&
            load_bin_have - load_bin_ack_at >= LOAD_BIN_ACK_INTERVAL) {
            load_bin_ack_at = load_bin_have;
            char m[32];
            snprintf(m, sizeof(m), "{\"ack\":%u}", (unsigned)load_bin_have);
            cloud_send_json_line(conn_id, m);
        }
    }
    if (load_bin_have >= load_bin_total) {
        replay_len      = load_bin_total / 2u;   /* 16-bit samples */
        replay_in_psram = load_bin_psram;        /* trace lives in PSRAM => deep replay */
        if (load_bin_psram) psram_stage_len = (uint32_t)load_bin_total;
        conn_proto_set(conn_id, PROTO_JSON);   /* heavy stays claimed for `replay` */
        loaded_hold_conn  = conn_id;      /* ...for LOADED_HOLD_MS at most */
        loaded_hold_until = make_timeout_time_ms(LOADED_HOLD_MS);
        char payload[32];
        snprintf(payload, sizeof(payload), "{\"total\":%u}",
                 (unsigned)replay_len);
        send_ok_str(conn_id, payload);
        /* Read a few samples back (bus still held) so the console shows the DAC data really
           made it into PSRAM and isn't flat — a quick check when a replayed DAC doesn't
           seem to reach the target. */
        if (load_bin_psram)
            dac_psram_verify_log(signal_engine_dac_psram_base(), (uint32_t)load_bin_total);
        load_bin_clear();   /* releases the shared bus if this was a psram upload */
    }
    return take;
}
