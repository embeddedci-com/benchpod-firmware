#include "command_handler.h"
#include "command_handler_internal.h"   /* reply helpers + gate query shared with the CAN/OTA subsystem files */
#include "console.h"
#include "at_driver.h"
#include "signal_engine.h"
#include "stm32h5xx_hal.h"   /* NVIC_SystemReset for the psram_recover command */
#include "psram_alloc.h"
#include "psram.h"          /* deep DAC replay: stage the waveform into PSRAM */
#include "fpga_config.h"    /* FPGA_PSRAM_* region map, FPGA_DAC_BASE_FOR, caps */
#include "sensor_sim.h"
#include "wifi_manager.h"
#include "gpio_control.h"
#include "target_power.h"
#include "scpi_server.h"
#include "device_identity.h"
#include "i2c_bus.h"
#include "can_bus.h"
#include "cal_data.h"
#include "adc_scale.h"     /* circular-mean + unwrap for adc_read's sample burst */
#include "pico_compat.h"   /* sleep_ms (yields to FreeRTOS) */
#include "ina238.h"
#include "b64url.h"
#include "cloud_config.h"
#include "cloud_client.h"
#include "net_server.h"   /* net_eth_stop/start/restart for the `eth` command */
#include "config_store.h"
#include "esp_wifi_ctrl.h"
#include "esp_hosted_spi.h"
#include "dap.h"
#include "board_info.h"
#include "board_rev.h"
#include "usb_cc.h"
#include "nrst_ctrl.h"
#include "bp_json.h"     /* shared flat-JSON parser + bounds-tracked emitter */
#include "bp_limits.h"   /* coupled cloud/command buffer sizes */
#include "dac_loop_params.h"  /* closed-loop DAC: curve upsample + parameter validation (host-tested) */
#include "fault.h"       /* reset cause + last-crash summary for status */
#include "boot_guard.h"  /* safe mode: status fields + the iCE40/PSRAM-off command gate */
#include "sys_health.h"  /* heap/stack headroom for status */
#include "bp_err.h"      /* shared error vocabulary (bp_err_str) */
#include "ice40_flash.h"
#include "version.h"     /* FIRMWARE_VERSION (single source) */
#include "ota.h"         /* firmware OTA (PSRAM-staged) */
#include "hw_lock.h"     /* serialize the shared I2C bus (power_status vs the profile sampler) */
#include "la_pins.h"     /* LA pin ownership table + capture-trigger parsing/messages */
#include "power_profile.h"  /* INA238 rail profile sampler (poll + status) */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "pico/time.h"   /* absolute_time_t, get_absolute_time, *_diff_us (UART escape guard timing) */

#define CHUNK_SAMPLES    256

/* JSON parsing/emitting is shared (see bp_json.h).  These thin aliases keep the
   handler bodies below reading the way they did before the parser was unified. */
#define json_get_value      bp_json_get
#define json_flag           bp_json_flag
#define json_get_byte_array bp_json_byte_array

/* ---- Response helpers ---- */

/* send_error / send_ok_str / heavy_in_flight have external linkage (declared in
   command_handler_internal.h) so the extracted CAN/OTA subsystem files can use them. */

void send_ok_str(int conn_id, const char *payload_json) {
    /* 256, and it has now grown TWICE — 128 -> 192 when the control-loop arm reply started
       echoing the input source, 192 -> 256 when v30 added the derived input map.  Worst case
       MEASURED, not estimated: the full arm payload is 192 bytes and the
       {"status":"ok","data":…} wrapper takes it to 216.  It is worth sizing carefully because
       of the failure mode — "reply too large" on a command that WORKED: the loop is armed and
       driving, and the caller is told it failed.  Check any new reply field against this. */
    char resp[256];
    bp_emit_t e;
    bp_emit_init(&e, resp, sizeof(resp));
    bp_emit_raw(&e, "{\"status\":\"ok\",\"data\":");
    bp_emit_raw(&e, payload_json);   /* caller-built, already-valid JSON value */
    bp_emit_raw(&e, "}\n");
    /* Skip tracing the keepalive pong — the web UI pings on a timer, which would
       otherwise flood the console with identical ok lines (the "<-" side is
       already suppressed via cmd_is_noisy_poll). */
    if (strcmp(payload_json, "\"pong\"") != 0)
        printf("[cmd] -> ok  data=%s\n", payload_json);
    if (!bp_emit_ok(&e)) {
        /* Payload didn't fit the fixed reply buffer — send a structured error
           rather than a truncated (malformed) ok.  Handlers whose payload can be
           large build+send their reply directly instead of via send_ok_str. */
        send_error(conn_id, bp_err_str(BP_ERR_TOO_LARGE));
        return;
    }
    if (at_send_data(conn_id, (const uint8_t *)resp, bp_emit_len(&e)) != 0) {
        at_close_connection(conn_id);   /* reclaim slot if client already left */
    }
}

/* Big enough for the longest message a caller can build (LA_PINS_ERR_MAX) even if bp_emit_jstr
   had to escape every single byte into two.  The pin/pull-conflict messages carry a JSON snippet
   of the fix, so they really do escape a run of quotes: at 160 bytes they overflowed, and because
   a full emitter drops every later write the closing "}\n" went with them — leaving the client
   waiting for a newline that never came, until its socket timed out. */
#define CH_ERR_RESP_MAX  (2u * LA_PINS_ERR_MAX + 48u)
_Static_assert(CH_ERR_RESP_MAX > LA_PINS_ERR_MAX, "error reply must outgrow its longest message");

void send_error(int conn_id, const char *message) {
    char resp[CH_ERR_RESP_MAX];
    bp_emit_t e;
    bp_emit_init(&e, resp, sizeof(resp));
    bp_emit_raw(&e, "{\"status\":\"error\",\"message\":");
    bp_emit_jstr(&e, message);        /* escaped — a message with quotes stays valid JSON */
    bp_emit_raw(&e, "}\n");
    if (!bp_emit_ok(&e)) {
        /* Never send a torn line — the client blocks on the missing newline.  The full text is
           still in the device log above. */
        bp_emit_init(&e, resp, sizeof(resp));
        bp_emit_raw(&e, "{\"status\":\"error\",\"message\":\"error message too long\"}\n");
    }
    printf("[cmd] -> error: %s\n", message);
    if (at_send_data(conn_id, (const uint8_t *)resp, bp_emit_len(&e)) != 0) {
        at_close_connection(conn_id);
    }
}

/* Every LA-bank operation (la / la_capture / dap_start / uart_proxy / i2c-sensor)
   needs the LA I/O voltage chosen first — the TPS2116 mux is left "unset" at boot
   so the host must explicitly pick 1.8 V or 3.3 V for the DUT.  Returns true when
   a voltage is set; otherwise emits an error and returns false. */
bool require_la_voltage(int conn_id) {
    if (la_vccio_get_mv() == LA_VCCIO_UNSET) {
        send_error(conn_id, "la voltage not set; set it with la_voltage (mv 1800 or 3300) first");
        return false;
    }
    return true;
}

/* ---- Cloud command capture (CH_CLOUD_CONN) ----
   A command.request arriving over the cloud WebSocket is dispatched through the
   normal handlers with conn_id = CH_CLOUD_CONN; at_send_data() routes the reply
   into this buffer instead of the TCP server so the cloud client can wrap it in
   a command.response frame. Single-threaded net loop -> one capture at a time.
   Sized to match the cloud client's reply[] (BP_CLOUD_REPLY_MAX). */
static char   cloud_cap_buf[BP_CLOUD_REPLY_MAX];
static size_t cloud_cap_len = 0;
static bool   cloud_cap_active = false;

void command_handler_cloud_capture_append(const uint8_t *buf, size_t len) {
    if (!cloud_cap_active) return;
    size_t space = sizeof(cloud_cap_buf) - 1 - cloud_cap_len;
    if (len > space) len = space;
    memcpy(cloud_cap_buf + cloud_cap_len, buf, len);
    cloud_cap_len += len;
    cloud_cap_buf[cloud_cap_len] = '\0';
}

/* ---- ADC buffer and chunk sender ---- */

static uint8_t adc_cmd_buf[SIGNAL_BUF_SIZE];

/* v2 captures are 16-bit (MCP33131); samples are read back from the PSRAM into
   this buffer and streamed as 16-bit JSON values. */
/* Small MONOLITHIC ADC read-back buffer, and the staging buffer for `load`/`replay`.
   DECOUPLED from ADC_CAP_MAX_SAMPLES (which is now the multi-MB PSRAM region size):
   the shallow `capture`/`stream`/`test` verbs and capture->replay use this fixed 64 KB
   buffer, while DEEP captures (`capture_dual`) stream CHUNKED straight out of PSRAM via
   the bulk sender and never touch it.  Sizing this to ADC_CAP_MAX_SAMPLES would ask for
   several MB of SRAM the STM32H563 does not have. */
#define ADC_MONO_BUF_SAMPLES 32768u
static uint16_t adc_buf16[ADC_MONO_BUF_SAMPLES];

/* Number of valid samples currently held in adc_cmd_buf for `replay` to play
   out the DAC.  Set by a successful `capture`/`stream` (the just-recorded
   trace) or by a `load` upload (a host-supplied trace).  0 = nothing to
   replay yet. */
static size_t replay_len = 0;

/* Binary-upload (`load_bin`) state while a connection is in PROTO_LOAD raw mode:
   destination buffer + capacity (adc_buf16 on v2, adc_cmd_buf otherwise), total
   bytes expected, bytes received, and the arming conn (only its raw bytes count). */
static uint8_t *load_bin_dst   = NULL;
static size_t   load_bin_total = 0;
static size_t   load_bin_have  = 0;
static int      load_bin_conn  = -1;

/* Deferred reboot for the `psram_recover` command: send the ack first, then reset a beat
   later (command_handler_poll) so the reply flushes to the client before the pod reboots. */
static bool             s_recover_pending;
static absolute_time_t  s_recover_at;
/* Deep-replay staging: when true, load_bin streams the raw bytes STRAIGHT INTO
   PSRAM (at the top-anchored DAC base) instead of the adc_buf16 RAM buffer — for
   waveforms too big for the 4 KB DAC BRAM (up to the full 8 MB region).  The
   shared quad bus is held (psram_bus_acquire) for the whole upload and released
   at completion so the iCE40 can read it back for CMD_START_DAC_PSRAM. */
static bool     load_bin_psram = false;
/* Set at load completion: the staged trace lives in PSRAM (replay must use the
   deep CMD_START_DAC_PSRAM path, not the BRAM LOAD_WAVE). */
static bool     replay_in_psram = false;
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

/* ---- Heavy-op gate ----
   capture/stream/measure/test all share the single adc_cmd_buf and
   chunk_buf, so only one may run at a time.  A second client requesting a
   heavy op while one is in flight is rejected with "busy" rather than
   silently corrupting the in-flight transfer.  Light commands (ping,
   status, generate, gpio_*) use only local buffers and are not gated.

   ONE owner enforces the invariant: heavy_in_flight() is the single source of
   truth for "a capture/measure/LA/bulk-send is running", and heavy_begin() is
   the single entry point every heavy handler goes through (reject-if-busy then
   claim-the-gate).  This replaced the duplicated
   `if (v2cap.active || bulk.active) busy; if (!heavy_try_claim) busy;` pair that
   used to sit — subtly inconsistently — at the top of ~10 handlers. */
#define CH_MAX_CONN   5
static int heavy_owner = -1;   /* conn_id holding adc_cmd_buf, or -1 */

static bool heavy_try_claim(int conn_id) {
    if (heavy_owner != -1 && heavy_owner != conn_id) return false;
    heavy_owner = conn_id;
    return true;
}

static void heavy_release(int conn_id) {
    if (heavy_owner == conn_id) heavy_owner = -1;
}

/* True while any async capture path owns the shared buffers: a v2 PSRAM
   capture/measure (v2cap), a deep LA→PSRAM capture (lacap), or a paced bulk send
   (bulk).  Declared here; the structs are defined just above. */
bool heavy_in_flight(void);   /* external linkage: also used by command_handler_ota.c */

/* The single gate for a heavy handler: reject with "busy" if a heavy op is in
   flight or another connection owns the gate, otherwise claim it for conn_id and
   return true.  On false it has already sent the error. */
static bool heavy_begin(int conn_id) {
    if (heavy_in_flight() || !heavy_try_claim(conn_id)) {
        send_error(conn_id, bp_err_str(BP_ERR_BUSY));
        return false;
    }
    return true;
}

/* Exposed to the SCPI handler so its blocking captures share the same single-
   ADC mutual exclusion as the JSON capture/stream/measure/test commands.  Same test as
   heavy_begin: OTA staging never claims heavy_owner, so the claim alone let a SCPI capture
   write PSRAM 0 (the LA region, where OTA stages its image) mid-OTA. */
bool command_handler_acquire_adc(int conn_id) {
    return !heavy_in_flight() && heavy_try_claim(conn_id);
}
void command_handler_release_adc(int conn_id) { heavy_release(conn_id); }

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
} bulk;

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

/* Single source of truth for "a heavy op is running" (see the gate section).
   OTA staging/verify also owns the PSRAM bus, so it counts as heavy — this gives
   OTA and captures mutual exclusion (a capture can't start mid-OTA and vice-versa). */
bool heavy_in_flight(void) {
    ota_state_t o = ota_get_state();
    return v2cap.active || lacap.active || dualcap.active || bulk.active ||
           o == OTA_RECEIVING || o == OTA_VERIFIED;
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
        if (avail < 96) return;

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

/* ---- Cloud throughput speed test -----------------------------------------
   A synthetic transfer used to BENCHMARK the cloud data path (device<->server
   over the WS/Cloudflare), independent of any real capture/DAC hardware. It runs
   over a TUNNEL conn (streaming is banned on the single-reply command channel),
   reuses the same conn_tx/altcp_sndbuf pacing as bulk_pump / load_bin, and adds
   NO large static buffers (bss is tight). Two directions:
     "up"   — device streams `total` synthetic bytes out (server times receipt);
     "down" — device sinks `total` bytes (PROTO_SPEEDTEST, counts+discards) then
              acks — exercises the same direction as a load_bin upload.
   Completion is marked with a `{"speedtest":"done","bytes":N}` line the server
   watches for. Only one runs at a time (one cloud data path). */
static struct {
    bool   active;   /* upload source running */
    int    conn;
    size_t total;
    size_t sent;
} sptx;

/* Ack cadence for the download sink: the device reports cumulative bytes-received
   every SPEEDTEST_ACK_INTERVAL so the server can pace to a byte-window and never
   get more than a window ahead of what the pod has actually taken off the wire.
   This is the end-to-end backpressure the server->device path otherwise lacks
   (conn.Write into Cloudflare succeeds even when CF->pod delivery has stalled). */
#define SPEEDTEST_ACK_INTERVAL 8192u

static struct {
    size_t total;    /* download sink target (conn is in PROTO_SPEEDTEST) */
    size_t recv;
    size_t last_ack; /* recv value at the last ack sent */
} sprx;

/* Emit one raw newline-delimited JSON line back over the (tunnel) conn. */
static void cloud_send_json_line(int conn_id, const char *json) {
    char line[72];
    int n = snprintf(line, sizeof(line), "%s\n", json);
    if (n > 0 && (size_t)n < sizeof(line)) at_send_data(conn_id, (const uint8_t *)line, (size_t)n);
}

static void speedtest_ack_progress(int conn_id, size_t bytes) {
    char m[64];
    snprintf(m, sizeof(m), "{\"speedtest\":\"ack\",\"bytes\":%u}", (unsigned)bytes);
    cloud_send_json_line(conn_id, m);
}

static void speedtest_ack_done(int conn_id, size_t bytes) {
    char m[64];
    snprintf(m, sizeof(m), "{\"speedtest\":\"done\",\"bytes\":%u}", (unsigned)bytes);
    cloud_send_json_line(conn_id, m);
}

/* Cancel any in-flight speed test bound to conn_id (tunnel reset / close). */
static void speedtest_cancel(int conn_id) {
    if (sptx.active && sptx.conn == conn_id) sptx.active = false;
}

/* Paced upload pump — mirrors bulk_pump: fill the conn's TX ring as far as its
   free space allows, resume next worker poll. Synthetic 0x5A payload from the
   stack (no static buffer). */
static void speedtest_pump(void) {
    if (!sptx.active) return;
    int conn = sptx.conn;
    uint8_t chunk[256];
    memset(chunk, 0x5A, sizeof(chunk));
    while (sptx.sent < sptx.total) {
        int avail = at_send_avail(conn);
        if (avail < 64) return;                 /* ring full — resume next poll */
        size_t room = (size_t)avail;
        if (room > sizeof(chunk)) room = sizeof(chunk);
        size_t need = sptx.total - sptx.sent;
        if (room > need) room = need;
        if (at_send_data(conn, chunk, room) != 0) { sptx.active = false; return; }
        sptx.sent += room;
    }
    sptx.active = false;
    speedtest_ack_done(conn, sptx.sent);        /* ordered after the payload in the ring */
}
/* handle_speedtest() is defined later (needs proto[], declared below). */

/* ---- Per-connection command reassembly ----
   ESP-AT delivers one +IPD per TCP segment; a JSON command may be split
   across segments, or two commands may share one segment.  We buffer per
   connection and dispatch on each '\n'. */
/* Sized to match the cloud command buffer (BP_CLOUD_CMD_IN_MAX) so a compact closed-loop
   curve LUT uploaded over LAN/serial reassembles in one line, same as over cloud. */
#define LINE_ASM_SIZE ((int)BP_CLOUD_CMD_IN_MAX)
typedef struct {
    char   buf[LINE_ASM_SIZE];
    size_t len;
    bool   discarding;   /* dropping an over-long line until its newline */
} line_asm_t;
/* Sized to cover the TCP conns (0..CH_MAX_CONN-1) plus the console (CH_CONSOLE_CONN), cloud command
   (CH_CLOUD_CONN) and the cloud tunnels (CH_CLOUD_TUNNEL_CONN..CH_CLOUD_TUNNEL_CONN_LAST) pseudo-
   connections, so a JSON handler that touches proto[]/line_asm[] for any of them stays in bounds. The
   tunnel conns, unlike the others, run the full buffered state machine in command_handler_process(). */
_Static_assert(CH_CONSOLE_CONN >= CH_MAX_CONN, "console conn id must sit above the TCP conns");
_Static_assert(CH_CLOUD_CONN > CH_CONSOLE_CONN, "cloud conn id must sit above the console conn");
_Static_assert(CH_CLOUD_TUNNEL_CONN > CH_CLOUD_CONN, "tunnel conn id must sit above the cloud conn");
_Static_assert(CH_CLOUD_TUNNEL_CONN_COUNT >= 1, "need at least one tunnel conn");
static line_asm_t line_asm[CH_CLOUD_TUNNEL_CONN_LAST + 1];

/* Per-connection protocol, decided from the first non-whitespace byte:
   '{' -> JSON, anything else -> SCPI.  Lets a VISA/SCPI client and a JSON
   client share the single ESP-AT TCP server.  PROTO_DAP is entered only by an
   explicit JSON "dap_start" command and routes the connection's raw bytes to
   the on-pod CMSIS-DAP processor as length-framed packets until a zero-length
   frame leaves DAP mode. */
typedef enum { PROTO_UNKNOWN, PROTO_JSON, PROTO_SCPI, PROTO_UART, PROTO_DAP, PROTO_LOAD, PROTO_SPEEDTEST } proto_t;
static proto_t proto[CH_CLOUD_TUNNEL_CONN_LAST + 1];

/* is_tunnel_conn: true for any of the N cloud byte-tunnel pseudo-connections. */
static inline bool is_tunnel_conn(int conn_id) {
    return conn_id >= CH_CLOUD_TUNNEL_CONN && conn_id <= CH_CLOUD_TUNNEL_CONN_LAST;
}

/* conn_runs_state_machine: true for the TCP conns and the cloud tunnels — the connections whose raw
   bytes flow through command_handler_process()'s proto state machine (JSON line assembly plus the
   DAP/UART raw modes). The console/cloud-command pseudo-conns dispatch single lines instead. */
static inline bool conn_runs_state_machine(int conn_id) {
    return (conn_id >= 0 && conn_id < CH_MAX_CONN) || is_tunnel_conn(conn_id);
}

/* conn_has_socket: true only for real LwIP TCP conns — the ones with a pcb backing tcp_nodelay.
   The tunnel reuses the state machine but has no socket, so socket-only calls (at_set_tcp_nodelay)
   must be skipped for it. */
static inline bool conn_has_socket(int conn_id) {
    return conn_id >= 0 && conn_id < CH_MAX_CONN;
}

/* ---- UART transparent proxy state (one active connection at a time) -------
   Entered by JSON "uart_proxy_start"; the connection then carries raw bytes
   to/from the DUT UART.  Exit: the client closes the socket (primary), or
   sends the guard-timed Hayes escape  <≥1s idle> +++ <≥1s idle>  to return to
   JSON on the same connection.  RX from the DUT is drained to the client in
   command_handler_poll() (it is asynchronous, unlike SWD's `c` replies). */
#define UART_ESC_GUARD_MS  1000
static int             uart_proxy_conn = -1;     /* conn_id owning the proxy, or -1 */
static uint8_t         uart_plus_count = 0;      /* consecutive '+' in an escape candidate */
static bool            uart_plus_pending = false;/* saw "+++", awaiting the trailing guard */
static absolute_time_t uart_last_rx_at;          /* last byte received from the client */
static absolute_time_t uart_last_plus_at;        /* time of the most recent '+' */
static uint8_t         uart_rx_buf[256];         /* DUT→client drain buffer */
/* True while the proxy is SUSPENDED for the duration of a load_bin upload: we stop touching the
   iCE40 for UART entirely (no per-poll SPI drain) so nothing competes with the upload's PSRAM
   writes on the worker task; on resume we flush the FIFO once so stale bytes aren't forwarded. */
static bool            uart_upload_suspended = false;
/* RX-channel pull-up auto-hold: a DUT's TX pin floats before its firmware brings
   up the UART, and an un-held RX line decodes as a flood of 0x00 during boot. We
   pull the RX channel high (LA1-8 only) for the session and restore it on exit. */
static int             uart_rx_pu_ch   = -1;     /* RX LA channel we forced a pull-up on, or -1 */
static bool            uart_rx_pu_prev = false;  /* its pull-up state before we forced it */

static void uart_proxy_end(int conn_id, bool restore_json) {
    fpga_uart_disable();
    if (uart_rx_pu_ch >= 1) {                     /* restore the RX pull-up we forced on */
        /* A failed restore leaves the DUT's line biased by a resistor nobody asked for, which
           then shows up as an unexplained idle level on the next session — say so. */
        int rc = pca9555_set_la_pullup((uint8_t)uart_rx_pu_ch, uart_rx_pu_prev);
        if (rc != 0)
            printf("[uart] WARNING: could not restore LA%d's pull-up to %s (rc %d)\n",
                   uart_rx_pu_ch, uart_rx_pu_prev ? "on" : "off", rc);
        uart_rx_pu_ch = -1;
    }
    la_pins_release_fn(LA_FN_UART_RX);
    la_pins_release_fn(LA_FN_UART_TX);
    uart_proxy_conn   = -1;
    uart_plus_count   = 0;
    uart_plus_pending = false;
    uart_upload_suspended = false;   /* next proxy starts un-suspended */
    if (restore_json && conn_runs_state_machine(conn_id)) {
        proto[conn_id] = PROTO_UNKNOWN;           /* re-detect the next byte as JSON/SCPI */
        if (conn_has_socket(conn_id))
            at_set_tcp_nodelay(conn_id, false);   /* back to normal Nagle batching (socket only) */
    }
}

/* CMSIS-DAP-over-tunnel reassembly (PROTO_DAP).  The SWD engine is a single
   shared resource (fpga_swd_arm rejects a second arm), so at most one connection
   is ever in DAP mode — one static reassembly buffer suffices.  A frame is
   [len_lo][len_hi][packet]; a zero-length frame leaves DAP mode. */
static uint8_t dap_rx[2 + DAP_PACKET_SIZE];
static size_t  dap_rx_have;                 /* bytes accumulated incl. 2-byte header */
static uint8_t dap_resp[2 + DAP_PACKET_SIZE];

/* Disarming hands SWCLK/SWDIO back to the LA bank, so the ownership table has to follow. */
static void swd_disarm_and_release(void) {
    fpga_swd_disarm();
    la_pins_release_fn(LA_FN_SWD_CLK);
    la_pins_release_fn(LA_FN_SWD_DIO);
}

void command_handler_poll(void) {
    /* ---- deferred reboot for `psram_recover`: fire once the ack has had time to flush ---- */
    if (s_recover_pending && time_reached(s_recover_at)) {
        printf("[recover] rebooting to clear the PSRAM datapath (boot auto-reflashes the iCE40)\n");
        NVIC_SystemReset();   /* does not return */
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
        if (conn_runs_state_machine(conn) && proto[conn] == PROTO_LOAD)
            proto[conn] = PROTO_JSON;
        heavy_release(conn);
        load_bin_clear();
        send_error(conn, "load_bin timeout");
    }

    /* ---- LA pin housekeeping: a finished step train releases its pins; an SWD session that
       hit its inactivity backstop (fpga_swd_poll was never called until now) releases its
       two.  Both are cheap RAM checks until something is actually running. ---- */
    la_step_poll();
    fpga_swd_poll();
    if (!fpga_swd_armed() && la_pins_mask_fn(LA_FN_SWD_CLK)) {
        la_pins_release_fn(LA_FN_SWD_CLK);
        la_pins_release_fn(LA_FN_SWD_DIO);
    }

    /* ---- power profile: one INA238 register read per pass, plus its reply pacing ---- */
    power_profile_poll();
    power_profile_service();

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
                last_cap.la_rate_hz  = 0;
                printf("[cmd] la_capture: PSRAM data ready (%u samples), streaming back over websocket...\n",
                       (unsigned)samples);
                bulk_begin_capture16(cid, 0, samples);
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

    /* ---- cloud speed-test upload: paced synthetic byte source ---- */
    speedtest_pump();

    /* ---- UART proxy: stream DUT→client and apply the +++ trailing guard ---- */
    fpga_uart_tx_pump();   /* queued proxy TX bytes -> the FPGA FIFO as it drains */
    if (uart_proxy_conn >= 0) {
        if (load_bin_conn >= 0) {
            /* A DAC-replay upload is streaming in — SUSPEND the proxy for its duration.  The pod's
               single net task can't encrypt+send a chatty console AND receive the (large) upload at
               once, and an earlier version still ran the per-poll FIFO drain here — ~1 ms of blocking
               iCE40 SPI at a full FIFO, stealing the worker from the upload's PSRAM writes.  So while
               a load_bin is active we do NOT touch the iCE40 for UART at all (no drain, no forward).
               DUT output during the transfer is dropped either way (it was already being discarded);
               we just stop paying for it.  The FIFO is left to fill and flushed once on resume. */
            if (!uart_upload_suspended)
                printf("[uart] proxy suspended (conn %d) for load_bin upload on conn %d\n",
                       uart_proxy_conn, load_bin_conn);
            uart_upload_suspended = true;
        } else {
            if (uart_upload_suspended) {
                /* Upload finished — discard whatever piled up in the FIFO while we were suspended,
                   so the resumed stream is fresh DUT output rather than stale/overflowed bytes. One
                   read empties the 256-deep FIFO. */
                uart_upload_suspended = false;
                (void)fpga_uart_read(uart_rx_buf, sizeof(uart_rx_buf));
            }
            size_t n = fpga_uart_read(uart_rx_buf, sizeof(uart_rx_buf));
            if (n > 0) {
                if (at_send_data(uart_proxy_conn, uart_rx_buf, n) != 0) {
                    /* client gone — auto-stop and reclaim the slot */
                    int c = uart_proxy_conn;
                    uart_proxy_end(c, false);
                    at_close_connection(c);
                }
            }
            /* Hayes escape: "+++" was seen; if a full guard has elapsed with no
               further client bytes, leave the proxy and return to JSON. */
            if (uart_proxy_conn >= 0 && uart_plus_pending &&
                absolute_time_diff_us(uart_last_plus_at, get_absolute_time())
                    >= (int64_t)UART_ESC_GUARD_MS * 1000) {
                printf("[uart] +++ escape — returning conn %d to JSON\n", uart_proxy_conn);
                uart_proxy_end(uart_proxy_conn, true);
            }
        }
    }

}

/* ---- Command handlers ---- */

/* Parse an optional "sample_rate_mhz" field into a sample rate in Hz.
   Returns 0.0f (= "auto-pick") when the field is absent.  Accepts decimals
   like 0.5, 1, 12.  Clamps to the achievable range; the firmware will pick
   the closest divider and log what it actually used. */
static float parse_sample_rate_hz(const char *json) {
    char sr_s[16] = {0};
    if (!json_get_value(json, "sample_rate_mhz", sr_s, sizeof(sr_s))) return 0.0f;
    float mhz = (float)atof(sr_s);
    if (mhz <= 0.0f) return 0.0f;   /* invalid → auto */
    return mhz * 1.0e6f;
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

static void handle_generate(int conn_id, const char *json) {
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

static void handle_capture(int conn_id, const char *json) {
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
    replay_len     = samples;
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
static void handle_capture_dual(int conn_id, const char *json) {
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
static void handle_capture_read(int conn_id, const char *json) {
    if (!last_cap.valid) {
        send_error(conn_id, "no capture to resume (run capture_dual first)");
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

static void handle_stream(int conn_id, const char *json) {
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
    replay_len     = samples;
}

static void handle_ping(int conn_id) {
    send_ok_str(conn_id, "\"pong\"");
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
static void handle_test(int conn_id, const char *json) {
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

static void handle_measure(int conn_id, const char *json) {
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
    replay_len     = samples;
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
static void handle_load(int conn_id, const char *json) {
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
    } else if (heavy_owner != conn_id) {
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

    char payload[64];
    snprintf(payload, sizeof(payload),
             "{\"offset\":%u,\"len\":%u,\"total\":%u}",
             (unsigned)offset, (unsigned)dec_len, (unsigned)replay_len);
    send_ok_str(conn_id, payload);
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
   adc_buf16 and replay_len is total/2 samples; v1-mode uses adc_cmd_buf 1:1.
   Refused over the single-reply cloud command channel (needs a stream conn).
     {"cmd":"load_bin","total":N}
   Reply (immediate): {"status":"ok","data":{"ready":N}}  -> stream N bytes next.
   Reply (after N bytes): {"status":"ok","data":{"total":S}}  (S = samples). */
static void handle_load_bin(int conn_id, const char *json) {
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

    if (to_psram) {
        /* Take the shared quad bus so the STM32 can write PSRAM directly over XSPI;
           released at completion (load_bin_clear) so the iCE40 can read it back for
           CMD_START_DAC_PSRAM.  The STM32 owns the bus only during the upload; once
           released a capture MAY run concurrently with the replay (>=v18 arbiter). */
        psram_bus_acquire();
        /* Fix the top-anchored DAC base now that we know the total length, so the
           staging writes and the later CMD_START_DAC_PSRAM agree on the base. */
        signal_engine_dac_psram_stage((uint32_t)total);
        load_bin_psram = true;
        load_bin_dst   = NULL;                 /* bytes go to PSRAM, not a RAM buffer */
    } else {
        load_bin_psram = false;
        load_bin_dst   = (uint8_t *)adc_buf16;
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
    proto[conn_id]    = PROTO_LOAD;   /* subsequent raw bytes go to the buffer/PSRAM */

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
static void handle_replay(int conn_id, const char *json) {
    char samples_s[16] = {0};
    json_get_value(json, "samples", samples_s, sizeof(samples_s));
    float  sr_hz   = parse_sample_rate_hz(json);   /* 0 = max rate */
    size_t samples = samples_s[0] ? (size_t)strtoul(samples_s, NULL, 10) : replay_len;

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
static void handle_dac_stop(int conn_id) {
    dac_stop();
    send_ok_str(conn_id, "null");
}

/* `dac_set` — hold the DAC at a fixed DC level (multimeter-probe / fault
   injection).  Single-reply, so it rides the cloud command channel.  On v2 the
   FPGA DAC sequencer clocks a single byte continuously; `value` is the 0..255
   level and `divider` (optional) sets DAC_CLK = HFOSC / divider.
     {"cmd":"dac_set","value":0..255,"divider":N} */
static void handle_dac_set(int conn_id, const char *json) {
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

/* Unified logic-analyzer-pin command. The action is inferred from the fields:
 *
 *   {"cmd":"la","la":N,"steps":S,"delay_us":D[,"dir_la":M,"direction":0|1]}
 *        → run a step pulse train on LA N (1..14); the FPGA runs it autonomously.
 *   {"cmd":"la","la":N,"pullup":"on"|"off"}  → switch LA N's pull-up (LA1..8).
 *   {"cmd":"la","la":N}                       → report LA N's pull-up state.
 *   {"cmd":"la"}                              → bitmask of enabled LA pull-ups.
 *
 * This replaces the old gpio_set/gpio_step/pullup/pullup_status commands. Active
 * drive (the former gpio_set) is gone: a pull-up — or its absence, which leaves
 * the board's pull-down — sets a line's idle level instead. */
static void handle_la(int conn_id, const char *json) {
    if (!require_la_voltage(conn_id)) return;
    char steps_s[12] = {0};

    /* --- step pulse train: distinguished by the "steps" field --- */
    if (json_get_value(json, "steps", steps_s, sizeof(steps_s))) {
        char la_s[8] = {0}, delay_s[12] = {0}, dir_la_s[8] = {0}, dir_s[8] = {0};
        if (!json_get_value(json, "la", la_s, sizeof(la_s))) {
            send_error(conn_id, "missing la");
            return;
        }
        if (!json_get_value(json, "delay_us", delay_s, sizeof(delay_s))) {
            send_error(conn_id, "missing delay_us");
            return;
        }
        unsigned la       = (unsigned)atoi(la_s);
        int      steps    = atoi(steps_s);
        int      delay_us = atoi(delay_s);
        if (steps <= 0)    { send_error(conn_id, "invalid steps");    return; }
        if (delay_us <= 0) { send_error(conn_id, "invalid delay_us"); return; }

        /* Optional direction channel — driven before stepping (stepper dir). */
        unsigned dir_la = 0;
        bool     dir    = false;
        if (json_get_value(json, "dir_la", dir_la_s, sizeof(dir_la_s))) {
            dir_la = (unsigned)atoi(dir_la_s);
            if (json_get_value(json, "direction", dir_s, sizeof(dir_s)))
                dir = (atoi(dir_s) != 0);
        }

        /* Ownership: a free step/dir pin is CLAIMED for the train (and released when the
           fabric reports STEP_BUSY low again, from command_handler_poll); a gpio output is
           pulsed in place; anything else is a `pin conflict:`.  la_step_begin drives the
           direction pin and starts the train. */
        char err[LA_PINS_ERR_MAX];
        int rc = la_step_begin(la, (uint32_t)steps, (uint32_t)delay_us, dir_la, dir,
                               err, sizeof(err));
        if (rc != 0) { send_error(conn_id, err); return; }

        /* Non-blocking: the FPGA runs the train autonomously, so we report that
           it has started rather than waiting for completion. */
        char payload[80];
        snprintf(payload, sizeof(payload),
                 "{\"la\":%u,\"steps\":%d,\"delay_us\":%d,\"status\":\"started\"}",
                 la, steps, delay_us);
        send_ok_str(conn_id, payload);
        return;
    }

    /* --- no "la" field: report the pull-up bitmask, bit (la-1)=LA<la> --- */
    char la_s[8] = {0};
    if (!json_get_value(json, "la", la_s, sizeof(la_s))) {
        unsigned mask = 0;
        for (unsigned la = 1; la <= 8; la++) {
            if (pca9555_la_pullup_enabled((uint8_t)la)) mask |= (1u << (la - 1));
        }
        /* pullups_available says whether a pull-up CAN be engaged right now (bank at
           3.3 V); the mask is always the truth about what is engaged. */
        char payload[64];
        snprintf(payload, sizeof(payload), "{\"la_pullup_mask\":%u,\"pullups_available\":%d}",
                 mask, la_pullups_available() ? 1 : 0);
        send_ok_str(conn_id, payload);
        return;
    }

    /* --- pull-up set ("pullup":"on|off") or query for one pin (LA1..8) --- */
    unsigned la = (unsigned)atoi(la_s);
    if (la < 1 || la > 8) {
        send_error(conn_id, "no pull-up on this la");   /* LA9-14 / out of range */
        return;
    }

    char state_s[8] = {0};
    if (json_get_value(json, "pullup", state_s, sizeof(state_s))) {
        char c = state_s[0];
        bool on = (c == 'o' || c == 'O') ? (state_s[1] == 'n' || state_s[1] == 'N')
                                         : (atoi(state_s) != 0);
        /* The other direction of the pull-compatibility rule: LA7/LA8's resistor pulls DOWN,
           which an open-drain bus / an idle-high UART / SWDIO cannot live with.  Refuse before
           touching the expander so the wire never briefly contradicts the function on it. */
        char pull_err[LA_PINS_ERR_MAX];
        if (on && !la_pins_check_pull_enable(la, pull_err, sizeof(pull_err))) {
            send_error(conn_id, pull_err);
            return;
        }
        int rc = pca9555_set_la_pullup((uint8_t)la, on);
        if (rc == LA_PULLUP_ERR_VOLTAGE) {
            /* Not a failure to talk to the expander: the resistors are 3V3-referenced
               and the bank is at 1.8 V, so engaging one would drive the DUT above its
               own rail. Say which, so the host can switch the bank instead of retrying. */
            send_error(conn_id, "pull-ups are 3V3-referenced; not available with the LA bank at 1.8 V");
            return;
        }
        if (rc != 0) {
            send_error(conn_id, "pca9555 write failed");
            return;
        }
    }

    /* "pull" says which way the channel's fixed resistor goes: LA1-LA6 up,
       LA7/LA8 down. Without it "ohms" is ambiguous, and a client that assumed
       every biased channel pulls up would drive an open-drain bus the wrong way. */
    char payload[128];
    snprintf(payload, sizeof(payload),
             "{\"la\":%u,\"pullup\":%d,\"ohms\":\"%s\",\"pull\":\"%s\",\"pullups_available\":%d}",
             la, pca9555_la_pullup_enabled((uint8_t)la) ? 1 : 0,
             pca9555_la_pullup_ohms((uint8_t)la),
             pca9555_la_pull_is_down((uint8_t)la) ? "down" : "up",
             la_pullups_available() ? 1 : 0);
    send_ok_str(conn_id, payload);
}

static void handle_target_power(int conn_id, const char *json) {
    char efuse_s[8]  = {0};
    char state_s[8]  = {0};
    char delay_s[12] = {0};

    if (!json_get_value(json, "efuse", efuse_s, sizeof(efuse_s))) {
        send_error(conn_id, "missing efuse");
        return;
    }
    if (!json_get_value(json, "state", state_s, sizeof(state_s))) {
        send_error(conn_id, "missing state");
        return;
    }

    int      efuse    = atoi(efuse_s);
    bool     on       = (atoi(state_s) != 0);
    uint32_t delay_ms = 0;
    if (json_get_value(json, "delay_ms", delay_s, sizeof(delay_s)))
        delay_ms = (uint32_t)strtoul(delay_s, NULL, 0);

    /* delay_ms == 0 applies immediately; > 0 schedules and fires from the main
       poll loop, so the connection can switch to e.g. UART proxy meanwhile. */
    if (target_power_schedule(efuse, on, delay_ms) != 0) {
        send_error(conn_id, "invalid efuse");
        return;
    }

    /* "enabled" echoes the requested state; with delay_ms > 0 it is the state
       the eFuse will reach once the delay elapses. */
    char payload[56];
    snprintf(payload, sizeof(payload),
             "{\"efuse\":%d,\"enabled\":%d,\"delay_ms\":%u}",
             efuse, on ? 1 : 0, (unsigned)delay_ms);
    send_ok_str(conn_id, payload);
}

static void handle_target_status(int conn_id) {
    target_power_status_t e1, e2;
    target_power_get_status(1, &e1);
    target_power_get_status(2, &e2);

    /* Payload (~112 chars) + wrapper exceeds send_ok_str's 128-byte buffer,
       so build and send the response directly like handle_status. */
    char resp[224];
    snprintf(resp, sizeof(resp),
             "{\"status\":\"ok\",\"data\":{"
             "\"efuse1\":{\"enabled\":%d,\"fault\":%d,\"valid\":%d},"
             "\"efuse2\":{\"enabled\":%d,\"fault\":%d,\"valid\":%d},"
             "\"status_supported\":%s}}\n",
             e1.enabled, e1.fault, e1.valid,
             e2.enabled, e2.fault, e2.valid,
             e1.status_supported ? "true" : "false");
    if (at_send_data(conn_id, (const uint8_t *)resp, strlen(resp)) != 0) {
        at_close_connection(conn_id);
    }
}

/* `power_status` — read the on-board INA238 current monitors (0x40 = internal,
   0x44 = external supply). Reports bus voltage (mV) + current (µA) per rail;
   `ok:false` when a rail can't be read. Single-reply (cloud command channel). */
static void handle_power_status(int conn_id) {
    int ibus = 0, ish = 0, icur = 0;
    int ebus = 0, esh = 0, ecur = 0;
    /* I2C1 is shared with the console `ina` command, the TCA9554 expanders and the
       power_profile sampler, all of which take hw_lock — this one read did not, so a
       concurrent transaction could interleave and return a torn register. */
    hw_lock();
    bool int_ok = (ina238_read(I2C_ADDR_INA238_INTERNAL, &ibus, &ish, &icur) == 0);
    hw_unlock();
    hw_lock();
    bool ext_ok = (ina238_read(I2C_ADDR_INA238_EXTERNAL, &ebus, &esh, &ecur) == 0);
    hw_unlock();

    char resp[224];
    snprintf(resp, sizeof(resp),
             "{\"status\":\"ok\",\"data\":{"
             "\"internal\":{\"ok\":%s,\"bus_mv\":%d,\"current_ua\":%d},"
             "\"external\":{\"ok\":%s,\"bus_mv\":%d,\"current_ua\":%d}}}\n",
             int_ok ? "true" : "false", ibus, icur,
             ext_ok ? "true" : "false", ebus, ecur);
    if (at_send_data(conn_id, (const uint8_t *)resp, strlen(resp)) != 0) {
        at_close_connection(conn_id);
    }
}

static void handle_status(int conn_id) {
    const char *ip = wifi_get_ip();
    const char *wstate;
    wifi_state_t st = wifi_get_state();
    switch (st) {
        case WIFI_DISCONNECTED: wstate = "disconnected"; break;
        case WIFI_CONNECTING:   wstate = "connecting";   break;
        case WIFI_CONNECTED:    wstate = "connected";    break;
        case WIFI_READY:        wstate = "ready";        break;
        default:                wstate = "unknown";      break;
    }

    /* RSSI is null if not associated; otherwise the AP signal strength in
       dBm.  Querying it costs an AT round-trip (~10s of ms) so we only do
       it when associated. */
    char rssi_field[24] = "null";
    if (st == WIFI_CONNECTED || st == WIFI_READY) {
        int rssi = 0;
        if (wifi_get_rssi(&rssi) == 0) {
            snprintf(rssi_field, sizeof(rssi_field), "%d", rssi);
        }
    }

    /* Built with the bounds-tracked emitter: the payload has grown past what a
       hand-sized buffer safely holds, and last_crash is free-form text (escaped
       via bp_emit_jstr) so it can never break the JSON. */
    /* 1024, not 768: caps[] below is now the full, dynamically-built feature set (was a
       7-name literal), which adds ~150 B on a v2 pod.  The emitter is bounds-tracked and
       last_crash is free-form, so keep real headroom rather than sizing to today's payload. */
    /* 1152: the LA-pin / trigger / power-profile names below add another ~60 B of caps[].
       1344: safe_mode + safe_reason (up to ~130 B). */
    /* 1408: gateware + gateware_embedded. */
    char resp[1408];
    bp_emit_t e;
    bp_emit_init(&e, resp, sizeof(resp));
    bp_emit(&e, "{\"status\":\"ok\",\"data\":{"
                "\"device\":\"benchpod\",\"version\":\"%s\",\"board\":\"%s\",\"net\":\"%s\","
                "\"ip\":\"%s\",\"rssi_dbm\":%s,\"wifi\":\"%s\",\"cloud\":\"%s\",",
            FIRMWARE_VERSION, BOARD_NAME, wstate, ip, rssi_field,
            esp_wifi_ctrl_state_str(), cloud_client_state_str());
    bp_emit(&e, "\"rx_dropped\":%lu,\"tx_dropped\":%lu,"
                "\"adc_bits\":%d,\"adc_fullscale_mv\":%d,\"adc_channels\":%d,\"la_vccio_mv\":%d,",
            (unsigned long)console_rx_dropped(), (unsigned long)console_tx_dropped(),
            ADC_BITS, ADC_FULLSCALE_MV, ADC_CHANNELS, la_vccio_get_mv());
    /* Board revision decides which of the rev3 features below actually exist,
       so clients read it once here rather than probing each command. */
    bp_emit(&e, "\"board_rev\":\"%s\",\"board_rev_mv\":%d,\"nrst_pin\":%s,",
            board_rev_str(), board_rev_strap_mv(),
            nrst_ctrl_supported() ? "true" : "false");
    /* The gateware running in the iCE40 and the one this firmware embeds (0 = unknown). They
       differ after a firmware update until the boot-time gateware update has run. */
    bp_emit(&e, "\"gateware\":%u,\"gateware_embedded\":%u,\"loop_tripped\":%s,\"uart_rx_overflow\":%s,",
            (unsigned)signal_engine_fpga_version(), (unsigned)ice40_embedded_gw_version(),
            fpga_dac_loop_tripped() ? "true" : "false", fpga_uart_rx_overflowed() ? "true" : "false");
    bp_emit(&e, "\"psram\":\"%s\",\"psram_ok\":%s,\"heap_free\":%u,\"heap_min\":%u,\"stack_min\":%u,"
                "\"reset\":\"%s\",\"last_crash\":",
            psram_selftest_str(), signal_engine_psram_operable() ? "true" : "false",
            sys_health_heap_free(), sys_health_heap_min_free(),
            sys_health_stack_min_free(), fault_last_reset_str());
    bp_emit_jstr(&e, fault_last_crash_str());
    /* Safe mode (boot_guard.h): the reason says what is off and why; "" when not. */
    bp_emit(&e, ",\"safe_mode\":%s,\"safe_reason\":",
            boot_guard_safe_mode() ? "true" : "false");
    bp_emit_jstr(&e, boot_guard_reason());
    /* caps[]: the pod's ADVERTISED feature set, and the ONLY capability source a client on a
       direct LAN/serial connection ever sees (the richer `capabilities` frame goes to the cloud
       server alone).  It used to be a hardcoded 7-name literal that named no DAC feature at all,
       so a direct client could not tell that deep replay / the control loop / the co-trigger
       exist — the SDK skipped hardware tests for features this very pod passes over the cloud.
       The dynamic half now comes from signal_engine_caps(), the same call the cloud announce
       makes, so the two can no longer drift. */
    bp_emit_raw(&e, ",\"caps\":[\"signal\",\"gpio\",\"power\",\"swd\",\"i2c_sensor\",\"uart\",\"la\""
                    ",\"scope\",\"analyzer\",\"command\",\"tunnel\",\"ota\""
    /* Firmware-side features that do not depend on the gateware image. */
                    ",\"la_pins\",\"power_profile\",\"capture_b64\"");
    /* Build-time analog features (what the BOARD has). */
    if (DAC_AC)     bp_emit_raw(&e, ",\"dac\"");
    if (DAC_DC)     bp_emit_raw(&e, ",\"dac_dc\"");
    if (DAC_REPLAY) bp_emit_raw(&e, ",\"dac_replay\"");
    /* Run-time gateware features (what the RUNNING iCE40 image has). */
    signal_engine_caps_t fcaps;
    signal_engine_caps(&fcaps);
    if (fcaps.deep_replay)    bp_emit_raw(&e, ",\"dac_deep_replay\"");
    if (fcaps.control_loop)   bp_emit_raw(&e, ",\"dac_control_loop\"");
    if (fcaps.cotrig)         bp_emit_raw(&e, ",\"dac_cotrig\"");
    if (fcaps.loop_sources)   bp_emit_raw(&e, ",\"dac_loop_sources\"");
    if (fcaps.loop_input_map) bp_emit_raw(&e, ",\"dac_loop_input_map\"");
    if (fcaps.gpio_read)      bp_emit_raw(&e, ",\"gpio_read\"");
    if (fcaps.capture_trigger) bp_emit_raw(&e, ",\"capture_trigger\"");
    /* rev3 hardware features (what the BOARD has, decided by board_rev). */
    if (nrst_ctrl_supported()) bp_emit_raw(&e, ",\"nrst_pin\"");
    if (usb_cc_supported())    bp_emit_raw(&e, ",\"usb_cc\"");
    bp_emit_raw(&e, "]}}\n");
    if (!bp_emit_ok(&e)) { send_error(conn_id, "status too large"); return; }
    if (at_send_data(conn_id, (const uint8_t *)resp, bp_emit_len(&e)) != 0) {
        at_close_connection(conn_id);
    }
}

/* ---- Cloud connection provisioning ----
   cloud_set persists the server endpoint + this device's id and tells the
   firmware to open the control WebSocket itself (and reconnect on boot).
   Provisioned once by `benchpod register`. */
static bool cloud_truthy(const char *s) {
    return s[0] == 't' || s[0] == 'T' || s[0] == '1';
}

static void handle_cloud_set(int conn_id, const char *json) {
    char host[CLOUD_HOST_MAX]      = {0};
    char did[CLOUD_DEVICE_ID_MAX]  = {0};
    char port_s[8] = {0}, tls_s[8] = {0}, en_s[8] = {0};

    if (!json_get_value(json, "host", host, sizeof(host)))           { send_error(conn_id, "missing host");      return; }
    if (!json_get_value(json, "device_id", did, sizeof(did)))        { send_error(conn_id, "missing device_id"); return; }
    json_get_value(json, "port", port_s, sizeof(port_s));
    json_get_value(json, "tls", tls_s, sizeof(tls_s));
    bool have_en  = json_get_value(json, "enabled", en_s, sizeof(en_s)) != 0;

    bool tls = cloud_truthy(tls_s);
    int  port = atoi(port_s);
    if (port <= 0 || port > 65535) port = tls ? 443 : 80;

    cloud_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.magic   = CLOUD_CONFIG_MAGIC;
    cfg.version = CLOUD_CONFIG_VERSION;
    strncpy(cfg.host, host, sizeof(cfg.host) - 1);
    strncpy(cfg.device_id, did, sizeof(cfg.device_id) - 1);
    cfg.port    = (uint16_t)port;
    cfg.tls     = tls ? 1 : 0;
    cfg.enabled = (!have_en || cloud_truthy(en_s)) ? 1 : 0;   /* default enabled */
    /* TLS ALWAYS verifies the server cert chain + hostname against the embedded ISRG
       roots (cloud_ca.h). The old "verify":false bring-up override has been removed:
       an encrypted-but-unauthenticated link is MITM-able, and leaving the knob in the
       provisioning path was one typo away from shipping an unauthenticated pod. A
       "verify" field in the request is now ignored. */
    cfg.verify  = tls ? 1 : 0;

    if (cloud_config_save(&cfg) != 0) { send_error(conn_id, "config save failed"); return; }
    cloud_client_reload();   /* pick up the new config and (re)connect */

    char resp[256];
    snprintf(resp, sizeof(resp),
             "{\"status\":\"ok\",\"data\":{\"host\":\"%s\",\"port\":%u,\"tls\":%s,\"verify\":%s,"
             "\"enabled\":%s,\"device_id\":\"%s\"}}\n",
             cfg.host, cfg.port, cfg.tls ? "true" : "false", cfg.verify ? "true" : "false",
             cfg.enabled ? "true" : "false", cfg.device_id);
    if (at_send_data(conn_id, (const uint8_t *)resp, strlen(resp)) != 0) {
        at_close_connection(conn_id);
    }
}

static void handle_cloud_status(int conn_id) {
    cloud_config_t cfg;
    bool have = (cloud_config_load(&cfg) == 0);
    char last_error[80];
    cloud_client_last_error(last_error, sizeof(last_error));
    char resp[384];
    snprintf(resp, sizeof(resp),
             "{\"status\":\"ok\",\"data\":{\"state\":\"%s\",\"last_error\":\"%s\",\"configured\":%s,"
             "\"host\":\"%s\",\"port\":%u,\"tls\":%s,\"verify\":%s,\"device_id\":\"%s\"}}\n",
             cloud_client_state_str(), last_error,
             have ? "true" : "false",
             have ? cfg.host : "",
             have ? cfg.port : 0,
             (have && cfg.tls) ? "true" : "false",
             (have && cfg.verify) ? "true" : "false",
             have ? cfg.device_id : "");
    if (at_send_data(conn_id, (const uint8_t *)resp, strlen(resp)) != 0) {
        at_close_connection(conn_id);
    }
}

static void handle_cloud_clear(int conn_id) {
    cloud_config_clear();
    cloud_client_reload();   /* drops to DISABLED now that there's no config */
    send_ok_str(conn_id, "\"cleared\"");
}

/* ---- Wi-Fi provisioning (ESP32-C3 via esp-hosted) ----
   Stores the SSID/passphrase that esp_wifi_ctrl associates with; the wired
   Ethernet stays primary, Wi-Fi is the fallback (see net_server multihoming). */
static void handle_wifi_set(int conn_id, const char *json) {
    config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.magic = CONFIG_MAGIC; cfg.version = CONFIG_VERSION;
    if (!json_get_value(json, "ssid", cfg.ssid, sizeof(cfg.ssid))) {
        send_error(conn_id, "missing ssid"); return;
    }
    json_get_value(json, "password", cfg.password, sizeof(cfg.password));   /* open AP: empty */
    if (config_save(&cfg) != 0) { send_error(conn_id, "config save failed"); return; }
    esp_wifi_ctrl_reload();   /* bring the ESP32 up + (re)connect with the new creds */

    char resp[128];
    snprintf(resp, sizeof(resp), "{\"status\":\"ok\",\"data\":{\"ssid\":\"%s\"}}\n", cfg.ssid);
    if (at_send_data(conn_id, (const uint8_t *)resp, strlen(resp)) != 0) at_close_connection(conn_id);
}

static void handle_wifi_clear(int conn_id) {
    config_clear();
    esp_wifi_ctrl_reload();   /* drops Wi-Fi; ESP32 returns to reset */
    send_ok_str(conn_id, "\"cleared\"");
}

/* {"cmd":"eth","action":"stop"|"start"|"restart"} — manually control the wired
   interface (down/up, PHY reset + DHCP re-acquire) without a power cycle. The
   action is latched and applied on the net task; the reply just confirms intent.
   {"cmd":"eth","action":"stats"} returns the wired-link diagnostics (eth_diag.h).
   {"cmd":"eth","action":"speed","mbit":0|10|100[,"duplex":"half"|"full"]} forces the link
   mode (0 = back to autonegotiation) — a debug aid for a link that errors at 100M.
   {"cmd":"eth","action":"refclk"} measures the PHY's RMII reference clock (nominally
   50 MHz) against the MCU crystal and returns {hz, ppm, on_hsi}.
   {"cmd":"eth","action":"loopback","mbit":10|100[,"n":N]} runs a PHY near-end loopback
   test and returns {mbit, sent, received, intact, corrupt, crc, align, tx_fail}. */
bool clock_on_hsi(void);   /* main.c: true when the MCU crystal did not start */

static void handle_eth(int conn_id, const char *buf) {
    char action[16] = {0};
    json_get_value(buf, "action", action, sizeof(action));
    if (strcmp(action, "stats") == 0) {
        eth_diag_t d;
        char data[512], resp[560];
        net_eth_diag(&d);
        eth_diag_json(&d, data, sizeof(data));
        bp_emit_t e;
        bp_emit_init(&e, resp, sizeof(resp));
        bp_emit_raw(&e, "{\"status\":\"ok\",\"data\":");
        bp_emit_raw(&e, data);
        bp_emit_raw(&e, "}\n");
        if (!bp_emit_ok(&e)) { send_error(conn_id, bp_err_str(BP_ERR_TOO_LARGE)); return; }
        if (at_send_data(conn_id, (const uint8_t *)resp, bp_emit_len(&e)) != 0) at_close_connection(conn_id);
        return;
    }
    if (strcmp(action, "loopback") == 0) {
        char mbuf[8] = {0}, nbuf[8] = {0};
        json_get_value(buf, "mbit", mbuf, sizeof(mbuf));
        int mbit = atoi(mbuf);
        if (mbit != 10 && mbit != 100) { send_error(conn_id, "eth loopback mbit must be 10 or 100"); return; }
        uint32_t n = json_get_value(buf, "n", nbuf, sizeof(nbuf)) && nbuf[0] ? (uint32_t)atoi(nbuf) : 200u;
        if (n == 0) n = 1;
        if (n > 1000) n = 1000;
        uint32_t seq = net_eth_loopback_seq();
        eth_loopback_result_t r;
        net_eth_loopback(mbit, n);
        bool done = false;
        for (int i = 0; i < 150 && !(done = net_eth_loopback_result(seq, &r)); i++) sleep_ms(100);
        if (!done) { send_error(conn_id, "eth loopback: timed out"); return; }
        char resp[192];
        snprintf(resp, sizeof(resp),
                 "{\"mbit\":%d,\"sent\":%lu,\"received\":%lu,\"intact\":%lu,\"corrupt\":%lu,"
                 "\"crc\":%lu,\"align\":%lu,\"tx_fail\":%lu}",
                 r.mbit, (unsigned long)r.sent, (unsigned long)r.received, (unsigned long)r.intact,
                 (unsigned long)r.corrupt, (unsigned long)r.crc, (unsigned long)r.align,
                 (unsigned long)r.tx_fail);
        send_ok_str(conn_id, resp);
        return;
    }
    if (strcmp(action, "refclk") == 0) {
        uint32_t seq = net_eth_refclk_seq(), hz = 0;
        net_eth_refclk_measure();
        for (int i = 0; i < 40 && !net_eth_refclk_result(seq, &hz); i++)
            sleep_ms(100);   /* the net task runs the ~200 ms gate, then restarts the PHY */
        long ppm = hz ? (((long long)hz - 50000000LL) * 1000000LL / 50000000LL) : 0;
        char resp[96];
        snprintf(resp, sizeof(resp), "{\"hz\":%lu,\"ppm\":%ld,\"on_hsi\":%s}",
                 (unsigned long)hz, ppm, clock_on_hsi() ? "true" : "false");
        send_ok_str(conn_id, resp);
        return;
    }
    if (strcmp(action, "speed") == 0) {
        char nbuf[8] = {0}, dbuf[8] = {0};
        if (!json_get_value(buf, "mbit", nbuf, sizeof(nbuf)) || !nbuf[0]) {
            send_error(conn_id, "eth speed needs mbit (0 = autoneg, 10 or 100)"); return;
        }
        int mbit = atoi(nbuf);
        if (mbit != 0 && mbit != 10 && mbit != 100) {
            send_error(conn_id, "eth speed mbit must be 0 (autoneg), 10 or 100"); return;
        }
        json_get_value(buf, "duplex", dbuf, sizeof(dbuf));
        int full = (strcmp(dbuf, "full") == 0);
        net_eth_force_speed(mbit, full);
        char resp[64];
        snprintf(resp, sizeof(resp), "{\"speed\":%d,\"duplex\":\"%s\"}",
                 mbit, mbit == 0 ? "auto" : (full ? "full" : "half"));
        send_ok_str(conn_id, resp);
        return;
    }
    if      (strcmp(action, "stop")    == 0) { net_eth_stop();    }
    else if (strcmp(action, "start")   == 0) { net_eth_start();   }
    else if (strcmp(action, "restart") == 0 || action[0] == '\0') { net_eth_restart(); strcpy(action, "restart"); }
    else { send_error(conn_id, "eth action must be stop|start|restart|stats|speed|refclk|loopback"); return; }
    char resp[48];
    snprintf(resp, sizeof(resp), "\"%s\"", action);
    send_ok_str(conn_id, resp);
}

/* {"cmd":"speedtest","dir":"up"|"down","bytes":N}  — cloud throughput probe over
   a tunnel conn. See the speedtest state block / speedtest_pump above. */
static void handle_speedtest(int conn_id, const char *buf) {
    char dir[8] = {0};
    json_get_value(buf, "dir", dir, sizeof(dir));
    char nbuf[16] = {0};
    long bytes = json_get_value(buf, "bytes", nbuf, sizeof(nbuf)) ? atol(nbuf) : 0;
    if (bytes <= 0)           bytes = 1048576;           /* default 1 MiB */
    if (bytes > 64*1024*1024) bytes = 64*1024*1024;      /* sanity clamp */

    if (strcmp(dir, "down") == 0) {
        /* Sink: the next `bytes` raw bytes on this conn are counted + discarded,
           then we ack. The server starts blasting right after this command line. */
        sprx.total    = (size_t)bytes;
        sprx.recv     = 0;
        sprx.last_ack = 0;
        proto[conn_id] = PROTO_SPEEDTEST;
        return;                                          /* server paces to our acks */
    }
    /* up (default): stream synthetic bytes out, pumped from command_handler_poll. */
    sptx.active = true;
    sptx.conn   = conn_id;
    sptx.total  = (size_t)bytes;
    sptx.sent   = 0;
}

static void handle_wifi_status(int conn_id) {
    config_t cfg;
    bool have = (config_load(&cfg) == 0 && cfg.ssid[0] != '\0');
    /* esp-hosted transport counters since the link came up.  `batch` is the
       histogram of transactions-per-net_poll: index 0 = passes that clocked
       nothing, 1 = a single transaction (what the transport used to be capped
       at), >=2 = batched passes.  See esp_hosted_pump.h. */
    const esp_hosted_pump_stats_t *ps = esp_hosted_spi_stats();
    char batch[64];
    int n = 0;
    for (unsigned i = 0; i <= ESP_HOSTED_PUMP_MAX_BATCH; i++) {
        n += snprintf(batch + n, (size_t)((int)sizeof(batch) - n), "%s%lu",
                      i ? "," : "", (unsigned long)ps->batch_len[i]);
        if (n >= (int)sizeof(batch)) { n = (int)sizeof(batch) - 1; break; }
    }
    batch[n] = '\0';

    char resp[384];
    snprintf(resp, sizeof(resp),
             "{\"status\":\"ok\",\"data\":{\"state\":\"%s\",\"configured\":%s,"
             "\"connected\":%s,\"ssid\":\"%s\","
             "\"xacts\":%lu,\"pump_calls\":%lu,\"batch\":[%s],"
             "\"settle_hit\":%lu,\"settle_miss\":%lu,\"xact_err\":%lu}}\n",
             esp_wifi_ctrl_state_str(),
             have ? "true" : "false",
             esp_wifi_ctrl_connected() ? "true" : "false",
             have ? cfg.ssid : "",
             (unsigned long)ps->xacts, (unsigned long)ps->calls, batch,
             (unsigned long)ps->settle_hit, (unsigned long)ps->settle_miss,
             (unsigned long)ps->xact_err);
    if (at_send_data(conn_id, (const uint8_t *)resp, strlen(resp)) != 0) at_close_connection(conn_id);
}

/* ---- Device identity (Ed25519) ---- */

#define IDENT_NONCE_MAX 128   /* max nonce we will sign, in bytes */

static void handle_identity_public(int conn_id) {
    uint8_t pub[DEVICE_ID_PUBLIC_LEN];
    if (device_identity_get_public(pub) != 0) {
        send_error(conn_id, "identity not available");
        return;
    }
    char b64[B64URL_ENCODED_LEN(DEVICE_ID_PUBLIC_LEN) + 1];
    b64url_encode(pub, sizeof(pub), b64, sizeof(b64));

    char payload[B64URL_ENCODED_LEN(DEVICE_ID_PUBLIC_LEN) + 16];
    snprintf(payload, sizeof(payload), "{\"public\":\"%s\"}", b64);
    send_ok_str(conn_id, payload);
}

static void handle_identity_pop(int conn_id, const char *json) {
    char nonce_b64[B64URL_ENCODED_LEN(IDENT_NONCE_MAX) + 4];
    if (!json_get_value(json, "nonce", nonce_b64, sizeof(nonce_b64))) {
        send_error(conn_id, "missing nonce");
        return;
    }

    uint8_t nonce[IDENT_NONCE_MAX];
    size_t  nonce_len = 0;
    if (b64url_decode(nonce_b64, nonce, sizeof(nonce), &nonce_len) != 0) {
        send_error(conn_id, "invalid nonce");
        return;
    }

    uint8_t sig[DEVICE_ID_SIG_LEN];
    /* Signed under the proof-of-possession context, NOT the WS-auth context, so
       a pop signature obtained over the open command port can't be replayed as
       cloud WS authentication (see device_identity.h). */
    if (device_identity_sign_ctx(DEVICE_ID_CTX_POP, nonce, nonce_len, sig) != 0) {
        send_error(conn_id, "identity not available");
        return;
    }

    char b64[B64URL_ENCODED_LEN(DEVICE_ID_SIG_LEN) + 1];
    b64url_encode(sig, sizeof(sig), b64, sizeof(b64));

    /* {"status":"ok","data":{"signature":"<86 chars>"}}\n exceeds send_ok_str's
       128-byte buffer, so build and send directly like handle_target_status. */
    char resp[B64URL_ENCODED_LEN(DEVICE_ID_SIG_LEN) + 64];
    snprintf(resp, sizeof(resp),
             "{\"status\":\"ok\",\"data\":{\"signature\":\"%s\"}}\n", b64);
    if (at_send_data(conn_id, (const uint8_t *)resp, strlen(resp)) != 0) {
        at_close_connection(conn_id);
    }
}

/* ---- SWD probe ---- */

/* Arm the SWD engine and flip this connection to PROTO_DAP, where its byte
   stream carries length-framed CMSIS-DAP packets executed on-pod by dap.c.
   The pod is its own CMSIS-DAP probe — the per-bit remote_bitbang torrent is
   replaced by batched DAP transfers, the path that makes flashing fast over the
   internet.  The "ok" response is sent FIRST, while the connection is still
   JSON, so the client sees the ack before the byte stream switches meaning. */
static void handle_dap_start(int conn_id, const char *json) {
    if (!require_la_voltage(conn_id)) return;
    char s[8] = {0};
    if (!json_get_value(json, "swclk", s, sizeof(s))) {
        send_error(conn_id, "missing swclk");
        return;
    }
    unsigned swclk = (unsigned)atoi(s);
    if (!json_get_value(json, "swdio", s, sizeof(s))) {
        send_error(conn_id, "missing swdio");
        return;
    }
    unsigned swdio = (unsigned)atoi(s);
    /* "nreset" used to name an LA channel to borrow as the target reset line.
       rev3 has a dedicated /NRST_CONTROL pin (PF4 -> J1 pin 22), so the field is
       gone: it is accepted and ignored for old clients rather than rejected,
       because a stale "nreset":N would otherwise fail a flash that will now work
       fine on the real pin. */
    /* Claim both pins BEFORE touching the FPGA: la_bank.v gives SWD the highest priority, so
       arming over a running UART proxy or sensor used to silently take the line away. */
    uint8_t clk_pin = (uint8_t)swclk, dio_pin = (uint8_t)swdio;
    if (!la_claim_or_error(conn_id, LA_FN_SWD_CLK, &clk_pin, 1, 0)) return;
    if (!la_claim_or_error(conn_id, LA_FN_SWD_DIO, &dio_pin, 1, 0)) return;

    int rc = fpga_swd_arm(swclk, swdio);
    if (rc == -2) { send_error(conn_id, "swd busy"); return; }
    if (rc != 0)  { send_error(conn_id, "invalid swd la channel"); return; }
    la_pins_claim(LA_FN_SWD_CLK, LA_GPIO_NONE, 0, &clk_pin, 1);
    la_pins_claim(LA_FN_SWD_DIO, LA_GPIO_NONE, 0, &dio_pin, 1);

    dap_reset();
    dap_rx_have = 0;

    send_ok_str(conn_id, "\"dap ready\"");
    proto[conn_id] = PROTO_DAP;   /* AFTER the ack: bytes are now framed packets */

    /* A DAP flashing session is a torrent of small request/response packets, so
       turn Nagle OFF for the duration to avoid per-packet batching delay.  The
       client may have already pipelined its first DAP frame right behind
       dap_start; at_set_tcp_nodelay() preserves that pending command across the
       option change so it isn't dropped. Socket-only: the cloud tunnel has no pcb. */
    if (conn_has_socket(conn_id))
        at_set_tcp_nodelay(conn_id, true);
}

/* ---- Emulated I2C sensor ----
 * Mock an I2C sensor on two LA channels, driven by the FPGA's generic target.
 *
 *   {"cmd":"sensor_start","type":"bmp280","addr":"0x76","sda":1,"scl":2}
 *   {"cmd":"sensor_set","temperature_c":25.0,"pressure_pa":101325}
 *   {"cmd":"sensor_stop"}
 *   {"cmd":"sensor_status"}
 *   {"cmd":"sensor_regs","start":"0xF7","len":6}     → register bytes
 *   {"cmd":"sensor_la","samples":1024,"sample_rate_mhz":2.0} → raw bus capture
 */
static void handle_sensor_start(int conn_id, const char *json) {
    if (!require_la_voltage(conn_id)) return;
    char type[16] = {0}, addr_s[8] = {0}, sda_s[8] = {0}, scl_s[8] = {0};

    if (!json_get_value(json, "type", type, sizeof(type))) {
        send_error(conn_id, "missing type");
        return;
    }
    if (!json_get_value(json, "sda", sda_s, sizeof(sda_s)) ||
        !json_get_value(json, "scl", scl_s, sizeof(scl_s))) {
        send_error(conn_id, "missing sda/scl");
        return;
    }
    json_get_value(json, "addr", addr_s, sizeof(addr_s));   /* optional */

    uint8_t  addr7 = addr_s[0] ? (uint8_t)strtol(addr_s, NULL, 0) : 0;
    unsigned sda   = (unsigned)atoi(sda_s);
    unsigned scl   = (unsigned)atoi(scl_s);

    /* A replacing sensor_start may take over the running sensor's own pins, so its two
       functions count as free here — but any OTHER owner is a conflict, checked before
       sensor_sim_start touches the FPGA so a refused start leaves the old sensor running. */
    uint16_t i2c_fns = LA_FN_BIT(LA_FN_I2C_SDA) | LA_FN_BIT(LA_FN_I2C_SCL);
    uint8_t  sda_pin = (uint8_t)sda, scl_pin = (uint8_t)scl;
    if (!la_claim_or_error(conn_id, LA_FN_I2C_SDA, &sda_pin, 1, i2c_fns)) return;
    if (!la_claim_or_error(conn_id, LA_FN_I2C_SCL, &scl_pin, 1, i2c_fns)) return;

    int rc = sensor_sim_start(type, addr7, sda, scl);
    if (rc == -1) { send_error(conn_id, "unknown sensor type"); return; }
    if (rc != 0)  { send_error(conn_id, "sensor start failed (bad channel?)"); return; }
    la_pins_release_fn(LA_FN_I2C_SDA);   /* the replaced sensor's pins, if any */
    la_pins_release_fn(LA_FN_I2C_SCL);
    la_pins_claim(LA_FN_I2C_SDA, LA_GPIO_NONE, 0, &sda_pin, 1);
    la_pins_claim(LA_FN_I2C_SCL, LA_GPIO_NONE, 0, &scl_pin, 1);

    char payload[80];
    snprintf(payload, sizeof(payload),
             "{\"type\":\"%s\",\"addr\":%u,\"sda\":%u,\"scl\":%u}",
             sensor_sim_type(), sensor_sim_addr7(), sda, scl);
    send_ok_str(conn_id, payload);
}

static void handle_sensor_set(int conn_id, const char *json) {
    char t_s[16] = {0}, p_s[16] = {0};
    bool any = false;

    if (!sensor_sim_active()) { send_error(conn_id, "no sensor active"); return; }

    if (json_get_value(json, "temperature_c", t_s, sizeof(t_s))) {
        if (sensor_sim_set("temperature_c", (float)atof(t_s)) != 0) {
            send_error(conn_id, "temperature_c rejected");
            return;
        }
        any = true;
    }
    if (json_get_value(json, "pressure_pa", p_s, sizeof(p_s))) {
        if (sensor_sim_set("pressure_pa", (float)atof(p_s)) != 0) {
            send_error(conn_id, "pressure_pa rejected");
            return;
        }
        any = true;
    }
    if (!any) { send_error(conn_id, "no recognised parameters"); return; }

    char payload[64];
    snprintf(payload, sizeof(payload), "{\"type\":\"%s\"}", sensor_sim_type());
    send_ok_str(conn_id, payload);
}

static void handle_sensor_stop(int conn_id) {
    sensor_sim_stop();
    la_pins_release_fn(LA_FN_I2C_SDA);
    la_pins_release_fn(LA_FN_I2C_SCL);
    send_ok_str(conn_id, "null");
}

static void handle_sensor_status(int conn_id) {
    char resp[256];
    if (!sensor_sim_active()) {
        snprintf(resp, sizeof(resp),
                 "{\"status\":\"ok\",\"data\":{\"active\":false}}\n");
    } else {
        i2c_sensor_status_t st = {0};
        sensor_sim_get_status(&st);
        snprintf(resp, sizeof(resp),
                 "{\"status\":\"ok\",\"data\":{"
                 "\"active\":true,\"type\":\"%s\",\"addr\":%u,"
                 "\"transactions\":%u,\"writes\":%u,"
                 "\"last_reg\":%u,\"last_val\":%u}}\n",
                 sensor_sim_type(), sensor_sim_addr7(),
                 st.xfer_count, st.wr_count, st.last_wr_addr, st.last_wr_val);
    }
    if (at_send_data(conn_id, (const uint8_t *)resp, strlen(resp)) != 0) {
        at_close_connection(conn_id);
    }
}

static void handle_sensor_regs(int conn_id, const char *json) {
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

static void handle_sensor_la(int conn_id, const char *json) {
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
 * "samples" is the LA sample count (2 bytes each); 2*samples must fit the buffer. */
static void handle_la_capture(int conn_id, const char *json) {
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
    if (fpga_la_capture_psram_start(samples, sr_hz) != 0) {
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
}

/* ---- UART transparent proxy ----
 * Bridge a DUT UART through two LA channels. Like dap_start, this is the one
 * command that changes how the connection's bytes are interpreted: after the
 * `"uart ready"` ack the same TCP connection carries raw UART bytes both ways.
 *   {"cmd":"uart_proxy_start","rx":<1-12>,"tx":<1-12>,"baud":115200}
 * Exit: close the socket (primary), or send  <≥1s idle> +++ <≥1s idle>  to
 * return to JSON on the same connection. */
static void handle_uart_proxy_start(int conn_id, const char *json) {
    if (!require_la_voltage(conn_id)) return;
    char s[12] = {0};
    if (!json_get_value(json, "rx", s, sizeof(s))) { send_error(conn_id, "missing rx"); return; }
    unsigned rx = (unsigned)atoi(s);
    if (!json_get_value(json, "tx", s, sizeof(s))) { send_error(conn_id, "missing tx"); return; }
    unsigned tx = (unsigned)atoi(s);
    uint32_t baud = 115200;
    if (json_get_value(json, "baud", s, sizeof(s))) baud = (uint32_t)strtoul(s, NULL, 0);

    /* Claim both channels first: a proxy sharing a line with SWD or an emulated sensor used
       to lose it silently to la_bank.v's priority.  This also rejects RX/TX on an LA7/LA8
       whose 10k pull-DOWN is engaged — the UART idles high. */
    uint8_t rx_pin = (uint8_t)rx, tx_pin = (uint8_t)tx;
    if (!la_claim_or_error(conn_id, LA_FN_UART_RX, &rx_pin, 1, 0)) return;
    if (!la_claim_or_error(conn_id, LA_FN_UART_TX, &tx_pin, 1, 0)) return;

    int rc = fpga_uart_config(rx, tx, baud, true);
    if (rc == -2) { send_error(conn_id, "uart busy"); return; }
    if (rc != 0)  { send_error(conn_id, "invalid uart args"); return; }
    la_pins_claim(LA_FN_UART_RX, LA_GPIO_NONE, 0, &rx_pin, 1);
    la_pins_claim(LA_FN_UART_TX, LA_GPIO_NONE, 0, &tx_pin, 1);

    /* Verify the iCE40 actually answers before committing to raw UART mode: a bogus
       status read (rx_avail out of range) means the FPGA isn't responding over SPI
       (unconfigured / bad gateware), which otherwise surfaces as a silent dead
       terminal flooded with 0xFF.  Refuse with a clear error the UI can show. */
    uint16_t rx_avail = 0;
    if (fpga_uart_status(&rx_avail, NULL) != 0) {
        fpga_uart_disable();
        la_pins_release_fn(LA_FN_UART_RX);
        la_pins_release_fn(LA_FN_UART_TX);
        send_error(conn_id, "uart proxy failed: FPGA not responding (check gateware)");
        return;
    }

    /* Hold the RX line high for the session: a DUT's TX pin floats until its
       firmware configures the UART, which the soft-UART otherwise decodes as a
       flood of 0x00 during boot. Restored in uart_proxy_end.
       LA1-LA6 only: LA7/LA8's resistor pulls DOWN, so engaging "the pull" there held RX at
       the opposite of the idle level and guaranteed the 0x00 flood it was meant to prevent.
       LA9-LA14 have no resistor at all, and at a 1.8 V bank the 3V3-referenced ones are
       off-limits — the proxy still runs, it just cannot hold RX high. */
    uart_rx_pu_ch = -1;
    if (la_pull_dir(rx) == LA_PULL_UP && la_pullups_available()) {
        uart_rx_pu_ch   = (int)rx;
        uart_rx_pu_prev = pca9555_la_pullup_enabled((uint8_t)rx);
        if (pca9555_set_la_pullup((uint8_t)rx, true) != 0) {
            printf("[uart] WARNING: could not engage LA%u's pull-up to hold RX high\n", rx);
            uart_rx_pu_ch = -1;
        }
    }

    send_ok_str(conn_id, "\"uart ready\"");   /* ack while still in JSON mode */
    proto[conn_id]    = PROTO_UART;            /* AFTER the ack: bytes are now raw UART */
    uart_proxy_conn   = conn_id;
    uart_plus_count   = 0;
    uart_plus_pending = false;
    uart_last_rx_at   = get_absolute_time();
    uart_last_plus_at = uart_last_rx_at;
    if (conn_has_socket(conn_id))
        at_set_tcp_nodelay(conn_id, true);     /* low-latency interactive bytes (socket only) */
}

/* ---- Entry point ---- */

/* Dispatch one complete, NUL-terminated JSON command line. */
/* Set (or query) the LA I/O-bank voltage via the TPS2116 mux.
     {"cmd":"la_voltage","mv":1800|3300}  -> switch the bank (required before any
                                             LA op) and report the new state.
     {"cmd":"la_voltage"}                 -> report the current mv + status pin. */
static void handle_la_voltage(int conn_id, const char *json) {
    char mv_s[8] = {0};
    if (json_get_value(json, "mv", mv_s, sizeof(mv_s))) {
        /* Switching the bank under a running UART proxy / SWD session / sensor emulation
           glitches every LA line and strands the 3V3-referenced pulls; refuse and name the
           pins instead.  Re-setting the CURRENT voltage stays a no-op and is allowed. */
        char why[LA_PINS_ERR_MAX];
        if (!la_pins_check_voltage_change(la_vccio_get_mv(), atoi(mv_s), why, sizeof(why))) {
            send_error(conn_id, why);
            return;
        }
        int rc = la_vccio_set_mv(atoi(mv_s));
        if (rc == -2) {
            send_error(conn_id, "1.8 V needs a v3 pod; this board is v2 "
                                "(its TPS2116 has no 1.8 V setting)");
            return;
        }
        if (rc != 0) {
            send_error(conn_id, "la voltage must be 1800 or 3300 (mv)");
            return;
        }
    }
    char payload[80];
    snprintf(payload, sizeof(payload),
             "{\"mv\":%d,\"st\":%d,\"readback_mv\":%d}",
             la_vccio_get_mv(), la_vccio_status_pin(), la_vccio_readback_mv());
    send_ok_str(conn_id, payload);
}

/* Report the USB-C CC-line state (rev3+): which orientation the cable is in and
   how much current the upstream source advertises.
     {"cmd":"usb_cc"}  -> {"supported":true,"cc1_mv":..,"cc2_mv":..,
                           "orientation":"cc1"|"cc2"|"none",
                           "advertised":"none"|"default"|"1.5A"|"3.0A",
                           "advertised_ma":0|500|1500|3000}
   Report-only: nothing in the firmware gates on it. */
static void handle_usb_cc(int conn_id) {
    usb_cc_t cc;
    if (usb_cc_read(&cc) != 0) {
        /* Distinguish "this board has no CC taps" from "the conversion failed",
           so a genuine ADC fault is never reported as a valid 0 mV reading. */
        send_error(conn_id, cc.supported ? "usb cc read failed"
                                         : "usb cc monitoring needs a v3 pod");
        return;
    }
    static const char *const orient[] = { "none", "cc1", "cc2" };
    char payload[176];
    snprintf(payload, sizeof(payload),
             "{\"supported\":%s,\"cc1_mv\":%d,\"cc2_mv\":%d,\"orientation\":\"%s\","
             "\"advertised\":\"%s\",\"advertised_ma\":%d}",
             cc.supported ? "true" : "false", cc.cc1_mv, cc.cc2_mv,
             orient[(cc.orientation >= 0 && cc.orientation <= 2) ? cc.orientation : 0],
             cc.advertised, cc.advertised_ma);
    send_ok_str(conn_id, payload);
}

/* Drive the dedicated target-reset line (rev3+, /NRST_CONTROL on J1 pin 22).
     {"cmd":"nrst","assert":1}      -> hold the target in reset
     {"cmd":"nrst","assert":0}      -> release (Hi-Z; the DUT's pull-up wins)
     {"cmd":"nrst","pulse_ms":50}   -> assert, wait, release
     {"cmd":"nrst"}                 -> report state
   Flashing does NOT need this: dap_start's CMSIS-DAP SWJ_PINS already drives the
   same pin, so an OpenOCD/pyOCD connect-under-reset works with no extra call.
   This is for power-on-reset style test steps that are not flashing. */
static void handle_nrst(int conn_id, const char *json) {
    if (!nrst_ctrl_supported()) {
        send_error(conn_id, "nrst pin needs a v3 pod");
        return;
    }
    char s[12] = {0};
    if (json_get_value(json, "pulse_ms", s, sizeof(s))) {
        int ms = atoi(s);
        if (ms <= 0) { send_error(conn_id, "pulse_ms must be > 0"); return; }
        nrst_ctrl_pulse((uint32_t)ms);
    } else if (json_get_value(json, "assert", s, sizeof(s))) {
        nrst_ctrl_assert(json_flag(json, "assert"));
    }
    char payload[64];
    snprintf(payload, sizeof(payload), "{\"supported\":true,\"asserted\":%s}",
             nrst_ctrl_is_asserted() ? "true" : "false");
    send_ok_str(conn_id, payload);
}

/* Route the buffered DAC through the U55 TMUX muxes (U47/U48). Set a CTRLn by
   including its ``ctrlN_en``; ``ctrlN_sel`` picks the path (ctrl1: 0=3V3,1=5V,
   2=12V,3=12V_ADC; ctrl2: 0=12V_VMID,1=ADC_VMID,2=GND). Omitted CTRLn is left
   as-is; a bare command just reports state. */
static void handle_dac_mux(int conn_id, const char *json) {
    char s[8] = {0};
    if (json_get_value(json, "ctrl1_en", s, sizeof(s))) {
        int sel = json_get_value(json, "ctrl1_sel", s, sizeof(s)) ? atoi(s) : 0;
        if (sel < 0 || sel > 3) { send_error(conn_id, "ctrl1_sel 0..3"); return; }
        if (dacmux_set_ctrl1(json_flag(json, "ctrl1_en"), (uint8_t)sel) != 0) {
            send_error(conn_id, "dac_mux ctrl1 failed"); return;
        }
    }
    if (json_get_value(json, "ctrl2_en", s, sizeof(s))) {
        int sel = json_get_value(json, "ctrl2_sel", s, sizeof(s)) ? atoi(s) : 0;
        if (sel < 0 || sel > 3) { send_error(conn_id, "ctrl2_sel 0..3"); return; }
        if (dacmux_set_ctrl2(json_flag(json, "ctrl2_en"), (uint8_t)sel) != 0) {
            send_error(conn_id, "dac_mux ctrl2 failed"); return;
        }
    }
    uint8_t reg = 0;
    if (dacmux_read(&reg) != 0) { send_error(conn_id, "dac_mux read failed"); return; }
    char payload[112];
    snprintf(payload, sizeof(payload),
        "{\"reg\":%u,\"ctrl1_en\":%u,\"ctrl1_sel\":%u,\"ctrl2_en\":%u,\"ctrl2_sel\":%u}",
        reg, (reg & 0x01u) ? 1u : 0u, (unsigned)((reg >> 1) & 3u),
        (reg & 0x08u) ? 1u : 0u, (unsigned)((reg >> 4) & 3u));
    send_ok_str(conn_id, payload);
}

/* Calibration relay switching via U58 (relays energised by a bit high): cal1
   (5V DAC->ADC), cal2 (12V DAC->ADC, exclusive with cal1), amp_measure
   (ADC<-amps terminal), cal_path (ADC<-cal path). Any field present sets the
   whole state (absent = off); a bare command just reports state. */
static void handle_cal_switch(int conn_id, const char *json) {
    char s[8] = {0};
    bool any = json_get_value(json, "cal1", s, sizeof(s))
             | json_get_value(json, "cal2", s, sizeof(s))
             | json_get_value(json, "amp_measure", s, sizeof(s))
             | json_get_value(json, "cal_path", s, sizeof(s));
    if (any) {
        int rc = calsw_set(json_flag(json, "cal1"), json_flag(json, "cal2"),
                           json_flag(json, "amp_measure"), json_flag(json, "cal_path"));
        if (rc == -2) { send_error(conn_id, "cal1 and cal2 are mutually exclusive"); return; }
        if (rc != 0)  { send_error(conn_id, "cal_switch failed"); return; }
    }
    uint8_t reg = 0;
    if (calsw_read(&reg) != 0) { send_error(conn_id, "cal_switch read failed"); return; }
    char payload[112];
    snprintf(payload, sizeof(payload),
        "{\"reg\":%u,\"cal1\":%u,\"cal2\":%u,\"amp_measure\":%u,\"cal_path\":%u}",
        reg, (reg & 0x01u) ? 1u : 0u, (reg & 0x02u) ? 1u : 0u,
        (reg & 0x04u) ? 1u : 0u, (reg & 0x08u) ? 1u : 0u);
    send_ok_str(conn_id, payload);
}

/* analog_path — apply a named analog path (the SINGLE SOURCE OF TRUTH lives in
   i2c_bus.c::analog_path_set).  {"cmd":"analog_path","path":"cal1"} flips every
   switch the path needs and returns the resulting mux/relay registers.
   Names: off dac_3v3|3v3 dac_5v|5v dac_12v|12v adc_ext|ext|sma cal1 cal2 amp. */
static void handle_analog_path(int conn_id, const char *json) {
    char name[16] = {0};
    if (!json_get_value(json, "path", name, sizeof(name))) { send_error(conn_id, "missing path"); return; }
    analog_path_t p;
    if (analog_path_from_name(name, &p) != 0) { send_error(conn_id, "unknown path"); return; }
    if (analog_path_set(p) != 0) { send_error(conn_id, "analog_path failed"); return; }
    uint8_t u55 = 0, u58 = 0; dacmux_read(&u55); calsw_read(&u58);
    char payload[80];
    snprintf(payload, sizeof(payload), "{\"path\":\"%s\",\"u55\":%u,\"u58\":%u}",
             analog_path_name(p), u55, u58);
    send_ok_str(conn_id, payload);
}

/* dac_out — route a DAC output path AND set a CALIBRATED voltage (one step, no
   manual mux flipping).  {"cmd":"dac_out","path":"5v","volts":2.5} → achieved
   mv + code.  path off just parks the output; omit volts to only route. */
static void handle_dac_out(int conn_id, const char *json) {
    char name[16] = {0}, volts_s[16] = {0};
    if (!json_get_value(json, "path", name, sizeof(name))) { send_error(conn_id, "missing path"); return; }
    analog_path_t p; int idx = -1;
    if (analog_path_from_name(name, &p) != 0) { send_error(conn_id, "unknown path"); return; }
    if      (p == ANALOG_PATH_DAC_3V3) idx = 0;
    else if (p == ANALOG_PATH_DAC_5V)  idx = 1;
    else if (p == ANALOG_PATH_DAC_12V) idx = 2;
    else if (p != ANALOG_PATH_OFF) { send_error(conn_id, "path must be 3v3|5v|12v|off"); return; }
    if (analog_path_set(p) != 0) { send_error(conn_id, "route failed"); return; }
    long code = -1; int got_mv = 0;
    if (idx >= 0 && json_get_value(json, "volts", volts_s, sizeof(volts_s))) {
        float v = (float)atof(volts_s);
        float a = DAC_CAL[idx].a, b = DAC_CAL[idx].b;
        code = lroundf((v - a) / b);
        if (code < 0) code = 0;
        if (code > 255) code = 255;
        if (dac_set_constant((uint8_t)code, 240) != 0) { send_error(conn_id, "dac set failed"); return; }
        got_mv = (int)lroundf((a + b * (float)code) * 1000.0f);
    }
    char payload[80];
    snprintf(payload, sizeof(payload), "{\"path\":\"%s\",\"mv\":%d,\"code\":%ld}",
             analog_path_name(p), got_mv, code);
    send_ok_str(conn_id, payload);
}

/* adc_read — route an ADC source AND return a CALIBRATED reading in mV.
   {"cmd":"adc_read","source":"ext"} → {"source","mv","count","span"}.  A short
   16-sample burst, averaged ON THE 16-BIT CIRCLE and then unwrapped — see
   adc_scale.h.  Averaging the RAW counts first (what this did until 2026-07-29)
   returns a plausible-looking number that is wrong by tens of volts whenever the
   burst straddles the wrap, which is exactly where every source's 0 V point sits.
   `span` is the burst's pk-pk in counts; a burst wider than ADC_BURST_MAX_SPAN has
   no single voltage (the input is still moving — e.g. a DAC left driving by a
   preceding `measure`/`generate`) and is REFUSED rather than averaged.  Quick, so
   it blocks briefly. */
static void handle_adc_read(int conn_id, const char *json) {
    char name[16] = {0};
    analog_path_t p = ANALOG_PATH_ADC_EXT;
    if (json_get_value(json, "source", name, sizeof(name))) {
        if (analog_path_from_name(name, &p) != 0 ||
            (p != ANALOG_PATH_ADC_EXT && p != ANALOG_PATH_CAL1 &&
             p != ANALOG_PATH_CAL2 && p != ANALOG_PATH_AMP)) {
            send_error(conn_id, "source must be ext|cal1|cal2|amp"); return;
        }
    }
    if (!heavy_begin(conn_id)) return;
    if (analog_path_set(p) != 0) { heavy_release(conn_id); send_error(conn_id, "route failed"); return; }
    sleep_ms(20);    /* let the G6K relays (~4ms) + front-end RC settle (yields to FreeRTOS) */
    uint16_t s16[16] = {0};
    int rc = adc_capture_psram(s16, 16, 0.0f);
    heavy_release(conn_id);
    if (rc != 0) { send_error(conn_id, "adc read failed"); return; }
    /* Per-source affine fit.  The UNWRAP is not per-source — it belongs to the
       shared front end and adc_scale_burst always applies it (adc_scale.h). */
    cal_lin_t c = ADC_CAL_CAL1;
    if      (p == ANALOG_PATH_CAL2)    c = ADC_CAL_CAL2;
    else if (p == ANALOG_PATH_ADC_EXT) c = ADC_CAL_EXT;
    adc_reading_t rd = adc_scale_burst(s16, 16, c.a, c.b);
    if (!rd.valid) {
        /* Plausible-looking garbage is worse than an error: a sweep would record it. */
        char msg[112];
        snprintf(msg, sizeof(msg),
                 "adc_read: input not settled on %s (%u counts pk-pk over the 16-sample "
                 "burst, limit %u) — stop the DAC (dac_stop) or let the node settle",
                 analog_path_name(p), (unsigned)rd.span, (unsigned)ADC_BURST_MAX_SPAN);
        send_error(conn_id, msg);
        return;
    }
    char payload[96];
    snprintf(payload, sizeof(payload),
             "{\"source\":\"%s\",\"mv\":%d,\"count\":%d,\"span\":%u}",
             analog_path_name(p), (int)lroundf(rd.volts * 1000.0f),
             (int)lroundf(rd.count), (unsigned)rd.span);
    send_ok_str(conn_id, payload);
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
   called from the same place, ice40_reflash_image(). */
void command_handler_on_gateware_reconfigured(void) {
    s_loop_src  = DAC_LOOP_SRC_ADC;
    s_loop_in   = 0;
    s_loop_step = 0;

    /* The reconfiguration also reset the UART, SWD, I2C-target and stepper engines, so every
       pin they owned is back to plain LA mode.  Tear the sessions down here (the table must
       not claim a function nothing is driving) and let la_pins_on_gateware_reconfigured()
       re-apply the gpio pins' GPIO_SET latches, which the fabric also lost. */
    if (uart_proxy_conn >= 0) uart_proxy_end(uart_proxy_conn, true);
    swd_disarm_and_release();
    if (sensor_sim_active()) sensor_sim_stop();
    la_pins_release_fn(LA_FN_I2C_SDA);
    la_pins_release_fn(LA_FN_I2C_SCL);
    trigwait.active    = false;   /* an armed capture died with the fabric */
    trig_reply.present = false;
    la_pins_on_gateware_reconfigured();
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
static void handle_dac_control_loop(int conn_id, const char *json) {
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

    static char    curve_b64[SIGNAL_BUF_SIZE * 2];   /* base64url of the uploaded curve bytes */
    static uint8_t curve[SIGNAL_BUF_SIZE];
    static uint8_t curve_full[SIGNAL_BUF_SIZE];       /* the full 2048-point gateware LUT       */
    size_t curve_len = 0;
    if (json_get_value(json, "curve", curve_b64, sizeof(curve_b64)) && curve_b64[0]) {
        if (b64url_decode(curve_b64, curve, sizeof(curve), &curve_len) != 0) {
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
                                             curve_full, sizeof(curve_full))
            : dac_loop_upsample_curve(curve, curve_len, curve_full, sizeof(curve_full));
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
       the loop opens by driving whatever the raw count happened to point at. */
    if (rc == 0 && have_map)
        rc = dac_loop_set_inmap(map.in_zero, map.in_gain, map.in_trip, true, map.trip_en);
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
static void handle_dac_loop_input(int conn_id, const char *json) {
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
static void handle_dac_loop_probe(int conn_id, const char *json) {
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

/* {"cmd":"fpga_image","image":0|1} — switch the running iCE40 gateware IMAGE (0 = closed-loop,
   1 = deep-DAC-PSRAM-replay). SB_WARMBOOT can't reconfigure at runtime on this board (its
   config-SPI pins ARE the shared PSRAM bus), so fpga_warmboot() actually REFLASHES the selected
   image and does a CRESET cold-reconfig (~2-3 s). Returns the new image's {version, features};
   also asks the net task to re-announce capabilities so the server's cached caps track the swap. */
static void handle_fpga_image(int conn_id, const char *json) {
    char s[8] = {0};
    if (!json_get_value(json, "image", s, sizeof(s)) || !s[0]) {
        send_error(conn_id, "image required (0=loop, 1=deep-replay)"); return;
    }
    uint8_t img = (uint8_t)strtoul(s, NULL, 10);
    if (!heavy_begin(conn_id)) return;
    int rc = fpga_warmboot(img);
    uint8_t ver = signal_engine_fpga_version();
    uint8_t feats = signal_engine_fpga_features();
    heavy_release(conn_id);
    if (rc == -1) { send_error(conn_id, "image out of range (0=loop, 1=deep-replay)"); return; }
    if (rc == -3) { send_error(conn_id, "image switch: reflash failed (CDONE never rose)"); return; }
    if (rc == -5) { signal_engine_psram_report_wedge(); send_error(conn_id, "image switch: iCE40->PSRAM write inoperable after retries (run psram_recover to reboot+reflash)"); return; }
    if (rc != 0)  { send_error(conn_id, "image switch failed"); return; }
    /* The live image (and thus dac_control_loop / dac_deep_replay / dac_replay_max_samples) just
       changed — re-announce so the cloud/webapp gate on the NEW image, not the connect-time one. */
    cloud_client_request_caps_resend();
    char payload[96];
    snprintf(payload, sizeof(payload), "{\"image\":%u,\"version\":%u,\"features\":%u}", img, ver, feats);
    send_ok_str(conn_id, payload);
}

/* {"cmd":"psram_recover"} — one-click remote recovery for an inoperable PSRAM datapath: ack,
   then reboot.  The reboot clears the STM32-OCTOSPI side (the only thing that does), and the
   boot self-test auto-reflashes the iCE40 side (psram_boot_selftest_with_recovery), so the
   pod returns healthy WITHOUT a physical power-cycle.  The client will not receive a
   command.response (the pod reboots first) — treat "no reply, device reconnects" as success
   and re-poll status.psram_ok. */
static void handle_psram_recover(int conn_id) {
    send_ok_str(conn_id, "{\"recover\":\"rebooting\"}");
    s_recover_at = make_timeout_time_ms(400);   /* let the ack flush, then reset in the poll loop */
    s_recover_pending = true;
}

/* {"cmd":"psram_ping"[,"count":N]} → {"pass":bool,"count":N,"first_bad":i}
   Shared-bus write solidity check: the iCE40 streams a known +0x0101 ramp through
   the REAL capture datapath (writer -> shared quad bus -> PSRAM) and the STM32
   reads it back.  PASS => the iCE40 writes PSRAM and the STM32 sees it identically;
   FAIL => the iCE40->PSRAM write path is broken (NOT the analog ADC).  Cheap,
   deterministic, self-contained — used by hwe2e TestHW_V2_PsramPing. */
static void handle_psram_ping(int conn_id, const char *json) {
    char v[12] = {0};
    int n = 16;
    if (json_get_value(json, "count", v, sizeof(v))) n = atoi(v);
    if (n < 4)   n = 4;
    if (n > 256) n = 256;
    if (!heavy_begin(conn_id)) return;
    int bad = -1;
    int rc = signal_engine_capture_selftest(n, &bad);
    heavy_release(conn_id);
    char payload[64];
    snprintf(payload, sizeof(payload), "{\"pass\":%s,\"count\":%d,\"first_bad\":%d}",
             rc == 0 ? "true" : "false", n, bad);
    send_ok_str(conn_id, payload);
}

/* High-frequency polled reads (the web UI refreshes these on a timer) would flood
   the serial console with identical lines, so they're not traced.  Every other
   command still logs, so a real interaction is still visible. */
static bool cmd_is_noisy_poll(const char *cmd) {
    return strcmp(cmd, "ping")          == 0 ||
           strcmp(cmd, "power_status")  == 0 ||
           strcmp(cmd, "target_status") == 0;
}

/* Commands that need neither the iCE40 nor the PSRAM, so they still run in a safe mode that
   turned those off (boot_guard.h).  Everything else is refused with a clear reason instead of
   driving an uninitialised SPI1/XSPI. */
static bool cmd_ok_without_hw(const char *cmd) {
    static const char *const ok[] = {
        "ping", "status", "cloud_set", "cloud_status", "cloud_clear",
        "wifi_set", "wifi_status", "wifi_clear", "eth", "speedtest",
        "la_voltage", "usb_cc", "nrst", "target_power", "target_status", "power_status",
        "power_profile", "identity_public", "identity_pop",
        "can_config", "can_write", "can_read", "can_status", "can_term", "can_respond",
        "can_disable",
    };
    for (size_t i = 0; i < sizeof(ok) / sizeof(ok[0]); i++)
        if (strcmp(cmd, ok[i]) == 0) return true;
    return false;
}

static void dispatch_line(int conn_id, const char *buf) {
    char cmd[32] = {0};
    if (!json_get_value(buf, "cmd", cmd, sizeof(cmd))) {
        send_error(conn_id, "missing cmd");
        return;
    }

    if (!cmd_is_noisy_poll(cmd))
        printf("[cmd] <- \"%s\" (id=%d)\n", cmd, conn_id);

    if (boot_guard_skip_hw() && !cmd_ok_without_hw(cmd)) {
        send_error(conn_id, "safe mode: iCE40/PSRAM are off. Unplug and replug the pod");
        return;
    }

    if      (strcmp(cmd, "ping")      == 0) handle_ping(conn_id);
    else if (strcmp(cmd, "generate")  == 0) handle_generate(conn_id, buf);
    else if (strcmp(cmd, "capture")   == 0) handle_capture(conn_id, buf);
    else if (strcmp(cmd, "capture_dual") == 0) handle_capture_dual(conn_id, buf);
    else if (strcmp(cmd, "capture_read") == 0) handle_capture_read(conn_id, buf);
    else if (strcmp(cmd, "stream")    == 0) handle_stream(conn_id, buf);
    else if (strcmp(cmd, "measure")   == 0) handle_measure(conn_id, buf);
    else if (strcmp(cmd, "load")      == 0) handle_load(conn_id, buf);
    else if (strcmp(cmd, "load_bin")  == 0) handle_load_bin(conn_id, buf);
    else if (strcmp(cmd, "replay")    == 0) handle_replay(conn_id, buf);
    else if (strcmp(cmd, "dac_stop")  == 0) handle_dac_stop(conn_id);
    else if (strcmp(cmd, "dac_set")   == 0) handle_dac_set(conn_id, buf);
    else if (strcmp(cmd, "dac_mux")   == 0) handle_dac_mux(conn_id, buf);
    else if (strcmp(cmd, "cal_switch") == 0) handle_cal_switch(conn_id, buf);
    else if (strcmp(cmd, "analog_path") == 0) handle_analog_path(conn_id, buf);
    else if (strcmp(cmd, "psram_ping") == 0) handle_psram_ping(conn_id, buf);
    else if (strcmp(cmd, "dac_out")   == 0) handle_dac_out(conn_id, buf);
    else if (strcmp(cmd, "adc_read")  == 0) handle_adc_read(conn_id, buf);
    else if (strcmp(cmd, "dac_control_loop") == 0) handle_dac_control_loop(conn_id, buf);
    else if (strcmp(cmd, "dac_loop_probe")   == 0) handle_dac_loop_probe(conn_id, buf);
    else if (strcmp(cmd, "dac_loop_input")   == 0) handle_dac_loop_input(conn_id, buf);
    else if (strcmp(cmd, "fpga_image")       == 0) handle_fpga_image(conn_id, buf);
    else if (strcmp(cmd, "psram_recover")    == 0) handle_psram_recover(conn_id);
    else if (strcmp(cmd, "test")      == 0) handle_test(conn_id, buf);
    else if (strcmp(cmd, "status")    == 0) handle_status(conn_id);
    else if (strcmp(cmd, "cloud_set")     == 0) handle_cloud_set(conn_id, buf);
    else if (strcmp(cmd, "cloud_status")  == 0) handle_cloud_status(conn_id);
    else if (strcmp(cmd, "cloud_clear")   == 0) handle_cloud_clear(conn_id);
    else if (strcmp(cmd, "wifi_set")      == 0) handle_wifi_set(conn_id, buf);
    else if (strcmp(cmd, "wifi_status")   == 0) handle_wifi_status(conn_id);
    else if (strcmp(cmd, "wifi_clear")    == 0) handle_wifi_clear(conn_id);
    else if (strcmp(cmd, "eth")           == 0) handle_eth(conn_id, buf);
    else if (strcmp(cmd, "speedtest")     == 0) handle_speedtest(conn_id, buf);
    else if (strcmp(cmd, "la")        == 0) handle_la(conn_id, buf);
    else if (strcmp(cmd, "la_pins")       == 0) handle_la_pins(conn_id);
    else if (strcmp(cmd, "gpio")          == 0) handle_gpio(conn_id, buf);
    else if (strcmp(cmd, "la_voltage")    == 0) handle_la_voltage(conn_id, buf);
    else if (strcmp(cmd, "usb_cc")        == 0) handle_usb_cc(conn_id);
    else if (strcmp(cmd, "nrst")          == 0) handle_nrst(conn_id, buf);
    else if (strcmp(cmd, "target_power")  == 0) handle_target_power(conn_id, buf);
    else if (strcmp(cmd, "target_status") == 0) handle_target_status(conn_id);
    else if (strcmp(cmd, "power_status")  == 0) handle_power_status(conn_id);
    else if (strcmp(cmd, "power_profile") == 0) handle_power_profile(conn_id, buf);
    else if (strcmp(cmd, "dap_start")     == 0) handle_dap_start(conn_id, buf);
    else if (strcmp(cmd, "identity_public") == 0) handle_identity_public(conn_id);
    else if (strcmp(cmd, "identity_pop")    == 0) handle_identity_pop(conn_id, buf);
    else if (strcmp(cmd, "sensor_start")  == 0) handle_sensor_start(conn_id, buf);
    else if (strcmp(cmd, "sensor_set")    == 0) handle_sensor_set(conn_id, buf);
    else if (strcmp(cmd, "sensor_stop")   == 0) handle_sensor_stop(conn_id);
    else if (strcmp(cmd, "sensor_status") == 0) handle_sensor_status(conn_id);
    else if (strcmp(cmd, "sensor_regs")   == 0) handle_sensor_regs(conn_id, buf);
    else if (strcmp(cmd, "sensor_la")     == 0) handle_sensor_la(conn_id, buf);
    else if (strcmp(cmd, "la_capture")    == 0) handle_la_capture(conn_id, buf);
    else if (strcmp(cmd, "uart_proxy_start") == 0) handle_uart_proxy_start(conn_id, buf);
    else if (strcmp(cmd, "can_config")   == 0) handle_can_config(conn_id, buf);
    else if (strcmp(cmd, "can_write")    == 0) handle_can_write(conn_id, buf);
    else if (strcmp(cmd, "can_read")     == 0) handle_can_read(conn_id, buf);
    else if (strcmp(cmd, "can_status")   == 0) handle_can_status(conn_id);
    else if (strcmp(cmd, "can_term")     == 0) handle_can_term(conn_id, buf);
    else if (strcmp(cmd, "can_respond")  == 0) handle_can_respond(conn_id, buf);
    else if (strcmp(cmd, "can_disable")  == 0) handle_can_disable(conn_id);
    else if (strcmp(cmd, "ota_begin")    == 0) handle_ota_begin(conn_id, buf);
    else if (strcmp(cmd, "ota_data")     == 0) handle_ota_data(conn_id, buf);
    else if (strcmp(cmd, "ota_end")      == 0) handle_ota_end(conn_id);
    else if (strcmp(cmd, "ota_status")   == 0) handle_ota_status(conn_id);
    else if (strcmp(cmd, "ota_abort")    == 0) handle_ota_abort(conn_id);
    else if (strcmp(cmd, "ota_selftest") == 0) handle_ota_selftest(conn_id);
    else if (strcmp(cmd, "ota_commit")   == 0) handle_ota_commit(conn_id);
    else send_error(conn_id, "unknown cmd");
}

void command_handler_dispatch_console(const char *json_line) {
    /* The console "json" mode already hands us one complete, NUL-terminated JSON
       object, so dispatch it directly (no per-connection byte reassembly).
       Replies route to stdout via at_send_data(CH_CONSOLE_CONN, ...). */
    dispatch_line(CH_CONSOLE_CONN, json_line);
}

/* Commands that stream chunks or switch the connection into a raw protocol
   (SWD/UART) cannot be carried over the single-reply cloud channel. */
static bool cloud_cmd_is_streaming(const char *cmd) {
    return strcmp(cmd, "capture")  == 0 || strcmp(cmd, "stream") == 0 ||
           strcmp(cmd, "capture_dual") == 0 || strcmp(cmd, "capture_read") == 0 ||
           strcmp(cmd, "measure")  == 0 || strcmp(cmd, "test")   == 0 ||
           strcmp(cmd, "load")     == 0 || strcmp(cmd, "replay") == 0 ||
           strcmp(cmd, "load_bin") == 0 ||
           strcmp(cmd, "sensor_regs") == 0 || strcmp(cmd, "sensor_la") == 0 ||
           strcmp(cmd, "dap_start") == 0 || strcmp(cmd, "uart_proxy_start") == 0 ||
           strcmp(cmd, "la_capture") == 0 || strcmp(cmd, "speedtest") == 0;
}

size_t command_handler_dispatch_cloud(const char *command_json, char *out, size_t out_cap) {
    char cmd[32] = {0};
    if (!json_get_value(command_json, "cmd", cmd, sizeof(cmd))) {
        int n = snprintf(out, out_cap, "{\"status\":\"error\",\"message\":\"missing cmd\"}");
        return (n > 0 && (size_t)n < out_cap) ? (size_t)n : 0;
    }
    if (cloud_cmd_is_streaming(cmd)) {
        int n = snprintf(out, out_cap,
                         "{\"status\":\"error\",\"message\":\"command not supported over cloud channel\"}");
        return (n > 0 && (size_t)n < out_cap) ? (size_t)n : 0;
    }

    /* Dispatch through the normal handlers; the reply is captured (see
       command_handler_cloud_capture_append) rather than sent to the TCP server. */
    cloud_cap_active = true;
    cloud_cap_len    = 0;
    cloud_cap_buf[0] = '\0';
    dispatch_line(CH_CLOUD_CONN, command_json);
    cloud_cap_active = false;

    while (cloud_cap_len > 0 &&
           (cloud_cap_buf[cloud_cap_len - 1] == '\n' ||
            cloud_cap_buf[cloud_cap_len - 1] == '\r')) {
        cloud_cap_buf[--cloud_cap_len] = '\0';
    }
    if (cloud_cap_len == 0 || cloud_cap_len >= out_cap) {
        int n = snprintf(out, out_cap, "{\"status\":\"error\",\"message\":\"no reply\"}");
        return (n > 0 && (size_t)n < out_cap) ? (size_t)n : 0;
    }
    memcpy(out, cloud_cap_buf, cloud_cap_len);
    out[cloud_cap_len] = '\0';
    return cloud_cap_len;
}

void command_handler_process(int conn_id, const uint8_t *json_buf, size_t len) {
    /* conn_id outside the buffered set (console/cloud-command pseudo-conns or an unexpected id) —
       best-effort one-shot parse without per-connection buffering. The cloud tunnel IS buffered so
       its raw SWD/UART byte stream survives across segments like a real TCP conn. */
    if (!conn_runs_state_machine(conn_id)) {
        char buf[LINE_ASM_SIZE];
        size_t copy = len < sizeof(buf) - 1 ? len : sizeof(buf) - 1;
        memcpy(buf, json_buf, copy);
        buf[copy] = '\0';
        dispatch_line(conn_id, buf);
        return;
    }

    /* Accumulate into the per-connection line buffer and dispatch each
       newline-terminated command.  Handles commands split across TCP
       segments and multiple commands concatenated in one segment. */
    line_asm_t *la = &line_asm[conn_id];
    size_t i = 0;
    while (i < len) {
        /* Cloud speed-test sink: count + DISCARD `total` raw bytes (no buffer),
           then ack with the done marker. Same byte-counted ingest as PROTO_LOAD
           but throws the data away — it only measures server->device throughput. */
        if (proto[conn_id] == PROTO_SPEEDTEST) {
            size_t need = sprx.total - sprx.recv;
            size_t avail = len - i;
            size_t take  = need < avail ? need : avail;
            sprx.recv += take;
            i += take;
            if (sprx.recv >= sprx.total) {
                proto[conn_id] = PROTO_JSON;
                speedtest_ack_done(conn_id, sprx.recv);
            } else if (sprx.recv - sprx.last_ack >= SPEEDTEST_ACK_INTERVAL) {
                sprx.last_ack = sprx.recv;
                speedtest_ack_progress(conn_id, sprx.recv);   /* window feedback to the server */
            }
            continue;
        }

        /* Raw binary waveform upload (load_bin): consume bytes straight into the
           target buffer (adc_buf16 on v2) until `total` received, then ack and
           return to JSON.  Byte-counted (raw bytes may include newlines). */
        if (proto[conn_id] == PROTO_LOAD) {
            size_t need  = load_bin_total - load_bin_have;
            size_t avail = len - i;
            size_t take  = need < avail ? need : avail;
            if (take > 0) {
                if (load_bin_psram) {
                    /* Deep replay: write the chunk straight into PSRAM at the running
                       byte offset from the TOP-ANCHORED base (fixed at load_bin arm from
                       `total`; the bus is held for the whole upload).  psram_write
                       tCEM-chunks internally, so an arbitrary `take` is fine. */
                    psram_write(signal_engine_dac_psram_base() + (uint32_t)load_bin_have,
                                (const uint8_t *)(json_buf + i), (uint32_t)take);
                } else if (load_bin_dst) {
                    memcpy(load_bin_dst + load_bin_have, json_buf + i, take);
                }
                load_bin_have += take;
                i += take;
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
                proto[conn_id]  = PROTO_JSON;   /* heavy stays claimed for `replay` */
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
            continue;
        }

        /* CMSIS-DAP mode: reassemble [len_lo][len_hi][packet] frames, run each
           through dap_process() on-pod, and write the framed response back.  A
           zero-length frame (or an over-long declared length) leaves DAP mode. */
        if (proto[conn_id] == PROTO_DAP) {
            bool leave = false;
            for (;;) {
                /* fill the 2-byte length header */
                while (dap_rx_have < 2 && i < len)
                    dap_rx[dap_rx_have++] = json_buf[i++];
                if (dap_rx_have < 2) break;           /* need more header bytes */

                size_t need = 2 + ((size_t)dap_rx[0] | ((size_t)dap_rx[1] << 8));
                if (need > sizeof(dap_rx)) {          /* bad length: leave DAP */
                    dap_rx_have = 0; leave = true; break;
                }
                if (dap_rx_have < need) {             /* fill the payload */
                    size_t take = need - dap_rx_have;
                    size_t avail = len - i;
                    if (take > avail) take = avail;
                    memcpy(dap_rx + dap_rx_have, json_buf + i, take);
                    dap_rx_have += take;
                    i += take;
                }
                if (dap_rx_have < need) break;        /* frame split across segments */

                size_t payload = need - 2;
                dap_rx_have = 0;
                if (payload == 0) { leave = true; break; }   /* zero-length: leave */

                size_t rlen = dap_process(dap_rx + 2, payload,
                                          dap_resp + 2, DAP_PACKET_SIZE);
                dap_resp[0] = (uint8_t)(rlen & 0xFF);
                dap_resp[1] = (uint8_t)((rlen >> 8) & 0xFF);
                at_send_data(conn_id, dap_resp, rlen + 2);
            }
            if (leave) {
                swd_disarm_and_release();
                proto[conn_id] = PROTO_UNKNOWN;   /* re-detect the next byte */
                /* Back to JSON/SCPI: restore default Nagle batching.
                   Socket-only: the cloud tunnel has no pcb. */
                if (conn_has_socket(conn_id))
                    at_set_tcp_nodelay(conn_id, false);
            }
            continue;   /* loop: process any trailing JSON, or exit at end */
        }

        /* UART transparent proxy: forward the whole segment to the DUT TX, and
           scan for the guard-timed "+++" escape.  Timing is per-segment (TCP
           NODELAY makes interactive keystrokes their own segments), which is
           enough for the Hayes guard.  The "+++" is also forwarded to the DUT
           (harmless); the trailing-guard check that actually exits lives in
           command_handler_poll(). */
        if (proto[conn_id] == PROTO_UART) {
            absolute_time_t now = get_absolute_time();
            bool first = true;
            fpga_uart_write(json_buf + i, len - i);   /* transparent passthrough */
            for (; i < len; i++) {
                uint8_t b = json_buf[i];
                int64_t idle_us = first
                    ? absolute_time_diff_us(uart_last_rx_at, now) : 0;
                if (b == '+') {
                    if (uart_plus_count == 0) {
                        if (idle_us >= (int64_t)UART_ESC_GUARD_MS * 1000)
                            uart_plus_count = 1;       /* guard before 1st '+' */
                    } else if (uart_plus_count < 3) {
                        uart_plus_count++;
                    }
                    uart_last_plus_at = now;
                    if (uart_plus_count >= 3) uart_plus_pending = true;
                } else {
                    uart_plus_count = 0;               /* any other byte aborts */
                    uart_plus_pending = false;
                }
                first = false;
            }
            uart_last_rx_at = now;
            continue;
        }

        char c = (char)json_buf[i++];
        if (c == '\r') continue;          /* tolerate CRLF */
        if (c == '\n') {
            if (la->discarding) {         /* end of an over-long line */
                la->discarding = false;
                la->len = 0;
            } else if (la->len > 0) {
                la->buf[la->len] = '\0';
                if (proto[conn_id] == PROTO_SCPI) scpi_dispatch_line(conn_id, la->buf);
                else                              dispatch_line(conn_id, la->buf);
                la->len = 0;
            }
            continue;
        }
        if (la->discarding) continue;
        /* Decide the protocol on the first non-whitespace byte of the
           connection.  Leading whitespace before that byte is skipped. */
        if (proto[conn_id] == PROTO_UNKNOWN) {
            if (c == ' ' || c == '\t') continue;
            proto[conn_id] = (c == '{') ? PROTO_JSON : PROTO_SCPI;
        }
        if (la->len < LINE_ASM_SIZE - 1) {
            la->buf[la->len++] = c;
        } else {
            /* No newline within LINE_ASM_SIZE — drop the line, then swallow the
               rest until its terminating newline.  Notify JSON clients; SCPI
               lines this long are not expected, so just discard. */
            la->len = 0;
            la->discarding = true;
            if (proto[conn_id] != PROTO_SCPI) send_error(conn_id, "command too long");
        }
    }
}

void command_handler_conn_closed(int conn_id) {
    /* Abort an in-flight stream owned by this connection.  Keep the gate
       claimed until the trailing async DMA reports back (command_handler_poll
       releases it once stream_active is observed false), so a freshly-claimed
       capture can't race the dying stream's DMA over adc_cmd_buf. */
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
    /* Abort an in-flight speed-test upload owned by this conn (the sink direction
       clears via the proto reset below). */
    speedtest_cancel(conn_id);

    /* A power profile keeps running (pod state), but its reply stream belongs to this conn. */
    power_profile_conn_closed(conn_id);

    /* Heavy ops complete within a single dispatch, so if this conn owns the gate,
       just free it. */
    heavy_release(conn_id);

    /* Drop any partial command buffered for this connection and reset its
       protocol so the next client on this slot is re-detected.  If it was an
       SWD session, release the wire to a safe state. Covers the cloud tunnel too
       (it runs the same state machine). */
    if (conn_runs_state_machine(conn_id)) {
        if (proto[conn_id] == PROTO_DAP) {
            swd_disarm_and_release();
            dap_rx_have = 0;
        }
        /* A raw load_bin upload owned by this conn is abandoned — clear its state
           so a stale total/have/dst can't corrupt the next upload (the heavy gate
           is freed by the heavy_release above). */
        if (proto[conn_id] == PROTO_LOAD || load_bin_conn == conn_id)
            load_bin_clear();
        /* A UART proxy auto-stops on socket close (the primary exit path). */
        if (proto[conn_id] == PROTO_UART || uart_proxy_conn == conn_id)
            uart_proxy_end(conn_id, false);
        line_asm[conn_id].len = 0;
        line_asm[conn_id].discarding = false;
        proto[conn_id] = PROTO_UNKNOWN;
    }

    scpi_conn_closed(conn_id);
}

/* Reset one cloud-tunnel pseudo-connection between tunnels (tunnel.open / tunnel.close). Reuses the
   per-connection teardown so a torn-down tunnel that was mid-SWD/UART leaves the wire safe and the
   next tunnel starts in PROTO_UNKNOWN. */
void command_handler_tunnel_reset(int conn_id) {
    if (!is_tunnel_conn(conn_id)) return;
    command_handler_conn_closed(conn_id);
}
