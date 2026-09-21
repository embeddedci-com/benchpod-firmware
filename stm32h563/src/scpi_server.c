#include "scpi_server.h"
#include "command_handler.h"
#include "at_driver.h"
#include "signal_engine.h"
#include "gpio_control.h"
#include "target_power.h"
#include "wifi_manager.h"
#include "device_identity.h"
#include "b64url.h"

#include "scpi/scpi.h"

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

/* SCPI (IEEE-488.2 / SCPI-99) interface, implemented on libscpi
   (j123b567/scpi-parser).  Shares the single ESP-AT TCP server with the JSON
   protocol; command_handler routes a connection here once its first byte is
   not '{'.  All actions reuse the same engine APIs as the JSON handler, and
   bulk sample data is returned as ASCII CSV. */

#define SCPI_FW_VERSION "0.2.0"   /* keep in sync with command_handler FIRMWARE_VERSION */

#define SCPI_INPUT_BUFFER_LENGTH 256
#define SCPI_ERROR_QUEUE_SIZE    16
#define RESP_BUF_SIZE            1024

/* ---- Instrument state (device-wide; one DAC/ADC) ---- */

typedef struct {
    char     shape;        /* 's' sine, 'q' square, 'r' ramp/sawtooth */
    float    freq;
    uint8_t  amplitude;
    uint8_t  offset;
    float    srate_hz;     /* 0 = auto-pick */
    uint32_t duration_ms;  /* 0 = continuous */
} src_cfg_t;

static src_cfg_t src;
static bool      output_on;
static size_t    sense_points;
static float     sense_srate_hz;   /* ADC capture clock for READ? (0 = max) */

/* SCPI's own capture/replay buffer.  Shares the heavy-op gate with the JSON
   handler (the single ADC) but keeps a separate buffer so the two never alias.
   READ? fills it; TRACe:DATA can overwrite it with a host-uploaded trace; the
   USER source shape replays it out the DAC. */
static uint8_t  scpi_adc_buf[SIGNAL_BUF_SIZE];
static uint16_t scpi_adc16[SIGNAL_BUF_SIZE];   /* v2 16-bit capture buffer */
static size_t  scpi_replay_len;    /* valid samples in scpi_adc_buf for replay */

static void scpi_reset_state(void) {
    src.shape       = 's';
    src.freq        = 1000.0f;
    src.amplitude   = 127;
    src.offset      = 128;
    src.srate_hz    = 0.0f;
    src.duration_ms = 0;
    output_on       = false;
    sense_points    = 256;
    sense_srate_hz  = 0.0f;
    scpi_replay_len = 0;
}

/* 'u' (USER) replays the captured/uploaded buffer; it has no parametric
   generator, so the engine-name fallback is "sine" (only MEASure? consults
   this, and MEASure? with a USER shape is not a meaningful combination). */
static const char *shape_engine_name(char s) {
    return s == 'q' ? "square" : s == 'r' ? "sawtooth" : "sine";
}
static const char *shape_scpi_name(char s) {
    return s == 'q' ? "SQU" : s == 'r' ? "RAMP" : s == 'u' ? "USER" : "SIN";
}

/* ---- libscpi context + buffered response path ---- */

static scpi_t       scpi_ctx;
static char         scpi_input_buf[SCPI_INPUT_BUFFER_LENGTH];
static scpi_error_t scpi_err_queue[SCPI_ERROR_QUEUE_SIZE];

/* Single-threaded main loop + device-wide state → one shared response buffer.
   conn_id for the in-flight line is carried in scpi_ctx.user_context. */
static char   resp_buf[RESP_BUF_SIZE];
static size_t resp_len;

static int cur_conn(scpi_t *ctx) { return (int)(intptr_t)ctx->user_context; }

/* libscpi calls write in fragments; batch into resp_buf and emit on flush (or
   when full) so a query is one AT+CIPSEND instead of many. */
static size_t cb_write(scpi_t *ctx, const char *data, size_t len) {
    int conn = cur_conn(ctx);
    size_t off = 0;
    while (off < len) {
        if (resp_len == RESP_BUF_SIZE) {
            if (at_send_data(conn, (const uint8_t *)resp_buf, resp_len) != 0) {
                at_close_connection(conn);
                resp_len = 0;
                return off;   /* connection died — drop the remainder */
            }
            resp_len = 0;
        }
        size_t space = RESP_BUF_SIZE - resp_len;
        size_t n = (len - off) < space ? (len - off) : space;
        memcpy(resp_buf + resp_len, data + off, n);
        resp_len += n;
        off += n;
    }
    return len;
}

static scpi_result_t cb_flush(scpi_t *ctx) {
    int conn = cur_conn(ctx);
    if (resp_len > 0) {
        if (at_send_data(conn, (const uint8_t *)resp_buf, resp_len) != 0)
            at_close_connection(conn);
        resp_len = 0;
    }
    return SCPI_RES_OK;
}

static int cb_error(scpi_t *ctx, int_fast16_t err) {
    (void)ctx;
    printf("[scpi] error %d: %s\n", (int)err, SCPI_ErrorTranslate((int16_t)err));
    return 0;
}

static scpi_result_t cb_control(scpi_t *ctx, scpi_ctrl_name_t ctrl, scpi_reg_val_t val) {
    (void)ctx; (void)ctrl; (void)val;
    return SCPI_RES_OK;
}

/* *RST handler (interface reset) — stop output, restore defaults. */
static scpi_result_t cb_reset(scpi_t *ctx) {
    (void)ctx;
    dac_stop();
    scpi_reset_state();
    return SCPI_RES_OK;
}

/* ---- SYSTem custom queries ---- */

static scpi_result_t scpi_system_pingQ(scpi_t *ctx) {
    SCPI_ResultMnemonic(ctx, "PONG");
    return SCPI_RES_OK;
}

static scpi_result_t scpi_wifi_stateQ(scpi_t *ctx) {
    const char *s;
    switch (wifi_get_state()) {
        case WIFI_DISCONNECTED: s = "DISCONNECTED"; break;
        case WIFI_CONNECTING:   s = "CONNECTING";   break;
        case WIFI_CONNECTED:    s = "CONNECTED";    break;
        case WIFI_READY:        s = "READY";        break;
        default:                s = "UNKNOWN";      break;
    }
    SCPI_ResultMnemonic(ctx, s);
    return SCPI_RES_OK;
}

static scpi_result_t scpi_wifi_rssiQ(scpi_t *ctx) {
    int rssi = 0;
    if (wifi_get_rssi(&rssi) != 0) {
        SCPI_ErrorPush(ctx, SCPI_ERROR_EXECUTION_ERROR);
        return SCPI_RES_ERR;
    }
    SCPI_ResultInt32(ctx, rssi);
    return SCPI_RES_OK;
}

static scpi_result_t scpi_lan_ipQ(scpi_t *ctx) {
    SCPI_ResultMnemonic(ctx, wifi_get_ip());
    return SCPI_RES_OK;
}

/* ---- Device identity (Ed25519) ----
   PUBlic? returns the base64url public key.  POP? <nonce> signs the supplied
   base64url nonce and returns the base64url signature (proof of possession). */

#define IDENT_NONCE_MAX 128   /* max nonce length we will sign, in bytes */

static scpi_result_t scpi_identity_publicQ(scpi_t *ctx) {
    uint8_t pub[DEVICE_ID_PUBLIC_LEN];
    if (device_identity_get_public(pub) != 0) {
        SCPI_ErrorPush(ctx, SCPI_ERROR_SYSTEM_ERROR);
        return SCPI_RES_ERR;
    }
    char b64[B64URL_ENCODED_LEN(DEVICE_ID_PUBLIC_LEN) + 1];
    b64url_encode(pub, sizeof(pub), b64, sizeof(b64));
    SCPI_ResultMnemonic(ctx, b64);
    return SCPI_RES_OK;
}

static scpi_result_t scpi_identity_popQ(scpi_t *ctx) {
    char   nonce_b64[B64URL_ENCODED_LEN(IDENT_NONCE_MAX) + 4];
    size_t b64_len = 0;
    if (!SCPI_ParamCopyText(ctx, nonce_b64, sizeof(nonce_b64), &b64_len, TRUE)) {
        return SCPI_RES_ERR;   /* libscpi already queued the parameter error */
    }

    uint8_t nonce[IDENT_NONCE_MAX];
    size_t  nonce_len = 0;
    if (b64url_decode(nonce_b64, nonce, sizeof(nonce), &nonce_len) != 0) {
        SCPI_ErrorPush(ctx, SCPI_ERROR_ILLEGAL_PARAMETER_VALUE);
        return SCPI_RES_ERR;
    }

    uint8_t sig[DEVICE_ID_SIG_LEN];
    /* Proof-of-possession context — a signature obtained here can't be replayed
       as cloud WS auth (which uses DEVICE_ID_CTX_WS_AUTH). See device_identity.h. */
    if (device_identity_sign_ctx(DEVICE_ID_CTX_POP, nonce, nonce_len, sig) != 0) {
        SCPI_ErrorPush(ctx, SCPI_ERROR_SYSTEM_ERROR);
        return SCPI_RES_ERR;
    }

    char b64[B64URL_ENCODED_LEN(DEVICE_ID_SIG_LEN) + 1];
    b64url_encode(sig, sizeof(sig), b64, sizeof(b64));
    SCPI_ResultMnemonic(ctx, b64);
    return SCPI_RES_OK;
}

/* ---- SOURce config ---- */

static const scpi_choice_def_t shape_choices[] = {
    {"SINusoid", 's'},
    {"SQUare",   'q'},
    {"RAMP",     'r'},
    {"USER",     'u'},   /* replay the captured/uploaded trace buffer */
    SCPI_CHOICE_LIST_END,
};

static scpi_result_t scpi_src_function(scpi_t *ctx) {
    int32_t tag;
    if (!SCPI_ParamChoice(ctx, shape_choices, &tag, TRUE)) return SCPI_RES_ERR;
    src.shape = (char)tag;
    return SCPI_RES_OK;
}
static scpi_result_t scpi_src_functionQ(scpi_t *ctx) {
    SCPI_ResultMnemonic(ctx, shape_scpi_name(src.shape));
    return SCPI_RES_OK;
}

static scpi_result_t scpi_src_freq(scpi_t *ctx) {
    scpi_number_t num;
    if (!SCPI_ParamNumber(ctx, scpi_special_numbers_def, &num, TRUE)) return SCPI_RES_ERR;
    if (num.special) {
        switch (num.content.tag) {
            case SCPI_NUM_MIN: src.freq = 1.0f;       break;
            case SCPI_NUM_MAX: src.freq = 1.0e6f;     break;
            case SCPI_NUM_DEF: src.freq = 1000.0f;    break;
            default:
                SCPI_ErrorPush(ctx, SCPI_ERROR_ILLEGAL_PARAMETER_VALUE);
                return SCPI_RES_ERR;
        }
    } else {
        if (num.content.value <= 0.0) {
            SCPI_ErrorPush(ctx, SCPI_ERROR_DATA_OUT_OF_RANGE);
            return SCPI_RES_ERR;
        }
        src.freq = (float)num.content.value;   /* unit prefix (k/M) already applied */
    }
    return SCPI_RES_OK;
}
static scpi_result_t scpi_src_freqQ(scpi_t *ctx) {
    SCPI_ResultDouble(ctx, (double)src.freq);
    return SCPI_RES_OK;
}

static scpi_result_t scpi_src_ampl(scpi_t *ctx) {
    uint32_t v;
    if (!SCPI_ParamUInt32(ctx, &v, TRUE)) return SCPI_RES_ERR;
    if (v > 255) { SCPI_ErrorPush(ctx, SCPI_ERROR_DATA_OUT_OF_RANGE); return SCPI_RES_ERR; }
    src.amplitude = (uint8_t)v;
    return SCPI_RES_OK;
}
static scpi_result_t scpi_src_amplQ(scpi_t *ctx) {
    SCPI_ResultUInt32(ctx, src.amplitude);
    return SCPI_RES_OK;
}

static scpi_result_t scpi_src_offset(scpi_t *ctx) {
    uint32_t v;
    if (!SCPI_ParamUInt32(ctx, &v, TRUE)) return SCPI_RES_ERR;
    if (v > 255) { SCPI_ErrorPush(ctx, SCPI_ERROR_DATA_OUT_OF_RANGE); return SCPI_RES_ERR; }
    src.offset = (uint8_t)v;
    return SCPI_RES_OK;
}
static scpi_result_t scpi_src_offsetQ(scpi_t *ctx) {
    SCPI_ResultUInt32(ctx, src.offset);
    return SCPI_RES_OK;
}

static scpi_result_t scpi_src_srate(scpi_t *ctx) {
    double mhz;
    if (!SCPI_ParamDouble(ctx, &mhz, TRUE)) return SCPI_RES_ERR;
    if (mhz < 0.0) { SCPI_ErrorPush(ctx, SCPI_ERROR_DATA_OUT_OF_RANGE); return SCPI_RES_ERR; }
    src.srate_hz = mhz > 0.0 ? (float)(mhz * 1.0e6) : 0.0f;   /* 0 = auto */
    return SCPI_RES_OK;
}
static scpi_result_t scpi_src_srateQ(scpi_t *ctx) {
    SCPI_ResultDouble(ctx, (double)(src.srate_hz / 1.0e6f));
    return SCPI_RES_OK;
}

static scpi_result_t scpi_src_duration(scpi_t *ctx) {
    uint32_t v;
    if (!SCPI_ParamUInt32(ctx, &v, TRUE)) return SCPI_RES_ERR;
    src.duration_ms = v;
    return SCPI_RES_OK;
}
static scpi_result_t scpi_src_durationQ(scpi_t *ctx) {
    SCPI_ResultUInt32(ctx, src.duration_ms);
    return SCPI_RES_OK;
}

/* ---- OUTPut[:STATe] — DAC waveform enable ---- */

static int output_start(void) {
    int rc;
    switch (src.shape) {
        case 'q': rc = dac_generate_square(src.freq, src.amplitude, src.offset,
                                           src.duration_ms, src.srate_hz); break;
        case 'r': rc = dac_generate_sawtooth(src.freq, src.amplitude, src.offset,
                                             src.duration_ms, src.srate_hz); break;
        case 'u': /* USER: replay the captured/uploaded buffer at SOURce:SRATe */
                  if (scpi_replay_len == 0) return -1;
                  rc = dac_generate_arbitrary_rate(scpi_adc_buf, scpi_replay_len,
                                                   true, src.srate_hz); break;
        default:  rc = dac_generate_sine(src.freq, src.amplitude, src.offset,
                                         src.duration_ms, src.srate_hz); break;
    }
    if (rc == 0) output_on = true;
    return rc;
}

static scpi_result_t scpi_output(scpi_t *ctx) {
    scpi_bool_t on;
    if (!SCPI_ParamBool(ctx, &on, TRUE)) return SCPI_RES_ERR;
    if (on) {
        if (output_start() != 0) {
            SCPI_ErrorPush(ctx, SCPI_ERROR_EXECUTION_ERROR);
            return SCPI_RES_ERR;
        }
    } else {
        dac_stop();
        output_on = false;
    }
    return SCPI_RES_OK;
}
static scpi_result_t scpi_outputQ(scpi_t *ctx) {
    SCPI_ResultBool(ctx, output_on);
    return SCPI_RES_OK;
}

/* ---- SENSe:SWEep:POINts ---- */

static scpi_result_t scpi_points(scpi_t *ctx) {
    uint32_t v;
    if (!SCPI_ParamUInt32(ctx, &v, TRUE)) return SCPI_RES_ERR;
    if (v == 0 || v > SIGNAL_BUF_SIZE) {
        SCPI_ErrorPush(ctx, SCPI_ERROR_DATA_OUT_OF_RANGE);
        return SCPI_RES_ERR;
    }
    sense_points = (size_t)v;
    return SCPI_RES_OK;
}
static scpi_result_t scpi_pointsQ(scpi_t *ctx) {
    SCPI_ResultUInt32(ctx, (uint32_t)sense_points);
    return SCPI_RES_OK;
}

/* SENSe:SRATe — ADC capture sample clock in MHz (0 = max 12 MSPS).  A lower
   rate stretches the READ? capture window so a slow waveform fits the buffer
   (e.g. 0.08 → 80 kS/s → 4096 samples ≈ 51 ms). */
static scpi_result_t scpi_sense_srate(scpi_t *ctx) {
    double mhz;
    if (!SCPI_ParamDouble(ctx, &mhz, TRUE)) return SCPI_RES_ERR;
    if (mhz < 0.0) { SCPI_ErrorPush(ctx, SCPI_ERROR_DATA_OUT_OF_RANGE); return SCPI_RES_ERR; }
    sense_srate_hz = mhz > 0.0 ? (float)(mhz * 1.0e6) : 0.0f;
    return SCPI_RES_OK;
}
static scpi_result_t scpi_sense_srateQ(scpi_t *ctx) {
    SCPI_ResultDouble(ctx, (double)(sense_srate_hz / 1.0e6f));
    return SCPI_RES_OK;
}

/* ---- READ? / MEASure? / DIAGnostic:PATTern? — CSV sample data ---- */

static scpi_result_t scpi_readQ(scpi_t *ctx) {
    int conn = cur_conn(ctx);
    size_t n = sense_points;
    uint32_t v;
    if (SCPI_ParamUInt32(ctx, &v, FALSE)) {            /* optional inline count */
        if (v == 0 || v > SIGNAL_BUF_SIZE) {
            SCPI_ErrorPush(ctx, SCPI_ERROR_DATA_OUT_OF_RANGE);
            return SCPI_RES_ERR;
        }
        n = (size_t)v;
    }
    if (!command_handler_acquire_adc(conn)) {
        SCPI_ErrorPush(ctx, SCPI_ERROR_EXECUTION_ERROR);   /* ADC busy */
        return SCPI_RES_ERR;
    }
    if (adc_capture_psram(scpi_adc16, n, sense_srate_hz) != 0) {
        command_handler_release_adc(conn);
        SCPI_ErrorPush(ctx, SCPI_ERROR_EXECUTION_ERROR);
        return SCPI_RES_ERR;
    }
    command_handler_release_adc(conn);
    /* Leave the capture in the buffer so SOURce:FUNCtion USER + OUTPut ON can
       replay this exact trace out the DAC. */
    scpi_replay_len = n;
    SCPI_ResultArrayUInt16(ctx, scpi_adc16, n, SCPI_FORMAT_ASCII);
    return SCPI_RES_OK;
}

static scpi_result_t scpi_measureQ(scpi_t *ctx) {
    int conn = cur_conn(ctx);
    size_t n = sense_points;
    if (!command_handler_acquire_adc(conn)) {
        SCPI_ErrorPush(ctx, SCPI_ERROR_EXECUTION_ERROR);
        return SCPI_RES_ERR;
    }
    /* Phase-locked DAC waveform + 16-bit ADC->PSRAM (measure_psram stops the DAC
       itself).  Blocking is fine here — SCPI is synchronous — and bounded by the
       real capture time. */
    int rc = measure_psram(scpi_adc16, shape_engine_name(src.shape), src.freq,
                           src.amplitude, src.offset, n, src.srate_hz);
    output_on = false;
    command_handler_release_adc(conn);
    if (rc != 0) {
        SCPI_ErrorPush(ctx, SCPI_ERROR_EXECUTION_ERROR);   /* measure failed */
        return SCPI_RES_ERR;
    }
    scpi_replay_len = n;
    SCPI_ResultArrayUInt16(ctx, scpi_adc16, n, SCPI_FORMAT_ASCII);
    return SCPI_RES_OK;
}

static const scpi_choice_def_t pattern_choices[] = {
    {"SINusoid", 0},
    {"COUNter",  1},
    {"RAMP",     2},
    {"CONStant", 3},
    SCPI_CHOICE_LIST_END,
};

static scpi_result_t scpi_diag_patternQ(scpi_t *ctx) {
    int conn = cur_conn(ctx);
    int32_t pat;
    if (!SCPI_ParamChoice(ctx, pattern_choices, &pat, TRUE)) return SCPI_RES_ERR;

    uint32_t value = 255;   /* used by CONStant */
    SCPI_ParamUInt32(ctx, &value, FALSE);
    size_t n = 256;
    uint32_t pts;
    if (SCPI_ParamUInt32(ctx, &pts, FALSE)) {
        if (pts == 0 || pts > SIGNAL_BUF_SIZE) {
            SCPI_ErrorPush(ctx, SCPI_ERROR_DATA_OUT_OF_RANGE);
            return SCPI_RES_ERR;
        }
        n = (size_t)pts;
    }

    if (!command_handler_acquire_adc(conn)) {
        SCPI_ErrorPush(ctx, SCPI_ERROR_EXECUTION_ERROR);
        return SCPI_RES_ERR;
    }
    switch (pat) {
        case 3: /* CONStant */
            memset(scpi_adc_buf, (uint8_t)value, n);
            break;
        case 1: /* COUNter */
            for (size_t i = 0; i < n; i++) scpi_adc_buf[i] = (uint8_t)(i & 0xFF);
            break;
        case 2: /* RAMP */
            for (size_t i = 0; i < n; i++)
                scpi_adc_buf[i] = (uint8_t)((i * 255u) / (n > 1 ? n - 1 : 1));
            break;
        default: /* SINusoid */
            for (size_t i = 0; i < n; i++) {
                float angle = 2.0f * 3.14159265f * (float)i / (float)n;
                int32_t v = (int32_t)(sinf(angle) * 100.0f) + 128;
                if (v < 0)   v = 0;
                if (v > 255) v = 255;
                scpi_adc_buf[i] = (uint8_t)v;
            }
            break;
    }
    command_handler_release_adc(conn);
    SCPI_ResultArrayUInt8(ctx, scpi_adc_buf, n, SCPI_FORMAT_ASCII);
    return SCPI_RES_OK;
}

/* ---- DIAGnostic:CAPture? [<adc_n>][,<la_n>] ----
   v2: simultaneously capture adc_n ADC + la_n raw 12-ch LA samples off ONE trigger
   and return them as CSV: the adc_n ADC samples first, then the la_n LA words (low
   12 bits = LA1..LA12).  Protocol decode is done off-device.  Defaults 16 + 16. */
#define SCPI_CAP_MAX 128u
static uint32_t scpi_cap[SCPI_CAP_MAX * 2];
static scpi_result_t scpi_diag_captureQ(scpi_t *ctx) {
    int conn = cur_conn(ctx);
    uint32_t an = 16, ln = 16, v;
    if (SCPI_ParamUInt32(ctx, &v, FALSE)) an = v;   /* optional inline counts */
    if (SCPI_ParamUInt32(ctx, &v, FALSE)) ln = v;
    if (an > SCPI_CAP_MAX || ln > SCPI_CAP_MAX || (an == 0 && ln == 0)) {
        SCPI_ErrorPush(ctx, SCPI_ERROR_DATA_OUT_OF_RANGE);
        return SCPI_RES_ERR;
    }
    if (!command_handler_acquire_adc(conn)) {
        SCPI_ErrorPush(ctx, SCPI_ERROR_EXECUTION_ERROR);   /* ADC busy */
        return SCPI_RES_ERR;
    }
    static uint16_t adcb[SCPI_CAP_MAX], lab[SCPI_CAP_MAX];
    int rc = fpga_dual_capture(an ? adcb : NULL, (uint16_t)an, 240,
                               ln ? lab : NULL, (uint16_t)ln, 24);
    command_handler_release_adc(conn);
    if (rc != 0) {
        SCPI_ErrorPush(ctx, SCPI_ERROR_EXECUTION_ERROR);
        return SCPI_RES_ERR;
    }
    size_t n = 0;
    for (uint32_t i = 0; i < an; i++) scpi_cap[n++] = adcb[i];
    for (uint32_t i = 0; i < ln; i++) scpi_cap[n++] = lab[i] & 0xFFFu;
    SCPI_ResultArrayUInt32(ctx, scpi_cap, n, SCPI_FORMAT_ASCII);
    return SCPI_RES_OK;
}

/* ---- TRACe — upload a waveform for replay ----
   TRACe:DATA <offset>,"<base64url>" uploads one chunk of raw sample bytes into
   the replay buffer.  A full trace won't fit one 256-byte line, so the host
   sends offset 0 first then successive offsets (≤150 bytes/chunk).  Replay the
   result with SOURce:FUNCtion USER; OUTPut ON.  TRACe:POINts? reports the
   loaded length. */
static scpi_result_t scpi_trace_data(scpi_t *ctx) {
    uint32_t offset;
    if (!SCPI_ParamUInt32(ctx, &offset, TRUE)) return SCPI_RES_ERR;
    if (offset > SIGNAL_BUF_SIZE) {
        SCPI_ErrorPush(ctx, SCPI_ERROR_DATA_OUT_OF_RANGE);
        return SCPI_RES_ERR;
    }

    char   data_b64[B64URL_ENCODED_LEN(SIGNAL_BUF_SIZE) + 4];
    size_t b64_len = 0;
    if (!SCPI_ParamCopyText(ctx, data_b64, sizeof(data_b64), &b64_len, TRUE)) {
        return SCPI_RES_ERR;   /* libscpi already queued the parameter error */
    }

    size_t dec_len = 0;
    if (b64url_decode(data_b64, scpi_adc_buf + offset,
                      SIGNAL_BUF_SIZE - offset, &dec_len) != 0) {
        SCPI_ErrorPush(ctx, SCPI_ERROR_ILLEGAL_PARAMETER_VALUE);
        return SCPI_RES_ERR;
    }
    if (offset == 0) scpi_replay_len = dec_len;
    else             scpi_replay_len = offset + dec_len;
    return SCPI_RES_OK;
}

static scpi_result_t scpi_trace_pointsQ(scpi_t *ctx) {
    SCPI_ResultUInt32(ctx, (uint32_t)scpi_replay_len);
    return SCPI_RES_OK;
}

/* ---- DIGital (GPIO) ---- */

static scpi_result_t scpi_dig_output(scpi_t *ctx) {
    uint32_t la, state;
    if (!SCPI_ParamUInt32(ctx, &la, TRUE))    return SCPI_RES_ERR;
    if (!SCPI_ParamUInt32(ctx, &state, TRUE)) return SCPI_RES_ERR;
    /* The pin becomes a gpio output (JSON `gpio`), so it shows up in la_pins, is released with
       gpio mode off, and is refused while another function owns it. */
    int rc = command_handler_dig_output((unsigned)la, state != 0 ? 1 : 0);
    if (rc == -1) {
        SCPI_ErrorPush(ctx, SCPI_ERROR_DATA_OUT_OF_RANGE);
        return SCPI_RES_ERR;
    }
    if (rc != 0) {
        SCPI_ErrorPush(ctx, SCPI_ERROR_SETTINGS_CONFLICT);
        return SCPI_RES_ERR;
    }
    return SCPI_RES_OK;
}

static scpi_result_t scpi_dig_step(scpi_t *ctx) {
    uint32_t la, steps, delay_us;
    if (!SCPI_ParamUInt32(ctx, &la, TRUE))       return SCPI_RES_ERR;
    if (!SCPI_ParamUInt32(ctx, &steps, TRUE))    return SCPI_RES_ERR;
    if (!SCPI_ParamUInt32(ctx, &delay_us, TRUE)) return SCPI_RES_ERR;
    if (steps == 0 || delay_us == 0) {
        SCPI_ErrorPush(ctx, SCPI_ERROR_DATA_OUT_OF_RANGE);
        return SCPI_RES_ERR;
    }
    /* Optional direction channel: <dir_la>[,<dir>] driven before stepping. */
    uint32_t dir_la = 0, dir = 0;
    if (SCPI_ParamUInt32(ctx, &dir_la, FALSE)) {
        SCPI_ParamUInt32(ctx, &dir, FALSE);
        if (dir_la == 0) {
            SCPI_ErrorPush(ctx, SCPI_ERROR_DATA_OUT_OF_RANGE);
            return SCPI_RES_ERR;
        }
    }
    /* Same ownership rules as the JSON `la` step: free pins are claimed for the train. */
    int rc = command_handler_dig_step((unsigned)la, steps, delay_us, (unsigned)dir_la, dir != 0);
    if (rc == -2) {
        SCPI_ErrorPush(ctx, SCPI_ERROR_SETTINGS_CONFLICT);
        return SCPI_RES_ERR;
    }
    if (rc == -3) {
        /* A step train is already running — device is busy, not a data error. */
        SCPI_ErrorPush(ctx, SCPI_ERROR_DEVICE_ERROR);
        return SCPI_RES_ERR;
    }
    if (rc != 0) {
        SCPI_ErrorPush(ctx, SCPI_ERROR_DATA_OUT_OF_RANGE);
        return SCPI_RES_ERR;
    }
    /* Non-blocking: returns immediately once the FPGA step train is started. */
    return SCPI_RES_OK;
}

/* DIGital:STEP:BUSY? — 1 while a step train is running, 0 once it completes. */
static scpi_result_t scpi_dig_step_busyQ(scpi_t *ctx) {
    SCPI_ResultBool(ctx, command_handler_step_busy());
    return SCPI_RES_OK;
}

/* ---- OUTPut:POWer<n> — target eFuse power ---- */

static int efuse_from_cmd(scpi_t *ctx) {
    int32_t nums[1] = {1};
    SCPI_CommandNumbers(ctx, nums, 1, 1);
    return (int)nums[0];
}

static scpi_result_t scpi_pow_state(scpi_t *ctx) {
    int efuse = efuse_from_cmd(ctx);
    scpi_bool_t on;
    if (!SCPI_ParamBool(ctx, &on, TRUE)) return SCPI_RES_ERR;
    if (target_power_enable(efuse, on) != 0) {
        SCPI_ErrorPush(ctx, SCPI_ERROR_DATA_OUT_OF_RANGE);
        return SCPI_RES_ERR;
    }
    return SCPI_RES_OK;
}
static scpi_result_t scpi_pow_stateQ(scpi_t *ctx) {
    target_power_status_t s;
    if (target_power_get_status(efuse_from_cmd(ctx), &s) != 0) {
        SCPI_ErrorPush(ctx, SCPI_ERROR_DATA_OUT_OF_RANGE);
        return SCPI_RES_ERR;
    }
    SCPI_ResultBool(ctx, s.enabled);
    return SCPI_RES_OK;
}
static scpi_result_t scpi_pow_faultQ(scpi_t *ctx) {
    target_power_status_t s;
    if (target_power_get_status(efuse_from_cmd(ctx), &s) != 0) {
        SCPI_ErrorPush(ctx, SCPI_ERROR_DATA_OUT_OF_RANGE);
        return SCPI_RES_ERR;
    }
    SCPI_ResultBool(ctx, s.fault);
    return SCPI_RES_OK;
}
static scpi_result_t scpi_pow_validQ(scpi_t *ctx) {
    target_power_status_t s;
    if (target_power_get_status(efuse_from_cmd(ctx), &s) != 0) {
        SCPI_ErrorPush(ctx, SCPI_ERROR_DATA_OUT_OF_RANGE);
        return SCPI_RES_ERR;
    }
    SCPI_ResultBool(ctx, s.valid);
    return SCPI_RES_OK;
}

/* ---- Command table ---- */

static const scpi_command_t scpi_commands[] = {
    /* IEEE 488.2 mandated (handled by libscpi core) */
    {"*CLS",  SCPI_CoreCls},
    {"*ESE",  SCPI_CoreEse},
    {"*ESE?", SCPI_CoreEseQ},
    {"*ESR?", SCPI_CoreEsrQ},
    {"*IDN?", SCPI_CoreIdnQ},
    {"*OPC",  SCPI_CoreOpc},
    {"*OPC?", SCPI_CoreOpcQ},
    {"*RST",  SCPI_CoreRst},
    {"*SRE",  SCPI_CoreSre},
    {"*SRE?", SCPI_CoreSreQ},
    {"*STB?", SCPI_CoreStbQ},
    {"*TST?", SCPI_CoreTstQ},
    {"*WAI",  SCPI_CoreWai},

    /* Required SCPI system nodes (libscpi core) */
    {"SYSTem:ERRor[:NEXT]?", SCPI_SystemErrorNextQ},
    {"SYSTem:ERRor:COUNt?",  SCPI_SystemErrorCountQ},
    {"SYSTem:VERSion?",      SCPI_SystemVersionQ},

    /* Custom system queries */
    {"SYSTem:PING?",                     scpi_system_pingQ},
    {"SYSTem:WIFI:STATe?",               scpi_wifi_stateQ},
    {"SYSTem:WIFI:RSSI?",                scpi_wifi_rssiQ},
    {"SYSTem:COMMunicate:LAN:IPADdress?", scpi_lan_ipQ},

    /* Device identity (Ed25519) */
    {"SYSTem:IDENtity:PUBlic?", scpi_identity_publicQ},
    {"SYSTem:IDENtity:POP?",    scpi_identity_popQ},

    /* Source (waveform generation config) */
    {"[SOURce]:FUNCtion[:SHAPe]",   scpi_src_function},
    {"[SOURce]:FUNCtion[:SHAPe]?",  scpi_src_functionQ},
    {"[SOURce]:FREQuency",          scpi_src_freq},
    {"[SOURce]:FREQuency?",         scpi_src_freqQ},
    {"[SOURce]:VOLTage[:AMPLitude]",  scpi_src_ampl},
    {"[SOURce]:VOLTage[:AMPLitude]?", scpi_src_amplQ},
    {"[SOURce]:VOLTage:OFFSet",     scpi_src_offset},
    {"[SOURce]:VOLTage:OFFSet?",    scpi_src_offsetQ},
    {"[SOURce]:SRATe",              scpi_src_srate},
    {"[SOURce]:SRATe?",             scpi_src_srateQ},
    {"[SOURce]:DURation",           scpi_src_duration},
    {"[SOURce]:DURation?",          scpi_src_durationQ},

    /* Output enable (waveform) */
    {"OUTPut[:STATe]",  scpi_output},
    {"OUTPut[:STATe]?", scpi_outputQ},

    /* Sense (acquisition length + capture rate) */
    {"SENSe:SWEep:POINts",  scpi_points},
    {"SENSe:SWEep:POINts?", scpi_pointsQ},
    {"SENSe:SRATe",         scpi_sense_srate},
    {"SENSe:SRATe?",        scpi_sense_srateQ},

    /* Acquisition */
    {"READ?",                scpi_readQ},
    {"MEASure?",             scpi_measureQ},
    {"DIAGnostic:PATTern?",  scpi_diag_patternQ},
    {"DIAGnostic:CAPture?", scpi_diag_captureQ},

    /* Trace upload for replay (play back via SOURce:FUNCtion USER; OUTPut ON) */
    {"TRACe[:DATA]",   scpi_trace_data},
    {"TRACe:POINts?",  scpi_trace_pointsQ},

    /* Digital GPIO */
    {"DIGital:OUTPut", scpi_dig_output},
    {"DIGital:STEP",   scpi_dig_step},
    {"DIGital:STEP:BUSY?", scpi_dig_step_busyQ},

    /* Target eFuse power */
    {"OUTPut:POWer#[:STATe]",  scpi_pow_state},
    {"OUTPut:POWer#[:STATe]?", scpi_pow_stateQ},
    {"OUTPut:POWer#:FAULt?",   scpi_pow_faultQ},
    {"OUTPut:POWer#:VALid?",   scpi_pow_validQ},

    SCPI_CMD_LIST_END,
};

static scpi_interface_t scpi_if = {
    .error   = cb_error,
    .write   = cb_write,
    .control = cb_control,
    .flush   = cb_flush,
    .reset   = cb_reset,
};

/* ---- Public entry points ---- */

static bool inited = false;

void scpi_dispatch_line(int conn_id, const char *line) {
    if (!inited) {
        scpi_reset_state();
        SCPI_Init(&scpi_ctx, scpi_commands, &scpi_if, scpi_units_def,
                  "EmbeddedCI", "BenchPod", "0", SCPI_FW_VERSION,
                  scpi_input_buf, SCPI_INPUT_BUFFER_LENGTH,
                  scpi_err_queue, SCPI_ERROR_QUEUE_SIZE);
        inited = true;
    }

    /* Route this line's response to the right TCP connection. */
    scpi_ctx.user_context = (void *)(intptr_t)conn_id;
    resp_len = 0;

    /* command_handler already assembled one complete line (no terminator);
       feed it plus a terminator so libscpi parses and dispatches it now. */
    SCPI_Input(&scpi_ctx, line, (int)strlen(line));
    SCPI_Input(&scpi_ctx, "\r\n", 2);
}

void scpi_conn_closed(int conn_id) {
    (void)conn_id;   /* SCPI state is device-wide; captures are synchronous. */
}
