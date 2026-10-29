/*
 * command_handler_analog.c — the analog front end: muxes, relays, calibrated DAC/ADC and 4-20 mA.
 *
 *   dac_mux, cal_switch, analog_path   the U55 muxes and U58 relays (i2c_bus.c)
 *   dac_out, current_out               a calibrated DAC voltage / loop current
 *   adc_read, calibrate                a calibrated ADC reading / this pod's own calibration
 *
 * The digital-only board refuses these before they get here (cmd_gate, CMD_F_ANALOG). Runs on the
 * hw worker task, like the rest of command_handler.
 */
#include "command_handler.h"
#include "command_handler_internal.h"
#include "bp_json.h"
#include "signal_engine.h"  /* dac_set_constant*, adc_capture_psram */
#include "i2c_bus.h"        /* dacmux_*, calsw_*, analog_path_* */
#include "cal_data.h"
#include "current_out.h"   /* 4-20 mA output (J9): uA <-> DAC code */
#include "adc_scale.h"     /* circular-mean + unwrap for adc_read's sample burst */
#include "adc_cal.h"       /* per-pod ADC calibration on top of cal_data.h */
#include "pico_compat.h"   /* sleep_ms (yields to FreeRTOS) */
#include "cloud_client.h"  /* cloud_client_request_caps_resend: the server scales captures with the fit */
#include "dac_limits.h"    /* dac_limits_path_index: "3v3"/"5v"/"12v" */
#include "dac_quad.h"      /* analog v3r2: a DAC8554 channel per output */
#include "at_driver.h"     /* at_send_data: calibrate sends its own (large) reply */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* JSON parsing is shared (see bp_json.h); same aliases as command_handler.c. */
#define json_get_value      bp_json_get
#define json_flag           bp_json_flag

/* Route the buffered DAC through the U55 TMUX muxes (U47/U48). Set a CTRLn by
   including its ``ctrlN_en``; ``ctrlN_sel`` picks the path (ctrl1: 0=3V3,1=5V,
   2=12V,3=12V_ADC; ctrl2: 0=12V_VMID,1=ADC_VMID,2=GND). Omitted CTRLn is left
   as-is; a bare command just reports state. */
void handle_dac_mux(int conn_id, const char *json) {
    char s[8] = {0};
    if (dac_quad_present()) {
        send_error(conn_id, "dac_mux: this board (analog v3r2) has no DAC mux, one DAC channel per output; use analog_path");
        return;
    }
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
   (5V DAC->ADC), cal2 (12V DAC->ADC, exclusive with cal1), current_in
   (ADC<-amps terminal), cal_path (ADC<-cal path). Any field present sets the
   whole state (absent = off); a bare command just reports state. */
void handle_cal_switch(int conn_id, const char *json) {
    char s[8] = {0};
    bool any = json_get_value(json, "cal1", s, sizeof(s))
             | json_get_value(json, "cal2", s, sizeof(s))
             | json_get_value(json, "current_in", s, sizeof(s))
             | json_get_value(json, "cal_path", s, sizeof(s));
    if (any) {
        int rc = calsw_set(json_flag(json, "cal1"), json_flag(json, "cal2"),
                           json_flag(json, "current_in"), json_flag(json, "cal_path"));
        if (rc == -2) { send_error(conn_id, "cal1 and cal2 are mutually exclusive"); return; }
        if (rc != 0)  { send_error(conn_id, "cal_switch failed"); return; }
    }
    uint8_t reg = 0;
    if (calsw_read(&reg) != 0) { send_error(conn_id, "cal_switch read failed"); return; }
    char payload[112];
    snprintf(payload, sizeof(payload),
        "{\"reg\":%u,\"cal1\":%u,\"cal2\":%u,\"current_in\":%u,\"cal_path\":%u,\"low_range\":%u}",
        reg, (reg & 0x01u) ? 1u : 0u, (reg & 0x02u) ? 1u : 0u,
        (reg & 0x04u) ? 1u : 0u, (reg & 0x08u) ? 1u : 0u, (reg & U58_LOW_RANGE_BIT) ? 1u : 0u);
    send_ok_str(conn_id, payload);
}

/* The optional "range" of analog_path / adc_read: normal|low (i2c_bus.h adc_range_t).  *want is
   false without one (the range stays as it is).  false = refused, the error is sent. */
static bool range_arg(int conn_id, const char *json, analog_path_t p, adc_range_t *r, bool *want) {
    char v[12] = {0};
    *want = json_get_value(json, "range", v, sizeof(v));
    if (!*want) return true;
    if (adc_range_from_name(v, r) != 0) { send_error(conn_id, "range must be normal or low"); return false; }
    if (*r != ADC_RANGE_LOW) return true;
    if (!adc_range_supported()) { send_error(conn_id, ADC_RANGE_UNSUPPORTED); return false; }
    if (!analog_path_keeps_low_range(p)) {
        send_error(conn_id, "range: the low range is for ext, cal1 and current_in (cal2 and off use normal)");
        return false;
    }
    return true;
}

/* Switch the range once the path is set (adc_range_set waits for K5).  false = the error is sent. */
static bool range_apply(int conn_id, adc_range_t r) {
    int rc = adc_range_set(r);
    if (rc == 0) return true;
    send_error(conn_id, rc == -1 ? ADC_RANGE_UNSUPPORTED
                      : rc == -2 ? "range: cal2 has no low range" : "range: relay switch failed");
    return false;
}

/* analog_path — apply a named analog path (the SINGLE SOURCE OF TRUTH lives in
   i2c_bus.c::analog_path_set).  {"cmd":"analog_path","path":"cal1"} flips every
   switch the path needs and returns the resulting mux/relay registers.
   Names: off dac_3v3|3v3 dac_5v|5v dac_12v|12v adc_ext|ext|sma cal1 cal2 current_in current_out. */
void handle_analog_path(int conn_id, const char *json) {
    char name[16] = {0};
    if (!json_get_value(json, "path", name, sizeof(name))) { send_error(conn_id, "missing path"); return; }
    analog_path_t p;
    if (analog_path_from_name(name, &p) != 0) { send_error(conn_id, "unknown path"); return; }
    adc_range_t r = ADC_RANGE_NORMAL;
    bool want = false;
    if (!range_arg(conn_id, json, p, &r, &want)) return;
    if (analog_path_set(p) != 0) { send_error(conn_id, "analog_path failed"); return; }
    if (want && !range_apply(conn_id, r)) return;
    uint8_t u55 = 0, u58 = 0; dacmux_read(&u55); calsw_read(&u58);
    char payload[96];
    snprintf(payload, sizeof(payload), "{\"path\":\"%s\",\"u55\":%u,\"u58\":%u,\"range\":\"%s\"}",
             analog_path_name(p), u55, u58, adc_range_name(adc_range_get()));
    send_ok_str(conn_id, payload);
}

/* dac_out — route a DAC output path AND set a CALIBRATED voltage (one step, no
   manual mux flipping).  {"cmd":"dac_out","path":"5v","volts":2.5} → achieved
   mv and uv, the 16-bit code16 and its 8-bit equivalent code.  path off just parks the output; omit volts to only route. */
void handle_dac_out(int conn_id, const char *json) {
    char name[16] = {0}, volts_s[16] = {0};
    if (!json_get_value(json, "path", name, sizeof(name))) { send_error(conn_id, "missing path"); return; }
    analog_path_t p; int idx = -1;
    if (analog_path_from_name(name, &p) != 0) { send_error(conn_id, "unknown path"); return; }
    if      (p == ANALOG_PATH_DAC_3V3) idx = 0;
    else if (p == ANALOG_PATH_DAC_5V)  idx = 1;
    else if (p == ANALOG_PATH_DAC_12V) idx = 2;
    else if (p != ANALOG_PATH_OFF) { send_error(conn_id, "path must be 3v3|5v|12v|off"); return; }
    if (analog_path_set(p) != 0) { send_error(conn_id, "route failed"); return; }
    long code = -1, code16 = -1; int got_mv = 0; long got_uv = 0;
    if (idx >= 0 && json_get_value(json, "volts", volts_s, sizeof(volts_s))) {
        /* The full 16 bits of the DAC8551: one step is 1/256 of the 8-bit one (about 76 uV on
           the 5v path).  `code` stays the 8-bit equivalent for older clients. */
        uint16_t c16 = dac_code16_for_volts(idx, (float)atof(volts_s));
        if (dac_set_constant16(c16, 240) != 0) { send_error(conn_id, "dac set failed"); return; }
        float got = dac_volts_for_code16(idx, c16);
        code16 = c16;
        code = (c16 + 128) >> 8;
        if (code > 255) code = 255;
        got_mv = (int)lroundf(got * 1000.0f);
        got_uv = lroundf(got * 1e6f);
    }
    char payload[112];
    if (code16 >= 0)
        snprintf(payload, sizeof(payload),
                 "{\"path\":\"%s\",\"mv\":%d,\"uv\":%ld,\"code\":%ld,\"code16\":%ld}",
                 analog_path_name(p), got_mv, got_uv, code, code16);
    else
        snprintf(payload, sizeof(payload), "{\"path\":\"%s\",\"mv\":%d,\"code\":%ld}",
                 analog_path_name(p), got_mv, code);
    send_ok_str(conn_id, payload);
}

/* current_out — hold a current on the 4-20 mA output (J9), in microamps (current_out.h).
     {"cmd":"current_out","ua":12000} -> {"ua":12000,"code":32576,"min_ua":4016,"max_ua":20078}
     {"cmd":"current_out"}            -> {"min_ua":4016,"max_ua":20078}   (the range; nothing moves)
   `ua` in the reply is the current the nearest 16-bit DAC code gives. A request from 4000 uA up
   to min_ua gives min_ua; anything else outside min_ua..max_ua is refused, there is no clamp.
   Setting a current switches the DAC voltage outputs off first (analog path current_out): they
   share the DAC and would follow it.  Analog v3r2 has a DAC channel for it (dac_quad.h): the
   voltage outputs stay as they are. The loop needs an external floating supply; the pod cannot see
   whether current flows. dac_stop does not return the loop to 4 mA: send 4000 uA for that. */
void handle_current_out(int conn_id, const char *json) {
    char ua_s[24] = {0}, payload[112];
    if (!json_get_value(json, "ua", ua_s, sizeof(ua_s))) {
        snprintf(payload, sizeof(payload), "{\"min_ua\":%ld,\"max_ua\":%ld}",
                 current_out_min_ua(), current_out_max_ua());
        send_ok_str(conn_id, payload);
        return;
    }
    char *end = NULL;
    double req = strtod(ua_s, &end);
    if (end == ua_s || *end != '\0') { send_error(conn_id, "current_out: ua must be a number of microamps"); return; }
    uint16_t code = 0;
    const char *why = current_out_code(lround(req), &code);
    if (why) { send_error(conn_id, why); return; }
    if (dac_quad_present()) {
        /* Analog v3r2: channel D is the 4-20 mA output's own, the voltage outputs stay as they are. */
        if (dac_quad_current_out(code) != 0) { send_error(conn_id, "dac set failed"); return; }
    } else {
        /* Outputs off first, then the level: the voltage outputs never see the new code. */
        if (analog_path_set(ANALOG_PATH_CURRENT_OUT) != 0) { send_error(conn_id, "route failed"); return; }
        if (dac_set_constant16(code, 240) != 0) { send_error(conn_id, "dac set failed"); return; }
    }
    snprintf(payload, sizeof(payload), "{\"ua\":%ld,\"code\":%u,\"min_ua\":%ld,\"max_ua\":%ld}",
             current_out_ua(code), (unsigned)code, current_out_min_ua(), current_out_max_ua());
    send_ok_str(conn_id, payload);
}

/* adc_read — route an ADC source AND return a CALIBRATED reading in mV.
   {"cmd":"adc_read","source":"ext"} → {"source","mv","count","span"}.  `current_in` also
   returns "offset_mv", this pod's calibration offset that was taken out of `mv`
   (adc_cal.h; 0 when the pod was never calibrated), and "ua", the loop current in
   microamps (mv across the 249 ohm sense resistor).  `count` stays raw.  A short
   16-sample burst, averaged ON THE 16-BIT CIRCLE and then unwrapped — see
   adc_scale.h.  Averaging the RAW counts first (what this did until 2026-07-29)
   returns a plausible-looking number that is wrong by tens of volts whenever the
   burst straddles the wrap, which is exactly where every source's 0 V point sits.
   `span` is the burst's pk-pk in counts; a burst wider than ADC_BURST_MAX_SPAN has
   no single voltage (the input is still moving — e.g. a DAC left driving by a
   preceding `measure`/`generate`) and is REFUSED rather than averaged.  Quick, so
   it blocks briefly. */
void handle_adc_read(int conn_id, const char *json) {
    char name[16] = {0};
    analog_path_t p = ANALOG_PATH_ADC_EXT;
    if (json_get_value(json, "source", name, sizeof(name))) {
        if (analog_path_from_name(name, &p) != 0 ||
            (p != ANALOG_PATH_ADC_EXT && p != ANALOG_PATH_CAL1 &&
             p != ANALOG_PATH_CAL2 && p != ANALOG_PATH_CURRENT_IN)) {
            send_error(conn_id, "source must be ext|cal1|cal2|current_in"); return;
        }
    }
    adc_range_t want_r = ADC_RANGE_NORMAL;
    bool want = false;
    if (!range_arg(conn_id, json, p, &want_r, &want)) return;
    if (!heavy_begin(conn_id)) return;
    if (analog_path_set(p) != 0) { heavy_release(conn_id); send_error(conn_id, "route failed"); return; }
    if (want && !range_apply(conn_id, want_r)) { heavy_release(conn_id); return; }
    sleep_ms(20);    /* let the G6K relays (~4ms) + front-end RC settle (yields to FreeRTOS) */
    uint16_t s16[16] = {0};
    int rc = adc_capture_psram(s16, 16, 0.0f);
    heavy_release(conn_id);
    if (rc != 0) { send_error(conn_id, "adc read failed"); return; }
    /* Per-source affine fit.  The UNWRAP is not per-source — it belongs to the
       shared front end and adc_scale_burst always applies it (adc_scale.h). */
    /* This pod's fit for the source: compiled, its offset and gain (adc_cal.h). current_in has no
       fit of its own and is scaled like cal1. */
    adc_range_t range = adc_range_get();
    cal_lin_t c = adc_cal_range_fit(p == ANALOG_PATH_CAL2 ? ADC_CAL_SRC_CAL2
                                  : p == ANALOG_PATH_ADC_EXT ? ADC_CAL_SRC_EXT : ADC_CAL_SRC_CAL1,
                                    range == ADC_RANGE_LOW ? 1 : 0);
    adc_reading_t rd = adc_scale_burst(s16, 16, c.a, c.b);
    /* Samples on the ADC's rails: the input is past the range's full scale (adc_scale.h), and the
       reading is only a bound.  A clipped burst is usually also a wide one, refused below. */
    bool over = adc_count_clipped(s16, 16) > 0;
    if (!rd.valid) {
        /* Plausible-looking garbage is worse than an error: a sweep would record it. */
        char msg[176];   /* 112 cut the advice off the end */
        snprintf(msg, sizeof(msg),
                 "adc_read: input not settled on %s (%u counts pk-pk over the 16-sample "
                 "burst, limit %u). Stop the DAC (dac_stop) or let the node settle",
                 analog_path_name(p), (unsigned)rd.span, (unsigned)ADC_BURST_MAX_SPAN);
        send_error(conn_id, msg);
        return;
    }
    /* The range the reading was taken in, its full scale and whether it clipped. */
    char payload[200];
    int n = snprintf(payload, sizeof(payload),
                     "{\"source\":\"%s\",\"mv\":%d,\"count\":%d,\"span\":%u,\"range\":\"%s\","
                     "\"fullscale_mv\":%ld,\"overrange\":%s",
                     analog_path_name(p), (int)lroundf(rd.volts * 1000.0f),
                     adc_count_u16(rd.count), (unsigned)rd.span, adc_range_name(range),
                     adc_cal_fullscale_mv(c), over ? "true" : "false");
    if (p == ANALOG_PATH_CURRENT_IN && n > 0 && n < (int)sizeof(payload))
        /* Say which per-pod offset was taken out of `mv` (every source gets it), and give the
           loop current the voltage stands for, so no client needs to know the sense resistor. */
        n += snprintf(payload + n, sizeof(payload) - (size_t)n, ",\"offset_mv\":%ld,\"ua\":%ld",
                      (long)adc_cal_current_in_offset_mv(), current_in_ua(rd.volts));
    if (n > 0 && n < (int)sizeof(payload)) snprintf(payload + n, sizeof(payload) - (size_t)n, "}");
    send_ok_str(conn_id, payload);
}

/* A DMM line against the compiled one, for the reply: the offset at the code that gives 0 V (12v)
   or at code 0 (3v3, 5v, whose range starts there), microvolts, and the gain difference in ppm. */
static void dac_line_vs_compiled(int path, cal_lin_t pod, long *offset_uv, long *gain_ppm) {
    cal_lin_t c = DAC_CAL_COMPILED[path];
    double code8 = (path == 2 && c.b != 0.0f) ? -(double)c.a / (double)c.b : 0.0;
    double comp = (double)c.a + (double)c.b * code8, mine = (double)pod.a + (double)pod.b * code8;
    *offset_uv = lround((mine - comp) * 1e6);
    *gain_ppm = c.b != 0.0f ? lround(((double)pod.b / (double)c.b - 1.0) * 1e6) : 0;
}

/* The per-pod calibration as a reply. a_uv / b_nv are the fit this pod scales `current_in` with
   (volts = a + b*count), in the units of the capabilities frame's adc_cal_a_uv / adc_cal_b_nv, so a
   client can scale raw `current_in` counts the way adc_read does. `dac` and `adc_gain_ppm` are the
   DMM parts (null = the compiled fit); a DAC line is volts = a + b*code16, a in uV, b in pV per
   16-bit code. `rd` (an offset run) and `gm` (a gain run) add what was measured. */
static void calibrate_reply(int conn_id, const adc_reading_t *rd, const adc_cal_gain_meas_t *gm) {
    static const char *const path_name[ADC_CAL_DAC_PATHS] = { "3v3", "5v", "12v" };
    cal_lin_t fit = adc_cal_current_in_fit();
    bool dmm = false;
    /* The whole frame is built here and sent as is: with the DMM parts it is up to about 650 B,
       past send_ok_str's 256 B. */
    char payload[832];
    int n = snprintf(payload, sizeof(payload),
                     "{\"status\":\"ok\",\"data\":{\"source\":\"current_in\",\"calibrated\":%s,\"offset_mv\":%ld,\"offset_uv\":%ld,"
                     "\"a_uv\":%ld,\"b_nv\":%ld,\"dac\":{",
                     adc_cal_current_in_is_set() ? "true" : "false",
                     (long)adc_cal_current_in_offset_mv(), (long)adc_cal_current_in_offset_uv(),
                     lround((double)fit.a * 1000000.0), lround((double)fit.b * 1000000000.0));
    for (int i = 0; i < ADC_CAL_DAC_PATHS && n < (int)sizeof(payload); i++) {
        const char *sep = i ? "," : "";
        if (!adc_cal_dac_is_set(i)) {
            n += snprintf(payload + n, sizeof(payload) - (size_t)n, "%s\"%s\":null", sep, path_name[i]);
            continue;
        }
        dmm = true;
        cal_lin_t l = adc_cal_dac_line(i);
        long off_uv, gain_ppm;
        dac_line_vs_compiled(i, l, &off_uv, &gain_ppm);
        n += snprintf(payload + n, sizeof(payload) - (size_t)n,
                      "%s\"%s\":{\"a_uv\":%ld,\"b_pv\":%ld,\"offset_uv\":%ld,\"gain_ppm\":%ld}", sep, path_name[i],
                      lround((double)l.a * 1e6), lround((double)l.b / 256.0 * 1e12), off_uv, gain_ppm);
    }
    if (n < (int)sizeof(payload)) n += snprintf(payload + n, sizeof(payload) - (size_t)n, "},\"adc_gain_ppm\":{");
    for (int i = 0; i < ADC_CAL_SRCS && n < (int)sizeof(payload); i++) {
        const char *sep = i ? "," : "";
        if (!adc_cal_gain_is_set(i)) {
            n += snprintf(payload + n, sizeof(payload) - (size_t)n, "%s\"%s\":null", sep, adc_cal_src_name(i));
            continue;
        }
        dmm = true;
        n += snprintf(payload + n, sizeof(payload) - (size_t)n, "%s\"%s\":%ld", sep, adc_cal_src_name(i),
                      lroundf((adc_cal_gain(i) - 1.0f) * 1e6f));
    }
    if (n < (int)sizeof(payload)) n += snprintf(payload + n, sizeof(payload) - (size_t)n, "}");
    /* The low range's gain (analog module v3r4), on pods that have it. */
    if (adc_range_supported() && n < (int)sizeof(payload)) {
        if (adc_cal_low_gain_is_set()) {
            dmm = true;
            n += snprintf(payload + n, sizeof(payload) - (size_t)n, ",\"adc_low_gain_ppm\":%ld",
                          lroundf((adc_cal_low_gain() - 1.0f) * 1e6f));
        } else {
            n += snprintf(payload + n, sizeof(payload) - (size_t)n, ",\"adc_low_gain_ppm\":null");
        }
    }
    if (n < (int)sizeof(payload)) n += snprintf(payload + n, sizeof(payload) - (size_t)n, ",\"dmm\":%s", dmm ? "true" : "false");
    if (adc_cal_current_in_is_set() && n < (int)sizeof(payload))   /* the raw zero the offset is derived from */
        n += snprintf(payload + n, sizeof(payload) - (size_t)n, ",\"zero_count_mc\":%ld", (long)adc_cal_zero_count_mc());
    if (rd && n < (int)sizeof(payload))
        n += snprintf(payload + n, sizeof(payload) - (size_t)n, ",\"count\":%d,\"span\":%u,\"samples\":%u",
                      adc_count_u16(rd->count), (unsigned)rd->span, (unsigned)ADC_CAL_SAMPLES);
    if (gm && n < (int)sizeof(payload))
        n += snprintf(payload + n, sizeof(payload) - (size_t)n,
                      ",\"measured\":{\"v1_uv\":%ld,\"v2_uv\":%ld,\"r1_uv\":%ld,\"r2_uv\":%ld,\"residual_uv\":%ld}",
                      lroundf(gm->v[0] * 1e6f), lroundf(gm->v[1] * 1e6f), lroundf(gm->r[0] * 1e6f),
                      lroundf(gm->r[1] * 1e6f), lroundf(gm->residual_v * 1e6f));
    if (n < (int)sizeof(payload)) n += snprintf(payload + n, sizeof(payload) - (size_t)n, "}}\n");
    if (n >= (int)sizeof(payload)) { send_error(conn_id, "calibrate: reply too long"); return; }
    printf("[cmd] -> ok  %s", payload);
    if (at_send_data(conn_id, (const uint8_t *)payload, (size_t)n) != 0) at_close_connection(conn_id);
}

/* A whole number from the request, e.g. "uv1":-5025500. */
static bool json_long(const char *json, const char *key, long *out) {
    char v[24] = {0};
    if (!json_get_value(json, key, v, sizeof(v)) || !v[0]) return false;
    char *end = NULL;
    long x = strtol(v, &end, 10);
    if (end == v || *end != '\0') return false;
    *out = x;
    return true;
}

/* `calibrate` — this pod's own calibration (adc_cal.h), on top of the compiled-in fits
   (cal_data.h) and kept in flash.
     {"cmd":"calibrate"}                         -> the stored calibration
     {"cmd":"calibrate","source":"current_in"}   -> the ADC offset: J8 must be DISCONNECTED
     {"cmd":"calibrate","dac":"5v","code1":N,"uv1":N,"code2":N,"uv2":N}
                                                 -> this pod's DAC line for a path, from two
                                                    DMM readings of set points it drove
     {"cmd":"calibrate","adc":"cal1"}            -> an ADC gain (cal1|cal2|ext) via a loopback
     {"cmd":"calibrate","clear":true}            -> back to the compiled-in fits
     {"cmd":"calibrate","clear":"dac"}           -> one part: offset|dac|adc
   `current_in` is the source a pod can calibrate on its own: with J8 open the terminal is 0 V,
   so what it reads is the front end's offset, which every source shares (adc_cal.h). A reading
   outside +/-150 mV means something is driving J8: it is refused and the old calibration stays.
   The DMM forms are refused while DAC output limits are set (dac_limits.c). */
void handle_calibrate(int conn_id, const char *json) {
    char v[16] = {0};
    if (json_get_value(json, "clear", v, sizeof(v)) && strcmp(v, "false") != 0) {
        int rc = strcmp(v, "true") == 0 ? adc_cal_clear() : adc_cal_clear_part(v);
        if (rc == -2) { send_error(conn_id, "calibrate: clear must be true, offset, dac or adc"); return; }
        if (rc != 0) { send_error(conn_id, "could not clear the calibration"); return; }
        cloud_client_request_caps_resend();
        calibrate_reply(conn_id, NULL, NULL);
        return;
    }
    if (json_get_value(json, "dac", v, sizeof(v))) {
        int path = dac_limits_path_index(v);
        long c1, u1, c2, u2;
        if (path < 0) { send_error(conn_id, "calibrate: dac must be 3v3, 5v or 12v"); return; }
        if (!json_long(json, "code1", &c1) || !json_long(json, "uv1", &u1) ||
            !json_long(json, "code2", &c2) || !json_long(json, "uv2", &u2)) {
            send_error(conn_id, "calibrate: dac needs code1, uv1, code2 and uv2 (16-bit codes, DMM microvolts)");
            return;
        }
        if (c1 < 0 || c1 > 65535 || c2 < 0 || c2 > 65535 || labs(u1) > 20000000L || labs(u2) > 20000000L) {
            send_error(conn_id, "calibrate: codes are 0-65535 and readings within +/-20 V");
            return;
        }
        const char *why = adc_cal_dac_store(path, (uint16_t)c1, (int32_t)u1, (uint16_t)c2, (int32_t)u2);
        if (why) { send_error(conn_id, why); return; }
        calibrate_reply(conn_id, NULL, NULL);
        return;
    }
    if (json_get_value(json, "adc", v, sizeof(v))) {
        int src = adc_cal_src_index(v);
        /* "range":"low": the low range's gain, through cal1 (adc_cal.h). */
        char rv[12] = {0};
        bool low = json_get_value(json, "range", rv, sizeof(rv)) && strcmp(rv, "normal") != 0;
        if (low && strcmp(rv, "low") != 0) { send_error(conn_id, "calibrate: range must be normal or low"); return; }
        if (low && !adc_range_supported()) { send_error(conn_id, ADC_RANGE_UNSUPPORTED); return; }
        if (low && src != ADC_CAL_SRC_CAL1) {
            send_error(conn_id, "calibrate: the low range is calibrated through cal1 (adc cal1), and serves every source");
            return;
        }
        const char *why = adc_cal_gain_ready(src);
        if (why) { send_error(conn_id, why); return; }
        if (!heavy_begin(conn_id)) return;
        adc_cal_gain_meas_t m = {0};
        uint32_t span = 0;
        int rc = adc_cal_measure_gain(src, low, &m, &span);
        heavy_release(conn_id);
        if (rc == -3) {
            char msg[160];
            snprintf(msg, sizeof(msg), "calibrate: %s not settled (%lu counts pk-pk). Is something connected to the DAC output?",
                     adc_cal_src_name(src), (unsigned long)span);
            send_error(conn_id, msg);
            return;
        }
        if (rc != 0) { send_error(conn_id, rc == -1 ? "route failed" : "adc read failed"); return; }
        why = low ? adc_cal_low_gain_store(&m) : adc_cal_gain_store(src, &m);
        if (why) { send_error(conn_id, why); return; }
        cloud_client_request_caps_resend();   /* the ext fit in the capabilities may have changed */
        calibrate_reply(conn_id, NULL, &m);
        return;
    }
    if (!json_get_value(json, "source", v, sizeof(v))) { calibrate_reply(conn_id, NULL, NULL); return; }
    if (strcmp(v, "current_in") != 0) { send_error(conn_id, "only current_in can be calibrated on the pod: source must be current_in"); return; }
    if (!heavy_begin(conn_id)) return;
    adc_reading_t rd;
    int rc = adc_cal_measure_current_in(&rd);
    heavy_release(conn_id);
    if (rc != 0) { send_error(conn_id, rc == -1 ? "route failed" : "adc read failed"); return; }
    const char *why = adc_cal_current_in_store(&rd);
    if (why) { send_error(conn_id, why); return; }
    cloud_client_request_caps_resend();   /* the ext fit in the capabilities changed */
    calibrate_reply(conn_id, &rd, NULL);
}

/* adc_range — the ADC input range (analog module v3r4 and later, relay K5; i2c_bus.h).
     {"cmd":"adc_range"}                  -> {"range":"normal","supported":true,"fullscale_mv":32850,
                                              "normal_fullscale_mv":32850,"low_fullscale_mv":5461}
     {"cmd":"adc_range","range":"low"}    -> switch, wait for K5 to settle (10 ms), the same reply
     {"cmd":"adc_range","test":true}      -> the self-test through the cal1 loopback (adc_cal.h)
   The range applies to ext, cal1 and current_in (it is one divider in front of all of them); cal2
   and the `off` path put it back to normal, as does a restart.  The low range has a 1.82 MOhm
   input: 1x leads, not 10x probes.  The full scales come from this pod's ext fits.  On a board
   without K5 the range is always normal and asking for low is refused. */
void handle_adc_range(int conn_id, const char *json) {
    char v[12] = {0};
    if (json_get_value(json, "test", v, sizeof(v)) && strcmp(v, "false") != 0) {
        if (!adc_range_supported()) { send_error(conn_id, ADC_RANGE_UNSUPPORTED); return; }
        if (!heavy_begin(conn_id)) return;
        adc_range_test_pt_t pts[ADC_RANGE_TEST_POINTS];
        uint32_t span = 0;
        int rc = adc_range_selftest(pts, &span);
        heavy_release(conn_id);
        if (rc == -3) {
            char msg[128];
            snprintf(msg, sizeof(msg), "adc_range: cal1 not settled (%lu counts pk-pk). Is something connected to the 5 V output?",
                     (unsigned long)span);
            send_error(conn_id, msg);
            return;
        }
        if (rc != 0) { send_error(conn_id, rc == -1 ? "route failed" : "adc read failed"); return; }
        bool pass = true;
        char payload[256];
        int n = snprintf(payload, sizeof(payload), "{\"points\":[");
        for (int i = 0; i < ADC_RANGE_TEST_POINTS && n < (int)sizeof(payload); i++) {
            pass = pass && pts[i].ok;
            n += snprintf(payload + n, sizeof(payload) - (size_t)n,
                          "%s{\"set_mv\":%ld,\"normal_mv\":%ld,\"low_mv\":%ld,\"ok\":%s}", i ? "," : "",
                          lroundf(pts[i].set_v * 1000.0f), lroundf(pts[i].normal_v * 1000.0f),
                          lroundf(pts[i].low_v * 1000.0f), pts[i].ok ? "true" : "false");
        }
        if (n < (int)sizeof(payload))
            snprintf(payload + n, sizeof(payload) - (size_t)n, "],\"pass\":%s,\"range\":\"%s\"}",
                     pass ? "true" : "false", adc_range_name(adc_range_get()));
        send_ok_str(conn_id, payload);
        return;
    }
    if (json_get_value(json, "range", v, sizeof(v))) {
        adc_range_t r;
        if (adc_range_from_name(v, &r) != 0) { send_error(conn_id, "range must be normal or low"); return; }
        if (r == ADC_RANGE_LOW && !adc_range_supported()) { send_error(conn_id, ADC_RANGE_UNSUPPORTED); return; }
        if (!range_apply(conn_id, r)) return;
    }
    adc_range_t r = adc_range_get();
    cal_lin_t normal = adc_cal_range_fit(ADC_CAL_SRC_EXT, 0), low = adc_cal_range_fit(ADC_CAL_SRC_EXT, 1);
    char payload[176];
    if (adc_range_supported())
        snprintf(payload, sizeof(payload),
                 "{\"range\":\"%s\",\"supported\":true,\"fullscale_mv\":%ld,\"normal_fullscale_mv\":%ld,"
                 "\"low_fullscale_mv\":%ld}",
                 adc_range_name(r), adc_cal_fullscale_mv(r == ADC_RANGE_LOW ? low : normal),
                 adc_cal_fullscale_mv(normal), adc_cal_fullscale_mv(low));
    else
        snprintf(payload, sizeof(payload),
                 "{\"range\":\"normal\",\"supported\":false,\"fullscale_mv\":%ld,\"normal_fullscale_mv\":%ld}",
                 adc_cal_fullscale_mv(normal), adc_cal_fullscale_mv(normal));
    send_ok_str(conn_id, payload);
}
