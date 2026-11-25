/* dac_quad.c — the pure half of dac_quad.h: the analog v3r2 DAC8554 channel map. */
#include "dac_quad.h"
#include "cal_data.h"     /* dac_code16_for_volts */
#include "dac_limits.h"   /* the park level of the limited output */
#include "bp_json.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static bool s_present;

void dac_quad_set_present(bool present) { s_present = present; }
bool dac_quad_present(void)              { return s_present; }

bool dac_quad_for_board(bool board_quad, bool has_analog, bool module)
{
    return has_analog && (module || board_quad);
}

uint8_t dac_quad_ctrl(int ch)
{
    if (ch < DAC_QUAD_A || ch > DAC_QUAD_D) return 0x00u;
    return (uint8_t)(0x10u | ((unsigned)ch << 1));   /* LD1:LD0 = 01, DB18:17 = ch, PD0 = 0 */
}

const char *dac_quad_name(int ch)
{
    static const char *const n[4] = { "A", "B", "C", "D" };
    return (ch >= DAC_QUAD_A && ch <= DAC_QUAD_D) ? n[ch] : "none";
}

/* The channel a path points the DAC at; DAC_QUAD_NONE for off.  -2 = the path does not touch the
   DAC (adc_ext and current_in only move relays, as on v3). */
static int path_channel(analog_path_t path)
{
    switch (path) {
    case ANALOG_PATH_OFF:         return DAC_QUAD_NONE;
    case ANALOG_PATH_DAC_3V3:     return DAC_QUAD_A;
    case ANALOG_PATH_DAC_5V:      return DAC_QUAD_B;
    case ANALOG_PATH_CAL1:        return DAC_QUAD_B;   /* the 5 V stage through K1 */
    case ANALOG_PATH_DAC_12V:     return DAC_QUAD_C;
    case ANALOG_PATH_CAL2:        return DAC_QUAD_C;   /* the +-12 V stage's cal2 copy (U51C) */
    case ANALOG_PATH_CURRENT_OUT: return DAC_QUAD_D;
    default:                      return -2;
    }
}

dac_quad_plan_t dac_quad_plan(analog_path_t path, int cur_ch, uint8_t live)
{
    dac_quad_plan_t p = { false, cur_ch, false, 0u };
    int ch = path_channel(path);
    if (ch == -2) return p;
    p.touch = true;
    live &= DAC_QUAD_VOLTAGE_MASK;
    if (path == ANALOG_PATH_OFF) {
        /* Voltage outputs off.  A 4-20 mA stream keeps playing: on v3 "off" opened the voltage
           muxes, and the loop current followed the DAC whatever they did. */
        p.ch   = (cur_ch == DAC_QUAD_D) ? DAC_QUAD_D : DAC_QUAD_NONE;
        p.stop = (cur_ch != DAC_QUAD_NONE && cur_ch != DAC_QUAD_D);
        p.park = live;
        return p;
    }
    p.ch   = ch;
    p.stop = (cur_ch != DAC_QUAD_NONE && cur_ch != ch);   /* the stream moves: end it */
    /* One voltage output at a time: park every other voltage channel.  current_out leaves the
       voltage outputs alone (the 4-20 mA output is independent on v3r2). */
    if (ch != DAC_QUAD_D) p.park = (uint8_t)(live & ~(1u << ch));
    return p;
}

uint16_t dac_quad_park_code(int ch)
{
    const dac_limits_t *l = dac_limits_get();
    if (l->enabled && (int)l->path == ch) return dac_limits_park_code16();
    return dac_code16_for_volts(ch, 0.0f);   /* channel A..C = dac[] index 0..2 */
}

/* ---- The ADC low range and the +-12 V output (dac_quad.h) ------------------------------------
   No %f: newlib-nano's printf has none, so volts are printed as whole mV. */

static char s_low_msg[288];

#define LOW_RANGE_NOTE "the ADC is in the low range (+-5.46 V full scale)"
#define TO_NORMAL      "Switch the ADC to the normal range first: {\"cmd\":\"adc_range\",\"range\":\"normal\"}"

const char *dac_quad_low_last(void) { return s_low_msg; }

static long c_mv(uint16_t code) { return lroundf(dac_volts_for_code16(DAC_QUAD_C, code) * 1000.0f); }

static bool mv_ok(long mv) { return labs(mv) <= DAC_QUAD_LOW_RANGE_MAX_MV + DAC_QUAD_LOW_RANGE_TOL_MV; }

static const char *refuse_mv(long mv) {
    snprintf(s_low_msg, sizeof(s_low_msg),
             "refused: %s, and %ld mV on the 12v output is past +-%d mV. %s",
             LOW_RANGE_NOTE, mv, DAC_QUAD_LOW_RANGE_MAX_MV, TO_NORMAL);
    return s_low_msg;
}

static const char *refuse_stream(const char *what) {
    snprintf(s_low_msg, sizeof(s_low_msg),
             "refused: %s, and %.24s on the 12v output is not held within +-%d mV. %s",
             LOW_RANGE_NOTE, what, DAC_QUAD_LOW_RANGE_MAX_MV, TO_NORMAL);
    return s_low_msg;
}

const char *dac_quad_low_check_hold(bool low, int ch, uint16_t code)
{
    if (!low || ch != DAC_QUAD_C) return NULL;
    long mv = c_mv(code);
    return mv_ok(mv) ? NULL : refuse_mv(mv);
}

const char *dac_quad_low_check_stream(bool low, int ch)
{
    if (!low || ch != DAC_QUAD_C) return NULL;
    return refuse_stream("a waveform or control loop");
}

const char *dac_quad_low_check_enter(bool c_known, uint16_t c_code)
{
    if (c_known && mv_ok(c_mv(c_code))) return NULL;
    if (c_known)
        snprintf(s_low_msg, sizeof(s_low_msg),
                 "refused: the low range (+-5.46 V full scale) while the 12v output holds %ld mV, past +-%d mV. "
                 "Set it within +-5 V or switch it off first ({\"cmd\":\"dac_out\",\"path\":\"off\"})",
                 c_mv(c_code), DAC_QUAD_LOW_RANGE_MAX_MV);
    else
        snprintf(s_low_msg, sizeof(s_low_msg),
                 "refused: the low range (+-5.46 V full scale) after a waveform or control loop played on the "
                 "12v output. dac_stop, then switch it off ({\"cmd\":\"dac_out\",\"path\":\"off\"}) "
                 "or hold a level within +-5 V first");
    return s_low_msg;
}

const char *dac_quad_low_check_command(const char *cmd, const char *json, bool low, int cur_ch)
{
    if (!low || !cmd) return NULL;
    char v[24] = {0};
    if (!strcmp(cmd, "dac_out")) {
        bp_json_get(json, "path", v, sizeof(v));
        analog_path_t p;
        if (analog_path_from_name(v, &p) != 0 || p != ANALOG_PATH_DAC_12V) return NULL;
        if (!bp_json_get(json, "v", v, sizeof(v))) return NULL;   /* route only: C keeps its level */
        /* The handler converts with atof and the 16-bit code; check the code it will hold. */
        return dac_quad_low_check_hold(true, DAC_QUAD_C, dac_code16_for_volts(DAC_QUAD_C, (float)atof(v)));
    }
    int ch = cur_ch;
    if (!strcmp(cmd, "generate")) {
        /* The calibrated form routes `path` first unless "route":false. */
        char r[8] = {0};
        bool route_off = bp_json_get(json, "route", r, sizeof(r)) && !strcmp(r, "false");
        analog_path_t p;
        if (bp_json_get(json, "amplitude_ua", v, sizeof(v))) ch = route_off ? cur_ch : DAC_QUAD_D;
        else if (!route_off && bp_json_get(json, "path", v, sizeof(v)) && analog_path_from_name(v, &p) == 0)
            ch = p == ANALOG_PATH_DAC_12V ? DAC_QUAD_C : DAC_QUAD_NONE;   /* another output: not C */
        return ch == DAC_QUAD_C ? refuse_stream("generate") : NULL;
    }
    static const char *const streams[] = {
        "dac_set", "load", "load_bin", "replay", "measure", "dac_control_loop",
    };
    for (size_t i = 0; i < sizeof(streams) / sizeof(streams[0]); i++)
        if (!strcmp(cmd, streams[i])) return ch == DAC_QUAD_C ? refuse_stream(cmd) : NULL;
    return NULL;
}
