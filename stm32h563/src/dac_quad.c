/* dac_quad.c — the pure half of dac_quad.h: the analog v3r2 DAC8554 channel map. */
#include "dac_quad.h"
#include "cal_data.h"     /* dac_code16_for_volts */
#include "dac_limits.h"   /* the park level of the limited output */
#include "board_rev.h"    /* BOARD_REV_V3R2 */

static bool s_present;

void dac_quad_set_present(bool present) { s_present = present; }
bool dac_quad_present(void)              { return s_present; }

bool dac_quad_for_board(int board_rev, bool has_analog)
{
    return has_analog && board_rev == BOARD_REV_V3R2;
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
    if (l->enabled && (int)l->path == ch) return (uint16_t)((unsigned)dac_limits_park_code() << 8);
    return dac_code16_for_volts(ch, 0.0f);   /* channel A..C = dac[] index 0..2 */
}
