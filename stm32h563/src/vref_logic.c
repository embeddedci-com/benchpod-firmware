/* vref_logic.c — the pure part of vref_out.h (host tests). */
#include "vref_out.h"

int vref_setpoint_mv(vref_src_t src, int bank_mv)
{
    if (src == VREF_SRC_5V) return VREF_5V_NOMINAL_MV;
    return (bank_mv == 1800 || bank_mv == 3300) ? bank_mv : 3300;
}

vref_state_t vref_classify(vref_src_t src, int set_mv, int mv)
{
    if (mv < 0 || set_mv <= 0) return VREF_UNKNOWN;
    if (mv < set_mv / 2) return VREF_SHORT;
    if (src == VREF_SRC_5V) {
        if (mv < VREF_5V_MIN_MV) return VREF_LOW;
        if (mv > VREF_5V_MAX_MV) return VREF_HIGH;
        return VREF_OK;
    }
    if (mv < set_mv - VREF_LOW_MARGIN_MV) return VREF_LOW;
    if (mv > set_mv + VREF_HIGH_MARGIN_MV) return VREF_HIGH;
    return VREF_OK;
}

bool vref_rail_up(bool was_up, int mv)
{
    if (mv < 0) return was_up;
    return was_up ? mv >= VREF_DOWN_MV : mv >= VREF_UP_MV;
}

const char *vref_state_name(vref_state_t s)
{
    switch (s) {
    case VREF_OK:    return "ok";
    case VREF_HIGH:  return "high";
    case VREF_LOW:   return "low";
    case VREF_SHORT: return "short";
    default:         return "unknown";
    }
}

const char *vref_src_name(vref_src_t s) { return s == VREF_SRC_5V ? "5v" : "bank"; }

void vref_emit_fields(bp_emit_t *e, vref_src_t src, int set_mv, int mv, vref_state_t st, bool up)
{
    bp_emit(e, "\"vref_out\":{\"source\":\"%s\",\"set_mv\":%d,\"mv\":%d,\"state\":\"%s\",\"up\":%s}",
            vref_src_name(src), set_mv, mv, vref_state_name(st), up ? "true" : "false");
}
