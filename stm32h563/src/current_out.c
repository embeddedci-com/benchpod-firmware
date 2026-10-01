/*
 * current_out.c — the 4-20 mA output's transfer function (see current_out.h).
 *
 * Pure: no HAL, so the whole file runs in the host tests.
 */
#include "current_out.h"

#include <math.h>
#include <stdio.h>

/* Refusal text. Commands are dispatched one at a time and the caller copies the text into its
   reply straight away, so one buffer is enough. */
static char s_msg[160];

/* uA at code 0, and uA per code. Double: the step is 0.245 uA, a float loses it at the top. */
static double zero_ua(void) {
    return (double)CURRENT_OUT_GAIN * (double)CURRENT_OUT_AREF_UV / (double)CURRENT_OUT_ZERO_OHMS;
}
static double ua_per_code(void) {
    return (double)CURRENT_OUT_GAIN * (double)CURRENT_OUT_AREF_UV / (double)CURRENT_OUT_SPAN_OHMS / 65536.0;
}

long current_out_ua(uint16_t code) { return lround(zero_ua() + ua_per_code() * (double)code); }
long current_out_min_ua(void)      { return current_out_ua(0); }
long current_out_max_ua(void)      { return current_out_ua(0xFFFF); }

const char *current_out_code(long ua, uint16_t *code) {
    if (ua < CURRENT_OUT_REQUEST_MIN_UA || ua > current_out_max_ua()) {
        snprintf(s_msg, sizeof(s_msg),
                 "current_out: %ld uA is out of range. The output can do %ld to %ld uA: "
                 "it cannot go below the 4 mA live zero or above the top of the DAC",
                 ua, current_out_min_ua(), current_out_max_ua());
        return s_msg;
    }
    long c = lround(((double)ua - zero_ua()) / ua_per_code());
    if (c < 0) c = 0;
    if (c > 0xFFFF) c = 0xFFFF;
    if (code) *code = (uint16_t)c;
    return NULL;
}
