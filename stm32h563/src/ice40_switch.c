/*
 * ice40_switch.c: see ice40_switch.h.
 */
#include "ice40_switch.h"
#include <string.h>

static ice40_switch_stats_t s_stats;

ice40_sw_action_t ice40_switch_plan(int want, int running, bool configured, bool healthy,
                                    bool current, bool force)
{
    if (force) return ICE40_SW_LOAD;
    /* Only skip on positive proof: CDONE high, a live features read naming this image, a working
       PSRAM write path and the gateware version the firmware expects. Anything else loads. */
    if (configured && healthy && current && running >= 0 && running == want) return ICE40_SW_SKIP;
    return ICE40_SW_LOAD;
}

void ice40_switch_note_skip(void)          { s_stats.skipped++; }
void ice40_switch_note_switch(void)        { s_stats.switches++; }
void ice40_switch_note_failed(void)        { s_stats.failed++; }
void ice40_switch_note_reflash(void)       { s_stats.reflashes++; }
void ice40_switch_note_erases(uint32_t n)  { s_stats.erases += n; }
void ice40_switch_note_slave(bool ok)      { if (ok) s_stats.slave_loads++; else s_stats.slave_fails++; }
void ice40_switch_note_time_ms(uint32_t ms) { s_stats.last_ms = ms; }
const ice40_switch_stats_t *ice40_switch_stats(void) { return &s_stats; }
void ice40_switch_reset_stats(void)        { memset(&s_stats, 0, sizeof(s_stats)); }

void ice40_switch_emit(bp_emit_t *e)
{
    const ice40_switch_stats_t *t = &s_stats;
    bp_emit(e, "\"fpga_switch\":{\"switches\":%lu,\"skipped\":%lu,\"failed\":%lu,\"slave_loads\":%lu,"
               "\"slave_fails\":%lu,\"reflashes\":%lu,\"erases\":%lu,\"last_ms\":%lu}",
            (unsigned long)t->switches, (unsigned long)t->skipped, (unsigned long)t->failed,
            (unsigned long)t->slave_loads, (unsigned long)t->slave_fails,
            (unsigned long)t->reflashes, (unsigned long)t->erases, (unsigned long)t->last_ms);
}
