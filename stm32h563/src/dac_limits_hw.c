/*
 * dac_limits_hw.c — the hardware half of dac_limits.h: hold the DAC at the park level.
 * Kept out of dac_limits.c so the checks and the store stay host-testable.
 */
#include "dac_limits.h"
#include "i2c_bus.h"         /* analog_path_set, analog_path_from_name */
#include "signal_engine.h"   /* dac_set_constant */
#include "bp_log.h"

#include <stdio.h>

int dac_limits_park_now(void) {
    const dac_limits_t *l = dac_limits_get();
    if (!l->enabled) return 1;
    analog_path_t p;
    if (analog_path_from_name(dac_limits_path_name(l->path), &p) != 0) return -1;
    uint8_t code = dac_limits_park_code();
    /* Level first, then route: the path then connects straight onto the safe level instead of
       whatever the DAC held (0 at boot, i.e. full output on an inverted stage). */
    if (dac_set_constant(code, 240) != 0) { log_printf("[dac-limits] park: DAC set failed\r\n"); return -1; }
    if (analog_path_set(p) != 0) { log_printf("[dac-limits] park: route failed\r\n"); return -1; }
    log_printf("[dac-limits] parked %s at code %u (%ld mV)\r\n", dac_limits_path_name(l->path), code,
           (long)(l->inverted ? l->max_mv : l->min_mv));
    return 0;
}
