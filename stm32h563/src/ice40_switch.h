#ifndef ICE40_SWITCH_H
#define ICE40_SWITCH_H

/*
 * ice40_switch.c: the decision and the bookkeeping around a runtime gateware image switch
 * (fpga_warmboot, signal_engine.c). Pure logic, host-tested (test/test_ice40_switch.c).
 *
 * A switch used to rewrite the iCE40 config flash every time, also when the requested image was
 * already running: 27 sector erases for nothing.
 * ice40_switch_plan() says when the running image already is the one asked for, and the counters
 * show how often the pod switches and how much flash that cost since boot (`status`).
 */
#include <stdbool.h>
#include <stdint.h>
#include "bp_json.h"

typedef enum {
    ICE40_SW_SKIP = 0,   /* the requested image runs and answers: nothing to do */
    ICE40_SW_LOAD = 1,   /* load the requested image */
} ice40_sw_action_t;

/* want: the requested image (0 loop, 1 deep, 2 emulation). running: the image a LIVE
   FPGA_FEATURES read reports, -1 when that read failed or the iCE40 does not answer.
   configured: CDONE high. healthy: the iCE40->PSRAM write path works (status.psram_ok), so a
   reload is not the way to recover it. current: the running gateware version is the one this
   firmware carries (or the firmware does not know its own); when not, a request for the running
   kind reloads it from its slot (`benchpod install-blobs` relies on that after a downgrade).
   force: the caller wants a reload regardless. */
ice40_sw_action_t ice40_switch_plan(int want, int running, bool configured, bool healthy,
                                    bool current, bool force);

typedef struct {
    uint32_t switches;     /* image loads (requests that were not skipped) */
    uint32_t skipped;      /* requests for the image already running */
    uint32_t reflashes;    /* writes of the config flash's boot image (offset 0) */
    uint32_t erases;       /* 4 KB sector erases of the config flash by those writes */
    uint32_t failed;       /* image loads that did not end on the requested image */
} ice40_switch_stats_t;

void ice40_switch_note_skip(void);
void ice40_switch_note_switch(void);
void ice40_switch_note_failed(void);
void ice40_switch_note_reflash(void);
void ice40_switch_note_erases(uint32_t n);
const ice40_switch_stats_t *ice40_switch_stats(void);
void ice40_switch_reset_stats(void);   /* tests */

/* "fpga_switch":{"switches":N,"skipped":N,"reflashes":N,"erases":N,"failed":N} (no comma). */
void ice40_switch_emit(bp_emit_t *e);

#endif /* ICE40_SWITCH_H */
