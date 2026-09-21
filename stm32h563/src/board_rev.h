#ifndef BOARD_REV_H
#define BOARD_REV_H

#include <stdbool.h>

/* ============================================================================
 * board_rev — which BenchPod PCB revision this firmware is running on.
 *
 * rev3 added a strap on PA3: R164 (1 k) to +3V3 over R163 (10 k) to GND, so the
 * pin idles at ~3.0 V.  On v2 the pad is not connected to anything.  Detection
 * is two steps:
 *
 *   1. PA3 as a digital input with the internal pull-down enabled.  An NC pad
 *      collapses to 0; the 1 k/10 k divider does not (10 k in parallel with the
 *      ~40 k internal pull-down still leaves ~2.9 V), so LOW means v2.
 *   2. If it did not collapse, read the strap voltage on ADC1_INP15 and map it
 *      to a revision.  Today only the ~3.0 V bucket is defined (v3); a future
 *      board can pick a different divider ratio without changing this contract.
 *
 * The revision decides three things that are NOT compatible between v2 and v3:
 *   - the TPS2116 LA-bank mux polarity, and whether 1.8 V is reachable at all
 *     (on v2, MODE is grounded, so the "1.8 V" setting shuts the mux down —
 *     see signal_engine.c la_vccio_set_mv);
 *   - whether the dedicated NRST-control pin exists (nrst_ctrl.c);
 *   - whether USB-C CC monitoring exists (usb_cc.c).
 *
 * Sampled once at boot; every accessor is a plain RAM read afterwards, so they
 * are safe to call from any task.
 * ==========================================================================*/

#define BOARD_REV_UNKNOWN 0
#define BOARD_REV_V2      2
#define BOARD_REV_V3      3

/* Sample the strap and latch the revision.  Requires mcu_adc_init() first (for
   the voltage bucket); without it detection still distinguishes v2 from
   "something newer" using the digital test alone. */
void board_rev_init(void);

/* BOARD_REV_V2, BOARD_REV_V3, or BOARD_REV_UNKNOWN before board_rev_init(). */
int  board_rev_get(void);

/* True on rev3 and later — the gate for every rev3-only feature. */
bool board_rev_is_v3(void);

/* Strap voltage in millivolts as measured at boot (-1 if the ADC was not up).
   Reported in `status` so an unexpected board reads back diagnosably. */
int  board_rev_strap_mv(void);

/* "v2" / "v3" / "unknown" — for logs and JSON. */
const char *board_rev_str(void);

#endif /* BOARD_REV_H */
