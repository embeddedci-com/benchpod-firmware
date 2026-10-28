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
 *      to a revision (board_rev_from_strap_mv):
 *        2850..3400 mV  ~3.00 V (R164 1 k over R163 10 k)   v3, analog-v3r1, digital boards
 *        2450..2849 mV  ~2.70 V (R164 2.2 k over R163 10 k) analog v3r2 (DAC8554 quad DAC)
 *      Anything else runs as v3 (today's behaviour for an unknown strap) with a warning.
 *      Step 1 still reads v3r2 HIGH: with the ~40 k internal pull-down it sits near
 *      2.59 V, above VIH (0.7 VDD = 2.31 V).  Firmware from before v3r2 puts 2.70 V in
 *      its 2.6-3.4 V v3 window and runs the board as v3.
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
/* Analog board v3r2: the DAC8551 and its two TMUX1104 output muxes (and their
   TCA9554 at 0x24) are replaced by a DAC8554 quad DAC, one channel per output
   (dac_quad.h).  Everything else is v3, so board_rev_is_v3() is true on it. */
#define BOARD_REV_V3R2    4

/* The strap voltage bands (mV), host-tested through board_rev_from_strap_mv(). */
#define BOARD_REV_V3R2_MIN_MV  2450   /* nominal 2700 - 250 */
#define BOARD_REV_V3_MIN_MV    2850   /* the split between 2700 and 3000 */
#define BOARD_REV_V3_MAX_MV    3400   /* nominal 3000 + 400, as before v3r2 */

/* Sample the strap and latch the revision.  Requires mcu_adc_init() first (for
   the voltage bucket); without it detection still distinguishes v2 from
   "something newer" using the digital test alone. */
void board_rev_init(void);

/* The revision for a measured strap voltage (step 2 above; step 1 already ruled out v2):
   v3r2 only inside its band, v3 for everything else (an unrecognised voltage runs as v3, as it
   did before v3r2, and board_rev_init says so).  Pure. */
static inline int board_rev_from_strap_mv(int mv)
{
    return (mv >= BOARD_REV_V3R2_MIN_MV && mv < BOARD_REV_V3_MIN_MV) ? BOARD_REV_V3R2 : BOARD_REV_V3;
}

/* The strap voltage is one of the two known bands (else board_rev_init warns). */
static inline int board_rev_strap_known(int mv)
{
    return mv >= BOARD_REV_V3R2_MIN_MV && mv <= BOARD_REV_V3_MAX_MV;
}

/* BOARD_REV_V2, BOARD_REV_V3, BOARD_REV_V3R2, or BOARD_REV_UNKNOWN before board_rev_init(). */
int  board_rev_get(void);

/* True on rev3 and later (v3r2 included) — the gate for every rev3-only feature. */
bool board_rev_is_v3(void);

/* Strap voltage in millivolts as measured at boot (-1 if the ADC was not up).
   Reported in `status` so an unexpected board reads back diagnosably. */
int  board_rev_strap_mv(void);

/* "v2" / "v3" / "v3r2" / "unknown" — for logs and JSON. */
const char *board_rev_str(void);

#endif /* BOARD_REV_H */
