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
 *        ~3.27 V (R164 1 k over R163 100 k): digital v3r4, told by its ratio to VREF+, step 3
 *      Anything else runs as v3 (today's behaviour for an unknown strap) with a warning.
 *   3. Only when step 2 reads the v3r4 band: the second strap on PF12 (ADC1_INP6), 10 k / 10 k on
 *      digital v3r4.  The first strap is at the top of its range there, so later revisions change
 *      this divider.  v3r4 needs BOTH straps: PA3 between 95.0 % and 99.5 % of VREF+ (v3 is
 *      90.9 %, v3r4 99.0 % from the resistor values, a pin stuck at the rail 100 %), AND PF12 between 40 % and 60 % of
 *      VREF+.  The split is a ratio, not a voltage: the ADC measures the strap against VREF+,
 *      which is the same +3V3 rail that feeds the divider, so the ratio needs neither the rail
 *      nor the VREFINT reading (a 3.13 V split in mV fails a v3r4 board on a 3.2 V rail read
 *      2 % low; test_board_caps.c sweeps it).  Anything less certain runs as v3, exactly as before
 *      v3r4: on a v3r4 board that is safe (pin 21 follows the bank on its pull-up and pull-down
 *      resistors, the pull expanders stay in reset, old TCA9554 writes at 0x20 never reach the
 *      new expanders' direction registers).  PF12 is never read on any other board.
 *      Step 1 still reads v3r2 HIGH: with the ~40 k internal pull-down it sits near
 *      2.59 V, above VIH (0.7 VDD = 2.31 V).  Firmware from before v3r2 puts 2.70 V in
 *      its 2.6-3.4 V v3 window and runs the board as v3.
 *
 * The revision decides three things that are NOT compatible between v2 and v3:
 *   - the TPS2116 LA-bank mux polarity, and whether 1.8 V is reachable at all
 *     (on v2, MODE is grounded, so the "1.8 V" setting shuts the mux down —
 *     see la_io.c la_vccio_set_mv);
 *   - whether the dedicated NRST-control pin exists (nrst_ctrl.c);
 *   - whether USB-C CC monitoring exists (usb_cc.c).
 *
 * None of that is gated on the revision outside this file: board_rev_init() latches
 * board_caps.h, the one place that maps a revision to what the board has, and every feature
 * reads its field there.
 *
 * Sampled once at boot; every accessor is a plain RAM read afterwards, so they
 * are safe to call from any task.
 * ==========================================================================*/

#define BOARD_REV_UNKNOWN 0
#define BOARD_REV_V2      2
#define BOARD_REV_V3      3
/* Analog board v3r2: the DAC8551 and its two TMUX1104 output muxes (and their
   TCA9554 at 0x24) are replaced by a DAC8554 quad DAC, one channel per output
   (dac_quad.h).  Everything else is v3, so it has every v3 capability (board_caps.h). */
#define BOARD_REV_V3R2    4
/* Digital v3r4: J1 pin 21 is VREF_OUT (switched, sensed), 5 V I/O, two pull expanders with a
   direction and a strength per channel (board_caps.h has the list).  It has every v3
   capability, and everything that is v3 on a digital board stays as it is. */
#define BOARD_REV_V3R4    5

/* The strap voltage bands (mV), host-tested through board_rev_from_strap_mv(). */
#define BOARD_REV_V3R2_MIN_MV  2450   /* nominal 2700 - 250 */
#define BOARD_REV_V3_MIN_MV    2850   /* the split between 2700 and 3000 */
#define BOARD_REV_V3_MAX_MV    3400   /* nominal 3000 + 400, as before v3r2 */
/* Digital v3r4: PA3 at 3267 mV nominal is inside the v3 band above; it is v3r4 only when its
   ratio to VREF+ and the second strap agree. */
#define BOARD_REV_V3R4_MIN_PERMILLE   950   /* PA3 / VREF+: v3 909, v3r4 990 */
#define BOARD_REV_V3R4_MAX_PERMILLE   995   /* a pin stuck at the rail reads 1000 */
#define BOARD_REV2_V3R4_MIN_PERMILLE  400   /* PF12 / VREF+: v3r4 500 */
#define BOARD_REV2_V3R4_MAX_PERMILLE  600

/* Sample the strap, latch the revision and its capabilities (board_caps_init).  Requires mcu_adc_init() first (for
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

/* A strap reading as a fraction of VREF+ in 1/1000 (the ADC's own ratio), -1 when vref_mv <= 0. */
static inline int board_rev_permille(int mv, int vref_mv)
{
    if (vref_mv <= 0 || mv < 0) return -1;
    return (int)(((long)mv * 1000L + vref_mv / 2) / vref_mv);
}

/* Step 2 says "digital v3r4 or later": the second strap is worth reading.  Pure. */
static inline bool board_rev_strap_says_v3r4(int mv, int vref_mv)
{
    int pm = board_rev_permille(mv, vref_mv);
    return pm >= BOARD_REV_V3R4_MIN_PERMILLE && pm <= BOARD_REV_V3R4_MAX_PERMILLE;
}

/* The second strap (PF12) of a board whose first strap says v3r4: BOARD_REV_V3R4 inside its band,
   else BOARD_REV_V3 (not recognised: run as before v3r4).  rev2_mv < 0 = not read.  Pure. */
static inline int board_rev_from_rev2_mv(int rev2_mv, int vref_mv)
{
    int pm = board_rev_permille(rev2_mv, vref_mv);
    return (pm >= BOARD_REV2_V3R4_MIN_PERMILLE && pm <= BOARD_REV2_V3R4_MAX_PERMILLE)
           ? BOARD_REV_V3R4 : BOARD_REV_V3;
}

/* The whole decision after step 1 (host tests): PA3 in mV, VREF+ in mV, and PF12 in mV (-1 when
   it was not read).  Pure. */
static inline int board_rev_decide(int strap_mv, int vref_mv, int rev2_mv)
{
    if (board_rev_strap_says_v3r4(strap_mv, vref_mv))
        return board_rev_from_rev2_mv(rev2_mv, vref_mv);
    return board_rev_from_strap_mv(strap_mv);
}

/* BOARD_REV_V2, BOARD_REV_V3, BOARD_REV_V3R2, BOARD_REV_V3R4, or BOARD_REV_UNKNOWN before
   board_rev_init().  For board_caps and reports only: features ask board_caps(). */
int  board_rev_get(void);

/* Strap voltage in millivolts as measured at boot (-1 if the ADC was not up).
   Reported in `status` so an unexpected board reads back diagnosably. */
int  board_rev_strap_mv(void);

/* The second strap (PF12) in millivolts, -1 when it was not read (every board before v3r4). */
int  board_rev2_strap_mv(void);

/* "v2" / "v3" / "v3r2" / "v3r4" / "unknown" — for logs and JSON. */
const char *board_rev_str(void);

#endif /* BOARD_REV_H */
