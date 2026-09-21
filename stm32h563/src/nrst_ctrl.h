#ifndef NRST_CTRL_H
#define NRST_CTRL_H

#include <stdbool.h>
#include <stdint.h>

/* ============================================================================
 * nrst_ctrl — the dedicated target-reset line (rev3+).
 *
 * Before rev3 the pod had no reset pin of its own: nRESET had to be borrowed
 * from a logic-analyzer channel and driven by the FPGA's SWD engine, which
 * meant giving up an LA channel and telling every client which one.  rev3 wires
 * /NRST_CONTROL to J1 pin 22 and back to PF4 through a 330 R series resistor,
 * so the reset line is always in the same place.
 *
 * The DUT side is pulled up by R7 (10 k) to LA_VCCIO — the LA bank rail, i.e.
 * 1.8 V or 3.3 V depending on what the host selected with `la_voltage`.  That
 * makes the drive discipline non-negotiable:
 *
 *   - PF4 is configured OPEN-DRAIN.  Released, it is genuinely Hi-Z and R7 (or
 *     the DUT's own reset circuit) sets the level.
 *   - Asserted, it pulls the net to LA_VCCIO * 330/10330 ~= 105 mV.
 *   - It NEVER drives high.  A 3.3 V push-pull high would push ~4.5 mA into a
 *     1.8 V DUT's reset net through the 330 R and lift it above its own rail.
 *
 * Before nrst_ctrl_init() runs, PF4 is still in its reset state (analog, Hi-Z),
 * so there is no window where the pod holds a DUT in reset at power-up.
 *
 * On v2 the pad is not connected; every entry point here is a safe no-op and
 * nrst_ctrl_supported() returns false.
 * ==========================================================================*/

/* Configure PF4 open-drain and released.  No-op on v2.  Call after
   board_rev_init(). */
void nrst_ctrl_init(void);

/* True on rev3 and later — i.e. whether the pod can drive the target's reset. */
bool nrst_ctrl_supported(void);

/* Assert (drive low) or release (Hi-Z) the target's reset line. */
void nrst_ctrl_assert(bool asserted);

/* Last state requested through nrst_ctrl_assert(). */
bool nrst_ctrl_is_asserted(void);

/* Hold reset low for hold_ms (clamped to 1..1000) then release. */
void nrst_ctrl_pulse(uint32_t hold_ms);

#endif /* NRST_CTRL_H */
