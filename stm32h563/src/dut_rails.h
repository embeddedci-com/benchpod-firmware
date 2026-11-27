#ifndef DUT_RAILS_H
#define DUT_RAILS_H

#include <stdbool.h>
#include <stdint.h>

/* ============================================================================
 * dut_rails — the two switched DUT supplies on J1 of digital v4: pin 17 (+3V3_DUT) and pin 18
 * (+1V8_DUT).  board_pins.h has the wiring.  Both are on from boot (dut_rails_init), so a DUT
 * powered from J1 works as on older boards; the host can switch each (`target_power` "rail").
 * Independent of the target-power eFuses and of resets: nothing else moves them.
 *
 * Does nothing (and reports "not present") unless board_caps()->dut_rails: on every older board
 * PD3 and PD4 are never touched.  Hw worker and boot only.
 * ==========================================================================*/

typedef enum { DUT_RAIL_3V3 = 0, DUT_RAIL_1V8 = 1 } dut_rail_t;

/* Parse a `target_power` rail name: "j1_3v3" or "j1_1v8".  -1 when unknown. */
int         dut_rail_from_name(const char *name);
const char *dut_rail_name(dut_rail_t r);

/* Drive both enables high (level first, then the mode, so the pins never glitch low). */
void dut_rails_init(void);
bool dut_rails_present(void);
/* 0, or -1 without the hardware or for a bad rail. */
int  dut_rails_set(dut_rail_t r, bool on);
bool dut_rails_get(dut_rail_t r);
/* Switch after delay_ms (0 = now); a new schedule for the same rail replaces a pending one.  Same
   returns as dut_rails_set. */
int  dut_rails_schedule(dut_rail_t r, bool on, uint32_t delay_ms);
/* Apply the schedules whose delay elapsed; the hw worker calls it with target_power_poll(). */
void dut_rails_poll(void);

#endif /* DUT_RAILS_H */
