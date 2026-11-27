#ifndef DUT_SUPPLY_H
#define DUT_SUPPLY_H

#include <stdbool.h>
#include <stdint.h>

/* ============================================================================
 * dut_supply — the two switched DUT supplies on J1 of digital v4: pin 17 (+3V3_DUT) and pin 18
 * (+1V8_DUT).  board_pins.h has the wiring.  Both are on from boot (dut_supply_init), so a DUT
 * powered from J1 works as on older boards; the host can switch each (`target_power` supply "3v3" or "1v8", supply.h).
 * Independent of the target-power eFuses and of resets: nothing else moves them.
 *
 * Does nothing (and reports "not present") unless board_caps()->supply_3v3_1v8: on every older board
 * PD3 and PD4 are never touched.  Hw worker and boot only.
 * ==========================================================================*/

typedef enum { DUT_SUPPLY_3V3 = 0, DUT_SUPPLY_1V8 = 1 } dut_supply_t;

/* Drive both enables high (level first, then the mode, so the pins never glitch low). */
void dut_supply_init(void);
bool dut_supply_present(void);
/* 0, or -1 without the hardware or for a bad rail. */
int  dut_supply_set(dut_supply_t r, bool on);
bool dut_supply_get(dut_supply_t r);
/* Switch after delay_ms (0 = now); a new schedule for the same rail replaces a pending one.  Same
   returns as dut_supply_set. */
int  dut_supply_schedule(dut_supply_t r, bool on, uint32_t delay_ms);
/* Apply the schedules whose delay elapsed; the hw worker calls it with target_power_poll(). */
void dut_supply_poll(void);

#endif /* DUT_SUPPLY_H */
