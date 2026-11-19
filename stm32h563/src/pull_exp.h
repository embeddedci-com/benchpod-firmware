#ifndef PULL_EXP_H
#define PULL_EXP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "la_pull.h"   /* pull_str_t; this is its digital v3r4 backend */

/* ============================================================================
 * pull_exp — the LA pull resistors of digital v3r4: two TCA6416A expanders on the main I2C bus.
 *
 *   Each channel has a 10k and a 2.2k, both on one expander: U54 at 0x21 serves LA1-LA7 and LA9
 *   (all 16 ports), U18 at 0x20 serves LA8 and LA10-LA14 (P00-P13; P14-P17 stay inputs).  The
 *   port of each resistor follows the PCB layout (pull_exp_map, from the digital v3r4 netlist,
 *   la-pulls.kicad_sch).  A port driven high pulls the line up to J1 pin 21 (vref_out.h), driven
 *   low pulls it down, an input is off.  Per channel: 10k, 2.2k or both (about 1.8k), up or down.
 *
 * Rules:
 *   - Both resistors of one channel always point the same way; a change turns both off first
 *     (configuration), then writes the levels, then turns them on (pull_exp_plan).
 *   - PULL_RST (PG7, a pull-down on the board) holds both in reset (every port an input, all
 *     pulls off) until pin 21 is up, and again whenever it goes down: U54's address pin follows
 *     pin 21, so with pin 21 down it answers at 0x20 together with U18, and the expanders' own
 *     power-on reset follows pin 21 too.  When pin 21 comes back both are rewritten from the
 *     state the host asked for.
 *   - Pull-ups are suspended while the pod has switched the DUT's power off (a rail went from on
 *     to off), so pin 21 does not feed an unpowered DUT through them; they come back when a rail
 *     turns on, or when the host sets a pull.  A DUT the pod never powers is not affected.
 *   - Only on board_caps()->pull_expanders (la_pull_init picks this backend, la_pull_backend_exp).
 *     Older boards keep the TCA9554 backend at 0x20 (pull_tca9554.c) and never see PG7 or 0x21.
 * ==========================================================================*/

#define PULL_EXP_ADDR_U18   0x20u   /* U18, ADDR = GND */
#define PULL_EXP_ADDR_U54   0x21u   /* U54, ADDR = pin 21 */
#define PULL_EXP_CHANNELS   14u

/* Where one channel's resistors are: the expander (PULL_EXP_U18 / PULL_EXP_U54) and the port bit
   (0-7 = P00-P07, 8-15 = P10-P17) of its 10k and of its 2.2k. */
enum { PULL_EXP_U18 = 0, PULL_EXP_U54 = 1 };
typedef struct { uint8_t chip, bit10k, bit2k2; } pull_exp_pin_t;
extern const pull_exp_pin_t pull_exp_map[PULL_EXP_CHANNELS + 1];   /* index LA 1..14 */

/* TCA6416A command bytes: a write auto-increments to the pair's second register. */
#define TCA6416_REG_OUTPUT0  0x02u
#define TCA6416_REG_CONFIG0  0x06u

/* Strength (pull_str_t) and its names: la_pull.h. */
/* Direction: same values as la_pull_dir_t (la_pull.h). */
typedef enum { PULL_EXP_OFF = 0, PULL_EXP_UP = 1, PULL_EXP_DOWN = 2 } pull_exp_dir_t;

typedef struct { uint16_t out, cfg; } tca6416_regs_t;   /* cfg bit 1 = input (off) */

/* ---- pure (host tests) ---- */

/* The register values for a state: dir[la] / str[la], la 1..14 (index 0 unused).  suspend_up:
   leave every pull-up off (the pod switched the DUT's power off; pull-downs stay). */
void pull_exp_regs(const uint8_t dir[PULL_EXP_CHANNELS + 1], const uint8_t str[PULL_EXP_CHANNELS + 1],
                   bool suspend_up, tca6416_regs_t *r18, tca6416_regs_t *r54);

/* The registers right after reset or power-on: every port an input, outputs latched high. */
tca6416_regs_t pull_exp_reset_regs(void);

/* One register write: a 16-bit pair, low byte (port 0) first. */
typedef struct { uint8_t addr, reg; uint16_t val; } pull_exp_write_t;
#define PULL_EXP_PLAN_MAX 6u

/* The writes that take both expanders from `from` to `to` without any channel pulling two ways:
   (1) configuration: both resistors of every channel that changes go off, (2) the output levels,
   (3) the final configuration.  Writes that change nothing are left out.  Returns the count. */
size_t pull_exp_plan(tca6416_regs_t from18, tca6416_regs_t from54,
                     tca6416_regs_t to18, tca6416_regs_t to54, pull_exp_write_t out[PULL_EXP_PLAN_MAX]);

/* What one channel does with these registers: its direction (PULL_EXP_OFF when neither resistor
   is engaged, or when the two disagree) and strength. */
pull_exp_dir_t pull_exp_channel(tca6416_regs_t r18, tca6416_regs_t r54, unsigned la, pull_str_t *str);

/* Why the pulls the host asked for are not all on the wire, or NULL when they are:
   "pin21_down" (pin 21 is down: both expanders held in reset, every pull off), "fault" (a write
   failed: held in reset until the next pull turned on, pull_exp_recover), "dut_off" (pull-ups suspended after the pod
   switched the DUT off; only when a pull-up is asked for).  live: out of reset and written. */
const char *pull_exp_hold_name(bool live, bool rail_up, bool suspend_up, bool any_up_asked);

/* ---- hardware (no-ops unless board_caps()->pull_expanders) ---- */

/* PG7 low (both in reset), then released once pin 21 is up.  After vref_out_init(). */
void pull_exp_init(void);
/* The board has the expanders (board_caps()->pull_expanders, latched by pull_exp_init()).
   Test hook: only the host tests call it (test_pull_exp_state.c). */
bool pull_exp_present(void);

/* Set one channel.  0, -1 (not present, bad channel, I2C failure), PULL_EXP_ERR_RAIL (pin 21 is
   down: the expanders are held in reset; nothing changed). */
#define PULL_EXP_ERR_RAIL (-3)
int  pull_exp_set(unsigned la, pull_exp_dir_t dir, pull_str_t str);
/* Every channel off (always allowed; also while held). */
void pull_exp_all_off(void);

/* Follow the pod's target power (hw worker loop; RAM reads only). */
void pull_exp_poll(void);

/* The state the host asked for (also while held in reset or suspended). */
pull_exp_dir_t pull_exp_dir(unsigned la);
pull_str_t     pull_exp_str(unsigned la);
/* Both expanders are out of reset and written. */
bool pull_exp_live(void);
/* pull_exp_live, and after a failed write ("fault": pin 21 up, both held in reset) first try once
   more to bring them out of reset and rewrite every asked-for pull.  false while pin 21 is down
   (they come back by themselves when it returns) or when the retry fails too. */
bool pull_exp_recover(void);
/* What one channel really does now, from what the chips hold: PULL_EXP_OFF while held in reset or
   suspended.  *str (may be NULL) the strength engaged. */
pull_exp_dir_t pull_exp_live_dir(unsigned la, pull_str_t *str);
/* pull_exp_hold_name for the state now (NULL when everything asked for is on the wire). */
const char *pull_exp_held(void);

#endif /* PULL_EXP_H */
