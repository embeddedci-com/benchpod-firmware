#ifndef PULL_EXP_H
#define PULL_EXP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ============================================================================
 * pull_exp — the LA pull resistors of digital v3r4: two TCA6416A expanders on the main I2C bus.
 *
 *   U18 at 0x20 drives a 2.2k per channel, U54 at 0x21 a 10k per channel.  P00-P07 = LA1-LA8,
 *   P10-P15 = LA9-LA14, so LA n is bit n-1 of the 16-bit port pair.  P16, P17 stay inputs.
 *   A port driven high pulls the line up to J1 pin 21 (vref_out.h), driven low pulls it down,
 *   an input is off.  Per channel: 10k, 2.2k or both (about 1.8k), up or down.
 *
 * Rules:
 *   - Both expanders of one channel always point the same way; a change turns the channel off on
 *     both first (configuration), then writes the levels, then turns it on (pull_exp_plan).
 *   - PULL_RST (PG7, a pull-down on the board) holds both in reset (every port an input, all
 *     pulls off) until pin 21 is up, and again whenever it goes down: U54's address pin follows
 *     pin 21, so with pin 21 down it answers at 0x20 together with U18, and the expanders' own
 *     power-on reset follows pin 21 too.  When pin 21 comes back both are rewritten from the
 *     state the host asked for.
 *   - Pull-ups are suspended while the pod has switched the DUT's power off (a rail went from on
 *     to off), so pin 21 does not feed an unpowered DUT through them; they come back when a rail
 *     turns on, or when the host sets a pull.  A DUT the pod never powers is not affected.
 *   - Only on board_caps()->pull_expanders.  Older boards keep the TCA9554 driver at 0x20
 *     (i2c_bus.c) and never see PG7 or 0x21.
 * ==========================================================================*/

#define PULL_EXP_ADDR_2K2   0x20u   /* U18, ADDR = GND */
#define PULL_EXP_ADDR_10K   0x21u   /* U54, ADDR = pin 21 */
#define PULL_EXP_CHANNELS   14u

/* TCA6416A command bytes: a write auto-increments to the pair's second register. */
#define TCA6416_REG_INPUT0   0x00u
#define TCA6416_REG_OUTPUT0  0x02u
#define TCA6416_REG_POL0     0x04u
#define TCA6416_REG_CONFIG0  0x06u

/* Strength: which resistors a channel engages. */
typedef enum { PULL_STR_NONE = 0, PULL_STR_10K = 1, PULL_STR_2K2 = 2, PULL_STR_BOTH = 3 } pull_str_t;
/* Direction: same values as la_pull_dir_t (la_pins.h). */
typedef enum { PULL_EXP_OFF = 0, PULL_EXP_UP = 1, PULL_EXP_DOWN = 2 } pull_exp_dir_t;

typedef struct { uint16_t out, cfg; } tca6416_regs_t;   /* cfg bit 1 = input (off) */

/* ---- pure (host tests) ---- */

/* The register values for a state: dir[la] / str[la], la 1..14 (index 0 unused).  suspend_up:
   leave every pull-up off (the pod switched the DUT's power off; pull-downs stay). */
void pull_exp_regs(const uint8_t dir[PULL_EXP_CHANNELS + 1], const uint8_t str[PULL_EXP_CHANNELS + 1],
                   bool suspend_up, tca6416_regs_t *r2k2, tca6416_regs_t *r10k);

/* The registers right after reset or power-on: every port an input, outputs latched high. */
tca6416_regs_t pull_exp_reset_regs(void);

/* One register write: a 16-bit pair, low byte (port 0) first. */
typedef struct { uint8_t addr, reg; uint16_t val; } pull_exp_write_t;
#define PULL_EXP_PLAN_MAX 6u

/* The writes that take both expanders from `from` to `to` without any channel pulling two ways:
   (1) configuration: every channel that changes goes off on both, (2) the output levels,
   (3) the final configuration.  Writes that change nothing are left out.  Returns the count. */
size_t pull_exp_plan(tca6416_regs_t from2k2, tca6416_regs_t from10k,
                     tca6416_regs_t to2k2, tca6416_regs_t to10k, pull_exp_write_t out[PULL_EXP_PLAN_MAX]);

/* What one channel does with these registers: its direction (PULL_EXP_OFF when neither engages
   it, or when the two disagree) and strength. */
pull_exp_dir_t pull_exp_channel(tca6416_regs_t r2k2, tca6416_regs_t r10k, unsigned la, pull_str_t *str);

/* "10k", "2.2k", "1.8k" (both), or NULL. */
const char *pull_str_ohms(pull_str_t s);
/* "10k" / "2.2k" / "both" -> strength; false otherwise. */
bool        pull_str_parse(const char *s, pull_str_t *out);
const char *pull_str_name(pull_str_t s);   /* "10k", "2.2k", "both", "none" */

/* ---- hardware (no-ops unless board_caps()->pull_expanders) ---- */

/* PG7 low (both in reset), then released once pin 21 is up.  After vref_out_init(). */
void pull_exp_init(void);
bool pull_exp_present(void);

/* Set one channel.  0, -1 (not present, bad channel, I2C failure), PULL_EXP_ERR_RAIL (pin 21 is
   down: the expanders are held in reset; nothing changed). */
#define PULL_EXP_ERR_RAIL (-3)
int  pull_exp_set(unsigned la, pull_exp_dir_t dir, pull_str_t str);
/* Every channel off (always allowed; also while held). */
void pull_exp_all_off(void);

/* Follow the pod's target power (hw worker loop; RAM reads only). */
void pull_exp_poll(void);
/* Pull-ups are suspended (the pod switched the DUT off). */
bool pull_exp_ups_suspended(void);

/* The state the host asked for (also while held in reset or suspended). */
pull_exp_dir_t pull_exp_dir(unsigned la);
pull_str_t     pull_exp_str(unsigned la);
/* Both expanders are out of reset and written. */
bool pull_exp_live(void);

#endif /* PULL_EXP_H */
