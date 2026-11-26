#ifndef VREF_OUT_H
#define VREF_OUT_H

#include <stdbool.h>
#include <stdint.h>

#include "bp_json.h"

/* ============================================================================
 * vref_out — J1 pin 21 (VREF_OUT) on digital v4: the rail the pull resistors pull up to, and
 * the reference a DUT can use.  board_pins.h has the wiring.
 *
 * Two sources: the LA bank (1.8 / 3.3 V, the default, also with no firmware running) or the
 * 5 V switch (the USB supply, about 4.4 to 5.1 V) for 5 V I/O.  A sense divider reads one third
 * of the pin, so the pod knows what pin 21 really is:
 *   ok     within the band of its setting
 *   high   more than 200 mV above it: a DUT signal above the I/O voltage lifts pin 21 through
 *          the pull resistors and the expanders' clamps (5 V signals with the pod at 1.8 or
 *          3.3 V).  Select 5 V or remove the signal.
 *   low    below the band: overloaded, or 5 V that did not come up
 *   short  under half the setting: shorted or held down
 * Pin 21 is on whenever the pod is on.
 *
 * Everything here does nothing (and reports "not present") unless board_caps()->vref_out: on
 * every older board PG5, PG6 and PF11 are never touched.
 *
 * Not thread-safe: hw worker and boot only (the MCU ADC is shared, mcu_adc.h).
 * ==========================================================================*/

typedef enum { VREF_SRC_BANK = 0, VREF_SRC_5V } vref_src_t;
typedef enum { VREF_OK = 0, VREF_HIGH, VREF_LOW, VREF_SHORT, VREF_UNKNOWN } vref_state_t;

/* Bands (mV).  5 V comes from the USB supply through a switch: 4.4 to 5.1 V is normal. */
#define VREF_HIGH_MARGIN_MV   200
#define VREF_LOW_MARGIN_MV    300
#define VREF_5V_MIN_MV       4300
#define VREF_5V_MAX_MV       5300
#define VREF_5V_NOMINAL_MV   5000
/* Pin 21 counts as up (the pull expanders may be used) from this level, and as down below the
   lower one.  The expanders' own power-on reset is at 1.0 to 1.4 V of VCCP, and U54's address
   pin follows pin 21, so with pin 21 down it answers at 0x20 with U18. */
#define VREF_UP_MV           1500
#define VREF_DOWN_MV         1200

/* ---- pure (host tests) ---- */

/* The voltage pin 21 should be at: the bank (3300 while it is unset, the board's power-on
   default), or 5000 on the 5 V source. */
int          vref_setpoint_mv(vref_src_t src, int bank_mv);
/* Classify a measured pin 21 voltage against its source and setpoint; mv < 0 = no reading. */
vref_state_t vref_classify(vref_src_t src, int set_mv, int mv);
/* The rail-up decision with hysteresis: was_up is the last decision. */
bool         vref_rail_up(bool was_up, int mv);
const char  *vref_state_name(vref_state_t s);
const char  *vref_src_name(vref_src_t s);

/* "vref_out":{"source":"bank|5v","set_mv":N,"mv":N,"state":"ok|high|low|short|unknown","up":bool}
   (mv -1 without a reading).  Pure over its arguments. */
void vref_emit_fields(bp_emit_t *e, vref_src_t src, int set_mv, int mv, vref_state_t st, bool up);

/* ---- hardware (no-ops unless board_caps()->vref_out) ---- */

/* Pins to their safe state: 5 V switch off (PG6 low), PR1 released (PG5 input, R208 pulls it
   high, pin 21 follows the bank), PF11 analog.  Call once at boot after board_caps_init(). */
void vref_out_init(void);
bool vref_out_present(void);

/* Switch the source.  To 5 V: switch on, wait, then select; back: select the bank, then switch
   off, so pin 21 is never left on an unpowered input.  0, or -1 when not present. */
int  vref_out_set_source(vref_src_t src);
vref_src_t vref_out_source(void);

/* Read pin 21 now (blocking, a few hundred microseconds): mV, or -1 (not present / ADC error).
   Updates the state and the rail-up decision. */
int  vref_out_measure_mv(void);

/* Sample on a schedule (hw worker loop): every VREF_POLL_MS. Logs state changes. */
#define VREF_POLL_MS 50u
void vref_out_poll(uint32_t now_ms);

/* The last reading and state (VREF_UNKNOWN before the first). */
int          vref_out_last_mv(void);
vref_state_t vref_out_state(void);
/* Pin 21 is up (vref_rail_up over the readings).  False when not present. */
bool         vref_out_rail_up(void);
/* vref_emit_fields with the live values (status, la_voltage).  Only call when present. */
void vref_out_emit(bp_emit_t *e);
/* Called when the rail-up decision changes (pull expanders: hold reset while down). */
void vref_out_set_rail_hook(void (*hook)(bool up));

#endif /* VREF_OUT_H */
