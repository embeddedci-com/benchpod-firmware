#ifndef CURRENT_OUT_H
#define CURRENT_OUT_H

#include <stdint.h>

/* ---- 4-20 mA output (J9) -------------------------------------------------------------------
 *
 * J9 is an XTR116 two-wire transmitter (U9). It is LOOP POWERED: an external floating supply
 * drives the loop, and the pod only sets how much current the XTR116 lets through. Its input
 * sums a fixed live zero (AREF through R162) and a span from the DAC buffer (through R82), and
 * the loop current is 100 times that input current:
 *
 *     I = 100 * (AREF / R162 + Vbuf / R82),   Vbuf = code / 65536 * AREF
 *
 * About 4.016 mA at code 0 and 20.078 mA at code 65535, 0.245 uA per code. The output cannot go
 * below the live zero or above the top code: there is no 0 mA and no 21 mA.
 *
 * The transmitter follows the DAC buffer on EVERY analog path. The DAC is shared with the
 * 3.3 V / 5 V / +-12 V outputs, so a voltage output also moves the loop current. `current_out`
 * switches those outputs off (analog path current_out) before it sets the DAC.
 *
 * These are the board's nominal values, the same on every pod: there is no per-pod calibration
 * of the output (it needs a reference meter in the loop).
 *
 * Pure: no HAL, integers in and out (newlib-nano's printf has no %f). Routing and the DAC write
 * are the caller's job (command_handler.c, console.c).
 * ------------------------------------------------------------------------------------------*/

#define CURRENT_OUT_AREF_UV    4096000   /* AREF, also the DAC reference */
#define CURRENT_OUT_ZERO_OHMS  102000    /* R162, AREF -> IIN: the 4 mA live zero */
#define CURRENT_OUT_SPAN_OHMS  25500     /* R82, DAC buffer -> IIN: the 16 mA span */
#define CURRENT_OUT_GAIN       100       /* XTR116: loop current = 100 * input current */

/* The lowest request that is accepted. The live zero sits a little above 4 mA (4016 uA), so a
   request from here up to current_out_min_ua() gives code 0: "4 mA" has to work. */
#define CURRENT_OUT_REQUEST_MIN_UA  4000

/* Loop current in microamps for a 16-bit DAC code. */
long current_out_ua(uint16_t code);

/* What the output can do: the current at code 0 and at code 65535. */
long current_out_min_ua(void);
long current_out_max_ua(void);

/* The DAC code for a requested current. NULL = *code is set (the nearest code), else why the
   request was refused: below CURRENT_OUT_REQUEST_MIN_UA or above current_out_max_ua(). */
const char *current_out_code(long ua, uint16_t *code);

#endif /* CURRENT_OUT_H */
