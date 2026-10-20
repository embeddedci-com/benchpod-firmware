#ifndef DAC_QUAD_H
#define DAC_QUAD_H

#include <stdbool.h>
#include <stdint.h>

#include "i2c_bus.h"   /* analog_path_t */

/* ---- Quad DAC8554 on the analog v3r2 board (board_rev.h BOARD_REV_V3R2) ----------------------
 *
 * v3r2 replaced the DAC8551 and its two TMUX1104 output muxes (and their TCA9554 at 0x24) with a
 * TI DAC8554: one channel per output, on the same three iCE40 pins, with the same 24-bit frame.
 *
 *   A  TO_3V3   the 0..3.3 V output                   (dac[] index 0)
 *   B  TO_5V    the 0..5 V output, and cal1 through K1 (dac[] index 1)
 *   C  TO_12V   the +-12 V output, and cal2 through U51C/K2 (dac[] index 2)
 *   D  DAC_OUTPUT -> U44B -> XTR116, the 4-20 mA output (J9)
 *
 * The frame's top byte selects the channel (gateware >= DAC_CTRL_MIN_GW, OP_DAC_CTRL): A1:A0 = 00,
 * LD1:LD0 = 01 (load and update that channel), the channel in DB18:17, PD0 = 0.
 *
 * The analog paths keep their v3 meaning: one voltage output at a time.  Where v3 switched the
 * mux, v3r2 points the DAC at the path's channel (the channel every stream and held level goes
 * to) and parks the voltage channel it leaves at its park level: 0 V, or the dac_limits park level
 * on the limited path.  A channel holds its last code until written again, so "off" is a written
 * level, not a disconnected input.  The 4-20 mA output is independent: current_out writes channel
 * D directly and leaves the voltage outputs alone, and a voltage output does not move the loop
 * current any more.
 *
 * The DAC8554 powers up with every channel at code 0, which would be -12 V on the +-12 V output,
 * so that stage's +-14 V supply (U58 P4, ANA14_EN) is off from power-up and holds the output near
 * 0 V.  dac_quad_boot parks A..C as soon as the iCE40 can drive the DAC, and only then switches
 * the +-14 V supply on (it stays on).
 *
 * This file is the pure half (host-tested); signal_engine.c sends the frames.
 * ----------------------------------------------------------------------------------------------*/

/* The board has the DAC8554 (set once at boot from board_rev; host tests set it directly).
   False until then: a v2/v3 board never sees a DAC_CTRL frame. */
void dac_quad_set_present(bool present);
bool dac_quad_present(void);

/* The one gate: the DAC8554 is assumed only on an analog board whose strap reads v3r2.  v2, v3,
   an unknown or unrecognised strap (which runs as v3) and the digital boards keep the DAC8551
   behaviour, byte for byte: no DAC_CTRL frame is ever sent to them. */
bool dac_quad_for_board(int board_rev, bool has_analog);

#define DAC_QUAD_A     0
#define DAC_QUAD_B     1
#define DAC_QUAD_C     2
#define DAC_QUAD_D     3
#define DAC_QUAD_NONE  (-1)            /* no channel: streams and held levels go nowhere */
#define DAC_QUAD_VOLTAGE_MASK 0x07u    /* A..C, the channels a park applies to */

/* The frame control byte for a channel: 0x10 | ch << 1.  DAC_QUAD_NONE gives 0x00, which on a
   DAC8554 only loads channel A's input buffer (LD1:LD0 = 00) and moves no output. */
uint8_t dac_quad_ctrl(int ch);

/* "A".."D", or "none". */
const char *dac_quad_name(int ch);

/* What applying an analog path does to the DAC. */
typedef struct {
    bool    touch;      /* false: the path leaves the DAC alone (adc_ext, current_in) */
    int     ch;         /* the channel the DAC points at from now on */
    bool    stop;       /* stop the running stream first: it plays to a channel being left */
    uint8_t park;       /* voltage channels to park, bit n = channel n */
} dac_quad_plan_t;

/* The plan for `path`, given the channel the DAC points at now and the voltage channels that may
   be away from their park level (`live`, bit n = channel n). */
dac_quad_plan_t dac_quad_plan(analog_path_t path, int cur_ch, uint8_t live);

/* The park level of a voltage channel (A..C): the dac_limits park code when that output is the
   limited one, else the code nearest 0 V on its calibrated line (cal_data.h). */
uint16_t dac_quad_park_code(int ch);

#endif /* DAC_QUAD_H */
