#ifndef DAC_QUAD_H
#define DAC_QUAD_H

#include <stdbool.h>
#include <stdint.h>

#include "i2c_bus.h"   /* analog_path_t */

/* ---- Quad DAC8554 on the analog v3r2 board (board_caps()->dac_quad) ----------------------
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

/* The one gate: the DAC8554 is assumed only on an analog board whose strap reads v3r2
   (`board_quad` = board_caps()->dac_quad), or on the
   analog module of digital v3r3 (`module`, board_variant.h), which has the same DAC.  v2, v3, an
   unknown or unrecognised strap (which runs as v3) and the digital boards keep the DAC8551
   behaviour, byte for byte: no DAC_CTRL frame is ever sent to them. */
bool dac_quad_for_board(bool board_quad, bool has_analog, bool module);

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

/* ---- The ADC low range and the +-12 V output (channel C) -------------------------------------
 *
 * The ADC low range (i2c_bus.h adc_range_t, +-5.46 V full scale) exists only on the analog module
 * v3r4 and later, which always has this DAC.  The +-12 V output reaches the ADC through the front
 * SMA when it is cabled to the ADC input (the `calibrate adc:ext` setup), so while the low range
 * is on, channel C stays within +-DAC_QUAD_LOW_RANGE_MAX_MV:
 *   - a held level on C past it is refused (dac_out, sensor emulation, SCPI, anything that holds);
 *   - a stream on C (generate, replay, load/load_bin, dac_set, measure, the control loop) is
 *     refused outright: its samples are not checked one by one;
 *   - switching to the low range is refused while C holds a level past it, or a level the pod
 *     does not know (a stream played on C since its last held level).
 * The normal range is never limited: C keeps its full +-12 V there.
 *
 * Pure: the checks take the state as arguments.  signal_engine.c and i2c_bus.c call them on the
 * DAC writes and the range switch (the backstop nothing goes around); command_handler.c calls
 * dac_quad_low_check_command first, for a refusal that names the command.  Each returns NULL
 * when allowed, else the refusal text, which dac_quad_low_last() also returns afterwards.
 * ----------------------------------------------------------------------------------------------*/

#define DAC_QUAD_LOW_RANGE_MAX_MV 5000
/* What a held level of exactly +-5 V may be off by: the 16-bit code rounds (about 1.5 mV a step
   on the +-12 V line). */
#define DAC_QUAD_LOW_RANGE_TOL_MV 5

/* Holding `code` on channel `ch` (with the ADC in the low range when low). */
const char *dac_quad_low_check_hold(bool low, int ch, uint16_t code);
/* Starting a stream on channel `ch`. */
const char *dac_quad_low_check_stream(bool low, int ch);
/* Switching to the low range, with channel C at `c_code` (c_known false: a stream played on C). */
const char *dac_quad_low_check_enter(bool c_known, uint16_t c_code);
/* One JSON command (cmd = its "cmd" field), with the DAC pointing at `cur_ch`.  Covers dac_out
   on 12v and the stream commands; the range switch is checked when it happens. */
const char *dac_quad_low_check_command(const char *cmd, const char *json, bool low, int cur_ch);
/* The text of the last refusal above ("" before any). */
const char *dac_quad_low_last(void);

#endif /* DAC_QUAD_H */
