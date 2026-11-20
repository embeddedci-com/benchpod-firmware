#ifndef DAC_LINK_H
#define DAC_LINK_H

/* ============================================================================
 * dac_link — the DAC's two lines on the module flat cable (pure, host-tested).
 *
 * On a two-board pod the DAC's serial clock and data cross the 24-pin flat cable (CN1) from
 * the digital board to the analog module.  Digital v3r4 and analog module 3.5 swapped them, so
 * the clock sits next to a ground pin:
 *
 *                       CN1 pin 7   CN1 pin 8
 *   older boards        DAC data    DAC clock    digital v3r3, analog module 3.4 and earlier
 *   newer boards        DAC clock   DAC data     digital v3r4+, analog module 3.5+
 *
 * The gateware is the same for both: only the cable pins moved, on both boards.  A pair that
 * mixes the two clocks DAC frames on the data line, so the DAC latches garbage and its outputs
 * can jump anywhere.  Both lines are outputs of the digital board, so nothing is damaged; the
 * firmware refuses the DAC on such a pair (BP0035) and sends no DAC traffic at all.
 *
 * The module's revision is the one in its EEPROM header (module_eeprom.h); a blank, unreadable
 * or bad header counts as an older module.  Only two-board pods with a module present are
 * checked: a single-board pod has no flat cable.
 * ==========================================================================*/

#include <stdbool.h>
#include <stdint.h>

/* The first analog module revision with the DAC clock on CN1 pin 7. */
#define DAC_LINK_SWAP_REV_MAJOR 3u
#define DAC_LINK_SWAP_REV_MINOR 5u

/* The two boards put the DAC clock on different CN1 pins.  link_board: a two-board pod
   (module_link_board); present: its module was seen at boot; digital_swap: the digital board
   has the clock on pin 7 (board_caps()->cn1_dac_swap); module_swap: so does the module
   (module_ee_has_cn1_dac_swap). */
bool dac_link_mismatch(bool link_board, bool present, bool digital_swap, bool module_swap);

/* The command drives the DAC (or would, with these arguments): refused on a mismatched pair
   before its handler runs.  Commands that reach the DAC only in some modes (a sensor, the EMI
   build) are stopped by the opcode block below instead. */
bool dac_link_command_drives_dac(const char *cmd, const char *json);

/* The iCE40 command sends frames to the DAC (a DAC start, the control loop, a single frame). */
bool dac_link_opcode_drives_dac(uint8_t opcode);

#endif /* DAC_LINK_H */
