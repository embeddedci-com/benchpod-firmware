/*
 * module_link.h — digital v3r3's link to the plug-in analog module, and its two power switches.
 *
 * Digital v3r3 is the digital core board of a two-board BenchPod: the analog section is a module
 * on a 24-pin 1.0 mm FFC (CN1). The link (bsp/board_pins.h): the module bus on I2C2 (PF0/PF1),
 * MOD_PRESENT# (PD9, pulled up, the module grounds it), MOD_PWR_EN (PD10, U59 for +5V_MOD) and
 * BRIDGE_PWR_EN (PD14, U60 for the always-on USB-A bridge port). Both enables have 10k pull-ups,
 * so both rails are on from power-up, before any firmware runs.
 *
 * Recognising the board: the revision strap is the v3 one, so the pins tell. PD10 and PD14 are
 * read with the MCU's internal pull-down (~40k) on: the 10k pull-ups win (about 2.6 V) on digital
 * v3r3, while on every earlier board those pins go to the H3 user header and read low. The rails
 * stay on during the test. Nothing on PD8..PD15 or PF0/PF1 is ever driven on another board.
 *
 * The enables are left as inputs at boot (their pull-ups keep the rails on); module_power_set()
 * and bridge_power_set() drive them from then on. Turning the module off stops the DAC and drops
 * the +-14 V supply first; turning it back on re-initialises U58 and parks the DAC again
 * (dac_quad_boot), as at boot.
 *
 * The module's 24C32 (module_eeprom.h) is read at boot when a module is present.
 */
#ifndef MODULE_LINK_H
#define MODULE_LINK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "module_eeprom.h"

/* Recognise digital v3r3, latch MOD_PRESENT#, and bring up I2C2 there. Call once after
   i2c_bus_init(), before board_variant_init(). */
void module_link_init(void);

/* The board is digital v3r3 (the module-link pins were seen at boot). */
bool module_link_board(void);

/* MOD_PRESENT# read low at boot (false off digital v3r3). */
bool module_present(void);
/* The live level of MOD_PRESENT#: a module is plugged in now. */
bool module_present_now(void);

/* The rails: true when enabled (read back from the pin). False off digital v3r3. */
bool module_power_on(void);
bool bridge_power_on(void);

/* Switch a rail. 0, or -1 off digital v3r3; module_power_set returns -2 when the module came
   back on but U58 did not answer. */
int  module_power_set(bool on);
int  bridge_power_set(bool on);

/* Raw EEPROM access on the module bus. 0 / -1 (no module bus, NACK, or out of range).
   module_eeprom_write splits at page boundaries and waits for each write cycle (<= 5 ms). */
int  module_eeprom_read(uint16_t addr, uint8_t *buf, size_t len);
int  module_eeprom_write(uint16_t addr, const uint8_t *buf, size_t len);

/* The header read at boot (or after the last write): a MOD_EE_* result, *id filled on
   MOD_EE_OK. MOD_EE_NO_CHIP when the EEPROM did not answer. */
#define MOD_EE_NO_CHIP (-10)
int  module_id(module_id_t *id);
/* Read the header again. Returns the new result. */
int  module_id_reload(void);
/* The calibration blob's CRC matched at the last read (false with no blob). */
bool module_cal_valid(void);

/* Write the ID fields (type, revision, serial), keeping the calibration fields. 0 / -1. */
int  module_id_write(uint8_t type, uint8_t rev_major, uint8_t rev_minor, const char *serial);
/* Record `len` bytes at MOD_EE_CAL_ADDR as the calibration blob: reads them back, stores their
   length and CRC in the header. 0 / -1. A blank or invalid header gets an empty ID. */
int  module_cal_commit(uint16_t len);

#endif /* MODULE_LINK_H */
