#ifndef TCA9554_H
#define TCA9554_H

#include <stdint.h>
#include <stdbool.h>

/* TCA9554 8-bit I2C GPIO expander (replaces the RP2350B's PCA9555).
 *
 * Registers: 0x00 input, 0x01 output, 0x02 polarity, 0x03 config (1=input).
 * These helpers are stateless: pin direction/level changes read-modify-write the
 * device's own output/config registers, so multiple expanders on the bus are
 * driven purely by their 7-bit address.
 *
 * Returns 0 on success, <0 on I2C error.  pin is 0..7. */

int  tca9554_read_reg(uint8_t addr, uint8_t reg, uint8_t *val);
int  tca9554_write_reg(uint8_t addr, uint8_t reg, uint8_t val);

int  tca9554_set_dir(uint8_t addr, uint8_t pin, bool is_output);
int  tca9554_write_pin(uint8_t addr, uint8_t pin, bool value);
int  tca9554_read_pin(uint8_t addr, uint8_t pin, bool *value);
int  tca9554_read_inputs(uint8_t addr, uint8_t *port);

#endif /* TCA9554_H */
