#ifndef INA238_H
#define INA238_H

#include <stdint.h>

/* INA238 current/power monitor.  Replaces the RP2350B's INA219 with the same
   shunt resistors (internal 50 mΩ @ 0x40, external 30 mΩ @ 0x44) and better ADC
   resolution.  Current is derived from the measured shunt voltage and the known
   shunt value (no SHUNT_CAL dependency), so a sensor that powers up late still
   reads correctly.  ADCRANGE is forced to 0 (±163.84 mV, 5 µV/LSB).

   Any out-pointer may be NULL.  Returns 0 on success, <0 on I2C error. */
int ina238_read(uint8_t addr, int *bus_mv, int *shunt_uv, int *current_ua);

/* Read the device ID register (0x3F).  INA238 returns 0x2381.  Returns 0 on
   success. */
int ina238_read_id(uint8_t addr, uint16_t *id);

/* ---- single-register access, for the profile sampler (power_profile.c) ----
 * ina238_read() writes CONFIG and reads both channels on every call, which is one
 * transaction too many for a ~1 kHz sampler that must not block the hw worker.  These
 * split it into one I2C transaction each; the caller holds hw_lock around them.
 */

/* ADC_CONFIG (0x01) at power-on: MODE 0xF (continuous shunt+bus), 1052 us per channel,
   AVG 1.  The sampler restores this when it finishes so the other readers are unaffected. */
#define INA238_ADC_CONFIG_RESET  0xFB68u

/* CONFIG (0x00) = 0: ADCRANGE 0, the fine +-163.84 mV / 5 uV-per-LSB shunt range every
   reading here assumes.  Same write ina238_read() makes. */
int ina238_set_adcrange_fine(uint8_t addr);
/* Set the conversion times / averaging (ADC_CONFIG, 0x01). */
int ina238_write_adc_config(uint8_t addr, uint16_t adc_config);
/* One VSHUNT read, already converted to uA through this connector's shunt. */
int ina238_read_shunt_ua(uint8_t addr, int32_t *current_ua);
/* One VBUS read, in mV. */
int ina238_read_bus_mv(uint8_t addr, uint16_t *bus_mv);

#endif /* INA238_H */
