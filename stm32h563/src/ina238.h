#ifndef INA238_H
#define INA238_H

#include <stdbool.h>
#include <stdint.h>

/* INA238 current/power monitor.  Replaces the RP2350B's INA219 with the same
   shunt resistors (internal 50 mΩ @ 0x40, external 30 mΩ @ 0x44) and better ADC
   resolution.  Boards fitted with the pin-compatible INA226 instead (the INA238 went out of
   stock) run the same driver: the chip is detected per address on first use (ina238_chip) and
   every reading comes out in the same units.  The INA226 has half the shunt range (±81.92 mV)
   and a finer step (2.5 µV, 1.25 mV bus); its boards fit smaller shunts (internal 30 mΩ,
   external 20 mΩ: 2.73 A and 4.10 A full scale) and the driver picks them from the chip.  Current is derived from the measured shunt voltage and the known
   shunt value (no SHUNT_CAL dependency), so a sensor that powers up late still
   reads correctly.  ADCRANGE is forced to 0 (±163.84 mV, 5 µV/LSB).

   Any out-pointer may be NULL.  Returns 0 on success, <0 on I2C error. */
int ina238_read(uint8_t addr, int *bus_mv, int *shunt_uv, int *current_ua);

typedef enum {
    INA_CHIP_NONE = 0,     /* not answering (the external rail has no supply), retried next call */
    INA_CHIP_INA238,
    INA_CHIP_INA226,
} ina_chip_t;

/* Which monitor answers at `addr` (INA238 DEVICE_ID 0x238x at 0x3F, INA226 DIE_ID 0x226x at 0xFF).
   Cached once found. */
ina_chip_t  ina238_chip(uint8_t addr);
const char *ina_chip_name(ina_chip_t c);
/* Drop the cache (host tests). */
void        ina238_forget_chips(void);
/* The board has the pod's own current monitor (I2C_ADDR_INA_POD). Probed once, then cached. */
bool        ina_pod_present(void);

/* Read the chip's ID register: INA238 DEVICE_ID (0x3F) = 0x2381, INA226 DIE_ID (0xFF) =
   0x2260.  Returns 0 on success. */
int ina238_read_id(uint8_t addr, uint16_t *id);

/* ---- single-register access, for the profile sampler (power_profile.c) ----
 * ina238_read() writes CONFIG and reads both channels on every call, which is one
 * transaction too many for a ~1 kHz sampler that must not block the hw worker.  These
 * split it into one I2C transaction each; the caller holds hw_lock around them.
 */

/* ADC_CONFIG (0x01) at power-on: MODE 0xF (continuous shunt+bus), 1052 us per channel,
   AVG 1.  The sampler restores this when it finishes so the other readers are unaffected. */
#define INA238_ADC_CONFIG_RESET  0xFB68u
/* INA226 CONFIG at power-on: AVG 1, 1.1 ms bus and shunt conversions, continuous shunt+bus. */
#define INA226_CONFIG_RESET      0x4127u

/* CONFIG (0x00) = 0: ADCRANGE 0, the fine +-163.84 mV / 5 uV-per-LSB shunt range every
   reading here assumes.  Same write ina238_read() makes. */
int ina238_set_adcrange_fine(uint8_t addr);
/* Set the conversion times / averaging: INA238 ADC_CONFIG (0x01), INA226 CONFIG (0x00).
   `adc_config` must be in the detected chip's encoding (pp_rate_select builds it). */
int ina238_write_adc_config(uint8_t addr, uint16_t adc_config);
/* Put back the chip's power-on conversion timing. */
int ina238_restore_adc_config(uint8_t addr);
/* One VSHUNT read, already converted to uA through this connector's shunt. */
int ina238_read_shunt_ua(uint8_t addr, int32_t *current_ua);
/* One VBUS read, in mV. */
int ina238_read_bus_mv(uint8_t addr, uint16_t *bus_mv);

#endif /* INA238_H */
