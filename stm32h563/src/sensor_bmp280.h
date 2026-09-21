/* ============================================================================
 * sensor_bmp280.h — BMP280 model for the emulated-I2C-sensor manager.
 *
 * Returns a singleton sensor_model_t whose register image emulates a Bosch
 * BMP280 (chip ID, fixed calibration trim, and pressure/temperature ADC bytes
 * computed from the requested temperature/pressure).  Parameters:
 *   "temperature_c"  float, °C
 *   "pressure_pa"    float, Pascals (1000..200000)
 * Handshake: a DUT write to ctrl_meas (0xF4) sets the "measuring" bit (0x08)
 * in the status register (0xF3) for the conversion time (forced-mode realism).
 * ========================================================================== */
#pragma once

#include "sensor_sim.h"

sensor_model_t *sensor_bmp280_model(void);
