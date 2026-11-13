/* ============================================================================
 * sensor_bmp280.h — Bosch BMP280 and BME280 models for the emulated-I2C-sensor
 * manager.
 *
 * Register images emulating a BMP280 (chip ID 0x58) or a BME280 (chip ID 0x60):
 * fixed calibration trim, and pressure/temperature (and, BME280, humidity) ADC
 * bytes computed from the requested values with Bosch's compensation inverted.
 * Parameters:
 *   "temperature_c"  float, °C (-100..150)
 *   "pressure_pa"    float, Pascals (1000..200000)
 *   "humidity_pct"   float, %RH (0..100), BME280 only
 * Handshake: a DUT write to ctrl_meas (0xF4) sets the "measuring" bit (0x08)
 * in the status register (0xF3) for the conversion time (forced-mode realism).
 * ========================================================================== */
#pragma once

#include "sensor_sim.h"

extern const sensor_model_t sensor_bmp280_model;
extern const sensor_model_t sensor_bme280_model;
