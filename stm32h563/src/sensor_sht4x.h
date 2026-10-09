/* ============================================================================
 * sensor_sht4x.h — Sensirion SHT40/41/43/45 model for the emulated-I2C-sensor
 * manager.
 *
 * The SHT4x has no registers: the DUT writes a one-byte command, waits, then
 * reads six bytes (T msb, T lsb, CRC, RH msb, RH lsb, CRC).  On the gateware's
 * I2C target the command byte becomes the register pointer, so each command's
 * answer sits in the image AT the command's value, six bytes long:
 *   0xFD measure, high precision   (0xFD..0xFF then wraps to 0x00..0x02)
 *   0xF6 measure, medium precision (0xF6..0xFB)
 *   0xE0 measure, low precision    (0xE0..0xE5)
 *   0x89 read serial number        (0x89..0x8E)
 *   heater + measure 0x39 0x32 0x2F 0x24 0x1E 0x15: best effort, 0x2F's answer
 *   overlaps 0x32's and 0x32 wins (heater commands are rarely used)
 * The DUT rewrites the command before every read, so the pointer is always
 * where it should be.  A real SHT4x NACKs reads during its measurement; this
 * model answers at once (the DUT's wait is harmless).
 * Parameters:
 *   "temperature_c"  float, °C (-40..125)
 *   "humidity_pct"   float, %RH (0..100)
 * ========================================================================== */
#pragma once

#include "sensor_sim.h"

extern const sensor_model_t sensor_sht4x_model;

/* Sensirion CRC-8 (polynomial 0x31, init 0xFF) over `len` bytes. */
uint8_t sht4x_crc8(const uint8_t *data, size_t len);
