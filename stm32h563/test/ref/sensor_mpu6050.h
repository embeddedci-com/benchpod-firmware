/* ============================================================================
 * sensor_mpu6050.h — InvenSense MPU-6050 model (3-axis accelerometer, 3-axis
 * gyroscope, die temperature) for the emulated-I2C-sensor manager.
 *
 * Register map (MPU-6000/6050 register map rev 4.2): WHO_AM_I 0x75 = 0x68,
 * PWR_MGMT_1 0x6B powers up 0x40 (SLEEP), GYRO_CONFIG 0x1B / ACCEL_CONFIG 0x1C
 * carry the full-scale range in bits 4:3, INT_STATUS 0x3A bit0 = data ready,
 * measurements big-endian from 0x3B: accel X/Y/Z, temperature, gyro X/Y/Z.
 *
 * The DUT configures the part, so the model watches its writes: the data
 * window 0x3A..0x48 is rebuilt from the live image whenever the DUT writes a
 * register.  The scale follows the range the DUT chose, and while SLEEP is set
 * (as at power-up, until the driver clears it) every measurement reads 0, as
 * on the real part.  DEVICE_RESET (PWR_MGMT_1 bit7) restores the power-on
 * registers and reads back clear a few milliseconds later, so a driver that
 * polls for the reset to finish gets past it.
 * Parameters:
 *   "accel_x_g", "accel_y_g", "accel_z_g"     float, g (-16..16; default 0, 0, 1)
 *   "gyro_x_dps", "gyro_y_dps", "gyro_z_dps"  float, °/s (-2000..2000)
 *   "temperature_c"                           float, °C (-40..85)
 * A value past the selected range saturates at the 16-bit limit, as on the part.
 * ========================================================================== */
#pragma once

#include "sensor_sim.h"

extern const sensor_model_t sensor_mpu6050_model;
