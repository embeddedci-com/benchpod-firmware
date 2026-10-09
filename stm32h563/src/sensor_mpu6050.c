/* ============================================================================
 * sensor_mpu6050.c — InvenSense MPU-6050 model.  See sensor_mpu6050.h.
 *
 * Scales (register map 4.17, 4.19, 4.18):
 *   accel LSB/g      16384, 8192, 4096, 2048    for AFS_SEL 0..3 (±2, 4, 8, 16 g)
 *   gyro  LSB/(°/s)  131, 65.5, 32.8, 16.4      for FS_SEL 0..3  (±250 .. 2000 °/s)
 *   temperature      T = raw / 340 + 36.53      =>  raw = (T - 36.53) * 340
 * ========================================================================== */

#include "sensor_mpu6050.h"

#include <string.h>
#include <math.h>

#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_INT_STATUS   0x3A
#define REG_ACCEL_XOUT_H 0x3B   /* 0x3B..0x48: ax ay az temp gx gy gz, big-endian */
#define REG_PWR_MGMT_1   0x6B
#define REG_WHO_AM_I     0x75
#define WHO_AM_I_VAL     0x68
#define PWR_MGMT_1_RESET 0x40   /* SLEEP */
#define PWR_SLEEP        0x40
#define PWR_DEVICE_RESET 0x80   /* self-clearing: drivers poll until it reads 0 */
#define INT_DATA_RDY     0x01

static int16_t sat16(float v) {
    float r = roundf(v);
    if (r > 32767.0f)  r = 32767.0f;
    if (r < -32768.0f) r = -32768.0f;
    return (int16_t)r;
}

static void put_s16_be(uint8_t *img, uint8_t addr, int16_t v) {
    img[addr]     = (uint8_t)(((uint16_t)v >> 8) & 0xFF);
    img[addr + 1] = (uint8_t)((uint16_t)v & 0xFF);
}

/* values: ax ay az (g), gx gy gz (dps), temperature_c — in k_params order. */
static bool mpu6050_build(const float *values, const uint8_t *live, uint8_t img[SENSOR_REGIMAGE_LEN]) {
    /* DEVICE_RESET puts every register back to its power-on value and clears itself; the
       manager loads the whole image for it, a few ms after the write (sensor_sim_poll). */
    bool reset = !live || (live[REG_PWR_MGMT_1] & PWR_DEVICE_RESET);
    if (reset) {
        memset(img, 0, SENSOR_REGIMAGE_LEN);         /* power-on state */
        img[REG_PWR_MGMT_1] = PWR_MGMT_1_RESET;
    } else {
        memcpy(img, live, SENSOR_REGIMAGE_LEN);      /* keep everything the DUT configured */
    }
    img[REG_WHO_AM_I] = WHO_AM_I_VAL;

    static const float accel_lsb[4] = { 16384.0f, 8192.0f, 4096.0f, 2048.0f };
    static const float gyro_lsb[4]  = { 131.0f, 65.5f, 32.8f, 16.4f };
    float al = accel_lsb[(img[REG_ACCEL_CONFIG] >> 3) & 3];
    float gl = gyro_lsb[(img[REG_GYRO_CONFIG] >> 3) & 3];
    bool asleep = (img[REG_PWR_MGMT_1] & PWR_SLEEP) != 0;

    int16_t raw[7] = {
        sat16(values[0] * al), sat16(values[1] * al), sat16(values[2] * al),
        sat16((values[6] - 36.53f) * 340.0f),
        sat16(values[3] * gl), sat16(values[4] * gl), sat16(values[5] * gl),
    };
    for (unsigned i = 0; i < 7; i++)
        put_s16_be(img, (uint8_t)(REG_ACCEL_XOUT_H + 2u * i), asleep ? 0 : raw[i]);
    img[REG_INT_STATUS] = asleep ? 0 : INT_DATA_RDY;
    return reset;
}

static const sensor_param_t k_params[] = {
    { "accel_x_g",     "g",   -16.0f,   16.0f,   0.0f },
    { "accel_y_g",     "g",   -16.0f,   16.0f,   0.0f },
    { "accel_z_g",     "g",   -16.0f,   16.0f,   1.0f },
    { "gyro_x_dps",    "dps", -2000.0f, 2000.0f, 0.0f },
    { "gyro_y_dps",    "dps", -2000.0f, 2000.0f, 0.0f },
    { "gyro_z_dps",    "dps", -2000.0f, 2000.0f, 0.0f },
    { "temperature_c", "C",   -40.0f,   85.0f,   25.0f },
};

const sensor_model_t sensor_mpu6050_model = {
    .name           = "mpu6050",
    .label          = "InvenSense MPU-6050: accelerometer and gyroscope",
    .default_addr7  = 0x68,
    .alt_addr7      = 0x69,
    .data_lo        = REG_INT_STATUS,          /* 0x3A..0x48 */
    .data_hi        = REG_ACCEL_XOUT_H + 13,
    .watch_writes   = true,
    .params         = k_params,
    .n_params       = 7,
    .build_regimage = mpu6050_build,
};
