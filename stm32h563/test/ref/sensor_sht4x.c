/* ============================================================================
 * sensor_sht4x.c — Sensirion SHT4x model.  See sensor_sht4x.h.
 *
 * Conversions from the SHT4x datasheet (section 4.6):
 *   T  = -45 + 175 * S_T / 65535      =>  S_T  = (T + 45) * 65535 / 175
 *   RH =  -6 + 125 * S_RH / 65535     =>  S_RH = (RH + 6) * 65535 / 125
 * ========================================================================== */

#include "sensor_sht4x.h"

#include <string.h>
#include <math.h>

#define CMD_MEASURE_HIGH 0xFD
#define CMD_MEASURE_MED  0xF6
#define CMD_MEASURE_LOW  0xE0
#define CMD_SERIAL       0x89

/* The serial number the model reports (two 16-bit words). */
#define SERIAL_HI 0x0B1Eu
#define SERIAL_LO 0x5EA4u

uint8_t sht4x_crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
    }
    return crc;
}

static uint16_t to_ticks(float v) {
    float t = roundf(v);
    if (t < 0.0f) t = 0.0f;
    if (t > 65535.0f) t = 65535.0f;
    return (uint16_t)t;
}

/* Two words with their CRCs at img[at..at+5], wrapping past 0xFF like the pointer does. */
static void put_answer(uint8_t img[SENSOR_REGIMAGE_LEN], uint8_t at, uint16_t w0, uint16_t w1) {
    uint8_t b[6] = { (uint8_t)(w0 >> 8), (uint8_t)w0, 0, (uint8_t)(w1 >> 8), (uint8_t)w1, 0 };
    b[2] = sht4x_crc8(&b[0], 2);
    b[5] = sht4x_crc8(&b[3], 2);
    for (unsigned i = 0; i < 6; i++) img[(uint8_t)(at + i)] = b[i];
}

/* values[0] = temperature_c, values[1] = humidity_pct. */
static bool sht4x_build(const float *values, const uint8_t *live, uint8_t img[SENSOR_REGIMAGE_LEN]) {
    (void)live;
    memset(img, 0, SENSOR_REGIMAGE_LEN);
    uint16_t st  = to_ticks((values[0] + 45.0f) * 65535.0f / 175.0f);
    uint16_t srh = to_ticks((values[1] + 6.0f) * 65535.0f / 125.0f);

    /* Heater commands first, lowest first, so where two overlap the higher
       command's answer wins (0x32 over 0x2F). */
    static const uint8_t heater[] = { 0x15, 0x1E, 0x24, 0x2F, 0x32, 0x39 };
    for (unsigned i = 0; i < sizeof(heater); i++) put_answer(img, heater[i], st, srh);

    put_answer(img, CMD_SERIAL, SERIAL_HI, SERIAL_LO);
    put_answer(img, CMD_MEASURE_LOW, st, srh);
    put_answer(img, CMD_MEASURE_MED, st, srh);
    put_answer(img, CMD_MEASURE_HIGH, st, srh);
    return false;
}

static const sensor_param_t k_params[] = {
    { "temperature_c", "C", -40.0f, 125.0f, 25.0f },
    { "humidity_pct",  "%",   0.0f, 100.0f, 40.0f },
};

const sensor_model_t sensor_sht4x_model = {
    .name           = "sht4x",
    .label          = "Sensirion SHT4x: temperature and humidity",
    .default_addr7  = 0x44,
    .alt_addr7      = 0x45,
    .data_lo        = 0x00,             /* the answers are spread over the image */
    .data_hi        = 0xFF,
    .params         = k_params,
    .n_params       = 2,
    .build_regimage = sht4x_build,
};
