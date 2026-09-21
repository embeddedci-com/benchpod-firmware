/* ============================================================================
 * sensor_bmp280.c — BMP280 sensor model.  See sensor_bmp280.h.
 *
 * The compensation math + binary-search inversion are ported verbatim from the
 * software reference rp2350-hil-adapter/src/simulators/bmp280_math.cpp and
 * bmp280_calib.hpp (pure integer/float math, no Pico-I2C dependency).  The
 * register layout mirrors bmp280_simulator.cpp::init_registers() and the 6-byte
 * ADC packing in write_raw_measurements_to_regs_atomic().
 * ========================================================================== */

#include "sensor_bmp280.h"

#include <string.h>
#include <limits.h>
#include <math.h>

/* ---- fixed calibration trim (bmp280_calib.hpp) ---- */
#define DIG_T1 ((uint16_t)27504)
#define DIG_T2 ((int16_t)26435)
#define DIG_T3 ((int16_t)-1000)
#define DIG_P1 ((uint16_t)36477)
#define DIG_P2 ((int16_t)-10685)
#define DIG_P3 ((int16_t)3024)
#define DIG_P4 ((int16_t)2855)
#define DIG_P5 ((int16_t)140)
#define DIG_P6 ((int16_t)-7)
#define DIG_P7 ((int16_t)15500)
#define DIG_P8 ((int16_t)-14600)
#define DIG_P9 ((int16_t)6000)

/* ---- BMP280 register addresses ---- */
#define REG_CALIB_START 0x88
#define REG_ID          0xD0
#define REG_STATUS      0xF3
#define REG_CTRL_MEAS   0xF4
#define REG_CONFIG      0xF5
#define REG_PRESS_MSB   0xF7   /* 0xF7..0xFC: press[3] then temp[3] */
#define CHIP_ID         0x58
#define STATUS_MEASURING 0x08  /* bit3 */

/* ---- compensation math (Bosch fixed-point, ported from bmp280_math.cpp) ---- */

static int32_t compensate_temp_centi(int32_t adc_t, int32_t *t_fine_out) {
    const int32_t var1 = ((((adc_t >> 3) - ((int32_t)DIG_T1 << 1))) * (int32_t)DIG_T2) >> 11;
    const int32_t var2 = (((((adc_t >> 4) - (int32_t)DIG_T1) *
                            ((adc_t >> 4) - (int32_t)DIG_T1)) >> 12) * (int32_t)DIG_T3) >> 14;
    const int32_t t_fine = var1 + var2;
    if (t_fine_out) *t_fine_out = t_fine;
    return (t_fine * 5 + 128) >> 8;
}

static float compensate_press_pa(int32_t adc_p, int32_t t_fine) {
    const double t_f = (double)t_fine;
    double var1 = t_f - 128000.0;
    double var2 = var1 * var1 * (double)DIG_P6;
    var2 += var1 * (double)DIG_P5 * 131072.0;       /* << 17 */
    var2 += (double)DIG_P4 * 34359738368.0;         /* 1 << 35 */

    var1 = (var1 * var1 * (double)DIG_P3) / 256.0 +
           var1 * (double)DIG_P2 * 4096.0;          /* << 12 */
    var1 = ((140737488355328.0 + var1) * (double)DIG_P1) / 8589934592.0; /* (1<<47), >>33 */

    if (var1 == 0.0) return 0.0f;

    double p = (double)(1048576 - adc_p);
    p = ((p * 2147483648.0 - var2) * 3125.0) / var1;

    const double p_r13 = trunc(p / 8192.0);
    var1 = trunc((double)DIG_P9 * p_r13 * p_r13 / 33554432.0);  /* >> 25 */
    var2 = trunc((double)DIG_P8 * p / 524288.0);                /* >> 19 */

    p = trunc((p + var1 + var2) / 256.0) + (double)DIG_P7 * 16.0;  /* >> 8, +(P7<<4) */

    if (!isfinite(p)) return nanf("");
    const float out = (float)(p / 256.0);
    return isfinite(out) ? out : nanf("");
}

static int32_t find_adc_t_for_temp_c(float target_c) {
    const int32_t target_centi = (int32_t)lroundf(target_c * 100.0f);
    int32_t lo = 0, hi = 0xFFFFF, best = 0, best_err = INT_MAX;
    while (lo <= hi) {
        const int32_t mid = lo + ((hi - lo) / 2);
        int32_t t_fine = 0;
        const int32_t tc = compensate_temp_centi(mid, &t_fine);
        const int32_t err = tc >= target_centi ? tc - target_centi : target_centi - tc;
        if (err < best_err) { best_err = err; best = mid; }
        if      (tc < target_centi) lo = mid + 1;
        else if (tc > target_centi) hi = mid - 1;
        else return mid;
    }
    return best;
}

static int32_t find_adc_p_for_pressure_pa(float target_pa, int32_t adc_t) {
    int32_t t_fine = 0;
    (void)compensate_temp_centi(adc_t, &t_fine);
    int32_t lo = 0, hi = 0xFFFFF, best = 0;
    float best_err = 1e30f;
    while (lo <= hi) {
        const int32_t mid = lo + ((hi - lo) / 2);
        const float pa = compensate_press_pa(mid, t_fine);
        if (!isfinite(pa)) { hi = mid - 1; continue; }
        const float err = fabsf(pa - target_pa);
        if (err < best_err) { best_err = err; best = mid; }
        if      (pa > target_pa) lo = mid + 1;
        else if (pa < target_pa) hi = mid - 1;
        else return mid;
    }
    return best;
}

/* ---- model parameters (file-static state) ---- */
static float s_temp_c   = 25.0f;
static float s_press_pa = 101325.0f;

static void put_u16_le(uint8_t *img, uint8_t addr, uint16_t v) {
    img[addr]     = (uint8_t)(v & 0xFF);
    img[addr + 1] = (uint8_t)((v >> 8) & 0xFF);
}

static void bmp280_reset(sensor_model_t *m) {
    (void)m;
    s_temp_c   = 25.0f;
    s_press_pa = 101325.0f;
}

static int bmp280_set_param(sensor_model_t *m, const char *key, float value) {
    (void)m;
    if (strcmp(key, "temperature_c") == 0) {
        if (!isfinite(value)) return -1;
        s_temp_c = value;
        return 0;
    }
    if (strcmp(key, "pressure_pa") == 0) {
        if (!isfinite(value) || value < 1000.0f || value > 200000.0f) return -1;
        s_press_pa = value;
        return 0;
    }
    return -1;  /* unknown key */
}

static void bmp280_build_regimage(sensor_model_t *m, uint8_t img[SENSOR_REGIMAGE_LEN]) {
    (void)m;
    memset(img, 0, SENSOR_REGIMAGE_LEN);

    img[REG_ID]        = CHIP_ID;
    img[REG_STATUS]    = 0x00;
    img[REG_CTRL_MEAS] = 0x00;
    img[REG_CONFIG]    = 0x00;

    /* calibration block 0x88..0x9F (little-endian, signed stored as u16) */
    put_u16_le(img, 0x88, DIG_T1);
    put_u16_le(img, 0x8A, (uint16_t)DIG_T2);
    put_u16_le(img, 0x8C, (uint16_t)DIG_T3);
    put_u16_le(img, 0x8E, DIG_P1);
    put_u16_le(img, 0x90, (uint16_t)DIG_P2);
    put_u16_le(img, 0x92, (uint16_t)DIG_P3);
    put_u16_le(img, 0x94, (uint16_t)DIG_P4);
    put_u16_le(img, 0x96, (uint16_t)DIG_P5);
    put_u16_le(img, 0x98, (uint16_t)DIG_P6);
    put_u16_le(img, 0x9A, (uint16_t)DIG_P7);
    put_u16_le(img, 0x9C, (uint16_t)DIG_P8);
    put_u16_le(img, 0x9E, (uint16_t)DIG_P9);

    /* measurement: invert the requested temp/pressure to 20-bit ADC codes,
       pack into 0xF7..0xFC (press MSB/LSB/XLSB then temp MSB/LSB/XLSB). */
    int32_t adc_t = find_adc_t_for_temp_c(s_temp_c)      & 0xFFFFF;
    int32_t adc_p = find_adc_p_for_pressure_pa(s_press_pa, adc_t) & 0xFFFFF;

    img[REG_PRESS_MSB + 0] = (uint8_t)((adc_p >> 12) & 0xFF);
    img[REG_PRESS_MSB + 1] = (uint8_t)((adc_p >> 4) & 0xFF);
    img[REG_PRESS_MSB + 2] = (uint8_t)((adc_p & 0x0F) << 4);
    img[REG_PRESS_MSB + 3] = (uint8_t)((adc_t >> 12) & 0xFF);
    img[REG_PRESS_MSB + 4] = (uint8_t)((adc_t >> 4) & 0xFF);
    img[REG_PRESS_MSB + 5] = (uint8_t)((adc_t & 0x0F) << 4);
}

static sensor_model_t s_bmp280 = {
    .name           = "bmp280",
    .default_addr7  = 0x76,
    .trig_reg       = REG_CTRL_MEAS,    /* 0xF4 */
    .busy_reg       = REG_STATUS,       /* 0xF3 */
    .busy_mask      = STATUS_MEASURING, /* 0x08 */
    .conv_us        = 5500,             /* ~5.5 ms (typical max conversion) */
    .reset          = bmp280_reset,
    .set_param      = bmp280_set_param,
    .build_regimage = bmp280_build_regimage,
};

sensor_model_t *sensor_bmp280_model(void) { return &s_bmp280; }
