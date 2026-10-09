/*
 * test_sensor.c — host tests for the emulated I2C sensors: each model's register image read
 * back the way a DUT driver reads it (src/sensor_bmp280.c, sensor_sht4x.c, sensor_mpu6050.c),
 * and the manager (src/sensor_sim.c) against a fake I2C target: what a start, a parameter
 * change, a DUT register write, a soft reset and a gateware reconfiguration load.
 *
 * The driver-side math below is the vendors' reference code, written independently of the
 * models, so a model that inverts its compensation wrongly fails here.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "sensor_sim.h"
#include "sensor_bmp280.h"
#include "sensor_sht4x.h"
#include "sensor_mpu6050.h"

static int fails;
#define CHECK(cond, ...) do { if (!(cond)) { printf("FAIL %s:%d: ", __func__, __LINE__); \
        printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

/* ---- fake I2C target (signal_engine.c's fpga_i2c_*) and clock ---------------------------- */
static uint8_t  t_img[256];          /* the FPGA's register file */
static int      t_loads, t_last_lo, t_last_len, t_configs, t_disables, t_load_rc;
static bool     t_armed;
static uint16_t t_wr_count;
static uint64_t t_now_us = 1000000;
uint64_t time_us_64(void) { return t_now_us; }

int fpga_i2c_load_regs(uint8_t start, const uint8_t *data, size_t len) {
    if (t_load_rc) return t_load_rc;
    memcpy(&t_img[start], data, len);
    t_loads++; t_last_lo = start; t_last_len = (int)len;
    return 0;
}
int fpga_i2c_read_regs(uint8_t start, uint8_t *buf, size_t len) { memcpy(buf, &t_img[start], len); return 0; }
int fpga_i2c_sensor_config(uint8_t a, unsigned sda, unsigned scl, bool en, uint8_t tr,
                           uint8_t br, uint8_t bm, uint16_t cu) {
    (void)a; (void)sda; (void)scl; (void)tr; (void)br; (void)bm; (void)cu;
    t_configs++; t_armed = en; return 0;
}
void fpga_i2c_sensor_disable(void) { t_disables++; t_armed = false; }
int fpga_i2c_sensor_status(i2c_sensor_status_t *out) {
    memset(out, 0, sizeof(*out));
    out->wr_count = t_wr_count;
    return 0;
}
/* The DUT writes a register: the FPGA stores it and counts the write. */
static void dut_write(uint8_t reg, uint8_t val) { t_img[reg] = val; t_wr_count++; }
static void poll_after_ms(unsigned ms) { t_now_us += (uint64_t)ms * 1000u; (void)sensor_sim_poll(); }

/* ---- Bosch reference compensation (BMP280 datasheet 3.11.3, BME280 4.2.3) ----------------- */
typedef struct { uint16_t T1; int16_t T2, T3; uint16_t P1; int16_t P2, P3, P4, P5, P6, P7, P8, P9;
                 uint8_t H1, H3; int16_t H2, H4, H5; int8_t H6; } bosch_cal_t;

static uint16_t u16le(const uint8_t *r, int a) { return (uint16_t)(r[a] | (r[a + 1] << 8)); }
static bosch_cal_t bosch_read_cal(const uint8_t *r, bool hum) {
    bosch_cal_t c = {0};
    c.T1 = u16le(r, 0x88); c.T2 = (int16_t)u16le(r, 0x8A); c.T3 = (int16_t)u16le(r, 0x8C);
    c.P1 = u16le(r, 0x8E); c.P2 = (int16_t)u16le(r, 0x90); c.P3 = (int16_t)u16le(r, 0x92);
    c.P4 = (int16_t)u16le(r, 0x94); c.P5 = (int16_t)u16le(r, 0x96); c.P6 = (int16_t)u16le(r, 0x98);
    c.P7 = (int16_t)u16le(r, 0x9A); c.P8 = (int16_t)u16le(r, 0x9C); c.P9 = (int16_t)u16le(r, 0x9E);
    if (hum) {   /* BME280 datasheet table 16, the packed H4/H5 nibbles */
        c.H1 = r[0xA1]; c.H2 = (int16_t)u16le(r, 0xE1); c.H3 = r[0xE3];
        c.H4 = (int16_t)(((int8_t)r[0xE4] * 16) | (r[0xE5] & 0x0F));
        c.H5 = (int16_t)(((int8_t)r[0xE6] * 16) | (r[0xE5] >> 4));
        c.H6 = (int8_t)r[0xE7];
    }
    return c;
}
static double bosch_t(const bosch_cal_t *c, int32_t adc_T, int32_t *t_fine) {   /* double API */
    double v1 = ((double)adc_T / 16384.0 - (double)c->T1 / 1024.0) * (double)c->T2;
    double v2 = (((double)adc_T / 131072.0 - (double)c->T1 / 8192.0) *
                 ((double)adc_T / 131072.0 - (double)c->T1 / 8192.0)) * (double)c->T3;
    *t_fine = (int32_t)(v1 + v2);
    return (v1 + v2) / 5120.0;
}
static double bosch_p(const bosch_cal_t *c, int32_t adc_P, int32_t t_fine) {
    double v1 = (double)t_fine / 2.0 - 64000.0;
    double v2 = v1 * v1 * (double)c->P6 / 32768.0;
    v2 = v2 + v1 * (double)c->P5 * 2.0;
    v2 = v2 / 4.0 + (double)c->P4 * 65536.0;
    v1 = ((double)c->P3 * v1 * v1 / 524288.0 + (double)c->P2 * v1) / 524288.0;
    v1 = (1.0 + v1 / 32768.0) * (double)c->P1;
    if (v1 == 0.0) return 0;
    double p = 1048576.0 - (double)adc_P;
    p = (p - v2 / 4096.0) * 6250.0 / v1;
    v1 = (double)c->P9 * p * p / 2147483648.0;
    v2 = p * (double)c->P8 / 32768.0;
    return p + (v1 + v2 + (double)c->P7) / 16.0;
}
static double bosch_h(const bosch_cal_t *c, int32_t adc_H, int32_t t_fine) {
    double h = (double)t_fine - 76800.0;
    h = (adc_H - ((double)c->H4 * 64.0 + (double)c->H5 / 16384.0 * h)) *
        ((double)c->H2 / 65536.0 * (1.0 + (double)c->H6 / 67108864.0 * h *
         (1.0 + (double)c->H3 / 67108864.0 * h)));
    h = h * (1.0 - (double)c->H1 * h / 524288.0);
    if (h > 100.0) h = 100.0;
    if (h < 0.0) h = 0.0;
    return h;
}
/* A driver's forced-mode read: burst 0xF7..0xFE. */
static void bosch_measure(const uint8_t *r, bool hum, double *t, double *p, double *h) {
    bosch_cal_t c = bosch_read_cal(r, hum);
    int32_t adc_P = (r[0xF7] << 12) | (r[0xF8] << 4) | (r[0xF9] >> 4);
    int32_t adc_T = (r[0xFA] << 12) | (r[0xFB] << 4) | (r[0xFC] >> 4);
    int32_t t_fine = 0;
    *t = bosch_t(&c, adc_T, &t_fine);
    *p = bosch_p(&c, adc_P, t_fine);
    if (hum) *h = bosch_h(&c, (r[0xFD] << 8) | r[0xFE], t_fine);
}

static void test_bmp280_bme280_images(void) {
    uint8_t img[256];
    float v[3] = { 25.0f, 101325.0f, 40.0f };
    CHECK(!sensor_bmp280_model.build_regimage(v, NULL, img), "bmp280 asks for a full reload");
    CHECK(img[0xD0] == 0x58, "BMP280 chip id 0x%02x", img[0xD0]);
    CHECK(!sensor_bme280_model.build_regimage(v, NULL, img), "bme280 asks for a full reload");
    CHECK(img[0xD0] == 0x60, "BME280 chip id 0x%02x", img[0xD0]);

    static const float temps[] = { -100.0f, -40.0f, 0.0f, 23.5f, 85.0f, 150.0f };
    static const float press[] = { 1000.0f, 30000.0f, 90000.0f, 101325.0f, 110000.0f, 200000.0f };
    for (unsigned i = 0; i < 6; i++) {
        v[0] = temps[i]; v[1] = press[i];
        double t, p, h;
        sensor_bmp280_model.build_regimage(v, NULL, img);
        bosch_measure(img, false, &t, &p, &h);
        CHECK(fabs(t - temps[i]) < 0.02, "BMP280 %.2f C reads %.3f", temps[i], t);
        CHECK(fabs(p - press[i]) < 2.0, "BMP280 %.0f Pa reads %.1f", press[i], p);
    }
    static const float hums[] = { 0.0f, 5.0f, 40.0f, 63.2f, 99.0f, 100.0f };
    for (unsigned i = 0; i < 6; i++) {
        v[0] = temps[(i + 1) % 5]; v[1] = 95000.0f; v[2] = hums[i];
        double t, p, h;
        sensor_bme280_model.build_regimage(v, NULL, img);
        bosch_measure(img, true, &t, &p, &h);
        CHECK(fabs(t - v[0]) < 0.02, "BME280 %.2f C reads %.3f", v[0], t);
        CHECK(fabs(p - v[1]) < 2.0, "BME280 %.0f Pa reads %.1f", v[1], p);
        CHECK(fabs(h - hums[i]) < 0.1, "BME280 %.1f %%RH at %.1f C reads %.3f", hums[i], v[0], h);
    }
}

/* ---- SHT4x: what a driver reads after each command ---------------------------------------- */
static void sht_read(const uint8_t *img, uint8_t cmd, uint8_t out[6]) {
    for (unsigned i = 0; i < 6; i++) out[i] = img[(uint8_t)(cmd + i)];   /* the pointer wraps */
}
static void test_sht4x_image(void) {
    uint8_t img[256], r[6];
    CHECK(sht4x_crc8((const uint8_t *)"\xBE\xEF", 2) == 0x92, "Sensirion CRC example 0xBEEF -> 0x92");
    static const float temps[] = { -40.0f, 0.0f, 21.7f, 125.0f };
    static const float hums[]  = { 0.0f, 33.3f, 75.0f, 100.0f };
    static const uint8_t cmds[] = { 0xFD, 0xF6, 0xE0, 0x39, 0x32, 0x24, 0x1E, 0x15 };
    for (unsigned k = 0; k < 4; k++) {
        float v[2] = { temps[k], hums[k] };
        CHECK(!sensor_sht4x_model.build_regimage(v, NULL, img), "sht4x full reload");
        for (unsigned c = 0; c < sizeof(cmds); c++) {
            sht_read(img, cmds[c], r);
            CHECK(sht4x_crc8(r, 2) == r[2] && sht4x_crc8(r + 3, 2) == r[5], "cmd 0x%02x CRCs", cmds[c]);
            double t  = -45.0 + 175.0 * (double)((r[0] << 8) | r[1]) / 65535.0;
            double rh = -6.0 + 125.0 * (double)((r[3] << 8) | r[4]) / 65535.0;
            CHECK(fabs(t - temps[k]) < 0.01, "cmd 0x%02x: %.2f C reads %.3f", cmds[c], temps[k], t);
            CHECK(fabs(rh - hums[k]) < 0.01, "cmd 0x%02x: %.2f %%RH reads %.3f", cmds[c], hums[k], rh);
        }
        sht_read(img, 0x89, r);
        CHECK(sht4x_crc8(r, 2) == r[2] && sht4x_crc8(r + 3, 2) == r[5], "serial number CRCs");
        CHECK(((r[0] << 8) | r[1]) != 0, "serial number is not zero");
    }
}

/* ---- MPU-6050 ------------------------------------------------------------------------------ */
static int16_t s16be(const uint8_t *r, int a) { return (int16_t)((r[a] << 8) | r[a + 1]); }

static void test_mpu6050_image(void) {
    uint8_t img[256], live[256];
    float v[7] = { 0.5f, -0.25f, 1.0f, 100.0f, -50.0f, 250.0f, 30.0f };

    /* power-on: WHO_AM_I, asleep, every measurement 0 */
    CHECK(sensor_mpu6050_model.build_regimage(v, NULL, img), "power-on image is a full load");
    CHECK(img[0x75] == 0x68, "WHO_AM_I 0x%02x", img[0x75]);
    CHECK(img[0x6B] == 0x40, "PWR_MGMT_1 powers up asleep, 0x%02x", img[0x6B]);
    for (int a = 0x3A; a <= 0x48; a++) CHECK(img[a] == 0, "asleep: reg 0x%02x = 0x%02x", a, img[a]);

    /* awake at the default ranges (+-2 g, +-250 dps) */
    memcpy(live, img, sizeof(live));
    live[0x6B] = 0x01;
    CHECK(!sensor_mpu6050_model.build_regimage(v, live, img), "a data rebuild is not a full load");
    CHECK(img[0x6B] == 0x01, "the DUT's PWR_MGMT_1 is kept");
    CHECK(img[0x3A] & 0x01, "INT_STATUS data ready");
    CHECK(s16be(img, 0x3B) == 8192 && s16be(img, 0x3D) == -4096 && s16be(img, 0x3F) == 16384,
          "accel at 2 g: %d %d %d", s16be(img, 0x3B), s16be(img, 0x3D), s16be(img, 0x3F));
    CHECK(s16be(img, 0x43) == 13100 && s16be(img, 0x45) == -6550 && s16be(img, 0x47) == 32750,
          "gyro at 250 dps: %d %d %d", s16be(img, 0x43), s16be(img, 0x45), s16be(img, 0x47));
    double t = s16be(img, 0x41) / 340.0 + 36.53;
    CHECK(fabs(t - 30.0) < 0.01, "temperature reads %.3f", t);

    /* the DUT picks +-8 g and +-1000 dps: the codes follow */
    live[0x1C] = 2u << 3;
    live[0x1B] = 2u << 3;
    sensor_mpu6050_model.build_regimage(v, live, img);
    CHECK(s16be(img, 0x3B) == 2048 && s16be(img, 0x3F) == 4096, "accel at 8 g: %d %d",
          s16be(img, 0x3B), s16be(img, 0x3F));
    CHECK(s16be(img, 0x47) == 8200, "gyro z at 1000 dps: %d", s16be(img, 0x47));

    /* past the range saturates */
    float big[7] = { 16.0f, -16.0f, 0, 2000.0f, 0, 0, 25.0f };
    live[0x1C] = 0;
    live[0x1B] = 0;
    sensor_mpu6050_model.build_regimage(big, live, img);
    CHECK(s16be(img, 0x3B) == 32767 && s16be(img, 0x3D) == -32768 && s16be(img, 0x43) == 32767,
          "saturation: %d %d %d", s16be(img, 0x3B), s16be(img, 0x3D), s16be(img, 0x43));

    /* DEVICE_RESET: back to power-on, bit 7 reads clear */
    live[0x6B] = 0x80;
    CHECK(sensor_mpu6050_model.build_regimage(v, live, img), "DEVICE_RESET asks for a full load");
    CHECK(img[0x6B] == 0x40 && img[0x1C] == 0 && img[0x1B] == 0, "reset restores power-on registers");
}

/* ---- the manager against the fake target ---------------------------------------------------- */
static void test_manager(void) {
    size_t n = 0;
    const sensor_model_t *const *models = sensor_sim_models(&n);
    CHECK(n == 4, "4 models, got %zu", n);
    for (size_t i = 0; i < n; i++) {
        const sensor_model_t *m = models[i];
        CHECK(sensor_sim_find(m->name) == m, "find %s", m->name);
        CHECK(m->n_params > 0 && m->n_params <= SENSOR_MAX_PARAMS, "%s params", m->name);
        CHECK(m->data_lo <= m->data_hi, "%s data window", m->name);
        for (unsigned k = 0; k < m->n_params; k++)
            CHECK(m->params[k].min <= m->params[k].def && m->params[k].def <= m->params[k].max,
                  "%s.%s default inside its range", m->name, m->params[k].key);
    }
    CHECK(sensor_sim_find("bh1750") == NULL, "unknown type");
    CHECK(sensor_sim_start("nope", 0, 1, 2) == -1, "unknown type refused");

    /* start: the full image, then armed at the model's address */
    t_loads = 0;
    CHECK(sensor_sim_start("bme280", 0, 3, 4) == 0, "start bme280");
    CHECK(t_loads == 1 && t_last_lo == 0 && t_last_len == 256, "start loads all 256 bytes");
    CHECK(t_armed && sensor_sim_addr7() == 0x76 && sensor_sim_sda() == 3 && sensor_sim_scl() == 4, "armed");
    CHECK(fabsf(sensor_sim_value(2) - 40.0f) < 1e-6f, "humidity starts at its default");

    /* the DUT configures ctrl_hum/ctrl_meas; a parameter change must not undo that */
    dut_write(0xF2, 0x01);
    dut_write(0xF4, 0x25);
    CHECK(sensor_sim_set("humidity_pct", 55.0f) == 0, "set humidity");
    CHECK(t_last_lo == 0xF7 && t_last_len == 8, "only 0xF7..0xFE reloads (lo 0x%02x len %d)", t_last_lo, t_last_len);
    CHECK(t_img[0xF2] == 0x01 && t_img[0xF4] == 0x25, "the DUT's ctrl registers survive the reload");
    double t, p, h;
    bosch_measure(t_img, true, &t, &p, &h);
    CHECK(fabs(h - 55.0) < 0.1, "the FPGA image reads 55 %%RH: %.3f", h);
    CHECK(sensor_sim_set("humidity_pct", 101.0f) == -3, "out of range");
    CHECK(sensor_sim_set("pressure", 1.0f) == -2, "unknown key");
    CHECK(fabsf(sensor_sim_value(2) - 55.0f) < 1e-6f, "a refused value leaves the old one");

    /* a model that does not watch writes is never reloaded by poll */
    int loads = t_loads;
    dut_write(0xF4, 0x27);
    poll_after_ms(10);
    CHECK(t_loads == loads, "bme280 is not reloaded on a DUT write");

    /* MPU-6050: the DUT wakes it and picks a range; poll follows */
    CHECK(sensor_sim_start("mpu6050", 0x69, 5, 6) == 0, "start mpu6050 (replaces the bme280)");
    CHECK(t_disables >= 1, "the replaced sensor was disarmed");
    CHECK(sensor_sim_addr7() == 0x69, "explicit address kept");
    CHECK(s16be(t_img, 0x3F) == 0, "asleep after start");
    dut_write(0x6B, 0x00);
    poll_after_ms(3);
    CHECK(s16be(t_img, 0x3F) == 16384, "awake: accel z = 1 g at 2 g range (%d)", s16be(t_img, 0x3F));
    dut_write(0x1C, 3u << 3);
    poll_after_ms(3);
    CHECK(s16be(t_img, 0x3F) == 2048, "16 g range: accel z %d", s16be(t_img, 0x3F));
    CHECK(t_last_lo == 0x3A && t_last_len == 15, "a data rebuild loads 0x3A..0x48 only");
    CHECK(sensor_sim_set("accel_x_g", -1.5f) == 0, "set accel x");
    CHECK(s16be(t_img, 0x3B) == -3072, "accel x -1.5 g at 16 g: %d", s16be(t_img, 0x3B));

    /* DEVICE_RESET: full power-on image a poll later, the driver's busy-wait ends */
    dut_write(0x6B, 0x80);
    poll_after_ms(3);
    CHECK(t_last_lo == 0 && t_last_len == 256, "a soft reset reloads the whole image");
    CHECK(t_img[0x6B] == 0x40 && t_img[0x1C] == 0, "registers at power-on after the reset");

    /* gateware reconfiguration: re-armed with the same values on the next poll */
    t_configs = 0;
    memset(t_img, 0, sizeof(t_img));
    sensor_sim_on_gateware_reconfigured();
    CHECK(sensor_sim_set("accel_y_g", 0.5f) == 0, "a set while a re-arm is pending is kept");
    CHECK(sensor_sim_poll() == 0, "re-arm succeeds");
    CHECK(t_configs == 1 && t_img[0x75] == 0x68, "re-armed and the image is back");
    CHECK(fabsf(sensor_sim_value(0) + 1.5f) < 1e-6f && fabsf(sensor_sim_value(1) - 0.5f) < 1e-6f,
          "values survive the re-arm");

    /* a failed re-arm stops the sensor and reports it */
    sensor_sim_on_gateware_reconfigured();
    t_load_rc = -1;
    CHECK(sensor_sim_poll() == -1, "failed re-arm reported");
    CHECK(!sensor_sim_active(), "and the sensor is stopped");
    t_load_rc = 0;

    CHECK(sensor_sim_start("sht4x", 0, 1, 2) == 0, "start sht4x");
    CHECK(sensor_sim_set("temperature_c", 30.0f) == 0, "set sht4x temperature");
    CHECK(t_last_lo == 0 && t_last_len == 256, "sht4x reloads the whole image (answers are spread)");
    sensor_sim_stop();
    CHECK(!sensor_sim_active() && !t_armed, "stopped");
    CHECK(sensor_sim_set("temperature_c", 1.0f) == -1, "set with no sensor");
}

int main(void) {
    test_bmp280_bme280_images();
    test_sht4x_image();
    test_mpu6050_image();
    test_manager();
    if (fails) { printf("test_sensor: %d FAILED\n", fails); return 1; }
    printf("test_sensor: all passed\n");
    return 0;
}
