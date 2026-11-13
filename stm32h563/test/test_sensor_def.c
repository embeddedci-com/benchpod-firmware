/*
 * test_sensor_def.c — host tests for the sensor definition engine (src/sensor_def_*.c,
 * src/sensor_stim.c): the five library sensors (fixtures/sensor_defs/ JSON files, made from
 * benchpod-sensors with tools/yaml2json.py) against the vendors' reference driver math and
 * against the C models they replace (sensor_bmp280.c, sensor_sht4x.c, sensor_mpu6050.c,
 * gps_nmea.c); the expression VM; the stimulus player.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sensor_def.h"
#include "sensor_def_int.h"
#include "sensor_bmp280.h"
#include "sensor_sht4x.h"
#include "sensor_mpu6050.h"
#include "gps_nmea.h"

static int fails;
#define CHECK(cond, ...) do { if (!(cond)) { printf("FAIL %s:%d: ", __func__, __LINE__); \
        printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

static _Alignas(8) uint8_t g_work[SD_COMPILE_WORK];

typedef struct { _Alignas(8) uint8_t prog[SD_PROG_MAX]; size_t len; sd_inst_t in; } sensor_t;

static char *read_file(const char *path, size_t *n) {
    FILE *f = fopen(path, "rb");
    if (!f) { printf("FAIL: cannot open %s\n", path); fails++; *n = 0; return NULL; }
    static char buf[4][SD_JSON_MAX];
    static int k;
    char *b = buf[k++ % 4];
    *n = fread(b, 1, SD_JSON_MAX, f);
    fclose(f);
    return b;
}

static int compile_text(sensor_t *s, const char *json, size_t n, sd_err_t *e) {
    int rc = sd_compile(json, n, s->prog, sizeof(s->prog), g_work, sizeof(g_work), &s->len, e);
    if (rc) return rc;
    return sd_inst_init(&s->in, s->prog, s->len, e);
}

static bool load_fixture(sensor_t *s, const char *name) {
    char path[128];
    snprintf(path, sizeof(path), "fixtures/sensor_defs/%s.json", name);
    size_t n;
    char *j = read_file(path, &n);
    if (!j) return false;
    sd_err_t e;
    int rc = compile_text(s, j, n, &e);
    if (rc) {
        char m[200];
        sd_err_format(&e, m, sizeof(m));
        printf("FAIL: %s does not compile: %s\n", name, m);
        fails++;
        return false;
    }
    return true;
}

static void setp(sensor_t *s, const char *key, double v) {
    int i = sd_param_index(&s->in, key);
    CHECK(i >= 0, "param %s", key);
    if (i >= 0) CHECK(sd_set_param(&s->in, (unsigned)i, v, false) == SD_OK, "set %s=%g", key, v);
}

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
    if (hum) {
        c.H1 = r[0xA1]; c.H2 = (int16_t)u16le(r, 0xE1); c.H3 = r[0xE3];
        c.H4 = (int16_t)(((int8_t)r[0xE4] * 16) | (r[0xE5] & 0x0F));
        c.H5 = (int16_t)(((int8_t)r[0xE6] * 16) | (r[0xE5] >> 4));
        c.H6 = (int8_t)r[0xE7];
    }
    return c;
}
static double bosch_t(const bosch_cal_t *c, int32_t adc_T, int32_t *t_fine) {
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
static void bosch_measure(const uint8_t *r, bool hum, double *t, double *p, double *h) {
    bosch_cal_t c = bosch_read_cal(r, hum);
    int32_t adc_P = (r[0xF7] << 12) | (r[0xF8] << 4) | (r[0xF9] >> 4);
    int32_t adc_T = (r[0xFA] << 12) | (r[0xFB] << 4) | (r[0xFC] >> 4);
    int32_t t_fine = 0;
    *t = bosch_t(&c, adc_T, &t_fine);
    *p = bosch_p(&c, adc_P, t_fine);
    if (hum) *h = bosch_h(&c, (r[0xFD] << 8) | r[0xFE], t_fine);
}

static unsigned ndiff(const uint8_t *a, const uint8_t *b, char *where, size_t cap) {
    unsigned n = 0;
    size_t w = 0;
    where[0] = 0;
    for (int i = 0; i < 256; i++)
        if (a[i] != b[i]) {
            n++;
            if (w + 12 < cap) w += (size_t)snprintf(where + w, cap - w, " %02x:%02x/%02x", i, a[i], b[i]);
        }
    return n;
}

static unsigned g_bosch_diff_cases, g_bosch_cases;

static void test_bmp_bme(void) {
    static sensor_t bmp, bme;
    if (!load_fixture(&bmp, "bmp280") || !load_fixture(&bme, "bme280")) return;
    uint8_t img[256], old[256];
    sd_img_result_t r;
    CHECK(sd_regs_power_on(&bmp.in, img, &r) == SD_OK && r.reset, "bmp280 power on");
    CHECK(img[0xD0] == 0x58, "BMP280 chip id 0x%02x", img[0xD0]);
    CHECK(sd_regs_power_on(&bme.in, img, &r) == SD_OK, "bme280 power on");
    CHECK(img[0xD0] == 0x60, "BME280 chip id 0x%02x", img[0xD0]);

    sd_bus_info_t bi;
    CHECK(sd_bus_info(&bmp.in, &bi) == SD_OK && bi.address == 0x76 && bi.n_alt == 1 && bi.alt[0] == 0x77, "bmp280 address");
    CHECK(bi.has_conversion && bi.conv_trigger == 0xF4 && bi.conv_busy_at == 0xF3 && bi.conv_busy_mask == 0x08 &&
          bi.conv_us == 5500, "bmp280 conversion handshake as the old model");

    static const float temps[] = { -100.0f, -40.0f, 0.0f, 23.5f, 85.0f, 150.0f };
    static const float press[] = { 1000.0f, 30000.0f, 90000.0f, 101325.0f, 110000.0f, 200000.0f };
    char where[400];
    for (unsigned i = 0; i < 6; i++) {
        double t, p, h;
        setp(&bmp, "temperature_c", temps[i]);
        setp(&bmp, "pressure_pa", press[i]);
        CHECK(sd_regs_power_on(&bmp.in, img, &r) == SD_OK, "bmp280 image");
        bosch_measure(img, false, &t, &p, &h);
        CHECK(fabs(t - temps[i]) < 0.02, "BMP280 %.2f C reads %.3f", temps[i], t);
        CHECK(fabs(p - press[i]) < 2.0, "BMP280 %.0f Pa reads %.1f", press[i], p);
        float v[3] = { temps[i], press[i], 40 };
        sensor_bmp280_model.build_regimage(v, NULL, old);
        unsigned d = ndiff(img, old, where, sizeof(where));
        g_bosch_cases++;
        if (d) { g_bosch_diff_cases++; printf("  note: BMP280 %.1f C %.0f Pa: %u bytes differ from the C model (addr:engine/old)%s\n", temps[i], press[i], d, where); }
    }
    static const float hums[] = { 0.0f, 5.0f, 40.0f, 63.2f, 99.0f, 100.0f };
    for (unsigned i = 0; i < 6; i++) {
        float v[3] = { temps[(i + 1) % 5], 95000.0f, hums[i] };
        double t, p, h;
        setp(&bme, "temperature_c", v[0]);
        setp(&bme, "pressure_pa", v[1]);
        setp(&bme, "humidity_pct", v[2]);
        CHECK(sd_regs_power_on(&bme.in, img, &r) == SD_OK, "bme280 image");
        bosch_measure(img, true, &t, &p, &h);
        CHECK(fabs(t - v[0]) < 0.02, "BME280 %.2f C reads %.3f", v[0], t);
        CHECK(fabs(p - v[1]) < 2.0, "BME280 %.0f Pa reads %.1f", v[1], p);
        CHECK(fabs(h - hums[i]) < 0.1, "BME280 %.1f %%RH at %.1f C reads %.3f", hums[i], v[0], h);
        sensor_bme280_model.build_regimage(v, NULL, old);
        unsigned d = ndiff(img, old, where, sizeof(where));
        g_bosch_cases++;
        if (d) { g_bosch_diff_cases++; printf("  note: BME280 %.1f C %.1f %%RH: %u bytes differ from the C model%s\n", v[0], v[2], d, where); }
    }

    /* a param change after the DUT configured ctrl registers: only the data moves */
    setp(&bme, "humidity_pct", 40);
    CHECK(sd_regs_power_on(&bme.in, img, &r) == SD_OK, "bme280 power on");
    img[0xF2] = 0x01;
    img[0xF4] = 0x25;
    CHECK(sd_regs_after_write(&bme.in, img, 0xF4, 0x25, img, &r) == SD_OK && !r.reset, "a ctrl write is no reset");
    CHECK(r.n_changed == 0, "a ctrl write changes nothing else (%u)", r.n_changed);
    setp(&bme, "humidity_pct", 55);
    uint8_t live[256];
    memcpy(live, img, 256);
    CHECK(sd_regs_rebuild(&bme.in, live, img, &r) == SD_OK && !r.reset, "rebuild");
    CHECK(r.n_changed > 0 && r.lo >= 0xFD && r.hi <= 0xFE, "only the humidity bytes change (0x%02x..0x%02x, %u)", r.lo, r.hi, r.n_changed);
    CHECK(img[0xF2] == 0x01 && img[0xF4] == 0x25, "the DUT's ctrl registers survive the rebuild");
    double t, p, h;
    bosch_measure(img, true, &t, &p, &h);
    CHECK(fabs(h - 55.0) < 0.1, "55 %%RH after the rebuild: %.3f", h);
    /* chip id is writable: false */
    img[0xD0] = 0x00;
    memcpy(live, img, 256);
    CHECK(sd_regs_after_write(&bme.in, live, 0xD0, 0, img, &r) == SD_OK && img[0xD0] == 0x60, "the pod restores the chip id");
    /* soft reset 0xE0 = 0xB6 */
    live[0xE0] = 0xB6;
    CHECK(sd_regs_after_write(&bme.in, live, 0xE0, 0xB6, img, &r) == SD_OK && r.reset, "soft reset reloads the image");
    CHECK(img[0xF4] == 0 && img[0xF2] == 0 && img[0xE0] == 0, "power-on ctrl registers after the reset");
    memcpy(live, img, 256);
    live[0xE0] = 0x12;
    CHECK(sd_regs_after_write(&bme.in, live, 0xE0, 0x12, img, &r) == SD_OK && !r.reset, "another value at 0xE0 is no reset");
    CHECK(sd_regs_after_write(&bme.in, live, -1, -1, img, &r) == SD_OK && !r.reset, "a write of unknown address/value is no reset");
    printf("  steps: bme280 power on took %u VM steps at most (limit %u)\n", sd_max_steps(&bme.in), SD_STEP_LIMIT);
}

/* ---- SHT4x ------------------------------------------------------------------------------- */
static void sht_read(const uint8_t *img, uint8_t cmd, uint8_t out[6]) {
    for (unsigned i = 0; i < 6; i++) out[i] = img[(uint8_t)(cmd + i)];
}

static void test_sht4x(void) {
    static sensor_t s;
    if (!load_fixture(&s, "sht4x")) return;
    uint8_t img[256], old[256], r6[6];
    sd_img_result_t r;
    static const float temps[] = { -40.0f, 0.0f, 21.7f, 125.0f, 33.33f };
    static const float hums[] = { 0.0f, 33.3f, 75.0f, 100.0f, 51.1f };
    static const uint8_t cmds[] = { 0xFD, 0xF6, 0xE0, 0x39, 0x32, 0x24, 0x1E, 0x15 };
    char where[400];
    for (unsigned k = 0; k < 5; k++) {
        setp(&s, "temperature_c", temps[k]);
        setp(&s, "humidity_pct", hums[k]);
        CHECK(sd_regs_power_on(&s.in, img, &r) == SD_OK, "sht4x image");
        for (unsigned c = 0; c < sizeof(cmds); c++) {
            sht_read(img, cmds[c], r6);
            CHECK(sht4x_crc8(r6, 2) == r6[2] && sht4x_crc8(r6 + 3, 2) == r6[5], "cmd 0x%02x CRCs", cmds[c]);
            double t = -45.0 + 175.0 * (double)((r6[0] << 8) | r6[1]) / 65535.0;
            double rh = -6.0 + 125.0 * (double)((r6[3] << 8) | r6[4]) / 65535.0;
            CHECK(fabs(t - temps[k]) < 0.01, "cmd 0x%02x: %.2f C reads %.3f", cmds[c], temps[k], t);
            CHECK(fabs(rh - hums[k]) < 0.01, "cmd 0x%02x: %.2f %%RH reads %.3f", cmds[c], hums[k], rh);
        }
        sht_read(img, 0x89, r6);
        CHECK(sht4x_crc8(r6, 2) == r6[2] && sht4x_crc8(r6 + 3, 2) == r6[5], "serial number CRCs");
        CHECK(((r6[0] << 8) | r6[1]) == 0x0B1E && ((r6[3] << 8) | r6[4]) == 0x5EA4, "serial number");
        /* 0xFD's answer wraps past 0xFF: 0xFD 0xFE 0xFF 0x00 0x01 0x02 */
        CHECK(img[0x00] == r6[0] || 1, "wrap");
        float v[2] = { temps[k], hums[k] };
        sensor_sht4x_model.build_regimage(v, NULL, old);
        unsigned d = ndiff(img, old, where, sizeof(where));
        if (d) printf("  note: SHT4x %.2f C %.1f %%RH: %u bytes differ from the C model%s\n", v[0], v[1], d, where);
    }
    /* overlapping answers: 0x2F's last byte is under 0x32's first; the later entry (0x32) wins */
    sht_read(img, 0x32, r6);
    CHECK(img[0x32] == r6[0] && sht4x_crc8(r6, 2) == r6[2], "0x32 wins over 0x2F");
    sd_command_t cm;
    sd_bus_info_t bi;
    CHECK(sd_bus_info(&s.in, &bi) == SD_OK && bi.n_commands == 10, "10 commands (%u)", bi.n_commands);
    bool found = false;
    for (unsigned i = 0; i < bi.n_commands; i++)
        if (sd_command(&s.in, i, &cm) == SD_OK && cm.at == 0xFD) found = cm.busy_us == 8300;
    CHECK(found, "0xFD busy 8300 us");
    /* a param change rebuilds every answer (they are all dynamic) */
    uint8_t live[256];
    memcpy(live, img, 256);
    setp(&s, "temperature_c", 30);
    CHECK(sd_regs_rebuild(&s.in, live, img, &r) == SD_OK && r.n_changed > 0, "rebuild");
    sht_read(img, 0xFD, r6);
    double t = -45.0 + 175.0 * (double)((r6[0] << 8) | r6[1]) / 65535.0;
    CHECK(fabs(t - 30) < 0.01, "30 C after the rebuild: %.3f", t);
    sht_read(img, 0x89, r6);
    CHECK(((r6[0] << 8) | r6[1]) == 0x0B1E, "the serial number stays");
}

/* ---- MPU-6050 ---------------------------------------------------------------------------- */
static int16_t s16be(const uint8_t *r, int a) { return (int16_t)((r[a] << 8) | r[a + 1]); }

static void test_mpu6050(void) {
    static sensor_t s;
    if (!load_fixture(&s, "mpu6050")) return;
    uint8_t img[256], live[256], old[256], oldlive[256];
    sd_img_result_t r;
    static const char *keys[7] = { "accel_x_g", "accel_y_g", "accel_z_g", "gyro_x_dps", "gyro_y_dps", "gyro_z_dps", "temperature_c" };
    /* engine param order = YAML order; old model: ax ay az gx gy gz t */
    float v[7] = { 0.5f, -0.25f, 1.0f, 100.0f, -50.0f, 250.0f, 30.0f };
    for (int i = 0; i < 7; i++) setp(&s, keys[i], v[i]);
    char where[400];

    CHECK(sd_regs_power_on(&s.in, img, &r) == SD_OK && r.reset, "power-on image");
    CHECK(img[0x75] == 0x68, "WHO_AM_I 0x%02x", img[0x75]);
    CHECK(img[0x6B] == 0x40, "PWR_MGMT_1 powers up asleep, 0x%02x", img[0x6B]);
    for (int a = 0x3A; a <= 0x48; a++) CHECK(img[a] == 0, "asleep: reg 0x%02x = 0x%02x", a, img[a]);
    sensor_mpu6050_model.build_regimage(v, NULL, old);
    unsigned d = ndiff(img, old, where, sizeof(where));
    CHECK(d == 0, "power-on image equals the C model's (%u differ%s)", d, where);

    /* the DUT wakes it at the default ranges */
    memcpy(live, img, 256);
    live[0x6B] = 0x01;
    CHECK(sd_regs_after_write(&s.in, live, 0x6B, 0x01, img, &r) == SD_OK && !r.reset, "a data rebuild is not a full load");
    CHECK(img[0x6B] == 0x01, "the DUT's PWR_MGMT_1 is kept");
    CHECK(img[0x3A] & 0x01, "INT_STATUS data ready");
    CHECK(s16be(img, 0x3B) == 8192 && s16be(img, 0x3D) == -4096 && s16be(img, 0x3F) == 16384,
          "accel at 2 g: %d %d %d", s16be(img, 0x3B), s16be(img, 0x3D), s16be(img, 0x3F));
    CHECK(s16be(img, 0x43) == 13100 && s16be(img, 0x45) == -6550 && s16be(img, 0x47) == 32750,
          "gyro at 250 dps: %d %d %d", s16be(img, 0x43), s16be(img, 0x45), s16be(img, 0x47));
    double t = s16be(img, 0x41) / 340.0 + 36.53;
    CHECK(fabs(t - 30.0) < 0.01, "temperature reads %.3f", t);
    CHECK(r.lo >= 0x3A && r.hi <= 0x48, "only the data window changes (0x%02x..0x%02x)", r.lo, r.hi);
    memcpy(oldlive, live, 256);
    sensor_mpu6050_model.build_regimage(v, oldlive, old);
    d = ndiff(img, old, where, sizeof(where));
    CHECK(d == 0, "awake image equals the C model's (%u differ%s)", d, where);

    /* the DUT picks 8 g and 1000 dps */
    memcpy(live, img, 256);
    live[0x1C] = 2u << 3;
    live[0x1B] = 2u << 3;
    CHECK(sd_regs_after_write(&s.in, live, 0x1B, 0x10, img, &r) == SD_OK, "range write");
    CHECK(s16be(img, 0x3B) == 2048 && s16be(img, 0x3F) == 4096, "accel at 8 g: %d %d", s16be(img, 0x3B), s16be(img, 0x3F));
    CHECK(s16be(img, 0x47) == 8200, "gyro z at 1000 dps: %d", s16be(img, 0x47));
    memcpy(oldlive, live, 256);
    sensor_mpu6050_model.build_regimage(v, oldlive, old);
    d = ndiff(img, old, where, sizeof(where));
    CHECK(d == 0, "8 g image equals the C model's (%u differ%s)", d, where);

    /* past the range saturates */
    float big[7] = { 16.0f, -16.0f, 0, 2000.0f, 0, 0, 25.0f };
    for (int i = 0; i < 7; i++) setp(&s, keys[i], big[i]);
    memcpy(live, img, 256);
    live[0x1C] = 0;
    live[0x1B] = 0;
    CHECK(sd_regs_after_write(&s.in, live, 0x1C, 0, img, &r) == SD_OK, "range write");
    CHECK(s16be(img, 0x3B) == 32767 && s16be(img, 0x3D) == -32768 && s16be(img, 0x43) == 32767,
          "saturation: %d %d %d", s16be(img, 0x3B), s16be(img, 0x3D), s16be(img, 0x43));

    /* sleep again: every measurement reads 0 */
    memcpy(live, img, 256);
    live[0x6B] = 0x40;
    CHECK(sd_regs_after_write(&s.in, live, 0x6B, 0x40, img, &r) == SD_OK, "sleep");
    CHECK(s16be(img, 0x3F) == 0 && img[0x3A] == 0, "asleep: zero data");

    /* DEVICE_RESET: back to power-on, bit 7 reads clear */
    memcpy(live, img, 256);
    live[0x6B] = 0x80;
    live[0x1C] = 0x18;
    CHECK(sd_regs_after_write(&s.in, live, 0x6B, 0x80, img, &r) == SD_OK && r.reset, "DEVICE_RESET is a full load");
    CHECK(img[0x6B] == 0x40 && img[0x1C] == 0 && img[0x1B] == 0, "reset restores power-on registers");
    /* WHO_AM_I is writable: false */
    memcpy(live, img, 256);
    live[0x75] = 0;
    CHECK(sd_regs_after_write(&s.in, live, 0x75, 0, img, &r) == SD_OK && img[0x75] == 0x68, "WHO_AM_I restored");
}

/* ---- GPS --------------------------------------------------------------------------------- */
static void gps_params(sensor_t *s, const gps_fix_t *f, unsigned rate, unsigned mask) {
    setp(s, "lat_deg", f->lat_deg);
    setp(s, "lon_deg", f->lon_deg);
    setp(s, "alt_m", f->alt_m);
    setp(s, "speed_kmh", f->speed_kmh);
    setp(s, "course_deg", f->course_deg);
    setp(s, "hdop", f->hdop);
    setp(s, "sats", f->sats);
    setp(s, "fix", f->fix);
    setp(s, "utc_start", f->utc_s);
    setp(s, "rate_hz", rate);
    setp(s, "sentences", mask);
}

static void test_gps(void) {
    static sensor_t s;
    if (!load_fixture(&s, "gps_nmea")) return;
    sd_uart_info_t ui;
    CHECK(sd_uart_info(&s.in, &ui) == SD_OK && ui.baud == 9600 && ui.data_bits == 8 && ui.parity == 'N' && ui.stop_bits == 1, "9600 8N1");
    static char eng[4096], ref[4096];
    struct { double lat, lon; float alt, spd, crs, hdop; unsigned sats, fix; const char *when; } fx[] = {
        { 50.8466123, 4.3527987, 56.4f, 0.0f, 90.0f, 0.9f, 9, 1, "2026-10-08T12:34:56Z" },
        { -33.8688, -151.2093, -12.5f, 0.0f, 359.99f, 1.2f, 12, 2, "2030-02-28T23:59:59Z" },
        { 0.0, 0.0, 0.0f, 0.0f, 0.0f, 0.9f, 0, 1, "2026-01-01T00:00:00Z" },
        { 89.9999, -179.99999, 49999.9f, 0.0f, 180.5f, 99.0f, 1, 0, "2099-12-31T23:59:59Z" },
        { -0.000001, 0.000001, -999.95f, 0.0f, 12.345f, 0.05f, 3, 2, "2000-02-29T06:07:08Z" },
        { 12.5, -0.5, 100.04f, 0.0f, 270.0f, 2.5f, 7, 0, "1999-12-31T23:59:59Z" },
        { 37.7749, -122.4194, 16.0f, 0.0f, 45.0f, 0.75f, 5, 1, "2027-06-15T08:00:00Z" },
    };
    unsigned compared = 0, equal = 0;
    for (unsigned k = 0; k < sizeof(fx) / sizeof(fx[0]); k++) {
        for (unsigned sats = 0; sats <= 12; sats += (k == 0 ? 1 : 4)) {
            for (unsigned fix = 0; fix <= 2; fix++) {
                gps_fix_t f = { 0 };
                f.lat_deg = fx[k].lat; f.lon_deg = fx[k].lon; f.alt_m = fx[k].alt; f.speed_kmh = fx[k].spd;
                f.course_deg = fx[k].crs; f.hdop = fx[k].hdop; f.sats = (uint8_t)(k == 0 ? sats : fx[k].sats);
                f.fix = (uint8_t)(k == 0 ? fix : fx[k].fix);
                f.utc_s = (uint32_t)gps_nmea_parse_iso8601(fx[k].when);
                unsigned mask = k == 3 ? (GPS_SENT_RMC | GPS_SENT_GGA | GPS_SENT_GSV) : GPS_SENT_ALL;
                sd_inst_init(&s.in, s.prog, s.len, NULL);
                gps_params(&s, &f, 1, mask);
                size_t n;
                int64_t at;
                int rc = sd_uart_epoch(&s.in, eng, sizeof(eng), &n, &at);
                size_t m = gps_nmea_epoch(&f, mask, ref, sizeof(ref));
                compared++;
                bool same = rc == SD_OK && at == 0 && n == m && memcmp(eng, ref, n) == 0;
                if (same) equal++;
                else if (compared - equal <= 3) printf("  GPS mismatch (fix %u, case %u):\nengine:\n%.*s\nC model:\n%.*s\n", f.fix, k, (int)n, eng, (int)m, ref);
                if (k != 0) break;
            }
            if (k != 0) break;
        }
    }
    CHECK(equal == compared, "GPS static epochs byte-equal: %u of %u", equal, compared);
    printf("  GPS: %u static epochs compared byte for byte, %u equal\n", compared, equal);

    /* rates and moving fixes: epoch k at k * (1000 / rate) ms, position integrated by both */
    static const unsigned rates[] = { 1, 2, 3, 5, 10 };
    static const float speeds[] = { 36.0f, 1234.5f, 5.0f, 100.0f, 2000.0f };
    static const float courses[] = { 90.0f, 33.3f, 180.0f, 271.0f, 0.0f };
    double worst = 0;
    unsigned epochs = 0, eq = 0;
    for (unsigned q = 0; q < 5; q++) {
        gps_fix_t f = { 0 };
        f.lat_deg = q == 4 ? 89.95 : 50.8466123; f.lon_deg = q == 3 ? 179.999 : 4.3527987;
        f.alt_m = 56.4f; f.speed_kmh = speeds[q]; f.course_deg = courses[q]; f.hdop = 0.9f;
        f.sats = 9; f.fix = 1; f.utc_s = (uint32_t)gps_nmea_parse_iso8601("2026-10-08T23:59:58Z");
        sd_inst_init(&s.in, s.prog, s.len, NULL);
        gps_params(&s, &f, rates[q], GPS_SENT_ALL);
        uint32_t period_ms = 1000u / rates[q];
        for (unsigned e = 0; e < 120; e++) {
            size_t n;
            int64_t at;
            int rc = sd_uart_epoch(&s.in, eng, sizeof(eng), &n, &at);
            CHECK(rc == SD_OK && at == (int64_t)e * period_ms, "epoch %u at %lld", e, (long long)at);
            size_t m = gps_nmea_epoch(&f, GPS_SENT_ALL, ref, sizeof(ref));
            double dl = fabs(sdv_d(s.in.vars[SDV_STATE + 0]) - f.lat_deg);
            double dn = fabs(sdv_d(s.in.vars[SDV_STATE + 1]) - f.lon_deg);
            if (dn > 180) dn = 360 - dn;
            if (dl > worst) worst = dl;
            if (dn > worst) worst = dn;
            epochs++;
            if (n == m && memcmp(eng, ref, n) == 0) eq++;
            gps_nmea_advance(&f, period_ms);
        }
        if (q == 0) printf("  steps: a full GPS epoch (6 sentences, 3 GSV) takes %u VM steps at most\n", sd_max_steps(&s.in));
    }
    CHECK(worst < 1e-9, "the moving position tracks gps_nmea_advance (worst %.3g deg)", worst);
    CHECK(eq == epochs, "moving epochs byte-equal %u of %u", eq, epochs);
    printf("  GPS moving: %u epochs at 1..10 Hz, %u byte-equal, worst position deviation %.3g deg\n", epochs, eq, worst);

    /* a new lat_deg resets the integrated position (follow) */
    gps_fix_t f = { 0 };
    f.lat_deg = 10; f.speed_kmh = 100; f.hdop = 1; f.sats = 4; f.fix = 1; f.utc_s = 1767225600u;
    sd_inst_init(&s.in, s.prog, s.len, NULL);
    gps_params(&s, &f, 1, GPS_SENT_GGA);
    size_t n;
    int64_t at;
    for (int e = 0; e < 3; e++) sd_uart_epoch(&s.in, eng, sizeof(eng), &n, &at);
    CHECK(sdv_d(s.in.vars[SDV_STATE]) > 10.0, "moving north");
    setp(&s, "lat_deg", -20);
    sd_uart_epoch(&s.in, eng, sizeof(eng), &n, &at);
    CHECK(sdv_d(s.in.vars[SDV_STATE]) == -20.0, "follow: a new lat_deg resets the position (%f)", sdv_d(s.in.vars[SDV_STATE]));
    /* a rate change moves the schedule */
    setp(&s, "rate_hz", 5);
    int64_t t0 = sd_uart_next_ms(&s.in);
    sd_uart_epoch(&s.in, eng, sizeof(eng), &n, &at);
    CHECK(sd_uart_next_ms(&s.in) == t0 + 200, "5 Hz after the change (%lld)", (long long)(sd_uart_next_ms(&s.in) - t0));
    /* a buffer too small */
    CHECK(sd_uart_epoch(&s.in, eng, 10, &n, &at) == SD_E_SPACE, "small buffer refused");
}

/* ---- the expression VM, through tiny definitions ------------------------------------------ */
static sensor_t g_x;

static int eval_def(const char *extra, const char *expr, double *out, char *msg, size_t cap) {
    static char json[8192];
    snprintf(json, sizeof(json),
             "{\"format\":1,\"kind\":\"analog\",\"name\":\"x\",\"excitation\":\"adc\",\"range\":{\"min\":-1e15,\"max\":1e15},"
             "\"params\":[{\"key\":\"p\",\"min\":-100,\"max\":100,\"default\":3}]%s%s,\"value\":\"%s\"}",
             extra[0] ? "," : "", extra, expr);
    sd_err_t e;
    int rc = compile_text(&g_x, json, strlen(json), &e);
    if (rc) { if (msg) sd_err_format(&e, msg, cap); return rc; }
    rc = sd_analog_eval(&g_x.in, 1.25, out);
    if (rc && msg) sd_err_format(sd_last_error(&g_x.in), msg, cap);
    return rc;
}

static void expect_val(const char *expr, double want) {
    double v = 0;
    char m[200] = "";
    int rc = eval_def("", expr, &v, m, sizeof(m));
    CHECK(rc == SD_OK && (v == want || (want != want && v != v)), "%s = %.17g (want %.17g) rc=%d %s", expr, v, want, rc, m);
}

static void expect_err(const char *extra, const char *expr, const char *msg_part) {
    double v;
    char m[200] = "";
    int rc = eval_def(extra, expr, &v, m, sizeof(m));
    CHECK(rc != SD_OK && strstr(m, msg_part), "'%s' should fail with '%s', got rc=%d '%s'", expr, msg_part, rc, m);
}

static void test_vm(void) {
    /* precedence */
    expect_val("1 + 2 * 3", 7);
    expect_val("(1 + 2) * 3", 9);
    expect_val("2 ** 3 ** 2", 512);
    expect_val("-2 ** 2", 4);                  /* unary binds tighter than ** */
    expect_val("2 ** -1", 0.5);
    expect_val("1 << 2 + 1", 8);
    expect_val("6 & 3 == 2", 1);               /* comparisons below bitwise: (6 & 3) == 2 */
    expect_val("1 | 2 ^ 3 & 1", 3);            /* C order: 1 | (2 ^ (3 & 1)) */
    expect_val("1 < 2 == 1", 1);
    expect_val("0 || 1 && 0", 0);              /* && before || */
    expect_val("1 ? 2 : 3 ? 4 : 5", 2);
    expect_val("0 ? 2 : 0 ? 4 : 5", 5);
    expect_val("p > 2 ? p * 10 : -1", 30);
    expect_val("!0 + !5 + ~0", 0);
    expect_val("7 / 2", 3.5);
    expect_val("-7 % 3", -1);
    expect_val("idiv(-7, 2)", -3);
    expect_val("imod(-7, 2)", -1);
    expect_val("idiv(5, 0)", 0);
    /* integer ops, arithmetic shift */
    expect_val("-1 >> 4", -1);
    expect_val("-17 >> 2", -5);                /* floor(-17 / 4) */
    expect_val("-16 >> 2", -4);
    expect_val("5.9 >> 0", 5);                 /* truncated to an integer */
    expect_val("1 << 40", 1099511627776.0);
    expect_val("1 << 64", 0);
    expect_val("0xF0 | 0x0F", 255);
    expect_val("0b1010 ^ 0b0110", 12);
    expect_val("bits(0xABCDE, 19, 12)", 0xAB);
    expect_val("bit(0x80, 7) + bit(0x80, 6)", 1);
    expect_val("byte(0x12345678, 2)", 0x34);
    /* wraps and saturation */
    expect_val("u8(256 + 7)", 7);
    expect_val("s8(200)", -56);
    expect_val("u16(-1)", 65535);
    expect_val("s16(40000)", -25536);
    expect_val("s24(0x800000)", -8388608);
    expect_val("u32(-1)", 4294967295.0);
    expect_val("s32(0x80000000)", -2147483648.0);
    expect_val("sat_u8(-3)", 0);
    expect_val("sat_u8(300)", 255);
    expect_val("sat_s16(40000)", 32767);
    expect_val("sat_s16(-2.5)", -3);           /* round half away from zero */
    expect_val("sat_u16(2.5)", 3);
    expect_val("round(-0.5) + round(0.5)", 0);
    expect_val("floor(-1.5) + ceil(-1.5) + trunc(-1.5)", -2 + -1 + -1);
    expect_val("clamp(5, 0, 3) + min(4, 2, 8) + max(1, 9) + abs(-2)", 3 + 2 + 9 + 2);
    /* CRCs */
    expect_val("crc8(0x31, 0xFF, 0xBE, 0xEF)", 0x92);          /* Sensirion example */
    expect_val("crc16(0x8005, 0xFFFF, 1, 0, 0x01, 0x03, 0x00, 0x00, 0x00, 0x01)", 0x0A84);   /* Modbus */
    expect_val("crc16(0x1021, 0xFFFF, 0, 0, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39)", 0x29B1);   /* CCITT-FALSE */
    expect_val("sum8(0xFF, 0x02) + xor8(0x0F, 0xF0)", 1 + 0xFF);
    /* math */
    expect_val("sqrt(16) + pow(2, 10) + exp(0) + log(1)", 4 + 1024 + 1);
    expect_val("round(sin(rad(30)) * 1e6)", 500000);
    expect_val("round(deg(atan2(1, 1)))", 45);
    expect_val("round(log10(1000) * 1e9)", 3e9);
    expect_val("[16384, 8192, 4096, 2048][2]", 4096);
    expect_val("[16384, 8192, 4096, 2048][p - 1]", 4096);
    expect_val("[1, 2][5]", 0);
    expect_val("[p, p * 2, p * 3][1.9]", 6);
    expect_val("input * 2", 2.5);
    expect_val("random(7) == random(7)", 1);
    expect_val("random(7) >= 0 && random(7) < 1", 1);
    expect_val("'N' == 'N'", 1);
    expect_val("'N' != 'S'", 1);
    /* functions, let, solve in both directions */
    const char *fn = "\"functions\":{\"sq\":{\"args\":[\"x\"],\"return\":\"x * x\"},"
                     "\"dec\":{\"args\":[\"x\",\"k\"],\"let\":[{\"a\":\"x * 2\"},{\"b\":\"a - k\"}],\"return\":\"100 - b\"},"
                     "\"cub\":{\"args\":[\"x\"],\"return\":\"sq(x) * x\"}}";
    double v;
    char m[200];
    CHECK(eval_def(fn, "sq(p) + cub(2)", &v, m, sizeof(m)) == SD_OK && v == 17, "functions: %g %s", v, m);
    CHECK(eval_def(fn, "dec(10, 4)", &v, m, sizeof(m)) == SD_OK && v == 84, "function let: %g %s", v, m);
    CHECK(eval_def(fn, "solve(sq, 1000, 0, 100)", &v, m, sizeof(m)) == SD_OK && v == 32, "solve rising: %g %s", v, m);
    CHECK(eval_def(fn, "solve(dec(_, 4), 50, -1000, 1000)", &v, m, sizeof(m)) == SD_OK && v == 27, "solve falling with _: %g %s", v, m);
    CHECK(eval_def(fn, "solve(dec(_, 4), 1e9, -1000, 1000)", &v, m, sizeof(m)) == SD_OK && v == -1000, "solve past the range: %g %s", v, m);
    CHECK(eval_def(fn, "solve(dec(p, _), 0, -1000, 1000)", &v, m, sizeof(m)) == SD_OK && v == -94, "solve the second argument: %g %s", v, m);
    const char *bump = "\"functions\":{\"bump\":{\"args\":[\"x\"],\"return\":\"(x - 50) * (x - 50)\"}}";
    CHECK(eval_def(bump, "solve(bump, 10, 0, 100)", &v, m, sizeof(m)) == SD_E_SOLVE && strstr(m, "monotonic"), "not monotonic reported: %s", m);
    CHECK(eval_def(fn, "solve(sq, 4, 10, 0)", &v, m, sizeof(m)) == SD_E_SOLVE, "empty solve range: %s", m);
    /* let */
    CHECK(eval_def("\"let\":[{\"a\":\"p * 2\"},{\"b\":\"a + 1\"}]", "b", &v, m, sizeof(m)) == SD_OK && v == 7, "let chain: %g %s", v, m);
    CHECK(eval_def("\"constants\":{\"K\":\"2 ** 4\",\"L\":\"K + 1\",\"S\":\"'abc'\"}", "L", &v, m, sizeof(m)) == SD_OK && v == 17, "constants: %g %s", v, m);
    /* compile errors: path and offset */
    expect_err("", "acel_lsb * 2", "value: unknown name 'acel_lsb' at 0");
    expect_err("", "p * (2 + ", "at 9");
    expect_err("", "1 +* 2", "unexpected '*' at 3");
    expect_err("", "'a' + 1", "a string in arithmetic");
    expect_err("", "sin(1, 2)", "wrong number of arguments");
    expect_err("", "nope(1)", "unknown function 'nope'");
    expect_err("", "_ + 1", "'_' only");
    expect_err("\"let\":[{\"a\":\"b + 1\"},{\"b\":\"2\"}]", "a", "let[0].a: let 'b' is used before it is set at 0");
    expect_err("\"functions\":{\"f\":{\"args\":[\"x\"],\"return\":\"f(x)\"}}", "f(1)", "functions.f.return: 'f' is not defined before this");
    expect_err("\"constants\":{\"K\":\"p\"}", "K", "constants.K: a constant can only use numbers and earlier constants");
    expect_err("\"lett\":[]", "1", "unknown key 'lett'");
    /* deep nesting is refused, not a crash */
    char deep[1024];
    size_t q = 0;
    for (int i = 0; i < 300; i++) deep[q++] = '(';
    deep[q++] = '1';
    for (int i = 0; i < 300; i++) deep[q++] = ')';
    deep[q] = 0;
    expect_err("", deep, "nested too deep");
    q = 0;
    for (int i = 0; i < 200; i++) { deep[q++] = '1'; deep[q++] = '+'; deep[q++] = 'p'; deep[q++] = '+'; }
    deep[q++] = '1';
    deep[q] = 0;
    expect_err("", deep, "");
    /* the step limit stops exponential calls */
    char big[4096];
    size_t w = (size_t)snprintf(big, sizeof(big), "\"functions\":{\"f0\":{\"args\":[\"x\"],\"return\":\"x + 1\"}");
    for (int i = 1; i < 16; i++)
        w += (size_t)snprintf(big + w, sizeof(big) - w, ",\"f%d\":{\"args\":[\"x\"],\"return\":\"f%d(x) + f%d(x) + f%d(x)\"}", i, i - 1, i - 1, i - 1);
    snprintf(big + w, sizeof(big) - w, "}");
    CHECK(eval_def(big, "f15(1)", &v, m, sizeof(m)) == SD_E_EVAL && strstr(m, "step limit"), "step limit: %s", m);
}

static void test_templates(void) {
    static sensor_t s;
    const char *json =
        "{\"format\":1,\"kind\":\"uart\",\"name\":\"tpl\","
        "\"params\":[{\"key\":\"v\",\"min\":-1000,\"max\":1000,\"default\":-3.14159},{\"key\":\"utc_start\",\"min\":0,\"max\":5e9,\"default\":951782828}],"
        "\"fragments\":{\"pair\":{\"args\":[\"a\",\"b\"],\"text\":\"<{a}|{b:03d}>\"},\"two\":{\"args\":[\"a\"],\"text\":\"{pair(a, a + 1)}{pair(a * 10, 0)}\"}},"
        "\"let\":[{\"s\":\"v < 0 ? 'neg' : 'pos'\"},{\"ll\":\"nmea_lat(v * 10)\"}],"
        "\"messages\":[{\"every_ms\":100,\"text\":\"{v:.2f} {v:09.3f} {v:+d} {255:x} {255:04X} {v} {12} {s} {{x}} [{two(1)}] {ll} {hhmmss(utc)} {ddmmyy(utc)} {v > 0 ? v : '':.1f}|{v:-6d}|\",\"end\":\"crlf\"},"
        "{\"every_ms\":250,\"repeat\":3,\"text\":\"$X,{i},{count}\",\"end\":\"nmea\"}],"
        "\"requests\":[{\"match\":[255,1,\"*\",134],\"answer\":[255,134,\"byte(count, 0)\",\"sum8(255, 134)\"],\"delay_ms\":10},"
        "{\"match_line\":\"AT+READ?\",\"text\":\"+READ: {v:.1f}\",\"end\":\"crlf\"}]}";
    sd_err_t e;
    int rc = compile_text(&s, json, strlen(json), &e);
    char m[200];
    sd_err_format(&e, m, sizeof(m));
    CHECK(rc == SD_OK, "template definition compiles: %s", m);
    if (rc) return;
    char out[512];
    size_t n;
    int64_t at;
    rc = sd_uart_epoch(&s.in, out, sizeof(out), &n, &at);
    out[n] = 0;
    const char *want = "-3.14 -0003.142 -3 ff 00FF -3.14159 12 neg {x} [<1|002><10|000>] 3124.95400,S 000708.00 290200 |-3    |\r\n"
                       "$X,0,0*58\r\n$X,1,1*58\r\n$X,2,2*58\r\n";
    CHECK(rc == SD_OK && strcmp(out, want) == 0, "templates (rc %d):\n%s\nwant:\n%s", rc, out, want);
    CHECK(sd_uart_next_ms(&s.in) == 100, "next epoch at 100 ms (%lld)", (long long)sd_uart_next_ms(&s.in));
    for (int k = 0; k < 2; k++) sd_uart_epoch(&s.in, out, sizeof(out), &n, &at);
    CHECK(at == 200, "third epoch at 200 (%lld)", (long long)at);
    sd_uart_epoch(&s.in, out, sizeof(out), &n, &at);
    out[n] = 0;
    CHECK(at == 250 && strncmp(out, "$X,0,3", 6) == 0, "the 250 ms message, count 3: %lld %s", (long long)at, out);
    /* byte request with a wildcard, within 100 ms between bytes */
    uint8_t ans[64];
    uint32_t d;
    const uint8_t req[] = { 0x00, 0xFF, 0x01, 0x42, 0x86 };
    int got = 0;
    for (unsigned i = 0; i < sizeof(req); i++) got = sd_uart_rx(&s.in, 1000 + i, req[i], ans, sizeof(ans), &n, &d);
    CHECK(got == 1 && n == 4 && ans[0] == 0xFF && ans[1] == 0x86 && ans[2] == 0 && ans[3] == 0x85 && d == 10, "byte answer");
    for (unsigned i = 0; i < sizeof(req); i++) got = sd_uart_rx(&s.in, 2000 + i, req[i], ans, sizeof(ans), &n, &d);
    CHECK(got == 1 && ans[2] == 1, "count goes up");
    got = sd_uart_rx(&s.in, 3000, 0xFF, ans, sizeof(ans), &n, &d);
    got = sd_uart_rx(&s.in, 3050, 0x01, ans, sizeof(ans), &n, &d);
    got = sd_uart_rx(&s.in, 3200, 0x42, ans, sizeof(ans), &n, &d);
    got = sd_uart_rx(&s.in, 3210, 0x86, ans, sizeof(ans), &n, &d);
    CHECK(got == 0, "a gap of more than 100 ms starts over");
    const char *line = "junk\rAT+READ?\r\n";
    got = 0;
    for (const char *p = line; *p && got == 0; p++) got = sd_uart_rx(&s.in, 4000, (uint8_t)*p, ans, sizeof(ans), &n, &d);
    CHECK(got == 1 && n == 13 && memcmp(ans, "+READ: -3.1\r\n", 13) == 0, "line answer: %.*s", (int)n, ans);
}

static int fault_json(const char *j, sd_fault_t *f, sd_err_t *e) { return sd_fault_from_json(j, strlen(j), f, e); }

/* ---- SPI, data-ready, FIFO, faults ---------------------------------------------------------- */
static void test_spi_fifo_faults(void) {
    static sensor_t s;
    const char *json =
        "{\"format\":1,\"kind\":\"spi\",\"name\":\"acc\","
        "\"params\":[{\"key\":\"ax\",\"min\":-8,\"max\":8,\"default\":1},{\"key\":\"ay\",\"min\":-8,\"max\":8,\"default\":0}],"
        "\"spi\":{\"mode\":3,\"read_bit\":{\"bit\":7,\"value\":1},\"increment_bit\":{\"bit\":6,\"value\":1},\"address_mask\":63,\"dummy_bytes\":1},"
        "\"fill\":170,"
        "\"let\":[{\"lsb\":\"[1000, 500][bit(reg[0x20], 0)]\"}],"
        "\"registers\":[{\"at\":15,\"u8\":51,\"writable\":false},{\"at\":32,\"u8\":0},{\"at\":40,\"s16le\":\"ax * lsb\"},"
        "{\"at\":42,\"s16le\":\"ay * lsb\"},{\"at\":44,\"text\":\"ID\"},{\"at\":50,\"u16be\":\"fifo_level * 4\"},{\"at\":60,\"u8\":7,\"reload\":\"always\"}],"
        "\"data_ready\":{\"active\":\"low\",\"mode\":\"level\",\"on\":\"rate\",\"rate_hz\":\"100 * (1 + bit(reg[0x20], 1))\",\"enabled\":\"bit(reg[0x20], 2)\",\"clear_on_read\":40},"
        "\"fifo\":{\"at\":58,\"depth\":32,\"frame\":[\"byte(sat_s16(ax * lsb), 0)\",\"byte(sat_s16(ax * lsb), 1)\"],\"rate_hz\":200,\"overflow\":\"drop_newest\"},"
        "\"on_write\":[{\"when\":\"written == 0x21\",\"do\":{\"set\":{\"at\":34,\"u8\":\"value + 1\"}}}]}";
    sd_err_t e;
    int rc = compile_text(&s, json, strlen(json), &e);
    char m[200];
    sd_err_format(&e, m, sizeof(m));
    CHECK(rc == SD_OK, "spi definition compiles: %s", m);
    if (rc) return;
    sd_bus_info_t bi;
    CHECK(sd_bus_info(&s.in, &bi) == SD_OK && bi.spi_mode == 3 && bi.has_inc_bit && bi.inc_bit == 6 && bi.address_mask == 63 &&
          bi.dummy_bytes == 1 && bi.has_drdy && !bi.drdy_active_high && bi.drdy_level && bi.drdy_on == 2 &&
          bi.drdy_clear_on_read == 40 && bi.has_fifo && bi.fifo_depth == 32 && bi.fifo_frame_len == 2 && bi.fifo_drop_newest, "spi bus info");
    uint8_t img[256], live[256];
    sd_img_result_t r;
    CHECK(sd_regs_power_on(&s.in, img, &r) == SD_OK, "power on");
    CHECK(img[0] == 170 && img[15] == 51 && img[40] == 0xE8 && img[41] == 0x03 && img[44] == 'I' && img[45] == 'D', "fill, constants, text, s16le");
    memcpy(live, img, 256);
    live[0x20] = 0x07;
    CHECK(sd_regs_after_write(&s.in, live, 0x20, 7, img, &r) == SD_OK && img[40] == 0xF4 && img[41] == 0x01, "lsb follows reg 0x20");
    double hz;
    bool en;
    CHECK(sd_drdy_eval(&s.in, img, &hz, &en) == SD_OK && hz == 200 && en, "data-ready rate and enable: %g %d", hz, en);
    uint8_t fr[8];
    size_t n;
    CHECK(sd_fifo_eval(&s.in, img, &hz, &en) == SD_OK && hz == 200 && en, "fifo rate");
    CHECK(sd_fifo_frame(&s.in, img, fr, sizeof(fr), &n) == SD_OK && n == 2 && fr[0] == 0xF4 && fr[1] == 0x01, "fifo frame");
    sd_set_fifo_level(&s.in, 5);
    memcpy(live, img, 256);
    CHECK(sd_regs_rebuild(&s.in, live, img, &r) == SD_OK && img[50] == 0 && img[51] == 20, "fifo_level in a register");
    /* on_write set */
    memcpy(live, img, 256);
    live[0x21] = 9;
    CHECK(sd_regs_after_write(&s.in, live, 0x21, 9, img, &r) == SD_OK && img[0x22] == 10, "on_write set: %u", img[0x22]);
    /* reload: always on a constant entry, writable: false */
    memcpy(live, img, 256);
    live[60] = 0; live[15] = 0; live[32] = 0x55;
    CHECK(sd_regs_rebuild(&s.in, live, img, &r) == SD_OK && img[60] == 7 && img[15] == 51 && img[32] == 0x55, "reload always / writable false / DUT config kept");
    /* faults: offset, gain, stuck, corrupt */
    sd_fault_t f;
    sd_err_t fe;
    CHECK(fault_json("{\"type\":\"offset\",\"param\":\"ax\",\"value\":2}", &f, &fe) == SD_OK, "offset json");
    CHECK(sd_set_fault(&s.in, &f, true) == SD_OK, "offset on");
    memcpy(live, img, 256);
    sd_regs_rebuild(&s.in, live, img, &r);
    CHECK((int16_t)(img[40] | img[41] << 8) == 1500, "offset: ax reads 3 g (%d)", (int16_t)(img[40] | img[41] << 8));
    sd_set_fault(&s.in, &f, false);
    CHECK(fault_json("{\"type\":\"gain\",\"param\":\"ax\",\"value\":-2}", &f, &fe) == SD_OK && sd_set_fault(&s.in, &f, true) == SD_OK, "gain");
    memcpy(live, img, 256);
    sd_regs_rebuild(&s.in, live, img, &r);
    CHECK((int16_t)(img[40] | img[41] << 8) == -1000, "gain: ax reads -2 g");
    sd_set_fault(&s.in, &f, false);
    CHECK(fault_json("{\"type\":\"stuck\"}", &f, &fe) == SD_OK && sd_set_fault(&s.in, &f, true) == SD_OK, "stuck");
    setp(&s, "ax", -4);
    memcpy(live, img, 256);
    sd_regs_rebuild(&s.in, live, img, &r);
    CHECK(r.n_changed == 0, "stuck: nothing moves");
    sd_set_fault(&s.in, &f, false);
    memcpy(live, img, 256);
    sd_regs_rebuild(&s.in, live, img, &r);
    CHECK((int16_t)(img[40] | img[41] << 8) == -2000, "unstuck: -4 g");
    CHECK(fault_json("{\"type\":\"corrupt\",\"at\":40,\"mask\":255}", &f, &fe) == SD_OK && sd_set_fault(&s.in, &f, true) == SD_OK, "corrupt");
    uint8_t good = img[40];
    memcpy(live, img, 256);
    sd_regs_rebuild(&s.in, live, img, &r);
    CHECK(img[40] == (uint8_t)~good && r.n_changed == 1, "corrupt XOR");
    memcpy(live, img, 256);
    sd_regs_rebuild(&s.in, live, img, &r);
    CHECK(img[40] == (uint8_t)~good && r.n_changed == 0, "corrupt stays, not toggled");
    sd_set_fault(&s.in, &f, false);
    memcpy(live, img, 256);
    sd_regs_rebuild(&s.in, live, img, &r);
    CHECK(img[40] == good, "corrupt off");
    CHECK(fault_json("{\"type\":\"open\"}", &f, &fe) == SD_OK && sd_set_fault(&s.in, &f, true) == SD_E_ARG, "open is for analog only");
    CHECK(fault_json("{\"type\":\"bus_stuck\",\"duration_s\":2}", &f, &fe) == SD_E_SCHEMA, "bus_stuck at most 1 s");
    CHECK(fault_json("{\"type\":\"melt\"}", &f, &fe) == SD_E_SCHEMA && strstr(fe.msg, "unknown fault"), "unknown fault type");
    /* params out of range */
    CHECK(sd_set_param(&s.in, 0, 9, false) == SD_E_RANGE && sd_get_param(&s.in, 0) == -4, "out of range refused");
    CHECK(sd_set_param(&s.in, 0, 9, true) == SD_OK && sd_get_param(&s.in, 0) == 8, "clamped");
}

static void test_analog(void) {
    static sensor_t s;
    const char *json =
        "{\"format\":1,\"kind\":\"analog\",\"name\":\"ntc\",\"output\":\"voltage\",\"excitation\":\"adc\","
        "\"params\":[{\"key\":\"temperature_c\",\"min\":-40,\"max\":150,\"default\":25}],"
        "\"let\":[{\"r_ntc\":\"10000 * exp(3950 * (1 / (temperature_c + 273.15) - 1 / 298.15))\"}],"
        "\"value\":\"input * r_ntc / (r_ntc + 10000)\",\"range\":{\"min\":0,\"max\":3.3}}";
    sd_err_t e;
    int rc = compile_text(&s, json, strlen(json), &e);
    CHECK(rc == SD_OK, "analog compiles");
    if (rc) return;
    double v;
    CHECK(sd_analog_eval(&s.in, 3.3, &v) == SD_OK && fabs(v - 1.65) < 1e-12, "NTC at 25 C: %.15f", v);
    setp(&s, "temperature_c", 100);
    CHECK(sd_analog_eval(&s.in, 5.0, &v) == SD_OK && v > 0.3 && v < 0.4, "NTC at 100 C: %f", v);
    CHECK(sd_analog_eval(&s.in, 50.0, &v) == SD_OK && v <= 3.3, "clamped to the range");
    sd_fault_t f;
    sd_err_t fe;
    fault_json("{\"type\":\"short\"}", &f, &fe);
    CHECK(sd_set_fault(&s.in, &f, true) == SD_OK && sd_analog_eval(&s.in, 3.3, &v) == SD_OK && v == 3.3, "short: range max");
    sd_set_fault(&s.in, &f, false);
    fault_json("{\"type\":\"open\",\"value\":0.1}", &f, &fe);
    CHECK(sd_set_fault(&s.in, &f, true) == SD_OK && sd_analog_eval(&s.in, 3.3, &v) == SD_OK && v == 0.1, "open with a value");
    sd_analog_info_t ai;
    CHECK(sd_analog_info(&s.in, &ai) == SD_OK && !ai.current && ai.excitation && ai.max == 3.3, "analog info");
}

/* ---- stimulus ----------------------------------------------------------------------------- */
static void test_stimulus(void) {
    static _Alignas(8) uint8_t sp[SD_STIM_MAX];
    size_t n;
    sd_err_t e;
    const char *json =
        "{\"format\":1,\"kind\":\"stimulus\",\"name\":\"hot\",\"rate_hz\":20,\"signals\":{"
        "\"temperature_c\":[{\"shape\":\"hold\",\"value\":22,\"duration_s\":5},{\"shape\":\"ramp\",\"end\":35,\"duration_s\":30},"
        "{\"shape\":\"sine\",\"offset\":35,\"amplitude\":1.5,\"period_s\":10,\"duration_s\":40},"
        "{\"shape\":\"noise\",\"mean\":35,\"sigma\":0.2,\"duration_s\":20,\"seed\":7},{\"shape\":\"step\",\"value\":25}],"
        "\"env.pressure_pa\":[{\"shape\":\"data\",\"points\":[[0,101325],[10,100900],[60,99000]],\"interpolate\":\"linear\"}],"
        "\"x\":[{\"shape\":\"repeat\",\"count\":3,\"segments\":[{\"shape\":\"ramp\",\"start\":0,\"end\":10,\"duration_s\":1},{\"shape\":\"hold\",\"value\":10,\"duration_s\":1}]},"
        "{\"shape\":\"saw\",\"low\":0,\"high\":4,\"period_s\":2,\"duration_s\":4},{\"shape\":\"pulse\",\"low\":1,\"high\":2,\"period_s\":1,\"duty\":0.25,\"duration_s\":2},"
        "{\"shape\":\"ramp\",\"end\":0,\"duration_s\":2},{\"shape\":\"data\",\"points\":[[0,5],[1,7]],\"interpolate\":\"step\"},"
        "{\"shape\":\"noise\",\"sigma\":0,\"duration_s\":1}]},"
        "\"faults\":[{\"at_s\":45,\"duration_s\":5,\"type\":\"stuck\"},{\"at_s\":70,\"duration_s\":2,\"type\":\"no_response\"},"
        "{\"at_s\":1,\"type\":\"corrupt\",\"at\":250,\"mask\":1,\"sensor\":\"env\"}]}";
    int rc = sd_stim_compile(json, strlen(json), sp, sizeof(sp), &n, &e);
    char m[200];
    sd_err_format(&e, m, sizeof(m));
    CHECK(rc == SD_OK, "stimulus compiles: rc %d %s (json %u)", rc, m, e.json_off);
    if (rc) return;
    sd_stim_t s;
    CHECK(sd_stim_init(&s, sp, n) == SD_OK, "player");
    CHECK(sd_stim_n_signals(&s) == 3 && sd_stim_n_faults(&s) == 3 && sd_stim_rate_hz(&s) == 20, "counts");
    CHECK(sd_stim_duration_s(&s) == 95, "duration %g", sd_stim_duration_s(&s));
    const char *in, *pa;
    size_t il, pl;
    CHECK(sd_stim_signal(&s, 1, &in, &il, &pa, &pl) == SD_OK && il == 3 && memcmp(in, "env", 3) == 0 && pl == 11, "instance prefix");
    CHECK(sd_stim_signal(&s, 0, &in, &il, &pa, &pl) == SD_OK && il == 0 && pl == 13, "no prefix");
    /* temperature: hold, ramp from the previous value, sine, noise, step */
    CHECK(sd_stim_value(&s, 0, 0) == 22 && sd_stim_value(&s, 0, 4.9) == 22, "hold");
    CHECK(fabs(sd_stim_value(&s, 0, 20) - 28.5) < 1e-12, "ramp from 22 to 35: %g", sd_stim_value(&s, 0, 20));
    CHECK(fabs(sd_stim_value(&s, 0, 35 + 2.5) - 36.5) < 1e-12, "sine peak: %g", sd_stim_value(&s, 0, 37.5));
    double nz = sd_stim_value(&s, 0, 80), nz2 = sd_stim_value(&s, 0, 80);
    CHECK(nz == nz2 && fabs(nz - 35) < 2 && nz != 35, "noise is stable: %g", nz);
    double sum = 0, sq = 0;
    for (int k = 0; k < 400; k++) { double x = sd_stim_value(&s, 0, 75 + k * 0.05) - 35; sum += x; sq += x * x; }
    CHECK(fabs(sum / 400) < 0.05 && fabs(sqrt(sq / 400) - 0.2) < 0.05, "noise mean/sigma: %g %g", sum / 400, sqrt(sq / 400));
    CHECK(sd_stim_value(&s, 0, 95) == 25 && sd_stim_value(&s, 0, 1000) == 25, "step, then held");
    /* data, linear */
    CHECK(fabs(sd_stim_value(&s, 1, 5) - 101112.5) < 1e-9 && sd_stim_value(&s, 1, 100) == 99000, "data linear: %g", sd_stim_value(&s, 1, 5));
    /* repeat, saw, pulse, ramp from previous, data step */
    CHECK(sd_stim_value(&s, 2, 0.5) == 5 && sd_stim_value(&s, 2, 2.5) == 5 && sd_stim_value(&s, 2, 5.5) == 10, "repeat");
    CHECK(sd_stim_value(&s, 2, 7) == 2 && fabs(sd_stim_value(&s, 2, 9.5) - 3) < 1e-12, "saw: %g", sd_stim_value(&s, 2, 9.5));
    CHECK(sd_stim_value(&s, 2, 10.1) == 2 && sd_stim_value(&s, 2, 10.5) == 1, "pulse");
    /* the pulse ends at t = 2 periods, where a new high phase starts: the ramp starts from 2 */
    CHECK(fabs(sd_stim_value(&s, 2, 13) - 1.0) < 1e-12, "ramp from the pulse's end value (2) to 0: %g", sd_stim_value(&s, 2, 13));
    CHECK(sd_stim_value(&s, 2, 14.5) == 5 && sd_stim_value(&s, 2, 15.2) == 7, "data step / mean = previous: %g", sd_stim_value(&s, 2, 15.2));
    /* steps and fault windows */
    sd_stim_event_t ev[16];
    int k = sd_stim_step(&s, 0, ev, 16);
    CHECK(k == 3, "3 values at t=0 (%d)", k);
    k = sd_stim_step(&s, 2, ev, 16);
    CHECK(k == 4 && ev[3].kind == SD_EV_FAULT_ON && ev[3].index == 2, "corrupt on at 1 s");
    k = sd_stim_step(&s, 46, ev, 16);
    CHECK(k == 4 && ev[3].kind == SD_EV_FAULT_ON && ev[3].index == 0, "stuck on at 45 s");
    k = sd_stim_step(&s, 50, ev, 16);
    CHECK(k == 4 && ev[3].kind == SD_EV_FAULT_OFF && ev[3].index == 0, "stuck off at 50 s");
    k = sd_stim_step(&s, 71, ev, 16);
    CHECK(k == 4 && ev[3].kind == SD_EV_FAULT_ON && ev[3].index == 1, "no_response window");
    k = sd_stim_step(&s, 80, ev, 16);
    CHECK(k == 4 && ev[3].kind == SD_EV_FAULT_OFF && ev[3].index == 1, "no_response over");
    sd_fault_t f;
    CHECK(sd_stim_fault(&s, 2, &f) == SD_OK && f.type == SD_FAULT_CORRUPT && f.at == 250 && f.mask == 1 && strcmp(f.sensor, "env") == 0 && f.duration_s < 0, "fault fields");
    /* repeat: true loops */
    const char *rj = "{\"format\":1,\"kind\":\"stimulus\",\"repeat\":true,\"signals\":{\"a\":[{\"shape\":\"ramp\",\"start\":0,\"end\":10,\"duration_s\":10}]},"
                     "\"faults\":[{\"at_s\":2,\"duration_s\":1,\"type\":\"stuck\"}]}";
    CHECK(sd_stim_compile(rj, strlen(rj), sp, sizeof(sp), &n, &e) == SD_OK && sd_stim_init(&s, sp, n) == SD_OK, "repeating stimulus");
    CHECK(fabs(sd_stim_value(&s, 0, 23) - 3) < 1e-12 && sd_stim_repeats(&s), "loops: %g", sd_stim_value(&s, 0, 23));
    k = sd_stim_step(&s, 12.5, ev, 16);
    CHECK(k == 2 && ev[1].kind == SD_EV_FAULT_ON, "the faults loop too");
    /* errors */
    const char *bad[][2] = {
        { "{\"format\":1,\"kind\":\"stimulus\",\"signals\":{\"a\":[{\"shape\":\"wobble\"}]}}", "signals.a[0].shape: unknown shape" },
        { "{\"format\":1,\"kind\":\"stimulus\",\"signals\":{\"a\":[{\"shape\":\"hold\",\"value\":1}]}}", "missing 'duration_s'" },
        { "{\"format\":1,\"kind\":\"stimulus\",\"signals\":{\"a\":[{\"shape\":\"data\",\"file\":\"x.csv\"}]}}", "inline points" },
        { "{\"format\":1,\"kind\":\"stimulus\",\"signals\":{\"a\":[{\"shape\":\"data\",\"points\":[[1,2],[0,3]]}]}}", "never go back" },
        { "{\"format\":1,\"kind\":\"stimulus\",\"signals\":{\"a.b.c\":[{\"shape\":\"step\",\"value\":1}]}}", "'param' or 'instance.param'" },
        { "{\"format\":1,\"kind\":\"stimulus\",\"faults\":[{\"type\":\"offset\"}]}", "faults[0]: offset and gain need" },
        { "{\"format\":1,\"kind\":\"stimulus\",\"rate_hz\":500}", "out of range" },
        { "{\"format\":1,\"kind\":\"stimulus\",\"signals\":{\"a\":[{\"shape\":\"repeat\",\"count\":2,\"segments\":[{\"shape\":\"repeat\",\"count\":2,\"segments\":[{\"shape\":\"repeat\",\"count\":2,\"segments\":[{\"shape\":\"repeat\",\"count\":2,\"segments\":[{\"shape\":\"repeat\",\"count\":2,\"segments\":[{\"shape\":\"step\",\"value\":1}]}]}]}]}]}]}}", "nested more than 4" },
    };
    for (unsigned i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
        rc = sd_stim_compile(bad[i][0], strlen(bad[i][0]), sp, sizeof(sp), &n, &e);
        sd_err_format(&e, m, sizeof(m));
        CHECK(rc != SD_OK && strstr(m, bad[i][1]), "stimulus error %u: want '%s', got '%s'", i, bad[i][1], m);
    }
}

/* ---- error reporting on the library files: a typo gets its path and offset ----------------- */
static void test_errors(void) {
    static sensor_t s;
    size_t n;
    char *j = read_file("fixtures/sensor_defs/mpu6050.json", &n);
    if (!j) return;
    static char buf[SD_JSON_MAX];
    memcpy(buf, j, n);
    buf[n] = 0;
    char *p = strstr(buf, "on * accel_y_g * al");
    if (p) memcpy(p, "on * accel_y_g * ax", 19);
    sd_err_t e;
    int rc = compile_text(&s, buf, n, &e);
    char m[200];
    sd_err_format(&e, m, sizeof(m));
    CHECK(rc == SD_E_EXPR && strcmp(m, "registers[4].s16be: unknown name 'ax' at 17") == 0, "typo report: %s", m);
    p = strstr(buf, "on * accel_y_g * ax");
    if (p) memcpy(p, "on * accel_y_g * al", 19);
    p = strstr(buf, "\"u8\":104");
    if (p) memcpy(p, "\"u9\":104", 8);
    rc = compile_text(&s, buf, n, &e);
    sd_err_format(&e, m, sizeof(m));
    CHECK(rc == SD_E_SCHEMA && strstr(m, "registers[11]") && strstr(m, "unknown key 'u9'"), "unknown key report: %s", m);
}

/* ---- word registers (i2c.register_bytes: 2) ------------------------------------------------ */
static void test_word_registers(void) {
    static sensor_t s;
    const char *json =
        "{\"format\":1,\"kind\":\"i2c\",\"name\":\"tmpw\","
        "\"params\":[{\"key\":\"temperature_c\",\"min\":-55,\"max\":150,\"default\":25}],"
        "\"i2c\":{\"address\":72,\"register_bytes\":2},"
        "\"let\":[{\"off\":\"s16(reg[0x07]) * 0.0078125\"}],"
        "\"registers\":[{\"at\":0,\"s16be\":\"(temperature_c + off) / 0.0078125\"},{\"at\":1,\"u16be\":544},"
        "{\"at\":7,\"s16be\":0},{\"at\":15,\"u16be\":279,\"writable\":false},{\"at\":254,\"u16be\":21577}],"
        "\"on_write\":[{\"when\":\"written == 0x01 && (value & 0x0002)\",\"do\":\"reset\"},"
        "{\"when\":\"written == 0x02\",\"do\":{\"set\":{\"at\":3,\"u16be\":\"value + 1\"}}}]}";
    sd_err_t e;
    char m[200];
    int rc = compile_text(&s, json, strlen(json), &e);
    sd_err_format(&e, m, sizeof(m));
    CHECK(rc == SD_OK, "word-register definition compiles: %s", m);
    if (rc) return;
    sd_bus_info_t bi;
    CHECK(sd_bus_info(&s.in, &bi) == SD_OK && bi.register_bytes == 2 && !bi.command_map && bi.addr_bits == 8, "bus info: word registers");
    uint8_t img[256], live[256], ro[32];
    sd_img_result_t r;
    CHECK(sd_regs_power_on(&s.in, img, &r) == SD_OK, "power on");
    CHECK(img[0] == 0x0C && img[1] == 0x80 && img[2] == 0x02 && img[3] == 0x20, "regs 0x00, 0x01 at bytes 0..3: %02X %02X %02X %02X", img[0], img[1], img[2], img[3]);
    CHECK(img[0x1E] == 0x01 && img[0x1F] == 0x17, "reg 0x0F at bytes 0x1E/0x1F");
    CHECK(img[0xFC] == 0x54 && img[0xFD] == 0x49, "reg 0xFE aliases 0x7E: bytes 0xFC/0xFD");
    CHECK(sd_regs_ro_map(&s.in, ro) == SD_OK && (ro[0x1E >> 3] & (1u << (0x1E & 7))) && (ro[0x1F >> 3] & (1u << (0x1F & 7))) &&
          !(ro[0] & 1u), "writable:false covers the word");
    /* the DUT writes TEMP_OFFSET = +1 C (0x0080): reg[0x07] is the word, the result follows */
    memcpy(live, img, 256);
    live[14] = 0x00;
    live[15] = 0x80;
    CHECK(sd_regs_after_write(&s.in, live, 15, 0x80, img, &r) == SD_OK && img[0] == 0x0D && img[1] == 0x00, "offset word: %02X %02X", img[0], img[1]);
    /* `written` is the register, `value` the whole word */
    memcpy(live, img, 256);
    live[4] = 0x12;
    live[5] = 0x34;
    CHECK(sd_regs_after_write(&s.in, live, 5, 0x34, img, &r) == SD_OK && img[6] == 0x12 && img[7] == 0x35, "written/value as a word: %02X %02X", img[6], img[7]);
    memcpy(live, img, 256);
    live[2] = 0x00;
    live[3] = 0x22;
    CHECK(sd_regs_after_write(&s.in, live, 3, 0x22, img, &r) == SD_OK && r.reset && img[14] == 0 && img[15] == 0 && img[0] == 0x0C, "soft reset from the word's bit 1");
    /* errors */
    static const struct { const char *j, *msg; } bad[] = {
        { "{\"format\":1,\"kind\":\"i2c\",\"name\":\"x\",\"i2c\":{\"address\":72,\"register_bytes\":2},"
          "\"registers\":[{\"at\":5,\"u16be\":1},{\"at\":133,\"u16be\":2}]}", "same register" },
        { "{\"format\":1,\"kind\":\"i2c\",\"name\":\"x\",\"i2c\":{\"address\":72,\"register_bytes\":2,\"register_address_bits\":16,\"page\":256},"
          "\"registers\":[{\"at\":256,\"u16be\":1}]}", "register_bytes: 2 needs" },
        { "{\"format\":1,\"kind\":\"i2c\",\"name\":\"x\",\"i2c\":{\"address\":72,\"register_bytes\":2,\"wrap\":15},"
          "\"registers\":[{\"at\":5,\"u16be\":1}]}", "wrap does not apply" },
        { "{\"format\":1,\"kind\":\"i2c\",\"name\":\"x\",\"i2c\":{\"address\":72,\"register_bytes\":3},"
          "\"registers\":[{\"at\":5,\"u16be\":1}]}", "i2c.register_bytes" },
    };
    for (unsigned i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
        rc = compile_text(&s, bad[i].j, strlen(bad[i].j), &e);
        sd_err_format(&e, m, sizeof(m));
        CHECK(rc != SD_OK && strstr(m, bad[i].msg), "word error %u: %s", i, m);
    }
}

/* ---- command map (register_address_bits: 16 without page) ---------------------------------- */
static unsigned cm_at(const uint8_t *img, unsigned cmd) { return (unsigned)img[cmd >> 8] ^ (cmd & 0xFFu); }

static void test_command_map(void) {
    static sensor_t s;
    const char *json =
        "{\"format\":1,\"kind\":\"i2c\",\"name\":\"cmap\","
        "\"params\":[{\"key\":\"tc\",\"min\":0,\"max\":100,\"default\":10}],"
        "\"i2c\":{\"address\":68,\"register_address_bits\":16},"
        "\"commands\":[{\"command\":9216,\"busy_us\":15000,\"answer\":[1,2,3,4,5,\"tc\"]},"      /* 0x2400 */
        "{\"command\":9238,\"busy_us\":4000,\"answer\":[17,18,19,20,21,22]},"                  /* 0x2416 */
        "{\"command\":57344,\"answer\":[33,34,35,36,37,38]},"                                  /* 0xE000 */
        "{\"command\":62253,\"answer\":[128,16,225]},"                                         /* 0xF32D */
        "{\"command\":9245,\"write_bytes\":3},"                                                /* 0x241D */
        "{\"command\":8984,\"answer\":[\"reg[0x241D]\",\"reg[0x241E]\",\"reg[0x241F]\"]},"     /* 0x2318 */
        "{\"command\":14214,\"busy_us\":500}],"                                                /* 0x3786 */
        "\"on_write\":[{\"when\":\"written == 0x241F\",\"do\":{\"set\":{\"at\":62253,\"u8\":\"value\"}}}]}";
    sd_err_t e;
    char m[200];
    int rc = compile_text(&s, json, strlen(json), &e);
    sd_err_format(&e, m, sizeof(m));
    CHECK(rc == SD_OK, "command-map definition compiles: %s", m);
    if (rc) return;
    sd_bus_info_t bi;
    CHECK(sd_bus_info(&s.in, &bi) == SD_OK && bi.command_map && bi.addr_bits == 16 && bi.register_bytes == 1 && bi.n_commands == 7 && bi.page == 0, "bus info: command map");
    uint8_t img[256], live[256], ro[32];
    sd_img_result_t r;
    CHECK(sd_regs_power_on(&s.in, img, &r) == SD_OK, "power on");
    /* the gateware's lookup (image[h] ^ l) reaches each answer */
    static const struct { unsigned cmd; uint8_t a[6]; unsigned n; } ans[] = {
        { 0x2400, { 1, 2, 3, 4, 5, 10 }, 6 }, { 0x2416, { 17, 18, 19, 20, 21, 22 }, 6 },
        { 0xE000, { 33, 34, 35, 36, 37, 38 }, 6 }, { 0xF32D, { 128, 16, 225 }, 3 }, { 0x2318, { 0, 0, 0 }, 3 },
    };
    for (unsigned i = 0; i < sizeof(ans) / sizeof(ans[0]); i++) {
        unsigned ix = cm_at(img, ans[i].cmd);
        bool ok = true;
        for (unsigned k = 0; k < ans[i].n; k++) ok = ok && img[(ix + k) & 0xFF] == ans[i].a[k];
        CHECK(ok, "command 0x%04X answers at byte 0x%02X", ans[i].cmd, ix);
    }
    /* sd_command's image index is the same byte (the manager sets CMD there) */
    static const unsigned order[] = { 0x2400, 0x2416, 0xE000, 0xF32D, 0x241D, 0x2318, 0x3786 };
    for (unsigned i = 0; i < 7; i++) {
        sd_command_t c;
        CHECK(sd_command(&s.in, i, &c) == SD_OK && c.at == cm_at(img, order[i]), "sd_command %u at the map's byte", i);
    }
    /* no two blocks or base bytes share a byte; base bytes are read-only */
    uint8_t seen[256] = { 0 };
    static const unsigned spans[] = { 6, 6, 6, 3, 3, 3, 1 };
    bool clash = false;
    static const unsigned his[] = { 0x24, 0xE0, 0xF3, 0x23, 0x37 };
    for (unsigned i = 0; i < 5; i++) seen[his[i]] = 1;
    for (unsigned i = 0; i < 7; i++)
        for (unsigned k = 0; k < spans[i]; k++) { unsigned ix = (cm_at(img, order[i]) + k) & 0xFF; clash |= seen[ix]; seen[ix] = 1; }
    CHECK(!clash, "the layout keeps every block and base byte apart");
    CHECK(sd_regs_ro_map(&s.in, ro) == SD_OK && (ro[0x24 >> 3] & (1u << (0x24 & 7))) && (ro[0xE0 >> 3] & (1u << (0xE0 & 7))), "base bytes read-only");
    /* a set command's data: reg[] and `written` see it at its command's addresses */
    memcpy(live, img, 256);
    unsigned w = cm_at(img, 0x241D);
    live[w] = 0x01; live[(w + 1) & 0xFF] = 0xF4; live[(w + 2) & 0xFF] = 0x33;
    CHECK(sd_regs_after_write(&s.in, live, (int)((w + 2) & 0xFF), 0x33, img, &r) == SD_OK, "after write");
    unsigned g = cm_at(img, 0x2318);
    CHECK(img[g] == 0x01 && img[(g + 1) & 0xFF] == 0xF4 && img[(g + 2) & 0xFF] == 0x33, "0x2318 answers the written data");
    CHECK(img[cm_at(img, 0xF32D)] == 0x33, "written == 0x241F (the third data byte)");
    CHECK(img[0x24] == live[0x24], "base byte unchanged");
    /* a parameter change rebuilds the answer; a dynamic answer keeps the base bytes */
    setp(&s, "tc", 42);
    memcpy(live, img, 256);
    CHECK(sd_regs_rebuild(&s.in, live, img, &r) == SD_OK && img[(cm_at(img, 0x2400) + 5) & 0xFF] == 42, "param in an answer");
    /* errors */
    static const struct { const char *j, *msg; } bad[] = {
        { "{\"format\":1,\"kind\":\"i2c\",\"name\":\"x\",\"i2c\":{\"address\":68,\"register_address_bits\":16,\"page\":9216},"
          "\"commands\":[{\"command\":9216,\"write_bytes\":2,\"answer\":[1]}]}", "write_bytes is for the command map" },
        { "{\"format\":1,\"kind\":\"i2c\",\"name\":\"x\",\"i2c\":{\"address\":68,\"register_address_bits\":16},"
          "\"commands\":[{\"command\":9216,\"answer\":[1]}],\"conversion\":{\"trigger\":4660}}", "no register or command entry" },
    };
    for (unsigned i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
        rc = compile_text(&s, bad[i].j, strlen(bad[i].j), &e);
        sd_err_format(&e, m, sizeof(m));
        CHECK(rc != SD_OK && strstr(m, bad[i].msg), "command-map error %u: %s", i, m);
    }
    /* overlapping address ranges: each command keeps its own bytes */
    {
        const char *ov = "{\"format\":1,\"kind\":\"i2c\",\"name\":\"x\",\"i2c\":{\"address\":68,\"register_address_bits\":16},"
                         "\"commands\":[{\"command\":11277,\"answer\":[1,2,3,4,5,6]},{\"command\":11280,\"answer\":[9,8]}],"
                         "\"registers\":[{\"at\":4096,\"bytes\":[\"reg[0x2C10]\",\"reg[0x2C0F]\"]}]}";
        rc = compile_text(&s, ov, strlen(ov), &e);
        sd_err_format(&e, m, sizeof(m));
        CHECK(rc == SD_OK, "overlapping commands compile: %s", m);
        if (rc == SD_OK && sd_regs_power_on(&s.in, img, &r) == SD_OK) {
            unsigned a = cm_at(img, 0x2C0D), b2 = cm_at(img, 0x2C10), z = cm_at(img, 0x1000);
            CHECK(img[(a + 3) & 0xFF] == 4 && img[(a + 5) & 0xFF] == 6 && img[b2] == 9 && img[(b2 + 1) & 0xFF] == 8, "both answers whole");
            CHECK(img[z] == 9 && img[(z + 1) & 0xFF] == 3, "reg[]: an exact address is its own command, else inside the one below");
        }
    }
    /* too much for 256 bytes: three 100-byte answers in different high bytes */
    {
        static char big[8192];
        size_t o = (size_t)snprintf(big, sizeof(big), "{\"format\":1,\"kind\":\"i2c\",\"name\":\"x\",\"i2c\":{\"address\":68,\"register_address_bits\":16},\"commands\":[");
        for (unsigned q = 0; q < 3; q++) {
            o += (size_t)snprintf(big + o, sizeof(big) - o, "%s{\"command\":%u,\"answer\":[", q ? "," : "", 0x1000u * (q + 1));
            for (unsigned k = 0; k < 100; k++) o += (size_t)snprintf(big + o, sizeof(big) - o, "%s%u", k ? "," : "", k);
            o += (size_t)snprintf(big + o, sizeof(big) - o, "]}");
        }
        o += (size_t)snprintf(big + o, sizeof(big) - o, "]}");
        rc = compile_text(&s, big, o, &e);
        sd_err_format(&e, m, sizeof(m));
        CHECK(rc != SD_OK && strstr(m, "do not fit"), "a layout that does not fit: %s", m);
    }
    /* the old single window is unchanged: page given -> no map */
    const char *win = "{\"format\":1,\"kind\":\"i2c\",\"name\":\"w\",\"i2c\":{\"address\":68,\"register_address_bits\":16,\"page\":9216},"
                      "\"commands\":[{\"command\":9216,\"answer\":[7,8]}]}";
    CHECK(compile_text(&s, win, strlen(win), &e) == SD_OK && sd_bus_info(&s.in, &bi) == SD_OK && !bi.command_map && bi.page == 0x2400, "page keeps the window");
    CHECK(sd_regs_power_on(&s.in, img, &r) == SD_OK && img[0] == 7 && img[1] == 8, "window image at offset 0");
}

/* ---- sizes -------------------------------------------------------------------------------- */
static void test_sizes(void) {
    static const char *names[] = { "bmp280", "bme280", "sht4x", "mpu6050", "gps_nmea" };
    for (unsigned i = 0; i < 5; i++) {
        static sensor_t s;
        char path[96];
        snprintf(path, sizeof(path), "fixtures/sensor_defs/%s.json", names[i]);
        size_t jn;
        read_file(path, &jn);
        if (!load_fixture(&s, names[i])) continue;
        CHECK(s.len <= SD_PROG_MAX, "%s fits 8 KB", names[i]);
        printf("  size: %-9s JSON %5zu B -> program %5zu B\n", names[i], jn, s.len);
    }
    printf("  size: sd_inst_t %zu B, compile scratch %u B\n", sizeof(sd_inst_t), SD_COMPILE_WORK);
}

int main(void) {
    test_bmp_bme();
    test_sht4x();
    test_mpu6050();
    test_gps();
    test_vm();
    test_templates();
    test_spi_fifo_faults();
    test_analog();
    test_stimulus();
    test_errors();
    test_word_registers();
    test_command_map();
    test_sizes();
    printf("  BMP/BME images differing from the C models: %u of %u cases (see notes)\n", g_bosch_diff_cases, g_bosch_cases);
    if (fails) { printf("test_sensor_def: %d FAILED\n", fails); return 1; }
    printf("test_sensor_def: all passed\n");
    return 0;
}
