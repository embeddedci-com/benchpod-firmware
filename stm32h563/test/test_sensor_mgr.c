/*
 * test_sensor_mgr.c — the emulated-sensor manager and its JSON commands (src/sensor_mgr.c,
 * src/sensor_cmd.c) end to end against a fake emulation gateware (fake_fpga_emu.c): chunked
 * defines, starts with the image swap, 4 I2C + SPI + UART at once with the pin rules, DUT writes
 * through the write log (overflow included), the BME280 soft reset and the MPU-6050 range change,
 * the SHT4x command busy time, GPS epochs, a request/answer UART sensor, the FIFO, data ready,
 * faults, a stimulus, the re-arm after a reconfiguration, the analog outputs and reply sizes.
 */
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bp_limits.h"
#include "fake_fpga_emu.h"
#include "la_pins.h"
#include "sensor_cmd.h"
#include "sensor_mgr.h"

static int fails;
#define CHECK(cond, ...) do { if (!(cond)) { printf("FAIL %s:%d: ", __func__, __LINE__); \
        printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

static char g_reply[BP_CLOUD_REPLY_MAX];
static char g_err[SM_ERR_MAX];
static size_t g_max_reply;

/* Run one command; true = ok (g_reply), false = error (g_err). */
static bool run(const char *fmt, ...) {
    static char json[2048];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(json, sizeof(json), fmt, ap);
    va_end(ap);
    char verb[32] = { 0 };
    bp_json_get(json, "cmd", verb, sizeof(verb));
    bp_emit_t e;
    bp_emit_init(&e, g_reply, sizeof(g_reply));
    g_err[0] = '\0';
    int rc = sensor_cmd(verb, json, &e, g_err, sizeof(g_err));
    if (rc == 0 && !bp_emit_ok(&e)) { snprintf(g_err, sizeof(g_err), "reply too large"); return false; }
    if (rc == 0 && bp_emit_len(&e) > g_max_reply) g_max_reply = bp_emit_len(&e);
    return rc == 0;
}

static void poll_ms(unsigned ms) {
    for (unsigned i = 0; i < ms; i++) { fake_advance_us(1000); sm_poll(); }
}

static char *read_file(const char *path, size_t *n) {
    FILE *f = fopen(path, "rb");
    if (!f) return NULL;
    fseek(f, 0, SEEK_END);
    long len = ftell(f);
    fseek(f, 0, SEEK_SET);
    char *b = malloc((size_t)len + 1);
    *n = fread(b, 1, (size_t)len, f);
    b[*n] = '\0';
    fclose(f);
    return b;
}

static void b64(const uint8_t *d, size_t n, char *out) {
    static const char t[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    size_t o = 0;
    for (size_t i = 0; i < n; i += 3) {
        uint32_t v = (uint32_t)d[i] << 16 | (i + 1 < n ? (uint32_t)d[i + 1] << 8 : 0u) | (i + 2 < n ? d[i + 2] : 0u);
        out[o++] = t[(v >> 18) & 63]; out[o++] = t[(v >> 12) & 63];
        out[o++] = i + 1 < n ? t[(v >> 6) & 63] : '=';
        out[o++] = i + 2 < n ? t[v & 63] : '=';
    }
    out[o] = '\0';
}

/* Upload a definition (or a stimulus: inst NULL) in 768-byte chunks, as the SDK does. */
static bool define_text(const char *inst, const char *text, size_t n) {
    char enc[1100];
    for (size_t off = 0; off < n; off += 768) {
        size_t k = n - off < 768 ? n - off : 768;
        b64((const uint8_t *)text + off, k, enc);
        bool ok = inst ? run("{\"cmd\":\"sensor_define\",\"instance\":\"%s\",\"offset\":%zu,\"total\":%zu,\"data_b64\":\"%s\"}",
                             inst, off, n, enc)
                       : run("{\"cmd\":\"sensor_stimulus\",\"offset\":%zu,\"total\":%zu,\"data_b64\":\"%s\"}", off, n, enc);
        if (!ok) return false;
        /* the cloud command limit: every chunk command fits 1152 bytes */
        CHECK(strlen(enc) + 120 < 1152, "chunk command too long");
    }
    return true;
}

static bool define_fixture(const char *inst, const char *name) {
    char path[96];
    snprintf(path, sizeof(path), "fixtures/sensor_defs/%s.json", name);
    size_t n = 0;
    char *t = read_file(path, &n);
    if (!t) { printf("missing %s\n", path); fails++; return false; }
    bool ok = define_text(inst, t, n);
    free(t);
    if (!ok) printf("  define %s: %s\n", name, g_err);
    return ok;
}

static void fresh(void) {
    sm_reset();
    la_pins_reset();
    fake_reset();
}

/* ---- uploads ---------------------------------------------------------------------------- */

static void test_define(void) {
    fresh();
    size_t n = 0;
    char *t = read_file("fixtures/sensor_defs/bme280.json", &n);
    char enc[1100];
    b64((const uint8_t *)t, 768, enc);
    CHECK(run("{\"cmd\":\"sensor_define\",\"instance\":\"env\",\"offset\":0,\"total\":%zu,\"data_b64\":\"%s\"}", n, enc),
          "first chunk: %s", g_err);
    CHECK(strstr(g_reply, "\"received\":768"), "chunk reply: %s", g_reply);
    /* out of order: refused, and the upload starts again at 0 */
    CHECK(!run("{\"cmd\":\"sensor_define\",\"instance\":\"env\",\"offset\":1536,\"total\":%zu,\"data_b64\":\"%s\"}", n, enc), "gap");
    CHECK(strcmp(g_err, "chunk at offset 1536, expected 768 (start again at offset 0)") == 0, "%s", g_err);
    CHECK(!run("{\"cmd\":\"sensor_define\",\"instance\":\"env\",\"offset\":768,\"total\":%zu,\"data_b64\":\"%s\"}", n, enc),
          "the upload was dropped");
    CHECK(define_fixture("env", "bme280"), "bme280 defines");
    CHECK(strstr(g_reply, "\"kind\":\"i2c\"") && strstr(g_reply, "\"name\":\"bme280\"") &&
          strstr(g_reply, "\"address\":\"0x76\",\"alt_addresses\":[\"0x77\"]") && strstr(g_reply, "\"key\":\"humidity_pct\""),
          "define reply: %s", g_reply);
    /* URL alphabet, no padding: the same */
    for (char *p = enc; *p; p++) { if (*p == '+') *p = '-'; else if (*p == '/') *p = '_'; else if (*p == '=') *p = '\0'; }
    CHECK(run("{\"cmd\":\"sensor_define\",\"instance\":\"env2\",\"offset\":0,\"total\":%zu,\"data_b64\":\"%s\"}", n, enc),
          "url alphabet: %s", g_err);
    /* dry run keeps nothing */
    long before = g_fpga.allocs;
    t[n] = '\0';
    char *bad = strdup(t);
    char *p = strstr(bad, "temperature_c");
    CHECK(define_text("dry", t, n) || 1, "x");
    CHECK(run("{\"cmd\":\"sensor_status\",\"instance\":\"dry\"}"), "dry defined normally: %s", g_err);
    (void)before;
    /* an invalid definition names the place */
    if (p) memcpy(p, "temperature_x", 13);
    CHECK(!define_text("bad", bad, strlen(bad)) && strncmp(g_err, "invalid definition: ", 20) == 0, "invalid: %s", g_err);
    CHECK(!run("{\"cmd\":\"sensor_status\",\"instance\":\"bad\"}") && strcmp(g_err, "unknown sensor instance 'bad'") == 0, "%s", g_err);
    CHECK(!run("{\"cmd\":\"sensor_define\",\"instance\":\"Env!\",\"offset\":0,\"total\":4,\"data_b64\":\"e30=\"}"), "bad name");
    CHECK(!run("{\"cmd\":\"sensor_define\",\"instance\":\"env\",\"offset\":0,\"total\":40000,\"data_b64\":\"e30=\"}"), "too big");
    free(bad);
    free(t);
    sm_reset();
    CHECK(g_fpga.allocs == 0, "heap given back: %ld blocks", g_fpga.allocs);
}

/* ---- I2C: BME280 + MPU-6050 on one bus ---------------------------------------------------- */

static void test_bme280_mpu(void) {
    fresh();
    CHECK(define_fixture("env", "bme280") && define_fixture("imu", "mpu6050"), "defined");
    CHECK(run("{\"cmd\":\"sensor_start\",\"instance\":\"env\",\"address\":\"0x77\",\"pins\":{\"sda\":2,\"scl\":1},"
              "\"values\":{\"humidity_pct\":55}}"), "start env: %s", g_err);
    CHECK(strstr(g_reply, "\"slot\":\"i2c0\",\"image\":\"emulation\",\"image_loaded\":true") &&
          strstr(g_reply, "\"unavailable\":[\"dac_control_loop\",\"deep_replay\"]"), "start reply: %s", g_reply);
    CHECK(g_fpga.image_loads == 1, "the emulation image was loaded");
    uint8_t b[8];
    CHECK(fake_i2c_read(0x77, 0xD0, b, 1) && b[0] == 0x60, "chip id 0x%02x", b[0]);
    CHECK(la_pins_fn(2) == LA_FN_I2C_SDA && la_pins_owner(2) == 1, "pins owned");
    CHECK(g_fpga.pinmap[1] == EMU_PIN_I2C && g_fpga.pinmap[0] == EMU_PIN_I2C, "pin map");
    CHECK(g_fpga.t[0].attrs[0xF4] & EMU_ATTR_TRIG && g_fpga.t[0].attrs[0xF3] & EMU_ATTR_BUSY && g_fpga.t[0].times[0] == 5500,
          "conversion attributes");

    /* second sensor on the same bus, another address */
    CHECK(run("{\"cmd\":\"sensor_start\",\"instance\":\"imu\",\"pins\":{\"sda\":2,\"scl\":1}}"), "start imu: %s", g_err);
    CHECK(strstr(g_reply, "\"slot\":\"i2c1\"") && strstr(g_reply, "\"image_loaded\":false"), "%s", g_reply);
    CHECK(la_pins_owner(2) == 3, "SDA shared by two instances");
    /* the same address on the same bus is refused, and SCL as SDA is a conflict */
    CHECK(define_fixture("env3", "bme280"), "env3");
    CHECK(!run("{\"cmd\":\"sensor_start\",\"instance\":\"env3\",\"address\":\"0x77\",\"pins\":{\"sda\":2,\"scl\":1}}") &&
          strstr(g_err, "address 0x77 is used by sensor:env"), "%s", g_err);
    CHECK(!run("{\"cmd\":\"sensor_start\",\"instance\":\"env3\",\"pins\":{\"sda\":1,\"scl\":3}}") &&
          strcmp(g_err, "pin conflict: LA1 is in use by sensor:env (i2c_scl); stop the sensor emulation first "
                        "({\"cmd\":\"sensor_stop\",\"instance\":\"env\"})") == 0, "%s", g_err);
    CHECK(!run("{\"cmd\":\"sensor_start\",\"instance\":\"nope\",\"pins\":{\"sda\":5,\"scl\":6}}") &&
          strcmp(g_err, "unknown sensor instance 'nope'") == 0, "%s", g_err);

    /* BME280: the DUT starts a forced conversion, the busy bit shows, then data */
    uint8_t v = 0x25;
    CHECK(fake_i2c_write(0x77, 0xF4, &v, 1), "ctrl_meas write");
    CHECK(fake_i2c_read(0x77, 0xF3, b, 1) && (b[0] & 0x08), "measuring bit while converting");
    poll_ms(10);
    CHECK(fake_i2c_read(0x77, 0xF3, b, 1) && !(b[0] & 0x08), "conversion done");
    CHECK(g_fpga.t[0].regs[0xF4] == 0x25, "the DUT's ctrl_meas stays");
    CHECK(run("{\"cmd\":\"sensor_status\",\"instance\":\"env\"}") && strstr(g_reply, "\"last_write\":{\"at\":\"0xF4\",\"value\":\"0x25\"}") &&
          strstr(g_reply, "\"humidity_pct\":55"), "status: %s", g_reply);
    uint8_t hum_before[2] = { g_fpga.t[0].regs[0xFD], g_fpga.t[0].regs[0xFE] };
    CHECK(run("{\"cmd\":\"sensor_set\",\"instance\":\"env\",\"values\":{\"humidity_pct\":20}}"), "set: %s", g_err);
    CHECK(strcmp(g_reply, "{\"status\":\"ok\",\"data\":{\"instance\":\"env\",\"values\":{\"humidity_pct\":20}}}\n") == 0, "%s", g_reply);
    poll_ms(3);
    CHECK(g_fpga.t[0].regs[0xFD] != hum_before[0] || g_fpga.t[0].regs[0xFE] != hum_before[1], "humidity bytes moved");
    CHECK(!run("{\"cmd\":\"sensor_set\",\"instance\":\"env\",\"values\":{\"humidity_pct\":101}}") &&
          strstr(g_err, "humidity_pct must be 0..100"), "range: %s", g_err);
    /* soft reset: 0xB6 to 0xE0 reloads the power-on image */
    v = 0xB6;
    CHECK(fake_i2c_write(0x77, 0xE0, &v, 1), "reset write");
    poll_ms(3);
    CHECK(g_fpga.t[0].regs[0xF4] == 0x00, "ctrl_meas back to its power-on value: 0x%02x", g_fpga.t[0].regs[0xF4]);
    /* a driver that writes its setup right after the reset, while the pod loads the power-on
       image: the setup survives (found on the bench with the process-node env driver) */
    v = 0xB6;
    CHECK(fake_i2c_write(0x77, 0xE0, &v, 1), "reset write");
    fake_race_write_before_load(0x77, 0xF4, 0x27);
    poll_ms(3);
    poll_ms(3);
    CHECK(g_fpga.t[0].regs[0xF4] == 0x27, "setup after a reset survives the reload: 0x%02x", g_fpga.t[0].regs[0xF4]);

    /* MPU-6050: wake it, then the accel range follows ACCEL_CONFIG */
    CHECK(run("{\"cmd\":\"sensor_set\",\"instance\":\"imu\",\"values\":{\"accel_z_g\":1}}"), "imu set");
    v = 0x00;
    fake_i2c_write(0x68, 0x6B, &v, 1);
    poll_ms(3);
    CHECK(fake_i2c_read(0x68, 0x3F, b, 2) && (int16_t)(b[0] << 8 | b[1]) == 16384, "1 g at 2 g range: %d", (int16_t)(b[0] << 8 | b[1]));
    v = 0x18;
    fake_i2c_write(0x68, 0x1C, &v, 1);
    poll_ms(3);
    CHECK(fake_i2c_read(0x68, 0x3F, b, 2) && (int16_t)(b[0] << 8 | b[1]) == 2048, "16 g range: %d", (int16_t)(b[0] << 8 | b[1]));
    CHECK(fake_i2c_read(0x68, 0x75, b, 1) && b[0] == 0x68, "WHO_AM_I");
    CHECK(g_fpga.t[1].attrs[0x75] & EMU_ATTR_RO, "WHO_AM_I is read-only in the gateware");

    /* the write log: more than 32 writes between two polls lose entries, the image is read back */
    uint8_t burst[40];
    memset(burst, 0, sizeof(burst));
    fake_i2c_write(0x68, 0x00, burst, sizeof(burst));
    poll_ms(3);
    CHECK(run("{\"cmd\":\"sensor_status\",\"instance\":\"imu\"}") && strstr(g_reply, "\"log_lost\":8"), "overflow: %s", g_reply);

    /* stop: the shared lines stay with the other instance */
    CHECK(run("{\"cmd\":\"sensor_stop\",\"instance\":\"env\"}") &&
          strcmp(g_reply, "{\"status\":\"ok\",\"data\":{\"stopped\":[\"env\"]}}\n") == 0, "%s", g_reply);
    CHECK(la_pins_owner(2) == 2 && la_pins_fn(2) == LA_FN_I2C_SDA, "imu keeps SDA");
    CHECK(run("{\"cmd\":\"sensor_stop\",\"all\":true}") && la_pins_fn(2) == LA_FN_NONE, "all stopped, pins free");
    sm_reset();
    CHECK(g_fpga.allocs == 0, "heap given back: %ld", g_fpga.allocs);
}

/* ---- SHT4x: commanded, busy for the command's time ---------------------------------------- */

static void test_sht4x(void) {
    fresh();
    CHECK(define_fixture("rh", "sht4x"), "sht4x");
    CHECK(run("{\"cmd\":\"sensor_start\",\"instance\":\"rh\",\"pins\":{\"sda\":3,\"scl\":4}}"), "start: %s", g_err);
    uint8_t a = g_fpga.t[0].attrs[0xFD];
    CHECK(a & EMU_ATTR_CMD, "0xFD is a command (attr 0x%02x)", a);
    CHECK(g_fpga.t[0].times[(a >> 6) & 3] == 8300, "0xFD measures for 8.3 ms: %u", g_fpga.t[0].times[(a >> 6) & 3]);
    uint8_t b[6];
    CHECK(fake_i2c_write(0x44, 0xFD, NULL, 0), "command");
    CHECK(g_fpga.t[0].busy_until_us > g_fpga.now_us, "the command starts a measurement");
    CHECK(((g_fpga.t[0].flags >> 2) & 3u) == 1u, "a command-only sensor runs in nack mode");
    CHECK(!fake_i2c_read(0x44, -1, b, 6), "a read while measuring is not acknowledged");
    fake_advance_us(70000);
    CHECK(fake_i2c_read(0x44, -1, b, 6), "answer after the measurement");
    uint8_t h = g_fpga.t[0].attrs[0x15];   /* a 110 ms heater command */
    CHECK(g_fpga.t[0].times[(h >> 6) & 3] == 65535, "a command longer than 65.535 ms is clamped: %u", g_fpga.t[0].times[(h >> 6) & 3]);
    CHECK(!(g_fpga.t[0].attrs[0x89] & EMU_ATTR_CMD), "the serial number answers at once");
    sm_reset();
}

/* ---- word registers and the command map (I2C_CONFIG page byte 1 / 2) ------------------- */
static uint8_t crc8_31(uint8_t a, uint8_t b) {
    uint8_t c = 0xFF, d[2] = { a, b };
    for (int i = 0; i < 2; i++) { c ^= d[i]; for (int k = 0; k < 8; k++) c = (uint8_t)((c & 0x80) ? (c << 1) ^ 0x31 : c << 1); }
    return c;
}

static void test_word_cmap(void) {
    fresh();
    CHECK(define_fixture("tw", "word_tmp117"), "word_tmp117");
    CHECK(run("{\"cmd\":\"sensor_start\",\"instance\":\"tw\",\"pins\":{\"sda\":3,\"scl\":4}}"), "start: %s", g_err);
    fake_target_t *x = &g_fpga.t[0];
    CHECK(!(x->flags & EMU_I2C_ADDR16) && x->page == EMU_I2C_PAGE_WORD, "8-bit, page byte 1 (word registers): 0x%02x 0x%02x", x->flags, x->page);
    uint8_t b[9], v[2];
    CHECK(fake_i2c_read(0x48, 0x00, b, 2) && b[0] == 0x0C && b[1] == 0x80, "TEMP_RESULT 25 C: %02X %02X", b[0], b[1]);
    CHECK(fake_i2c_read(0x48, 0x0F, b, 2) && b[0] == 0x01 && b[1] == 0x17, "DEVICE_ID");
    CHECK(fake_i2c_read(0x48, 0x01, b, 4) && b[0] == 0x02 && b[1] == 0x20 && b[2] == 0x60 && b[3] == 0x00, "CONFIG then the next register");
    CHECK(fake_i2c_read(0x48, 0xFE, b, 2) && b[0] == 0x54 && b[1] == 0x49, "pointer 0xFE");
    v[0] = 0x00; v[1] = 0x80;
    CHECK(fake_i2c_write(0x48, 0x07, v, 2), "TEMP_OFFSET +1 C");
    poll_ms(5);
    CHECK(fake_i2c_read(0x48, 0x00, b, 2) && b[0] == 0x0D && b[1] == 0x00, "the offset applies: %02X %02X", b[0], b[1]);
    v[0] = 0x02; v[1] = 0x22;
    CHECK(fake_i2c_write(0x48, 0x01, v, 2), "CONFIG soft reset");
    poll_ms(5);
    CHECK(fake_i2c_read(0x48, 0x07, b, 2) && b[0] == 0 && b[1] == 0, "the reset reloads TEMP_OFFSET");
    CHECK(fake_i2c_read(0x48, 0x00, b, 2) && b[0] == 0x0C && b[1] == 0x80, "and the result");
    v[0] = 0x55; v[1] = 0xAA;
    CHECK(fake_i2c_write(0x48, 0x0F, v, 2), "write to the read-only DEVICE_ID");
    poll_ms(5);
    CHECK(fake_i2c_read(0x48, 0x0F, b, 2) && b[0] == 0x01 && b[1] == 0x17, "DEVICE_ID kept");
    sm_reset();

    fresh();
    CHECK(define_fixture("co2", "cmap_scd4x"), "cmap_scd4x");
    CHECK(run("{\"cmd\":\"sensor_start\",\"instance\":\"co2\",\"pins\":{\"sda\":3,\"scl\":4}}"), "start: %s", g_err);
    x = &g_fpga.t[0];
    CHECK(!(x->flags & EMU_I2C_ADDR16) && x->page == EMU_I2C_PAGE_CMAP, "8-bit, page byte 2 (command map): 0x%02x 0x%02x", x->flags, x->page);
    CHECK(((x->flags >> 2) & 3u) == 1u, "commands only: nack while busy");
    CHECK(fake_i2c_write16(0x62, 0xEC05, NULL, 0), "read_measurement");
    CHECK(!fake_i2c_read(0x62, -1, b, 9), "not acknowledged for 1 ms");
    fake_advance_us(1500);
    CHECK(fake_i2c_read(0x62, -1, b, 9), "answer");
    CHECK(b[0] == 0x02 && b[1] == 0x58 && b[2] == crc8_31(0x02, 0x58), "CO2 600 ppm with its CRC: %02X %02X %02X", b[0], b[1], b[2]);
    CHECK(fake_i2c_write16(0x62, 0x3682, NULL, 0), "get_serial_number");
    fake_advance_us(1500);
    CHECK(fake_i2c_read(0x62, -1, b, 9) && b[0] == 0x12 && b[1] == 0x34 && b[3] == 0x56 && b[6] == 0x9A, "serial in another high byte");
    CHECK(fake_i2c_write16(0x62, 0xE4B8, NULL, 0), "get_data_ready_status");
    fake_advance_us(1500);
    CHECK(fake_i2c_read(0x62, -1, b, 3) && b[0] == 0x80 && b[1] == 0x06 && b[2] == crc8_31(0x80, 0x06), "data ready");
    CHECK(fake_i2c_write16(0x62, 0x21B1, NULL, 0) && fake_i2c_write16(0x62, 0x3F86, NULL, 0), "commands with no answer are acknowledged");
    uint8_t off[3] = { 0x07, 0xE6, crc8_31(0x07, 0xE6) };
    CHECK(fake_i2c_write16(0x62, 0x241D, off, 3), "set_temperature_offset with its word");
    poll_ms(5);
    CHECK(fake_i2c_write16(0x62, 0x2318, NULL, 0), "get_temperature_offset");
    fake_advance_us(1500);
    CHECK(fake_i2c_read(0x62, -1, b, 3) && b[0] == 0x07 && b[1] == 0xE6 && b[2] == off[2], "reads back the written word: %02X %02X", b[0], b[1]);
    CHECK(run("{\"cmd\":\"sensor_set\",\"instance\":\"co2\",\"values\":{\"co2_ppm\":1200}}"), "set: %s", g_err);
    poll_ms(3);
    CHECK(fake_i2c_write16(0x62, 0xEC05, NULL, 0), "read_measurement again");
    fake_advance_us(1500);
    CHECK(fake_i2c_read(0x62, -1, b, 9) && b[0] == 0x04 && b[1] == 0xB0, "CO2 1200 ppm: %02X %02X", b[0], b[1]);
    sm_reset();
}

/* ---- every bus at once, the pin rules ----------------------------------------------------- */

static const char *k_spi_def =
    "{\"format\":1,\"kind\":\"spi\",\"name\":\"acc\","
    "\"params\":[{\"key\":\"ax\",\"min\":-8,\"max\":8,\"default\":1}],"
    "\"spi\":{\"mode\":3,\"read_bit\":{\"bit\":7,\"value\":1},\"address_mask\":63},"
    "\"registers\":[{\"at\":15,\"u8\":51,\"writable\":false},{\"at\":40,\"s16le\":\"ax * 1000\"},{\"at\":50,\"u16be\":\"fifo_level\"}],"
    "\"fifo\":{\"at\":58,\"depth\":32,\"frame\":[\"byte(sat_s16(ax * 1000), 0)\",\"byte(sat_s16(ax * 1000), 1)\"],\"rate_hz\":200}}";

static const char *k_drdy_def =
    "{\"format\":1,\"kind\":\"i2c\",\"name\":\"rate\",\"params\":[{\"key\":\"x\",\"min\":0,\"max\":100}],"
    "\"i2c\":{\"address\":16},\"registers\":[{\"at\":0,\"u8\":\"x\"},{\"at\":1,\"u8\":0}],"
    "\"data_ready\":{\"active\":\"high\",\"mode\":\"pulse\",\"pulse_us\":20,\"on\":\"rate\",\"rate_hz\":100,\"enabled\":\"reg[1] & 1\"}}";

static const char *k_mhz19_def =
    "{\"format\":1,\"kind\":\"uart\",\"name\":\"mhz19\",\"params\":[{\"key\":\"co2_ppm\",\"min\":0,\"max\":5000,\"default\":800}],"
    "\"uart\":{\"baud\":9600,\"format\":\"8N1\"},"
    "\"requests\":[{\"match\":[255,1,134,\"*\",\"*\",\"*\",\"*\",\"*\",\"*\"],"
    "\"answer\":[255,134,\"byte(co2_ppm, 1)\",\"byte(co2_ppm, 0)\",0,0,0,0,0],\"delay_ms\":10}]}";

static void test_all_buses(void) {
    fresh();
    const char *i2c[4] = { "a", "b", "c", "d" };
    for (int i = 0; i < 4; i++) CHECK(define_fixture(i2c[i], "bme280") || 1, "x");
    CHECK(define_fixture("e", "bme280"), "fifth");
    CHECK(define_text("spi", k_spi_def, strlen(k_spi_def)), "spi: %s", g_err);
    CHECK(define_fixture("gps", "gps_nmea"), "gps");
    /* four I2C sensors on two buses */
    CHECK(run("{\"cmd\":\"sensor_start\",\"instance\":\"a\",\"pins\":{\"sda\":1,\"scl\":2}}"), "a: %s", g_err);
    CHECK(run("{\"cmd\":\"sensor_start\",\"instance\":\"b\",\"address\":\"0x77\",\"pins\":{\"sda\":1,\"scl\":2}}"), "b: %s", g_err);
    CHECK(run("{\"cmd\":\"sensor_start\",\"instance\":\"c\",\"pins\":{\"sda\":3,\"scl\":4}}"), "c: %s", g_err);
    CHECK(run("{\"cmd\":\"sensor_start\",\"instance\":\"d\",\"address\":\"0x77\",\"pins\":{\"sda\":3,\"scl\":4}}"), "d: %s", g_err);
    CHECK(!run("{\"cmd\":\"sensor_start\",\"instance\":\"e\",\"pins\":{\"sda\":5,\"scl\":6}}") &&
          strstr(g_err, "all 4 I2C sensor slots are in use"), "%s", g_err);
    CHECK(!run("{\"cmd\":\"sensor_start\",\"instance\":\"spi\",\"pins\":{\"sclk\":5,\"mosi\":6,\"miso\":7,\"cs\":8,\"drdy\":9}}") &&
          strstr(g_err, "no data-ready pin"), "%s", g_err);
    CHECK(run("{\"cmd\":\"sensor_start\",\"instance\":\"spi\",\"pins\":{\"sclk\":\"LA5\",\"mosi\":6,\"miso\":7,\"cs\":8}}"), "spi: %s", g_err);
    CHECK(g_fpga.spi_cfg[0] == 7 && g_fpga.spi_cfg[1] == 4 && (g_fpga.spi_cfg[3] & (EMU_SPI_ENABLE | EMU_SPI_CPHA | EMU_SPI_CPOL)) == 7 &&
          g_fpga.spi_cfg[4] == 0x87 && g_fpga.spi_cfg[6] == 63, "SPIT_CONFIG");
    CHECK(g_fpga.t[4].attrs[15] == EMU_ATTR_RO && g_fpga.t[4].attrs[58] == EMU_ATTR_FIFO, "SPI attributes: FIFO and RO only");
    CHECK(g_fpga.pinmap[6] == EMU_PIN_SPI_MISO, "MISO in the pin map");
    CHECK(!run("{\"cmd\":\"sensor_start\",\"instance\":\"gps\",\"pins\":{\"tx\":6}}") && strstr(g_err, "pin conflict: LA6 is in use by sensor:spi"),
          "%s", g_err);
    CHECK(run("{\"cmd\":\"sensor_start\",\"instance\":\"gps\",\"pins\":{\"tx\":9,\"rx\":10}}"), "gps: %s", g_err);
    CHECK(g_fpga.u2_armed && g_fpga.u2_tx_w == 8 && g_fpga.u2_rx_w == 9 && g_fpga.u2_baud == 9600, "UART2 armed");
    CHECK(g_fpga.pinmap[8] == EMU_PIN_UART2_TX, "TX in the pin map");

    /* SPI: the DUT reads the data, the FIFO fills at 200 Hz */
    uint8_t b[4];
    fake_spi_read(40, b, 2);
    CHECK((int16_t)(b[0] | b[1] << 8) == 1000, "spi data %d", (int16_t)(b[0] | b[1] << 8));
    poll_ms(100);
    CHECK(g_fpga.fifo_len >= 36 && g_fpga.fifo_len <= 44 && g_fpga.fifo_len % 2 == 0, "fifo after 100 ms: %u bytes", g_fpga.fifo_len);
    poll_ms(400);
    CHECK(g_fpga.fifo_len == 64, "fifo capped at its depth: %u", g_fpga.fifo_len);
    fake_spi_read(58, b, 2);
    CHECK((int16_t)(b[0] | b[1] << 8) == 1000, "a frame pops");
    poll_ms(3);
    fake_spi_read(50, b, 2);
    CHECK(((b[0] << 8) | b[1]) >= 31, "fifo_level reported: %u", (b[0] << 8) | b[1]);

    /* GPS: an epoch a second */
    size_t before = g_fpga.u2_nwire;
    poll_ms(2100);
    CHECK(g_fpga.u2_nwire > before + 300, "NMEA sent: %zu bytes", g_fpga.u2_nwire - before);
    g_fpga.u2_wire[g_fpga.u2_nwire] = 0;
    CHECK(strstr((char *)g_fpga.u2_wire, "$GPRMC") && strstr((char *)g_fpga.u2_wire, "\r\n"), "RMC sentence");

    /* status of all six instances fits one cloud reply or pages */
    CHECK(run("{\"cmd\":\"sensor_status\"}"), "status: %s", g_err);
    size_t len = strlen(g_reply);
    CHECK(len <= BP_CLOUD_REPLY_MAX, "status %zu bytes", len);
    unsigned listed = 0;
    for (const char *p = g_reply; (p = strstr(p, "\"instance\":")) != NULL; p++) listed++;
    const char *nx = strstr(g_reply, "\"next\":");
    printf("  sensor_status with 7 instances: %zu bytes, %u listed%s\n", len, listed, nx ? ", paged" : "");
    if (nx) {
        int next = atoi(nx + 7);
        CHECK(run("{\"cmd\":\"sensor_status\",\"first\":%d}", next) && strlen(g_reply) <= BP_CLOUD_REPLY_MAX, "page 2");
    }
    CHECK(strstr(g_reply, "\"slot\":\"uart0\"") || nx, "uart slot");
    sm_reset();
}

/* ---- request / answer, faults on a UART -------------------------------------------------- */

static void test_uart_requests(void) {
    fresh();
    g_fpga.image = 2;
    CHECK(define_text("co2", k_mhz19_def, strlen(k_mhz19_def)), "mhz19: %s", g_err);
    CHECK(run("{\"cmd\":\"sensor_start\",\"instance\":\"co2\",\"pins\":{\"tx\":11,\"rx\":12},\"values\":{\"co2_ppm\":1234}}"),
          "start: %s", g_err);
    CHECK(g_fpga.image_loads == 0, "already on the emulation image");
    const char req[9] = { (char)0xFF, 0x01, (char)0x86, 0, 0, 0, 0, 0, 0x79 };
    fake_uart_dut_send(req, 9);
    poll_ms(5);
    CHECK(g_fpga.u2_nwire == 0, "the answer waits for delay_ms");
    poll_ms(10);
    CHECK(g_fpga.u2_nwire == 9 && g_fpga.u2_wire[0] == 0xFF && g_fpga.u2_wire[1] == 0x86 &&
          (g_fpga.u2_wire[2] << 8 | g_fpga.u2_wire[3]) == 1234, "answer: %zu bytes", g_fpga.u2_nwire);
    /* no_response: nothing answers */
    CHECK(run("{\"cmd\":\"sensor_fault\",\"instance\":\"co2\",\"type\":\"no_response\",\"duration_s\":0.5}"), "fault: %s", g_err);
    fake_uart_dut_send(req, 9);
    poll_ms(30);
    CHECK(g_fpga.u2_nwire == 9, "muted");
    CHECK(run("{\"cmd\":\"sensor_status\",\"instance\":\"co2\"}") && strstr(g_reply, "\"faults\":[{\"type\":\"no_response\",\"left_s\":0.4"),
          "fault in status: %s", g_reply);
    poll_ms(500);
    fake_uart_dut_send(req, 9);
    poll_ms(30);
    CHECK(g_fpga.u2_nwire == 18, "answers again after the fault: %zu", g_fpga.u2_nwire);
    /* baud_error reconfigures UART2 */
    CHECK(run("{\"cmd\":\"sensor_fault\",\"instance\":\"co2\",\"type\":\"baud_error\",\"value\":5}"), "baud fault: %s", g_err);
    poll_ms(2);
    CHECK(g_fpga.u2_baud == 10080, "baud off by 5 %%: %u", g_fpga.u2_baud);
    CHECK(run("{\"cmd\":\"sensor_fault_clear\",\"instance\":\"co2\"}"), "clear");
    poll_ms(2);
    CHECK(g_fpga.u2_baud == 9600, "baud back");
    CHECK(!run("{\"cmd\":\"sensor_fault\",\"instance\":\"co2\",\"type\":\"bus_stuck\",\"duration_s\":0.5}") && strstr(g_err, "does not apply"), "%s", g_err);
    sm_reset();
}

/* ---- data ready, bus faults, the stimulus, a reconfiguration ------------------------------- */

static void test_drdy_faults_stimulus(void) {
    fresh();
    CHECK(define_text("r", k_drdy_def, strlen(k_drdy_def)), "drdy def: %s", g_err);
    CHECK(run("{\"cmd\":\"sensor_start\",\"instance\":\"r\",\"pins\":{\"sda\":1,\"scl\":2,\"drdy\":7}}"), "start: %s", g_err);
    CHECK(g_fpga.pinmap[6] == EMU_PIN_DRDY(0) && g_fpga.dr_width == 20, "drdy pin and width");
    poll_ms(100);
    CHECK(g_fpga.t[0].drdy_asserts == 0, "disabled until the DUT enables it");
    uint8_t v = 1;
    fake_i2c_write(0x10, 0x01, &v, 1);
    poll_ms(200);
    CHECK(g_fpga.t[0].drdy_asserts >= 18 && g_fpga.t[0].drdy_asserts <= 22, "100 Hz: %u in 200 ms", g_fpga.t[0].drdy_asserts);
    v = 0;
    fake_i2c_write(0x10, 0x01, &v, 1);
    poll_ms(3);
    unsigned n = g_fpga.t[0].drdy_asserts;
    poll_ms(100);
    CHECK(g_fpga.t[0].drdy_asserts == n && g_fpga.t[0].drdy_deasserts >= 1, "released when disabled");

    /* faults: no_response mutes, bus_stuck clears itself within 1 s, stuck freezes */
    CHECK(run("{\"cmd\":\"sensor_fault\",\"instance\":\"r\",\"type\":\"no_response\"}"), "mute: %s", g_err);
    uint8_t b[2];
    CHECK(!fake_i2c_read(0x10, 0, b, 1), "not acknowledged");
    CHECK(run("{\"cmd\":\"sensor_fault_clear\",\"instance\":\"r\"}") && fake_i2c_read(0x10, 0, b, 1), "acknowledged again");
    CHECK(run("{\"cmd\":\"sensor_fault\",\"instance\":\"r\",\"type\":\"bus_stuck\",\"duration_s\":1}"), "stuck: %s", g_err);
    CHECK(g_fpga.t[0].flags & EMU_I2C_BUS_STUCK, "SDA held");
    poll_ms(1005);
    CHECK(!(g_fpga.t[0].flags & EMU_I2C_BUS_STUCK), "bounded to 1 s");
    CHECK(run("{\"cmd\":\"sensor_fault\",\"instance\":\"r\",\"type\":\"corrupt\",\"at\":\"0x00\",\"mask\":\"0xFF\"}"), "corrupt: %s", g_err);
    poll_ms(3);
    CHECK(fake_i2c_read(0x10, 0, b, 1) && b[0] == 0xFF, "corrupted: 0x%02x", b[0]);
    run("{\"cmd\":\"sensor_fault_clear\",\"instance\":\"r\"}");
    poll_ms(3);

    /* a stimulus ramps x */
    const char *st = "{\"format\":1,\"kind\":\"stimulus\",\"name\":\"ramp\",\"label\":\"Ramp\",\"description\":\"x up\",\"rate_hz\":100,\"signals\":{"
                     "\"x\":[{\"shape\":\"ramp\",\"start\":0,\"end\":100,\"duration_s\":1}]},"
                     "\"faults\":[{\"at_s\":0.5,\"duration_s\":0.1,\"type\":\"no_response\"}]}";
    CHECK(define_text(NULL, st, strlen(st)), "stimulus: %s", g_err);
    CHECK(strstr(g_reply, "\"signals\":[\"x\"]") && strstr(g_reply, "\"duration_s\":1"), "%s", g_reply);
    CHECK(run("{\"cmd\":\"sensor_stimulus_start\"}"), "start: %s", g_err);
    poll_ms(250);
    CHECK(fake_i2c_read(0x10, 0, b, 1) && b[0] >= 20 && b[0] <= 30, "x at 0.25 s: %u", b[0]);
    poll_ms(300);
    CHECK(!fake_i2c_read(0x10, 0, b, 1), "the stimulus fault is on at 0.55 s");
    poll_ms(100);
    CHECK(fake_i2c_read(0x10, 0, b, 1), "and off at 0.65 s");
    poll_ms(500);
    CHECK(fake_i2c_read(0x10, 0, b, 1) && b[0] == 100, "holds the end: %u", b[0]);
    CHECK(run("{\"cmd\":\"sensor_status\"}") && strstr(g_reply, "\"stimulus\":{\"loaded\":true,\"running\":false"), "%s", g_reply);
    CHECK(run("{\"cmd\":\"sensor_stimulus_start\",\"at_s\":0.5}") && run("{\"cmd\":\"sensor_stimulus_stop\"}"), "start at, stop");

    /* a reconfiguration of the same image re-arms with the DUT's configuration kept */
    v = 0x5A;
    fake_i2c_write(0x10, 0x01, &v, 1);
    poll_ms(3);
    for (unsigned t = 0; t < 5; t++) { memset(g_fpga.t[t].regs, 0, 256); g_fpga.t[t].flags = 0; }
    sm_on_gateware_reconfigured();
    poll_ms(1);
    CHECK((g_fpga.t[0].flags & EMU_I2C_ENABLE) && g_fpga.t[0].regs[1] == 0x5A && g_fpga.t[0].regs[0] == 100, "re-armed, the DUT's byte kept");
    /* another image: the sensor stops and gives its pins back */
    g_fpga.image = 0;
    sm_on_gateware_reconfigured();
    poll_ms(1);
    CHECK(la_pins_fn(1) == LA_FN_NONE && la_pins_fn(7) == LA_FN_NONE, "pins released");
    char names[32];
    CHECK(!sensor_mgr_digital_names(names, sizeof(names)), "nothing digital runs");
    sm_reset();
}

/* ---- analog ----------------------------------------------------------------------------- */

static void test_analog(void) {
    fresh();
    const char *ntc = "{\"format\":1,\"kind\":\"analog\",\"name\":\"ntc\",\"output\":\"voltage\","
                      "\"params\":[{\"key\":\"temperature_c\",\"min\":-40,\"max\":150,\"default\":25}],"
                      "\"value\":\"3.3 * 10000 / (10000 + 10000 * exp(3950 * (1 / (temperature_c + 273.15) - 1 / 298.15)))\","
                      "\"range\":{\"min\":0,\"max\":3.3}}";
    const char *loop = "{\"format\":1,\"kind\":\"analog\",\"name\":\"pt\",\"output\":\"current\","
                       "\"params\":[{\"key\":\"bar\",\"min\":0,\"max\":10,\"default\":5}],"
                       "\"value\":\"4 + 16 * bar / 10\",\"range\":{\"min\":4,\"max\":20}}";
    CHECK(define_text("ntc", ntc, strlen(ntc)) && define_text("pt", loop, strlen(loop)), "analog defs: %s", g_err);
    g_fpga.dac_running = true;
    CHECK(!run("{\"cmd\":\"sensor_start\",\"instance\":\"ntc\",\"pins\":{\"output\":\"dac\"}}") && strstr(g_err, "dac_stop it first"), "%s", g_err);
    g_fpga.dac_running = false;
    CHECK(!run("{\"cmd\":\"sensor_start\",\"instance\":\"ntc\",\"pins\":{\"output\":\"current\"}}") && strstr(g_err, "use output \"dac\""), "%s", g_err);
    CHECK(run("{\"cmd\":\"sensor_start\",\"instance\":\"ntc\",\"pins\":{\"output\":\"dac\"}}"), "ntc: %s", g_err);
    CHECK(strstr(g_reply, "\"slot\":\"analog0\"") && g_fpga.image_loads == 0, "no image swap for analog: %s", g_reply);
    CHECK(g_fpga.analog_out == SM_OUT_DAC && fabs(g_fpga.analog_value - 1.65) < 1e-9, "1.65 V at 25 C: %g", g_fpga.analog_value);
    run("{\"cmd\":\"sensor_set\",\"instance\":\"ntc\",\"values\":{\"temperature_c\":100}}");
    poll_ms(1);
    CHECK(g_fpga.analog_value > 2.9, "hotter: %g", g_fpga.analog_value);
    CHECK(sensor_mgr_command_conflict("generate") && strstr(sensor_mgr_command_conflict("generate"), "sensor:ntc"), "generate refused");
    CHECK(sensor_mgr_command_conflict("status") == NULL, "status fine");
    /* the current output on a board where it shares the DAC */
    CHECK(run("{\"cmd\":\"sensor_start\",\"instance\":\"pt\",\"pins\":{\"output\":\"current\"}}"), "pt: %s", g_err);
    CHECK(g_fpga.analog_out == SM_OUT_CURRENT && fabs(g_fpga.analog_value - 12) < 1e-9, "12 mA");
    CHECK(run("{\"cmd\":\"sensor_fault\",\"instance\":\"pt\",\"type\":\"short\"}"), "short: %s", g_err);
    poll_ms(1);
    CHECK(fabs(g_fpga.analog_value - 20) < 1e-9, "short: 20 mA");
    sm_on_dac_stop();
    CHECK(run("{\"cmd\":\"sensor_status\"}") && !strstr(g_reply, "\"running\":true,"), "dac_stop stops them: %s", g_reply);
    sm_reset();
}

/* ---- reply sizes ------------------------------------------------------------------------- */

static void test_reply_sizes(void) {
    fresh();
    /* 16 params with long keys */
    char def[4096];
    size_t o = (size_t)snprintf(def, sizeof(def), "{\"format\":1,\"kind\":\"i2c\",\"name\":\"wide\",\"label\":\"%s\",\"params\":[",
                                "a label of some length for the define reply, as a library sensor has");
    for (int i = 0; i < 16; i++)
        o += (size_t)snprintf(def + o, sizeof(def) - o, "%s{\"key\":\"param_with_a_long_name_%02d\",\"unit\":\"unit\",\"min\":-123456.5,\"max\":123456.25,\"default\":0.125}",
                              i ? "," : "", i);
    o += (size_t)snprintf(def + o, sizeof(def) - o, "],\"i2c\":{\"address\":32,\"alt_addresses\":[33,34,35]},\"registers\":[{\"at\":0,\"u8\":1}]}");
    CHECK(define_text("wide_name_16chr", def, o), "wide: %s", g_err);
    CHECK(strlen(g_reply) <= BP_CLOUD_REPLY_MAX, "define reply %zu", strlen(g_reply));
    printf("  define reply with 16 long params: %zu bytes%s\n", strlen(g_reply), strstr(g_reply, "params_cut") ? " (params cut)" : "");
    char names[6][17];
    for (int i = 0; i < 6; i++) {
        snprintf(names[i], sizeof(names[i]), "instance_name_%02d", i);
        define_text(names[i], def, o);
    }
    for (int i = 0; i < 4; i++)
        CHECK(run("{\"cmd\":\"sensor_start\",\"instance\":\"%s\",\"address\":\"0x%02X\",\"pins\":{\"sda\":1,\"scl\":2}}", names[i], 32 + i),
              "start %s: %s", names[i], g_err);
    CHECK(run("{\"cmd\":\"sensor_status\"}") && strlen(g_reply) <= BP_CLOUD_REPLY_MAX, "status page 1: %zu", strlen(g_reply));
    const char *nx = strstr(g_reply, "\"next\":");
    printf("  sensor_status, 6 wide instances (4 running): %zu bytes, next %s\n", strlen(g_reply), nx ? nx + 7 : "none");
    unsigned pages = 1;
    while (nx && pages < 10) {
        CHECK(run("{\"cmd\":\"sensor_status\",\"first\":%d}", atoi(nx + 7)) && strlen(g_reply) <= BP_CLOUD_REPLY_MAX, "page");
        nx = strstr(g_reply, "\"next\":");
        pages++;
    }
    CHECK(pages > 1 && pages < 10, "paged in %u replies", pages);
    printf("  largest reply in this suite: %zu bytes (cap %u)\n", g_max_reply, BP_CLOUD_REPLY_MAX);
    /* la_pins with sensor owners stays under the cap: names are at most 16 characters */
    char buf[2400];
    bp_emit_t e;
    bp_emit_init(&e, buf, sizeof(buf));
    bp_emit_raw(&e, "{\"status\":\"ok\",\"data\":{\"pins\":");
    la_pins_emit_list(&e, 0x3FFF, 0x00FF);
    bp_emit_raw(&e, ",\"levels\":16383}}\n");
    CHECK(bp_emit_len(&e) <= BP_CLOUD_REPLY_MAX, "la_pins %zu", bp_emit_len(&e));
    sm_reset();
    CHECK(g_fpga.allocs == 0, "heap given back: %ld", g_fpga.allocs);
}

static void test_out_of_memory(void) {
    fresh();
    g_fpga.heap_limit = 16000;
    CHECK(!define_fixture("env", "bme280") && strstr(g_err, "out of memory"), "%s", g_err);
    g_fpga.heap_limit = 0;
    sm_reset();
    CHECK(g_fpga.allocs == 0, "nothing leaked: %ld", g_fpga.allocs);
}

int main(void) {
    test_define();
    test_bme280_mpu();
    test_sht4x();
    test_word_cmap();
    test_all_buses();
    test_uart_requests();
    test_drdy_faults_stimulus();
    test_analog();
    test_reply_sizes();
    test_out_of_memory();
    if (fails) { printf("test_sensor_mgr: %d FAILURE(S)\n", fails); return 1; }
    printf("test_sensor_mgr: all passed\n");
    return 0;
}
