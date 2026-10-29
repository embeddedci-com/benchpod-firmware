/*
 * test_ch_calibrate.c — host tests for the `calibrate` reply (src/command_handler_analog.c).
 *
 * The reply grows with the DMM parts: every DAC line, every gain and a gain run's measurement.
 * It is sent as one frame of its own (send_ok_str stops at 256 B and answered "reply too large"
 * on a pod with just the zero, found on hardware). This fills every part and checks the frame
 * that goes out: whole, valid in shape, and every field there.
 *
 * The analog hardware (routing, DAC, ADC capture) is fake; the calibration store is the real one
 * on the mock flash.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "command_handler_internal.h"
#include "at_driver.h"
#include "adc_cal.h"
#include "cal_data.h"
#include "i2c_bus.h"
#include "signal_engine.h"
#include "mocks/mock_flash.h"

static int fails;
#define CHECK(cond, ...) do { if (!(cond)) { printf("FAIL %s:%d: ", __func__, __LINE__); \
        printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

/* ---- fakes ----------------------------------------------------------------------------- */
static char g_reply[2048];
static size_t g_reply_len;
int at_send_data(int c, const uint8_t *b, size_t n) {
    (void)c;
    g_reply_len = n < sizeof(g_reply) - 1 ? n : sizeof(g_reply) - 1;
    memcpy(g_reply, b, g_reply_len);
    g_reply[g_reply_len] = 0;
    return 0;
}
int at_close_connection(int c) { (void)c; return 0; }
void send_ok_str(int c, const char *p) { (void)c; snprintf(g_reply, sizeof(g_reply), "ok:%s", p); }
void send_error(int c, const char *m) { (void)c; snprintf(g_reply, sizeof(g_reply), "err:%s", m); }
bool heavy_begin(int c) { (void)c; return true; }
void heavy_release(int c) { (void)c; }
void cloud_client_request_caps_resend(void) {}
void sleep_ms(uint32_t ms) { (void)ms; }

int dacmux_set_ctrl1(bool en, uint8_t sel) { (void)en; (void)sel; return 0; }
int dacmux_set_ctrl2(bool en, uint8_t sel) { (void)en; (void)sel; return 0; }
int dacmux_read(uint8_t *r) { *r = 0; return 0; }
int calsw_set(bool a, bool b, bool c, bool d) { (void)a; (void)b; (void)c; (void)d; return 0; }
int calsw_read(uint8_t *r) { *r = 0; return 0; }
/* The range (i2c_bus.c): K5 when g_low_hw, put back to normal by off and cal2. */
static bool g_low_hw;
static adc_range_t g_range;
static analog_path_t g_path;
int analog_path_set(analog_path_t p) {
    g_path = p;
    if (p == ANALOG_PATH_OFF || p == ANALOG_PATH_CAL2) g_range = ADC_RANGE_NORMAL;
    return 0;
}
int analog_path_from_name(const char *n, analog_path_t *p) {
    static const char *const names[] = { "off", "dac_3v3", "dac_5v", "dac_12v", "ext", "cal1", "cal2", "current_in" };
    for (int i = 0; i < 8; i++) if (strcmp(n, names[i]) == 0) { *p = (analog_path_t)i; return 0; }
    return -1;
}
const char *analog_path_name(analog_path_t p) {
    static const char *const names[] = { "off", "dac_3v3", "dac_5v", "dac_12v", "adc_ext", "cal1", "cal2", "current_in" };
    return (unsigned)p < 8 ? names[p] : "";
}
bool analog_path_keeps_low_range(analog_path_t p) { return p != ANALOG_PATH_OFF && p != ANALOG_PATH_CAL2; }
const char *adc_range_name(adc_range_t r) { return r == ADC_RANGE_LOW ? "low" : "normal"; }
int adc_range_from_name(const char *n, adc_range_t *r) {
    if (strcmp(n, "low") == 0) { *r = ADC_RANGE_LOW; return 0; }
    if (strcmp(n, "normal") == 0) { *r = ADC_RANGE_NORMAL; return 0; }
    return -1;
}
bool adc_range_supported(void) { return g_low_hw; }
adc_range_t adc_range_get(void) { return g_range; }
int adc_range_set(adc_range_t r) {
    if (r == ADC_RANGE_LOW && !g_low_hw) return -1;
    if (r == ADC_RANGE_LOW && g_path == ANALOG_PATH_CAL2) return -2;
    g_range = r;
    return 0;
}
static adc_range_test_pt_t g_test[ADC_RANGE_TEST_POINTS];
int adc_range_selftest(adc_range_test_pt_t pts[ADC_RANGE_TEST_POINTS], uint32_t *span) {
    (void)span;
    memcpy(pts, g_test, sizeof(g_test));
    return 0;
}
int dac_set_constant16(uint16_t code, uint32_t div) { (void)code; (void)div; return 0; }
int dac_quad_current_out(uint16_t code) { (void)code; return 0; }
static uint16_t g_adc_count = 65522;
int adc_capture_psram(uint16_t *out, size_t n, float rate) { (void)rate; for (size_t i = 0; i < n; i++) out[i] = g_adc_count; return 0; }

int adc_cal_measure_current_in(adc_reading_t *out) {
    uint16_t b[16];
    for (int i = 0; i < 16; i++) b[i] = 49;   /* benchpod-445e46: J8 open at count 49 */
    *out = adc_scale_burst(b, 16, ADC_CAL_CAL1.a, ADC_CAL_CAL1.b);
    return 0;
}

/* A loopback that reads 0.18 % low against the DAC line it was driven with (the low range: 1.2 %
   low against its fit before the low gain). */
int adc_cal_measure_gain(int src, bool low, adc_cal_gain_meas_t *m, uint32_t *span) {
    (void)span;
    cal_lin_t f = adc_cal_fit(src == ADC_CAL_SRC_CAL2 ? ADC_CAL_CAL2 : src == ADC_CAL_SRC_EXT ? ADC_CAL_EXT : ADC_CAL_CAL1);
    if (low) {
        float k = cal_adc_low_ratio() * adc_cal_gain(ADC_CAL_SRC_CAL1);
        f.a *= k;
        f.b *= k;
    }
    for (int i = 0; i < 2; i++) {
        int path = adc_cal_gain_path(src);
        m->v[i] = dac_volts_for_code16(path, dac_code16_for_volts(path, adc_cal_gain_point(src, i)));
        m->count[i] = (float)((m->v[i] / (low ? 1.012 : 1.0018) - f.a) / f.b);
    }
    return 0;
}

/* ---- tests ----------------------------------------------------------------------------- */
static int count_char(const char *s, char c) { int n = 0; for (; *s; s++) n += (*s == c); return n; }

/* A whole frame: the ok wrapper, balanced braces, one newline at the end. */
static bool whole_frame(void) {
    return strncmp(g_reply, "{\"status\":\"ok\",\"data\":{", 23) == 0 && g_reply_len > 2 &&
           g_reply[g_reply_len - 1] == '\n' && g_reply[g_reply_len - 2] == '}' &&
           count_char(g_reply, '{') == count_char(g_reply, '}') && count_char(g_reply, '\n') == 1;
}

static void test_reply_with_every_part(void) {
    mock_flash_reset();
    cal_data_select(3);
    adc_cal_load();

    handle_calibrate(0, "{\"cmd\":\"calibrate\"}");
    CHECK(whole_frame() && strstr(g_reply, "\"calibrated\":false"), "bare read: %s", g_reply);

    handle_calibrate(0, "{\"cmd\":\"calibrate\",\"source\":\"current_in\"}");
    CHECK(whole_frame() && strstr(g_reply, "\"calibrated\":true") && strstr(g_reply, "\"zero_count_mc\":65585000") &&
          strstr(g_reply, "\"samples\":512"), "zero run (was 'reply too large' on hardware): %s", g_reply);

    static const struct { const char *path; float v1, v2, err; } dmm[3] = {
        { "3v3", 0.5f, 3.0f, 0.998f }, { "5v", 1.0f, 4.0f, 0.999f }, { "12v", -5.0f, 5.0f, 1.00055f },
    };
    for (int i = 0; i < 3; i++) {
        uint16_t c1 = dac_code16_for_volts(i, dmm[i].v1), c2 = dac_code16_for_volts(i, dmm[i].v2);
        char req[160];
        snprintf(req, sizeof(req), "{\"cmd\":\"calibrate\",\"dac\":\"%s\",\"code1\":%u,\"uv1\":%ld,\"code2\":%u,\"uv2\":%ld}",
                 dmm[i].path, (unsigned)c1, lroundf(dmm[i].v1 * dmm[i].err * 1e6f), (unsigned)c2,
                 lroundf(dmm[i].v2 * dmm[i].err * 1e6f));
        handle_calibrate(0, req);
        CHECK(whole_frame() && !strstr(g_reply, "err:"), "dac %s: %s", dmm[i].path, g_reply);
    }
    static const char *const src[3] = { "cal1", "cal2", "ext" };
    for (int i = 0; i < 3; i++) {
        char req[64];
        snprintf(req, sizeof(req), "{\"cmd\":\"calibrate\",\"adc\":\"%s\"}", src[i]);
        handle_calibrate(0, req);
        CHECK(whole_frame() && strstr(g_reply, "\"measured\":{") && strstr(g_reply, "\"residual_uv\":"), "adc %s: %s", src[i], g_reply);
    }
    /* The largest reply there is: every line, every gain and a measurement. */
    CHECK(strstr(g_reply, "\"3v3\":{") && strstr(g_reply, "\"5v\":{") && strstr(g_reply, "\"12v\":{") &&
          strstr(g_reply, "\"ext\":18") && strstr(g_reply, "\"dmm\":true"), "parts missing: %s", g_reply);
    printf("largest calibrate reply: %u bytes (buffer 832, cloud cap 1791)\n", (unsigned)g_reply_len);
    CHECK(g_reply_len < 800, "reply %u bytes, too close to the buffer", (unsigned)g_reply_len);

    handle_calibrate(0, "{\"cmd\":\"calibrate\",\"clear\":\"dac\"}");
    CHECK(whole_frame() && strstr(g_reply, "\"3v3\":null,\"5v\":null,\"12v\":null"), "clear dac: %s", g_reply);
    handle_calibrate(0, "{\"cmd\":\"calibrate\",\"clear\":\"nonsense\"}");
    CHECK(strstr(g_reply, "err:") && strstr(g_reply, "clear must be"), "unknown part: %s", g_reply);
    handle_calibrate(0, "{\"cmd\":\"calibrate\",\"clear\":true}");
    CHECK(whole_frame() && strstr(g_reply, "\"calibrated\":false") && strstr(g_reply, "\"dmm\":false"), "clear all: %s", g_reply);
}

/* The low range: calibrate through cal1, the reply gains adc_low_gain_ppm; only where K5 exists. */
static void test_low_range_calibrate(void) {
    mock_flash_reset();
    cal_data_select(3);
    adc_cal_load();
    g_low_hw = false;
    handle_calibrate(0, "{\"cmd\":\"calibrate\"}");
    CHECK(whole_frame() && !strstr(g_reply, "adc_low_gain_ppm"), "no K5, no low gain field: %s", g_reply);
    handle_calibrate(0, "{\"cmd\":\"calibrate\",\"source\":\"current_in\"}");
    uint16_t c1 = dac_code16_for_volts(1, 1.0f), c2 = dac_code16_for_volts(1, 4.0f);
    char req[160];
    snprintf(req, sizeof(req), "{\"cmd\":\"calibrate\",\"dac\":\"5v\",\"code1\":%u,\"uv1\":999000,\"code2\":%u,\"uv2\":3996000}",
             (unsigned)c1, (unsigned)c2);
    handle_calibrate(0, req);
    handle_calibrate(0, "{\"cmd\":\"calibrate\",\"adc\":\"cal1\",\"range\":\"low\"}");
    CHECK(strstr(g_reply, "err:") && strstr(g_reply, "v3r4"), "low without K5: %s", g_reply);

    g_low_hw = true;
    handle_calibrate(0, "{\"cmd\":\"calibrate\"}");
    CHECK(whole_frame() && strstr(g_reply, "\"adc_low_gain_ppm\":null"), "low gain null: %s", g_reply);
    handle_calibrate(0, "{\"cmd\":\"calibrate\",\"adc\":\"ext\",\"range\":\"low\"}");
    CHECK(strstr(g_reply, "err:") && strstr(g_reply, "through cal1"), "low on ext: %s", g_reply);
    handle_calibrate(0, "{\"cmd\":\"calibrate\",\"adc\":\"cal1\",\"range\":\"high\"}");
    CHECK(strstr(g_reply, "err:") && strstr(g_reply, "normal or low"), "bad range: %s", g_reply);
    handle_calibrate(0, "{\"cmd\":\"calibrate\",\"adc\":\"cal1\",\"range\":\"low\"}");
    CHECK(whole_frame() && strstr(g_reply, "\"adc_low_gain_ppm\":12") && strstr(g_reply, "\"measured\":{"),
          "low gain run: %s", g_reply);
    CHECK(adc_cal_low_gain_is_set() && !adc_cal_gain_is_set(ADC_CAL_SRC_CAL1), "the low run set the normal gain");
    g_low_hw = false;
}

/* adc_range: read, switch, refuse low without K5 and on cal2; adc_read reports the range, its
   full scale and clipped samples. */
static void test_adc_range_and_read(void) {
    mock_flash_reset();
    cal_data_select(3);
    adc_cal_load();
    g_low_hw = false;
    g_range = ADC_RANGE_NORMAL;
    handle_adc_range(0, "{\"cmd\":\"adc_range\"}");
    CHECK(strstr(g_reply, "ok:") && strstr(g_reply, "\"range\":\"normal\",\"supported\":false"), "no K5: %s", g_reply);
    CHECK(!strstr(g_reply, "low_fullscale_mv") && strstr(g_reply, "\"adc_cal_a_uv\":") && strstr(g_reply, "\"adc_cal_b_nv\":-1002"),
          "no K5 fields: %s", g_reply);
    handle_adc_range(0, "{\"cmd\":\"adc_range\",\"range\":\"low\"}");
    CHECK(strstr(g_reply, "err:") && strstr(g_reply, "v3r4"), "low without K5: %s", g_reply);
    handle_adc_range(0, "{\"cmd\":\"adc_range\",\"test\":true}");
    CHECK(strstr(g_reply, "err:") && strstr(g_reply, "v3r4"), "test without K5: %s", g_reply);
    handle_adc_read(0, "{\"cmd\":\"adc_read\",\"source\":\"ext\",\"range\":\"low\"}");
    CHECK(strstr(g_reply, "err:") && strstr(g_reply, "v3r4"), "adc_read low without K5: %s", g_reply);

    g_low_hw = true;
    handle_adc_range(0, "{\"cmd\":\"adc_range\",\"range\":\"low\"}");
    CHECK(strstr(g_reply, "\"range\":\"low\",\"supported\":true,\"fullscale_mv\":54") &&
          strstr(g_reply, "\"normal_fullscale_mv\":32") && strstr(g_reply, "\"low_fullscale_mv\":54"), "low: %s", g_reply);
    CHECK(strstr(g_reply, "\"adc_cal_b_nv\":-166"), "the low fit: %s", g_reply);
    CHECK(strlen(g_reply) + 26 < 256, "reply %u B, send_ok_str holds 256 with its wrapper", (unsigned)strlen(g_reply));
    handle_adc_range(0, "{\"cmd\":\"adc_range\",\"range\":\"medium\"}");
    CHECK(strstr(g_reply, "err:") && strstr(g_reply, "normal or low"), "bad name: %s", g_reply);

    /* adc_read in the low range: 0 V near count 65538 reads ~0 mV; the full scale is the low one. */
    g_adc_count = 2;
    handle_adc_read(0, "{\"cmd\":\"adc_read\",\"source\":\"ext\"}");
    CHECK(strstr(g_reply, "\"range\":\"low\"") && strstr(g_reply, "\"fullscale_mv\":54") &&
          strstr(g_reply, "\"overrange\":false"), "low read: %s", g_reply);
    /* One count is about 167 uV: count 6000 (wrapped) is about -1 V... the inverting front end. */
    g_adc_count = 65538 - 6000;
    handle_adc_read(0, "{\"cmd\":\"adc_read\",\"source\":\"cal1\"}");
    CHECK(strstr(g_reply, "\"mv\":100") || strstr(g_reply, "\"mv\":99"), "6000 counts in the low range ~1 V: %s", g_reply);
    g_adc_count = 0x8000;
    handle_adc_read(0, "{\"cmd\":\"adc_read\",\"source\":\"current_in\"}");
    CHECK(strstr(g_reply, "\"overrange\":true") && strstr(g_reply, "\"ua\":"), "clipped: %s", g_reply);
    /* cal2 has no low range: refused when asked, and routing it puts the range back to normal. */
    handle_adc_read(0, "{\"cmd\":\"adc_read\",\"source\":\"cal2\",\"range\":\"low\"}");
    CHECK(strstr(g_reply, "err:") && strstr(g_reply, "cal2"), "cal2 low: %s", g_reply);
    g_adc_count = 2;
    handle_adc_read(0, "{\"cmd\":\"adc_read\",\"source\":\"cal2\"}");
    CHECK(strstr(g_reply, "\"range\":\"normal\"") && strstr(g_reply, "\"fullscale_mv\":32"), "cal2 read: %s", g_reply);
    handle_adc_read(0, "{\"cmd\":\"adc_read\",\"source\":\"current_in\",\"range\":\"low\"}");
    CHECK(strstr(g_reply, "\"range\":\"low\"") && g_range == ADC_RANGE_LOW, "read with range: %s", g_reply);
    handle_analog_path(0, "{\"cmd\":\"analog_path\",\"path\":\"off\"}");
    CHECK(strstr(g_reply, "\"range\":\"normal\"") && g_range == ADC_RANGE_NORMAL, "off: %s", g_reply);
    handle_analog_path(0, "{\"cmd\":\"analog_path\",\"path\":\"ext\",\"range\":\"low\"}");
    CHECK(strstr(g_reply, "\"path\":\"adc_ext\"") && strstr(g_reply, "\"range\":\"low\""), "path with range: %s", g_reply);

    /* The self-test reply. */
    g_test[0] = (adc_range_test_pt_t){ 1.0f, 1.001f, 0.998f, true };
    g_test[1] = (adc_range_test_pt_t){ 3.5f, 3.502f, 0.582f, false };
    handle_adc_range(0, "{\"cmd\":\"adc_range\",\"test\":true}");
    CHECK(strstr(g_reply, "\"points\":[{\"set_mv\":1000,\"normal_mv\":1001,\"low_mv\":998,\"ok\":true}") &&
          strstr(g_reply, "\"pass\":false"), "test: %s", g_reply);
    g_low_hw = false;
    g_range = ADC_RANGE_NORMAL;
    g_adc_count = 65522;
}

int main(void) {
    test_reply_with_every_part();
    test_low_range_calibrate();
    test_adc_range_and_read();
    if (fails) { printf("test_ch_calibrate: %d FAILED\n", fails); return 1; }
    printf("test_ch_calibrate: all passed\n");
    return 0;
}
