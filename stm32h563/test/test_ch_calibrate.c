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
int analog_path_set(analog_path_t p) { (void)p; return 0; }
int analog_path_from_name(const char *n, analog_path_t *p) { (void)n; *p = ANALOG_PATH_OFF; return 0; }
const char *analog_path_name(analog_path_t p) { (void)p; return "off"; }
int dac_set_constant16(uint16_t code, uint32_t div) { (void)code; (void)div; return 0; }
int dac_quad_current_out(uint16_t code) { (void)code; return 0; }
int adc_capture_psram(uint16_t *out, size_t n, float rate) { (void)rate; for (size_t i = 0; i < n; i++) out[i] = 65522; return 0; }

int adc_cal_measure_current_in(adc_reading_t *out) {
    uint16_t b[16];
    for (int i = 0; i < 16; i++) b[i] = 49;   /* benchpod-445e46: J8 open at count 49 */
    *out = adc_scale_burst(b, 16, ADC_CAL_CAL1.a, ADC_CAL_CAL1.b);
    return 0;
}

/* A loopback that reads 0.18 % low against the DAC line it was driven with. */
int adc_cal_measure_gain(int src, adc_cal_gain_meas_t *m, uint32_t *span) {
    (void)span;
    cal_lin_t f = adc_cal_fit(src == ADC_CAL_SRC_CAL2 ? ADC_CAL_CAL2 : src == ADC_CAL_SRC_EXT ? ADC_CAL_EXT : ADC_CAL_CAL1);
    for (int i = 0; i < 2; i++) {
        int path = adc_cal_gain_path(src);
        m->v[i] = dac_volts_for_code16(path, dac_code16_for_volts(path, adc_cal_gain_point(src, i)));
        m->count[i] = (float)((m->v[i] / 1.0018 - f.a) / f.b);
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

int main(void) {
    test_reply_with_every_part();
    if (fails) { printf("test_ch_calibrate: %d FAILED\n", fails); return 1; }
    printf("test_ch_calibrate: all passed\n");
    return 0;
}
