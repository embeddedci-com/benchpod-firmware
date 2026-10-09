/*
 * test_ch_capture.c — host tests for captures and the paced bulk read-back
 * (src/command_handler_capture.c): the chunk framing (decimal and "enc":"b64"), pacing against
 * the send ring, the deep capture_dual read-back (dense ADC region, then the LA region as
 * run-length edges) with its rates, the capture_read resume, the stall that drops a reader who
 * stopped, the trigger timeout, and the teardown of a capture whose conn went away.
 *
 * The capture engine, PSRAM, the heavy gate, the send ring and the clocks are fakes; replies are
 * parsed back into samples and compared with what the fake engine captured.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "command_handler_internal.h"
#include "at_driver.h"
#include "signal_engine.h"
#include "sensor_sim.h"
#include "psram.h"
#include "b64url.h"
#include "stm32h5xx_hal.h"
#include "pico_compat.h"

static int fails;
#define CHECK(cond, ...) do { if (!(cond)) { printf("FAIL %s:%d: ", __func__, __LINE__); \
        printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

/* ---- fakes ----------------------------------------------------------------------------- */
static uint64_t g_now_us = 1000000u;
static uint32_t g_tick;
uint64_t time_us_64(void) { return g_now_us; }
uint32_t HAL_GetTick(void) { return g_tick; }
static void advance_ms(uint32_t ms) { g_now_us += (uint64_t)ms * 1000u; g_tick += ms; }

static int g_owner = -1;
static char g_err[256];
void send_error(int c, const char *m) { (void)c; snprintf(g_err, sizeof(g_err), "%s", m); }
bool heavy_begin(int c) { if (g_owner != -1 && g_owner != c) { send_error(c, "busy"); return false; } g_owner = c; return true; }
void heavy_release(int c) { if (g_owner == c) g_owner = -1; }
float parse_sample_rate_hz(const char *json) { (void)json; return 0.0f; }
bool require_la_voltage(int c) { (void)c; return true; }
static size_t g_note;
void dac_trace_note_capture(size_t samples) { g_note = samples; }
static int g_dac_stops;
void dac_stop(void) { g_dac_stops++; }

static char  *g_tx;      /* everything sent on the conn */
static size_t g_ntx, g_cap, g_avail = 1u << 20;
static int    g_closed = -1;
int at_send_data(int c, const uint8_t *b, size_t n) {
    (void)c;
    if (n > g_avail) return -1;
    if (g_ntx + n + 1 > g_cap) { g_cap = (g_ntx + n + 1) * 2; g_tx = realloc(g_tx, g_cap); }
    memcpy(g_tx + g_ntx, b, n); g_ntx += n; g_tx[g_ntx] = '\0';
    return 0;
}
size_t at_send_avail(int c) { (void)c; return g_avail; }
int at_close_connection(int c) { g_closed = c; return 0; }

/* the capture engine: g_adc / g_la are what it "captured" */
static uint16_t g_adc[3000], g_la[3000];
static int  g_wait_rc, g_aborts, g_releases, g_trig_status;
static bool g_bus;
int adc_capture_psram_start(size_t n, float hz) { (void)n; (void)hz; return 0; }
int adc_capture_psram_poll(uint16_t *out, size_t n) { if (g_wait_rc > 0) memcpy(out, g_adc, n * 2); return g_wait_rc; }
int measure_psram_start(const char *w, float f, uint8_t a, uint8_t o, size_t n, float hz) {
    (void)w; (void)f; (void)a; (void)o; (void)n; (void)hz; return 0;
}
int fpga_dual_capture_start_hz(uint32_t an, float ahz, uint32_t ln, float lhz, float *aact, float *lact) {
    (void)an; (void)ahz; (void)ln; (void)lhz; *aact = 400000.0f; *lact = 24000000.0f; return 0;
}
int fpga_capture_psram_wait(void) { if (g_wait_rc > 0) g_bus = true; return g_wait_rc; }
int fpga_capture_psram_read16(size_t idx, uint16_t *out, size_t n, size_t adc_n) {
    for (size_t i = 0; i < n; i++) out[i] = (idx + i < adc_n) ? g_adc[idx + i] : g_la[idx + i - adc_n];
    return 0;
}
void fpga_capture_psram_release(void) { g_bus = false; g_releases++; }
void fpga_capture_abort(void) { g_aborts++; }
bool fpga_capture_adc_sentinel_survived(void) { return false; }
void fpga_capture_extend_deadline_ms(uint32_t ms) { (void)ms; }
/* the deep-LA engine rounds the request to a whole divider: 2.304 MHz -> 24 MHz / 11 */
static float g_la_req_hz;
int fpga_la_capture_psram_start(size_t n, float hz, float *act) { (void)n; g_la_req_hz = hz; if (act) *act = 24000000.0f / 11.0f; return 0; }
int fpga_la_capture_psram_wait(void) { if (g_wait_rc > 0) g_bus = true; return g_wait_rc; }
int fpga_la_psram_read(uint32_t off, uint8_t *b, uint32_t n) { (void)off; memset(b, 0, n); return 0; }
void fpga_la_psram_release(void) { g_bus = false; }
int fpga_set_capture_bases(uint32_t la, uint32_t adc) { (void)la; (void)adc; return 0; }
void fpga_set_dac_stop_after_us(uint32_t us) { (void)us; }
int fpga_set_trigger(unsigned la, uint8_t mode) { (void)la; (void)mode; return 0; }
int fpga_trigger_status(uint8_t *st) { *st = (uint8_t)g_trig_status; return 0; }
int fpga_i2c_la_capture(uint8_t *b, size_t n, float hz) { (void)hz; for (size_t i = 0; i < n; i++) b[i] = (uint8_t)i; return 0; }
void psram_bus_acquire(void) { g_bus = true; }
bool sensor_sim_active(void) { return true; }
int sensor_sim_read_regs(uint8_t s, uint8_t *b, size_t n) { for (size_t i = 0; i < n; i++) b[i] = (uint8_t)(s + i); return 0; }
uint8_t  signal_engine_fpga_version(void) { return 47; }
uint32_t signal_engine_adc_cap_base(void) { return 0x400000u; }
uint32_t signal_engine_la_cap_base(void) { return 0; }
uint32_t signal_engine_dac_resident_bytes(void) { return 0; }
uint32_t signal_engine_la_max_samples(bool adc) { (void)adc; return 4000000u; }
uint32_t signal_engine_adc_max_samples(void) { return 2000000u; }
void signal_engine_psram_report_wedge(void) {}

/* ---- reply parsing --------------------------------------------------------------------- */
/* Walk the newline-separated frames of g_tx: dense values go to dense[], LA edges are expanded
   into la[] (up to each frame's la_upto). Returns the number of frames; *last_more is the last
   frame's "more", *first is a copy of the first frame. */
static int parse(uint16_t *dense, size_t *ndense, uint16_t *la, size_t *nla, bool *last_more, char *first, size_t fcap) {
    int frames = 0;
    *ndense = 0; *nla = 0; *last_more = true;
    long la_prev = -1; unsigned la_word = 0;
    char *line = g_tx;
    while (line && *line) {
        char *nl = strchr(line, '\n');
        if (!nl) break;
        *nl = '\0';
        if (frames == 0 && first) snprintf(first, fcap, "%s", line);
        frames++;
        char *p;
        if ((p = strstr(line, "\"data\":["))) {
            p += 8;
            while (*p && *p != ']') { dense[(*ndense)++] = (uint16_t)strtoul(p, &p, 10); if (*p == ',') p++; }
        } else if ((p = strstr(line, "\"b64\":\""))) {
            p += 7;
            char *q = strchr(p, '"'); *q = '\0';
            size_t got = 0;
            b64url_decode(p, (uint8_t *)(dense + *ndense), 65536, &got);
            *ndense += got / 2;
            *q = '"';
        } else if ((p = strstr(line, "\"la_edges\":["))) {
            p += 12;
            while (*p == '[') {
                unsigned long idx = strtoul(p + 1, &p, 10);
                unsigned long w = strtoul(p + 1, &p, 10);
                for (long i = la_prev + 1; i < (long)idx; i++) la[i] = (uint16_t)la_word;
                la[idx] = (uint16_t)w; la_word = (unsigned)w; la_prev = (long)idx;
                p += 1; if (*p == ',') p++;
            }
            char *u = strstr(line, "\"la_upto\":");
            unsigned long upto = strtoul(u + 10, NULL, 10);
            for (long i = la_prev + 1; i < (long)upto; i++) la[i] = (uint16_t)la_word;
            if ((long)upto - 1 > la_prev) la_prev = (long)upto - 1;
            *nla = upto;
        }
        *last_more = strstr(line, "\"more\":true") != NULL;
        *nl = '\n';
        line = nl + 1;
    }
    return frames;
}
static void clear_tx(void) { g_ntx = 0; if (g_tx) g_tx[0] = '\0'; g_closed = -1; g_err[0] = '\0'; }
static void pump_all(void) { for (int i = 0; i < 10000 && capture_busy(); i++) capture_poll(); }

static uint16_t d[8192], l[8192];
static size_t nd, nl;
static bool more;
static char first[256];

/* ---- tests ----------------------------------------------------------------------------- */
static void test_test_pattern_decimal_paced(void) {
    clear_tx();
    g_avail = 400;   /* forces many small chunks */
    handle_test(0, "{\"cmd\":\"test\",\"pattern\":\"counter\",\"samples\":300}");
    CHECK(capture_busy() && g_owner == 0, "test did not start a bulk send");
    pump_all();
    int frames = parse(d, &nd, l, &nl, &more, first, sizeof(first));
    CHECK(frames > 3, "expected paced chunks, got %d frame(s)", frames);
    CHECK(strncmp(first, "{\"status\":\"ok\",\"bits\":16,\"data\":[0,1,", 36) == 0, "first frame: %.60s", first);
    CHECK(strstr(g_tx, "\n{\"status\":\"chunk\",\"data\":[") != NULL, "later frames are not chunk frames");
    CHECK(nd == 300 && !more, "got %zu samples, more=%d", nd, more);
    bool ok = true;
    for (size_t i = 0; i < nd; i++) if (d[i] != i) ok = false;
    CHECK(ok, "counter pattern corrupted");
    CHECK(g_owner == -1 && !capture_busy(), "gate not released after the last chunk");
    /* every frame fit the room the ring had */
    char *p = g_tx, *nlp;
    while ((nlp = strchr(p, '\n'))) { CHECK((size_t)(nlp - p + 1) <= 400, "a frame of %zu bytes overran the ring", (size_t)(nlp - p + 1)); p = nlp + 1; }
    g_avail = 1u << 20;
}

static void test_test_pattern_b64(void) {
    clear_tx();
    handle_test(0, "{\"cmd\":\"test\",\"pattern\":\"ramp\",\"samples\":1000,\"enc\":\"b64\"}");
    pump_all();
    parse(d, &nd, l, &nl, &more, first, sizeof(first));
    CHECK(strncmp(first, "{\"status\":\"ok\",\"bits\":16,\"b64\":\"", 31) == 0, "first frame: %.60s", first);
    CHECK(nd == 1000 && !more && d[0] == 0 && d[999] == 65535, "b64 samples: %zu, %u..%u", nd, d[0], d[999]);
}

static void test_capture_dual(void) {
    for (size_t i = 0; i < 1000; i++) g_adc[i] = (uint16_t)(i * 3);
    for (size_t i = 0; i < 2000; i++) g_la[i] = (uint16_t)((i / 300) & 0xFFF);   /* edges every 300 */
    clear_tx();
    g_wait_rc = 0;
    handle_capture_dual(1, "{\"cmd\":\"capture_dual\",\"adc_samples\":1000,\"la_samples\":2000}");
    CHECK(capture_busy() && g_owner == 1, "capture_dual did not arm");
    capture_poll();
    CHECK(g_ntx == 0, "replied before the capture completed");
    g_wait_rc = 1;
    pump_all();
    parse(d, &nd, l, &nl, &more, first, sizeof(first));
    CHECK(strncmp(first, "{\"status\":\"ok\",\"bits\":16,\"adc_rate_hz\":400000,\"la_rate_hz\":24000000,\"data\":[", 75) == 0,
          "first frame: %.90s", first);
    CHECK(nd == 1000 && nl == 2000 && !more, "adc %zu la %zu more %d", nd, nl, more);
    bool ok = true;
    for (size_t i = 0; i < 1000; i++) if (d[i] != g_adc[i]) ok = false;
    for (size_t i = 0; i < 2000; i++) if (l[i] != g_la[i]) ok = false;
    CHECK(ok, "capture data corrupted");
    CHECK(!g_bus && g_owner == -1, "bus or gate held after the read-back");
    size_t edges = 0;
    for (char *p = g_tx; (p = strstr(p, "],[")); p++) edges++;
    CHECK(edges < 20, "LA region was not run-length encoded (%zu separators)", edges);

    /* resume from the middle of the LA region */
    clear_tx();
    handle_capture_read(1, "{\"cmd\":\"capture_read\",\"offset\":1500}");
    pump_all();
    CHECK(strncmp(g_tx, "{\"status\":\"chunk\",\"la\":true,\"la_edges\":[[500,", 44) == 0, "resume frame: %.60s", g_tx);
    CHECK(strstr(g_tx, "\"la_upto\":2000,\"more\":false}") != NULL, "resume did not reach the end");
    clear_tx();
    handle_capture_read(1, "{\"cmd\":\"capture_read\",\"offset\":3000}");
    CHECK(strcmp(g_tx, "{\"status\":\"ok\",\"data\":[],\"more\":false}\n") == 0 && g_owner == -1, "resume at the end: %s", g_tx);
    handle_capture_read(1, "{\"cmd\":\"capture_read\",\"offset\":3001}");
    CHECK(strcmp(g_err, "offset out of range") == 0, "%s", g_err);
    g_wait_rc = 0;
}

/* la_capture reports the rate it achieved, not the one asked for: a 2.304 MHz request runs at
   24 MHz / 11 = 2181818 Hz, and a host that labels the trace 2.304 MHz decodes 115200 baud with
   ~6% timing error (80% framing errors on the bench). The resume keeps the same rate. */
static void test_la_capture_reports_achieved_rate(void) {
    for (size_t i = 0; i < 2000; i++) g_la[i] = (uint16_t)((i / 100) & 1);
    clear_tx();
    g_wait_rc = 0;
    handle_la_capture(6, "{\"cmd\":\"la_capture\",\"samples\":2000,\"sample_rate_mhz\":2.304}");
    CHECK(capture_busy() && g_owner == 6, "la_capture did not arm: %s", g_err);
    g_wait_rc = 1;
    pump_all();
    parse(d, &nd, l, &nl, &more, first, sizeof(first));
    CHECK(strstr(first, "\"la_rate_hz\":2181818,") != NULL, "first frame: %.90s", first);
    CHECK(nl == 2000 && !more && l[100] == 1 && l[199] == 1 && l[200] == 0, "la %zu more %d", nl, more);
    CHECK(!g_bus && g_owner == -1, "bus or gate held after the read-back");

    /* a capture_read resume carries the same timebase as the first attempt */
    clear_tx();
    handle_capture_read(6, "{\"cmd\":\"capture_read\",\"offset\":0}");
    pump_all();
    CHECK(strstr(g_tx, "\"la_rate_hz\":2181818,") != NULL, "resume frame: %.90s", g_tx);
    g_wait_rc = 0;
}

static void test_stall(void) {
    clear_tx();
    g_avail = 50;   /* below the pump's floor */
    handle_test(2, "{\"cmd\":\"test\",\"samples\":100}");
    capture_poll();
    advance_ms(29000);
    capture_poll();
    CHECK(capture_busy() && g_closed == -1, "ended before BULK_STALL_MS");
    advance_ms(1500);
    capture_poll();
    CHECK(!capture_busy() && g_closed == 2 && g_owner == -1, "a reader who stopped was not dropped");
    g_avail = 1u << 20;
}

static void test_trigger_timeout(void) {
    clear_tx();
    int aborts = g_aborts;
    handle_capture(3, "{\"cmd\":\"capture\",\"samples\":64,\"trigger\":{\"la\":9,\"edge\":\"rising\"},\"trigger_timeout_ms\":1000}");
    CHECK(capture_busy() && g_err[0] == '\0', "triggered capture refused: %s", g_err);
    CHECK(g_note == 64, "the RAM capture is not the next replay's trace");
    advance_ms(500);
    capture_poll();
    CHECK(capture_busy(), "timed out early");
    advance_ms(600);
    capture_poll();
    CHECK(strncmp(g_err, "trigger timeout:", 16) == 0, "error: %s", g_err);
    CHECK(!capture_busy() && g_owner == -1 && g_aborts == aborts + 1, "timed-out capture not aborted");
}

static void test_conn_closed(void) {
    int aborts = g_aborts, stops = g_dac_stops;
    handle_measure(4, "{\"cmd\":\"measure\",\"waveform\":\"sine\",\"samples\":64}");
    CHECK(capture_busy(), "measure did not arm");
    heavy_release(4);           /* command_handler_conn_closed frees the gate after the hook */
    capture_conn_closed(4);
    CHECK(!capture_busy() && g_aborts == aborts + 1 && g_dac_stops == stops + 1,
          "a dead conn's measure kept running (aborts %d, dac stops %d)", g_aborts - aborts, g_dac_stops - stops);
    /* someone else's capture survives */
    g_wait_rc = 0;
    handle_capture(5, "{\"cmd\":\"capture\",\"samples\":8}");
    capture_conn_closed(4);
    CHECK(capture_busy(), "closing another conn aborted this capture");
    capture_conn_closed(5);
    heavy_release(5);
}

int main(void) {
    test_test_pattern_decimal_paced();
    test_test_pattern_b64();
    test_capture_dual();
    test_la_capture_reports_achieved_rate();
    test_stall();
    test_trigger_timeout();
    test_conn_closed();
    free(g_tx);
    if (fails) { printf("test_ch_capture: %d FAILED\n", fails); return 1; }
    printf("test_ch_capture: all passed\n");
    return 0;
}
