/*
 * test_ch_dac.c — host tests for the DAC uploads (src/command_handler_dac.c): `load_bin` and its
 * PROTO_LOAD raw mode (bytes counted, newlines and all, into the RAM pool or PSRAM; tunnel acks;
 * the completion reply and the return to JSON), the idle stall guard and its re-arm on progress,
 * the gate an upload holds for its `replay` and its release after LOADED_HOLD_MS, `load`, and the
 * teardown of an abandoned upload.
 *
 * The DAC engine, PSRAM, the heavy gate and the clock are fakes.
 */
#include <stdio.h>
#include <string.h>

#include "command_handler_internal.h"
#include "at_driver.h"
#include "signal_engine.h"
#include "psram.h"
#include "adc_pool.h"
#include "cloud_client.h"
#include "dac_limits.h"
#include "pico_compat.h"

static int fails;
#define CHECK(cond, ...) do { if (!(cond)) { printf("FAIL %s:%d: ", __func__, __LINE__); \
        printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

int shim_rtos_fail_after = -1;

/* ---- fakes ----------------------------------------------------------------------------- */
static uint64_t g_now_us = 1000000u;
uint64_t time_us_64(void) { return g_now_us; }
static void advance_ms(uint32_t ms) { g_now_us += (uint64_t)ms * 1000u; }

static proto_t g_proto[CH_CLOUD_TUNNEL_CONN_LAST + 1];
proto_t conn_proto(int c) { return g_proto[c]; }
void conn_proto_set(int c, proto_t p) { g_proto[c] = p; }

static int g_owner = -1;
bool heavy_in_flight(void) { return false; }
bool heavy_try_claim(int c) { if (g_owner != -1 && g_owner != c) return false; g_owner = c; return true; }
void heavy_release(int c) { if (g_owner == c) g_owner = -1; }
int  heavy_owner_conn(void) { return g_owner; }
static char g_reply[512];
void send_ok_str(int c, const char *p) { (void)c; snprintf(g_reply, sizeof(g_reply), "ok:%s", p); }
void send_error(int c, const char *m) { (void)c; snprintf(g_reply, sizeof(g_reply), "err:%s", m); }
bool heavy_begin(int c) { if (!heavy_try_claim(c)) { send_error(c, "busy"); return false; } return true; }
float parse_sample_rate_hz(const char *json) { (void)json; return 0.0f; }
static char g_lines[512];
void cloud_send_json_line(int c, const char *json) { (void)c; strcat(g_lines, json); strcat(g_lines, "\n"); }
void cloud_client_request_link_snapshot(const char *tag) { (void)tag; }

static uint8_t g_psram[65536];
static bool    g_bus_held;
static uint32_t g_psram_base = 0x1000;
int psram_write(uint32_t a, const uint8_t *b, uint32_t n) { memcpy(g_psram + (a - g_psram_base), b, n); return 0; }
int psram_read(uint32_t a, uint8_t *b, uint32_t n) { memcpy(b, g_psram + (a - g_psram_base), n); return 0; }
void psram_bus_acquire(void) { g_bus_held = true; }
void psram_bus_release(void) { g_bus_held = false; }
void signal_engine_dac_psram_stage(uint32_t total) { (void)total; }
uint32_t signal_engine_dac_psram_base(void) { return g_psram_base; }

static size_t   g_played_len;
static uint32_t g_played_psram;
int dac_generate_arbitrary_rate(const uint8_t *d, size_t n, bool loop, float hz) {
    (void)d; (void)loop; (void)hz;
    if (n > SIGNAL_BUF_SIZE) return -1;   /* signal_engine refuses more than the DAC BRAM */
    g_played_len = n; return 0;
}
int dac_replay_psram(uint32_t count, float hz) { (void)hz; g_played_psram = count; return 0; }
int dac_generate_sine(float f, uint8_t a, uint8_t o, uint32_t d, float hz) { (void)f; (void)a; (void)o; (void)d; (void)hz; return 0; }
int dac_generate_square(float f, uint8_t a, uint8_t o, uint32_t d, float hz) { (void)f; (void)a; (void)o; (void)d; (void)hz; return 0; }
int dac_generate_sawtooth(float f, uint8_t a, uint8_t o, uint32_t d, float hz) { (void)f; (void)a; (void)o; (void)d; (void)hz; return 0; }
int dac_set_constant(uint8_t v, uint32_t div) { (void)v; (void)div; return 0; }
void dac_stop(void) {}
bool fpga_dac_arm_on_capture(void) { return false; }
uint8_t signal_engine_fpga_version(void) { return 47; }
bool signal_engine_has_control_loop(void) { return true; }
int dac_control_loop_start(const uint8_t *c, size_t n, uint16_t k, uint16_t lo, uint16_t hi, uint16_t t) {
    (void)c; (void)n; (void)k; (void)lo; (void)hi; (void)t; return 0;
}
int dac_loop_set_source(uint8_t s, uint16_t i, uint16_t st) { (void)s; (void)i; (void)st; return 0; }
int dac_loop_set_inmap(uint16_t z, int16_t g, uint16_t t, bool m, bool te) { (void)z; (void)g; (void)t; (void)m; (void)te; return 0; }
uint16_t fpga_dac_loop_probe(void) { return 0; }
uint16_t fpga_dac_loop_input(void) { return 0; }
bool fpga_dac_loop_tripped(void) { return false; }
int signal_engine_adc_spi(uint16_t *out) { *out = 0; return 0; }
static dac_limits_t g_lim;
const dac_limits_t *dac_limits_get(void) { return &g_lim; }
const char *dac_limits_set(const dac_limits_t *in) { (void)in; return NULL; }
int dac_limits_clear(void) { return 0; }
const char *dac_limits_path_name(int path) { (void)path; return "5v"; }
int dac_limits_path_index(const char *name) { (void)name; return 1; }
int dac_limits_park_now(void) { return 0; }

/* ---- helpers --------------------------------------------------------------------------- */
static size_t rx(int conn, const uint8_t *b, size_t n) { return load_bin_receive(conn, b, n); }
static void arm(int conn, const char *json) { g_proto[conn] = PROTO_JSON; g_lines[0] = '\0'; handle_load_bin(conn, json); }

/* ---- tests ----------------------------------------------------------------------------- */
static void test_load_bin_refusals(void) {
    handle_load_bin(CH_CLOUD_CONN, "{\"cmd\":\"load_bin\",\"total\":4}");
    CHECK(strcmp(g_reply, "err:load_bin needs a stream connection") == 0, "%s", g_reply);
    arm(0, "{\"cmd\":\"load_bin\"}");
    CHECK(strcmp(g_reply, "err:missing total") == 0, "%s", g_reply);
    arm(0, "{\"cmd\":\"load_bin\",\"total\":0}");
    CHECK(strcmp(g_reply, "err:total out of range") == 0, "%s", g_reply);
    arm(0, "{\"cmd\":\"load_bin\",\"total\":4097}");   /* the DAC BRAM holds SIGNAL_BUF_SIZE bytes */
    CHECK(strcmp(g_reply, "err:total out of range") == 0, "%s", g_reply);
    g_owner = 3;
    arm(0, "{\"cmd\":\"load_bin\",\"total\":4}");
    CHECK(strcmp(g_reply, "err:busy") == 0 && g_proto[0] == PROTO_JSON, "%s", g_reply);
    g_owner = -1;
}

static void test_load_bin_ram(void) {
    arm(0, "{\"cmd\":\"load_bin\",\"total\":8}");
    CHECK(strcmp(g_reply, "ok:{\"ready\":8}") == 0 && g_proto[0] == PROTO_LOAD, "%s", g_reply);
    CHECK(g_owner == 0 && load_bin_owner() == 0, "the upload does not hold the gate");
    const uint8_t data[] = { 1, 0, '\n', 0, 3, 0, 4, 0, '{', '}' };
    CHECK(rx(0, data, 3) == 3 && g_proto[0] == PROTO_LOAD, "first chunk");
    CHECK(rx(0, data + 3, sizeof(data) - 3) == 5, "took more than the upload's bytes");
    CHECK(strcmp(g_reply, "ok:{\"total\":4}") == 0, "completion reply %s", g_reply);
    CHECK(g_proto[0] == PROTO_JSON && load_bin_owner() == -1, "not back to JSON");
    CHECK(memcmp(adc_pool, data, 8) == 0, "bytes not in the pool");
    CHECK(g_owner == 0, "the gate was released before the replay");
    CHECK(g_lines[0] == '\0', "a LAN conn got acks");
    handle_replay(0, "{\"cmd\":\"replay\"}");
    CHECK(strcmp(g_reply, "ok:{\"samples\":4,\"cotrig\":false}") == 0 && g_played_len == 8, "replay %s", g_reply);
    CHECK(g_owner == -1, "replay kept the gate");
}

static void test_load_bin_psram_tunnel(void) {
    const int conn = CH_CLOUD_TUNNEL_CONN;
    static uint8_t data[20000];
    for (size_t i = 0; i < sizeof(data); i++) data[i] = (uint8_t)(i * 7);
    arm(conn, "{\"cmd\":\"load_bin\",\"total\":20000,\"psram\":true}");
    CHECK(g_bus_held && g_proto[conn] == PROTO_LOAD, "bus not held for a PSRAM upload");
    size_t off = 0;
    while (off < sizeof(data)) {
        size_t n = sizeof(data) - off < 3000 ? sizeof(data) - off : 3000;
        off += rx(conn, data + off, n);
    }
    CHECK(strcmp(g_lines, "{\"ack\":9000}\n{\"ack\":18000}\n") == 0, "ack cadence: %s", g_lines);
    CHECK(memcmp(g_psram, data, sizeof(data)) == 0, "bytes not in PSRAM");
    CHECK(!g_bus_held && strcmp(g_reply, "ok:{\"total\":10000}") == 0, "completion: %s", g_reply);
    uint32_t base = 0, len = 0;
    CHECK(command_handler_psram_stage(&base, &len) && base == g_psram_base && len == 20000, "staged region");
    handle_replay(conn, "{\"cmd\":\"replay\",\"samples\":\"5000\"}");
    CHECK(g_played_psram == 5000 && g_owner == -1, "deep replay %s", g_reply);
}

static void test_stall_guard(void) {
    arm(1, "{\"cmd\":\"load_bin\",\"total\":4000}");
    static uint8_t data[4000];
    advance_ms(29000);
    dac_poll();
    CHECK(g_proto[1] == PROTO_LOAD, "timed out early");
    rx(1, data, 2000);                     /* under LOAD_BIN_REARM_BYTES: no re-arm */
    advance_ms(1001);
    dac_poll();
    CHECK(strcmp(g_reply, "err:load_bin timeout") == 0, "no timeout: %s", g_reply);
    CHECK(g_proto[1] == PROTO_JSON && g_owner == -1 && load_bin_owner() == -1, "stall not cleaned up");
    /* progress past LOAD_BIN_REARM_BYTES pushes the deadline out */
    static uint8_t big[20000];
    arm(CH_CLOUD_TUNNEL_CONN, "{\"cmd\":\"load_bin\",\"total\":20000,\"psram\":true}");
    advance_ms(25000);
    rx(CH_CLOUD_TUNNEL_CONN, big, 9000);
    advance_ms(25000);
    dac_poll();
    CHECK(g_proto[CH_CLOUD_TUNNEL_CONN] == PROTO_LOAD, "a progressing upload timed out");
    advance_ms(6000);
    dac_poll();
    CHECK(g_proto[CH_CLOUD_TUNNEL_CONN] == PROTO_JSON && !g_bus_held, "stall after progress not caught");
}

static void test_hold_release(void) {
    arm(2, "{\"cmd\":\"load_bin\",\"total\":4}");
    const uint8_t d[4] = { 1, 2, 3, 4 };
    rx(2, d, 4);
    advance_ms(119000);
    dac_poll();
    CHECK(g_owner == 2, "gate released early");
    advance_ms(2000);
    dac_poll();
    CHECK(g_owner == -1, "gate kept past LOADED_HOLD_MS");
    handle_replay(2, "{\"cmd\":\"replay\"}");
    CHECK(strcmp(g_reply, "err:nothing to replay") == 0, "a dropped RAM upload replayed: %s", g_reply);
}

static void test_load(void) {
    handle_load(0, "{\"cmd\":\"load\",\"offset\":4,\"data\":\"AQI\"}");
    CHECK(strcmp(g_reply, "err:load not started") == 0, "%s", g_reply);
    handle_load(0, "{\"cmd\":\"load\",\"offset\":0,\"data\":\"AQIDBA\"}");
    CHECK(strcmp(g_reply, "ok:{\"offset\":0,\"len\":4,\"total\":2}") == 0 && g_owner == 0, "%s", g_reply);
    handle_load(1, "{\"cmd\":\"load\",\"offset\":0,\"data\":\"AQIDBA\"}");
    CHECK(strcmp(g_reply, "err:busy") == 0, "another conn took a held upload: %s", g_reply);
    handle_load(0, "{\"cmd\":\"load\",\"offset\":4,\"data\":\"BQYHCA\"}");
    CHECK(strcmp(g_reply, "ok:{\"offset\":4,\"len\":4,\"total\":4}") == 0, "%s", g_reply);
    handle_replay(0, "{\"cmd\":\"replay\"}");
    CHECK(g_played_len == 8 && g_owner == -1, "load + replay: %s", g_reply);
}

static void test_teardown(void) {
    arm(CH_CLOUD_TUNNEL_CONN, "{\"cmd\":\"load_bin\",\"total\":100,\"psram\":true}");
    uint8_t d[10] = {0};
    rx(CH_CLOUD_TUNNEL_CONN, d, 10);
    heavy_release(CH_CLOUD_TUNNEL_CONN);   /* command_handler_conn_closed frees the gate */
    dac_conn_closed(CH_CLOUD_TUNNEL_CONN);
    CHECK(!g_bus_held && load_bin_owner() == -1, "abandoned upload left the bus held");
    /* the next upload starts clean */
    arm(0, "{\"cmd\":\"load_bin\",\"total\":2}");
    const uint8_t two[2] = { 9, 9 };
    CHECK(rx(0, two, 2) == 2 && strcmp(g_reply, "ok:{\"total\":1}") == 0, "next upload: %s", g_reply);
    heavy_release(0);
}

int main(void) {
    test_load_bin_refusals();
    test_load_bin_ram();
    test_load_bin_psram_tunnel();
    test_stall_guard();
    test_hold_release();
    test_load();
    test_teardown();
    if (fails) { printf("test_ch_dac: %d FAILED\n", fails); return 1; }
    printf("test_ch_dac: all passed\n");
    return 0;
}
