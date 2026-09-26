/*
 * test_scpi_paced.c — host tests for SCPI reply pacing (src/scpi_server.c on the
 * real libscpi).
 *
 * A reply goes into the connection's TX ring (conn_tx, 2 KB) which the net task
 * drains into lwIP.  at_send_data() refuses a write the ring cannot take whole,
 * so before pacing a READ? of 4096 points (~24 KB of CSV) was cut off after the
 * first ~2 KB and the connection closed.  The fake connection here behaves like
 * conn_tx: a 2 KB ring, a short write fails, and sleep_ms() is where the "net
 * task" drains it.  Checks:
 *   - a 4096-point READ? arrives whole and in order, with no overrun and no close;
 *   - the same at a slow drain rate, feeding the worker watchdog meanwhile;
 *   - a peer that stops reading gets its reply aborted once (log + close) after
 *     the stall deadline, not after one deadline per 1 KB chunk;
 *   - the next line on a live connection works again (the abort is per reply);
 *   - short replies are unchanged.
 */
#include "scpi_server.h"
#include "at_driver.h"
#include "command_handler.h"
#include "signal_engine.h"
#include "target_power.h"
#include "wifi_manager.h"
#include "device_identity.h"
#include "watchdog.h"
#include "pico/time.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

static int failures;
#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); failures++; } \
} while (0)

/* ---- fake time + the net task draining the ring ------------------------------ */
static uint64_t s_now_us = 1000000;
uint64_t time_us_64(void) { return s_now_us; }
uint32_t time_us_32(void) { return (uint32_t)s_now_us; }

#define RING 2048u
static struct {
    size_t used;             /* bytes queued in the ring */
    size_t drain_per_ms;     /* how fast the "net task" empties it */
    bool   dead;             /* connection gone: avail 0 */
    char   out[64 * 1024];   /* everything the peer received, in order */
    size_t out_len;
    int    overruns;         /* at_send_data asked for more than the ring had */
    int    closes;
    int    heartbeats;
    int    sleeps;
} K;

void sleep_ms(uint32_t ms) {
    K.sleeps++;
    s_now_us += (uint64_t)ms * 1000u;
    size_t d = K.drain_per_ms * ms;
    K.used = d >= K.used ? 0 : K.used - d;
}

size_t at_send_avail(int conn_id) { (void)conn_id; return K.dead ? 0 : RING - K.used; }
int at_send_data(int conn_id, const uint8_t *buf, size_t len) {
    (void)conn_id;
    if (K.dead || len > RING - K.used) { K.overruns++; return -1; }
    memcpy(K.out + K.out_len, buf, len);
    K.out_len += len;
    K.used += len;
    return 0;
}
int at_close_connection(int conn_id) { (void)conn_id; K.closes++; return 0; }
void watchdog_heartbeat(int task, const char *name) { (void)name; if (task == WD_TASK_WORKER) K.heartbeats++; }

/* ---- the instrument, faked ---------------------------------------------------- */
static uint16_t sample(size_t i) { return (uint16_t)(60000u + (i * 7u) % 5000u); }   /* 5 digits */

bool command_handler_acquire_adc(int conn_id) { (void)conn_id; return true; }
void command_handler_release_adc(int conn_id) { (void)conn_id; }
int  adc_capture_psram(uint16_t *out16, size_t samples, float rate) {
    (void)rate;
    for (size_t i = 0; i < samples; i++) out16[i] = sample(i);
    return 0;
}
int  command_handler_dig_output(unsigned la, int level) { (void)la; (void)level; return 0; }
int  command_handler_dig_step(unsigned la, uint32_t s, uint32_t d, unsigned dl, int dir) {
    (void)la; (void)s; (void)d; (void)dl; (void)dir; return 0;
}
bool command_handler_step_busy(void) { return false; }
int  dac_generate_arbitrary_rate(const uint8_t *d, size_t n, bool loop, float r) { (void)d; (void)n; (void)loop; (void)r; return 0; }
int  dac_generate_sine(float f, uint8_t a, uint8_t o, uint32_t d, float r) { (void)f; (void)a; (void)o; (void)d; (void)r; return 0; }
int  dac_generate_square(float f, uint8_t a, uint8_t o, uint32_t d, float r) { (void)f; (void)a; (void)o; (void)d; (void)r; return 0; }
int  dac_generate_sawtooth(float f, uint8_t a, uint8_t o, uint32_t d, float r) { (void)f; (void)a; (void)o; (void)d; (void)r; return 0; }
void dac_stop(void) {}
int  fpga_dual_capture(uint16_t *a, uint16_t ac, uint16_t ad, uint16_t *l, uint16_t lc, uint16_t ld) {
    (void)a; (void)ac; (void)ad; (void)l; (void)lc; (void)ld; return -1;
}
int  measure_psram(uint16_t *o, const char *w, float f, uint8_t a, uint8_t off, size_t n, float r) {
    (void)o; (void)w; (void)f; (void)a; (void)off; (void)n; (void)r; return -1;
}
int  device_identity_get_public(uint8_t pub[DEVICE_ID_PUBLIC_LEN]) { (void)pub; return -1; }
int  device_identity_sign_ctx(const char *c, const uint8_t *m, size_t n, uint8_t sig[DEVICE_ID_SIG_LEN]) {
    (void)c; (void)m; (void)n; (void)sig; return -1;
}
int  target_power_enable(int efuse, bool on) { (void)efuse; (void)on; return 0; }
int  target_power_get_status(int efuse, target_power_status_t *out) { (void)efuse; memset(out, 0, sizeof(*out)); return 0; }
const char *wifi_get_ip(void) { return "0.0.0.0"; }
int  wifi_get_rssi(int *dbm) { (void)dbm; return -1; }
wifi_state_t wifi_get_state(void) { return WIFI_DISCONNECTED; }

/* ---- helpers ------------------------------------------------------------------- */
static void reset_conn(size_t drain) {
    memset(&K, 0, sizeof(K));
    K.drain_per_ms = drain;
}

/* Parse the CSV the peer received and check it is exactly sample(0..n-1). */
static bool reply_is_samples(size_t n) {
    K.out[K.out_len] = '\0';
    const char *p = K.out;
    for (size_t i = 0; i < n; i++) {
        char *end;
        unsigned long v = strtoul(p, &end, 10);
        if (end == p || v != sample(i)) {
            printf("  sample %zu: got \"%.12s\" want %u\n", i, p, (unsigned)sample(i));
            return false;
        }
        p = end;
        if (i + 1 < n) { if (*p != ',') return false; p++; }
    }
    return *p == '\r' || *p == '\n';
}

static void test_big_read_arrives_whole(void) {
    reset_conn(1024);                          /* ~1 MB/s: a normal LAN peer */
    scpi_dispatch_line(0, "READ? 4096");
    CHECK(K.out_len > 20000);                  /* 4096 x "6xxxx," */
    CHECK(reply_is_samples(4096));
    CHECK(K.overruns == 0);
    CHECK(K.closes == 0);
}

static void test_slow_peer_feeds_watchdog(void) {
    reset_conn(8);                             /* 8 KB/s: ~3 s for the reply */
    uint64_t t0 = s_now_us;
    scpi_dispatch_line(0, "READ? 4096");
    CHECK(reply_is_samples(4096));
    CHECK(K.overruns == 0);
    CHECK(K.closes == 0);
    CHECK(s_now_us - t0 > 2000000);            /* it really waited on the drain */
    CHECK(K.heartbeats > 1000);                /* and proved liveness while it did */
}

static void test_stalled_peer_aborts_once(void) {
    reset_conn(0);                             /* peer stopped reading */
    uint64_t t0 = s_now_us;
    scpi_dispatch_line(0, "READ? 4096");
    uint64_t waited_ms = (s_now_us - t0) / 1000u;
    CHECK(K.closes == 1);
    CHECK(K.overruns == 0);
    CHECK(K.out_len <= RING);                  /* only what fit before the stall */
    CHECK(waited_ms >= 4900 && waited_ms <= 6000);   /* one stall deadline, not one per chunk */
    CHECK(K.heartbeats > 0);

    /* The abort is per reply: the next line on a live connection is answered. */
    reset_conn(1024);
    scpi_dispatch_line(0, "SYST:PING?");
    K.out[K.out_len] = '\0';
    CHECK(strncmp(K.out, "PONG", 4) == 0);
    CHECK(K.closes == 0);
}

static void test_dead_conn_aborts(void) {
    reset_conn(1024);
    K.dead = true;                             /* conn gone: avail 0 from the start */
    scpi_dispatch_line(0, "READ? 4096");
    CHECK(K.closes == 1);
    CHECK(K.out_len == 0);
}

static void test_short_reply_unchanged(void) {
    reset_conn(1024);
    scpi_dispatch_line(0, "*IDN?");
    K.out[K.out_len] = '\0';
    CHECK(strstr(K.out, "EmbeddedCI,BenchPod") == K.out);
    CHECK(K.sleeps == 0);                      /* room available: no waiting at all */
    CHECK(K.closes == 0);
}

int main(void) {
    test_short_reply_unchanged();
    test_big_read_arrives_whole();
    test_slow_peer_feeds_watchdog();
    test_stalled_peer_aborts_once();
    test_dead_conn_aborts();
    if (failures) { printf("FAIL test_scpi_paced: %d failure(s)\n", failures); return 1; }
    printf("PASS test_scpi_paced\n");
    return 0;
}
