/*
 * test_scpi_paced.c — host tests for SCPI reply pacing (src/scpi_server.c on the
 * real libscpi).
 *
 * A reply goes into the connection's TX ring (conn_tx, 2 KB) which the net task
 * drains into lwIP.  at_send_data() refuses a write the ring cannot take whole,
 * so before pacing a READ? of 4096 points (~24 KB of CSV) was cut off after the
 * first ~2 KB and the connection closed.  The fake connection here behaves like
 * conn_tx: a 2 KB ring, a write that does not fit fails and queues nothing
 * (all or nothing), and sleep_ms() is where the "net task" drains it.  Checks:
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
#include "b64url.h"
#include "cmd_gate.h"
#include "adc_pool.h"

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

/* The device gate is the real cmd_gate_device over a faked pod state, so the JSON verb each SCPI
   command asks with is checked against the real command table. */
static bool g_skip_hw, g_digital;
const char *command_handler_device_gate(const char *verb, const char *json) {
    return cmd_gate_device(verb, json, g_skip_hw, !g_digital);
}
static int g_captures, g_dual_calls, g_dig_calls;
static bool g_dual_ok;
static bool g_gate_busy;     /* another client holds the heavy gate */
bool command_handler_acquire_adc(int conn_id) { (void)conn_id; return !g_gate_busy; }
void command_handler_release_adc(int conn_id) { (void)conn_id; }
int  adc_capture_psram(uint16_t *out16, size_t samples, float rate) {
    (void)rate;
    g_captures++;
    for (size_t i = 0; i < samples; i++) out16[i] = sample(i);
    return 0;
}
int  command_handler_dig_output(unsigned la, int level) { (void)la; (void)level; g_dig_calls++; return 0; }
int  command_handler_dig_step(unsigned la, uint32_t s, uint32_t d, unsigned dl, int dir) {
    (void)la; (void)s; (void)d; (void)dl; (void)dir; return 0;
}
bool command_handler_step_busy(void) { return false; }
static int g_generates;      /* DAC waveform starts, to prove a refused OUTPut never reached it */
static bool g_limits_on;     /* dac_limits enabled? */
const char *dac_limits_check_raw(const char *what) { (void)what; return g_limits_on ? "refused: limits" : NULL; }
static uint8_t g_replay[8192];  /* what the last USER replay handed the DAC */
static size_t  g_replay_len;
int  dac_generate_arbitrary_rate(const uint8_t *d, size_t n, bool loop, float r) {
    (void)loop; (void)r;
    g_replay_len = n;
    memcpy(g_replay, d, n < sizeof(g_replay) ? n : sizeof(g_replay));
    g_generates++;
    return 0;
}
int  dac_generate_sine(float f, uint8_t a, uint8_t o, uint32_t d, float r) { (void)f; (void)a; (void)o; (void)d; (void)r; g_generates++; return 0; }
int  dac_generate_square(float f, uint8_t a, uint8_t o, uint32_t d, float r) { (void)f; (void)a; (void)o; (void)d; (void)r; return 0; }
int  dac_generate_sawtooth(float f, uint8_t a, uint8_t o, uint32_t d, float r) { (void)f; (void)a; (void)o; (void)d; (void)r; return 0; }
void dac_stop(void) {}
int  fpga_dual_capture(uint16_t *a, uint16_t ac, uint16_t ad, uint16_t *l, uint16_t lc, uint16_t ld) {
    (void)ad; (void)ld;
    g_dual_calls++;
    if (!g_dual_ok) return -1;
    for (uint16_t i = 0; i < ac; i++) a[i] = (uint16_t)(1000u + i);
    for (uint16_t i = 0; i < lc; i++) l[i] = (uint16_t)(0xF000u | i);   /* bits above the 12 LA bits */
    return 0;
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

/* dac_limits must hold on SCPI too: with limits set, OUTPut ON and MEASure? (both play a raw
   waveform) are refused with -221 and never reach the DAC; without limits OUTPut ON still works. */
static void test_dac_limits_refuse_raw_output(void) {
    g_limits_on = true;
    g_generates = 0;
    reset_conn(4096);
    scpi_dispatch_line(0, "OUTP ON");
    scpi_dispatch_line(0, "MEAS?");
    CHECK(g_generates == 0);
    reset_conn(4096);
    scpi_dispatch_line(0, "SYST:ERR?");
    K.out[K.out_len] = '\0';
    CHECK(strstr(K.out, "-221") != NULL);

    g_limits_on = false;
    scpi_dispatch_line(0, "*CLS");
    scpi_dispatch_line(0, "OUTP ON");
    CHECK(g_generates == 1);
    scpi_dispatch_line(0, "OUTP OFF");
}

/* SOURce:FUNCtion USER replays what READ? captured (16-bit samples as LE byte pairs, like the
   JSON replay), capped at the DAC BRAM; TRACe:DATA uploads replace it; DIAGnostic:PATTern? does
   not touch it.  USER used to replay the 8-bit pattern/upload buffer instead of the capture. */
static void test_user_replays_the_capture(void) {
    reset_conn(4096);
    scpi_dispatch_line(0, "READ? 16");
    reset_conn(4096);
    scpi_dispatch_line(0, "DIAG:PATT? CONS,7,64");
    scpi_dispatch_line(0, "FUNC USER");
    g_replay_len = 0;
    scpi_dispatch_line(0, "OUTP ON");
    CHECK(g_replay_len == 32);
    bool same = true;
    for (size_t i = 0; i < 16; i++) {
        uint16_t v = (uint16_t)(g_replay[2 * i] | (g_replay[2 * i + 1] << 8));
        if (v != sample(i)) same = false;
    }
    CHECK(same);
    scpi_dispatch_line(0, "OUTP OFF");

    /* a capture longer than the BRAM replays its first SIGNAL_MAX_SAMPLES samples */
    reset_conn(1024);
    scpi_dispatch_line(0, "READ? 4096");
    scpi_dispatch_line(0, "OUTP ON");
    CHECK(g_replay_len == SIGNAL_MAX_SAMPLES * 2u);
    scpi_dispatch_line(0, "OUTP OFF");
    reset_conn(4096);
    scpi_dispatch_line(0, "TRAC:POIN?");
    K.out[K.out_len] = '\0';
    CHECK(atoi(K.out) == (int)SIGNAL_MAX_SAMPLES);

    /* an upload replaces it: 3 samples, reported in samples */
    const uint8_t up[6] = { 0x34, 0x12, 0x78, 0x56, 0xBC, 0x9A };
    char b64[16], line[64];
    b64url_encode(up, sizeof(up), b64, sizeof(b64));
    snprintf(line, sizeof(line), "TRAC:DATA 0,\"%s\"", b64);
    scpi_dispatch_line(0, line);
    reset_conn(4096);
    scpi_dispatch_line(0, "TRAC:POIN?");
    K.out[K.out_len] = '\0';
    CHECK(atoi(K.out) == 3);
    scpi_dispatch_line(0, "OUTP ON");
    CHECK(g_replay_len == 6 && memcmp(g_replay, up, 6) == 0);
    scpi_dispatch_line(0, "OUTP OFF");
    scpi_dispatch_line(0, "FUNC SIN");
}

/* The last queued SCPI error, as SYST:ERR? reports it (the error queue is drained). */
static int next_error(char *msg, size_t cap) {
    reset_conn(4096);
    scpi_dispatch_line(0, "SYST:ERR?");
    K.out[K.out_len] = '\0';
    if (msg) snprintf(msg, cap, "%s", K.out);
    int code = atoi(K.out);
    scpi_dispatch_line(0, "*CLS");
    return code;
}

/* SCPI gets the safe-mode and digital-board refusals JSON gets (it used to skip both): hardware
   commands are refused before they reach the instrument, the rest keep working. */
static void test_safe_mode_and_digital_board(void) {
    char msg[256];
    scpi_dispatch_line(0, "*CLS");
    g_skip_hw = true;
    g_captures = g_generates = g_dual_calls = g_dig_calls = 0;
    static const char *const refused[] = {
        "READ? 16", "MEAS?", "OUTP ON", "OUTP OFF", "DIAG:CAP? 0,16", "DIG:OUTP 3,1",
        "DIG:STEP 3,10,100", "DIG:STEP:BUSY?", "TRAC:DATA 0,\"AAAA\"",
    };
    for (size_t i = 0; i < sizeof(refused) / sizeof(refused[0]); i++) {
        reset_conn(4096);
        scpi_dispatch_line(0, refused[i]);
        int code = next_error(msg, sizeof(msg));
        if (code != -240) printf("  %s: %s\n", refused[i], msg);
        CHECK(code == -240);
        CHECK(strstr(msg, "safe mode") != NULL);
    }
    CHECK(g_captures == 0 && g_generates == 0 && g_dual_calls == 0 && g_dig_calls == 0);
    reset_conn(4096);
    scpi_dispatch_line(0, "SYST:PING?");
    K.out[K.out_len] = '\0';
    CHECK(strncmp(K.out, "PONG", 4) == 0);
    scpi_dispatch_line(0, "OUTP:POW1 ON");    /* target power runs in safe mode */
    scpi_dispatch_line(0, "FREQ 500");        /* a setting touches no hardware */
    CHECK(next_error(NULL, 0) == 0);
    g_skip_hw = false;

    g_digital = true;
    g_captures = g_generates = g_dual_calls = g_dig_calls = 0;
    reset_conn(4096);
    scpi_dispatch_line(0, "READ? 16");
    CHECK(next_error(msg, sizeof(msg)) == -241);
    CHECK(strstr(msg, "no analog front end") != NULL);
    scpi_dispatch_line(0, "OUTP ON");
    CHECK(next_error(NULL, 0) == -241);
    scpi_dispatch_line(0, "DIAG:CAP? 16,16");
    CHECK(next_error(NULL, 0) == -241);
    CHECK(g_captures == 0 && g_generates == 0 && g_dual_calls == 0);
    /* LA-only and digital commands still reach the instrument */
    reset_conn(4096);
    scpi_dispatch_line(0, "DIAG:CAP? 0,16");
    CHECK(g_dual_calls == 1);                 /* (the fake capture then fails: -200) */
    scpi_dispatch_line(0, "DIG:OUTP 3,1");
    CHECK(g_dig_calls == 1);
    scpi_dispatch_line(0, "OUTP OFF");        /* dac_stop: harmless cleanup is allowed */
    scpi_dispatch_line(0, "*CLS");
    g_digital = false;
}

/* The trace lives in the shared pool: once a JSON capture or upload fills the pool (it takes a
   generation), USER replay refuses and TRACe:POINts? says 0 instead of playing the other client's
   data. TRACe:DATA takes the heavy gate, and takes the pool back for a new trace. */
static void test_trace_in_the_shared_pool(void) {
    scpi_dispatch_line(0, "*CLS");
    reset_conn(4096);
    scpi_dispatch_line(0, "READ? 16");
    scpi_dispatch_line(0, "FUNC USER");
    g_generates = 0;
    scpi_dispatch_line(0, "OUTP ON");
    CHECK(g_generates == 1);
    scpi_dispatch_line(0, "OUTP OFF");

    (void)adc_pool_take();                     /* a JSON capture fills the pool */
    g_generates = 0;
    scpi_dispatch_line(0, "OUTP ON");
    CHECK(g_generates == 0);
    CHECK(next_error(NULL, 0) == -200);
    reset_conn(4096);
    scpi_dispatch_line(0, "TRAC:POIN?");
    K.out[K.out_len] = '\0';
    CHECK(atoi(K.out) == 0);

    /* TRACe:DATA while another client holds the gate: refused, nothing written */
    const uint8_t up[4] = { 0x11, 0x22, 0x33, 0x44 };
    char b64[16], line[64];
    b64url_encode(up, sizeof(up), b64, sizeof(b64));
    snprintf(line, sizeof(line), "TRAC:DATA 0,\"%s\"", b64);
    g_gate_busy = true;
    scpi_dispatch_line(0, line);
    g_gate_busy = false;
    CHECK(next_error(NULL, 0) == -200);
    reset_conn(4096);
    scpi_dispatch_line(0, "TRAC:POIN?");
    K.out[K.out_len] = '\0';
    CHECK(atoi(K.out) == 0);
    /* free gate: the upload takes the pool back and replays */
    scpi_dispatch_line(0, line);
    CHECK(next_error(NULL, 0) == 0);
    g_replay_len = 0;
    scpi_dispatch_line(0, "OUTP ON");
    CHECK(g_replay_len == 4 && memcmp(g_replay, up, 4) == 0);
    scpi_dispatch_line(0, "OUTP OFF");
    scpi_dispatch_line(0, "FUNC SIN");
}

/* DIAGnostic:CAPture? in the pool's scratch: the ADC samples, then the LA words cut to 12 bits, as
   one CSV list; it does not disturb the trace. */
static void test_dual_capture_csv(void) {
    reset_conn(4096);
    scpi_dispatch_line(0, "READ? 4");
    g_dual_ok = true;
    reset_conn(4096);
    scpi_dispatch_line(0, "DIAG:CAP? 3,2");
    K.out[K.out_len] = '\0';
    CHECK(strcmp(K.out, "1000,1001,1002,0,1\r\n") == 0);
    if (strcmp(K.out, "1000,1001,1002,0,1\r\n") != 0) printf("  got \"%s\"\n", K.out);
    reset_conn(4096);
    scpi_dispatch_line(0, "DIAG:CAP? 0,2");
    K.out[K.out_len] = '\0';
    CHECK(strcmp(K.out, "0,1\r\n") == 0);
    g_dual_ok = false;
    reset_conn(4096);
    scpi_dispatch_line(0, "TRAC:POIN?");
    K.out[K.out_len] = '\0';
    CHECK(atoi(K.out) == 4);
}

int main(void) {
    test_short_reply_unchanged();
    test_user_replays_the_capture();
    test_dac_limits_refuse_raw_output();
    test_safe_mode_and_digital_board();
    test_trace_in_the_shared_pool();
    test_dual_capture_csv();
    test_big_read_arrives_whole();
    test_slow_peer_feeds_watchdog();
    test_stalled_peer_aborts_once();
    test_dead_conn_aborts();
    if (failures) { printf("FAIL test_scpi_paced: %d failure(s)\n", failures); return 1; }
    printf("PASS test_scpi_paced\n");
    return 0;
}
