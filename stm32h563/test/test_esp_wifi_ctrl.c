/*
 * test_esp_wifi_ctrl.c — host tests for the Wi-Fi control state machine
 * (src/esp_wifi_ctrl.c) against a fake esp-hosted transport + fake ESP32-C3.
 *
 * Pins down the "dead C3 while connected" recovery:
 *   - a healthy link stays up: answered GetRssi never restarts anything;
 *   - RSSI_MAX_MISSES (3) GetRssi in a row that time out take the link down,
 *     hold the C3 in reset and restart it after the backoff (counted);
 *   - the same when the requests cannot even be queued (TX ring full, which is
 *     what a stuck HANDSHAKE looks like);
 *   - two misses and then an answer do NOT restart (the count is consecutive);
 *   - a boot event from the C3 while connected (or mid-connect) is a slave
 *     reset: link down + restart, counted separately;
 *   - a boot event while waiting for the boot event is just the boot event.
 *
 * The module under test is compiled unmodified; everything it calls below it
 * (esp_hosted_spi_*, esp_netif_*, config_load, hw_worker, time) is faked here.
 */
#include "esp_wifi_ctrl.h"
#include "esp_hosted_spi.h"
#include "esp_netif.h"
#include "config_store.h"
#include "esp_rom_flash.h"
#include "hw_worker.h"
#include "pico/time.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

static int failures;
#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); failures++; } \
} while (0)

/* ---- fake time ------------------------------------------------------------ */
static uint64_t s_now_us = 1000000;
uint64_t time_us_64(void) { return s_now_us; }
uint32_t time_us_32(void) { return (uint32_t)s_now_us; }

/* ---- fake transport + C3 --------------------------------------------------- */
static struct {
    bool running, ready, reset_latch;
    bool tx_ok;          /* esp_hosted_spi_send accepts frames */
    bool answer;         /* the fake C3 answers requests */
    int  starts, stops;
    uint32_t pending[16]; int npending;   /* request msg_ids awaiting an answer */
    uint32_t last_req;
    int  dropped;        /* requests the fake C3 ignored (answer == false) */
} C;
static esp_hosted_frame_cb_t s_serial_cb;

static struct { bool up; int downs, ups; } L;

void esp_hosted_spi_start(void) {
    if (C.running) return;
    C.running = true; C.ready = false; C.reset_latch = false; C.npending = 0; C.starts++;
}
void esp_hosted_spi_stop(void) {
    C.running = false; C.ready = false; C.reset_latch = false; C.npending = 0; C.stops++;
}
bool esp_hosted_spi_ready(void) { return C.ready; }
bool esp_hosted_spi_take_slave_reset(void) { bool r = C.reset_latch; C.reset_latch = false; return r; }
void esp_hosted_spi_set_serial_cb(esp_hosted_frame_cb_t cb) { s_serial_cb = cb; }

/* Pull msg_id (Rpc field 2) out of a TLV-wrapped request. */
static uint32_t req_msg_id(const uint8_t *p, uint16_t len) {
    size_t eplen = (size_t)p[1] | ((size_t)p[2] << 8);
    const uint8_t *pb = p + 3 + eplen + 3, *end = p + len;
    while (pb < end) {
        uint64_t key = 0, v = 0; int s = 0;
        do { key |= (uint64_t)(*pb & 0x7F) << s; s += 7; } while (*pb++ & 0x80);
        uint32_t field = (uint32_t)(key >> 3), wire = (uint32_t)(key & 7);
        if (wire == 0) {
            s = 0; do { v |= (uint64_t)(*pb & 0x7F) << s; s += 7; } while (*pb++ & 0x80);
            if (field == 2) return (uint32_t)v;
        } else if (wire == 2) {
            s = 0; do { v |= (uint64_t)(*pb & 0x7F) << s; s += 7; } while (*pb++ & 0x80);
            pb += v;
        } else {
            return 0;
        }
    }
    return 0;
}

int esp_hosted_spi_send(uint8_t if_type, uint8_t if_num, const uint8_t *payload, uint16_t len) {
    (void)if_num;
    if (!C.running || !C.tx_ok || if_type != ESP_SERIAL_IF) return -1;
    C.last_req = req_msg_id(payload, len);
    if (C.npending < 16) C.pending[C.npending++] = C.last_req;
    return 0;
}

/* ---- protobuf helpers for the fake C3's replies ---------------------------- */
typedef struct { uint8_t b[128]; size_t n; } buf_t;
static void w_varint(buf_t *w, uint64_t v) { do { uint8_t x = v & 0x7F; v >>= 7; if (v) x |= 0x80; w->b[w->n++] = x; } while (v); }
static void w_int(buf_t *w, uint32_t f, uint64_t v) { w_varint(w, ((uint64_t)f << 3) | 0); w_varint(w, v); }
static void w_bytes(buf_t *w, uint32_t f, const uint8_t *d, size_t n) {
    w_varint(w, ((uint64_t)f << 3) | 2); w_varint(w, n); memcpy(w->b + w->n, d, n); w->n += n;
}

static void deliver(uint32_t type, uint32_t msg_id, const buf_t *inner) {
    buf_t rpc = {0};
    w_int(&rpc, 1, type);
    w_int(&rpc, 2, msg_id);
    w_bytes(&rpc, msg_id, inner->b, inner->n);
    esp_hosted_rx_t rx;
    memset(&rx, 0, sizeof(rx));
    rx.if_type = ESP_SERIAL_IF;
    rx.payload = rpc.b;            /* bare protobuf: tlv_unwrap passes it through */
    rx.payload_len = (uint16_t)rpc.n;
    s_serial_cb(&rx);
}

static void answer(uint32_t req) {
    buf_t in = {0};
    if (req == 257) {                               /* GetMac */
        static const uint8_t mac[6] = { 0x02, 0x11, 0x22, 0x33, 0x44, 0x55 };
        w_bytes(&in, 1, mac, 6);
        w_int(&in, 2, 0);
    } else if (req == 341) {                        /* GetRssi */
        w_int(&in, 1, 0);
        w_int(&in, 2, (uint64_t)(int64_t)-52);
    } else {
        w_int(&in, 1, 0);
    }
    deliver(2, req + 256, &in);
}

static void evt_connected(void) { buf_t in = {0}; deliver(3, 775, &in); }

/* One net-loop pass: the transport runs first (answers arrive), then control. */
static void pass(void) {
    if (C.answer && C.running) {
        for (int i = 0; i < C.npending; i++) answer(C.pending[i]);
    } else {
        C.dropped += C.npending;             /* a dead C3 never answers them later */
    }
    C.npending = 0;
    esp_wifi_ctrl_poll();
}
static void run_ms(uint32_t ms) {
    for (uint32_t t = 0; t < ms; t += 10) { s_now_us += 10000; pass(); }
}

/* ---- faked neighbours ------------------------------------------------------- */
void esp_netif_set_link_up(bool up) { L.up = up; if (up) L.ups++; else L.downs++; }
void esp_netif_set_hwaddr(const uint8_t mac[6]) { (void)mac; }
int config_load(config_t *out) {
    memset(out, 0, sizeof(*out));
    strcpy(out->ssid, "bench-lab");
    strcpy(out->password, "not-a-real-password");
    return 0;
}
static int s_flash_submits;
bool hw_worker_submit_esp_flash(void) { s_flash_submits++; return true; }
const uint32_t esp_slave_fw_len = 0;   /* no embedded image: never auto-flash here */

/* ---- scenarios -------------------------------------------------------------- */

static bool connected(void) { return esp_wifi_ctrl_connected(); }

/* Bring the link from reset to associated.  Returns true if it got there. */
static bool bring_up(void) {
    C.tx_ok = true; C.answer = true;
    run_ms(20);                              /* start() -> waiting for the boot event */
    C.ready = true;                          /* the C3 announces itself */
    for (int i = 0; i < 50 && strcmp(esp_wifi_ctrl_state_str(), "connecting") != 0; i++) run_ms(10);
    for (int i = 0; i < 50 && C.last_req != 257; i++) run_ms(10);   /* up to GetMac */
    run_ms(20);                              /* GetMac answered -> waiting for association */
    evt_connected();
    run_ms(20);
    return connected() && L.up;
}

static void fresh(void) {
    memset(&C, 0, sizeof(C));
    esp_wifi_ctrl_reload();                  /* re-inits from config_load, link down */
    memset(&L, 0, sizeof(L));
}

static void test_healthy_link_stays_up(void) {
    fresh();
    CHECK(bring_up());
    uint32_t nr0, rb0; esp_wifi_ctrl_slave_lost_counts(&nr0, &rb0);
    int starts = C.starts;
    run_ms(120000);                          /* two minutes of answered RSSI polls */
    uint32_t nr, rb; esp_wifi_ctrl_slave_lost_counts(&nr, &rb);
    CHECK(connected());
    CHECK(L.up);
    CHECK(L.downs == 0);
    CHECK(C.starts == starts);
    CHECK(nr == nr0 && rb == rb0);
    int dbm = 0;
    CHECK(esp_wifi_ctrl_rssi(&dbm) && dbm == -52);
}

/* Returns the time (ms) the link stayed up after the C3 went quiet. */
static uint32_t wait_for_link_down(uint32_t limit_ms) {
    uint32_t t = 0;
    while (L.up && t < limit_ms) { run_ms(10); t += 10; }
    return t;
}

static void test_rssi_timeouts_restart_c3(void) {
    fresh();
    CHECK(bring_up());
    uint32_t nr0, rb0; esp_wifi_ctrl_slave_lost_counts(&nr0, &rb0);
    int stops = C.stops, starts = C.starts;

    C.answer = false;                        /* C3 crashed: requests go unanswered */
    uint32_t t = wait_for_link_down(60000);
    CHECK(!L.up);
    CHECK(t > 10000);                        /* not on the first miss... */
    CHECK(t < 40000);                        /* ...but within 3 x (period + timeout) */
    CHECK(!connected());
    CHECK(strcmp(esp_wifi_ctrl_state_str(), "backoff") == 0);
    CHECK(C.stops == stops + 1);             /* held in reset right away */
    CHECK(!C.running);
    uint32_t nr, rb; esp_wifi_ctrl_slave_lost_counts(&nr, &rb);
    CHECK(nr == nr0 + 1);
    CHECK(rb == rb0);

    /* After the backoff the C3 is released from reset again and can re-associate. */
    run_ms(6000);
    CHECK(C.starts == starts + 1);
    CHECK(C.running);
    CHECK(bring_up());
}

static void test_tx_full_counts_as_miss(void) {
    fresh();
    CHECK(bring_up());
    uint32_t nr0; esp_wifi_ctrl_slave_lost_counts(&nr0, NULL);
    C.tx_ok = false;                         /* stuck HANDSHAKE: the TX ring never drains */
    C.answer = false;
    uint32_t t = wait_for_link_down(60000);
    CHECK(!L.up);
    CHECK(t < 30000);
    uint32_t nr; esp_wifi_ctrl_slave_lost_counts(&nr, NULL);
    CHECK(nr == nr0 + 1);
    CHECK(!C.running);
}

static void test_two_misses_then_answer_does_not_restart(void) {
    fresh();
    CHECK(bring_up());
    uint32_t nr0; esp_wifi_ctrl_slave_lost_counts(&nr0, NULL);
    int downs = L.downs;
    for (int round = 0; round < 3; round++) {
        C.answer = false;
        C.dropped = 0;
        for (int i = 0; i < 3000 && C.dropped < 2; i++) run_ms(10);   /* 2 requests ignored */
        C.answer = true;
        run_ms(15000);                       /* both time out, then one is answered */
    }
    uint32_t nr; esp_wifi_ctrl_slave_lost_counts(&nr, NULL);
    CHECK(nr == nr0);
    CHECK(L.up);
    CHECK(L.downs == downs);
    CHECK(connected());
}

static void test_boot_event_while_up_restarts(void) {
    fresh();
    CHECK(bring_up());
    uint32_t nr0, rb0; esp_wifi_ctrl_slave_lost_counts(&nr0, &rb0);
    int stops = C.stops;
    C.reset_latch = true;                    /* the C3 announced a second boot */
    run_ms(10);
    CHECK(!L.up);                            /* immediately, not after RSSI misses */
    CHECK(C.stops == stops + 1);
    CHECK(strcmp(esp_wifi_ctrl_state_str(), "backoff") == 0);
    uint32_t nr, rb; esp_wifi_ctrl_slave_lost_counts(&nr, &rb);
    CHECK(rb == rb0 + 1);
    CHECK(nr == nr0);
    run_ms(6000);
    CHECK(C.running);
    CHECK(bring_up());
}

static void test_boot_event_mid_connect_restarts(void) {
    fresh();
    C.tx_ok = true; C.answer = false;        /* stall the sequence at the first step */
    run_ms(20);
    C.ready = true;
    run_ms(20);
    CHECK(strcmp(esp_wifi_ctrl_state_str(), "connecting") == 0);
    uint32_t rb0; esp_wifi_ctrl_slave_lost_counts(NULL, &rb0);
    C.reset_latch = true;
    run_ms(10);
    uint32_t rb; esp_wifi_ctrl_slave_lost_counts(NULL, &rb);
    CHECK(rb == rb0 + 1);
    CHECK(strcmp(esp_wifi_ctrl_state_str(), "backoff") == 0);
    CHECK(!C.running);
}

static void test_boot_event_while_waiting_is_ignored(void) {
    fresh();
    C.tx_ok = true; C.answer = true;
    run_ms(20);
    CHECK(strcmp(esp_wifi_ctrl_state_str(), "waiting-slave") == 0);
    uint32_t rb0; esp_wifi_ctrl_slave_lost_counts(NULL, &rb0);
    C.reset_latch = true;                    /* two boot announces before we looked */
    C.ready = true;
    run_ms(10);
    uint32_t rb; esp_wifi_ctrl_slave_lost_counts(NULL, &rb);
    CHECK(rb == rb0);
    CHECK(strcmp(esp_wifi_ctrl_state_str(), "connecting") == 0);
}

int main(void) {
    esp_wifi_ctrl_init();
    test_healthy_link_stays_up();
    test_rssi_timeouts_restart_c3();
    test_tx_full_counts_as_miss();
    test_two_misses_then_answer_does_not_restart();
    test_boot_event_while_up_restarts();
    test_boot_event_mid_connect_restarts();
    test_boot_event_while_waiting_is_ignored();
    CHECK(s_flash_submits == 0);
    if (failures) { printf("FAIL test_esp_wifi_ctrl: %d failure(s)\n", failures); return 1; }
    printf("PASS test_esp_wifi_ctrl\n");
    return 0;
}
