/* Host test for wifi_bridge.c, the pod's Wi-Fi bridge client, against a simulated bridge that
   runs the bridge firmware's own pairing state machine and codec
   (wifi-bridge/components/bridge_core): pairing on first contact, signed requests, retries,
   pod and bridge reboots, a bridge paired with another pod, the PAIR button, spoofed answers. */
#include "wifi_bridge.h"
#include "pairing.h"

#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); failures++; } } while (0)

static const uint8_t POD[6] = {0x02, 0x11, 0x22, 0x33, 0x44, 0x55};
static const uint8_t BRIDGE[6] = {0x24, 0x0a, 0xc4, 0x00, 0x00, 0x03};

/* ---- simulated bridge (a slice of wifi-bridge/main/control.c) */
typedef struct {
    bwp_pairing_t pair;
    uint8_t state, nonce_seed;
    char ssid[33];
    int drop_next;          /* swallow this many pod frames */
    int handled;            /* requests it acted on */
    uint8_t out[8][BWP_MAX_FRAME];
    size_t out_len[8];
    int n_out;
    uint8_t last_type;
    uint32_t last_counter_seen;
} sim_t;

static sim_t sim;
static uint32_t now_ms;

static void sim_boot(void)
{
    uint8_t nonce[BWP_NONCE_LEN];
    uint8_t key[BWP_KEY_LEN];
    bool had = sim.pair.paired;
    memcpy(key, sim.pair.key, sizeof(key));
    memset(nonce, ++sim.nonce_seed, sizeof(nonce));
    pairing_init(&sim.pair, nonce, had ? key : NULL);
}

static void sim_emit(size_t n)
{
    if (n && sim.n_out < 8) {
        sim.out_len[sim.n_out++] = n;
    }
}

static void sim_hello(void)
{
    bwp_hello_t h = {.fw_version = "0.1.0", .hw_rev = "A", .state = sim.state,
                     .flags = (uint8_t)(BWP_F_ETH_LINK | BWP_F_POD_KNOWN | (sim.pair.paired ? BWP_F_PAIRED : 0))};
    memcpy(h.bridge_mac, BRIDGE, 6);
    memcpy(h.pod_mac, POD, 6);
    memcpy(h.nonce, sim.pair.nonce, BWP_NONCE_LEN);
    sim_emit(bwp_build_hello(sim.out[sim.n_out], BWP_MAX_FRAME, POD, &h));
}

static void sim_rx(const uint8_t *f, size_t len)
{
    if (sim.drop_next > 0) {
        sim.drop_next--;
        return;
    }
    bwp_msg_t m;
    if (bwp_parse(f, len, &m) != BWP_PARSE_OK) {
        return;
    }
    bool authed;
    uint32_t counter;
    const uint8_t *nk;
    pair_verdict_t v = pairing_check(&sim.pair, f, &m, &authed, &counter, &nk);
    uint8_t *out = sim.out[sim.n_out];
    sim.last_type = m.type;
    if (v != PAIR_ACCEPT && v != PAIR_STORE_KEY) {
        bwp_writer_t w;
        bwp_begin(&w, out, BWP_MAX_FRAME, POD, BRIDGE, m.type | BWP_RESPONSE, m.seq);
        bwp_put_u8(&w, BWP_T_RESULT, pairing_result(v));
        bwp_put(&w, BWP_T_NONCE, sim.pair.nonce, BWP_NONCE_LEN);
        if (v == PAIR_REPLAY) {
            bwp_put_u32(&w, BWP_T_COUNTER, sim.pair.last_counter);
        }
        sim_emit(bwp_finish(&w));
        return;
    }
    sim.handled++;
    sim.last_counter_seen = counter;
    size_t n;
    if (m.type == BWP_PAIR) {
        pairing_set_key(&sim.pair, nk);
        n = bwp_build_result(out, BWP_MAX_FRAME, POD, BRIDGE, m.type, m.seq, BWP_OK, NULL);
    } else if (m.type == BWP_STATUS) {
        bwp_status_t s = {.state = sim.state, .flags = BWP_F_PAIRED, .channel = 6, .rssi = -58, .uptime_s = 77};
        strcpy(s.ssid, sim.ssid);
        s.counters.eth_rx = 1234;
        n = bwp_build_status(out, BWP_MAX_FRAME, POD, BRIDGE, m.seq, &s);
    } else if (m.type == BWP_SET_WIFI) {
        bwp_set_wifi_t sw;
        uint8_t r = bwp_parse_set_wifi(&m, &sw);
        if (r == BWP_OK) {
            strcpy(sim.ssid, sw.ssid);
            sim.state = BWP_ST_JOINING;
        }
        n = bwp_build_result(out, BWP_MAX_FRAME, POD, BRIDGE, m.type, m.seq, r, r ? "bad SSID" : NULL);
    } else if (m.type == BWP_SCAN) {
        bwp_ap_t aps[2] = {{.ssid = "lab", .rssi = -40, .channel = 1, .authmode = 3},
                           {.ssid = "guest", .rssi = -70, .channel = 11, .authmode = 0}};
        n = bwp_build_scan_result(out, BWP_MAX_FRAME, POD, BRIDGE, m.seq, BWP_OK, aps, 2);
    } else {
        n = bwp_build_result(out, BWP_MAX_FRAME, POD, BRIDGE, m.type, m.seq, BWP_E_UNSUPPORTED, NULL);
    }
    if (n && authed) {
        n = bwp_sign_frame(out, BWP_MAX_FRAME, sim.pair.key, sim.pair.nonce, counter);
    }
    sim_emit(n);
}

/* ---- pod side */
static wb_t pod;
static int sent;

static void io_send(const uint8_t *f, size_t len, void *ctx)
{
    (void)ctx;
    sent++;
    sim_rx(f, len);
}

static int io_key(const uint8_t mac[6], uint8_t key[BWP_KEY_LEN], void *ctx)
{
    uint8_t seed = *(uint8_t *)ctx;  /* stands in for the pod's device key */
    for (int i = 0; i < BWP_KEY_LEN; i++) {
        key[i] = (uint8_t)(mac[i % 6] ^ seed ^ i);
    }
    return 0;
}

static uint8_t pod_seed = 0x5a;
static const wb_io_t IO = {.send = io_send, .derive_key = io_key, .ctx = &pod_seed};

/* Deliver what the bridge sent, then let both sides run for `ms`. */
static void run(uint32_t ms)
{
    for (uint32_t t = 0; t <= ms; t += 10) {
        int n = sim.n_out;
        sim.n_out = 0;
        for (int i = 0; i < n; i++) {
            wb_rx(&pod, sim.out[i], sim.out_len[i], now_ms);
        }
        if (now_ms % 2000 == 0) {
            sim_hello();
        }
        wb_poll(&pod, now_ms);
        now_ms += 10;
    }
}

/* A caller's request, run to completion like wifi_bridge_net.c does for the worker. */
static wb_req_state_t call(uint8_t type, const uint8_t *tlvs, size_t len, uint32_t timeout_ms)
{
    for (int i = 0; i < 300; i++) {
        int r = wb_request(&pod, type, tlvs, len, timeout_ms, now_ms);
        if (r == 0) {
            break;
        }
        if (r != -1) {
            return WB_REQ_TIMEOUT;
        }
        run(10);
    }
    for (int i = 0; i < 1000 && wb_request_state(&pod) == WB_REQ_PENDING; i++) {
        run(10);
    }
    return wb_request_state(&pod);
}

static size_t set_wifi_tlvs(uint8_t *buf, const char *ssid, const char *pass)
{
    size_t n = 0;
    size_t a = strlen(ssid), b = strlen(pass);
    buf[n++] = BWP_T_SSID, buf[n++] = 0, buf[n++] = (uint8_t)a;
    memcpy(buf + n, ssid, a), n += a;
    buf[n++] = BWP_T_PASSPHRASE, buf[n++] = 0, buf[n++] = (uint8_t)b;
    memcpy(buf + n, pass, b), n += b;
    return n;
}

int main(void)
{
    wb_info_t info;
    uint8_t t[128];

    memset(&sim, 0, sizeof(sim));
    sim.state = BWP_ST_CONNECTED;  /* STATUS carries RSSI only when connected */
    strcpy(sim.ssid, "old net");
    sim_boot();
    wb_init(&pod, POD, &IO);
    wb_set_link(&pod, true, now_ms);

    /* No HELLO yet: no bridge. */
    CHECK(wb_request(&pod, BWP_STATUS, NULL, 0, 1000, now_ms) == -2);
    wb_get_info(&pod, now_ms, &info);
    CHECK(!info.present);

    /* First contact: HELLO (unpaired) -> the pod pairs on its own, then polls STATUS, signed. */
    run(3000);
    wb_get_info(&pod, now_ms, &info);
    CHECK(info.present && memcmp(info.mac, BRIDGE, 6) == 0);
    CHECK(sim.pair.paired);
    uint8_t expect[BWP_KEY_LEN];
    io_key(BRIDGE, expect, &pod_seed);
    CHECK(memcmp(sim.pair.key, expect, BWP_KEY_LEN) == 0);
    CHECK(info.paired && info.problem == NULL);
    CHECK(info.have_status && info.rssi == -58 && info.counters.eth_rx == 1234);
    CHECK(strcmp(info.fw_version, "0.1.0") == 0);

    /* A caller's SET_WIFI: signed, answered, and the signed answer is accepted. */
    size_t n = set_wifi_tlvs(t, "lab net", "correct horse");
    CHECK(call(BWP_SET_WIFI, t, n, 1000) == WB_REQ_DONE);
    CHECK(wb_request_reply(&pod)->result == BWP_OK);
    CHECK(strcmp(sim.ssid, "lab net") == 0);
    wb_request_release(&pod);

    /* A bad request comes back as the bridge's error, still signed. */
    n = set_wifi_tlvs(t, "", "x");
    CHECK(call(BWP_SET_WIFI, t, n, 1000) == WB_REQ_DONE);
    CHECK(wb_request_reply(&pod)->result == BWP_E_BAD_REQUEST);
    CHECK(strcmp(wb_request_reply(&pod)->message, "bad SSID") == 0);
    wb_request_release(&pod);

    /* SCAN, with the reply parsed. */
    CHECK(call(BWP_SCAN, NULL, 0, 1000) == WB_REQ_DONE);
    CHECK(wb_request_reply(&pod)->n_aps == 2);
    CHECK(strcmp(wb_request_reply(&pod)->aps[1].ssid, "guest") == 0 && wb_request_reply(&pod)->aps[1].rssi == -70);
    wb_request_release(&pod);

    /* A lost request: the retry is re-signed with a new counter and accepted. */
    int before = sim.handled;
    sim.drop_next = 1;
    CHECK(call(BWP_STATUS, NULL, 0, 200) == WB_REQ_DONE && wb_request_reply(&pod)->result == BWP_OK);
    CHECK(sim.handled == before + 1);
    wb_request_release(&pod);

    /* All three sends lost: timeout. */
    sim.drop_next = 3;
    CHECK(call(BWP_STATUS, NULL, 0, 100) == WB_REQ_TIMEOUT);
    CHECK(strstr(wb_request_reply(&pod)->message, "no answer") != NULL);
    wb_request_release(&pod);

    /* The pod reboots: same derived key, counter back at 0. The bridge says REPLAY with its
       last counter; the pod continues above it without bothering the caller. */
    uint32_t bridge_last = sim.pair.last_counter;
    CHECK(bridge_last > 3);
    wb_init(&pod, POD, &IO);
    wb_set_link(&pod, true, now_ms);
    run(2100);  /* HELLO, then the background STATUS resyncs */
    wb_get_info(&pod, now_ms, &info);
    CHECK(info.paired && info.problem == NULL && info.have_status);
    CHECK(sim.pair.last_counter > bridge_last);

    /* The bridge reboots: new nonce in HELLO, the counter starts over, requests still work. */
    sim_boot();
    run(2100);
    CHECK(call(BWP_STATUS, NULL, 0, 1000) == WB_REQ_DONE && wb_request_reply(&pod)->result == BWP_OK);
    CHECK(sim.last_counter_seen < 5);
    wb_request_release(&pod);

    /* A spoofed answer (right seq, unsigned) to a signed request is ignored. */
    sim.drop_next = 1;
    CHECK(wb_request(&pod, BWP_CLEAR_WIFI, NULL, 0, 300, now_ms) == 0);
    uint8_t fake[BWP_MAX_FRAME];
    size_t fn = bwp_build_result(fake, sizeof(fake), POD, BRIDGE, BWP_CLEAR_WIFI, pod.req_seq, BWP_OK, NULL);
    wb_rx(&pod, fake, fn, now_ms);
    CHECK(wb_request_state(&pod) == WB_REQ_PENDING);
    for (int i = 0; i < 200 && wb_request_state(&pod) == WB_REQ_PENDING; i++) {
        run(10);
    }
    /* ...and the real one (after the retry) is the bridge's: CLEAR_WIFI is unsupported in this sim. */
    CHECK(wb_request_state(&pod) == WB_REQ_DONE && wb_request_reply(&pod)->result == BWP_E_UNSUPPORTED);
    wb_request_release(&pod);

    /* A frame for another pod, or another EtherType, changes nothing. */
    fn = bwp_build_result(fake, sizeof(fake), BRIDGE, BRIDGE, BWP_STATUS, 1, BWP_OK, NULL);
    wb_rx(&pod, fake, fn, now_ms);
    fake[12] = 0x08;
    wb_rx(&pod, fake, fn, now_ms);

    /* PAIR pressed on the bridge: HELLO without the paired flag -> the pod pairs again. */
    pairing_forget(&sim.pair);
    run(2100);
    run(6000);
    CHECK(sim.pair.paired && memcmp(sim.pair.key, expect, BWP_KEY_LEN) == 0);
    CHECK(call(BWP_STATUS, NULL, 0, 1000) == WB_REQ_DONE && wb_request_reply(&pod)->result == BWP_OK);
    wb_request_release(&pod);

    /* The bridge is paired with another pod: our requests are refused, the user is told why. */
    uint8_t other[BWP_KEY_LEN];
    memset(other, 0xee, sizeof(other));
    pairing_set_key(&sim.pair, other);
    CHECK(call(BWP_STATUS, NULL, 0, 1000) == WB_REQ_DONE);
    CHECK(wb_request_reply(&pod)->result == BWP_E_AUTH);
    CHECK(strstr(wb_request_reply(&pod)->message, "press PAIR") != NULL);
    wb_request_release(&pod);
    wb_get_info(&pod, now_ms, &info);
    CHECK(!info.paired && info.problem != NULL);
    /* ...until PAIR is pressed there. */
    pairing_forget(&sim.pair);
    run(8000);
    wb_get_info(&pod, now_ms, &info);
    CHECK(info.paired && info.problem == NULL && sim.pair.paired);

    /* The bridge goes away (no HELLO): requests fail fast, the info says so. */
    sim.drop_next = 1000;
    uint32_t until = now_ms + 7000;
    while (now_ms < until) {
        wb_poll(&pod, now_ms);  /* no HELLOs delivered */
        now_ms += 10;
    }
    wb_get_info(&pod, now_ms, &info);
    CHECK(!info.present);
    CHECK(wb_request(&pod, BWP_STATUS, NULL, 0, 1000, now_ms) == -2);

    /* Ethernet link down: no bridge either. */
    sim.drop_next = 0;
    run(2100);
    wb_get_info(&pod, now_ms, &info);
    CHECK(info.present);
    wb_set_link(&pod, false, now_ms);
    wb_get_info(&pod, now_ms, &info);
    CHECK(!info.present);

    CHECK(strcmp(wb_state_str(BWP_ST_CONNECTED), "connected") == 0);
    CHECK(strcmp(wb_result_str(BWP_E_ALREADY_PAIRED), "already-paired") == 0);
    printf("test_wifi_bridge: %s (%d failures, %d frames sent)\n", failures ? "FAIL" : "ok", failures, sent);
    return failures ? 1 : 0;
}
