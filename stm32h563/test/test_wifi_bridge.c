/* Host test for wifi_bridge.c, the pod's Wi-Fi bridge client, against a simulated bridge that
   runs the bridge firmware's own pairing state machine and codec
   (wifi-bridge/components/bridge_core): pairing on first contact, signed requests, retries,
   pod and bridge reboots, a bridge paired with another pod, the PAIR button, spoofed answers. */
#include "wifi_bridge.h"
#include "pairing.h"
#include "bwp_crypto.h"
#include "wifi_bridge_ota.h"

#include <stdlib.h>

#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); failures++; } } while (0)

static const uint8_t POD[6] = {0x02, 0x11, 0x22, 0x33, 0x44, 0x55};
static const uint8_t BRIDGE[6] = {0x24, 0x0a, 0xc4, 0x00, 0x00, 0x03};

/* ---- simulated bridge (a slice of wifi-bridge/main/control.c) */
typedef struct {
    bwp_pairing_t pair;
    uint8_t state, nonce_seed, key_seed;
    uint8_t pub[32];        /* our X25519 public key from the pairing (sent in HELLO) */
    char ssid[33];
    char pass[65];
    int drop_next;          /* swallow this many pod frames */
    int handled;            /* requests it acted on */
    uint8_t out[8][BWP_MAX_FRAME];
    size_t out_len[8];
    int n_out;
    uint8_t last_type;
    uint32_t last_counter_seen;
    /* OTA, as wifi-bridge/main/ota.c does it */
    char fw[16];
    uint8_t img[8192];
    uint32_t ota_size, ota_next, ota_last_off, ota_last_len, ota_version;
    bool ota_active, ota_done, refuse_begin, rollback;
    int data_frames;
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
    bwp_hello_t h = {.fw_version = sim.fw, .hw_rev = "A", .state = sim.state,
                     .flags = (uint8_t)(BWP_F_ETH_LINK | BWP_F_POD_KNOWN | (sim.pair.paired ? BWP_F_PAIRED : 0))};
    memcpy(h.bridge_mac, BRIDGE, 6);
    memcpy(h.pod_mac, POD, 6);
    memcpy(h.nonce, sim.pair.nonce, BWP_NONCE_LEN);
    memcpy(h.bridge_pub, sim.pub, 32);
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
        /* As the bridge firmware does: a fresh key pair, the key from X25519, answer, then switch. */
        uint8_t secret[32], pub[32], key[32];
        memset(secret, 0x80 + ++sim.key_seed, sizeof(secret));
        bwp_x25519_public(secret, pub);
        CHECK(bwp_pair_key(secret, nk, nk, pub, key) == 0);
        bwp_writer_t w;
        bwp_begin(&w, out, BWP_MAX_FRAME, POD, BRIDGE, m.type | BWP_RESPONSE, m.seq);
        bwp_put_u8(&w, BWP_T_RESULT, BWP_OK);
        bwp_put(&w, BWP_T_PUBKEY, pub, 32);
        n = bwp_finish(&w);
        if (authed) {
            n = bwp_sign_frame(out, BWP_MAX_FRAME, sim.pair.key, sim.pair.nonce, counter);
        }
        sim_emit(n);
        pairing_set_key(&sim.pair, key);
        memcpy(sim.pub, pub, 32);
        return;
    } else if (m.type == BWP_STATUS) {
        bwp_status_t s = {.state = sim.state, .flags = BWP_F_PAIRED, .channel = 6, .rssi = -58, .uptime_s = 77};
        strcpy(s.ssid, sim.ssid);
        s.counters.eth_rx = 1234;
        n = bwp_build_status(out, BWP_MAX_FRAME, POD, BRIDGE, m.seq, &s);
    } else if (m.type == BWP_SET_WIFI) {
        bwp_set_wifi_t sw;
        uint8_t r = bwp_parse_set_wifi(&m, sim.pair.key, &sw);
        if (r == BWP_OK) {
            strcpy(sim.ssid, sw.ssid);
            strcpy(sim.pass, sw.passphrase);
            sim.state = BWP_ST_JOINING;
        }
        n = bwp_build_result(out, BWP_MAX_FRAME, POD, BRIDGE, m.type, m.seq, r, r ? "bad SSID" : NULL);
    } else if (m.type == BWP_OTA_BEGIN) {
        uint32_t size = 0;
        const uint8_t *sg;
        uint16_t sl = 0;
        uint8_t r = BWP_OK;
        if (!bwp_get_u32(m.body, m.body_len, BWP_T_IMAGE_SIZE, &size) ||
            !bwp_find(m.body, m.body_len, BWP_T_SIGNATURE, &sg, &sl) || sl != 128 || size > sizeof(sim.img)) {
            r = BWP_E_BAD_REQUEST;
        } else if (sim.refuse_begin) {
            r = BWP_E_OTA_VERIFY;
        } else {
            sim.ota_size = size, sim.ota_next = 0, sim.ota_last_len = 0, sim.ota_active = true, sim.ota_done = false;
            sim.ota_version = (uint32_t)sg[44] | (uint32_t)sg[45] << 8 | (uint32_t)sg[46] << 16 | (uint32_t)sg[47] << 24;
        }
        n = bwp_build_result(out, BWP_MAX_FRAME, POD, BRIDGE, m.type, m.seq, r,
                             r == BWP_E_OTA_VERIFY ? "signature: unknown-key" : NULL);
    } else if (m.type == BWP_OTA_DATA) {
        uint32_t off = 0;
        const uint8_t *d;
        uint16_t dl = 0;
        uint8_t r = BWP_OK;
        sim.data_frames++;
        if (!bwp_get_u32(m.body, m.body_len, BWP_T_OFFSET, &off) || !bwp_find(m.body, m.body_len, BWP_T_DATA, &d, &dl)) {
            r = BWP_E_BAD_REQUEST;
        } else if (!sim.ota_active) {
            r = BWP_E_OTA_NOT_STARTED;
        } else if (sim.ota_last_len && off == sim.ota_last_off && dl == sim.ota_last_len) {
            /* a retransmission: ack again */
        } else if (off != sim.ota_next || off + dl > sim.ota_size) {
            r = BWP_E_OTA_BAD_OFFSET;
        } else {
            memcpy(sim.img + off, d, dl);
            sim.ota_last_off = off, sim.ota_last_len = dl, sim.ota_next += dl;
        }
        bwp_writer_t w;
        bwp_begin(&w, out, BWP_MAX_FRAME, POD, BRIDGE, m.type | BWP_RESPONSE, m.seq);
        bwp_put_u8(&w, BWP_T_RESULT, r);
        bwp_put_u32(&w, BWP_T_OFFSET, sim.ota_next);
        n = bwp_finish(&w);
    } else if (m.type == BWP_OTA_END) {
        extern const uint8_t g_image[];
        bool good = sim.ota_active && sim.ota_next == sim.ota_size && memcmp(sim.img, g_image, sim.ota_size) == 0;
        sim.ota_active = false;
        sim.ota_done = good;
        n = bwp_build_result(out, BWP_MAX_FRAME, POD, BRIDGE, m.type, m.seq, good ? BWP_OK : BWP_E_OTA_VERIFY, NULL);
    } else if (m.type == BWP_REBOOT) {
        n = bwp_build_result(out, BWP_MAX_FRAME, POD, BRIDGE, m.type, m.seq, BWP_OK, NULL);
        if (authed) {
            n = bwp_sign_frame(out, BWP_MAX_FRAME, sim.pair.key, sim.pair.nonce, counter);
        }
        sim_emit(n);
        if (sim.ota_done && !sim.rollback) {
            wbo_version_str(sim.ota_version, sim.fw);
        }
        sim.ota_done = false;
        sim_boot();
        return;
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
static bool leaked;   /* the passphrase seen in clear in any frame the pod sent */

static bool contains(const uint8_t *hay, size_t n, const char *needle)
{
    size_t k = strlen(needle);
    for (size_t i = 0; i + k <= n; i++) {
        if (memcmp(hay + i, needle, k) == 0) {
            return true;
        }
    }
    return false;
}

static void io_send(const uint8_t *head, size_t hl, const uint8_t *body, size_t bl, const uint8_t *tail, size_t tl,
                    void *ctx)
{
    (void)ctx;
    static uint8_t f[BWP_MAX_FRAME];
    size_t len = hl + bl + tl;
    memcpy(f, head, hl);
    if (bl) memcpy(f + hl, body, bl);
    if (tl) memcpy(f + hl + bl, tail, tl);
    if (len < BWP_MIN_FRAME) {
        memset(f + len, 0, BWP_MIN_FRAME - len);
        len = BWP_MIN_FRAME;
    }
    sent++;
    if (contains(f, len, "correct horse")) {
        leaked = true;
    }
    sim_rx(f, len);
}

static int io_secret(const uint8_t mac[6], uint8_t secret[32], void *ctx)
{
    uint8_t seed = *(uint8_t *)ctx;  /* stands in for the pod's device key */
    for (int i = 0; i < 32; i++) {
        secret[i] = (uint8_t)(mac[i % 6] ^ seed ^ i);
    }
    return 0;
}

static uint8_t pod_seed = 0x5a;
static const wb_io_t IO = {.send = io_send, .derive_secret = io_secret, .ctx = &pod_seed};

/* The key the pod should have: from its secret for this bridge and the bridge's public key. */
static void expected_key(uint8_t key[32])
{
    uint8_t secret[32], pub[32];
    io_secret(BRIDGE, secret, &pod_seed);
    bwp_x25519_public(secret, pub);
    CHECK(bwp_pair_key(secret, sim.pub, pub, sim.pub, key) == 0);
}

/* Deliver what the bridge sent, then let both sides run for `ms`. */
static void (*tick_hook)(void);   /* the relay, while its tests run */

static void run(uint32_t ms)
{
    for (uint32_t t = 0; t <= ms; t += 10) {
        if (tick_hook) {
            tick_hook();
        }
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

static wb_req_state_t call_set_wifi(const char *ssid, const char *pass)
{
    static const uint8_t nonce[BWP_SEAL_NONCE_LEN] = {7};
    int r = wb_request_set_wifi(&pod, ssid, pass, nonce, 1000, now_ms);
    if (r != 0) {
        return WB_REQ_TIMEOUT;
    }
    for (int i = 0; i < 1000 && wb_request_state(&pod) == WB_REQ_PENDING; i++) {
        run(10);
    }
    return wb_request_state(&pod);
}

/* ---- the firmware relay (wifi_bridge_ota.c) against the simulated bridge */
const uint8_t g_image[5000] = {1};   /* filled in test_relay */
static wbo_t relay;
static wbo_image_t slot;
static bool bus_free = true;

static int ro_start(void *ctx, uint8_t type, const uint8_t *tlvs, size_t len, uint32_t timeout_ms)
{
    (void)ctx;
    int r = wb_request_ext(&pod, type, tlvs, len, timeout_ms, now_ms);
    return r == 0 ? 0 : r == -1 ? -1 : -2;
}

static wb_req_state_t ro_take(void *ctx, wb_reply_t *out)
{
    (void)ctx;
    wb_req_state_t st = wb_request_state(&pod);
    if (st == WB_REQ_DONE || st == WB_REQ_TIMEOUT) {
        *out = *wb_request_reply(&pod);
        wb_request_release(&pod);
    }
    return st;
}

static int ro_read(void *ctx, uint32_t off, uint8_t *buf, uint32_t n)
{
    (void)ctx;
    if (off + n > sizeof(g_image)) {
        return -1;
    }
    memcpy(buf, g_image + off, n);
    return 0;
}

static void relay_tick(void)
{
    wb_info_t i;
    wb_get_info(&pod, now_ms, &i);
    wbo_poll(&relay, &i, &slot, bus_free, now_ms);
}

static void test_relay(void)
{
    static const wbo_io_t io = {.start = ro_start, .take = ro_take, .read = ro_read, .alloc = malloc, .release = free};
    for (size_t i = 0; i < sizeof(g_image); i++) {
        ((uint8_t *)g_image)[i] = (uint8_t)(i * 7 + 3);
    }
    memset(&slot, 0, sizeof(slot));
    slot.present = true;
    slot.size = sizeof(g_image);
    slot.version = wbo_pack_version("0.2.0");
    memcpy(slot.manifest, "BPSG", 4);
    slot.manifest[44] = (uint8_t)slot.version, slot.manifest[45] = (uint8_t)(slot.version >> 8);
    slot.manifest[46] = (uint8_t)(slot.version >> 16);

    sim.drop_next = 0;
    bus_free = false;   /* the bus is busy: the update waits */
    wb_set_link(&pod, true, now_ms);
    static wb_reply_t reply;
    wbo_init(&relay, &io, &reply);
    tick_hook = relay_tick;
    run(3000);
    wb_info_t info;
    wb_get_info(&pod, now_ms, &info);
    CHECK(info.present && info.paired);

    CHECK(wbo_pack_version("0.1.0") == 0x000100 && wbo_pack_version("1.2.3") == 0x010203);
    CHECK(wbo_pack_version("1.2") == 0 && wbo_pack_version("1.2.3x") == 0 && wbo_pack_version(NULL) == 0);

    run(6000);
    CHECK(relay.state == WBO_IDLE && strstr(wbo_status(&relay), "bus is busy") != NULL);
    bus_free = true;

    /* Newer image: updated on its own, one chunk lost on the way, then back with 0.2.0. */
    sim.drop_next = 0;
    int dropped_at = 0;
    for (int t = 0; t < 2000 && relay.state == WBO_IDLE; t++) {
        run(10);
    }
    for (int t = 0; t < 4000 && relay.state != WBO_IDLE; t++) {
        if (relay.state == WBO_DATA && !dropped_at && relay.off >= 2048) {
            sim.drop_next = 1;  /* lose one chunk: the client resends it */
            dropped_at = 1;
        }
        run(10);
    }
    CHECK(dropped_at);
    CHECK(strcmp(sim.fw, "0.2.0") == 0);
    CHECK(memcmp(sim.img, g_image, sizeof(g_image)) == 0);
    CHECK(strcmp(wbo_status(&relay), "updated to 0.2.0") == 0);
    CHECK(sim.data_frames >= 5);  /* 5000 bytes in 1 KB chunks, plus the resend */

    /* Same version now: nothing happens. */
    int frames = sim.data_frames;
    run(12000);
    CHECK(relay.state == WBO_IDLE && sim.data_frames == frames);

    /* A newer one that the bridge rolls back: failed, and not retried by itself... */
    slot.version = wbo_pack_version("0.3.0");
    slot.manifest[44] = (uint8_t)slot.version, slot.manifest[45] = (uint8_t)(slot.version >> 8);
    sim.rollback = true;
    run(30000);
    CHECK(relay.state == WBO_WAIT_BACK);
    run(WBO_COME_BACK_MS + 6000);
    CHECK(relay.state == WBO_IDLE && strcmp(sim.fw, "0.2.0") == 0);
    CHECK(strstr(wbo_status(&relay), "update to 0.3.0 failed: the bridge came back with its old firmware") != NULL);
    frames = sim.data_frames;
    run(12000);
    CHECK(sim.data_frames == frames);
    /* ...until forced: then it works. */
    sim.rollback = false;
    wbo_force(&relay);
    for (int t = 0; t < 8000 && strcmp(wbo_status(&relay), "updated to 0.3.0") != 0; t++) {
        run(10);
    }
    CHECK(strcmp(sim.fw, "0.3.0") == 0 && strcmp(wbo_status(&relay), "updated to 0.3.0") == 0);

    /* The bridge refuses the manifest: failed with its reason. */
    slot.version = wbo_pack_version("0.4.0");
    sim.refuse_begin = true;
    run(12000);
    CHECK(strstr(wbo_status(&relay), "update to 0.4.0 failed: ota-verify: signature: unknown-key") != NULL);
    CHECK(strcmp(sim.fw, "0.3.0") == 0);
    sim.refuse_begin = false;

    /* Forced without a bridge: says why. */
    wb_set_link(&pod, false, now_ms);
    wbo_force(&relay);
    run(100);
    CHECK(strstr(wbo_status(&relay), "cannot update: no Wi-Fi bridge") != NULL);
    tick_hook = NULL;
}

int main(void)
{
    wb_info_t info;

    memset(&sim, 0, sizeof(sim));
    sim.state = BWP_ST_CONNECTED;  /* STATUS carries RSSI only when connected */
    strcpy(sim.ssid, "old net");
    strcpy(sim.fw, "0.1.0");
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
    expected_key(expect);
    CHECK(memcmp(sim.pair.key, expect, BWP_KEY_LEN) == 0);  /* same key on both sides, never sent */
    CHECK(memcmp(pod.key, expect, BWP_KEY_LEN) == 0);
    CHECK(info.paired && info.problem == NULL);
    CHECK(info.have_status && info.rssi == -58 && info.counters.eth_rx == 1234);
    CHECK(strcmp(info.fw_version, "0.1.0") == 0);

    /* A caller's SET_WIFI: signed, the passphrase sealed, answered, the signed answer accepted. */
    CHECK(call_set_wifi("lab net", "correct horse") == WB_REQ_DONE);
    CHECK(wb_request_reply(&pod)->result == BWP_OK);
    CHECK(strcmp(sim.ssid, "lab net") == 0 && strcmp(sim.pass, "correct horse") == 0);
    CHECK(!leaked);
    wb_request_release(&pod);
    CHECK(call_set_wifi("guest", "") == WB_REQ_DONE && wb_request_reply(&pod)->result == BWP_OK);
    CHECK(sim.pass[0] == 0);
    wb_request_release(&pod);

    /* Invalid SSID or passphrase: refused before anything is sent. */
    static const uint8_t nonce0[BWP_SEAL_NONCE_LEN];
    CHECK(wb_request_set_wifi(&pod, "", "correct horse", nonce0, 1000, now_ms) == -4);
    CHECK(wb_request_set_wifi(&pod, "lab", "short", nonce0, 1000, now_ms) == -4);

    /* A bad request comes back as the bridge's error, still signed. */
    static const uint8_t bad[] = {BWP_T_SSID, 0, 0};
    CHECK(call(BWP_SET_WIFI, bad, sizeof(bad), 1000) == WB_REQ_DONE);
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
    memset(sim.pub, 0, sizeof(sim.pub));
    run(2100);
    run(6000);
    expected_key(expect);  /* a new pairing: new bridge key pair, new key */
    CHECK(sim.pair.paired && memcmp(sim.pair.key, expect, BWP_KEY_LEN) == 0);
    CHECK(memcmp(pod.key, expect, BWP_KEY_LEN) == 0);
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
    memset(sim.pub, 0, sizeof(sim.pub));
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

    test_relay();

    CHECK(strcmp(wb_state_str(BWP_ST_CONNECTED), "connected") == 0);
    CHECK(strcmp(wb_result_str(BWP_E_ALREADY_PAIRED), "already-paired") == 0);
    printf("test_wifi_bridge: %s (%d failures, %d frames sent)\n", failures ? "FAIL" : "ok", failures, sent);
    return failures ? 1 : 0;
}
