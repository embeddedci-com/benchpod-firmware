// Control channel on EtherType 0x88B5 (docs/PROTOCOL.md). One task handles requests in order,
// sends HELLO every 2 s and an unsolicited STATUS when the state changes.
//
// Pairing (components/bridge_core/pairing.c): the first pod that sends PAIR with a key is
// trusted; after that every request must be signed with that key and this boot's nonce, and
// responses to signed requests are signed too. The PAIR button forgets the key.
#include <stdlib.h>
#include <string.h>

#include "bridge.h"
#include "bridge_util.h"
#include "bwp_crypto.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "pairing.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char *TAG = "ctl";

typedef enum { IT_FRAME, IT_NOTIFY, IT_FORGET } item_kind_t;

typedef struct {
    item_kind_t kind;
    uint8_t *buf;
    uint16_t len;
} item_t;

static QueueHandle_t s_q;
static uint8_t s_out[BWP_MAX_FRAME];
static bwp_ap_t s_aps[BWP_SCAN_MAX_APS];
static const uint8_t BCAST[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
static bwp_pairing_t s_pair;      // owned by the control task
static volatile bool s_paired;    // copy for HELLO flags and the console
static uint8_t s_pub[BWP_PUBKEY_LEN];  // our X25519 public key from the pairing (HELLO)
static bool s_sign;               // sign the response to the request being handled
static uint32_t s_counter;        // its COUNTER

void control_rx(uint8_t *frame, size_t len)
{
    item_t it = {.kind = IT_FRAME, .buf = frame, .len = (uint16_t)len};
    if (!s_q || xQueueSend(s_q, &it, 0) != pdTRUE) {
        free(frame);
    }
}

void control_notify(void)
{
    item_t it = {.kind = IT_NOTIFY};
    if (s_q) {
        xQueueSend(s_q, &it, 0);
    }
}

void control_forget_pairing(void)
{
    item_t it = {.kind = IT_FORGET};
    if (s_q) {
        xQueueSend(s_q, &it, portMAX_DELAY);
    }
}

bool control_paired(void)
{
    return s_paired;
}

static void forget_pairing(void)
{
    pairkey_clear();
    pairing_forget(&s_pair);
    memset(s_pub, 0, sizeof(s_pub));
    s_paired = false;
    // Let a different pod on the cable become "the pod" without unplugging it.
    portENTER_CRITICAL(&g_bridge.lock);
    l2fwd_link_down(&g_bridge.fwd);
    portEXIT_CRITICAL(&g_bridge.lock);
    ESP_LOGW(TAG, "pairing forgotten: the next pod that sends PAIR is trusted");
}

static void secure_zero(void *p, size_t n)
{
    volatile uint8_t *v = p;
    while (n--) {
        *v++ = 0;
    }
}

static void send(size_t n)
{
    if (n && s_sign) {
        n = bwp_sign_frame(s_out, sizeof(s_out), s_pair.key, s_pair.nonce, s_counter);
    }
    if (n) {
        eth_send(s_out, n);
    }
}

static void send_status(const uint8_t dst[6], uint16_t seq)
{
    bwp_status_t s;
    uint8_t mac[6];
    bridge_status(&s);
    eth_get_mac(mac);
    send(bwp_build_status(s_out, sizeof(s_out), dst, mac, seq, &s));
}

static void send_hello(void)
{
    bwp_hello_t h = {.fw_version = esp_app_get_description()->version, .hw_rev = CONFIG_BRIDGE_HW_REV,
                     .state = wifi_state(), .flags = bridge_flags()};
    memcpy(h.nonce, s_pair.nonce, BWP_NONCE_LEN);
    memcpy(h.bridge_pub, s_pub, BWP_PUBKEY_LEN);
    eth_get_mac(h.bridge_mac);
    bool known = bridge_pod_mac(h.pod_mac);
    send(bwp_build_hello(s_out, sizeof(s_out), known ? h.pod_mac : NULL, &h));
}

static void reply(const bwp_msg_t *m, const uint8_t dst[6], uint8_t result, const char *msg)
{
    uint8_t mac[6];
    eth_get_mac(mac);
    send(bwp_build_result(s_out, sizeof(s_out), dst, mac, m->type, m->seq, result, msg));
}

static void handle(uint8_t *buf, size_t len)
{
    bwp_msg_t m;
    bwp_parse_t p = bwp_parse(buf, len, &m);
    if (p != BWP_PARSE_OK) {
        if (p != BWP_PARSE_NOT_BWP) {
            ESP_LOGW(TAG, "dropped frame: %s", p == BWP_PARSE_BAD_VERSION ? "unknown version" : "malformed");
        }
        return;
    }
    if (m.type & BWP_RESPONSE) {
        return;  // the bridge never sends requests
    }
    uint8_t dst[6], mac[6];
    memcpy(dst, m.src, 6);
    eth_get_mac(mac);
    const char *msg = NULL;
    bwp_result_t r;

    bool authed;
    const uint8_t *new_key;
    pair_verdict_t v = pairing_check(&s_pair, buf, &m, &authed, &s_counter, &new_key);
    if (v != PAIR_ACCEPT && v != PAIR_STORE_KEY) {
        // Unsigned answer with our nonce, so a pod that missed a reboot can resync.
        static const char *why[] = {[PAIR_NOT_PAIRED] = "not paired: send PAIR first",
                                    [PAIR_ALREADY_PAIRED] = "paired with another pod: press PAIR on the bridge",
                                    [PAIR_AUTH_MISSING] = "AUTH required",
                                    [PAIR_AUTH_BAD] = "AUTH wrong (key or nonce)",
                                    [PAIR_REPLAY] = "COUNTER not increasing",
                                    [PAIR_BAD_KEY] = "KEY must be 32 bytes"};
        ESP_LOGW(TAG, "refused request 0x%02x: %s", m.type, why[v]);
        bwp_writer_t w;
        bwp_begin(&w, s_out, sizeof(s_out), dst, mac, m.type | BWP_RESPONSE, m.seq);
        bwp_put_u8(&w, BWP_T_RESULT, pairing_result(v));
        bwp_put_str(&w, BWP_T_MESSAGE, why[v]);
        bwp_put(&w, BWP_T_NONCE, s_pair.nonce, BWP_NONCE_LEN);
        if (v == PAIR_REPLAY) {
            // The MAC was right, only the counter is old: a rebooted pod that started again at 1.
            // Tell it where to continue. This reveals nothing: counters are not secret.
            bwp_put_u32(&w, BWP_T_COUNTER, s_pair.last_counter);
        }
        send(bwp_finish(&w));
        return;
    }
    s_sign = authed;
    // A valid request from the pod proves a new image works end to end: keep it. Before any
    // OTA_BEGIN is handled, which would otherwise overwrite the rollback slot.
    ota_confirm();

    switch (m.type) {
    case BWP_PAIR: {
        // X25519: a fresh key pair for this pairing; the key is derived on both sides and never
        // crosses the cable. Our public key goes back in the answer (and in every HELLO).
        uint8_t secret[32], pub[BWP_PUBKEY_LEN], key[BWP_KEY_LEN];
        esp_fill_random(secret, sizeof(secret));
        bwp_x25519_public(secret, pub);
        if (bwp_pair_key(secret, new_key, new_key, pub, key) != 0) {
            r = BWP_E_BAD_REQUEST;
            msg = "bad public key";
        } else if (pairkey_store(key, pub) != ESP_OK) {
            r = BWP_E_STORAGE;
        } else {
            r = BWP_OK;
        }
        secure_zero(secret, sizeof(secret));
        bwp_writer_t w;
        bwp_begin(&w, s_out, sizeof(s_out), dst, mac, m.type | BWP_RESPONSE, m.seq);
        bwp_put_u8(&w, BWP_T_RESULT, r);
        if (r == BWP_OK) {
            bwp_put(&w, BWP_T_PUBKEY, pub, sizeof(pub));
        } else if (msg) {
            bwp_put_str(&w, BWP_T_MESSAGE, msg);
        }
        send(bwp_finish(&w));  // a rotation is answered under the old key, so switch after
        if (r == BWP_OK) {
            pairing_set_key(&s_pair, key);
            memcpy(s_pub, pub, sizeof(s_pub));
            s_paired = true;
            char m6[18];
            bu_mac_str(dst, m6);
            ESP_LOGI(TAG, "%s with pod %s", authed ? "pairing key rotated" : "paired", m6);
        }
        secure_zero(key, sizeof(key));
        return;
    }
    case BWP_STATUS:
        send_status(dst, m.seq);
        return;
    case BWP_SCAN: {
        size_t n = 0;
        r = wifi_scan(s_aps, BWP_SCAN_MAX_APS, &n);
        send(bwp_build_scan_result(s_out, sizeof(s_out), dst, mac, m.seq, r, s_aps, n));
        return;
    }
    case BWP_SET_WIFI: {
        bwp_set_wifi_t sw;
        r = bwp_parse_set_wifi(&m, s_pair.paired ? s_pair.key : NULL, &sw);
        if (r == BWP_OK && ota_active()) {
            r = BWP_E_BUSY;
            msg = "OTA in progress";
        } else if (r == BWP_OK && bridge_set_wifi(sw.ssid, sw.ssid_len, sw.passphrase) != ESP_OK) {
            r = BWP_E_STORAGE;
        } else if (r != BWP_OK) {
            msg = "SSID 1-32 bytes; passphrase sealed (ENC_PASSPHRASE), empty, 8-63 printable ASCII or 64 hex";
        }
        secure_zero(&sw, sizeof(sw));
        reply(&m, dst, r, msg);
        return;
    }
    case BWP_CLEAR_WIFI:
        reply(&m, dst, bridge_clear_wifi() == ESP_OK ? BWP_OK : BWP_E_STORAGE, NULL);
        return;
    case BWP_OTA_BEGIN: {
        uint32_t size;
        const uint8_t *sig = NULL;
        uint16_t sig_len = 0;
        bwp_find(m.body, m.body_len, BWP_T_SIGNATURE, &sig, &sig_len);  // the pod's signed manifest
        r = bwp_get_u32(m.body, m.body_len, BWP_T_IMAGE_SIZE, &size) ? ota_begin(size, sig, sig_len, &msg)
                                                                     : BWP_E_BAD_REQUEST;
        reply(&m, dst, r, msg);
        return;
    }
    case BWP_OTA_DATA: {
        uint32_t off, next = 0;
        const uint8_t *data;
        uint16_t dlen;
        if (bwp_get_u32(m.body, m.body_len, BWP_T_OFFSET, &off) &&
            bwp_find(m.body, m.body_len, BWP_T_DATA, &data, &dlen)) {
            r = ota_data(off, data, dlen, &next, &msg);
        } else {
            r = BWP_E_BAD_REQUEST;
        }
        bwp_writer_t w;
        bwp_begin(&w, s_out, sizeof(s_out), dst, mac, m.type | BWP_RESPONSE, m.seq);
        bwp_put_u8(&w, BWP_T_RESULT, r);
        bwp_put_u32(&w, BWP_T_OFFSET, next);
        if (msg) {
            bwp_put_str(&w, BWP_T_MESSAGE, msg);
        }
        send(bwp_finish(&w));
        return;
    }
    case BWP_OTA_END:
        r = ota_end(&msg);
        reply(&m, dst, r, msg);
        return;
    case BWP_OTA_ABORT:
        ota_abort();
        reply(&m, dst, BWP_OK, NULL);
        return;
    case BWP_REBOOT:
        reply(&m, dst, BWP_OK, NULL);
        ESP_LOGW(TAG, "reboot requested by the pod");
        vTaskDelay(pdMS_TO_TICKS(200));
        board_reboot();
        return;
    case BWP_FACTORY_RESET:
        r = creds_clear() == ESP_OK && pairkey_clear() == ESP_OK ? BWP_OK : BWP_E_STORAGE;
        reply(&m, dst, r, NULL);
        ESP_LOGW(TAG, "factory reset requested by the pod");
        vTaskDelay(pdMS_TO_TICKS(200));
        board_reboot();
        return;
    default:
        reply(&m, dst, BWP_E_UNSUPPORTED, NULL);
        return;
    }
}

static void control_task(void *arg)
{
    int64_t next_hello = 0;
    const int64_t interval = (int64_t)CONFIG_BRIDGE_HELLO_INTERVAL_MS * 1000;
    item_t it;
    for (;;) {
        int64_t now = esp_timer_get_time();
        if (now >= next_hello) {
            if (eth_link_up()) {
                send_hello();
            }
            ota_poll();
            next_hello = now + interval;
        }
        if (xQueueReceive(s_q, &it, pdMS_TO_TICKS((next_hello - now) / 1000 + 1)) != pdTRUE) {
            continue;
        }
        if (it.kind == IT_NOTIFY) {
            uint8_t pod[6];
            if (eth_link_up()) {
                send_status(bridge_pod_mac(pod) ? pod : BCAST, 0);
            }
            continue;
        }
        if (it.kind == IT_FORGET) {
            forget_pairing();
            continue;
        }
        handle(it.buf, it.len);
        s_sign = false;
        secure_zero(it.buf, it.len);  // may hold a passphrase
        free(it.buf);
    }
}

void control_start(void)
{
    uint8_t nonce[BWP_NONCE_LEN], key[BWP_KEY_LEN];
    esp_fill_random(nonce, sizeof(nonce));  // true random: Wi-Fi (RF) is already started
    bool have = pairkey_load(key, s_pub);
    pairing_init(&s_pair, nonce, have ? key : NULL);
    secure_zero(key, sizeof(key));
    s_paired = have;
    ESP_LOGI(TAG, "%s", have ? "paired with a pod" : "not paired: the first pod that sends PAIR is trusted");
    s_q = xQueueCreate(16, sizeof(item_t));
    xTaskCreate(control_task, "control", 6144, NULL, 6, NULL);
}
