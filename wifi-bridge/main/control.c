// Control channel on EtherType 0x88B5 (docs/PROTOCOL.md). One task handles requests in order,
// sends HELLO every 2 s and an unsolicited STATUS when the state changes.
#include <stdlib.h>
#include <string.h>

#include "bridge.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char *TAG = "ctl";

typedef struct {
    uint8_t *buf;  // NULL: state-change notification
    uint16_t len;
} item_t;

static QueueHandle_t s_q;
static uint8_t s_out[BWP_MAX_FRAME];
static bwp_ap_t s_aps[BWP_SCAN_MAX_APS];
static const uint8_t BCAST[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

void control_rx(uint8_t *frame, size_t len)
{
    item_t it = {.buf = frame, .len = (uint16_t)len};
    if (!s_q || xQueueSend(s_q, &it, 0) != pdTRUE) {
        free(frame);
    }
}

void control_notify(void)
{
    item_t it = {0};
    if (s_q) {
        xQueueSend(s_q, &it, 0);
    }
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
    // A valid request from the pod proves a new image works end to end: keep it. Before any
    // OTA_BEGIN is handled, which would otherwise overwrite the rollback slot.
    ota_confirm();
    uint8_t dst[6], mac[6];
    memcpy(dst, m.src, 6);
    eth_get_mac(mac);
    const char *msg = NULL;
    bwp_result_t r;

    switch (m.type) {
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
        r = bwp_parse_set_wifi(&m, &sw);
        if (r == BWP_OK && ota_active()) {
            r = BWP_E_BUSY;
            msg = "OTA in progress";
        } else if (r == BWP_OK && bridge_set_wifi(sw.ssid, sw.ssid_len, sw.passphrase) != ESP_OK) {
            r = BWP_E_STORAGE;
        } else if (r != BWP_OK) {
            msg = "SSID 1-32 bytes; passphrase empty, 8-63 printable ASCII or 64 hex digits";
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
        r = bwp_get_u32(m.body, m.body_len, BWP_T_IMAGE_SIZE, &size) ? ota_begin(size, &msg) : BWP_E_BAD_REQUEST;
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
        r = creds_clear() == ESP_OK ? BWP_OK : BWP_E_STORAGE;
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
        if (!it.buf) {
            uint8_t pod[6];
            if (eth_link_up()) {
                send_status(bridge_pod_mac(pod) ? pod : BCAST, 0);
            }
            continue;
        }
        handle(it.buf, it.len);
        secure_zero(it.buf, it.len);  // may hold a passphrase
        free(it.buf);
    }
}

void control_start(void)
{
    s_q = xQueueCreate(16, sizeof(item_t));
    xTaskCreate(control_task, "control", 6144, NULL, 6, NULL);
}
