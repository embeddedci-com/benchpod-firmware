// Wi-Fi station with the pod's MAC and no network interface: frames go to and come from the
// Ethernet port through esp_wifi_internal_tx / esp_wifi_internal_reg_rxcb, the same raw path
// esp_netif uses, so there is no IP stack on the radio side.
//
// One task owns the station: it (re)starts it with the pod MAC, connects with the stored
// credentials, retries with backoff and runs scans, so all esp_wifi calls happen in one place.
#include <stdlib.h>
#include <string.h>

#include "bridge.h"
#include "bridge_util.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_private/wifi.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char *TAG = "wifi";

typedef struct {
    bwp_ap_t *aps;
    size_t max;
    size_t n;
    bwp_result_t result;
    SemaphoreHandle_t done;
} scan_req_t;

typedef enum { EV_APPLY, EV_STA_START, EV_CONNECTED, EV_DISCONNECTED, EV_SCAN, EV_POD_GONE, EV_POD_BACK } ev_kind_t;

typedef struct {
    ev_kind_t kind;
    uint16_t reason;
    scan_req_t *scan;
} ev_t;

#define REASON_ASSOC_LEAVE 8  // we left on purpose (disconnect, stop, MAC change)
#define BACKOFF_MIN_MS 1000
#define BACKOFF_MAX_MS 30000
// No pod for this long after a link-down: off Wi-Fi, radio off. Long enough for a pod to reboot.
// On the air the station would keep using the pod's MAC, also once the pod is on a wired LAN.
#define POD_GONE_PARK_MS 30000

static QueueHandle_t s_q;
static SemaphoreHandle_t s_scan_lock;
static volatile uint8_t s_state = BWP_ST_UNCONFIGURED;
static void set_state(uint8_t st);
static volatile bool s_connected;
static volatile uint8_t s_reason;
// Owned by the Wi-Fi task:
static bool s_started, s_connecting;
static uint8_t s_sta_mac[6];
static uint32_t s_creds_gen = 1, s_conn_gen;
static int s_backoff_ms;
static int64_t s_retry_at;  // esp_timer time in us, 0: none
static int64_t s_park_at;   // same, 0: none
static bool s_parked;

static void post(ev_kind_t kind, uint16_t reason)
{
    ev_t ev = {.kind = kind, .reason = reason};
    if (s_q) {
        xQueueSend(s_q, &ev, 0);
    }
}

void wifi_pod_mac_changed(void)
{
    post(EV_APPLY, 0);
}

void wifi_pod_gone(void)
{
    post(EV_POD_GONE, 0);
}

void wifi_pod_back(void)
{
    post(EV_POD_BACK, 0);
}

void wifi_creds_changed(void)
{
    __atomic_add_fetch(&s_creds_gen, 1, __ATOMIC_SEQ_CST);
    // Before SET_WIFI is answered: a STATUS right behind it must not report the old network's result.
    if (creds_present()) {
        set_state(BWP_ST_JOINING);
    }
    post(EV_APPLY, 0);
}

bool wifi_is_connected(void)
{
    return s_connected;
}

uint8_t wifi_state(void)
{
    return s_state;
}

static void set_state(uint8_t st)
{
    if (st != s_state) {
        static const char *names[] = {"unconfigured", "wait-pod", "joining", "connected", "auth-failed", "no-ap"};
        ESP_LOGI(TAG, "state %s", st < 6 ? names[st] : "?");
        s_state = st;
        control_notify();
    }
}

// Wi-Fi -> Ethernet. Runs in the Wi-Fi driver task.
static esp_err_t wifi_rx(void *buffer, uint16_t len, void *eb)
{
    g_bridge.wifi_rx++;
    portENTER_CRITICAL(&g_bridge.lock);
    l2_action_t a = l2fwd_from_wifi(&g_bridge.fwd, buffer, len);
    portEXIT_CRITICAL(&g_bridge.lock);
    if (a == L2_FORWARD) {
        if (eth_send(buffer, len) == ESP_OK) {
            g_bridge.eth_tx++;
        } else {
            g_bridge.eth_tx_err++;
        }
    } else {
        g_bridge.drop_wifi++;
    }
    if (eb) {
        esp_wifi_internal_free_rx_buffer(eb);
    }
    return ESP_OK;
}

static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    switch (id) {
    case WIFI_EVENT_STA_START:
        post(EV_STA_START, 0);
        break;
    case WIFI_EVENT_STA_CONNECTED:
        post(EV_CONNECTED, 0);
        break;
    case WIFI_EVENT_STA_DISCONNECTED:
        post(EV_DISCONNECTED, ((wifi_event_sta_disconnected_t *)data)->reason);
        break;
    default:
        break;
    }
}

static void schedule_retry(void)
{
    s_backoff_ms = s_backoff_ms ? s_backoff_ms * 2 : BACKOFF_MIN_MS;
    if (s_backoff_ms > BACKOFF_MAX_MS) {
        s_backoff_ms = BACKOFF_MAX_MS;
    }
    s_retry_at = esp_timer_get_time() + (int64_t)s_backoff_ms * 1000;
}

static void leave(void)
{
    if (s_connected || s_connecting) {
        esp_wifi_internal_reg_rxcb(WIFI_IF_STA, NULL);
        s_connected = false;
        s_connecting = false;
        esp_wifi_disconnect();
    }
}

// Brings the station to where the stored credentials and the learned pod MAC say it should be.
static void apply(void)
{
    char ssid[33], pass[65];
    size_t ssid_len;
    uint8_t pod[6];

    if (!creds_load(ssid, &ssid_len, pass)) {
        leave();
        s_retry_at = 0;
        set_state(BWP_ST_UNCONFIGURED);
        return;
    }
    if (s_parked || !bridge_pod_mac(pod)) {
        memset(pass, 0, sizeof(pass));
        set_state(BWP_ST_WAIT_POD);
        return;
    }
    if (!s_started) {
        memset(pass, 0, sizeof(pass));
        return;  // EV_STA_START applies again
    }
    if (memcmp(pod, s_sta_mac, 6) != 0) {
        // The station MAC can only change while Wi-Fi is stopped.
        char m[18];
        bu_mac_str(pod, m);
        ESP_LOGI(TAG, "station MAC -> %s", m);
        leave();
        esp_wifi_stop();
        s_started = false;
        esp_err_t err = esp_wifi_set_mac(WIFI_IF_STA, pod);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "set_mac: %s", esp_err_to_name(err));
        } else {
            memcpy(s_sta_mac, pod, 6);
        }
        esp_wifi_start();
        set_state(BWP_ST_JOINING);
        memset(pass, 0, sizeof(pass));
        return;
    }
    uint32_t gen = __atomic_load_n(&s_creds_gen, __ATOMIC_SEQ_CST);
    if ((s_connected || s_connecting) && s_conn_gen == gen) {
        memset(pass, 0, sizeof(pass));
        return;  // already on (or joining) the right network
    }
    leave();

    wifi_config_t cfg = {0};
    memcpy(cfg.sta.ssid, ssid, ssid_len);
    strlcpy((char *)cfg.sta.password, pass, sizeof(cfg.sta.password));
    memset(pass, 0, sizeof(pass));
    // No WEP, no WPA1/TKIP: an open network only when no passphrase is set.
    cfg.sta.threshold.authmode = cfg.sta.password[0] ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    cfg.sta.pmf_cfg.capable = true;
    cfg.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;
    cfg.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    cfg.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &cfg);
    memset(&cfg, 0, sizeof(cfg));
    if (err == ESP_OK) {
        err = esp_wifi_connect();
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "connect: %s", esp_err_to_name(err));
        schedule_retry();
        return;
    }
    s_connecting = true;
    s_conn_gen = gen;
    s_retry_at = 0;
    set_state(BWP_ST_JOINING);
}

static int ap_cmp(const void *a, const void *b)
{
    return ((const wifi_ap_record_t *)b)->rssi - ((const wifi_ap_record_t *)a)->rssi;
}

static void do_scan(scan_req_t *r)
{
    r->n = 0;
    r->result = BWP_E_INTERNAL;
    if (!s_started) {
        return;
    }
    // A connect attempt in progress blocks scanning: drop it, resume after the scan.
    bool resume = false;
    if (s_connecting && !s_connected) {
        leave();
        resume = true;
    }
    wifi_scan_config_t sc = {.show_hidden = false, .scan_type = WIFI_SCAN_TYPE_ACTIVE};
    esp_err_t err = ESP_FAIL;
    for (int i = 0; i < 5 && err != ESP_OK; i++) {
        err = esp_wifi_scan_start(&sc, true);
        if (err == ESP_ERR_WIFI_STATE) {
            vTaskDelay(pdMS_TO_TICKS(200));
        } else if (err != ESP_OK) {
            break;
        }
    }
    uint16_t num = 0;
    wifi_ap_record_t *recs = NULL;
    if (err == ESP_OK) {
        esp_wifi_scan_get_ap_num(&num);
        if (num > 64) {
            num = 64;
        }
        recs = calloc(num ? num : 1, sizeof(*recs));
        if (!recs || esp_wifi_scan_get_ap_records(&num, recs) != ESP_OK) {
            num = 0;
            err = ESP_ERR_NO_MEM;
        }
        esp_wifi_clear_ap_list();
    } else {
        ESP_LOGW(TAG, "scan: %s", esp_err_to_name(err));
    }
    if (err == ESP_OK) {
        qsort(recs, num, sizeof(*recs), ap_cmp);
        // Strongest first, one entry per SSID (mesh networks repeat it), no hidden networks.
        for (uint16_t i = 0; i < num && r->n < r->max; i++) {
            size_t len = strnlen((const char *)recs[i].ssid, sizeof(recs[i].ssid));
            if (len == 0 || len > 32) {
                continue;
            }
            bool dup = false;
            for (size_t j = 0; j < r->n && !dup; j++) {
                dup = strncmp(r->aps[j].ssid, (const char *)recs[i].ssid, 33) == 0;
            }
            if (dup) {
                continue;
            }
            bwp_ap_t *ap = &r->aps[r->n++];
            memcpy(ap->ssid, recs[i].ssid, len);
            ap->ssid[len] = '\0';
            ap->rssi = recs[i].rssi;
            ap->channel = recs[i].primary;
            ap->authmode = recs[i].authmode <= WIFI_AUTH_WPA2_WPA3_PSK ? (uint8_t)recs[i].authmode : BWP_AUTH_OTHER;
            memcpy(ap->bssid, recs[i].bssid, 6);
        }
        r->result = BWP_OK;
    }
    free(recs);
    if (resume) {
        s_retry_at = esp_timer_get_time();
    }
}

static void park(void)
{
    ESP_LOGI(TAG, "no pod for %d s: Wi-Fi off until it is back", POD_GONE_PARK_MS / 1000);
    s_parked = true;
    s_retry_at = 0;
    leave();
    esp_wifi_stop();
    s_started = false;
    apply();
}

static void wifi_task(void *arg)
{
    ev_t ev;
    for (;;) {
        TickType_t wait = portMAX_DELAY;
        int64_t next = s_retry_at && (!s_park_at || s_retry_at < s_park_at) ? s_retry_at : s_park_at;
        if (next) {
            int64_t ms = (next - esp_timer_get_time()) / 1000;
            wait = ms > 0 ? pdMS_TO_TICKS(ms) : 0;
        }
        if (xQueueReceive(s_q, &ev, wait) != pdTRUE) {
            int64_t now = esp_timer_get_time();
            if (s_park_at && now >= s_park_at) {
                s_park_at = 0;
                park();
            }
            if (s_retry_at && now >= s_retry_at) {
                s_retry_at = 0;
                apply();
            }
            continue;
        }
        switch (ev.kind) {
        case EV_APPLY:
            apply();
            break;
        case EV_STA_START:
            s_started = true;
            esp_wifi_set_ps(WIFI_PS_NONE);  // latency and throughput over power: USB-powered
            apply();
            break;
        case EV_CONNECTED:
            s_connecting = false;
            esp_wifi_internal_reg_rxcb(WIFI_IF_STA, wifi_rx);
            s_connected = true;
            s_reason = 0;
            s_backoff_ms = 0;
            set_state(BWP_ST_CONNECTED);
            break;
        case EV_DISCONNECTED:
            esp_wifi_internal_reg_rxcb(WIFI_IF_STA, NULL);
            s_connected = false;
            if (s_parked) {
                s_connecting = false;
                break;
            }
            if (ev.reason == REASON_ASSOC_LEAVE) {
                // Usually our own leave(), and a newer attempt may already be in flight. If
                // nothing is, retry anyway (an AP can send this reason too).
                if (!s_connecting && !s_retry_at && creds_present()) {
                    if (s_state == BWP_ST_CONNECTED) {
                        set_state(BWP_ST_JOINING);
                    }
                    schedule_retry();
                }
                break;
            }
            s_connecting = false;
            s_reason = (uint8_t)ev.reason;
            ESP_LOGW(TAG, "disconnected, reason %u", ev.reason);
            if (creds_present()) {
                set_state(bu_state_for_reason(ev.reason));
                schedule_retry();
            }
            break;
        case EV_SCAN:
            do_scan(ev.scan);
            xSemaphoreGive(ev.scan->done);
            break;
        case EV_POD_GONE:
            if (!s_parked) {
                s_park_at = esp_timer_get_time() + (int64_t)POD_GONE_PARK_MS * 1000;
            }
            break;
        case EV_POD_BACK:
            s_park_at = 0;
            if (s_parked) {
                s_parked = false;
                ESP_LOGI(TAG, "pod is back: Wi-Fi on");
                esp_wifi_start();  // EV_STA_START applies
            }
            break;
        }
    }
}

bwp_result_t wifi_scan(bwp_ap_t *aps, size_t max, size_t *n)
{
    *n = 0;
    if (xSemaphoreTake(s_scan_lock, 0) != pdTRUE) {
        return BWP_E_BUSY;
    }
    scan_req_t r = {.aps = aps, .max = max, .done = xSemaphoreCreateBinary()};
    ev_t ev = {.kind = EV_SCAN, .scan = &r};
    bwp_result_t res = BWP_E_INTERNAL;
    if (r.done && xQueueSend(s_q, &ev, pdMS_TO_TICKS(1000)) == pdTRUE) {
        xSemaphoreTake(r.done, portMAX_DELAY);  // the task always answers; r lives on our stack
        res = r.result;
        *n = r.n;
    }
    if (r.done) {
        vSemaphoreDelete(r.done);
    }
    xSemaphoreGive(s_scan_lock);
    return res;
}

void wifi_fill_status(bwp_status_t *s)
{
    size_t ssid_len;
    char pass[65];
    if (creds_load(s->ssid, &ssid_len, pass)) {
        memset(pass, 0, sizeof(pass));
    } else {
        s->ssid[0] = '\0';
    }
    s->reason = s_reason;
    wifi_ap_record_t ap;
    if (s_connected && esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        s->channel = ap.primary;
        s->rssi = ap.rssi;
        memcpy(s->bssid, ap.bssid, 6);
    }
}

esp_err_t wifi_start(void)
{
    s_q = xQueueCreate(16, sizeof(ev_t));
    s_scan_lock = xSemaphoreCreateMutex();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_country_code(CONFIG_BRIDGE_COUNTRY, false));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event, NULL));
    esp_read_mac(s_sta_mac, ESP_MAC_WIFI_STA);  // factory MAC until the pod's is known (scans only)

    s_state = creds_present() ? BWP_ST_WAIT_POD : BWP_ST_UNCONFIGURED;
    xTaskCreate(wifi_task, "wifi_ctl", 4096, NULL, 5, NULL);
    return esp_wifi_start();
}
