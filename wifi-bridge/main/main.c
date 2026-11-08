// BenchPod Wi-Fi bridge firmware: the pod's Ethernet port to Wi-Fi, layer 2, with the pod's MAC.
//
//   Ethernet (LAN8742A, RMII) --eth.c--> l2fwd --esp_wifi_internal_tx--> Wi-Fi station
//   Wi-Fi station --wifi_rx--> l2fwd --esp_eth_transmit--> Ethernet
//   EtherType 0x88B5 from the pod --> control.c (docs/PROTOCOL.md)
//   UART0 (TC2030 pads) --> console.c (factory/debug only)
//
// No IP address, no esp_netif, no open ports, no SoftAP.
#include <string.h>

#include "bridge.h"
#include "esp_app_desc.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"   // esp_reset_reason
#include "esp_mac.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

static const char *TAG = "bridge";

bridge_t g_bridge = {.lock = portMUX_INITIALIZER_UNLOCKED};

bool bridge_pod_mac(uint8_t mac[6])
{
    portENTER_CRITICAL(&g_bridge.lock);
    bool known = g_bridge.fwd.pod_known;
    memcpy(mac, g_bridge.fwd.pod_mac, 6);
    portEXIT_CRITICAL(&g_bridge.lock);
    return known;
}

void bridge_counters(bwp_counters_t *c)
{
    c->eth_rx = g_bridge.eth_rx;
    c->wifi_tx = g_bridge.wifi_tx;
    c->wifi_tx_err = g_bridge.wifi_tx_err;
    c->wifi_rx = g_bridge.wifi_rx;
    c->eth_tx = g_bridge.eth_tx;
    c->eth_tx_err = g_bridge.eth_tx_err;
    c->dropped = g_bridge.drop_eth + g_bridge.drop_wifi;
#if CONFIG_BRIDGE_EMC_TEST
    // EMC test build: the pod's `bridge` shows the radio's counts (no UART needed).
    emc_counts_t e;
    emc_test_counts(&e);
    c->wifi_tx = e.sent;           // frames put on the air
    c->wifi_tx_err = e.errors;     // sends that failed (a full queue is not counted)
    c->wifi_rx = e.sent_eth;       // of those, frames that carried the pod's frames
    c->dropped = e.eth_skipped;    // the pod's frames replaced before the radio took them
#endif
}

uint8_t bridge_flags(void)
{
    uint8_t mac[6];
    uint8_t f = 0;
    if (creds_present()) {
        f |= BWP_F_CONFIGURED;
    }
    if (eth_link_up()) {
        f |= BWP_F_ETH_LINK;
    }
    if (ota_active()) {
        f |= BWP_F_OTA_ACTIVE;
    }
    if (ota_pending_verify()) {
        f |= BWP_F_PENDING_VERIFY;
    }
    if (control_paired()) {
        f |= BWP_F_PAIRED;
    }
    if (bridge_pod_mac(mac)) {
        f |= BWP_F_POD_KNOWN;
    }
    return f;
}

// Why this boot happened, for STATUS (BWP_T_RESET).
uint8_t bridge_reset_reason(void)
{
    switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  return BWP_RESET_POWER_ON;
    case ESP_RST_EXT:      return BWP_RESET_PIN;
    case ESP_RST_SW:       return BWP_RESET_SOFTWARE;
    case ESP_RST_PANIC:    return BWP_RESET_PANIC;
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:      return BWP_RESET_WATCHDOG;
    case ESP_RST_BROWNOUT: return BWP_RESET_BROWNOUT;
    default:               return BWP_RESET_OTHER;
    }
}

void bridge_status(bwp_status_t *s)
{
    memset(s, 0, sizeof(*s));
    s->state = wifi_state();
    s->flags = bridge_flags();
    wifi_fill_status(s);
#if CONFIG_BRIDGE_EMC_TEST
    emc_counts_t e;
    emc_test_counts(&e);
    s->channel = e.channel;  // the channel it transmits on
#endif
    s->uptime_s = (uint32_t)(esp_timer_get_time() / 1000000);
    s->reset = bridge_reset_reason();
#if CONFIG_BRIDGE_EMC_TEST
    switch (emc_test_state()) {
    case EMC_TX_ETH:     s->emc = BWP_EMC_TX_POD; break;
    case EMC_TX_PATTERN: s->emc = BWP_EMC_TX_PATTERN; break;
    case EMC_STOPPED:    s->emc = BWP_EMC_STOPPED; break;
    default:             s->emc = BWP_EMC_FAILING; break;
    }
#endif
    bridge_pod_mac(s->pod_mac);
    bridge_counters(&s->counters);
}

void app_main(void)
{
    board_init();

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS: %s, erasing", esp_err_to_name(err));
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    const esp_app_desc_t *d = esp_app_get_description();
    if (CONFIG_BRIDGE_HW_REV[0]) {
        ESP_LOGI(TAG, "%s %s, hardware rev %s", BWP_MODEL, d->version, CONFIG_BRIDGE_HW_REV);
    } else {
        ESP_LOGI(TAG, "%s %s", BWP_MODEL, d->version);
    }

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_ETH);
    l2fwd_init(&g_bridge.fwd, mac);

    ota_init();
    ota_check_pending();  // before anything that may hang: the rollback timer must run
#if CONFIG_BRIDGE_EMC_TEST
    // EMC test build: the radio transmits from boot instead of joining a network.
    if (emc_test_start() != ESP_OK) {
        ESP_LOGE(TAG, "EMC test: the radio did not start");
    }
#else
    ESP_ERROR_CHECK(wifi_start());
#endif
    control_start();  // after Wi-Fi: the RF noise makes esp_random() a true RNG for the nonce
    if (eth_start() != ESP_OK) {
        // Keep running for the debug console. A new OTA image that cannot start Ethernet never
        // hears from the pod, so its confirmation timer reboots it back to the previous slot.
        ESP_LOGE(TAG, "Ethernet did not start; bridging disabled");
    }
#if CONFIG_BRIDGE_EMC_TEST
    else {
        emc_test_eth_started();
    }
#endif
    board_start_tasks();
    console_start();
}
