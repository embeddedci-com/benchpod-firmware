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

void bridge_status(bwp_status_t *s)
{
    memset(s, 0, sizeof(*s));
    s->state = wifi_state();
    s->flags = bridge_flags();
    wifi_fill_status(s);
    s->uptime_s = (uint32_t)(esp_timer_get_time() / 1000000);
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
    ESP_ERROR_CHECK(wifi_start());
    control_start();  // after Wi-Fi: the RF noise makes esp_random() a true RNG for the nonce
    if (eth_start() != ESP_OK) {
        // Keep running for the debug console. A new OTA image that cannot start Ethernet never
        // hears from the pod, so its confirmation timer reboots it back to the previous slot.
        ESP_LOGE(TAG, "Ethernet did not start; bridging disabled");
    }
    board_start_tasks();
    console_start();
}
