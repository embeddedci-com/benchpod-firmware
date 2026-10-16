// Ethernet: ESP32 EMAC + LAN8742A over RMII, promiscuous, frames go straight to the bridge logic
// (no esp_netif, no lwIP: the bridge has no IP address).
#include <stdlib.h>
#include <string.h>

#include "bridge.h"
#include "bridge_util.h"
#include "esp_check.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_private/wifi.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"

static const char *TAG = "eth";
static esp_eth_handle_t s_eth;
static SemaphoreHandle_t s_tx_lock;  // esp_eth_transmit is called from the Wi-Fi and control tasks
static volatile bool s_link;
static uint8_t s_mac[6];

esp_err_t eth_send(const uint8_t *frame, size_t len)
{
    if (!s_eth || !s_link) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(s_tx_lock, portMAX_DELAY);
    esp_err_t err = esp_eth_transmit(s_eth, (void *)frame, len);
    xSemaphoreGive(s_tx_lock);
    return err;
}

bool eth_link_up(void)
{
    return s_link;
}

void eth_get_mac(uint8_t mac[6])
{
    memcpy(mac, s_mac, 6);
}

// Runs in the EMAC rx task. Owns `buf` (malloc'd by the driver).
static esp_err_t eth_input(esp_eth_handle_t h, uint8_t *buf, uint32_t len, void *priv)
{
    l2_event_t ev;
    g_bridge.eth_rx++;
    portENTER_CRITICAL(&g_bridge.lock);
    l2_action_t a = l2fwd_from_eth(&g_bridge.fwd, buf, len, wifi_is_connected(), &ev);
    portEXIT_CRITICAL(&g_bridge.lock);

    if (ev != L2_EV_NONE) {
        char m[18];
        bu_mac_str(buf + 6, m);
        ESP_LOGI(TAG, "pod MAC %s %s", ev == L2_EV_LEARNED ? "learned:" : "changed to", m);
        wifi_pod_mac_changed();
    }
    switch (a) {
    case L2_FORWARD:
        // esp_wifi_internal_tx copies the frame.
        if (esp_wifi_internal_tx(WIFI_IF_STA, buf, (uint16_t)len) == ESP_OK) {
            g_bridge.wifi_tx++;
            board_activity();
        } else {
            g_bridge.wifi_tx_err++;
        }
        break;
    case L2_CONTROL:
        control_rx(buf, len);
        return ESP_OK;  // control owns it now
    default:
        g_bridge.drop_eth++;
        break;
    }
    free(buf);
    return ESP_OK;
}

static void eth_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    switch (id) {
    case ETHERNET_EVENT_CONNECTED:
        s_link = true;
        ESP_LOGI(TAG, "link up");
        control_notify();
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        s_link = false;
        portENTER_CRITICAL(&g_bridge.lock);
        l2fwd_link_down(&g_bridge.fwd);  // a different pod may be plugged in next
        portEXIT_CRITICAL(&g_bridge.lock);
        ESP_LOGI(TAG, "link down");
        break;
    default:
        break;
    }
}

esp_err_t eth_start(void)
{
    s_tx_lock = xSemaphoreCreateMutex();
    esp_read_mac(s_mac, ESP_MAC_ETH);

    // REF_CLK In mode: the 50 MHz clock must run before the EMAC is reset (in mac_new) and
    // before the PHY leaves reset (in driver_install).
    board_rmii_clock(true);

    eth_mac_config_t mac_cfg = ETH_MAC_DEFAULT_CONFIG();
    eth_esp32_emac_config_t emac_cfg = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    emac_cfg.smi_gpio.mdc_num = PIN_MDC;
    emac_cfg.smi_gpio.mdio_num = PIN_MDIO;
#if CONFIG_BRIDGE_RMII_CLK_EXT_OSC
    emac_cfg.clock_config.rmii.clock_mode = EMAC_CLK_EXT_IN;
    emac_cfg.clock_config.rmii.clock_gpio = EMAC_CLK_IN_GPIO;
#else
    emac_cfg.clock_config.rmii.clock_mode = EMAC_CLK_OUT;
    emac_cfg.clock_config.rmii.clock_gpio = EMAC_CLK_OUT_180_GPIO;
#endif
    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&emac_cfg, &mac_cfg);
    if (!mac) {
        ESP_LOGE(TAG, "EMAC init failed (no RMII clock?)");
        return ESP_FAIL;
    }

    eth_phy_config_t phy_cfg = ETH_PHY_DEFAULT_CONFIG();
    phy_cfg.phy_addr = CONFIG_BRIDGE_PHY_ADDR;
    phy_cfg.reset_gpio_num = PIN_PHY_NRST;
    phy_cfg.hw_reset_assert_time_us = 200;  // LAN8742A: nRST low >= 100 us
    phy_cfg.post_hw_reset_delay_ms = 2;
    esp_eth_phy_t *phy = esp_eth_phy_new_lan87xx(&phy_cfg);

    esp_eth_config_t cfg = ETH_DEFAULT_CONFIG(mac, phy);
    cfg.stack_input = eth_input;
    ESP_RETURN_ON_ERROR(esp_eth_driver_install(&cfg, &s_eth), TAG, "driver install");
    ESP_RETURN_ON_ERROR(esp_eth_ioctl(s_eth, ETH_CMD_S_MAC_ADDR, s_mac), TAG, "set MAC");
    // The pod talks to the router's MAC, not ours: receive everything, filter in l2fwd.
    bool promiscuous = true;
    ESP_RETURN_ON_ERROR(esp_eth_ioctl(s_eth, ETH_CMD_S_PROMISCUOUS, &promiscuous), TAG, "promiscuous");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, eth_event, NULL), TAG, "events");
    ESP_RETURN_ON_ERROR(esp_eth_start(s_eth), TAG, "start");

    char m[18];
    bu_mac_str(s_mac, m);
    ESP_LOGI(TAG, "started, bridge MAC %s, PHY addr %d", m, CONFIG_BRIDGE_PHY_ADDR);
    return ESP_OK;
}
