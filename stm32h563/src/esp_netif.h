#ifndef ESP_NETIF_H
#define ESP_NETIF_H

#include <stdint.h>
#include <stdbool.h>
#include "lwip/netif.h"
#include "lwip/err.h"

/* ---- LwIP Wi-Fi netif over the esp-hosted SPI transport --------------------
 *
 * An ethernet-style netif (parallel to ethernetif.c) whose link output sends
 * 802.3 frames to the ESP32-C3 on ESP_STA_IF, and whose input is fed by the
 * transport's data callback. The host MAC mirrors the ESP32 STA MAC (set via
 * esp_netif_set_hwaddr() once esp_wifi_ctrl has fetched it over RPC).
 * ---------------------------------------------------------------------------*/

/* netif init callback (pass to netif_add as the `init` argument). Registers the
   transport data callback so received STA frames are injected into this netif. */
err_t esp_netif_init(struct netif *netif);

/* Set the interface MAC (the ESP32 STA MAC). Call before bringing the link up. */
void  esp_netif_set_hwaddr(const uint8_t mac[6]);

/* Reflect the Wi-Fi association state into the netif link flag (drives routing
   and restarts DHCP on link-up). Call from the control path on connect/disconnect. */
void  esp_netif_set_link_up(bool up);

#endif /* ESP_NETIF_H */
