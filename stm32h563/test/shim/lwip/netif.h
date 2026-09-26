/* Host-test shim: esp_netif.h only needs the netif type name (esp_wifi_ctrl.c never
   touches lwIP directly; the tests fake esp_netif_set_link_up/set_hwaddr). */
#ifndef TEST_SHIM_LWIP_NETIF_H
#define TEST_SHIM_LWIP_NETIF_H
struct netif;
#endif
