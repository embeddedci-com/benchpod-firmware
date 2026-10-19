#ifndef WIFI_BRIDGE_NET_H
#define WIFI_BRIDGE_NET_H
/*
 * wifi_bridge_net — runs the Wi-Fi bridge client (wifi_bridge.c) on the net task and gives the
 * hw worker (JSON commands, console lines) a blocking request API.
 *
 * Net task: wifi_bridge_net_init after the Ethernet netif is added, wifi_bridge_net_poll from
 * net_poll, and the lwIP LWIP_HOOK_UNKNOWN_ETH_PROTOCOL hook (config/lwipopts.h) feeds
 * wifi_bridge_eth_input. Only built with BENCHPOD_WIFI_BRIDGE (dev builds, see the Makefile).
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "wifi_bridge.h"

struct netif;
struct pbuf;

void wifi_bridge_net_init(struct netif *eth);
void wifi_bridge_net_poll(void);
int wifi_bridge_eth_input(struct pbuf *p, struct netif *netif);

/* Any task: a bridge said HELLO recently on a live Ethernet link. */
bool wifi_bridge_present(void);

/* hw worker only (it is the single net_call_sync caller). Sends one request and waits for the
   answer, the timeout, or the bridge going away; waits first while the pod pairs on first
   contact. Returns 0 with *out filled when the bridge answered (out->result may be an error),
   -1 with *err set otherwise. `out` is large (scan results): pass a static. */
int wifi_bridge_call(uint8_t type, const uint8_t *tlvs, size_t len, uint32_t timeout_ms, wb_reply_t *out,
                     const char **err);

/* The one reply buffer for wifi_bridge_call (scan results make it ~900 B): JSON commands and
   console lines both run on the hw worker, one at a time, so they share it. */
extern wb_reply_t g_wifi_bridge_reply;

/* Snapshot of what the pod knows about the bridge (any task; refreshed by the net task every 100 ms). */
void wifi_bridge_get_info(wb_info_t *out);

/* Encoders for the request bodies the commands send. Return the TLV length, 0 if it does not fit. */
size_t wifi_bridge_tlvs_set_wifi(uint8_t *buf, size_t cap, const char *ssid, const char *pass);

#endif /* WIFI_BRIDGE_NET_H */
