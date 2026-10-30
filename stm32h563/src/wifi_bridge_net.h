#ifndef WIFI_BRIDGE_NET_H
#define WIFI_BRIDGE_NET_H
/*
 * wifi_bridge_net — runs the Wi-Fi bridge client (wifi_bridge.c) on the net task and gives the
 * hw worker (JSON commands, console lines) a blocking request API.
 *
 * Net task: wifi_bridge_net_init after the Ethernet netif is added, wifi_bridge_net_poll from
 * net_poll, and the lwIP LWIP_HOOK_UNKNOWN_ETH_PROTOCOL hook (config/lwipopts.h) feeds
 * wifi_bridge_eth_input. Only built with BENCHPOD_WIFI_BRIDGE (see the Makefile).
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

/* Any task: built-in Wi-Fi is configured, so it wins over a bridge: the pod takes no address on
   the Ethernet port and wifi_set / wifi_clear / wifi_status / wifi_scan go to the built-in Wi-Fi.
   The control channel stays up, so `bridge` and bridge updates still work. */
bool wifi_bridge_ignored(void);
/* Any task: a bridge is plugged in and not ignored: it is the pod's Wi-Fi. */
bool wifi_bridge_in_use(void);

/* Net task: true once each time the bridge's Wi-Fi comes up (state "connected"). The Ethernet
   DHCP uses it to send DISCOVER right away instead of waiting out lwIP's backoff, whose
   earlier tries the bridge dropped while it was still joining. */
bool wifi_bridge_take_joined(void);

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

/* SET_WIFI with the passphrase sealed under the pairing key (never sent in the clear). Same
   contract as wifi_bridge_call; validates the SSID and passphrase first. */
int wifi_bridge_set_wifi(const char *ssid, const char *pass, wb_reply_t *out, const char **err);

/* Bridge firmware updates (wifi_bridge_ota.c) on the hw worker: one step per call, from its loop. */
void wifi_bridge_ota_poll(void);
bool wifi_bridge_ota_busy(void);          /* an update holds the image bus (bus_busy_reason) */
const char *wifi_bridge_ota_status(void);
void wifi_bridge_ota_force(void);         /* `bridge update`: now, whatever the versions */
/* Slot "bridge" holds an image with its manifest; *version = the manifest's. */
bool wifi_bridge_image(uint32_t *version);

#endif /* WIFI_BRIDGE_NET_H */
