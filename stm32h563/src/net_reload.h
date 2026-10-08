#ifndef NET_RELOAD_H
#define NET_RELOAD_H

/*
 * net_reload — reload a link only after the reply that changed its settings has left.
 *
 * A settings command (cloud_proxy set/clear, cloud_ca clear, cloud_set, cloud_clear, wifi_set,
 * wifi_clear) or a company-CA install has to reconnect the cloud link (or restart Wi-Fi). When
 * the command came in over that same link, reloading it inside the handler dropped the link
 * before the reply was sent: the change was applied, but the caller got "device is not
 * reachable (offline)". So the handler persists the change and only REQUESTS the reload; the
 * net task carries it out once the reply is out.
 *
 *   request  (worker, inside the handler): latch what to reload and when it was asked for.
 *   commit   (worker, after the item that asked has finished and its reply is queued).
 *   take     (net task, every poll): due once committed, NET_RELOAD_SETTLE_MS have passed and
 *            the cloud link has nothing left to send; or NET_RELOAD_MAX_WAIT_MS after the
 *            request whatever the link says, so a stuck link (or a request nobody commits)
 *            still reloads.
 *
 * A caller whose reply goes out on another link (USB, LAN) just sees the reload a few
 * milliseconds later. Times are milliseconds from a caller-supplied clock, so this file builds
 * on the host. Single writer per side: the worker requests and commits, the net task takes.
 */

#include <stdbool.h>
#include <stdint.h>

#define NET_RELOAD_CLOUD  0x01u   /* cloud_client_reload() */
#define NET_RELOAD_WIFI   0x02u   /* esp_wifi_ctrl_reload() */

#define NET_RELOAD_SETTLE_MS    20u
#define NET_RELOAD_MAX_WAIT_MS  1000u

void net_reload_request(uint32_t what, uint32_t now_ms);
void net_reload_commit(uint32_t now_ms);

/* Net task: the reload bits that are due now (0 = nothing yet). Returned bits are consumed.
   `tx_busy`: the cloud link still has a reply (or other queued output) to send. */
uint32_t net_reload_take(uint32_t now_ms, bool tx_busy);

/* Anything requested and not yet taken? */
bool net_reload_pending(void);

#endif /* NET_RELOAD_H */
