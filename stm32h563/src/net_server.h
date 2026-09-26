#ifndef NET_SERVER_H
#define NET_SERVER_H

#include <stdbool.h>
#include <stdint.h>

/* Ethernet + LwIP (NO_SYS=1) TCP command server.
 *   net_init()  — lwIP init, add the RMII netif (DHCP), start the server.
 *   net_poll()  — drive the stack: drain RX, run timeouts, link + DHCP checks.
 *                 Call repeatedly from the net task (the NO_SYS poll loop).
 *   net_ip_str()— current IPv4 address as a string ("0.0.0.0" until assigned).
 *   net_mac_str()— wired MAC as "aa:bb:cc:dd:ee:ff".
 */
void        net_init(void);
void        net_poll(void);
const char *net_ip_str(void);
const char *net_mac_str(void);
/* Wi-Fi STA IPv4 as a string, "(none)" until the Wi-Fi netif has a DHCP lease.
   Unlike wifi_get_ip() (Ethernet-preferred), this is the Wi-Fi interface only —
   used by the `wifi-show` console command. */
const char *wifi_sta_ip(void);

/* Bring-up diagnostic (console `wifi-static`): pin a static IP on the Wi-Fi
   netif and disable DHCP on it, so a peer can ping the pod to test unicast RX. */
void net_wifi_static(const char *ip, const char *mask, const char *gw);

/* Run fn(arg) on the net task (which owns lwIP, the cloud client and the Wi-Fi transport) and
   wait up to timeout_ms for it; runs it directly when called on the net task.  false = the net
   task did not run it in time (not polling: safe mode with networking off). */
typedef void (*net_call_fn_t)(void *arg);
bool net_call_sync(net_call_fn_t fn, void *arg, uint32_t timeout_ms);
/* The worker-side entry points that must run on the net task (via net_call_sync). */
void net_cloud_reload(void);                 /* cloud_client_reload() */
void net_wifi_reload(void);                  /* esp_wifi_ctrl_reload() */
/* true: pause Wi-Fi control and stop the ESP SPI transport so the worker may drive the C3's
   EN/BOOT and ROM loader; false: resume and restart the Wi-Fi join. */
void net_wifi_hold_for_flash(bool hold);

/* Manual wired-interface control (console `eth stop|start|restart` / JSON
   {"cmd":"eth",...}). Safe to call from any task — they only latch a request that
   net_poll() applies on the net task. `stop` halts the MAC and holds it down;
   `start` re-establishes it (PHY reset + DHCP); `restart` = stop-then-start in one.
   The same PHY-reset recovery also runs automatically when DHCP stays stuck. */
void net_eth_stop(void);
void net_eth_start(void);
void net_eth_restart(void);
/* Force the wired link to 10 or 100 Mbit (mbit), or back to autonegotiation (mbit = 0).
   A debug aid for a suspect link: 10BASE-T is far more tolerant of a degraded analog
   path, so clean at 10M and lossy at 100M accuses the magnetics/RJ45/PHY clock. */
void net_eth_force_speed(int mbit, int full);
/* Measure the PHY's RMII reference clock (see ethernetif.c). Latches a request the net task
   runs; capture net_eth_refclk_seq() first, then poll net_eth_refclk_result(seq, &hz) until
   it returns true. hz = 0 means the PHY drove no clock at all. Costs the link ~200 ms. */
void     net_eth_refclk_measure(void);
uint32_t net_eth_refclk_seq(void);
bool     net_eth_refclk_result(uint32_t since_seq, uint32_t *hz_out);
/* PHY near-end loopback test at 10 or 100 Mbit with n frames (see ethernetif.c): tells a
   fault on the RMII/digital side from one in the analog front end. Same request/poll
   pattern as refclk. Takes the link down for up to ~n x 5 ms plus a PHY restart. */
#include "ethernetif.h"
void     net_eth_loopback(int mbit, uint32_t n);
uint32_t net_eth_loopback_seq(void);
bool     net_eth_loopback_result(uint32_t since_seq, eth_loopback_result_t *out);
/* The latest wired-link diagnostics snapshot (refreshed every 2 s on the net task). */
#include "eth_diag.h"
void net_eth_diag(eth_diag_t *out);

#endif /* NET_SERVER_H */
