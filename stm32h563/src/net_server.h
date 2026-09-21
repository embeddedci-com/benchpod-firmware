#ifndef NET_SERVER_H
#define NET_SERVER_H

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

/* Manual wired-interface control (console `eth stop|start|restart` / JSON
   {"cmd":"eth",...}). Safe to call from any task — they only latch a request that
   net_poll() applies on the net task. `stop` halts the MAC and holds it down;
   `start` re-establishes it (PHY reset + DHCP); `restart` = stop-then-start in one.
   The same PHY-reset recovery also runs automatically when DHCP stays stuck. */
void net_eth_stop(void);
void net_eth_start(void);
void net_eth_restart(void);

#endif /* NET_SERVER_H */
