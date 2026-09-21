/*
 * esp_netif.c — LwIP Wi-Fi netif bridging to the ESP32-C3 over esp-hosted SPI.
 *
 * Mirrors ethernetif.c but the "wire" is the SPI transport (esp_hosted_spi):
 * link output frames go out on ESP_STA_IF, and received STA frames arrive via
 * the transport's data callback and are injected into LwIP. Runs in the net
 * task (NO_SYS=1), so the static TX scratch buffer is reentrancy-safe.
 */
#include "esp_netif.h"
#include "esp_hosted_spi.h"
#include "esp_hosted_frame.h"

#include "lwip/pbuf.h"
#include "lwip/etharp.h"
#include "netif/ethernet.h"

#include <string.h>

#define ESP_NETIF_MTU   1500

static struct netif *s_netif;
/* Provisional locally-administered MAC until esp_wifi_ctrl fetches the real
   ESP32 STA MAC over RPC (esp_netif_set_hwaddr). */
static uint8_t s_mac[6] = { 0x02, 0x00, 0x00, 0x57, 0x49, 0x46 };

/* TX flatten buffer (one ethernet frame; net-task single-threaded). */
static uint8_t s_txbuf[ESP_HOSTED_SPI_BUF_SIZE];

/* LwIP link output: flatten the pbuf chain and hand it to the transport. */
static err_t esp_netif_linkoutput(struct netif *netif, struct pbuf *p) {
    (void)netif;
    if (p->tot_len > sizeof(s_txbuf)) return ERR_IF;
    u16_t n = pbuf_copy_partial(p, s_txbuf, p->tot_len, 0);
    if (n != p->tot_len) return ERR_IF;
    return (esp_hosted_spi_send(ESP_STA_IF, 0, s_txbuf, n) == 0) ? ERR_OK : ERR_IF;
}

/* Transport data callback: inject a received STA frame into LwIP. */
static void esp_netif_data_cb(const esp_hosted_rx_t *rx) {
    if (!s_netif || rx->payload_len == 0) return;
    struct pbuf *p = pbuf_alloc(PBUF_RAW, rx->payload_len, PBUF_POOL);
    if (!p) return;                                   /* out of pbufs — drop */
    if (pbuf_take(p, rx->payload, rx->payload_len) != ERR_OK) { pbuf_free(p); return; }
    if (s_netif->input(p, s_netif) != ERR_OK) pbuf_free(p);
}

err_t esp_netif_init(struct netif *netif) {
    s_netif = netif;
#if LWIP_NETIF_HOSTNAME
    netif->hostname = "benchpod-wl";
#endif
    netif->name[0] = 'w';
    netif->name[1] = 'l';
    netif->output     = etharp_output;
    netif->linkoutput = esp_netif_linkoutput;
    netif->mtu        = ESP_NETIF_MTU;
    netif->hwaddr_len = 6;
    memcpy(netif->hwaddr, s_mac, 6);
    /* NETIF_FLAG_IGMP: join the mDNS multicast group on the Wi-Fi path too.
       esp-hosted passes multicast frames up by default, so no MAC filter tweak
       is needed here (unlike the wired ETH MAC). */
    netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_IGMP;   /* LINK set on connect */
    /* No checksum hardware on the SPI/ESP32 path — compute IP/UDP/TCP/ICMP in
       software. Without this the netif emits a zero IP checksum and the AP drops
       every frame (DHCP DISCOVER unanswered). The eth netif offloads to its MAC
       and disables these (see net_server.c); hence per-netif control. */
    NETIF_SET_CHECKSUM_CTRL(netif, NETIF_CHECKSUM_ENABLE_ALL);

    esp_hosted_spi_set_data_cb(esp_netif_data_cb);
    return ERR_OK;
}

void esp_netif_set_hwaddr(const uint8_t mac[6]) {
    memcpy(s_mac, mac, 6);
    if (s_netif) memcpy(s_netif->hwaddr, mac, 6);
}

void esp_netif_set_link_up(bool up) {
    if (!s_netif) return;
    if (up) {
        netif_set_up(s_netif);        /* admin up — REQUIRED before lwIP will transmit
                                         (DHCP DISCOVER); link-up alone is not enough. */
        netif_set_link_up(s_netif);
    } else {
        netif_set_link_down(s_netif);
    }
}
