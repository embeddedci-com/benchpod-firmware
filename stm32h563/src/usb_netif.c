/*
 * usb_netif.c — lwIP netif over the USB CDC-NCM link, plus its DHCP server.
 *
 * Mirrors esp_netif.c: frames cross to the usb task through usb_device.c's slots.
 * The link is a point-to-point /30 with a fixed address (usb_dhcpd_subnet), so the
 * pod needs no DHCP client here; it serves the host its one address instead.
 * Net task only (NO_SYS lwIP).
 */
#include "usb_netif.h"
#include "usb_device.h"
#include "usb_dhcpd.h"
#include "board_uid.h"

#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/etharp.h"
#include "lwip/ip4_addr.h"
#include "netif/ethernet.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include <string.h>

#define USB_NETIF_MTU      1500
#define DHCPD_LEASE_S      3600
#define TX_WAIT_MS         5      /* a full TX queue drains at ~1 frame/ms on full speed */

static usb_dhcpd_cfg_t  s_dhcp;
static struct udp_pcb  *s_dhcp_pcb;
static char             s_pod_ip[16], s_host_ip[16];

static err_t usb_netif_linkoutput(struct netif *netif, struct pbuf *p)
{
    (void)netif;
    if (p->tot_len > USB_NET_FRAME_MAX) return ERR_IF;
    if (!usb_net_link_up()) return ERR_IF;
    /* Waiting here holds up the net task for a few ms at most, but a dropped TCP
       segment costs a retransmission timeout (200 ms+). */
    uint8_t *slot = usb_net_tx_slot(TX_WAIT_MS);
    if (!slot) return ERR_MEM;
    u16_t n = pbuf_copy_partial(p, slot, p->tot_len, 0);
    usb_net_tx_commit(n);
    return ERR_OK;
}

/* ---- DHCP server (bound to this netif only, so it never answers on the LAN) ------------- */
static void dhcpd_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                       const ip_addr_t *addr, u16_t port)
{
    struct netif *netif = (struct netif *)arg;
    static uint8_t req[576], rep[USB_DHCPD_MAX_MSG];
    (void)addr; (void)port;
    u16_t n = pbuf_copy_partial(p, req, sizeof(req), 0);
    pbuf_free(p);
    size_t len = usb_dhcpd_reply(&s_dhcp, req, n, rep, sizeof(rep));
    if (len == 0) return;
    struct pbuf *out = pbuf_alloc(PBUF_TRANSPORT, (u16_t)len, PBUF_RAM);
    if (!out) return;
    memcpy(out->payload, rep, len);
    /* The host has no address yet: broadcast, as RFC 2131 allows. */
    udp_sendto_if(pcb, out, IP4_ADDR_BROADCAST, 68, netif);
    pbuf_free(out);
}

static void dhcpd_start(struct netif *netif)
{
    s_dhcp_pcb = udp_new();
    if (!s_dhcp_pcb) return;
    udp_bind_netif(s_dhcp_pcb, netif);
    if (udp_bind(s_dhcp_pcb, IP4_ADDR_ANY, 67) != ERR_OK) {
        udp_remove(s_dhcp_pcb);
        s_dhcp_pcb = NULL;
        return;
    }
    udp_recv(s_dhcp_pcb, dhcpd_recv, netif);
}

/* ---- netif ------------------------------------------------------------------------------ */
static void ip_str(char *out, uint32_t ip)
{
    snprintf(out, 16, "%u.%u.%u.%u", (unsigned)(ip >> 24), (unsigned)((ip >> 16) & 0xFF),
             (unsigned)((ip >> 8) & 0xFF), (unsigned)(ip & 0xFF));
}

err_t usb_netif_init(struct netif *netif)
{
#if LWIP_NETIF_HOSTNAME
    netif->hostname = "benchpod-usb";
#endif
    netif->name[0] = 'u';
    netif->name[1] = 's';
    netif->output     = etharp_output;
    netif->linkoutput = usb_netif_linkoutput;
    netif->mtu        = USB_NETIF_MTU;
    netif->hwaddr_len = 6;
    usb_net_dev_mac(netif->hwaddr);
    netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_IGMP;
    /* No checksum hardware on this path. */
    NETIF_SET_CHECKSUM_CTRL(netif, NETIF_CHECKSUM_ENABLE_ALL);

    uint32_t uid[3];
    board_uid_words(uid);
    usb_dhcpd_subnet(uid, &s_dhcp.server_ip, &s_dhcp.client_ip, &s_dhcp.netmask);
    s_dhcp.lease_s = DHCPD_LEASE_S;
    ip_str(s_pod_ip, s_dhcp.server_ip);
    ip_str(s_host_ip, s_dhcp.client_ip);

    ip4_addr_t ip, mask, gw;
    ip4_addr_set_u32(&ip,   lwip_htonl(s_dhcp.server_ip));
    ip4_addr_set_u32(&mask, lwip_htonl(s_dhcp.netmask));
    ip4_addr_set_zero(&gw);
    netif_set_addr(netif, &ip, &mask, &gw);
    return ERR_OK;
}

void usb_netif_poll(struct netif *netif)
{
    if (!s_dhcp_pcb && netif_is_up(netif)) dhcpd_start(netif);

    bool up = usb_net_link_up();
    if (up && !netif_is_link_up(netif))  netif_set_link_up(netif);
    if (!up && netif_is_link_up(netif))  netif_set_link_down(netif);

    const uint8_t *frame;
    uint16_t len;
    while (usb_net_rx_peek(&frame, &len)) {
        struct pbuf *p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);
        if (p) {
            pbuf_take(p, frame, len);
            if (netif->input(p, netif) != ERR_OK) pbuf_free(p);
        }
        usb_net_rx_release();   /* dropped when out of pbufs */
    }
}

const char *usb_netif_pod_ip(void)  { return s_pod_ip; }
const char *usb_netif_host_ip(void) { return s_host_ip; }
