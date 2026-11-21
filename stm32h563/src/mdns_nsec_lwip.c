/*
 * mdns_nsec_lwip.c — puts mdns_nsec_reply() in front of lwIP's mDNS responder.
 *
 * lwIP 2.2's responder has no NSEC support ("Things left to implement" at the top of
 * lib/stm32_mw_lwip/src/apps/mdns/mdns.c), so instead of patching the submodule this wraps the
 * receive callback of its UDP pcb: each packet first goes through mdns_nsec_reply() (which only
 * answers AAAA questions for the host name) and then, unchanged, to lwIP's own handler, which
 * still answers A, PTR, SRV and TXT. It applies to every netif lwIP's mDNS is registered on
 * (Ethernet, Wi-Fi, USB), with that netif's host name, and to none it was removed from.
 *
 * Call mdns_nsec_install() once, right after mdns_resp_init(). Runs in the lwIP (net) task.
 */
#include "mdns_nsec_lwip.h"
#include "mdns_nsec.h"

#include "lwip/apps/mdns.h"
#include "lwip/apps/mdns_priv.h"
#include "lwip/ip.h"
#include "lwip/pbuf.h"
#include "lwip/prot/iana.h"
#include "lwip/udp.h"

static udp_recv_fn s_lwip_recv;
static void       *s_lwip_arg;

/* Enough for the header and the questions of any query a resolver sends for one name. Only used
   when the pbuf is chained; a single pbuf is parsed in place. */
#define NSEC_PARSE_MAX 256u
/* A reply for a host label of up to 25 characters (mdns_nsec.h: 53 + 3 * len bytes, legacy);
   the pod's names are 15 ("benchpod-a1b2c3"). */
#define NSEC_REPLY_BUF 128u

static void nsec_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr,
                      u16_t port)
{
    struct netif *nif = ip_current_input_netif();
    struct mdns_host *host = nif ? netif_mdns_data(nif) : NULL;
    LWIP_UNUSED_ARG(arg);

    /* Same gates as lwIP's mdns_recv / mdns_handle_question: a netif with mDNS on and its name
       claimed (announcing or done), IPv4, and a unicast query only from the netif's own subnet. */
    if (host && (host->state == MDNS_STATE_ANNOUNCING || host->state == MDNS_STATE_COMPLETE) &&
        IP_IS_V4(addr) && !ip4_addr_isany_val(*netif_ip4_addr(nif)) &&
        (ip4_addr_ismulticast(ip4_current_dest_addr()) ||
         ip4_addr_net_eq(ip4_current_src_addr(), netif_ip4_addr(nif), netif_ip4_netmask(nif)))) {
        uint8_t buf[NSEC_PARSE_MAX];
        u16_t n = p->tot_len < NSEC_PARSE_MAX ? p->tot_len : (u16_t)NSEC_PARSE_MAX;
        const uint8_t *msg = pbuf_get_contiguous(p, buf, sizeof(buf), n, 0);
        uint8_t reply[NSEC_REPLY_BUF];
        bool legacy = port != LWIP_IANA_PORT_MDNS;
        size_t len = msg ? mdns_nsec_reply(msg, n, host->name, legacy, reply, sizeof(reply)) : 0;
        if (len) {
            struct pbuf *r = pbuf_alloc(PBUF_TRANSPORT, (u16_t)len, PBUF_RAM);
            if (r) {
                ip_addr_t group;
                IP_ADDR4(&group, 224, 0, 0, 251);
                pbuf_take(r, reply, (u16_t)len);
                udp_sendto_if(pcb, r, legacy ? addr : &group,
                              legacy ? port : LWIP_IANA_PORT_MDNS, nif);
                pbuf_free(r);
            }
        }
    }
    s_lwip_recv(s_lwip_arg, pcb, p, addr, port);   /* lwIP answers the rest and frees p */
}

void mdns_nsec_install(void)
{
    struct udp_pcb *pcb = get_mdns_pcb();
    if (!pcb || !pcb->recv || pcb->recv == nsec_recv) return;
    s_lwip_recv = pcb->recv;
    s_lwip_arg  = pcb->recv_arg;
    udp_recv(pcb, nsec_recv, NULL);
}
