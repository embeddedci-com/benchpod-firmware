#ifndef USB_DHCPD_H
#define USB_DHCPD_H

#include <stddef.h>
#include <stdint.h>

/* ---- DHCP server for the USB network link (host-testable) ------------------------------
 * The USB link is a point-to-point /30: the pod is .1, the host is .2. The server
 * hands the host that one address and nothing else: no router and no DNS, so the
 * host keeps its own default route and the pod never takes over its internet.
 * Addresses are IPv4 in host byte order. */
typedef struct {
    uint32_t server_ip;   /* the pod */
    uint32_t client_ip;   /* the host */
    uint32_t netmask;
    uint32_t lease_s;
} usb_dhcpd_cfg_t;

#define USB_DHCPD_MAX_MSG  300   /* fixed header (236) + cookie + our options, padded */

/* Builds the reply to one DHCP request in `req`: OFFER for a DISCOVER, ACK for a
   REQUEST of client_ip, NAK for a REQUEST of anything else. Returns its length, or 0
   when there is nothing to send (not a BOOTREQUEST, no message type, another type). */
size_t usb_dhcpd_reply(const usb_dhcpd_cfg_t *cfg, const uint8_t *req, size_t req_len,
                       uint8_t *out, size_t out_cap);

/* The pod's /30 on the USB link, from its UID: 10.237.X.Y, so two pods on one host get
   different subnets (1 in 16384 collide) and home LANs (192.168.x, 10.0.x) are unlikely
   to clash. *pod = base + 1, *host = base + 2, mask 255.255.255.252. */
void usb_dhcpd_subnet(const uint32_t uid[3], uint32_t *pod, uint32_t *host, uint32_t *mask);

#endif /* USB_DHCPD_H */
