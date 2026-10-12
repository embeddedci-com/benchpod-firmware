#ifndef USB_NETIF_H
#define USB_NETIF_H

#include <stdbool.h>
#include "lwip/netif.h"

/* lwIP netif over the USB CDC-NCM link (usb_device.c). Net task only. */

/* netif_add init function. Gives the netif its fixed /30 address (usb_dhcpd_subnet). */
err_t usb_netif_init(struct netif *netif);

/* Each net_poll: received frames into lwIP, link state, the DHCP server. */
void usb_netif_poll(struct netif *netif);

/* "10.237.x.y" of the pod and the host on the USB link. */
const char *usb_netif_pod_ip(void);
const char *usb_netif_host_ip(void);

#endif /* USB_NETIF_H */
