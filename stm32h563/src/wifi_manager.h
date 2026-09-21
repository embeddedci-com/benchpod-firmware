#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

/* Connectivity-status seam used by command_handler.c / scpi_server.c for the
   `status` command.  On this board it is backed by the wired Ethernet link
   (see net_server.c), not WiFi — names kept for a drop-in port. */

typedef enum {
    WIFI_DISCONNECTED = 0,
    WIFI_CONNECTING,
    WIFI_CONNECTED,
    WIFI_READY,
} wifi_state_t;

wifi_state_t wifi_get_state(void);   /* WIFI_READY when the link/IP is up */
const char  *wifi_get_ip(void);      /* current IPv4 string */
int          wifi_get_rssi(int *out_rssi_dbm);  /* always -1 on Ethernet */

#endif /* WIFI_MANAGER_H */
