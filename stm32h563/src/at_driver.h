#ifndef AT_DRIVER_H
#define AT_DRIVER_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* TCP transmit seam used by command_handler.c / scpi_server.c.  On the RP2350
   these went to the ESP32 AT modem; here they map onto the LwIP TCP command
   server (see net_server.c) by connection id.  Return 0 on success, <0 on
   error (which makes the caller close the connection). */
int at_send_data(int conn_id, const uint8_t *buf, size_t len);
int at_close_connection(int conn_id);
int at_set_tcp_nodelay(int conn_id, bool enable);

/* Free space (bytes) currently available in the connection's TCP send buffer.
   A bulk sender uses this to pace itself against TCP flow control instead of
   overrunning the send buffer (which returns ERR_MEM and would otherwise abort
   the connection).  Returns a large value for the local console pseudo-conn
   (its UART/USB sink is blocking, not TCP-flow-controlled), 0 for a dead/unknown
   connection. */
size_t at_send_avail(int conn_id);

#endif /* AT_DRIVER_H */
