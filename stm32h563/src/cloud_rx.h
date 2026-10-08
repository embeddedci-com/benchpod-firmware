#ifndef CLOUD_RX_H
#define CLOUD_RX_H

/*
 * cloud_rx — the cloud client's inbound (decrypted) byte accumulator: append, consume, and the
 * accept-or-refuse decision cl_recv_cb makes for each segment.  The storage stays in
 * cloud_client.c (s_rx, BP_CLOUD_RX_ACCUM); these are the operations on it, pure so the host test
 * (test_cloud_rx.c) runs the code the firmware runs.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Append n bytes to buf[0..*len) of capacity cap.  False (nothing written) when they do not
   fit: dropping some would slice through a WS frame, so the caller flags an overflow and
   reconnects instead. */
bool cloud_rx_append(uint8_t *buf, size_t cap, size_t *len, const uint8_t *data, size_t n);

/* Drop the first n bytes (all of them when n >= *len), shifting the rest down. */
void cloud_rx_consume(uint8_t *buf, size_t *len, size_t n);

/* Should a segment of `incoming` bytes be refused for now (the TLS layer keeps it and the TCP
   window stays shut until the net task drains and asks again)?  Only when it does not fit AND
   the buffer holds something to drain: an empty buffer that still cannot hold it takes it, and
   the append then overflows, since waiting would never make room. */
bool cloud_rx_refuse(size_t cap, size_t len, size_t incoming);

#endif /* CLOUD_RX_H */
