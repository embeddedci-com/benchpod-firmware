#ifndef CONN_TX_H
#define CONN_TX_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/*
 * conn_tx — per-connection outbound byte rings that decouple command execution
 * (the hw-worker task) from lwIP transmission (the net task).
 *
 * lwIP runs NO_SYS=1 (single-threaded): tcp_write / altcp_write may ONLY be
 * called from the net task.  The worker task, which now owns all command
 * execution, therefore cannot transmit directly — instead at_send_data() writes a
 * reply into the ring for that connection (single producer = worker) and the net
 * task drains it into lwIP (single consumer = net).  Each ring is SPSC, so the
 * head/tail indices need no lock (producer only advances head, consumer only
 * advances tail; both are volatile).
 *
 * Rings exist for the real TCP connections (0..CH_MAX_CONN-1) and the cloud
 * byte-tunnels (their pseudo-conn ids); other pseudo-conns (console, cloud
 * command) route elsewhere and have no ring.  bulk_pump paces its output against
 * conn_tx_free(), so a ring is never overrun in normal operation.
 */

/* Map a conn_id to a ring slot, or -1 if this conn has no ring (console / cloud
   command / out of range). */
int conn_tx_slot(int conn_id);

/* Producer (worker): append up to len bytes; returns the number actually queued
   (< len if the ring filled).  Returns 0 for a conn with no ring. */
size_t conn_tx_write(int conn_id, const uint8_t *buf, size_t len);

/* Producer/consumer: free space / queued bytes for this conn's ring. */
size_t conn_tx_free(int conn_id);
size_t conn_tx_used(int conn_id);

/* Consumer (net): peek a contiguous run of queued bytes without removing them
   (returns a pointer into the ring + its length), then advance past `n` of them
   once written to lwIP.  Peek returns 0 when empty or no ring. */
size_t conn_tx_peek(int conn_id, const uint8_t **out);
void   conn_tx_advance(int conn_id, size_t n);

/* Drop all queued bytes for this conn (on close / reset). */
void conn_tx_reset(int conn_id);

#endif /* CONN_TX_H */
