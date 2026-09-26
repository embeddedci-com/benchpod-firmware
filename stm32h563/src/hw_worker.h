#ifndef HW_WORKER_H
#define HW_WORKER_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/*
 * hw_worker — the single task that owns ALL instrument-hardware access and all
 * command_handler state.
 *
 * lwIP is NO_SYS=1 (single-threaded on the net task), so command execution can't
 * live there without long operations (flash, captures) starving the network.
 * Instead this worker task drains a queue of inbound work — TCP/tunnel bytes,
 * console lines, cloud command.requests, and connection-closed / tunnel-reset
 * events — and runs command_handler on them, plus the async capture poll and the
 * signal-engine / target-power periodic polls.  Because it is the ONLY task that
 * touches the instrument buses and command state, there is no cross-task hardware
 * race and no dispatch lock is needed.
 *
 * Replies never call lwIP directly: at_send_data() writes them into per-connection
 * conn_tx rings (conn_tx.c) that the net task drains into lwIP.  A cloud command's
 * reply is handed back through a one-slot channel the net task frames as a
 * command.response.
 *
 * All hw_worker_submit_* functions are called from the net or console tasks
 * (producers); the worker is the single consumer.
 */

/* Work-queue sizing (exposed so net_server can static_assert the invariant that a
   full lwIP receive window fits the queue — see hw_worker.c / srv_recv). */
#define HW_WORK_DATA_MAX     1536
#define HW_WORK_QUEUE_DEPTH  16

/* Create the work queue and start the worker task.  Call before the scheduler
   starts, after command_handler / signal_engine are initialised. */
void hw_worker_init(void);

/* Inbound command bytes for a real TCP conn or a cloud tunnel pseudo-conn.
   Non-blocking (net-task safe): returns false if the queue is full so the caller
   can apply transport backpressure (e.g. return ERR_MEM to lwIP). */
bool hw_worker_submit_bytes(int conn_id, const uint8_t *buf, size_t len);

/* True iff `len` bytes fit the work queue right now (as ceil(len/HW_WORK_DATA_MAX)
   items).  Lets a producer accept an inbound buffer all-or-nothing so a bulk
   upload is never partially taken and truncated when the queue briefly fills. */
bool hw_worker_can_accept(size_t len);

/* A TCP connection closed (net task): the worker runs command_handler_conn_closed
   so gate/stream state is released on the task that owns it. */
void hw_worker_submit_closed(int conn_id);

/* A cloud tunnel opened/closed (net task): the worker runs
   command_handler_tunnel_reset for that tunnel pseudo-conn. */
void hw_worker_submit_tunnel_reset(int conn_id);
/* A close/reset for this id is still on its way to the worker (queued or parked because the
   queue was full).  The net task must not reuse the slot, and drops its leftover output. */
bool hw_worker_conn_busy(int conn_id);

/* A cloud command.request (net task).  The reply is delivered back via
   hw_worker_take_cloud_reply().  Returns false if a cloud command is already in
   flight or the queue is full (the server will retry). */
bool hw_worker_submit_cloud(const char *req_id, const char *command_json);

/* A console command line (console task).  Blocks briefly if the queue is momentarily
   full; returns false only if it still could not be queued. */
bool hw_worker_submit_console(const char *line);

/* Net task: if the worker has finished a cloud command, copy its request_id and
   reply out and return true (one-shot).  The net task frames it as a
   command.response.  Returns false when nothing is ready. */
bool hw_worker_take_cloud_reply(char *req_id, size_t req_id_cap,
                                char *reply, size_t reply_cap, size_t *reply_len);

/* ---- OTA (firmware update) — driven from the WS ota.* frames (or LAN) -------
   The bytes are staged into PSRAM + verified on the worker (it owns the PSRAM
   bus).  begin/data/end/abort/commit map to ota.c.  All are called from the net
   task (WS) or console; non-blocking submits. */
bool hw_worker_submit_ota_begin(uint32_t size, const char *sha256_hex);
bool hw_worker_submit_ota_data(uint32_t offset, const uint8_t *buf, size_t len);
bool hw_worker_submit_ota_end(void);
bool hw_worker_submit_ota_abort(void);
bool hw_worker_submit_ota_commit(void);

/* Net task (Wi-Fi control): flash the embedded esp-hosted image onto the ESP32-C3 (~140 s),
   then report through esp_wifi_ctrl_flash_done().  False if it could not be queued. */
bool hw_worker_submit_esp_flash(void);

#endif /* HW_WORKER_H */
