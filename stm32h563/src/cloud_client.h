#ifndef CLOUD_CLIENT_H
#define CLOUD_CLIENT_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* ---- Direct cloud WebSocket client (STM32H5 / LwIP altcp_tls) ---------------
 *
 * Opens an OUTBOUND (TLS) WebSocket from the bench pod to embeddedci-server and
 * keeps it alive across reboots, so the server can push benchpod command.request
 * frames straight to the device (no host/CLI bridge). Authentication reuses the
 * device-key challenge: the firmware fetches a single-use nonce over HTTPS, signs
 * it with its Ed25519 key, and connects to /api/benchpod/ws with the signature.
 *
 * This is the LwIP port of the RP2350's cloud_client (which drove the ESP-AT
 * modem's TLS).  Here the outbound socket is a LwIP altcp connection wrapped in
 * altcp_tls/mbedTLS — the client owns its own pcb and feeds itself via the altcp
 * recv callback, so (unlike the RP2350) there is no shared modem-link routing.
 *
 * Driven cooperatively from the net task via cloud_client_poll(). Config comes
 * from cloud_config (provisioned by `benchpod register`).
 * ---------------------------------------------------------------------------*/

/* Load persisted config at boot. Call once after networking is set up. */
void cloud_client_init(void);

/* Advance the connection state machine. Call every net-loop pass (after the
   lwIP poll). No-op until the link is READY and a config is present + enabled. */
void cloud_client_poll(void);

/* Re-read cloud_config and reset the state machine — call after cloud_set /
   cloud_clear so a new endpoint takes effect (or the client drops to disabled). */
void cloud_client_reload(void);

/* Human-readable current state (for the cloud_status command). */
const char *cloud_client_state_str(void);

/* Why the last connection attempt failed ("" when none, or once connected). Copies into out
   (always NUL-terminated); safe to call from any task. */
void cloud_client_last_error(char *out, size_t n);

/* Emit device→client bytes for one cloud byte-tunnel as one or more tunnel.data WS frames. conn_id is
   the tunnel pseudo-conn (CH_CLOUD_TUNNEL_CONN + slot); the slot's tunnel id is stamped on each frame
   so the server routes it to the right client. Called by at_send_data() when the command handler
   writes to a tunnel conn. No-op when that slot has no open tunnel. */
void cloud_client_tunnel_out(int conn_id, const uint8_t *buf, size_t len);

/* Raw bytes that may be written to this tunnel pseudo-conn right now without overrunning the TLS/WS
   send buffer (accounts for the base64 + envelope + WS-frame expansion in cloud_client_tunnel_out).
   Returns 0 when the tunnel/link isn't ready. Used by at_send_avail() to pace bulk capture streams so
   they don't drop trailing frames. */
size_t cloud_client_tunnel_avail(int conn_id);

/* Wedge diagnostics: print a snapshot of the cloud link's flow-control state (ms since last inbound,
   RX accumulator level/peak/overflow, and the TLS/TCP send buffer) tagged with `when`.  Lets a
   silent upload stall be attributed to the pod's send path vs. the server/Cloudflare send path.
   Net-task only (reads the cloud client's state); safe to call from command_handler_poll. */
void cloud_client_log_link_state(const char *when);

/* ota.data frames the net task has taken off the wire this boot (see cloud_client.c). */
uint32_t cloud_client_ota_frames_seen(void);

/* Latch a request for the wedge snapshot above, logged on the next net-task poll.  Safe from ANY
   task (only sets flags); use this from the worker (e.g. a load_bin stall) instead of calling
   cloud_client_log_link_state directly, which must stay net-task-only. */
void cloud_client_request_link_snapshot(const char *tag);

/* Ask the net task to re-announce the capabilities frame on its next poll.  Safe from ANY task
   (only sets a flag).  Call after a runtime FPGA image swap (handle_fpga_image) so the server's
   cached caps track the now-running gateware image instead of the one announced on connect. */
void cloud_client_request_caps_resend(void);

/* Frame + send a command.response for a cloud command.request the hw worker has
   finished executing.  `reply` is the worker's captured reply line
   ({"status":"ok","data":...} or {"status":"error","message":...}); this wraps it
   in the command.response envelope with request_id + device_id.  Called by the net
   task (net_frame_cloud_reply) — command execution itself runs on the worker. */
void cloud_client_send_command_response(const char *request_id,
                                        const char *reply, size_t reply_len);

#endif /* CLOUD_CLIENT_H */
