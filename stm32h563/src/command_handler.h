#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* Process a newline-terminated JSON command received on conn_id.
   Dispatches to signal_engine functions and sends JSON responses
   back via at_send_data(conn_id, ...). */
void command_handler_process(int conn_id, const uint8_t *json_buf, size_t len);

/* Reserved pseudo-connection id for console-originated JSON commands (the serial
   console "json" mode).  Its responses are routed to stdout by at_send_data()
   instead of the TCP server, so the same JSON handlers serve a serial client. */
#define CH_CONSOLE_CONN 5

/* Reserved pseudo-connection id for cloud-originated JSON commands (a server
   command.request arriving over the outbound WebSocket).  at_send_data() routes
   replies for this id into a capture buffer (see command_handler_dispatch_cloud)
   instead of the TCP server, so the cloud client can wrap them in a
   command.response. */
#define CH_CLOUD_CONN 6

/* Reserved pseudo-connection id for the cloud byte-tunnel (a remote programmatic client driving
   this device through embeddedci-server). Unlike CH_CLOUD_CONN (single-command), tunnel bytes flow
   through command_handler_process()'s full state machine — JSON commands AND the raw SWD/UART modes
   used for flashing and captures — exactly like a local TCP client. at_send_data() routes this id's
   output back out as tunnel.data WS frames (see cloud_client_tunnel_out). */
#define CH_CLOUD_TUNNEL_CONN 7

/* Number of concurrent cloud byte-tunnels the device supports. Each tunnel binds to its own pseudo-
   connection in the range CH_CLOUD_TUNNEL_CONN .. CH_CLOUD_TUNNEL_CONN_LAST, each with independent
   protocol state — so a held UART proxy, an LA capture, AND a DAC waveform replay can run on three
   separate tunnels at the same time (the three independent FPGA engines). The cloud client maps
   tunnel_id -> slot and feeds bytes to the matching conn; line_asm[]/proto[] are sized by
   CH_CLOUD_TUNNEL_CONN_LAST so raising this count grows them automatically. */
#define CH_CLOUD_TUNNEL_CONN_COUNT 3
#define CH_CLOUD_TUNNEL_CONN_LAST  (CH_CLOUD_TUNNEL_CONN + CH_CLOUD_TUNNEL_CONN_COUNT - 1)

/* Reset one cloud-tunnel pseudo-connection's protocol state (disarm any in-flight SWD/UART, clear the
   line buffer). Called by the cloud client on tunnel.open / tunnel.close. conn_id must be in
   CH_CLOUD_TUNNEL_CONN .. CH_CLOUD_TUNNEL_CONN_LAST. */
void command_handler_tunnel_reset(int conn_id);

/* Dispatch one complete JSON command line that arrived on the serial console's
   "json" mode, through the same handlers as the TCP path.  Replies go to stdout
   via at_send_data(CH_CONSOLE_CONN, ...).  `json_line` is a NUL-terminated JSON
   object (no trailing newline required). */
void command_handler_dispatch_console(const char *json_line);

/* Run a single command (the `command` object from a cloud command.request) and
   capture its full reply line ({"status":"ok","data":...} or
   {"status":"error","message":...}) into out.  command_json is a NUL-terminated
   flat JSON object, e.g. {"cmd":"status"}.  Streaming/heavy commands
   (capture/stream/measure/test/load/replay/dap_start/uart_proxy_start) are
   refused with an error reply — only the single-packet command surface is
   carried over the cloud channel.  Returns the number of bytes written to out
   (excluding the NUL), or 0 if the reply did not fit. */
size_t command_handler_dispatch_cloud(const char *command_json, char *out, size_t out_cap);

/* Append bytes to the in-progress cloud reply capture.  Called only by
   at_send_data() for CH_CLOUD_CONN; not for general use. */
void command_handler_cloud_capture_append(const uint8_t *buf, size_t len);

/* Call from main loop: sends pending stream chunks after async ADC DMA
   completes.  Must run in main-loop context (not IRQ) because it calls
   at_send_data which does blocking UART I/O. */
void command_handler_poll(void);

/* Notify the command handler that a TCP connection has closed (or is being
   reclaimed).  Releases the shared ADC buffer if this connection owned an
   in-flight heavy operation (capture/stream/measure/test) and clears any
   partial command buffered for it.  Safe to call for any conn_id. */
void command_handler_conn_closed(int conn_id);

/* Re-sync this module's mirrors of gateware registers after a RECONFIGURATION (image swap,
 * flash-ice40, boot/OTA reflash).  Called by ice40_reflash_image(). */
void command_handler_on_gateware_reconfigured(void);

/* Claim / release the single shared ADC (and its capture path) so the SCPI
   handler's blocking captures cannot collide with an in-flight JSON
   capture/stream/measure/test.  acquire returns false if another connection
   currently owns it. */
bool command_handler_acquire_adc(int conn_id);
void command_handler_release_adc(int conn_id);

/* SCPI DIGital:* entry points, under the same LA pin-ownership rules as the JSON `gpio` / `la`
   commands (the refusal reason is logged on the console).
     dig_output: 0 ok, -1 pin out of range, -2 refused (la voltage unset, or the pin belongs to
                 another function).  The pin becomes a gpio output at that level.
     dig_step:   0 ok, -1 bad arguments, -2 refused (voltage / ownership), -3 a train is running. */
int  command_handler_dig_output(unsigned la, int level);
int  command_handler_dig_step(unsigned la, uint32_t steps, uint32_t delay_us, unsigned dir_la, int dir);
bool command_handler_step_busy(void);
