/*
 * command_handler_transport.c — the per-connection byte stream: line assembly and protocol modes.
 *
 * Every TCP conn and cloud tunnel feeds its raw bytes to command_handler_process. The first
 * non-whitespace byte picks JSON ('{') or SCPI; complete lines go to dispatch_line or the SCPI
 * parser. A command can switch the connection into a raw mode (load_bin, dap_start,
 * uart_proxy_start, speedtest down), whose bytes go to the owning module's receive function until
 * that module hands the connection back. The console and cloud-command pseudo-conns dispatch
 * single lines instead.
 *
 * Runs on the hw worker task, like the rest of command_handler.
 */
#include "command_handler.h"
#include "command_handler_internal.h"
#include "bp_limits.h"      /* BP_CLOUD_CMD_IN_MAX */
#include "scpi_server.h"
#include "lease_local.h"
#include "bp_log.h"
#include "cmd_gate.h"       /* cmd_gate_scpi_line_writes */
#include "lease_gate.h"
#include "pod_policy.h"     /* pod_policy_scpi / pod_policy_https_only */
#include "at_driver.h"      /* at_send_data / at_close_connection */
#include "stm32h5xx_hal.h"  /* HAL_GetTick */

#include <stdio.h>
#include <string.h>

/* ---- Per-connection command reassembly ----
   ESP-AT delivers one +IPD per TCP segment; a JSON command may be split
   across segments, or two commands may share one segment.  We buffer per
   connection and dispatch on each '\n'. */
/* Sized to match the cloud command buffer (BP_CLOUD_CMD_IN_MAX) so a compact closed-loop
   curve LUT uploaded over LAN/serial reassembles in one line, same as over cloud. */
#define LINE_ASM_SIZE ((int)BP_CLOUD_CMD_IN_MAX)
typedef struct {
    char   buf[LINE_ASM_SIZE];
    size_t len;
    bool   discarding;   /* dropping an over-long line until its newline */
} line_asm_t;
/* Sized to cover the TCP conns (0..CH_MAX_CONN-1) plus the console (CH_CONSOLE_CONN), cloud command
   (CH_CLOUD_CONN) and the cloud tunnels (CH_CLOUD_TUNNEL_CONN..CH_CLOUD_TUNNEL_CONN_LAST) pseudo-
   connections, so a JSON handler that touches proto[]/line_asm[] for any of them stays in bounds. The
   tunnel conns, unlike the others, run the full buffered state machine in command_handler_process(). */
_Static_assert(CH_CONSOLE_CONN >= CH_MAX_CONN, "console conn id must sit above the TCP conns");
_Static_assert(CH_CLOUD_CONN > CH_CONSOLE_CONN, "cloud conn id must sit above the console conn");
_Static_assert(CH_CLOUD_TUNNEL_CONN > CH_CLOUD_CONN, "tunnel conn id must sit above the cloud conn");
_Static_assert(CH_CLOUD_TUNNEL_CONN_COUNT >= 1, "need at least one tunnel conn");
static line_asm_t line_asm[CH_CLOUD_TUNNEL_CONN_LAST + 1];

static proto_t proto[CH_CLOUD_TUNNEL_CONN_LAST + 1];

proto_t conn_proto(int conn_id) {
    return (conn_id >= 0 && conn_id <= CH_CLOUD_TUNNEL_CONN_LAST) ? proto[conn_id] : PROTO_UNKNOWN;
}

void conn_proto_set(int conn_id, proto_t p) {
    if (conn_id >= 0 && conn_id <= CH_CLOUD_TUNNEL_CONN_LAST) proto[conn_id] = p;
}

/* Drop any partial command buffered for this connection and reset its protocol so the next
   client on this slot is re-detected. */
void transport_conn_closed(int conn_id) {
    if (!conn_runs_state_machine(conn_id)) return;
    line_asm[conn_id].len = 0;
    line_asm[conn_id].discarding = false;
    proto[conn_id] = PROTO_UNKNOWN;
}

/* A LAN SCPI line with any non-query part, while a cloud job holds the pod (lease_gate.h), or
   from a connection that does not own the pod's own lease (lease_local.h). */
static bool scpi_line_blocked_by_lease(int conn_id, const char *line) {
    const uint32_t now = HAL_GetTick();
    const bool cloud = command_handler_policy_src(conn_id) == POLICY_SRC_LAN && lease_gate_active(now, NULL);
    lease_local_view_t v;
    const bool local = lease_local_get(conn_id, now, &v) && !v.mine;
    if (!(cloud || local) || !cmd_gate_scpi_line_writes(line)) return false;
    LOG_D("[cmd] SCPI \"%s\" refused: %s holds the pod\n", line, cloud ? "a cloud job" : v.holder);
    return true;
}

/* Is protocol `p` served on TCP connection conn_id (firmware 4.0.0, pod_policy.h)? SCPI only while
   it is switched on; JSON on a plaintext connection from the external LAN only while HTTPS-only
   is off (with it on, net_server lets such a connection in for SCPI alone). The cloud tunnels
   are not TCP connections and are not affected. */
bool net_conn_on_usb(int conn_id);   /* net_server.h (which pulls in lwIP for the host tests) */
bool net_conn_on_tls(int conn_id);
static bool proto_served(int conn_id, proto_t p) {
    if (conn_id < 0 || conn_id >= CH_MAX_CONN) return true;
    if (p == PROTO_SCPI) return pod_policy_scpi();
    if (p == PROTO_JSON)
        return net_conn_on_usb(conn_id) || net_conn_on_tls(conn_id) || !pod_policy_https_only();
    return true;
}

/* Refuse the connection's protocol once and close it (the rest of its bytes are dropped). */
static void proto_refuse(int conn_id, proto_t p) {
    proto[conn_id] = PROTO_REFUSED;
    if (p == PROTO_SCPI) {
        static const char k_off[] = "SCPI is off on this BenchPod: switch it on over USB or from the cloud "
                                    "(console: lan-tls scpi on)\n";
        at_send_data(conn_id, (const uint8_t *)k_off, sizeof(k_off) - 1u);
    } else {
        bp_err_lan_plaintext_off(conn_id);   /* BP0075 */
    }
    at_close_connection(conn_id);
}

void command_handler_process(int conn_id, const uint8_t *json_buf, size_t len) {
    /* conn_id outside the buffered set (console/cloud-command pseudo-conns or an unexpected id) —
       best-effort one-shot parse without per-connection buffering. The cloud tunnel IS buffered so
       its raw SWD/UART byte stream survives across segments like a real TCP conn. */
    if (!conn_runs_state_machine(conn_id)) {
        char buf[LINE_ASM_SIZE];
        size_t copy = len < sizeof(buf) - 1 ? len : sizeof(buf) - 1;
        memcpy(buf, json_buf, copy);
        buf[copy] = '\0';
        dispatch_line(conn_id, buf);
        return;
    }

    /* Accumulate into the per-connection line buffer and dispatch each
       newline-terminated command.  Handles commands split across TCP
       segments and multiple commands concatenated in one segment. */
    line_asm_t *la = &line_asm[conn_id];
    size_t i = 0;
    while (i < len) {
        /* Cloud speed-test sink (command_handler_net.c). */
        if (proto[conn_id] == PROTO_SPEEDTEST) {
            i += speedtest_receive(conn_id, json_buf + i, len - i);
            continue;
        }

        /* Raw binary waveform upload (command_handler_dac.c). */
        if (proto[conn_id] == PROTO_LOAD) {
            i += load_bin_receive(conn_id, json_buf + i, len - i);
            continue;
        }

        /* CMSIS-DAP mode (command_handler_dap.c). */
        if (proto[conn_id] == PROTO_DAP) {
            i += dap_proxy_receive(conn_id, json_buf + i, len - i);
            continue;   /* loop: process any trailing JSON, or exit at end */
        }

        /* UART transparent proxy (command_handler_uart.c). */
        if (proto[conn_id] == PROTO_UART) {
            i += uart_proxy_receive(conn_id, json_buf + i, len - i);
            continue;
        }

        /* Refused, closing: nothing more is read. */
        if (proto[conn_id] == PROTO_REFUSED) return;

        char c = (char)json_buf[i++];
        if (c == '\r') continue;          /* tolerate CRLF */
        if (c == '\n') {
            if (la->discarding) {         /* end of an over-long line */
                la->discarding = false;
                la->len = 0;
            } else if (la->len > 0) {
                la->buf[la->len] = '\0';
                /* Checked on every line: a setting switched off meanwhile ends the session. */
                if (!proto_served(conn_id, proto[conn_id])) {
                    la->len = 0;
                    proto_refuse(conn_id, proto[conn_id]);
                    return;
                }
                if (proto[conn_id] == PROTO_SCPI && scpi_line_blocked_by_lease(conn_id, la->buf)) {
                    /* SCPI setters wait while a cloud job holds the pod; queries still answer. */
                    scpi_push_execution_error();
                } else if (proto[conn_id] == PROTO_SCPI) scpi_dispatch_line(conn_id, la->buf);
                else                              dispatch_line(conn_id, la->buf);
                la->len = 0;
            }
            continue;
        }
        if (la->discarding) continue;
        /* Decide the protocol on the first non-whitespace byte of the
           connection.  Leading whitespace before that byte is skipped. */
        if (proto[conn_id] == PROTO_UNKNOWN) {
            if (c == ' ' || c == '\t') continue;
            proto[conn_id] = (c == '{') ? PROTO_JSON : PROTO_SCPI;
            if (!proto_served(conn_id, proto[conn_id])) { proto_refuse(conn_id, proto[conn_id]); return; }
        }
        if (la->len < LINE_ASM_SIZE - 1) {
            la->buf[la->len++] = c;
        } else {
            /* No newline within LINE_ASM_SIZE — drop the line, then swallow the
               rest until its terminating newline.  Notify JSON clients; SCPI
               lines this long are not expected, so just discard. */
            la->len = 0;
            la->discarding = true;
            if (proto[conn_id] != PROTO_SCPI) send_error(conn_id, "command too long");
        }
    }
}
