/*
 * command_handler_uart.c — the UART transparent proxy (`uart_proxy_start`, the PROTO_UART raw mode).
 *
 * One connection at a time bridges a DUT UART through two LA channels: client bytes go to the
 * soft UART's TX (uart_proxy_rx), DUT bytes are drained to the client from command_handler_poll
 * (uart_proxy_poll). The session ends when the client closes the connection, or on the guard-timed
 * Hayes escape, which returns the connection to JSON.
 *
 * Runs on the hw worker task, like the rest of command_handler.
 */
#include "command_handler.h"
#include "command_handler_internal.h"
#include "at_driver.h"
#include "bp_json.h"
#include "signal_engine.h"   /* fpga_uart_* */
#include "i2c_bus.h"         /* pca9555_* LA pull-ups */
#include "la_pins.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/time.h"   /* absolute_time_t, get_absolute_time, *_diff_us (UART escape guard timing) */

/* JSON parsing is shared (see bp_json.h); same aliases as command_handler.c. */
#define json_get_value      bp_json_get

/* ---- UART transparent proxy state (one active connection at a time) -------
   Entered by JSON "uart_proxy_start"; the connection then carries raw bytes
   to/from the DUT UART.  Exit: the client closes the socket (primary), or
   sends the guard-timed Hayes escape  <≥1s idle> +++ <≥1s idle>  to return to
   JSON on the same connection.  RX from the DUT is drained to the client in
   command_handler_poll() (it is asynchronous, unlike SWD's `c` replies). */
#define UART_ESC_GUARD_MS  1000
static int             uart_proxy_conn = -1;     /* conn_id owning the proxy, or -1 */
static uint8_t         uart_plus_count = 0;      /* consecutive '+' in an escape candidate */
static bool            uart_plus_pending = false;/* saw "+++", awaiting the trailing guard */
static absolute_time_t uart_last_rx_at;          /* last byte received from the client */
static absolute_time_t uart_last_plus_at;        /* time of the most recent '+' */
static uint8_t         uart_rx_buf[256];         /* DUT→client drain buffer */
static uint8_t         uart_proxy_rx, uart_proxy_tx;   /* the session's pins and baud, kept so a */
static uint32_t        uart_proxy_baud;                /* gateware reconfig can re-apply them    */
static int             uart_rearm_conn = -1;           /* session to re-arm on the next poll     */
/* True while the proxy is SUSPENDED for the duration of a load_bin upload: we stop touching the
   iCE40 for UART entirely (no per-poll SPI drain) so nothing competes with the upload's PSRAM
   writes on the worker task; on resume we flush the FIFO once so stale bytes aren't forwarded. */
static bool            uart_upload_suspended = false;
/* RX-channel pull-up auto-hold: a DUT's TX pin floats before its firmware brings
   up the UART, and an un-held RX line decodes as a flood of 0x00 during boot. We
   pull the RX channel high (LA1-8 only) for the session and restore it on exit. */
static int             uart_rx_pu_ch   = -1;     /* RX LA channel we forced a pull-up on, or -1 */
static bool            uart_rx_pu_prev = false;  /* its pull-up state before we forced it */

static void uart_proxy_end(int conn_id, bool restore_json) {
    fpga_uart_disable();
    if (uart_rx_pu_ch >= 1) {                     /* restore the RX pull-up we forced on */
        /* A failed restore leaves the DUT's line biased by a resistor nobody asked for, which
           then shows up as an unexplained idle level on the next session — say so. */
        int rc = pca9555_set_la_pullup((uint8_t)uart_rx_pu_ch, uart_rx_pu_prev);
        if (rc != 0)
            printf("[uart] WARNING: could not restore LA%d's pull-up to %s (rc %d)\n",
                   uart_rx_pu_ch, uart_rx_pu_prev ? "on" : "off", rc);
        uart_rx_pu_ch = -1;
    }
    la_pins_release_fn(LA_FN_UART_RX);
    la_pins_release_fn(LA_FN_UART_TX);
    uart_proxy_conn   = -1;
    uart_plus_count   = 0;
    uart_plus_pending = false;
    uart_upload_suspended = false;   /* next proxy starts un-suspended */
    if (restore_json && conn_runs_state_machine(conn_id)) {
        conn_proto_set(conn_id, PROTO_UNKNOWN);   /* re-detect the next byte as JSON/SCPI */
        if (conn_has_socket(conn_id))
            at_set_tcp_nodelay(conn_id, false);   /* back to normal Nagle batching (socket only) */
    }
}

void uart_proxy_conn_closed(int conn_id) {
    if (!conn_runs_state_machine(conn_id)) return;
    /* A UART proxy auto-stops on socket close (the primary exit path). */
    if (conn_proto(conn_id) == PROTO_UART || uart_proxy_conn == conn_id)
        uart_proxy_end(conn_id, false);
}

/* ---- UART transparent proxy ----
 * Bridge a DUT UART through two LA channels. Like dap_start, this is the one
 * command that changes how the connection's bytes are interpreted: after the
 * `"uart ready"` ack the same TCP connection carries raw UART bytes both ways.
 *   {"cmd":"uart_proxy_start","rx":<1-12>,"tx":<1-12>,"baud":115200}
 * Exit: close the socket (primary), or send  <≥1s idle> +++ <≥1s idle>  to
 * return to JSON on the same connection. */
void handle_uart_proxy_start(int conn_id, const char *json) {
    if (!require_la_voltage(conn_id)) return;
    char s[12] = {0};
    if (!json_get_value(json, "rx", s, sizeof(s))) { send_error(conn_id, "missing rx"); return; }
    unsigned rx = (unsigned)atoi(s);
    if (!json_get_value(json, "tx", s, sizeof(s))) { send_error(conn_id, "missing tx"); return; }
    unsigned tx = (unsigned)atoi(s);
    uint32_t baud = 115200;
    if (json_get_value(json, "baud", s, sizeof(s))) baud = (uint32_t)strtoul(s, NULL, 0);

    /* Claim both channels first: a proxy sharing a line with SWD or an emulated sensor used
       to lose it silently to la_bank.v's priority.  This also rejects RX/TX on an LA7/LA8
       whose 10k pull-DOWN is engaged — the UART idles high. */
    uint8_t rx_pin = (uint8_t)rx, tx_pin = (uint8_t)tx;
    if (!la_claim_or_error(conn_id, LA_FN_UART_RX, &rx_pin, 1, 0)) return;
    if (!la_claim_or_error(conn_id, LA_FN_UART_TX, &tx_pin, 1, 0)) return;

    int rc = fpga_uart_config(rx, tx, baud, true);
    if (rc == -2) { send_error(conn_id, "uart busy"); return; }
    if (rc != 0)  { send_error(conn_id, "invalid uart args"); return; }
    la_pins_claim(LA_FN_UART_RX, LA_GPIO_NONE, 0, &rx_pin, 1);
    la_pins_claim(LA_FN_UART_TX, LA_GPIO_NONE, 0, &tx_pin, 1);

    /* Verify the iCE40 actually answers before committing to raw UART mode: a bogus
       status read (rx_avail out of range) means the FPGA isn't responding over SPI
       (unconfigured / bad gateware), which otherwise surfaces as a silent dead
       terminal flooded with 0xFF.  Refuse with a clear error the UI can show. */
    uint16_t rx_avail = 0;
    if (fpga_uart_status(&rx_avail, NULL) != 0) {
        fpga_uart_disable();
        la_pins_release_fn(LA_FN_UART_RX);
        la_pins_release_fn(LA_FN_UART_TX);
        send_error(conn_id, "uart proxy failed: FPGA not responding (check gateware)");
        return;
    }

    /* Hold the RX line high for the session: a DUT's TX pin floats until its
       firmware configures the UART, which the soft-UART otherwise decodes as a
       flood of 0x00 during boot. Restored in uart_proxy_end.
       LA1-LA6 only: LA7/LA8's resistor pulls DOWN, so engaging "the pull" there held RX at
       the opposite of the idle level and guaranteed the 0x00 flood it was meant to prevent.
       LA9-LA14 have no resistor at all, and at a 1.8 V bank the 3V3-referenced ones are
       off-limits — the proxy still runs, it just cannot hold RX high. */
    uart_rx_pu_ch = -1;
    if (la_pull_dir(rx) == LA_PULL_UP && la_pullups_available()) {
        uart_rx_pu_ch   = (int)rx;
        uart_rx_pu_prev = pca9555_la_pullup_enabled((uint8_t)rx);
        if (pca9555_set_la_pullup((uint8_t)rx, true) != 0) {
            printf("[uart] WARNING: could not engage LA%u's pull-up to hold RX high\n", rx);
            uart_rx_pu_ch = -1;
        }
    }

    send_ok_str(conn_id, "\"uart ready\"");   /* ack while still in JSON mode */
    conn_proto_set(conn_id, PROTO_UART);       /* AFTER the ack: bytes are now raw UART */
    uart_proxy_conn   = conn_id;
    uart_proxy_rx     = (uint8_t)rx;
    uart_proxy_tx     = (uint8_t)tx;
    uart_proxy_baud   = baud;
    uart_plus_count   = 0;
    uart_plus_pending = false;
    uart_last_rx_at   = get_absolute_time();
    uart_last_plus_at = uart_last_rx_at;
    if (conn_has_socket(conn_id))
        at_set_tcp_nodelay(conn_id, true);     /* low-latency interactive bytes (socket only) */
}

/* UART transparent proxy: forward the whole segment to the DUT TX, and
   scan for the guard-timed "+++" escape.  Timing is per-segment (TCP
   NODELAY makes interactive keystrokes their own segments), which is
   enough for the Hayes guard.  The "+++" is also forwarded to the DUT
   (harmless); the trailing-guard check that actually exits lives in
   uart_proxy_poll().  The PROTO_UART receive path of command_handler_process: takes all of buf. */
size_t uart_proxy_receive(int conn_id, const uint8_t *buf, size_t len) {
    (void)conn_id;
    absolute_time_t now = get_absolute_time();
    bool first = true;
    fpga_uart_write(buf, len);   /* transparent passthrough */
    for (size_t i = 0; i < len; i++) {
        uint8_t b = buf[i];
        int64_t idle_us = first
            ? absolute_time_diff_us(uart_last_rx_at, now) : 0;
        if (b == '+') {
            if (uart_plus_count == 0) {
                if (idle_us >= (int64_t)UART_ESC_GUARD_MS * 1000)
                    uart_plus_count = 1;       /* guard before 1st '+' */
            } else if (uart_plus_count < 3) {
                uart_plus_count++;
            }
            uart_last_plus_at = now;
            if (uart_plus_count >= 3) uart_plus_pending = true;
        } else {
            uart_plus_count = 0;               /* any other byte aborts */
            uart_plus_pending = false;
        }
        first = false;
    }
    uart_last_rx_at = now;
    return len;
}

/* command_handler_poll's UART pass: stream DUT→client and apply the +++ trailing guard. */
void uart_proxy_poll(void) {
    fpga_uart_tx_pump();   /* queued proxy TX bytes -> the FPGA FIFO as it drains */
    if (uart_proxy_conn >= 0) {
        if (load_bin_owner() >= 0) {
            /* A DAC-replay upload is streaming in — SUSPEND the proxy for its duration.  The pod's
               single net task can't encrypt+send a chatty console AND receive the (large) upload at
               once, and an earlier version still ran the per-poll FIFO drain here — ~1 ms of blocking
               iCE40 SPI at a full FIFO, stealing the worker from the upload's PSRAM writes.  So while
               a load_bin is active we do NOT touch the iCE40 for UART at all (no drain, no forward).
               DUT output during the transfer is dropped either way (it was already being discarded);
               we just stop paying for it.  The FIFO is left to fill and flushed once on resume. */
            if (!uart_upload_suspended)
                printf("[uart] proxy suspended (conn %d) for load_bin upload on conn %d\n",
                       uart_proxy_conn, load_bin_owner());
            uart_upload_suspended = true;
        } else {
            if (uart_upload_suspended) {
                /* Upload finished — discard whatever piled up in the FIFO while we were suspended,
                   so the resumed stream is fresh DUT output rather than stale/overflowed bytes. One
                   read empties the 256-deep FIFO. */
                uart_upload_suspended = false;
                (void)fpga_uart_read(uart_rx_buf, sizeof(uart_rx_buf));
            }
            /* Read no more than the ring takes whole (at_send_data is all or nothing): what we
               cannot send yet stays in the FIFO for the next pass instead of being lost. */
            size_t room = at_send_avail(uart_proxy_conn);
            if (room > sizeof(uart_rx_buf)) room = sizeof(uart_rx_buf);
            size_t n = room ? fpga_uart_read(uart_rx_buf, room) : 0;
            if (n > 0) {
                if (at_send_data(uart_proxy_conn, uart_rx_buf, n) != 0) {
                    /* client gone — auto-stop and reclaim the slot */
                    int c = uart_proxy_conn;
                    uart_proxy_end(c, false);
                    at_close_connection(c);
                }
            }
            /* Hayes escape: "+++" was seen; if a full guard has elapsed with no
               further client bytes, leave the proxy and return to JSON. */
            if (uart_proxy_conn >= 0 && uart_plus_pending &&
                absolute_time_diff_us(uart_last_plus_at, get_absolute_time())
                    >= (int64_t)UART_ESC_GUARD_MS * 1000) {
                printf("[uart] +++ escape — returning conn %d to JSON\n", uart_proxy_conn);
                uart_proxy_end(uart_proxy_conn, true);
            }
        }
    }
}

/* An open UART session is KEPT across a gateware reconfiguration: its soft UART is re-armed on the
   new fabric. Ending it here (as before) put the connection back in JSON mode without telling the
   client, whose terminal then went silent for good after any image swap. Re-armed on the next
   command_handler_poll pass (uart_rearm_poll), not here: this hook runs right after CDONE, before
   the caller has pinged the new fabric, and it does not answer SPI yet. */
void uart_proxy_on_gateware_reconfigured(void) {
    uart_rearm_conn = uart_proxy_conn;
}

/* The deferred half of uart_proxy_on_gateware_reconfigured(): re-arm the soft UART of a session that was open across a
   gateware reconfiguration. On failure end it and drop the connection, so the client sees the
   session close instead of a terminal that never answers again. */
void uart_rearm_poll(void) {
    int conn = uart_rearm_conn;
    if (conn < 0) return;
    uart_rearm_conn = -1;
    if (uart_proxy_conn != conn) return;              /* the session ended meanwhile */
    uint16_t rx_avail = 0;
    fpga_uart_disable();   /* the firmware still marks the old (gone) session armed: -2 otherwise */
    if (fpga_uart_config(uart_proxy_rx, uart_proxy_tx, uart_proxy_baud, true) == 0 &&
        fpga_uart_status(&rx_avail, NULL) == 0) {
        printf("[uart] gateware reconfigured: UART session on LA%u/LA%u re-armed\n",
               uart_proxy_rx, uart_proxy_tx);
        return;
    }
    printf("[uart] gateware reconfigured: could not re-arm the UART session, closing it\n");
    uart_proxy_end(conn, false);
    at_close_connection(conn);
}
