/*
 * command_handler_dap.c — the on-pod CMSIS-DAP probe (`dap_start`, the PROTO_DAP raw mode).
 *
 * dap_start arms the SWD engine on two LA channels and switches the connection to length-framed
 * CMSIS-DAP packets, which dap.c executes on the pod (dap_proxy_receive). A zero-length frame
 * returns the connection to JSON; closing it disarms SWD.
 *
 * Runs on the hw worker task, like the rest of command_handler.
 */
#include "command_handler.h"
#include "command_handler_internal.h"
#include "at_driver.h"
#include "bp_json.h"
#include "signal_engine.h"   /* fpga_swd_*, fpga_spi_armed */
#include "la_pins.h"
#include "dap.h"
#include "swd_ll.h"
#include "watchdog.h"
#include "hw_worker.h"       /* hw_worker_submit_tunnel_reset (DAP send stall) */
#include "pico_compat.h"     /* sleep_ms (yields to FreeRTOS) */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/time.h"

/* JSON parsing is shared (see bp_json.h); same aliases as command_handler.c. */
#define json_get_value      bp_json_get

/* CMSIS-DAP-over-tunnel reassembly (PROTO_DAP).  The SWD engine is a single
   shared resource (fpga_swd_arm rejects a second arm), so at most one connection
   is ever in DAP mode — one static reassembly buffer suffices.  A frame is
   [len_lo][len_hi][packet]; a zero-length frame leaves DAP mode. */
static uint8_t dap_rx[2 + DAP_PACKET_SIZE];
static size_t  dap_rx_have;                 /* bytes accumulated incl. 2-byte header */
static uint8_t dap_resp[2 + DAP_PACKET_SIZE];

/* A DAP response must go out whole (the host reads [len][packet] frames; a lost or torn one
   desyncs the session for good), and the request must not run unless its response can: a
   DAP_Transfer write that ran but whose answer was dropped leaves the host retrying a write the
   target already took.  So before running a packet, wait for room for the largest response,
   like send_paced in scpi_server.c: 1 ms sleeps with the worker's watchdog heartbeat (a slow
   cloud tunnel draining is progress, not a hang), bounded by DAP_SEND_STALL_MS. */
#ifndef DAP_SEND_STALL_MS
#define DAP_SEND_STALL_MS 5000u
#endif
static int dap_dead_conn = -1;   /* timed out waiting: drop its bytes until it is torn down */

static bool dap_wait_send_room(int conn_id) {
    const size_t need = 2u + DAP_PACKET_SIZE;
    if (at_send_avail(conn_id) >= need) return true;
    absolute_time_t deadline = make_timeout_time_ms(DAP_SEND_STALL_MS);
    while (at_send_avail(conn_id) < need) {
        if (time_reached(deadline)) return false;
        watchdog_heartbeat(WD_TASK_WORKER, "dap-send");
        sleep_ms(1);
    }
    return true;
}

/* Disarming hands SWCLK/SWDIO back to the LA bank, so the ownership table has to follow. */
/* One line per DAP session on the console: how much of the session the pod spent executing
   packets (SWD over the FPGA link) versus waiting for the next one (host + network). */
static struct {
    uint32_t packets;
    uint32_t bytes;          /* request + response payload */
    uint64_t busy_us;
    absolute_time_t start;
} s_dap_stats;

static void dap_stats_report(void) {
    if (s_dap_stats.packets == 0) return;
    uint64_t total_us = (uint64_t)absolute_time_diff_us(s_dap_stats.start, get_absolute_time());
    printf("[dap] session: %lu packets, %lu bytes, %lu ms executing of %lu ms (%lu%%)\n",
           (unsigned long)s_dap_stats.packets, (unsigned long)s_dap_stats.bytes,
           (unsigned long)(s_dap_stats.busy_us / 1000u), (unsigned long)(total_us / 1000u),
           (unsigned long)(total_us ? s_dap_stats.busy_us * 100u / total_us : 0u));
    s_dap_stats.packets = 0;
}

/* dap_start "wait_ms": keep trying a line reset + DP IDR read until the target answers, so a
   host that has just switched the target's rail on does not start OpenOCD against a board that
   is still booting (an STM32 NUCLEO answers ~2.1 s after power-on: its on-board ST-LINK comes up
   first).  The IDR is readable with the target held in reset, so this works under
   connect-under-reset too.  It waits for a STABLE answer (DAP_WAIT_STABLE_OK reads in a row,
   DAP_WAIT_STABLE_MS apart): the first OK came ~100 ms before the NUCLEO's ST-LINK let go of
   the target (its F446 runs ~2.2 s after power-on), and the first flash after a pod boot then
   failed with "cannot read IDR" while the retry passed.  Returns ms until the answer was
   stable, or -1 on timeout. */
#define DAP_WAIT_MAX_MS 10000u
#define DAP_WAIT_STABLE_OK 5
#define DAP_WAIT_STABLE_MS 50
static int dap_wait_for_target(uint32_t wait_ms) {
    static const uint8_t line_reset[7] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };  /* 56 ones */
    static const uint8_t jtag_to_swd[2] = { 0x9E, 0xE7 };
    static const uint8_t idle[1] = { 0x00 };
    if (wait_ms > DAP_WAIT_MAX_MS) wait_ms = DAP_WAIT_MAX_MS;
    absolute_time_t t0 = get_absolute_time();
    absolute_time_t deadline = make_timeout_time_ms(wait_ms);
    int ok = 0;
    for (;;) {
        swd_ll_seq_out(line_reset, 56);
        swd_ll_seq_out(jtag_to_swd, 16);
        swd_ll_seq_out(line_reset, 56);
        swd_ll_seq_out(idle, 8);
        uint32_t idr = 0;
        if (swd_ll_transfer(SWD_REQ_RnW, &idr) == SWD_ACK_OK) {  /* DP read, A[3:2]=0: DPIDR */
            if (++ok >= DAP_WAIT_STABLE_OK)
                return (int)(absolute_time_diff_us(t0, get_absolute_time()) / 1000);
        } else {
            ok = 0;
        }
        if (time_reached(deadline)) return -1;
        watchdog_heartbeat(WD_TASK_WORKER, "dap-wait");
        sleep_ms(ok ? DAP_WAIT_STABLE_MS : 20);
    }
}

void swd_disarm_and_release(void) {
    dap_stats_report();
    fpga_swd_disarm();
    la_pins_release_fn(LA_FN_SWD_CLK);
    la_pins_release_fn(LA_FN_SWD_DIO);
}

/* The connection's teardown: an SWD session releases the wire to a safe state. */
void dap_conn_closed(int conn_id) {
    if (!conn_runs_state_machine(conn_id)) return;
    if (conn_proto(conn_id) == PROTO_DAP) {
        swd_disarm_and_release();
        dap_rx_have = 0;
    }
    if (dap_dead_conn == conn_id) dap_dead_conn = -1;
}

/* command_handler_poll's SWD pass: an SWD session that hit its inactivity backstop
   (fpga_swd_poll was never called until now) releases its two pins. */
void dap_poll(void) {
    fpga_swd_poll();
    if (!fpga_swd_armed() && la_pins_mask_fn(LA_FN_SWD_CLK)) {
        la_pins_release_fn(LA_FN_SWD_CLK);
        la_pins_release_fn(LA_FN_SWD_DIO);
    }
}

/* ---- SWD probe ---- */

/* Arm the SWD engine and flip this connection to PROTO_DAP, where its byte
   stream carries length-framed CMSIS-DAP packets executed on-pod by dap.c.
   The pod is its own CMSIS-DAP probe — the per-bit remote_bitbang torrent is
   replaced by batched DAP transfers, the path that makes flashing fast over the
   internet.  The "ok" response is sent FIRST, while the connection is still
   JSON, so the client sees the ack before the byte stream switches meaning. */
void handle_dap_start(int conn_id, const char *json) {
    if (!require_la_voltage(conn_id)) return;
    char s[8] = {0};
    if (!json_get_value(json, "swclk", s, sizeof(s))) {
        send_error(conn_id, "missing swclk");
        return;
    }
    unsigned swclk = (unsigned)atoi(s);
    if (!json_get_value(json, "swdio", s, sizeof(s))) {
        send_error(conn_id, "missing swdio");
        return;
    }
    unsigned swdio = (unsigned)atoi(s);
    /* "nreset" used to name an LA channel to borrow as the target reset line.
       rev3 has a dedicated /NRST_CONTROL pin (PF4 -> J1 pin 22), so the field is
       gone: it is accepted and ignored for old clients rather than rejected,
       because a stale "nreset":N would otherwise fail a flash that will now work
       fine on the real pin. */
    /* Claim both pins BEFORE touching the FPGA: la_bank.v gives SWD the highest priority, so
       arming over a running UART proxy or sensor used to silently take the line away. */
    uint8_t clk_pin = (uint8_t)swclk, dio_pin = (uint8_t)swdio;
    if (!la_claim_or_error(conn_id, LA_FN_SWD_CLK, &clk_pin, 1, 0)) return;
    if (!la_claim_or_error(conn_id, LA_FN_SWD_DIO, &dio_pin, 1, 0)) return;

    int rc = fpga_swd_arm(swclk, swdio);
    if (rc == -2 && fpga_spi_armed()) {
        send_error(conn_id, "swd busy: an SPI session is using the engine ({\"cmd\":\"spi_stop\"})");
        return;
    }
    if (rc == -2) { send_error(conn_id, "swd busy"); return; }
    if (rc != 0)  { send_error(conn_id, "invalid swd la channel"); return; }
    la_pins_claim(LA_FN_SWD_CLK, LA_GPIO_NONE, 0, &clk_pin, 1);
    la_pins_claim(LA_FN_SWD_DIO, LA_GPIO_NONE, 0, &dio_pin, 1);

    dap_reset();
    /* Opt-in larger packets / several in flight (see dap.h: the old CLI bridge breaks on them). */
    {
        char v[12];
        unsigned size = DAP_PACKET_DEFAULT, count = 1;
        if (json_get_value(json, "packet_size", v, sizeof(v)))  size  = (unsigned)atoi(v);
        if (json_get_value(json, "packet_count", v, sizeof(v))) count = (unsigned)atoi(v);
        dap_configure(size, count);
        if (json_get_value(json, "wait_ms", v, sizeof(v)) && atoi(v) > 0) {
            int ms = dap_wait_for_target((uint32_t)atoi(v));
            if (ms >= 0) printf("[dap] target answered after %d ms\n", ms);
            else         printf("[dap] no target answer within %s ms; starting anyway\n", v);
        }
    }
    dap_rx_have = 0;
    memset(&s_dap_stats, 0, sizeof(s_dap_stats));
    s_dap_stats.start = get_absolute_time();

    send_ok_str(conn_id, "\"dap ready\"");
    conn_proto_set(conn_id, PROTO_DAP);   /* AFTER the ack: bytes are now framed packets */

    /* A DAP flashing session is a torrent of small request/response packets, so
       turn Nagle OFF for the duration to avoid per-packet batching delay.  The
       client may have already pipelined its first DAP frame right behind
       dap_start; at_set_tcp_nodelay() preserves that pending command across the
       option change so it isn't dropped. Socket-only: the cloud tunnel has no pcb. */
    if (conn_has_socket(conn_id))
        at_set_tcp_nodelay(conn_id, true);
}

/* CMSIS-DAP mode: reassemble [len_lo][len_hi][packet] frames, run each
   through dap_process() on-pod, and write the framed response back.  A
   zero-length frame (or an over-long declared length) leaves DAP mode.
   The PROTO_DAP receive path of command_handler_process: returns the bytes
   taken from buf (what follows a leaving frame is the next protocol's). */
size_t dap_proxy_receive(int conn_id, const uint8_t *buf, size_t len) {
    if (dap_dead_conn == conn_id) return len;   /* being torn down: drop the rest */
    size_t i = 0;
    bool leave = false;
    for (;;) {
        /* fill the 2-byte length header */
        while (dap_rx_have < 2 && i < len)
            dap_rx[dap_rx_have++] = buf[i++];
        if (dap_rx_have < 2) break;           /* need more header bytes */

        size_t need = 2 + ((size_t)dap_rx[0] | ((size_t)dap_rx[1] << 8));
        if (need > sizeof(dap_rx)) {          /* bad length: leave DAP */
            dap_rx_have = 0; leave = true; break;
        }
        if (dap_rx_have < need) {             /* fill the payload */
            size_t take = need - dap_rx_have;
            size_t avail = len - i;
            if (take > avail) take = avail;
            memcpy(dap_rx + dap_rx_have, buf + i, take);
            dap_rx_have += take;
            i += take;
        }
        if (dap_rx_have < need) break;        /* frame split across segments */

        size_t payload = need - 2;
        dap_rx_have = 0;
        if (payload == 0) { leave = true; break; }   /* zero-length: leave */

        if (!dap_wait_send_room(conn_id)) {
            /* The peer stopped reading. Do not run the packet; tear the link down
               (command_handler_conn_closed disarms SWD) and drop its bytes until then. */
            printf("[dap] conn %d: no room for a response in %u ms; %s\n", conn_id,
                   (unsigned)DAP_SEND_STALL_MS, is_tunnel_conn(conn_id) ? "resetting the tunnel"
                                                                        : "closing");
            dap_dead_conn = conn_id;
            if (is_tunnel_conn(conn_id)) hw_worker_submit_tunnel_reset(conn_id);
            else                         at_close_connection(conn_id);
            i = len;
            break;
        }
        absolute_time_t t0 = get_absolute_time();
        size_t rlen = dap_process(dap_rx + 2, payload,
                                  dap_resp + 2, DAP_PACKET_SIZE);
        s_dap_stats.busy_us += (uint64_t)absolute_time_diff_us(t0, get_absolute_time());
        s_dap_stats.packets++;
        s_dap_stats.bytes   += (uint32_t)(payload + rlen);
        dap_resp[0] = (uint8_t)(rlen & 0xFF);
        dap_resp[1] = (uint8_t)((rlen >> 8) & 0xFF);
        /* Room was checked above and only this task fills the ring, so this is whole. */
        at_send_data(conn_id, dap_resp, rlen + 2);
    }
    if (leave) {
        swd_disarm_and_release();
        conn_proto_set(conn_id, PROTO_UNKNOWN);   /* re-detect the next byte */
        /* Back to JSON/SCPI: restore default Nagle batching.
           Socket-only: the cloud tunnel has no pcb. */
        if (conn_has_socket(conn_id))
            at_set_tcp_nodelay(conn_id, false);
    }
    return i;
}
