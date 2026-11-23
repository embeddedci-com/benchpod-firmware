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
#include "bp_errors.h"
#include "at_driver.h"
#include "bp_json.h"
#include "signal_engine.h"   /* fpga_swd_*, fpga_spi_armed */
#include "la_pins.h"
#include "dap.h"
#include "prog_flash.h"
#include "swd_ll.h"
#include "jtag_ll.h"
#include "gw_caps.h"
#include "watchdog.h"
#include "hw_worker.h"       /* hw_worker_submit_tunnel_reset (DAP send stall) */
#include "pico_compat.h"     /* sleep_ms (yields to FreeRTOS) */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/time.h"
#include "bp_log.h"

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
    LOG_D("[dap] session: %lu packets, %lu bytes, %lu ms executing of %lu ms (%lu%%)\n",
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

/* The pins of the DAP session (LA channels, 0 = none), for DAP_Connect's port switch. */
static struct {
    unsigned swclk, swdio;               /* SWD */
    unsigned tck, tms, tdi, tdo, trst;   /* JTAG */
    unsigned half;                       /* JTAG TCK half period when armed */
} s_dap_pins;

static const la_fn_t k_jtag_fns[5] = { LA_FN_JTAG_TCK, LA_FN_JTAG_TMS, LA_FN_JTAG_TDI, LA_FN_JTAG_TDO,
                                       LA_FN_JTAG_TRST };

static bool dap_pins_claimed(void) {
    if (la_pins_mask_fn(LA_FN_SWD_CLK) || la_pins_mask_fn(LA_FN_SWD_DIO)) return true;
    for (size_t i = 0; i < 5; i++) if (la_pins_mask_fn(k_jtag_fns[i])) return true;
    return false;
}

static void dap_release_pins(void) {
    if (s_dap_pins.trst) fpga_la_set(s_dap_pins.trst, 2);   /* nTRST back to high-Z */
    la_pins_release_fn(LA_FN_SWD_CLK);
    la_pins_release_fn(LA_FN_SWD_DIO);
    for (size_t i = 0; i < 5; i++) la_pins_release_fn(k_jtag_fns[i]);
    jtag_ll_set_trst_channel(0);
    memset(&s_dap_pins, 0, sizeof(s_dap_pins));
}

void swd_disarm_and_release(void) {
    dap_stats_report();
    fpga_swd_disarm();
    fpga_jtag_disarm();
    dap_release_pins();
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

/* command_handler_poll's SWD pass: a DAP session that hit its inactivity backstop
   (fpga_swd_poll disarms the engine) releases its pins. */
void dap_poll(void) {
    fpga_swd_poll();
    if (!fpga_swd_armed() && !fpga_jtag_armed() && dap_pins_claimed()) dap_release_pins();
}

/* DAP_Connect asked for the other port: re-arm the engine on that port's pins. */
static bool dap_switch_port(uint8_t port) {
    if (port == DAP_PORT_JTAG) {
        if (!s_dap_pins.tck) return false;
        fpga_swd_disarm();
        if (fpga_jtag_arm(s_dap_pins.tck, s_dap_pins.tms, s_dap_pins.tdi, s_dap_pins.tdo,
                          s_dap_pins.half) == 0) return true;
        fpga_swd_arm(s_dap_pins.swclk, s_dap_pins.swdio);
        return false;
    }
    if (!s_dap_pins.swclk) return false;
    fpga_jtag_disarm();
    if (fpga_swd_arm(s_dap_pins.swclk, s_dap_pins.swdio) == 0) return true;
    fpga_jtag_arm(s_dap_pins.tck, s_dap_pins.tms, s_dap_pins.tdi, s_dap_pins.tdo, s_dap_pins.half);
    return false;
}

/* dap_wait_for_target for JTAG: Test-Logic-Reset, then the first IDCODE on the chain (nearest
   TDO), until it reads as an IDCODE (bit 0 set, not all ones) DAP_WAIT_STABLE_OK times in a row. */
static int dap_wait_for_target_jtag(uint32_t wait_ms) {
    static const uint8_t seq[] = {
        0x45, 0xFF,                    /* 5 x TMS 1: Test-Logic-Reset */
        0x01, 0x00,                    /* TMS 0: Run-Test/Idle */
        0x41, 0xFF,                    /* Select-DR */
        0x02, 0x00,                    /* Capture-DR, Shift-DR */
        0xA0, 0xFF, 0xFF, 0xFF, 0xFF,  /* 32 bits, captured */
        0x45, 0xFF,                    /* out to Test-Logic-Reset */
        0x01, 0x00,                    /* Run-Test/Idle */
    };
    if (wait_ms > DAP_WAIT_MAX_MS) wait_ms = DAP_WAIT_MAX_MS;
    absolute_time_t t0 = get_absolute_time();
    absolute_time_t deadline = make_timeout_time_ms(wait_ms);
    int ok = 0;
    for (;;) {
        uint8_t tdo[4] = { 0 };
        if (jtag_ll_run(seq, sizeof(seq), tdo, sizeof(tdo)) == 0) {
            uint32_t id = (uint32_t)tdo[0] | ((uint32_t)tdo[1] << 8) | ((uint32_t)tdo[2] << 16) |
                          ((uint32_t)tdo[3] << 24);
            if ((id & 1u) && id != 0xFFFFFFFFu) {
                if (++ok >= DAP_WAIT_STABLE_OK)
                    return (int)(absolute_time_diff_us(t0, get_absolute_time()) / 1000);
            } else {
                ok = 0;
            }
        }
        if (time_reached(deadline)) return -1;
        watchdog_heartbeat(WD_TASK_WORKER, "dap-wait");
        sleep_ms(ok ? DAP_WAIT_STABLE_MS : 20);
    }
}

/* ---- SWD probe ---- */

/* Arm the SWD engine and flip this connection to PROTO_DAP, where its byte
   stream carries length-framed CMSIS-DAP packets executed on-pod by dap.c.
   The pod is its own CMSIS-DAP probe — the per-bit remote_bitbang torrent is
   replaced by batched DAP transfers, the path that makes flashing fast over the
   internet.  The "ok" response is sent FIRST, while the connection is still
   JSON, so the client sees the ack before the byte stream switches meaning. */
/* An optional LA channel argument: 0 when absent. */
static unsigned json_la(const char *json, const char *key) {
    char s[8] = {0};
    return json_get_value(json, key, s, sizeof(s)) ? (unsigned)atoi(s) : 0u;
}

void handle_dap_start(int conn_id, const char *json) {
    if (!require_la_voltage(conn_id)) return;
    unsigned swclk = json_la(json, "swclk"), swdio = json_la(json, "swdio");
    unsigned tck = json_la(json, "tck"), tms = json_la(json, "tms"),
             tdi = json_la(json, "tdi"), tdo = json_la(json, "tdo"), trst = json_la(json, "trst");
    bool any_jtag = tck || tms || tdi || tdo || trst;
    bool have_jtag = tck && tms && tdi && tdo;
    bool have_swd  = swclk && swdio;
    if (any_jtag && !have_jtag) {
        send_error(conn_id, "JTAG needs tck, tms, tdi and tdo");
        return;
    }
    if (!have_swd && !have_jtag) {
        send_error(conn_id, swclk ? "missing swdio" : "missing swclk");
        return;
    }
    /* JTAG needs the gateware's JTAG master: gateware v61, loop or deep image. */
    if (have_jtag && !gw_caps()->jtag) {
        if (!have_swd) {
            if (!gw_caps()->jtag_gw) { bp_err_gateware_too_old(conn_id, "JTAG", JTAG_MIN_GW); return; }
            if (signal_engine_image_kind() == 2) bp_err_fpga_image_sensors_first(conn_id, "JTAG", "loop");
            else                                 bp_err_fpga_image_required(conn_id, "JTAG", "loop");
            return;
        }
        LOG_D("[dap] JTAG pins given but the running gateware has no JTAG master: SWD only\n");
        have_jtag = false;
        tck = tms = tdi = tdo = trst = 0;
    }
    /* "port": which one to arm first when both are wired (DAP_Connect can switch later). */
    char pv[8] = {0};
    bool jtag_first = have_jtag && (!have_swd ||
                      (json_get_value(json, "port", pv, sizeof(pv)) && strcmp(pv, "jtag") == 0));

    /* A prog_flash job holds the engine (two-wire) or the DAP's scratch buffer (UPDI). */
    if (prog_flash_busy()) { bp_err_bus_engine_busy(conn_id, jtag_first ? "jtag" : "swd"); return; }

    /* Claim every pin BEFORE touching the FPGA: la_bank.v gives the engine the highest priority,
       so arming over a running UART proxy or sensor used to silently take the line away.  A pin
       both ports use (SWCLK = TCK and SWDIO = TMS on an ARM part) is claimed once, as SWD. */
    if (have_swd) {
        uint8_t clk_pin = (uint8_t)swclk, dio_pin = (uint8_t)swdio;
        if (!la_claim_or_error(conn_id, LA_FN_SWD_CLK, &clk_pin, 1, 0)) return;
        if (!la_claim_or_error(conn_id, LA_FN_SWD_DIO, &dio_pin, 1, 0)) return;
    }
    if (have_jtag) {
        const unsigned pins[5] = { tck, tms, tdi, tdo, trst };
        for (size_t i = 0; i < 5; i++) {
            if (!pins[i] || (have_swd && (pins[i] == swclk || pins[i] == swdio))) continue;
            uint8_t pin = (uint8_t)pins[i];
            if (!la_claim_or_error(conn_id, k_jtag_fns[i], &pin, 1, 0)) return;
        }
    }

    memset(&s_dap_pins, 0, sizeof(s_dap_pins));
    s_dap_pins.half = SPI_HALF_MAX;   /* ~190 kHz until DAP_SWJ_Clock */
    int rc;
    if (jtag_first) rc = fpga_jtag_arm(tck, tms, tdi, tdo, s_dap_pins.half);
    else            rc = fpga_swd_arm(swclk, swdio);
    if (rc == -2 && (fpga_spi_armed() || fpga_i2c_armed())) {
        bp_err_bus_engine_busy(conn_id, jtag_first ? "jtag" : "swd");
        return;
    }
    if (rc == -2) { bp_err_swd_session_open(conn_id); return; }
    if (rc != 0)  { send_error(conn_id, jtag_first ? "invalid jtag la channel" : "invalid swd la channel"); return; }

    if (have_swd) {
        uint8_t clk_pin = (uint8_t)swclk, dio_pin = (uint8_t)swdio;
        la_pins_claim(LA_FN_SWD_CLK, LA_GPIO_NONE, 0, &clk_pin, 1);
        la_pins_claim(LA_FN_SWD_DIO, LA_GPIO_NONE, 0, &dio_pin, 1);
        s_dap_pins.swclk = swclk; s_dap_pins.swdio = swdio;
    }
    if (have_jtag) {
        const unsigned pins[5] = { tck, tms, tdi, tdo, trst };
        for (size_t i = 0; i < 5; i++) {
            if (!pins[i] || (have_swd && (pins[i] == swclk || pins[i] == swdio))) continue;
            uint8_t pin = (uint8_t)pins[i];
            la_pins_claim(k_jtag_fns[i], LA_GPIO_NONE, 0, &pin, 1);
        }
        s_dap_pins.tck = tck; s_dap_pins.tms = tms; s_dap_pins.tdi = tdi; s_dap_pins.tdo = tdo;
        s_dap_pins.trst = trst;
        if (trst) fpga_la_set(trst, 1);   /* nTRST released (high) */
    }

    dap_reset();
    dap_set_ports((uint8_t)((have_swd ? DAP_PORTS_SWD : 0u) | (have_jtag ? DAP_PORTS_JTAG : 0u)),
                  jtag_first ? DAP_PORT_JTAG : DAP_PORT_SWD, dap_switch_port);
    jtag_ll_set_trst_channel(trst);
    /* Opt-in larger packets / several in flight (see dap.h: the old CLI bridge breaks on them). */
    {
        char v[12];
        unsigned size = DAP_PACKET_DEFAULT, count = 1;
        if (json_get_value(json, "packet_size", v, sizeof(v)))  size  = (unsigned)atoi(v);
        if (json_get_value(json, "packet_count", v, sizeof(v))) count = (unsigned)atoi(v);
        dap_configure(size, count);
        if (json_get_value(json, "wait_ms", v, sizeof(v)) && atoi(v) > 0) {
            int ms = jtag_first ? dap_wait_for_target_jtag((uint32_t)atoi(v))
                                : dap_wait_for_target((uint32_t)atoi(v));
            if (ms >= 0) LOG_D("[dap] target answered after %d ms\n", ms);
            else         LOG_D("[dap] no target answer within %s ms; starting anyway\n", v);
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
            LOG_W("[dap] conn %d: no room for a response in %u ms; %s\n", conn_id,
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
