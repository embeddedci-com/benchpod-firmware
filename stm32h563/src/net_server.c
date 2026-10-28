/*
 * net_server.c — LwIP (NO_SYS=1) bring-up + the JSON/SCPI TCP command server.
 *
 * Replaces the RP2350 ESP32-AT path: the wired RMII interface comes up via DHCP,
 * and a TCP server on port 8080 (plaintext) and 8443 (TLS, lan_tls.h) hands each connection's
 * bytes to the hw worker
 * task (hw_worker_submit_bytes) — command_handler now runs THERE, not here, so a
 * long command can't stall lwIP.  This file runs entirely on the net task and
 * owns lwIP exclusively (NO_SYS=1 is single-threaded): it drains the worker's
 * per-connection conn_tx reply rings into lwIP (net_tx_drain) and applies the
 * worker's close / Nagle requests.  The at_*() transmit seam (used by
 * command_handler.c / scpi_server.c ON THE WORKER) writes into those rings rather
 * than touching lwIP directly.
 */
#include "net_server.h"
#include "pod_policy.h"
#include "ethernetif.h"
#include "command_handler.h"
#include "at_driver.h"
#include "wifi_manager.h"
#include "cloud_client.h"
#include "mbedtls_port.h"
#include "esp_hosted_spi.h"
#include "net_dhcp.h"
#include "net_route.h"
#include "esp_netif.h"
#include "usb_netif.h"
#include "usb_device.h"
#include "esp_wifi_ctrl.h"
#include "board_variant.h"   /* board_has_wifi_module */
#include "esp_rom_flash.h"
#include "conn_tx.h"
#include "hw_worker.h"
#include "bp_limits.h"
#if defined(BENCHPOD_EMI_TEST)
#include "emi_test.h"
#endif
#include "net_reload.h"

#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/tcp.h"
#include "lwip/altcp.h"
#include "lwip/altcp_tcp.h"
#include "lwip/altcp_tls.h"
#include "altcp_tls_bp/altcp_tls_bp.h"
#include "lan_tls.h"
#include "lwip/timeouts.h"

#include "lwip/dhcp.h"
#include "lwip/apps/mdns.h"
#include "netif/ethernet.h"

#include "FreeRTOS.h"
#include "task.h"        /* vTaskDelay — rare backstop in srv_recv's bulk path */

/* net_call_sync (defined further down, next to net_wifi_static) */
static TaskHandle_t s_net_task;
static void net_run_pending_call(void);

/* srv_recv accepts each inbound pbuf all-or-nothing into the hw_worker queue, so
   the queue must be able to hold a full lwIP receive window once drained — else a
   maximum-size pbuf could never fit and the connection would stall.  Pin it. */
_Static_assert((size_t)HW_WORK_QUEUE_DEPTH * (size_t)HW_WORK_DATA_MAX >= TCP_WND,
               "hw_worker queue smaller than TCP_WND — srv_recv could deadlock");

#include "device_identity.h"
#include "b64url.h"

#include "stm32h5xx_hal.h"
#include "bp_log.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#if defined(BENCHPOD_WIFI_BRIDGE)
#include "wifi_bridge_net.h"
#endif

#define TCP_SERVER_PORT   8080
#define NET_MAX_CONN      5            /* TCP conn ids 0..4 (= CH_MAX_CONN), plaintext and TLS */
/* TLS sessions at once: each holds ~25-30 KB of FreeRTOS heap (16 KB IN record buffer, 4 KB OUT,
   handshake state). A TLS client beyond this, or one arriving while the heap is below the floor
   (room the cloud link needs to reconnect), is refused. Plaintext slots are not affected. */
#define LAN_TLS_MAX_SESSIONS   2
#define LAN_TLS_HEAP_FLOOR     (48u * 1024u)

static struct altcp_pcb *conn_pcb[NET_MAX_CONN];
/* The TLS configuration a connection holds (lan_tls_ref), NULL for plaintext. */
static struct altcp_tls_config *s_conn_tls[NET_MAX_CONN];
static bool s_conn_refused[NET_MAX_CONN];   /* srv_recv refused data (TLS: kick when room) */
static volatile int8_t s_https_req = -1;    /* -1 none, else the new HTTPS-only setting */
static bool s_https_only;                   /* the net task's copy (pod_policy_https_only) */

/* ---- worker -> net requests (the worker task can't touch lwIP) ------------
   The hw worker executes commands and produces replies into the conn_tx rings,
   but the transport-control actions it triggers (close a socket, toggle Nagle)
   touch the lwIP pcb, which only the net task may do.  The worker flags them here
   and the net task applies them in net_poll(). */
static volatile uint8_t s_close_req[NET_MAX_CONN];    /* worker asked to close    */
static volatile bool    s_conn_usb[NET_MAX_CONN];     /* accepted on the USB network adapter */
static bool             s_lan_up;                     /* the external LAN may connect (lan_access) */
static volatile int8_t  s_nodelay_req[NET_MAX_CONN];  /* -1 none, 0/1 desired     */

/* ---- multihoming: one LwIP, two netifs (Ethernet always + Wi-Fi via the
   ESP32-C3). Ethernet is preferred; Wi-Fi is a fallback used only when it is
   configured + associated and the wired link is down. The ESP32 is held in
   reset until Wi-Fi credentials are provisioned (see esp_wifi_ctrl). --------- */
/* dhcp_process() runs on a ~500 ms cadence (see net_poll), so this is ~20 s: if
   a lease hasn't arrived by then we re-kick DISCOVER (resets lwIP's backoff to a
   fast 2/4/8 s cadence) rather than letting it drift out to the 60 s cap. Keeps
   reconnection snappy after a slow home router finishes rebooting. lwIP does its
   own retransmission on the fine timer; this is just a coarse backstop. */
#define DHCP_REKICK_TICKS  40

typedef struct {
    struct netif     netif;
    net_dhcp_t       dhcp;          /* pure acquire/retry state machine (net_dhcp.c) */
    uint32_t         dhcp_timer;
    uint8_t          dhcp_fails;    /* consecutive DISCOVER re-kicks with no lease */
    char             ip[16];
    bool             is_eth;        /* wired vs Wi-Fi (log tag / route priority) */
} net_if_t;

static net_if_t s_eth  = { .ip = "0.0.0.0", .is_eth = true  };
static net_if_t s_wifi = { .ip = "0.0.0.0", .is_eth = false };
/* USB CDC-NCM link: fixed /30, no DHCP client (usb_netif.c). */
static struct netif s_usb_netif;
static bool     s_wifi_static;   /* wifi-static console test: skip DHCP on wifi */
static uint32_t link_timer;

/* After this many consecutive re-kicks (~DHCP_REKICK_TICKS each) with no lease,
   the wired link is presumed wedged (bad autoneg/PLL, not just a slow router) and
   we escalate from "re-send DISCOVER" to a full PHY soft reset. 3 * ~20 s ≈ 60 s
   before the first deep recovery. This is what lets a boot that missed its lease
   self-heal instead of printing "still no lease" forever. */
#define DHCP_ESCALATE_REKICKS  3

/* Ethernet admin control (from the `eth` console/JSON command or the DHCP
   self-heal). Touching HAL_ETH + lwIP must happen on the net task, so other tasks
   only latch a request here; net_poll() applies it. */
typedef enum { ETH_REQ_NONE = 0, ETH_REQ_DOWN, ETH_REQ_UP, ETH_REQ_RESTART, ETH_REQ_SPEED,
               ETH_REQ_REFCLK, ETH_REQ_LOOPBACK } eth_req_t;
static volatile eth_req_t s_eth_req;

/* Wired-link diagnostics (eth_diag.h): refreshed every 2 s on this task, logged when an error
   counter moves, printed with every "no lease" line, and read by `eth stats`. */
#define ETH_DIAG_PERIOD_MS    2000u
#define ETH_DIAG_LOG_GAP_MS  10000u
static eth_diag_t s_eth_diag, s_eth_diag_logged;
static uint32_t   s_eth_diag_timer, s_eth_err_log_ms;
static bool       s_eth_diag_primed, s_eth_pool_stall_logged;

static void eth_diag_print(void)
{
    char line[384];
    eth_diag_format(&s_eth_diag, line, sizeof(line));
    log_printf("[net] eth diag: %s\r\n", line);
}

static void eth_diag_poll(struct netif *netif, uint32_t now)
{
    if (now - s_eth_diag_timer < ETH_DIAG_PERIOD_MS) return;
    s_eth_diag_timer = now;
    bool was_stuck = s_eth_diag.rx_alloc_stuck;
    ethernetif_diag_refresh(netif, &s_eth_diag);
    if (!s_eth_diag_primed) {
        s_eth_diag_logged = s_eth_diag;
        s_eth_diag_primed = true;
        return;
    }
    char delta[160];
    if (eth_diag_error_delta(&s_eth_diag_logged, &s_eth_diag, delta, sizeof(delta)) > 0 &&
        now - s_eth_err_log_ms >= ETH_DIAG_LOG_GAP_MS) {
        s_eth_err_log_ms = now;
        s_eth_diag_logged = s_eth_diag;
        log_printf("[net] eth errors: %s (phy %s, mac %s)\r\n", delta,
               eth_diag_phy_mode(s_eth_diag.physcsr), eth_diag_mac_mode(s_eth_diag.maccr));
    }
    if (was_stuck && s_eth_diag.rx_alloc_stuck && !s_eth_pool_stall_logged) {
        s_eth_pool_stall_logged = true;
        log_printf("[net] eth receive stalled: RX buffer pool empty for over %u ms\r\n",
               (unsigned)ETH_DIAG_PERIOD_MS);
    } else if (!s_eth_diag.rx_alloc_stuck) {
        s_eth_pool_stall_logged = false;
    }
}

void net_eth_diag(eth_diag_t *out) { *out = s_eth_diag; }
static bool               s_eth_admin_down;  /* held down by an explicit `eth stop` */
static int                s_eth_force_mbit;  /* `eth speed`: 0 = autoneg, else 10 or 100 */
static int                s_eth_force_full;  /* `eth speed`: duplex when forced */
/* `eth refclk` result: the net task measures, the caller polls net_eth_refclk_result().
   seq increments on every completed measurement so a caller can tell a fresh one from
   the previous answer. */
static volatile uint32_t  s_eth_refclk_seq;
static volatile uint32_t  s_eth_refclk_hz;
/* `eth loopback`: same request/seq pattern as refclk. */
static int                   s_eth_lb_mbit;
static uint32_t              s_eth_lb_n;
static volatile uint32_t     s_eth_lb_seq;
static eth_loopback_result_t s_eth_lb_result;

static bool iface_addressed(const net_if_t *i) {
    return i->dhcp.state == NET_DHCP_DONE;
}

/* ---- at_driver transmit seam ---------------------------------------------
   Called by command_handler / scpi_server ON THE WORKER TASK.  Because lwIP is
   single-threaded on the net task, replies are NOT written to lwIP here — they go
   into the per-connection conn_tx rings (or the cloud reply capture) and the net
   task drains them (net_tx_drain). */
int at_send_data(int conn_id, const uint8_t *buf, size_t len)
{
    /* Cloud command channel: capture the reply (worker-local buffer) so it can be
       framed as a command.response once the worker finishes the command. */
    if (conn_id == CH_CLOUD_CONN) {
        command_handler_cloud_capture_append(buf, len);
        return 0;
    }
#if defined(BENCHPOD_EMI_TEST)
    if (conn_id == CH_EMI_CONN) { emi_test_reply(buf, len); return 0; }
#endif
    /* Real TCP conns and cloud tunnels: enqueue into the conn's TX ring; the net
       task moves it into lwIP.  All or nothing (conn_tx_write): when the ring cannot
       take the whole buffer nothing is queued and this fails, so the peer never sees a
       torn line or frame.  Callers that may meet a full ring check at_send_avail()
       first (bulk senders, DAP, UART proxy); the others close the stuck connection. */
    if (conn_tx_slot(conn_id) >= 0)
        return (conn_tx_write(conn_id, buf, len) == len) ? 0 : -1;
    return -1;
}

size_t at_send_avail(int conn_id)
{
    if (conn_id == CH_CLOUD_CONN)   return 0xFFFFu;   /* captured to RAM, framed once */
#if defined(BENCHPOD_EMI_TEST)
    if (conn_id == CH_EMI_CONN)     return emi_test_avail();
#endif
    /* Real TCP conns and cloud tunnels: pace against the ring free space (the net
       task, in turn, only drains a tunnel ring as fast as the TLS/WS send buffer
       allows — so bulk_pump can never overrun the wire).  This ring-based pacing
       replaced reading tcp_sndbuf / cloud_client_tunnel_avail directly, which is
       lwIP state the worker task must not touch. */
    if (conn_tx_slot(conn_id) >= 0) return conn_tx_free(conn_id);
    return 0;
}

/* Worker-side request to close a connection: flag it; the net task performs the
   actual lwIP close in net_poll and then notifies the worker (WK_CLOSED). */
int at_close_connection(int conn_id)
{
    if (conn_id < 0 || conn_id >= NET_MAX_CONN) return -1;
    s_close_req[conn_id] = 1;
    return 0;
}

/* Worker-side request to toggle Nagle: record the desired state; net applies it. */
int at_set_tcp_nodelay(int conn_id, bool enable)
{
    if (conn_id < 0 || conn_id >= NET_MAX_CONN) return -1;
    s_nodelay_req[conn_id] = enable ? 1 : 0;
    return 0;
}

/* ---- connectivity-status seam (reports the active/default interface) ---- */
wifi_state_t wifi_get_state(void)
{
    return (iface_addressed(&s_eth) || iface_addressed(&s_wifi)) ? WIFI_READY
                                                                 : WIFI_DISCONNECTED;
}
const char *wifi_get_ip(void)
{
    if (iface_addressed(&s_eth))  return s_eth.ip;   /* Ethernet preferred */
    if (iface_addressed(&s_wifi)) return s_wifi.ip;
    return "0.0.0.0";
}

const char *wifi_sta_ip(void)
{
    return iface_addressed(&s_wifi) ? s_wifi.ip : "(none)";
}
int wifi_get_rssi(int *out_rssi_dbm) { if (out_rssi_dbm) *out_rssi_dbm = -1; return -1; }

/* ---- raw TCP server callbacks ------------------------------------------ */
static int conn_id_of(void *arg) { return (int)(intptr_t)arg - 1; }

/* The slot no longer has a connection: drop its TLS configuration reference (lan_tls_collect
   frees a retired one later, after lwIP has freed the TLS state that points at it). */
static void conn_detach(int id)
{
    conn_pcb[id] = NULL;
    s_conn_refused[id] = false;
    if (s_conn_tls[id]) { lan_tls_unref(s_conn_tls[id]); s_conn_tls[id] = NULL; }
}

static void conn_unhook(struct altcp_pcb *pcb)
{
    altcp_arg(pcb, NULL); altcp_recv(pcb, NULL); altcp_err(pcb, NULL);
}

static err_t srv_recv(void *arg, struct altcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    int id = conn_id_of(arg);

    if (p == NULL) {                          /* remote closed */
        if (id >= 0 && id < NET_MAX_CONN) { hw_worker_submit_closed(id); conn_detach(id); }
        conn_unhook(tpcb);
        if (altcp_close(tpcb) != ERR_OK) altcp_abort(tpcb);
        return ERR_OK;
    }
    if (err != ERR_OK) { pbuf_free(p); return err; }

    /* Hand the inbound bytes to the worker task (it owns command_handler).
       ALL-OR-NOTHING: only consume this pbuf if the WHOLE thing fits the worker
       queue right now; otherwise return ERR_MEM without consuming or freeing it, so
       lwIP keeps the pbuf and redelivers it later (tcp_fasttmr retries refused data
       ~every 250 ms; any later inbound segment retries it sooner) once the worker
       has drained.  The queue is sized so a full receive window always fits when
       drained (see the _Static_assert above), so this never deadlocks.

       This replaces an earlier path that consumed a prefix and then DROPPED the
       rest when the queue filled mid-pbuf — which silently TRUNCATED a fast bulk
       upload (e.g. a >=32 KB deep-replay `load_bin{psram}`): the received byte
       count never reached `total`, so the completion ack never fired and the client
       timed out.  See docs / the concurrent-capture hwe2e tests. */
    if (id >= 0 && id < NET_MAX_CONN) {
        if (!hw_worker_can_accept(p->tot_len)) { s_conn_refused[id] = true; return ERR_MEM; }

        static uint8_t seg[HW_WORK_DATA_MAX];   /* net task only (single-threaded) */
        u16_t off = 0;
        while (off < p->tot_len) {
            u16_t n = (u16_t)(p->tot_len - off);
            if (n > sizeof(seg)) n = sizeof(seg);
            pbuf_copy_partial(p, seg, n, off);
            /* can_accept() reserved room, so this succeeds on the common path.  If
               another producer task (console) raced a slot away in between, wait
               briefly for the worker to drain instead of truncating the tail — the
               pbuf is already partly consumed and can't be rewound for redelivery. */
            int spins = 0;
            while (!hw_worker_submit_bytes(id, seg, n)) {
                vTaskDelay(pdMS_TO_TICKS(5));
                if (++spins >= 100) {            /* ~0.5 s: worker wedged (should never) */
                    /* We already consumed `off` bytes of this pbuf into the worker and
                       cannot rewind, so acking a prefix and dropping the tail would
                       SILENTLY TRUNCATE the stream — corrupting a deep-replay/bulk
                       upload exactly like the pre-all-or-nothing bug. Abort the
                       connection instead: the client sees a reset and can retry
                       cleanly rather than receiving a corrupt trace. */
                    log_printf("[net] conn %d: worker wedged mid-pbuf after %u/%u bytes — "
                           "aborting to avoid silent truncation\n",
                           id, (unsigned)off, (unsigned)p->tot_len);
                    hw_worker_submit_closed(id);
                    conn_detach(id);
                    conn_unhook(tpcb);
                    pbuf_free(p);
                    altcp_abort(tpcb);
                    return ERR_ABRT;
                }
            }
            off += n;
        }
        altcp_recved(tpcb, p->tot_len);
        pbuf_free(p);
        return ERR_OK;
    }
    altcp_recved(tpcb, p->tot_len);
    pbuf_free(p);
    return ERR_OK;
}

static void srv_err(void *arg, err_t err)
{
    (void)err;
    int id = conn_id_of(arg);
    if (id >= 0 && id < NET_MAX_CONN) { hw_worker_submit_closed(id); conn_detach(id); }
}

static int tls_sessions(void)
{
    int n = 0;
    for (int i = 0; i < NET_MAX_CONN; i++) if (s_conn_tls[i]) n++;
    return n;
}

/* arg: the listener's TLS configuration, NULL for the plaintext listener. */
static err_t srv_accept(void *arg, struct altcp_pcb *newpcb, err_t err)
{
    struct altcp_tls_config *tls = arg;
    if (err != ERR_OK || newpcb == NULL) return ERR_VAL;

    int id = -1;
    /* Not a slot whose previous connection's close the worker has not handled yet: its state
       (upload mode, SWD session, busy gate) and late replies still belong to that client. */
    for (int i = 0; i < NET_MAX_CONN; i++) {
        if (conn_pcb[i] == NULL && !hw_worker_conn_busy(i)) { id = i; break; }
    }
    if (id < 0) { altcp_abort(newpcb); return ERR_ABRT; }   /* too many clients */

    /* The LAN policy is for the external LAN (Ethernet, Wi-Fi). The USB network adapter means
       physical presence: it stays reachable under "off" too (pod_policy.h). So does plaintext
       under HTTPS-only: the USB link never crosses a network. */
    bool usb = ip4_addr_eq(ip_2_ip4(altcp_get_ip(newpcb, 1)), netif_ip4_addr(&s_usb_netif));
    if (!usb && !s_lan_up) { altcp_abort(newpcb); return ERR_ABRT; }
    if (!tls && !usb && s_https_only) { altcp_abort(newpcb); return ERR_ABRT; }
    if (tls && (tls_sessions() >= LAN_TLS_MAX_SESSIONS || xPortGetFreeHeapSize() < LAN_TLS_HEAP_FLOOR)) {
        log_printf("[net] TLS client refused: %d sessions, %u B heap free\n", tls_sessions(),
                   (unsigned)xPortGetFreeHeapSize());
        altcp_abort(newpcb);
        return ERR_ABRT;
    }

    conn_tx_reset(id);           /* clear any stale bytes from a prior conn on this slot */
    s_close_req[id]   = 0;
    s_nodelay_req[id] = -1;
    conn_pcb[id] = newpcb;
    s_conn_usb[id] = usb;
    s_conn_refused[id] = false;
    s_conn_tls[id] = tls;
    if (tls) lan_tls_ref(tls);
    altcp_setprio(newpcb, TCP_PRIO_MIN);
    /* a dead idle client frees its slot (~50 s) */
    altcp_keepalive_enable(newpcb, TCP_KEEPIDLE_DEFAULT, TCP_KEEPINTVL_DEFAULT, TCP_KEEPCNT_DEFAULT);
    altcp_arg(newpcb, (void *)(intptr_t)(id + 1));
    altcp_recv(newpcb, srv_recv);
    altcp_err(newpcb, srv_err);
    return ERR_OK;
}

/* Drain the per-connection TX rings the worker filled: real TCP conns straight
   into lwIP (bounded by tcp_sndbuf), cloud tunnels via the WS framing (bounded by
   the TLS/WS send buffer).  Net task only. */
static void net_tx_drain(void)
{
    for (int id = 0; id < NET_MAX_CONN; id++) {
        struct altcp_pcb *pcb = conn_pcb[id];
        if (!pcb) continue;
        if (hw_worker_conn_busy(id)) continue;   /* (cannot be: accept skips busy slots) */
        bool wrote = false;
        const uint8_t *chunk;
        size_t n;
        while ((n = conn_tx_peek(id, &chunk)) > 0) {
            u16_t sb = altcp_sndbuf(pcb);
            if (sb == 0) break;
            if (n > sb) n = sb;
            if (altcp_write(pcb, chunk, (u16_t)n, TCP_WRITE_FLAG_COPY) != ERR_OK) break;
            conn_tx_advance(id, n);
            wrote = true;
        }
        if (wrote) altcp_output(pcb);
        /* A TLS connection whose plaintext srv_recv refused: hand it back once there is room
           (plaintext TCP retries by itself on the next segment or tcp_fasttmr). */
        if (s_conn_refused[id] && s_conn_tls[id] && hw_worker_can_accept(HW_WORK_DATA_MAX)) {
            s_conn_refused[id] = false;
            altcp_tls_bp_kick(pcb);
        }
    }
    for (int t = 0; t < CH_CLOUD_TUNNEL_CONN_COUNT; t++) {
        int id = CH_CLOUD_TUNNEL_CONN + t;
        size_t used = conn_tx_used(id);
        if (!used) continue;
        const uint8_t *chunk;
        if (hw_worker_conn_busy(id)) {
            /* The previous session's reset has not reached the worker yet: whatever is in the
               ring is that session's late output, not for the tunnel now on this slot. */
            size_t n = conn_tx_peek(id, &chunk);
            conn_tx_advance(id, n);
            continue;
        }
        size_t avail = cloud_client_tunnel_avail(id);   /* raw bytes the WS buffer can take now */
        if (avail == 0) continue;
        size_t n = conn_tx_peek(id, &chunk);
        if (n > avail) n = avail;
        conn_tx_advance(id, cloud_client_tunnel_out(id, chunk, n));   /* only what went out */
    }
}

/* Apply the transport-control actions the worker requested (it can't touch lwIP):
   toggle Nagle, and close sockets (then tell the worker to clean up its state). */
static void net_apply_worker_requests(void)
{
    for (int id = 0; id < NET_MAX_CONN; id++) {
        if (s_nodelay_req[id] >= 0) {
            if (conn_pcb[id]) {
                if (s_nodelay_req[id]) altcp_nagle_disable(conn_pcb[id]);
                else                   altcp_nagle_enable(conn_pcb[id]);
            }
            s_nodelay_req[id] = -1;
        }
        if (s_close_req[id]) {
            s_close_req[id] = 0;
            if (conn_pcb[id]) {
                struct altcp_pcb *pcb = conn_pcb[id];
                conn_detach(id);
                conn_unhook(pcb);
                if (altcp_close(pcb) != ERR_OK) altcp_abort(pcb);
            }
            conn_tx_reset(id);
            hw_worker_submit_closed(id);   /* worker releases gate/stream state */
        }
    }
}

/* If the worker finished a cloud command.request, frame + send its reply. */
static void net_frame_cloud_reply(void)
{
    char        req_id[BP_TUNNEL_ID_MAX];
    static char reply[BP_CLOUD_REPLY_MAX];   /* net task only */
    size_t rl = 0;
    if (hw_worker_take_cloud_reply(req_id, sizeof(req_id), reply, sizeof(reply), &rl))
        cloud_client_send_command_response(req_id, reply, rl);
}

static struct altcp_pcb *s_listen;       /* :8080 (srv_accept applies the LAN policy) */
static struct altcp_pcb *s_tls_listen;   /* :8443, with s_tls_listen_cfg */
static struct altcp_tls_config *s_tls_listen_cfg;
static uint32_t s_tls_listen_gen;

static struct altcp_pcb *listen_on(struct altcp_pcb *pcb, u16_t port, void *arg)
{
    if (pcb == NULL) return NULL;
    if (altcp_bind(pcb, IP_ANY_TYPE, port) != ERR_OK) { altcp_close(pcb); return NULL; }
    struct altcp_pcb *l = altcp_listen(pcb);
    if (l == NULL) { altcp_close(pcb); return NULL; }
    altcp_arg(l, arg);
    altcp_accept(l, srv_accept);
    return l;
}

/* Start the :8443 listener, or give it the current certificate. Open TLS sessions keep the
   configuration they started with (lan_tls_ref). The listener stays open across a change: closing
   and binding :8443 again fails in lwIP while a connection (say, the one that installed the new
   certificate) still has that local port, and the port would then go dark. */
static void tls_listener_sync(void)
{
    struct altcp_tls_config *cfg = lan_tls_config();
    s_tls_listen_gen = lan_tls_generation();
    if (s_tls_listen && s_tls_listen_cfg == cfg) return;
    if (s_tls_listen && cfg) {
        altcp_tls_bp_set_listen_config(s_tls_listen, cfg);
        altcp_arg(s_tls_listen, cfg);   /* srv_accept's arg: the configuration accepted conns hold */
        lan_tls_ref(cfg);
        lan_tls_unref(s_tls_listen_cfg);
        s_tls_listen_cfg = cfg;
        log_printf("[net] :%d now serves the new certificate\n", LAN_TLS_PORT);
        return;
    }
    if (s_tls_listen) {
        altcp_accept(s_tls_listen, NULL);
        altcp_close(s_tls_listen);
        s_tls_listen = NULL;
    }
    if (s_tls_listen_cfg) { lan_tls_unref(s_tls_listen_cfg); s_tls_listen_cfg = NULL; }
    if (!cfg) return;
    s_tls_listen = listen_on(altcp_tls_new(cfg, IPADDR_TYPE_ANY), LAN_TLS_PORT, cfg);
    if (s_tls_listen) {
        s_tls_listen_cfg = cfg;
        lan_tls_ref(cfg);
    } else {
        log_printf("[net] could not listen on :%d\n", LAN_TLS_PORT);
    }
}

static void server_start(void)
{
    if (!s_listen) s_listen = listen_on(altcp_tcp_new_ip_type(IPADDR_TYPE_ANY), TCP_SERVER_PORT, NULL);
    tls_listener_sync();
}

/* ---- DHCP management (per interface) ----------------------------------- */
/* Drive the pure net_dhcp_step() state machine and perform the side effect it
   asks for. Link state is POLLED here (netif_is_link_up) every ~500 ms tick, so
   there is no dependence on a link-change callback edge being delivered — a
   missed edge can no longer strand us at 0.0.0.0 (net_dhcp self-heals from OFF
   when the link is up). */
static void dhcp_process(net_if_t *i)
{
    struct netif *netif = &i->netif;
    const char *tag = i->is_eth ? "eth" : "wifi";

    bool link_up   = netif_is_link_up(netif);
    bool has_lease = dhcp_supplied_address(netif);
    bool ip_changed = has_lease &&
                      strcmp(ip4addr_ntoa(netif_ip4_addr(netif)), i->ip) != 0;

    switch (net_dhcp_step(&i->dhcp, link_up, has_lease, ip_changed, DHCP_REKICK_TICKS)) {
    case NET_DHCP_DO_DISCOVER:
        ip_addr_set_zero_ip4(&netif->ip_addr);
        ip_addr_set_zero_ip4(&netif->netmask);
        ip_addr_set_zero_ip4(&netif->gw);
        dhcp_start(netif);
        log_printf("[net] %s DHCP: requesting address...\r\n", tag);
        break;
    case NET_DHCP_DO_REKICK:
        /* No static fallback: keep retrying forever so a slow/awkward home router
           is tolerated. lwIP retransmits DISCOVER on its own fine timer; this is
           a coarse backstop that forces a fresh DISCOVER if a lease still hasn't
           arrived after ~20 s (e.g. after the router finishes rebooting).

           Escalation (wired only): if several re-kicks in a row still yield no
           lease, re-sending DISCOVER clearly isn't the problem — the MAC/PHY or
           autoneg is likely wedged (a state a power cycle "fixes"). Do a PHY soft
           reset to re-establish the link from scratch, then re-acquire. This is
           the self-heal for a boot that got stuck at "requesting address" forever. */
        if (i->is_eth && ++i->dhcp_fails >= DHCP_ESCALATE_REKICKS) {
            i->dhcp_fails = 0;
            i->dhcp.state = NET_DHCP_OFF;   /* re-DISCOVER once the link is back up */
            i->dhcp.wait_ticks = 0;
            ethernetif_phy_restart(netif);
            log_printf("[net] %s DHCP: no lease after %d re-kicks — PHY reset + re-acquire\r\n",
                   tag, DHCP_ESCALATE_REKICKS);
            eth_diag_print();   /* the state that led to the reset */
            break;
        }
        dhcp_start(netif);
        log_printf("[net] %s DHCP: still no lease, re-probing...\r\n", tag);
        if (i->is_eth) eth_diag_print();
        break;
    case NET_DHCP_DO_BOUND:
        i->dhcp_fails = 0;
        strncpy(i->ip, ip4addr_ntoa(netif_ip4_addr(netif)), sizeof(i->ip) - 1);
        mdns_resp_announce(netif);   /* broadcast the now-valid A record */
        log_printf("[net] %s DHCP: ip=%s\r\n", tag, i->ip);
        break;
    case NET_DHCP_DO_IPCHANGE:
        /* Bound but the router handed out a different address on renew/rebind. */
        strncpy(i->ip, ip4addr_ntoa(netif_ip4_addr(netif)), sizeof(i->ip) - 1);
        mdns_resp_announce(netif);   /* re-broadcast the new A record */
        log_printf("[net] %s DHCP: address changed -> %s\r\n", tag, i->ip);
        break;
    case NET_DHCP_DO_LINKDOWN:
        strcpy(i->ip, "0.0.0.0");
        log_printf("[net] %s link down\r\n", tag);
        break;
    case NET_DHCP_DO_NOTHING:
    default:
        break;
    }
}

/* Ethernet-priority default route: wired when it has an address, else Wi-Fi when
   it is associated + addressed. Outbound traffic (cloud_client) follows this. */
static void update_default_route(void)
{
    struct netif *want = &s_eth.netif;   /* default to wired */
    if (iface_addressed(&s_eth) && netif_is_link_up(&s_eth.netif)) {
        want = &s_eth.netif;
    } else if (iface_addressed(&s_wifi) && netif_is_link_up(&s_wifi.netif)) {
        want = &s_wifi.netif;
    }
    if (netif_default != want) {
        netif_set_default(want);
        log_printf("[net] default route -> %s\r\n", (want == &s_eth.netif) ? "eth" : "wifi");
    }
}

/* LWIP_HOOK_IP4_ROUTE_SRC (config/lwipopts.h): pick the outbound netif by source
   address, then eth-first on a shared subnet. Mirrors update_default_route()'s
   priority; returning NULL lets lwIP fall back to its subnet scan + default route.
   src is NULL when lwIP calls this after that scan found nothing. */
static void route_if_fill(net_route_if_t *r, const struct netif *n)
{
    r->ip     = ip4_addr_get_u32(netif_ip4_addr(n));
    r->mask   = ip4_addr_get_u32(netif_ip4_netmask(n));
    r->usable = netif_is_up(n) && netif_is_link_up(n) && r->ip != 0;
}

const char *net_uplink_name(uint32_t local_ip)
{
    net_route_if_t ifs[3];
    route_if_fill(&ifs[0], &s_eth.netif);
    route_if_fill(&ifs[1], &s_wifi.netif);
    route_if_fill(&ifs[2], &s_usb_netif);   /* through `benchpod usb-bridge` on the computer */
    switch (net_route_owner(ifs, 3, local_ip)) {
    case 0:  return "eth";
    case 1:  return "wifi";
    case 2:  return "usb";
    default: return "unknown";
    }
}

struct netif *net_route_src_hook(const ip4_addr_t *src, const ip4_addr_t *dest)
{
    struct netif *order[3] = { &s_eth.netif, &s_wifi.netif, &s_usb_netif };   /* priority */
    net_route_if_t ifs[3];
    for (int k = 0; k < 3; k++) route_if_fill(&ifs[k], order[k]);
    int i = net_route_pick(ifs, 3, src ? ip4_addr_get_u32(src) : 0, ip4_addr_get_u32(dest));
    return (i < 0) ? NULL : order[i];
}

/* Clear the wired netif's address + DHCP state so acquisition restarts clean. */
static void eth_reset_addr_state(void)
{
    dhcp_stop(&s_eth.netif);
    ip_addr_set_zero_ip4(&s_eth.netif.ip_addr);
    ip_addr_set_zero_ip4(&s_eth.netif.netmask);
    ip_addr_set_zero_ip4(&s_eth.netif.gw);
    s_eth.dhcp.state   = NET_DHCP_OFF;   /* dhcp_process re-DISCOVERs when the link is up */
    s_eth.dhcp.wait_ticks = 0;
    s_eth.dhcp_fails   = 0;
    strcpy(s_eth.ip, "0.0.0.0");
}

/* Apply a latched `eth` stop/start/restart request. Runs on the net task only
   (net_poll), so it can touch HAL_ETH + lwIP directly. */
static void net_eth_apply_request(void)
{
    eth_req_t req = s_eth_req;
    if (req == ETH_REQ_NONE) return;
    s_eth_req = ETH_REQ_NONE;

    switch (req) {
    case ETH_REQ_DOWN:
        s_eth_admin_down = true;
        ethernetif_stop(&s_eth.netif);
        eth_reset_addr_state();
        update_default_route();          /* fail over to Wi-Fi if it's up */
        log_printf("[net] eth: stopped (admin down)\r\n");
        break;
    case ETH_REQ_SPEED:
        s_eth_admin_down = false;
        eth_reset_addr_state();
        if (ethernetif_force_speed(&s_eth.netif, s_eth_force_mbit, s_eth_force_full) != 0) {
            log_printf("[net] eth: forcing the link mode failed (MDIO write)\r\n");
            break;
        }
        if (s_eth_force_mbit == 0) {
            log_printf("[net] eth: autonegotiation restored, re-acquiring link + DHCP\r\n");
        } else {
            log_printf("[net] eth: link forced to %d M %s, re-acquiring link + DHCP%s\r\n",
                   s_eth_force_mbit, s_eth_force_full ? "full" : "half",
                   s_eth_force_full ? " (a switch port that autonegotiates falls back to HALF:"
                                      " expect a duplex mismatch)" : "");
        }
        break;
    case ETH_REQ_REFCLK: {
        uint32_t hz = 0;
        s_eth_admin_down = false;
        eth_reset_addr_state();
        (void)ethernetif_measure_refclk(&s_eth.netif, &hz);
        s_eth_refclk_hz = hz;
        s_eth_refclk_seq++;
        if (hz == 0) {
            log_printf("[net] eth refclk: no edges on PA1 — the PHY is not driving the RMII clock\r\n");
        } else {
            long ppm = ((long long)hz - 50000000LL) * 1000000LL / 50000000LL;
            log_printf("[net] eth refclk: %lu Hz (%+ld ppm vs 50 MHz, measured against the MCU crystal)\r\n",
                   (unsigned long)hz, ppm);
        }
        break;
    }
    case ETH_REQ_LOOPBACK: {
        eth_loopback_result_t r;
        s_eth_admin_down = false;
        eth_reset_addr_state();
        if (ethernetif_loopback_test(&s_eth.netif, s_eth_lb_mbit, s_eth_lb_n, &r) != 0)
            log_printf("[net] eth loopback: could not put the PHY in loopback (MDIO write)\r\n");
        else
            log_printf("[net] eth loopback %dM: sent %lu, back %lu, intact %lu, corrupt %lu, "
                   "crc %lu, align %lu, tx fail %lu\r\n",
                   r.mbit, (unsigned long)r.sent, (unsigned long)r.received,
                   (unsigned long)r.intact, (unsigned long)r.corrupt,
                   (unsigned long)r.crc, (unsigned long)r.align, (unsigned long)r.tx_fail);
        s_eth_lb_result = r;
        s_eth_lb_seq++;
        break;
    }
    case ETH_REQ_UP:
    case ETH_REQ_RESTART:
        s_eth_admin_down = false;
        eth_reset_addr_state();
        ethernetif_phy_restart(&s_eth.netif);   /* link re-negotiates; check_state re-ups */
        log_printf("[net] eth: %s — PHY reset, re-acquiring link + DHCP\r\n",
               req == ETH_REQ_UP ? "started" : "restarted");
        break;
    default:
        break;
    }
}

/* Public control seam (see net_server.h). Callable from any task — these only
   latch a request; net_poll() applies it on the net task. */
void net_eth_stop(void)    { s_eth_req = ETH_REQ_DOWN; }
void net_eth_start(void)   { s_eth_req = ETH_REQ_UP; }
void net_eth_restart(void) { s_eth_req = ETH_REQ_RESTART; }
/* Ask for a reference-clock measurement, then poll net_eth_refclk_result(): it returns
   true once seq has moved past the value captured before the request. */
void net_eth_refclk_measure(void) { s_eth_req = ETH_REQ_REFCLK; }
uint32_t net_eth_refclk_seq(void) { return s_eth_refclk_seq; }
bool net_eth_refclk_result(uint32_t since_seq, uint32_t *hz_out)
{
    if (s_eth_refclk_seq == since_seq) return false;
    if (hz_out) *hz_out = s_eth_refclk_hz;
    return true;
}

void net_eth_loopback(int mbit, uint32_t n)
{
    s_eth_lb_mbit = mbit;
    s_eth_lb_n    = n;
    s_eth_req     = ETH_REQ_LOOPBACK;
}
uint32_t net_eth_loopback_seq(void) { return s_eth_lb_seq; }
bool net_eth_loopback_result(uint32_t since_seq, eth_loopback_result_t *out)
{
    if (s_eth_lb_seq == since_seq) return false;
    if (out) *out = s_eth_lb_result;
    return true;
}

void net_eth_force_speed(int mbit, int full)
{
    s_eth_force_mbit = mbit;
    s_eth_force_full = full;
    s_eth_req = ETH_REQ_SPEED;
}

/* ---- mDNS / DNS-SD responder ------------------------------------------------
   Advertises the pod as "_benchpod._tcp" so a client can browse the LAN and
   enumerate every pod at once (one PTR -> N instances, each with its own A
   record + TXT). The hostname and DNS-SD instance name are made unique from a
   slice of the device's Ed25519 public key, and the full public key travels in
   the TXT record as the stable, collision-proof identifier. The same hostname
   is registered on both netifs, so a multihomed pod shows up once and resolves
   to whichever interface answered. ---------------------------------------- */
static char s_mdns_host[24];   /* "benchpod-a1b2c3"               */
static char s_mdns_inst[32];   /* "BenchPod a1b2c3" (instance)    */
static char s_mdns_id[B64URL_ENCODED_LEN(DEVICE_ID_PUBLIC_LEN) + 1]; /* full pubkey */

static void mdns_build_identity(void)
{
    if (s_mdns_host[0]) return;                 /* build once */
    uint8_t pub[DEVICE_ID_PUBLIC_LEN] = {0};
    device_identity_get_public(pub);            /* leaves zeros if id not ready */

    /* 3-byte slice -> 6 hex chars: a 16M-instance space, ample for a bench LAN. */
    snprintf(s_mdns_host, sizeof(s_mdns_host), "benchpod-%02x%02x%02x",
             pub[0], pub[1], pub[2]);
    snprintf(s_mdns_inst, sizeof(s_mdns_inst), "BenchPod %02x%02x%02x",
             pub[0], pub[1], pub[2]);
    b64url_encode(pub, sizeof(pub), s_mdns_id, sizeof(s_mdns_id));
}

/* TXT builder: clients read these to tell pods apart without connecting. */
static void mdns_txt(struct mdns_service *svc, void *arg)
{
    (void)arg;
    char kv[B64URL_ENCODED_LEN(DEVICE_ID_PUBLIC_LEN) + 8];
    int n;
    n = snprintf(kv, sizeof(kv), "id=%s",   s_mdns_id);       mdns_resp_add_service_txtitem(svc, kv, n);
    n = snprintf(kv, sizeof(kv), "host=%s", s_mdns_host);     mdns_resp_add_service_txtitem(svc, kv, n);
    n = snprintf(kv, sizeof(kv), "port=%d", TCP_SERVER_PORT); mdns_resp_add_service_txtitem(svc, kv, n);
    if (s_tls_listen) {
        n = snprintf(kv, sizeof(kv), "tls=%d", LAN_TLS_PORT);    mdns_resp_add_service_txtitem(svc, kv, n);
    }
    n = snprintf(kv, sizeof(kv), "https_only=%d", s_https_only ? 1 : 0);
    mdns_resp_add_service_txtitem(svc, kv, n);
}

static void mdns_register(struct netif *netif)
{
    mdns_build_identity();
    if (mdns_resp_add_netif(netif, s_mdns_host) != ERR_OK) return;
    mdns_resp_add_service(netif, s_mdns_inst, "_benchpod", DNSSD_PROTO_TCP,
                          TCP_SERVER_PORT, mdns_txt, NULL);
    /* The same :8080 socket speaks SCPI: VISA / LXI discovery finds the pod as a raw socket
       instrument (TCPIP::<host>::8080::SOCKET). */
    mdns_resp_add_service(netif, s_mdns_inst, "_scpi-raw", DNSSD_PROTO_TCP,
                          TCP_SERVER_PORT, NULL, NULL);
}

/* LAN policy "off" (pod_policy.h): the external LAN (Ethernet, Wi-Fi) gets no :8080 connections
   and no mDNS advertisement; the cloud link and USB (console and network adapter) are untouched.
   The worker asks (pod_policy_on_lan_change); the net task applies it, since it owns lwIP. */
static volatile int8_t s_lan_req = -1;   /* -1 none, 0 = stop LAN access, 1 = start it */

static void lan_access(bool up)
{
    if (up == s_lan_up) return;
    s_lan_up = up;
    if (up) {
        mdns_register(&s_eth.netif);
        mdns_register(&s_wifi.netif);
        if (iface_addressed(&s_eth))  mdns_resp_announce(&s_eth.netif);
        if (iface_addressed(&s_wifi)) mdns_resp_announce(&s_wifi.netif);
    } else {
        mdns_resp_remove_netif(&s_eth.netif);
        mdns_resp_remove_netif(&s_wifi.netif);
        for (int id = 0; id < NET_MAX_CONN; id++)
            if (conn_pcb[id] && !s_conn_usb[id]) s_close_req[id] = 1;   /* net_apply_worker_requests */
    }
    log_printf("[net] external LAN access %s\r\n", up ? "on" : "off (policy; USB stays on)");
}

bool net_conn_on_usb(int conn_id)
{
    return conn_id >= 0 && conn_id < NET_MAX_CONN && conn_pcb[conn_id] && s_conn_usb[conn_id];
}

bool net_conn_on_tls(int conn_id)
{
    return conn_id >= 0 && conn_id < NET_MAX_CONN && conn_pcb[conn_id] && s_conn_tls[conn_id];
}

int net_tls_sessions(void) { return tls_sessions(); }

void pod_policy_on_lan_change(pod_lan_policy_t now) { s_lan_req = (now == POD_LAN_OFF) ? 0 : 1; }
void pod_policy_on_https_only_change(bool on) { s_https_req = on ? 1 : 0; }

/* HTTPS-only (pod_policy.h): plaintext :8080 connections from the external LAN close and new
   ones are refused (srv_accept). The USB link keeps plaintext. Net task. */
static void https_only_apply(bool on)
{
    if (on == s_https_only) return;
    s_https_only = on;
    if (on)
        for (int id = 0; id < NET_MAX_CONN; id++)
            if (conn_pcb[id] && !s_conn_usb[id] && !s_conn_tls[id]) s_close_req[id] = 1;
    /* The TXT record carries the setting: tell the LAN. */
    if (s_lan_up) {
        if (iface_addressed(&s_eth))  mdns_resp_announce(&s_eth.netif);
        if (iface_addressed(&s_wifi)) mdns_resp_announce(&s_wifi.netif);
    }
    log_printf("[net] HTTPS-only %s: plaintext :%d %s\r\n", on ? "on" : "off", TCP_SERVER_PORT,
               on ? "answers on USB only" : "open again");
}

/* ---- public API -------------------------------------------------------- */
void net_init(void)
{
    ip_addr_t zero;
    ip_addr_set_zero_ip4(&zero);

    s_net_task = xTaskGetCurrentTaskHandle();                         /* net_call_sync */
    for (int i = 0; i < NET_MAX_CONN; i++) s_nodelay_req[i] = -1;   /* -1 = no pending toggle */

    /* Route mbedTLS allocation to the FreeRTOS heap before any TLS work (the
       outbound WSS cloud client uses it). */
    mbedtls_port_init();

    lwip_init();
    mdns_resp_init();

    /* Ethernet netif (always present, default route). dhcp_process() polls the
       link every tick (netif_is_link_up), so no link-change callback is wired. */
    netif_add(&s_eth.netif, &zero, &zero, &zero, NULL, &ethernetif_init, &ethernet_input);
    /* The ETH MAC inserts IP/UDP/TCP checksums in hardware, so leave those OFF in
       software (netif_add defaults to ENABLE_ALL — computing them in software too
       would clash with the MAC's pseudo-header insertion and corrupt UDP/TCP).
       ICMP stays software. The Wi-Fi netif (no HW) enables all — see esp_netif.c. */
    NETIF_SET_CHECKSUM_CTRL(&s_eth.netif,
        NETIF_CHECKSUM_ENABLE_ALL & ~(NETIF_CHECKSUM_GEN_IP |
                                      NETIF_CHECKSUM_GEN_UDP |
                                      NETIF_CHECKSUM_GEN_TCP));
    netif_set_default(&s_eth.netif);
#if defined(BENCHPOD_WIFI_BRIDGE)
    wifi_bridge_net_init(&s_eth.netif);   /* the Wi-Fi bridge's control channel (dev builds) */
#endif

    /* Wi-Fi netif over the ESP32-C3. The netif is registered (inert until the
       link comes up); the co-processor itself stays in reset until Wi-Fi is
       provisioned, gated inside esp_wifi_ctrl. Digital v3r3 has no C3: its EN, BOOT
       and CS nets are left unconnected, so the pins stay as they are at reset. */
    esp_wifi_ctrl_set_module_present(board_has_wifi_module());
    if (esp_wifi_ctrl_module_present()) esp_hosted_spi_init();
    netif_add(&s_wifi.netif, &zero, &zero, &zero, NULL, &esp_netif_init, &ethernet_input);
    /* netif_add() prepends, leaving Wi-Fi first in netif_list. Put eth back in
       front so lwIP's own subnet scan (ICMP errors, anything that bypasses the
       route hook) also prefers the wire. */
    if (netif_list == &s_wifi.netif && s_wifi.netif.next == &s_eth.netif) {
        s_wifi.netif.next = s_eth.netif.next;
        s_eth.netif.next  = &s_wifi.netif;
        netif_list        = &s_eth.netif;
    }
    esp_wifi_ctrl_init();
    usb_net_set_enabled(!esp_wifi_ctrl_configured());   /* see net_poll */

    /* USB network adapter (CDC-NCM): its own /30, never the default route. Appended
       after eth and Wi-Fi so lwIP's subnet scan keeps preferring those. */
    netif_add(&s_usb_netif, &zero, &zero, &zero, NULL, &usb_netif_init, &ethernet_input);
    if (netif_list == &s_usb_netif) {
        struct netif *tail = s_usb_netif.next;
        netif_list = tail;
        while (tail->next) tail = tail->next;
        tail->next = &s_usb_netif;
        s_usb_netif.next = NULL;
    }
    netif_set_up(&s_usb_netif);
    log_printf("[net] usb: pod %s, host %s\r\n", usb_netif_pod_ip(), usb_netif_host_ip());

    /* The :8080 and :8443 listeners and mDNS: always on the USB network adapter, on the external
       LAN unless the LAN policy is "off" (pod_policy.h). The TLS certificate first (generated on
       the first boot of firmware with LAN TLS). */
    s_https_only = pod_policy_https_only();
    lan_tls_init();
    server_start();
    mdns_register(&s_usb_netif);
    lan_access(pod_policy_lan() != POD_LAN_OFF);
    cloud_client_init();   /* outbound WSS control channel (if provisioned) */
    log_printf("[net] LwIP up — eth(RMII/LAN8742)%s, TCP server on :%d, TLS on :%d%s\r\n",
           esp_wifi_ctrl_module_present() ? " + wifi(ESP32-C3)" : "",
           TCP_SERVER_PORT, LAN_TLS_PORT, s_https_only ? " (HTTPS-only)" : "");
}

/* Carry out a reload a settings command requested (net_reload.h): after its reply has left the
   cloud link, or NET_RELOAD_MAX_WAIT_MS after the request if the link never drains. */
static void net_apply_reload(void)
{
    if (!net_reload_pending()) return;
    uint32_t what = net_reload_take(HAL_GetTick(), cloud_client_tx_busy());
    if (what & NET_RELOAD_CLOUD) cloud_client_reload();
    if (what & NET_RELOAD_WIFI)  esp_wifi_ctrl_reload();
}

void net_poll(void)
{
    ethernetif_input(&s_eth.netif);
    esp_hosted_spi_poll();           /* pump the Wi-Fi SPI transport (RX/TX)   */
    {
        /* USB networking and Wi-Fi are one or the other: stored Wi-Fi credentials
           (wifi-set) turn USB networking off, wifi-clear turns it back on. */
        bool usb_on = !esp_wifi_ctrl_configured();
        if (usb_on != usb_net_enabled()) {
            usb_net_set_enabled(usb_on);
            log_printf("[net] usb networking %s\r\n", usb_on ? "on (no Wi-Fi configured)" : "off (Wi-Fi configured)");
        }
        bool was_up = netif_is_link_up(&s_usb_netif);
        usb_netif_poll(&s_usb_netif);    /* USB CDC-NCM frames, link, DHCP server */
        bool is_up = netif_is_link_up(&s_usb_netif);
        if (was_up != is_up) {
            log_printf("[net] usb link %s\r\n", is_up ? "up" : "down");
            if (is_up) mdns_resp_announce(&s_usb_netif);
        }
    }
    sys_check_timeouts();

    /* The net task owns lwIP; it no longer runs command_handler (that moved to
       the hw worker).  It drives the WS/Wi-Fi transport, then moves the worker's
       queued replies into lwIP and applies the worker's close/nodelay requests. */
    cloud_client_poll();             /* drive the outbound WSS control channel */
    esp_wifi_ctrl_poll();            /* drive Wi-Fi association (if configured) */
#if defined(BENCHPOD_WIFI_BRIDGE)
    wifi_bridge_net_poll();          /* Wi-Fi bridge: pairing, retries, STATUS  */
#endif
    net_tx_drain();                  /* worker reply rings -> lwIP              */
    if (s_lan_req >= 0) { int8_t r = s_lan_req; s_lan_req = -1; lan_access(r == 1); }
    if (s_https_req >= 0) { int8_t r = s_https_req; s_https_req = -1; https_only_apply(r == 1); }
    if (s_tls_listen_gen != lan_tls_generation()) tls_listener_sync();   /* new certificate */
    lan_tls_collect();
    net_apply_worker_requests();     /* worker-requested close / Nagle toggles  */
    net_frame_cloud_reply();         /* a finished cloud command -> response    */
    net_eth_apply_request();         /* apply a pending eth stop/start/restart  */
    net_run_pending_call();          /* a worker call handed over by net_call_sync */
    net_apply_reload();              /* a settings change's reload, once its reply has left */

    uint32_t now = HAL_GetTick();
    if (now - link_timer >= 100) {
        link_timer = now;
        /* While the wired link is held down by `eth stop`, skip the link check so
           it stays down until an explicit `eth start` (the check would otherwise
           re-up it as soon as the PHY reports link). */
        if (!s_eth_admin_down) ethernet_link_check_state(&s_eth.netif);
        update_default_route();
    }
    if (!s_eth_admin_down) eth_diag_poll(&s_eth.netif, now);
    if (!s_eth_admin_down && now - s_eth.dhcp_timer >= 500) { s_eth.dhcp_timer  = now; dhcp_process(&s_eth);  }
    if (!s_wifi_static && now - s_wifi.dhcp_timer >= 500) { s_wifi.dhcp_timer = now; dhcp_process(&s_wifi); }
}

/* Bring-up diagnostic: pin a static IP on the Wi-Fi netif (disables DHCP on it)
   so a peer can ping the pod — a definitive test of whether the esp-hosted bridge
   delivers UNICAST-to-STA frames to this host. `mask`/`gw` dotted-quad. */
/* ---- run a function on the net task --------------------------------------------------------
   lwIP runs NO_SYS on this task only (SYS_LIGHTWEIGHT_PROT 0), and the Wi-Fi control and SPI
   transport are driven from it too.  The worker used to call cloud_client_reload (altcp close,
   TLS config free), esp_wifi_ctrl_reload / esp_hosted_spi_stop (netif link down, TX ring reset)
   and the wifi-static netif calls directly, and the net task (higher priority, every tick) could
   preempt it mid-update: lwIP list or heap corruption.  net_call_sync hands the call over and
   waits for it.  Single caller at a time (the worker runs every command and console line). */
static volatile net_call_fn_t    s_net_call;
static void *volatile            s_net_call_arg;
static volatile uint32_t         s_net_call_done;

static void net_run_pending_call(void)
{
    net_call_fn_t fn = s_net_call;
    if (!fn) return;
    fn(s_net_call_arg);
    s_net_call = NULL;
    s_net_call_done++;
}

bool net_call_sync(net_call_fn_t fn, void *arg, uint32_t timeout_ms)
{
    if (!s_net_task || xTaskGetCurrentTaskHandle() == s_net_task) {
        fn(arg);                           /* already the net task (or before it runs) */
        return true;
    }
    uint32_t seq = s_net_call_done;
    s_net_call_arg = arg;
    s_net_call     = fn;
    uint32_t t0 = HAL_GetTick();
    while (s_net_call_done == seq) {
        if (HAL_GetTick() - t0 > timeout_ms) {
            s_net_call = NULL;             /* withdraw it; the net task is not polling */
            log_printf("[net] net_call_sync: the net task did not run the call in %lu ms\r\n",
                   (unsigned long)timeout_ms);
            return false;
        }
        vTaskDelay(1);
    }
    return true;
}

static void call_cloud_reload(void *a) { (void)a; cloud_client_reload(); }
static void call_wifi_reload(void *a)  { (void)a; esp_wifi_ctrl_reload(); }
static void call_wifi_hold(void *a) {
    if (a) { esp_wifi_ctrl_pause(true); esp_hosted_spi_stop(); }   /* EN/BOOT + SPI are ours */
    else   { esp_wifi_ctrl_pause(false); esp_wifi_ctrl_reload(); }
}
void net_cloud_reload(void)          { (void)net_call_sync(call_cloud_reload, NULL, 2000); }
void net_cloud_reload_after_reply(void) { net_reload_request(NET_RELOAD_CLOUD, HAL_GetTick()); }
void net_wifi_reload_after_reply(void)  { net_reload_request(NET_RELOAD_WIFI, HAL_GetTick()); }
void net_wifi_reload(void)           { (void)net_call_sync(call_wifi_reload, NULL, 2000); }
void net_wifi_hold_for_flash(bool h) { (void)net_call_sync(call_wifi_hold, h ? (void *)1 : NULL, 2000); }

bool net_wifi_wipe_c3(void) {
    net_wifi_hold_for_flash(true);   /* link down, C3 in reset, EN/BOOT are the worker's */
    bool wiped = (esp_rom_flash_erase_nvs() == 0);
    net_wifi_hold_for_flash(false);  /* reload: with nothing configured the C3 stays in reset */
    return wiped;
}

typedef struct { char ip[16], mask[16], gw[16]; } wifi_static_args_t;

static void net_wifi_static_on_net(void *arg);

void net_wifi_static(const char *ip_s, const char *mask_s, const char *gw_s)
{
    static wifi_static_args_t a;
    strncpy(a.ip, ip_s, sizeof(a.ip) - 1);     a.ip[sizeof(a.ip) - 1] = '\0';
    strncpy(a.mask, mask_s, sizeof(a.mask) - 1); a.mask[sizeof(a.mask) - 1] = '\0';
    strncpy(a.gw, gw_s, sizeof(a.gw) - 1);     a.gw[sizeof(a.gw) - 1] = '\0';
    (void)net_call_sync(net_wifi_static_on_net, &a, 2000);
}

static void net_wifi_static_on_net(void *arg)
{
    const wifi_static_args_t *a = (const wifi_static_args_t *)arg;
    const char *ip_s = a->ip, *mask_s = a->mask, *gw_s = a->gw;
    ip4_addr_t ip, mask, gw;
    if (!ip4addr_aton(ip_s, &ip) || !ip4addr_aton(mask_s, &mask) || !ip4addr_aton(gw_s, &gw)) {
        log_printf("[net] wifi-static: bad address\r\n");
        return;
    }
    dhcp_stop(&s_wifi.netif);
    netif_set_addr(&s_wifi.netif, &ip, &mask, &gw);
    netif_set_up(&s_wifi.netif);
    netif_set_link_up(&s_wifi.netif);
    strncpy(s_wifi.ip, ip4addr_ntoa(&ip), sizeof(s_wifi.ip) - 1);
    s_wifi.dhcp.state = NET_DHCP_DONE;
    s_wifi_static = true;
    log_printf("[net] wifi static ip=%s gw=%s (DHCP off on wifi — ping to test unicast RX)\r\n",
           s_wifi.ip, gw_s);
}

const char *net_ip_str(void) { return wifi_get_ip(); }

/* Wired MAC as "aa:bb:cc:dd:ee:ff" (set by ethernetif_init). Returned in a
   static buffer — single-threaded console/command use only. */
const char *net_mac_str(void)
{
    static char buf[18];
    const uint8_t *m = s_eth.netif.hwaddr;
    snprintf(buf, sizeof(buf), "%02x:%02x:%02x:%02x:%02x:%02x",
             m[0], m[1], m[2], m[3], m[4], m[5]);
    return buf;
}
