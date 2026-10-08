#include "cloud_client.h"
#include "ina238.h"
#include "board_variant.h"
#include "nrst_ctrl.h"
#include "cloud_config.h"
#include "wifi_manager.h"
#include "device_identity.h"
#include "command_handler.h"
#include "b64url.h"
#include "ws_frame.h"
#include "cloud_ca.h"
#include "board_info.h"
#include "signal_engine.h"
#include "boot_guard.h"   /* signal_engine_fpga_version / DAC_DEEP_REPLAY_MIN_GW / SIGNAL_MAX_SAMPLES */
#include "fault.h"        /* reset cause + last crash, announced with the capabilities */
#include "fpga_config.h"     /* FPGA_DAC_REPLAY_MAX_SAMPLES */
#include "cal_data.h"        /* ADC_CAL_EXT — front-SMA cal shipped in capabilities */
#include "current_out.h"     /* the 4-20 mA output's range, shipped in capabilities */
#include "target_power.h"
#include "bp_json.h"
#include "bp_limits.h"
#include "cloud_rx.h"     /* the s_rx accumulator operations (host-tested) */
#include "hw_worker.h"
#include "conn_tx.h"
#include "version.h"
#include "ota.h"
#include "fw_sign.h"
#include "pod_policy.h"
#include "lease_gate.h"
#include "cloud_extras.h"
#include "cloud_caps.h"
#include "altcp_bp_proxy.h"
#include "mbedtls/base64.h"
#include "stm32h5xx_hal.h"   /* HAL_GetTick: the lease deadline clock */

#include "FreeRTOS.h"
#include "task.h"            /* taskENTER_CRITICAL — guard the cross-task event slot */

#include "pico/time.h"
#include "pico/rand.h"

#include "lwip/opt.h"
#include "lwip/altcp.h"
#include "lwip/altcp_tcp.h"
#include "lwip/altcp_tls.h"
#include "lwip/tcp.h"
#include "lwip/priv/tcp_priv.h"
#include "altcp_tls_bp/altcp_tls_mbedtls_structs.h"
#include "altcp_tls_bp/altcp_tls_bp.h"
#include "lwip/dns.h"
#include "lwip/ip_addr.h"
#include "lwip/tcp.h"        /* TCP_WRITE_FLAG_COPY */

#include "mbedtls/ssl.h"     /* mbedtls_ssl_set_hostname (SNI) */
#include "flash_layout.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>          /* lround — integer-scale the cal (nano printf has no %f) */

/* Ping often and time out fast so a WEDGED server->pod direction self-heals quickly.  The failure
   mode (a single Cloudflare-proxied WS shared by a bulk DAC upload + a UART flood) stalls
   server->pod delivery while pod->server stays healthy: the pod's own RFC6455 PINGs then get no
   PONG back (the pong is server->pod too), so a short idle window detects the wedge in ~20 s and
   reconnects, instead of stranding the UI for 90 s of 502/503/504.  IDLE must stay a comfortable
   multiple of PING so a healthy-but-quiet link (pong every PING) never trips it — inbound of ANY
   kind, including the pong, refreshes s_last_rx. */
#define PING_INTERVAL_MS      6000
#define IDLE_TIMEOUT_MS      20000   /* no inbound (not even a pong) while connected -> reconnect */
/* Idle budget while an OTA is staging: SHORTER than the normal budget, not longer.
 *
 * While an image is staging the server pushes a frame every ~16 ms and waits on our 500 ms
 * acknowledgements, and our pings get a pong every PING_INTERVAL_MS on top: a healthy link is
 * never silent for more than a second or so. Silence during a transfer therefore means the
 * server->pod direction has died, and the sooner we reconnect the sooner the server resumes the
 * push from our acknowledged mark (the staged image survives a reconnect, see OTA_STAGE_IDLE_MS).
 *
 * Measured 2026-10-07 on a v3 pod over Cloudflare: mid-transfer the downlink went silent while
 * the uplink stayed healthy ([cloud/diag] ms_since_rx=30000 rx_acc=0 sndbuf=11659, every frame
 * received already staged). At 30 s each such wedge cost the update half a minute before the pod
 * even tried to reconnect. Two ping intervals is the shortest budget that a pong arriving late
 * cannot trip.
 *
 * History: 90 s when the server could pause indefinitely on a byte threshold, then 30 s once
 * the server was rate-capped. */
#define OTA_IDLE_TIMEOUT_MS  (2 * PING_INTERVAL_MS)
#define HTTP_TIMEOUT_MS       8000   /* await an HTTP/handshake response */
#define CONNECT_TIMEOUT_MS   12000   /* TCP connect + TLS handshake budget */
#define DNS_TIMEOUT_MS        8000
#define BACKOFF_START_MS      2000
#define BACKOFF_MAX_MS       30000

/* The connection sequence is event-driven on LwIP and runs over a SINGLE TLS
   link: it is opened once, the challenge nonce fetched over HTTP keep-alive, and
   the WS upgrade GET sent on the SAME link. This avoids a second full TLS
   handshake (the dominant cost of connecting on the M33) — the challenge
   endpoint sends an explicit Content-Length + Connection: keep-alive so the pod
   can find the body boundary and reuse the link instead of tearing it down. */
typedef enum {
    CL_DISABLED = 0,   /* no config / not enabled */
    CL_WAIT_WIFI,      /* waiting for the link (DHCP) to come up */
    CL_RESOLVE,        /* kick off DNS for the server host */
    CL_RESOLVE_WAIT,   /* await the DNS result */
    CL_CHAL_OPEN,      /* open the (single, shared) TLS link */
    CL_CHAL_CONNECT,   /* await connect, then POST the challenge */
    CL_CHAL_WAIT,      /* await the nonce, then send the WS upgrade on the same link */
    CL_WS_WAIT,        /* await the 101 handshake response */
    CL_CONNECTED,      /* WS open: ping + handle command.request */
    CL_BACKOFF,        /* wait, then retry from CL_RESOLVE */
} cl_state_t;

/* Connect/handshake completion is reported by the altcp callbacks; poll() owns
   all the state transitions (no work is done from callback context). */
typedef enum { LINK_IDLE, LINK_CONNECTING, LINK_UP, LINK_FAILED } link_ev_t;

static cloud_config_t   s_cfg;
static bool             s_have_cfg = false;
static cl_state_t       s_state = CL_DISABLED;
static absolute_time_t  s_deadline;                /* per-state timeout */
static absolute_time_t  s_next_ping;
static absolute_time_t  s_last_rx;
static uint32_t         s_backoff_ms = BACKOFF_START_MS;
/* The backoff is reset to the minimum only once a connection has stayed up this long —
   NOT the instant the WS handshake succeeds. Otherwise a server that immediately closes
   the ws (or a flapping link) makes the pod reconnect every BACKOFF_START_MS, and each
   reconnect's CPU-heavy TLS handshake starves the console task. Gating the reset makes
   repeated fast-close reconnects back off exponentially (spacing out the handshakes). */
#define CONN_STABLE_MS        10000
static absolute_time_t  s_stable_at;               /* when to trust the link + reset backoff */
static bool             s_backoff_reset_pending;
static char             s_nonce[96];               /* base64url server nonce */
static char             s_sig[B64URL_ENCODED_LEN(DEVICE_ID_SIG_LEN) + 1];

/* ---- LwIP transport ---- */
static struct altcp_pcb        *s_pcb = NULL;        /* the link we currently own, or NULL */
static struct altcp_tls_config *s_tls_conf = NULL;   /* created once, reused */
/* The proxy settings of the current connect attempt (cloud_extras.h) and the CONNECT layer's
   config, which must outlive the pcb. */
static cloud_proxy_t                 s_proxy;
static struct altcp_bp_proxy_config  s_proxy_conf;
static char                          s_proxy_auth[160];
/* How long after boot the first connect waits for the W25Q settings. */
#define CL_EXTRAS_WAIT_MS 20000u
/* Sign the login with v1 instead of v2: set after a 401 to a v2 login (an older server). */
static bool s_auth_v1;
static ip_addr_t                s_ip;                /* resolved server address */
static volatile link_ev_t       s_link = LINK_IDLE;  /* connect/handshake event */
static volatile bool            s_dropped = false;   /* peer closed / link error */
static volatile bool            s_rx_overflow = false; /* rx buffer overran — force reconnect */
static enum { DNS_NONE, DNS_PENDING, DNS_OK, DNS_FAIL } s_dns = DNS_NONE;
static bool                     s_tcp_up = false;    /* TCP reached ESTABLISHED on this attempt */

/* Why the last attempt failed, for cloud_status / console `status` ("" once connected). Written by
   the net task, read by others: both sides go through a critical section so a reader never sees a
   half-written string. */
static char s_last_error[80];

static void cl_set_error(const char *fmt, ...) {
    char tmp[sizeof(s_last_error)];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(tmp, sizeof(tmp), fmt, ap);
    va_end(ap);
    taskENTER_CRITICAL();
    memcpy(s_last_error, tmp, sizeof(tmp));
    taskEXIT_CRITICAL();
}

void cloud_client_last_error(char *out, size_t n) {
    if (!out || n == 0) return;
    taskENTER_CRITICAL();
    strncpy(out, s_last_error, n - 1);
    taskEXIT_CRITICAL();
    out[n - 1] = '\0';
}

/* Active cloud byte-tunnels. Each slot i binds a tunnel id to pseudo-conn CH_CLOUD_TUNNEL_CONN + i, so
   independent tunnels (e.g. a UART proxy + an LA capture) multiplex over the one device WS without
   clobbering each other. A slot is free when its id string is empty; the id is echoed back on the
   device→client tunnel.data frames so the server routes them to the right client. */
static char s_tunnels[CH_CLOUD_TUNNEL_CONN_COUNT][BP_TUNNEL_ID_MAX];

static int cl_tunnel_slot_by_id(const char *id) {
    if (!id || !id[0]) return -1;
    for (int i = 0; i < CH_CLOUD_TUNNEL_CONN_COUNT; i++)
        if (s_tunnels[i][0] && strcmp(s_tunnels[i], id) == 0) return i;
    return -1;
}

static int cl_tunnel_free_slot(void) {
    /* Prefer a slot whose previous session's reset the worker has already handled (its late
       replies are gone); fall back to any free one. */
    for (int i = 0; i < CH_CLOUD_TUNNEL_CONN_COUNT; i++)
        if (!s_tunnels[i][0] && !hw_worker_conn_busy(CH_CLOUD_TUNNEL_CONN + i)) return i;
    for (int i = 0; i < CH_CLOUD_TUNNEL_CONN_COUNT; i++)
        if (!s_tunnels[i][0]) return i;
    return -1;
}

/* Inbound (decrypted) byte accumulator: HTTP responses, then WS frames.  Sized to
   absorb a burst of upload frames between net-task drains (BP_CLOUD_RX_ACCUM), not
   just one frame — cl_recv_cb eager-acks, so without this a chunked DAC replay burst
   overflows and resets the link. */
static uint8_t          s_rx[BP_CLOUD_RX_ACCUM];
static size_t           s_rx_len = 0;
static size_t           s_rx_len_peak = 0;   /* high-water mark of the RX accumulator (diag) */
/* Cross-task wedge-snapshot request: any task latches (tag, req); cloud_client_poll (net task)
   services it so the actual pcb/send-buffer read stays on the net task.  volatile: set off-task. */
static volatile bool    s_diag_req = false;
static const char      *s_diag_tag = NULL;

/* Async eFuse-event queue: target_power's callback (which may run on the console
   OR net task) latches the latest EN/FLT state per eFuse here; cloud_client_poll
   (net task, CL_CONNECTED only) drains it and pushes an efuse.event WS frame.
   A brief critical section guards the slot — cl_ws_send is net-task-only, so the
   callback must never send directly.  Coalescing (a slot holds only the latest
   event) is fine: the frame carries the full {enabled,fault} snapshot. */
static volatile struct {
    bool pending;
    bool enabled;
    bool fault;
} s_efuse_ev[2];

/* Cross-task capabilities-resend request: a runtime FPGA image swap (handle_fpga_image, on the
   hw_worker task) changes which gateware image — and therefore which caps (dac_control_loop /
   dac_deep_replay / dac_replay_max_samples) — is live. It latches this; cloud_client_poll (net
   task, CL_CONNECTED only) re-sends the capabilities frame so the server's cached caps follow the
   swap instead of staying at whatever was announced on connect. volatile: set off-task. */
static volatile bool s_resend_caps = false;

/* Ask the net task to re-announce capabilities (call after a successful FPGA image swap). Safe to
   call from any task — just sets a flag the net loop services; the actual cl_ws_send stays on the
   net task. */
void cloud_client_request_caps_resend(void) { s_resend_caps = true; }

/* Runs on whichever task called target_power_enable()/poll(). Keep it tiny.
   Registered with target_power_set_event_cb() so target_power stays cloud-agnostic. */
static void cl_efuse_event_cb(int efuse, bool enabled, bool fault) {
    int idx = (efuse == 1) ? 0 : (efuse == 2) ? 1 : -1;
    if (idx < 0) return;
    taskENTER_CRITICAL();
    s_efuse_ev[idx].enabled = enabled;
    s_efuse_ev[idx].fault   = fault;
    s_efuse_ev[idx].pending = true;
    taskEXIT_CRITICAL();
}

const char *cloud_client_state_str(void) {
    switch (s_state) {
        case CL_DISABLED:  return "disabled";
        case CL_WAIT_WIFI: return "waiting-wifi";
        case CL_RESOLVE:
        case CL_RESOLVE_WAIT:
        case CL_CHAL_OPEN:
        case CL_CHAL_CONNECT:
        case CL_CHAL_WAIT:
        case CL_WS_WAIT:   return "connecting";
        case CL_CONNECTED: return "connected";
        case CL_BACKOFF:   return "backoff";
        default:           return "unknown";
    }
}

/* Internal per-state name for timing logs — cloud_client_state_str() collapses
   every handshake step to "connecting", which hides where the seconds go. */
static const char *cl_state_dbg_name(cl_state_t st) {
    switch (st) {
        case CL_DISABLED:     return "DISABLED";
        case CL_WAIT_WIFI:    return "WAIT_WIFI";
        case CL_RESOLVE:      return "RESOLVE";
        case CL_RESOLVE_WAIT: return "RESOLVE_WAIT";
        case CL_CHAL_OPEN:    return "LINK_OPEN";
        case CL_CHAL_CONNECT: return "LINK_CONNECT";
        case CL_CHAL_WAIT:    return "CHAL_WAIT";
        case CL_WS_WAIT:      return "WS_WAIT";
        case CL_CONNECTED:    return "CONNECTED";
        case CL_BACKOFF:      return "BACKOFF";
        default:              return "?";
    }
}

/* Phase timing: t0 of the current state, and t0 of the whole connect attempt
   (set when we leave CL_WAIT_WIFI / enter CL_RESOLVE). The per-transition delta
   tells us how long the state we just LEFT took; the running total shows how
   long since the connect attempt began. */
static absolute_time_t s_state_t0;
static absolute_time_t s_connect_t0;

static void cl_set_state(cl_state_t st) {
    cl_state_t prev = s_state;
    absolute_time_t now = get_absolute_time();
    uint32_t in_prev_ms = (uint32_t)(absolute_time_diff_us(s_state_t0, now) / 1000);

    if (st == CL_RESOLVE && prev != CL_RESOLVE) s_connect_t0 = now;
    uint32_t total_ms = (uint32_t)(absolute_time_diff_us(s_connect_t0, now) / 1000);

    s_state = st;
    s_state_t0 = now;
    /* e.g. "[cloud] state -> connected [CONNECTED]  CHAL_CONNECT took 1832 ms, +4210 ms total" */
    printf("[cloud] state -> %s [%s]  %s took %lu ms, +%lu ms total\n",
           cloud_client_state_str(), cl_state_dbg_name(st),
           cl_state_dbg_name(prev), (unsigned long)in_prev_ms,
           (unsigned long)total_ms);
}

static void cl_rx_reset(void) { s_rx_len = 0; }

static void cl_rx_consume(size_t n) { cloud_rx_consume(s_rx, &s_rx_len, n); }

/* Append inbound (decrypted) bytes.  Runs in lwIP recv-callback context (the
   callbacks only set flags; poll() acts on them), so on overflow we don't tear the
   link down here — dropping oldest bytes would slice through a WS frame boundary
   and desync ws_parse_frame for the rest of the link, so flag an overflow and let
   poll() drop the whole buffer and reconnect (the server resends after reopen). */
static void cl_rx_append(const uint8_t *data, size_t len) {
    if (!cloud_rx_append(s_rx, sizeof(s_rx), &s_rx_len, data, len)) {
        s_rx_overflow = true;
        return;
    }
    if (s_rx_len > s_rx_len_peak) s_rx_len_peak = s_rx_len;
}

/* Close (or abort) the link we own and tear down any in-flight tunnel. Safe to
   call when s_pcb is already NULL (e.g. after the err callback freed it). */
static void cl_drop_link(void) {
    lease_gate_clear();   /* without the link, the server's lease says nothing any more */
    if (s_pcb) {
        altcp_arg(s_pcb, NULL);
        altcp_recv(s_pcb, NULL);
        altcp_err(s_pcb, NULL);
        if (altcp_close(s_pcb) != ERR_OK) altcp_abort(s_pcb);
        s_pcb = NULL;
    }
    s_link    = LINK_IDLE;
    s_dropped = false;
    s_rx_overflow = false;
    /* A dropped WS abandons any in-flight tunnels; tear them all down so a mid-flash SWD/UART session
       leaves the wire in a safe state and the next tunnel starts clean. */
    for (int i = 0; i < CH_CLOUD_TUNNEL_CONN_COUNT; i++) {
        if (s_tunnels[i][0]) {
            s_tunnels[i][0] = '\0';
            conn_tx_reset(CH_CLOUD_TUNNEL_CONN + i);
            hw_worker_submit_tunnel_reset(CH_CLOUD_TUNNEL_CONN + i);
        }
    }
    cl_rx_reset();
}

static bool        s_send_full;    /* the last cl_tcp_send failed only because the buffer was full */
static const char *s_pend_buf;     /* a command.response that met a full buffer, resent from the poll */
static size_t      s_pend_len;

static void cl_backoff(const char *reason) {
    printf("[cloud] backoff: %s\n", reason);
    s_pend_len = 0;                 /* the server times the request out and retries it */
    cl_drop_link();
    cl_set_state(CL_BACKOFF);
    s_deadline = make_timeout_time_ms(s_backoff_ms);
    s_backoff_ms *= 2;
    if (s_backoff_ms > BACKOFF_MAX_MS) s_backoff_ms = BACKOFF_MAX_MS;
}

/* Wedge diagnostics: snapshot the cloud link's flow-control state at a point of interest
   (an idle-timeout backoff, or a load_bin stall via the accessor below).  This tells the two
   directions apart when an upload silently stalls:
     - sndbuf == 0  → the pod's SEND is backed up (can't push ACKs/replies) → the wedge is on the
       pod→server path (its ACKs aren't leaving, so the server's window closes and it stops sending).
     - sndbuf healthy + rx_acc small + old ms_since_rx → the pod COULD send and simply received
       nothing → the server/Cloudflare stopped delivering (server→pod path is the culprit).
     - rx_peak near capacity or overflow=1 → the inbound burst outran the net task draining the
       accumulator (a pod-side processing bottleneck), which would reset as "rx overflow" not a
       silent stall. */
/* ota.data frames the NET task took off the wire, vs. those it handed to the worker. Comparing
   these with ota_received() separates "the frames never arrived" from "they arrived and staging
   stalled" — the two explanations for a stalled cloud OTA that look identical from the server,
   which only knows what it wrote. This is what showed the device seeing 6 of 33 pushed frames. */
static uint32_t s_ota_frames_seen;
static uint32_t s_ota_frames_submitted;
static uint32_t s_ota_frames_requeued;

uint32_t cloud_client_ota_frames_seen(void) { return s_ota_frames_seen; }

/* Wedge diag: the TLS layer's window bookkeeping and the TCP receive window, next to
   cloud_client_log_link_state's snapshot. rcv_wnd=0 with bio_read>0 and nothing queued is the
   record-larger-than-window deadlock altcp_tls_mbedtls_bp.c fixes. */
static void cl_log_tls_window(const char *when) {
    if (!s_pcb || !s_cfg.tls || s_proxy.host[0]) return;
    altcp_mbedtls_state_t *st = (altcp_mbedtls_state_t *)s_pcb->state;
    struct altcp_pcb *inner = s_pcb->inner_conn;
    struct tcp_pcb *tp = inner ? (struct tcp_pcb *)inner->state : NULL;
    unsigned rx = (st && st->rx) ? st->rx->tot_len : 0, rxapp = (st && st->rx_app) ? st->rx_app->tot_len : 0;
    printf("[cloud/tls] %s: rcv_wnd=%u rcv_ann_wnd=%u refused=%u unrecved=%d bio_read=%d bio_appl=%d loaned=%d debt=%d rx=%u rx_app=%u ssl_avail=%u\n",
           when, tp ? (unsigned)tp->rcv_wnd : 0, tp ? (unsigned)tp->rcv_ann_wnd : 0,
           (tp && tp->refused_data) ? (unsigned)tp->refused_data->tot_len : 0,
           st ? st->rx_passed_unrecved : -1, st ? st->bio_bytes_read : -1, st ? st->bio_bytes_appl : -1,
           st ? st->bp_loaned : -1, st ? st->bp_debt : -1,
           rx, rxapp, st ? (unsigned)mbedtls_ssl_get_bytes_avail(&st->ssl_context) : 0);
}

void cloud_client_log_link_state(const char *when) {
    long ms_since_rx = (long)(absolute_time_diff_us(s_last_rx, get_absolute_time()) / 1000);
    unsigned sndbuf = (s_pcb && s_state == CL_CONNECTED) ? (unsigned)altcp_sndbuf(s_pcb) : 0;
    printf("[cloud/diag] %s: state=%d ms_since_rx=%ld rx_acc=%u/%u rx_peak=%u overflow=%d sndbuf=%u "
           "ota_seen=%lu ota_submitted=%lu ota_requeued=%lu\n",
           when ? when : "?", (int)s_state, ms_since_rx,
           (unsigned)s_rx_len, (unsigned)sizeof(s_rx), (unsigned)s_rx_len_peak,
           s_rx_overflow ? 1 : 0, sndbuf,
           (unsigned long)s_ota_frames_seen, (unsigned long)s_ota_frames_submitted,
           (unsigned long)s_ota_frames_requeued);
}

/* Latch a request for the above snapshot, to be logged on the next net-task poll.  Safe to call
   from any task (the worker at a load_bin stall) — it only sets flags; the net task does the read. */
void cloud_client_request_link_snapshot(const char *tag) {
    s_diag_tag = tag;
    s_diag_req = true;
}

/* Backoff after a DNS failure, naming the resolver being used (learned via DHCP
   option 6) so a dead/misconfigured DNS server is obvious in the log — an
   otherwise-online device that just can't resolve looks identical without it. */
static void cl_dns_backoff(const char *what) {
    const ip_addr_t *srv = dns_getserver(0);
    char msg[64];
    snprintf(msg, sizeof(msg), "%s (dns server %s)", what,
             (srv && !ip_addr_isany(srv)) ? ipaddr_ntoa(srv) : "none");
    cl_backoff(msg);
}

/* ---- altcp callbacks (set flags only; poll() acts on them) ---------------- */

static err_t cl_connected_cb(void *arg, struct altcp_pcb *conn, err_t err) {
    (void)arg; (void)conn;
    if (err != ERR_OK) {
        printf("[cloud] connect-cb err=%d\n", (int)err);
        cl_set_error("tcp connect failed (err %d)", (int)err);
        s_link = LINK_FAILED;
        return ERR_OK;
    }
    s_link    = LINK_UP;        /* TLS: fired after the handshake completes */
    s_last_rx = get_absolute_time();
    return ERR_OK;
}

/* Set when cl_recv_cb refused data for lack of room in s_rx: the net task hands it back
   (altcp_tls_bp_kick) once it has drained s_rx. */
static bool s_rx_refused;

static err_t cl_recv_cb(void *arg, struct altcp_pcb *conn, struct pbuf *p, err_t err) {
    (void)arg;
    if (p == NULL) { s_dropped = true; return ERR_OK; }   /* peer sent FIN */
    if (err != ERR_OK) { pbuf_free(p); return err; }
    s_last_rx = get_absolute_time();
    /* No room: refuse it. The TLS layer keeps it (and the TCP window stays shut for it) until we
       ask again, which is the backpressure that keeps a fast server from overflowing s_rx. Only
       an empty buffer that still cannot hold it counts as an overflow. */
    if (cloud_rx_refuse(sizeof(s_rx), s_rx_len, p->tot_len)) {
        s_rx_refused = true;
        return ERR_MEM;
    }
    for (struct pbuf *q = p; q != NULL; q = q->next)
        cl_rx_append((const uint8_t *)q->payload, q->len);
    altcp_recved(conn, p->tot_len);
    pbuf_free(p);
    return ERR_OK;
}

static void cl_err_cb(void *arg, err_t err) {
    (void)arg;
    printf("[cloud] err-cb err=%d\n", (int)err);   /* diag: -13 ERR_ABRT, -14 ERR_RST, -3 ERR_TIMEOUT ... */
    /* altcp_tls reports a failed handshake as ERR_CLSD; its pcb is still valid here. */
    if (err == ERR_CLSD) s_tcp_up = true;
    if (s_link == LINK_CONNECTING) {
        if (s_tcp_up)            cl_set_error("tls handshake failed");
        else if (err == ERR_RST) cl_set_error("tcp connect refused");
        else                     cl_set_error("tcp connect failed (err %d)", (int)err);
    }
    /* lwIP has already freed the pcb — do not touch/close it. */
    s_pcb     = NULL;
    s_link    = LINK_FAILED;
    s_dropped = true;
}

/* Open an outbound link to the resolved server address. tls selects altcp_tls
   (with SNI) vs plain altcp. Returns false on immediate failure. */
static bool cl_open(bool tls, uint16_t port) {
    struct altcp_pcb *pcb;
    /* The bottom of the stack: plain TCP, or the HTTP CONNECT layer when a proxy is set
       (cloud_extras.h); TLS goes on top of either, so it starts only once the tunnel is up. */
    struct altcp_pcb *base;
    if (s_proxy.host[0]) {
        ip_addr_copy(s_proxy_conf.proxy_addr, s_ip);
        s_proxy_conf.proxy_port  = s_proxy.port;
        s_proxy_conf.target_host = s_cfg.host;
        s_proxy_conf.auth_b64    = NULL;
        s_proxy_conf.last_status = 0;
        s_proxy_conf.closed_early = 0;
        if (s_proxy.user[0]) {
            char up[CLOUD_PROXY_USER_MAX + CLOUD_PROXY_PASS_MAX + 2];
            int n = snprintf(up, sizeof(up), "%s:%s", s_proxy.user, s_proxy.pass);
            size_t olen = 0;
            if (n > 0 && mbedtls_base64_encode((unsigned char *)s_proxy_auth, sizeof(s_proxy_auth), &olen,
                                               (const unsigned char *)up, (size_t)n) == 0)
                s_proxy_conf.auth_b64 = s_proxy_auth;
            memset(up, 0, sizeof(up));
        }
        base = altcp_bp_proxy_new_tcp(&s_proxy_conf, IPADDR_TYPE_V4);
    } else {
        base = altcp_tcp_new_ip_type(IPADDR_TYPE_V4);
    }
    if (!base) return false;
    if (tls) {
        if (!s_tls_conf) {
            /* TLS always anchors trust on the embedded ISRG roots (Let's Encrypt), plus the
               company CA when one is installed (cloud_extras.h), so the server cert chain +
               hostname are validated in cl_tls_verified() after the handshake. authmode is
               VERIFY_OPTIONAL (the handshake completes; we check the result ourselves and
               drop the link on a verify failure). */
            /* Only a config built from what was asked for is cached: out of memory fails this
               attempt (the backoff retries), it never quietly caches a roots-only config that a
               TLS-inspecting network would refuse until the next reboot. A company CA that does
               not parse is dropped by cloud_extras (with a log line and cloud_ca's "error"), and
               the next attempt uses the built-in roots. */
            size_t ca_len = 0;
            char *ca = NULL;
            if (cloud_extras_ca_pem(&ca, &ca_len) != 0) {
                printf("[cloud] out of memory for the company CA: retrying later\n");
                cl_set_error("out of memory for the company CA");
                altcp_abort(base);
                return false;
            }
            s_tls_conf = ca ? altcp_tls_create_config_client((const u8_t *)ca, ca_len)
                            : altcp_tls_create_config_client(cloud_ca_pem, cloud_ca_pem_len);
            if (ca) cloud_extras_free(ca);
            if (!s_tls_conf) {
                if (ca && cloud_extras_ca_tls_refused())
                    cl_set_error("company CA unusable: using the built-in roots");
                else {
                    printf("[cloud] altcp_tls config create failed (out of memory?)\n");
                    cl_set_error("tls config create failed (out of memory?)");
                }
                altcp_abort(base);
                return false;
            }
        }
        pcb = altcp_tls_wrap(s_tls_conf, base);
        if (!pcb) { printf("[cloud] altcp_tls_wrap failed (out of mem?)\n"); altcp_abort(base); return false; }
        mbedtls_ssl_context *ssl = (mbedtls_ssl_context *)altcp_tls_context(pcb);
        if (ssl) mbedtls_ssl_set_hostname(ssl, s_cfg.host);   /* SNI + CN/SAN check host */
    } else {
        pcb = base;
    }

    altcp_arg(pcb, NULL);
    altcp_recv(pcb, cl_recv_cb);
    altcp_err(pcb, cl_err_cb);

    s_pcb     = pcb;
    s_link    = LINK_CONNECTING;
    s_dropped = false;
    s_tcp_up  = false;
    cl_rx_reset();

    {
        char ipbuf[16];
        printf("[cloud] connecting to %s:%u (%s)\n", ipaddr_ntoa_r(&s_ip, ipbuf, sizeof ipbuf),
               (unsigned)port, tls ? "tls" : "plain");
    }
    err_t ce = altcp_connect(pcb, &s_ip, port, cl_connected_cb);
    if (ce != ERR_OK) {
        printf("[cloud] altcp_connect immediate err=%d\n", (int)ce);
        cl_drop_link();
        return false;
    }
    return true;
}

/* True once the raw TCP pcb under our link (below altcp_tls, if any) is ESTABLISHED.
   altcp_dbg_get_tcp_state() would do this but only exists with LWIP_DEBUG. */
static bool cl_tcp_established(void) {
    struct altcp_pcb *c = s_pcb;
    while (c && c->inner_conn) c = c->inner_conn;
    const struct tcp_pcb *t = c ? (const struct tcp_pcb *)c->state : NULL;
    return t && t->state == ESTABLISHED;
}

static int cl_tcp_send(const uint8_t *buf, size_t len) {
    if (!s_pcb) return -1;
    err_t err = altcp_write(s_pcb, buf, (u16_t)len, TCP_WRITE_FLAG_COPY);
    /* ERR_MEM = the send buffer is full for now (a tunnel read-back is filling it), not a
       broken link: callers that can wait check s_send_full and retry instead of reconnecting. */
    s_send_full = (err == ERR_MEM);
    if (err != ERR_OK) return -1;
    /* The frame is queued once altcp_write succeeds; an altcp_output error only delays it (the
       TCP timer sends it).  Reporting that as a failure made callers resend a queued frame. */
    (void)altcp_output(s_pcb);
    return 0;
}

/* Enforce cfg.verify once the TLS handshake has completed (link is UP).  authmode
   is VERIFY_OPTIONAL, so mbedTLS recorded — but did not act on — the chain/hostname
   result; we gate on it here.  Returns true when verification is not required
   (plain TCP or verify=0) or the cert validated against the embedded roots and the
   hostname matched.  (Validity dates are not checked — no RTC; MBEDTLS_HAVE_TIME_DATE
   is off — so an expired cert from a trusted root is still accepted.) */
static bool cl_tls_verified(void) {
    /* TLS always verifies now — the verify=0 bring-up mode was removed. Plain TCP
       (tls=0) has no cert to check. */
    if (!s_cfg.tls) return true;
    if (!s_pcb) return false;
    mbedtls_ssl_context *ssl = (mbedtls_ssl_context *)altcp_tls_context(s_pcb);
    if (!ssl) return false;
    uint32_t flags = mbedtls_ssl_get_verify_result(ssl);
    if (flags != 0) {
        printf("[cloud] TLS cert verify failed: flags=0x%08lx\n", (unsigned long)flags);
        cl_set_error("tls certificate not trusted (flags 0x%lx)", (unsigned long)flags);
        return false;
    }
    return true;
}

/* JSON parsing is shared (see bp_json.h).  Aliased to the cl_* names the frame
   handlers below already use.  The parser matches keys at a real key position
   and is escape-aware, so a value that merely contains the key text (or an
   escaped quote inside a string) is handled correctly.

   These bind to the TOP-LEVEL variants on purpose.  Everything parsed in this
   file is a frame ENVELOPE, and an envelope CARRIES a caller-controlled object
   (command.request wraps the whole command).  With the flat getters — a plain
   strstr for the first "key": anywhere — a carried command's own key shadowed the
   envelope's: a {"cmd":"sensor_start","type":"bmp280"} command made the router
   read "bmp280" as the FRAME type (Go marshals the frame map with sorted keys, so
   "command" lands before "type" on the wire), match no branch, and drop the frame
   with no reply, so every such call hung until the 30 s timeout.  Envelope fields
   must never be shadowable by payload. */
#define cl_json_str     bp_json_get_top
#define cl_json_object  bp_json_object_top

/* True if the first n bytes (the HTTP status line) report "101". */
static bool cl_status_is_101(const uint8_t *buf, size_t n) {
    for (size_t i = 0; i + 3 < n; i++) {
        if (buf[i] == '1' && buf[i + 1] == '0' && buf[i + 2] == '1' &&
            (buf[i + 3] == ' ' || buf[i + 3] == '\r')) {
            return true;
        }
    }
    return false;
}

/* The status code from an HTTP status line ("HTTP/1.1 404 Not Found"), or 0 if unparsable. */
static int cl_http_status(const uint8_t *buf, size_t n) {
    size_t i = 0;
    while (i < n && buf[i] != ' ') i++;
    if (i + 3 >= n) return 0;
    int code = 0;
    for (size_t k = i + 1; k < i + 4; k++) {
        if (buf[k] < '0' || buf[k] > '9') return 0;
        code = code * 10 + (buf[k] - '0');
    }
    return code;
}

/* Locate the end of the HTTP response headers (the \r\n\r\n terminator). Returns
   the offset just past it (start of body) or 0 if not yet fully received. */
static size_t cl_http_headers_end(const uint8_t *buf, size_t n) {
    for (size_t i = 0; i + 3 < n; i++) {
        if (buf[i] == '\r' && buf[i + 1] == '\n' &&
            buf[i + 2] == '\r' && buf[i + 3] == '\n')
            return i + 4;
    }
    return 0;
}

/* Parse a Content-Length header value (case-insensitive) from the first hdr_len
   bytes of buf. Returns the value, or -1 if the header is absent/malformed. The
   challenge endpoint sends an explicit Content-Length so the body boundary is
   deterministic — this lets us keep the TLS link open (keep-alive) and reuse it
   for the WS upgrade instead of tearing it down and re-handshaking. */
static long cl_content_length(const uint8_t *buf, size_t hdr_len) {
    static const char key[] = "content-length:";
    for (size_t i = 0; i + sizeof(key) - 1 < hdr_len; i++) {
        size_t k = 0;
        while (k < sizeof(key) - 1) {
            char c = (char)buf[i + k];
            if (c >= 'A' && c <= 'Z') c = (char)(c + 32);   /* tolower */
            if (c != key[k]) break;
            k++;
        }
        if (k != sizeof(key) - 1) continue;
        size_t j = i + k;
        while (j < hdr_len && (buf[j] == ' ' || buf[j] == '\t')) j++;
        long v = 0;
        bool any = false;
        while (j < hdr_len && buf[j] >= '0' && buf[j] <= '9') {
            v = v * 10 + (buf[j] - '0');
            any = true;
            j++;
        }
        return any ? v : -1;
    }
    return -1;
}

/* ---- WS send helpers ------------------------------------------------------ */

static bool cl_ws_send(uint8_t opcode, const char *payload, size_t len) {
    if (!s_pcb) return false;
    uint8_t mask[4];
    uint64_t r = get_rand_64();
    memcpy(mask, &r, 4);
    static uint8_t frame[BP_WS_FRAME_MAX];
    size_t n = ws_build_frame(opcode, (const uint8_t *)payload, len, mask, frame, sizeof(frame));
    if (n == 0) { printf("[cloud] frame too big (%u B)\n", (unsigned)len); return false; }
    return cl_tcp_send(frame, n) == 0;
}

static bool cl_send_capabilities(void) {
    /* scope/analyzer are now driven server-side over the byte-tunnel (the server
       runs the `capture` / `sensor_la` JSON commands and decodes the raw bytes),
       so the pod advertises both as available along with its ADC metadata.

       adc_cal_a_uv/adc_cal_b_nv/adc_cal_unwrap ship the affine front-end fit so
       the server scales raw capture counts to the true probe voltage instead of
       the naive count/full-scale (which reports the ADC-pin voltage — the
       inverting ×gain+offset front end makes a 1.3 V input read ~4 V under that
       model). We send ADC_CAL_EXT (the front-SMA path the scope capture uses); it
       is exactly what handle_adc_read applies, so a `capture` and an `adc_read`
       now agree. unwrap=true: ext is bipolar, so its 16-bit count wraps for
       near-0/negative inputs and must be unwrapped (count += 65536 when
       count < 32768).

       The coefficients are shipped as INTEGERS (a in microvolts, b in nanovolts
       per count) — NOT floats: newlib-nano's printf has no %f/%g (see the note in
       signal_engine.c), so a %g here emitted a MALFORMED frame that failed to
       parse server-side (→ "no tunnel"). Same integer-scaling trick
       handle_adc_read uses. The server divides back: a[V]=a_uv/1e6,
       b[V/count]=b_nv/1e9. lround() keeps full precision (double math, no float
       printf). */
    /* Deep DAC replay (stream a waveform straight out of PSRAM, past the 4 KB DAC
       BRAM cap) needs gateware >= v17.  Advertise it + the actual replay depth the
       device supports so the server can offer full-length recording replay and clamp
       requests: FPGA_DAC_REPLAY_MAX_SAMPLES (8 MB region) when deep, else the shallow
       BRAM depth (SIGNAL_MAX_SAMPLES). */
    /* The cached gateware version is current: the hw worker re-reads it after boot bring-up,
       the boot gateware update and every image swap, and re-announces when it is done.  This ran
       on the net task and used to re-read it over SPI1, which the worker drives WITHOUT hw_lock,
       so a (re)connect could cut a worker transaction mid-command. */
    /* Feature flags come from signal_engine_caps() — the SAME call the `status` reply uses, so
       a cloud client and a direct LAN/serial client cannot disagree about what this pod can do.
       (They used to: status carried a hardcoded caps[] literal that named none of these.) */
    signal_engine_caps_t caps;
    signal_engine_caps(&caps);
    /* A digital-only board has the gateware's DAC/ADC engines but no DAC or ADC behind them
       (board_variant.h): announce none of the analog features there. */
    bool analog    = board_has_analog();
    bool deep      = analog && caps.deep_replay;
    bool ctrl_loop = analog && caps.control_loop;
    bool cotrig    = analog && caps.cotrig;
    bool loop_src  = analog && caps.loop_sources;
    bool loop_map  = analog && caps.loop_input_map;
    unsigned long replay_max = deep ? (unsigned long)FPGA_DAC_REPLAY_MAX_SAMPLES
                                    : (unsigned long)SIGNAL_MAX_SAMPLES;
    cloud_caps_t c = {
        .device_id = s_cfg.device_id,
        .firmware_version = FIRMWARE_VERSION,
        .flash_kb = (unsigned long)(flash_layout_size() / 1024u),
        .sig_policy = fw_sign_policy_name(fw_sign_policy()),
        .lan_policy = pod_policy_lan_name(pod_policy_lan()),
        .analog = analog,
        .adc_bits = ADC_BITS, .adc_fullscale_mv = ADC_FULLSCALE_MV, .adc_channels = ADC_CHANNELS,
        .adc_cal_a_uv = lround((double)ADC_CAL_EXT.a * 1000000.0),
        .adc_cal_b_nv = lround((double)ADC_CAL_EXT.b * 1000000000.0),
        .dac_ac = analog && DAC_AC, .dac_replay = analog && DAC_REPLAY, .dac_dc = analog && DAC_DC,
        .dac_bits = DAC_BITS, .dac_replay_bits = DAC_REPLAY_BITS,
        .dac_fullscale_mv = DAC_FULLSCALE_MV, .dac_channels = DAC_CHANNELS,
        .deep_replay = deep, .replay_max_samples = replay_max,
        .control_loop = ctrl_loop, .loop_sources = loop_src, .loop_input_map = loop_map,
        .cotrig = cotrig,
        .gpio_read = caps.gpio_read, .capture_trigger = caps.capture_trigger,
        .spi_master = caps.spi_master,
        .nrst_pin = nrst_ctrl_supported(),     /* the DUT reset pin (rev3+): hold reset for SPI/SWD */
        .pod_current = ina_pod_present(),      /* the pod's own current monitor (0x41) */
        /* The 4-20 mA output's range, so the server can turn a waveform in mA into DAC codes
           with the pod's own numbers (current_out.h). */
        .current_out_min_ua = current_out_min_ua(), .current_out_max_ua = current_out_max_ua(),
        .board = BOARD_NAME,
        /* Boot health, sent on every connect so the server always holds the current boot's
           values: last_crash is "none" and safe_reason "" after a clean start, which clears an
           old warning. */
        .safe_mode = boot_guard_safe_mode(),
        .safe_reason = boot_guard_reason(),
        .reset_cause = fault_last_reset_str(),
        .last_crash = fault_last_crash_str(),
        .boot_id = fault_boot_id(),
        .unclean_resets = fault_unclean_resets(),
    };
    /* cloud_caps_build keeps the frame inside one WS frame (it shortens the free-form boot
       health when it has to), so a long crash line can no longer make the connect fail and
       loop. Static: net-task only. */
    static char f[CLOUD_CAPS_MAX + 1];
    size_t n = cloud_caps_build(&c, f, sizeof(f));
    if (n == 0) {
        /* Cannot happen (test_cloud_caps.c builds the worst case); keep the link rather than
           reconnect over it. */
        printf("[cloud] capabilities frame does not fit %u bytes: not sent\n", (unsigned)CLOUD_CAPS_MAX);
        return true;
    }
    if (!cl_ws_send(WS_OP_TEXT, f, n)) return false;
    /* The server has the count now (it keys the crash on boot_id, so a reconnect that sends the
       same boot again does not count twice). */
    fault_unclean_resets_ack();
    return true;
}

/* Build + push one efuse.event WS frame (efuse is 1 or 2).  Net-task only. */
static void cl_send_efuse_event(int efuse, bool enabled, bool fault) {
    char f[160];
    int n = snprintf(f, sizeof(f),
        "{\"type\":\"efuse.event\",\"device_id\":\"%s\",\"efuse\":%d,"
        "\"enabled\":%s,\"fault\":%s}",
        s_cfg.device_id, efuse,
        enabled ? "true" : "false", fault ? "true" : "false");
    if (n > 0 && (size_t)n < sizeof(f)) cl_ws_send(WS_OP_TEXT, f, (size_t)n);
}

/* Drain any pending eFuse events and push them as efuse.event WS frames.
   Called only from CL_CONNECTED (net task), where cl_ws_send is safe.  The slot
   is read+cleared under a critical section so a concurrent callback (console or
   net task) can't lose or tear an event. */
static void cl_drain_efuse_events(void) {
    for (int idx = 0; idx < 2; idx++) {
        taskENTER_CRITICAL();
        bool pending = s_efuse_ev[idx].pending;
        bool enabled = s_efuse_ev[idx].enabled;
        bool fault   = s_efuse_ev[idx].fault;
        s_efuse_ev[idx].pending = false;
        taskEXIT_CRITICAL();
        if (!pending) continue;
        cl_send_efuse_event(idx + 1, enabled, fault);
    }
}

/* On every (re)connect, push the CURRENT eFuse state so the server's cached
   target-power status matches reality immediately.  A fresh boot/flash always comes
   up with both eFuses DISABLED, but the server would otherwise keep whatever it had
   cached (possibly a stale "on") until the next EN/FLT change produced an
   efuse.event.  A target_status reconcile can't fill this gap either: a disabled rail
   reads power-good low, so the server's reconcile treats it as untrustworthy and skips
   it.  The tracked EN state we send here is authoritative regardless of power-good, so
   it is the reliable "we booted, power is off" signal.  We read the CACHED state
   (target_power_get_cached — plain RAM, no I2C) because this runs on the net task and
   target_power_get_status's V/FLT reads would race the I2C bus.  Net-task only (called
   right after capabilities on connect). */
static void cl_send_efuse_snapshot(void) {
    for (int efuse = 1; efuse <= 2; efuse++) {
        bool enabled = false, fault = false;
        if (target_power_get_cached(efuse, &enabled, &fault) != 0) continue;
        cl_send_efuse_event(efuse, enabled, fault);
    }
}

/* Send an error command.response immediately (net task). */
/* Send a command.response frame. A full send buffer keeps it (in the caller's static frame,
   which nothing rewrites until the next response) for the poll loop to resend; any other
   failure is a broken link. A newer response replaces one still waiting: the server has
   timed that request out by then. */
static void cl_send_reply_frame(const char *buf, size_t len) {
    if (cl_ws_send(WS_OP_TEXT, buf, len)) { s_pend_len = 0; return; }
    if (s_send_full) { s_pend_buf = buf; s_pend_len = len; return; }
    cl_backoff("command.response send failed");
}

static void cl_send_command_error(const char *request_id, const char *error) {
    static char frame[BP_CLOUD_CMD_FRAME_MAX];
    bp_emit_t e;
    bp_emit_init(&e, frame, sizeof(frame));
    bp_emit(&e, "{\"type\":\"command.response\",\"request_id\":");
    bp_emit_jstr(&e, request_id);
    bp_emit(&e, ",\"device_id\":\"%s\",\"status\":\"error\",\"error\":", s_cfg.device_id);
    bp_emit_jstr(&e, error);
    bp_emit_raw(&e, "}");
    if (bp_emit_ok(&e)) cl_send_reply_frame(frame, bp_emit_len(&e));
}

/* A command.request arrived: hand the command object to the hw worker for
   execution (it owns command_handler).  The worker's reply is framed later by
   cloud_client_send_command_response (driven from the net task).  Command
   execution no longer runs inline here — that would run command_handler on the
   net task, racing the worker. */
static void cl_handle_command_request(const char *json) {
    char request_id[BP_TUNNEL_ID_MAX] = {0};
    cl_json_str(json, "request_id", request_id, sizeof(request_id));

    /* Static (a single cloud command is in flight at a time, guarded by s_cloud_pending):
       big enough for a compact closed-loop curve LUT carried inline. */
    static char command[BP_CLOUD_CMD_IN_MAX];
    if (!cl_json_object(json, "command", command, sizeof(command))) {
        /* bp_json_object fails the same way for "no such key" and "the object does not fit",
           and an over-budget command (e.g. a full 2048-point closed-loop curve inline) is by
           far the likelier of the two — say so, with the actual budget, instead of the
           misleading "missing command" the caller used to get. */
        if (strstr(json, "\"command\"")) {
            char msg[96];
            snprintf(msg, sizeof(msg),
                     "command too large for this device (limit %u bytes)",
                     (unsigned)(BP_CLOUD_CMD_IN_MAX - 1u));
            cl_send_command_error(request_id, msg);
        } else {
            cl_send_command_error(request_id, "missing command");
        }
        return;
    }
    if (!hw_worker_submit_cloud(request_id, command)) {
        /* A previous cloud command is still in flight, or the queue is full. The
           server will retry; report busy so it doesn't wait the full timeout. */
        cl_send_command_error(request_id, "device busy");
    }
}

void cloud_client_send_command_response(const char *request_id,
                                        const char *reply, size_t reply_len) {
    (void)reply_len;
    static char frame[BP_CLOUD_CMD_FRAME_MAX];
    /* The worker can finish a command after the link dropped.  A reply sent while a reconnect
       is mid-handshake landed in the HTTP upgrade exchange (or tripped a backoff); the server
       re-issues the command after reconnecting anyway, so drop it. */
    if (s_state != CL_CONNECTED) {
        printf("[cloud] dropping the reply to %s: link not connected (%s)\n",
               request_id, cloud_client_state_str());
        return;
    }
    if (strstr(reply, "\"status\":\"error\"")) {
        char msg[320] = {0};   /* pin-conflict / la_voltage refusals run to ~250 characters */
        cl_json_str(reply, "message", msg, sizeof(msg));
        cl_send_command_error(request_id, msg);
        return;
    }
    /* ok: graft the inner reply's status+data tail onto the envelope by skipping
       the inner's leading '{'. */
    const char *tail = reply;
    if (*tail == '{') tail++;
    bp_emit_t e;
    bp_emit_init(&e, frame, sizeof(frame));
    bp_emit(&e, "{\"type\":\"command.response\",\"request_id\":");
    bp_emit_jstr(&e, request_id);
    bp_emit(&e, ",\"device_id\":\"%s\",%s", s_cfg.device_id, tail);
    if (!bp_emit_ok(&e)) {
        /* An error the caller can read, not a reconnect (bp_limits.h sizes the frame for the
           largest captured reply, so this needs an unusually long request id). */
        cl_send_command_error(request_id, "reply too large for the cloud channel");
        return;
    }
    cl_send_reply_frame(frame, bp_emit_len(&e));
}

/* ---- Cloud byte-tunnel (flash/capture bridge) ----------------------------- */

/* lease.state: a cloud consumer took, renewed or released the pod's lease. While it holds it,
   LAN clients get light reads only (lease_gate.h, cloud-hardening.md section 2). */
static void cl_handle_lease_state(const char *json) {
    char held[8] = {0}, holder[LEASE_GATE_HOLDER_MAX] = {0}, exp[12] = {0};
    cl_json_str(json, "held", held, sizeof(held));
    cl_json_str(json, "holder", holder, sizeof(holder));
    cl_json_str(json, "expires_in_s", exp, sizeof(exp));
    bool on = strcmp(held, "true") == 0;
    lease_gate_update(on, holder, (uint32_t)strtoul(exp, NULL, 10), HAL_GetTick());
    printf("[cloud] lease %s%s\n", on ? "held by " : "released", on ? lease_gate_holder() : "");
}

/* End a tunnel because its byte stream can no longer be trusted (a frame was lost or garbled):
   tell the client why on the stream itself, close it towards the server with the reason, and
   reset the virtual connection so a half-done raw transfer (load_bin, spi_stream) ends at once
   instead of waiting out its idle timeout with a gap in the data. Net task. */
static void cl_tunnel_fail(int slot, const char *reason) {
    char id[BP_TUNNEL_ID_MAX];
    memcpy(id, s_tunnels[slot], sizeof(id));
    printf("[cloud] tunnel %s reset: %s\n", id, reason);
    char line[192];
    bp_emit_t e;
    bp_emit_init(&e, line, sizeof(line));
    bp_emit_raw(&e, "{\"status\":\"error\",\"message\":");
    bp_emit_jstr(&e, reason);
    bp_emit_raw(&e, "}\n");
    if (bp_emit_ok(&e)) cloud_client_tunnel_out(CH_CLOUD_TUNNEL_CONN + slot, (const uint8_t *)line, bp_emit_len(&e));
    char f[BP_TUNNEL_ID_MAX + 256];
    bp_emit_init(&e, f, sizeof(f));
    bp_emit_raw(&e, "{\"type\":\"tunnel.close\",\"tunnel_id\":");
    bp_emit_jstr(&e, id);
    bp_emit_raw(&e, ",\"reason\":");
    bp_emit_jstr(&e, reason);
    bp_emit_raw(&e, "}");
    if (bp_emit_ok(&e)) cl_ws_send(WS_OP_TEXT, f, bp_emit_len(&e));
    s_tunnels[slot][0] = '\0';
    conn_tx_reset(CH_CLOUD_TUNNEL_CONN + slot);
    hw_worker_submit_tunnel_reset(CH_CLOUD_TUNNEL_CONN + slot);
}

/* tunnel.open: bind a fresh virtual connection to this tunnel id. */
static void cl_handle_tunnel_open(const char *json) {
    char id[BP_TUNNEL_ID_MAX];
    id[0] = '\0';
    cl_json_str(json, "tunnel_id", id, sizeof(id));
    if (!id[0]) return;
    int slot = cl_tunnel_slot_by_id(id);      /* re-open of a known id reuses its slot */
    if (slot < 0) slot = cl_tunnel_free_slot();
    if (slot < 0) return;                      /* all tunnels in use (the server caps this) */
    strncpy(s_tunnels[slot], id, sizeof(s_tunnels[slot]) - 1);
    s_tunnels[slot][sizeof(s_tunnels[slot]) - 1] = '\0';
    /* The highest command tier the server allows this tunnel's user (policy-commands.md section
       3); an older server sends none, which means everything, as before. */
    char tier_s[8] = {0};
    int max_tier = cl_json_str(json, "max_tier", tier_s, sizeof(tier_s)) && tier_s[0] ? atoi(tier_s) : 3;
    command_handler_set_tunnel_max_tier(CH_CLOUD_TUNNEL_CONN + slot, max_tier);
    conn_tx_reset(CH_CLOUD_TUNNEL_CONN + slot);                  /* drop stale ring bytes */
    hw_worker_submit_tunnel_reset(CH_CLOUD_TUNNEL_CONN + slot);  /* start the virtual conn fresh */
}

/* tunnel.data (server→device): decode and feed the bytes through the command handler's state
   machine on the tunnel pseudo-connection, exactly as a local TCP client's bytes would flow.
   Returns false when the worker queue is full so the caller leaves the frame in s_rx and
   retries next poll — NON-BLOCKING backpressure. (An earlier version vTaskDelay-blocked the
   net task here; that DEADLOCKED a deep DAC upload running alongside the live UART proxy — a
   blocked net task stops draining the UART TX rings, so the worker's at_send_data stalls, so
   the worker stops draining this queue, so the net task never unblocks. Deferring instead
   keeps the net task servicing everything else while the worker catches up.) */
static bool cl_handle_tunnel_data(const char *json) {
    char id[BP_TUNNEL_ID_MAX];
    id[0] = '\0';
    cl_json_str(json, "tunnel_id", id, sizeof(id));
    int slot = cl_tunnel_slot_by_id(id);
    if (slot < 0) return true;                  /* unknown/closed tunnel — drop, don't retry */
    static char    b64[BP_CLOUD_RX_MAX];
    static uint8_t raw[B64URL_DECODED_MAX(sizeof(b64))];
    if (!cl_json_str(json, "data_b64", b64, sizeof(b64))) {
        cl_tunnel_fail(slot, "tunnel.data without data_b64");
        return true;
    }
    size_t rawlen = 0;
    if (b64url_decode(b64, raw, sizeof(raw), &rawlen) != 0) {
        cl_tunnel_fail(slot, "tunnel.data with bad base64");
        return true;
    }
    if (rawlen == 0) return true;
    /* Feed the bytes to the worker (it owns command_handler); FIFO-ordered behind this
       tunnel's open. Do NOT drop on a full queue: for a raw PROTO_LOAD stream (a deep DAC
       replay upload) a lost chunk corrupts the trace. Signal "retry" so the frame stays in
       s_rx (holds a burst — BP_CLOUD_RX_ACCUM) and is re-fed next poll once the worker drains,
       mirroring the LAN path's ERR_MEM redelivery in net_server.c. */
    return hw_worker_submit_bytes(CH_CLOUD_TUNNEL_CONN + slot, raw, rawlen);
}

/* tunnel.close: tear down the matching virtual connection (safe SWD/UART teardown happens in reset). */
static void cl_handle_tunnel_close(const char *json) {
    char id[BP_TUNNEL_ID_MAX];
    id[0] = '\0';
    cl_json_str(json, "tunnel_id", id, sizeof(id));
    int slot = cl_tunnel_slot_by_id(id);
    if (slot < 0) return;
    s_tunnels[slot][0] = '\0';
    conn_tx_reset(CH_CLOUD_TUNNEL_CONN + slot);
    hw_worker_submit_tunnel_reset(CH_CLOUD_TUNNEL_CONN + slot);
}

size_t cloud_client_tunnel_out(int conn_id, const uint8_t *buf, size_t len) {
    int slot = conn_id - CH_CLOUD_TUNNEL_CONN;
    if (slot < 0 || slot >= CH_CLOUD_TUNNEL_CONN_COUNT) return len;   /* nowhere to go: discard */
    if (!s_tunnels[slot][0] || len == 0) return len;
    /* Chunk so each tunnel.data frame fits cl_ws_send's frame budget after base64 + envelope. */
    char b64[B64URL_ENCODED_LEN(BP_TUNNEL_CHUNK) + 1];
    char frame[BP_TUNNEL_OUT_FRAME_MAX];
    size_t off = 0;
    while (off < len) {
        size_t n = len - off;
        if (n > BP_TUNNEL_CHUNK) n = BP_TUNNEL_CHUNK;
        if (b64url_encode(buf + off, n, b64, sizeof(b64)) == 0) return len;   /* cannot happen */
        int fn = snprintf(frame, sizeof(frame),
            "{\"type\":\"tunnel.data\",\"tunnel_id\":\"%s\",\"data_b64\":\"%s\"}",
            s_tunnels[slot], b64);
        if (fn <= 0 || (size_t)fn >= sizeof(frame)) return len;               /* cannot happen */
        /* A full send queue (ERR_MEM) is not a loss: report what went out and the caller keeps
           the rest in its ring for the next poll.  It used to return here while the caller
           advanced past everything, dropping bytes from the middle of a capture stream. */
        if (!cl_ws_send(WS_OP_TEXT, frame, (size_t)fn)) return off;
        off += n;
    }
    return len;
}

size_t cloud_client_tunnel_avail(int conn_id) {
    int slot = conn_id - CH_CLOUD_TUNNEL_CONN;
    if (slot < 0 || slot >= CH_CLOUD_TUNNEL_CONN_COUNT) return 0;
    if (!s_tunnels[slot][0] || !s_pcb) return 0;   /* no tunnel open / no link */
    /* Plaintext bytes we may hand the TLS stack right now.  altcp_mbedtls_sndbuf()
       already nets out the SSL record expansion and returns 0 before the handshake
       completes, so this is the real send-buffer headroom. */
    u16_t sb = altcp_sndbuf(s_pcb);
    if (sb <= 96u) return 0;
    /* Each raw tunnel byte costs ~1.5 bytes on the wire (base64 4/3 + the
       tunnel.data JSON envelope + WS header per <=768-byte frame), so budget half
       the send buffer (leaving margin for one frame's fixed envelope) as the raw
       byte count bulk_pump may stream now.  Returning an honest figure here is what
       lets at_send_avail() pace bulk_pump — instead of the old 0xFFFF stub that let
       it blast a whole capture and drop the trailing frames (the cloud-tunnel
       "capture timed out"). */
    return (size_t)((sb - 96u) / 2u);
}

/* ---- OTA over the WebSocket (server -> device ota.* frames) ---------------- */

/* ota.begin: {"size":N,"sha256":"<hex>"[,"target":"gw0|gw1|esp","version":V,"sig":"<b64url>"]}
   -> stage in PSRAM (on the worker). No target = the firmware; sig = the signed manifest. */
static bool cl_handle_ota_begin(const char *json) {
    char size_s[16] = {0}, sha[80] = {0}, target[16] = {0}, ver_s[16] = {0}, sig[200] = {0};
    if (!cl_json_str(json, "size", size_s, sizeof(size_s)) ||
        !cl_json_str(json, "sha256", sha, sizeof(sha)))
        return true;   /* malformed: consumed, not replayed */
    cl_json_str(json, "target", target, sizeof(target));
    cl_json_str(json, "version", ver_s, sizeof(ver_s));
    cl_json_str(json, "sig", sig, sizeof(sig));
    /* Also returns false on a full queue; losing the BEGIN loses the whole update, so it
       gets the same replay treatment as the data frames. */
    return hw_worker_submit_ota_begin((uint32_t)strtoul(size_s, NULL, 0), sha, target,
                                      (uint32_t)strtoul(ver_s, NULL, 0), sig);
}

/* ota.data: {"offset":O,"data_b64":"..."} -> decode + stage.

   Returns false ONLY when the worker queue is full, so the caller leaves the frame in
   s_rx and retries it next poll — the same backpressure tunnel.data has always had.

   This used to return void and DISCARD hw_worker_submit_ota_data()'s bool. The server
   pushes ota.data frames as fast as the socket accepts them and never waits for a
   per-frame ack (its sent_bytes counts what it wrote, not what landed), so once the
   worker queue filled — after about seven 1 KB frames — every remaining frame was
   dropped in silence. A 512 KB dry run reached the device as 7168 bytes and then stalled
   until the server gave up waiting for a verify result. OTA over the cloud could not
   have worked for any image larger than the queue, which is every real image. */
static bool cl_handle_ota_data(const char *json) {
    char off_s[16] = {0};
    static char    b64[BP_CLOUD_RX_MAX];
    static uint8_t raw[B64URL_DECODED_MAX(sizeof(b64))];
    cl_json_str(json, "offset", off_s, sizeof(off_s));
    /* A malformed frame is CONSUMED (true): retrying it would just spin — only a full
       queue is worth replaying. */
    if (!cl_json_str(json, "data_b64", b64, sizeof(b64))) return true;
    size_t rawlen = 0;
    if (b64url_decode(b64, raw, sizeof(raw), &rawlen) != 0 || rawlen == 0) return true;
    s_ota_frames_seen++;
    if (!hw_worker_submit_ota_data((uint32_t)strtoul(off_s, NULL, 0), raw, rawlen)) {
        s_ota_frames_requeued++;
        return false;
    }
    s_ota_frames_submitted++;
    return true;
}

/* Returns false only when a tunnel.data frame couldn't be handed to the worker (queue
   full) — the caller then leaves it in s_rx and retries next poll. All other frames are
   handled unconditionally and return true. */
static bool cl_handle_text_frame(const uint8_t *payload, size_t len) {
    /* NUL-terminate a copy so the flat JSON helpers can scan it. Sized to carry a full tunnel.data
       frame (≈1.4 KB raw → base64 + envelope). */
    static char msg[BP_CLOUD_RX_MAX];
    size_t n = len < sizeof(msg) - 1 ? len : sizeof(msg) - 1;
    memcpy(msg, payload, n);
    msg[n] = '\0';

    char type[32] = {0};
    if (len >= sizeof(msg)) {
        /* Never act on a cut-off frame: the flat JSON helpers would still find their keys and a
           shortened base64 field decodes fine, silently losing upload or OTA bytes. The ids are
           read from the whole frame: Go marshals keys in sorted order, so in a tunnel.data frame
           "tunnel_id" and "type" come AFTER the base64 data. */
        bp_json_scan_str((const char *)payload, len, "type", type, sizeof(type));
        printf("[cloud] dropping a %u-byte %s frame (limit %u)\n", (unsigned)len,
               type[0] ? type : "untyped", (unsigned)(sizeof(msg) - 1));
        if (strcmp(type, "command.request") == 0) {
            char rid[64] = {0};
            bp_json_scan_str((const char *)payload, len, "request_id", rid, sizeof(rid));
            if (rid[0]) cl_send_command_error(rid, "command too large");
        } else if (strcmp(type, "tunnel.data") == 0) {
            /* A raw stream (load_bin, spi_stream) cannot skip a frame: the bytes after it would
               land at the wrong offset. End the tunnel with the reason instead. */
            char id[BP_TUNNEL_ID_MAX] = {0};
            bp_json_scan_str((const char *)payload, len, "tunnel_id", id, sizeof(id));
            int slot = id[0] ? cl_tunnel_slot_by_id(id) : -1;
            if (slot >= 0) {
                char why[96];
                snprintf(why, sizeof(why), "tunnel.data frame too large (%u bytes, limit %u)",
                         (unsigned)len, (unsigned)(sizeof(msg) - 1));
                cl_tunnel_fail(slot, why);
            }
        }
        return true;
    }
    cl_json_str(msg, "type", type, sizeof(type));
    if (strcmp(type, "command.request") == 0) {
        cl_handle_command_request(msg);
    } else if (strcmp(type, "tunnel.data") == 0) {
        return cl_handle_tunnel_data(msg);   /* false => worker busy, retry this frame next poll */
    } else if (strcmp(type, "lease.state") == 0) {
        cl_handle_lease_state(msg);
    } else if (strcmp(type, "tunnel.open") == 0) {
        cl_handle_tunnel_open(msg);
    } else if (strcmp(type, "tunnel.close") == 0) {
        cl_handle_tunnel_close(msg);
    } else if (strcmp(type, "ota.begin") == 0) {
        return cl_handle_ota_begin(msg);  /* false => worker busy, retry this frame next poll */
    } else if (strcmp(type, "ota.data") == 0) {
        return cl_handle_ota_data(msg);   /* false => worker busy, retry this frame next poll */
    } else if (strcmp(type, "ota.end") == 0) {
        return hw_worker_submit_ota_end();     /* false => worker busy, retry this frame next poll */
    } else if (strcmp(type, "ota.abort") == 0) {
        return hw_worker_submit_ota_abort();   /* (these were dropped on a full queue) */
    } else if (strcmp(type, "ota.commit") == 0) {
        return hw_worker_submit_ota_commit();
    } else if (strcmp(type, "ping") == 0) {
        cl_ws_send(WS_OP_TEXT, "{\"type\":\"pong\"}", 15);
    }
    /* "pong" and anything else: ignore. */
    return true;
}

/* Push an ota.status frame when the OTA state changes or progress advances, so
   the server can track staging.  ota.c state is owned by the worker; these reads
   are atomic scalars (best-effort progress reporting). */
/* Progress-report interval while receiving; doubles as the server's pacing ACK. */
#define OTA_STATUS_EVERY  8192u
/* Also report at least this often while receiving, so a paced server never blocks waiting on a
   byte threshold we have not reached. Must stay well under the server's stall timeout. */
#define OTA_STATUS_TICK_MS  500u

/* What the last ota.status frame reported (cloud_client_tx_busy reads these too). */
static ota_state_t s_ota_last_state = OTA_IDLE;
static uint32_t    s_ota_last_refusals;

static void cl_ota_status_poll(void) {
    static uint32_t    last_reported;
    /* The cloud's view: the session, or the refusal its last ota.* frame got because another
       transport (LAN, USB) holds the session (ota.h). */
    ota_view_t  v;
    ota_view_for(OTA_OWNER_CLOUD, &v);
    ota_state_t st  = v.state;
    uint32_t    rcv = v.received;
    /* 8 KB, not 64 KB: this frame is not just a progress bar, it is the ACK the server
       paces its push against (see otaWindow in the server's benchpod_ota.go). At a 64 KB
       reporting interval the server had no usable catch-up signal inside a sane window, so
       it ran open-loop and outran the device — a 512 KB image reached the pod as ~20 KB.
       One status frame per 8 data frames is cheap; being outrun is not. */
    /* Report on the byte threshold OR a timer. The threshold alone DEADLOCKS a paced push: the
       server waits for our high-water to reach (pushed - window) before sending more, and while we
       sit below the next OTA_STATUS_EVERY boundary we never say where we are — so it waits for us
       and we wait for bytes that never come. Measured exactly that: every image larger than one
       window stalled at byte 33792, the first point the server pauses, while images that fit in
       one window verified fine. Adding the tick took the frames the device saw from 7 to 327. */
    static absolute_time_t next_tick;
    bool tick = (st == OTA_RECEIVING) && time_reached(next_tick);
    /* A refusal (another transport holds the session) is reported every time, even when it reads
       like the last one: the cloud view stays "error" between jobs, so a state-change test alone
       left a later job's refused begin unanswered and the server waited for acks that never came. */
    bool changed = (st != s_ota_last_state) || (v.refusals != s_ota_last_refusals) ||
                   (st == OTA_RECEIVING && rcv - last_reported >= OTA_STATUS_EVERY) ||
                   tick;
    if (!changed) return;

    char f[320];
    bp_emit_t e;
    bp_emit_init(&e, f, sizeof(f));
    bp_emit(&e, "{\"type\":\"ota.status\",\"device_id\":\"%s\",\"state\":\"%s\","
                "\"received\":%lu,\"size\":%lu,\"sig\":\"%s\",\"sig_key\":\"%s\",\"error\":",
            s_cfg.device_id, ota_state_name(st),
            (unsigned long)rcv, (unsigned long)v.size, ota_sig_result(), ota_sig_key_id());
    bp_emit_jstr(&e, v.error);
    bp_emit_raw(&e, "}");
    /* Only a frame that went out counts as reported: a full send buffer used to lose a state
       change (an error included) for good. Retried on the next poll. */
    if (!bp_emit_ok(&e) || !cl_ws_send(WS_OP_TEXT, f, bp_emit_len(&e))) return;
    next_tick = make_timeout_time_ms(OTA_STATUS_TICK_MS);
    s_ota_last_state = st;
    last_reported = rcv;
    s_ota_last_refusals = v.refusals;
}

/* Parse and act on any complete WS frames buffered in s_rx. */
static void cl_process_ws_frames(void) {
    for (;;) {
        ws_frame_t fr;
        int consumed = ws_parse_frame(s_rx, s_rx_len, &fr);
        if (consumed == 0) break;            /* need more bytes */
        if (consumed == WS_PARSE_FRAGMENTED) {
            cl_set_error("fragmented websocket message (not supported)");
            cl_backoff("fragmented ws frame");
            return;
        }
        if (consumed < 0) { cl_set_error("bad websocket frame"); cl_backoff("bad ws frame"); return; }

        switch (fr.opcode) {
            case WS_OP_TEXT:
                /* On worker-queue-full the handler returns false: STOP here without
                   consuming, so this frame (and those behind it, preserving order) stay
                   in s_rx and are retried next poll — non-blocking backpressure, the net
                   task keeps draining the TX rings meanwhile. */
                if (!cl_handle_text_frame(fr.payload, fr.payload_len)) return;
                break;
            case WS_OP_PING:
                cl_ws_send(WS_OP_PONG, (const char *)fr.payload, fr.payload_len);
                break;
            case WS_OP_CLOSE:
                cl_backoff("server closed ws");
                return;
            case WS_OP_PONG:
            default:
                break;
        }
        if (s_state != CL_CONNECTED) return;  /* a handler backed off; s_rx is gone */
        cl_rx_consume((size_t)consumed);
    }
}

/* ---- HTTP request builders ------------------------------------------------ */

static bool cl_send_challenge_post(void) {
    char req[256];
    int n = snprintf(req, sizeof(req),
        "POST /api/benchpod/devices/%s/challenge HTTP/1.1\r\n"
        "Host: %s\r\n"
        "Content-Length: 0\r\n"
        "Connection: keep-alive\r\n\r\n",
        s_cfg.device_id, s_cfg.host);
    return cl_tcp_send((const uint8_t *)req, (size_t)n) == 0;
}

static bool cl_send_ws_upgrade(void) {
    char key[25];
    ws_make_sec_key(key);
    char req[512];
    int n = snprintf(req, sizeof(req),
        "GET /api/benchpod/ws?device_id=%s&nonce=%s&signature=%s&auth=%s HTTP/1.1\r\n"
        "Host: %s\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Key: %s\r\n"
        "Sec-WebSocket-Version: 13\r\n\r\n",
        s_cfg.device_id, s_nonce, s_sig, s_auth_v1 ? "v1" : "v2", s_cfg.host, key);
    if (n <= 0 || (size_t)n >= sizeof(req)) return false;
    return cl_tcp_send((const uint8_t *)req, (size_t)n) == 0;
}

/* Decode the base64url nonce, sign  host || 0x00 || nonce  under the v2 WS-auth context, and
   base64url-encode the signature.  The host is the one this connection went to and whose
   certificate was checked, so a challenge relayed through another server cannot be signed for
   the real one (DEVICE_ID_CTX_WS_AUTH_V2; the server verifies with its own names, benchpod_ws.go). */
static bool cl_sign_nonce(void) {
    uint8_t nonce[128];
    size_t  nlen = 0;
    if (b64url_decode(s_nonce, nonce, sizeof(nonce), &nlen) != 0) return false;
    if (s_auth_v1) {   /* fallback for a server without v2 (see CL_WS_WAIT) */
        uint8_t sig1[DEVICE_ID_SIG_LEN];
        if (device_identity_sign_ctx(DEVICE_ID_CTX_WS_AUTH, nonce, nlen, sig1) != 0) return false;
        b64url_encode(sig1, sizeof(sig1), s_sig, sizeof(s_sig));
        return true;
    }
    uint8_t payload[CLOUD_HOST_MAX + 1 + sizeof(nonce)];
    size_t hlen = strnlen(s_cfg.host, CLOUD_HOST_MAX - 1);
    for (size_t i = 0; i < hlen; i++) {   /* host names compare case-insensitively: sign lowercase */
        char c = s_cfg.host[i];
        payload[i] = (uint8_t)((c >= 'A' && c <= 'Z') ? c - 'A' + 'a' : c);
    }
    payload[hlen] = 0x00;
    memcpy(payload + hlen + 1, nonce, nlen);
    uint8_t sig[DEVICE_ID_SIG_LEN];
    absolute_time_t t0 = get_absolute_time();
    if (device_identity_sign_ctx(DEVICE_ID_CTX_WS_AUTH_V2, payload, hlen + 1 + nlen, sig) != 0) return false;
    printf("[cloud] nonce sign (ed25519) took %lu ms\n",
           (unsigned long)(absolute_time_diff_us(t0, get_absolute_time()) / 1000));
    b64url_encode(sig, sizeof(sig), s_sig, sizeof(s_sig));
    return true;
}

/* ---- DNS ------------------------------------------------------------------ */

static void cl_dns_cb(const char *name, const ip_addr_t *ipaddr, void *arg) {
    (void)name; (void)arg;
    if (ipaddr) { s_ip = *ipaddr; s_dns = DNS_OK; }
    else        { s_dns = DNS_FAIL; }
}

/* ---- lifecycle ------------------------------------------------------------ */

void cloud_client_init(void) {
    /* Route async eFuse EN/FLT changes to our WS push queue (idempotent). */
    target_power_set_event_cb(cl_efuse_event_cb);

    bool loaded = cloud_config_load(&s_cfg) == 0;
#if defined(BENCHPOD_RELEASE)
    /* A plain config stored by older or dev firmware is not used by a release build (see
       handle_cloud_set): connecting without TLS would let anyone on the path stand in. */
    if (loaded && !s_cfg.tls && s_cfg.enabled) {
        printf("[cloud] stored config is plain ws: release firmware connects over TLS only\n");
        cl_set_error("plain (non-TLS) cloud config refused by release firmware; run cloud_set with tls");
        loaded = false;
    }
#endif
    if (loaded && s_cfg.enabled && s_cfg.host[0] && s_cfg.device_id[0]) {
        s_have_cfg = true;
        cl_set_state(CL_WAIT_WIFI);
        printf("[cloud] configured: %s:%u tls=%d device=%s\n",
               s_cfg.host, s_cfg.port, s_cfg.tls, s_cfg.device_id);
    } else {
        s_have_cfg = false;
        cl_set_state(CL_DISABLED);
    }
}

void cloud_client_reload(void) {
    cl_drop_link();                 /* closes any pcb that referenced s_tls_conf first */
    if (s_tls_conf) {
        /* Rebuild the TLS config next connect so a changed cfg.verify (CA vs no-CA)
           takes effect. */
        altcp_tls_free_config(s_tls_conf);
        s_tls_conf = NULL;
    }
    s_backoff_ms = BACKOFF_START_MS;
    s_dns = DNS_NONE;
    cl_set_error("%s", "");          /* a new config starts with a clean slate */
    cloud_client_init();
}

bool cloud_client_tx_busy(void) {
    if (s_state != CL_CONNECTED || !s_pcb) return false;
    if (hw_worker_cloud_pending() || s_pend_len) return true;
    for (int i = 0; i < CH_CLOUD_TUNNEL_CONN_COUNT; i++)
        if (s_tunnels[i][0] && conn_tx_used(CH_CLOUD_TUNNEL_CONN + i)) return true;
    ota_view_t v;
    ota_view_for(OTA_OWNER_CLOUD, &v);
    if (v.state != s_ota_last_state || v.refusals != s_ota_last_refusals) return true;
    /* Queued in TCP but not yet acknowledged: closing now could still lose it. */
    struct altcp_pcb *c = s_pcb;
    while (c->inner_conn) c = c->inner_conn;
    const struct tcp_pcb *t = (const struct tcp_pcb *)c->state;
    return t && (t->unsent || t->unacked);
}

/* ---- net-loop driver ------------------------------------------------------ */

void cloud_client_poll(void) {
    /* Serve a wedge-snapshot request latched by another task (e.g. the worker at a load_bin
       stall): log the link state HERE, on the net task, so altcp_sndbuf/s_pcb are read safely. */
    if (s_diag_req) {
        s_diag_req = false;
        cloud_client_log_link_state(s_diag_tag ? s_diag_tag : "req");
    }
    lease_gate_expire(HAL_GetTick());   /* the lease's writer is this task: it ends it too */
    switch (s_state) {
    case CL_DISABLED:
        return;

    case CL_WAIT_WIFI:
        /* Also wait (up to CL_EXTRAS_WAIT_MS after boot) for the company CA and proxy settings,
           which the hw worker reads from the W25Q at boot (cloud_extras.h). */
        if (wifi_get_state() == WIFI_READY &&
            (cloud_extras_ready() || to_ms_since_boot(get_absolute_time()) > CL_EXTRAS_WAIT_MS))
            cl_set_state(CL_RESOLVE);
        return;

    case CL_RESOLVE: {
        s_dns = DNS_PENDING;
        /* Through a proxy only the proxy needs resolving: the CONNECT line names the server and
           the proxy resolves it (corporate networks often cannot resolve outside names). */
        cloud_extras_proxy_get(&s_proxy);
        err_t e = dns_gethostbyname(s_proxy.host[0] ? s_proxy.host : s_cfg.host, &s_ip, cl_dns_cb, NULL);
        if (e == ERR_OK)             s_dns = DNS_OK;        /* cached */
        else if (e == ERR_INPROGRESS) s_dns = DNS_PENDING;
        else { cl_set_error("dns lookup failed"); cl_dns_backoff("dns request failed"); return; }
        s_deadline = make_timeout_time_ms(DNS_TIMEOUT_MS);
        cl_set_state(CL_RESOLVE_WAIT);
        return;
    }

    case CL_RESOLVE_WAIT:
        if (s_dns == DNS_OK)   { cl_set_state(CL_CHAL_OPEN); return; }
        if (s_dns == DNS_FAIL) { cl_set_error("dns lookup failed"); cl_dns_backoff("dns lookup failed"); return; }
        if (time_reached(s_deadline)) { cl_set_error("dns lookup timed out"); cl_dns_backoff("dns timeout"); }
        return;

    case CL_CHAL_OPEN:
        if (!cl_open(s_cfg.tls, s_cfg.port)) {
            cl_set_error("tcp connect failed");
            cl_backoff("challenge connect failed");
            return;
        }
        s_deadline = make_timeout_time_ms(CONNECT_TIMEOUT_MS);
        cl_set_state(CL_CHAL_CONNECT);
        return;

    case CL_CHAL_CONNECT:
        /* Latch TCP ESTABLISHED so a later failure reads as TLS, not TCP. */
        if (!s_tcp_up && cl_tcp_established()) s_tcp_up = true;
        if (s_link == LINK_UP) {
            if (!cl_tls_verified()) { cl_backoff("challenge TLS cert verify failed"); return; }
            if (!cl_send_challenge_post()) { cl_backoff("challenge POST failed"); return; }
            s_deadline = make_timeout_time_ms(HTTP_TIMEOUT_MS);
            cl_set_state(CL_CHAL_WAIT);
            return;
        }
        if (s_link == LINK_FAILED || s_dropped) {
            if (s_proxy.host[0] && s_proxy_conf.last_status && s_proxy_conf.last_status != 200)
                cl_set_error("the proxy refused the connection (HTTP %u%s)", (unsigned)s_proxy_conf.last_status,
                             s_proxy_conf.last_status == 407 ? ": proxy user or password" : "");
            else if (s_proxy.host[0] && (s_proxy_conf.closed_early || s_proxy_conf.last_status == 0))
                /* No reply from the proxy yet: it closed (FIN) or reset (RST) the connection. */
                cl_set_error("the proxy closed the connection before it replied");
            cl_backoff("challenge connect failed");
            return;
        }
        if (time_reached(s_deadline)) {
            cl_set_error(s_tcp_up ? "tls handshake timed out" : "tcp connect timed out");
            cl_backoff("challenge connect timeout");
        }
        return;

    case CL_CHAL_WAIT: {
        /* The challenge response is buffered in s_rx. The server sends it with an
           explicit Content-Length and Connection: keep-alive, so we wait for the
           full header+body, then reuse THIS SAME TLS link for the WS upgrade —
           no second handshake. An early server close here is a real failure. */
        if (s_dropped) { cl_set_error("connection lost"); cl_backoff("challenge link closed early"); return; }
        size_t body = cl_http_headers_end(s_rx, s_rx_len);
        if (body == 0) {                    /* headers not fully in yet */
            if (time_reached(s_deadline)) { cl_set_error("no reply from server"); cl_backoff("challenge timeout"); }
            return;
        }
        /* 404 = unknown device, 403 = deregistered (see handleBenchpodDeviceChallenge). */
        int code = cl_http_status(s_rx, body);
        if (code < 200 || code > 299) {
            cl_set_error("server refused the device (HTTP %d)", code);
            cl_backoff("challenge refused");
            return;
        }
        long clen = cl_content_length(s_rx, body);
        if (clen < 0) { cl_backoff("challenge missing content-length"); return; }
        size_t total = body + (size_t)clen;
        if (s_rx_len < total) {             /* body still arriving */
            if (time_reached(s_deadline)) { cl_set_error("no reply from server"); cl_backoff("challenge timeout"); }
            return;
        }
        /* Full response present. NUL-terminate the body and pull the nonce out. */
        size_t nul = total < sizeof(s_rx) ? total : sizeof(s_rx) - 1;
        uint8_t saved = s_rx[nul];
        s_rx[nul] = '\0';
        bool got = cl_json_str((const char *)(s_rx + body), "nonce", s_nonce, sizeof(s_nonce));
        s_rx[nul] = saved;
        if (!got) { cl_backoff("challenge nonce missing"); return; }
        /* Discard the whole challenge response so the WS 101 parser starts clean.
           Keep the link open. */
        cl_rx_consume(total);
        if (!cl_sign_nonce()) {
            if (device_identity_problem()[0])
                cl_set_error("device identity: %s", device_identity_problem());
            cl_backoff("sign failed");
            return;
        }
        if (!cl_send_ws_upgrade()) { cl_backoff("ws upgrade send failed"); return; }
        s_deadline = make_timeout_time_ms(HTTP_TIMEOUT_MS);
        cl_set_state(CL_WS_WAIT);
        return;
    }

    case CL_WS_WAIT: {
        if (s_dropped) { cl_set_error("websocket upgrade failed (connection closed)"); cl_backoff("ws closed during handshake"); return; }
        /* Find the end of the HTTP response headers (\r\n\r\n). */
        for (size_t i = 0; i + 3 < s_rx_len; i++) {
            if (s_rx[i] == '\r' && s_rx[i + 1] == '\n' &&
                s_rx[i + 2] == '\r' && s_rx[i + 3] == '\n') {
                if (!cl_status_is_101(s_rx, i)) {
                    int code = cl_http_status(s_rx, i);
                    cl_set_error("websocket upgrade failed (HTTP %d)", code);
                    /* A server from before the host-bound login rejects the v2 signature (HTTP
                       403 "signature mismatch" on those servers, 401 on newer ones): try v1 next
                       time. Safe, because a server that knows v2 refuses v1 from any pod that has
                       logged in with v2 once (cloud-hardening.md section 1). */
                    bool auth_refused = (code == 401 || code == 403);
                    if (auth_refused && !s_auth_v1) {
                        s_auth_v1 = true;
                        printf("[cloud] login v2 refused (HTTP %d): next attempt signs v1\n", code);
                    } else if (auth_refused) {
                        s_auth_v1 = false;   /* v1 refused too: back to v2 */
                    }
                    cl_backoff("ws handshake rejected");
                    return;
                }
                cl_rx_consume(i + 4);              /* leftover bytes are WS frames */
                /* Don't reset the backoff yet — only after the link proves stable
                   (CONN_STABLE_MS), so an immediate server-close doesn't spin us into a
                   fast reconnect storm (which starves the console with TLS handshakes). */
                s_stable_at = make_timeout_time_ms(CONN_STABLE_MS);
                s_backoff_reset_pending = true;
                s_last_rx = get_absolute_time();
                s_next_ping = make_timeout_time_ms(PING_INTERVAL_MS);
                cl_set_state(CL_CONNECTED);
                cl_set_error("%s", "");
                printf("[cloud] connected to %s as device %s\n", s_cfg.host, s_cfg.device_id);
                if (!cl_send_capabilities()) { cl_backoff("capabilities send failed"); return; }
                /* Announce the current eFuse state up front so the server's cached
                   target-power status reflects the just-booted (default-disabled) reality. */
                cl_send_efuse_snapshot();
                cl_process_ws_frames();
                return;
            }
        }
        if (time_reached(s_deadline)) { cl_set_error("websocket upgrade timed out"); cl_backoff("ws handshake timeout"); }
        return;
    }

    case CL_CONNECTED:
        if (s_dropped) { cl_set_error("connection lost"); cl_backoff("link dropped"); return; }
        if (s_rx_overflow) { cl_set_error("connection lost (rx overflow)"); cl_backoff("rx overflow"); return; }
        if (s_rx_len > 0) cl_process_ws_frames();
        if (s_state != CL_CONNECTED) return;
        /* Room again for what we refused: take it now rather than at the next segment. */
        if (s_rx_refused && s_pcb && s_rx_len < sizeof(s_rx) / 2) {
            s_rx_refused = false;
            if (s_cfg.tls) altcp_tls_bp_kick(s_pcb);
        }
        /* The link stayed up long enough to trust it — reset the backoff so the NEXT
           genuine drop reconnects promptly (a flapping link never reaches here). */
        if (s_backoff_reset_pending && time_reached(s_stable_at)) {
            s_backoff_ms = BACKOFF_START_MS;
            s_backoff_reset_pending = false;
        }
        /* While an OTA is actively receiving, the server LEGITIMATELY goes quiet: it paces the
           push to our acknowledged mark and will sit waiting for our next ota.status before
           sending another frame (otaWindowStall is 60 s). The plain 20 s inbound budget treats
           that deliberate silence as a dead link and reconnects mid-transfer, which kills the
           very transfer it is pacing — the transfer then never completes, no matter how correct
           the pacing is. Give an in-flight OTA a budget that outlasts the server's stall. */
        uint32_t idle_budget_ms = (ota_get_state() == OTA_RECEIVING)
                                      ? OTA_IDLE_TIMEOUT_MS : IDLE_TIMEOUT_MS;
        if (absolute_time_diff_us(s_last_rx, get_absolute_time()) > (int64_t)idle_budget_ms * 1000) {
            cloud_client_log_link_state("idle-timeout");   /* wedge diag: which direction died */
            cl_log_tls_window("idle-timeout");
            cl_set_error("connection lost (no reply from server)");
            cl_backoff("idle timeout (no pong)");
            return;
        }
        if (s_pend_len) {                          /* a reply that met a full buffer */
            if (cl_ws_send(WS_OP_TEXT, s_pend_buf, s_pend_len)) s_pend_len = 0;
            else if (!s_send_full) { cl_backoff("command.response send failed"); return; }
        }
        if (time_reached(s_next_ping)) {
            s_next_ping = make_timeout_time_ms(PING_INTERVAL_MS);
            /* RFC 6455 PING (not the app-level JSON {"type":"ping"}): the server's
               WS stack answers with a PONG, and any inbound frame refreshes s_last_rx
               (cl_recv_cb), so this keeps the idle-timeout from tripping even when
               there's no application traffic. */
            if (!cl_ws_send(WS_OP_PING, NULL, 0)) {
                if (s_send_full) {
                    /* Busy, not broken: try again shortly. A dead link still trips the idle
                       timeout above, so this cannot hide one. */
                    s_next_ping = make_timeout_time_ms(250);
                } else {
                    cl_backoff("ping send failed");
                    return;
                }
            }
        }
        /* Spontaneous push: drain any queued eFuse EN/FLT change events. */
        cl_drain_efuse_events();
        /* Re-announce capabilities after a runtime FPGA image swap so the server's cached caps
           (dac_control_loop / dac_deep_replay / dac_replay_max_samples) reflect the now-running
           image. Clear only on success; a transient full send buffer just retries next poll (no
           reconnect — the reconnect path already re-sends caps anyway). */
        if (s_resend_caps && cl_send_capabilities()) s_resend_caps = false;
        /* Report OTA staging progress/state transitions to the server. */
        cl_ota_status_poll();
        return;

    case CL_BACKOFF:
        if (time_reached(s_deadline)) cl_set_state(CL_RESOLVE);
        return;
    }
}
