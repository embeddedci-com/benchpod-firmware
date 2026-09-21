#include "cloud_client.h"
#include "cloud_config.h"
#include "wifi_manager.h"
#include "device_identity.h"
#include "command_handler.h"
#include "b64url.h"
#include "ws_frame.h"
#include "cloud_ca.h"
#include "board_info.h"
#include "signal_engine.h"   /* signal_engine_fpga_version / DAC_DEEP_REPLAY_MIN_GW / SIGNAL_MAX_SAMPLES */
#include "fpga_config.h"     /* FPGA_DAC_REPLAY_MAX_SAMPLES */
#include "cal_data.h"        /* ADC_CAL_EXT — front-SMA cal shipped in capabilities */
#include "target_power.h"
#include "bp_json.h"
#include "bp_limits.h"
#include "hw_worker.h"
#include "conn_tx.h"
#include "version.h"
#include "ota.h"

#include "FreeRTOS.h"
#include "task.h"            /* taskENTER_CRITICAL — guard the cross-task event slot */

#include "pico/time.h"
#include "pico/rand.h"

#include "lwip/opt.h"
#include "lwip/altcp.h"
#include "lwip/altcp_tcp.h"
#include "lwip/altcp_tls.h"
#include "lwip/dns.h"
#include "lwip/ip_addr.h"
#include "lwip/tcp.h"        /* TCP_WRITE_FLAG_COPY */

#include "mbedtls/ssl.h"     /* mbedtls_ssl_set_hostname (SNI) */

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
/* Idle budget while an OTA is staging. Longer than the normal budget because a paced push may
   legitimately go quiet between acknowledgements — but NOT much longer, because this is also how
   long we take to notice a genuinely dead downlink.
 *
 * Was 90 s, chosen when the server could pause indefinitely waiting on a byte threshold. The
 * server now rate-caps the push (a frame every ~16 ms at the default) and never goes quiet for
 * more than about a second, so 90 s bought nothing and cost a lot: every wedge left the pod
 * unreachable for a minute and a half before it even tried to reconnect, which is most of why an
 * interrupted transfer left the pod looking broken long afterwards. */
#define OTA_IDLE_TIMEOUT_MS  30000
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

static void cl_rx_consume(size_t n) {
    if (n >= s_rx_len) { s_rx_len = 0; return; }
    memmove(s_rx, s_rx + n, s_rx_len - n);
    s_rx_len -= n;
}

/* Append inbound (decrypted) bytes.  Runs in lwIP recv-callback context (the
   callbacks only set flags; poll() acts on them), so on overflow we don't tear the
   link down here — dropping oldest bytes would slice through a WS frame boundary
   and desync ws_parse_frame for the rest of the link, so flag an overflow and let
   poll() drop the whole buffer and reconnect (the server resends after reopen). */
static void cl_rx_append(const uint8_t *data, size_t len) {
    if (len > sizeof(s_rx) - s_rx_len) {
        s_rx_overflow = true;
        return;
    }
    memcpy(s_rx + s_rx_len, data, len);
    s_rx_len += len;
    if (s_rx_len > s_rx_len_peak) s_rx_len_peak = s_rx_len;
}

/* Close (or abort) the link we own and tear down any in-flight tunnel. Safe to
   call when s_pcb is already NULL (e.g. after the err callback freed it). */
static void cl_drop_link(void) {
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

static void cl_backoff(const char *reason) {
    printf("[cloud] backoff: %s\n", reason);
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

static err_t cl_recv_cb(void *arg, struct altcp_pcb *conn, struct pbuf *p, err_t err) {
    (void)arg;
    if (p == NULL) { s_dropped = true; return ERR_OK; }   /* peer sent FIN */
    if (err != ERR_OK) { pbuf_free(p); return err; }
    s_last_rx = get_absolute_time();
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
    if (tls) {
        if (!s_tls_conf) {
            /* TLS always anchors trust on the embedded ISRG roots (Let's Encrypt) so
               the server cert chain + hostname are validated in cl_tls_verified() after
               the handshake. authmode is VERIFY_OPTIONAL (the handshake completes; we
               check the result ourselves and drop the link on a verify failure). The
               old no-CA/unauthenticated bring-up path has been removed. */
            s_tls_conf = altcp_tls_create_config_client(cloud_ca_pem, cloud_ca_pem_len);
            if (!s_tls_conf) { printf("[cloud] altcp_tls config create failed\n"); return false; }
        }
        pcb = altcp_tls_new(s_tls_conf, IPADDR_TYPE_V4);
        if (!pcb) { printf("[cloud] altcp_tls_new failed (out of mem?)\n"); return false; }
        mbedtls_ssl_context *ssl = (mbedtls_ssl_context *)altcp_tls_context(pcb);
        if (ssl) mbedtls_ssl_set_hostname(ssl, s_cfg.host);   /* SNI + CN/SAN check host */
    } else {
        pcb = altcp_tcp_new_ip_type(IPADDR_TYPE_V4);
        if (!pcb) return false;
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
    if (altcp_write(s_pcb, buf, (u16_t)len, TCP_WRITE_FLAG_COPY) != ERR_OK) return -1;
    if (altcp_output(s_pcb) != ERR_OK) return -1;
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
    /* Live-read the gateware version first: the boot-time ping (signal_engine_init) can
       race the iCE40 configuring from flash and cache v0, which would wrongly advertise
       deep=false and make the server fall back to shallow replay (load_bin "total out of
       range").  By connect time the FPGA is up, so refresh from it (hw_lock-guarded). */
    signal_engine_refresh_version();
    /* Feature flags come from signal_engine_caps() — the SAME call the `status` reply uses, so
       a cloud client and a direct LAN/serial client cannot disagree about what this pod can do.
       (They used to: status carried a hardcoded caps[] literal that named none of these.) */
    signal_engine_caps_t caps;
    signal_engine_caps(&caps);
    bool deep      = caps.deep_replay;
    bool ctrl_loop = caps.control_loop;
    bool cotrig    = caps.cotrig;
    bool loop_src  = caps.loop_sources;
    bool loop_map  = caps.loop_input_map;
    unsigned long replay_max = deep ? (unsigned long)FPGA_DAC_REPLAY_MAX_SAMPLES
                                    : (unsigned long)SIGNAL_MAX_SAMPLES;
    char f[1024];   /* 768 until the pin/trigger/power flags; the frame is ~700 B with them */
    int n = snprintf(f, sizeof(f),
        "{\"type\":\"capabilities\",\"device_id\":\"%s\","
        "\"firmware_version\":\"%s\",\"ota\":true,"
        "\"serial\":false,\"scope\":true,\"analyzer\":true,\"command\":true,\"tunnel\":true,"
        "\"adc_bits\":%d,\"adc_fullscale_mv\":%d,\"adc_channels\":%d,"
        "\"adc_cal_a_uv\":%ld,\"adc_cal_b_nv\":%ld,\"adc_cal_unwrap\":true,"
        "\"dac\":%s,\"dac_replay\":%s,\"dac_dc\":%s,\"dac_bits\":%d,\"dac_replay_bits\":%d,"
        "\"dac_fullscale_mv\":%d,\"dac_channels\":%d,"
        "\"dac_deep_replay\":%s,\"dac_replay_max_samples\":%lu,"
        "\"dac_control_loop\":%s,\"dac_loop_sources\":%s,\"dac_loop_input_map\":%s,"
        "\"dac_cotrig\":%s,"
        "\"la_pins\":true,\"gpio_read\":%s,\"capture_trigger\":%s,\"power_profile\":true,"
        "\"board\":\"%s\"}",
        s_cfg.device_id, FIRMWARE_VERSION, ADC_BITS, ADC_FULLSCALE_MV, ADC_CHANNELS,
        lround((double)ADC_CAL_EXT.a * 1000000.0), lround((double)ADC_CAL_EXT.b * 1000000000.0),
        DAC_AC ? "true" : "false", DAC_REPLAY ? "true" : "false", DAC_DC ? "true" : "false",
        DAC_BITS, DAC_REPLAY_BITS, DAC_FULLSCALE_MV, DAC_CHANNELS,
        deep ? "true" : "false", replay_max,
        ctrl_loop ? "true" : "false", loop_src ? "true" : "false",
        loop_map ? "true" : "false", cotrig ? "true" : "false",
        caps.gpio_read ? "true" : "false", caps.capture_trigger ? "true" : "false",
        BOARD_NAME);
    if (n <= 0 || (size_t)n >= sizeof(f)) return false;
    return cl_ws_send(WS_OP_TEXT, f, (size_t)n);
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
static void cl_send_command_error(const char *request_id, const char *error) {
    static char frame[BP_CLOUD_CMD_FRAME_MAX];
    bp_emit_t e;
    bp_emit_init(&e, frame, sizeof(frame));
    bp_emit(&e, "{\"type\":\"command.response\",\"request_id\":");
    bp_emit_jstr(&e, request_id);
    bp_emit(&e, ",\"device_id\":\"%s\",\"status\":\"error\",\"error\":", s_cfg.device_id);
    bp_emit_jstr(&e, error);
    bp_emit_raw(&e, "}");
    if (bp_emit_ok(&e)) cl_ws_send(WS_OP_TEXT, frame, bp_emit_len(&e));
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
    if (!bp_emit_ok(&e) || !cl_ws_send(WS_OP_TEXT, frame, bp_emit_len(&e))) {
        cl_backoff("command.response send failed");
    }
}

/* ---- Cloud byte-tunnel (flash/capture bridge) ----------------------------- */

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
    if (!cl_json_str(json, "data_b64", b64, sizeof(b64))) return true;
    size_t rawlen = 0;
    if (b64url_decode(b64, raw, sizeof(raw), &rawlen) != 0 || rawlen == 0) return true;
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

void cloud_client_tunnel_out(int conn_id, const uint8_t *buf, size_t len) {
    int slot = conn_id - CH_CLOUD_TUNNEL_CONN;
    if (slot < 0 || slot >= CH_CLOUD_TUNNEL_CONN_COUNT) return;
    if (!s_tunnels[slot][0] || len == 0) return;
    /* Chunk so each tunnel.data frame fits cl_ws_send's frame budget after base64 + envelope. */
    char b64[B64URL_ENCODED_LEN(BP_TUNNEL_CHUNK) + 1];
    char frame[BP_TUNNEL_OUT_FRAME_MAX];
    size_t off = 0;
    while (off < len) {
        size_t n = len - off;
        if (n > BP_TUNNEL_CHUNK) n = BP_TUNNEL_CHUNK;
        if (b64url_encode(buf + off, n, b64, sizeof(b64)) == 0) return;
        int fn = snprintf(frame, sizeof(frame),
            "{\"type\":\"tunnel.data\",\"tunnel_id\":\"%s\",\"data_b64\":\"%s\"}",
            s_tunnels[slot], b64);
        if (fn <= 0 || (size_t)fn >= sizeof(frame)) return;
        if (!cl_ws_send(WS_OP_TEXT, frame, (size_t)fn)) return;
        off += n;
    }
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

/* ota.begin: {"size":N,"sha256":"<hex>"} -> stage in PSRAM (on the worker). */
static bool cl_handle_ota_begin(const char *json) {
    char size_s[16] = {0}, sha[80] = {0};
    if (!cl_json_str(json, "size", size_s, sizeof(size_s)) ||
        !cl_json_str(json, "sha256", sha, sizeof(sha)))
        return true;   /* malformed: consumed, not replayed */
    /* Also returns false on a full queue; losing the BEGIN loses the whole update, so it
       gets the same replay treatment as the data frames. */
    return hw_worker_submit_ota_begin((uint32_t)strtoul(size_s, NULL, 0), sha);
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
    cl_json_str(msg, "type", type, sizeof(type));
    if (strcmp(type, "command.request") == 0) {
        cl_handle_command_request(msg);
    } else if (strcmp(type, "tunnel.data") == 0) {
        return cl_handle_tunnel_data(msg);   /* false => worker busy, retry this frame next poll */
    } else if (strcmp(type, "tunnel.open") == 0) {
        cl_handle_tunnel_open(msg);
    } else if (strcmp(type, "tunnel.close") == 0) {
        cl_handle_tunnel_close(msg);
    } else if (strcmp(type, "ota.begin") == 0) {
        return cl_handle_ota_begin(msg);  /* false => worker busy, retry this frame next poll */
    } else if (strcmp(type, "ota.data") == 0) {
        return cl_handle_ota_data(msg);   /* false => worker busy, retry this frame next poll */
    } else if (strcmp(type, "ota.end") == 0) {
        hw_worker_submit_ota_end();
    } else if (strcmp(type, "ota.abort") == 0) {
        hw_worker_submit_ota_abort();
    } else if (strcmp(type, "ota.commit") == 0) {
        hw_worker_submit_ota_commit();
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

static void cl_ota_status_poll(void) {
    static ota_state_t last_state = OTA_IDLE;
    static uint32_t    last_reported;
    ota_state_t st  = ota_get_state();
    uint32_t    rcv = ota_received();
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
    bool changed = (st != last_state) ||
                   (st == OTA_RECEIVING && rcv - last_reported >= OTA_STATUS_EVERY) ||
                   tick;
    if (!changed) return;
    next_tick = make_timeout_time_ms(OTA_STATUS_TICK_MS);
    last_state = st;
    last_reported = rcv;

    char f[256];
    bp_emit_t e;
    bp_emit_init(&e, f, sizeof(f));
    bp_emit(&e, "{\"type\":\"ota.status\",\"device_id\":\"%s\",\"state\":\"%s\","
                "\"received\":%lu,\"size\":%lu,\"error\":",
            s_cfg.device_id, ota_state_str(),
            (unsigned long)rcv, (unsigned long)ota_size());
    bp_emit_jstr(&e, ota_error());
    bp_emit_raw(&e, "}");
    if (bp_emit_ok(&e)) cl_ws_send(WS_OP_TEXT, f, bp_emit_len(&e));
}

/* Parse and act on any complete WS frames buffered in s_rx. */
static void cl_process_ws_frames(void) {
    for (;;) {
        ws_frame_t fr;
        int consumed = ws_parse_frame(s_rx, s_rx_len, &fr);
        if (consumed == 0) break;            /* need more bytes */
        if (consumed < 0) { cl_backoff("bad ws frame"); return; }

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
        "GET /api/benchpod/ws?device_id=%s&nonce=%s&signature=%s HTTP/1.1\r\n"
        "Host: %s\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Key: %s\r\n"
        "Sec-WebSocket-Version: 13\r\n\r\n",
        s_cfg.device_id, s_nonce, s_sig, s_cfg.host, key);
    if (n <= 0 || (size_t)n >= sizeof(req)) return false;
    return cl_tcp_send((const uint8_t *)req, (size_t)n) == 0;
}

/* Decode the base64url nonce, sign it under the WS-auth domain context, and
   base64url-encode the signature.  The server verifies over the SAME
   DEVICE_ID_CTX_WS_AUTH-prefixed message (see benchpod_ws.go). */
static bool cl_sign_nonce(void) {
    uint8_t nonce[128];
    size_t  nlen = 0;
    if (b64url_decode(s_nonce, nonce, sizeof(nonce), &nlen) != 0) return false;
    uint8_t sig[DEVICE_ID_SIG_LEN];
    absolute_time_t t0 = get_absolute_time();
    if (device_identity_sign_ctx(DEVICE_ID_CTX_WS_AUTH, nonce, nlen, sig) != 0) return false;
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

    if (cloud_config_load(&s_cfg) == 0 && s_cfg.enabled && s_cfg.host[0] && s_cfg.device_id[0]) {
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

/* ---- net-loop driver ------------------------------------------------------ */

void cloud_client_poll(void) {
    /* Serve a wedge-snapshot request latched by another task (e.g. the worker at a load_bin
       stall): log the link state HERE, on the net task, so altcp_sndbuf/s_pcb are read safely. */
    if (s_diag_req) {
        s_diag_req = false;
        cloud_client_log_link_state(s_diag_tag ? s_diag_tag : "req");
    }
    switch (s_state) {
    case CL_DISABLED:
        return;

    case CL_WAIT_WIFI:
        if (wifi_get_state() == WIFI_READY) cl_set_state(CL_RESOLVE);
        return;

    case CL_RESOLVE: {
        s_dns = DNS_PENDING;
        err_t e = dns_gethostbyname(s_cfg.host, &s_ip, cl_dns_cb, NULL);
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
        if (s_link == LINK_FAILED) { cl_backoff("challenge connect failed"); return; }
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
        if (!cl_sign_nonce()) { cl_backoff("sign failed"); return; }
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
                    cl_set_error("websocket upgrade failed (HTTP %d)", cl_http_status(s_rx, i));
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
            cl_set_error("connection lost (no reply from server)");
            cl_backoff("idle timeout (no pong)");
            return;
        }
        if (time_reached(s_next_ping)) {
            s_next_ping = make_timeout_time_ms(PING_INTERVAL_MS);
            /* RFC 6455 PING (not the app-level JSON {"type":"ping"}): the server's
               WS stack answers with a PONG, and any inbound frame refreshes s_last_rx
               (cl_recv_cb), so this keeps the idle-timeout from tripping even when
               there's no application traffic. */
            if (!cl_ws_send(WS_OP_PING, NULL, 0)) {
                cl_backoff("ping send failed");
                return;
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
