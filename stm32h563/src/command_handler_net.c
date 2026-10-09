/*
 * command_handler_net.c — network provisioning and the cloud speed test.
 *
 *   cloud_set / cloud_status / cloud_clear   the cloud endpoint (cloud_config.h)
 *   wifi_set / wifi_clear / wifi_status      the ESP32-C3 Wi-Fi credentials (config_store.h)
 *   eth                                      the wired interface (net_server.h, eth_diag.h)
 *   speedtest                                a synthetic cloud throughput probe; its download
 *                                            direction is the PROTO_SPEEDTEST raw mode
 *
 * Runs on the hw worker task, like the rest of command_handler.
 */
#include "command_handler.h"
#include "command_handler_internal.h"
#include "at_driver.h"
#include "bp_json.h"
#include "bp_err.h"
#include "cloud_config.h"
#include "cloud_client.h"
#include "net_server.h"     /* net_eth_*, net_*_reload_after_reply */
#include "config_store.h"
#include "esp_wifi_ctrl.h"
#include "esp_hosted_spi.h"
#include "pico_compat.h"    /* sleep_ms (yields to FreeRTOS), time_reached */
#include "net_reload.h"     /* net_reload_pending: the wifi_clear reply is out */
#include "boot_guard.h"     /* boot_guard_skip_hw: no W25Q in safe mode */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* JSON parsing is shared (see bp_json.h); same aliases as command_handler.c. */
#define json_get_value      bp_json_get

/* ---- Cloud throughput speed test -----------------------------------------
   A synthetic transfer used to BENCHMARK the cloud data path (device<->server
   over the WS/Cloudflare), independent of any real capture/DAC hardware. It runs
   over a TUNNEL conn (streaming is banned on the single-reply command channel),
   reuses the same conn_tx/altcp_sndbuf pacing as bulk_pump / load_bin, and adds
   NO large static buffers (bss is tight). Two directions:
     "up"   — device streams `total` synthetic bytes out (server times receipt);
     "down" — device sinks `total` bytes (PROTO_SPEEDTEST, counts+discards) then
              acks — exercises the same direction as a load_bin upload.
   Completion is marked with a `{"speedtest":"done","bytes":N}` line the server
   watches for. Only one runs at a time (one cloud data path). */
static struct {
    bool   active;   /* upload source running */
    int    conn;
    size_t total;
    size_t sent;
} sptx;

/* Ack cadence for the download sink: the device reports cumulative bytes-received
   every SPEEDTEST_ACK_INTERVAL so the server can pace to a byte-window and never
   get more than a window ahead of what the pod has actually taken off the wire.
   This is the end-to-end backpressure the server->device path otherwise lacks
   (conn.Write into Cloudflare succeeds even when CF->pod delivery has stalled). */
#define SPEEDTEST_ACK_INTERVAL 8192u

static struct {
    size_t total;    /* download sink target (conn is in PROTO_SPEEDTEST) */
    size_t recv;
    size_t last_ack; /* recv value at the last ack sent */
} sprx;

static void speedtest_ack_progress(int conn_id, size_t bytes) {
    char m[64];
    snprintf(m, sizeof(m), "{\"speedtest\":\"ack\",\"bytes\":%u}", (unsigned)bytes);
    cloud_send_json_line(conn_id, m);
}

static void speedtest_ack_done(int conn_id, size_t bytes) {
    char m[64];
    snprintf(m, sizeof(m), "{\"speedtest\":\"done\",\"bytes\":%u}", (unsigned)bytes);
    cloud_send_json_line(conn_id, m);
}

/* Cancel any in-flight speed test bound to conn_id (tunnel reset / close). */
void speedtest_conn_closed(int conn_id) {
    if (sptx.active && sptx.conn == conn_id) sptx.active = false;
}

/* Paced upload pump — mirrors bulk_pump: fill the conn's TX ring as far as its
   free space allows, resume next worker poll. Synthetic 0x5A payload from the
   stack (no static buffer). */
void speedtest_pump(void) {
    if (!sptx.active) return;
    int conn = sptx.conn;
    uint8_t chunk[256];
    memset(chunk, 0x5A, sizeof(chunk));
    while (sptx.sent < sptx.total) {
        int avail = at_send_avail(conn);
        if (avail < 64) return;                 /* ring full — resume next poll */
        size_t room = (size_t)avail;
        if (room > sizeof(chunk)) room = sizeof(chunk);
        size_t need = sptx.total - sptx.sent;
        if (room > need) room = need;
        if (at_send_data(conn, chunk, room) != 0) { sptx.active = false; return; }
        sptx.sent += room;
    }
    if (at_send_avail(conn) < 72) return;       /* the done line goes out whole: next poll */
    sptx.active = false;
    speedtest_ack_done(conn, sptx.sent);        /* ordered after the payload in the ring */
}

/* PROTO_SPEEDTEST receive path (command_handler_process): count + DISCARD `total` raw bytes (no
   buffer), then ack with the done marker. Same byte-counted ingest as PROTO_LOAD but throws the
   data away — it only measures server->device throughput. Returns the bytes taken from buf. */
size_t speedtest_receive(int conn_id, const uint8_t *buf, size_t len) {
    (void)buf;
    size_t need = sprx.total - sprx.recv;
    size_t take = need < len ? need : len;
    sprx.recv += take;
    if (sprx.recv >= sprx.total) {
        conn_proto_set(conn_id, PROTO_JSON);
        speedtest_ack_done(conn_id, sprx.recv);
    } else if (sprx.recv - sprx.last_ack >= SPEEDTEST_ACK_INTERVAL) {
        sprx.last_ack = sprx.recv;
        speedtest_ack_progress(conn_id, sprx.recv);   /* window feedback to the server */
    }
    return take;
}

/* ---- Cloud connection provisioning ----
   cloud_set persists the server endpoint + this device's id and tells the
   firmware to open the control WebSocket itself (and reconnect on boot).
   Provisioned once by `benchpod register`. */
static bool cloud_truthy(const char *s) {
    return s[0] == 't' || s[0] == 'T' || s[0] == '1';
}

void handle_cloud_set(int conn_id, const char *json) {
    char host[CLOUD_HOST_MAX]      = {0};
    char did[CLOUD_DEVICE_ID_MAX]  = {0};
    char port_s[8] = {0}, tls_s[8] = {0}, en_s[8] = {0};

    /* Refuse over-long values instead of saving them cut short (a truncated host never connects). */
    int hr = bp_json_get_fit(json, "host", host, sizeof(host));
    if (hr == 0)  { send_error(conn_id, "missing host");  return; }
    if (hr < 0)   { send_error(conn_id, "host too long"); return; }
    int dr = bp_json_get_fit(json, "device_id", did, sizeof(did));
    if (dr == 0)  { send_error(conn_id, "missing device_id");  return; }
    if (dr < 0)   { send_error(conn_id, "device_id too long"); return; }
    json_get_value(json, "port", port_s, sizeof(port_s));
    json_get_value(json, "tls", tls_s, sizeof(tls_s));
    bool have_en  = json_get_value(json, "enabled", en_s, sizeof(en_s)) != 0;

    bool tls = cloud_truthy(tls_s);
#if defined(BENCHPOD_RELEASE)
    /* A plain ws:// link has no certificate to check, so anyone on the path could stand in for
       the server. Release firmware only talks TLS; dev builds keep plain for local servers. */
    if (!tls) { send_error(conn_id, "cloud_set: release firmware needs \"tls\":true (plain ws is for development builds)"); return; }
#endif
    int  port = atoi(port_s);
    if (port <= 0 || port > 65535) port = tls ? 443 : 80;

    cloud_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.magic   = CLOUD_CONFIG_MAGIC;
    cfg.version = CLOUD_CONFIG_VERSION;
    strncpy(cfg.host, host, sizeof(cfg.host) - 1);
    strncpy(cfg.device_id, did, sizeof(cfg.device_id) - 1);
    cfg.port    = (uint16_t)port;
    cfg.tls     = tls ? 1 : 0;
    cfg.enabled = (!have_en || cloud_truthy(en_s)) ? 1 : 0;   /* default enabled */
    /* TLS ALWAYS verifies the server cert chain + hostname against the embedded ISRG
       roots (cloud_ca.h). The old "verify":false bring-up override has been removed:
       an encrypted-but-unauthenticated link is MITM-able, and leaving the knob in the
       provisioning path was one typo away from shipping an unauthenticated pod. A
       "verify" field in the request is now ignored. */
    cfg.verify  = tls ? 1 : 0;

    if (cloud_config_save(&cfg) != 0) { send_error(conn_id, "config save failed"); return; }
    net_cloud_reload_after_reply();   /* pick up the new config and (re)connect, once this reply is out */

    char resp[256];
    snprintf(resp, sizeof(resp),
             "{\"status\":\"ok\",\"data\":{\"host\":\"%s\",\"port\":%u,\"tls\":%s,\"verify\":%s,"
             "\"enabled\":%s,\"device_id\":\"%s\"}}\n",
             cfg.host, cfg.port, cfg.tls ? "true" : "false", cfg.verify ? "true" : "false",
             cfg.enabled ? "true" : "false", cfg.device_id);
    if (at_send_data(conn_id, (const uint8_t *)resp, strlen(resp)) != 0) {
        at_close_connection(conn_id);
    }
}

void handle_cloud_status(int conn_id, const char *json) {
    (void)json;
    cloud_config_t cfg;
    bool have = (cloud_config_load(&cfg) == 0);
    char last_error[80];
    cloud_client_last_error(last_error, sizeof(last_error));
    /* last_error, host and device_id are free text: escape them (a quote broke the reply). */
    char resp[640];
    bp_emit_t e;
    bp_emit_init(&e, resp, sizeof(resp));
    bp_emit(&e, "{\"status\":\"ok\",\"data\":{\"state\":\"%s\",\"last_error\":",
            cloud_client_state_str());
    bp_emit_jstr(&e, last_error);
    bp_emit(&e, ",\"configured\":%s,\"host\":", have ? "true" : "false");
    bp_emit_jstr(&e, have ? cfg.host : "");
    bp_emit(&e, ",\"port\":%u,\"tls\":%s,\"verify\":%s,\"device_id\":",
            have ? cfg.port : 0, (have && cfg.tls) ? "true" : "false",
            (have && cfg.verify) ? "true" : "false");
    bp_emit_jstr(&e, have ? cfg.device_id : "");
    bp_emit_raw(&e, "}}\n");
    if (!bp_emit_ok(&e) ||
        at_send_data(conn_id, (const uint8_t *)resp, bp_emit_len(&e)) != 0) {
        at_close_connection(conn_id);
    }
}

void handle_cloud_clear(int conn_id, const char *json) {
    (void)json;
    cloud_config_clear();
    net_cloud_reload_after_reply();   /* drops to DISABLED once this reply is out */
    send_ok_str(conn_id, "\"cleared\"");
}

/* ---- Wi-Fi provisioning (ESP32-C3 via esp-hosted) ----
   Stores the SSID/passphrase that esp_wifi_ctrl associates with; the wired
   Ethernet stays primary, Wi-Fi is the fallback (see net_server multihoming). */
void handle_wifi_set(int conn_id, const char *json) {
    config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.magic = CONFIG_MAGIC; cfg.version = CONFIG_VERSION;
    int sr = bp_json_get_fit(json, "ssid", cfg.ssid, sizeof(cfg.ssid));
    if (sr == 0) { send_error(conn_id, "missing ssid"); return; }
    if (sr < 0)  { send_error(conn_id, "ssid too long"); return; }   /* never save it cut short */
    if (bp_json_get_fit(json, "password", cfg.password, sizeof(cfg.password)) < 0) {   /* open AP: empty */
        send_error(conn_id, "password too long"); return;
    }
    if (config_save(&cfg) != 0) { send_error(conn_id, "config save failed"); return; }
    net_wifi_reload_after_reply();   /* (re)connect with the new creds, once this reply is out */

    char resp[160];
    bp_emit_t e;                          /* the SSID is escaped: it may hold a quote */
    bp_emit_init(&e, resp, sizeof(resp));
    bp_emit_raw(&e, "{\"status\":\"ok\",\"data\":{\"ssid\":");
    bp_emit_jstr(&e, cfg.ssid);
    bp_emit_raw(&e, "}}\n");
    if (at_send_data(conn_id, (const uint8_t *)resp, bp_emit_len(&e)) != 0) at_close_connection(conn_id);
}

/* ---- wifi_clear: the ESP32-C3's copy -------------------------------------------------------
   Pod firmware up to 3.7.0 left the C3 on the IDF default WIFI_STORAGE_FLASH, so the C3 kept the
   SSID and password in its own NVS partition too. wifi_clear erases that partition after the
   config store. Over JSON it runs after the reply, like the Wi-Fi drop: the reply may ride the
   Wi-Fi link, and the erase holds the C3 in its ROM loader for ~3 s. wifi_status reports the
   outcome as c3_nvs. The console runs it at once (wifi_clear_c3_now). */
typedef enum { C3_NVS_NONE = 0, C3_NVS_PENDING, C3_NVS_ERASED, C3_NVS_FAILED } c3_nvs_t;
static c3_nvs_t        s_c3_nvs;          /* since boot: no wifi_clear yet */
static absolute_time_t s_c3_giveup;       /* a pending erase stops waiting for the bus here */
#define C3_WIPE_WAIT_MS 60000u            /* for the reply to go out and the PSRAM bus to free */

static const char *c3_nvs_str(void) {
    switch (s_c3_nvs) {
    case C3_NVS_PENDING: return "pending";
    case C3_NVS_ERASED:  return "erased";
    case C3_NVS_FAILED:  return "failed";
    default:             return "none";
    }
}

const char *wifi_clear_c3_now(void) {
    const char *why = NULL;
    if (boot_guard_skip_hw())
        why = "safe mode: the W25Q that holds the C3 image is off";
    else if (!(why = bus_busy_reason()) && !net_wifi_wipe_c3())
        why = "the C3 did not answer its ROM loader or the erase did not verify (see log)";
    s_c3_nvs = why ? C3_NVS_FAILED : C3_NVS_ERASED;
    if (why) printf("[wifi] ESP32-C3 NVS not erased: %s\n", why);
    return why;
}

void wifi_clear_poll(void) {
    if (s_c3_nvs != C3_NVS_PENDING) return;
    bool late = time_reached(s_c3_giveup);
    if (!late && net_reload_pending()) return;   /* the reply and the Wi-Fi drop go first */
    if (!late && !boot_guard_skip_hw() && bus_busy_reason()) return;   /* a capture is running */
    (void)wifi_clear_c3_now();
}

void handle_wifi_clear(int conn_id, const char *json) {
    (void)json;
    config_clear();
    net_wifi_reload_after_reply();   /* drops Wi-Fi once this reply is out (the cloud may ride it) */
    s_c3_nvs    = C3_NVS_PENDING;    /* then wifi_clear_poll erases the C3's NVS */
    s_c3_giveup = make_timeout_time_ms(C3_WIPE_WAIT_MS);
    send_ok_str(conn_id, "\"cleared\"");
}

/* {"cmd":"eth","action":"stop"|"start"|"restart"} — manually control the wired
   interface (down/up, PHY reset + DHCP re-acquire) without a power cycle. The
   action is latched and applied on the net task; the reply just confirms intent.
   {"cmd":"eth","action":"stats"} returns the wired-link diagnostics (eth_diag.h).
   {"cmd":"eth","action":"speed","mbit":0|10|100[,"duplex":"half"|"full"]} forces the link
   mode (0 = back to autonegotiation) — a debug aid for a link that errors at 100M.
   {"cmd":"eth","action":"refclk"} measures the PHY's RMII reference clock (nominally
   50 MHz) against the MCU crystal and returns {hz, ppm, on_hsi}.
   {"cmd":"eth","action":"loopback","mbit":10|100[,"n":N]} runs a PHY near-end loopback
   test and returns {mbit, sent, received, intact, corrupt, crc, align, tx_fail}. */
bool clock_on_hsi(void);   /* main.c: true when the MCU crystal did not start */

void handle_eth(int conn_id, const char *buf) {
    char action[16] = {0};
    json_get_value(buf, "action", action, sizeof(action));
    if (strcmp(action, "stats") == 0) {
        eth_diag_t d;
        char data[512], resp[560];
        net_eth_diag(&d);
        eth_diag_json(&d, data, sizeof(data));
        bp_emit_t e;
        bp_emit_init(&e, resp, sizeof(resp));
        bp_emit_raw(&e, "{\"status\":\"ok\",\"data\":");
        bp_emit_raw(&e, data);
        bp_emit_raw(&e, "}\n");
        if (!bp_emit_ok(&e)) { send_error(conn_id, bp_err_str(BP_ERR_TOO_LARGE)); return; }
        if (at_send_data(conn_id, (const uint8_t *)resp, bp_emit_len(&e)) != 0) at_close_connection(conn_id);
        return;
    }
    if (strcmp(action, "loopback") == 0) {
        char mbuf[8] = {0}, nbuf[8] = {0};
        json_get_value(buf, "mbit", mbuf, sizeof(mbuf));
        int mbit = atoi(mbuf);
        if (mbit != 10 && mbit != 100) { send_error(conn_id, "eth loopback mbit must be 10 or 100"); return; }
        uint32_t n = json_get_value(buf, "n", nbuf, sizeof(nbuf)) && nbuf[0] ? (uint32_t)atoi(nbuf) : 200u;
        if (n == 0) n = 1;
        if (n > 1000) n = 1000;
        uint32_t seq = net_eth_loopback_seq();
        eth_loopback_result_t r;
        net_eth_loopback(mbit, n);
        bool done = false;
        for (int i = 0; i < 150 && !(done = net_eth_loopback_result(seq, &r)); i++) sleep_ms(100);
        if (!done) { send_error(conn_id, "eth loopback: timed out"); return; }
        char resp[192];
        snprintf(resp, sizeof(resp),
                 "{\"mbit\":%d,\"sent\":%lu,\"received\":%lu,\"intact\":%lu,\"corrupt\":%lu,"
                 "\"crc\":%lu,\"align\":%lu,\"tx_fail\":%lu}",
                 r.mbit, (unsigned long)r.sent, (unsigned long)r.received, (unsigned long)r.intact,
                 (unsigned long)r.corrupt, (unsigned long)r.crc, (unsigned long)r.align,
                 (unsigned long)r.tx_fail);
        send_ok_str(conn_id, resp);
        return;
    }
    if (strcmp(action, "refclk") == 0) {
        uint32_t seq = net_eth_refclk_seq(), hz = 0;
        net_eth_refclk_measure();
        for (int i = 0; i < 40 && !net_eth_refclk_result(seq, &hz); i++)
            sleep_ms(100);   /* the net task runs the ~200 ms gate, then restarts the PHY */
        long ppm = hz ? (((long long)hz - 50000000LL) * 1000000LL / 50000000LL) : 0;
        char resp[96];
        snprintf(resp, sizeof(resp), "{\"hz\":%lu,\"ppm\":%ld,\"on_hsi\":%s}",
                 (unsigned long)hz, ppm, clock_on_hsi() ? "true" : "false");
        send_ok_str(conn_id, resp);
        return;
    }
    if (strcmp(action, "speed") == 0) {
        char nbuf[8] = {0}, dbuf[8] = {0};
        if (!json_get_value(buf, "mbit", nbuf, sizeof(nbuf)) || !nbuf[0]) {
            send_error(conn_id, "eth speed needs mbit (0 = autoneg, 10 or 100)"); return;
        }
        int mbit = atoi(nbuf);
        if (mbit != 0 && mbit != 10 && mbit != 100) {
            send_error(conn_id, "eth speed mbit must be 0 (autoneg), 10 or 100"); return;
        }
        json_get_value(buf, "duplex", dbuf, sizeof(dbuf));
        int full = (strcmp(dbuf, "full") == 0);
        net_eth_force_speed(mbit, full);
        char resp[64];
        snprintf(resp, sizeof(resp), "{\"speed\":%d,\"duplex\":\"%s\"}",
                 mbit, mbit == 0 ? "auto" : (full ? "full" : "half"));
        send_ok_str(conn_id, resp);
        return;
    }
    if      (strcmp(action, "stop")    == 0) { net_eth_stop();    }
    else if (strcmp(action, "start")   == 0) { net_eth_start();   }
    else if (strcmp(action, "restart") == 0 || action[0] == '\0') { net_eth_restart(); strcpy(action, "restart"); }
    else { send_error(conn_id, "eth action must be stop|start|restart|stats|speed|refclk|loopback"); return; }
    char resp[48];
    snprintf(resp, sizeof(resp), "\"%s\"", action);
    send_ok_str(conn_id, resp);
}

/* {"cmd":"speedtest","dir":"up"|"down","bytes":N}  — cloud throughput probe over
   a tunnel conn. See the speedtest state block / speedtest_pump above. */
void handle_speedtest(int conn_id, const char *buf) {
    char dir[8] = {0};
    json_get_value(buf, "dir", dir, sizeof(dir));
    char nbuf[16] = {0};
    long bytes = json_get_value(buf, "bytes", nbuf, sizeof(nbuf)) ? atol(nbuf) : 0;
    if (bytes <= 0)           bytes = 1048576;           /* default 1 MiB */
    if (bytes > 64*1024*1024) bytes = 64*1024*1024;      /* sanity clamp */

    if (strcmp(dir, "down") == 0) {
        /* Sink: the next `bytes` raw bytes on this conn are counted + discarded,
           then we ack. The server starts blasting right after this command line. */
        sprx.total    = (size_t)bytes;
        sprx.recv     = 0;
        sprx.last_ack = 0;
        conn_proto_set(conn_id, PROTO_SPEEDTEST);
        return;                                          /* server paces to our acks */
    }
    /* up (default): stream synthetic bytes out, pumped from command_handler_poll. */
    sptx.active = true;
    sptx.conn   = conn_id;
    sptx.total  = (size_t)bytes;
    sptx.sent   = 0;
}

void handle_wifi_status(int conn_id, const char *json) {
    (void)json;
    config_t cfg;
    bool have = (config_load(&cfg) == 0 && cfg.ssid[0] != '\0');
    /* esp-hosted transport counters since the link came up.  `batch` is the
       histogram of transactions-per-net_poll: index 0 = passes that clocked
       nothing, 1 = a single transaction (what the transport used to be capped
       at), >=2 = batched passes.  See esp_hosted_pump.h. */
    const esp_hosted_pump_stats_t *ps = esp_hosted_spi_stats();
    char batch[64];
    int n = 0;
    for (unsigned i = 0; i <= ESP_HOSTED_PUMP_MAX_BATCH; i++) {
        n += snprintf(batch + n, (size_t)((int)sizeof(batch) - n), "%s%lu",
                      i ? "," : "", (unsigned long)ps->batch_len[i]);
        if (n >= (int)sizeof(batch)) { n = (int)sizeof(batch) - 1; break; }
    }
    batch[n] = '\0';

    uint32_t c3_noresp = 0, c3_reboot = 0;   /* C3 restarts: no RSSI answers / booted while up */
    esp_wifi_ctrl_slave_lost_counts(&c3_noresp, &c3_reboot);
    /* The SSID is user data: escape it (a quote in it broke the whole reply). */
    char ssid_j[2 * sizeof(cfg.ssid) + 8];
    bp_emit_t ej;
    bp_emit_init(&ej, ssid_j, sizeof(ssid_j));
    bp_emit_jstr(&ej, have ? cfg.ssid : "");
    char resp[512];
    snprintf(resp, sizeof(resp),
             "{\"status\":\"ok\",\"data\":{\"state\":\"%s\",\"configured\":%s,"
             "\"connected\":%s,\"ssid\":%s,"
             "\"xacts\":%lu,\"pump_calls\":%lu,\"batch\":[%s],"
             "\"settle_hit\":%lu,\"settle_miss\":%lu,\"xact_err\":%lu,"
             "\"c3_lost_noresp\":%lu,\"c3_lost_reboot\":%lu,\"c3_nvs\":\"%s\"}}\n",
             esp_wifi_ctrl_state_str(),
             have ? "true" : "false",
             esp_wifi_ctrl_connected() ? "true" : "false",
             bp_emit_ok(&ej) ? ssid_j : "\"\"",
             (unsigned long)ps->xacts, (unsigned long)ps->calls, batch,
             (unsigned long)ps->settle_hit, (unsigned long)ps->settle_miss,
             (unsigned long)ps->xact_err, (unsigned long)c3_noresp, (unsigned long)c3_reboot,
             c3_nvs_str());
    if (at_send_data(conn_id, (const uint8_t *)resp, strlen(resp)) != 0) at_close_connection(conn_id);
}
