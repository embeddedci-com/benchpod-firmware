/*
 * esp_wifi_ctrl.c — Wi-Fi association over the esp-hosted RPC channel.
 *
 * Hand-rolled minimal protobuf (no protobuf-c dependency) driving the ESP32-C3
 * through the esp-hosted RPC envelope on ESP_SERIAL_IF: Init -> SetConfig(STA)
 * -> Start -> Connect -> GetMac, then waits for the sta-connected event and
 * brings the Wi-Fi netif link up. Credentials come from config_store.
 *
 * ⚠ NOT hardware-verified. The RPC ids/field numbers are from esp-hosted-mcu's
 *   esp_hosted_rpc.proto (commit 8f0770d). The wifi_init_config defaults below
 *   mirror IDF v5.x WIFI_INIT_CONFIG_DEFAULT and MUST match the slave's IDF
 *   (esp_wifi_init validates `magic`); the exact connect-step ordering is a
 *   bench bring-up item.
 */
#include "esp_wifi_ctrl.h"
#include "esp_hosted_spi.h"
#include "esp_hosted_frame.h"
#include "esp_netif.h"
#include "config_store.h"
#include "esp_rom_flash.h"
#include "hw_worker.h"

#include "pico/time.h"
#include <string.h>
#include <stdio.h>

/* ---- esp-hosted RPC ids (esp_hosted_rpc.proto) ---- */
enum { RPC_REQ = 1, RPC_RESP = 2, RPC_EVENT = 3 };
enum {
    ID_REQ_GET_MAC      = 257, ID_RESP_GET_MAC      = 513,
    ID_REQ_SET_MODE     = 260, ID_RESP_SET_MODE     = 516,
    ID_REQ_GET_RSSI     = 341, ID_RESP_GET_RSSI     = 597,
    ID_REQ_WIFI_INIT    = 278, ID_RESP_WIFI_INIT    = 534,
    ID_REQ_WIFI_START   = 280, ID_RESP_WIFI_START   = 536,
    ID_REQ_WIFI_CONNECT = 282, ID_RESP_WIFI_CONNECT = 538,
    ID_REQ_WIFI_SETCFG  = 284, ID_RESP_WIFI_SETCFG  = 540,
    ID_EVENT_STA_CONNECTED    = 775,
    ID_EVENT_STA_DISCONNECTED = 776,
};
#define WIFI_IF_STA            0
#define WIFI_MODE_STA          1     /* esp_wifi_types wifi_mode_t WIFI_MODE_STA */
#define WIFI_INIT_CONFIG_MAGIC 0x1F2F3F4F   /* IDF v5.x — verify vs slave IDF */

#define STEP_TIMEOUT_MS  4000
#define RETRY_BACKOFF_MS 5000
/* The esp-hosted slave sends its boot event ~1-2 s after reset.  No event this long after
   release means the C3 is blank or runs something else (a new pod ships with it empty), so
   Wi-Fi flashes the embedded image once and tries again. */
#define SLAVE_BOOT_TIMEOUT_MS 20000

/* ---- tiny protobuf writer ------------------------------------------------- */
typedef struct { uint8_t *buf; size_t cap, len; bool err; } pb_w;
static void pb_raw(pb_w *w, uint8_t b) { if (w->len < w->cap) w->buf[w->len++] = b; else w->err = true; }
static void pb_varint(pb_w *w, uint64_t v) { do { uint8_t b = v & 0x7F; v >>= 7; if (v) b |= 0x80; pb_raw(w, b); } while (v); }
static void pb_tag(pb_w *w, uint32_t field, uint32_t wire) { pb_varint(w, ((uint64_t)field << 3) | wire); }
static void pb_int(pb_w *w, uint32_t field, uint64_t val) { pb_tag(w, field, 0); pb_varint(w, val); }
static void pb_bytes(pb_w *w, uint32_t field, const uint8_t *d, size_t n) {
    pb_tag(w, field, 2); pb_varint(w, n);
    for (size_t i = 0; i < n; i++) pb_raw(w, d[i]);
}

/* ---- tiny protobuf reader ------------------------------------------------- */
typedef struct { const uint8_t *p, *end; } pb_r;
static bool pb_rd_varint(pb_r *r, uint64_t *out) {
    uint64_t v = 0; int s = 0;
    while (r->p < r->end) {
        uint8_t b = *r->p++;
        v |= (uint64_t)(b & 0x7F) << s;
        if (!(b & 0x80)) { *out = v; return true; }
        s += 7; if (s > 63) return false;
    }
    return false;
}
/* Iterate one field. For varint, ival is set; for len-delim, bytes + blen. */
static bool pb_next(pb_r *r, uint32_t *field, uint32_t *wire,
                    uint64_t *ival, const uint8_t **bytes, size_t *blen) {
    if (r->p >= r->end) return false;
    uint64_t key; if (!pb_rd_varint(r, &key)) return false;
    *field = (uint32_t)(key >> 3); *wire = (uint32_t)(key & 7);
    if (*wire == 0) { return pb_rd_varint(r, ival); }
    if (*wire == 2) {
        uint64_t n; if (!pb_rd_varint(r, &n)) return false;
        if ((size_t)(r->end - r->p) < n) return false;
        *bytes = r->p; *blen = (size_t)n; r->p += n; return true;
    }
    if (*wire == 5) { if (r->end - r->p < 4) return false; r->p += 4; return true; }
    if (*wire == 1) { if (r->end - r->p < 8) return false; r->p += 8; return true; }
    return false;
}

/* ---- RPC request builders ------------------------------------------------- */

/* Wrap a oneof payload in the Rpc envelope. The oneof field number equals the
   msg_id; an empty (len-0) payload still marks the oneof as set. */
static size_t build_rpc(uint8_t *out, size_t cap, uint32_t msg_id, uint32_t uid,
                        const uint8_t *payload, size_t plen) {
    pb_w w = { out, cap, 0, false };
    pb_int(&w, 1, RPC_REQ);
    pb_int(&w, 2, msg_id);
    pb_int(&w, 3, uid);
    pb_bytes(&w, msg_id, payload, plen);
    return w.err ? 0 : w.len;
}

/* Rpc_Req_SetMode { int32 mode = 1 } — MUST precede SetConfig, else the slave
   stays WIFI_MODE_NULL, SetConfig(STA) is dropped, and Connect fails with
   ESP_ERR_WIFI_SSID (no association attempt, no event). */
static size_t build_set_mode(uint8_t *out, size_t cap, uint32_t mode) {
    pb_w w = { out, cap, 0, false };
    pb_int(&w, 1, mode);
    return w.err ? 0 : w.len;
}

/* wifi_init_config with the non-zero IDF defaults (zero-valued fields are
   omitted; proto3 defaults them to 0 on the slave). */
static size_t build_init_cfg(uint8_t *out, size_t cap) {
    uint8_t inner[96];
    pb_w c = { inner, sizeof(inner), 0, false };
    pb_int(&c, 1, 10);                 /* static_rx_buf_num   */
    pb_int(&c, 2, 32);                 /* dynamic_rx_buf_num  */
    pb_int(&c, 3, 1);                  /* tx_buf_type=dynamic */
    pb_int(&c, 5, 32);                 /* dynamic_tx_buf_num  */
    pb_int(&c, 6, 32);                 /* cache_tx_buf_num    */
    pb_int(&c, 8, 1);                  /* ampdu_rx_enable     */
    pb_int(&c, 9, 1);                  /* ampdu_tx_enable     */
    pb_int(&c, 11, 1);                 /* nvs_enable          */
    pb_int(&c, 13, 6);                 /* rx_ba_win           */
    pb_int(&c, 15, 752);               /* beacon_max_len      */
    pb_int(&c, 16, 32);                /* mgmt_sbuf_num       */
    pb_int(&c, 17, 1);                 /* feature_caps        */
    pb_int(&c, 19, 7);                 /* espnow_max_encrypt_num */
    pb_int(&c, 20, WIFI_INIT_CONFIG_MAGIC); /* magic (last)   */
    if (c.err) return 0;
    /* Rpc_Req_WifiInit { wifi_init_config cfg = 1 } */
    pb_w w = { out, cap, 0, false };
    pb_bytes(&w, 1, inner, c.len);
    return w.err ? 0 : w.len;
}

/* Rpc_Req_WifiSetConfig { iface=1=STA; wifi_config{ wifi_sta_config{ssid,password} sta=2 } cfg=2 } */
static size_t build_set_config(uint8_t *out, size_t cap,
                               const char *ssid, const char *pass) {
    uint8_t sta[160];
    pb_w s = { sta, sizeof(sta), 0, false };
    pb_bytes(&s, 1, (const uint8_t *)ssid, strlen(ssid));
    pb_bytes(&s, 2, (const uint8_t *)pass, strlen(pass));
    if (s.err) return 0;

    uint8_t cfg[176];
    pb_w c = { cfg, sizeof(cfg), 0, false };
    pb_bytes(&c, 2, sta, s.len);       /* wifi_config.sta = 2 */
    if (c.err) return 0;

    pb_w w = { out, cap, 0, false };
    pb_int(&w, 1, WIFI_IF_STA);        /* iface */
    pb_bytes(&w, 2, cfg, c.len);       /* cfg   */
    return w.err ? 0 : w.len;
}

/* ---- state machine -------------------------------------------------------- */
typedef enum {
    WC_DOWN = 0,   /* unconfigured / ESP held in reset */
    WC_WAIT_READY, /* started, awaiting slave boot event */
    WC_FLASHING,   /* no boot event: the worker is writing the embedded image to the C3 */
    WC_INIT, WC_SETMODE, WC_SETCFG, WC_START, WC_CONNECT, WC_GETMAC,
    WC_WAIT_ASSOC, /* connect issued, awaiting sta-connected event */
    WC_UP,
    WC_UP_RSSI,    /* connected; a periodic GetRssi is outstanding */
    WC_BACKOFF,
} wc_state_t;

static config_t       s_cfg;
static bool           s_configured;
static bool           s_started;
static wc_state_t     s_state = WC_DOWN;
static uint32_t       s_uid;
static uint32_t       s_await_resp;      /* msg_id of the Resp we're waiting for, 0 = none */
static volatile bool  s_resp_ready;
static volatile int32_t s_resp_status;
static volatile bool  s_evt_connected, s_evt_disconnected;
static volatile bool  s_have_mac;
static uint8_t        s_mac[6];
static absolute_time_t s_deadline;
static volatile int32_t s_rssi;        /* last STA RSSI in dBm (valid iff s_have_rssi) */
static volatile bool  s_have_rssi;
static absolute_time_t s_rssi_deadline; /* when to issue the next GetRssi while WC_UP */
static absolute_time_t s_boot_deadline; /* WC_WAIT_READY gives up on the boot event here */
static bool           s_flash_tried;    /* one automatic C3 flash per configuration */
static volatile bool  s_flash_done, s_flash_ok;   /* set by the worker when it finishes */
static volatile bool  s_paused;         /* a console command owns the C3 straps */

const char *esp_wifi_ctrl_state_str(void) {
    switch (s_state) {
        case WC_DOWN:        return s_configured ? "starting" : "disabled";
        case WC_WAIT_READY:  return "waiting-slave";
        case WC_FLASHING:    return "flashing-c3";
        case WC_INIT: case WC_SETMODE: case WC_SETCFG: case WC_START:
        case WC_CONNECT: case WC_GETMAC: case WC_WAIT_ASSOC: return "connecting";
        case WC_UP: case WC_UP_RSSI: return "connected";
        case WC_BACKOFF:     return "backoff";
        default:             return "unknown";
    }
}

void esp_wifi_ctrl_pause(bool paused) { s_paused = paused; }

void esp_wifi_ctrl_flash_done(bool ok) {
    s_flash_ok   = ok;
    s_flash_done = true;
}

bool esp_wifi_ctrl_configured(void) { return s_configured; }
bool esp_wifi_ctrl_connected(void) { return s_state == WC_UP || s_state == WC_UP_RSSI; }

/* Last measured STA RSSI (dBm). Returns true + fills *dbm iff a reading is known. */
bool esp_wifi_ctrl_rssi(int *dbm) {
    if (!s_have_rssi) return false;
    if (dbm) *dbm = (int)s_rssi;
    return true;
}

/* ---- esp-hosted serial-interface TLV ("protocomm pserial") -----------------
 * RPC protobuf on ESP_SERIAL_IF is wrapped as:
 *   [0x01][ep_len:2 LE][ep_name][0x02][data_len:2 LE][protobuf]
 * with ep_name = "RPCRsp" (RPC_EP_NAME_RSP); events come back on "RPCEvt". The
 * slave routes by ep_name and silently drops a bare protobuf — this wrapper was
 * missing and is what caused "init timeout". */
#define PSER_TLV_EPNAME  0x01
#define PSER_TLV_DATA    0x02
static const char RPC_EP_NAME[] = "RPCRsp";

static size_t tlv_wrap(uint8_t *out, size_t cap, const uint8_t *data, size_t dlen) {
    size_t eplen = sizeof(RPC_EP_NAME) - 1;            /* 6, excludes NUL */
    if (3 + eplen + 3 + dlen > cap) return 0;
    size_t c = 0;
    out[c++] = PSER_TLV_EPNAME;
    out[c++] = (uint8_t)eplen; out[c++] = (uint8_t)(eplen >> 8);
    memcpy(out + c, RPC_EP_NAME, eplen); c += eplen;
    out[c++] = PSER_TLV_DATA;
    out[c++] = (uint8_t)dlen; out[c++] = (uint8_t)(dlen >> 8);
    memcpy(out + c, data, dlen); c += dlen;
    return c;
}

/* Strip the pserial TLV to the inner protobuf; passes through if not wrapped. */
static void tlv_unwrap(const uint8_t *in, size_t inlen, const uint8_t **pb, size_t *pblen) {
    *pb = in; *pblen = inlen;
    if (inlen >= 3 && in[0] == PSER_TLV_EPNAME) {
        size_t eplen = (size_t)in[1] | ((size_t)in[2] << 8);
        size_t off = 3 + eplen;
        if (off + 3 <= inlen && in[off] == PSER_TLV_DATA) {
            size_t dlen = (size_t)in[off + 1] | ((size_t)in[off + 2] << 8);
            if (off + 3 + dlen <= inlen) { *pb = in + off + 3; *pblen = dlen; }
        }
    }
}

/* Serial (RPC) frames from the slave: parse the envelope, match resp/events. */
static void on_serial_frame(const esp_hosted_rx_t *rx) {
    const uint8_t *pbuf; size_t pblen;
    tlv_unwrap(rx->payload, rx->payload_len, &pbuf, &pblen);
    pb_r r = { pbuf, pbuf + pblen };
    uint32_t field, wire; uint64_t iv; const uint8_t *b = NULL; size_t bl = 0;
    uint32_t msg_type = 0, msg_id = 0;
    const uint8_t *payload = NULL; size_t payload_len = 0;
    while (pb_next(&r, &field, &wire, &iv, &b, &bl)) {
        if (field == 1 && wire == 0)      msg_type = (uint32_t)iv;
        else if (field == 2 && wire == 0) msg_id = (uint32_t)iv;
        else if (wire == 2 && field == msg_id) { payload = b; payload_len = bl; }
    }

    if (msg_type == RPC_EVENT) {
        if (msg_id == ID_EVENT_STA_CONNECTED) {
            s_evt_connected = true;
        } else if (msg_id == ID_EVENT_STA_DISCONNECTED) {
            s_evt_disconnected = true;
            /* Pull the 802.11 reason (f4) and the RSSI-at-disconnect (f5) out of
               Rpc_Event_StaDisconnected.sta_disconnected(f2) — both are useful for
               diagnosing a marginal/distant link (e.g. reason 200 = beacon timeout). */
            int reason = -1; int drssi = 0; bool have_drssi = false;
            if (payload) {
                pb_r er = { payload, payload + payload_len };
                uint32_t ef, ew; uint64_t ei; const uint8_t *eb; size_t ebl;
                while (pb_next(&er, &ef, &ew, &ei, &eb, &ebl))
                    if (ef == 2 && ew == 2) {
                        pb_r dr = { eb, eb + ebl };
                        uint32_t df, dw; uint64_t di; const uint8_t *db; size_t dbl;
                        while (pb_next(&dr, &df, &dw, &di, &db, &dbl)) {
                            if (df == 4 && dw == 0) reason = (int)di;
                            else if (df == 5 && dw == 0) { drssi = (int)(int32_t)di; have_drssi = true; }
                        }
                    }
            }
            if (have_drssi) {
                s_rssi = drssi; s_have_rssi = true;   /* freshest signal reading */
                printf("[wifi] disconnected (reason %d, rssi %d dBm)\n", reason, drssi);
            } else {
                printf("[wifi] disconnected (reason %d)\n", reason);
            }
        }
        return;
    }
    if (msg_type != RPC_RESP) return;

    /* For GetMac, pull the 6-byte MAC out of the response submessage. */
    if (msg_id == ID_RESP_GET_MAC && payload) {
        pb_r pr = { payload, payload + payload_len };
        while (pb_next(&pr, &field, &wire, &iv, &b, &bl)) {
            if (field == 1 && wire == 2 && bl == 6) { memcpy(s_mac, b, 6); s_have_mac = true; }
            else if (field == 2 && wire == 0) s_resp_status = (int32_t)iv;
        }
    } else if (msg_id == ID_RESP_GET_RSSI && payload) {
        /* Rpc_Resp_WifiStaGetRssi { resp = 1; rssi = 2 } — rssi is a signed int32. */
        pb_r pr = { payload, payload + payload_len };
        while (pb_next(&pr, &field, &wire, &iv, &b, &bl)) {
            if (field == 1 && wire == 0) s_resp_status = (int32_t)iv;
            else if (field == 2 && wire == 0) { s_rssi = (int32_t)iv; s_have_rssi = true; }
        }
    } else if (payload) {
        pb_r pr = { payload, payload + payload_len };
        while (pb_next(&pr, &field, &wire, &iv, &b, &bl))
            if (field == 1 && wire == 0) s_resp_status = (int32_t)iv;   /* resp code */
    }

    if (msg_id == s_await_resp) s_resp_ready = true;
}

/* Send one request and arm the wait for its response. */
static bool send_req(uint32_t req_id, uint32_t resp_id,
                     const uint8_t *payload, size_t plen) {
    uint8_t frame[256];
    size_t n = build_rpc(frame, sizeof(frame), req_id, ++s_uid, payload, plen);
    if (n == 0) return false;
    uint8_t tlv[280];
    size_t tn = tlv_wrap(tlv, sizeof(tlv), frame, n);   /* wrap for the pserial endpoint */
    if (tn == 0) return false;
    s_resp_ready = false;
    s_await_resp = resp_id;
    s_deadline   = make_timeout_time_ms(STEP_TIMEOUT_MS);
    return esp_hosted_spi_send(ESP_SERIAL_IF, 0, tlv, (uint16_t)tn) == 0;
}

static void to_backoff(const char *why) {
    printf("[wifi] %s — backoff\n", why);
    s_await_resp = 0;
    s_state = WC_BACKOFF;
    s_deadline = make_timeout_time_ms(RETRY_BACKOFF_MS);
}

/* Shared by WC_UP / WC_UP_RSSI: if the slave reported a disconnect, drop the
   Wi-Fi link and re-issue connect (the slave stays inited). Returns true if a
   drop was handled (caller should return immediately). */
static bool esp_wifi_ctrl_reconnect_if_dropped(void) {
    if (!s_evt_disconnected) return false;
    s_evt_disconnected = false;
    esp_netif_set_link_up(false);
    printf("[wifi] disconnected — reconnecting\n");
    if (send_req(ID_REQ_WIFI_CONNECT, ID_RESP_WIFI_CONNECT, NULL, 0)) {
        s_deadline = make_timeout_time_ms(15000);
        s_state = WC_WAIT_ASSOC;
    } else {
        to_backoff("reconnect send failed");
    }
    return true;
}

void esp_wifi_ctrl_init(void) {
    s_configured = (config_load(&s_cfg) == 0 && s_cfg.ssid[0] != '\0');
    esp_hosted_spi_set_serial_cb(on_serial_frame);
    s_state = WC_DOWN;
    if (s_configured) printf("[wifi] configured for SSID \"%s\"\n", s_cfg.ssid);
    else              printf("[wifi] no SSID configured — Wi-Fi disabled\n");
}

void esp_wifi_ctrl_reload(void) {
    if (s_started) { esp_hosted_spi_stop(); s_started = false; }
    s_flash_tried = false;   /* new credentials (or a manual flash) earn another automatic try */
    esp_netif_set_link_up(false);
    s_state = WC_DOWN;
    esp_wifi_ctrl_init();
}

void esp_wifi_ctrl_poll(void) {
    /* A backoff that expires mid-flash would release the C3's reset and knock it out of its
       ROM loader, so leave the straps alone while a console command holds them. */
    if (s_paused) return;
    /* Gate the whole co-processor on configuration. */
    if (!s_configured) {
        if (s_started) { esp_hosted_spi_stop(); s_started = false; s_state = WC_DOWN; }
        return;
    }
    if (s_state == WC_FLASHING) {           /* the C3 is the worker's until it reports back */
        if (!s_flash_done) return;
        s_flash_done = false;
        if (s_flash_ok) {
            printf("[wifi] ESP32-C3 flashed — starting it\n");
            s_state = WC_DOWN;              /* s_started is false: the next poll releases reset */
        } else {
            to_backoff("ESP32-C3 flash failed");
        }
        return;
    }
    if (!s_started) {
        esp_hosted_spi_start(); s_started = true; s_state = WC_WAIT_READY;
        s_boot_deadline = make_timeout_time_ms(SLAVE_BOOT_TIMEOUT_MS);
    }

    switch (s_state) {
    case WC_DOWN:
        s_state = WC_WAIT_READY;
        s_boot_deadline = make_timeout_time_ms(SLAVE_BOOT_TIMEOUT_MS);
        return;

    case WC_FLASHING:                       /* handled above */
        return;

    case WC_WAIT_READY:
        if (!esp_hosted_spi_ready() && time_reached(s_boot_deadline)) {
            if (!s_flash_tried && esp_slave_fw_len > 0) {
                /* Hold the C3 in reset and hand it to the worker: the ROM-loader flash
                   blocks for ~140 s, which the net task (lwIP, cloud) cannot afford. */
                esp_hosted_spi_stop(); s_started = false;
                s_flash_tried = true;
                s_flash_done  = false;
                if (hw_worker_submit_esp_flash()) {
                    printf("[wifi] no boot event from the ESP32-C3 in %u s — flashing its "
                           "esp-hosted image (~2-3 min)\n", SLAVE_BOOT_TIMEOUT_MS / 1000u);
                    s_state = WC_FLASHING;
                } else {
                    to_backoff("could not queue the ESP32-C3 flash");
                }
            } else {
                to_backoff("no boot event from the ESP32-C3 (already flashed once; "
                           "run flash-esp32 or check the board)");
            }
            return;
        }
        if (esp_hosted_spi_ready()) {
            s_evt_connected = s_evt_disconnected = false;
            uint8_t p[128]; size_t n = build_init_cfg(p, sizeof(p));
            if (n && send_req(ID_REQ_WIFI_INIT, ID_RESP_WIFI_INIT, p, n)) s_state = WC_INIT;
            else to_backoff("init build/send failed");
        }
        return;

    case WC_INIT:
        if (s_resp_ready) {
            uint8_t p[16];
            size_t n = build_set_mode(p, sizeof(p), WIFI_MODE_STA);
            if (n && send_req(ID_REQ_SET_MODE, ID_RESP_SET_MODE, p, n)) s_state = WC_SETMODE;
            else to_backoff("setmode build/send failed");
        } else if (time_reached(s_deadline)) to_backoff("init timeout");
        return;

    case WC_SETMODE:
        if (s_resp_ready) {
            uint8_t p[192];
            size_t n = build_set_config(p, sizeof(p), s_cfg.ssid, s_cfg.password);
            if (n && send_req(ID_REQ_WIFI_SETCFG, ID_RESP_WIFI_SETCFG, p, n)) s_state = WC_SETCFG;
            else to_backoff("setcfg build/send failed");
        } else if (time_reached(s_deadline)) to_backoff("setmode timeout");
        return;

    case WC_SETCFG:
        if (s_resp_ready) {
            if (send_req(ID_REQ_WIFI_START, ID_RESP_WIFI_START, NULL, 0)) s_state = WC_START;
            else to_backoff("start send failed");
        } else if (time_reached(s_deadline)) to_backoff("setcfg timeout");
        return;

    case WC_START:
        if (s_resp_ready) {
            if (send_req(ID_REQ_WIFI_CONNECT, ID_RESP_WIFI_CONNECT, NULL, 0)) s_state = WC_CONNECT;
            else to_backoff("connect send failed");
        } else if (time_reached(s_deadline)) to_backoff("start timeout");
        return;

    case WC_CONNECT:
        if (s_resp_ready) {
            if (send_req(ID_REQ_GET_MAC, ID_RESP_GET_MAC, NULL, 0)) s_state = WC_GETMAC;
            else to_backoff("getmac send failed");
        } else if (time_reached(s_deadline)) to_backoff("connect timeout");
        return;

    case WC_GETMAC:
        if (s_resp_ready) {
            if (s_have_mac) {
                esp_netif_set_hwaddr(s_mac);
                printf("[wifi] sta MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
                       s_mac[0], s_mac[1], s_mac[2], s_mac[3], s_mac[4], s_mac[5]);
            } else {
                printf("[wifi] GetMac: no MAC in resp — netif keeps provisional MAC (DHCP will fail)\n");
            }
            s_await_resp = 0;
            s_deadline = make_timeout_time_ms(15000);   /* assoc can take a few s */
            s_state = WC_WAIT_ASSOC;
        } else if (time_reached(s_deadline)) to_backoff("getmac timeout");
        return;

    case WC_WAIT_ASSOC:
        if (s_evt_connected) {
            s_evt_connected = false;
            esp_netif_set_link_up(true);     /* lwIP restarts DHCP on link-up */
            printf("[wifi] associated — link up\n");
            s_rssi_deadline = make_timeout_time_ms(1000);   /* first RSSI read soon */
            s_state = WC_UP;
        } else if (s_evt_disconnected || time_reached(s_deadline)) {
            to_backoff("association failed");
        }
        return;

    case WC_UP:
        if (esp_wifi_ctrl_reconnect_if_dropped()) return;
        /* Periodically poll the STA RSSI so `wifi-show` / status can report link
           quality (useful when the AP is far and DHCP won't complete). */
        if (time_reached(s_rssi_deadline)) {
            if (send_req(ID_REQ_GET_RSSI, ID_RESP_GET_RSSI, NULL, 0)) s_state = WC_UP_RSSI;
            else s_rssi_deadline = make_timeout_time_ms(5000);  /* TX busy — retry later */
        }
        return;

    case WC_UP_RSSI:
        if (esp_wifi_ctrl_reconnect_if_dropped()) return;
        if (s_resp_ready || time_reached(s_deadline)) {   /* got RSSI (or it timed out) */
            s_await_resp = 0;
            s_rssi_deadline = make_timeout_time_ms(5000);
            s_state = WC_UP;
        }
        return;

    case WC_BACKOFF:
        if (time_reached(s_deadline)) {
            /* Full restart of the co-processor link. */
            esp_hosted_spi_stop(); s_started = false;
            esp_netif_set_link_up(false);
            s_state = WC_DOWN;
        }
        return;
    }
}
