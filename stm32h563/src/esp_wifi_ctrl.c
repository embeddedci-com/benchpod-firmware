/*
 * esp_wifi_ctrl.c — Wi-Fi association over the esp-hosted RPC channel.
 *
 * Hand-rolled minimal protobuf (no protobuf-c dependency) driving the ESP32-C3
 * through the esp-hosted RPC envelope on ESP_SERIAL_IF: Init -> SetStorage(RAM)
 * -> SetMode(STA) -> SetConfig(STA) -> Start -> Connect -> GetMac, then waits for
 * the sta-connected event and brings the Wi-Fi netif link up. Credentials come
 * from config_store, which is the only place they are stored: the C3 is told to
 * keep its Wi-Fi configuration in RAM before it is given any (see WC_SETSTORAGE).
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
#include "blob_store.h"
#include "hw_worker.h"
#include "wifi_scan.h"

#include "pico/time.h"
#include "bp_log.h"
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
    ID_REQ_WIFI_SETSTORAGE = 313, ID_RESP_WIFI_SETSTORAGE = 569,
    ID_REQ_SCAN_START   = 286, ID_RESP_SCAN_START   = 542,
    ID_REQ_SCAN_AP_NUM  = 288, ID_RESP_SCAN_AP_NUM  = 544,
    ID_REQ_SCAN_CLEAR   = 290, ID_RESP_SCAN_CLEAR   = 546,
    ID_REQ_SCAN_AP_REC  = 351, ID_RESP_SCAN_AP_REC  = 607,
    ID_EVENT_STA_CONNECTED    = 775,
    ID_EVENT_STA_DISCONNECTED = 776,
};
#define WIFI_IF_STA            0
#define WIFI_MODE_STA          1     /* esp_wifi_types wifi_mode_t WIFI_MODE_STA */
#define WIFI_STORAGE_RAM       1     /* wifi_storage_t; 0 = WIFI_STORAGE_FLASH, the IDF default */
#define WIFI_INIT_CONFIG_MAGIC 0x1F2F3F4F   /* IDF v5.x — verify vs slave IDF */

#define STEP_TIMEOUT_MS  4000
#define RETRY_BACKOFF_MS 5000
/* The esp-hosted slave sends its boot event ~1-2 s after reset.  No event this long after
   release means the C3 is blank or runs something else (a new pod ships with it empty), so
   Wi-Fi flashes the embedded image once and tries again. */
#define SLAVE_BOOT_TIMEOUT_MS 20000
/* While connected, GetRssi doubles as a liveness probe of the C3: this many requests in a
   row without a response (timed out, or not even queued because the TX ring never drains,
   which is what a stuck HANDSHAKE looks like) means the C3 is gone, and the link restarts. */
#define RSSI_MAX_MISSES       3
#define RSSI_PERIOD_MS        5000
/* Wi-Fi scan (esp_wifi_ctrl_scan_request).  The C3 runs it as a blocking esp_wifi_scan_start:
   its answer comes when the scan is over, about 2 s for the 2.4 GHz channels (longer while
   joined: it returns to the home channel between channels). */
#define SCAN_TIMEOUT_MS       12000
/* A request that finds the link mid-connect waits for it to finish (joined: scan alongside;
   failed: scan on its own) this long, then fails. */
#define SCAN_WAIT_MS          30000
/* Access points read back one at a time (GetApRecord: one record per frame, since this SPI host
   does not reassemble fragmented frames); the rest are dropped with ClearApList. */
#define SCAN_READ_MAX         40

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

/* Rpc_Req_WifiSetStorage { int32 storage = 1 }.  The slave never selects a storage itself, so
   it runs with the IDF default, WIFI_STORAGE_FLASH: esp_wifi_set_mode/set_config would write
   the mode and the SSID + password to the C3's own NVS partition, where wifi-clear (which
   clears config_store) does not reach.  RAM storage keeps them out of the C3's flash. */
static size_t build_set_storage(uint8_t *out, size_t cap, uint32_t storage) {
    pb_w w = { out, cap, 0, false };
    pb_int(&w, 1, storage);
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
    WC_INIT,
    WC_SETSTORAGE, /* RAM storage requested; nothing persistent is sent until the C3 confirms */
    WC_SETMODE, WC_SETCFG, WC_START, WC_CONNECT, WC_GETMAC,
    WC_WAIT_ASSOC, /* connect issued, awaiting sta-connected event */
    WC_UP,
    WC_UP_RSSI,    /* connected; a periodic GetRssi is outstanding */
    WC_BACKOFF,
    /* A Wi-Fi scan (esp_wifi_ctrl_scan_request): joined, from WC_UP; else on a C3 started for
       the scan alone (s_scan_only), which is held in reset again when it ends. */
    WC_SCAN_START, /* ScanStart(block) sent: answered when the scan is over */
    WC_SCAN_NUM,   /* GetApNum */
    WC_SCAN_REC,   /* GetApRecord, one access point per request */
    WC_SCAN_CLEAR, /* ClearApList: frees what was not read */
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
static uint8_t        s_rssi_misses;    /* consecutive unanswered GetRssi while up */
static volatile uint32_t s_lost_rssi;   /* link restarts: C3 stopped answering */
static volatile uint32_t s_lost_reboot; /* link restarts: C3 announced a new boot */

/* Wi-Fi scan.  The worker task asks (s_scan_req_id), the net task runs it and publishes the
   result (s_scan_list, s_scan_err, then s_scan_done_id). */
static volatile uint32_t s_scan_req_id;   /* the newest request; 0 = none since boot */
static volatile uint32_t s_scan_done_id;  /* the newest request answered */
static uint32_t       s_scan_run_id;      /* the request being run, 0 = none */
static const char *volatile s_scan_err;   /* NULL = s_scan_list is the answer */
static wifi_scan_list_t s_scan_list;
static bool           s_scan_only;        /* the C3 runs for this scan only */
static absolute_time_t s_scan_wait;       /* a request waiting for a connect gives up here */
static bool           s_scan_waiting;
static int32_t        s_scan_num;         /* GetApNum's answer */
static int            s_scan_read;        /* GetApRecord requests sent */
static volatile bool  s_scan_rec_ok;      /* the last GetApRecord answer held a record */
static wifi_scan_ap_t s_scan_rec;

static bool s_module = true;   /* the C3 is fitted (esp_wifi_ctrl_set_module_present) */

void esp_wifi_ctrl_set_module_present(bool present) { s_module = present; }
bool esp_wifi_ctrl_module_present(void) { return s_module; }

const char *esp_wifi_ctrl_state_str(void) {
    if (!s_module) return "absent";
    if (s_scan_only) return "scanning";     /* the C3 runs for a scan, not for a link */
    switch (s_state) {
        case WC_DOWN:        return s_configured ? "starting" : "disabled";
        case WC_WAIT_READY:  return "waiting-slave";
        case WC_FLASHING:    return "flashing-c3";
        case WC_INIT: case WC_SETSTORAGE: case WC_SETMODE: case WC_SETCFG: case WC_START:
        case WC_CONNECT: case WC_GETMAC: case WC_WAIT_ASSOC: return "connecting";
        case WC_UP: case WC_UP_RSSI: return "connected";
        case WC_BACKOFF:     return "backoff";
        case WC_SCAN_START: case WC_SCAN_NUM: case WC_SCAN_REC: case WC_SCAN_CLEAR:
            return "connected";             /* a scan alongside the link */
        default:             return "unknown";
    }
}

void esp_wifi_ctrl_pause(bool paused) { s_paused = paused; }

void esp_wifi_ctrl_flash_done(bool ok) {
    s_flash_ok   = ok;
    s_flash_done = true;
}

void esp_wifi_ctrl_slave_lost_counts(uint32_t *no_response, uint32_t *rebooted) {
    if (no_response) *no_response = s_lost_rssi;
    if (rebooted)    *rebooted    = s_lost_reboot;
}

bool esp_wifi_ctrl_configured(void) { return s_configured; }
bool esp_wifi_ctrl_connected(void) {
    return s_state == WC_UP || s_state == WC_UP_RSSI ||
           (!s_scan_only && (s_state == WC_SCAN_START || s_state == WC_SCAN_NUM ||
                             s_state == WC_SCAN_REC || s_state == WC_SCAN_CLEAR));
}

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
                log_printf("[wifi] disconnected (reason %d, rssi %d dBm)\n", reason, drssi);
            } else {
                log_printf("[wifi] disconnected (reason %d)\n", reason);
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
    } else if (msg_id == ID_RESP_SCAN_AP_NUM && payload) {
        /* Rpc_Resp_WifiScanGetApNum { resp = 1; number = 2 } */
        pb_r pr = { payload, payload + payload_len };
        while (pb_next(&pr, &field, &wire, &iv, &b, &bl)) {
            if (field == 1 && wire == 0) s_resp_status = (int32_t)iv;
            else if (field == 2 && wire == 0) s_scan_num = (int32_t)iv;
        }
    } else if (msg_id == ID_RESP_SCAN_AP_REC && payload) {
        /* Rpc_Resp_WifiScanGetApRecord { resp = 1; wifi_ap_record ap_record = 2 { ssid = 2;
           primary = 3; rssi = 5 (int32); authmode = 6 } } */
        pb_r pr = { payload, payload + payload_len };
        while (pb_next(&pr, &field, &wire, &iv, &b, &bl)) {
            if (field == 1 && wire == 0) s_resp_status = (int32_t)iv;
            else if (field == 2 && wire == 2) {
                pb_r ar = { b, b + bl };
                uint32_t af, aw; uint64_t ai; const uint8_t *ab = NULL; size_t abl = 0;
                memset(&s_scan_rec, 0, sizeof(s_scan_rec));
                while (pb_next(&ar, &af, &aw, &ai, &ab, &abl)) {
                    if (af == 2 && aw == 2) {
                        size_t n = abl < sizeof(s_scan_rec.ssid) - 1 ? abl : sizeof(s_scan_rec.ssid) - 1;
                        memcpy(s_scan_rec.ssid, ab, n);
                    } else if (af == 3 && aw == 0) s_scan_rec.channel = (uint8_t)ai;
                    else if (af == 5 && aw == 0) s_scan_rec.rssi = (int8_t)(int32_t)ai;
                    else if (af == 6 && aw == 0) s_scan_rec.authmode = (uint8_t)ai;
                }
                s_scan_rec_ok = true;
            }
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
    s_resp_status = 0;      /* proto3 omits a zero resp code: a stale one must not survive */
    s_await_resp = resp_id;
    s_deadline   = make_timeout_time_ms(STEP_TIMEOUT_MS);
    return esp_hosted_spi_send(ESP_SERIAL_IF, 0, tlv, (uint16_t)tn) == 0;
}

/* ---- Wi-Fi scan -------------------------------------------------------------- */

/* Answer the running request (err NULL: s_scan_list is the answer).  State is the caller's. */
static void scan_publish(const char *err) {
    s_scan_err = err;
    if (err) log_printf("[wifi] scan failed: %s\n", err);
    else     log_printf("[wifi] scan: %u networks (%u access points)\n",
                        (unsigned)s_scan_list.n, (unsigned)s_scan_list.seen);
    __sync_synchronize();               /* the list and the error before the id that publishes them */
    s_scan_done_id = s_scan_run_id;
    s_scan_run_id  = 0;
    s_scan_waiting = false;
}

/* The scan is over: answer it, then back to the link (joined) or the C3 back into reset (a C3
   started for the scan; a configured pod's next poll starts its normal connect). */
static void scan_finish(const char *err) {
    scan_publish(err);
    s_await_resp = 0;
    if (s_scan_only) {
        s_scan_only = false;
        esp_hosted_spi_stop(); s_started = false;
        s_state = WC_DOWN;
    } else {
        s_rssi_deadline = make_timeout_time_ms(1000);
        s_state = WC_UP;
    }
}

static void scan_send_start(void) {
    wifi_scan_list_clear(&s_scan_list);
    uint8_t p[8];
    pb_w w = { p, sizeof(p), 0, false };
    pb_int(&w, 2, 1);                   /* Rpc_Req_WifiScanStart.block: answer when done */
    if (w.err || !send_req(ID_REQ_SCAN_START, ID_RESP_SCAN_START, p, w.len)) {
        scan_finish("could not send the scan request to the ESP32-C3");
        return;
    }
    s_deadline = make_timeout_time_ms(SCAN_TIMEOUT_MS);
    s_state = WC_SCAN_START;
}

static void scan_send_record(void) {
    s_scan_rec_ok = false;
    if (!send_req(ID_REQ_SCAN_AP_REC, ID_RESP_SCAN_AP_REC, NULL, 0)) {
        scan_finish("could not read the scan result from the ESP32-C3");
        return;
    }
    s_scan_read++;
    s_state = WC_SCAN_REC;
}

/* Net task: take a new request (esp_wifi_ctrl_scan_request) when nothing else holds the C3. */
static void scan_intake(void) {
    uint32_t want = s_scan_req_id;
    if (s_scan_run_id || want == s_scan_done_id) return;
    if (!s_module) {
        s_scan_run_id = want;
        scan_publish(ESP_WIFI_NO_MODULE);
        return;
    }
    if (s_paused || esp_rom_flash_busy() || s_state == WC_FLASHING) {
        s_scan_run_id = want;
        scan_publish("the ESP32-C3 is being flashed or erased; try again when that is done");
        return;
    }
    if (s_state == WC_UP) {             /* joined: scan alongside, the link stays up */
        s_scan_run_id = want;
        s_scan_only = false;
        scan_send_start();
        return;
    }
    if (!s_configured || s_state == WC_BACKOFF || (s_state == WC_DOWN && !s_started)) {
        /* No link to keep: start the C3 for the scan alone (Init, SetStorage, SetMode, Start,
           no credentials, no Connect). */
        s_scan_run_id = want;
        s_scan_only = true;
        if (s_started) esp_hosted_spi_stop();
        s_await_resp = 0;
        esp_hosted_spi_start(); s_started = true;
        s_state = WC_WAIT_READY;
        s_boot_deadline = make_timeout_time_ms(SLAVE_BOOT_TIMEOUT_MS);
        return;
    }
    /* Mid-connect (or a GetRssi outstanding): wait for it to end, joined or failed. */
    if (!s_scan_waiting) {
        s_scan_waiting = true;
        s_scan_wait = make_timeout_time_ms(SCAN_WAIT_MS);
    } else if (time_reached(s_scan_wait)) {
        s_scan_run_id = want;
        scan_publish("Wi-Fi is still connecting; try the scan again in a moment");
    }
}

uint32_t esp_wifi_ctrl_scan_request(void) {
    uint32_t id = s_scan_req_id;
    if (id != s_scan_done_id) return id;   /* one is waiting or running: share its answer */
    if (++id == 0) id = 1;
    s_scan_req_id = id;
    return id;
}

esp_wifi_scan_state_t esp_wifi_ctrl_scan_result(uint32_t id, const wifi_scan_list_t **list,
                                                const char **err) {
    uint32_t done = s_scan_done_id;
    if (id == 0 || id > s_scan_req_id) return ESP_WIFI_SCAN_UNKNOWN;
    if (done < id) return ESP_WIFI_SCAN_RUNNING;
    __sync_synchronize();
    if (s_scan_err) { if (err) *err = s_scan_err; return ESP_WIFI_SCAN_FAILED; }
    if (list) *list = &s_scan_list;
    return ESP_WIFI_SCAN_DONE;
}

static void to_backoff(const char *why) {
    if (s_scan_run_id) {                /* a failure mid-scan ends the scan with it */
        bool only = s_scan_only;
        scan_finish(why);
        if (only) return;               /* nothing to back off from: the C3 is off again */
    }
    log_printf("[wifi] %s — backoff\n", why);
    s_await_resp = 0;
    s_state = WC_BACKOFF;
    s_deadline = make_timeout_time_ms(RETRY_BACKOFF_MS);
}

/* The C3 is dead or has rebooted under us: its association is gone, so drop the Wi-Fi
   link now (not after the backoff, or the netif keeps routing into a dead slave for
   another RETRY_BACKOFF_MS), hold the C3 in reset, and let the normal backoff restart
   it from reset.  s_started stays true so the poll does not skip the backoff; the
   WC_BACKOFF expiry clears it.  Runs on the net task, the only task that touches lwIP. */
static void slave_lost(const char *why) {
    esp_netif_set_link_up(false);
    esp_hosted_spi_stop();
    s_rssi_misses = 0;
    s_evt_connected = s_evt_disconnected = false;
    log_printf("[wifi] ESP32-C3 lost: %s (no-response %lu, rebooted %lu) — link down, restarting it\n",
           why, (unsigned long)s_lost_rssi, (unsigned long)s_lost_reboot);
    to_backoff("ESP32-C3 restart");
}

/* One GetRssi went unanswered.  Returns true if that was the last straw (the link was
   restarted; caller returns). */
static bool rssi_missed(const char *what) {
    s_await_resp = 0;
    s_rssi_deadline = make_timeout_time_ms(RSSI_PERIOD_MS);
    s_state = WC_UP;
    if (++s_rssi_misses < RSSI_MAX_MISSES) {
        log_printf("[wifi] RSSI %s (%u/%u)\n", what, (unsigned)s_rssi_misses, (unsigned)RSSI_MAX_MISSES);
        return false;
    }
    s_lost_rssi++;
    slave_lost("no response to RSSI requests");
    return true;
}

/* Shared by WC_UP / WC_UP_RSSI: if the slave reported a disconnect, drop the
   Wi-Fi link and re-issue connect (the slave stays inited). Returns true if a
   drop was handled (caller should return immediately). */
static bool esp_wifi_ctrl_reconnect_if_dropped(void) {
    if (!s_evt_disconnected) return false;
    s_evt_disconnected = false;
    esp_netif_set_link_up(false);
    log_printf("[wifi] disconnected — reconnecting\n");
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
    if (!s_module) {   /* digital v3r3: stored credentials (from an older board) stay unused */
        s_configured = false;
        log_printf("[wifi] no ESP32-C3 on this board: built-in Wi-Fi off\n");
        return;
    }
    if (s_configured) log_printf("[wifi] configured for SSID \"%s\"\n", s_cfg.ssid);
    else              log_printf("[wifi] no SSID configured — Wi-Fi disabled\n");
}

void esp_wifi_ctrl_image_installed(void) {
    s_flash_tried = false;   /* a new image in the W25Q earns another automatic try */
}

void esp_wifi_ctrl_reload(void) {
    if (s_scan_run_id) scan_publish("the Wi-Fi settings changed during the scan; scan again");
    s_scan_only = false;
    if (s_started) { esp_hosted_spi_stop(); s_started = false; }
    s_flash_tried = false;   /* new credentials (or a manual flash) earn another automatic try */
    esp_netif_set_link_up(false);
    s_state = WC_DOWN;
    esp_wifi_ctrl_init();
}

void esp_wifi_ctrl_poll(void) {
    scan_intake();
    if (!s_module) return;   /* nothing to start, nothing to flash */
    /* A backoff that expires mid-flash would release the C3's reset and knock it out of its
       ROM loader, so leave the straps alone while a console command holds them. */
    if (s_paused) return;
    /* A C3 flash runs in the background on the hw worker and owns EN/BOOT until it ends, also
       when new credentials (wifi-set, wifi-clear) reload this control in the meantime. */
    if (esp_rom_flash_busy()) return;
    /* Gate the whole co-processor on configuration. */
    if (!s_configured && !s_scan_only) {
        if (s_started) { esp_hosted_spi_stop(); s_started = false; s_state = WC_DOWN; }
        return;
    }
    if (s_state == WC_FLASHING) {           /* the C3 is the worker's until it reports back */
        if (!s_flash_done) return;
        s_flash_done = false;
        if (s_flash_ok) {
            log_printf("[wifi] ESP32-C3 flashed — starting it\n");
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

    /* A boot announce is only expected while waiting for one.  Anywhere later in the
       sequence it means the C3 reset itself and lost everything we configured. */
    if (esp_hosted_spi_take_slave_reset()) {
        switch (s_state) {
        case WC_INIT: case WC_SETSTORAGE: case WC_SETMODE: case WC_SETCFG: case WC_START:
        case WC_CONNECT: case WC_GETMAC: case WC_WAIT_ASSOC: case WC_UP: case WC_UP_RSSI:
        case WC_SCAN_START: case WC_SCAN_NUM: case WC_SCAN_REC: case WC_SCAN_CLEAR:
            s_lost_reboot++;
            slave_lost("it announced a new boot");
            return;
        default:                            /* waiting for it anyway, or restarting */
            break;
        }
    }

    switch (s_state) {
    case WC_DOWN:
        s_state = WC_WAIT_READY;
        s_boot_deadline = make_timeout_time_ms(SLAVE_BOOT_TIMEOUT_MS);
        return;

    case WC_FLASHING:                       /* handled above */
        return;

    case WC_WAIT_READY:
        if (!esp_hosted_spi_ready() && time_reached(s_boot_deadline) && s_scan_only) {
            to_backoff("the ESP32-C3 did not start (no boot event; flash-esp32 reinstalls it)");
            return;
        }
        if (!esp_hosted_spi_ready() && time_reached(s_boot_deadline)) {
            if (!s_flash_tried && blob_store_present(BLOB_ESP)) {
                /* Hold the C3 in reset and hand it to the worker, which runs the ~2 minute
                   ROM-loader flash in the background (esp_rom_flash_start). */
                esp_hosted_spi_stop(); s_started = false;
                s_flash_tried = true;
                s_flash_done  = false;
                if (hw_worker_submit_esp_flash()) {
                    log_printf("[wifi] no boot event from the ESP32-C3 in %u s — flashing its "
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
            size_t n = build_set_storage(p, sizeof(p), WIFI_STORAGE_RAM);
            if (n && send_req(ID_REQ_WIFI_SETSTORAGE, ID_RESP_WIFI_SETSTORAGE, p, n))
                s_state = WC_SETSTORAGE;
            else to_backoff("setstorage build/send failed");
        } else if (time_reached(s_deadline)) to_backoff("init timeout");
        return;

    case WC_SETSTORAGE:
        /* The one step whose result code gates the next: without RAM storage confirmed, the
           mode and the credentials would land in the C3's flash, so they are not sent. */
        if (s_resp_ready) {
            if (s_resp_status != 0) {
                printf("[wifi] ESP32-C3 refused RAM storage (0x%lx): credentials not sent\n",
                       (unsigned long)(uint32_t)s_resp_status);
                to_backoff("setstorage failed");
                return;
            }
            uint8_t p[16];
            size_t n = build_set_mode(p, sizeof(p), WIFI_MODE_STA);
            if (n && send_req(ID_REQ_SET_MODE, ID_RESP_SET_MODE, p, n)) s_state = WC_SETMODE;
            else to_backoff("setmode build/send failed");
        } else if (time_reached(s_deadline)) to_backoff("setstorage timeout");
        return;

    case WC_SETMODE:
        if (s_resp_ready && s_scan_only) {      /* a scan needs no credentials */
            if (send_req(ID_REQ_WIFI_START, ID_RESP_WIFI_START, NULL, 0)) s_state = WC_START;
            else to_backoff("start send failed");
        } else if (s_resp_ready) {
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
        if (s_resp_ready && s_scan_only) {
            scan_send_start();
        } else if (s_resp_ready) {
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
                log_printf("[wifi] sta MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
                       s_mac[0], s_mac[1], s_mac[2], s_mac[3], s_mac[4], s_mac[5]);
            } else {
                log_printf("[wifi] GetMac: no MAC in resp — netif keeps provisional MAC (DHCP will fail)\n");
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
            log_printf("[wifi] associated — link up\n");
            s_rssi_deadline = make_timeout_time_ms(1000);   /* first RSSI read soon */
            s_rssi_misses = 0;
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
            else (void)rssi_missed("request not queued (TX full)");   /* retry later */
        }
        return;

    case WC_UP_RSSI:
        if (esp_wifi_ctrl_reconnect_if_dropped()) return;
        if (s_resp_ready) {                               /* the C3 is alive */
            s_await_resp = 0;
            s_rssi_misses = 0;
            s_rssi_deadline = make_timeout_time_ms(RSSI_PERIOD_MS);
            s_state = WC_UP;
        } else if (time_reached(s_deadline)) {
            (void)rssi_missed("request timed out");
        }
        return;

    case WC_SCAN_START:
        if (s_resp_ready) {
            if (s_resp_status != 0) {
                static char why[64];
                snprintf(why, sizeof(why), "the ESP32-C3 refused the scan (error 0x%lx)",
                         (unsigned long)(uint32_t)s_resp_status);
                scan_finish(why);
            } else if (send_req(ID_REQ_SCAN_AP_NUM, ID_RESP_SCAN_AP_NUM, NULL, 0)) {
                s_scan_num = 0;
                s_state = WC_SCAN_NUM;
            } else {
                scan_finish("could not read the scan result from the ESP32-C3");
            }
        } else if (time_reached(s_deadline)) {
            scan_finish("the ESP32-C3 did not finish the scan");
        }
        return;

    case WC_SCAN_NUM:
        if (s_resp_ready) {
            s_scan_read = 0;
            if (s_scan_num > 0) scan_send_record();
            else scan_finish(NULL);
        } else if (time_reached(s_deadline)) {
            scan_finish("the ESP32-C3 did not report the scan result");
        }
        return;

    case WC_SCAN_REC:
        if (s_resp_ready) {
            if (s_scan_rec_ok)
                wifi_scan_list_add(&s_scan_list, s_scan_rec.ssid, strlen(s_scan_rec.ssid),
                                   s_scan_rec.rssi, s_scan_rec.channel, s_scan_rec.authmode);
            if (s_scan_rec_ok && s_scan_read < s_scan_num && s_scan_read < SCAN_READ_MAX) {
                scan_send_record();
            } else if (s_scan_read < s_scan_num) {   /* drop the rest on the C3 */
                if (send_req(ID_REQ_SCAN_CLEAR, ID_RESP_SCAN_CLEAR, NULL, 0)) s_state = WC_SCAN_CLEAR;
                else scan_finish(NULL);
            } else {
                scan_finish(NULL);
            }
        } else if (time_reached(s_deadline)) {
            scan_finish("the ESP32-C3 did not report the scan result");
        }
        return;

    case WC_SCAN_CLEAR:
        if (s_resp_ready || time_reached(s_deadline)) scan_finish(NULL);
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
