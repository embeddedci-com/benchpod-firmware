/*
 * wifi_bridge_net.c — the Wi-Fi bridge client on the net task (see wifi_bridge_net.h).
 */
#include "wifi_bridge_net.h"

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "bp_log.h"
#include "device_identity.h"
#include "lwip/err.h"
#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "mbedtls/sha256.h"
#include "net_server.h"
#include "rng.h"
#include "stm32h5xx_hal.h"
#include "task.h"
#include "watchdog.h"
#include "blob_store.h"
#include "ota.h"
#include "command_handler_internal.h"
#include "esp_rom_flash.h"
#include "w25q.h"
#include "wifi_bridge_ota.h"

static wb_t s_wb;                  /* net task only */
static struct netif *s_eth;
static volatile bool s_present;
static wb_info_t s_info;           /* snapshot for other tasks, copied under a critical section */
static uint32_t s_info_at;
wb_reply_t g_wifi_bridge_reply;    /* hw worker only, see the header */

static void io_send(const uint8_t *head, size_t hl, const uint8_t *body, size_t bl, const uint8_t *tail,
                    size_t tl, void *ctx)
{
    (void)ctx;
    if (!s_eth || !netif_is_link_up(s_eth)) {
        return;
    }
    size_t len = hl + bl + tl;
    size_t padded = len < BWP_MIN_FRAME ? BWP_MIN_FRAME : len;
    struct pbuf *p = pbuf_alloc(PBUF_RAW, (u16_t)padded, PBUF_RAM);  /* contiguous, from the lwIP heap */
    if (!p) {
        return;
    }
    uint8_t *f = p->payload;
    memcpy(f, head, hl);
    if (bl) memcpy(f + hl, body, bl);
    if (tl) memcpy(f + hl + bl, tail, tl);
    memset(f + len, 0, padded - len);
    s_eth->linkoutput(s_eth, p);
    pbuf_free(p);
}

/* Our X25519 secret for a bridge = SHA-256(Ed25519 signature of the bridge MAC under
   DEVICE_ID_CTX_WIFI_BRIDGE): the same after every reboot, different for every bridge, and only
   this pod can compute it. With the bridge's public key (HELLO) it gives the pairing key again. */
static int io_secret(const uint8_t mac[6], uint8_t secret[32], void *ctx)
{
    (void)ctx;
    uint8_t sig[DEVICE_ID_SIG_LEN];
    if (device_identity_sign_ctx(DEVICE_ID_CTX_WIFI_BRIDGE, mac, 6, sig) != 0) {
        return -1;
    }
    int r = mbedtls_sha256(sig, sizeof(sig), secret, 0);
    memset(sig, 0, sizeof(sig));
    return r == 0 ? 0 : -1;
}

void wifi_bridge_net_init(struct netif *eth)
{
    static const wb_io_t io = {.send = io_send, .derive_secret = io_secret, .ctx = NULL};
    s_eth = eth;
    wb_init(&s_wb, eth->hwaddr, &io);
    log_printf("[bridge] Wi-Fi bridge control channel on (dev build)\r\n");
}

int wifi_bridge_eth_input(struct pbuf *p, struct netif *netif)
{
    if (netif != s_eth || p->len < 14) {
        return ERR_VAL;
    }
    const uint8_t *h = p->payload;
    if (h[12] != (BWP_ETHERTYPE >> 8) || h[13] != (BWP_ETHERTYPE & 0xFF)) {
        return ERR_VAL;  /* not ours: lwIP counts and drops it as before */
    }
    if (p->tot_len == p->len) {
        wb_rx(&s_wb, h, p->len, HAL_GetTick());
    } else {
        struct pbuf *q = pbuf_clone(PBUF_RAW, PBUF_RAM, p);  /* chained: make it contiguous */
        if (q) {
            wb_rx(&s_wb, q->payload, q->len, HAL_GetTick());
            pbuf_free(q);
        }
    }
    pbuf_free(p);
    return ERR_OK;
}

static bool s_joined_edge;
static uint8_t s_last_state = 0xFF;

bool wifi_bridge_take_joined(void)
{
    bool j = s_joined_edge;
    s_joined_edge = false;
    return j;
}

void wifi_bridge_net_poll(void)
{
    if (!s_eth) {
        return;
    }
    uint32_t now = HAL_GetTick();
    bool was = s_present;
    wb_set_link(&s_wb, netif_is_link_up(s_eth), now);
    wb_poll(&s_wb, now);
    if ((uint32_t)(now - s_info_at) >= 100u) {
        wb_info_t i;
        wb_get_info(&s_wb, now, &i);
        taskENTER_CRITICAL();
        s_info = i;
        taskEXIT_CRITICAL();
        s_info_at = now;
        s_present = i.present;
        uint8_t st = i.present ? i.state : 0xFF;
        if (st == BWP_ST_CONNECTED && s_last_state != BWP_ST_CONNECTED) s_joined_edge = true;
        s_last_state = st;
        if (i.present != was) {
            char mac[18];
            snprintf(mac, sizeof(mac), "%02x:%02x:%02x:%02x:%02x:%02x", i.mac[0], i.mac[1], i.mac[2], i.mac[3],
                     i.mac[4], i.mac[5]);
            log_printf("[bridge] Wi-Fi bridge %s %s (fw %s)\r\n", i.present ? "found:" : "gone:", mac,
                       i.fw_version[0] ? i.fw_version : "?");
        }
    }
}

bool wifi_bridge_present(void)
{
    return s_present;
}

void wifi_bridge_get_info(wb_info_t *out)
{
    taskENTER_CRITICAL();
    *out = s_info;
    taskEXIT_CRITICAL();
}

/* ---- worker side: one request at a time, through net_call_sync */
typedef struct {
    const char *ssid;            /* set: a sealed SET_WIFI instead of type/tlvs */
    const char *pass;
    uint8_t seal_nonce[BWP_SEAL_NONCE_LEN];
    uint8_t type;
    const uint8_t *tlvs;
    size_t len;
    uint32_t timeout_ms;
    int rc;
    wb_reply_t *out;
    wb_req_state_t state;
} call_t;

static void on_net_start(void *arg)
{
    call_t *c = arg;
    c->rc = c->ssid ? wb_request_set_wifi(&s_wb, c->ssid, c->pass, c->seal_nonce, c->timeout_ms, HAL_GetTick())
                    : wb_request(&s_wb, c->type, c->tlvs, c->len, c->timeout_ms, HAL_GetTick());
}

static void on_net_take(void *arg)
{
    call_t *c = arg;
    c->state = wb_request_state(&s_wb);
    if (c->state == WB_REQ_DONE || c->state == WB_REQ_TIMEOUT) {
        *c->out = *wb_request_reply(&s_wb);
        wb_request_release(&s_wb);
    }
}

static int run_call(call_t *cp, wb_reply_t *out, const char **err)
{
    call_t c = *cp;
    if (wifi_bridge_ota_busy()) {
        memset(out, 0, sizeof(*out));
        *err = "the Wi-Fi bridge is being updated (about a minute); try again when it ends";
        return -1;
    }
    uint32_t timeout_ms = c.timeout_ms;
    /* Long enough for the first pairing (a few seconds) plus every retry of this request. */
    uint32_t start = HAL_GetTick(), limit = 6000u + WB_TRIES * timeout_ms + 1000u;
    memset(out, 0, sizeof(*out));
    *err = NULL;
    for (;;) {
        if (!net_call_sync(on_net_start, &c, 2000)) {
            *err = "net task busy";
            return -1;
        }
        if (c.rc == 0) {
            break;
        }
        if (c.rc == -2) {
            *err = "no Wi-Fi bridge on the Ethernet port";
            return -1;
        }
        if (c.rc == -4) {
            *err = "SSID 1-32 bytes; password empty, 8-63 printable characters or 64 hex digits";
            return -1;
        }
        if (c.rc == -3) {
            wb_info_t i;
            wifi_bridge_get_info(&i);
            *err = i.problem ? i.problem : "the Wi-Fi bridge cannot take this request";
            return -1;
        }
        if ((uint32_t)(HAL_GetTick() - start) >= limit) {
            *err = "the Wi-Fi bridge is busy or not paired yet";
            return -1;
        }
        watchdog_heartbeat(WD_TASK_WORKER, "bridge");
        vTaskDelay(pdMS_TO_TICKS(20));  /* our own PAIR or STATUS is in flight */
    }
    for (;;) {
        watchdog_heartbeat(WD_TASK_WORKER, "bridge");  /* a SCAN takes seconds */
        vTaskDelay(pdMS_TO_TICKS(10));
        if (!net_call_sync(on_net_take, &c, 2000)) {
            *err = "net task busy";
            return -1;
        }
        if (c.state == WB_REQ_DONE) {
            return 0;
        }
        if (c.state == WB_REQ_TIMEOUT) {
            *err = out->message[0] ? out->message : "no answer from the Wi-Fi bridge";
            return -1;
        }
    }
}

int wifi_bridge_call(uint8_t type, const uint8_t *tlvs, size_t len, uint32_t timeout_ms, wb_reply_t *out,
                     const char **err)
{
    call_t c = {.type = type, .tlvs = tlvs, .len = len, .timeout_ms = timeout_ms, .rc = -1, .out = out};
    return run_call(&c, out, err);
}

int wifi_bridge_set_wifi(const char *ssid, const char *pass, wb_reply_t *out, const char **err)
{
    call_t c = {.ssid = ssid, .pass = pass, .type = BWP_SET_WIFI, .timeout_ms = 1000, .rc = -1, .out = out};
    if (rng_fill(c.seal_nonce, sizeof(c.seal_nonce)) != 0) {  /* a fresh nonce per message */
        memset(out, 0, sizeof(*out));
        *err = "no random numbers (TRNG failed)";
        return -1;
    }
    return run_call(&c, out, err);
}

/* ---- bridge firmware updates (wifi_bridge_ota.c), driven from the hw worker ---------------- */
static wbo_t s_relay;          /* hw worker only */
static wbo_image_t s_img;      /* what slot "bridge" holds, with its manifest */
static uint8_t s_img_sha[32];
static bool s_img_read, s_quiesced;

typedef struct { uint8_t type; const uint8_t *tlvs; size_t len; uint32_t timeout_ms; int rc; } ext_t;
typedef struct { wb_reply_t *out; wb_req_state_t st; } take_t;

static void on_net_ext(void *arg)
{
    ext_t *e = arg;
    e->rc = wb_request_ext(&s_wb, e->type, e->tlvs, e->len, e->timeout_ms, HAL_GetTick());
}

static void on_net_take_any(void *arg)
{
    take_t *t = arg;
    t->st = wb_request_state(&s_wb);
    if (t->st == WB_REQ_DONE || t->st == WB_REQ_TIMEOUT) {
        *t->out = *wb_request_reply(&s_wb);
        wb_request_release(&s_wb);
    }
}

static int ro_start(void *ctx, uint8_t type, const uint8_t *tlvs, size_t len, uint32_t timeout_ms)
{
    (void)ctx;
    ext_t e = {.type = type, .tlvs = tlvs, .len = len, .timeout_ms = timeout_ms};
    if (!net_call_sync(on_net_ext, &e, 2000)) return -1;
    return e.rc == 0 ? 0 : e.rc == -1 ? -1 : -2;
}

static wb_req_state_t ro_take(void *ctx, wb_reply_t *out)
{
    (void)ctx;
    take_t t = {.out = out, .st = WB_REQ_PENDING};
    return net_call_sync(on_net_take_any, &t, 2000) ? t.st : WB_REQ_PENDING;
}

/* Like the ESP32-C3 flasher: stop the gateware's PSRAM masters once when the update starts (the
   busy gate, wifi_bridge_ota_busy -> bus_busy_reason, keeps them stopped), then take the bus for
   each read only. */
static int ro_read(void *ctx, uint32_t off, uint8_t *buf, uint32_t n)
{
    (void)ctx;
    if (!s_quiesced) {
        w25q_session_open();
        w25q_session_close();
        s_quiesced = true;
    }
    w25q_open();
    int rc = blob_store_read((void *)(uintptr_t)BLOB_BRIDGE, off, buf, n);
    w25q_close();
    return rc;
}

static void *ro_alloc(size_t n) { return pvPortMalloc(n); }
static void ro_release(void *p) { vPortFree(p); }

static uint32_t le32(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

/* Read the slot's manifest once per image (the blob store kept it in the header sector). */
static void refresh_image(void)
{
    const blob_info_t *bi = blob_store_info(BLOB_BRIDGE);
    if (!bi || !bi->present) {
        s_img.present = false;
        s_img_read = false;
        return;
    }
    if (s_img_read && memcmp(s_img_sha, bi->sha256, 32) == 0) return;
    if (bus_busy_reason()) return;   /* read it when the bus is free */
    uint8_t *m = s_img.manifest;
    w25q_open();
    int rc = blob_store_read_extra(BLOB_BRIDGE, m, WBO_MANIFEST_LEN);
    w25q_close();
    s_img_read = true;
    memcpy(s_img_sha, bi->sha256, 32);
    /* The pod checked the signature when the image was installed; here only that the manifest
       is the one for this image (the bridge checks the signature again). */
    s_img.present = rc == 0 && memcmp(m, "BPSG", 4) == 0 && m[6] == OTA_TARGET_BRIDGE &&
                    le32(m + 8) == bi->len && memcmp(m + 12, bi->sha256, 32) == 0;
    s_img.size = bi->len;
    s_img.version = le32(m + 44);
}

void wifi_bridge_ota_poll(void)
{
    static const wbo_io_t io = {.start = ro_start, .take = ro_take, .read = ro_read,
                                .alloc = ro_alloc, .release = ro_release};
    static bool init;
    if (!init) {
        wbo_init(&s_relay, &io, &g_wifi_bridge_reply);   /* shared: see wifi_bridge_call */
        init = true;
    }
    refresh_image();
    wb_info_t i;
    wifi_bridge_get_info(&i);
    bool was = wbo_busy(&s_relay);
    char before[sizeof(s_relay.status)];
    strcpy(before, s_relay.status);
    wbo_poll(&s_relay, &i, &s_img, !heavy_or_claimed() && !esp_rom_flash_busy(), HAL_GetTick());
    if (!wbo_busy(&s_relay)) s_quiesced = false;
    if (wbo_busy(&s_relay) != was || (strcmp(before, s_relay.status) != 0 && !strstr(s_relay.status, "%")))
        log_printf("[bridge] %s\r\n", s_relay.status);
}

bool wifi_bridge_ota_busy(void) { return wbo_busy(&s_relay); }
const char *wifi_bridge_ota_status(void) { return wbo_status(&s_relay); }
void wifi_bridge_ota_force(void) { wbo_force(&s_relay); }
bool wifi_bridge_image(uint32_t *version) {
    if (version) *version = s_img.version;
    return s_img.present;
}
