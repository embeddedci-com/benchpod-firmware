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
#include "stm32h5xx_hal.h"
#include "task.h"
#include "watchdog.h"

static wb_t s_wb;                  /* net task only */
static struct netif *s_eth;
static volatile bool s_present;
static wb_info_t s_info;           /* snapshot for other tasks, copied under a critical section */
static uint32_t s_info_at;
wb_reply_t g_wifi_bridge_reply;    /* hw worker only, see the header */

static void io_send(const uint8_t *frame, size_t len, void *ctx)
{
    (void)ctx;
    if (!s_eth || !netif_is_link_up(s_eth)) {
        return;
    }
    struct pbuf *p = pbuf_alloc(PBUF_RAW, (u16_t)len, PBUF_RAM);
    if (!p) {
        return;
    }
    if (pbuf_take(p, frame, (u16_t)len) == ERR_OK) {
        s_eth->linkoutput(s_eth, p);
    }
    pbuf_free(p);
}

/* key = SHA-256(Ed25519 signature of the bridge MAC under DEVICE_ID_CTX_WIFI_BRIDGE): the same
   after every reboot, different for every bridge, and only this pod can compute it. */
static int io_key(const uint8_t mac[6], uint8_t key[BWP_KEY_LEN], void *ctx)
{
    (void)ctx;
    uint8_t sig[DEVICE_ID_SIG_LEN];
    if (device_identity_sign_ctx(DEVICE_ID_CTX_WIFI_BRIDGE, mac, 6, sig) != 0) {
        return -1;
    }
    int r = mbedtls_sha256(sig, sizeof(sig), key, 0);
    memset(sig, 0, sizeof(sig));
    return r == 0 ? 0 : -1;
}

void wifi_bridge_net_init(struct netif *eth)
{
    static const wb_io_t io = {.send = io_send, .derive_key = io_key, .ctx = NULL};
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
    c->rc = wb_request(&s_wb, c->type, c->tlvs, c->len, c->timeout_ms, HAL_GetTick());
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

int wifi_bridge_call(uint8_t type, const uint8_t *tlvs, size_t len, uint32_t timeout_ms, wb_reply_t *out,
                     const char **err)
{
    call_t c = {.type = type, .tlvs = tlvs, .len = len, .timeout_ms = timeout_ms, .rc = -1, .out = out};
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

size_t wifi_bridge_tlvs_set_wifi(uint8_t *buf, size_t cap, const char *ssid, const char *pass)
{
    size_t a = strlen(ssid), b = strlen(pass);
    size_t n = 3 + a + (b ? 3 + b : 0);
    if (a > 0xff || b > 0xff || n > cap) {
        return 0;
    }
    size_t i = 0;
    buf[i++] = BWP_T_SSID, buf[i++] = 0, buf[i++] = (uint8_t)a;
    memcpy(buf + i, ssid, a), i += a;
    if (b) {
        buf[i++] = BWP_T_PASSPHRASE, buf[i++] = 0, buf[i++] = (uint8_t)b;
        memcpy(buf + i, pass, b), i += b;
    }
    return i;
}
