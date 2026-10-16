// BenchPod Wi-Fi bridge control protocol: encode/decode. See docs/PROTOCOL.md.
#include "bwp.h"

#include <string.h>

#include "hmac_sha256.h"

static void put_be16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)v;
}

static uint16_t get_be16(const uint8_t *p)
{
    return (uint16_t)((p[0] << 8) | p[1]);
}

static void put_be32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)v;
}

static uint32_t get_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}

static uint8_t *reserve(bwp_writer_t *w, size_t n)
{
    if (w->overflow || w->len + n > w->cap) {
        w->overflow = true;
        return NULL;
    }
    uint8_t *p = w->buf + w->len;
    w->len += n;
    return p;
}

void bwp_begin(bwp_writer_t *w, uint8_t *buf, size_t cap, const uint8_t dst[6], const uint8_t src[6],
               uint8_t type, uint16_t seq)
{
    w->buf = buf;
    w->cap = cap > BWP_MAX_FRAME ? BWP_MAX_FRAME : cap;
    w->len = 0;
    w->nest_at = 0;
    w->overflow = false;
    uint8_t *p = reserve(w, BWP_ETH_HDR_LEN + BWP_HDR_LEN);
    if (!p) {
        return;
    }
    memcpy(p, dst, 6);
    memcpy(p + 6, src, 6);
    put_be16(p + 12, BWP_ETHERTYPE);
    p[14] = BWP_MAGIC0;
    p[15] = BWP_MAGIC1;
    p[16] = BWP_VERSION;
    p[17] = type;
    put_be16(p + 18, seq);
    put_be16(p + 20, 0);  // body length, patched by bwp_finish
}

void bwp_put(bwp_writer_t *w, uint8_t tag, const void *val, size_t len)
{
    if (len > 0xFFFF) {
        w->overflow = true;
        return;
    }
    uint8_t *p = reserve(w, 3 + len);
    if (!p) {
        return;
    }
    p[0] = tag;
    put_be16(p + 1, (uint16_t)len);
    if (len) {
        memcpy(p + 3, val, len);
    }
}

void bwp_put_u8(bwp_writer_t *w, uint8_t tag, uint8_t v)
{
    bwp_put(w, tag, &v, 1);
}

void bwp_put_u32(bwp_writer_t *w, uint8_t tag, uint32_t v)
{
    uint8_t b[4];
    put_be32(b, v);
    bwp_put(w, tag, b, 4);
}

void bwp_put_str(bwp_writer_t *w, uint8_t tag, const char *s)
{
    bwp_put(w, tag, s, strlen(s));
}

void bwp_nest_begin(bwp_writer_t *w, uint8_t tag)
{
    uint8_t *p = reserve(w, 3);
    if (!p) {
        return;
    }
    p[0] = tag;
    w->nest_at = w->len - 2;
}

void bwp_nest_end(bwp_writer_t *w)
{
    if (!w->overflow && w->nest_at) {
        put_be16(w->buf + w->nest_at, (uint16_t)(w->len - w->nest_at - 2));
    }
    w->nest_at = 0;
}

size_t bwp_finish(bwp_writer_t *w)
{
    if (w->overflow) {
        return 0;
    }
    put_be16(w->buf + 20, (uint16_t)(w->len - BWP_ETH_HDR_LEN - BWP_HDR_LEN));
    if (w->len < BWP_MIN_FRAME) {
        if (w->cap < BWP_MIN_FRAME) {
            return 0;
        }
        memset(w->buf + w->len, 0, BWP_MIN_FRAME - w->len);
        w->len = BWP_MIN_FRAME;
    }
    return w->len;
}

static void auth_mac(const uint8_t key[BWP_KEY_LEN], const uint8_t nonce[BWP_NONCE_LEN], const uint8_t *from,
                     size_t len, uint8_t out[BWP_AUTH_LEN])
{
    hmac_sha256_ctx_t h;
    uint8_t full[32];
    hmac_sha256_init(&h, key, BWP_KEY_LEN);
    hmac_sha256_update(&h, nonce, BWP_NONCE_LEN);
    hmac_sha256_update(&h, from, len);
    hmac_sha256_final(&h, full);
    memcpy(out, full, BWP_AUTH_LEN);
    memset(full, 0, sizeof(full));
}

void bwp_sign(bwp_writer_t *w, const uint8_t key[BWP_KEY_LEN], const uint8_t nonce[BWP_NONCE_LEN],
              uint32_t counter)
{
    bwp_put_u32(w, BWP_T_COUNTER, counter);
    if (w->overflow || w->len + 3 + BWP_AUTH_LEN > w->cap) {
        w->overflow = true;
        return;
    }
    // The MAC covers the header with its final body length, so patch it first.
    put_be16(w->buf + 20, (uint16_t)(w->len + 3 + BWP_AUTH_LEN - BWP_ETH_HDR_LEN - BWP_HDR_LEN));
    uint8_t mac[BWP_AUTH_LEN];
    auth_mac(key, nonce, w->buf + BWP_ETH_HDR_LEN, w->len - BWP_ETH_HDR_LEN, mac);
    bwp_put(w, BWP_T_AUTH, mac, sizeof(mac));
}

size_t bwp_sign_frame(uint8_t *buf, size_t cap, const uint8_t key[BWP_KEY_LEN],
                      const uint8_t nonce[BWP_NONCE_LEN], uint32_t counter)
{
    bwp_writer_t w = {.buf = buf, .cap = cap > BWP_MAX_FRAME ? BWP_MAX_FRAME : cap};
    w.len = BWP_ETH_HDR_LEN + BWP_HDR_LEN + get_be16(buf + 20);  // drop the padding
    if (w.len > w.cap) {
        return 0;
    }
    bwp_sign(&w, key, nonce, counter);
    return bwp_finish(&w);
}

bwp_verify_t bwp_verify(const uint8_t *frame, const bwp_msg_t *m, const uint8_t key[BWP_KEY_LEN],
                        const uint8_t nonce[BWP_NONCE_LEN], uint32_t *counter)
{
    bwp_iter_t it;
    uint8_t tag;
    const uint8_t *v, *auth = NULL;
    uint16_t vl;
    bool bad = false;
    bwp_iter_init(&it, m->body, m->body_len);
    while (bwp_iter_next(&it, &tag, &v, &vl)) {
        if (auth) {
            bad = true;  // something after AUTH: not covered by the MAC
        }
        if (tag == BWP_T_AUTH) {
            if (vl != BWP_AUTH_LEN || auth) {
                bad = true;
            }
            auth = v;
        }
    }
    if (!auth || !bwp_get_u32(m->body, m->body_len, BWP_T_COUNTER, counter)) {
        return BWP_VERIFY_MISSING;
    }
    if (bad) {
        return BWP_VERIFY_BAD;
    }
    uint8_t want[BWP_AUTH_LEN];
    const uint8_t *from = frame + BWP_ETH_HDR_LEN;
    auth_mac(key, nonce, from, (size_t)(auth - 3 - from), want);
    return ct_memcmp(want, auth, BWP_AUTH_LEN) == 0 ? BWP_VERIFY_OK : BWP_VERIFY_BAD;
}

bool bwp_tlv_valid(const uint8_t *body, size_t len)
{
    while (len) {
        if (len < 3) {
            return false;
        }
        size_t vlen = get_be16(body + 1);
        if (vlen > len - 3) {
            return false;
        }
        body += 3 + vlen;
        len -= 3 + vlen;
    }
    return true;
}

bwp_parse_t bwp_parse(const uint8_t *frame, size_t len, bwp_msg_t *out)
{
    if (len < BWP_ETH_HDR_LEN + 2 || get_be16(frame + 12) != BWP_ETHERTYPE) {
        return BWP_PARSE_NOT_BWP;
    }
    if (frame[14] != BWP_MAGIC0 || frame[15] != BWP_MAGIC1) {
        return BWP_PARSE_NOT_BWP;
    }
    if (len < BWP_ETH_HDR_LEN + BWP_HDR_LEN) {
        return BWP_PARSE_MALFORMED;
    }
    out->dst = frame;
    out->src = frame + 6;
    out->version = frame[16];
    out->type = frame[17];
    out->seq = get_be16(frame + 18);
    out->body_len = get_be16(frame + 20);
    out->body = frame + BWP_ETH_HDR_LEN + BWP_HDR_LEN;
    if (out->version != BWP_VERSION) {
        return BWP_PARSE_BAD_VERSION;
    }
    if (out->body_len > len - BWP_ETH_HDR_LEN - BWP_HDR_LEN) {
        return BWP_PARSE_MALFORMED;
    }
    if (!bwp_tlv_valid(out->body, out->body_len)) {
        return BWP_PARSE_MALFORMED;
    }
    return BWP_PARSE_OK;
}

void bwp_iter_init(bwp_iter_t *it, const uint8_t *body, size_t len)
{
    it->p = body;
    it->left = len;
}

bool bwp_iter_next(bwp_iter_t *it, uint8_t *tag, const uint8_t **val, uint16_t *len)
{
    if (it->left < 3) {
        return false;
    }
    uint16_t vlen = get_be16(it->p + 1);
    if (vlen > it->left - 3) {
        it->left = 0;
        return false;
    }
    *tag = it->p[0];
    *val = it->p + 3;
    *len = vlen;
    it->p += 3 + vlen;
    it->left -= 3 + vlen;
    return true;
}

bool bwp_find(const uint8_t *body, size_t len, uint8_t tag, const uint8_t **val, uint16_t *vlen)
{
    bwp_iter_t it;
    uint8_t t;
    bwp_iter_init(&it, body, len);
    while (bwp_iter_next(&it, &t, val, vlen)) {
        if (t == tag) {
            return true;
        }
    }
    return false;
}

bool bwp_get_u8(const uint8_t *body, size_t len, uint8_t tag, uint8_t *out)
{
    const uint8_t *v;
    uint16_t n;
    if (!bwp_find(body, len, tag, &v, &n) || n != 1) {
        return false;
    }
    *out = v[0];
    return true;
}

bool bwp_get_u32(const uint8_t *body, size_t len, uint8_t tag, uint32_t *out)
{
    const uint8_t *v;
    uint16_t n;
    if (!bwp_find(body, len, tag, &v, &n) || n != 4) {
        return false;
    }
    *out = get_be32(v);
    return true;
}

bool bwp_get_str(const uint8_t *body, size_t len, uint8_t tag, char *out, size_t cap)
{
    const uint8_t *v;
    uint16_t n;
    if (!bwp_find(body, len, tag, &v, &n) || n >= cap) {
        return false;
    }
    memcpy(out, v, n);
    out[n] = '\0';
    return true;
}

static const uint8_t BCAST[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

size_t bwp_build_hello(uint8_t *buf, size_t cap, const uint8_t dst[6], const bwp_hello_t *h)
{
    bwp_writer_t w;
    bwp_begin(&w, buf, cap, dst ? dst : BCAST, h->bridge_mac, BWP_HELLO, 0);
    bwp_put_str(&w, BWP_T_MODEL, BWP_MODEL);
    bwp_put_str(&w, BWP_T_FW_VERSION, h->fw_version);
    bwp_put_str(&w, BWP_T_HW_REV, h->hw_rev);
    bwp_put_u8(&w, BWP_T_STATE, h->state);
    bwp_put_u8(&w, BWP_T_FLAGS, h->flags);
    bwp_put(&w, BWP_T_BRIDGE_MAC, h->bridge_mac, 6);
    if (h->flags & BWP_F_POD_KNOWN) {
        bwp_put(&w, BWP_T_POD_MAC, h->pod_mac, 6);
    }
    bwp_put(&w, BWP_T_NONCE, h->nonce, BWP_NONCE_LEN);
    return bwp_finish(&w);
}

size_t bwp_build_status(uint8_t *buf, size_t cap, const uint8_t dst[6], const uint8_t src[6], uint16_t seq,
                        const bwp_status_t *s)
{
    bwp_writer_t w;
    bwp_begin(&w, buf, cap, dst, src, BWP_STATUS | BWP_RESPONSE, seq);
    bwp_put_u8(&w, BWP_T_RESULT, BWP_OK);
    bwp_put_u8(&w, BWP_T_STATE, s->state);
    bwp_put_u8(&w, BWP_T_FLAGS, s->flags);
    if (s->ssid[0]) {
        bwp_put_str(&w, BWP_T_SSID, s->ssid);
    }
    if (s->state == BWP_ST_CONNECTED) {
        bwp_put_u8(&w, BWP_T_CHANNEL, s->channel);
        bwp_put_u8(&w, BWP_T_RSSI, (uint8_t)s->rssi);
        bwp_put(&w, BWP_T_BSSID, s->bssid, 6);
    }
    if (s->reason) {
        bwp_put_u8(&w, BWP_T_REASON, s->reason);
    }
    bwp_put_u32(&w, BWP_T_UPTIME, s->uptime_s);
    if (s->flags & BWP_F_POD_KNOWN) {
        bwp_put(&w, BWP_T_POD_MAC, s->pod_mac, 6);
    }
    uint8_t c[28];
    const uint32_t v[7] = {s->counters.eth_rx, s->counters.wifi_tx, s->counters.wifi_tx_err, s->counters.wifi_rx,
                           s->counters.eth_tx, s->counters.eth_tx_err, s->counters.dropped};
    for (int i = 0; i < 7; i++) {
        put_be32(c + 4 * i, v[i]);
    }
    bwp_put(&w, BWP_T_COUNTERS, c, sizeof(c));
    return bwp_finish(&w);
}

size_t bwp_build_scan_result(uint8_t *buf, size_t cap, const uint8_t dst[6], const uint8_t src[6], uint16_t seq,
                             uint8_t result, const bwp_ap_t *aps, size_t n)
{
    bwp_writer_t w;
    bwp_begin(&w, buf, cap, dst, src, BWP_SCAN | BWP_RESPONSE, seq);
    bwp_put_u8(&w, BWP_T_RESULT, result);
    if (n > BWP_SCAN_MAX_APS) {
        n = BWP_SCAN_MAX_APS;
    }
    for (size_t i = 0; i < n; i++) {
        bwp_nest_begin(&w, BWP_T_AP);
        bwp_put_str(&w, BWP_T_SSID, aps[i].ssid);
        bwp_put_u8(&w, BWP_T_RSSI, (uint8_t)aps[i].rssi);
        bwp_put_u8(&w, BWP_T_CHANNEL, aps[i].channel);
        bwp_put_u8(&w, BWP_T_AUTHMODE, aps[i].authmode);
        bwp_put(&w, BWP_T_BSSID, aps[i].bssid, 6);
        bwp_nest_end(&w);
    }
    return bwp_finish(&w);
}

size_t bwp_build_result(uint8_t *buf, size_t cap, const uint8_t dst[6], const uint8_t src[6], uint8_t type,
                        uint16_t seq, uint8_t result, const char *msg)
{
    bwp_writer_t w;
    bwp_begin(&w, buf, cap, dst, src, type | BWP_RESPONSE, seq);
    bwp_put_u8(&w, BWP_T_RESULT, result);
    if (msg && msg[0]) {
        bwp_put_str(&w, BWP_T_MESSAGE, msg);
    }
    return bwp_finish(&w);
}

bool bwp_ssid_valid(const uint8_t *ssid, size_t len)
{
    (void)ssid;
    return len >= 1 && len <= 32;
}

static bool is_hex(char c)
{
    return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
}

bool bwp_passphrase_valid(const char *pass, size_t len)
{
    if (len == 0) {
        return true;
    }
    if (len == 64) {
        for (size_t i = 0; i < len; i++) {
            if (!is_hex(pass[i])) {
                return false;
            }
        }
        return true;
    }
    if (len < 8 || len > 63) {
        return false;
    }
    for (size_t i = 0; i < len; i++) {
        if (pass[i] < 0x20 || pass[i] > 0x7e) {
            return false;
        }
    }
    return true;
}

bwp_result_t bwp_parse_set_wifi(const bwp_msg_t *m, bwp_set_wifi_t *out)
{
    const uint8_t *v;
    uint16_t n;
    memset(out, 0, sizeof(*out));
    if (!bwp_find(m->body, m->body_len, BWP_T_SSID, &v, &n) || !bwp_ssid_valid(v, n)) {
        return BWP_E_BAD_REQUEST;
    }
    memcpy(out->ssid, v, n);
    out->ssid_len = n;
    if (bwp_find(m->body, m->body_len, BWP_T_PASSPHRASE, &v, &n)) {
        if (n > 64 || !bwp_passphrase_valid((const char *)v, n)) {
            return BWP_E_BAD_REQUEST;
        }
        memcpy(out->passphrase, v, n);
    }
    return BWP_OK;
}
