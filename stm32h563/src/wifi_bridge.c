/*
 * wifi_bridge.c — Wi-Fi bridge control-channel client (see wifi_bridge.h and
 * wifi-bridge/docs/PROTOCOL.md).
 */
#include "wifi_bridge.h"

#include <string.h>

#include "hmac_sha256.h"

#define BE32(p) (((uint32_t)(p)[0] << 24) | ((uint32_t)(p)[1] << 16) | ((uint32_t)(p)[2] << 8) | (p)[3])

static bool elapsed(uint32_t now, uint32_t since, uint32_t ms)
{
    return (uint32_t)(now - since) >= ms;
}

static void set_msg(wb_reply_t *r, const char *s)
{
    strncpy(r->message, s, sizeof(r->message) - 1);
    r->message[sizeof(r->message) - 1] = '\0';
}

void wb_init(wb_t *b, const uint8_t pod_mac[6], const wb_io_t *io)
{
    memset(b, 0, sizeof(*b));
    memcpy(b->pod_mac, pod_mac, 6);
    b->io = *io;
}

static bool present(const wb_t *b, uint32_t now)
{
    return b->seen && b->link_up && !elapsed(now, b->last_hello, WB_HELLO_TIMEOUT_MS);
}

static bool bridge_paired(const wb_t *b)
{
    return (b->info.flags & BWP_F_PAIRED) != 0;
}

static void finish(wb_t *b, wb_req_state_t st)
{
    b->req_state = b->req_internal ? WB_REQ_IDLE : st;  /* our own requests leave no reply behind */
}

/* Header || body || [COUNTER, AUTH], sent in parts so a large body is never copied. The AUTH is
   the one bwp_sign makes: HMAC-SHA256(key, nonce || header from the magic || body || COUNTER). */
static void send_req(wb_t *b, uint32_t now)
{
    const uint8_t *body = b->req_ext ? b->req_ext : b->req_tlvs;
    uint8_t head[BWP_ETH_HDR_LEN + BWP_HDR_LEN], tail[WB_TRAILER_LEN];
    size_t tl = 0;
    bwp_writer_t w;
    bwp_begin(&w, head, sizeof(head), b->mac, b->pod_mac, b->req_type, b->req_seq);
    uint16_t total = (uint16_t)(b->req_len + (b->req_signed ? WB_TRAILER_LEN : 0));
    head[20] = (uint8_t)(total >> 8);
    head[21] = (uint8_t)total;
    if (b->req_signed) {
        b->req_counter = ++b->counter;
        uint32_t c = b->req_counter;
        tail[0] = BWP_T_COUNTER, tail[1] = 0, tail[2] = 4;
        tail[3] = (uint8_t)(c >> 24), tail[4] = (uint8_t)(c >> 16), tail[5] = (uint8_t)(c >> 8), tail[6] = (uint8_t)c;
        uint8_t mac[32];
        hmac_sha256_ctx_t h;
        hmac_sha256_init(&h, b->key, BWP_KEY_LEN);
        hmac_sha256_update(&h, b->nonce, BWP_NONCE_LEN);
        hmac_sha256_update(&h, head + BWP_ETH_HDR_LEN, BWP_HDR_LEN);
        hmac_sha256_update(&h, body, b->req_len);
        hmac_sha256_update(&h, tail, 7);
        hmac_sha256_final(&h, mac);
        tail[7] = BWP_T_AUTH, tail[8] = 0, tail[9] = BWP_AUTH_LEN;
        memcpy(tail + 10, mac, BWP_AUTH_LEN);
        tl = WB_TRAILER_LEN;
    }
    b->req_sent = now;
    b->req_tries++;
    b->io.send(head, sizeof(head), body, b->req_len, tail, tl, b->io.ctx);
}

static void start_req(wb_t *b, uint8_t type, const uint8_t *tlvs, size_t len, uint32_t timeout_ms, bool internal,
                      const uint8_t *borrowed, uint32_t now)
{
    b->req_state = WB_REQ_PENDING;
    b->req_internal = internal;
    b->req_type = type;
    if (++b->seq == 0) {
        b->seq = 1;  /* seq 0 is for unsolicited messages */
    }
    b->req_seq = b->seq;
    b->req_ext = borrowed;
    if (!borrowed && len) {
        memcpy(b->req_tlvs, tlvs, len);
    }
    b->req_len = (uint16_t)len;
    b->req_timeout = timeout_ms;
    b->req_tries = 0;
    b->req_resynced = false;
    /* PAIR goes unsigned to an unpaired bridge; everything else is signed once it is paired. */
    b->req_signed = bridge_paired(b);
    memset(&b->reply, 0, sizeof(b->reply));
    send_req(b, now);
}

static void forget_bridge(wb_t *b)
{
    if (b->req_state == WB_REQ_PENDING) {
        set_msg(&b->reply, "the bridge went away");
        finish(b, WB_REQ_TIMEOUT);
    }
    b->seen = false;
    b->have_nonce = false;
    b->have_key = false;
    b->have_secret = false;
    b->counter = 0;
    memset(b->key, 0, sizeof(b->key));
    memset(b->secret, 0, sizeof(b->secret));
    memset(&b->info, 0, sizeof(b->info));
}

void wb_set_link(wb_t *b, bool up, uint32_t now)
{
    (void)now;
    b->link_up = up;
}

static void copy_str(const bwp_msg_t *m, uint8_t tag, char *out, size_t cap)
{
    if (!bwp_get_str(m->body, m->body_len, tag, out, cap)) {
        out[0] = '\0';
    }
}

/* The pairing key from the bridge's public key (from its PAIR answer or HELLO). If the bridge
   paired with another pod, this key is simply wrong and its requests get AUTH refusals. */
static void learn_bridge_pub(wb_t *b, const uint8_t pub[BWP_PUBKEY_LEN])
{
    if (!b->have_secret || (b->have_key && memcmp(pub, b->bridge_pub, BWP_PUBKEY_LEN) == 0)) {
        return;
    }
    memcpy(b->bridge_pub, pub, BWP_PUBKEY_LEN);
    b->have_key = bwp_pair_key(b->secret, pub, b->pub, pub, b->key) == 0;
}

/* The state from a HELLO or an unsolicited STATUS: on a change, fetch the signed STATUS (SSID,
   rssi) now instead of at the next poll. */
static void update_state(wb_t *b, const uint8_t *body, uint16_t len, uint32_t now)
{
    uint8_t st = b->info.state;
    if (bwp_get_u8(body, len, BWP_T_STATE, &st) && st != b->info.state) {
        b->info.state = st;
        b->next_status = now;
    }
}

static void on_hello(wb_t *b, const bwp_msg_t *m, uint32_t now)
{
    if (!b->seen || memcmp(m->src, b->mac, 6) != 0) {
        forget_bridge(b);  /* a different bridge: new key, new counter */
        memcpy(b->mac, m->src, 6);
        b->have_secret = b->io.derive_secret(b->mac, b->secret, b->io.ctx) == 0;
        if (b->have_secret) {
            bwp_x25519_public(b->secret, b->pub);
        }
        b->info.problem = b->have_secret ? NULL : "no device identity on this pod: cannot pair";
        b->next_pair = now;
        b->next_status = now;
    }
    b->seen = true;
    b->last_hello = now;
    const uint8_t *v;
    uint16_t n;
    if (bwp_find(m->body, m->body_len, BWP_T_NONCE, &v, &n) && n == BWP_NONCE_LEN &&
        (!b->have_nonce || memcmp(v, b->nonce, BWP_NONCE_LEN) != 0)) {
        memcpy(b->nonce, v, BWP_NONCE_LEN);  /* the bridge rebooted: its counter starts over */
        b->have_nonce = true;
        b->counter = 0;
        b->next_status = now;
    }
    update_state(b, m->body, m->body_len, now);
    bwp_get_u8(m->body, m->body_len, BWP_T_FLAGS, &b->info.flags);
    b->info.hello_flags = b->info.flags;
    copy_str(m, BWP_T_FW_VERSION, b->info.fw_version, sizeof(b->info.fw_version));
    copy_str(m, BWP_T_HW_REV, b->info.hw_rev, sizeof(b->info.hw_rev));
    if (!bridge_paired(b)) {
        b->info.paired = false;  /* PAIR pressed, factory reset or a new bridge: we pair again */
        b->have_key = false;
    } else if (bwp_find(m->body, m->body_len, BWP_T_PUBKEY, &v, &n) && n == BWP_PUBKEY_LEN) {
        learn_bridge_pub(b, v);  /* after our reboot, or the bridge was paired by another pod */
    }
}

static void parse_status(wb_t *b, const bwp_msg_t *m, uint32_t now)
{
    wb_info_t *i = &b->info;
    uint8_t v8;
    uint32_t v32;
    const uint8_t *v;
    uint16_t n;
    bwp_get_u8(m->body, m->body_len, BWP_T_STATE, &i->state);
    i->status_state = i->state;
    bwp_get_u8(m->body, m->body_len, BWP_T_FLAGS, &i->flags);
    copy_str(m, BWP_T_SSID, i->ssid, sizeof(i->ssid));
    i->channel = bwp_get_u8(m->body, m->body_len, BWP_T_CHANNEL, &v8) ? v8 : 0;
    i->rssi = bwp_get_u8(m->body, m->body_len, BWP_T_RSSI, &v8) ? (int8_t)v8 : 0;
    i->reason = bwp_get_u8(m->body, m->body_len, BWP_T_REASON, &v8) ? v8 : 0;
    i->uptime_s = bwp_get_u32(m->body, m->body_len, BWP_T_UPTIME, &v32) ? v32 : 0;
    if (bwp_find(m->body, m->body_len, BWP_T_COUNTERS, &v, &n) && n == 28) {
        uint32_t *c[7] = {&i->counters.eth_rx, &i->counters.wifi_tx, &i->counters.wifi_tx_err, &i->counters.wifi_rx,
                          &i->counters.eth_tx, &i->counters.eth_tx_err, &i->counters.dropped};
        for (int k = 0; k < 7; k++) {
            *c[k] = BE32(v + 4 * k);
        }
    }
    i->have_status = true;
    b->status_at = now;
}

static void parse_scan(wb_t *b, const bwp_msg_t *m)
{
    bwp_iter_t it;
    uint8_t tag;
    const uint8_t *v;
    uint16_t n;
    bwp_iter_init(&it, m->body, m->body_len);
    while (bwp_iter_next(&it, &tag, &v, &n) && b->reply.n_aps < WB_SCAN_MAX) {
        if (tag != BWP_T_AP || !bwp_tlv_valid(v, n)) {
            continue;
        }
        bwp_ap_t *ap = &b->reply.aps[b->reply.n_aps];
        uint8_t u;
        const uint8_t *bs;
        uint16_t bn;
        if (!bwp_get_str(v, n, BWP_T_SSID, ap->ssid, sizeof(ap->ssid))) {
            continue;
        }
        ap->rssi = bwp_get_u8(v, n, BWP_T_RSSI, &u) ? (int8_t)u : 0;
        ap->channel = bwp_get_u8(v, n, BWP_T_CHANNEL, &u) ? u : 0;
        ap->authmode = bwp_get_u8(v, n, BWP_T_AUTHMODE, &u) ? u : BWP_AUTH_OTHER;
        if (bwp_find(v, n, BWP_T_BSSID, &bs, &bn) && bn == 6) {
            memcpy(ap->bssid, bs, 6);
        }
        b->reply.n_aps++;
    }
}

static const char *const REJECTED = "the bridge is paired with another pod: press PAIR on the bridge";

static void on_reply(wb_t *b, const bwp_msg_t *m, const uint8_t *frame, uint32_t now)
{
    uint8_t r = BWP_E_INTERNAL;
    bwp_get_u8(m->body, m->body_len, BWP_T_RESULT, &r);
    bool refusal = r == BWP_E_AUTH || r == BWP_E_NOT_PAIRED || r == BWP_E_ALREADY_PAIRED;

    if (b->req_signed && !refusal) {
        /* The answer to a signed request must be signed with our key and that request's counter:
           anything else is not from our bridge. Ignore it and let the retry/timeout run. */
        uint32_t c;
        if (bwp_verify(frame, m, b->key, b->nonce, &c) != BWP_VERIFY_OK || c != b->req_counter) {
            return;
        }
        b->info.paired = true;
        b->info.problem = NULL;
    }

    const uint8_t *v;
    uint16_t n;
    uint32_t c;
    switch (r) {
    case BWP_E_AUTH:
        if (bwp_find(m->body, m->body_len, BWP_T_NONCE, &v, &n) && n == BWP_NONCE_LEN &&
            (!b->have_nonce || memcmp(v, b->nonce, BWP_NONCE_LEN) != 0)) {
            memcpy(b->nonce, v, BWP_NONCE_LEN);  /* we missed a reboot of the bridge */
            b->have_nonce = true;
            b->counter = 0;
            if (b->req_tries < WB_TRIES) {
                send_req(b, now);
                return;
            }
        } else if (!b->req_resynced && bwp_get_u32(m->body, m->body_len, BWP_T_COUNTER, &c)) {
            /* REPLAY: we rebooted and started counting at 1 again; carry on above the bridge's. */
            if (c >= b->counter) {
                b->counter = c;
            }
            b->req_resynced = true;
            send_req(b, now);
            return;
        } else {
            b->info.paired = false;
            b->info.problem = REJECTED;
        }
        break;
    case BWP_E_ALREADY_PAIRED:
        b->info.paired = false;
        b->info.problem = REJECTED;
        break;
    case BWP_E_NOT_PAIRED:
        b->info.flags &= (uint8_t)~BWP_F_PAIRED;  /* PAIR was pressed: pair again, now */
        b->info.paired = false;
        b->next_pair = now;
        break;
    case BWP_OK:
        if (b->req_type == BWP_PAIR) {
            if (!bwp_find(m->body, m->body_len, BWP_T_PUBKEY, &v, &n) || n != BWP_PUBKEY_LEN) {
                break;  /* no public key in the answer: try again later */
            }
            b->have_key = false;
            learn_bridge_pub(b, v);
            b->info.flags |= BWP_F_PAIRED;
            b->info.paired = b->have_key;
            b->info.problem = b->have_key ? NULL : "the bridge sent a bad public key";
            b->next_status = now;
        } else if (b->req_type == BWP_STATUS) {
            parse_status(b, m, now);
        } else if (b->req_type == BWP_SCAN) {
            parse_scan(b, m);
        } else if (b->req_type == BWP_SET_WIFI) {
            /* What we knew was about the previous network: nobody may read it as this join's. */
            b->info.have_status = false;
            b->info.state = b->info.status_state = BWP_ST_JOINING;
            b->next_status = now;
        }
        break;
    default:
        break;
    }
    if (b->req_type == BWP_OTA_DATA) {
        bwp_get_u32(m->body, m->body_len, BWP_T_OFFSET, &b->reply.offset);
    }
    b->reply.result = r;
    if (!bwp_get_str(m->body, m->body_len, BWP_T_MESSAGE, b->reply.message, sizeof(b->reply.message))) {
        b->reply.message[0] = '\0';
    }
    if (r == BWP_E_AUTH || r == BWP_E_ALREADY_PAIRED) {
        set_msg(&b->reply, REJECTED);
    } else if (r == BWP_E_NOT_PAIRED) {
        set_msg(&b->reply, "the bridge is not paired yet; pairing now, try again");
    }
    finish(b, WB_REQ_DONE);
}

void wb_rx(wb_t *b, const uint8_t *frame, size_t len, uint32_t now)
{
    bwp_msg_t m;
    if (bwp_parse(frame, len, &m) != BWP_PARSE_OK) {
        return;
    }
    if (memcmp(m.dst, b->pod_mac, 6) != 0 && !(m.dst[0] & 0x01)) {
        return;  /* for someone else */
    }
    if (m.type == BWP_HELLO) {
        on_hello(b, &m, now);
        return;
    }
    if (!(m.type & BWP_RESPONSE) || !b->seen || memcmp(m.src, b->mac, 6) != 0) {
        return;  /* only answers, and only from our bridge */
    }
    if (m.seq == 0) {
        /* Unsolicited STATUS on a state change: unsigned, so it only refreshes the state. */
        if (m.type == (BWP_STATUS | BWP_RESPONSE)) {
            update_state(b, m.body, m.body_len, now);
            bwp_get_u8(m.body, m.body_len, BWP_T_FLAGS, &b->info.flags);
        }
        return;
    }
    if (b->req_state == WB_REQ_PENDING && m.seq == b->req_seq && m.type == (b->req_type | BWP_RESPONSE)) {
        on_reply(b, &m, frame, now);
    }
}

void wb_poll(wb_t *b, uint32_t now)
{
    bool here = present(b, now);
    if (b->req_state == WB_REQ_PENDING) {
        if (!here) {
            set_msg(&b->reply, "no Wi-Fi bridge on the Ethernet port");
            finish(b, WB_REQ_TIMEOUT);
        } else if (elapsed(now, b->req_sent, b->req_timeout)) {
            if (b->req_tries < WB_TRIES) {
                send_req(b, now);
            } else {
                set_msg(&b->reply, "no answer from the Wi-Fi bridge");
                finish(b, WB_REQ_TIMEOUT);
            }
        }
        return;
    }
    if (b->req_state != WB_REQ_IDLE || !here || !b->have_secret || !b->have_nonce) {
        return;
    }
    if (!bridge_paired(b)) {
        if ((int32_t)(now - b->next_pair) >= 0) {
            uint8_t t[3 + BWP_PUBKEY_LEN] = {BWP_T_PUBKEY, 0, BWP_PUBKEY_LEN};
            memcpy(t + 3, b->pub, BWP_PUBKEY_LEN);  /* public: the key itself is never sent */
            b->next_pair = now + WB_PAIR_RETRY_MS;
            start_req(b, BWP_PAIR, t, sizeof(t), 1000, true, NULL, now);
        }
    } else if (b->have_key && (int32_t)(now - b->next_status) >= 0) {
        b->next_status = now + WB_STATUS_EVERY_MS;
        start_req(b, BWP_STATUS, NULL, 0, 1000, true, NULL, now);
    }
}

static int request(wb_t *b, uint8_t type, const uint8_t *tlvs, size_t len, uint32_t timeout_ms, bool borrow,
                   uint32_t now);

int wb_request(wb_t *b, uint8_t type, const uint8_t *tlvs, size_t len, uint32_t timeout_ms, uint32_t now)
{
    return len > WB_REQ_MAX ? -3 : request(b, type, tlvs, len, timeout_ms, false, now);
}

int wb_request_ext(wb_t *b, uint8_t type, const uint8_t *tlvs, size_t len, uint32_t timeout_ms, uint32_t now)
{
    return len > WB_BODY_MAX ? -3 : request(b, type, tlvs, len, timeout_ms, true, now);
}

static int request(wb_t *b, uint8_t type, const uint8_t *tlvs, size_t len, uint32_t timeout_ms, bool borrow,
                   uint32_t now)
{
    if (!present(b, now)) {
        return -2;
    }
    if (!b->have_secret || (bridge_paired(b) && !b->have_key)) {
        return -3;  /* info.problem says why */
    }
    if (b->req_state != WB_REQ_IDLE) {
        return -1;
    }
    if (!bridge_paired(b) && type != BWP_STATUS && type != BWP_SCAN) {
        return -1;  /* auto-pair runs first (wb_poll); the caller retries */
    }
    start_req(b, type, tlvs, len, timeout_ms, false, borrow ? tlvs : NULL, now);
    return 0;
}

int wb_request_set_wifi(wb_t *b, const char *ssid, const char *pass, const uint8_t seal_nonce[BWP_SEAL_NONCE_LEN],
                        uint32_t timeout_ms, uint32_t now)
{
    size_t a = strlen(ssid), p = strlen(pass);
    if (!bwp_ssid_valid((const uint8_t *)ssid, a) || !bwp_passphrase_valid(pass, p)) {
        return -4;
    }
    if (!present(b, now)) {
        return -2;
    }
    if (!bridge_paired(b) || !b->have_key) {
        return b->have_secret && !bridge_paired(b) ? -1 : -3;  /* pairing first: sealing needs the key */
    }
    uint8_t t[WB_REQ_MAX];
    size_t n = 0;
    t[n++] = BWP_T_SSID, t[n++] = 0, t[n++] = (uint8_t)a;
    memcpy(t + n, ssid, a), n += a;
    if (p) {
        t[n++] = BWP_T_ENC_PASSPHRASE, t[n++] = 0, t[n++] = BWP_SEALED_PASS_LEN;
        bwp_seal_passphrase(b->key, seal_nonce, pass, t + n);
        n += BWP_SEALED_PASS_LEN;
    }
    int r = wb_request(b, BWP_SET_WIFI, t, n, timeout_ms, now);
    memset(t, 0, sizeof(t));
    return r;
}

wb_req_state_t wb_request_state(const wb_t *b)
{
    return b->req_internal ? WB_REQ_IDLE : b->req_state;
}

const wb_reply_t *wb_request_reply(const wb_t *b)
{
    return &b->reply;
}

void wb_request_release(wb_t *b)
{
    if (!b->req_internal && b->req_state != WB_REQ_PENDING) {
        b->req_state = WB_REQ_IDLE;
    }
}

void wb_get_info(const wb_t *b, uint32_t now, wb_info_t *out)
{
    *out = b->info;
    out->present = present(b, now);
    memcpy(out->mac, b->mac, 6);
    out->status_age_ms = out->have_status ? now - b->status_at : 0;
}

const char *wb_state_str(uint8_t state)
{
    static const char *const names[] = {"unconfigured", "wait-pod", "joining", "connected", "auth-failed", "no-ap"};
    return state < sizeof(names) / sizeof(names[0]) ? names[state] : "unknown";
}

const char *wb_result_str(uint8_t r)
{
    static const char *const names[] = {"ok", "bad-request", "unsupported", "busy", "storage", "ota-not-started",
                                        "ota-bad-offset", "ota-verify", "ota-too-large", "internal",
                                        "ota-wrong-image", "not-paired", "auth", "already-paired"};
    return r < sizeof(names) / sizeof(names[0]) ? names[r] : "error";
}
