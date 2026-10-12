/*
 * usb_dhcpd.c — the DHCP server for the USB network link. Pure: no lwIP, so the
 * host tests run it (test/test_usb_dhcpd.c); usb_netif.c feeds it the datagrams.
 */
#include "usb_dhcpd.h"

#include <string.h>

/* RFC 2131 fixed header offsets */
#define OFF_OP      0
#define OFF_HTYPE   1
#define OFF_HLEN    2
#define OFF_XID     4
#define OFF_FLAGS   10
#define OFF_CIADDR  12
#define OFF_YIADDR  16
#define OFF_SIADDR  20
#define OFF_CHADDR  28
#define OFF_COOKIE  236
#define OFF_OPTIONS 240

#define OPT_PAD        0
#define OPT_MASK       1
#define OPT_REQ_IP     50
#define OPT_LEASE      51
#define OPT_MSG_TYPE   53
#define OPT_SERVER_ID  54
#define OPT_END        255

#define DISCOVER 1
#define OFFER    2
#define REQUEST  3
#define ACK      5
#define NAK      6

static const uint8_t k_cookie[4] = { 99, 130, 83, 99 };

static void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);  p[3] = (uint8_t)v;
}

static uint32_t get_u32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}

/* Finds option `code` in the options area; returns a pointer to its length byte. */
static const uint8_t *find_opt(const uint8_t *req, size_t len, uint8_t code)
{
    size_t i = OFF_OPTIONS;
    while (i < len) {
        uint8_t c = req[i];
        if (c == OPT_END) return NULL;
        if (c == OPT_PAD) { i++; continue; }
        if (i + 1 >= len) return NULL;
        uint8_t l = req[i + 1];
        if (i + 2 + l > len) return NULL;
        if (c == code) return &req[i + 1];
        i += 2u + l;
    }
    return NULL;
}

static uint8_t *opt_u32(uint8_t *p, uint8_t code, uint32_t v)
{
    *p++ = code; *p++ = 4; put_u32(p, v);
    return p + 4;
}

size_t usb_dhcpd_reply(const usb_dhcpd_cfg_t *cfg, const uint8_t *req, size_t req_len,
                       uint8_t *out, size_t out_cap)
{
    if (req_len < OFF_OPTIONS || out_cap < USB_DHCPD_MAX_MSG) return 0;
    if (req[OFF_OP] != 1 || req[OFF_HTYPE] != 1 || req[OFF_HLEN] != 6) return 0;
    if (memcmp(&req[OFF_COOKIE], k_cookie, 4) != 0) return 0;

    const uint8_t *mt = find_opt(req, req_len, OPT_MSG_TYPE);
    if (!mt || mt[0] != 1) return 0;

    uint8_t type;
    if (mt[1] == DISCOVER) {
        type = OFFER;
    } else if (mt[1] == REQUEST) {
        /* The address asked for: option 50 (SELECTING / INIT-REBOOT) or ciaddr (RENEWING). */
        const uint8_t *ri = find_opt(req, req_len, OPT_REQ_IP);
        uint32_t want = (ri && ri[0] == 4) ? get_u32(&ri[1]) : get_u32(&req[OFF_CIADDR]);
        /* A REQUEST that names another server's offer is not ours to answer. */
        const uint8_t *sid = find_opt(req, req_len, OPT_SERVER_ID);
        if (sid && sid[0] == 4 && get_u32(&sid[1]) != cfg->server_ip) return 0;
        type = (want == cfg->client_ip) ? ACK : NAK;
    } else {
        return 0;   /* RELEASE, DECLINE, INFORM: nothing to do on a one-host link */
    }

    memset(out, 0, USB_DHCPD_MAX_MSG);
    out[OFF_OP] = 2;
    out[OFF_HTYPE] = 1;
    out[OFF_HLEN] = 6;
    memcpy(&out[OFF_XID], &req[OFF_XID], 4);
    memcpy(&out[OFF_FLAGS], &req[OFF_FLAGS], 2);
    memcpy(&out[OFF_CHADDR], &req[OFF_CHADDR], 16);
    memcpy(&out[OFF_COOKIE], k_cookie, 4);

    uint8_t *p = &out[OFF_OPTIONS];
    *p++ = OPT_MSG_TYPE; *p++ = 1; *p++ = type;
    p = opt_u32(p, OPT_SERVER_ID, cfg->server_ip);
    if (type != NAK) {
        put_u32(&out[OFF_YIADDR], cfg->client_ip);
        put_u32(&out[OFF_SIADDR], cfg->server_ip);
        p = opt_u32(p, OPT_LEASE, cfg->lease_s);
        p = opt_u32(p, OPT_MASK, cfg->netmask);
    }
    *p++ = OPT_END;
    return USB_DHCPD_MAX_MSG;
}

void usb_dhcpd_subnet(const uint32_t uid[3], uint32_t *pod, uint32_t *host, uint32_t *mask)
{
    /* FNV-1a over the UID words, folded to 14 bits: X (8 bits) and a /30 index (6 bits). */
    uint32_t h = 2166136261u;
    for (int w = 0; w < 3; w++)
        for (int b = 0; b < 4; b++) { h ^= (uid[w] >> (8 * b)) & 0xFF; h *= 16777619u; }
    h ^= h >> 14;
    uint32_t base = (10u << 24) | (237u << 16) | (((h >> 6) & 0xFF) << 8) | ((h & 0x3F) << 2);
    *pod  = base + 1;
    *host = base + 2;
    *mask = 0xFFFFFFFCu;
}
