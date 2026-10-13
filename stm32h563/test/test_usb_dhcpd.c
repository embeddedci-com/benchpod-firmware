/*
 * test_usb_dhcpd.c — host unit tests for the USB link's DHCP server
 * (src/usb_dhcpd.c). No lwIP / HAL. Run via test/Makefile (`make`).
 */
#include "usb_dhcpd.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); failures++; } \
} while (0)

#define IP(a, b, c, d) (((uint32_t)(a) << 24) | ((uint32_t)(b) << 16) | ((uint32_t)(c) << 8) | (uint32_t)(d))

static const usb_dhcpd_cfg_t cfg = {
    .server_ip = IP(10, 237, 5, 1), .client_ip = IP(10, 237, 5, 2),
    .netmask = 0xFFFFFFFCu, .lease_s = 3600,
};

static uint32_t get_u32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}

/* A client message: BOOTREQUEST, Ethernet, xid 0x11223344, chaddr 06:..., options. */
static size_t make_req(uint8_t *b, const uint8_t *opts, size_t opts_len)
{
    memset(b, 0, 600);
    b[0] = 1; b[1] = 1; b[2] = 6;
    b[4] = 0x11; b[5] = 0x22; b[6] = 0x33; b[7] = 0x44;
    b[10] = 0x80;   /* broadcast flag */
    static const uint8_t mac[6] = { 0x06, 1, 2, 3, 4, 5 };
    memcpy(&b[28], mac, 6);
    b[236] = 99; b[237] = 130; b[238] = 83; b[239] = 99;
    memcpy(&b[240], opts, opts_len);
    return 240 + opts_len;
}

/* The value of option `code` in a reply, or NULL. */
static const uint8_t *opt(const uint8_t *r, size_t len, uint8_t code)
{
    for (size_t i = 240; i + 1 < len && r[i] != 255; i += 2u + r[i + 1])
        if (r[i] == code) return &r[i + 2];
    return NULL;
}

static void test_discover_gets_offer_without_router_or_dns(void)
{
    uint8_t req[600], rep[USB_DHCPD_MAX_MSG];
    static const uint8_t o[] = { 53, 1, 1, 55, 3, 1, 3, 6, 255 };
    size_t n = usb_dhcpd_reply(&cfg, req, make_req(req, o, sizeof(o)), rep, sizeof(rep));
    CHECK(n == USB_DHCPD_MAX_MSG);
    CHECK(rep[0] == 2);
    CHECK(memcmp(&rep[4], &req[4], 4) == 0);            /* xid echoed */
    CHECK(memcmp(&rep[28], &req[28], 6) == 0);          /* chaddr echoed */
    CHECK(rep[10] == 0x80);                             /* flags echoed */
    CHECK(get_u32(&rep[16]) == cfg.client_ip);          /* yiaddr */
    const uint8_t *t = opt(rep, n, 53);
    CHECK(t && t[0] == 2);                              /* OFFER */
    const uint8_t *m = opt(rep, n, 1);
    CHECK(m && get_u32(m) == 0xFFFFFFFCu);
    const uint8_t *s = opt(rep, n, 54);
    CHECK(s && get_u32(s) == cfg.server_ip);
    CHECK(opt(rep, n, 3) == NULL);                      /* no router: host keeps its route */
    CHECK(opt(rep, n, 6) == NULL);                      /* no DNS */
}

static void test_request_for_our_address_is_acked(void)
{
    uint8_t req[600], rep[USB_DHCPD_MAX_MSG];
    static const uint8_t o[] = { 53, 1, 3, 50, 4, 10, 237, 5, 2, 54, 4, 10, 237, 5, 1, 255 };
    size_t n = usb_dhcpd_reply(&cfg, req, make_req(req, o, sizeof(o)), rep, sizeof(rep));
    const uint8_t *t = opt(rep, n, 53);
    CHECK(n && t && t[0] == 5);
    CHECK(get_u32(&rep[16]) == cfg.client_ip);
    const uint8_t *l = opt(rep, n, 51);
    CHECK(l && get_u32(l) == 3600);
}

static void test_renew_by_ciaddr_is_acked(void)
{
    uint8_t req[600], rep[USB_DHCPD_MAX_MSG];
    static const uint8_t o[] = { 53, 1, 3, 255 };
    size_t len = make_req(req, o, sizeof(o));
    req[12] = 10; req[13] = 237; req[14] = 5; req[15] = 2;
    size_t n = usb_dhcpd_reply(&cfg, req, len, rep, sizeof(rep));
    const uint8_t *t = opt(rep, n, 53);
    CHECK(n && t && t[0] == 5);
}

static void test_request_for_old_address_is_naked(void)
{
    /* A host that remembers another pod's lease (another /30) must move. */
    uint8_t req[600], rep[USB_DHCPD_MAX_MSG];
    static const uint8_t o[] = { 53, 1, 3, 50, 4, 10, 237, 9, 6, 255 };
    size_t n = usb_dhcpd_reply(&cfg, req, make_req(req, o, sizeof(o)), rep, sizeof(rep));
    const uint8_t *t = opt(rep, n, 53);
    CHECK(n && t && t[0] == 6);
    CHECK(get_u32(&rep[16]) == 0);
    CHECK(opt(rep, n, 1) == NULL);
}

static void test_request_to_other_server_is_ignored(void)
{
    uint8_t req[600], rep[USB_DHCPD_MAX_MSG];
    static const uint8_t o[] = { 53, 1, 3, 50, 4, 10, 237, 5, 2, 54, 4, 192, 168, 1, 1, 255 };
    CHECK(usb_dhcpd_reply(&cfg, req, make_req(req, o, sizeof(o)), rep, sizeof(rep)) == 0);
}

static void test_junk_is_ignored(void)
{
    uint8_t req[600], rep[USB_DHCPD_MAX_MSG];
    static const uint8_t disc[] = { 53, 1, 1, 255 };
    static const uint8_t release[] = { 53, 1, 7, 255 };
    static const uint8_t none[] = { 255 };
    size_t len = make_req(req, disc, sizeof(disc));
    CHECK(usb_dhcpd_reply(&cfg, req, 100, rep, sizeof(rep)) == 0);             /* short */
    CHECK(usb_dhcpd_reply(&cfg, req, len, rep, 10) == 0);                      /* no room */
    req[0] = 2;
    CHECK(usb_dhcpd_reply(&cfg, req, len, rep, sizeof(rep)) == 0);             /* a reply */
    req[0] = 1; req[236] = 0;
    CHECK(usb_dhcpd_reply(&cfg, req, len, rep, sizeof(rep)) == 0);             /* no cookie */
    CHECK(usb_dhcpd_reply(&cfg, req, make_req(req, release, sizeof(release)), rep, sizeof(rep)) == 0);
    CHECK(usb_dhcpd_reply(&cfg, req, make_req(req, none, sizeof(none)), rep, sizeof(rep)) == 0);
    /* An option length running past the end must not be read. */
    static const uint8_t bad[] = { 12, 200 };
    CHECK(usb_dhcpd_reply(&cfg, req, make_req(req, bad, sizeof(bad)), rep, sizeof(rep)) == 0);
}

static void test_subnet_from_uid(void)
{
    uint32_t a[3] = { 0x357D377F, 0x31331234, 0x00420042 }, b[3] = { 0x11111111, 0x2, 0x3 };
    uint32_t pod, host, mask, pod2, host2, mask2;
    usb_dhcpd_subnet(a, &pod, &host, &mask);
    CHECK((pod >> 16) == ((10u << 8) | 237u));
    CHECK((pod & 3u) == 1 && host == pod + 1 && mask == 0xFFFFFFFCu);
    usb_dhcpd_subnet(a, &pod2, &host2, &mask2);
    CHECK(pod2 == pod);                                 /* stable across boots */
    usb_dhcpd_subnet(b, &pod2, &host2, &mask2);
    CHECK(pod2 != pod);                                 /* another pod, another /30 */
}

int main(void)
{
    test_discover_gets_offer_without_router_or_dns();
    test_request_for_our_address_is_acked();
    test_renew_by_ciaddr_is_acked();
    test_request_for_old_address_is_naked();
    test_request_to_other_server_is_ignored();
    test_junk_is_ignored();
    test_subnet_from_uid();
    if (failures) { printf("test_usb_dhcpd: %d failure(s)\n", failures); return 1; }
    printf("test_usb_dhcpd: all passed\n");
    return 0;
}
