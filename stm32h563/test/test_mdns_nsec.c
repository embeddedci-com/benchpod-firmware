/*
 * test_mdns_nsec.c — NSEC answers to AAAA questions (src/mdns_nsec.c, src/mdns_nsec_lwip.c).
 *
 * Part 1 tests the packet builder on its own. Part 2 runs lwIP's real mDNS responder (NO_SYS,
 * IPv4 + UDP + IGMP, test/shim_mdns/lwipopts.h) with the hook installed, on a fake netif: the
 * test feeds IPv4/UDP query packets to ip4_input() and parses what the stack sends back.
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "mdns_nsec.h"
#include "mdns_nsec_lwip.h"

#include "lwip/init.h"
#include "lwip/ip4.h"
#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "lwip/timeouts.h"
#include "lwip/apps/mdns.h"
#include "lwip/apps/mdns_priv.h"

static int g_fail;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); g_fail++; } } while (0)

#define HOST "benchpod-a1b2c3"
#define T_A    1
#define T_AAAA 28
#define T_NSEC 47
#define T_ANY  255

/* ---- DNS helpers -------------------------------------------------------- */

static size_t put_name(uint8_t *p, const char *a, const char *b)
{
    size_t n = 0, la = strlen(a), lb = strlen(b);
    p[n++] = (uint8_t)la; memcpy(p + n, a, la); n += la;
    p[n++] = (uint8_t)lb; memcpy(p + n, b, lb); n += lb;
    p[n++] = 0;
    return n;
}

/* A query with the given questions, all for name a.b. */
static size_t build_query(uint8_t *out, uint16_t id, const char *a, const char *b,
                          const uint16_t *types, int nq, uint16_t qclass)
{
    size_t n = 0;
    memset(out, 0, 12);
    out[0] = (uint8_t)(id >> 8); out[1] = (uint8_t)id;
    out[5] = (uint8_t)nq;
    n = 12;
    for (int i = 0; i < nq; i++) {
        n += put_name(out + n, a, b);
        out[n++] = (uint8_t)(types[i] >> 8); out[n++] = (uint8_t)types[i];
        out[n++] = (uint8_t)(qclass >> 8);   out[n++] = (uint8_t)qclass;
    }
    return n;
}

static uint16_t rd16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }

/* Skip a name (pointers allowed); returns the offset after it, 0 if malformed. Writes the
   dotted lowercase form to txt. */
static size_t skip_name(const uint8_t *m, size_t len, size_t off, char *txt, size_t tcap)
{
    size_t end = 0, t = 0;
    int hops = 0;
    for (;;) {
        if (off >= len) return 0;
        uint8_t n = m[off];
        if ((n & 0xC0) == 0xC0) {
            if (!end) end = off + 2;
            off = ((size_t)(n & 0x3F) << 8) | m[off + 1];
            if (++hops > 16) return 0;
            continue;
        }
        if (n == 0) { off++; break; }
        if (off + 1 + n > len) return 0;
        if (t && t < tcap - 1) txt[t++] = '.';
        for (size_t i = 0; i < n && t < tcap - 1; i++) {
            char c = (char)m[off + 1 + i];
            txt[t++] = (c >= 'A' && c <= 'Z') ? (char)(c + 32) : c;
        }
        off += 1u + n;
    }
    txt[t] = 0;
    return end ? end : off;
}

typedef struct {
    bool     ok;
    uint16_t id, flags, qd, an;
    int      n_a, n_aaaa, n_nsec;          /* records (any section) for HOST.local */
    uint32_t a_addr;                       /* host order */
    uint32_t nsec_ttl;
    uint16_t nsec_class;
    char     nsec_next[80];
    uint8_t  nsec_bitmap[8];
    size_t   nsec_bitmap_len;
} reply_t;

static reply_t parse_reply(const uint8_t *m, size_t len)
{
    reply_t r;
    char name[80];
    memset(&r, 0, sizeof(r));
    if (len < 12) return r;
    r.id = rd16(m); r.flags = rd16(m + 2); r.qd = rd16(m + 4); r.an = rd16(m + 6);
    int rrs = r.an + rd16(m + 8) + rd16(m + 10);
    size_t off = 12;
    for (int i = 0; i < r.qd; i++) {
        off = skip_name(m, len, off, name, sizeof(name));
        if (!off || off + 4 > len) return r;
        off += 4;
    }
    for (int i = 0; i < rrs; i++) {
        off = skip_name(m, len, off, name, sizeof(name));
        if (!off || off + 10 > len) return r;
        uint16_t type = rd16(m + off), cls = rd16(m + off + 2);
        uint32_t ttl = ((uint32_t)rd16(m + off + 4) << 16) | rd16(m + off + 6);
        uint16_t rdlen = rd16(m + off + 8);
        size_t rd = off + 10;
        if (rd + rdlen > len) return r;
        if (strcmp(name, HOST ".local") == 0) {
            if (type == T_A) {
                r.n_a++;
                if (rdlen == 4) r.a_addr = ((uint32_t)rd16(m + rd) << 16) | rd16(m + rd + 2);
            } else if (type == T_AAAA) {
                r.n_aaaa++;
            } else if (type == T_NSEC) {
                r.n_nsec++;
                r.nsec_ttl = ttl; r.nsec_class = cls;
                size_t b = skip_name(m, rd + rdlen, rd, r.nsec_next, sizeof(r.nsec_next));
                if (b && rd + rdlen - b <= sizeof(r.nsec_bitmap)) {
                    r.nsec_bitmap_len = rd + rdlen - b;
                    memcpy(r.nsec_bitmap, m + b, r.nsec_bitmap_len);
                }
            }
        }
        off = rd + rdlen;
    }
    r.ok = off == len;
    return r;
}

/* NSEC bitmap says "A only": window 0, length 1, bit for type 1. */
static bool bitmap_is_a_only(const reply_t *r)
{
    return r->nsec_bitmap_len == 3 && r->nsec_bitmap[0] == 0 && r->nsec_bitmap[1] == 1 &&
           r->nsec_bitmap[2] == 0x40;
}

/* ---- Part 1: the builder ------------------------------------------------ */

static void test_builder(void)
{
    uint8_t q[512], out[MDNS_NSEC_REPLY_MAX];
    size_t qn, n;
    reply_t r;

    /* AAAA for our name: one NSEC answer, A in the bitmap, no AAAA, multicast form. */
    qn = build_query(q, 0, HOST, "local", (uint16_t[]){T_AAAA}, 1, 1);
    n = mdns_nsec_reply(q, qn, HOST, false, out, sizeof(out));
    CHECK(n > 0);
    r = parse_reply(out, n);
    CHECK(r.ok);
    CHECK(r.id == 0 && r.flags == 0x8400 && r.qd == 0 && r.an == 1);
    CHECK(r.n_nsec == 1 && r.n_aaaa == 0 && r.n_a == 0);
    CHECK(r.nsec_class == 0x8001 && r.nsec_ttl == MDNS_NSEC_TTL);
    CHECK(strcmp(r.nsec_next, HOST ".local") == 0);
    CHECK(bitmap_is_a_only(&r));
    CHECK(n == 41 + 2 * strlen(HOST));

    /* Legacy unicast: id and question echoed, short TTL, no cache-flush bit. */
    qn = build_query(q, 0x1234, HOST, "local", (uint16_t[]){T_AAAA}, 1, 1);
    n = mdns_nsec_reply(q, qn, HOST, true, out, sizeof(out));
    r = parse_reply(out, n);
    CHECK(r.ok && r.id == 0x1234 && r.qd == 1 && r.an == 1 && r.n_nsec == 1);
    CHECK(r.nsec_class == 1 && r.nsec_ttl == MDNS_NSEC_LEGACY_TTL);
    CHECK(n == 53 + 3 * strlen(HOST));

    /* Case-insensitive name, QU bit, A + AAAA in one query. */
    qn = build_query(q, 0, "BenchPod-A1B2C3", "LOCAL", (uint16_t[]){T_A, T_AAAA}, 2, 0x8001);
    CHECK(mdns_nsec_reply(q, qn, HOST, false, out, sizeof(out)) > 0);

    /* No reply: A only, ANY, another name, a response, a non-zero opcode. */
    qn = build_query(q, 0, HOST, "local", (uint16_t[]){T_A}, 1, 1);
    CHECK(mdns_nsec_reply(q, qn, HOST, false, out, sizeof(out)) == 0);
    qn = build_query(q, 0, HOST, "local", (uint16_t[]){T_ANY}, 1, 1);
    CHECK(mdns_nsec_reply(q, qn, HOST, false, out, sizeof(out)) == 0);
    qn = build_query(q, 0, "benchpod-ffffff", "local", (uint16_t[]){T_AAAA}, 1, 1);
    CHECK(mdns_nsec_reply(q, qn, HOST, false, out, sizeof(out)) == 0);
    qn = build_query(q, 0, HOST, "lan", (uint16_t[]){T_AAAA}, 1, 1);
    CHECK(mdns_nsec_reply(q, qn, HOST, false, out, sizeof(out)) == 0);
    qn = build_query(q, 0, HOST, "local", (uint16_t[]){T_AAAA}, 1, 1);
    q[2] = 0x84;
    CHECK(mdns_nsec_reply(q, qn, HOST, false, out, sizeof(out)) == 0);
    q[2] = 0x28;                                     /* opcode 5 */
    CHECK(mdns_nsec_reply(q, qn, HOST, false, out, sizeof(out)) == 0);

    /* Second question compressed to the first name (as mDNSResponder does). */
    qn = build_query(q, 0, HOST, "local", (uint16_t[]){T_A}, 1, 1);
    q[5] = 2;
    q[qn++] = 0xC0; q[qn++] = 12;
    q[qn++] = 0; q[qn++] = T_AAAA; q[qn++] = 0; q[qn++] = 1;
    n = mdns_nsec_reply(q, qn, HOST, false, out, sizeof(out));
    r = parse_reply(out, n);
    CHECK(n > 0 && r.ok && r.n_nsec == 1);

    /* Malformed: truncated anywhere, pointer loop, cap too small: never a reply, never a crash. */
    qn = build_query(q, 0, HOST, "local", (uint16_t[]){T_AAAA}, 1, 1);
    for (size_t cut = 0; cut < qn; cut++)
        CHECK(mdns_nsec_reply(q, cut, HOST, false, out, sizeof(out)) == 0);
    CHECK(mdns_nsec_reply(q, qn, HOST, false, out, 41 + 2 * strlen(HOST) - 1) == 0);
    memset(q, 0, 12); q[5] = 1; q[12] = 0xC0; q[13] = 12; q[14] = 0; q[15] = T_AAAA; q[16] = 0; q[17] = 1;
    CHECK(mdns_nsec_reply(q, 18, HOST, false, out, sizeof(out)) == 0);
    q[12] = 0x80;                                    /* reserved label type */
    CHECK(mdns_nsec_reply(q, 18, HOST, false, out, sizeof(out)) == 0);
    CHECK(mdns_nsec_reply(q, 18, "", false, out, sizeof(out)) == 0);
}

/* ---- Part 2: lwIP's responder with the hook ------------------------------ */

static uint32_t g_now;
u32_t sys_now(void) { return g_now; }

typedef struct { uint8_t b[1600]; size_t len; } frame_t;
static frame_t g_tx[32];
static int     g_ntx;

static err_t fake_output(struct netif *nif, struct pbuf *p, const ip4_addr_t *dst)
{
    (void)nif; (void)dst;
    if (g_ntx < (int)(sizeof(g_tx) / sizeof(g_tx[0]))) {
        g_tx[g_ntx].len = pbuf_copy_partial(p, g_tx[g_ntx].b, sizeof(g_tx[0].b), 0);
        g_ntx++;
    }
    return ERR_OK;
}

static err_t fake_init(struct netif *nif)
{
    nif->output = fake_output;
    nif->mtu = 1500;
    nif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_IGMP;
    nif->name[0] = 't'; nif->name[1] = 'e';
    return ERR_OK;
}

static void run_ms(uint32_t ms)
{
    for (uint32_t t = 0; t < ms; t += 10) { g_now += 10; sys_check_timeouts(); }
}

#define IP4(a, b, c, d) (((uint32_t)(a) << 24) | ((uint32_t)(b) << 16) | ((uint32_t)(c) << 8) | (d))
#define POD_IP   IP4(192, 168, 1, 50)
#define PEER_IP  IP4(192, 168, 1, 20)
#define GROUP_IP IP4(224, 0, 0, 251)

static void put32(uint8_t *p, uint32_t v) { p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = v; }

/* Feed an IPv4/UDP datagram to the stack as if it arrived on nif. */
static void inject(struct netif *nif, uint32_t src, uint16_t sport, uint32_t dst, uint16_t dport,
                   const uint8_t *payload, size_t plen)
{
    size_t tot = 20 + 8 + plen;
    struct pbuf *p = pbuf_alloc(PBUF_RAW, (u16_t)tot, PBUF_RAM);
    uint8_t *b = (uint8_t *)p->payload;
    memset(b, 0, 28);
    b[0] = 0x45; b[2] = (uint8_t)(tot >> 8); b[3] = (uint8_t)tot; b[8] = 255; b[9] = 17;
    put32(b + 12, src); put32(b + 16, dst);
    b[20] = sport >> 8; b[21] = (uint8_t)sport; b[22] = dport >> 8; b[23] = (uint8_t)dport;
    b[24] = (uint8_t)((8 + plen) >> 8); b[25] = (uint8_t)(8 + plen);
    memcpy(b + 28, payload, plen);
    ip4_input(p, nif);
}

typedef struct { uint32_t dst; uint16_t dport; reply_t r; } sent_t;

/* The mDNS (UDP from 5353) packets sent since the last call, parsed. */
static int collect(sent_t *s, int cap)
{
    int n = 0;
    for (int i = 0; i < g_ntx && n < cap; i++) {
        const uint8_t *b = g_tx[i].b;
        size_t ihl = (size_t)(b[0] & 0x0F) * 4;
        if (g_tx[i].len < ihl + 8 || b[9] != 17 || rd16(b + ihl) != 5353) continue;
        s[n].dst = ((uint32_t)rd16(b + 16) << 16) | rd16(b + 18);
        s[n].dport = rd16(b + ihl + 2);
        s[n].r = parse_reply(b + ihl + 8, g_tx[i].len - ihl - 8);
        n++;
    }
    g_ntx = 0;
    return n;
}

static void totals(const sent_t *s, int n, int *a, int *aaaa, int *nsec)
{
    *a = *aaaa = *nsec = 0;
    for (int i = 0; i < n; i++) { *a += s[i].r.n_a; *aaaa += s[i].r.n_aaaa; *nsec += s[i].r.n_nsec; }
}

static void test_lwip(void)
{
    static struct netif nif, other;
    ip4_addr_t ip, mask, gw;
    uint8_t q[256];
    size_t qn;
    sent_t s[16];
    int n, a, aaaa, nsec;

    lwip_init();
    mdns_resp_init();
    mdns_nsec_install();
    mdns_nsec_install();                        /* idempotent: no double wrap */

    IP4_ADDR(&ip, 192, 168, 1, 50); IP4_ADDR(&mask, 255, 255, 255, 0); IP4_ADDR(&gw, 192, 168, 1, 1);
    netif_add(&nif, &ip, &mask, &gw, NULL, fake_init, ip4_input);
    netif_set_up(&nif);
    netif_set_link_up(&nif);
    IP4_ADDR(&ip, 10, 0, 0, 2); IP4_ADDR(&mask, 255, 255, 255, 0); IP4_ADDR(&gw, 0, 0, 0, 0);
    netif_add(&other, &ip, &mask, &gw, NULL, fake_init, ip4_input);   /* no mDNS on it */
    netif_set_up(&other);
    netif_set_link_up(&other);

    CHECK(mdns_resp_add_netif(&nif, HOST) == ERR_OK);
    for (int i = 0; i < 600 && netif_mdns_data(&nif)->state != MDNS_STATE_COMPLETE; i++) run_ms(10);
    CHECK(netif_mdns_data(&nif)->state == MDNS_STATE_COMPLETE);
    run_ms(2000);                               /* past the per-record multicast rate limit */
    g_ntx = 0;

    /* AAAA query: one NSEC to the group, A in its bitmap, no AAAA anywhere. */
    qn = build_query(q, 0, HOST, "local", (uint16_t[]){T_AAAA}, 1, 1);
    inject(&nif, PEER_IP, 5353, GROUP_IP, 5353, q, qn);
    run_ms(1500);
    n = collect(s, 16);
    totals(s, n, &a, &aaaa, &nsec);
    CHECK(nsec == 1 && aaaa == 0);
    for (int i = 0; i < n; i++)
        if (s[i].r.n_nsec) {
            CHECK(s[i].r.ok && s[i].dst == GROUP_IP && s[i].dport == 5353);
            CHECK(bitmap_is_a_only(&s[i].r));
        }

    /* A query: lwIP still answers with the pod's address, and no NSEC is added. */
    run_ms(2000);
    qn = build_query(q, 0, HOST, "local", (uint16_t[]){T_A}, 1, 1);
    inject(&nif, PEER_IP, 5353, GROUP_IP, 5353, q, qn);
    run_ms(1500);
    n = collect(s, 16);
    totals(s, n, &a, &aaaa, &nsec);
    CHECK(a >= 1 && nsec == 0 && aaaa == 0);
    for (int i = 0; i < n; i++)
        if (s[i].r.n_a) CHECK(s[i].r.a_addr == POD_IP);

    /* A + AAAA in one query (what getaddrinfo AF_UNSPEC sends): the A and the NSEC. */
    run_ms(2000);
    qn = build_query(q, 0, HOST, "local", (uint16_t[]){T_A, T_AAAA}, 2, 1);
    inject(&nif, PEER_IP, 5353, GROUP_IP, 5353, q, qn);
    run_ms(1500);
    n = collect(s, 16);
    totals(s, n, &a, &aaaa, &nsec);
    CHECK(a >= 1 && nsec == 1 && aaaa == 0);

    /* Legacy unicast resolver (source port not 5353): unicast reply, id echoed. */
    qn = build_query(q, 0xBEEF, HOST, "local", (uint16_t[]){T_AAAA}, 1, 1);
    inject(&nif, PEER_IP, 40000, GROUP_IP, 5353, q, qn);
    run_ms(200);
    n = collect(s, 16);
    totals(s, n, &a, &aaaa, &nsec);
    CHECK(nsec == 1);
    for (int i = 0; i < n; i++)
        if (s[i].r.n_nsec) {
            CHECK(s[i].dst == PEER_IP && s[i].dport == 40000 && s[i].r.id == 0xBEEF);
            CHECK(s[i].r.nsec_ttl == MDNS_NSEC_LEGACY_TTL);
        }

    /* Unicast query from off the subnet: ignored, like lwIP does. */
    inject(&nif, IP4(10, 9, 9, 9), 5353, POD_IP, 5353, q, qn);
    run_ms(200);
    n = collect(s, 16);
    totals(s, n, &a, &aaaa, &nsec);
    CHECK(nsec == 0);

    /* A netif without mDNS, and one where it was removed (LAN policy off): no NSEC. */
    qn = build_query(q, 0, HOST, "local", (uint16_t[]){T_AAAA}, 1, 1);
    inject(&other, IP4(10, 0, 0, 9), 5353, GROUP_IP, 5353, q, qn);
    run_ms(200);
    n = collect(s, 16);
    totals(s, n, &a, &aaaa, &nsec);
    CHECK(nsec == 0);
    mdns_resp_remove_netif(&nif);
    g_ntx = 0;
    inject(&nif, PEER_IP, 5353, GROUP_IP, 5353, q, qn);
    run_ms(200);
    n = collect(s, 16);
    totals(s, n, &a, &aaaa, &nsec);
    CHECK(nsec == 0);
}

int main(void)
{
    test_builder();
    test_lwip();
    if (g_fail) { printf("test_mdns_nsec: %d FAILED\n", g_fail); return 1; }
    printf("test_mdns_nsec: all passed\n");
    return 0;
}
