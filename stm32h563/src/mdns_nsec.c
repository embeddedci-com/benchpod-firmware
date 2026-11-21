/*
 * mdns_nsec.c — NSEC negative answer for AAAA questions (no lwIP dependency).
 * See mdns_nsec.h. Unit-tested by test/test_mdns_nsec.c.
 */
#include "mdns_nsec.h"

#include <string.h>

#define DNS_HDR_LEN     12u
#define DNS_FLAG_QR     0x8000u
#define DNS_FLAG_AA     0x0400u
#define DNS_OPCODE_MASK 0x7800u
#define DNS_TYPE_A      1u
#define DNS_TYPE_AAAA   28u
#define DNS_TYPE_NSEC   47u
#define DNS_CLASS_IN    1u
#define DNS_CLASS_ANY   255u
#define MDNS_CACHE_FLUSH 0x8000u   /* rrclass top bit in answers; the QU bit in questions */

static uint16_t rd16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }

static uint8_t *wr16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v; return p + 2; }

static uint8_t *wr32(uint8_t *p, uint32_t v) { p = wr16(p, (uint16_t)(v >> 16)); return wr16(p, (uint16_t)v); }

static char lower(char c) { return (c >= 'A' && c <= 'Z') ? (char)(c - 'A' + 'a') : c; }

static bool label_eq(const uint8_t *lab, size_t n, const char *want, size_t wn)
{
    if (n != wn) return false;
    for (size_t i = 0; i < n; i++)
        if (lower((char)lab[i]) != lower(want[i])) return false;
    return true;
}

/* Walk the name at *off (compression pointers allowed). Sets *off past the name as it sits in
   the message and *match to whether it is exactly <host>.local. false = malformed. */
static bool name_walk(const uint8_t *msg, size_t len, size_t *off, const char *host, size_t hn,
                      bool *match)
{
    size_t p = *off, end = 0;
    int idx = 0, hops = 0;
    bool eq = true;

    for (;;) {
        if (p >= len) return false;
        uint8_t n = msg[p];
        if ((n & 0xC0u) == 0xC0u) {
            if (p + 1 >= len || ++hops > 16) return false;
            if (!end) end = p + 2;
            p = ((size_t)(n & 0x3Fu) << 8) | msg[p + 1];
            continue;
        }
        if (n & 0xC0u) return false;               /* reserved label types */
        if (n == 0) { p++; break; }
        if (p + 1 + n > len) return false;
        if (idx == 0)      eq = eq && label_eq(msg + p + 1, n, host, hn);
        else if (idx == 1) eq = eq && label_eq(msg + p + 1, n, "local", 5);
        else               eq = false;
        idx++;
        p += 1u + n;
    }
    *off = end ? end : p;
    *match = eq && idx == 2;
    return true;
}

static uint8_t *wr_name(uint8_t *p, const char *host, size_t hn)
{
    *p++ = (uint8_t)hn;
    memcpy(p, host, hn); p += hn;
    *p++ = 5;
    memcpy(p, "local", 5); p += 5;
    *p++ = 0;
    return p;
}

size_t mdns_nsec_reply(const uint8_t *msg, size_t len, const char *host, bool legacy,
                       uint8_t *out, size_t cap)
{
    if (!msg || !host || !out || len < DNS_HDR_LEN) return 0;
    size_t hn = strlen(host);
    if (hn == 0 || hn > 63) return 0;

    uint16_t flags = rd16(msg + 2);
    if (flags & (DNS_FLAG_QR | DNS_OPCODE_MASK)) return 0;   /* responses, non-standard queries */

    uint16_t qd = rd16(msg + 4);
    size_t off = DNS_HDR_LEN;
    bool asked = false;
    for (uint16_t i = 0; i < qd; i++) {
        bool match;
        if (!name_walk(msg, len, &off, host, hn, &match) || off + 4 > len) return 0;
        uint16_t type = rd16(msg + off), cls = (uint16_t)(rd16(msg + off + 2) & ~MDNS_CACHE_FLUSH);
        off += 4;
        if (match && type == DNS_TYPE_AAAA && (cls == DNS_CLASS_IN || cls == DNS_CLASS_ANY))
            asked = true;
    }
    if (!asked) return 0;

    size_t name_len = hn + 8;                               /* len host 5 "local" 0 */
    size_t need = DNS_HDR_LEN + (legacy ? name_len + 4 : 0) + name_len + 10 + name_len + 3;
    if (need > cap) return 0;

    uint8_t *p = out;
    p = wr16(p, legacy ? rd16(msg) : 0);                    /* id */
    p = wr16(p, DNS_FLAG_QR | DNS_FLAG_AA);
    p = wr16(p, legacy ? 1 : 0);                            /* questions */
    p = wr16(p, 1);                                         /* answers */
    p = wr16(p, 0);
    p = wr16(p, 0);
    if (legacy) {
        p = wr_name(p, host, hn);
        p = wr16(p, DNS_TYPE_AAAA);
        p = wr16(p, DNS_CLASS_IN);
    }
    p = wr_name(p, host, hn);
    p = wr16(p, DNS_TYPE_NSEC);
    p = wr16(p, legacy ? DNS_CLASS_IN : (DNS_CLASS_IN | MDNS_CACHE_FLUSH));
    p = wr32(p, legacy ? MDNS_NSEC_LEGACY_TTL : MDNS_NSEC_TTL);
    p = wr16(p, (uint16_t)(name_len + 3));
    p = wr_name(p, host, hn);                               /* next domain name = own name */
    *p++ = 0;                                               /* window block 0 */
    *p++ = 1;                                               /* bitmap length */
    *p++ = 0x80u >> DNS_TYPE_A;                             /* types present: A */
    return (size_t)(p - out);
}
