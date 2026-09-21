#include "ws_frame.h"

#include "pico/rand.h"   /* get_rand_64 — RP2350 hardware TRNG */

#include <string.h>

size_t ws_build_frame(uint8_t opcode, const uint8_t *payload, size_t len,
                      const uint8_t mask_key[4], uint8_t *out, size_t out_cap) {
    if (len > 0xFFFF) return 0;   /* we never send anything this big */

    size_t hdr = 2;
    if (len >= 126) hdr += 2;     /* 16-bit extended length */
    hdr += 4;                     /* masking key */
    if (out_cap < hdr + len) return 0;

    out[0] = 0x80 | (opcode & 0x0F);   /* FIN=1 */
    if (len < 126) {
        out[1] = 0x80 | (uint8_t)len;  /* MASK=1 */
    } else {
        out[1] = 0x80 | 126;
        out[2] = (uint8_t)(len >> 8);
        out[3] = (uint8_t)(len & 0xFF);
    }
    size_t mpos = hdr - 4;
    memcpy(out + mpos, mask_key, 4);
    for (size_t i = 0; i < len; i++) {
        out[hdr + i] = payload[i] ^ mask_key[i & 3];
    }
    return hdr + len;
}

int ws_parse_frame(const uint8_t *buf, size_t len, ws_frame_t *out) {
    if (len < 2) return 0;
    uint8_t b0 = buf[0];
    uint8_t b1 = buf[1];
    bool    masked = (b1 & 0x80) != 0;
    if (masked) return -1;          /* server frames must be unmasked */

    size_t   offset = 2;
    uint64_t payload_len = (uint8_t)(b1 & 0x7F);
    if (payload_len == 126) {
        if (len < 4) return 0;
        payload_len = ((uint64_t)buf[2] << 8) | buf[3];
        offset = 4;
    } else if (payload_len == 127) {
        if (len < 10) return 0;
        uint64_t n = 0;
        for (int i = 0; i < 8; i++) n = (n << 8) | buf[2 + i];
        if (n > 0xFFFF) return -1;   /* refuse anything we can't buffer */
        payload_len = n;
        offset = 10;
    }

    if (len < offset + payload_len) return 0;   /* incomplete */

    out->opcode      = b0 & 0x0F;
    out->fin         = (b0 & 0x80) != 0;
    out->payload     = buf + offset;
    out->payload_len = (size_t)payload_len;
    return (int)(offset + payload_len);
}

void ws_make_sec_key(char out[25]) {
    static const char b64[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    uint8_t key[16];
    for (int i = 0; i < 16; i += 8) {
        uint64_t r = get_rand_64();
        memcpy(key + i, &r, 8);
    }
    /* Standard base64 of 16 bytes -> 24 chars, always ending in "==". */
    int o = 0;
    for (int i = 0; i < 15; i += 3) {
        uint32_t v = ((uint32_t)key[i] << 16) | ((uint32_t)key[i + 1] << 8) | key[i + 2];
        out[o++] = b64[(v >> 18) & 0x3F];
        out[o++] = b64[(v >> 12) & 0x3F];
        out[o++] = b64[(v >> 6) & 0x3F];
        out[o++] = b64[v & 0x3F];
    }
    /* Final byte (key[15]) is a 1-byte remainder -> 2 significant chars + TWO '=' pad chars
       ("XX=="), per RFC 4648. The previous code emitted only one '=', so Sec-WebSocket-Key was
       23 chars and decoded to <16 bytes; the server's WebSocket library strictly validates a
       16-byte key and rejected the handshake with HTTP 400. */
    uint32_t v = (uint32_t)key[15] << 16;
    out[o++] = b64[(v >> 18) & 0x3F];
    out[o++] = b64[(v >> 12) & 0x3F];
    out[o++] = '=';
    out[o++] = '=';
    out[o]   = '\0';
}
