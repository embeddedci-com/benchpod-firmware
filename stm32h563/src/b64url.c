#include "b64url.h"

#include <string.h>

static const char ENC[64] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_";

/* Reverse lookup: maps an ASCII byte to its 6-bit value, or 0xFF if not part
   of the base64url alphabet.  Built once on first use. */
static uint8_t DEC[256];
static int dec_ready = 0;

static void dec_init(void) {
    memset(DEC, 0xFF, sizeof(DEC));
    for (int i = 0; i < 64; i++) {
        DEC[(unsigned char)ENC[i]] = (uint8_t)i;
    }
    dec_ready = 1;
}

size_t b64url_encode(const uint8_t *in, size_t n, char *out, size_t out_cap) {
    size_t need = B64URL_ENCODED_LEN(n);
    if (out_cap < need + 1) return 0;

    size_t o = 0;
    size_t i = 0;
    while (i + 3 <= n) {
        uint32_t v = ((uint32_t)in[i] << 16) | ((uint32_t)in[i + 1] << 8) | in[i + 2];
        out[o++] = ENC[(v >> 18) & 0x3F];
        out[o++] = ENC[(v >> 12) & 0x3F];
        out[o++] = ENC[(v >> 6) & 0x3F];
        out[o++] = ENC[v & 0x3F];
        i += 3;
    }
    size_t rem = n - i;
    if (rem == 1) {
        uint32_t v = (uint32_t)in[i] << 16;
        out[o++] = ENC[(v >> 18) & 0x3F];
        out[o++] = ENC[(v >> 12) & 0x3F];
    } else if (rem == 2) {
        uint32_t v = ((uint32_t)in[i] << 16) | ((uint32_t)in[i + 1] << 8);
        out[o++] = ENC[(v >> 18) & 0x3F];
        out[o++] = ENC[(v >> 12) & 0x3F];
        out[o++] = ENC[(v >> 6) & 0x3F];
    }
    out[o] = '\0';
    return o;
}

int b64url_decode(const char *in, uint8_t *out, size_t out_cap, size_t *out_len) {
    if (!dec_ready) dec_init();

    /* Length excluding any optional '=' padding. */
    size_t len = strlen(in);
    while (len > 0 && in[len - 1] == '=') len--;

    /* A base64url group is 2, 3, or 4 chars; a remainder of exactly 1 char is
       impossible (it carries no whole byte). */
    if (len % 4 == 1) return -1;

    size_t decoded = (len / 4) * 3 + (len % 4 ? (len % 4) - 1 : 0);
    if (decoded > out_cap) return -1;

    size_t o = 0;
    size_t i = 0;
    while (i + 4 <= len) {
        uint8_t a = DEC[(unsigned char)in[i]];
        uint8_t b = DEC[(unsigned char)in[i + 1]];
        uint8_t c = DEC[(unsigned char)in[i + 2]];
        uint8_t d = DEC[(unsigned char)in[i + 3]];
        if ((a | b | c | d) == 0xFF) return -1;
        out[o++] = (uint8_t)((a << 2) | (b >> 4));
        out[o++] = (uint8_t)((b << 4) | (c >> 2));
        out[o++] = (uint8_t)((c << 6) | d);
        i += 4;
    }
    size_t rem = len - i;
    if (rem == 2) {
        uint8_t a = DEC[(unsigned char)in[i]];
        uint8_t b = DEC[(unsigned char)in[i + 1]];
        if ((a | b) == 0xFF) return -1;
        out[o++] = (uint8_t)((a << 2) | (b >> 4));
    } else if (rem == 3) {
        uint8_t a = DEC[(unsigned char)in[i]];
        uint8_t b = DEC[(unsigned char)in[i + 1]];
        uint8_t c = DEC[(unsigned char)in[i + 2]];
        if ((a | b | c) == 0xFF) return -1;
        out[o++] = (uint8_t)((a << 2) | (b >> 4));
        out[o++] = (uint8_t)((b << 4) | (c >> 2));
    }

    if (out_len) *out_len = o;
    return 0;
}
