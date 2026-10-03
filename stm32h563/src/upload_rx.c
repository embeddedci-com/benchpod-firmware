/*
 * upload_rx.c — raw chunk receive for uploads over the USB console (see upload_rx.h).
 */
#include "upload_rx.h"
#include <stdlib.h>

uint32_t upload_crc32(const uint8_t *p, uint32_t n)
{
    uint32_t crc = ~0u;
    while (n--) {
        crc ^= *p++;
        for (int k = 0; k < 8; k++)
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return ~crc;
}

int upload_rx_start(upload_rx_t *u, const char *args, uint32_t now_ms)
{
    char *end;
    u->active = false;
    if (!args) return -1;
    unsigned long off = strtoul(args, &end, 0);
    if (end == args) return -1;
    const char *p = end;
    unsigned long len = strtoul(p, &end, 0);
    if (end == p || len == 0 || len > UPLOAD_CHUNK_MAX) return -1;
    p = end;
    unsigned long crc = strtoul(p, &end, 16);
    if (end == p) return -1;
    while (*end == ' ' || *end == '\t') end++;
    if (*end != '\0') return -1;
    u->offset  = (uint32_t)off;
    u->len     = (uint32_t)len;
    u->crc     = (uint32_t)crc;
    u->got     = 0;
    u->last_ms = now_ms;
    u->active  = true;
    return 0;
}

bool upload_rx_byte(upload_rx_t *u, uint8_t b, uint32_t now_ms)
{
    if (!u->active) return false;
    u->buf[u->got++] = b;
    u->last_ms = now_ms;
    if (u->got < u->len) return false;
    u->active = false;
    return true;
}

bool upload_rx_crc_ok(const upload_rx_t *u)
{
    return u->got == u->len && upload_crc32(u->buf, u->len) == u->crc;
}

bool upload_rx_timed_out(upload_rx_t *u, uint32_t now_ms)
{
    if (!u->active || (uint32_t)(now_ms - u->last_ms) < UPLOAD_RX_TIMEOUT_MS) return false;
    u->active = false;
    return true;
}
