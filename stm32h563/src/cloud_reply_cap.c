/*
 * cloud_reply_cap.c: see cloud_reply_cap.h. Hw worker task only (one cloud command at a time).
 */
#include "cloud_reply_cap.h"

#include <stdio.h>
#include <string.h>

static char   s_buf[CLOUD_REPLY_CAP_MAX];
static size_t s_len;
static size_t s_want;      /* every byte the handler wrote, kept or not */
static size_t s_nl;        /* how many of them, at the end, are line-ending characters */
static bool   s_active;

void cloud_reply_cap_begin(void) {
    s_active = true;
    s_len = 0;
    s_want = 0;
    s_nl = 0;
    s_buf[0] = '\0';
}

void cloud_reply_cap_append(const uint8_t *buf, size_t len) {
    if (!s_active) return;
    s_want += len;
    for (size_t i = 0; i < len; i++) s_nl = (buf[i] == '\n' || buf[i] == '\r') ? s_nl + 1 : 0;
    size_t space = sizeof(s_buf) - 1 - s_len;
    if (len > space) len = space;
    memcpy(s_buf + s_len, buf, len);
    s_len += len;
    s_buf[s_len] = '\0';
}

static size_t err_reply(char *out, size_t out_cap, const char *msg) {
    int n = snprintf(out, out_cap, "{\"status\":\"error\",\"message\":\"%s\"}", msg);
    return (n > 0 && (size_t)n < out_cap) ? (size_t)n : 0;
}

size_t cloud_reply_cap_end(char *out, size_t out_cap) {
    s_active = false;
    if (!out || out_cap == 0) return 0;
    /* The reply proper, without the line ending; only that may have been cut off. */
    size_t want = s_want - s_nl;
    if (s_len > want) { s_len = want; s_buf[s_len] = '\0'; }
    size_t limit = (out_cap < sizeof(s_buf) ? out_cap : sizeof(s_buf)) - 1u;
    if (want > s_len || s_len > limit) {
        char msg[96];
        snprintf(msg, sizeof(msg), "reply too large for the cloud channel (%u bytes, limit %u)",
                 (unsigned)want, (unsigned)limit);
        printf("[cloud] %s\n", msg);
        return err_reply(out, out_cap, msg);
    }
    if (s_len == 0) return err_reply(out, out_cap, "no reply");
    memcpy(out, s_buf, s_len);
    out[s_len] = '\0';
    return s_len;
}
