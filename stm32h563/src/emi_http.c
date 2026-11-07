/*
 * emi_http.c — request parsing for the EMI test build's lab page (emi_http.h).
 */
#include "emi_http.h"

#include <stdio.h>
#include <string.h>

static void error_reply(emi_http_req_t *o, const char *status, const char *text) {
    o->kind = EMI_HTTP_ERROR;
    int n = snprintf(o->head, sizeof(o->head),
                     "HTTP/1.0 %s\r\nContent-Type: text/plain\r\nContent-Length: %u\r\n"
                     "Connection: close\r\n\r\n%s",
                     status, (unsigned)strlen(text), text);
    o->head_len = (n > 0 && (size_t)n < sizeof(o->head)) ? (size_t)n : 0;
}

static void ok_head(emi_http_req_t *o, const char *type, size_t len) {
    int n = len
        ? snprintf(o->head, sizeof(o->head),
                   "HTTP/1.0 200 OK\r\nContent-Type: %s\r\nContent-Length: %u\r\n"
                   "Cache-Control: no-store\r\nConnection: close\r\n\r\n", type, (unsigned)len)
        : snprintf(o->head, sizeof(o->head),
                   "HTTP/1.0 200 OK\r\nContent-Type: %s\r\n"
                   "Cache-Control: no-store\r\nConnection: close\r\n\r\n", type);
    o->head_len = (n > 0 && (size_t)n < sizeof(o->head)) ? (size_t)n : 0;
}

/* `s` (n bytes) equals the NUL-terminated `lit`. */
static int eq(const char *s, size_t n, const char *lit) {
    return strlen(lit) == n && memcmp(s, lit, n) == 0;
}

void emi_http_parse(const char *req, size_t len, size_t page_len, emi_http_req_t *out) {
    memset(out, 0, sizeof(*out));
    const char *eol = memchr(req, '\n', len);
    if (!eol) { error_reply(out, "400 Bad Request", "bad request\n"); return; }
    const size_t line = (size_t)(eol - req);
    if (line < 4 || memcmp(req, "GET ", 4) != 0) {
        error_reply(out, "405 Method Not Allowed", "only GET\n");
        return;
    }
    const char *path = req + 4;
    const char *sp = memchr(path, ' ', line - 4);
    const size_t plen = sp ? (size_t)(sp - path) : line - 4;
    const char *q = memchr(path, '?', plen);
    const size_t base = q ? (size_t)(q - path) : plen;

    if (eq(path, base, "/") || eq(path, base, "/index.html")) {
        out->kind = EMI_HTTP_PAGE;
        ok_head(out, "text/html; charset=utf-8", page_len);
        return;
    }
    if (eq(path, base, "/api/status")) {
        out->kind = EMI_HTTP_API;
        out->cmd_len = (size_t)snprintf(out->cmd, sizeof(out->cmd), "{\"cmd\":\"emi_log\"}\n");
        ok_head(out, "application/json", 0);
        return;
    }
    if (eq(path, base, "/api/log")) {
        /* after=<digits>, the only parameter */
        unsigned long after = 0;
        int digits = 0;
        if (q) {
            const char *p = q + 1, *end = path + plen;
            if (end - p > 6 && memcmp(p, "after=", 6) == 0) {
                for (p += 6; p < end && *p >= '0' && *p <= '9' && digits < 10; p++, digits++)
                    after = after * 10u + (unsigned long)(*p - '0');
                if (p != end) digits = 0;
            }
        }
        if (!digits || after > 0xFFFFFFFFul) {
            error_reply(out, "400 Bad Request", "use /api/log?after=<sequence number>\n");
            return;
        }
        out->kind = EMI_HTTP_API;
        out->cmd_len = (size_t)snprintf(out->cmd, sizeof(out->cmd),
                                        "{\"cmd\":\"emi_log\",\"after\":%lu,\"count\":16}\n", after);
        ok_head(out, "application/json", 0);
        return;
    }
    error_reply(out, "404 Not Found", "not found\n");
}
