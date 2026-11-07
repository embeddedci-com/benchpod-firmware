#ifndef EMI_HTTP_H
#define EMI_HTTP_H

/*
 * emi_http — the EMI test build's web page for a test lab (`make EMI_TEST=1` only).
 *
 * Port 80 of the pod serves one page (tools/emi/lab_page.html, built in) that shows the
 * activity's state live and keeps a copy of the run log (emi_log.h) in the browser, so the lab
 * opens http://<the pod's address>/ on any laptop on its network: no install, no account.
 *
 * Read-only by design: the page and two API paths, nothing that changes the pod (any web page the
 * lab's browser opens could send requests to the pod's address):
 *
 *   GET /                    the page
 *   GET /api/status          {"cmd":"emi_log"}: the log's state and the whole emi_test object
 *   GET /api/log?after=N     {"cmd":"emi_log","after":N,"count":16}: the records after seq N
 *
 * HTTP/1.0, one request per connection, closed after the reply. The API replies are the JSON
 * command's own reply line (the hw worker runs it as for a LAN client, net_server.c); the page is
 * sent from flash. This file is the request parsing only, so it runs in the host tests.
 */

#include <stddef.h>
#include <stdint.h>

#define EMI_HTTP_PORT  80

typedef enum {
    EMI_HTTP_PAGE = 1,    /* send emi_page_html */
    EMI_HTTP_API,         /* send `cmd` to the hw worker, its reply is the body */
    EMI_HTTP_ERROR,       /* send `head` (a complete short reply) and close */
} emi_http_kind_t;

typedef struct {
    emi_http_kind_t kind;
    char            head[224];   /* the status line and headers (the whole reply for ERROR) */
    size_t          head_len;
    char            cmd[80];     /* EMI_HTTP_API: the JSON command, newline-terminated */
    size_t          cmd_len;
} emi_http_req_t;

/* The page, generated from tools/emi/lab_page.html (Makefile). */
extern const unsigned char emi_page_html[];
extern const size_t emi_page_html_len;

/* Parse the start of a request: the request line has to be in `req` (the first TCP segment). */
void emi_http_parse(const char *req, size_t len, size_t page_len, emi_http_req_t *out);

#endif /* EMI_HTTP_H */
