/*
 * test_emi_http.c — host unit test for the EMI build's lab page requests (emi_http.h): the page,
 * the two read-only API paths, and refusals of everything else.
 */
#include "emi_http.h"

#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(c) do { if(!(c)){printf("FAIL %s:%d  %s\n",__FILE__,__LINE__,#c);failures++;} } while(0)

const unsigned char emi_page_html[] = "<html></html>";
const size_t emi_page_html_len = sizeof(emi_page_html) - 1;

static emi_http_req_t parse(const char *s) {
    emi_http_req_t r;
    emi_http_parse(s, strlen(s), 1234, &r);
    return r;
}
static int has(const emi_http_req_t *r, const char *s) { return strstr(r->head, s) != NULL; }

int main(void) {
    emi_http_req_t r = parse("GET / HTTP/1.1\r\nHost: 192.168.1.150\r\n\r\n");
    CHECK(r.kind == EMI_HTTP_PAGE && has(&r, "200 OK") && has(&r, "Content-Length: 1234") &&
          has(&r, "text/html") && strstr(r.head, "\r\n\r\n") == r.head + r.head_len - 4);
    r = parse("GET /index.html?x=1 HTTP/1.1\r\n\r\n");
    CHECK(r.kind == EMI_HTTP_PAGE);

    r = parse("GET /api/status HTTP/1.1\r\n\r\n");
    CHECK(r.kind == EMI_HTTP_API && strcmp(r.cmd, "{\"cmd\":\"emi_log\"}\n") == 0 &&
          r.cmd_len == strlen(r.cmd) && has(&r, "application/json") && !has(&r, "Content-Length"));

    r = parse("GET /api/log?after=4294967295 HTTP/1.1\r\n\r\n");
    CHECK(r.kind == EMI_HTTP_API &&
          strcmp(r.cmd, "{\"cmd\":\"emi_log\",\"after\":4294967295,\"count\":16}\n") == 0);
    r = parse("GET /api/log?after=0 HTTP/1.0\r\n\r\n");
    CHECK(r.kind == EMI_HTTP_API && strstr(r.cmd, "\"after\":0,"));
    /* No other parameter or command can get through. */
    r = parse("GET /api/log HTTP/1.1\r\n\r\n");                    CHECK(r.kind == EMI_HTTP_ERROR && has(&r, "400"));
    r = parse("GET /api/log?after=12x HTTP/1.1\r\n\r\n");          CHECK(r.kind == EMI_HTTP_ERROR);
    r = parse("GET /api/log?after=1&cmd=x HTTP/1.1\r\n\r\n");      CHECK(r.kind == EMI_HTTP_ERROR);
    r = parse("GET /api/log?after=99999999999 HTTP/1.1\r\n\r\n");  CHECK(r.kind == EMI_HTTP_ERROR);
    r = parse("GET /api/log?after=\",\"cmd\":\"dac_stop HTTP/1.1\r\n\r\n"); CHECK(r.kind == EMI_HTTP_ERROR);
    r = parse("GET /api/emi_stop HTTP/1.1\r\n\r\n");               CHECK(r.kind == EMI_HTTP_ERROR && has(&r, "404"));
    r = parse("GET /favicon.ico HTTP/1.1\r\n\r\n");                CHECK(r.kind == EMI_HTTP_ERROR && has(&r, "404"));
    r = parse("POST /api/status HTTP/1.1\r\n\r\n");                CHECK(r.kind == EMI_HTTP_ERROR && has(&r, "405"));
    r = parse("GET /api/sta");                                     CHECK(r.kind == EMI_HTTP_ERROR && has(&r, "400"));
    /* Error replies are complete: headers and body. */
    r = parse("GET /nope HTTP/1.1\r\n\r\n");
    CHECK(r.head_len == strlen(r.head) && has(&r, "Content-Length: 10\r\n") && has(&r, "\r\n\r\nnot found\n"));

    if (failures) { printf("test_emi_http: %d FAILED\n", failures); return 1; }
    printf("test_emi_http: all passed\n");
    return 0;
}
