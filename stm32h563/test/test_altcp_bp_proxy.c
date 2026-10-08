/*
 * test_altcp_bp_proxy.c — host tests for the HTTP CONNECT layer of the cloud link
 * (src/altcp_bp_proxy.c): the CONNECT request, the reply parsing, and above all the teardown
 * when the proxy refuses or closes early (CRASH-2).
 *
 * The real altcp structs come from lwIP's headers; the altcp core, the TCP layer below and the
 * TLS layer above are fakes here:
 *
 *   app (cl_*_cb stand-ins) -> fake TLS (T) -> proxy layer (P) -> fake TCP (A)
 *
 * Fake TCP behaves like altcp_tcp: abort calls its err callback (altcp_tcp_err) and frees itself.
 * Fake TLS hooks onto P the way altcp_tls_mbedtls does: its lower_err clears T->inner_conn,
 * calls the application's err and frees T. Every pcb, state and pbuf is counted, so a leak, a
 * double free or a layer left pointing at a freed one shows up as a failed check.
 */
#include "lwip/altcp.h"
#include "lwip/priv/altcp_priv.h"
#include "lwip/altcp_tcp.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "altcp_bp_proxy.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures;
#define CHECK(cond, ...) do { \
    if (!(cond)) { printf("FAIL %s:%d  ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); failures++; } \
} while (0)

/* ---- bookkeeping ---------------------------------------------------------------------- */

#define MAX_PCBS 16
static struct altcp_pcb *s_live[MAX_PCBS];   /* allocated, not yet freed */
static int s_pcb_allocs, s_pcb_frees, s_mem_live, s_pbuf_live, s_bad_free;

static void live_add(struct altcp_pcb *c) {
    for (int i = 0; i < MAX_PCBS; i++) if (!s_live[i]) { s_live[i] = c; return; }
    abort();
}
static int live_del(struct altcp_pcb *c) {
    for (int i = 0; i < MAX_PCBS; i++) if (s_live[i] == c) { s_live[i] = NULL; return 1; }
    return 0;
}
static int is_live(const struct altcp_pcb *c) {
    for (int i = 0; i < MAX_PCBS; i++) if (s_live[i] == c) return 1;
    return 0;
}

/* ---- fake altcp core (what lwIP's core/altcp.c does, minus memp) ---------------------- */

struct altcp_pcb *altcp_alloc(void) {
    struct altcp_pcb *c = calloc(1, sizeof(*c));
    live_add(c);
    s_pcb_allocs++;
    return c;
}

void altcp_free(struct altcp_pcb *conn) {
    if (!conn) return;
    if (!live_del(conn)) { s_bad_free++; return; }   /* double free / never allocated */
    if (conn->fns && conn->fns->dealloc) conn->fns->dealloc(conn);
    s_pcb_frees++;
    memset(conn, 0xA5, sizeof(*conn));               /* make a use-after-free visible */
    free(conn);
}

static void must_be_live(const struct altcp_pcb *c, const char *what) {
    if (!is_live(c)) { printf("FAIL use of a freed pcb in %s\n", what); failures++; abort(); }
}

void altcp_arg(struct altcp_pcb *c, void *arg)                 { if (c) { must_be_live(c, "arg"); c->arg = arg; } }
void altcp_recv(struct altcp_pcb *c, altcp_recv_fn f)          { if (c) { must_be_live(c, "recv"); c->recv = f; } }
void altcp_sent(struct altcp_pcb *c, altcp_sent_fn f)          { if (c) { must_be_live(c, "sent"); c->sent = f; } }
void altcp_err(struct altcp_pcb *c, altcp_err_fn f)            { if (c) { must_be_live(c, "err"); c->err = f; } }
void altcp_poll(struct altcp_pcb *c, altcp_poll_fn f, u8_t iv) {
    if (!c) return;
    must_be_live(c, "poll");
    c->poll = f;
    c->pollinterval = iv;
    if (c->fns && c->fns->set_poll) c->fns->set_poll(c, iv);
}
void  altcp_recved(struct altcp_pcb *c, u16_t len)  { if (c) { must_be_live(c, "recved"); c->fns->recved(c, len); } }
void  altcp_abort(struct altcp_pcb *c)              { if (c) { must_be_live(c, "abort"); c->fns->abort(c); } }
err_t altcp_close(struct altcp_pcb *c)              { if (!c) return ERR_VAL; must_be_live(c, "close"); return c->fns->close(c); }
err_t altcp_output(struct altcp_pcb *c)             { if (!c) return ERR_VAL; must_be_live(c, "output"); return c->fns->output(c); }
err_t altcp_write(struct altcp_pcb *c, const void *d, u16_t len, u8_t fl) {
    if (!c) return ERR_VAL;
    must_be_live(c, "write");
    return c->fns->write(c, d, len, fl);
}
err_t altcp_connect(struct altcp_pcb *c, const ip_addr_t *ip, u16_t port, altcp_connected_fn f) {
    if (!c) return ERR_VAL;
    must_be_live(c, "connect");
    return c->fns->connect(c, ip, port, f);
}

void  altcp_default_set_poll(struct altcp_pcb *c, u8_t iv) { (void)c; (void)iv; }
void  altcp_default_recved(struct altcp_pcb *c, u16_t len) { (void)c; (void)len; }
err_t altcp_default_bind(struct altcp_pcb *c, const ip_addr_t *ip, u16_t port) { (void)c; (void)ip; (void)port; return ERR_OK; }
err_t altcp_default_shutdown(struct altcp_pcb *c, int rx, int tx) { (void)c; (void)rx; (void)tx; return ERR_OK; }
err_t altcp_default_output(struct altcp_pcb *c) { return c && c->inner_conn ? altcp_output(c->inner_conn) : ERR_VAL; }
u16_t altcp_default_mss(struct altcp_pcb *c) { (void)c; return 1460; }
u16_t altcp_default_sndbuf(struct altcp_pcb *c) { (void)c; return 4096; }
u16_t altcp_default_sndqueuelen(struct altcp_pcb *c) { (void)c; return 8; }
void  altcp_default_nagle_disable(struct altcp_pcb *c) { (void)c; }
void  altcp_default_nagle_enable(struct altcp_pcb *c) { (void)c; }
int   altcp_default_nagle_disabled(struct altcp_pcb *c) { (void)c; return 0; }
void  altcp_default_setprio(struct altcp_pcb *c, u8_t prio) { (void)c; (void)prio; }
err_t altcp_default_get_tcp_addrinfo(struct altcp_pcb *c, int l, ip_addr_t *a, u16_t *p) { (void)c; (void)l; (void)a; (void)p; return ERR_VAL; }
ip_addr_t *altcp_default_get_ip(struct altcp_pcb *c, int l) { (void)c; (void)l; return NULL; }
u16_t altcp_default_get_port(struct altcp_pcb *c, int l) { (void)c; (void)l; return 0; }

void *mem_malloc(mem_size_t size) { s_mem_live++; return malloc(size); }
void *mem_calloc(mem_size_t n, mem_size_t size) { s_mem_live++; return calloc(n, size); }
void  mem_free(void *p) { if (p) { s_mem_live--; free(p); } }

u8_t pbuf_free(struct pbuf *p) {
    u8_t n = 0;
    while (p) { struct pbuf *next = p->next; free(p); s_pbuf_live--; n++; p = next; }
    return n;
}

static struct pbuf *mk_pbuf(const char *text) {
    size_t len = strlen(text);
    struct pbuf *p = calloc(1, sizeof(*p) + len);
    p->payload = (u8_t *)(p + 1);
    memcpy(p->payload, text, len);
    p->len = p->tot_len = (u16_t)len;
    p->ref = 1;
    s_pbuf_live++;
    return p;
}

/* ---- fake TCP layer (altcp_tcp) ------------------------------------------------------- */

static char  s_tx[1024];   /* what the proxy layer wrote to the wire */
static size_t s_tx_len;
static int   s_tcp_aborts, s_tcp_closes, s_tcp_recved;
static struct altcp_pcb *s_tcp;   /* the TCP pcb of the current test */

static void  ftcp_set_poll(struct altcp_pcb *c, u8_t iv) { (void)c; (void)iv; }
static void  ftcp_recved(struct altcp_pcb *c, u16_t len) { (void)c; s_tcp_recved += len; }
static err_t ftcp_connect(struct altcp_pcb *c, const ip_addr_t *ip, u16_t port, altcp_connected_fn f) {
    (void)ip; (void)port; c->connected = f; return ERR_OK;
}
/* altcp_tcp_abort -> tcp_abort -> altcp_tcp_err: err callback (if any), then free. */
static void ftcp_abort(struct altcp_pcb *c) {
    s_tcp_aborts++;
    if (c->err) c->err(c->arg, ERR_ABRT);
    altcp_free(c);
}
static err_t ftcp_close(struct altcp_pcb *c) { s_tcp_closes++; altcp_free(c); return ERR_OK; }
static err_t ftcp_write(struct altcp_pcb *c, const void *d, u16_t len, u8_t fl) {
    (void)c; (void)fl;
    if (s_tx_len + len > sizeof(s_tx)) return ERR_MEM;
    memcpy(s_tx + s_tx_len, d, len);
    s_tx_len += len;
    return ERR_OK;
}
static err_t ftcp_output(struct altcp_pcb *c) { (void)c; return ERR_OK; }
static void  ftcp_dealloc(struct altcp_pcb *c) { (void)c; }

static const struct altcp_functions k_ftcp = {
    ftcp_set_poll, ftcp_recved, altcp_default_bind, ftcp_connect, NULL, ftcp_abort, ftcp_close,
    altcp_default_shutdown, ftcp_write, ftcp_output, altcp_default_mss, altcp_default_sndbuf,
    altcp_default_sndqueuelen, altcp_default_nagle_disable, altcp_default_nagle_enable,
    altcp_default_nagle_disabled, altcp_default_setprio, ftcp_dealloc,
    altcp_default_get_tcp_addrinfo, altcp_default_get_ip, altcp_default_get_port,
};

struct altcp_pcb *altcp_tcp_new_ip_type(u8_t ip_type) {
    (void)ip_type;
    struct altcp_pcb *c = altcp_alloc();
    c->fns = &k_ftcp;
    s_tcp = c;
    return c;
}

/* ---- fake TLS layer above (what altcp_tls_mbedtls registers on its inner pcb) ---------- */

static int   s_app_err_calls, s_app_connected, s_app_rx;
static err_t s_app_err;

static void app_err(void *arg, err_t err) { (void)arg; s_app_err_calls++; s_app_err = err; }

static err_t tls_lower_connected(void *arg, struct altcp_pcb *inner, err_t err) {
    (void)arg; (void)inner; if (err == ERR_OK) s_app_connected++; return ERR_OK;
}
static err_t tls_lower_recv(void *arg, struct altcp_pcb *inner, struct pbuf *p, err_t err) {
    (void)arg; (void)err;
    if (p) { s_app_rx += p->tot_len; altcp_recved(inner, p->tot_len); pbuf_free(p); }
    return ERR_OK;
}
/* altcp_mbedtls_lower_err */
static void tls_lower_err(void *arg, err_t err) {
    struct altcp_pcb *t = arg;
    if (!t) return;
    t->inner_conn = NULL;
    if (t->err) t->err(t->arg, err);
    altcp_free(t);
}
static void  tls_dealloc(struct altcp_pcb *c) { (void)c; }
static void  tls_abort(struct altcp_pcb *c) { if (c->inner_conn) altcp_abort(c->inner_conn); }
static err_t tls_close(struct altcp_pcb *c) {
    if (c->inner_conn && altcp_close(c->inner_conn) != ERR_OK) return ERR_MEM;
    c->inner_conn = NULL;
    altcp_free(c);
    return ERR_OK;
}
static const struct altcp_functions k_ftls = {
    NULL, NULL, NULL, NULL, NULL, tls_abort, tls_close, NULL, NULL, NULL, NULL, NULL, NULL,
    NULL, NULL, NULL, NULL, tls_dealloc, NULL, NULL, NULL,
};

/* ---- harness -------------------------------------------------------------------------- */

static struct altcp_bp_proxy_config s_conf;
static struct altcp_pcb *s_tls, *s_proxy;

/* Build tls(proxy(tcp)), connect, and let the TCP connect succeed (the CONNECT goes out). */
static void setup(const char *auth_b64) {
    memset(s_live, 0, sizeof(s_live));
    s_pcb_allocs = s_pcb_frees = s_mem_live = s_pbuf_live = s_bad_free = 0;
    s_tx_len = 0; s_tx[0] = 0;
    s_tcp_aborts = s_tcp_closes = s_tcp_recved = 0;
    s_app_err_calls = s_app_connected = s_app_rx = 0; s_app_err = ERR_OK;

    memset(&s_conf, 0, sizeof(s_conf));
    s_conf.proxy_port = 3128;
    s_conf.target_host = "embeddedci.com";
    s_conf.auth_b64 = auth_b64;
    s_conf.last_status = 999;    /* the layer must overwrite it */

    s_proxy = altcp_bp_proxy_new_tcp(&s_conf, IPADDR_TYPE_V4);
    CHECK(s_proxy && s_tcp && s_proxy->inner_conn == s_tcp, "stack not built");
    s_tls = altcp_alloc();
    s_tls->fns = &k_ftls;
    s_tls->inner_conn = s_proxy;
    s_tls->err = app_err;
    altcp_arg(s_proxy, s_tls);
    altcp_recv(s_proxy, tls_lower_recv);
    altcp_err(s_proxy, tls_lower_err);

    ip_addr_t ip;
    memset(&ip, 0, sizeof(ip));
    CHECK(altcp_connect(s_proxy, &ip, 443, tls_lower_connected) == ERR_OK, "connect");
    CHECK(s_tcp->connected(s_tcp->arg, s_tcp, ERR_OK) == ERR_OK, "lower connected");
}

/* The TCP pcb delivers data (or the FIN, text NULL), as altcp_tcp_recv does. */
static err_t deliver(const char *text) {
    return s_tcp->recv(s_tcp->arg, s_tcp, text ? mk_pbuf(text) : NULL, ERR_OK);
}

/* After a setup failure: every layer freed exactly once, the app told once, nothing leaked. */
static void check_torn_down(const char *what) {
    CHECK(s_bad_free == 0, "%s: %d bad frees", what, s_bad_free);
    CHECK(s_pcb_frees == s_pcb_allocs && s_pcb_allocs == 3, "%s: %d of %d pcbs freed", what,
          s_pcb_frees, s_pcb_allocs);
    CHECK(s_mem_live == 0, "%s: %d mem blocks leaked", what, s_mem_live);
    CHECK(s_pbuf_live == 0, "%s: %d pbufs leaked", what, s_pbuf_live);
    CHECK(s_app_err_calls == 1 && s_app_err == ERR_ABRT, "%s: app err %d calls, err %d", what,
          s_app_err_calls, (int)s_app_err);
    CHECK(s_app_connected == 0, "%s: app told connected", what);
    CHECK(s_tcp_aborts == 1 && s_tcp_closes == 0, "%s: tcp aborts %d closes %d", what, s_tcp_aborts,
          s_tcp_closes);
}

/* ---- tests ---------------------------------------------------------------------------- */

static void test_request(void) {
    setup("dTpw");
    CHECK(strstr(s_tx, "CONNECT embeddedci.com:443 HTTP/1.1\r\n") == s_tx, "request: %s", s_tx);
    CHECK(strstr(s_tx, "Host: embeddedci.com:443\r\n") != NULL, "no Host");
    CHECK(strstr(s_tx, "Proxy-Authorization: Basic dTpw\r\n") != NULL, "no auth");
    CHECK(s_tx_len >= 4 && memcmp(s_tx + s_tx_len - 4, "\r\n\r\n", 4) == 0, "not terminated");
    altcp_close(s_tls);
    CHECK(s_pcb_frees == 3 && s_mem_live == 0 && s_bad_free == 0, "close leaked");

    setup(NULL);
    CHECK(strstr(s_tx, "Proxy-Authorization") == NULL, "auth without credentials");
    altcp_close(s_tls);
}

static void test_ok_split(void) {
    setup(NULL);
    /* the reply arrives in pieces, the end marker split across them */
    CHECK(deliver("HTTP/1.1 200 Connection established\r") == ERR_OK, "part 1");
    CHECK(s_app_connected == 0, "connected too early");
    CHECK(deliver("\nProxy-Agent: x\r\n\r") == ERR_OK, "part 2");
    CHECK(s_app_connected == 0, "connected before the blank line");
    CHECK(deliver("\n") == ERR_OK, "part 3");
    CHECK(s_app_connected == 1 && s_conf.last_status == 200 && !s_conf.closed_early, "200 not seen");
    CHECK(s_tcp_recved > 0, "reply bytes not acknowledged");
    /* application phase: data passes through to TLS */
    CHECK(deliver("\x16\x03\x03") == ERR_OK && s_app_rx == 3, "pass-through %d", s_app_rx);
    CHECK(s_app_err_calls == 0, "app err on success");
    altcp_close(s_tls);
    CHECK(s_pcb_frees == 3 && s_mem_live == 0 && s_pbuf_live == 0 && s_bad_free == 0, "close leaked");

    setup(NULL);
    CHECK(deliver("HTTP/1.0 200 OK\r\n\r\n") == ERR_OK && s_conf.last_status == 200, "HTTP/1.0");
    altcp_close(s_tls);
}

static void test_refused(void) {
    static const struct { const char *reply; unsigned status; } cases[] = {
        { "HTTP/1.1 407 Proxy Authentication Required\r\nProxy-Authenticate: Basic\r\n\r\n", 407 },
        { "HTTP/1.1 403 Forbidden\r\n\r\n", 403 },
        { "HTTP/1.1 502 Bad Gateway\r\n\r\n", 502 },
    };
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        setup("dTpw");
        CHECK(deliver(cases[i].reply) == ERR_ABRT, "%u: not ERR_ABRT", cases[i].status);
        CHECK(s_conf.last_status == cases[i].status, "status %u, want %u", s_conf.last_status,
              cases[i].status);
        CHECK(!s_conf.closed_early, "%u: closed_early", cases[i].status);
        check_torn_down("refused");
    }
}

/* CRASH-2: the proxy sends FIN before (or in the middle of) its reply. */
static void test_closed_early(void) {
    setup(NULL);
    CHECK(deliver(NULL) == ERR_ABRT, "FIN: not ERR_ABRT");
    CHECK(s_conf.last_status == 0 && s_conf.closed_early, "FIN: status %u closed_early %u",
          s_conf.last_status, s_conf.closed_early);
    check_torn_down("FIN before reply");

    setup(NULL);
    CHECK(deliver("HTTP/1.1 200 Conn") == ERR_OK, "partial reply");
    CHECK(deliver(NULL) == ERR_ABRT, "FIN mid-reply: not ERR_ABRT");
    CHECK(s_conf.last_status == 0 && s_conf.closed_early, "FIN mid-reply: status %u", s_conf.last_status);
    check_torn_down("FIN mid-reply");
}

/* The reply in a pbuf chain (one segment split over pbufs), and a stray CR before the end. */
static void test_reply_shapes(void) {
    setup(NULL);
    struct pbuf *p = mk_pbuf("HTTP/1.1 200 OK\r\n");
    p->next = mk_pbuf("\r\n");
    p->tot_len = (u16_t)(p->len + p->next->len);
    CHECK(s_tcp->recv(s_tcp->arg, s_tcp, p, ERR_OK) == ERR_OK, "chain");
    CHECK(s_app_connected == 1 && s_conf.last_status == 200, "chained reply not seen");
    CHECK(s_tcp_recved == 19, "acknowledged %d of 19", s_tcp_recved);
    altcp_close(s_tls);
    CHECK(s_pcb_frees == 3 && s_pbuf_live == 0 && s_bad_free == 0, "chain: close leaked");

    setup(NULL);
    CHECK(deliver("HTTP/1.1 200 OK\r\r\n\r\n") == ERR_OK, "CR CR LF");
    CHECK(s_app_connected == 1, "a doubled CR hid the end of the headers");
    altcp_close(s_tls);

    /* a blank line inside the status line is not the end: \r\n then text then \r\n\r\n */
    setup(NULL);
    CHECK(deliver("HTTP/1.1 200 OK\r\nX: \r\n") == ERR_OK && s_app_connected == 0, "early end");
    CHECK(deliver("\r\n") == ERR_OK && s_app_connected == 1, "end not seen");
    altcp_close(s_tls);
}

/* Something that is not an HTTP proxy answers (or a 200 from HTTP/2 or a lookalike). */
static void test_not_http(void) {
    static const char *const replies[] = {
        "SSH-2.0-OpenSSH_9.6\r\n\r\n",
        "HTTP/2 200\r\n\r\n",
        "HTTP/1.1 201 Created\r\n\r\n",
    };
    for (size_t i = 0; i < sizeof(replies) / sizeof(replies[0]); i++) {
        setup(NULL);
        CHECK(deliver(replies[i]) == ERR_ABRT, "%zu: accepted", i);
        CHECK(s_conf.last_status != 200 && !s_conf.closed_early, "%zu: status %u", i, s_conf.last_status);
        check_torn_down("not http");
    }
}

/* The application gives up mid-handshake (its connect timeout): aborting the TLS pcb runs the
   abort down the chain, and every layer is freed once. Before the fix the proxy layer was freed
   twice: once by its lower_err (the TCP abort's err callback) and again by its own abort. */
static void test_app_abort_during_setup(void) {
    setup("dTpw");
    CHECK(deliver("HTTP/1.1 200 Conn") == ERR_OK, "partial reply");
    altcp_abort(s_tls);
    CHECK(s_bad_free == 0, "abort: %d bad frees", s_bad_free);
    CHECK(s_pcb_frees == s_pcb_allocs && s_pcb_allocs == 3, "abort: %d of %d pcbs freed",
          s_pcb_frees, s_pcb_allocs);
    CHECK(s_mem_live == 0 && s_pbuf_live == 0, "abort: leaked mem %d pbuf %d", s_mem_live, s_pbuf_live);
    CHECK(s_tcp_aborts == 1 && s_app_connected == 0, "abort: tcp aborts %d", s_tcp_aborts);
    CHECK(s_app_err_calls == 1 && s_app_err == ERR_ABRT, "abort: app err %d calls", s_app_err_calls);

    /* after the 200, in the application phase */
    setup(NULL);
    CHECK(deliver("HTTP/1.1 200 OK\r\n\r\n") == ERR_OK && s_app_connected == 1, "200");
    altcp_abort(s_tls);
    CHECK(s_bad_free == 0 && s_pcb_frees == 3 && s_mem_live == 0, "abort after 200: bad %d freed %d",
          s_bad_free, s_pcb_frees);
}

/* cloud_client aborts the bare proxy pcb (nothing above it yet) when it cannot build the TLS
   config: the CA does not parse, or out of memory. */
static void test_abort_bare_proxy(void) {
    s_pcb_allocs = s_pcb_frees = s_mem_live = s_bad_free = s_tcp_aborts = 0;
    memset(s_live, 0, sizeof(s_live));
    memset(&s_conf, 0, sizeof(s_conf));
    s_conf.proxy_port = 3128;
    s_conf.target_host = "embeddedci.com";
    struct altcp_pcb *p = altcp_bp_proxy_new_tcp(&s_conf, IPADDR_TYPE_V4);
    CHECK(p != NULL, "stack not built");
    altcp_abort(p);
    CHECK(s_bad_free == 0, "bare abort: %d bad frees", s_bad_free);
    CHECK(s_pcb_frees == 2 && s_pcb_allocs == 2, "bare abort: %d of %d freed", s_pcb_frees, s_pcb_allocs);
    CHECK(s_mem_live == 0 && s_tcp_aborts == 1, "bare abort: mem %d aborts %d", s_mem_live, s_tcp_aborts);
}

int main(void) {
    test_request();
    test_ok_split();
    test_refused();
    test_closed_early();
    test_reply_shapes();
    test_not_http();
    test_app_abort_during_setup();
    test_abort_bare_proxy();
    if (failures) { printf("test_altcp_bp_proxy: %d FAILED\n", failures); return 1; }
    printf("test_altcp_bp_proxy: all passed\n");
    return 0;
}
