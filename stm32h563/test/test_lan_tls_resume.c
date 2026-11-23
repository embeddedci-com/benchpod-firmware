/*
 * test_lan_tls_resume.c — LAN TLS session resumption and pairing (src/lan_tls_resume.c,
 * docs/design/lan-pairing.md). A real mbedTLS server (the pod's configuration: client certificate
 * optional, no CA, session cache and tickets, KEEP_PEER_CERTIFICATE) and client talk through
 * memory pipes. The pairing verdict is computed like net_server.c does it: SHA-256 of the peer
 * certificate's public key, looked up in a lan_clients list, on every "command".
 */
#define MBEDTLS_ALLOW_PRIVATE_ACCESS
#include "lan_clients.h"
#include "lan_tls_resume.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/entropy.h"
#include "mbedtls/pk.h"
#include "mbedtls/sha256.h"
#include "mbedtls/ssl.h"
#include "mbedtls/ssl_cache.h"
#include "mbedtls/ssl_ticket.h"
#include "mbedtls/x509_crt.h"
#include "mbedtls/x509_csr.h"
#include "mbedtls/bignum.h"

static int failures;
#define CHECK(cond, ...) do { if (!(cond)) { printf("FAIL %s:%d: ", __func__, __LINE__); \
        printf(__VA_ARGS__); printf("\n"); failures++; } } while (0)

/* ---- platform hooks of mbedtls_bench_config.h ---- */
void *bench_mbedtls_calloc(size_t n, size_t s) { return calloc(n, s); }
void bench_mbedtls_free(void *p) { free(p); }
long bench_mbedtls_time(long *t) { long v = (long)time(NULL); if (t) *t = v; return v; }
int mbedtls_hardware_poll(void *d, unsigned char *o, size_t l, size_t *ol) {
    (void)d; for (size_t i = 0; i < l; i++) o[i] = (unsigned char)rand(); *ol = l; return 0;
}
mbedtls_ms_time_t mbedtls_ms_time(void) {
    struct timeval tv; gettimeofday(&tv, NULL); return (mbedtls_ms_time_t)tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

/* ---- memory pipes ---- */
typedef struct { unsigned char b[2][24000]; size_t n[2]; } pipe_t;
typedef struct { pipe_t *p; int rd; } io_t;   /* rd: the direction this side reads (1 = server->client) */
static int io_send(void *c, const unsigned char *b, size_t l) {
    io_t *i = c; int w = 1 - i->rd; memcpy(i->p->b[w] + i->p->n[w], b, l); i->p->n[w] += l; return (int)l;
}
static int io_recv(void *c, unsigned char *b, size_t l) {
    io_t *i = c; int d = i->rd; if (!i->p->n[d]) return MBEDTLS_ERR_SSL_WANT_READ;
    size_t k = l < i->p->n[d] ? l : i->p->n[d]; memcpy(b, i->p->b[d], k);
    memmove(i->p->b[d], i->p->b[d] + k, i->p->n[d] - k); i->p->n[d] -= k; return (int)k;
}

static mbedtls_ctr_drbg_context drbg;
static mbedtls_entropy_context ent;
typedef struct { mbedtls_x509_crt crt; mbedtls_pk_context key; } ident_t;

static void make_ident(ident_t *id, const char *cn) {
    mbedtls_x509_crt_init(&id->crt); mbedtls_pk_init(&id->key);
    mbedtls_x509write_cert w; mbedtls_mpi serial; unsigned char buf[2048]; char subj[64];
    mbedtls_pk_setup(&id->key, mbedtls_pk_info_from_type(MBEDTLS_PK_ECKEY));
    mbedtls_ecp_gen_key(MBEDTLS_ECP_DP_SECP256R1, mbedtls_pk_ec(id->key), mbedtls_ctr_drbg_random, &drbg);
    mbedtls_x509write_crt_init(&w); mbedtls_mpi_init(&serial); mbedtls_mpi_lset(&serial, 1);
    snprintf(subj, sizeof subj, "CN=%s", cn);
    mbedtls_x509write_crt_set_subject_key(&w, &id->key); mbedtls_x509write_crt_set_issuer_key(&w, &id->key);
    mbedtls_x509write_crt_set_subject_name(&w, subj); mbedtls_x509write_crt_set_issuer_name(&w, subj);
    mbedtls_x509write_crt_set_version(&w, MBEDTLS_X509_CRT_VERSION_3);
    mbedtls_x509write_crt_set_md_alg(&w, MBEDTLS_MD_SHA256);
    mbedtls_x509write_crt_set_serial(&w, &serial);
    mbedtls_x509write_crt_set_validity(&w, "20260101000000", "20360101000000");
    int n = mbedtls_x509write_crt_der(&w, buf, sizeof buf, mbedtls_ctr_drbg_random, &drbg);
    CHECK(n > 0, "cert write %d", n);
    mbedtls_x509_crt_parse_der(&id->crt, buf + sizeof buf - n, (size_t)n);
    mbedtls_x509write_crt_free(&w); mbedtls_mpi_free(&serial);
}

static void key_fp(const mbedtls_x509_crt *c, uint8_t fp[32]) {   /* as net_server.c conn_note_client_key */
    mbedtls_sha256(c->pk_raw.p, c->pk_raw.len, fp, 0);
}

/* ---- the pod's server configuration ---- */
static mbedtls_ssl_config sconf;
static mbedtls_ssl_cache_context cache;
static mbedtls_ssl_ticket_context tickets;
static int verify_calls;   /* the server parsed a client certificate: a full handshake with one */
static int vf(void *a, mbedtls_x509_crt *c, int d, uint32_t *f) { (void)a; (void)c; (void)d; (void)f; verify_calls++; return 0; }

static void server_setup(ident_t *srv, int use_cache, int use_tickets) {
    mbedtls_ssl_config_init(&sconf);
    mbedtls_ssl_config_defaults(&sconf, MBEDTLS_SSL_IS_SERVER, MBEDTLS_SSL_TRANSPORT_STREAM, 0);
    mbedtls_ssl_conf_rng(&sconf, mbedtls_ctr_drbg_random, &drbg);
    mbedtls_ssl_conf_own_cert(&sconf, &srv->crt, &srv->key);
    mbedtls_ssl_conf_ca_chain(&sconf, NULL, NULL);
    mbedtls_ssl_conf_authmode(&sconf, MBEDTLS_SSL_VERIFY_OPTIONAL);
    mbedtls_ssl_conf_verify(&sconf, vf, NULL);
    mbedtls_ssl_cache_init(&cache); mbedtls_ssl_ticket_init(&tickets);
    if (use_cache) mbedtls_ssl_conf_session_cache(&sconf, &cache, lan_tls_cache_get, mbedtls_ssl_cache_set);
    if (use_tickets) {
        mbedtls_ssl_ticket_setup(&tickets, mbedtls_ctr_drbg_random, &drbg, MBEDTLS_CIPHER_AES_256_GCM, 3600);
        mbedtls_ssl_conf_session_tickets_cb(&sconf, mbedtls_ssl_ticket_write, lan_tls_ticket_parse, &tickets);
    }
}
static void server_free(void) {
    mbedtls_ssl_config_free(&sconf); mbedtls_ssl_cache_free(&cache); mbedtls_ssl_ticket_free(&tickets);
}

/* What the server saw on one connection. */
typedef struct { int ok; int cert; uint8_t fp[32]; } seen_t;

/* One connection. `id` is the client's certificate (NULL: none); `resume` is a session to offer;
   `keep` receives the client's session afterwards. */
static seen_t connect_once(ident_t *id, mbedtls_ssl_session *resume, mbedtls_ssl_session *keep) {
    seen_t r = {0};
    mbedtls_ssl_config cconf; mbedtls_ssl_config_init(&cconf);
    mbedtls_ssl_config_defaults(&cconf, MBEDTLS_SSL_IS_CLIENT, MBEDTLS_SSL_TRANSPORT_STREAM, 0);
    mbedtls_ssl_conf_rng(&cconf, mbedtls_ctr_drbg_random, &drbg);
    mbedtls_ssl_conf_authmode(&cconf, MBEDTLS_SSL_VERIFY_NONE);
    if (id) mbedtls_ssl_conf_own_cert(&cconf, &id->crt, &id->key);
    pipe_t *p = calloc(1, sizeof *p); io_t ci = { p, 1 }, si = { p, 0 };
    mbedtls_ssl_context c, s; mbedtls_ssl_init(&c); mbedtls_ssl_init(&s);
    mbedtls_ssl_setup(&c, &cconf); mbedtls_ssl_setup(&s, &sconf);
    mbedtls_ssl_set_bio(&c, &ci, io_send, io_recv, NULL); mbedtls_ssl_set_bio(&s, &si, io_send, io_recv, NULL);
    if (resume && mbedtls_ssl_set_session(&c, resume) != 0) { printf("set_session failed\n"); failures++; }
    int rc = 0, rs = 0;
    for (int i = 0; i < 100 && (c.state != MBEDTLS_SSL_HANDSHAKE_OVER || s.state != MBEDTLS_SSL_HANDSHAKE_OVER); i++) {
        if (c.state != MBEDTLS_SSL_HANDSHAKE_OVER) rc = mbedtls_ssl_handshake(&c);
        if (s.state != MBEDTLS_SSL_HANDSHAKE_OVER) rs = mbedtls_ssl_handshake(&s);
        if ((rc && rc != MBEDTLS_ERR_SSL_WANT_READ && rc != MBEDTLS_ERR_SSL_WANT_WRITE) ||
            (rs && rs != MBEDTLS_ERR_SSL_WANT_READ && rs != MBEDTLS_ERR_SSL_WANT_WRITE)) break;
    }
    if (c.state == MBEDTLS_SSL_HANDSHAKE_OVER && s.state == MBEDTLS_SSL_HANDSHAKE_OVER) {
        unsigned char x[4];   /* application data: the client also reads the server's last flight */
        mbedtls_ssl_write(&s, (const unsigned char *)"abc", 3); mbedtls_ssl_read(&c, x, 3);
        r.ok = 1;
        const mbedtls_x509_crt *peer = mbedtls_ssl_get_peer_cert(&s);
        if (peer) { r.cert = 1; key_fp(peer, r.fp); }
        if (keep) { mbedtls_ssl_session_free(keep); mbedtls_ssl_session_init(keep); mbedtls_ssl_get_session(&c, keep); }
    }
    mbedtls_ssl_free(&c); mbedtls_ssl_free(&s); mbedtls_ssl_config_free(&cconf); free(p);
    return r;
}

/* The pairing verdict of net_conn_lan_paired(): a hash, looked up in the current list. */
static int paired(const seen_t *s, const lan_clients_t *l) { return s->ok && s->cert && lan_clients_find(l, s->fp) >= 0; }

static void run_mode(const char *name, int use_cache, int use_tickets) {
    ident_t srv, alice, bob; make_ident(&srv, "pod"); make_ident(&alice, "alice"); make_ident(&bob, "bob");
    server_setup(&srv, use_cache, use_tickets);
    lan_clients_t list; lan_clients_init(&list, 8);
    uint8_t fa[32]; key_fp(&alice.crt, fa);
    lan_clients_add(&list, fa, "alice");
    mbedtls_ssl_session ses; mbedtls_ssl_session_init(&ses);
    int v0, v1;

    /* 1. Full handshake with a paired certificate. */
    seen_t a = connect_once(&alice, NULL, &ses);
    CHECK(a.ok && a.cert && paired(&a, &list), "%s: full handshake, paired computer", name);
    v0 = verify_calls;

    /* 2. Resumed: no certificate is sent again, the server still has the original one, still paired. */
    seen_t b = connect_once(&alice, &ses, &ses);
    v1 = verify_calls;
    CHECK(b.ok && v1 == v0, "%s: the second connection resumed (no client certificate parsed)", name);
    CHECK(b.cert && !memcmp(b.fp, a.fp, 32), "%s: resumed session keeps the peer certificate", name);
    CHECK(paired(&b, &list), "%s: resumed session is paired", name);
    seen_t b2 = connect_once(&alice, &ses, &ses);   /* and a session that was itself resumed */
    CHECK(b2.ok && verify_calls == v1 && paired(&b2, &list), "%s: resumed twice, still paired", name);

    /* 3. The computer is removed: the same resumed session is unpaired on its next command. */
    CHECK(lan_clients_remove(&list, "alice") >= 0, "remove");
    CHECK(!paired(&b2, &list), "%s: removed computer is not paired (hash re-checked, not cached)", name);
    seen_t b3 = connect_once(&alice, &ses, &ses);
    CHECK(b3.ok && !paired(&b3, &list), "%s: resuming after removal: not paired", name);
    lan_clients_add(&list, fa, "alice");
    seen_t b4 = connect_once(&alice, &ses, &ses);
    CHECK(paired(&b4, &list), "%s: paired again after it is re-added", name);
    mbedtls_ssl_session_free(&ses);

    /* 4. A session made WITHOUT a certificate must not be resumed after the client got one: the
          server forces a full handshake, which carries the certificate. */
    mbedtls_ssl_session_init(&ses);
    seen_t n = connect_once(NULL, NULL, &ses);
    CHECK(n.ok && !n.cert && !paired(&n, &list), "%s: no certificate: connects, unpaired", name);
    v0 = verify_calls;
    seen_t c = connect_once(&alice, &ses, &ses);
    CHECK(c.ok && verify_calls == v0 + 1, "%s: certificate-less session not resumed (full handshake)", name);
    CHECK(c.cert && paired(&c, &list), "%s: the same client with a certificate is paired", name);
    /* and that session (with a certificate now) resumes and stays paired */
    v0 = verify_calls;
    seen_t d = connect_once(&alice, &ses, &ses);
    CHECK(d.ok && verify_calls == v0 && paired(&d, &list), "%s: later session resumes, paired", name);
    mbedtls_ssl_session_free(&ses);

    /* 5. A certificate that is not on the list: connects, never paired, resumed or not. */
    mbedtls_ssl_session_init(&ses);
    seen_t e = connect_once(&bob, NULL, &ses);
    seen_t e2 = connect_once(&bob, &ses, &ses);
    CHECK(e.ok && e2.ok && !paired(&e, &list) && !paired(&e2, &list), "%s: unknown key is never paired", name);
    mbedtls_ssl_session_free(&ses);

    /* 6. A client with no certificate at all: always works, always unpaired. */
    mbedtls_ssl_session_init(&ses);
    seen_t f = connect_once(NULL, NULL, &ses);
    seen_t f2 = connect_once(NULL, &ses, &ses);
    CHECK(f.ok && f2.ok && !paired(&f, &list) && !paired(&f2, &list), "%s: no certificate: unpaired", name);
    mbedtls_ssl_session_free(&ses);
    server_free();
}

int main(void) {
    mbedtls_ctr_drbg_init(&drbg); mbedtls_entropy_init(&ent);
    if (mbedtls_ctr_drbg_seed(&drbg, mbedtls_entropy_func, &ent, NULL, 0)) { puts("drbg"); return 1; }
    run_mode("tickets+cache", 1, 1);
    run_mode("tickets", 0, 1);
    run_mode("cache", 1, 0);
    if (failures) { printf("test_lan_tls_resume: %d FAILED\n", failures); return 1; }
    puts("test_lan_tls_resume: PASS");
    return 0;
}
