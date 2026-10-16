/*
 * lan_tls.c — the LAN TLS server certificate: store, generate, check, and the listener's
 * mbedTLS configuration (lan_tls.h, docs/design/lan-tls.md).
 */
#include "lan_tls.h"
#include "lan_tls_pem.h"
#include "config_store.h"
#include "device_identity.h"
#include "net_server.h"
#include "ota.h"
#include "bp_log.h"

#include "lwip/altcp_tls.h"

#include "mbedtls/ctr_drbg.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ecp.h"
#include "mbedtls/oid.h"
#include "mbedtls/pk.h"
#include "mbedtls/sha256.h"
#include "mbedtls/x509_crt.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include <string.h>

/* ---- the record ---------------------------------------------------------------------------- */

#define ORIGIN_GENERATED 1u
#define ORIGIN_USER      2u

typedef struct {
    uint8_t  origin;      /* ORIGIN_* */
    uint8_t  reserved;
    uint16_t key_len;     /* key PEM bytes, including its NUL */
    uint16_t cert_len;    /* certificate PEM bytes, including its NUL */
    uint16_t reserved2;
} rec_hdr_t;              /* followed by the key PEM, then the certificate PEM */

static const ab_store_t k_store = {
    .tag = "tls",
    .slot_off = { LAN_TLS_SLOT_A_OFFSET, LAN_TLS_SLOT_B_OFFSET },
    .magic = LAN_TLS_RECORD_MAGIC,
    .version = 1,
    .max_len = LAN_TLS_RECORD_MAX,
};

/* ---- state (net task) ---------------------------------------------------------------------- */

#define RETIRED_MAX 3
typedef struct {
    struct altcp_tls_config *cfg;
    int refs;
} cfg_slot_t;

static cfg_slot_t        s_cur;                    /* new connections get this one */
static cfg_slot_t        s_retired[RETIRED_MAX];   /* replaced, freed when refs reach 0 */
static volatile uint32_t s_gen;

static lan_tls_info_t    s_info;
static volatile uint32_t s_info_seq;               /* odd while s_info is being written */

static void info_publish(const lan_tls_info_t *in)
{
    s_info_seq++;
    __asm volatile("" ::: "memory");
    s_info = *in;
    __asm volatile("" ::: "memory");
    s_info_seq++;
}

void lan_tls_get_info(lan_tls_info_t *out)
{
    for (int tries = 0; tries < 100; tries++) {
        uint32_t a = s_info_seq;
        __asm volatile("" ::: "memory");
        if (a & 1u) { vTaskDelay(1); continue; }
        *out = s_info;
        __asm volatile("" ::: "memory");
        if (s_info_seq == a) return;
    }
    memset(out, 0, sizeof(*out));
    snprintf(out->error, sizeof(out->error), "busy");
}

/* ---- helpers ------------------------------------------------------------------------------- */

/* A DRBG seeded from the TRNG, for key generation, signing and the pair check. */
typedef struct {
    mbedtls_entropy_context  ent;
    mbedtls_ctr_drbg_context drbg;
} rng_t;

static int rng_open(rng_t *r)
{
    mbedtls_entropy_init(&r->ent);
    mbedtls_ctr_drbg_init(&r->drbg);
    static const char pers[] = "benchpod-lan-tls";
    return mbedtls_ctr_drbg_seed(&r->drbg, mbedtls_entropy_func, &r->ent,
                                 (const unsigned char *)pers, sizeof(pers) - 1);
}

static void rng_close(rng_t *r)
{
    mbedtls_ctr_drbg_free(&r->drbg);
    mbedtls_entropy_free(&r->ent);
}

static void pod_name(char out[24])
{
    char id[DEVICE_ID_SHORT_LEN + 1];
    device_identity_short_id(id);
    snprintf(out, 24, "benchpod-%s", id[0] ? id : "unknown");
}

/* The info for a parsed leaf certificate and key. */
static void info_from(lan_tls_info_t *in, const mbedtls_x509_crt *crt, const mbedtls_pk_context *key,
                      uint8_t origin)
{
    memset(in, 0, sizeof(*in));
    in->ready = true;
    snprintf(in->origin, sizeof(in->origin), "%s", origin == ORIGIN_GENERATED ? "generated" : "user");
    if (mbedtls_pk_get_type(key) == MBEDTLS_PK_ECKEY || mbedtls_pk_get_type(key) == MBEDTLS_PK_ECDSA)
        snprintf(in->key, sizeof(in->key), "EC P-%u", (unsigned)mbedtls_pk_get_bitlen(key));
    else
        snprintf(in->key, sizeof(in->key), "%s %u", mbedtls_pk_get_name(key), (unsigned)mbedtls_pk_get_bitlen(key));
    if (mbedtls_x509_dn_gets(in->subject, sizeof(in->subject), &crt->subject) < 0) in->subject[0] = '\0';
    for (const mbedtls_x509_sequence *s = &crt->subject_alt_names; s && s->buf.p && in->nsan < 3; s = s->next) {
        mbedtls_x509_subject_alternative_name san;
        if (mbedtls_x509_parse_subject_alt_name(&s->buf, &san) != 0) continue;
        if (san.type == MBEDTLS_X509_SAN_DNS_NAME) {
            size_t n = san.san.unstructured_name.len;
            if (n >= sizeof(in->san[0])) n = sizeof(in->san[0]) - 1;
            memcpy(in->san[in->nsan], san.san.unstructured_name.p, n);
            in->san[in->nsan][n] = '\0';
            in->nsan++;
        }
        mbedtls_x509_free_subject_alt_name(&san);
    }
    snprintf(in->not_after, sizeof(in->not_after), "%04d-%02d-%02d",
             crt->valid_to.year, crt->valid_to.mon, crt->valid_to.day);
    uint8_t h[32];
    mbedtls_sha256(crt->raw.p, crt->raw.len, h, 0);
    for (int i = 0; i < 32; i++) snprintf(in->sha256 + 2 * i, 3, "%02x", h[i]);
}

/* Parse key + certificate and check they belong together. NULL = fine; info filled when given. */
static const char *check_pair(const char *key, size_t key_len, const char *cert, size_t cert_len,
                              uint8_t origin, lan_tls_info_t *info)
{
    mbedtls_x509_crt crt;
    mbedtls_pk_context pk;
    rng_t r;
    mbedtls_x509_crt_init(&crt);
    mbedtls_pk_init(&pk);
    const char *why = NULL;
    if (rng_open(&r) != 0) { rng_close(&r); return "no random numbers"; }
    if (mbedtls_x509_crt_parse(&crt, (const unsigned char *)cert, cert_len) != 0)
        why = "the certificate does not parse";
    else if (mbedtls_pk_parse_key(&pk, (const unsigned char *)key, key_len, NULL, 0,
                                  mbedtls_ctr_drbg_random, &r.drbg) != 0)
        why = "the private key does not parse";
    else if (!mbedtls_pk_can_do(&pk, MBEDTLS_PK_ECDSA) && !mbedtls_pk_can_do(&pk, MBEDTLS_PK_RSA))
        why = "the key must be EC or RSA";
    else if (mbedtls_pk_check_pair(&crt.pk, &pk, mbedtls_ctr_drbg_random, &r.drbg) != 0)
        why = "the private key does not match the certificate";
    if (!why && info) info_from(info, &crt, &pk, origin);
    mbedtls_x509_crt_free(&crt);
    mbedtls_pk_free(&pk);
    rng_close(&r);
    return why;
}

/* The record goes through a heap buffer of LAN_TLS_RECORD_MAX bytes while it is built, stored or
   parsed (static RAM is short; mbedTLS copies what it keeps). */
static uint8_t *rec_alloc(void) { return pvPortMalloc(LAN_TLS_RECORD_MAX); }

static void rec_free(uint8_t *rec)
{
    if (!rec) return;
    memset(rec, 0, LAN_TLS_RECORD_MAX);   /* it held a private key */
    vPortFree(rec);
}

static size_t rec_build(uint8_t *rec, uint8_t origin, const char *key, size_t key_len,
                        const char *cert, size_t cert_len)
{
    rec_hdr_t h = { .origin = origin, .key_len = (uint16_t)key_len, .cert_len = (uint16_t)cert_len };
    size_t n = sizeof(h) + key_len + cert_len;
    if (n > LAN_TLS_RECORD_MAX) return 0;
    memcpy(rec, &h, sizeof(h));
    memcpy(rec + sizeof(h), key, key_len);
    memcpy(rec + sizeof(h) + key_len, cert, cert_len);
    return n;
}

/* Make cfg the listener's configuration; the old one retires. */
static void cfg_swap(struct altcp_tls_config *cfg)
{
    if (s_cur.cfg) {
        int i;
        for (i = 0; i < RETIRED_MAX; i++) if (!s_retired[i].cfg) break;
        if (i == RETIRED_MAX) {
            /* No free retire slot: four changes while old sessions linger. Keep the oldest
               (leak rather than free under a live session); this does not happen in practice. */
            log_printf("[tls] too many retired configurations, one is leaked\n");
            i = 0;
        }
        s_retired[i] = s_cur;
    }
    s_cur.cfg = cfg;
    s_cur.refs = 0;
    s_gen++;
}

/* Use a stored record (rec, n bytes): build a configuration and publish its info. */
static const char *apply_record(const uint8_t *rec, size_t n)
{
    rec_hdr_t h;
    if (n < sizeof(h)) return "the stored record is damaged";
    memcpy(&h, rec, sizeof(h));
    if (sizeof(h) + (size_t)h.key_len + h.cert_len != n || !h.key_len || !h.cert_len)
        return "the stored record is damaged";
    const char *key = (const char *)rec + sizeof(h);
    const char *cert = key + h.key_len;
    if (key[h.key_len - 1] != '\0' || cert[h.cert_len - 1] != '\0') return "the stored record is damaged";

    lan_tls_info_t in;
    const char *why = check_pair(key, h.key_len, cert, h.cert_len, h.origin, &in);
    if (why) return why;
    struct altcp_tls_config *cfg = altcp_tls_create_config_server_privkey_cert(
        (const u8_t *)key, h.key_len, NULL, 0, (const u8_t *)cert, h.cert_len);
    if (!cfg) return "out of memory for the TLS configuration";
    cfg_swap(cfg);
    info_publish(&in);
    return NULL;
}

/* Generate a P-256 key and a self-signed certificate into rec. Returns its length, 0 on error. */
static size_t generate(uint8_t *rec)
{
    char name[24], cn[32], local[32];
    pod_name(name);
    snprintf(cn, sizeof(cn), "CN=%s", name);
    snprintf(local, sizeof(local), "%s.local", name);

    enum { KEY_PEM_MAX = 300, CRT_PEM_MAX = 1200 };
    char *key_pem = pvPortMalloc(KEY_PEM_MAX + CRT_PEM_MAX);
    if (!key_pem) return 0;
    char *crt_pem = key_pem + KEY_PEM_MAX;
    size_t n = 0;
    rng_t r;
    mbedtls_pk_context pk;
    mbedtls_x509write_cert w;
    mbedtls_pk_init(&pk);
    mbedtls_x509write_crt_init(&w);
    TickType_t t0 = xTaskGetTickCount();
    if (rng_open(&r) != 0) goto out;
    if (mbedtls_pk_setup(&pk, mbedtls_pk_info_from_type(MBEDTLS_PK_ECKEY)) != 0) goto out;
    if (mbedtls_ecp_gen_key(MBEDTLS_ECP_DP_SECP256R1, mbedtls_pk_ec(pk), mbedtls_ctr_drbg_random, &r.drbg) != 0)
        goto out;

    uint8_t serial[8];
    if (mbedtls_ctr_drbg_random(&r.drbg, serial, sizeof(serial)) != 0) goto out;
    serial[0] = (uint8_t)((serial[0] & 0x7Fu) | 0x40u);   /* positive, full length */

    mbedtls_x509_san_list san2 = { .node = { .type = MBEDTLS_X509_SAN_DNS_NAME,
        .san.unstructured_name = { .p = (unsigned char *)local, .len = strlen(local) } }, .next = NULL };
    mbedtls_x509_san_list san1 = { .node = { .type = MBEDTLS_X509_SAN_DNS_NAME,
        .san.unstructured_name = { .p = (unsigned char *)name, .len = strlen(name) } }, .next = &san2 };
    mbedtls_asn1_sequence eku = { .buf = { .tag = MBEDTLS_ASN1_OID,
        .len = MBEDTLS_OID_SIZE(MBEDTLS_OID_SERVER_AUTH), .p = (unsigned char *)MBEDTLS_OID_SERVER_AUTH }, .next = NULL };

    mbedtls_x509write_crt_set_version(&w, MBEDTLS_X509_CRT_VERSION_3);
    mbedtls_x509write_crt_set_md_alg(&w, MBEDTLS_MD_SHA256);
    mbedtls_x509write_crt_set_subject_key(&w, &pk);
    mbedtls_x509write_crt_set_issuer_key(&w, &pk);
    if (mbedtls_x509write_crt_set_subject_name(&w, cn) != 0 ||
        mbedtls_x509write_crt_set_issuer_name(&w, cn) != 0 ||
        mbedtls_x509write_crt_set_serial_raw(&w, serial, sizeof(serial)) != 0 ||
        mbedtls_x509write_crt_set_validity(&w, "20250101000000", "99991231235959") != 0 ||
        mbedtls_x509write_crt_set_basic_constraints(&w, 0, -1) != 0 ||
        mbedtls_x509write_crt_set_key_usage(&w, MBEDTLS_X509_KU_DIGITAL_SIGNATURE) != 0 ||
        mbedtls_x509write_crt_set_ext_key_usage(&w, &eku) != 0 ||
        mbedtls_x509write_crt_set_subject_alternative_name(&w, &san1) != 0)
        goto out;
    if (mbedtls_x509write_crt_pem(&w, (unsigned char *)crt_pem, CRT_PEM_MAX, mbedtls_ctr_drbg_random, &r.drbg) != 0)
        goto out;
    if (mbedtls_pk_write_key_pem(&pk, (unsigned char *)key_pem, KEY_PEM_MAX) != 0) goto out;
    n = rec_build(rec, ORIGIN_GENERATED, key_pem, strlen(key_pem) + 1, crt_pem, strlen(crt_pem) + 1);
    log_printf("[tls] generated an EC P-256 key and a certificate for %s in %lu ms\n", name,
               (unsigned long)((xTaskGetTickCount() - t0) * portTICK_PERIOD_MS));
out:
    mbedtls_x509write_crt_free(&w);
    mbedtls_pk_free(&pk);
    rng_close(&r);
    memset(key_pem, 0, KEY_PEM_MAX);
    vPortFree(key_pem);
    return n;
}

static void publish_error(const char *why)
{
    lan_tls_info_t in;
    memset(&in, 0, sizeof(in));
    snprintf(in.error, sizeof(in.error), "%s", why);
    info_publish(&in);
}

/* Generate, store and use a new certificate (net task). */
static const char *regenerate_here(void)
{
    uint8_t *rec = rec_alloc();
    if (!rec) return "out of memory";
    const char *why = NULL;
    size_t n = generate(rec);
    if (!n) why = "could not generate a key and certificate";
    else if (ab_store_save(&k_store, rec, n) != 0) why = "could not save the certificate";
    else why = apply_record(rec, n);
    rec_free(rec);
    return why;
}

/* ---- public -------------------------------------------------------------------------------- */

void lan_tls_init(void)
{
    size_t n = 0;
    const char *why = "none stored";
    uint8_t *rec = rec_alloc();
    if (!rec) why = "out of memory";
    else if (ab_store_load_var(&k_store, rec, LAN_TLS_RECORD_MAX, &n) == 0) {
        why = apply_record(rec, n);
        if (why) log_printf("[tls] the stored certificate is not usable (%s)\n", why);
    }
    rec_free(rec);
    if (why) {
        /* None stored, or a damaged one: a pod must never be left without TLS. */
        why = regenerate_here();
        if (why) {
            log_printf("[tls] no certificate: %s\n", why);
            publish_error(why);
            return;
        }
    }
    lan_tls_info_t in;
    lan_tls_get_info(&in);
    log_printf("[tls] port %d: %s %s, %s, sha256 %.16s...\n", LAN_TLS_PORT, in.origin, in.key, in.subject, in.sha256);
}

struct altcp_tls_config *lan_tls_config(void) { return s_cur.cfg; }
uint32_t lan_tls_generation(void) { return s_gen; }

void lan_tls_ref(struct altcp_tls_config *cfg)
{
    if (cfg && cfg == s_cur.cfg) { s_cur.refs++; return; }
    for (int i = 0; i < RETIRED_MAX; i++) if (cfg && s_retired[i].cfg == cfg) { s_retired[i].refs++; return; }
}

void lan_tls_unref(struct altcp_tls_config *cfg)
{
    if (cfg && cfg == s_cur.cfg) { if (s_cur.refs > 0) s_cur.refs--; return; }
    for (int i = 0; i < RETIRED_MAX; i++)
        if (cfg && s_retired[i].cfg == cfg) { if (s_retired[i].refs > 0) s_retired[i].refs--; return; }
}

/* Not from the connection callbacks: lwIP frees the TLS state (which still points at its
   configuration) right after the error callback returns. net_poll calls this. */
void lan_tls_collect(void)
{
    for (int i = 0; i < RETIRED_MAX; i++) {
        if (s_retired[i].cfg && s_retired[i].refs == 0) {
            altcp_tls_free_config(s_retired[i].cfg);
            s_retired[i].cfg = NULL;
        }
    }
}

/* net_call_sync glue */
typedef struct {
    const uint8_t *pem;
    size_t len;
    bool store;
    const char *why;
} job_t;

#define KEY_BUF_MAX 3300u   /* an RSA 4096 key PEM is ~3.2 KB */

static void install_job(void *arg)
{
    job_t *j = arg;
    size_t kl = 0, cl = 0;
    /* key | certificates | record, one allocation */
    uint8_t *mem = pvPortMalloc(KEY_BUF_MAX + LAN_TLS_UPLOAD_MAX + LAN_TLS_RECORD_MAX);
    if (!mem) { j->why = "out of memory"; return; }
    char *key = (char *)mem, *crt = key + KEY_BUF_MAX;
    uint8_t *rec = mem + KEY_BUF_MAX + LAN_TLS_UPLOAD_MAX;
    j->why = lan_tls_pem_split((const char *)j->pem, j->len, key, KEY_BUF_MAX, &kl, crt, LAN_TLS_UPLOAD_MAX, &cl);
    if (!j->why) j->why = check_pair(key, kl, crt, cl, ORIGIN_USER, NULL);
    if (!j->why && j->store) {
        size_t n = rec_build(rec, ORIGIN_USER, key, kl, crt, cl);
        if (!n) j->why = "the key and certificates are too large";
        else if (ab_store_save(&k_store, rec, n) != 0) j->why = "could not save the certificate";
        else j->why = apply_record(rec, n);
        if (!j->why) log_printf("[tls] installed a user certificate\n");
    }
    memset(mem, 0, KEY_BUF_MAX + LAN_TLS_UPLOAD_MAX + LAN_TLS_RECORD_MAX);
    vPortFree(mem);
}

static void regen_job(void *arg)
{
    job_t *j = arg;
    j->why = regenerate_here();
}

static const char *run(void (*fn)(void *), job_t *j)
{
    if (!net_call_sync(fn, j, 15000)) return "the network task did not answer";
    return j->why;
}

const char *lan_tls_regenerate(void)
{
    job_t j = { 0 };
    return run(regen_job, &j);
}

const char *lan_tls_install(const uint8_t *pem, size_t len)
{
    if (len > LAN_TLS_UPLOAD_MAX) return "the upload is too large";
    job_t j = { .pem = pem, .len = len, .store = true };
    return run(install_job, &j);
}

const char *lan_tls_check(const uint8_t *pem, size_t len)
{
    if (len > LAN_TLS_UPLOAD_MAX) return "the upload is too large";
    job_t j = { .pem = pem, .len = len, .store = false };
    return run(install_job, &j);
}

/* ---- the upload path (ota.h, worker task) -------------------------------------------------- */

/* The staged bundle into a heap buffer (NULL with why filled on failure). */
static uint8_t *read_staged(uint32_t size, char *why, size_t cap)
{
    if (size == 0 || size > LAN_TLS_UPLOAD_MAX) { snprintf(why, cap, "lan_tls: upload larger than %u bytes", (unsigned)LAN_TLS_UPLOAD_MAX); return NULL; }
    uint8_t *buf = pvPortMalloc(size);
    if (!buf) { snprintf(why, cap, "lan_tls: out of memory"); return NULL; }
    if (ota_read_staged(0, buf, size) != 0) {
        vPortFree(buf);
        snprintf(why, cap, "lan_tls: psram read failed");
        return NULL;
    }
    return buf;
}

static void drop_staged(uint8_t *buf, uint32_t size)
{
    memset(buf, 0, size);   /* it held a private key */
    vPortFree(buf);
}

int lan_tls_validate_staged(uint32_t size, char *why, size_t cap)
{
    uint8_t *buf = read_staged(size, why, cap);
    if (!buf) return -1;
    const char *bad = lan_tls_check(buf, size);
    drop_staged(buf, size);
    if (bad) { snprintf(why, cap, "lan_tls: %s", bad); return -1; }
    return 0;
}

int ota_install_staged_config(ota_target_t t, uint32_t size, char *why, size_t cap)
{
    if (t != OTA_TARGET_TLS) { snprintf(why, cap, "not supported"); return -1; }
    uint8_t *buf = read_staged(size, why, cap);
    if (!buf) return -1;
    const char *bad = lan_tls_install(buf, size);
    drop_staged(buf, size);
    if (bad) { snprintf(why, cap, "lan_tls: %s", bad); return -1; }
    return 0;
}
