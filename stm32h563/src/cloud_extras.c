/*
 * cloud_extras.c — company CA and HTTP proxy for the cloud link (see cloud_extras.h).
 */
#include "cloud_extras.h"
#include "blob_store.h"
#include "w25q.h"
#include "cloud_ca.h"
#include "net_server.h"     /* net_cloud_reload: reconnect with the new settings */
#include "ota.h"            /* ota_validate_staged: the CA upload is checked before it is accepted */

#include "FreeRTOS.h"
#include "task.h"
#include "mbedtls/x509_crt.h"
#include "mbedtls/pem.h"        /* MBEDTLS_ERR_PEM_ALLOC_FAILED */
#include "mbedtls/sha256.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* The company CA as installed (PEM, NUL-terminated), heap; NULL = none. Written by the worker,
   read by the net task inside cloud_extras_ca_pem() under a critical section. */
/* blob_store has no erase: clearing writes a 1-byte marker, and anything shorter than a PEM
   certificate could ever be (64 bytes) reads as "no company CA". */
#define CA_MIN_BYTES 64u

static char             *s_ca;
static size_t            s_ca_len;
static cloud_proxy_t     s_proxy;
static volatile bool     s_ready;
/* Why an installed company CA is not in use (CLOUD_CA_ERR_*), NULL = none or in use. */
static const char * volatile s_ca_error;

bool cloud_extras_ready(void) { return s_ready; }
void cloud_extras_mark_ready(void) { s_ready = true; }

void cloud_extras_free(void *p) { vPortFree(p); }

/* Read a whole slot into a fresh heap buffer (+NUL). Bus held. NULL if absent or too big. */
static char *read_slot(blob_id_t id, size_t max, size_t *len) {
    const blob_info_t *in = blob_store_info(id);
    if (!in || !in->present || in->len == 0 || in->len > max) return NULL;
    char *buf = pvPortMalloc(in->len + 1);
    if (!buf) return NULL;
    for (uint32_t off = 0; off < in->len; ) {
        uint32_t n = in->len - off > 256u ? 256u : in->len - off;
        if (blob_store_read((void *)(uintptr_t)id, off, (uint8_t *)buf + off, n) != 0) {
            vPortFree(buf);
            return NULL;
        }
        off += n;
    }
    buf[in->len] = '\0';
    *len = in->len;
    return buf;
}

/* Does a company CA parse? 0 = yes, 1 = no (corrupt or not X.509), -1 = out of memory (unknown).
   The chain lives on the heap: this also runs on the net task, whose stack is tight. */
static int ca_parse_state(const char *pem, size_t len) {
    mbedtls_x509_crt *chain = pvPortMalloc(sizeof(*chain));
    if (!chain) return -1;
    mbedtls_x509_crt_init(chain);
    int rc = mbedtls_x509_crt_parse(chain, (const unsigned char *)pem, len + 1);   /* + its NUL */
    int certs = 0;
    for (const mbedtls_x509_crt *c = chain; c && c->raw.len; c = c->next) certs++;
    mbedtls_x509_crt_free(chain);
    vPortFree(chain);
    if (rc == MBEDTLS_ERR_X509_ALLOC_FAILED || rc == MBEDTLS_ERR_PEM_ALLOC_FAILED) return -1;
    return (rc != 0 || certs == 0) ? 1 : 0;
}

/* Drop the company CA from RAM and remember why: the link then trusts the built-in roots only. */
static void ca_reject(const char *why) {
    taskENTER_CRITICAL();
    char *old = s_ca;
    s_ca = NULL;
    s_ca_len = 0;
    s_ca_error = why;
    taskEXIT_CRITICAL();
    if (old) vPortFree(old);
    printf("[cloud] company CA %s: ignored, the cloud link uses the built-in roots only\n", why);
}

static void load_ca_locked(void) {
    size_t len = 0;
    char *ca = read_slot(BLOB_CA, CLOUD_CA_MAX_BYTES, &len);
    if (ca && len < CA_MIN_BYTES) { vPortFree(ca); ca = NULL; }   /* the "cleared" marker */
    const char *why = NULL;
    if (ca) {
        /* The slot header carries the hash of what was written: a mismatch is a damaged slot. */
        const blob_info_t *in = blob_store_info(BLOB_CA);
        uint8_t sha[32];
        mbedtls_sha256((const unsigned char *)ca, len, sha, 0);
        if (!in || memcmp(sha, in->sha256, sizeof(sha)) != 0)
            why = CLOUD_CA_ERR_HASH;
        else if (ca_parse_state(ca, len) > 0)
            why = CLOUD_CA_ERR_PARSE;
        /* Out of memory while parsing: keep it; the net task checks again if TLS refuses it. */
    }
    taskENTER_CRITICAL();
    char *old = s_ca;
    s_ca = why ? NULL : ca;
    s_ca_len = s_ca ? len : 0;
    s_ca_error = why;
    taskEXIT_CRITICAL();
    if (old) vPortFree(old);
    if (why) {
        vPortFree(ca);
        printf("[cloud] company CA %s: ignored, the cloud link uses the built-in roots only\n", why);
    }
}

void cloud_extras_load(void) {
    if (w25q_open() == 0) {
        load_ca_locked();
        size_t len = 0;
        char *p = read_slot(BLOB_PROXY, sizeof(cloud_proxy_t), &len);
        if (p && len == sizeof(cloud_proxy_t)) {
            cloud_proxy_t tmp;
            memcpy(&tmp, p, sizeof(tmp));
            tmp.host[sizeof(tmp.host) - 1] = '\0';
            tmp.user[sizeof(tmp.user) - 1] = '\0';
            tmp.pass[sizeof(tmp.pass) - 1] = '\0';
            taskENTER_CRITICAL();
            s_proxy = tmp;
            taskEXIT_CRITICAL();
        }
        if (p) vPortFree(p);
        w25q_close();
    }
    if (s_ca || s_proxy.host[0])
        printf("[cloud] company CA %s, proxy %s\n", s_ca ? "installed" : (s_ca_error ? s_ca_error : "none"),
               s_proxy.host[0] ? s_proxy.host : "none");
    s_ready = true;
}

/* ---- company CA ---------------------------------------------------------- */

const char *cloud_extras_ca_check(const uint8_t *pem, size_t len) {
    if (len == 0 || len > CLOUD_CA_MAX_BYTES) return "the company CA must be 1 to 16384 bytes of PEM";
    if (pem[len - 1] != '\0' && !memchr(pem, '\0', len)) {
        /* mbedtls wants the NUL counted for PEM: parse a terminated copy. */
        char *tmp = pvPortMalloc(len + 1);
        if (!tmp) return "out of memory";
        memcpy(tmp, pem, len);
        tmp[len] = '\0';
        const char *why = cloud_extras_ca_check((const uint8_t *)tmp, len + 1);
        vPortFree(tmp);
        return why;
    }
    mbedtls_x509_crt chain;
    mbedtls_x509_crt_init(&chain);
    int rc = mbedtls_x509_crt_parse(&chain, pem, len);
    int certs = 0;
    for (const mbedtls_x509_crt *c = &chain; c && c->raw.len; c = c->next) certs++;
    mbedtls_x509_crt_free(&chain);
    if (rc < 0 || certs == 0) return "not a PEM X.509 certificate";
    if (rc > 0) return "some certificates in the file did not parse";
    return NULL;
}

/* The staged company CA must parse before ota_end accepts it (ota.h). */
int ota_validate_staged(ota_target_t t, uint32_t size, char *why, size_t cap) {
    if (t != OTA_TARGET_CA) return 0;
    if (size > CLOUD_CA_MAX_BYTES) { snprintf(why, cap, "company CA larger than %u bytes", CLOUD_CA_MAX_BYTES); return -1; }
    uint8_t *buf = pvPortMalloc(size + 1);
    if (!buf) { snprintf(why, cap, "out of memory"); return -1; }
    int rc = ota_read_staged(0, buf, size);
    const char *bad = rc != 0 ? "psram read failed" : NULL;
    if (!bad) { buf[size] = 0; bad = cloud_extras_ca_check(buf, size + 1); }
    vPortFree(buf);
    if (bad) { snprintf(why, cap, "company CA: %s", bad); return -1; }
    return 0;
}

void cloud_extras_ca_changed(void) {
    if (w25q_open() == 0) {
        load_ca_locked();
        w25q_close();
    }
    printf("[cloud] company CA %s: reconnecting\n", s_ca ? "installed" : (s_ca_error ? s_ca_error : "removed"));
    net_cloud_reload();
}


static int marker_src(void *ctx, uint32_t off, uint8_t *buf, uint32_t n) {
    (void)ctx; (void)off;
    memset(buf, 0, n);
    return 0;
}

const char *cloud_extras_ca_clear(void) {
    static const uint8_t zero = 0;
    uint8_t sha[32];
    mbedtls_sha256(&zero, 1, sha, 0);
    if (w25q_open() != 0) return "the W25Q flash does not answer";
    int rc = blob_store_write(BLOB_CA, 1, 0, sha, marker_src, NULL);
    w25q_close();
    if (rc != 0) return "could not clear the company CA";
    cloud_extras_ca_changed();
    return NULL;
}

int cloud_extras_ca_pem(char **out, size_t *len_with_nul) {
    *out = NULL;
    *len_with_nul = 0;
    /* cloud_ca_pem_len counts its NUL; the company CA follows after a newline. */
    taskENTER_CRITICAL();
    size_t extra = s_ca ? s_ca_len : 0;
    taskEXIT_CRITICAL();
    if (!extra) return 0;   /* no company CA: the caller uses the built-in roots as they are */
    size_t base = cloud_ca_pem_len ? cloud_ca_pem_len - 1 : 0;
    char *buf = pvPortMalloc(base + 1 + extra + 1);
    if (!buf) return -1;
    memcpy(buf, cloud_ca_pem, base);
    size_t n = base;
    bool same = false;
    taskENTER_CRITICAL();
    if (s_ca && s_ca_len == extra) {
        buf[n++] = '\n';
        memcpy(buf + n, s_ca, extra);
        n += extra;
        same = true;
    }
    taskEXIT_CRITICAL();
    if (!same) { vPortFree(buf); return 0; }   /* removed meanwhile: built-in roots */
    buf[n++] = '\0';
    *out = buf;
    *len_with_nul = n;
    return 0;
}

bool cloud_extras_ca_tls_refused(void) {
    /* Copy it out first: parsing under a critical section would stall the scheduler. */
    taskENTER_CRITICAL();
    size_t len = s_ca ? s_ca_len : 0;
    taskEXIT_CRITICAL();
    if (!len) return false;
    char *copy = pvPortMalloc(len + 1);
    if (!copy) return false;
    bool same = false;
    taskENTER_CRITICAL();
    if (s_ca && s_ca_len == len) { memcpy(copy, s_ca, len + 1); same = true; }
    taskEXIT_CRITICAL();
    int st = same ? ca_parse_state(copy, len) : -1;
    vPortFree(copy);
    if (st <= 0) return false;
    ca_reject(CLOUD_CA_ERR_PARSE);
    return true;
}

const char *cloud_extras_ca_error(void) { return s_ca_error; }

int cloud_extras_ca_describe(char *out, size_t cap) {
    out[0] = '\0';
    if (!s_ca) return 0;
    mbedtls_x509_crt chain;
    mbedtls_x509_crt_init(&chain);
    if (mbedtls_x509_crt_parse(&chain, (const unsigned char *)s_ca, s_ca_len + 1) < 0) {
        mbedtls_x509_crt_free(&chain);
        return 0;
    }
    int certs = 0;
    size_t used = 0;
    for (const mbedtls_x509_crt *c = &chain; c && c->raw.len; c = c->next) {
        char subject[96] = "";
        mbedtls_x509_dn_gets(subject, sizeof(subject), &c->subject);
        for (char *p = subject; *p; p++) if (*p == '"' || *p == '\\') *p = '\'';
        uint8_t d[32];
        mbedtls_sha256(c->raw.p, c->raw.len, d, 0);
        char hex[65];
        for (int i = 0; i < 32; i++) snprintf(hex + 2 * i, 3, "%02x", d[i]);
        int w = snprintf(out + used, cap - used, "%s{\"subject\":\"%s\",\"sha256\":\"%s\"}",
                         certs ? "," : "", subject, hex);
        if (w < 0 || (size_t)w >= cap - used) break;
        used += (size_t)w;
        certs++;
    }
    mbedtls_x509_crt_free(&chain);
    return certs;
}

/* ---- proxy --------------------------------------------------------------- */

static int proxy_src(void *ctx, uint32_t off, uint8_t *buf, uint32_t n) {
    memcpy(buf, (const uint8_t *)ctx + off, n);
    return 0;
}

const char *cloud_extras_proxy_set(const char *spec, const char *user, const char *pass) {
    cloud_proxy_t p;
    memset(&p, 0, sizeof(p));
    if (spec && spec[0]) {
        const char *colon = strrchr(spec, ':');
        if (!colon || colon == spec) return "give the proxy as host:port";
        size_t hlen = (size_t)(colon - spec);
        if (hlen >= sizeof(p.host)) return "proxy host too long";
        long port = strtol(colon + 1, NULL, 10);
        if (port <= 0 || port > 65535) return "proxy port must be 1 to 65535";
        memcpy(p.host, spec, hlen);
        p.port = (uint16_t)port;
        if (user && user[0]) {
            if (strlen(user) >= sizeof(p.user)) return "proxy user too long";
            if (!pass || strlen(pass) >= sizeof(p.pass)) return "proxy password missing or too long";
            strcpy(p.user, user);
            strcpy(p.pass, pass);
        }
    }
    uint8_t sha[32];
    mbedtls_sha256((const unsigned char *)&p, sizeof(p), sha, 0);
    if (w25q_open() != 0) return "the W25Q flash does not answer";
    int rc = blob_store_write(BLOB_PROXY, sizeof(p), 0, sha, proxy_src, &p);
    w25q_close();
    if (rc != 0) return "could not save the proxy";
    taskENTER_CRITICAL();
    s_proxy = p;
    taskEXIT_CRITICAL();
    printf("[cloud] proxy %s%s: reconnecting\n", p.host[0] ? "set to " : "cleared", p.host);
    net_cloud_reload();
    return NULL;
}

void cloud_extras_proxy_get(cloud_proxy_t *out) {
    taskENTER_CRITICAL();
    *out = s_proxy;
    taskEXIT_CRITICAL();
}
