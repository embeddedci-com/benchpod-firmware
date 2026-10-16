#ifndef LAN_TLS_H
#define LAN_TLS_H

/*
 * lan_tls — the LAN API over TLS (docs/design/lan-tls.md): the pod's server certificate and key,
 * and the mbedTLS configuration the TCP 8443 listener (net_server.c) uses.
 *
 * The certificate and key live in internal config flash, a large A/B record (config_store.h) at
 * 0x1E0000 / 0x1E2000. Without one the pod generates an ECDSA P-256 key and a self-signed
 * certificate for benchpod-xxxxxx and benchpod-xxxxxx.local the first time the listener starts.
 *
 * All mbedTLS work (generate, parse, check, build the listener config) runs on the net task,
 * which owns lwIP and the other TLS sessions. The calls marked "any task" hand their work to it
 * with net_call_sync and wait.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define LAN_TLS_PORT              8443
#define LAN_TLS_SLOT_A_OFFSET     0x1E2000u   /* bank2 sector 113 */
#define LAN_TLS_SLOT_B_OFFSET     0x1E0000u   /* bank2 sector 112 */
#define LAN_TLS_RECORD_MAGIC      0x534C5442u /* "BTLS" */
#define LAN_TLS_RECORD_MAX        7680u       /* header + key PEM + certificate PEM(s) */
#define LAN_TLS_UPLOAD_MAX        (LAN_TLS_RECORD_MAX - 16u)

/* What lan_tls reports about the certificate in use (the JSON lan_tls and status replies). */
typedef struct {
    bool    ready;              /* a certificate and key are in use */
    char    origin[10];         /* "generated" or "user" */
    char    key[16];            /* "EC P-256", "RSA 2048", ... */
    char    subject[96];        /* "CN=benchpod-baea06" */
    char    san[3][64];         /* DNS names, at most three reported */
    uint8_t nsan;
    char    not_after[11];      /* "9999-12-31" */
    char    sha256[65];         /* of the leaf certificate (DER), lowercase hex */
    char    error[64];          /* why there is no certificate in use, else "" */
} lan_tls_info_t;

struct altcp_tls_config;

/* Net task, once from net_init: load the stored certificate, or generate and store one; then
   build the listener configuration. */
void lan_tls_init(void);

/* Net task: the configuration new TLS connections get (NULL when there is none). Each connection
   holds a reference from accept to close; a configuration replaced by a new certificate is freed
   by lan_tls_collect once the last connection using it is gone. */
struct altcp_tls_config *lan_tls_config(void);
void     lan_tls_ref(struct altcp_tls_config *cfg);
void     lan_tls_unref(struct altcp_tls_config *cfg);
void     lan_tls_collect(void);
/* Increments each time lan_tls_config() changes (the listener restarts with the new one). */
uint32_t lan_tls_generation(void);

/* Any task: a consistent copy of the current info. */
void lan_tls_get_info(lan_tls_info_t *out);

/* Any task. NULL = done (stored and in use for new connections), else why not. */
const char *lan_tls_regenerate(void);
/* pem: the certificate (chain) and an unencrypted private key, PEM, in any order. */
const char *lan_tls_install(const uint8_t *pem, size_t len);
/* Only the checks lan_tls_install makes (both parse, the key matches the certificate). */
const char *lan_tls_check(const uint8_t *pem, size_t len);

/* The upload path (OTA target "tls", worker task): ota_end checks the staged bundle with
   lan_tls_validate_staged; ota_commit installs it through ota_install_staged_config (ota.h). */
int lan_tls_validate_staged(uint32_t size, char *why, size_t cap);

#endif /* LAN_TLS_H */
