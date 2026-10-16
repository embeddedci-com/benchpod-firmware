#ifndef LAN_TLS_H
#define LAN_TLS_H

/*
 * lan_tls — the LAN API over TLS (docs/design/lan-tls.md): the pod's server certificate and key,
 * the TCP 8443 listener's mbedTLS configuration, and the HTTPS-only setting's side effects.
 *
 * The certificate and key live in internal config flash, a large A/B record (config_store.h) at
 * 0x1E0000 / 0x1E2000. Without one the pod generates an ECDSA P-256 key and a self-signed
 * certificate for benchpod-xxxxxx and benchpod-xxxxxx.local the first time the listener starts.
 */

#define LAN_TLS_PORT              8443
#define LAN_TLS_SLOT_A_OFFSET     0x1E2000u   /* bank2 sector 113 */
#define LAN_TLS_SLOT_B_OFFSET     0x1E0000u   /* bank2 sector 112 */
#define LAN_TLS_RECORD_MAGIC      0x534C5442u /* "BTLS" */

#endif /* LAN_TLS_H */
