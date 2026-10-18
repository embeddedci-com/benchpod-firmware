#ifndef LAN_TLS_PEM_H
#define LAN_TLS_PEM_H

/*
 * lan_tls_pem — split an uploaded PEM bundle into its private key and its certificates
 * (lan_tls.h). Pure text handling, no mbedTLS, so the host tests cover it.
 */

#include <stddef.h>

/* in: len bytes of PEM text (need not be NUL-terminated). Writes the one private key block to key
   and every CERTIFICATE block, in order, to cert; both NUL-terminated, *_len include the NUL (what
   mbedTLS's PEM parsers want). Other blocks are ignored. NULL = done, else why not. */
const char *lan_tls_pem_split(const char *in, size_t len,
                              char *key, size_t key_cap, size_t *key_len,
                              char *cert, size_t cert_cap, size_t *cert_len);

#endif /* LAN_TLS_PEM_H */
