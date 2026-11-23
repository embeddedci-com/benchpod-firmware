#ifndef LAN_TLS_RESUME_H
#define LAN_TLS_RESUME_H
/*
 * LAN TLS session resumption policy (docs/design/lan-pairing.md).
 *
 * A resumed TLS session is the session it resumes: the client does not send its certificate
 * again, and mbedTLS restores the one the original handshake saw (MBEDTLS_SSL_KEEP_PEER_CERTIFICATE,
 * from the ticket or the cache). So a resumed connection is paired exactly when the original one
 * was, and net_server hashes that certificate's key and checks the list on every command (a
 * computer removed from the list is refused at once, resumed or not).
 *
 * The one hole: a session that was set up WITHOUT a client certificate (an old CLI, a probe, a
 * connection from before the computer had a key) stays certificate-less for as long as it is
 * resumed, and a client that caches sessions by server address alone (the CLI does) resumes it
 * after it got a certificate: the resumed connection is unpaired for the whole ticket lifetime.
 * These two callbacks refuse to resume such a session. The server then runs a full handshake, with
 * a CertificateRequest, and the client presents its certificate; the new ticket carries it.
 * Cost: a client with no certificate always does a full handshake (~0.46 s on the pod).
 */
#include <stddef.h>
#include "mbedtls/ssl.h"

/* mbedtls_ssl_ticket_parse, but a session without a peer certificate is refused. */
int lan_tls_ticket_parse(void *p_ticket, mbedtls_ssl_session *session, unsigned char *buf, size_t len);

/* mbedtls_ssl_cache_get, but a session without a peer certificate is refused. */
int lan_tls_cache_get(void *data, const unsigned char *session_id, size_t session_id_len,
                      mbedtls_ssl_session *session);

#endif
