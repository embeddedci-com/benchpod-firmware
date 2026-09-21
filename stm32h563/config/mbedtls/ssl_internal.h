/*
 * mbedtls/ssl_internal.h — compatibility shim.
 *
 * mbedTLS 3.0 removed the public <mbedtls/ssl_internal.h> (its contents moved to
 * the library-private ssl_misc.h). ST's vendored LwIP altcp_tls_mbedtls.c (a
 * mbedTLS 2.x-era file) still includes it for two things, both still present as
 * real symbols in mbedTLS 3.6 — so we just re-declare what altcp uses:
 *
 *   - mbedtls_ssl_flush_output(): non-static in library/ssl_msg.c, only the
 *     declaration was lost with ssl_internal.h.
 *   - mbedtls_pk_parse_key(): 3.x added f_rng/p_rng params. altcp's 2.x call
 *     sites (the unused server path) pass 5 args; adapt them to the 7-arg form.
 *
 * config/ is searched before the real mbedTLS include dir, so this satisfies
 * altcp's one #include without shadowing any other mbedtls/*.h header.
 */
#ifndef MBEDTLS_SSL_INTERNAL_H_SHIM
#define MBEDTLS_SSL_INTERNAL_H_SHIM

#include "mbedtls/ssl.h"
#include "mbedtls/pk.h"   /* must be fully parsed before the macro below */

/* Flush data not yet written: library-internal but linkable in 3.6. */
int mbedtls_ssl_flush_output(mbedtls_ssl_context *ssl);

/* Adapt altcp's 5-arg mbedtls_pk_parse_key() calls to the 3.x 7-arg signature.
   The parenthesised name in the replacement suppresses re-expansion, so it
   resolves to the real (already-declared) function. */
#define mbedtls_pk_parse_key(a, b, c, d, e) \
    (mbedtls_pk_parse_key)(a, b, c, d, e, NULL, NULL)

#endif /* MBEDTLS_SSL_INTERNAL_H_SHIM */
