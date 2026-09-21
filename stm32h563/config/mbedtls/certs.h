/*
 * mbedtls/certs.h — empty shim.
 *
 * The vendored LwIP altcp_tls_mbedtls.c still #includes <mbedtls/certs.h>, a
 * header that mbedTLS removed in 3.x (it only ever held built-in *test* certs,
 * none of which altcp_tls actually references). config/ is searched before the
 * real mbedTLS include dir, so this empty file satisfies that one include
 * without shadowing any other mbedtls/*.h header.
 */
#ifndef MBEDTLS_CERTS_H_SHIM
#define MBEDTLS_CERTS_H_SHIM
#endif
