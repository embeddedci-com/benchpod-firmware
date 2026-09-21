/*
 * mbedtls_bench_config.h — mbedTLS 3.6 config for the bench-pod's outbound
 * TLS 1.2 WebSocket client (cloud_client.c, via LwIP altcp_tls).
 *
 * Scope: a CLIENT only, TLS 1.2 only (TLS 1.3 deliberately off so we don't pull
 * in the mandatory PSA-crypto subsystem).  No filesystem, no POSIX sockets (the
 * BIO is LwIP's altcp), no threading.  Heap is routed to FreeRTOS via
 * mbedtls_platform_set_calloc_free() (see mbedtls_port.c) and entropy comes from
 * the STM32H5 TRNG (MBEDTLS_ENTROPY_HARDWARE_ALT -> mbedtls_hardware_poll()).
 *
 * Selected via -DMBEDTLS_CONFIG_FILE='"mbedtls_bench_config.h"' (see Makefile).
 */
#ifndef MBEDTLS_BENCH_CONFIG_H
#define MBEDTLS_BENCH_CONFIG_H

/* ---- platform: bare-metal, heap via FreeRTOS, monotonic clock, no fs ------ */
#define MBEDTLS_PLATFORM_C
#define MBEDTLS_PLATFORM_MEMORY              /* mbedtls_calloc/free provided below */
/* Route ALL mbedTLS allocations to the FreeRTOS heap (heap_4, 256 KB) via compile-time
   macros. This is deliberate: the default (function-pointer) path lets LwIP's altcp_tls
   glue OVERRIDE the allocator with its own tls_malloc → lwIP mem_malloc (the 64 KB
   MEM_SIZE), because altcp defines ALTCP_MBEDTLS_PLATFORM_ALLOC=1 whenever
   MBEDTLS_PLATFORM_MEMORY is set without these MACROs. That confines every TLS session to
   the 64 KB lwIP heap — fine for ONE session, but a SECOND concurrent TLS connection
   (the bulk WS) can't fit. Defining the *_MACRO forms makes altcp's condition false
   (ALTCP_MBEDTLS_PLATFORM_ALLOC=0, no override) and sends mbedTLS to the FreeRTOS heap,
   where ~197 KB is free at runtime — room for two sessions. MEM_SIZE then only backs
   pbufs/windows. bench_mbedtls_calloc/free live in mbedtls_port.c. */
#define MBEDTLS_PLATFORM_CALLOC_MACRO   bench_mbedtls_calloc
#define MBEDTLS_PLATFORM_FREE_MACRO     bench_mbedtls_free
#ifndef __ASSEMBLER__
#include <stddef.h>
void *bench_mbedtls_calloc(size_t n, size_t size);
void  bench_mbedtls_free(void *p);
#endif
#define MBEDTLS_NO_PLATFORM_ENTROPY          /* no /dev/urandom — use the hardware source */
#define MBEDTLS_ENTROPY_HARDWARE_ALT         /* we provide mbedtls_hardware_poll() */
#define MBEDTLS_AES_ROM_TABLES               /* keep AES tables in flash, not RAM */

/* A time source is needed only so the vendored (2.x-era) LwIP altcp_tls glue can
   reference mbedtls_ssl_session.start; we have no RTC, so back it with the
   monotonic uptime clock (see bench_mbedtls_time in mbedtls_port.c). Cert
   validity dates are not enforced (no CA / VERIFY_OPTIONAL), so a non-wall clock
   is harmless. */
#define MBEDTLS_HAVE_TIME
#define MBEDTLS_PLATFORM_TIME_TYPE_MACRO   long
#define MBEDTLS_PLATFORM_TIME_MACRO        bench_mbedtls_time
#define MBEDTLS_PLATFORM_MS_TIME_ALT       /* we provide mbedtls_ms_time() */
#ifndef __ASSEMBLER__
long bench_mbedtls_time(long *timer);
#endif

/* ---- message digests ----------------------------------------------------- */
#define MBEDTLS_MD_C
#define MBEDTLS_SHA224_C
#define MBEDTLS_SHA256_C
#define MBEDTLS_SHA384_C
#define MBEDTLS_SHA512_C
#define MBEDTLS_SHA1_C                       /* legacy cert chains / sig algs */
#define MBEDTLS_MD5_C                        /* ESP32-C3 ROM flash verify (esp_rom_flash.c) */

/* ---- symmetric ciphers + AEAD modes -------------------------------------- */
#define MBEDTLS_CIPHER_C
#define MBEDTLS_AES_C
#define MBEDTLS_GCM_C
#define MBEDTLS_CIPHER_MODE_CBC              /* AES-CBC fallback suites */
#define MBEDTLS_CHACHA20_C
#define MBEDTLS_POLY1305_C
#define MBEDTLS_CHACHAPOLY_C

/* ---- public-key crypto --------------------------------------------------- */
#define MBEDTLS_BIGNUM_C

/* Bignum inner-loop in assembly. The custom config REPLACES mbedTLS's default
   config, so without this the schoolbook MULADDC runs in pure C — 3-5x slower on
   every MPI multiply, which dominates the ECDHE + cert-chain-verify cost of the
   TLS handshake. On this Cortex-M33 (Armv8-M.main + DSP: __ARM_FEATURE_DSP=1) it
   selects bn_mul.h's single-cycle UMAAL path, cutting the cold-boot handshake
   from ~7.6 s to ~2 s. (Measured lever: LINK_CONNECT phase in the cloud log.) */
#define MBEDTLS_HAVE_ASM

/* Trade a little handshake-time RAM/ROM for faster EC point multiplication:
   a wider comb window (default 4) speeds every ECDHE + ECDSA-verify op, and the
   fixed-point optimisation precomputes the base-point comb. Both are the hot path
   of the handshake on this device. */
#define MBEDTLS_ECP_WINDOW_SIZE        6
#define MBEDTLS_ECP_FIXED_POINT_OPTIM  1
#define MBEDTLS_OID_C
#define MBEDTLS_ASN1_PARSE_C
#define MBEDTLS_ASN1_WRITE_C
#define MBEDTLS_PK_C
#define MBEDTLS_PK_PARSE_C
#define MBEDTLS_RSA_C
#define MBEDTLS_PKCS1_V15
#define MBEDTLS_PKCS1_V21
#define MBEDTLS_ECP_C
#define MBEDTLS_ECDH_C
#define MBEDTLS_ECDSA_C
#define MBEDTLS_ECP_DP_SECP256R1_ENABLED
#define MBEDTLS_ECP_DP_SECP384R1_ENABLED
#define MBEDTLS_ECP_DP_SECP521R1_ENABLED
#define MBEDTLS_ECP_DP_CURVE25519_ENABLED

/* ★ The dominant TLS-handshake cost lever. This custom config REPLACES mbedTLS's
   default (which defines it), so without it the NIST primes fall back to GENERIC
   modular reduction (mbedtls_mpi_mod_mpi = long division) on every field op of
   every EC point multiply — NOT accelerated by MBEDTLS_HAVE_ASM. That is what
   made the cold-boot handshake spend ~7.1 s of pure CPU in one step (measured:
   [tls-hs] cpu=7151ms). Enabling the curve-specific fast reduction (ecp_mod_p256
   etc.) is expected to cut that ~10-50x. Pairs with MBEDTLS_HAVE_ASM above. */
#define MBEDTLS_ECP_NIST_OPTIM

/* ---- DRBG + entropy ------------------------------------------------------ */
#define MBEDTLS_ENTROPY_C
#define MBEDTLS_CTR_DRBG_C

/* ---- X.509 (server cert chain parsing) ----------------------------------- */
#define MBEDTLS_BASE64_C
#define MBEDTLS_PEM_PARSE_C
#define MBEDTLS_X509_USE_C
#define MBEDTLS_X509_CRT_PARSE_C

/* ---- TLS 1.2 client ------------------------------------------------------ */
#define MBEDTLS_SSL_TLS_C
#define MBEDTLS_SSL_CLI_C
#define MBEDTLS_SSL_PROTO_TLS1_2
#define MBEDTLS_SSL_SERVER_NAME_INDICATION   /* set SNI host on the ClientHello */
#define MBEDTLS_SSL_KEEP_PEER_CERTIFICATE

/* Key exchanges: ECDHE (RSA/ECDSA cert), with a plain-RSA fallback. */
#define MBEDTLS_KEY_EXCHANGE_ECDHE_RSA_ENABLED
#define MBEDTLS_KEY_EXCHANGE_ECDHE_ECDSA_ENABLED
#define MBEDTLS_KEY_EXCHANGE_RSA_ENABLED

/* TLS record buffers (per connection, from the FreeRTOS heap).  IN is large
   enough to receive a full server certificate flight in one record; OUT is
   small — we only ever send little JSON WS frames. */
#define MBEDTLS_SSL_IN_CONTENT_LEN   16384
#define MBEDTLS_SSL_OUT_CONTENT_LEN   4096

/* NB: do NOT include mbedtls/check_config.h here — build_info.h pulls it in
   automatically AFTER the config_adjust_*.h headers derive MBEDTLS_MD_CAN_*,
   so including it manually would falsely report "no hash algorithm". */

#endif /* MBEDTLS_BENCH_CONFIG_H */
