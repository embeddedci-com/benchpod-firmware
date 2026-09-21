/*
 * mbedtls_port.c — bare-metal glue for mbedTLS on the STM32H563:
 *   - heap routed to the FreeRTOS allocator (thread-safe; the tiny newlib
 *     linker heap is nowhere near big enough for a TLS handshake), and
 *   - entropy from the STM32H5 hardware TRNG (MBEDTLS_ENTROPY_HARDWARE_ALT).
 *
 * mbedtls_port_init() must run once before any TLS work (see net_init()).
 */
#include "mbedtls_port.h"

#include "mbedtls/platform.h"
#include "mbedtls/platform_time.h"

#include "FreeRTOS.h"
#include "pico/rand.h"      /* get_rand_32 — STM32H5 TRNG (rng.c) */
#include "pico/time.h"      /* time_us_64 — monotonic uptime for bench_mbedtls_time */

#include <string.h>

/* mbedtls_calloc/free replacement: FreeRTOS heap_4 + zero-fill (pvPortMalloc does
   not clear).  Bound to mbedTLS via MBEDTLS_PLATFORM_CALLOC_MACRO/FREE_MACRO in
   mbedtls_bench_config.h (compile-time), so EVERY mbedTLS allocation lands on the
   256 KB FreeRTOS heap — NOT the 64 KB lwIP MEM_SIZE (which altcp_tls would otherwise
   commandeer, blocking a 2nd concurrent TLS session). Non-static: referenced by the
   macro from mbedTLS headers across translation units. */
void *bench_mbedtls_calloc(size_t n, size_t size)
{
    if (n == 0 || size == 0) return NULL;
    if (n > (size_t)-1 / size) return NULL;     /* overflow */
    size_t total = n * size;
    void *p = pvPortMalloc(total);
    if (p) memset(p, 0, total);
    return p;
}

void bench_mbedtls_free(void *p)
{
    if (p) vPortFree(p);
}

void mbedtls_port_init(void)
{
    /* Allocator is bound at compile time via the *_MACRO config (see above); nothing
       to wire at runtime. Kept as the one-time TLS init hook. */
}

/* MBEDTLS_PLATFORM_TIME_MACRO target: monotonic uptime seconds (no RTC).  Used
   only for session bookkeeping; cert validity dates are not enforced. */
long bench_mbedtls_time(long *timer)
{
    long t = (long)(time_us_64() / 1000000ull);
    if (timer) *timer = t;
    return t;
}

/* MBEDTLS_PLATFORM_MS_TIME_ALT target: monotonic uptime milliseconds. */
mbedtls_ms_time_t mbedtls_ms_time(void)
{
    return (mbedtls_ms_time_t)(time_us_64() / 1000ull);
}

/* Hardware entropy source registered by mbedtls_entropy_init() because the
   config defines MBEDTLS_ENTROPY_HARDWARE_ALT.  Fill `output` from the TRNG. */
int mbedtls_hardware_poll(void *data, unsigned char *output, size_t len, size_t *olen)
{
    (void)data;
    size_t i = 0;
    while (i < len) {
        uint32_t r = get_rand_32();
        size_t n = (len - i < sizeof(r)) ? (len - i) : sizeof(r);
        memcpy(output + i, &r, n);
        i += n;
    }
    if (olen) *olen = len;
    return 0;
}
