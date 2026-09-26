/*
 * rng.h — STM32H5 hardware TRNG with error reporting (see rng.c).
 *
 * get_rand_32/64 (pico/rand.h) stay for callers that only need unpredictable-
 * enough values (WebSocket masks, backoff jitter).  Anything that makes KEYS
 * or seeds TLS must use rng_fill(), which fails instead of handing out zeros
 * when the RNG reports a seed or clock error.
 */
#ifndef RNG_H
#define RNG_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

/* Attempts per 32-bit word; the RNG is reset and re-initialised between them. */
#define RNG_WORD_ATTEMPTS  4

/* Fill buf[len] from the TRNG.  0 on success; -1 if the RNG kept failing, in
   which case buf is zeroed and must not be used. */
int rng_fill(void *buf, size_t len);

/* One word.  0 on success, -1 on failure (*out = 0). */
int rng_get32(uint32_t *out);

/* Failed reads since boot (each retry counts), for diagnostics. */
uint32_t rng_error_count(void);

/* ---- retry policy (pure logic, host-tested) ------------------------------
 * `word` reads one 32-bit value (0 = ok).  `reset` re-initialises the source
 * after a failure (per RM0481: clear SEIS / CEIS, toggle RNGEN, or a full
 * peripheral reset).  A word is used only from a successful read; if every
 * attempt for one word fails, the whole buffer is wiped and -1 is returned so a
 * caller can never mistake a partial or zero fill for entropy. */
typedef int  (*rng_word_fn)(void *ctx, uint32_t *out);
typedef void (*rng_reset_fn)(void *ctx);

static inline int rng_fill_policy(void *buf, size_t len, rng_word_fn word,
                                  rng_reset_fn reset, void *ctx, unsigned attempts,
                                  uint32_t *errors)
{
    unsigned char *p = (unsigned char *)buf;
    size_t i = 0;
    while (i < len) {
        uint32_t r = 0;
        unsigned a = 0;
        for (; a < attempts; a++) {
            if (word(ctx, &r) == 0) break;
            if (errors) (*errors)++;
            if (reset) reset(ctx);
        }
        if (a == attempts) {
            memset(buf, 0, len);
            return -1;
        }
        size_t n = (len - i < sizeof(r)) ? (len - i) : sizeof(r);
        memcpy(p + i, &r, n);
        i += n;
    }
    return 0;
}

#endif /* RNG_H */
