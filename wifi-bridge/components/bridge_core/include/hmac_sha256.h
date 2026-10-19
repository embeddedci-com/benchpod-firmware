// SHA-256 and HMAC-SHA256 (FIPS 180-4, RFC 2104), small and portable so the control-channel
// authentication is the same code in the firmware and in the host tests. Checked against the
// RFC 4231 test vectors in test/.
#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t h[8];
    uint64_t len;  // bytes hashed so far
    uint8_t buf[64];
    size_t used;
} sha256_ctx_t;

void sha256_init(sha256_ctx_t *c);
void sha256_update(sha256_ctx_t *c, const void *data, size_t len);
void sha256_final(sha256_ctx_t *c, uint8_t out[32]);

typedef struct {
    sha256_ctx_t inner;
    uint8_t opad_key[64];
} hmac_sha256_ctx_t;

void hmac_sha256_init(hmac_sha256_ctx_t *c, const uint8_t *key, size_t key_len);
void hmac_sha256_update(hmac_sha256_ctx_t *c, const void *data, size_t len);
void hmac_sha256_final(hmac_sha256_ctx_t *c, uint8_t out[32]);

// Constant-time comparison: 0 if equal.
int ct_memcmp(const void *a, const void *b, size_t n);

#ifdef __cplusplus
}
#endif
