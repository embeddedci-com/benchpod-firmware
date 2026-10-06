/*
 * fw_sign.c — signed-manifest check for OTA images (see fw_sign.h).
 */
#include "fw_sign.h"
#include "fw_sign_keys_gen.h"     /* generated from keys/release-N.pub by the Makefile */
#include "monocypher-ed25519.h"

#include <string.h>

#define CTX_LEN    (sizeof(FW_SIGN_CONTEXT))   /* includes the NUL separator */
#define SIGNED_LEN 64u

static const uint8_t k_keys[][32] = FW_SIGN_KEYS_INIT;

static uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t rd32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* key_id = the first 8 bytes of SHA-512(public key): Monocypher has SHA-512 for Ed25519 anyway. */
static void key_id_of(const uint8_t pub[32], uint8_t id[FW_SIGN_KEY_ID_LEN]) {
    uint8_t h[64];
    crypto_sha512(h, pub, 32);
    memcpy(id, h, FW_SIGN_KEY_ID_LEN);
}

fw_sig_result_t fw_sign_check_keys(const uint8_t *sig, size_t sig_len, uint8_t target,
                                   uint32_t size, const uint8_t sha256[32],
                                   const uint8_t (*keys)[32], size_t nkeys,
                                   fw_sig_manifest_t *out) {
    if (sig == NULL || sig_len == 0) return FW_SIG_NONE;
    if (sig_len != FW_SIGN_MANIFEST_LEN || memcmp(sig, "BPSG", 4) != 0 || rd16(sig + 4) != 1u ||
        sig[6] > 3u)
        return FW_SIG_FORMAT;

    fw_sig_manifest_t m;
    m.target       = sig[6];
    m.flags        = sig[7];
    m.size         = rd32(sig + 8);
    memcpy(m.sha256, sig + 12, 32);
    m.version      = rd32(sig + 44);
    m.release      = rd32(sig + 48);
    m.layout       = rd16(sig + 52);
    m.min_flash_kb = rd16(sig + 54);
    memcpy(m.key_id, sig + 56, FW_SIGN_KEY_ID_LEN);
    if (out) *out = m;

    const uint8_t *pub = NULL;
    for (size_t i = 0; i < nkeys && !pub; i++) {
        uint8_t id[FW_SIGN_KEY_ID_LEN];
        key_id_of(keys[i], id);
        if (memcmp(id, m.key_id, FW_SIGN_KEY_ID_LEN) == 0) pub = keys[i];
    }
    if (!pub) return FW_SIG_UNKNOWN_KEY;

    uint8_t msg[CTX_LEN + SIGNED_LEN];
    memcpy(msg, FW_SIGN_CONTEXT, CTX_LEN);
    memcpy(msg + CTX_LEN, sig, SIGNED_LEN);
    if (crypto_ed25519_check(sig + SIGNED_LEN, pub, msg, sizeof(msg)) != 0) return FW_SIG_SIGNATURE;
    if (m.target != target) return FW_SIG_TARGET;
    if (m.size != size || memcmp(m.sha256, sha256, 32) != 0) return FW_SIG_IMAGE;
    return FW_SIG_OK;
}

fw_sig_result_t fw_sign_check(const uint8_t *sig, size_t sig_len, uint8_t target, uint32_t size,
                              const uint8_t sha256[32], fw_sig_manifest_t *out) {
    return fw_sign_check_keys(sig, sig_len, target, size, sha256, k_keys, FW_SIGN_KEY_COUNT, out);
}

bool fw_sign_accept(fw_sig_result_t r, fw_sig_policy_t policy) {
    switch (policy) {
    case FW_SIG_POLICY_AUDIT:      return true;
    case FW_SIG_POLICY_PERMISSIVE: return r == FW_SIG_OK || r == FW_SIG_NONE;
    case FW_SIG_POLICY_REQUIRED:   return r == FW_SIG_OK;
    }
    return false;
}

const char *fw_sign_result_name(fw_sig_result_t r) {
    switch (r) {
    case FW_SIG_NONE:        return "none";
    case FW_SIG_OK:          return "ok";
    case FW_SIG_FORMAT:      return "format";
    case FW_SIG_UNKNOWN_KEY: return "unknown-key";
    case FW_SIG_SIGNATURE:   return "signature";
    case FW_SIG_TARGET:      return "target";
    case FW_SIG_IMAGE:       return "image";
    }
    return "?";
}

/* Audit in this release (see fw_sign.h); not persisted yet. */
static fw_sig_policy_t s_policy = FW_SIG_POLICY_AUDIT;
fw_sig_policy_t fw_sign_policy(void) { return s_policy; }
void fw_sign_set_policy(fw_sig_policy_t p) { s_policy = p; }

const char *fw_sign_policy_name(fw_sig_policy_t p) {
    switch (p) {
    case FW_SIG_POLICY_AUDIT:      return "audit";
    case FW_SIG_POLICY_PERMISSIVE: return "permissive";
    case FW_SIG_POLICY_REQUIRED:   return "required";
    }
    return "?";
}

size_t fw_sign_key_count(void) { return FW_SIGN_KEY_COUNT; }
