#include "fwsig.h"

#include <string.h>

#include "bwp_id.h"
#include "monocypher-ed25519.h"

static const char CONTEXT[] = "benchpod-fw-sign:v1";  // signed with its NUL

static uint32_t le32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

fwsig_result_t fwsig_check(const uint8_t *sig, size_t len, const uint8_t (*keys)[32], size_t nkeys,
                           uint8_t target, uint32_t size, fwsig_t *out)
{
    if (!sig || len != FWSIG_LEN || memcmp(sig, "BPSG", 4) != 0 || (sig[4] | (sig[5] << 8)) != 1) {
        return FWSIG_FORMAT;
    }
    fwsig_t m;
    m.target = sig[6];
    m.size = le32(sig + 8);
    memcpy(m.sha256, sig + 12, 32);
    m.version = le32(sig + 44);
    m.release = le32(sig + 48);
    memcpy(m.key_id, sig + 56, 8);
    if (out) {
        *out = m;
    }
    const uint8_t *key = NULL;
    for (size_t i = 0; i < nkeys && !key; i++) {
        uint8_t h[64];
        crypto_sha512(h, keys[i], 32);
        if (memcmp(h, m.key_id, 8) == 0) {
            key = keys[i];
        }
    }
    if (!key) {
        return FWSIG_UNKNOWN_KEY;
    }
    uint8_t msg[sizeof(CONTEXT) + 64];
    memcpy(msg, CONTEXT, sizeof(CONTEXT));  // includes the NUL
    memcpy(msg + sizeof(CONTEXT), sig, 64);
    if (crypto_ed25519_check(sig + 64, key, msg, sizeof(msg)) != 0) {
        return FWSIG_SIGNATURE;
    }
    if (m.target != target) {
        return FWSIG_TARGET;
    }
    if (m.size != size) {
        return FWSIG_IMAGE;
    }
    return FWSIG_OK;
}

const char *fwsig_result_str(fwsig_result_t r)
{
    static const char *const names[] = {"ok", "format", "unknown-key", "signature", "target", "image"};
    return (unsigned)r < sizeof(names) / sizeof(names[0]) ? names[r] : "?";
}

fwsig_result_t fwsig_check_bridge_id(const uint8_t cert[FWSIG_LEN], const uint8_t id_pub[32],
                                     const uint8_t (*keys)[32], size_t nkeys)
{
    fwsig_t m;
    fwsig_result_t r = fwsig_check(cert, FWSIG_LEN, keys, nkeys, FWSIG_TARGET_BRIDGE_ID, BWP_ID_PUB_LEN, &m);
    if (r != FWSIG_OK) {
        return r;
    }
    uint8_t want[32];
    bwp_id_cert_digest(id_pub, want);
    return memcmp(want, m.sha256, 32) == 0 ? FWSIG_OK : FWSIG_IMAGE;
}
