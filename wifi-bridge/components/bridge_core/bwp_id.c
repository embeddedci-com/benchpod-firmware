// Bridge identity: transcript signatures and the certificate digest (see bwp_id.h).
#include "bwp_id.h"

#include <string.h>

#include "hmac_sha256.h"
#include "monocypher-ed25519.h"

#define CTX_LEN (sizeof(BWP_ID_CONTEXT))  // with the NUL
#define MSG_LEN (CTX_LEN + 2 * BWP_PUBKEY_LEN + BWP_NONCE_LEN + BWP_POD_NONCE_LEN)

static void transcript(uint8_t msg[MSG_LEN], const uint8_t pod_pub[BWP_PUBKEY_LEN],
                       const uint8_t bridge_pub[BWP_PUBKEY_LEN], const uint8_t bridge_nonce[BWP_NONCE_LEN],
                       const uint8_t pod_nonce[BWP_POD_NONCE_LEN])
{
    uint8_t *p = msg;
    memcpy(p, BWP_ID_CONTEXT, CTX_LEN), p += CTX_LEN;
    memcpy(p, pod_pub, BWP_PUBKEY_LEN), p += BWP_PUBKEY_LEN;
    memcpy(p, bridge_pub, BWP_PUBKEY_LEN), p += BWP_PUBKEY_LEN;
    memcpy(p, bridge_nonce, BWP_NONCE_LEN), p += BWP_NONCE_LEN;
    memcpy(p, pod_nonce, BWP_POD_NONCE_LEN);
}

void bwp_id_cert_digest(const uint8_t id_pub[BWP_ID_PUB_LEN], uint8_t sha256[32])
{
    sha256_ctx_t c;
    sha256_init(&c);
    sha256_update(&c, id_pub, BWP_ID_PUB_LEN);
    sha256_final(&c, sha256);
}

void bwp_id_sign(const uint8_t secret_key[64], const uint8_t pod_pub[BWP_PUBKEY_LEN],
                 const uint8_t bridge_pub[BWP_PUBKEY_LEN], const uint8_t bridge_nonce[BWP_NONCE_LEN],
                 const uint8_t pod_nonce[BWP_POD_NONCE_LEN], uint8_t sig[BWP_ID_SIG_LEN])
{
    uint8_t msg[MSG_LEN];
    transcript(msg, pod_pub, bridge_pub, bridge_nonce, pod_nonce);
    crypto_ed25519_sign(sig, secret_key, msg, sizeof(msg));
}

bool bwp_id_verify(const uint8_t id_pub[BWP_ID_PUB_LEN], const uint8_t sig[BWP_ID_SIG_LEN],
                   const uint8_t pod_pub[BWP_PUBKEY_LEN], const uint8_t bridge_pub[BWP_PUBKEY_LEN],
                   const uint8_t bridge_nonce[BWP_NONCE_LEN], const uint8_t pod_nonce[BWP_POD_NONCE_LEN])
{
    uint8_t msg[MSG_LEN];
    transcript(msg, pod_pub, bridge_pub, bridge_nonce, pod_nonce);
    return crypto_ed25519_check(sig, id_pub, msg, sizeof(msg)) == 0;
}
