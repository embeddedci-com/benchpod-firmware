#include "bwp_crypto.h"

#include <string.h>

#include "monocypher.h"

static const char PAIR_LABEL[] = "BWP pairing v1";
static const char PASS_LABEL[] = "BWP passphrase v1";

void bwp_x25519_public(const uint8_t secret[32], uint8_t pub[BWP_PUBKEY_LEN])
{
    crypto_x25519_public_key(pub, secret);
}

int bwp_pair_key(const uint8_t my_secret[32], const uint8_t peer_pub[BWP_PUBKEY_LEN],
                 const uint8_t pod_pub[BWP_PUBKEY_LEN], const uint8_t bridge_pub[BWP_PUBKEY_LEN], uint8_t key[32])
{
    static const uint8_t zero[32];
    uint8_t shared[32], msg[sizeof(PAIR_LABEL) - 1 + 2 * BWP_PUBKEY_LEN];
    crypto_x25519(shared, my_secret, peer_pub);
    if (crypto_verify32(shared, zero) == 0) {  // low-order peer key: refuse
        crypto_wipe(shared, sizeof(shared));
        memset(key, 0, 32);
        return -1;
    }
    memcpy(msg, PAIR_LABEL, sizeof(PAIR_LABEL) - 1);
    memcpy(msg + sizeof(PAIR_LABEL) - 1, pod_pub, BWP_PUBKEY_LEN);
    memcpy(msg + sizeof(PAIR_LABEL) - 1 + BWP_PUBKEY_LEN, bridge_pub, BWP_PUBKEY_LEN);
    crypto_blake2b_keyed(key, 32, shared, sizeof(shared), msg, sizeof(msg));
    crypto_wipe(shared, sizeof(shared));
    return 0;
}

static void pass_key(const uint8_t key[32], uint8_t out[32])
{
    crypto_blake2b_keyed(out, 32, key, 32, (const uint8_t *)PASS_LABEL, sizeof(PASS_LABEL) - 1);
}

int bwp_seal_passphrase(const uint8_t key[32], const uint8_t nonce[BWP_SEAL_NONCE_LEN], const char *pass,
                        uint8_t out[BWP_SEALED_PASS_LEN])
{
    size_t n = strlen(pass);
    if (n > BWP_PASS_BLOCK - 1) {
        return -1;
    }
    uint8_t block[BWP_PASS_BLOCK] = {0}, k[32];
    block[0] = (uint8_t)n;
    memcpy(block + 1, pass, n);
    pass_key(key, k);
    memcpy(out, nonce, BWP_SEAL_NONCE_LEN);
    crypto_chacha20_x(out + BWP_SEAL_NONCE_LEN, block, sizeof(block), k, nonce, 0);
    crypto_wipe(block, sizeof(block));
    crypto_wipe(k, sizeof(k));
    return 0;
}

int bwp_open_passphrase(const uint8_t key[32], const uint8_t *sealed, size_t len, char out[65])
{
    if (len != BWP_SEALED_PASS_LEN) {
        return -1;
    }
    uint8_t block[BWP_PASS_BLOCK], k[32];
    pass_key(key, k);
    crypto_chacha20_x(block, sealed + BWP_SEAL_NONCE_LEN, sizeof(block), k, sealed, 0);
    crypto_wipe(k, sizeof(k));
    int n = block[0];
    if (n > BWP_PASS_BLOCK - 1) {
        crypto_wipe(block, sizeof(block));
        return -1;
    }
    memcpy(out, block + 1, (size_t)n);
    out[n] = '\0';
    crypto_wipe(block, sizeof(block));
    return n;
}
