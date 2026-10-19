// Control-channel crypto shared by the bridge and the pod: X25519 pairing and the sealed Wi-Fi
// passphrase. Built on Monocypher 4.0.2 (the pod's submodule; vendored for the bridge in
// components/monocypher). See docs/PROTOCOL.md "Pairing" and "SET_WIFI".
#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BWP_PUBKEY_LEN      32
#define BWP_SEAL_NONCE_LEN  24
#define BWP_PASS_BLOCK      65   // length byte + passphrase (<= 64) + zero padding: hides the length
#define BWP_SEALED_PASS_LEN (BWP_SEAL_NONCE_LEN + BWP_PASS_BLOCK)

// X25519 public key for a 32-byte secret.
void bwp_x25519_public(const uint8_t secret[32], uint8_t pub[BWP_PUBKEY_LEN]);

// The pairing key both sides compute:
//   key = BLAKE2b-256(key = X25519(my_secret, peer_pub), "BWP pairing v1" || pod_pub || bridge_pub)
// Returns -1 (and zeroes key) if the shared secret is all zero, i.e. a bogus peer key.
int bwp_pair_key(const uint8_t my_secret[32], const uint8_t peer_pub[BWP_PUBKEY_LEN],
                 const uint8_t pod_pub[BWP_PUBKEY_LEN], const uint8_t bridge_pub[BWP_PUBKEY_LEN], uint8_t key[32]);

// Sealed passphrase (the ENC_PASSPHRASE TLV): nonce (24 random bytes) || XChaCha20 of
// [len, passphrase, zero padding] (65 bytes) under BLAKE2b-256(key = pairing key, "BWP passphrase v1").
// Integrity comes from the request's AUTH, which covers it. The nonce must be fresh per message.
// Returns -1 if the passphrase is longer than 64 bytes.
int bwp_seal_passphrase(const uint8_t key[32], const uint8_t nonce[BWP_SEAL_NONCE_LEN], const char *pass,
                        uint8_t out[BWP_SEALED_PASS_LEN]);
// Opens it into out (NUL-terminated). Returns the passphrase length, or -1 if malformed.
int bwp_open_passphrase(const uint8_t key[32], const uint8_t *sealed, size_t len, char out[65]);

#ifdef __cplusplus
}
#endif
