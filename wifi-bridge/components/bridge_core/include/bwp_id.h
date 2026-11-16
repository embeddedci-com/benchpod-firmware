// Bridge identity (bridge 0.1.3+, pod firmware 3.10.7+): every bridge has its own Ed25519 key,
// and a certificate from us says that key belongs to a genuine bridge. The pod pairs only with a
// bridge that shows a valid certificate and signs the pairing transcript with that key.
//
// Certificate: the pod's signed manifest format (fwsig.h, tools/fwsign.py) with target
// BWP_ID_CERT_TARGET ("bridge-id"), size 32 and the SHA-256 of the bridge's identity public key,
// signed with one of the pod's release keys (the developer key too in development builds). A
// manifest for another target (firmware, a blob, a bridge image) is never a certificate, and a
// certificate is never an update. The MAC is not in it: it is not secret and can be copied, so
// only the key proves anything.
//
// Transcript signed by the bridge in its PAIR answer:
//   "benchpod-bridge-pair:v1\0" || pod_pub || bridge_pub || bridge_nonce || pod_nonce
// (both X25519 public keys of this pairing, the bridge's boot nonce from HELLO and the 16 random
// bytes the pod sent in PAIR), so an old answer cannot be replayed into a new pairing.
// Plain C, host-tested on both sides.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bwp.h"
#include "bwp_crypto.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BWP_ID_PUB_LEN      32
#define BWP_ID_SIG_LEN      64
#define BWP_ID_CERT_LEN     128
#define BWP_POD_NONCE_LEN   16
#define BWP_ID_CERT_TARGET  8    // ota_target_t numbering is 0..7: no update target is 8
#define BWP_ID_CONTEXT      "benchpod-bridge-pair:v1"  // signed with its NUL

// What a certificate's manifest must hold for this identity key: size 32 and this SHA-256.
void bwp_id_cert_digest(const uint8_t id_pub[BWP_ID_PUB_LEN], uint8_t sha256[32]);

// The bridge: sign the transcript with its identity key (secret_key: Monocypher's 64-byte form).
void bwp_id_sign(const uint8_t secret_key[64], const uint8_t pod_pub[BWP_PUBKEY_LEN],
                 const uint8_t bridge_pub[BWP_PUBKEY_LEN], const uint8_t bridge_nonce[BWP_NONCE_LEN],
                 const uint8_t pod_nonce[BWP_POD_NONCE_LEN], uint8_t sig[BWP_ID_SIG_LEN]);
// The pod: true when sig is the identity key's signature over this pairing's transcript.
bool bwp_id_verify(const uint8_t id_pub[BWP_ID_PUB_LEN], const uint8_t sig[BWP_ID_SIG_LEN],
                   const uint8_t pod_pub[BWP_PUBKEY_LEN], const uint8_t bridge_pub[BWP_PUBKEY_LEN],
                   const uint8_t bridge_nonce[BWP_NONCE_LEN], const uint8_t pod_nonce[BWP_POD_NONCE_LEN]);

#ifdef __cplusplus
}
#endif
