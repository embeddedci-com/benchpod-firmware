#ifndef DEVICE_IDENTITY_H
#define DEVICE_IDENTITY_H

#include <stdint.h>
#include <stddef.h>

/* ---- Device identity: a persistent Ed25519 keypair ------------------------
 *
 * At boot the device ensures it owns a stable Ed25519 private key, generating
 * one (from the STM32 hardware RNG, src/rng.c) only if flash has none, and NEVER
 * overwriting an existing key.  The public key is the device's identifier; a
 * signature over a server-supplied nonce is its proof of possession.
 *
 * Standard RFC 8032 Ed25519 (SHA-512) via Monocypher's optional
 * crypto_ed25519_* API — interoperable with Go's crypto/ed25519.
 *
 * The 32-byte private seed lives in its OWN 4 KB flash sector (separate from
 * the WiFi config sector), so a WiFi factory-reset never erases the identity.
 * ---------------------------------------------------------------------------*/

#define DEVICE_ID_PUBLIC_LEN  32
#define DEVICE_ID_SIG_LEN     64

/* ---- Domain-separation contexts ------------------------------------------
 * Every signature the device produces is bound to a purpose: the signed message
 * is  context || 0x00 || payload,  so a signature made for one purpose can never
 * be replayed as another.  This closes the signing-oracle: identity_pop (which
 * any LAN client can invoke) signs under DEVICE_ID_CTX_POP, while the cloud WS
 * authentication signs the server nonce under DEVICE_ID_CTX_WS_AUTH — a pop
 * signature therefore cannot be presented as a valid WS-auth proof.
 *
 * The server MUST prepend the SAME context bytes + NUL before ed25519.Verify.
 * Bump the :vN suffix on any breaking change so both ends move together. */
#define DEVICE_ID_CTX_WS_AUTH  "benchpod-ws-auth:v1"
/* v2 binds the login to the server name: the payload is  host || 0x00 || nonce, where host is the
   cloud host the pod connected to (and checked the certificate of). A signature for a challenge
   relayed through another host then does not verify at the real server (access-control.md F2,
   docs/design/cloud-hardening.md). */
#define DEVICE_ID_CTX_WS_AUTH_V2 "benchpod-ws-auth:v2"
#define DEVICE_ID_CTX_POP      "benchpod-pop:v1"

/* Largest message device_identity_sign_ctx will sign (context + NUL + payload): the v2 login
   payload is a host name (< 64 B) + NUL + a server nonce (<= 128 B). */
#define DEVICE_ID_SIGN_MSG_MAX 256

/* Load the key from flash, or generate-and-persist one on first boot.
   Derives the in-RAM keypair and logs the base64url public key.  Call once,
   early in boot (after stdio, before networking). */
void device_identity_init(void);

/* Copy the 32-byte public key into pub.
   Returns 0 on success, -1 if identity is not initialized. */
int device_identity_get_public(uint8_t pub[DEVICE_ID_PUBLIC_LEN]);

/* Sign  context || 0x00 || msg  (len bytes of msg) into sig with the device
   private key.  `context` is one of the DEVICE_ID_CTX_* domain tags.  Returns 0
   on success, -1 if identity is not initialized, -2 if context+msg is too long.
   There is intentionally no context-free signer: every signature is bound to a
   purpose so it can't be repurposed (see the oracle note above). */
int device_identity_sign_ctx(const char *context, const uint8_t *msg, size_t len,
                             uint8_t sig[DEVICE_ID_SIG_LEN]);

#endif /* DEVICE_IDENTITY_H */
