#ifndef FW_SIGN_H
#define FW_SIGN_H

/*
 * fw_sign — check the detached signed manifest that comes with a firmware or blob image
 * (docs/design/firmware-signing.md; tools/fwsign.py makes them).
 *
 * The manifest (128 bytes) names the target, size and SHA-256 of the image and carries an
 * Ed25519 signature by one of the release keys built into this firmware (keys/release-N.pub, plus the
 * developer key on a non-RELEASE build). ota_begin checks it against the size, SHA-256 and target
 * the client announces, before anything is staged; the existing SHA-256 check at ota_end then
 * ties the received bytes to the signed hash.
 *
 * Policy: FW_SIG_POLICY_AUDIT (the default) accepts every image and records the result (ota
 * status, ota.status, upload-status); PERMISSIVE refuses a bad signature; REQUIRED refuses
 * anything not signed. pod_policy.h persists it and decides who may change it (the cloud may
 * only tighten it, the USB console anything).
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define FW_SIGN_MANIFEST_LEN 128u
#define FW_SIGN_KEY_ID_LEN   8u
#define FW_SIGN_CONTEXT      "benchpod-fw-sign:v1"

typedef enum {
    FW_SIG_NONE = 0,       /* no manifest came with the image */
    FW_SIG_OK,
    FW_SIG_FORMAT,         /* wrong length, magic or format */
    FW_SIG_UNKNOWN_KEY,    /* key_id is not one of this build's keys */
    FW_SIG_SIGNATURE,      /* the Ed25519 signature does not verify */
    FW_SIG_TARGET,         /* signed for another target (a gateware blob sent as firmware...) */
    FW_SIG_IMAGE,          /* size or SHA-256 differ from the announced image */
} fw_sig_result_t;

typedef enum {
    FW_SIG_POLICY_AUDIT = 0,   /* accept everything, report the result */
    FW_SIG_POLICY_PERMISSIVE,  /* accept unsigned, refuse a bad signature */
    FW_SIG_POLICY_REQUIRED,    /* accept only a good signature */
} fw_sig_policy_t;

typedef struct {
    uint8_t  target;          /* ota_target_t numbering: 0 firmware, 1 gw0, 2 gw1, 3 esp */
    uint8_t  flags;
    uint32_t size;
    uint8_t  sha256[32];
    uint32_t version;         /* asset version */
    uint32_t release;         /* packed firmware version of the release */
    uint16_t layout;
    uint16_t min_flash_kb;
    uint8_t  key_id[FW_SIGN_KEY_ID_LEN];
} fw_sig_manifest_t;

/* Check `sig` (NULL or len 0 = none came) against the image the client announced.
   `out` (may be NULL) gets the parsed fields when the format is valid. */
fw_sig_result_t fw_sign_check(const uint8_t *sig, size_t sig_len, uint8_t target, uint32_t size,
                              const uint8_t sha256[32], fw_sig_manifest_t *out);
/* The same against an explicit key table (host tests). */
fw_sig_result_t fw_sign_check_keys(const uint8_t *sig, size_t sig_len, uint8_t target,
                                   uint32_t size, const uint8_t sha256[32],
                                   const uint8_t (*keys)[32], size_t nkeys,
                                   fw_sig_manifest_t *out);

/* Does `policy` let an image with this result in? */
bool fw_sign_accept(fw_sig_result_t r, fw_sig_policy_t policy);

/* "none", "ok", "format", "unknown-key", "signature", "target", "image" (the Python and Go
   checks use the same words). */
const char *fw_sign_result_name(fw_sig_result_t r);

fw_sig_policy_t fw_sign_policy(void);
/* Set by pod_policy (boot and the sig_policy command); nothing else should call it. */
void fw_sign_set_policy(fw_sig_policy_t p);
const char *fw_sign_policy_name(fw_sig_policy_t p);

/* Number of keys this build trusts (release keys, plus the dev key on a non-RELEASE build). */
size_t fw_sign_key_count(void);

#endif /* FW_SIGN_H */
