// The pod's signed firmware manifest (benchpod-firmware tools/fwsign.py,
// docs/design/firmware-signing.md), checked by the bridge before it installs an update.
//
// 128 bytes: "BPSG", format 1, target, flags, size, SHA-256, version, release, layout,
// min_flash_kb, key_id (8 bytes of SHA-512(public key)), then an Ed25519 signature over
// "benchpod-fw-sign:v1\0" || bytes 0..63. Integers little-endian. The keys are the pod's release
// keys (plus the developer key in development builds); see main/fw_keys_gen.h.
#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FWSIG_LEN            128
#define FWSIG_TARGET_BRIDGE  6   // ota_target_t numbering in the pod firmware

typedef enum {
    FWSIG_OK = 0,
    FWSIG_FORMAT,       // wrong length, magic or format
    FWSIG_UNKNOWN_KEY,  // key_id is not one we trust
    FWSIG_SIGNATURE,    // the Ed25519 signature does not verify
    FWSIG_TARGET,       // signed for another target
    FWSIG_IMAGE,        // size (or, checked by the caller, SHA-256) differs
} fwsig_result_t;

typedef struct {
    uint8_t target;
    uint32_t size;
    uint8_t sha256[32];
    uint32_t version;   // asset version: major << 16 | minor << 8 | patch
    uint32_t release;
    uint8_t key_id[8];
} fwsig_t;

// Checks format, key, signature, target and size. The SHA-256 of the received image is the
// caller's to compare with out->sha256 once it has all of it.
fwsig_result_t fwsig_check(const uint8_t *sig, size_t len, const uint8_t (*keys)[32], size_t nkeys,
                           uint8_t target, uint32_t size, fwsig_t *out);
const char *fwsig_result_str(fwsig_result_t r);  // the same words as the pod: "ok", "format", ...

#ifdef __cplusplus
}
#endif
