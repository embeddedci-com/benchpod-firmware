// Control-channel pairing: which requests the bridge accepts, and from whom.
//
//   unpaired --PAIR(KEY) from the pod--> paired      (trust on first use)
//   paired   --PAIR(KEY) with valid AUTH--> paired, new key (rotation by the same pod)
//   paired   --PAIR button / factory reset--> unpaired (the next pod that offers a key wins)
//
// While unpaired only STATUS, SCAN and PAIR are accepted. While paired every request must carry
// COUNTER + AUTH (bwp_sign) with the stored key and the bridge's per-boot nonce, and the counter
// must increase. Plain C, host-tested in test/.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "bwp.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool paired;
    uint8_t key[BWP_KEY_LEN];
    uint8_t nonce[BWP_NONCE_LEN];  // fresh random value every boot
    uint32_t last_counter;         // highest COUNTER accepted with this nonce
} bwp_pairing_t;

typedef enum {
    PAIR_ACCEPT = 0,     // handle the request
    PAIR_STORE_KEY,      // a PAIR to accept: persist *new_key, then call pairing_set_key
    PAIR_NOT_PAIRED,     // BWP_E_NOT_PAIRED
    PAIR_ALREADY_PAIRED, // BWP_E_ALREADY_PAIRED
    PAIR_AUTH_MISSING,   // BWP_E_AUTH
    PAIR_AUTH_BAD,       // BWP_E_AUTH
    PAIR_REPLAY,         // BWP_E_AUTH: counter not above the last accepted one
    PAIR_BAD_KEY,        // BWP_E_BAD_REQUEST: PAIR without a 32-byte KEY
} pair_verdict_t;

// key: the stored pairing key, or NULL when none is stored.
void pairing_init(bwp_pairing_t *p, const uint8_t nonce[BWP_NONCE_LEN], const uint8_t *key);
// Decides on one parsed request. *authed tells whether it carried a valid AUTH (then sign the
// response with *counter); *new_key points into the frame for PAIR_STORE_KEY.
pair_verdict_t pairing_check(bwp_pairing_t *p, const uint8_t *frame, const bwp_msg_t *m, bool *authed,
                             uint32_t *counter, const uint8_t **new_key);
void pairing_set_key(bwp_pairing_t *p, const uint8_t key[BWP_KEY_LEN]);
void pairing_forget(bwp_pairing_t *p);
bwp_result_t pairing_result(pair_verdict_t v);

#ifdef __cplusplus
}
#endif
