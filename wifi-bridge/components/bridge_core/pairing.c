#include "pairing.h"

#include <string.h>

void pairing_init(bwp_pairing_t *p, const uint8_t nonce[BWP_NONCE_LEN], const uint8_t *key)
{
    memset(p, 0, sizeof(*p));
    memcpy(p->nonce, nonce, BWP_NONCE_LEN);
    if (key) {
        pairing_set_key(p, key);
    }
}

void pairing_set_key(bwp_pairing_t *p, const uint8_t key[BWP_KEY_LEN])
{
    memcpy(p->key, key, BWP_KEY_LEN);
    p->paired = true;
    // A new key starts a new counter sequence: frames signed with any older key fail the MAC.
    p->last_counter = 0;
}

void pairing_forget(bwp_pairing_t *p)
{
    volatile uint8_t *k = p->key;
    for (int i = 0; i < BWP_KEY_LEN; i++) {
        k[i] = 0;
    }
    p->paired = false;
}

pair_verdict_t pairing_check(bwp_pairing_t *p, const uint8_t *frame, const bwp_msg_t *m, bool *authed,
                             uint32_t *counter, const uint8_t **new_key)
{
    *authed = false;
    *counter = 0;
    *new_key = NULL;

    if (p->paired) {
        switch (bwp_verify(frame, m, p->key, p->nonce, counter)) {
        case BWP_VERIFY_MISSING:
            return m->type == BWP_PAIR ? PAIR_ALREADY_PAIRED : PAIR_AUTH_MISSING;
        case BWP_VERIFY_BAD:
            return m->type == BWP_PAIR ? PAIR_ALREADY_PAIRED : PAIR_AUTH_BAD;
        case BWP_VERIFY_OK:
            break;
        }
        if (*counter <= p->last_counter) {
            return PAIR_REPLAY;
        }
        p->last_counter = *counter;
        *authed = true;
    } else if (m->type != BWP_STATUS && m->type != BWP_SCAN && m->type != BWP_PAIR) {
        return PAIR_NOT_PAIRED;
    }

    if (m->type == BWP_PAIR) {
        const uint8_t *k;
        uint16_t n;
        if (!bwp_find(m->body, m->body_len, BWP_T_PUBKEY, &k, &n) || n != 32) {
            return PAIR_BAD_KEY;
        }
        *new_key = k;  // the pod's X25519 public key
        return PAIR_STORE_KEY;
    }
    return PAIR_ACCEPT;
}

bwp_result_t pairing_result(pair_verdict_t v)
{
    switch (v) {
    case PAIR_ACCEPT:
    case PAIR_STORE_KEY:
        return BWP_OK;
    case PAIR_NOT_PAIRED:
        return BWP_E_NOT_PAIRED;
    case PAIR_ALREADY_PAIRED:
        return BWP_E_ALREADY_PAIRED;
    case PAIR_BAD_KEY:
        return BWP_E_BAD_REQUEST;
    default:
        return BWP_E_AUTH;
    }
}
