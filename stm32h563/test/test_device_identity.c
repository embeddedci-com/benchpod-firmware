/*
 * test_device_identity.c — which identity-sector contents get a new device key (FW-10).
 *
 * A record with another magic or version used to be taken as "no identity" and a new key was
 * generated over it, silently orphaning the pod's cloud registration. Only an erased sector may
 * get a new key; anything else is left alone.
 */
#include "device_identity.h"

#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); failures++; } \
} while (0)

int main(void) {
    device_identity_rec_t r;

    memset(&r, 0xFF, sizeof(r));
    CHECK(device_identity_rec_state(&r) == DEVICE_ID_REC_BLANK);

    memset(&r, 0, sizeof(r));
    r.magic = DEVICE_ID_REC_MAGIC;
    r.version = DEVICE_ID_REC_VERSION;
    memset(r.seed, 0x5A, sizeof(r.seed));
    CHECK(device_identity_rec_state(&r) == DEVICE_ID_REC_VALID);

    /* A newer (or older) layout of our own record: not ours to replace. */
    r.version = DEVICE_ID_REC_VERSION + 1u;
    CHECK(device_identity_rec_state(&r) == DEVICE_ID_REC_FOREIGN);
    r.version = 0;
    CHECK(device_identity_rec_state(&r) == DEVICE_ID_REC_FOREIGN);

    /* Another magic (the old RP2350 record, or something else entirely). */
    r.version = DEVICE_ID_REC_VERSION;
    r.magic = 0xC0FFEE01u;
    CHECK(device_identity_rec_state(&r) == DEVICE_ID_REC_FOREIGN);

    /* All zeros, and an erased sector with one stray programmed byte. */
    memset(&r, 0, sizeof(r));
    CHECK(device_identity_rec_state(&r) == DEVICE_ID_REC_FOREIGN);
    memset(&r, 0xFF, sizeof(r));
    r.reserved[3] = 0xFFFFFF00u;
    CHECK(device_identity_rec_state(&r) == DEVICE_ID_REC_FOREIGN);

    if (failures) { printf("test_device_identity: %d FAILED\n", failures); return 1; }
    printf("test_device_identity: all passed\n");
    return 0;
}
