/*
 * test_device_identity.c: which identity-sector contents get a new device key (FW-10).
 *
 * A record with another magic or version used to be taken as "no identity" and a new key was
 * generated over it, silently orphaning the pod's cloud registration. Only an erased sector may
 * get a new key; anything else is left alone. The USB console's identity-wipe is the way back:
 * it needs the current short id (or "unknown") as confirmation, then erases the sector and makes a
 * new key. device_identity.c runs here on the flash model with the real Ed25519 code.
 */
#include "device_identity.h"
#include "mock_flash.h"

#include <stdio.h>
#include <string.h>

#define ID_OFF 0x1FC000u   /* IDENTITY_FLASH_OFFSET */

/* The TRNG: a counter (distinct seeds per call), or a failure on demand. */
static int rng_fail;
static uint8_t rng_next = 1;
int rng_fill(void *buf, size_t len) {
    if (rng_fail) return -1;
    uint8_t *b = buf;
    for (size_t i = 0; i < len; i++) b[i] = (uint8_t)(rng_next + i * 7u);
    rng_next++;
    return 0;
}

static int failures;
#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); failures++; } \
} while (0)

static device_identity_rec_t stored(void) {
    device_identity_rec_t r;
    memcpy(&r, mock_flash_ptr(ID_OFF), sizeof(r));
    return r;
}

static void test_first_boot_and_wipe(void) {
    mock_flash_reset();
    device_identity_init();
    char id1[DEVICE_ID_SHORT_LEN + 1], id2[DEVICE_ID_SHORT_LEN + 1];
    device_identity_short_id(id1);
    CHECK(strlen(id1) == DEVICE_ID_SHORT_LEN);
    device_identity_rec_t r = stored();
    CHECK(device_identity_rec_state(&r) == DEVICE_ID_REC_VALID);
    uint8_t seed1[32];
    memcpy(seed1, r.seed, 32);

    /* A wrong or missing token changes nothing and says which token to use. */
    int erases = mock_flash_erases;
    const char *why = device_identity_wipe("000000");
    CHECK(why && strstr(why, id1) != NULL);
    CHECK(device_identity_wipe(NULL) != NULL);
    CHECK(device_identity_wipe("unknown") != NULL);   /* the pod HAS an identity */
    CHECK(mock_flash_erases == erases);
    device_identity_short_id(id2);
    CHECK(strcmp(id1, id2) == 0);

    CHECK(device_identity_wipe(id1) == NULL);
    CHECK(mock_flash_erases == erases + 1);
    r = stored();
    CHECK(device_identity_rec_state(&r) == DEVICE_ID_REC_VALID);
    CHECK(memcmp(r.seed, seed1, 32) != 0);
    device_identity_short_id(id2);
    CHECK(strlen(id2) == DEVICE_ID_SHORT_LEN);
    CHECK(device_identity_problem()[0] == '\0');
    /* The new key is the one a reboot loads. */
    device_identity_init();
    char id3[DEVICE_ID_SHORT_LEN + 1];
    device_identity_short_id(id3);
    CHECK(strcmp(id2, id3) == 0);
    CHECK(mock_flash_irq_depth == 0);
}

static void test_foreign_record_recovery(void) {
    mock_flash_reset();
    device_identity_rec_t r;
    memset(&r, 0, sizeof(r));
    r.magic = DEVICE_ID_REC_MAGIC;
    r.version = DEVICE_ID_REC_VERSION + 1u;
    memset(r.seed, 0x33, sizeof(r.seed));
    mock_flash_poke(ID_OFF, &r, sizeof(r));
    device_identity_init();
    char id[DEVICE_ID_SHORT_LEN + 1];
    device_identity_short_id(id);
    CHECK(id[0] == '\0');                                   /* offline, not a new key */
    CHECK(strstr(device_identity_problem(), "unknown identity record") != NULL);
    device_identity_rec_t back = stored();
    CHECK(memcmp(&back, &r, sizeof(r)) == 0);              /* left alone */
    uint8_t pub[DEVICE_ID_PUBLIC_LEN];
    CHECK(device_identity_get_public(pub) != 0);

    const char *why = device_identity_wipe("abcdef");
    CHECK(why && strstr(why, "identity-wipe unknown") != NULL);
    back = stored();
    CHECK(memcmp(&back, &r, sizeof(r)) == 0);

    /* No entropy: refused before anything is erased. */
    rng_fail = 1;
    int erases = mock_flash_erases;
    CHECK(device_identity_wipe("unknown") != NULL);
    CHECK(mock_flash_erases == erases);
    rng_fail = 0;

    CHECK(device_identity_wipe("unknown") == NULL);
    back = stored();
    CHECK(device_identity_rec_state(&back) == DEVICE_ID_REC_VALID);
    device_identity_short_id(id);
    CHECK(strlen(id) == DEVICE_ID_SHORT_LEN);
    CHECK(device_identity_get_public(pub) == 0);
    uint8_t sig[DEVICE_ID_SIG_LEN];
    CHECK(device_identity_sign_ctx(DEVICE_ID_CTX_POP, (const uint8_t *)"n", 1, sig) == 0);
}

static void test_wipe_check(void) {
    CHECK(device_identity_wipe_check("a1b2c3", "a1b2c3") == NULL);
    CHECK(device_identity_wipe_check("unknown", "") == NULL);
    CHECK(device_identity_wipe_check("A1B2C3", "a1b2c3") != NULL);
    CHECK(device_identity_wipe_check("benchpod-a1b2c3", "a1b2c3") != NULL);
    CHECK(device_identity_wipe_check("", "") != NULL);
    CHECK(device_identity_wipe_check(NULL, "a1b2c3") != NULL);
}

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

    test_first_boot_and_wipe();
    test_foreign_record_recovery();
    test_wipe_check();

    if (failures) { printf("test_device_identity: %d FAILED\n", failures); return 1; }
    printf("test_device_identity: all passed\n");
    return 0;
}
