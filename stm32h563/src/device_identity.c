#include "device_identity.h"
#include "flash_compat.h"
#include "b64url.h"

#include "rng.h"             /* rng_fill — TRNG with error reporting */
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/regs/addressmap.h"   /* XIP_BASE */

#include "monocypher-ed25519.h"

#include <string.h>
#include <stdio.h>
#include <stdbool.h>

/* ---- Persistent identity record -------------------------------------------
 *
 * Stored in its own 4 KB sector at flash offset 0x3FE000 — the sector just
 * below the WiFi config sector (0x3FF000, see config_store.c).  Survives
 * `make flash` for the same reasons the WiFi config does: the firmware ELF is
 * tiny and lives at the bottom of flash, and OpenOCD only programs ELF-loadable
 * segments, so this sector is never touched.  Keeping it separate from
 * config_t means config_clear() ("wifi-clear") does NOT wipe the device key.
 * ---------------------------------------------------------------------------*/

#define IDENTITY_FLASH_OFFSET 0x1FC000u   /* STM32H5: bank2 sector 126 (8 KB) */
#define IDENTITY_MAGIC        0xC0FFEE02u   /* distinct from CONFIG_MAGIC */
#define IDENTITY_VERSION      1u

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint8_t  seed[32];      /* Ed25519 private seed (RFC 8032 private key) */
    uint32_t reserved[4];
} identity_record_t;
/* sizeof = 4 + 4 + 32 + 16 = 56 B  (fits one 256 B program page) */

_Static_assert(sizeof(identity_record_t) <= FLASH_PAGE_SIZE,
               "identity_record_t must fit in a single 256 B flash page");

/* ---- In-RAM key material --------------------------------------------------*/

static uint8_t s_secret_key[64];               /* Ed25519 expanded secret key */
static uint8_t s_public_key[DEVICE_ID_PUBLIC_LEN];
static bool    s_ready = false;

#if PICO_NO_FLASH
/* RAM-only (no_flash) build: no flash to read/persist (e.g. an unusable flash
 * chip — see 'make flash SRAM=ON').  Back the identity with a volatile RAM
 * shadow.  Zero-initialised, so its magic != IDENTITY_MAGIC on first boot and
 * a fresh Ed25519 key is generated and "persisted" into the shadow — kept only
 * in RAM, regenerated on the next power cycle. */
static identity_record_t s_ram_id;

static const identity_record_t *flash_identity(void) {
    return &s_ram_id;
}
#endif   /* (flash builds read the record through flash_read_checked) */

/* Fill seed[32] from the STM32H5 TRNG (rng.c).  Returns 0, or -1 if the TRNG
   reported an error or produced an obviously dead value (every byte equal,
   e.g. all zeros).  This key is permanent, so never make one from bad entropy. */
static int fill_seed(uint8_t seed[32]) {
    if (rng_fill(seed, 32) != 0) return -1;
    bool constant = true;
    for (int i = 1; i < 32; i++) if (seed[i] != seed[0]) { constant = false; break; }
    if (constant) { memset(seed, 0, 32); return -1; }
    return 0;
}

/* Erase the identity sector and program a single page holding *rec.
   Returns 0 on success (verified by read-back), -1 otherwise. */
static int identity_persist(const identity_record_t *rec) {
#if PICO_NO_FLASH
    /* No flash: keep the record in the volatile RAM shadow (lost on power
       cycle).  Subsequent flash_identity() reads see it as valid this boot. */
    memcpy(&s_ram_id, rec, sizeof(s_ram_id));
    printf("[id] (sram) identity kept in RAM shadow (volatile — new key each boot)\n");
    return 0;
#else
    uint8_t page[FLASH_PAGE_SIZE];
    memset(page, 0xFF, sizeof(page));
    memcpy(page, rec, sizeof(*rec));

    printf("[id] persisting identity to flash offset 0x%08x...\n",
           (unsigned)IDENTITY_FLASH_OFFSET);

    /* Same single-core, IRQs-off discipline as config_store.c: XIP code cannot
       execute while flash is being erased/programmed. */
    uint32_t irqs = save_and_disable_interrupts();
    flash_range_erase(IDENTITY_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(IDENTITY_FLASH_OFFSET, page, FLASH_PAGE_SIZE);
    restore_interrupts(irqs);

    identity_record_t back;
    if (flash_read_checked(IDENTITY_FLASH_OFFSET, &back, sizeof(back)) != 0 ||
        memcmp(&back, rec, sizeof(back)) != 0) {
        printf("[id] ERROR: identity write verification failed\n");
        return -1;
    }
    return 0;
#endif
}

void device_identity_init(void) {
    identity_record_t rec;
#if PICO_NO_FLASH
    memcpy(&rec, flash_identity(), sizeof(rec));
#else
    /* Through the ECC-checked read: a plain pointer read of a damaged quad-word (a power cut
       during the one-time write) raises an NMI and resets the pod on every boot.  And an
       unreadable sector must NOT be taken as "no key": generating a new one would overwrite
       the key this pod is registered with.  Run without an identity instead (cloud auth and
       proof-of-possession refuse) and say so. */
    if (flash_read_checked(IDENTITY_FLASH_OFFSET, &rec, sizeof(rec)) != 0) {
        printf("[id] ERROR: the identity sector is unreadable (flash ECC error); running "
               "without an identity, NOT generating a new key over it\n");
        memset(&rec, 0, sizeof(rec));
        s_ready = false;
        return;
    }
#endif

    if (rec.magic == IDENTITY_MAGIC && rec.version == IDENTITY_VERSION) {
        /* Existing key — load it, never overwrite. */
        printf("[id] loaded device identity from flash\n");
    } else {
        /* First boot (or unknown schema): generate and persist a new key. */
        printf("[id] no identity in flash — generating new Ed25519 key\n");
        memset(&rec, 0, sizeof(rec));
        rec.magic   = IDENTITY_MAGIC;
        rec.version = IDENTITY_VERSION;
        if (fill_seed(rec.seed) != 0) {
            /* No usable entropy: refuse rather than persist a guessable key.
               The pod runs without an identity (cloud auth / pop refuse) and
               tries again on the next boot. */
            printf("[id] ERROR: hardware RNG failed; NOT generating a device key\n");
            memset(&rec, 0, sizeof(rec));
            s_ready = false;
            return;
        }
        if (identity_persist(&rec) != 0) {
            /* Persist failed: continue with the in-RAM key this boot so the
               device is still usable, but it won't survive a reboot. */
            printf("[id] WARNING: identity not persisted — key is volatile this boot\n");
        }
    }

    /* Derive the keypair.  crypto_ed25519_key_pair WIPES its seed argument, so
       pass a scratch copy and keep rec.seed intact (already in flash anyway). */
    uint8_t seed_copy[32];
    memcpy(seed_copy, rec.seed, sizeof(seed_copy));
    crypto_ed25519_key_pair(s_secret_key, s_public_key, seed_copy);
    s_ready = true;

    char pub_b64[B64URL_ENCODED_LEN(DEVICE_ID_PUBLIC_LEN) + 1];
    b64url_encode(s_public_key, sizeof(s_public_key), pub_b64, sizeof(pub_b64));
    printf("[id] device public key (ed25519): %s\n", pub_b64);
}

int device_identity_get_public(uint8_t pub[DEVICE_ID_PUBLIC_LEN]) {
    if (!s_ready) return -1;
    memcpy(pub, s_public_key, DEVICE_ID_PUBLIC_LEN);
    return 0;
}

int device_identity_sign_ctx(const char *context, const uint8_t *msg, size_t len,
                             uint8_t sig[DEVICE_ID_SIG_LEN]) {
    if (!s_ready || !context) return -1;
    size_t clen = strlen(context);
    /* Signed message = context || 0x00 || msg.  The NUL separator makes the
       encoding unambiguous even if one context were a prefix of another. */
    if (clen + 1 + len > DEVICE_ID_SIGN_MSG_MAX) return -2;
    uint8_t buf[DEVICE_ID_SIGN_MSG_MAX];
    memcpy(buf, context, clen);
    buf[clen] = 0x00;
    memcpy(buf + clen + 1, msg, len);
    crypto_ed25519_sign(sig, s_secret_key, buf, clen + 1 + len);
    return 0;
}
