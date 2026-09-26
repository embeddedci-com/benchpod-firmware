#ifndef CONFIG_STORE_H
#define CONFIG_STORE_H

#include <stdint.h>
#include <stddef.h>

/* ---- Power-loss-safe record store (A/B slots) --------------------------------
 *
 * Used for the Wi-Fi config (below) and the cloud config (cloud_config.h).
 *
 * The old layout erased ONE sector and programmed it. A power loss in that window
 * lost the config, and on the H5 a torn quad-word program leaves an ECC double
 * error, so every later read raised an NMI -> reset -> crash loop.
 *
 * Now each store owns TWO 8 KB sectors (slots). A record is:
 *
 *   +0   header quad-word: magic | seq | (version << 16 | len) | crc32
 *   +16  payload (len bytes, padded with 0xFF to a quad-word)
 *
 * crc32 covers seq, the version/len word and the payload. A save goes to the slot
 * that does NOT hold the newest record: erase it, program the payload, program the
 * header LAST, then read it back. Until the header lands the slot reads as blank,
 * and until the save verifies, the other slot (lower seq) is still the newest, so
 * a power loss at any point leaves the previous config readable.
 *
 * Load picks the valid slot with the highest seq (serial-number compare); a slot
 * failing magic or CRC is ignored. All reads go through flash_read_checked(), so a
 * torn quad-word is "invalid" rather than a crash (needs the NMI hook described in
 * flash_compat.h; without it a read of a torn quad-word still resets, but the
 * header-last order means only a tear DURING the header program or the erase can
 * leave one where the loader looks).
 *
 * A clear writes a tombstone (len 0) as the newest record, then erases the other
 * slot and the legacy sector: power loss mid-clear can never resurrect an old
 * config.
 *
 * Legacy: pods from before this layout kept the struct raw at offset 0 of one
 * sector. When neither slot holds a record, the loader reads that sector (inner
 * magic must match). The next save writes the new format; the legacy sector is
 * left alone until a clear, so it stays a fallback until then.
 *
 * Flash map (bank 2, 8 KB sectors, offsets from 0x08000000):
 *   0x1F0000  s120  OTA self-test scratch (ota_commit.c); FLASH_BLOBS ends here
 *   0x1F2000  s121  cloud config slot B
 *   0x1F4000  s122  cloud config slot A
 *   0x1F6000  s123  Wi-Fi config slot B
 *   0x1F8000  s124  Wi-Fi config slot A
 *   0x1FA000  s125  cloud config, legacy layout (read-only fallback)
 *   0x1FC000  s126  device identity (device_identity.c, unchanged)
 *   0x1FE000  s127  Wi-Fi config, legacy layout (read-only fallback)
 * ---------------------------------------------------------------------------*/

#define AB_STORE_MAX_PAYLOAD  240u     /* record (16 B header + payload) <= 256 B */

typedef struct {
    const char *tag;           /* log prefix, e.g. "cfg" */
    uint32_t    slot_off[2];   /* flash offsets of the two slots (sector aligned) */
    uint32_t    magic;         /* record magic (distinct per store) */
    uint16_t    version;       /* payload schema version stored in the header */
    uint32_t    legacy_off;    /* old single-sector layout; 0 = none */
    uint32_t    legacy_magic;  /* first word of a valid legacy struct */
} ab_store_t;

/* Copy the newest record's payload into out (exactly len bytes).
   0 = loaded (from a slot or, failing that, the legacy sector);
   -1 = none, cleared, or a record of another length/version. */
int  ab_store_load(const ab_store_t *s, void *out, size_t len);

/* Write data as the newest record (no-op if identical to the newest). 0 on success. */
int  ab_store_save(const ab_store_t *s, const void *data, size_t len);

/* Tombstone + erase the older slot and the legacy sector. 0 on success. */
int  ab_store_clear(const ab_store_t *s);

/* CRC-32 (IEEE 802.3, reflected, init/xorout 0xFFFFFFFF). Chainable: pass the
   previous result as crc, 0 to start. */
uint32_t ab_crc32(uint32_t crc, const void *data, size_t n);

/* ---- Wi-Fi config -----------------------------------------------------------*/

#define CONFIG_SLOT_A_OFFSET  0x1F8000u   /* bank2 sector 124 */
#define CONFIG_SLOT_B_OFFSET  0x1F6000u   /* bank2 sector 123 */
#define CONFIG_FLASH_OFFSET   0x1FE000u   /* legacy single-sector layout, bank2 s127 */
#define CONFIG_RECORD_MAGIC   0xAB5C0F01u
#define CONFIG_MAGIC          0xC0FFEE01u /* inner struct magic (also the legacy one) */
#define CONFIG_VERSION        1u

#define CONFIG_SSID_MAX       64
#define CONFIG_PASSWORD_MAX   64

typedef struct {
    uint32_t magic;                          /* CONFIG_MAGIC */
    uint32_t version;                        /* CONFIG_VERSION */
    char     ssid[CONFIG_SSID_MAX];          /* null-terminated */
    char     password[CONFIG_PASSWORD_MAX];  /* null-terminated */
    uint32_t reserved[8];                    /* headroom for future fields */
} config_t;
/* sizeof(config_t) = 4 + 4 + 64 + 64 + 32 = 168 B */

/* Returns 0 if a valid config was found and copied into *out.
   Returns -1 if there is none, it was cleared, or it has an unknown schema. */
int  config_load(config_t *out);

/* Store *cfg (magic/version are filled in). Power-loss safe: after a power loss
   the loader sees either the new config or the previous one.
   Returns 0 on success, -1 on failure. */
int  config_save(const config_t *cfg);

/* Forget the Wi-Fi config ("factory reset" for the credentials). */
void config_clear(void);

#endif /* CONFIG_STORE_H */
