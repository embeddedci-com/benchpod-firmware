#ifndef OTA_H
#define OTA_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/*
 * ota — hostless firmware update, staged in external PSRAM.
 *
 * The firmware image (~0.5 MB; it must fit the 1 MB H563's 912 KB code area) is
 * streamed into the 8 MB PSRAM, its SHA-256 verified, then a RAM-resident routine
 * erases + rewrites the internal flash from PSRAM and resets.  The same staging
 * carries the blobs (ESP32-C3 image, iCE40 gateware) to their W25Q slots.
 *
 * Transport-agnostic: the same calls are driven by the cloud WebSocket ota.*
 * frames AND by the LAN ota_* JSON commands (so OTA is testable without the
 * cloud).  All calls run on the hw worker task, which owns the PSRAM bus.
 *
 * Trust model: authenticity comes from the device-authenticated, TLS-encrypted
 * server link and the SHA-256. An image may also come with a signed manifest
 * (fw_sign.h), checked at begin; in this release that result is only reported
 * (policy "audit"), nothing is refused because of it.
 *
 * ⚠ ota_commit() overwrites the running firmware in place.  A power loss during
 * the write leaves the device needing a USB-DFU reflash — it is NOT power-atomic.
 */

/* Max image we will accept: everything below the persistence sectors at the top of flash
   (flash_layout.h): 0x1E4000 on a 2 MB part, 0x0E4000 on a 1 MB part.  There sit the pod policy, the per-pod
   ADC calibration (adc_cal.h), the DAC output limits (dac_limits.h), the OTA self-test scratch
   sector, the Wi-Fi and cloud config slots and the DEVICE IDENTITY KEY; ota_commit erases and
   rewrites up to the image size, so a larger image would wipe them, and a pod without its key
   cannot authenticate to the cloud again.  OTA_MAX_SIZE is the 2 MB part's limit. */
#define OTA_MAX_SIZE   0x1E4000u   /* 1936 KB */
uint32_t ota_max_size(void);

typedef enum {
    OTA_IDLE = 0,
    OTA_RECEIVING,
    OTA_VERIFIED,     /* staged image hash matched; ready to commit */
    OTA_ERROR,
    OTA_INSTALLED,    /* a blob was written to its W25Q slot (firmware never gets here: it resets) */
} ota_state_t;

/* What an update writes. The firmware goes to internal flash and resets; a blob goes to its
   W25Q slot (blob_store.h) and the pod carries on. */
typedef enum {
    OTA_TARGET_FIRMWARE = 0,
    OTA_TARGET_GW0,
    OTA_TARGET_GW1,
    OTA_TARGET_ESP,
    OTA_TARGET_CA,      /* the company CA for the cloud link: configuration, not code (no signature) */
} ota_target_t;

/* "firmware", "gw0", "gw1", "esp", "ca" (NULL or "" = firmware). -1 if unknown. */
int         ota_target_from_name(const char *name);
const char *ota_target_name(ota_target_t t);
ota_target_t ota_target(void);

/* Begin an update: `size` bytes total, `sha256_hex` the expected 64-char hex
   digest of the whole image.  Prepares PSRAM staging.  Returns 0, or <0 on bad
   params / busy (a heavy capture in flight). */
int ota_begin(uint32_t size, const char *sha256_hex);
/* The same for any target. `version` is stored with a gateware blob (0 = not known). */
int ota_begin_target(uint32_t size, const char *sha256_hex, ota_target_t target, uint32_t version);
/* The same with the signed manifest that came with the image (`sig` NULL or `sig_len` 0 = none).
   It is checked before staging; the policy (fw_sign_policy) decides whether a failed check
   refuses the update. The result is kept for ota_sig_result until the next begin. */
int ota_begin_signed(uint32_t size, const char *sha256_hex, ota_target_t target, uint32_t version,
                     const uint8_t *sig, size_t sig_len);
/* The last begin's signature check: fw_sign_result_name() words ("none", "ok", ...). */
const char *ota_sig_result(void);
/* key_id (16 hex) of the last begin's manifest, "" when it had none or an unreadable one. */
const char *ota_sig_key_id(void);
/* Decode a base64url manifest as the transports carry it. Returns its length (128 when valid
   base64url of the right size), 0 for an empty string, -1 when it cannot be decoded. */
int ota_sig_decode(const char *b64, uint8_t out[128]);
/* Refuse an OTA without starting it (e.g. the PSRAM bus is busy): state ERROR with `why`, which
   the cloud reports in ota.status like any other begin failure. */
void ota_refuse(const char *why);
/* Store the verified firmware image in the W25Q FW slot (install time; best effort). */
int ota_store_fw_copy(void);

/* Stage `len` bytes at image `offset` into PSRAM.  Offsets may arrive in order or
   with gaps re-sent; the received-byte high-water is tracked.  Returns 0, <0 on
   error / out of range / not begun. */
int ota_data(uint32_t offset, const uint8_t *buf, uint32_t len);

/* Finish: require the full size received, read the staged image back from PSRAM,
   compute its SHA-256 and compare to the expected digest.  On match -> state
   becomes OTA_VERIFIED and returns 0; on mismatch -> OTA_ERROR, returns <0. */
int ota_end(void);

/* Commit a VERIFIED image.  Firmware: erase + rewrite internal flash from PSRAM, then reset;
   does NOT return on success.  A blob: write it to its W25Q slot (ota_install_blob), return 0.
   Returns <0 if no verified image is staged or the write failed. */
int ota_commit(void);
/* The blob half of ota_commit: copy the verified staged blob into its slot, state -> installed.
   Calls blob_store_on_installed() so the pod can put the new blob to use. */
int ota_install_blob(void);
/* Re-hash the staged image with the PSRAM bus ALREADY held by the caller (ota_commit, right
   before it goes RAM-resident): 0 = still the verified image, -1 = changed or unreadable. */
int ota_reverify_held(void);

/* Abandon a staging session that has received nothing for too long, so an interrupted push
   cannot pin the device in OTA_RECEIVING (holding the PSRAM claim and the cloud client's long
   OTA idle budget) until someone sends an explicit ota_abort. Call periodically from the task
   that owns OTA; cheap and a no-op unless a stalled session is actually present. */
void ota_watchdog(uint32_t now_ms);

/* SAFE validation of the RAM-resident commit primitives (PSRAM read + flash
   erase/program) against a SCRATCH flash sector in the free region — never the
   running app, so it cannot brick.  Writes a known pattern to PSRAM, RAM-resident
   reads it back + programs the scratch sector, then verifies via a direct flash
   read.  Returns 0 on pass, <0 on failure (with a reason logged).  RUN THIS AND
   CONFIRM IT PASSES before ever calling ota_commit() on real hardware. */
int ota_commit_selftest(void);

/* Read staged bytes (takes the PSRAM bus). 0 = ok. For checks of a configuration blob. */
int ota_read_staged(uint32_t off, uint8_t *buf, uint32_t n);
/* Check a staged configuration blob before it is accepted (weak default: fine). 0 = fine, else
   `why` says what is wrong. */
int ota_validate_staged(ota_target_t t, uint32_t size, char *why, size_t cap);

/* Abort and free the staging state. */
void ota_abort(void);

/* Current state + progress (received bytes / expected size) for status frames. */
ota_state_t ota_get_state(void);
uint32_t    ota_received(void);
uint32_t    ota_size(void);
const char *ota_state_str(void);
/* Last error string (empty if none). */
const char *ota_error(void);

#endif /* OTA_H */
