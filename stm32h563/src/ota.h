#ifndef OTA_H
#define OTA_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/*
 * ota — hostless firmware update, staged in external PSRAM.
 *
 * The full firmware image (~1.6 MB — the STM32 app plus the embedded ESP + FPGA
 * blobs) is too big for a dual-bank A/B swap (it exceeds one 1 MB bank), so OTA
 * streams the whole image into the 8 MB PSRAM, verifies its SHA-256, then a
 * RAM-resident routine erases + rewrites the internal flash from PSRAM and resets.
 *
 * Transport-agnostic: the same calls are driven by the cloud WebSocket ota.*
 * frames AND by the LAN ota_* JSON commands (so OTA is testable without the
 * cloud).  All calls run on the hw worker task, which owns the PSRAM bus.
 *
 * Trust model (per design decision): authenticity comes from the
 * device-authenticated, TLS-encrypted server link; OTA verifies only the SHA-256
 * for integrity before committing.
 *
 * ⚠ ota_commit() overwrites the running firmware in place.  A power loss during
 * the write leaves the device needing a USB-DFU reflash — it is NOT power-atomic.
 */

/* Max image we will accept (the full internal flash). */
#define OTA_MAX_SIZE   0x200000u   /* 2 MB */

typedef enum {
    OTA_IDLE = 0,
    OTA_RECEIVING,
    OTA_VERIFIED,     /* staged image hash matched; ready to commit */
    OTA_ERROR,
} ota_state_t;

/* Begin an update: `size` bytes total, `sha256_hex` the expected 64-char hex
   digest of the whole image.  Prepares PSRAM staging.  Returns 0, or <0 on bad
   params / busy (a heavy capture in flight). */
int ota_begin(uint32_t size, const char *sha256_hex);

/* Stage `len` bytes at image `offset` into PSRAM.  Offsets may arrive in order or
   with gaps re-sent; the received-byte high-water is tracked.  Returns 0, <0 on
   error / out of range / not begun. */
int ota_data(uint32_t offset, const uint8_t *buf, uint32_t len);

/* Finish: require the full size received, read the staged image back from PSRAM,
   compute its SHA-256 and compare to the expected digest.  On match -> state
   becomes OTA_VERIFIED and returns 0; on mismatch -> OTA_ERROR, returns <0. */
int ota_end(void);

/* Commit a VERIFIED image: erase + rewrite internal flash from PSRAM, then reset.
   Does NOT return on success.  Returns <0 if no verified image is staged. */
int ota_commit(void);

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
