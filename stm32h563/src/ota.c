#include "ota.h"
#include "psram.h"
#include "signal_engine.h"   /* quiesce the gateware PSRAM masters for the whole OTA session */

#include "mbedtls/sha256.h"
#include <stdbool.h>

#include <string.h>
#include <stdio.h>

/* Staging base in PSRAM.  The 8 MB PSRAM dwarfs a <=2 MB image; OTA is exclusive
   with captures (the command handler refuses ota_begin while a heavy op is in
   flight), so reusing offset 0 is safe. */
#define OTA_PSRAM_BASE   0u
/* Read-back / hashing chunk (also the commit copy granularity by sector). */
#define OTA_CHUNK        4096u

static ota_state_t s_state = OTA_IDLE;
static uint32_t    s_size;                 /* expected image size          */
static uint32_t    s_received;             /* high-water extent staged     */
static uint8_t     s_expect[32];           /* expected SHA-256             */
static char        s_err[64];
static uint32_t    s_wd_seen;              /* s_received at the last watchdog mark */
static uint32_t    s_wd_mark_ms;           /* caller clock at that mark            */

/* How long a RECEIVING session may go with no new bytes before we abandon it.
   Longer than the server's own 60 s stall timeout, so the server reports the failure first and
   this is purely the device cleaning up after it. */
#define OTA_STAGE_IDLE_MS  120000u

static int hex_nibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static int parse_sha256_hex(const char *hex, uint8_t out[32]) {
    if (!hex) return -1;
    for (int i = 0; i < 32; i++) {
        int hi = hex_nibble(hex[i * 2]);
        int lo = hex_nibble(hex[i * 2 + 1]);
        if (hi < 0 || lo < 0) return -1;
        out[i] = (uint8_t)((hi << 4) | lo);
    }
    return hex[64] == '\0' ? 0 : -1;   /* exactly 64 hex chars */
}

static void set_err(const char *e) {
    strncpy(s_err, e ? e : "", sizeof(s_err) - 1);
    s_err[sizeof(s_err) - 1] = '\0';
    s_state = OTA_ERROR;
    printf("[ota] error: %s\n", s_err);
}

int ota_begin(uint32_t size, const char *sha256_hex) {
    if (size == 0 || size > OTA_MAX_SIZE) { set_err("bad size"); return -1; }
    uint8_t expect[32];
    if (parse_sha256_hex(sha256_hex, expect) != 0) { set_err("bad sha256"); return -1; }
    memcpy(s_expect, expect, 32);
    s_size     = size;
    s_received = 0;
    s_err[0]   = '\0';
    s_state    = OTA_RECEIVING;
    s_wd_seen = 0;
    s_wd_mark_ms = 0;
    /* Stop the gateware PSRAM masters for the whole OTA session: every stage/verify chunk grabs
       the shared bus (psram_bus_acquire), and a live DAC replay/reader mid-burst would contend
       and wedge the PSRAM.  Firmware OTA reboots the device anyway, so stopping a running DAC
       here is the correct behaviour.  ota_commit() re-quiesces as a brick-critical backstop. */
    signal_engine_quiesce_psram_masters();
    printf("[ota] begin: %lu bytes\n", (unsigned long)size);
    return 0;
}

int ota_data(uint32_t offset, const uint8_t *buf, uint32_t len) {
    if (s_state != OTA_RECEIVING) { set_err("not receiving"); return -1; }
    if (len == 0) return 0;
    if (offset > s_size || len > s_size - offset) { set_err("chunk out of range"); return -1; }

    psram_bus_acquire();
    int rc = psram_write(OTA_PSRAM_BASE + offset, buf, len);
    psram_bus_release();
    if (rc != 0) { set_err("psram write failed"); return -1; }

    uint32_t end = offset + len;
    if (end > s_received) s_received = end;
    return 0;
}

/* SHA-256 of the staged image == the expected one?  0 = match, 1 = mismatch, -1 = read error.
   held: the caller already owns the PSRAM bus (ota_commit); otherwise it is taken per chunk. */
static int staged_hash_check(bool held) {
    mbedtls_sha256_context ctx;
    mbedtls_sha256_init(&ctx);
    mbedtls_sha256_starts(&ctx, 0);   /* 0 = SHA-256 (not 224) */

    static uint8_t chunk[OTA_CHUNK];   /* worker task, single-threaded */
    uint32_t off = 0;
    int rc = 0;
    while (off < s_size) {
        uint32_t n = s_size - off;
        if (n > OTA_CHUNK) n = OTA_CHUNK;
        if (!held) psram_bus_acquire();
        rc = psram_read(OTA_PSRAM_BASE + off, chunk, n);
        if (!held) psram_bus_release();
        if (rc != 0) { set_err("psram read failed"); mbedtls_sha256_free(&ctx); return -1; }
        mbedtls_sha256_update(&ctx, chunk, n);
        off += n;
    }
    uint8_t got[32];
    mbedtls_sha256_finish(&ctx, got);
    mbedtls_sha256_free(&ctx);
    return memcmp(got, s_expect, 32) != 0 ? 1 : 0;
}

int ota_end(void) {
    if (s_state != OTA_RECEIVING) { set_err("not receiving"); return -1; }
    if (s_received < s_size) { set_err("incomplete image"); return -1; }
    int rc = staged_hash_check(false);
    if (rc < 0) return -1;
    if (rc > 0) { set_err("sha256 mismatch"); return -1; }
    s_state = OTA_VERIFIED;
    printf("[ota] verified: sha256 OK, %lu bytes staged\n", (unsigned long)s_size);
    return 0;
}

int ota_reverify_held(void) {
    if (s_state != OTA_VERIFIED) return -1;
    int rc = staged_hash_check(true);
    if (rc > 0) set_err("staged image changed after verify (sha256 mismatch)");
    return rc == 0 ? 0 : -1;
}

void ota_abort(void) {
    s_state    = OTA_IDLE;
    s_size     = 0;
    s_received = 0;
    s_err[0]   = '\0';
    printf("[ota] aborted\n");
}

ota_state_t ota_get_state(void) { return s_state; }
/* Abandon a staging session that has stopped receiving.
 *
 * Without this the pod sits in OTA_RECEIVING FOREVER after an interrupted push — measured: an
 * ota_begin with no data left it "receiving" indefinitely, and only an explicit ota_abort ever
 * cleared it. That is what made a wedged transfer leave the pod DEGRADED rather than merely
 * failing one job: while an OTA is nominally in flight the cloud client uses its longer
 * OTA_IDLE_TIMEOUT_MS budget, so it is slower to notice a dead downlink and reconnect, and the
 * staging claim keeps the PSRAM path occupied. The next command or update then fails against a
 * device that is still "offline", which reads as the pod being broken long after the transfer
 * that broke it.
 *
 * `now_ms` is supplied by the caller (HAL_GetTick on target) rather than read here, so this file
 * stays free of any HAL dependency and builds natively for the host tests — which also lets the
 * watchdog be driven with a synthetic clock instead of waiting out real minutes.
 *
 * Call it from the task that owns OTA + PSRAM, on every pass, so it runs whatever the cloud link
 * is doing — including while it is down, which is exactly when it is needed. */
void ota_watchdog(uint32_t now_ms) {
    if (s_state != OTA_RECEIVING) {
        s_wd_mark_ms = now_ms;
        return;
    }
    if (s_received != s_wd_seen || s_wd_mark_ms == 0) {   /* progress: re-arm */
        s_wd_seen    = s_received;
        s_wd_mark_ms = now_ms ? now_ms : 1;
        return;
    }
    if ((uint32_t)(now_ms - s_wd_mark_ms) < OTA_STAGE_IDLE_MS) return;
    printf("[ota] abandoning stalled staging after %u ms with no data (%lu/%lu bytes)\n",
           (unsigned)OTA_STAGE_IDLE_MS, (unsigned long)s_received, (unsigned long)s_size);
    ota_abort();
}

uint32_t    ota_received(void)  { return s_received; }
uint32_t    ota_size(void)      { return s_size; }
const char *ota_error(void)     { return s_err; }

const char *ota_state_str(void) {
    switch (s_state) {
        case OTA_IDLE:      return "idle";
        case OTA_RECEIVING: return "receiving";
        case OTA_VERIFIED:  return "verified";
        case OTA_ERROR:     return "error";
        default:            return "unknown";
    }
}

/* ota_commit() is defined in ota_commit.c (RAM-resident flash writer). */
