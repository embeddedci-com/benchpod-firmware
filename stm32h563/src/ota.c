#include "ota.h"
#include "psram.h"
#include "signal_engine.h"   /* quiesce the gateware PSRAM masters for the whole OTA session */
#include "flash_layout.h"
#include "fw_info.h"
#include "blob_store.h"
#include "w25q.h"
#include "fw_sign.h"
#include "b64url.h"

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
static ota_target_t s_target = OTA_TARGET_FIRMWARE;
static uint32_t    s_version;              /* gateware version of a blob   */
static char        s_err[64];
static fw_sig_result_t s_sig = FW_SIG_NONE;  /* last begin's signature check */
static char        s_sig_key[2 * FW_SIGN_KEY_ID_LEN + 1];
static uint32_t    s_wd_seen;              /* s_received at the last watchdog mark */
static uint32_t    s_wd_mark_ms;           /* caller clock at that mark            */

/* How long a RECEIVING session may go with no new bytes before we abandon it.
   Longer than the server's own 60 s stall timeout, so the server reports the failure first and
   this is purely the device cleaning up after it. */
#define OTA_STAGE_IDLE_MS  120000u
/* A verified image waits this long for its commit. The pod counts as busy meanwhile (captures
   refuse), so a dry run or a lost ota.commit used to keep it busy until a reboot. */
#define OTA_VERIFIED_HOLD_MS  600000u
static uint32_t    s_verified_mark_ms;     /* caller clock when VERIFIED was first seen, 0 = not */

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

uint32_t ota_max_size(void) {
    return flash_layout_store_off(FLASH_LAYOUT_STORE_BASE);
}

static const char *const k_target_names[] = { "firmware", "gw0", "gw1", "esp", "ca" };

int ota_target_from_name(const char *name) {
    if (!name || !name[0]) return OTA_TARGET_FIRMWARE;
    for (int i = 0; i < (int)(sizeof(k_target_names) / sizeof(k_target_names[0])); i++)
        if (strcmp(name, k_target_names[i]) == 0) return i;
    return -1;
}

const char *ota_target_name(ota_target_t t) {
    return (unsigned)t < sizeof(k_target_names) / sizeof(k_target_names[0]) ? k_target_names[t] : "?";
}

ota_target_t ota_target(void) { return s_target; }

/* The blob slot a target writes (only for a blob target). */
static blob_id_t target_blob(ota_target_t t) {
    return t == OTA_TARGET_CA ? BLOB_CA : (blob_id_t)(t - OTA_TARGET_GW0);
}

/* A configuration blob must make sense before it is installed (the company CA must parse);
   cloud_extras.c provides the check on the pod, the host tests take this default. 0 = fine. */
__attribute__((weak)) int ota_validate_staged(ota_target_t t, uint32_t size, char *why, size_t cap) {
    (void)t; (void)size; (void)why; (void)cap;
    return 0;
}

static uint32_t target_max_size(ota_target_t t) {
    return t == OTA_TARGET_FIRMWARE ? ota_max_size() : blob_slot_capacity(target_blob(t));
}

static void set_err(const char *e) {
    strncpy(s_err, e ? e : "", sizeof(s_err) - 1);
    s_err[sizeof(s_err) - 1] = '\0';
    s_state = OTA_ERROR;
    printf("[ota] error: %s\n", s_err);
}

int ota_begin(uint32_t size, const char *sha256_hex) {
    return ota_begin_target(size, sha256_hex, OTA_TARGET_FIRMWARE, 0);
}

void ota_refuse(const char *why) { set_err(why); }

int ota_begin_target(uint32_t size, const char *sha256_hex, ota_target_t target, uint32_t version) {
    return ota_begin_signed(size, sha256_hex, target, version, NULL, 0);
}

const char *ota_sig_result(void) { return fw_sign_result_name(s_sig); }
const char *ota_sig_key_id(void) { return s_sig_key; }

int ota_sig_decode(const char *b64, uint8_t out[FW_SIGN_MANIFEST_LEN]) {
    if (!b64 || !b64[0]) return 0;
    size_t n = strlen(b64);
    /* 128 bytes are 171 base64url characters without padding (172 with one '='). */
    if (n > 172) return -1;
    uint8_t buf[132];
    size_t got = 0;
    if (b64url_decode(b64, buf, sizeof(buf), &got) != 0 || got != FW_SIGN_MANIFEST_LEN) return -1;
    memcpy(out, buf, FW_SIGN_MANIFEST_LEN);
    return (int)got;
}

/* Check the manifest that came with the image and keep the result for the status replies.
   0 = the policy lets it in, -1 = refused (error set). */
static int check_signature(const uint8_t *sig, size_t sig_len, const uint8_t expect[32]) {
    fw_sig_manifest_t m;
    s_sig = fw_sign_check(sig, sig_len, (uint8_t)s_target, s_size, expect, &m);
    s_sig_key[0] = '\0';
    if (s_sig != FW_SIG_NONE && s_sig != FW_SIG_FORMAT)
        for (unsigned i = 0; i < FW_SIGN_KEY_ID_LEN; i++)
            snprintf(s_sig_key + 2 * i, 3, "%02x", m.key_id[i]);
    printf("[ota] signature: %s%s%s (policy %s)\n", fw_sign_result_name(s_sig),
           s_sig_key[0] ? ", key " : "", s_sig_key, fw_sign_policy_name(fw_sign_policy()));
    if (fw_sign_accept(s_sig, fw_sign_policy())) return 0;
    char e[64];
    snprintf(e, sizeof(e), "signature: %s", fw_sign_result_name(s_sig));
    set_err(e);
    return -1;
}

int ota_begin_signed(uint32_t size, const char *sha256_hex, ota_target_t target, uint32_t version,
                     const uint8_t *sig, size_t sig_len) {
    s_sig = FW_SIG_NONE;
    s_sig_key[0] = '\0';
    if ((unsigned)target > OTA_TARGET_CA) { set_err("bad target"); return -1; }
    s_target  = target;
    s_version = version;
    if (size == 0 || size > target_max_size(target)) { set_err("bad size"); return -1; }
    uint8_t expect[32];
    if (parse_sha256_hex(sha256_hex, expect) != 0) { set_err("bad sha256"); return -1; }
    s_size = size;
    /* Signatures cover code (firmware, gateware, the C3 image); the company CA is configuration
       an admin installs, so it is neither checked nor refused by the signature policy. */
    if (target != OTA_TARGET_CA && check_signature(sig, sig_len, expect) != 0) return -1;
    memcpy(s_expect, expect, 32);
    s_size     = size;
    s_received = 0;
    s_err[0]   = '\0';
    s_state    = OTA_RECEIVING;
    s_wd_seen = 0;
    s_wd_mark_ms = 0;
    s_verified_mark_ms = 0;   /* each staged image gets its own commit window */
    /* Stop the gateware PSRAM masters for the whole OTA session: every stage/verify chunk grabs
       the shared bus (psram_bus_acquire), and a live DAC replay/reader mid-burst would contend
       and wedge the PSRAM.  Firmware OTA reboots the device anyway, so stopping a running DAC
       here is the correct behaviour.  ota_commit() re-quiesces as a brick-critical backstop. */
    signal_engine_quiesce_psram_masters();
    printf("[ota] begin: %s, %lu bytes\n", ota_target_name(target), (unsigned long)size);
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

/* Does the staged image fit this pod's flash?  Its fw_info block (fw_info.h) says the smallest
   flash it was built for; an image without one is from before the 1 MB part and needs 2 MB.
   0 = fits, -1 = refused (error set). */
static int staged_fits_this_flash(void) {
    uint8_t info[sizeof(fw_info_t)];
    size_t n = 0;
    if (s_size >= FW_INFO_OFFSET + sizeof(info)) {
        psram_bus_acquire();
        int rc = psram_read(OTA_PSRAM_BASE + FW_INFO_OFFSET, info, sizeof(info));
        psram_bus_release();
        if (rc != 0) { set_err("psram read failed"); return -1; }
        n = sizeof(info);
    }
    uint32_t need_kb = fw_info_required_kb(n ? info : NULL, n);
    uint32_t have_kb = flash_layout_size() / 1024u;
    if (need_kb > have_kb) {
        char e[64];
        snprintf(e, sizeof(e), "image needs %lu KB flash, this pod has %lu KB",
                 (unsigned long)need_kb, (unsigned long)have_kb);
        set_err(e);
        return -1;
    }
    return 0;
}

/* A "required" pod must not install firmware that cannot refuse unsigned updates (fw_info.h
   FW_INFO_FLAG_ENFORCES_SIG), or one update would turn the check off. 0 = fine, -1 = refused. */
static int staged_keeps_enforcement(void) {
    if (fw_sign_policy() != FW_SIG_POLICY_REQUIRED) return 0;
    uint8_t info[sizeof(fw_info_t)];
    size_t n = 0;
    if (s_size >= FW_INFO_OFFSET + sizeof(info)) {
        psram_bus_acquire();
        int rc = psram_read(OTA_PSRAM_BASE + FW_INFO_OFFSET, info, sizeof(info));
        psram_bus_release();
        if (rc != 0) { set_err("psram read failed"); return -1; }
        n = sizeof(info);
    }
    if (fw_info_flags(n ? info : NULL, n) & FW_INFO_FLAG_ENFORCES_SIG) return 0;
    set_err("image cannot enforce signatures (policy required)");
    return -1;
}

int ota_read_staged(uint32_t off, uint8_t *buf, uint32_t n) {
    psram_bus_acquire();
    int rc = psram_read(OTA_PSRAM_BASE + off, buf, n);
    psram_bus_release();
    return rc;
}

int ota_end(void) {
    if (s_state != OTA_RECEIVING) { set_err("not receiving"); return -1; }
    if (s_received < s_size) { set_err("incomplete image"); return -1; }
    int rc = staged_hash_check(false);
    if (rc < 0) return -1;
    if (rc > 0) { set_err("sha256 mismatch"); return -1; }
    if (s_target == OTA_TARGET_FIRMWARE && staged_fits_this_flash() != 0) return -1;
    if (s_target == OTA_TARGET_FIRMWARE && staged_keeps_enforcement() != 0) return -1;
    if (s_target == OTA_TARGET_CA) {
        char why[64] = "";
        if (ota_validate_staged(s_target, s_size, why, sizeof(why)) != 0) { set_err(why[0] ? why : "invalid"); return -1; }
    }
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
    s_target   = OTA_TARGET_FIRMWARE;
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
    if (s_state == OTA_VERIFIED) {
        if (s_verified_mark_ms == 0) { s_verified_mark_ms = now_ms ? now_ms : 1; return; }
        if ((uint32_t)(now_ms - s_verified_mark_ms) < OTA_VERIFIED_HOLD_MS) return;
        printf("[ota] dropping a verified image never committed after %u ms\n",
               (unsigned)OTA_VERIFIED_HOLD_MS);
        ota_abort();
        return;
    }
    s_verified_mark_ms = 0;
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
        case OTA_INSTALLED: return "installed";
        default:            return "unknown";
    }
}

/* Blob data straight from the staging area; the bus is already held (w25q_open). */
static int staged_src(void *ctx, uint32_t off, uint8_t *buf, uint32_t n) {
    (void)ctx;
    return psram_read(OTA_PSRAM_BASE + off, buf, n);
}

int ota_install_blob(void) {
    if (s_state != OTA_VERIFIED) { set_err("no verified image staged"); return -1; }
    if (s_target == OTA_TARGET_FIRMWARE) { set_err("not a blob"); return -1; }
    blob_id_t id = target_blob(s_target);
    /* The W25Q shares the bus with the PSRAM and the gateware's PSRAM masters: the same
       quiesce every bus grab needs. blob_store_write hashes what it wrote back against the
       verified digest, so anything that wrote into the staging area since ota_end is caught. */
    signal_engine_quiesce_psram_masters();
    w25q_open();
    int rc = blob_store_write(id, s_size, s_version, s_expect, staged_src, NULL);
    w25q_close();
    if (rc != 0) { set_err("blob write failed"); return -1; }
    s_state = OTA_INSTALLED;
    printf("[ota] installed %s (%lu bytes)\n", blob_name(id), (unsigned long)s_size);
    blob_store_on_installed(id);
    return 0;
}

/* Copy a verified firmware image into the W25Q FW slot, at install time only (a dry run never
   writes it). The slot survives a power cut, unlike the PSRAM staging, so the bootloader planned
   next can restore the app from it (docs/design/ota-fallback.md). blob_store_write erases the
   header first and commits last, so a cut leaves an empty slot, never a valid-looking partial
   one, and it hashes what it wrote back against the verified digest. Best effort: a failure is
   logged and the install goes on, since the copy is only groundwork today. */
int ota_store_fw_copy(void) {
    if (s_state != OTA_VERIFIED || s_target != OTA_TARGET_FIRMWARE) return -1;
    signal_engine_quiesce_psram_masters();
    w25q_open();
    int rc = blob_store_write(BLOB_FW, s_size, 0u, s_expect, staged_src, NULL);
    w25q_close();
    printf("[ota] firmware copy in the W25Q fw slot: %s\n", rc == 0 ? "stored" : "FAILED (install continues)");
    return rc;
}

/* ota_commit() is defined in ota_commit.c (RAM-resident flash writer). */
