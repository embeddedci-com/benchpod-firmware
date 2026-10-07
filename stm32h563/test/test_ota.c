/*
 * test_ota.c — host tests for the PSRAM-staged OTA state machine (src/ota.c).
 *
 * What this covers is the half of OTA that is pure bookkeeping, and therefore testable without a
 * pod: the accept/refuse rules (size limits, malformed digests, chunks that do not fit, calls out
 * of order), the received-byte high-water tracking across in-order, out-of-order and re-sent
 * chunks, and the verify — a real SHA-256 over exactly what was staged, so a transfer that lost or
 * misplaced a byte is rejected. The staging buffer and the shared-bus calls are mocked
 * (mocks/mock_ota_psram.c), which additionally lets the tests assert the bus is never left held
 * and that a failing PSRAM surfaces as an error rather than a silent bad image.
 *
 * NOT covered here: everything in ota_commit.c. It is register-level STM32 code (RAM-resident
 * flash erase/program, raw OCTOSPI reads) that cannot link on the host, and stubbing it would
 * only test the stub. Its own safe equivalent is `ota_selftest` on a real pod, which runs the
 * identical primitives against a scratch flash sector.
 */
#include "ota.h"
#include "fw_info.h"
#include "blob_store.h"
#include "mocks/mock_w25q.h"
#include "mocks/mock_ota_psram.h"
#include "mbedtls/sha256.h"
#include "fw_sign.h"
#include "b64url.h"
#include "psram_regions.h"

#include "vectors/fwsign_vectors.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures;

/* ota_commit lives in ota_commit.c (register-level, not linked here); its blob half is this. */
#define ota_commit_blob_for_test ota_install_blob

#define CHECK(cond, ...)                                        \
    do {                                                        \
        if (!(cond)) {                                           \
            printf("FAIL %s:%d: ", __func__, __LINE__);          \
            printf(__VA_ARGS__);                                 \
            printf("\n");                                        \
            failures++;                                          \
        }                                                        \
    } while (0)

/* Deterministic pseudo-random image: random enough that a dropped, duplicated or misordered
   chunk cannot survive the hash by luck. */
static void fill_image(uint8_t *buf, uint32_t n, unsigned seed) {
    uint32_t x = seed ? seed : 1u;
    for (uint32_t i = 0; i < n; i++) {
        x ^= x << 13; x ^= x >> 17; x ^= x << 5;   /* xorshift32 */
        buf[i] = (uint8_t)x;
    }
}

static void sha256_hex(const uint8_t *buf, uint32_t n, char out[65]) {
    uint8_t d[32];
    mbedtls_sha256_context ctx;
    mbedtls_sha256_init(&ctx);
    mbedtls_sha256_starts(&ctx, 0);
    mbedtls_sha256_update(&ctx, buf, n);
    mbedtls_sha256_finish(&ctx, d);
    mbedtls_sha256_free(&ctx);
    for (int i = 0; i < 32; i++) sprintf(out + i * 2, "%02x", d[i]);
    out[64] = '\0';
}

/* The digest ota.c compares against must be the standard one — pin it to a known answer so a
   broken hash cannot make every other test pass by agreeing with itself. */
static void test_sha256_known_answer(void) {
    char hex[65];
    sha256_hex((const uint8_t *)"abc", 3, hex);
    CHECK(strcmp(hex, "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad") == 0,
          "sha256(\"abc\") = %s", hex);
}

/* The happy path, in the chunk sizes the transport actually uses. */
static void test_stage_and_verify(void) {
    enum { N = 5000 };
    static uint8_t img[N];
    char hex[65];
    fill_image(img, N, 0xC0FFEE);
    sha256_hex(img, N, hex);

    mock_ota_psram_reset();
    ota_abort();
    CHECK(ota_begin(N, hex) == 0, "begin rejected a valid image");
    CHECK(ota_get_state() == OTA_RECEIVING, "state after begin = %s", ota_state_str());
    CHECK(mock_quiesce_calls == 1, "begin must quiesce the gateware PSRAM masters (calls=%d)",
          mock_quiesce_calls);

    for (uint32_t off = 0; off < N; off += 1024) {
        uint32_t n = (N - off) < 1024 ? (N - off) : 1024;
        CHECK(ota_data(off, img + off, n) == 0, "data at %u rejected: %s", off, ota_error());
        CHECK(ota_received() == off + n, "received = %u after staging %u", ota_received(), off + n);
    }
    CHECK(ota_size() == N, "size = %u, want %d", ota_size(), N);
    CHECK(ota_end() == 0, "verify failed on a good image: %s", ota_error());
    CHECK(ota_get_state() == OTA_VERIFIED, "state after end = %s", ota_state_str());
    CHECK(memcmp(mock_psram, img, N) == 0, "staged bytes differ from what was sent");
    CHECK(mock_psram_acquired == 0, "PSRAM bus left held (depth=%d)", mock_psram_acquired);
    CHECK(ota_error()[0] == '\0', "clean run left an error: %s", ota_error());
}

/* Chunks may arrive out of order or be re-sent; the high-water mark must not go backwards, and
   the staged image must still be exactly right. */
/* ota_commit re-hashes the staged image with the bus held right before flashing it: PSRAM 0 is
   also the LA capture region, so anything that wrote there after ota_end must be refused. */
static void test_commit_reverifies_the_staged_image(void) {
    enum { N = 3000 };
    static uint8_t img[N];
    char hex[65];
    fill_image(img, N, 0xBEEF);
    sha256_hex(img, N, hex);

    mock_ota_psram_reset();
    ota_abort();
    CHECK(ota_begin(N, hex) == 0, "begin: %s", ota_error());
    CHECK(ota_data(0, img, N) == 0, "data: %s", ota_error());
    CHECK(ota_end() == 0, "verify: %s", ota_error());
    mock_psram_acquired = 1;                      /* ota_commit holds the bus */
    CHECK(ota_reverify_held() == 0, "an untouched staged image failed the re-verify: %s", ota_error());
    CHECK(mock_psram_acquired == 1, "re-verify changed the held bus (depth=%d)", mock_psram_acquired);
    mock_psram[1234] ^= 0x5A;                     /* a capture wrote into the staged image */
    CHECK(ota_reverify_held() != 0, "a staged image changed after verify was accepted");
    CHECK(strstr(ota_error(), "changed") != NULL, "error should say the image changed: %s", ota_error());
    mock_psram_acquired = 0;
    ota_abort();
}

/* The image may fill flash up to 0x081E4000 and no further: above it are the per-pod ADC calibration, the DAC limits, the
   config slots and the device identity key, which ota_commit would erase. */
static void test_size_limit_protects_the_high_sectors(void) {
    char hex[65];
    static uint8_t one[1] = {0};
    sha256_hex(one, 1, hex);
    mock_ota_psram_reset();
    ota_abort();
    CHECK(ota_begin(0x1E4000u, hex) == 0, "an image ending exactly at 0x081E4000 was refused: %s", ota_error());
    ota_abort();
    CHECK(ota_begin(0x1E8001u, hex) != 0, "an image one byte into the per-pod ADC calibration sectors was accepted");
    CHECK(strstr(ota_error(), "size") != NULL, "error should name the size: %s", ota_error());
    ota_abort();
    CHECK(ota_begin(0x200000u, hex) != 0, "a full-flash image was accepted");
    ota_abort();
}

static void test_out_of_order_and_resent_chunks(void) {
    enum { N = 3072 };
    static uint8_t img[N];
    char hex[65];
    fill_image(img, N, 0xBEEF);
    sha256_hex(img, N, hex);

    mock_ota_psram_reset();
    ota_abort();
    CHECK(ota_begin(N, hex) == 0, "begin: %s", ota_error());

    CHECK(ota_data(2048, img + 2048, 1024) == 0, "tail chunk: %s", ota_error());
    CHECK(ota_received() == 3072, "received = %u after the tail", ota_received());
    CHECK(ota_data(0, img, 1024) == 0, "head chunk: %s", ota_error());
    CHECK(ota_received() == 3072, "a re-sent earlier chunk moved the high-water back to %u",
          ota_received());
    CHECK(ota_data(1024, img + 1024, 1024) == 0, "middle chunk: %s", ota_error());
    CHECK(ota_data(0, img, 1024) == 0, "re-send of the head: %s", ota_error());

    CHECK(ota_end() == 0, "verify failed after out-of-order staging: %s", ota_error());
    CHECK(memcmp(mock_psram, img, N) == 0, "out-of-order staging assembled the wrong image");
}

/* A transfer that lost a byte must be caught by the hash, not committed. */
static void test_corrupted_image_is_rejected(void) {
    enum { N = 2048 };
    static uint8_t img[N];
    char hex[65];
    fill_image(img, N, 0x1234);
    sha256_hex(img, N, hex);

    mock_ota_psram_reset();
    ota_abort();
    CHECK(ota_begin(N, hex) == 0, "begin: %s", ota_error());
    CHECK(ota_data(0, img, 1024) == 0, "first chunk: %s", ota_error());

    static uint8_t bad[1024];
    memcpy(bad, img + 1024, 1024);
    bad[7] ^= 0x01;                       /* one flipped bit */
    CHECK(ota_data(1024, bad, 1024) == 0, "second chunk: %s", ota_error());

    CHECK(ota_end() != 0, "a corrupted image verified");
    CHECK(ota_get_state() == OTA_ERROR, "state after a bad verify = %s", ota_state_str());
    CHECK(strstr(ota_error(), "sha256") != NULL, "error should name the hash: %s", ota_error());
}

/* An image whose last chunk never arrived must not verify — the pod would otherwise flash a
   half-written image over itself. */
static void test_incomplete_image_is_rejected(void) {
    enum { N = 4096 };
    static uint8_t img[N];
    char hex[65];
    fill_image(img, N, 0x5150);
    sha256_hex(img, N, hex);

    mock_ota_psram_reset();
    ota_abort();
    CHECK(ota_begin(N, hex) == 0, "begin: %s", ota_error());
    CHECK(ota_data(0, img, 3072) == 0, "partial stage: %s", ota_error());
    CHECK(ota_end() != 0, "an incomplete image verified");
    CHECK(strstr(ota_error(), "incomplete") != NULL, "error should say incomplete: %s", ota_error());
}

/* Bad parameters are refused up front rather than at flash-write time. */
static void test_begin_rejects_bad_parameters(void) {
    char hex[65];
    static uint8_t one[1] = { 0x42 };
    sha256_hex(one, 1, hex);

    mock_ota_psram_reset();
    ota_abort();
    CHECK(ota_begin(0, hex) != 0, "a zero-length image was accepted");
    ota_abort();
    CHECK(ota_begin(OTA_MAX_SIZE + 1, hex) != 0, "an oversized image was accepted");
    ota_abort();
    CHECK(ota_begin(1024, "not-a-digest") != 0, "a malformed digest was accepted");
    ota_abort();
    /* 63 hex chars: right alphabet, wrong length. */
    CHECK(ota_begin(1024, "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcde") != 0,
          "a short digest was accepted");
    ota_abort();
    /* 64 chars with a non-hex digit. */
    CHECK(ota_begin(1024, "z123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef") != 0,
          "a non-hex digest was accepted");
}

/* Calls out of order, and chunks that do not fit the declared image. Note the deliberate
   fail-fast design: ANY rejected call ends the session (set_err moves the state to OTA_ERROR),
   so a protocol violation cannot be shrugged off and half-staged into a flashed image — the host
   has to start over with a fresh ota_begin. */
static void test_rejects_out_of_range_and_out_of_order(void) {
    enum { N = 2048 };
    static uint8_t img[N];
    char hex[65];
    fill_image(img, N, 0x77);
    sha256_hex(img, N, hex);

    mock_ota_psram_reset();
    ota_abort();
    /* Before begin, nothing may be staged or verified. */
    CHECK(ota_data(0, img, 16) != 0, "data was accepted before begin");
    CHECK(ota_end() != 0, "end was accepted before begin");

    /* A chunk that starts past the declared end. */
    ota_abort();
    CHECK(ota_begin(N, hex) == 0, "begin: %s", ota_error());
    CHECK(ota_data(N, img, 1) != 0, "a chunk starting past the end was accepted");
    CHECK(ota_get_state() == OTA_ERROR, "an out-of-range chunk left the state at %s", ota_state_str());
    CHECK(ota_data(0, img, 16) != 0, "staging continued after a rejected chunk");

    /* A chunk that overruns the declared end. */
    ota_abort();
    CHECK(ota_begin(N, hex) == 0, "begin: %s", ota_error());
    CHECK(ota_data(N - 4, img, 8) != 0, "a chunk overrunning the end was accepted");
    CHECK(ota_get_state() == OTA_ERROR, "an overrunning chunk left the state at %s", ota_state_str());

    /* An empty chunk is a no-op on a healthy session, not an error. */
    ota_abort();
    CHECK(ota_begin(N, hex) == 0, "begin: %s", ota_error());
    CHECK(ota_data(0, img, 0) == 0, "an empty chunk should be a no-op: %s", ota_error());
    CHECK(ota_get_state() == OTA_RECEIVING, "an empty chunk changed the state to %s", ota_state_str());
    CHECK(ota_received() == 0, "an empty chunk moved the high-water to %u", ota_received());
}

/* A failing PSRAM must surface as an error — the one thing that must never happen is a bad image
   being staged silently and then committed. */
static void test_psram_failures_surface(void) {
    enum { N = 2048 };
    static uint8_t img[N];
    char hex[65];
    fill_image(img, N, 0x99);
    sha256_hex(img, N, hex);

    /* Write failure. */
    mock_ota_psram_reset();
    ota_abort();
    CHECK(ota_begin(N, hex) == 0, "begin: %s", ota_error());
    mock_psram_fail_write_at = 1100;
    CHECK(ota_data(0, img, 1024) == 0, "chunk before the failure: %s", ota_error());
    CHECK(ota_data(1024, img + 1024, 1024) != 0, "a failed PSRAM write was reported as success");
    CHECK(ota_get_state() == OTA_ERROR, "state after a write failure = %s", ota_state_str());
    CHECK(mock_psram_acquired == 0, "PSRAM bus left held after a failed write (depth=%d)",
          mock_psram_acquired);

    /* Read failure during verify. */
    mock_ota_psram_reset();
    ota_abort();
    CHECK(ota_begin(N, hex) == 0, "begin: %s", ota_error());
    CHECK(ota_data(0, img, N) == 0, "stage: %s", ota_error());
    mock_psram_fail_read_at = 512;
    CHECK(ota_end() != 0, "a failed PSRAM read verified anyway");
    CHECK(ota_get_state() == OTA_ERROR, "state after a read failure = %s", ota_state_str());
    CHECK(mock_psram_acquired == 0, "PSRAM bus left held after a failed read (depth=%d)",
          mock_psram_acquired);
}

/* Abort clears the staged image and the error, so the pod is ready for another attempt — and, on
   real hardware, stops counting OTA as a heavy operation that blocks captures. */
static void test_abort_resets_state(void) {
    enum { N = 2048 };
    static uint8_t img[N];
    char hex[65];
    fill_image(img, N, 0xABCD);
    sha256_hex(img, N, hex);

    mock_ota_psram_reset();
    ota_abort();
    CHECK(ota_begin(N, hex) == 0, "begin: %s", ota_error());
    CHECK(ota_data(0, img, 1024) == 0, "stage: %s", ota_error());
    ota_abort();
    CHECK(ota_get_state() == OTA_IDLE, "state after abort = %s", ota_state_str());
    CHECK(ota_received() == 0 && ota_size() == 0, "abort left %u/%u staged",
          ota_received(), ota_size());
    CHECK(ota_error()[0] == '\0', "abort left an error: %s", ota_error());

    /* And a fresh update runs cleanly afterwards. */
    CHECK(ota_begin(N, hex) == 0, "begin after abort: %s", ota_error());
    CHECK(ota_data(0, img, N) == 0, "stage after abort: %s", ota_error());
    CHECK(ota_end() == 0, "verify after abort: %s", ota_error());
}

/* The staging watchdog must abandon a session that stops receiving, and must NOT touch one that
   is still making progress. Driven with a synthetic clock, so this costs no wall-clock time.

   Regression: before this existed, an interrupted push left the device in OTA_RECEIVING forever —
   only an explicit ota_abort ever cleared it. That pinned the cloud client on its longer OTA idle
   budget and kept the PSRAM staging claim, so one wedged transfer left the pod looking broken
   long after the transfer itself had failed. */
static void test_watchdog_abandons_stalled_staging(void) {
    uint8_t buf[256];
    memset(buf, 0xA5, sizeof(buf));

    CHECK(ota_begin(4096, "00112233445566778899aabbccddeeff00112233445566778899aabbccddeeff") == 0,
          "begin rejected a valid image: %s", ota_error());
    CHECK(ota_get_state() == OTA_RECEIVING, "state after begin = %s", ota_state_str());

    /* Progress keeps the session alive, however long it runs overall. */
    uint32_t t = 1000;
    for (int i = 0; i < 8; i++) {
        CHECK(ota_data((uint32_t)(i * (int)sizeof(buf)), buf, sizeof(buf)) == 0,
              "data at %d rejected: %s", i * (int)sizeof(buf), ota_error());
        t += 60000;                     /* a minute between chunks: still healthy */
        ota_watchdog(t);
        CHECK(ota_get_state() == OTA_RECEIVING,
              "watchdog abandoned a session that was still progressing (state=%s)", ota_state_str());
    }

    /* Silence short of the threshold does nothing... */
    ota_watchdog(t + 1);
    ota_watchdog(t + 119000);
    CHECK(ota_get_state() == OTA_RECEIVING,
          "watchdog fired early (state=%s)", ota_state_str());

    /* ...and past it the session is abandoned, freeing the device. */
    ota_watchdog(t + 121000);
    CHECK(ota_get_state() == OTA_IDLE,
          "watchdog did not abandon a stalled session (state=%s)", ota_state_str());

    /* An idle device is left alone. */
    ota_watchdog(t + 500000);
    CHECK(ota_get_state() == OTA_IDLE, "watchdog disturbed an idle device (state=%s)",
          ota_state_str());
}

/* A verified image that is never committed (a dry run, a lost ota.commit) must not keep the pod
   busy forever: VERIFIED counts as busy, so captures refused until a reboot (seen on HW
   2026-10-04 after a cloud dry run). It is dropped after 10 min; a commit before that is fine. */
static void test_watchdog_drops_uncommitted_verified_image(void) {
    uint8_t buf[64];
    memset(buf, 0x5A, sizeof(buf));
    char hex[65];
    sha256_hex(buf, sizeof(buf), hex);
    CHECK(ota_begin(sizeof(buf), hex) == 0, "begin: %s", ota_error());
    CHECK(ota_data(0, buf, sizeof(buf)) == 0, "data: %s", ota_error());
    CHECK(ota_end() == 0, "end: %s", ota_error());
    CHECK(ota_get_state() == OTA_VERIFIED, "state after end = %s", ota_state_str());

    uint32_t t = 5000;
    ota_watchdog(t);                      /* first sight of VERIFIED starts the clock */
    ota_watchdog(t + 599000);
    CHECK(ota_get_state() == OTA_VERIFIED, "verified image dropped early (state=%s)", ota_state_str());
    ota_watchdog(t + 601000);
    CHECK(ota_get_state() == OTA_IDLE, "verified image kept past the hold (state=%s)", ota_state_str());

    /* A second verified image gets its own full hold, not the leftover of the first. */
    CHECK(ota_begin(sizeof(buf), hex) == 0, "begin 2: %s", ota_error());
    CHECK(ota_data(0, buf, sizeof(buf)) == 0 && ota_end() == 0, "stage 2: %s", ota_error());
    ota_watchdog(t + 700000);
    ota_watchdog(t + 1200000);
    CHECK(ota_get_state() == OTA_VERIFIED, "second image dropped early (state=%s)", ota_state_str());
    ota_abort();
}

/* On a 1 MB part the persistence sectors start at 0x0E4000, so that is the largest image. */
static void test_size_limit_on_a_1mb_part(void) {
    char hex[65];
    static uint8_t one[1] = {0};
    sha256_hex(one, 1, hex);
    mock_ota_psram_reset();
    mock_flash_size = 0x100000u;
    ota_abort();
    CHECK(ota_max_size() == 0x0E4000u, "1 MB limit is 0x%x", (unsigned)ota_max_size());
    CHECK(ota_begin(0x0E4000u, hex) == 0, "an image ending at the 1 MB store was refused: %s", ota_error());
    ota_abort();
    CHECK(ota_begin(0x0E8001u, hex) != 0, "an image into the 1 MB part's calibration sector was accepted");
    ota_abort();
    mock_flash_size = 0x200000u;
    CHECK(ota_max_size() == OTA_MAX_SIZE, "2 MB limit moved: 0x%x", (unsigned)ota_max_size());
}

/* Stage `img` and run the verify; returns ota_end()'s result. */
static int stage_and_end(const uint8_t *img, uint32_t n) {
    char hex[65];
    sha256_hex(img, n, hex);
    ota_abort();
    if (ota_begin(n, hex) != 0) return -2;
    for (uint32_t off = 0; off < n; off += 1024) {
        uint32_t c = (n - off) < 1024 ? (n - off) : 1024;
        if (ota_data(off, img + off, c) != 0) return -3;
    }
    return ota_end();
}

/* A 1 MB pod takes only images whose fw_info block says they fit 1 MB. Images without the block
   are from before the 1 MB part: fine on a 2 MB pod, refused on a 1 MB pod. */
static void test_image_must_fit_this_flash(void) {
    enum { N = 4096 };
    static uint8_t img[N];
    fw_info_t info = { .magic = FW_INFO_MAGIC, .layout = FW_INFO_LAYOUT, .min_flash_kb = 1024 };

    mock_ota_psram_reset();
    fill_image(img, N, 99);                       /* no block */
    CHECK(stage_and_end(img, N) == 0, "a legacy image was refused on a 2 MB pod: %s", ota_error());
    mock_flash_size = 0x100000u;
    CHECK(stage_and_end(img, N) != 0, "a legacy image was accepted on a 1 MB pod");
    CHECK(strstr(ota_error(), "2048 KB") != NULL, "error should name the size needed: %s", ota_error());

    memcpy(img + FW_INFO_OFFSET, &info, sizeof(info));   /* fits 1 MB */
    CHECK(stage_and_end(img, N) == 0, "a 1 MB image was refused on a 1 MB pod: %s", ota_error());
    mock_flash_size = 0x200000u;
    CHECK(stage_and_end(img, N) == 0, "a 1 MB image was refused on a 2 MB pod: %s", ota_error());

    info.min_flash_kb = 2048;                      /* built for 2 MB only */
    memcpy(img + FW_INFO_OFFSET, &info, sizeof(info));
    mock_flash_size = 0x100000u;
    CHECK(stage_and_end(img, N) != 0, "a 2 MB image was accepted on a 1 MB pod");
    CHECK(mock_psram_acquired == 0, "PSRAM bus left held (depth=%d)", mock_psram_acquired);
    mock_flash_size = 0x200000u;
    ota_abort();
}

/* Blob targets: the limit is the slot, there is no fw_info check, and commit writes the slot and
   leaves the pod running (state "installed") instead of resetting. */
/* Staging writes PSRAM 0, the LA capture region: a capture kept for capture_read must go stale. */
static void test_staging_makes_a_kept_capture_stale(void) {
    enum { N = 2048 };
    static uint8_t img[N];
    char hex[65];
    fill_image(img, N, 9);
    sha256_hex(img, N, hex);
    mock_ota_psram_reset();
    ota_abort();
    psram_regions_track(0x400000u, 4096u, 0x000000u, 8192u);   /* ADC + LA of a capture_dual */
    CHECK(ota_begin(N, hex) == 0, "begin: %s", ota_error());
    CHECK(psram_regions_stale() == NULL, "begin alone made the capture stale");
    CHECK(ota_data(0, img, 512) == 0, "data: %s", ota_error());
    CHECK(psram_regions_stale() && strstr(psram_regions_stale(), "firmware update"),
          "staging over the LA region did not make the capture stale");
    psram_regions_forget();
    ota_abort();
}

static void test_blob_targets(void) {
    enum { N = 6000 };
    static uint8_t img[N];
    char hex[65];
    fill_image(img, N, 5);
    sha256_hex(img, N, hex);

    CHECK(ota_target_from_name("gw1") == OTA_TARGET_GW1 && ota_target_from_name("esp") == OTA_TARGET_ESP &&
          ota_target_from_name(NULL) == OTA_TARGET_FIRMWARE && ota_target_from_name("") == OTA_TARGET_FIRMWARE &&
          ota_target_from_name("bogus") == -1, "target names");

    mock_ota_psram_reset();
    mock_w25q_reset();
    ota_abort();
    CHECK(ota_begin_target(blob_slot_capacity(BLOB_GW0) + 1, hex, OTA_TARGET_GW0, 46) != 0,
          "a gateware larger than its slot was accepted");
    ota_abort();
    CHECK(ota_begin_target(0x300000u, hex, OTA_TARGET_ESP, 0) == 0,
          "a 3 MB ESP image was refused: %s", ota_error());
    ota_abort();

    /* A blob on a 1 MB pod: no fw_info block needed. */
    mock_flash_size = 0x100000u;
    CHECK(ota_begin_target(N, hex, OTA_TARGET_GW1, 46) == 0, "begin: %s", ota_error());
    CHECK(ota_target() == OTA_TARGET_GW1, "target not kept");
    for (uint32_t off = 0; off < N; off += 1024) {
        uint32_t c = (N - off) < 1024 ? (N - off) : 1024;
        CHECK(ota_data(off, img + off, c) == 0, "data: %s", ota_error());
    }
    CHECK(ota_end() == 0, "a blob without fw_info was refused: %s", ota_error());
    int sessions = mock_w25q_sessions;
    CHECK(ota_commit_blob_for_test() == 0, "install failed: %s", ota_error());
    CHECK(mock_w25q_sessions == sessions + 1, "the install did not use a quiescing W25Q session");
    CHECK(ota_get_state() == OTA_INSTALLED, "state after install = %s", ota_state_str());
    CHECK(mock_installed[BLOB_GW1] == 1, "the installed hook did not run");
    CHECK(blob_store_present(BLOB_GW1) && blob_store_info(BLOB_GW1)->version == 46 &&
          blob_store_info(BLOB_GW1)->len == N, "slot not written");
    CHECK(blob_store_verify(BLOB_GW1) == 0, "slot data does not hash");
    CHECK(mock_psram_acquired == 0 && mock_w25q_open_depth == 0, "bus left held");
    mock_flash_size = 0x200000u;
    ota_abort();
    CHECK(ota_target() == OTA_TARGET_FIRMWARE, "abort did not reset the target");

    /* The staging area changed after verify (a capture wrote into it): the slot stays empty. */
    mock_w25q_reset();
    blob_store_load();
    CHECK(ota_begin_target(N, hex, OTA_TARGET_ESP, 0) == 0, "begin");
    for (uint32_t off = 0; off < N; off += 1024) {
        uint32_t c = (N - off) < 1024 ? (N - off) : 1024;
        ota_data(off, img + off, c);
    }
    CHECK(ota_end() == 0, "end");
    mock_psram[100] ^= 0x01;
    CHECK(ota_commit_blob_for_test() != 0, "a changed staging area was installed");
    CHECK(!blob_store_present(BLOB_ESP), "the slot is present after a failed install");
    CHECK(mock_installed[BLOB_ESP] == 0, "the installed hook ran after a failure");
    ota_abort();
}

/* Stage a vector image with its manifest and run it through end. 0 = verified. */
static int signed_stage(int image, const uint8_t *sig, size_t sig_len, ota_target_t target) {
    const uint8_t *img = vec_images[image].data;
    uint32_t n = (uint32_t)vec_images[image].len;
    char hex[65];
    sha256_hex(img, n, hex);
    ota_abort();
    if (ota_begin_signed(n, hex, target, 0, sig, sig_len) != 0) return -1;
    if (ota_data(0, img, n) != 0) return -1;
    return ota_end();
}

/* The signed manifest (fw_sign.h): this release reports the check and refuses nothing; the
   stricter policies exist for a later release and are exercised here by setting them directly. */
static void test_signed_begin(void) {
    enum { FW = 0, BLOB = 1, FW_ENF = 4 };   /* vec_images order, see tools/fwsign.py vectors */
    const uint8_t *good = vec_cases[0].sig, *flipped = vec_cases[5].sig, *enf = vec_cases[10].sig;
    mock_ota_psram_reset();
    CHECK(strcmp(vec_cases[5].want, "signature") == 0 && strcmp(vec_cases[10].want, "ok") == 0,
          "vector order changed: update this test");

    CHECK(fw_sign_policy() == FW_SIG_POLICY_AUDIT, "the default policy must be audit");
    CHECK(signed_stage(FW, good, 128, OTA_TARGET_FIRMWARE) == 0, "good: %s", ota_error());
    CHECK(strcmp(ota_sig_result(), "ok") == 0, "good: sig %s", ota_sig_result());
    CHECK(strlen(ota_sig_key_id()) == 16, "key id '%s'", ota_sig_key_id());

    CHECK(signed_stage(FW, NULL, 0, OTA_TARGET_FIRMWARE) == 0, "unsigned: %s", ota_error());
    CHECK(strcmp(ota_sig_result(), "none") == 0 && ota_sig_key_id()[0] == '\0', "unsigned: %s",
          ota_sig_result());

    /* audit: a bad signature or a wrong target is reported, the update still goes through */
    CHECK(signed_stage(FW, flipped, 128, OTA_TARGET_FIRMWARE) == 0, "audit refused: %s", ota_error());
    CHECK(strcmp(ota_sig_result(), "signature") == 0, "flipped: %s", ota_sig_result());
    CHECK(signed_stage(FW, good, 128, OTA_TARGET_GW1) == 0, "audit refused a blob: %s", ota_error());
    CHECK(strcmp(ota_sig_result(), "target") == 0, "wrong target: %s", ota_sig_result());
    CHECK(signed_stage(BLOB, vec_cases[1].sig, 128, OTA_TARGET_GW1) == 0, "blob: %s", ota_error());
    CHECK(strcmp(ota_sig_result(), "ok") == 0, "blob: %s", ota_sig_result());

    /* plain ota_begin_target is the unsigned path */
    ota_abort();
    char hex[65];
    sha256_hex(vec_images[FW].data, (uint32_t)vec_images[FW].len, hex);
    CHECK(ota_begin_target((uint32_t)vec_images[FW].len, hex, OTA_TARGET_FIRMWARE, 0) == 0, "begin");
    CHECK(strcmp(ota_sig_result(), "none") == 0, "begin_target: %s", ota_sig_result());

    fw_sign_set_policy(FW_SIG_POLICY_PERMISSIVE);
    CHECK(signed_stage(FW, flipped, 128, OTA_TARGET_FIRMWARE) != 0, "permissive took a bad signature");
    CHECK(strcmp(ota_error(), "signature: signature") == 0, "error '%s'", ota_error());
    CHECK(mock_psram_acquire_count == 0 || mock_psram_acquired == 0, "bus held");
    CHECK(signed_stage(FW, NULL, 0, OTA_TARGET_FIRMWARE) == 0, "permissive refused unsigned: %s", ota_error());

    fw_sign_set_policy(FW_SIG_POLICY_REQUIRED);
    CHECK(signed_stage(FW, NULL, 0, OTA_TARGET_FIRMWARE) != 0, "required took an unsigned image");
    CHECK(strcmp(ota_error(), "signature: none") == 0, "error '%s'", ota_error());
    /* signed, but the image could not refuse unsigned updates itself: downgrade guard */
    CHECK(signed_stage(FW, good, 128, OTA_TARGET_FIRMWARE) != 0, "required installed a non-enforcing image");
    CHECK(strstr(ota_error(), "cannot enforce") != NULL, "error '%s'", ota_error());
    CHECK(signed_stage(FW_ENF, enf, 128, OTA_TARGET_FIRMWARE) == 0, "enforcing image: %s", ota_error());
    /* blobs have no fw_info: only the signature counts */
    CHECK(signed_stage(BLOB, vec_cases[1].sig, 128, OTA_TARGET_GW1) == 0, "required blob: %s", ota_error());
    fw_sign_set_policy(FW_SIG_POLICY_AUDIT);
    CHECK(mock_psram_acquired == 0, "PSRAM bus left held (depth=%d)", mock_psram_acquired);
    ota_abort();
}

/* The transports carry the manifest as base64url. */
static void test_sig_decode(void) {
    char b64[200];
    uint8_t out[128];
    CHECK(b64url_encode(vec_cases[0].sig, 128, b64, sizeof(b64)) == 171, "encode length");
    CHECK(ota_sig_decode(b64, out) == 128 && memcmp(out, vec_cases[0].sig, 128) == 0, "round trip");
    CHECK(ota_sig_decode("", out) == 0 && ota_sig_decode(NULL, out) == 0, "empty = none");
    CHECK(ota_sig_decode("not*base64", out) < 0, "garbage accepted");
    b64[100] = '\0';
    CHECK(ota_sig_decode(b64, out) < 0, "a short manifest decoded");
}

int main(void) {
    test_sha256_known_answer();
    test_stage_and_verify();
    test_commit_reverifies_the_staged_image();
    test_size_limit_protects_the_high_sectors();
    test_size_limit_on_a_1mb_part();
    test_image_must_fit_this_flash();
    test_blob_targets();
    test_staging_makes_a_kept_capture_stale();
    test_out_of_order_and_resent_chunks();
    test_corrupted_image_is_rejected();
    test_incomplete_image_is_rejected();
    test_begin_rejects_bad_parameters();
    test_rejects_out_of_range_and_out_of_order();
    test_psram_failures_surface();
    test_abort_resets_state();
    test_watchdog_abandons_stalled_staging();
    test_watchdog_drops_uncommitted_verified_image();
    test_signed_begin();
    test_sig_decode();

    if (failures) {
        printf("test_ota: %d FAILURE(S)\n", failures);
        return 1;
    }
    printf("test_ota: all passed\n");
    return 0;
}
