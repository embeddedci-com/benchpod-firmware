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
#include "mocks/mock_ota_psram.h"
#include "mbedtls/sha256.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures;

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

/* The image may fill flash up to 0x081F0000 and no further: above it are the config slots and
   the device identity key, which ota_commit would erase. */
static void test_size_limit_protects_the_high_sectors(void) {
    char hex[65];
    static uint8_t one[1] = {0};
    sha256_hex(one, 1, hex);
    mock_ota_psram_reset();
    ota_abort();
    CHECK(ota_begin(0x1F0000u, hex) == 0, "an image ending exactly at 0x081F0000 was refused: %s", ota_error());
    ota_abort();
    CHECK(ota_begin(0x1F0001u, hex) != 0, "an image one byte into the config/identity sectors was accepted");
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

int main(void) {
    test_sha256_known_answer();
    test_stage_and_verify();
    test_commit_reverifies_the_staged_image();
    test_size_limit_protects_the_high_sectors();
    test_out_of_order_and_resent_chunks();
    test_corrupted_image_is_rejected();
    test_incomplete_image_is_rejected();
    test_begin_rejects_bad_parameters();
    test_rejects_out_of_range_and_out_of_order();
    test_psram_failures_surface();
    test_abort_resets_state();
    test_watchdog_abandons_stalled_staging();

    if (failures) {
        printf("test_ota: %d FAILURE(S)\n", failures);
        return 1;
    }
    printf("test_ota: all passed\n");
    return 0;
}
