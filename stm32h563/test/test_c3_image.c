/* Host tests for c3_image: the record of which image the ESP32-C3 runs, now that the pod keeps no
   copy of it. */
#include "c3_image.h"
#include "blob_store.h"
#include "w25q.h"
#include "mocks/mock_w25q.h"
#include "mbedtls/sha256.h"

#include <stdio.h>
#include <string.h>

static int fails;
#define CHECK(cond, ...) do { if (!(cond)) { \
        printf("FAIL %s:%d: ", __func__, __LINE__); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

/* The esp blob this "firmware build" goes with. */
static blob_manifest_t s_want;
const blob_manifest_t *blob_manifest(blob_id_t id) { return id == BLOB_ESP ? &s_want : NULL; }
uint32_t blob_manifest_gw_version(void) { return 0; }

static uint8_t img[200000];

static void fill(uint8_t *buf, uint32_t n, uint32_t seed) {
    uint32_t x = seed ? seed : 1u;
    for (uint32_t i = 0; i < n; i++) { x ^= x << 13; x ^= x >> 17; x ^= x << 5; buf[i] = (uint8_t)x; }
}

static void sha(const uint8_t *buf, uint32_t n, uint8_t out[32]) {
    mbedtls_sha256_context c;
    mbedtls_sha256_init(&c); mbedtls_sha256_starts(&c, 0);
    mbedtls_sha256_update(&c, buf, n); mbedtls_sha256_finish(&c, out); mbedtls_sha256_free(&c);
}

static int mem_src(void *ctx, uint32_t off, uint8_t *buf, uint32_t n) {
    memcpy(buf, (const uint8_t *)ctx + off, n);
    return 0;
}

static void reboot(void) { blob_store_load(); c3_image_load(); c3_image_set_missing(false); }

static void test_layout(void) {
    CHECK(C3_IMAGE_REC_ADDR >= blob_slot_base(BLOB_ESP) + BLOB_HDR_SIZE + 0x200000u,
          "the record could collide with an image left in the slot");
    CHECK(C3_IMAGE_REC_ADDR + 0x1000u == blob_slot_base(BLOB_ESP) + BLOB_HDR_SIZE + blob_slot_capacity(BLOB_ESP),
          "the record is not the last sector of the esp slot");
    CHECK(C3_IMAGE_REC_ADDR % 0x1000u == 0, "the record is not sector aligned");
}

static void test_nothing_known(void) {
    mock_w25q_reset();
    reboot();
    CHECK(!c3_image_info()->present, "an erased flash reports a C3 image");
    CHECK(!c3_image_nvs(NULL, NULL), "an erased flash knows the NVS");
    s_want.len = 1000;
    CHECK(blob_report_state(BLOB_ESP) == BLOB_STATE_MISSING, "state %s", blob_state_str(blob_report_state(BLOB_ESP)));
}

/* Pods updated from older firmware: the copy in the slot is what the C3 was flashed from. */
static void test_legacy_copy_is_reported(void) {
    const uint32_t n = 150000;
    uint8_t d[32];
    mock_w25q_reset();
    blob_store_load();
    fill(img, n, 9); sha(img, n, d);
    CHECK(blob_store_write(BLOB_ESP, n, 3, d, mem_src, img) == 0, "legacy slot write");
    reboot();
    const blob_info_t *in = c3_image_info();
    CHECK(in->present && in->len == n && in->version == 3 && memcmp(in->sha256, d, 32) == 0,
          "the legacy copy is not reported");
    s_want.len = n; memcpy(s_want.sha256, d, 32);
    CHECK(blob_report_state(BLOB_ESP) == BLOB_STATE_OK, "state %s", blob_state_str(blob_report_state(BLOB_ESP)));
    CHECK(!c3_image_nvs(NULL, NULL), "a legacy copy has no recorded NVS");
    /* A C3 that did not start with the copy still there: the pod reflashes it itself, but a
       board without a copy reports missing. */
    c3_image_set_missing(true);
    CHECK(!c3_image_info()->present && blob_report_state(BLOB_ESP) == BLOB_STATE_MISSING, "missing not reported");
    c3_image_set_missing(false);
}

static void test_record_wins_and_survives_reboot(void) {
    const uint32_t n = 150000;
    uint8_t old[32], neu[32];
    mock_w25q_reset();
    blob_store_load();
    fill(img, n, 9); sha(img, n, old);
    blob_store_write(BLOB_ESP, n, 3, old, mem_src, img);
    reboot();
    fill(img, n + 10, 11); sha(img, n + 10, neu);
    CHECK(c3_image_save(n + 10, 4, neu, 0x9000u, 0x6000u) == 0, "save");
    CHECK(mock_w25q_bad_programs == 0, "programmed over unerased bits");
    reboot();
    const blob_info_t *in = c3_image_info();
    CHECK(in->present && in->len == n + 10 && in->version == 4 && memcmp(in->sha256, neu, 32) == 0,
          "the record is not reported over the legacy copy");
    uint32_t off = 0, size = 0;
    CHECK(c3_image_nvs(&off, &size) && off == 0x9000u && size == 0x6000u, "NVS 0x%x/0x%x", off, size);
    s_want.len = n; memcpy(s_want.sha256, old, 32);
    CHECK(blob_report_state(BLOB_ESP) == BLOB_STATE_OUTDATED, "state %s", blob_state_str(blob_report_state(BLOB_ESP)));
    /* Saving again rewrites the sector (erase first). */
    CHECK(c3_image_save(n, 3, old, 0x9000u, 0x6000u) == 0 && mock_w25q_bad_programs == 0, "second save");
    reboot();
    CHECK(blob_report_state(BLOB_ESP) == BLOB_STATE_OK, "state %s", blob_state_str(blob_report_state(BLOB_ESP)));
    /* The legacy copy is untouched by the record. */
    CHECK(blob_store_present(BLOB_ESP) && blob_store_verify(BLOB_ESP) == 0, "the legacy copy was damaged");
    /* A record without the NVS location (an automatic reflash from the legacy copy). */
    CHECK(c3_image_save(n, 3, old, 0, 0) == 0 && !c3_image_nvs(NULL, NULL), "an unknown NVS reads as known");
}

static void test_forget_and_corruption(void) {
    const uint32_t n = 150000;
    uint8_t d[32];
    mock_w25q_reset();
    blob_store_load();
    fill(img, n, 9); sha(img, n, d);
    blob_store_write(BLOB_ESP, n, 3, d, mem_src, img);
    reboot();
    CHECK(c3_image_forget() == 0, "forget");
    reboot();
    CHECK(!c3_image_info()->present, "a failed write still reports the legacy copy");
    CHECK(!c3_image_nvs(NULL, NULL), "a forgotten image knows its NVS");
    /* A torn or corrupt record reads as no record: back to the legacy view, never garbage. */
    c3_image_save(n, 3, d, 0x9000u, 0x6000u);
    mock_w25q[C3_IMAGE_REC_ADDR + 20] ^= 0x01;
    reboot();
    const blob_info_t *in = c3_image_info();
    CHECK(in->present && memcmp(in->sha256, d, 32) == 0 && in->version == 3, "corrupt record not ignored");
    CHECK(!c3_image_nvs(NULL, NULL), "a corrupt record gave an NVS location");
}

static void test_other_slots_pass_through(void) {
    mock_w25q_reset();
    reboot();
    CHECK(blob_report_info(BLOB_GW0) == blob_store_info(BLOB_GW0), "gw0 not from the blob store");
    CHECK(blob_report_state(BLOB_GW0) == blob_state(BLOB_GW0), "gw0 state differs");
}

int main(void) {
    test_layout();
    test_nothing_known();
    test_legacy_copy_is_reported();
    test_record_wins_and_survives_reboot();
    test_forget_and_corruption();
    test_other_slots_pass_through();
    if (fails) { printf("test_c3_image: %d FAILURE(S)\n", fails); return 1; }
    printf("test_c3_image: all passed\n");
    return 0;
}
