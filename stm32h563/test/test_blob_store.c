/* Host tests for blob_store: slots in the W25Q, written so that a power cut anywhere leaves a
   slot either empty or complete, never present with bad data. */
#include "blob_store.h"
#include "w25q.h"
#include "mocks/mock_w25q.h"
#include "mbedtls/sha256.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int fails;
#define CHECK(cond, ...) do { if (!(cond)) { \
        printf("FAIL %s:%d: ", __func__, __LINE__); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

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

static uint8_t img[1200000];   /* bigger than a gateware slot, smaller than the ESP slot */

static void test_empty_flash(void) {
    mock_w25q_reset();
    CHECK(blob_store_load() == 0, "an erased flash has slots");
    for (int i = 0; i < BLOB_COUNT; i++)
        CHECK(!blob_store_present((blob_id_t)i), "%s present on an erased flash", blob_name((blob_id_t)i));
}

static void test_write_read_verify(void) {
    const uint32_t n = 104090;   /* an iCE40UP5K bitstream */
    uint8_t d[32], back[300];
    mock_w25q_reset();
    blob_store_load();
    fill(img, n, 7); sha(img, n, d);
    CHECK(blob_store_write(BLOB_GW0, n, 46, d, mem_src, img) == 0, "write failed");
    CHECK(mock_w25q_bad_programs == 0, "programmed over unerased bits (%d)", mock_w25q_bad_programs);
    CHECK(blob_store_load() == 1, "reload sees %d slots", blob_store_load());
    const blob_info_t *in = blob_store_info(BLOB_GW0);
    CHECK(in->present && in->len == n && in->version == 46 && memcmp(in->sha256, d, 32) == 0,
          "header does not match what was written");
    CHECK(blob_store_verify(BLOB_GW0) == 0, "verify failed on a good slot");
    CHECK(blob_store_read((void *)(uintptr_t)BLOB_GW0, n - 300, back, 300) == 0 &&
          memcmp(back, img + n - 300, 300) == 0, "read-back differs at the tail");
    CHECK(blob_store_read((void *)(uintptr_t)BLOB_GW0, n - 10, back, 11) != 0, "read past the end allowed");
    CHECK(blob_store_read((void *)(uintptr_t)BLOB_GW1, 0, back, 1) != 0, "read of an empty slot allowed");
    /* The gateware the iCE40 boots from (offset 0) is never touched. */
    for (uint32_t a = 0; a < 0x100000u; a++)
        if (mock_w25q[a] != 0xFF) { CHECK(0, "offset 0x%06x below the slots was written", a); break; }
}

static void test_bad_hash_leaves_slot_empty(void) {
    const uint32_t n = 5000;
    uint8_t d[32];
    mock_w25q_reset();
    fill(img, n, 3); sha(img, n, d);
    d[0] ^= 1;
    CHECK(blob_store_write(BLOB_GW1, n, 1, d, mem_src, img) != 0, "a wrong digest was committed");
    CHECK(!blob_store_present(BLOB_GW1), "slot present after a failed write");
    CHECK(blob_store_load() == 0, "reload finds a slot after a failed write");
}

static void test_limits(void) {
    uint8_t d[32] = {0};
    mock_w25q_reset();
    CHECK(blob_store_write(BLOB_GW0, blob_slot_capacity(BLOB_GW0) + 1, 0, d, mem_src, img) != 0,
          "an oversized gateware was accepted");
    CHECK(blob_store_write(BLOB_ESP, 0, 0, d, mem_src, img) != 0, "an empty blob was accepted");
    CHECK(blob_from_name("esp") == BLOB_ESP && blob_from_name("gw1") == BLOB_GW1 &&
          blob_from_name("nope") == -1, "name lookup");
    /* Slots do not overlap and the ESP slot holds a whole 4 MB C3 flash. */
    CHECK(blob_slot_base(BLOB_GW0) + BLOB_HDR_SIZE + blob_slot_capacity(BLOB_GW0) <= blob_slot_base(BLOB_GW1), "gw0/gw1 overlap");
    CHECK(blob_slot_base(BLOB_GW1) + BLOB_HDR_SIZE + blob_slot_capacity(BLOB_GW1) <= blob_slot_base(BLOB_ESP), "gw1/esp overlap");
    CHECK(blob_slot_capacity(BLOB_ESP) >= 0x400000u - BLOB_HDR_SIZE, "esp slot too small");
    CHECK(blob_slot_base(BLOB_ESP) + BLOB_HDR_SIZE + blob_slot_capacity(BLOB_ESP) <= MOCK_W25Q_BYTES, "esp slot past 8 MB");
}

static void test_other_slots_untouched(void) {
    const uint32_t n1 = 104090, n2 = 1097568;
    static uint8_t gw[104090];
    uint8_t d1[32], d2[32];
    mock_w25q_reset();
    fill(gw, n1, 11); sha(gw, n1, d1);
    fill(img, n2, 12); sha(img, n2, d2);
    CHECK(blob_store_write(BLOB_GW0, n1, 46, d1, mem_src, gw) == 0, "gw0 write");
    CHECK(blob_store_write(BLOB_ESP, n2, 0, d2, mem_src, img) == 0, "esp write");
    CHECK(blob_store_load() == 2, "both slots present");
    CHECK(blob_store_verify(BLOB_GW0) == 0 && blob_store_verify(BLOB_ESP) == 0, "verify after both writes");
    /* Rewriting a slot with new content replaces it. */
    fill(gw, n1, 13); sha(gw, n1, d1);
    CHECK(blob_store_write(BLOB_GW0, n1, 47, d1, mem_src, gw) == 0, "gw0 rewrite");
    CHECK(blob_store_info(BLOB_GW0)->version == 47 && blob_store_verify(BLOB_GW0) == 0, "rewrite took");
    CHECK(blob_store_verify(BLOB_ESP) == 0, "esp damaged by a gw0 rewrite");
    CHECK(mock_w25q_bad_programs == 0, "programmed over unerased bits");
}

static void test_corruption_detected(void) {
    const uint32_t n = 20000;
    uint8_t d[32];
    mock_w25q_reset();
    fill(img, n, 21); sha(img, n, d);
    CHECK(blob_store_write(BLOB_GW1, n, 2, d, mem_src, img) == 0, "write");
    mock_w25q[blob_slot_base(BLOB_GW1) + BLOB_HDR_SIZE + 1234] ^= 0x10;
    CHECK(blob_store_verify(BLOB_GW1) != 0, "a flipped data bit passed verify");
    mock_w25q[blob_slot_base(BLOB_GW1) + BLOB_COMMIT_OFF] = 0x00;   /* torn commit word */
    blob_store_load();
    CHECK(!blob_store_present(BLOB_GW1), "a slot without its commit word reads as present");
}

/* Cut the power after every possible operation of a write: the slot must come back either empty
   (cut before the commit word) or complete and intact, and the other slot must be unharmed. */
static void test_power_cut_anywhere(void) {
    const uint32_t n = 9000;
    static uint8_t a[9000], b[9000];
    uint8_t da[32], db[32];
    fill(a, n, 31); sha(a, n, da);
    fill(b, n, 32); sha(b, n, db);

    mock_w25q_reset();
    CHECK(blob_store_write(BLOB_GW0, n, 1, da, mem_src, a) == 0, "baseline write");
    int total = mock_w25q_ops;
    mock_w25q_reset();
    CHECK(blob_store_write(BLOB_GW1, n, 9, db, mem_src, b) == 0, "other slot");
    int base_ops = mock_w25q_ops;
    (void)base_ops;

    int complete = 0, empty = 0;
    for (int cut = 1; cut <= total; cut++) {
        mock_w25q_reset();
        blob_store_write(BLOB_GW1, n, 9, db, mem_src, b);
        mock_w25q_ops = 0;
        mock_w25q_cut_after = cut;
        if (setjmp(mock_w25q_power_jmp) == 0) {
            blob_store_write(BLOB_GW0, n, 1, da, mem_src, a);
            mock_w25q_cut_after = 0;
        }
        mock_w25q_cut_after = 0;
        blob_store_load();
        if (blob_store_present(BLOB_GW0)) {
            complete++;
            CHECK(blob_store_verify(BLOB_GW0) == 0, "cut %d: slot present but its data is bad", cut);
        } else {
            empty++;
        }
        CHECK(blob_store_present(BLOB_GW1) && blob_store_verify(BLOB_GW1) == 0,
              "cut %d: the other slot was damaged", cut);
    }
    /* Only the very last op (the commit word) makes the slot valid. */
    CHECK(complete == 0 || complete == 1, "slot valid after %d different cut points", complete);
    CHECK(empty >= total - 1, "only %d of %d cuts left the slot empty", empty, total);
}

static void test_no_flash(void) {
    mock_w25q_reset();
    mock_w25q_id[0] = 0xFF; mock_w25q_id[1] = 0xFF; mock_w25q_id[2] = 0xFF;
    CHECK(blob_store_load() == -1, "a missing flash was accepted");
    mock_w25q_reset();
    mock_w25q_id[2] = 0x16;   /* 4 MB: too small for the ESP slot */
    CHECK(blob_store_load() == -1, "a 4 MB flash was accepted");
    mock_w25q_reset();
    mock_w25q_id[2] = 0x18;   /* W25Q128 */
    CHECK(blob_store_load() == 0, "a W25Q128 was refused");
}

/* blob_state_of: what an installer is told about each slot. */
static void test_state(void) {
    blob_info_t have = {0};
    blob_manifest_t want = { .len = 100, .version = 46 };
    memset(want.sha256, 0xAB, 32);
    CHECK(blob_state_of(&have, &want) == BLOB_STATE_MISSING, "empty slot not missing");
    have.present = true; have.len = 100; memset(have.sha256, 0xAB, 32);
    CHECK(blob_state_of(&have, &want) == BLOB_STATE_OK, "matching slot not ok");
    have.sha256[31] ^= 1;
    CHECK(blob_state_of(&have, &want) == BLOB_STATE_OUTDATED, "different blob not outdated");
    have.sha256[31] ^= 1; have.len = 99;
    CHECK(blob_state_of(&have, &want) == BLOB_STATE_OUTDATED, "different length not outdated");
    want.len = 0;
    CHECK(blob_state_of(&have, &want) == BLOB_STATE_UNKNOWN, "no expectation not unknown");
    have.present = false;
    CHECK(blob_state_of(&have, &want) == BLOB_STATE_UNKNOWN, "empty + no expectation not unknown");
    CHECK(strcmp(blob_state_str(BLOB_STATE_OUTDATED), "outdated") == 0, "state names");
}

/* The firmware's generated manifest is not linked into the host test. */
const blob_manifest_t *blob_manifest(blob_id_t id) { (void)id; return NULL; }

int main(void) {
    test_state();
    test_empty_flash();
    test_write_read_verify();
    test_bad_hash_leaves_slot_empty();
    test_limits();
    test_other_slots_untouched();
    test_corruption_detected();
    test_power_cut_anywhere();
    test_no_flash();
    if (fails) { printf("test_blob_store: %d FAILED\n", fails); return 1; }
    printf("test_blob_store: all passed\n");
    return 0;
}
