/*
 * test_fw_sign.c — host tests for the signed-manifest check (src/fw_sign.c).
 *
 * The cases come from tools/fwsign.py (vectors/fwsign_vectors.h, the same bytes as
 * vectors/fwsign_vectors.json that the server and CLI checks run), so the Python signer and
 * this C check have to agree byte for byte. fw_sign_keys_gen.h in this directory stands in for
 * the generated build header and holds the vectors' test key.
 */
#include "fw_sign.h"

#include <stdio.h>
#include <string.h>

#include "vectors/fwsign_vectors.h"

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

static void test_vectors(void) {
    for (size_t i = 0; i < sizeof(vec_cases) / sizeof(vec_cases[0]); i++) {
        const typeof(vec_cases[0]) *c = &vec_cases[i];
        const typeof(vec_images[0]) *img = &vec_images[c->image];
        fw_sig_manifest_t m;
        fw_sig_result_t r = fw_sign_check(c->sig, c->sig_len, c->target, (uint32_t)img->len,
                                          img->sha256, &m);
        CHECK(strcmp(fw_sign_result_name(r), c->want) == 0, "%s: got %s, want %s", c->name,
              fw_sign_result_name(r), c->want);
        /* the explicit-table entry point agrees */
        fw_sig_result_t r2 = fw_sign_check_keys(c->sig, c->sig_len, c->target, (uint32_t)img->len,
                                                img->sha256, &vec_pub, 1, NULL);
        CHECK(r2 == r, "%s: table check %s vs %s", c->name, fw_sign_result_name(r2),
              fw_sign_result_name(r));
    }
}

static void test_fields(void) {
    /* case 0 is the good firmware manifest: release 3.6.0, fw_info layout 2, 1024 KB */
    fw_sig_manifest_t m;
    memset(&m, 0, sizeof(m));
    fw_sig_result_t r = fw_sign_check(vec_cases[0].sig, 128, 0, (uint32_t)vec_images[0].len,
                                      vec_images[0].sha256, &m);
    CHECK(r == FW_SIG_OK, "good: %s", fw_sign_result_name(r));
    CHECK(m.target == 0 && m.size == vec_images[0].len, "target %u size %u", m.target, m.size);
    CHECK(m.release == 0x030600u && m.version == 0x030600u, "release %x version %x", m.release, m.version);
    CHECK(m.layout == 2 && m.min_flash_kb == 1024, "layout %u min %u", m.layout, m.min_flash_kb);
    /* case 1 is the gateware blob: version 45, no fw_info fields */
    r = fw_sign_check(vec_cases[1].sig, 128, 2, (uint32_t)vec_images[1].len, vec_images[1].sha256, &m);
    CHECK(r == FW_SIG_OK && m.version == 45 && m.layout == 0, "blob: %s v%u", fw_sign_result_name(r), m.version);
}

static void test_none_and_unknown_keys(void) {
    CHECK(fw_sign_check(NULL, 0, 0, 1, vec_images[0].sha256, NULL) == FW_SIG_NONE, "NULL = none");
    CHECK(fw_sign_check(vec_cases[0].sig, 0, 0, 1, vec_images[0].sha256, NULL) == FW_SIG_NONE,
          "len 0 = none");
    /* an empty key table knows no key */
    CHECK(fw_sign_check_keys(vec_cases[0].sig, 128, 0, (uint32_t)vec_images[0].len,
                             vec_images[0].sha256, NULL, 0, NULL) == FW_SIG_UNKNOWN_KEY,
          "no keys");
    /* a target number past esp is a format error, not a crash */
    uint8_t bad[128];
    memcpy(bad, vec_cases[0].sig, 128);
    bad[6] = 9;
    CHECK(fw_sign_check(bad, 128, 0, 1, vec_images[0].sha256, NULL) == FW_SIG_FORMAT, "target 9");
}

static void test_policy(void) {
    static const fw_sig_result_t all[] = { FW_SIG_NONE, FW_SIG_OK, FW_SIG_FORMAT,
                                           FW_SIG_UNKNOWN_KEY, FW_SIG_SIGNATURE, FW_SIG_TARGET,
                                           FW_SIG_IMAGE };
    for (size_t i = 0; i < sizeof(all) / sizeof(all[0]); i++) {
        fw_sig_result_t r = all[i];
        CHECK(fw_sign_accept(r, FW_SIG_POLICY_AUDIT), "audit refuses %s", fw_sign_result_name(r));
        CHECK(fw_sign_accept(r, FW_SIG_POLICY_PERMISSIVE) == (r == FW_SIG_OK || r == FW_SIG_NONE),
              "permissive on %s", fw_sign_result_name(r));
        CHECK(fw_sign_accept(r, FW_SIG_POLICY_REQUIRED) == (r == FW_SIG_OK),
              "required on %s", fw_sign_result_name(r));
    }
    CHECK(fw_sign_policy() == FW_SIG_POLICY_AUDIT, "this release reports only");
    CHECK(strcmp(fw_sign_policy_name(fw_sign_policy()), "audit") == 0, "policy name");
}

int main(void) {
    test_vectors();
    test_fields();
    test_none_and_unknown_keys();
    test_policy();
    if (failures) { printf("test_fw_sign: %d FAILED\n", failures); return 1; }
    printf("test_fw_sign: all passed\n");
    return 0;
}
