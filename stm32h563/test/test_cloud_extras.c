/*
 * test_cloud_extras.c: the company CA as cloud_extras.c loads it from its W25Q slot (FW-1).
 *
 * A damaged slot (hash mismatch) or a CA that does not parse must not take the cloud link down:
 * it is dropped with a reason and the link trusts the built-in roots. Out of memory while
 * building the TLS trust list is reported as such and never turns into a roots-only list.
 *
 * cloud_extras.c is compiled into this file (net_server.h is stubbed out) on the real blob store
 * over the W25Q model, the real mbedTLS X.509 parser and a malloc-backed FreeRTOS heap shim.
 */
#define NET_SERVER_H   /* keep lwIP out: cloud_extras.c only needs net_cloud_reload */
static int reloads;
static void net_cloud_reload(void) { reloads++; }

#include "../src/cloud_extras.c"

#include "mock_w25q.h"

#include <stdio.h>
#include <string.h>

int shim_rtos_fail_after = -1;

/* blob_store's release-blob comparison; no release blobs here. */
const blob_manifest_t *blob_manifest(blob_id_t id) { (void)id; return NULL; }

/* The CA upload check reads the staged image; unused here. */
int ota_read_staged(uint32_t off, uint8_t *buf, uint32_t n) { (void)off; (void)buf; (void)n; return -1; }

static int failures;
#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); failures++; } \
} while (0)

/* A self-signed P-256 test CA (openssl req -x509, 100 years). */
static const char test_ca[] =
    "-----BEGIN CERTIFICATE-----\n"
    "MIIBujCCAWGgAwIBAgIUSZoHdGmO6nXXvcBy7LvJRYe28BowCgYIKoZIzj0EAwIw\n"
    "MjEhMB8GA1UEAwwYQmVuY2hQb2QgVGVzdCBDb21wYW55IENBMQ0wCwYDVQQKDARU\n"
    "ZXN0MCAXDTI2MTAwNzE1NTc1MFoYDzIxMjYwOTEzMTU1NzUwWjAyMSEwHwYDVQQD\n"
    "DBhCZW5jaFBvZCBUZXN0IENvbXBhbnkgQ0ExDTALBgNVBAoMBFRlc3QwWTATBgcq\n"
    "hkjOPQIBBggqhkjOPQMBBwNCAATB/4P5DXr20LCKFLt5MtA6yxkXQLmJtn+MpPEf\n"
    "ndzw/2k9MLLy9zAkisc5t5zJhedMJmh/azhIS6Wa5NBJQbV/o1MwUTAdBgNVHQ4E\n"
    "FgQUItJ50gmchF3nobt15GtssFIqQ/swHwYDVR0jBBgwFoAUItJ50gmchF3nobt1\n"
    "5GtssFIqQ/swDwYDVR0TAQH/BAUwAwEB/zAKBggqhkjOPQQDAgNHADBEAiAugJkJ\n"
    "NLaJwI/d4PZGsMFKal4cj3dU+jr5xif3IG9U5wIgYU6Nbe28He94ucGyQtIRVyrL\n"
    "SD3tgOnj3zz/OTgIkg8=\n"
    "-----END CERTIFICATE-----\n";

static int mem_src(void *ctx, uint32_t off, uint8_t *buf, uint32_t n) {
    memcpy(buf, (const uint8_t *)ctx + off, n);
    return 0;
}

/* Install `data` into the CA slot as the OTA path would (header hash of the data), then boot. */
static void install_and_load(const void *data, size_t len) {
    mock_w25q_reset();
    CHECK(blob_store_init() >= 0);
    uint8_t sha[32];
    mbedtls_sha256(data, len, sha, 0);
    CHECK(w25q_open() == 0);
    CHECK(blob_store_write(BLOB_CA, (uint32_t)len, 0, sha, mem_src, (void *)data) == 0);
    w25q_close();
    cloud_extras_load();
}

static void test_valid_ca_is_used(void) {
    install_and_load(test_ca, strlen(test_ca));
    CHECK(cloud_extras_ca_error() == NULL);
    char *pem = NULL;
    size_t len = 0;
    CHECK(cloud_extras_ca_pem(&pem, &len) == 0);
    CHECK(pem != NULL);
    /* The built-in roots, a newline, the company CA, the NUL. */
    CHECK(len == (cloud_ca_pem_len - 1) + 1 + strlen(test_ca) + 1);
    CHECK(pem && strstr(pem, "MIIBujCCAWGgAwIBAgIUSZoHdGmO6nXX") != NULL);
    CHECK(pem && memcmp(pem, cloud_ca_pem, cloud_ca_pem_len - 1) == 0);
    cloud_extras_free(pem);
    char certs[640];
    CHECK(cloud_extras_ca_describe(certs, sizeof(certs)) == 1);
    CHECK(strstr(certs, "BenchPod Test Company CA") != NULL);
    /* TLS refused it anyway (out of memory there): it still parses, so it is kept. */
    CHECK(!cloud_extras_ca_tls_refused());
    CHECK(cloud_extras_ca_error() == NULL);
}

static void test_corrupt_slot_falls_back(void) {
    install_and_load(test_ca, strlen(test_ca));
    /* Flip one data byte behind the committed header: the slot hash no longer matches. */
    uint32_t data = blob_slot_base(BLOB_CA) + BLOB_HDR_SIZE + 100u;
    mock_w25q[data] &= 0x0F;
    cloud_extras_load();
    CHECK(cloud_extras_ca_error() != NULL);
    CHECK(cloud_extras_ca_error() && strcmp(cloud_extras_ca_error(), CLOUD_CA_ERR_HASH) == 0);
    char *pem = (char *)1;
    size_t len = 1;
    CHECK(cloud_extras_ca_pem(&pem, &len) == 0);
    CHECK(pem == NULL && len == 0);   /* built-in roots, as they are */
    char certs[64];
    CHECK(cloud_extras_ca_describe(certs, sizeof(certs)) == 0);
}

static void test_unparsable_ca_falls_back(void) {
    /* Intact slot, but not a certificate: garbage between PEM markers. */
    static const char junk[] =
        "-----BEGIN CERTIFICATE-----\n"
        "bm90IGEgY2VydGlmaWNhdGUsIGp1c3QgYmFzZTY0IHRleHQgdGhhdCBkZWNvZGVz\n"
        "-----END CERTIFICATE-----\n";
    install_and_load(junk, strlen(junk));
    CHECK(cloud_extras_ca_error() && strcmp(cloud_extras_ca_error(), CLOUD_CA_ERR_PARSE) == 0);
    char *pem = (char *)1;
    size_t len = 1;
    CHECK(cloud_extras_ca_pem(&pem, &len) == 0 && pem == NULL);
}

static void test_out_of_memory_is_not_roots_only(void) {
    install_and_load(test_ca, strlen(test_ca));
    CHECK(cloud_extras_ca_error() == NULL);
    shim_rtos_fail_after = 0;   /* the combined PEM allocation fails */
    char *pem = (char *)1;
    size_t len = 1;
    CHECK(cloud_extras_ca_pem(&pem, &len) == -1);
    CHECK(pem == NULL);
    shim_rtos_fail_after = -1;
    /* Nothing was dropped: the next attempt still carries the company CA. */
    CHECK(cloud_extras_ca_error() == NULL);
    CHECK(cloud_extras_ca_pem(&pem, &len) == 0 && pem != NULL);
    cloud_extras_free(pem);
    /* Out of memory while re-checking after a TLS refusal: kept, not dropped. */
    shim_rtos_fail_after = 0;
    CHECK(!cloud_extras_ca_tls_refused());
    shim_rtos_fail_after = -1;
    CHECK(cloud_extras_ca_error() == NULL);
}

static void test_cleared_marker_is_none(void) {
    install_and_load(test_ca, strlen(test_ca));
    CHECK(cloud_extras_ca_clear() == NULL);
    CHECK(cloud_extras_ca_error() == NULL);
    char *pem = (char *)1;
    size_t len = 1;
    CHECK(cloud_extras_ca_pem(&pem, &len) == 0 && pem == NULL);
}

static void test_no_w25q_no_error(void) {
    mock_w25q_reset();
    CHECK(blob_store_init() >= 0);
    cloud_extras_load();
    CHECK(cloud_extras_ready());
    CHECK(cloud_extras_ca_error() == NULL);
}

int main(void) {
    test_valid_ca_is_used();
    test_corrupt_slot_falls_back();
    test_unparsable_ca_falls_back();
    test_out_of_memory_is_not_roots_only();
    test_cleared_marker_is_none();
    test_no_w25q_no_error();
    CHECK(mock_w25q_open_depth == 0);
    if (failures) { printf("test_cloud_extras: %d FAILED\n", failures); return 1; }
    printf("test_cloud_extras: all passed\n");
    return 0;
}
