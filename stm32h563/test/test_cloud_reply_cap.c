/*
 * test_cloud_reply_cap.c: the captured reply of a cloud command (src/cloud_reply_cap.c, FW-10).
 *
 * A reply longer than the capture was cut and sent on as if whole: JSON with no end. It must
 * become a clear "too large" error instead; a reply that fits (also exactly) passes unchanged.
 */
#include "cloud_reply_cap.h"

#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); failures++; } \
} while (0)

static char out[CLOUD_REPLY_CAP_MAX];

static void put(const char *s) { cloud_reply_cap_append((const uint8_t *)s, strlen(s)); }

/* A {"status":"ok","data":"xxxx"} reply of exactly `len` bytes (without the newline). */
static void put_reply_of(size_t len) {
    static char r[4096];
    const char *head = "{\"status\":\"ok\",\"data\":\"";
    size_t h = strlen(head);
    memcpy(r, head, h);
    memset(r + h, 'x', len - h - 2);
    r[len - 2] = '"';
    r[len - 1] = '}';
    r[len] = '\0';
    /* In two writes, as a handler emitting header and body would. */
    cloud_reply_cap_append((const uint8_t *)r, len / 2);
    cloud_reply_cap_append((const uint8_t *)r + len / 2, len - len / 2);
    put("\n");
}

static void test_small_reply(void) {
    cloud_reply_cap_begin();
    put("{\"status\":\"ok\",\"data\":\"pong\"}\r\n");
    size_t n = cloud_reply_cap_end(out, sizeof(out));
    CHECK(n == strlen("{\"status\":\"ok\",\"data\":\"pong\"}"));
    CHECK(strcmp(out, "{\"status\":\"ok\",\"data\":\"pong\"}") == 0);
}

static void test_exact_fit(void) {
    cloud_reply_cap_begin();
    put_reply_of(CLOUD_REPLY_CAP_MAX - 1);   /* the newline is the byte that does not fit */
    size_t n = cloud_reply_cap_end(out, sizeof(out));
    CHECK(n == CLOUD_REPLY_CAP_MAX - 1);
    CHECK(out[n - 1] == '}');
}

static void test_too_large(void) {
    cloud_reply_cap_begin();
    put_reply_of(CLOUD_REPLY_CAP_MAX + 200);
    size_t n = cloud_reply_cap_end(out, sizeof(out));
    CHECK(n > 0);
    CHECK(strstr(out, "\"status\":\"error\"") != NULL);
    CHECK(strstr(out, "reply too large for the cloud channel (1736 bytes, limit 1535)") != NULL);
    CHECK(out[n - 1] == '}');
}

static void test_one_byte_over(void) {
    cloud_reply_cap_begin();
    put_reply_of(CLOUD_REPLY_CAP_MAX);
    cloud_reply_cap_end(out, sizeof(out));
    CHECK(strstr(out, "too large") != NULL);
}

static void test_no_reply_and_idle(void) {
    cloud_reply_cap_begin();
    CHECK(cloud_reply_cap_end(out, sizeof(out)) > 0);
    CHECK(strstr(out, "no reply") != NULL);
    /* Outside a capture the bytes go nowhere, and do not leak into the next one. */
    put("{\"stray\":1}\n");
    cloud_reply_cap_begin();
    put("{\"status\":\"ok\"}\n");
    cloud_reply_cap_end(out, sizeof(out));
    CHECK(strcmp(out, "{\"status\":\"ok\"}") == 0);
}

static void test_small_out_buffer(void) {
    char small[64];
    cloud_reply_cap_begin();
    put_reply_of(100);
    size_t n = cloud_reply_cap_end(small, sizeof(small));
    CHECK(n == 0 || strstr(small, "too large") != NULL);
}

int main(void) {
    test_small_reply();
    test_exact_fit();
    test_too_large();
    test_one_byte_over();
    test_no_reply_and_idle();
    test_small_out_buffer();
    if (failures) { printf("test_cloud_reply_cap: %d FAILED\n", failures); return 1; }
    printf("test_cloud_reply_cap: all passed\n");
    return 0;
}
