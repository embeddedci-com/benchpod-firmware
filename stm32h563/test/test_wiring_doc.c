/*
 * test_wiring_doc.c: host unit tests for wiring_doc (the wiring profiles document in the W25Q
 * slot): names, the JSON object check, put/delete/default, revisions, the default profile, limits,
 * and that a corrupt document reads as empty.
 */
#include "wiring_doc.h"

#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); failures++; } \
} while (0)

static uint8_t g_doc[WIRING_DOC_MAX];
static size_t  g_len;
static wiring_index_t g_idx;

static wiring_result_t apply(wiring_op_kind_t kind, const char *name, const char *json) {
    wiring_op_t op = { kind, name, (const uint8_t *)json, (uint16_t)(json ? strlen(json) : 0) };
    wiring_result_t r = wiring_doc_check(&g_idx, &op);
    if (r != WIRING_OK) return r;
    static uint8_t out[WIRING_DOC_MAX];
    size_t n = wiring_doc_build(g_doc, &g_idx, &op, out, sizeof(out));
    CHECK(n == wiring_doc_size_after(&g_idx, &op));
    memcpy(g_doc, out, n);
    g_len = n;
    CHECK(wiring_doc_parse(g_doc, g_len, &g_idx));
    return r;
}

static void reset(void) { g_len = 0; CHECK(wiring_doc_parse(g_doc, 0, &g_idx)); }

static const char *json_of(const char *name) {
    static char buf[WIRING_PROFILE_MAX + 1];
    int i = wiring_index_find(&g_idx, name);
    if (i < 0) return NULL;
    memcpy(buf, g_doc + g_idx.e[i].off, g_idx.e[i].len);
    buf[g_idx.e[i].len] = 0;
    return buf;
}

static void test_names(void) {
    CHECK(wiring_name_ok("default"));
    CHECK(wiring_name_ok("nrf52840-dk"));
    CHECK(wiring_name_ok("k66"));
    CHECK(!wiring_name_ok(""));
    CHECK(!wiring_name_ok("-x"));
    CHECK(!wiring_name_ok("K66"));
    CHECK(!wiring_name_ok("a b"));
    CHECK(!wiring_name_ok("a_b"));
    CHECK(wiring_name_ok("0123456789012345678901234567890123456789"));   /* 40 */
    CHECK(!wiring_name_ok("01234567890123456789012345678901234567890")); /* 41 */
}

static void test_json_object(void) {
    const char *ok[] = { "{}", " {\"a\":1}\n", "{\"a\":[1,{\"b\":\"}\"}],\"c\":\"\\\"\"}" };
    const char *bad[] = { "", "[]", "{", "{\"a\":\"x}", "{}{}", "{]", "{\"a\":\"\\q\"}", "x{}" };
    for (size_t i = 0; i < sizeof(ok) / sizeof(*ok); i++)
        CHECK(wiring_doc_json_object_ok((const uint8_t *)ok[i], strlen(ok[i])));
    for (size_t i = 0; i < sizeof(bad) / sizeof(*bad); i++)
        CHECK(!wiring_doc_json_object_ok((const uint8_t *)bad[i], strlen(bad[i])));
}

static void test_put_default_revisions(void) {
    reset();
    CHECK(g_idx.count == 0 && g_idx.def == WIRING_NO_DEFAULT);
    CHECK(apply(WIRING_OP_PUT, "default", "{\"la_mv\":3300}") == WIRING_OK);
    CHECK(g_idx.count == 1 && g_idx.def == 0 && g_idx.e[0].rev == 1);   /* first = default */
    CHECK(apply(WIRING_OP_PUT, "k66", "{\"la_mv\":1800}") == WIRING_OK);
    CHECK(g_idx.count == 2 && g_idx.def == 0 && g_idx.e[1].rev == 1);
    CHECK(apply(WIRING_OP_PUT, "default", "{\"la_mv\":3300,\"uart_baud\":9600}") == WIRING_OK);
    CHECK(g_idx.e[wiring_index_find(&g_idx, "default")].rev == 2);
    CHECK(!strcmp(json_of("default"), "{\"la_mv\":3300,\"uart_baud\":9600}"));
    CHECK(!strcmp(json_of("k66"), "{\"la_mv\":1800}"));
    CHECK(apply(WIRING_OP_DEFAULT, "k66", NULL) == WIRING_OK);
    CHECK(!strcmp(g_idx.e[g_idx.def].name, "k66"));
    CHECK(g_idx.e[wiring_index_find(&g_idx, "k66")].rev == 1);   /* default does not bump */
}

static void test_delete(void) {
    reset();
    apply(WIRING_OP_PUT, "default", "{}");
    apply(WIRING_OP_PUT, "a", "{\"x\":1}");
    apply(WIRING_OP_PUT, "b", "{\"x\":2}");
    CHECK(apply(WIRING_OP_DELETE, "default", NULL) == WIRING_E_DEFAULT);
    CHECK(apply(WIRING_OP_DELETE, "nope", NULL) == WIRING_E_NOT_FOUND);
    CHECK(apply(WIRING_OP_DELETE, "a", NULL) == WIRING_OK);
    CHECK(g_idx.count == 2 && wiring_index_find(&g_idx, "a") < 0);
    CHECK(!strcmp(json_of("b"), "{\"x\":2}"));
    CHECK(!strcmp(g_idx.e[g_idx.def].name, "default"));
    /* the default moves with its profile when an earlier one is deleted */
    apply(WIRING_OP_DEFAULT, "b", NULL);
    apply(WIRING_OP_PUT, "c", "{}");
    CHECK(apply(WIRING_OP_DELETE, "default", NULL) == WIRING_OK);
    CHECK(!strcmp(g_idx.e[g_idx.def].name, "b"));
}

static void test_limits(void) {
    reset();
    char name[8];
    for (unsigned i = 0; i < WIRING_MAX_PROFILES; i++) {
        snprintf(name, sizeof(name), "p%u", i);
        CHECK(apply(WIRING_OP_PUT, name, "{}") == WIRING_OK);
    }
    CHECK(apply(WIRING_OP_PUT, "one-more", "{}") == WIRING_E_FULL);
    CHECK(apply(WIRING_OP_PUT, "p3", "{\"replaced\":true}") == WIRING_OK);   /* replacing is fine */
    CHECK(apply(WIRING_OP_PUT, "Bad", "{}") == WIRING_E_NAME);
    CHECK(apply(WIRING_OP_PUT, "p4", "[1]") == WIRING_E_NOT_OBJECT);
    static char big[WIRING_PROFILE_MAX + 2];
    memset(big, ' ', sizeof(big) - 1);
    big[0] = '{'; big[sizeof(big) - 2] = '}'; big[sizeof(big) - 1] = 0;
    CHECK(apply(WIRING_OP_PUT, "p5", big) == WIRING_E_TOO_LARGE);
}

static void test_corrupt_reads_empty(void) {
    reset();
    apply(WIRING_OP_PUT, "default", "{\"a\":1}");
    uint8_t copy[64];
    memcpy(copy, g_doc, g_len);
    wiring_index_t idx;
    CHECK(wiring_doc_parse(copy, g_len, &idx) && idx.count == 1);
    copy[0] = 'X';
    CHECK(!wiring_doc_parse(copy, g_len, &idx) && idx.count == 0 && idx.def == WIRING_NO_DEFAULT);
    memcpy(copy, g_doc, g_len);
    CHECK(!wiring_doc_parse(copy, g_len - 1, &idx));   /* truncated */
    CHECK(!wiring_doc_parse(copy, g_len + 1, &idx));   /* trailing byte */
}

static uint16_t la_mv_of(const char *s) { return wiring_doc_json_la_mv((const uint8_t *)s, strlen(s)); }

static void test_la_mv(void) {
    CHECK(la_mv_of("{\"version\":2,\"la_mv\":3300}") == 3300);
    CHECK(la_mv_of("{ \"la_mv\" : 1800 , \"efuse\": 1}") == 1800);
    CHECK(la_mv_of("{\"la_mv\":null}") == 0);
    CHECK(la_mv_of("{\"la_mv\":5000}") == 0);
    CHECK(la_mv_of("{\"la_mv\":3300.5}") == 0);
    CHECK(la_mv_of("{\"la_mv\":33000}") == 0);
    CHECK(la_mv_of("{\"efuse\":1}") == 0);
    CHECK(la_mv_of("") == 0);
    /* Only the top level counts: a signal or a dut_dirs key named la_mv is not the voltage. */
    CHECK(la_mv_of("{\"dut_dirs\":{\"la_mv\":3300},\"signals\":[{\"name\":\"la_mv\",\"la_mv\":1800}]}") == 0);
    CHECK(la_mv_of("{\"signals\":[{\"la\":9}],\"la_mv\":1800}") == 1800);
    /* A string value that reads like the key, and escapes inside strings. */
    CHECK(la_mv_of("{\"swd_target\":\"la_mv\",\"i2c_addr\":\"a\\\"b\",\"la_mv\":3300}") == 3300);
    /* Not NUL-terminated: the length bounds it. */
    const char *cut = "{\"la_mv\":3300}";
    CHECK(wiring_doc_json_la_mv((const uint8_t *)cut, 10) == 0);
}

int main(void) {
    test_la_mv();
    test_names();
    test_json_object();
    test_put_default_revisions();
    test_delete();
    test_limits();
    test_corrupt_reads_empty();
    if (failures) { printf("test_wiring_doc: %d failure(s)\n", failures); return 1; }
    printf("test_wiring_doc: all passed\n");
    return 0;
}
