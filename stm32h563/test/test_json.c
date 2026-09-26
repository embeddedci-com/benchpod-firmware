/*
 * test_json.c — host unit tests for bp_json (the shared flat-JSON parser and the
 * bounds-tracked emitter).  Covers the behaviours the old per-file parsers had
 * to get right: key-position matching (no value false hits), escaped quotes in
 * string values, brace-matched objects, byte arrays, and emitter truncation +
 * string escaping.
 */
#include "bp_json.h"

#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); failures++; } \
} while (0)

static void test_get_basic(void) {
    char v[32];
    CHECK(bp_json_get("{\"cmd\":\"status\"}", "cmd", v, sizeof(v)) && !strcmp(v, "status"));
    CHECK(bp_json_get("{\"samples\":256}", "samples", v, sizeof(v)) && !strcmp(v, "256"));
    CHECK(!bp_json_get("{\"cmd\":\"status\"}", "missing", v, sizeof(v)));
    /* scalar with trailing space / brace terminators */
    CHECK(bp_json_get("{\"a\": 12 , \"b\":3}", "a", v, sizeof(v)) && !strcmp(v, "12"));
    CHECK(bp_json_get("{\"a\":12,\"b\":3}", "b", v, sizeof(v)) && !strcmp(v, "3"));
}

/* The key must be matched as a key, not where the same text appears as a value. */
static void test_key_vs_value(void) {
    char v[32];
    /* looking up "la" must not stop on the "la" that is cmd's value */
    CHECK(bp_json_get("{\"cmd\":\"la\",\"la\":3}", "la", v, sizeof(v)) && !strcmp(v, "3"));
    /* key present only as a value -> not found */
    CHECK(!bp_json_get("{\"cmd\":\"la\"}", "la", v, sizeof(v)));
}

static void test_escapes(void) {
    char v[64];
    /* an escaped quote inside the value must not terminate the string early */
    CHECK(bp_json_get("{\"msg\":\"a\\\"b\"}", "msg", v, sizeof(v)) && !strcmp(v, "a\"b"));
    CHECK(bp_json_get("{\"p\":\"x\\\\y\"}", "p", v, sizeof(v)) && !strcmp(v, "x\\y"));
    CHECK(bp_json_get("{\"p\":\"a\\nb\"}", "p", v, sizeof(v)) && !strcmp(v, "a\nb"));
}

static void test_flag(void) {
    CHECK(bp_json_flag("{\"term\":true}", "term"));
    CHECK(bp_json_flag("{\"term\":1}", "term"));
    CHECK(!bp_json_flag("{\"term\":false}", "term"));
    CHECK(!bp_json_flag("{\"x\":1}", "term"));
}

static void test_object(void) {
    char o[64];
    CHECK(bp_json_object("{\"command\":{\"cmd\":\"ping\"},\"id\":1}", "command", o, sizeof(o))
          && !strcmp(o, "{\"cmd\":\"ping\"}"));
    /* a '}' inside a string value must not end the object early */
    CHECK(bp_json_object("{\"c\":{\"s\":\"a}b\"}}", "c", o, sizeof(o))
          && !strcmp(o, "{\"s\":\"a}b\"}"));
    /* too small for cap -> false */
    char small[4];
    CHECK(!bp_json_object("{\"c\":{\"s\":\"abcdef\"}}", "c", small, sizeof(small)));
}

static void test_byte_array(void) {
    uint8_t b[8]; int len = -1;
    CHECK(bp_json_byte_array("{\"data\":[1,2,3]}", "data", b, 8, &len) == 1);
    CHECK(len == 3 && b[0] == 1 && b[1] == 2 && b[2] == 3);
    CHECK(bp_json_byte_array("{\"data\":[0x50,0x03]}", "data", b, 8, &len) == 1);
    CHECK(len == 2 && b[0] == 0x50 && b[1] == 0x03);
    /* overflow the cap: count keeps going but writes clamp */
    CHECK(bp_json_byte_array("{\"data\":[1,2,3,4,5]}", "data", b, 2, &len) == 1 && len == 2);
    /* absent */
    CHECK(bp_json_byte_array("{\"x\":1}", "data", b, 8, &len) == 0 && len == 0);
}

static void test_emit(void) {
    char buf[64];
    bp_emit_t e;
    bp_emit_init(&e, buf, sizeof(buf));
    bp_emit(&e, "{\"a\":%d,", 5);
    bp_emit_raw(&e, "\"b\":");
    bp_emit_jstr(&e, "he\"llo");
    bp_emit_raw(&e, "}");
    CHECK(bp_emit_ok(&e));
    CHECK(!strcmp(buf, "{\"a\":5,\"b\":\"he\\\"llo\"}"));

    /* truncation is reported, not silently clipped */
    char tiny[8];
    bp_emit_t t;
    bp_emit_init(&t, tiny, sizeof(tiny));
    bp_emit(&t, "%s", "0123456789");
    CHECK(!bp_emit_ok(&t));
    CHECK(strlen(tiny) < sizeof(tiny));   /* NUL-terminated, no overrun */
}

/* Regression: envelope fields must not be shadowable by a CARRIED object's keys.
   The exact frame the server sends — Go marshals map[string]any with keys SORTED,
   so "command" precedes "type" on the wire and the command's own "type" comes
   FIRST in the byte stream. The flat getter returns that inner value; the _top
   getter must return the envelope's. Reading it flat made the device route the
   frame as type "bmp280", match no branch, and drop it without a reply — every
   sensor_start over the cloud hung until the 30 s timeout and returned 504. */
static void test_top_level_not_shadowed_by_payload(void) {
    const char *frame =
        "{\"command\":{\"cmd\":\"sensor_start\",\"type\":\"bmp280\",\"sda\":1,\"scl\":2},"
        "\"device_id\":\"dev-1\",\"request_id\":\"req-9\",\"timeout_ms\":30000,"
        "\"type\":\"command.request\"}";
    char v[32];

    /* the flat getter is shadowed — this is the behaviour that caused the outage */
    CHECK(bp_json_get(frame, "type", v, sizeof(v)));
    CHECK(!strcmp(v, "bmp280"));

    /* the top-level getter reads the envelope, whatever the payload carries */
    CHECK(bp_json_get_top(frame, "type", v, sizeof(v)));
    CHECK(!strcmp(v, "command.request"));

    /* other envelope fields still resolve, and past a nested object */
    CHECK(bp_json_get_top(frame, "request_id", v, sizeof(v)));
    CHECK(!strcmp(v, "req-9"));
    CHECK(bp_json_get_top(frame, "device_id", v, sizeof(v)));
    CHECK(!strcmp(v, "dev-1"));
    CHECK(bp_json_get_top(frame, "timeout_ms", v, sizeof(v)));
    CHECK(!strcmp(v, "30000"));

    /* a key that exists ONLY inside the payload must not be found at top level */
    CHECK(!bp_json_get_top(frame, "cmd", v, sizeof(v)));
    CHECK(!bp_json_get_top(frame, "sda", v, sizeof(v)));

    /* the carried object still extracts whole, and its own keys read flat */
    char obj[128];
    CHECK(bp_json_object_top(frame, "command", obj, sizeof(obj)));
    CHECK(bp_json_get(obj, "type", v, sizeof(v)));
    CHECK(!strcmp(v, "bmp280"));
    CHECK(bp_json_get(obj, "cmd", v, sizeof(v)));
    CHECK(!strcmp(v, "sensor_start"));

    /* nested ARRAYS and strings containing braces/quotes must be skipped too */
    const char *tricky =
        "{\"command\":{\"data\":[1,2,3],\"note\":\"}{\\\"type\\\":\\\"nope\\\"\"},"
        "\"type\":\"command.request\"}";
    CHECK(bp_json_get_top(tricky, "type", v, sizeof(v)));
    CHECK(!strcmp(v, "command.request"));
}

/* bp_json_get_fit: provisioning values (SSID, password, cloud host) must never be stored cut short. */
static void test_get_fit(void) {
    char v[6];
    CHECK(bp_json_get_fit("{\"ssid\":\"abcde\"}", "ssid", v, sizeof(v)) == 1 && !strcmp(v, "abcde"));
    CHECK(bp_json_get_fit("{\"ssid\":\"abcdef\"}", "ssid", v, sizeof(v)) == -1);   /* one too long */
    CHECK(bp_json_get_fit("{\"ssid\":\"\"}", "ssid", v, sizeof(v)) == 1 && v[0] == '\0');
    CHECK(bp_json_get_fit("{\"x\":1}", "ssid", v, sizeof(v)) == 0);
    CHECK(bp_json_get_fit("{\"ssid\":\"a\\\"b\"}", "ssid", v, sizeof(v)) == 1 && !strcmp(v, "a\"b"));
    CHECK(bp_json_get_fit("{\"port\":12345}", "port", v, sizeof(v)) == 1 && !strcmp(v, "12345"));
    CHECK(bp_json_get_fit("{\"port\":123456}", "port", v, sizeof(v)) == -1);
}

int main(void) {
    test_get_fit();
    test_get_basic();
    test_top_level_not_shadowed_by_payload();
    test_key_vs_value();
    test_escapes();
    test_flag();
    test_object();
    test_byte_array();
    test_emit();
    if (failures) { printf("FAILED — %d json checks\n", failures); return 1; }
    printf("PASS — all json tests\n");
    return 0;
}
