/*
 * test_lan_clients.c — host tests for the paired-computer list (src/lan_clients.c): add, rename,
 * full, remove by name and by fingerprint prefix, the stored form and its validation.
 */
#include "lan_clients.h"

#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(cond, ...) do { if (!(cond)) { printf("FAIL %s:%d: ", __func__, __LINE__); \
        printf(__VA_ARGS__); printf("\n"); failures++; } } while (0)

static void fp_of(uint8_t fp[32], uint8_t seed) {
    for (int i = 0; i < 32; i++) fp[i] = (uint8_t)(seed * 7 + i);
}

static void test_names(void) {
    CHECK(lan_clients_name_ok("ci-runner-1"), "plain");
    CHECK(lan_clients_name_ok("Edward's MacBook Pro"), "spaces and apostrophe");
    CHECK(!lan_clients_name_ok(""), "empty");
    CHECK(!lan_clients_name_ok("a\"b"), "quote");
    CHECK(!lan_clients_name_ok("a\\b"), "backslash");
    CHECK(!lan_clients_name_ok("tab\there"), "control");
    CHECK(lan_clients_name_ok("12345678901234567890123"), "23 chars");
    CHECK(!lan_clients_name_ok("123456789012345678901234"), "24 chars");
    CHECK(!lan_clients_name_ok(NULL), "NULL");
}

static void test_hex(void) {
    uint8_t fp[32], back[32];
    fp_of(fp, 3);
    char hex[65];
    lan_clients_fp_to_hex(fp, hex);
    CHECK(strlen(hex) == 64, "length");
    CHECK(lan_clients_fp_from_hex(hex, back) && !memcmp(fp, back, 32), "round trip");
    hex[0] = (char)(hex[0] >= 'a' ? hex[0] - 'a' + 'A' : hex[0]);
    CHECK(lan_clients_fp_from_hex(hex, back), "upper case");
    CHECK(!lan_clients_fp_from_hex("abc", back), "short");
    hex[5] = 'g';
    CHECK(!lan_clients_fp_from_hex(hex, back), "not hex");
}

static void test_add_remove(void) {
    lan_clients_t l;
    lan_clients_init(&l, 3);
    uint8_t a[32], b[32], c[32], d[32];
    fp_of(a, 1); fp_of(b, 2); fp_of(c, 3); fp_of(d, 4);
    CHECK(lan_clients_add(&l, a, "laptop") == LAN_CLIENTS_ADDED, "add a");
    CHECK(lan_clients_add(&l, a, "laptop") == LAN_CLIENTS_SAME, "same");
    CHECK(lan_clients_add(&l, a, "desk") == LAN_CLIENTS_RENAMED && !strcmp(l.c[0].name, "desk"), "rename");
    CHECK(lan_clients_add(&l, b, "ci") == LAN_CLIENTS_ADDED, "add b");
    CHECK(lan_clients_add(&l, c, "ci") == LAN_CLIENTS_ADDED, "add c, same name");
    CHECK(lan_clients_add(&l, d, "x") == LAN_CLIENTS_FULL, "full at cap");
    CHECK(lan_clients_add(&l, d, "") == LAN_CLIENTS_BAD_NAME, "bad name before full");
    CHECK(lan_clients_find(&l, c) == 2 && lan_clients_find(&l, d) == -1, "find");

    CHECK(lan_clients_remove(&l, "ci") == 2 && l.n == 1, "a name removes every entry with it");
    CHECK(lan_clients_find(&l, a) == 0, "a stays");
    CHECK(lan_clients_remove(&l, "nobody") == 0, "no match");

    lan_clients_add(&l, b, "b");
    char hex[65];
    lan_clients_fp_to_hex(b, hex);
    hex[10] = '\0';
    CHECK(lan_clients_remove(&l, "abc") == 0, "a prefix needs 8 digits");
    CHECK(lan_clients_remove(&l, hex) == 1 && lan_clients_find(&l, b) < 0, "remove by prefix");

    /* two keys with the same first bytes: a short prefix is ambiguous */
    uint8_t e[32];
    memcpy(e, a, 32);
    e[31] ^= 1;
    lan_clients_add(&l, e, "e");
    lan_clients_fp_to_hex(a, hex);
    hex[16] = '\0';
    CHECK(lan_clients_remove(&l, hex) == -2 && l.n == 2, "ambiguous prefix removes nothing");
    lan_clients_fp_to_hex(a, hex);
    CHECK(lan_clients_remove(&l, hex) == 1 && lan_clients_find(&l, e) == 0, "full fingerprint");

    lan_clients_clear(&l);
    CHECK(l.n == 0 && l.cap == 3, "clear keeps the capacity");
}

static void test_pack(void) {
    lan_clients_t l, m;
    lan_clients_init(&l, 4);
    uint8_t a[32], b[32];
    fp_of(a, 9); fp_of(b, 10);
    lan_clients_add(&l, a, "one");
    lan_clients_add(&l, b, "two");
    uint8_t buf[1 + 4 * sizeof(lan_client_t)];
    size_t n = lan_clients_pack(&l, buf, sizeof(buf));
    CHECK(n == lan_clients_packed_size(2), "size");
    CHECK(lan_clients_pack(&l, buf, 10) == 0, "too small");

    lan_clients_init(&m, 4);
    CHECK(lan_clients_unpack(&m, buf, n) && m.n == 2 && !strcmp(m.c[1].name, "two"), "round trip");
    CHECK(!lan_clients_unpack(&m, buf, n - 1), "wrong length");

    lan_clients_init(&m, 1);
    CHECK(!lan_clients_unpack(&m, buf, n) && m.n == 0, "more than the capacity");

    lan_clients_init(&m, 4);
    uint8_t bad[sizeof(buf)];
    memcpy(bad, buf, n);
    bad[1 + 32] = '"';                    /* first name starts with a quote */
    CHECK(!lan_clients_unpack(&m, bad, n), "bad name");
    memcpy(bad, buf, n);
    memcpy(bad + 1 + sizeof(lan_client_t), bad + 1, 32);   /* the same key twice */
    CHECK(!lan_clients_unpack(&m, bad, n), "duplicate key");
    uint8_t zero = 0;
    CHECK(lan_clients_unpack(&m, &zero, 1) && m.n == 0, "empty list");
}

int main(void) {
    test_names();
    test_hex();
    test_add_remove();
    test_pack();
    if (failures) { printf("test_lan_clients: %d FAILED\n", failures); return 1; }
    printf("test_lan_clients: all passed\n");
    return 0;
}
