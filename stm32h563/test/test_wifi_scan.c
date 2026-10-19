/*
 * test_wifi_scan.c — host tests for the Wi-Fi network list (src/wifi_scan.c):
 *   - strongest first, one entry per SSID (the strongest access point), hidden networks apart;
 *   - capped at WIFI_SCAN_MAX, the weakest dropped;
 *   - the JSON list escapes SSIDs and stops before the byte limit, never mid-entry.
 */
#include "wifi_scan.h"
#include "bp_json.h"

#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); failures++; } \
} while (0)

static void add(wifi_scan_list_t *l, const char *ssid, int rssi, unsigned ch, unsigned auth) {
    wifi_scan_list_add(l, ssid, strlen(ssid), rssi, ch, auth);
}

static void test_sort_and_merge(void) {
    wifi_scan_list_t l;
    wifi_scan_list_clear(&l);
    add(&l, "lab", -70, 1, 3);
    add(&l, "office", -40, 6, 7);
    add(&l, "lab", -55, 11, 3);      /* a stronger access point of "lab" replaces the first */
    add(&l, "lab", -80, 13, 3);      /* a weaker one is dropped */
    add(&l, "", -60, 3, 0);          /* hidden networks are never merged */
    add(&l, "", -65, 4, 0);
    CHECK(l.n == 4);
    CHECK(l.seen == 6);
    CHECK(strcmp(l.ap[0].ssid, "office") == 0 && l.ap[0].rssi == -40);
    CHECK(strcmp(l.ap[1].ssid, "lab") == 0 && l.ap[1].rssi == -55 && l.ap[1].channel == 11);
    CHECK(l.ap[2].ssid[0] == '\0' && l.ap[2].rssi == -60);
    CHECK(l.ap[3].ssid[0] == '\0' && l.ap[3].rssi == -65);
}

static void test_cap_keeps_strongest(void) {
    wifi_scan_list_t l;
    wifi_scan_list_clear(&l);
    char name[16];
    for (int i = 0; i < 30; i++) {   /* -90 .. -61, in rising order */
        snprintf(name, sizeof(name), "net%02d", i);
        add(&l, name, -90 + i, 1, 3);
    }
    CHECK(l.n == WIFI_SCAN_MAX);
    CHECK(l.seen == 30);
    CHECK(l.ap[0].rssi == -61);
    CHECK(l.ap[WIFI_SCAN_MAX - 1].rssi == -80);
    add(&l, "weak", -95, 1, 3);       /* weaker than everything in a full list */
    CHECK(l.n == WIFI_SCAN_MAX && l.ap[WIFI_SCAN_MAX - 1].rssi == -80);
}

static void test_ssid_bounds(void) {
    wifi_scan_list_t l;
    wifi_scan_list_clear(&l);
    const char raw[] = "0123456789012345678901234567890123456789";
    wifi_scan_list_add(&l, raw, sizeof(raw) - 1, -50, 1, 3);   /* longer than 32: cut */
    CHECK(strlen(l.ap[0].ssid) == 32);
    const char padded[8] = {'a', 'b', 0, 0, 0, 0, 0, 0};   /* NUL-padded */
    wifi_scan_list_add(&l, padded, sizeof(padded), -60, 1, 3);
    CHECK(strcmp(l.ap[1].ssid, "ab") == 0);
}

static void test_auth_names(void) {
    CHECK(strcmp(wifi_scan_auth_str(0), "open") == 0);
    CHECK(strcmp(wifi_scan_auth_str(3), "wpa2") == 0);
    CHECK(strcmp(wifi_scan_auth_str(7), "wpa2/wpa3") == 0);
    CHECK(strcmp(wifi_scan_auth_str(9), "owe") == 0);
    CHECK(strcmp(wifi_scan_auth_str(42), "other") == 0);
}

static void test_json(void) {
    wifi_scan_list_t l;
    wifi_scan_list_clear(&l);
    add(&l, "say \"hi\"", -48, 6, 3);
    add(&l, "guest", -71, 11, 0);
    char buf[512];
    bp_emit_t e;
    bp_emit_init(&e, buf, sizeof(buf));
    CHECK(wifi_scan_emit_networks(&e, &l, sizeof(buf)) == 2);
    CHECK(bp_emit_ok(&e));
    CHECK(strcmp(buf, "\"networks\":[{\"ssid\":\"say \\\"hi\\\"\",\"rssi\":-48,\"channel\":6,\"auth\":\"wpa2\"},"
                      "{\"ssid\":\"guest\",\"rssi\":-71,\"channel\":11,\"auth\":\"open\"}]") == 0);

    /* A limit that fits only the first entry: the list stops there and stays valid JSON. */
    bp_emit_init(&e, buf, sizeof(buf));
    CHECK(wifi_scan_emit_networks(&e, &l, 90) == 1);
    CHECK(bp_emit_ok(&e) && bp_emit_len(&e) <= 90);
    CHECK(buf[bp_emit_len(&e) - 1] == ']');

    /* A full list of 32-character SSIDs does not fit 1.7 KB: it is cut, never overrun. */
    wifi_scan_list_clear(&l);
    char name[40];
    for (int i = 0; i < 20; i++) {
        snprintf(name, sizeof(name), "%02d-abcdefghijklmnopqrstuvwxyz-xx", i);
        add(&l, name, -40 - i, 1, 7);
    }
    static char big[1792];
    bp_emit_init(&e, big, sizeof(big));
    bp_emit_raw(&e, "{\"status\":\"ok\",\"data\":{\"state\":\"done\",\"id\":1,\"via\":\"built-in\",");
    unsigned n = wifi_scan_emit_networks(&e, &l, sizeof(big) - 32);
    CHECK(n >= 17 && n < 20);   /* 20 x ~90 B does not fit: the strongest ones do */
    CHECK(bp_emit_ok(&e) && bp_emit_len(&e) <= sizeof(big) - 32);
    CHECK(big[bp_emit_len(&e) - 1] == ']');
}

int main(void) {
    test_sort_and_merge();
    test_cap_keeps_strongest();
    test_ssid_bounds();
    test_auth_names();
    test_json();
    if (failures) { printf("test_wifi_scan: %d failure(s)\n", failures); return 1; }
    printf("test_wifi_scan: all passed\n");
    return 0;
}
