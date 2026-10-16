// Host unit tests for bridge_core: protocol codec, forwarding rules, helpers.
// Build and run: make -C test   (any C11 compiler; no ESP-IDF needed)
// `./build/test_bridge --dump` prints the example frames used in docs/PROTOCOL.md.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bridge_util.h"
#include "bwp.h"
#include "l2fwd.h"

static int failures, checks;

#define CHECK(cond)                                                          \
    do {                                                                     \
        checks++;                                                            \
        if (!(cond)) {                                                       \
            failures++;                                                      \
            fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
        }                                                                    \
    } while (0)

static const uint8_t BRIDGE[6] = {0x02, 0xbb, 0x00, 0x00, 0x00, 0x01};
static const uint8_t POD[6] = {0x00, 0x80, 0xe1, 0x12, 0x34, 0x56};
static const uint8_t OTHER[6] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55};
static const uint8_t ROUTER[6] = {0x60, 0x00, 0x00, 0x00, 0x00, 0x01};
static const uint8_t BCAST[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
static const uint8_t MDNS[6] = {0x01, 0x00, 0x5e, 0x00, 0x00, 0xfb};

static size_t frame(uint8_t *f, const uint8_t dst[6], const uint8_t src[6], uint16_t type)
{
    memset(f, 0, 64);
    memcpy(f, dst, 6);
    memcpy(f + 6, src, 6);
    f[12] = type >> 8;
    f[13] = type & 0xff;
    return 60;
}

static size_t unhex(const char *s, uint8_t *out)
{
    size_t n = 0;
    while (s[0] && s[1]) {
        if (s[0] == ' ') {
            s++;
            continue;
        }
        unsigned v;
        sscanf(s, "%2x", &v);
        out[n++] = (uint8_t)v;
        s += 2;
    }
    return n;
}

static void dump(const char *name, const uint8_t *f, size_t n)
{
    printf("%s (%zu bytes):\n", name, n);
    for (size_t i = 0; i < n; i++) {
        printf("%02x%s", f[i], (i % 16 == 15 || i == n - 1) ? "\n" : " ");
    }
}

// ---- Protocol codec

// HELLO with fixed values; the same bytes are in docs/PROTOCOL.md and tools/test_bridgectl.py.
static const char HELLO_HEX[] =
    "ffffffffffff" "02bb00000001" "88b5"               // dst, src, EtherType
    "4257" "01" "01" "0000" "003d"                     // magic, version, HELLO, seq, body length
    "02001462656e6368706f642d776966692d627269646765"  // MODEL
    "030005302e312e30"                               // FW_VERSION "0.1.0"
    "04000141"                                         // HW_REV "A"
    "05000101"                                         // STATE WAIT_POD
    "1600010b"                                         // FLAGS
    "06000602bb00000001"                               // BRIDGE_MAC
    "0700060080e1123456";                              // POD_MAC

static void build_example_hello(uint8_t *buf, size_t *n)
{
    bwp_hello_t h = {.fw_version = "0.1.0", .hw_rev = "A", .state = BWP_ST_WAIT_POD,
                     .flags = BWP_F_CONFIGURED | BWP_F_ETH_LINK | BWP_F_POD_KNOWN};
    memcpy(h.bridge_mac, BRIDGE, 6);
    memcpy(h.pod_mac, POD, 6);
    *n = bwp_build_hello(buf, BWP_MAX_FRAME, NULL, &h);
}

static void test_hello(void)
{
    uint8_t buf[BWP_MAX_FRAME], want[256];
    size_t n;
    build_example_hello(buf, &n);
    size_t wn = unhex(HELLO_HEX, want);
    CHECK(n == wn);
    CHECK(n == wn && memcmp(buf, want, n) == 0);

    bwp_msg_t m;
    CHECK(bwp_parse(buf, n, &m) == BWP_PARSE_OK);
    CHECK(m.type == BWP_HELLO && m.seq == 0 && m.version == 1);
    CHECK(memcmp(m.src, BRIDGE, 6) == 0 && memcmp(m.dst, BCAST, 6) == 0);
    char s[40];
    CHECK(bwp_get_str(m.body, m.body_len, BWP_T_MODEL, s, sizeof(s)) && strcmp(s, BWP_MODEL) == 0);
    CHECK(bwp_get_str(m.body, m.body_len, BWP_T_FW_VERSION, s, sizeof(s)) && strcmp(s, "0.1.0") == 0);
    uint8_t st = 99;
    CHECK(bwp_get_u8(m.body, m.body_len, BWP_T_STATE, &st) && st == BWP_ST_WAIT_POD);
    const uint8_t *v;
    uint16_t vl;
    CHECK(bwp_find(m.body, m.body_len, BWP_T_POD_MAC, &v, &vl) && vl == 6 && memcmp(v, POD, 6) == 0);
    // Too-small string buffer is refused, not truncated.
    CHECK(!bwp_get_str(m.body, m.body_len, BWP_T_MODEL, s, 5));
}

static void test_pad_and_overflow(void)
{
    uint8_t buf[BWP_MAX_FRAME];
    size_t n = bwp_build_result(buf, sizeof(buf), POD, BRIDGE, BWP_REBOOT, 7, BWP_OK, NULL);
    CHECK(n == BWP_MIN_FRAME);  // padded
    bwp_msg_t m;
    CHECK(bwp_parse(buf, n, &m) == BWP_PARSE_OK);
    CHECK(m.type == (BWP_REBOOT | BWP_RESPONSE) && m.seq == 7 && m.body_len == 4);
    uint8_t r = 99;
    CHECK(bwp_get_u8(m.body, m.body_len, BWP_T_RESULT, &r) && r == BWP_OK);
    CHECK(!bwp_find(m.body, m.body_len, BWP_T_MESSAGE, &(const uint8_t *){0}, &(uint16_t){0}));

    // Overflow: 2 KiB DATA does not fit in one frame.
    static uint8_t big[2048];
    bwp_writer_t w;
    bwp_begin(&w, buf, sizeof(buf), POD, BRIDGE, BWP_OTA_DATA, 1);
    bwp_put(&w, BWP_T_DATA, big, sizeof(big));
    CHECK(bwp_finish(&w) == 0);
    // Exactly the maximum fits.
    bwp_begin(&w, buf, sizeof(buf), POD, BRIDGE, BWP_OTA_DATA, 1);
    bwp_put(&w, BWP_T_DATA, big, BWP_MAX_BODY - 3);
    CHECK(bwp_finish(&w) == BWP_MAX_FRAME);
    // Cap larger than a frame is clamped to 1514.
    uint8_t huge[4096];
    bwp_begin(&w, huge, sizeof(huge), POD, BRIDGE, BWP_OTA_DATA, 1);
    bwp_put(&w, BWP_T_DATA, big, BWP_MAX_BODY - 2);
    CHECK(bwp_finish(&w) == 0);
}

static void test_parse_rejects(void)
{
    uint8_t buf[BWP_MAX_FRAME];
    size_t n = bwp_build_result(buf, sizeof(buf), POD, BRIDGE, BWP_STATUS, 1, BWP_OK, "x");
    bwp_msg_t m;

    uint8_t f[BWP_MAX_FRAME];
    memcpy(f, buf, n);
    f[12] = 0x08, f[13] = 0x00;  // IPv4
    CHECK(bwp_parse(f, n, &m) == BWP_PARSE_NOT_BWP);
    memcpy(f, buf, n);
    f[14] = 'X';  // someone else's 0x88B5 experiment
    CHECK(bwp_parse(f, n, &m) == BWP_PARSE_NOT_BWP);
    memcpy(f, buf, n);
    f[16] = 2;
    CHECK(bwp_parse(f, n, &m) == BWP_PARSE_BAD_VERSION);
    CHECK(bwp_parse(buf, 20, &m) == BWP_PARSE_MALFORMED);  // header cut
    memcpy(f, buf, n);
    f[20] = 0x05;  // body length beyond the frame
    CHECK(bwp_parse(f, n, &m) == BWP_PARSE_MALFORMED);
    memcpy(f, buf, n);
    f[21] = 5;  // body length 5: RESULT TLV (4) + 1 stray byte = truncated TLV
    CHECK(bwp_parse(f, n, &m) == BWP_PARSE_MALFORMED);
    memcpy(f, buf, n);
    f[22 + 5] = 0x10;  // RESULT TLV ok, MESSAGE TLV length 0x1001 runs past the body
    CHECK(bwp_parse(f, n, &m) == BWP_PARSE_MALFORMED);
    CHECK(bwp_parse(buf, n, &m) == BWP_PARSE_OK);
    // Empty body is fine (STATUS request).
    bwp_writer_t w;
    bwp_begin(&w, f, sizeof(f), BRIDGE, POD, BWP_STATUS, 3);
    n = bwp_finish(&w);
    CHECK(bwp_parse(f, n, &m) == BWP_PARSE_OK && m.body_len == 0 && m.type == BWP_STATUS && m.seq == 3);
}

static void test_status(void)
{
    uint8_t buf[BWP_MAX_FRAME];
    bwp_status_t s = {.state = BWP_ST_CONNECTED, .flags = BWP_F_CONFIGURED | BWP_F_POD_KNOWN, .channel = 6,
                      .rssi = -61, .uptime_s = 1234,
                      .counters = {.eth_rx = 1, .wifi_tx = 2, .wifi_tx_err = 3, .wifi_rx = 4, .eth_tx = 5,
                                   .eth_tx_err = 6, .dropped = 7}};
    strcpy(s.ssid, "lab net");
    memcpy(s.bssid, ROUTER, 6);
    memcpy(s.pod_mac, POD, 6);
    size_t n = bwp_build_status(buf, sizeof(buf), POD, BRIDGE, 42, &s);
    bwp_msg_t m;
    CHECK(n > 0 && bwp_parse(buf, n, &m) == BWP_PARSE_OK);
    CHECK(m.type == 0x82 && m.seq == 42);
    char ssid[33];
    uint8_t ch = 0, rssi = 0;
    uint32_t up = 0;
    CHECK(bwp_get_str(m.body, m.body_len, BWP_T_SSID, ssid, sizeof(ssid)) && strcmp(ssid, "lab net") == 0);
    CHECK(bwp_get_u8(m.body, m.body_len, BWP_T_CHANNEL, &ch) && ch == 6);
    CHECK(bwp_get_u8(m.body, m.body_len, BWP_T_RSSI, &rssi) && (int8_t)rssi == -61);
    CHECK(bwp_get_u32(m.body, m.body_len, BWP_T_UPTIME, &up) && up == 1234);
    const uint8_t *c;
    uint16_t cl;
    CHECK(bwp_find(m.body, m.body_len, BWP_T_COUNTERS, &c, &cl) && cl == 28);
    CHECK(c[3] == 1 && c[27] == 7);
    CHECK(!bwp_find(m.body, m.body_len, BWP_T_REASON, &c, &cl));

    // Not connected: no channel/RSSI/BSSID.
    s.state = BWP_ST_AUTH_FAILED;
    s.reason = 15;
    n = bwp_build_status(buf, sizeof(buf), POD, BRIDGE, 43, &s);
    CHECK(bwp_parse(buf, n, &m) == BWP_PARSE_OK);
    CHECK(!bwp_find(m.body, m.body_len, BWP_T_CHANNEL, &c, &cl));
    uint8_t reason = 0;
    CHECK(bwp_get_u8(m.body, m.body_len, BWP_T_REASON, &reason) && reason == 15);
}

static void test_scan_result(void)
{
    bwp_ap_t aps[25];
    memset(aps, 0, sizeof(aps));
    for (int i = 0; i < 25; i++) {
        memset(aps[i].ssid, 'a' + (i % 26), 32);  // worst case: 32-byte SSIDs
        aps[i].rssi = (int8_t)(-30 - i);
        aps[i].channel = (uint8_t)(1 + i % 13);
        aps[i].authmode = BWP_AUTH_WPA2_PSK;
    }
    uint8_t buf[BWP_MAX_FRAME];
    size_t n = bwp_build_scan_result(buf, sizeof(buf), POD, BRIDGE, 9, BWP_OK, aps, 25);
    bwp_msg_t m;
    CHECK(n > 0 && n <= BWP_MAX_FRAME);
    CHECK(bwp_parse(buf, n, &m) == BWP_PARSE_OK);
    bwp_iter_t it;
    uint8_t tag;
    const uint8_t *v;
    uint16_t vl;
    int count = 0;
    bwp_iter_init(&it, m.body, m.body_len);
    while (bwp_iter_next(&it, &tag, &v, &vl)) {
        if (tag != BWP_T_AP) {
            continue;
        }
        CHECK(bwp_tlv_valid(v, vl));
        char ssid[33];
        uint8_t rssi;
        CHECK(bwp_get_str(v, vl, BWP_T_SSID, ssid, sizeof(ssid)) && strlen(ssid) == 32);
        CHECK(bwp_get_u8(v, vl, BWP_T_RSSI, &rssi) && (int8_t)rssi == -30 - count);
        count++;
    }
    CHECK(count == BWP_SCAN_MAX_APS);  // capped at 20
}

static void test_set_wifi(void)
{
    uint8_t buf[BWP_MAX_FRAME];
    bwp_writer_t w;
    bwp_msg_t m;
    bwp_set_wifi_t sw;

    bwp_begin(&w, buf, sizeof(buf), BRIDGE, POD, BWP_SET_WIFI, 5);
    bwp_put_str(&w, BWP_T_SSID, "lab net");
    bwp_put_str(&w, BWP_T_PASSPHRASE, "correct horse");
    CHECK(bwp_parse(buf, bwp_finish(&w), &m) == BWP_PARSE_OK);
    CHECK(bwp_parse_set_wifi(&m, &sw) == BWP_OK);
    CHECK(strcmp(sw.ssid, "lab net") == 0 && sw.ssid_len == 7 && strcmp(sw.passphrase, "correct horse") == 0);

    // Open network: no passphrase TLV.
    bwp_begin(&w, buf, sizeof(buf), BRIDGE, POD, BWP_SET_WIFI, 5);
    bwp_put_str(&w, BWP_T_SSID, "guest");
    CHECK(bwp_parse(buf, bwp_finish(&w), &m) == BWP_PARSE_OK);
    CHECK(bwp_parse_set_wifi(&m, &sw) == BWP_OK && sw.passphrase[0] == 0);

    // Missing SSID, too long SSID, short passphrase.
    bwp_begin(&w, buf, sizeof(buf), BRIDGE, POD, BWP_SET_WIFI, 5);
    bwp_put_str(&w, BWP_T_PASSPHRASE, "12345678");
    CHECK(bwp_parse(buf, bwp_finish(&w), &m) == BWP_PARSE_OK && bwp_parse_set_wifi(&m, &sw) == BWP_E_BAD_REQUEST);
    bwp_begin(&w, buf, sizeof(buf), BRIDGE, POD, BWP_SET_WIFI, 5);
    bwp_put_str(&w, BWP_T_SSID, "123456789012345678901234567890123");
    CHECK(bwp_parse(buf, bwp_finish(&w), &m) == BWP_PARSE_OK && bwp_parse_set_wifi(&m, &sw) == BWP_E_BAD_REQUEST);
    bwp_begin(&w, buf, sizeof(buf), BRIDGE, POD, BWP_SET_WIFI, 5);
    bwp_put_str(&w, BWP_T_SSID, "x");
    bwp_put_str(&w, BWP_T_PASSPHRASE, "short");
    CHECK(bwp_parse(buf, bwp_finish(&w), &m) == BWP_PARSE_OK && bwp_parse_set_wifi(&m, &sw) == BWP_E_BAD_REQUEST);
}

static void test_passphrase_rules(void)
{
    CHECK(bwp_passphrase_valid("", 0));
    CHECK(!bwp_passphrase_valid("1234567", 7));
    CHECK(bwp_passphrase_valid("12345678", 8));
    char p[80];
    memset(p, 'x', 63);
    CHECK(bwp_passphrase_valid(p, 63));
    memset(p, 'a', 64);
    CHECK(bwp_passphrase_valid(p, 64));  // raw PSK in hex
    p[10] = 'g';
    CHECK(!bwp_passphrase_valid(p, 64));
    CHECK(!bwp_passphrase_valid("abc\tdefgh", 9));  // control character
    CHECK(!bwp_ssid_valid((const uint8_t *)"", 0));
    CHECK(bwp_ssid_valid((const uint8_t *)"a", 1));
}

// ---- Forwarding rules

static void test_learning(void)
{
    l2fwd_t s;
    l2_event_t ev;
    uint8_t f[64];
    l2fwd_init(&s, BRIDGE);

    // Nothing learned: Wi-Fi traffic is dropped.
    frame(f, BCAST, ROUTER, 0x0806);
    CHECK(l2fwd_from_wifi(&s, f, 60) == L2_DROP && s.last_why == L2_WHY_NO_POD);

    // Bad sources never teach us anything.
    frame(f, BCAST, MDNS, 0x0800);
    CHECK(l2fwd_from_eth(&s, f, 60, true, &ev) == L2_DROP && s.last_why == L2_WHY_BAD_SRC && !s.pod_known);
    frame(f, BCAST, (const uint8_t[6]){0}, 0x0800);
    CHECK(l2fwd_from_eth(&s, f, 60, true, &ev) == L2_DROP && !s.pod_known);
    frame(f, BCAST, BRIDGE, 0x0800);
    CHECK(l2fwd_from_eth(&s, f, 60, true, &ev) == L2_DROP && s.last_why == L2_WHY_OWN_SRC && !s.pod_known);
    CHECK(l2fwd_from_eth(&s, f, 10, true, &ev) == L2_DROP && s.last_why == L2_WHY_RUNT);

    // First frame from the pod (DHCP discover) is learned, and dropped while Wi-Fi is down.
    frame(f, BCAST, POD, 0x0800);
    CHECK(l2fwd_from_eth(&s, f, 60, false, &ev) == L2_DROP && s.last_why == L2_WHY_WIFI_DOWN);
    CHECK(ev == L2_EV_LEARNED && s.pod_known && memcmp(s.pod_mac, POD, 6) == 0);
    CHECK(l2fwd_from_eth(&s, f, 60, true, &ev) == L2_FORWARD && ev == L2_EV_NONE);

    // A second device on the cable is ignored while the link stays up.
    frame(f, ROUTER, OTHER, 0x0800);
    CHECK(l2fwd_from_eth(&s, f, 60, true, &ev) == L2_DROP && s.last_why == L2_WHY_OTHER_MAC && ev == L2_EV_NONE);
    CHECK(memcmp(s.pod_mac, POD, 6) == 0);

    // Link down, same pod comes back: no event.
    l2fwd_link_down(&s);
    frame(f, ROUTER, POD, 0x0800);
    CHECK(l2fwd_from_eth(&s, f, 60, true, &ev) == L2_FORWARD && ev == L2_EV_NONE && !s.relearn);

    // Link down, a different pod is plugged in: CHANGED.
    l2fwd_link_down(&s);
    frame(f, ROUTER, OTHER, 0x0800);
    CHECK(l2fwd_from_eth(&s, f, 60, true, &ev) == L2_FORWARD && ev == L2_EV_CHANGED);
    CHECK(memcmp(s.pod_mac, OTHER, 6) == 0);
    frame(f, ROUTER, POD, 0x0800);
    CHECK(l2fwd_from_eth(&s, f, 60, true, &ev) == L2_DROP && s.last_why == L2_WHY_OTHER_MAC);
}

static void test_forwarding(void)
{
    l2fwd_t s;
    l2_event_t ev;
    uint8_t f[64];
    l2fwd_init(&s, BRIDGE);
    frame(f, ROUTER, POD, 0x0800);
    CHECK(l2fwd_from_eth(&s, f, 60, true, &ev) == L2_FORWARD);

    // Ethernet side
    frame(f, BRIDGE, POD, 0x0800);
    CHECK(l2fwd_from_eth(&s, f, 60, true, NULL) == L2_DROP && s.last_why == L2_WHY_TO_BRIDGE);
    frame(f, MDNS, POD, 0x0800);
    CHECK(l2fwd_from_eth(&s, f, 60, true, NULL) == L2_FORWARD);
    frame(f, ROUTER, POD, 0x86dd);
    CHECK(l2fwd_from_eth(&s, f, 60, true, NULL) == L2_FORWARD);

    // Wi-Fi side
    frame(f, POD, ROUTER, 0x0800);
    CHECK(l2fwd_from_wifi(&s, f, 60) == L2_FORWARD);
    frame(f, BCAST, ROUTER, 0x0806);
    CHECK(l2fwd_from_wifi(&s, f, 60) == L2_FORWARD);
    frame(f, MDNS, OTHER, 0x0800);
    CHECK(l2fwd_from_wifi(&s, f, 60) == L2_FORWARD);
    frame(f, OTHER, ROUTER, 0x0800);
    CHECK(l2fwd_from_wifi(&s, f, 60) == L2_DROP && s.last_why == L2_WHY_NOT_FOR_POD);
    frame(f, BCAST, POD, 0x86dd);  // AP reflecting the pod's own multicast: breaks IPv6 DAD
    CHECK(l2fwd_from_wifi(&s, f, 60) == L2_DROP && s.last_why == L2_WHY_REFLECTED);
    CHECK(l2fwd_from_wifi(&s, f, 13) == L2_DROP && s.last_why == L2_WHY_RUNT);
}

static void test_control_frames(void)
{
    l2fwd_t s;
    l2_event_t ev;
    uint8_t f[64];
    l2fwd_init(&s, BRIDGE);

    // A control frame from the pod is accepted (unicast or broadcast) and teaches its MAC.
    frame(f, BRIDGE, POD, BWP_ETHERTYPE);
    CHECK(l2fwd_from_eth(&s, f, 60, false, &ev) == L2_CONTROL && ev == L2_EV_LEARNED);
    frame(f, BCAST, POD, BWP_ETHERTYPE);
    CHECK(l2fwd_from_eth(&s, f, 60, true, &ev) == L2_CONTROL);
    // Never forwarded, even when addressed elsewhere.
    frame(f, ROUTER, POD, BWP_ETHERTYPE);
    CHECK(l2fwd_from_eth(&s, f, 60, true, &ev) == L2_DROP && s.last_why == L2_WHY_CONTROL);
    // Not from another device on the cable.
    frame(f, BRIDGE, OTHER, BWP_ETHERTYPE);
    CHECK(l2fwd_from_eth(&s, f, 60, true, &ev) == L2_DROP && s.last_why == L2_WHY_OTHER_MAC);
    // Never from the radio, whoever it claims to be.
    frame(f, POD, ROUTER, BWP_ETHERTYPE);
    CHECK(l2fwd_from_wifi(&s, f, 60) == L2_DROP && s.last_why == L2_WHY_CONTROL);
    frame(f, BCAST, ROUTER, BWP_ETHERTYPE);
    CHECK(l2fwd_from_wifi(&s, f, 60) == L2_DROP && s.last_why == L2_WHY_CONTROL);
}

// ---- Helpers

static void test_tokenize(void)
{
    char line[128];
    char *argv[8];

    strcpy(line, "  set-wifi   \"lab net\" \"pass with \\\"quote\\\"\"  ");
    CHECK(bu_tokenize(line, argv, 8) == 3);
    CHECK(strcmp(argv[0], "set-wifi") == 0 && strcmp(argv[1], "lab net") == 0);
    CHECK(strcmp(argv[2], "pass with \"quote\"") == 0);

    strcpy(line, "set-wifi my\\ ssid pa\"ss wo\"rd");
    CHECK(bu_tokenize(line, argv, 8) == 3);
    CHECK(strcmp(argv[1], "my ssid") == 0 && strcmp(argv[2], "pass word") == 0);

    strcpy(line, "set-wifi \"\"");  // explicit empty argument
    CHECK(bu_tokenize(line, argv, 8) == 2 && argv[1][0] == 0);

    strcpy(line, "status");
    CHECK(bu_tokenize(line, argv, 8) == 1 && strcmp(argv[0], "status") == 0);
    strcpy(line, "   ");
    CHECK(bu_tokenize(line, argv, 8) == 0);
    strcpy(line, "a \"b");
    CHECK(bu_tokenize(line, argv, 8) == -1);
    strcpy(line, "a b c");
    CHECK(bu_tokenize(line, argv, 2) == -2);
}

static void test_reasons(void)
{
    CHECK(bu_state_for_reason(202) == BWP_ST_AUTH_FAILED);
    CHECK(bu_state_for_reason(15) == BWP_ST_AUTH_FAILED);
    CHECK(bu_state_for_reason(201) == BWP_ST_NO_AP);
    CHECK(bu_state_for_reason(210) == BWP_ST_NO_AP);
    CHECK(bu_state_for_reason(8) == BWP_ST_JOINING);    // ASSOC_LEAVE
    CHECK(bu_state_for_reason(200) == BWP_ST_JOINING);  // BEACON_TIMEOUT
}

static void test_ota_seq(void)
{
    bu_ota_seq_t s = {0};
    CHECK(bu_ota_check(&s, 0, 10) == BU_OTA_INACTIVE);
    bu_ota_begin(&s, 2500);
    CHECK(bu_ota_check(&s, 0, 1024) == BU_OTA_WRITE);
    bu_ota_advance(&s, 0, 1024);
    CHECK(bu_ota_check(&s, 0, 1024) == BU_OTA_DUPLICATE);  // our ack was lost
    CHECK(bu_ota_check(&s, 2048, 452) == BU_OTA_BAD_OFFSET);
    CHECK(bu_ota_check(&s, 512, 512) == BU_OTA_BAD_OFFSET);
    CHECK(bu_ota_check(&s, 1024, 1024) == BU_OTA_WRITE);
    bu_ota_advance(&s, 1024, 1024);
    CHECK(!bu_ota_complete(&s));
    CHECK(bu_ota_check(&s, 2048, 1024) == BU_OTA_TOO_LARGE);
    CHECK(bu_ota_check(&s, 2048, 452) == BU_OTA_WRITE);
    bu_ota_advance(&s, 2048, 452);
    CHECK(bu_ota_complete(&s));
}

static void test_image_ident(void)
{
    uint8_t img[BU_IMG_DESC_END];
    char proj[33], ver[33];
    memset(img, 0, sizeof(img));
    img[0] = 0xE9;
    img[32] = 0x32, img[33] = 0x54, img[34] = 0xCD, img[35] = 0xAB;
    strcpy((char *)img + 48, "0.2.0");
    strcpy((char *)img + 80, "benchpod-wifi-bridge");
    CHECK(bu_image_ident(img, sizeof(img), proj, ver));
    CHECK(strcmp(proj, "benchpod-wifi-bridge") == 0 && strcmp(ver, "0.2.0") == 0);
    CHECK(!bu_image_ident(img, sizeof(img) - 1, proj, ver));
    img[33] = 0;
    CHECK(!bu_image_ident(img, sizeof(img), proj, ver));
    img[33] = 0x54;
    img[0] = 0xE8;
    CHECK(!bu_image_ident(img, sizeof(img), proj, ver));

    char m[18];
    bu_mac_str(POD, m);
    CHECK(strcmp(m, "00:80:e1:12:34:56") == 0);
}

int main(int argc, char **argv)
{
    if (argc > 1 && strcmp(argv[1], "--dump") == 0) {
        uint8_t buf[BWP_MAX_FRAME];
        size_t n;
        build_example_hello(buf, &n);
        dump("HELLO", buf, n);
        bwp_writer_t w;
        bwp_begin(&w, buf, sizeof(buf), BRIDGE, POD, BWP_SET_WIFI, 0x0102);
        bwp_put_str(&w, BWP_T_SSID, "lab");
        bwp_put_str(&w, BWP_T_PASSPHRASE, "secret12");
        n = bwp_finish(&w);
        dump("SET_WIFI", buf, n);
        n = bwp_build_result(buf, sizeof(buf), POD, BRIDGE, BWP_SET_WIFI, 0x0102, BWP_OK, NULL);
        dump("SET_WIFI response", buf, n);
        return 0;
    }
    test_hello();
    test_pad_and_overflow();
    test_parse_rejects();
    test_status();
    test_scan_result();
    test_set_wifi();
    test_passphrase_rules();
    test_learning();
    test_forwarding();
    test_control_frames();
    test_tokenize();
    test_reasons();
    test_ota_seq();
    test_image_ident();
    printf("%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
