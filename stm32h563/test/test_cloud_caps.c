/*
 * test_cloud_caps.c: the capabilities frame (src/cloud_caps.c, FW-6).
 *
 * A capabilities frame that did not fit one WS frame was not sent, and a failed capabilities
 * send made the pod reconnect, over and over. The worst case here (every feature on, the longest
 * ids, numbers at their widest and boot-health text made entirely of characters that escape to
 * six bytes) must still produce one frame that fits, with every feature field intact.
 */
#include "cloud_caps.h"
#include "bp_json.h"
#include "cloud_config.h"

#include <limits.h>
#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); failures++; } \
} while (0)

static cloud_caps_t typical(void) {
    cloud_caps_t c = {
        .device_id = "0b7d2c1e-5a4f-4c3b-9e8d-7f6a5b4c3d2e",
        .firmware_version = "3.6.0",
        .flash_kb = 2048,
        .sig_policy = "audit",
        .lan_policy = "open",
        .analog = true,
        .adc_bits = 16, .adc_fullscale_mv = 4096, .adc_channels = 1,
        .adc_cal_a_uv = -1234567, .adc_cal_b_nv = 98765,
        .adc_low_range = true, .adc_range = "low", .adc_range_fs_mv = 5461,
        .dac_ac = true, .dac_replay = true, .dac_dc = true,
        .dac_bits = 16, .dac_replay_bits = 16, .dac_fullscale_mv = 4096, .dac_channels = 1,
        .deep_replay = true, .replay_max_samples = 4194304,
        .control_loop = true, .loop_sources = true, .loop_input_map = true, .cotrig = true,
        .generate16 = true,
        .gpio_read = true, .capture_trigger = true, .spi_master = true, .la_max_rate_hz = 48000000ul,
        .nrst_pin = true, .pod_current = true,
        .current_out_min_ua = 0, .current_out_max_ua = 24000,
        .board = "stm32h563",
        .uplink = "eth",
        .safe_mode = false, .safe_reason = "", .reset_cause = "power-on", .last_crash = "none",
        .boot_id = 0x2a, .unclean_resets = 0,
        .eth_mac = "02:0d:1a:80:f1:54",
        .features_rev = 1, .api = 1, .min_client_api = 0, .gateware_version = 56, .fpga_image = "loop",
    };
    return c;
}

/* Every key the frame carries (firmware 4.0.0: values and facts, no feature flags). */
static void check_features(const char *f) {
    static const char *keys[] = {
        "device_id", "firmware_version", "flash_kb", "sig_policy", "lan_policy", "ws_auth_v2", "analog",
        "nrst_pin", "sensors_digital", "wifi_scan", "pod_current", "adc_low_range",
        "adc_bits", "adc_fullscale_mv", "adc_channels", "adc_cal_a_uv", "adc_cal_b_nv",
        "dac_bits", "dac_replay_bits", "dac_fullscale_mv", "dac_channels", "dac_replay_max_samples",
        "generate_bits", "la_max_rate_hz", "current_out_min_ua", "current_out_max_ua",
        "adc_range", "adc_range_fs_mv", "board", "safe_mode", "safe_reason", "reset_cause", "last_crash",
        "boot_id", "unclean_resets", "uplink", "features_rev", "api", "min_client_api",
        "gateware_version", "fpga_image",
    };
    char v[64];
    for (size_t i = 0; i < sizeof(keys) / sizeof(keys[0]); i++) {
        if (!bp_json_get_top(f, keys[i], v, sizeof(v)) && strstr(f, keys[i]) == NULL) {
            printf("  missing key %s\n", keys[i]);
            failures++;
        }
    }
}

static void test_typical_frame(void) {
    cloud_caps_t c = typical();
    static char f[CLOUD_CAPS_MAX + 1];
    size_t n = cloud_caps_build(&c, f, sizeof(f));
    CHECK(n > 0 && n == strlen(f));
    CHECK(f[0] == '{' && f[n - 1] == '}');
    CHECK(strstr(f, "\"type\":\"capabilities\"") != NULL);
    CHECK(strstr(f, "\"device_id\":\"0b7d2c1e-5a4f-4c3b-9e8d-7f6a5b4c3d2e\"") != NULL);
    CHECK(strstr(f, "\"adc_cal_a_uv\":-1234567") != NULL);
    CHECK(strstr(f, "\"adc_low_range\":true,") != NULL);
    CHECK(strstr(f, "\"adc_range\":\"low\",\"adc_range_fs_mv\":5461,") != NULL);
    CHECK(strstr(f, "\"last_crash\":\"none\"}") != NULL);
    CHECK(strstr(f, "\"boot_id\":\"0000002a\",\"unclean_resets\":0,") != NULL);
    CHECK(strstr(f, "c3_restarts") == NULL);   /* no C3 on the board: no counts */
    CHECK(strstr(f, "\"eth_mac\":\"02:0d:1a:80:f1:54\",") != NULL);
    CHECK(strstr(f, "wifi_mac") == NULL);      /* not known: left out */
    CHECK(strstr(f, "boot_health_cut") == NULL);
    char up[16];
    CHECK(bp_json_get_top(f, "uplink", up, sizeof(up)) && strcmp(up, "eth") == 0);
    check_features(f);
    printf("typical capabilities frame: %u bytes (limit %u)\n", (unsigned)n, (unsigned)CLOUD_CAPS_MAX);
}

static void test_worst_case_fits(void) {
    cloud_caps_t c = typical();
    char id[CLOUD_DEVICE_ID_MAX];
    memset(id, 'f', sizeof(id) - 1);
    id[sizeof(id) - 1] = '\0';
    c.device_id = id;
    c.firmware_version = "999.999.999-rc.999";
    c.sig_policy = "permissive";
    c.lan_policy = "locked";
    c.flash_kb = ULONG_MAX;
    c.replay_max_samples = ULONG_MAX;
    c.adc_bits = c.adc_fullscale_mv = c.adc_channels = INT_MIN;
    c.dac_bits = c.dac_replay_bits = c.dac_fullscale_mv = c.dac_channels = INT_MIN;
    c.adc_cal_a_uv = c.adc_cal_b_nv = LONG_MIN;
    c.adc_range = "normal";
    c.adc_range_fs_mv = LONG_MIN;
    c.current_out_min_ua = c.current_out_max_ua = LONG_MIN;
    c.board = "stm32h563-digital-v3r2";
    c.uplink = "wifi-bridge";   /* the longest uplink name, with its RSSI */
    c.has_uplink_rssi = true;
    c.uplink_rssi_dbm = -100;
    c.safe_mode = true;
    c.reset_cause = "independent-watchdog";
    c.boot_id = 0xFFFFFFFFu;
    c.unclean_resets = 0xFFFFFFFFu;
    c.api = c.min_client_api = UINT_MAX;   /* the client API level at its longest */
    c.c3 = true;                      /* the C3 counts at their longest */
    c.c3_restarts = 0xFFFFFFFFu;
    c.c3_restart_last = "no_response";
    strcpy(c.wifi_mac, "ff:ff:ff:ff:ff:ff");   /* both MACs known: dropped when the texts are cut */
    /* boot_guard's report is 127 characters, fault's crash line 159; a control character
       escapes to six bytes (\u00XX). */
    char reason[128], crash[160];
    memset(reason, 0x01, sizeof(reason) - 1);
    reason[sizeof(reason) - 1] = '\0';
    memset(crash, 0x02, sizeof(crash) - 1);
    crash[sizeof(crash) - 1] = '\0';
    c.safe_reason = reason;
    c.last_crash = crash;

    static char f[CLOUD_CAPS_MAX + 1];
    size_t n = cloud_caps_build(&c, f, sizeof(f));
    CHECK(n > 0);
    CHECK(n <= CLOUD_CAPS_MAX);
    CHECK(f[n - 1] == '}');
    CHECK(strstr(f, "\"boot_health_cut\":true") != NULL);
    CHECK(strstr(f, "\"c3_restarts\":4294967295,\"c3_restart_last\":\"no_response\",") != NULL);
    CHECK(strstr(f, "\"safe_mode\":true") != NULL);
    CHECK(strstr(f, "\"reset_cause\":\"independent-watchdog\"") != NULL);
    CHECK(strstr(f, "_mac") == NULL);
    check_features(f);
    printf("worst-case capabilities frame: %u bytes (limit %u)\n", (unsigned)n, (unsigned)CLOUD_CAPS_MAX);

    /* The same values uncut need more than the frame holds: the old 1536-byte buffer (and
       even the full WS scratch) dropped this frame. */
    static char big[8192];
    bp_emit_t e;
    bp_emit_init(&e, big, sizeof(big));
    bp_emit_jstr(&e, reason);
    bp_emit_jstr(&e, crash);
    CHECK(bp_emit_len(&e) > 1536u);

    /* Printable text that only needs cutting (no escapes) is kept up to the short length. */
    char longtxt[160];
    memset(longtxt, 'x', sizeof(longtxt) - 1);
    longtxt[sizeof(longtxt) - 1] = '\0';
    char lr[128];
    memset(lr, '"', sizeof(lr) - 1);   /* quotes escape to two bytes */
    lr[sizeof(lr) - 1] = '\0';
    c.last_crash = longtxt;
    c.safe_reason = lr;
    n = cloud_caps_build(&c, f, sizeof(f));
    CHECK(n > 0 && n <= CLOUD_CAPS_MAX);

    /* Both texts made of characters that escape to two bytes ('"', '\\'): the cut keeps them
       at one byte each, or this frame overflowed (it did once the uplink field was added). */
    char lc[640];   /* escaped (2 B each) it is more than the frame holds */
    memset(lc, '\\', sizeof(lc) - 1);
    lc[sizeof(lc) - 1] = '\0';
    c.last_crash = lc;
    c.safe_reason = lr;
    n = cloud_caps_build(&c, f, sizeof(f));
    CHECK(n > 0 && n <= CLOUD_CAPS_MAX);
    CHECK(strstr(f, "\"boot_health_cut\":true") != NULL);
    check_features(f);
}

/* "uplink" names the interface under the cloud link; an unset one (a caller that did not fill it)
   still sends the key, as "unknown", so the server can tell an old firmware (no key) apart. */
static void test_uplink(void) {
    static char f[CLOUD_CAPS_MAX + 1];
    char up[16];
    cloud_caps_t c = typical();
    c.uplink = "wifi";
    CHECK(cloud_caps_build(&c, f, sizeof(f)) > 0);
    CHECK(bp_json_get_top(f, "uplink", up, sizeof(up)) && strcmp(up, "wifi") == 0);
    CHECK(!bp_json_get_top(f, "uplink_rssi_dbm", up, sizeof(up)));
    c.uplink = NULL;
    CHECK(cloud_caps_build(&c, f, sizeof(f)) > 0);
    CHECK(bp_json_get_top(f, "uplink", up, sizeof(up)) && strcmp(up, "unknown") == 0);
    c.uplink = "wifi-bridge";   /* over Ethernet into a Wi-Fi bridge box */
    c.has_uplink_rssi = true;
    c.uplink_rssi_dbm = -61;
    CHECK(cloud_caps_build(&c, f, sizeof(f)) > 0);
    CHECK(bp_json_get_top(f, "uplink", up, sizeof(up)) && strcmp(up, "wifi-bridge") == 0);
    CHECK(bp_json_get_top(f, "uplink_rssi_dbm", up, sizeof(up)) && strcmp(up, "-61") == 0);
}

static void test_too_small_buffer(void) {
    cloud_caps_t c = typical();
    char f[200];
    CHECK(cloud_caps_build(&c, f, sizeof(f)) == 0);
}

/* Firmware 4.0.0: no per-feature flags in the frame, only values and the facts. */
static void test_frame_has_no_feature_flags(void) {
    static char f[CLOUD_CAPS_MAX + 1];
    cloud_caps_t c = typical();
    c.usb_cc = true;
    CHECK(cloud_caps_build(&c, f, sizeof(f)) > 0);
    static const char *const gone[] = { "\"ota\":", "\"blob_slots\"", "\"ota_sig\"", "\"sig_policy_cmd\"",
        "\"lease_state\"", "\"lan_policy_cmd\"", "\"tunnel_max_tier\"", "\"lan_tls\"", "\"wiring_store\"",
        "\"scope\"", "\"analyzer\"", "\"tunnel\"", "\"dac\":", "\"dac_control_loop\"", "\"la_pins\"",
        "\"capture_b64\"", "\"can\"", "\"sensors\":", "\"sensors_format\"", "\"usb_cc\"", "\"serial\"" };
    for (size_t i = 0; i < sizeof(gone) / sizeof(gone[0]); i++)
        if (strstr(f, gone[i])) { printf("FAIL frame still has %s\n", gone[i]); failures++; }
    CHECK(strstr(f, "\"features_rev\":") && strstr(f, "\"analog\":true") && strstr(f, "\"wifi_scan\":true"));
    CHECK(strstr(f, "\"ws_auth_v2\":true") && strstr(f, "\"adc_cal_a_uv\":"));
    /* Digital v3r3 has no ESP32-C3: wifi_scan false. */
    cloud_caps_t d = typical();
    d.no_wifi_scan = true;
    CHECK(cloud_caps_build(&d, f, sizeof(f)) > 0 && strstr(f, "\"wifi_scan\":false") != NULL);
}

/* What a status with "caps":false gets instead of the list, and the image names. */
static void test_status_facts(void) {
    char buf[256];
    bp_emit_t e;
    cloud_caps_t c = typical();
    c.usb_cc = true;
    bp_emit_init(&e, buf, sizeof(buf));
    cloud_caps_emit_facts(&e, &c);
    CHECK(bp_emit_ok(&e));
    CHECK(strcmp(buf, ",\"features_rev\":1,\"api\":1,\"min_client_api\":0,\"fpga_image\":\"loop\","
                      "\"hw\":[\"wifi\",\"usb_cc\",\"pod_current\",\"adc_low_range\"]") == 0);
    if (strcmp(buf, ",\"features_rev\":1,\"api\":1,\"min_client_api\":0,\"fpga_image\":\"loop\","
                    "\"hw\":[\"wifi\",\"usb_cc\",\"pod_current\",\"adc_low_range\"]") != 0)
        printf("  %s\n", buf);

    /* Digital v3r3: no C3, no analog parts: an empty list, and no image known yet. */
    c.no_wifi_scan = true;
    c.usb_cc = c.pod_current = c.adc_low_range = false;
    c.fpga_image = NULL;
    bp_emit_init(&e, buf, sizeof(buf));
    cloud_caps_emit_facts(&e, &c);
    CHECK(strcmp(buf, ",\"features_rev\":1,\"api\":1,\"min_client_api\":0,\"fpga_image\":\"unknown\",\"hw\":[]") == 0);

    CHECK(strcmp(cloud_caps_image_name(0, 0x07), "unknown") == 0);   /* no gateware read yet */
    CHECK(strcmp(cloud_caps_image_name(56, 0x01), "loop") == 0);
    CHECK(strcmp(cloud_caps_image_name(56, 0x02), "deep") == 0);
    CHECK(strcmp(cloud_caps_image_name(56, 0x04), "emu") == 0);
    CHECK(strcmp(cloud_caps_image_name(20, 0x00), "unknown") == 0);

    /* The frame carries them too, and no longer the flags the server never read from it. */
    static char f[CLOUD_CAPS_MAX + 1];
    c = typical();
    CHECK(cloud_caps_build(&c, f, sizeof(f)) > 0);
    CHECK(strstr(f, "\"features_rev\":1,\"api\":1,\"min_client_api\":0,\"gateware_version\":56,\"fpga_image\":\"loop\",") != NULL);
    static const char *gone[] = { "cal_dmm", "can_fd", "uart_frame", "spi_stream", "error_codes",
                                  "cloud_ca", "cloud_proxy" };
    for (size_t i = 0; i < sizeof(gone) / sizeof(gone[0]); i++)
        if (strstr(f, gone[i])) { printf("  %s still in the frame\n", gone[i]); failures++; }
}

int main(void) {
    test_uplink();
    test_status_facts();
    test_typical_frame();
    test_frame_has_no_feature_flags();
    test_worst_case_fits();
    test_too_small_buffer();
    if (failures) { printf("test_cloud_caps: %d FAILED\n", failures); return 1; }
    printf("test_cloud_caps: all passed\n");
    return 0;
}
