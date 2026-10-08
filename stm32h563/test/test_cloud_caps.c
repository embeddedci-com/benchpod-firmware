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
        .dac_ac = true, .dac_replay = true, .dac_dc = true,
        .dac_bits = 16, .dac_replay_bits = 16, .dac_fullscale_mv = 4096, .dac_channels = 1,
        .deep_replay = true, .replay_max_samples = 4194304,
        .control_loop = true, .loop_sources = true, .loop_input_map = true, .cotrig = true,
        .gpio_read = true, .capture_trigger = true, .spi_master = true,
        .nrst_pin = true, .pod_current = true,
        .current_out_min_ua = 0, .current_out_max_ua = 24000,
        .board = "stm32h563",
        .safe_mode = false, .safe_reason = "", .reset_cause = "power-on", .last_crash = "none",
        .boot_id = 0x2a, .unclean_resets = 0,
    };
    return c;
}

/* Every feature key the server reads, present in the frame. */
static void check_features(const char *f) {
    static const char *keys[] = {
        "device_id", "firmware_version", "flash_kb", "sig_policy", "lan_policy", "scope", "analog",
        "adc_bits", "adc_cal_a_uv", "adc_cal_b_nv", "dac", "dac_replay", "dac_dc",
        "dac_deep_replay", "dac_replay_max_samples", "dac_control_loop", "dac_loop_sources",
        "dac_loop_input_map", "dac_cotrig", "gpio_read", "capture_trigger", "spi_master",
        "spi_stream", "nrst_pin", "pod_current", "current_out", "current_out_min_ua",
        "current_out_max_ua", "board", "safe_mode", "safe_reason", "reset_cause", "last_crash",
        "boot_id", "unclean_resets",
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
    CHECK(strstr(f, "\"last_crash\":\"none\"}") != NULL);
    CHECK(strstr(f, "\"boot_id\":\"0000002a\",\"unclean_resets\":0,") != NULL);
    CHECK(strstr(f, "boot_health_cut") == NULL);
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
    c.current_out_min_ua = c.current_out_max_ua = LONG_MIN;
    c.board = "stm32h563-digital-v3r2";
    c.safe_mode = true;
    c.reset_cause = "independent-watchdog";
    c.boot_id = 0xFFFFFFFFu;
    c.unclean_resets = 0xFFFFFFFFu;
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
    CHECK(strstr(f, "\"safe_mode\":true") != NULL);
    CHECK(strstr(f, "\"reset_cause\":\"independent-watchdog\"") != NULL);
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
}

static void test_too_small_buffer(void) {
    cloud_caps_t c = typical();
    char f[200];
    CHECK(cloud_caps_build(&c, f, sizeof(f)) == 0);
}

/* The status reply's caps[] from the same values, in the order status always listed them. */
static void test_status_list(void) {
    char buf[1024];
    bp_emit_t e;
    cloud_caps_t c = typical();
    c.usb_cc = true;
    bp_emit_init(&e, buf, sizeof(buf));
    cloud_caps_emit_list(&e, &c);
    CHECK(bp_emit_ok(&e));
    CHECK(strcmp(buf, ",\"caps\":[\"signal\",\"gpio\",\"power\",\"swd\",\"i2c_sensor\",\"uart\",\"la\","
                      "\"analyzer\",\"command\",\"tunnel\",\"ota\",\"la_pins\",\"power_profile\","
                      "\"capture_b64\",\"can\",\"pod_current\",\"analog\",\"scope\",\"dac_limits\","
                      "\"calibrate\",\"current_out\",\"dac\",\"dac_dc\",\"dac_replay\",\"dac_deep_replay\","
                      "\"dac_control_loop\",\"dac_cotrig\",\"dac_loop_sources\",\"dac_loop_input_map\","
                      "\"gpio_read\",\"capture_trigger\",\"spi_master\",\"spi_stream\",\"nrst_pin\","
                      "\"usb_cc\"]") == 0);

    /* The digital board: cloud_caps_collect clears every analog feature, so the list names none. */
    cloud_caps_t d = typical();
    d.analog = d.dac_ac = d.dac_dc = d.dac_replay = d.deep_replay = false;
    d.control_loop = d.cotrig = d.loop_sources = d.loop_input_map = false;
    d.pod_current = d.nrst_pin = d.usb_cc = false;
    bp_emit_init(&e, buf, sizeof(buf));
    cloud_caps_emit_list(&e, &d);
    CHECK(strcmp(buf, ",\"caps\":[\"signal\",\"gpio\",\"power\",\"swd\",\"i2c_sensor\",\"uart\",\"la\","
                      "\"analyzer\",\"command\",\"tunnel\",\"ota\",\"la_pins\",\"power_profile\","
                      "\"capture_b64\",\"can\",\"gpio_read\",\"capture_trigger\",\"spi_master\","
                      "\"spi_stream\"]") == 0);
    if (strstr(buf, "analog")) printf("  %s\n", buf);

    /* The frame does not carry usb_cc (the server never read it): adding the field changed nothing. */
    static char f[CLOUD_CAPS_MAX + 1];
    CHECK(cloud_caps_build(&c, f, sizeof(f)) > 0 && strstr(f, "usb_cc") == NULL);
}

int main(void) {
    test_typical_frame();
    test_status_list();
    test_worst_case_fits();
    test_too_small_buffer();
    if (failures) { printf("test_cloud_caps: %d FAILED\n", failures); return 1; }
    printf("test_cloud_caps: all passed\n");
    return 0;
}
