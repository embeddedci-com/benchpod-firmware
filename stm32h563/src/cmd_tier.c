/*
 * cmd_tier.c — the command table (cmd_table.h) and the tier rules (cmd_tier.h).
 *
 * The file keeps its name because embeddedci-server's shared-vector test reads k_tiers out of it.
 */
#include "cmd_table.h"
#include "cmd_tier.h"
#include "bp_json.h"

#include <string.h>

#ifdef CMD_TABLE_NO_HANDLERS
#define CMD_FN(fn) NULL
#else
#include "command_handler_internal.h"
#define CMD_FN(fn) fn
#endif

/* One row per JSON verb: { { verb, tier }, CMD_F_* flags, handler }. A verb missing here is
   "unknown cmd" and counts as T3. */
static const cmd_desc_t k_tiers[] = {
    /* T0: read-only */
    { { "ping", CMD_TIER_T0 }, CMD_F_NO_HW | CMD_F_NOISY, CMD_FN(handle_ping) },
    { { "status", CMD_TIER_T0 }, CMD_F_NO_HW, CMD_FN(handle_status) },
    { { "cloud_status", CMD_TIER_T0 }, CMD_F_NO_HW, CMD_FN(handle_cloud_status) },
    { { "wifi_status", CMD_TIER_T0 }, CMD_F_NO_HW, CMD_FN(handle_wifi_status) },
    { { "la_pins", CMD_TIER_T0 }, 0, CMD_FN(handle_la_pins) },
    { { "usb_cc", CMD_TIER_T0 }, CMD_F_NO_HW, CMD_FN(handle_usb_cc) },
    { { "target_status", CMD_TIER_T0 }, CMD_F_NO_HW | CMD_F_NOISY, CMD_FN(handle_target_status) },
    { { "power_status", CMD_TIER_T0 }, CMD_F_NO_HW | CMD_F_NOISY, CMD_FN(handle_power_status) },
    { { "identity_public", CMD_TIER_T0 }, CMD_F_NO_HW, CMD_FN(handle_identity_public) },
    { { "identity_pop", CMD_TIER_T0 }, CMD_F_NO_HW, CMD_FN(handle_identity_pop) },
    { { "identity_wipe", CMD_TIER_T0 }, CMD_F_NO_HW, CMD_FN(handle_identity_wipe) },   /* always refused off the USB console: it changes nothing */
    { { "spi_status", CMD_TIER_T0 }, 0, CMD_FN(handle_spi_status) },
    { { "sensor_status", CMD_TIER_T0 }, 0, CMD_FN(handle_sensor_status) },
    { { "can_status", CMD_TIER_T0 }, CMD_F_NO_HW, CMD_FN(handle_can_status) },
    { { "ota_status", CMD_TIER_T0 }, 0, CMD_FN(handle_ota_status) },
    { { "blob_status", CMD_TIER_T0 }, 0, CMD_FN(handle_blob_status) },
    { { "dac_loop_probe", CMD_TIER_T0 }, CMD_F_ANALOG, CMD_FN(handle_dac_loop_probe) },
    /* T0, observational: they read the DUT */
    { { "capture", CMD_TIER_T0 }, CMD_F_ANALOG | CMD_F_STREAM | CMD_F_HEAVY, CMD_FN(handle_capture) },
    { { "capture_dual", CMD_TIER_T0 }, CMD_F_STREAM | CMD_F_HEAVY, CMD_FN(handle_capture_dual) },
    { { "capture_read", CMD_TIER_T0 }, CMD_F_STREAM | CMD_F_HEAVY, CMD_FN(handle_capture_read) },
    { { "stream", CMD_TIER_T0 }, CMD_F_ANALOG | CMD_F_STREAM | CMD_F_HEAVY, CMD_FN(handle_stream) },
    { { "la_capture", CMD_TIER_T0 }, CMD_F_STREAM | CMD_F_HEAVY, CMD_FN(handle_la_capture) },
    { { "sensor_regs", CMD_TIER_T0 }, CMD_F_STREAM | CMD_F_HEAVY, CMD_FN(handle_sensor_regs) },
    { { "sensor_la", CMD_TIER_T0 }, CMD_F_STREAM | CMD_F_HEAVY, CMD_FN(handle_sensor_la) },
    { { "can_read", CMD_TIER_T0 }, CMD_F_NO_HW | CMD_F_HEAVY, CMD_FN(handle_can_read) },
    { { "psram_ping", CMD_TIER_T0 }, CMD_F_HEAVY, CMD_FN(handle_psram_ping) },
    { { "test", CMD_TIER_T0 }, CMD_F_STREAM | CMD_F_HEAVY, CMD_FN(handle_test) },
    /* mixed: read form T0, write form T2 (see mixed_write) */
    { { "dac_limits", CMD_TIER_T0 }, CMD_F_NO_HW | CMD_F_ANALOG | CMD_F_MIXED, CMD_FN(handle_dac_limits) },
    { { "calibrate", CMD_TIER_T0 }, CMD_F_ANALOG | CMD_F_MIXED, CMD_FN(handle_calibrate) },
    { { "eth", CMD_TIER_T0 }, CMD_F_NO_HW | CMD_F_MIXED, CMD_FN(handle_eth) },
    { { "sig_policy", CMD_TIER_T0 }, CMD_F_MIXED, CMD_FN(handle_sig_policy) },
    { { "lan_policy", CMD_TIER_T0 }, CMD_F_MIXED, CMD_FN(handle_lan_policy) },
    { { "cloud_ca", CMD_TIER_T0 }, CMD_F_MIXED, CMD_FN(handle_cloud_ca) },
    { { "cloud_proxy", CMD_TIER_T0 }, CMD_F_MIXED, CMD_FN(handle_cloud_proxy) },
    /* T1: instrument control */
    { { "generate", CMD_TIER_T1 }, CMD_F_ANALOG, CMD_FN(handle_generate) },
    { { "measure", CMD_TIER_T1 }, CMD_F_ANALOG | CMD_F_STREAM, CMD_FN(handle_measure) },
    { { "load", CMD_TIER_T1 }, CMD_F_ANALOG | CMD_F_STREAM, CMD_FN(handle_load) },
    { { "load_bin", CMD_TIER_T1 }, CMD_F_ANALOG | CMD_F_STREAM, CMD_FN(handle_load_bin) },
    { { "replay", CMD_TIER_T1 }, CMD_F_ANALOG | CMD_F_STREAM, CMD_FN(handle_replay) },
    { { "dac_stop", CMD_TIER_T1 }, 0, CMD_FN(handle_dac_stop) },
    { { "dac_set", CMD_TIER_T1 }, CMD_F_ANALOG, CMD_FN(handle_dac_set) },
    { { "dac_mux", CMD_TIER_T1 }, CMD_F_ANALOG, CMD_FN(handle_dac_mux) },
    { { "cal_switch", CMD_TIER_T1 }, CMD_F_ANALOG, CMD_FN(handle_cal_switch) },
    { { "analog_path", CMD_TIER_T1 }, CMD_F_ANALOG, CMD_FN(handle_analog_path) },
    { { "dac_out", CMD_TIER_T1 }, CMD_F_ANALOG, CMD_FN(handle_dac_out) },
    { { "current_out", CMD_TIER_T1 }, CMD_F_ANALOG, CMD_FN(handle_current_out) },
    { { "adc_read", CMD_TIER_T1 }, CMD_F_ANALOG, CMD_FN(handle_adc_read) },
    { { "dac_control_loop", CMD_TIER_T1 }, CMD_F_ANALOG, CMD_FN(handle_dac_control_loop) },
    { { "dac_loop_input", CMD_TIER_T1 }, CMD_F_ANALOG, CMD_FN(handle_dac_loop_input) },
    { { "la", CMD_TIER_T1 }, 0, CMD_FN(handle_la) },
    { { "gpio", CMD_TIER_T1 }, 0, CMD_FN(handle_gpio) },
    { { "la_voltage", CMD_TIER_T1 }, CMD_F_NO_HW, CMD_FN(handle_la_voltage) },
    { { "nrst", CMD_TIER_T1 }, CMD_F_NO_HW, CMD_FN(handle_nrst) },
    { { "target_power", CMD_TIER_T1 }, CMD_F_NO_HW, CMD_FN(handle_target_power) },
    { { "power_profile", CMD_TIER_T1 }, CMD_F_NO_HW, CMD_FN(handle_power_profile) },
    { { "dap_start", CMD_TIER_T1 }, CMD_F_STREAM, CMD_FN(handle_dap_start) },
    { { "uart_proxy_start", CMD_TIER_T1 }, CMD_F_STREAM, CMD_FN(handle_uart_proxy_start) },
    { { "spi_start", CMD_TIER_T1 }, 0, CMD_FN(handle_spi_start) },
    { { "spi_stop", CMD_TIER_T1 }, 0, CMD_FN(handle_spi_stop) },
    { { "spi_xfer", CMD_TIER_T1 }, 0, CMD_FN(handle_spi_xfer) },
    { { "spi_stream", CMD_TIER_T1 }, 0, CMD_FN(handle_spi_stream) },
    { { "spi_flash", CMD_TIER_T1 }, 0, CMD_FN(handle_spi_flash) },
    { { "sensor_start", CMD_TIER_T1 }, 0, CMD_FN(handle_sensor_start) },
    { { "sensor_set", CMD_TIER_T1 }, 0, CMD_FN(handle_sensor_set) },
    { { "sensor_stop", CMD_TIER_T1 }, 0, CMD_FN(handle_sensor_stop) },
    { { "can_config", CMD_TIER_T1 }, CMD_F_NO_HW, CMD_FN(handle_can_config) },
    { { "can_write", CMD_TIER_T1 }, CMD_F_NO_HW, CMD_FN(handle_can_write) },
    { { "can_term", CMD_TIER_T1 }, CMD_F_NO_HW, CMD_FN(handle_can_term) },
    { { "can_respond", CMD_TIER_T1 }, CMD_F_NO_HW, CMD_FN(handle_can_respond) },
    { { "can_disable", CMD_TIER_T1 }, CMD_F_NO_HW, CMD_FN(handle_can_disable) },
    { { "speedtest", CMD_TIER_T1 }, CMD_F_NO_HW | CMD_F_STREAM, CMD_FN(handle_speedtest) },
    { { "fpga_image", CMD_TIER_T1 }, 0, CMD_FN(handle_fpga_image) },
    { { "psram_recover", CMD_TIER_T1 }, 0, CMD_FN(handle_psram_recover) },
    /* T2: persisted config */
    { { "cloud_set", CMD_TIER_T2 }, CMD_F_NO_HW, CMD_FN(handle_cloud_set) },
    { { "cloud_clear", CMD_TIER_T2 }, CMD_F_NO_HW, CMD_FN(handle_cloud_clear) },
    { { "wifi_set", CMD_TIER_T2 }, CMD_F_NO_HW, CMD_FN(handle_wifi_set) },
    { { "wifi_clear", CMD_TIER_T2 }, CMD_F_NO_HW, CMD_FN(handle_wifi_clear) },
    /* T3: firmware */
    { { "ota_begin", CMD_TIER_T3 }, 0, CMD_FN(handle_ota_begin) },
    { { "ota_data", CMD_TIER_T3 }, 0, CMD_FN(handle_ota_data) },
    { { "ota_end", CMD_TIER_T3 }, 0, CMD_FN(handle_ota_end) },
    { { "ota_commit", CMD_TIER_T3 }, 0, CMD_FN(handle_ota_commit) },
    { { "ota_abort", CMD_TIER_T3 }, 0, CMD_FN(handle_ota_abort) },
    { { "ota_selftest", CMD_TIER_T3 }, 0, CMD_FN(handle_ota_selftest) },
};

const cmd_desc_t *cmd_find(const char *cmd) {
    if (!cmd) return NULL;
    for (size_t i = 0; i < sizeof(k_tiers) / sizeof(k_tiers[0]); i++)
        if (strcmp(cmd, k_tiers[i].key.name) == 0) return &k_tiers[i];
    return NULL;
}

const cmd_desc_t *cmd_table_rows(size_t *count) {
    if (count) *count = sizeof(k_tiers) / sizeof(k_tiers[0]);
    return k_tiers;
}

/* The write forms of the mixed verbs (the handlers' own tests). */
static int mixed_write(const char *cmd, const char *json) {
    char v[16] = {0};
    if (!json) return 0;
    if (strcmp(cmd, "dac_limits") == 0)
        return bp_json_get(json, "path", v, sizeof(v)) ||
               (bp_json_get(json, "enabled", v, sizeof(v)) && strcmp(v, "false") == 0);
    if (strcmp(cmd, "calibrate") == 0)
        return bp_json_get(json, "source", v, sizeof(v)) ||
               (bp_json_get(json, "clear", v, sizeof(v)) && strcmp(v, "true") == 0);
    if (strcmp(cmd, "sig_policy") == 0 || strcmp(cmd, "lan_policy") == 0)
        return bp_json_get(json, "set", v, sizeof(v));
    if (strcmp(cmd, "cloud_ca") == 0 || strcmp(cmd, "cloud_proxy") == 0)
        return bp_json_get(json, "set", v, sizeof(v)) ||
               (bp_json_get(json, "clear", v, sizeof(v)) && strcmp(v, "true") == 0);
    if (strcmp(cmd, "eth") == 0) {
        bp_json_get(json, "action", v, sizeof(v));
        return strcmp(v, "stats") != 0 && strcmp(v, "refclk") != 0;
    }
    return 0;
}

cmd_tier_t cmd_tier_of(const cmd_desc_t *d, const char *json) {
    if (!d) return CMD_TIER_T3;
    return (d->flags & CMD_F_MIXED) && mixed_write(d->key.name, json) ? CMD_TIER_T2
                                                                        : (cmd_tier_t)d->key.tier;
}

cmd_tier_t cmd_tier(const char *cmd, const char *json) { return cmd_tier_of(cmd_find(cmd), json); }

int cmd_tier_known(const char *cmd) { return cmd_find(cmd) != NULL; }

int cmd_tier_light(const char *cmd, const char *json) {
    const cmd_desc_t *d = cmd_find(cmd);
    return d && !(d->flags & CMD_F_HEAVY) && cmd_tier_of(d, json) == CMD_TIER_T0;
}

const char *cmd_tier_name(cmd_tier_t t) {
    static const char *const names[] = { "T0", "T1", "T2", "T3" };
    return (unsigned)t < 4u ? names[t] : "T?";
}
