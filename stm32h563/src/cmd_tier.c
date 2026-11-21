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

/* The write forms of the mixed verbs (the handlers' own tests), one per row below. */
static int writes_dac_limits(const char *json) {
    char v[16] = {0};
    return bp_json_get(json, "path", v, sizeof(v)) ||
           (bp_json_get(json, "enabled", v, sizeof(v)) && strcmp(v, "false") == 0);
}
/* run, store or clear; a bare one (or clear:false) reads */
static int writes_calibrate(const char *json) {
    char v[16] = {0};
    return bp_json_get(json, "source", v, sizeof(v)) || bp_json_get(json, "dac", v, sizeof(v)) ||
           bp_json_get(json, "adc", v, sizeof(v)) || bp_json_get(json, "comp_hf_dc_e4", v, sizeof(v)) ||
           (bp_json_get(json, "clear", v, sizeof(v)) && strcmp(v, "false") != 0);
}
/* sig_policy also writes with clear_floor */
static int writes_sig_policy(const char *json) {
    char v[16] = {0};
    return bp_json_get(json, "set", v, sizeof(v)) || (bp_json_get(json, "clear_floor", v, sizeof(v)) && strcmp(v, "false") != 0);
}
/* lan_policy (set, pairing) */
static int writes_set(const char *json) {
    char v[16] = {0};
    return bp_json_get(json, "set", v, sizeof(v)) || bp_json_get(json, "pairing", v, sizeof(v));
}
/* usb_network, usb_host */
static int writes_enabled(const char *json) {
    char v[16] = {0};
    return bp_json_get(json, "enabled", v, sizeof(v));
}
/* cloud_ca, cloud_proxy */
static int writes_set_or_clear(const char *json) {
    char v[16] = {0};
    return bp_json_get(json, "set", v, sizeof(v)) ||
           (bp_json_get(json, "clear", v, sizeof(v)) && strcmp(v, "true") == 0);
}
static int writes_lan_tls(const char *json) {
    char v[16] = {0};
    return bp_json_get(json, "https_only", v, sizeof(v)) || bp_json_get(json, "scpi", v, sizeof(v)) ||
           (bp_json_get(json, "regenerate", v, sizeof(v)) && strcmp(v, "true") == 0);
}
/* add, remove or clear; a bare one lists */
static int writes_lan_clients(const char *json) {
    char v[80] = {0};
    return bp_json_get(json, "add", v, sizeof(v)) || bp_json_get(json, "remove", v, sizeof(v)) ||
           (bp_json_get(json, "clear", v, sizeof(v)) && strcmp(v, "false") != 0);
}
/* an EEPROM write (ID or calibration); a read is T0 */
static int writes_module(const char *json) {
    char v[16] = {0};
    return bp_json_get(json, "eeprom", v, sizeof(v)) && strcmp(v, "read") != 0;
}
/* only "stats" and "refclk" read */
static int writes_eth(const char *json) {
    char v[16] = {0};
    bp_json_get(json, "action", v, sizeof(v));
    return strcmp(v, "stats") != 0 && strcmp(v, "refclk") != 0;
}

/* One row per JSON verb: { { verb, tier }, CMD_F_* flags, handler, write test }. A verb missing
   here is "unknown cmd" and counts as T3.  The write test is NULL except on the mixed verbs. */
static const cmd_desc_t k_tiers[] = {
    /* T0: read-only */
    { { "ping", CMD_TIER_T0 }, CMD_F_NO_HW | CMD_F_NOISY | CMD_F_ANY_CLIENT | CMD_F_UNPAIRED, CMD_FN(handle_ping), NULL },
    { { "status", CMD_TIER_T0 }, CMD_F_NO_HW | CMD_F_ANY_CLIENT | CMD_F_UNPAIRED, CMD_FN(handle_status), NULL },
    { { "diag", CMD_TIER_T0 }, CMD_F_NO_HW | CMD_F_ANY_CLIENT | CMD_F_UNPAIRED, CMD_FN(handle_diag), NULL },
    { { "cloud_status", CMD_TIER_T0 }, CMD_F_NO_HW | CMD_F_UNPAIRED, CMD_FN(handle_cloud_status), NULL },
    { { "wifi_status", CMD_TIER_T0 }, CMD_F_NO_HW, CMD_FN(handle_wifi_status), NULL },
    { { "wifi_scan", CMD_TIER_T0 }, CMD_F_NO_HW, CMD_FN(handle_wifi_scan), NULL },
    { { "la_pins", CMD_TIER_T0 }, 0, CMD_FN(handle_la_pins), NULL },
    { { "usb_cc", CMD_TIER_T0 }, CMD_F_NO_HW, CMD_FN(handle_usb_cc), NULL },
    { { "usb_host", CMD_TIER_T0 }, 0, CMD_FN(handle_usb_host), writes_enabled },
    { { "target_status", CMD_TIER_T0 }, CMD_F_NO_HW | CMD_F_NOISY, CMD_FN(handle_target_status), NULL },
    { { "power_status", CMD_TIER_T0 }, CMD_F_NO_HW | CMD_F_NOISY, CMD_FN(handle_power_status), NULL },
    { { "identity_public", CMD_TIER_T0 }, CMD_F_NO_HW | CMD_F_ANY_CLIENT | CMD_F_UNPAIRED, CMD_FN(handle_identity_public), NULL },
    { { "identity_pop", CMD_TIER_T0 }, CMD_F_NO_HW, CMD_FN(handle_identity_pop), NULL },
    { { "spi_status", CMD_TIER_T0 }, 0, CMD_FN(handle_spi_status), NULL },
    { { "i2c_status", CMD_TIER_T0 }, 0, CMD_FN(handle_i2c_status), NULL },
    { { "sensor_status", CMD_TIER_T0 }, 0, CMD_FN(handle_sensor), NULL },
    { { "can_status", CMD_TIER_T0 }, CMD_F_NO_HW, CMD_FN(handle_can_status), NULL },
    { { "ota_status", CMD_TIER_T0 }, CMD_F_ANY_CLIENT, CMD_FN(handle_ota_status), NULL },
    { { "blob_status", CMD_TIER_T0 }, CMD_F_ANY_CLIENT, CMD_FN(handle_blob_status), NULL },
    { { "wiring_list", CMD_TIER_T0 }, CMD_F_NO_HW, CMD_FN(handle_wiring_list), NULL },
    { { "wiring_get", CMD_TIER_T0 }, CMD_F_NO_HW, CMD_FN(handle_wiring_get), NULL },
#if defined(BENCHPOD_EMI_TEST)
    /* EMI test build only (emi_test.h): read the run log out of the W25Q */
    { { "emi_log", CMD_TIER_T0 }, 0, CMD_FN(handle_emi_log), NULL },
#endif
    { { "dac_loop_probe", CMD_TIER_T0 }, CMD_F_ANALOG, CMD_FN(handle_dac_loop_probe), NULL },
    /* T0, observational: they read the DUT */
    { { "capture", CMD_TIER_T0 }, CMD_F_ANALOG | CMD_F_STREAM | CMD_F_HEAVY, CMD_FN(handle_capture), NULL },
    { { "capture_dual", CMD_TIER_T0 }, CMD_F_ADC_SAMPLES | CMD_F_STREAM | CMD_F_HEAVY, CMD_FN(handle_capture_dual), NULL },
    { { "capture_read", CMD_TIER_T0 }, CMD_F_STREAM | CMD_F_HEAVY, CMD_FN(handle_capture_read), NULL },
    { { "la_capture", CMD_TIER_T0 }, CMD_F_STREAM | CMD_F_HEAVY, CMD_FN(handle_la_capture), NULL },
    { { "sensor_regs", CMD_TIER_T0 }, CMD_F_HEAVY, CMD_FN(handle_sensor), NULL },
    { { "sensor_la", CMD_TIER_T0 }, CMD_F_STREAM | CMD_F_HEAVY, CMD_FN(handle_sensor_la), NULL },
    { { "can_read", CMD_TIER_T0 }, CMD_F_NO_HW | CMD_F_HEAVY, CMD_FN(handle_can_read), NULL },
    { { "psram_ping", CMD_TIER_T0 }, CMD_F_HEAVY, CMD_FN(handle_psram_ping), NULL },
    { { "test", CMD_TIER_T0 }, CMD_F_STREAM | CMD_F_HEAVY, CMD_FN(handle_test), NULL },
    /* mixed: read form T0, write form T2 (the writes_* test) */
    { { "dac_limits", CMD_TIER_T0 }, CMD_F_NO_HW | CMD_F_ANALOG, CMD_FN(handle_dac_limits), writes_dac_limits },
    { { "calibrate", CMD_TIER_T0 }, CMD_F_ANALOG, CMD_FN(handle_calibrate), writes_calibrate },
    { { "eth", CMD_TIER_T0 }, CMD_F_NO_HW, CMD_FN(handle_eth), writes_eth },
    { { "sig_policy", CMD_TIER_T0 }, 0, CMD_FN(handle_sig_policy), writes_sig_policy },
    { { "lan_policy", CMD_TIER_T0 }, CMD_F_ADMIN, CMD_FN(handle_lan_policy), writes_set },
    { { "usb_network", CMD_TIER_T0 }, 0, CMD_FN(handle_usb_network), writes_enabled },
    /* read and set T0: it only changes what the pod prints on its own console */
    { { "log_level", CMD_TIER_T0 }, CMD_F_NO_HW, CMD_FN(handle_log_level), NULL },
    { { "module", CMD_TIER_T0 }, CMD_F_NO_HW, CMD_FN(handle_module), writes_module },
    { { "cloud_ca", CMD_TIER_T0 }, 0, CMD_FN(handle_cloud_ca), writes_set_or_clear },
    { { "cloud_proxy", CMD_TIER_T0 }, 0, CMD_FN(handle_cloud_proxy), writes_set_or_clear },
    { { "lan_tls", CMD_TIER_T0 }, CMD_F_UNPAIRED | CMD_F_ADMIN, CMD_FN(handle_lan_tls), writes_lan_tls },
    { { "lan_clients", CMD_TIER_T0 }, CMD_F_NO_HW | CMD_F_ADMIN, CMD_FN(handle_lan_clients), writes_lan_clients },
    /* T1: instrument control */
    { { "generate", CMD_TIER_T1 }, CMD_F_ANALOG, CMD_FN(handle_generate), NULL },
    { { "measure", CMD_TIER_T1 }, CMD_F_ANALOG | CMD_F_STREAM, CMD_FN(handle_measure), NULL },
    { { "load", CMD_TIER_T1 }, CMD_F_ANALOG | CMD_F_STREAM, CMD_FN(handle_load), NULL },
    { { "load_bin", CMD_TIER_T1 }, CMD_F_ANALOG | CMD_F_STREAM, CMD_FN(handle_load_bin), NULL },
    { { "replay", CMD_TIER_T1 }, CMD_F_ANALOG | CMD_F_STREAM, CMD_FN(handle_replay), NULL },
    { { "dac_stop", CMD_TIER_T1 }, 0, CMD_FN(handle_dac_stop), NULL },
    { { "dac_set", CMD_TIER_T1 }, CMD_F_ANALOG, CMD_FN(handle_dac_set), NULL },
    { { "dac_mux", CMD_TIER_T1 }, CMD_F_ANALOG, CMD_FN(handle_dac_mux), NULL },
    { { "cal_switch", CMD_TIER_T1 }, CMD_F_ANALOG, CMD_FN(handle_cal_switch), NULL },
    { { "analog_path", CMD_TIER_T1 }, CMD_F_ANALOG, CMD_FN(handle_analog_path), NULL },
    { { "dac_out", CMD_TIER_T1 }, CMD_F_ANALOG, CMD_FN(handle_dac_out), NULL },
    { { "current_out", CMD_TIER_T1 }, CMD_F_ANALOG, CMD_FN(handle_current_out), NULL },
    { { "adc_read", CMD_TIER_T1 }, CMD_F_ANALOG, CMD_FN(handle_adc_read), NULL },
    { { "adc_range", CMD_TIER_T1 }, CMD_F_ANALOG, CMD_FN(handle_adc_range), NULL },
    { { "dac_control_loop", CMD_TIER_T1 }, CMD_F_ANALOG, CMD_FN(handle_dac_control_loop), NULL },
    { { "dac_loop_input", CMD_TIER_T1 }, CMD_F_ANALOG, CMD_FN(handle_dac_loop_input), NULL },
    { { "la_step", CMD_TIER_T1 }, 0, CMD_FN(handle_la_step_cmd), NULL },
    { { "la_pull", CMD_TIER_T1 }, 0, CMD_FN(handle_la_pull_cmd), NULL },
    { { "gpio", CMD_TIER_T1 }, 0, CMD_FN(handle_gpio), NULL },
    { { "la_voltage", CMD_TIER_T1 }, CMD_F_NO_HW, CMD_FN(handle_la_voltage), NULL },
    { { "nrst", CMD_TIER_T1 }, CMD_F_NO_HW, CMD_FN(handle_nrst), NULL },
    { { "target_power", CMD_TIER_T1 }, CMD_F_NO_HW, CMD_FN(handle_target_power), NULL },
    { { "wiring_put", CMD_TIER_T1 }, 0, CMD_FN(handle_wiring_put), NULL },
    { { "wiring_delete", CMD_TIER_T1 }, 0, CMD_FN(handle_wiring_delete), NULL },
    { { "wiring_default", CMD_TIER_T1 }, 0, CMD_FN(handle_wiring_default), NULL },
    { { "module_power", CMD_TIER_T1 }, 0, CMD_FN(handle_module_power), NULL },
    { { "power_profile", CMD_TIER_T1 }, CMD_F_NO_HW, CMD_FN(handle_power_profile), NULL },
    { { "dap_start", CMD_TIER_T1 }, CMD_F_STREAM, CMD_FN(handle_dap_start), NULL },
    { { "uart_proxy_start", CMD_TIER_T1 }, CMD_F_STREAM, CMD_FN(handle_uart_proxy_start), NULL },
    { { "spi_start", CMD_TIER_T1 }, 0, CMD_FN(handle_spi_start), NULL },
    { { "spi_stop", CMD_TIER_T1 }, 0, CMD_FN(handle_spi_stop), NULL },
    { { "spi_xfer", CMD_TIER_T1 }, 0, CMD_FN(handle_spi_xfer), NULL },
    { { "spi_stream", CMD_TIER_T1 }, 0, CMD_FN(handle_spi_stream), NULL },
    { { "spi_flash", CMD_TIER_T1 }, 0, CMD_FN(handle_spi_flash), NULL },
    { { "esp_flash", CMD_TIER_T1 }, 0, CMD_FN(handle_esp_flash), NULL },
    { { "la_dir", CMD_TIER_T1 }, 0, CMD_FN(handle_la_dir), NULL },
    { { "lease_acquire", CMD_TIER_T1 }, CMD_F_NO_HW | CMD_F_LEASE, CMD_FN(handle_lease_acquire), NULL },
    { { "lease_renew", CMD_TIER_T1 }, CMD_F_NO_HW | CMD_F_LEASE, CMD_FN(handle_lease_renew), NULL },
    { { "lease_release", CMD_TIER_T1 }, CMD_F_NO_HW | CMD_F_LEASE | CMD_F_ANY_CLIENT, CMD_FN(handle_lease_release), NULL },
    { { "i2c_start", CMD_TIER_T1 }, 0, CMD_FN(handle_i2c_start), NULL },
    { { "i2c_stop", CMD_TIER_T1 }, 0, CMD_FN(handle_i2c_stop), NULL },
    { { "i2c_xfer", CMD_TIER_T1 }, 0, CMD_FN(handle_i2c_xfer), NULL },
    { { "i2c_scan", CMD_TIER_T1 }, 0, CMD_FN(handle_i2c_scan), NULL },
    { { "sensor_define", CMD_TIER_T1 }, 0, CMD_FN(handle_sensor), NULL },
    { { "sensor_start", CMD_TIER_T1 }, 0, CMD_FN(handle_sensor), NULL },
    { { "sensor_set", CMD_TIER_T1 }, 0, CMD_FN(handle_sensor), NULL },
    { { "sensor_stimulus", CMD_TIER_T1 }, 0, CMD_FN(handle_sensor), NULL },
    { { "sensor_stimulus_start", CMD_TIER_T1 }, 0, CMD_FN(handle_sensor), NULL },
    { { "sensor_stimulus_stop", CMD_TIER_T1 }, 0, CMD_FN(handle_sensor), NULL },
    { { "sensor_fault", CMD_TIER_T1 }, 0, CMD_FN(handle_sensor), NULL },
    { { "sensor_fault_clear", CMD_TIER_T1 }, 0, CMD_FN(handle_sensor), NULL },
    { { "sensor_stop", CMD_TIER_T1 }, 0, CMD_FN(handle_sensor), NULL },
    { { "can_start", CMD_TIER_T1 }, CMD_F_NO_HW, CMD_FN(handle_can_start), NULL },
    { { "can_write", CMD_TIER_T1 }, CMD_F_NO_HW, CMD_FN(handle_can_write), NULL },
    { { "can_term", CMD_TIER_T1 }, CMD_F_NO_HW, CMD_FN(handle_can_term), NULL },
    { { "can_respond", CMD_TIER_T1 }, CMD_F_NO_HW, CMD_FN(handle_can_respond), NULL },
    { { "can_stop", CMD_TIER_T1 }, CMD_F_NO_HW, CMD_FN(handle_can_stop), NULL },
    { { "speedtest", CMD_TIER_T1 }, CMD_F_NO_HW | CMD_F_STREAM, CMD_FN(handle_speedtest), NULL },
    { { "fpga_image", CMD_TIER_T1 }, 0, CMD_FN(handle_fpga_image), NULL },
    { { "psram_recover", CMD_TIER_T1 }, 0, CMD_FN(handle_psram_recover), NULL },
    { { "usb_reattach", CMD_TIER_T1 }, CMD_F_NO_HW, CMD_FN(handle_usb_reattach), NULL },
    /* T2: persisted config */
    { { "cloud_set", CMD_TIER_T2 }, CMD_F_NO_HW, CMD_FN(handle_cloud_set), NULL },
    { { "cloud_clear", CMD_TIER_T2 }, CMD_F_NO_HW, CMD_FN(handle_cloud_clear), NULL },
    { { "wifi_set", CMD_TIER_T2 }, CMD_F_NO_HW, CMD_FN(handle_wifi_set), NULL },
    { { "wifi_clear", CMD_TIER_T2 }, CMD_F_NO_HW, CMD_FN(handle_wifi_clear), NULL },
    /* the USB console's recovery commands up to 3.10.13 (command_handler_recovery.c) */
    { { "identity_wipe", CMD_TIER_T2 }, CMD_F_NO_HW, CMD_FN(handle_identity_wipe), NULL },   /* USB link only */
    { { "wifi_bridge", CMD_TIER_T2 }, CMD_F_NO_HW, CMD_FN(handle_wifi_bridge), NULL },     /* cert: USB link only */
    /* T3: firmware */
    { { "ota_begin", CMD_TIER_T3 }, CMD_F_ANY_CLIENT, CMD_FN(handle_ota_begin), NULL },
    { { "ota_data", CMD_TIER_T3 }, CMD_F_ANY_CLIENT, CMD_FN(handle_ota_data), NULL },
    { { "ota_end", CMD_TIER_T3 }, CMD_F_ANY_CLIENT, CMD_FN(handle_ota_end), NULL },
    { { "ota_commit", CMD_TIER_T3 }, CMD_F_ANY_CLIENT, CMD_FN(handle_ota_commit), NULL },
    { { "ota_abort", CMD_TIER_T3 }, CMD_F_ANY_CLIENT, CMD_FN(handle_ota_abort), NULL },
    { { "ota_selftest", CMD_TIER_T3 }, CMD_F_ANY_CLIENT, CMD_FN(handle_ota_selftest), NULL },
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

cmd_tier_t cmd_tier_of(const cmd_desc_t *d, const char *json) {
    if (!d) return CMD_TIER_T3;
    return d->writes && json && d->writes(json) ? CMD_TIER_T2 : (cmd_tier_t)d->key.tier;
}

cmd_tier_t cmd_tier(const char *cmd, const char *json) { return cmd_tier_of(cmd_find(cmd), json); }

int cmd_tier_known(const char *cmd) { return cmd_find(cmd) != NULL; }

int cmd_tier_light_of(const cmd_desc_t *d, const char *json) {
    return d && !(d->flags & CMD_F_HEAVY) && cmd_tier_of(d, json) == CMD_TIER_T0;
}

int cmd_tier_light(const char *cmd, const char *json) { return cmd_tier_light_of(cmd_find(cmd), json); }

int cmd_passes_lease_of(const cmd_desc_t *d, const char *json) {
    return cmd_tier_light_of(d, json) || cmd_has(d, CMD_F_ADMIN);
}

int cmd_passes_lease(const char *cmd, const char *json) { return cmd_passes_lease_of(cmd_find(cmd), json); }

const char *cmd_tier_name(cmd_tier_t t) {
    static const char *const names[] = { "T0", "T1", "T2", "T3" };
    return (unsigned)t < 4u ? names[t] : "T?";
}
