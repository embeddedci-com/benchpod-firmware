/*
 * cmd_tier.c — the command tier table (see cmd_tier.h).
 */
#include "cmd_tier.h"
#include "bp_json.h"

#include <string.h>

typedef struct {
    const char *cmd;
    cmd_tier_t  tier;   /* for the mixed verbs: the tier of their read form */
} tier_entry_t;

static const tier_entry_t k_tiers[] = {
    /* T0: read-only */
    { "ping", CMD_TIER_T0 }, { "status", CMD_TIER_T0 }, { "cloud_status", CMD_TIER_T0 },
    { "wifi_status", CMD_TIER_T0 }, { "la_pins", CMD_TIER_T0 }, { "usb_cc", CMD_TIER_T0 },
    { "target_status", CMD_TIER_T0 }, { "power_status", CMD_TIER_T0 },
    { "identity_public", CMD_TIER_T0 }, { "identity_pop", CMD_TIER_T0 },
    { "identity_wipe", CMD_TIER_T0 },   /* always refused off the USB console: it changes nothing */
    { "spi_status", CMD_TIER_T0 }, { "sensor_status", CMD_TIER_T0 }, { "can_status", CMD_TIER_T0 },
    { "ota_status", CMD_TIER_T0 }, { "blob_status", CMD_TIER_T0 }, { "dac_loop_probe", CMD_TIER_T0 },
    /* T0, observational: they read the DUT */
    { "capture", CMD_TIER_T0 }, { "capture_dual", CMD_TIER_T0 }, { "capture_read", CMD_TIER_T0 },
    { "stream", CMD_TIER_T0 }, { "la_capture", CMD_TIER_T0 }, { "sensor_regs", CMD_TIER_T0 },
    { "sensor_la", CMD_TIER_T0 }, { "can_read", CMD_TIER_T0 }, { "psram_ping", CMD_TIER_T0 },
    { "test", CMD_TIER_T0 },
    /* mixed: read form T0, write form T2 (see mixed_write) */
    { "dac_limits", CMD_TIER_T0 }, { "calibrate", CMD_TIER_T0 }, { "eth", CMD_TIER_T0 },
    { "sig_policy", CMD_TIER_T0 }, { "lan_policy", CMD_TIER_T0 },
    { "cloud_ca", CMD_TIER_T0 }, { "cloud_proxy", CMD_TIER_T0 },
    /* T1: instrument control */
    { "generate", CMD_TIER_T1 }, { "measure", CMD_TIER_T1 }, { "load", CMD_TIER_T1 },
    { "load_bin", CMD_TIER_T1 }, { "replay", CMD_TIER_T1 }, { "dac_stop", CMD_TIER_T1 },
    { "dac_set", CMD_TIER_T1 }, { "dac_mux", CMD_TIER_T1 }, { "cal_switch", CMD_TIER_T1 },
    { "analog_path", CMD_TIER_T1 }, { "dac_out", CMD_TIER_T1 }, { "current_out", CMD_TIER_T1 },
    { "adc_read", CMD_TIER_T1 }, { "dac_control_loop", CMD_TIER_T1 },
    { "dac_loop_input", CMD_TIER_T1 }, { "la", CMD_TIER_T1 }, { "gpio", CMD_TIER_T1 },
    { "la_voltage", CMD_TIER_T1 }, { "nrst", CMD_TIER_T1 }, { "target_power", CMD_TIER_T1 },
    { "power_profile", CMD_TIER_T1 }, { "dap_start", CMD_TIER_T1 },
    { "uart_proxy_start", CMD_TIER_T1 }, { "spi_start", CMD_TIER_T1 }, { "spi_stop", CMD_TIER_T1 },
    { "spi_xfer", CMD_TIER_T1 }, { "spi_stream", CMD_TIER_T1 }, { "spi_flash", CMD_TIER_T1 },
    { "sensor_start", CMD_TIER_T1 }, { "sensor_set", CMD_TIER_T1 }, { "sensor_stop", CMD_TIER_T1 },
    { "can_config", CMD_TIER_T1 }, { "can_write", CMD_TIER_T1 }, { "can_term", CMD_TIER_T1 },
    { "can_respond", CMD_TIER_T1 }, { "can_disable", CMD_TIER_T1 }, { "speedtest", CMD_TIER_T1 },
    { "fpga_image", CMD_TIER_T1 }, { "psram_recover", CMD_TIER_T1 },
    /* T2: persisted config */
    { "cloud_set", CMD_TIER_T2 }, { "cloud_clear", CMD_TIER_T2 }, { "wifi_set", CMD_TIER_T2 },
    { "wifi_clear", CMD_TIER_T2 },
    /* T3: firmware */
    { "ota_begin", CMD_TIER_T3 }, { "ota_data", CMD_TIER_T3 }, { "ota_end", CMD_TIER_T3 },
    { "ota_commit", CMD_TIER_T3 }, { "ota_abort", CMD_TIER_T3 }, { "ota_selftest", CMD_TIER_T3 },
};

static const tier_entry_t *find(const char *cmd) {
    if (!cmd) return NULL;
    for (size_t i = 0; i < sizeof(k_tiers) / sizeof(k_tiers[0]); i++)
        if (strcmp(cmd, k_tiers[i].cmd) == 0) return &k_tiers[i];
    return NULL;
}

/* The write forms of the three mixed verbs (the handlers' own tests, command_handler.c). */
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

cmd_tier_t cmd_tier(const char *cmd, const char *json) {
    const tier_entry_t *e = find(cmd);
    if (!e) return CMD_TIER_T3;
    return mixed_write(cmd, json) ? CMD_TIER_T2 : e->tier;
}

int cmd_tier_known(const char *cmd) { return find(cmd) != NULL; }

/* The T0 reads that take the capture hardware (PSRAM, the LA, the ADC, the sensor bus): not
   light, so a LAN client cannot run them while a cloud job holds the pod. */
static const char *const k_heavy_reads[] = {
    "capture", "capture_dual", "capture_read", "stream", "la_capture", "sensor_regs",
    "sensor_la", "can_read", "psram_ping", "test",
};

int cmd_tier_light(const char *cmd, const char *json) {
    if (!find(cmd) || cmd_tier(cmd, json) != CMD_TIER_T0) return 0;
    for (size_t i = 0; i < sizeof(k_heavy_reads) / sizeof(k_heavy_reads[0]); i++)
        if (strcmp(cmd, k_heavy_reads[i]) == 0) return 0;
    return 1;
}

const char *cmd_tier_name(cmd_tier_t t) {
    static const char *const names[] = { "T0", "T1", "T2", "T3" };
    return (unsigned)t < 4u ? names[t] : "T?";
}
