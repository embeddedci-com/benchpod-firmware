#include "cmd_gate.h"
#include "bp_json.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Commands that still work with the iCE40/PSRAM off (boot_guard safe mode): network config,
   status and target power, so the operator can see what happened and recover without
   driving an uninitialised SPI1/XSPI. */
bool cmd_gate_ok_without_hw(const char *cmd) {
    static const char *const ok[] = {
        "ping", "status", "cloud_set", "cloud_status", "cloud_clear",
        "wifi_set", "wifi_status", "wifi_clear", "eth", "speedtest",
        "la_voltage", "usb_cc", "nrst", "target_power", "target_status", "power_status",
        "power_profile", "identity_public", "identity_pop", "identity_wipe", "dac_limits",
        "can_config", "can_write", "can_read", "can_status", "can_term", "can_respond",
        "can_disable",
    };
    for (size_t i = 0; i < sizeof(ok) / sizeof(ok[0]); i++)
        if (strcmp(cmd, ok[i]) == 0) return true;
    return false;
}

/* Commands that need the analog front end (DAC, ADC, relays, the 4-20 mA terminals). Refused on
   the digital-only board (board_variant.h) with a clear reason, instead of "succeeding" against
   an ADC that is not there. dac_stop stays allowed: stopping nothing is harmless, and callers
   send it to clean up. capture_dual is handled separately: an LA-only capture is fine. */
bool cmd_gate_needs_analog(const char *cmd) {
    static const char *const analog[] = {
        "generate", "capture", "stream", "measure", "load", "load_bin", "replay",
        "dac_limits", "dac_set", "dac_mux", "cal_switch", "analog_path", "dac_out",
        "current_out", "adc_read", "calibrate", "dac_control_loop", "dac_loop_probe",
        "dac_loop_input",
    };
    for (size_t i = 0; i < sizeof(analog) / sizeof(analog[0]); i++)
        if (strcmp(cmd, analog[i]) == 0) return true;
    return false;
}

bool cmd_gate_scpi_line_writes(const char *line) {
    const char *p = line;
    for (;;) {                                   /* every ';'-separated part must be a query */
        const char *end = strchr(p, ';');
        size_t n = end ? (size_t)(end - p) : strlen(p);
        if (!memchr(p, '?', n)) return true;
        if (!end) return false;
        p = end + 1;
    }
}

const char *cmd_gate_check(const char *cmd, const char *json, cmd_tier_t tier,
                           const cmd_gate_ctx_t *ctx, char *why, size_t why_cap) {
    /* Tier gates (docs/design/policy-commands.md): a locked LAN keeps T2/T3 for the cloud and
       USB, and a cloud tunnel never goes above what the server allowed its user. */
    if (tier >= CMD_TIER_T2 && ctx->src == POLICY_SRC_LAN && ctx->lan_policy != POD_LAN_OPEN) {
        snprintf(why, why_cap, "locked: %s needs the cloud or the USB console", cmd);
        return why;
    }
    /* A cloud job holds the pod: the LAN may look, not touch (lease_gate.h). */
    if (ctx->src == POLICY_SRC_LAN && ctx->lease_active && !cmd_tier_light(cmd, json)) {
        snprintf(why, why_cap, "busy: a cloud job holds this pod (%s, %lu s left)",
                 ctx->lease_holder && ctx->lease_holder[0] ? ctx->lease_holder : "cloud",
                 (unsigned long)ctx->lease_left_s);
        return why;
    }
    if (ctx->tunnel_max_tier >= 0 && (int)tier > ctx->tunnel_max_tier) {
        snprintf(why, why_cap, "forbidden: %s needs an organization owner or admin", cmd);
        return why;
    }

    if (ctx->skip_hw && !cmd_gate_ok_without_hw(cmd))
        return "safe mode: iCE40/PSRAM are off. Unplug and replug the pod";

    if (!ctx->has_analog) {
        bool adc_capture = false;
        if (strcmp(cmd, "capture_dual") == 0) {
            char n[16] = {0};
            adc_capture = json && bp_json_get(json, "adc_samples", n, sizeof(n)) && atoi(n) > 0;
        }
        if (adc_capture || cmd_gate_needs_analog(cmd))
            return "this BenchPod has no analog front end (digital board): no DAC, ADC or analog outputs. "
                   "Restart the pod after fitting an analog add-on";
    }
    return NULL;
}
