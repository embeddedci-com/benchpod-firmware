#include "cmd_gate.h"
#include "cmd_table.h"
#include "bp_json.h"
#include "bp_errors.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Safe mode and the digital-only board read the command table's flags (cmd_table.h). An
   unknown verb needs the hardware and no analog front end. */
bool cmd_gate_ok_without_hw(const char *cmd) { return cmd_has(cmd_find(cmd), CMD_F_NO_HW); }

/* dac_stop stays allowed on the digital board: stopping nothing is harmless, and callers send it
   to clean up. */
bool cmd_gate_needs_analog(const char *cmd) { return cmd_has(cmd_find(cmd), CMD_F_ANALOG); }

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

/* client_api, top level only: a sensor definition or a wiring profile a command carries cannot
   set it for the command. */
static int32_t client_api_of(const char *json) {
    char v[16];
    if (!json || !bp_json_get_top(json, "client_api", v, sizeof(v))) return 0;
    char *end = NULL;
    long n = strtol(v, &end, 10);
    if (end == v || *end != '\0' || n < 0) return 0;
    return n > INT32_MAX ? INT32_MAX : (int32_t)n;
}

bool cmd_gate_client_api(const cmd_desc_t *d, const char *json, uint32_t min_api, int32_t *sent) {
    int32_t n = client_api_of(json);
    if (sent) *sent = n;
    if (min_api == 0 || cmd_has(d, CMD_F_ANY_CLIENT)) return true;
    return (uint32_t)n >= min_api;
}

const char *cmd_gate_check(const cmd_desc_t *d, const char *cmd, const char *json, cmd_tier_t tier,
                           const cmd_gate_ctx_t *ctx, char *why, size_t why_cap) {
    /* Tier gates (docs/design/policy-commands.md): a locked LAN keeps T2/T3 for the cloud and
       USB, and a cloud tunnel never goes above what the server allowed its user. */
    if (tier >= CMD_TIER_T2 && ctx->src == POLICY_SRC_LAN && !ctx->usb_link &&
        ctx->lan_policy != POD_LAN_OPEN) {
        return bp_note_lan_locked(why, why_cap, cmd);
    }
    /* A cloud job holds the pod: the LAN may look, not touch (lease_gate.h). */
    if (ctx->src == POLICY_SRC_LAN && ctx->lease_active && !cmd_tier_light_of(d, json) &&
        !cmd_has(d, CMD_F_LEASE)) {   /* lease_acquire answers busy with the holder */
        return bp_note_lease_busy(why, why_cap, CMD_GATE_CLOUD_LEASE_HOLDER,
                                  ctx->lease_holder && ctx->lease_holder[0] ? ctx->lease_holder : "cloud",
                                  (int32_t)ctx->lease_left_s);
    }
    if (ctx->tunnel_max_tier >= 0 && (int)tier > ctx->tunnel_max_tier) {
        return bp_note_role_required(why, why_cap, cmd);
    }

    return cmd_gate_device_row(d, json, ctx->skip_hw, ctx->has_analog);
}

const char *cmd_gate_device(const char *cmd, const char *json, bool skip_hw, bool has_analog) {
    return cmd_gate_device_row(cmd_find(cmd), json, skip_hw, has_analog);
}

const char *cmd_gate_device_row(const cmd_desc_t *d, const char *json, bool skip_hw, bool has_analog) {
    if (skip_hw && !cmd_has(d, CMD_F_NO_HW))
        return "safe mode: iCE40/PSRAM are off. Unplug and replug the pod";

    if (!has_analog) {
        bool adc_capture = false;
        if (cmd_has(d, CMD_F_ADC_SAMPLES)) {
            char n[16] = {0};
            adc_capture = json && bp_json_get(json, "adc_samples", n, sizeof(n)) && atoi(n) > 0;
        }
        if (adc_capture || cmd_has(d, CMD_F_ANALOG)) {
            /* SCPI shows this text to a person, so it keeps its words; JSON sends BP0046. */
            static const char k_no_analog[] = "this BenchPod has no analog front end (digital board): "
                "no DAC, ADC or analog outputs. Restart the pod after fitting an analog add-on";
            return bp_tag_no_analog_front_end(k_no_analog, d ? d->key.name : "");
        }
    }
    return NULL;
}
