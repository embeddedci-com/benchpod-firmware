#include "cmd_gate.h"
#include "cmd_table.h"
#include "bp_json.h"

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

    return cmd_gate_device(cmd, json, ctx->skip_hw, ctx->has_analog);
}

const char *cmd_gate_device(const char *cmd, const char *json, bool skip_hw, bool has_analog) {
    if (skip_hw && !cmd_gate_ok_without_hw(cmd))
        return "safe mode: iCE40/PSRAM are off. Unplug and replug the pod";

    if (!has_analog) {
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
