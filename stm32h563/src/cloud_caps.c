/*
 * cloud_caps.c — the capabilities frame (see cloud_caps.h).
 */
#include "cloud_caps.h"
#include "bp_json.h"

#include <string.h>

static const char *tf(bool b) { return b ? "true" : "false"; }

/* Feature fields: everything up to the boot health, as one object prefix. */
static void emit_features(bp_emit_t *e, const cloud_caps_t *c) {
    bp_emit_raw(e, "{\"type\":\"capabilities\",\"device_id\":");
    bp_emit_jstr(e, c->device_id);
    bp_emit_raw(e, ",\"firmware_version\":");
    bp_emit_jstr(e, c->firmware_version);
    bp_emit(e, ",\"ota\":true,\"flash_kb\":%lu,\"blob_slots\":true,"
               "\"ota_sig\":true,\"sig_policy\":\"%s\",\"sig_policy_cmd\":true,\"ws_auth_v2\":true,"
               "\"lease_state\":true,\"cloud_ca\":true,\"cloud_proxy\":true,"
               "\"lan_policy\":\"%s\",\"lan_policy_cmd\":true,\"tunnel_max_tier\":true,",
            c->flash_kb, c->sig_policy, c->lan_policy);
    bp_emit(e, "\"serial\":false,\"scope\":%s,\"analog\":%s,\"analyzer\":true,\"command\":true,\"tunnel\":true,"
               "\"adc_bits\":%d,\"adc_fullscale_mv\":%d,\"adc_channels\":%d,"
               "\"adc_cal_a_uv\":%ld,\"adc_cal_b_nv\":%ld,\"adc_cal_unwrap\":true,",
            tf(c->analog), tf(c->analog), c->adc_bits, c->adc_fullscale_mv, c->adc_channels,
            c->adc_cal_a_uv, c->adc_cal_b_nv);
    bp_emit(e, "\"dac\":%s,\"dac_replay\":%s,\"dac_dc\":%s,\"dac_bits\":%d,\"dac_replay_bits\":%d,"
               "\"dac_fullscale_mv\":%d,\"dac_channels\":%d,"
               "\"dac_deep_replay\":%s,\"dac_replay_max_samples\":%lu,"
               "\"dac_control_loop\":%s,\"dac_loop_sources\":%s,\"dac_loop_input_map\":%s,"
               "\"dac_cotrig\":%s,",
            tf(c->dac_ac), tf(c->dac_replay), tf(c->dac_dc), c->dac_bits, c->dac_replay_bits,
            c->dac_fullscale_mv, c->dac_channels, tf(c->deep_replay), c->replay_max_samples,
            tf(c->control_loop), tf(c->loop_sources), tf(c->loop_input_map), tf(c->cotrig));
    bp_emit(e, "\"la_pins\":true,\"gpio_read\":%s,\"capture_trigger\":%s,\"spi_master\":%s,\"spi_stream\":%s,"
               "\"nrst_pin\":%s,"
               "\"power_profile\":true,"
               "\"capture_b64\":true,\"dac_limits\":%s,\"calibrate\":%s,\"can\":true,\"pod_current\":%s,"
               "\"current_out\":%s,\"current_out_min_ua\":%ld,\"current_out_max_ua\":%ld,",
            tf(c->gpio_read), tf(c->capture_trigger), tf(c->spi_master),
            tf(c->spi_master),   /* spi_stream: firmware, on the SPI master */
            tf(c->nrst_pin), tf(c->analog), tf(c->analog), tf(c->pod_current),
            tf(c->analog), c->current_out_min_ua, c->current_out_max_ua);
    bp_emit_raw(e, "\"board\":");
    bp_emit_jstr(e, c->board);
    bp_emit_raw(e, ",");
}

/* s cut to `max` characters, control characters (each up to six escaped bytes) as '?'. */
static void short_text(char *out, size_t cap, const char *s, size_t max) {
    size_t n = 0;
    for (; s && *s && n < max && n + 1 < cap; s++)
        out[n++] = ((unsigned char)*s < 0x20) ? '?' : *s;
    out[n] = '\0';
}

static void emit_health(bp_emit_t *e, const cloud_caps_t *c, bool cut) {
    const char *reason = c->safe_reason ? c->safe_reason : "";
    const char *crash  = c->last_crash ? c->last_crash : "none";
    char r[CLOUD_CAPS_TEXT_SHORT + 1], k[CLOUD_CAPS_TEXT_SHORT + 1];
    if (cut) {
        short_text(r, sizeof(r), reason, CLOUD_CAPS_TEXT_SHORT);
        short_text(k, sizeof(k), crash, CLOUD_CAPS_TEXT_SHORT);
        reason = r;
        crash = k;
    }
    bp_emit(e, "\"safe_mode\":%s,\"safe_reason\":", tf(c->safe_mode));
    bp_emit_jstr(e, reason);
    bp_emit_raw(e, ",\"reset_cause\":");
    bp_emit_jstr(e, c->reset_cause ? c->reset_cause : "unknown");
    bp_emit_raw(e, ",\"last_crash\":");
    bp_emit_jstr(e, crash);
    if (cut) bp_emit_raw(e, ",\"boot_health_cut\":true");
    bp_emit_raw(e, "}");
}

size_t cloud_caps_build(const cloud_caps_t *c, char *out, size_t cap) {
    if (!c || !out || cap == 0) return 0;
    bp_emit_t e;
    bp_emit_init(&e, out, cap);
    emit_features(&e, c);
    if (!bp_emit_ok(&e)) return 0;
    size_t mark = bp_emit_len(&e);
    emit_health(&e, c, false);
    if (bp_emit_ok(&e)) return bp_emit_len(&e);
    /* Too long: start the boot health again with its free-form texts shortened. */
    e.len = mark;
    e.ok = true;
    out[mark] = '\0';
    emit_health(&e, c, true);
    return bp_emit_ok(&e) ? bp_emit_len(&e) : 0;
}
