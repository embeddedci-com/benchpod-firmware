/*
 * cloud_caps.c: the capabilities frame (see cloud_caps.h).
 */
#include "cloud_caps.h"
#include "bp_json.h"
#include "signal_engine.h"   /* FPGA_FEATURE_* */

#include <string.h>

static const char *tf(bool b) { return b ? "true" : "false"; }

/* Feature fields: everything up to the boot health, as one object prefix. Firmware 4.0.0 sends
   no per-feature flags: the server takes the features from features/features.json with
   features_rev, the running gateware and image, and the facts (features.toml [facts], their
   `frame` names). What is left are values: versions, policies, scaling, ranges, the uplink. */
static void emit_features(bp_emit_t *e, const cloud_caps_t *c) {
    bp_emit_raw(e, "{\"type\":\"capabilities\",\"device_id\":");
    bp_emit_jstr(e, c->device_id);
    bp_emit_raw(e, ",\"firmware_version\":");
    bp_emit_jstr(e, c->firmware_version);
    /* ws_auth_v2 is the cloud handshake the pod speaks, not a feature: the server keys its auth
       on it. */
    bp_emit(e, ",\"flash_kb\":%lu,\"sig_policy\":\"%s\",\"ws_auth_v2\":true,\"lan_policy\":\"%s\",",
            c->flash_kb, c->sig_policy, c->lan_policy);
    bp_emit(e, "\"features_rev\":%u,\"api\":%u,\"min_client_api\":%u,"
               "\"gateware_version\":%u,\"fpga_image\":\"%s\",",
            c->features_rev, c->api, c->min_client_api, c->gateware_version,
            c->fpga_image ? c->fpga_image : "unknown");
    /* The facts (features.toml [facts] with a `frame` name). */
    bp_emit(e, "\"analog\":%s,\"nrst_pin\":%s,\"sensors_digital\":%s,\"wifi_scan\":%s,"
               "\"pod_current\":%s,\"adc_low_range\":%s,",
            tf(c->analog), tf(c->nrst_pin), tf(c->sensors_digital), c->no_wifi_scan ? "false" : "true",
            tf(c->pod_current), tf(c->adc_low_range));
    bp_emit(e, "\"adc_bits\":%d,\"adc_fullscale_mv\":%d,\"adc_channels\":%d,"
               "\"adc_cal_a_uv\":%ld,\"adc_cal_b_nv\":%ld,\"adc_cal_unwrap\":true,",
            c->adc_bits, c->adc_fullscale_mv, c->adc_channels, c->adc_cal_a_uv, c->adc_cal_b_nv);
    /* The ADC input range the fit above is for (analog module v3r4: adc_range). */
    if (c->analog)
        bp_emit(e, "\"adc_range\":\"%s\",\"adc_range_fs_mv\":%ld,",
                c->adc_range ? c->adc_range : "normal", c->adc_range_fs_mv);
    bp_emit(e, "\"dac_bits\":%d,\"dac_replay_bits\":%d,\"dac_fullscale_mv\":%d,\"dac_channels\":%d,"
               "\"dac_replay_max_samples\":%lu,\"generate_bits\":%d,\"la_max_rate_hz\":%lu,"
               "\"current_out_min_ua\":%ld,\"current_out_max_ua\":%ld,",
            c->dac_bits, c->dac_replay_bits, c->dac_fullscale_mv, c->dac_channels, c->replay_max_samples,
            c->generate16 ? 16 : c->dac_bits, c->la_max_rate_hz, c->current_out_min_ua,
            c->current_out_max_ua);
    bp_emit_raw(e, "\"board\":");
    bp_emit_jstr(e, c->board);
    /* Which interface carries this connection, so the server can record eth vs Wi-Fi per connect. */
    bp_emit_raw(e, ",\"uplink\":");
    bp_emit_jstr(e, c->uplink ? c->uplink : "unknown");
    if (c->has_uplink_rssi) bp_emit(e, ",\"uplink_rssi_dbm\":%d", c->uplink_rssi_dbm);
    bp_emit_raw(e, ",");
}

void cloud_caps_emit_facts(bp_emit_t *e, const cloud_caps_t *c) {
    /* api / min_client_api: the client API level next to the table's revision (cloud_caps.h). */
    bp_emit(e, ",\"features_rev\":%u,\"api\":%u,\"min_client_api\":%u,\"fpga_image\":\"%s\",\"hw\":[",
            c->features_rev, c->api, c->min_client_api, c->fpga_image ? c->fpga_image : "unknown");
    /* The facts the table's `requires` names (features.toml [facts], `hw`), in a fixed order. */
    const char *sep = "";
    if (!c->no_wifi_scan) { bp_emit(e, "%s\"wifi\"", sep); sep = ","; }
    if (c->usb_cc)        { bp_emit(e, "%s\"usb_cc\"", sep); sep = ","; }
    if (c->pod_current)   { bp_emit(e, "%s\"pod_current\"", sep); sep = ","; }
    if (c->adc_low_range) { bp_emit(e, "%s\"adc_low_range\"", sep); sep = ","; }
    if (c->vref_out)      { bp_emit(e, "%s\"vref_out\"", sep); sep = ","; }
    if (c->supply_3v3_1v8)     { bp_emit(e, "%s\"supply_3v3_1v8\"", sep); sep = ","; }
    bp_emit_raw(e, "]");
}

const char *cloud_caps_image_name(unsigned gateware_version, unsigned feature_bits) {
    if (gateware_version == 0) return "unknown";
    if (feature_bits & FPGA_FEATURE_EMULATION)   return "emu";
    if (feature_bits & FPGA_FEATURE_CONTROL_LOOP) return "loop";
    if (feature_bits & FPGA_FEATURE_DEEP_REPLAY)  return "deep";
    return "unknown";
}

/* s cut to `max` characters, every character that JSON escapes (control characters up to six
   bytes, '"' and '\\' two) as '?', so a cut text costs exactly one byte per character. */
static void short_text(char *out, size_t cap, const char *s, size_t max) {
    size_t n = 0;
    for (; s && *s && n < max && n + 1 < cap; s++) {
        unsigned char ch = (unsigned char)*s;
        out[n++] = (ch < 0x20 || ch == '"' || ch == '\\') ? '?' : (char)ch;
    }
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
    /* The MACs, so the web shows them, only when known (the server keeps the last ones it saw).
       Left out of the shortened form: they are not features, and the frame must fit. */
    if (!cut && c->eth_mac[0])  bp_emit(e, "\"eth_mac\":\"%s\",", c->eth_mac);
    if (!cut && c->wifi_mac[0]) bp_emit(e, "\"wifi_mac\":\"%s\",", c->wifi_mac);
    bp_emit(e, "\"boot_id\":\"%08lx\",\"unclean_resets\":%lu,",
            (unsigned long)c->boot_id, (unsigned long)c->unclean_resets);
    if (c->c3) {   /* the built-in Wi-Fi's co-processor: how often the pod had to restart it */
        bp_emit(e, "\"c3_restarts\":%lu,", (unsigned long)c->c3_restarts);
        if (c->c3_restarts && c->c3_restart_last)
            bp_emit(e, "\"c3_restart_last\":\"%s\",", c->c3_restart_last);
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
