/*
 * cloud_caps.c: the capabilities frame (see cloud_caps.h).
 */
#include "cloud_caps.h"
#include "bp_json.h"
#include "signal_engine.h"   /* FPGA_FEATURE_* */

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
               "\"lease_state\":true,"
               "\"lan_policy\":\"%s\",\"lan_policy_cmd\":true,\"tunnel_max_tier\":true,\"lan_tls\":true,"
               "\"wiring_store\":true,"
               "\"wifi_scan\":%s,",
            c->flash_kb, c->sig_policy, c->lan_policy, c->no_wifi_scan ? "false" : "true");
    /* The feature table's revision, the running gateware and its image: the server takes the
       features from features/features.json with these (features.toml). The flags the server
       never read from this frame (cal_dmm, can_fd, uart_frame, spi_stream, error_codes,
       cloud_ca, cloud_proxy) left it for them; the rest go once every server reads the table. */
    bp_emit(e, "\"features_rev\":%u,\"gateware_version\":%u,\"fpga_image\":\"%s\",",
            c->features_rev, c->gateware_version, c->fpga_image ? c->fpga_image : "unknown");
    bp_emit(e, "\"serial\":false,\"scope\":%s,\"analog\":%s,\"analyzer\":true,\"command\":true,\"tunnel\":true,"
               "\"adc_bits\":%d,\"adc_fullscale_mv\":%d,\"adc_channels\":%d,"
               "\"adc_cal_a_uv\":%ld,\"adc_cal_b_nv\":%ld,\"adc_cal_unwrap\":true,",
            tf(c->analog), tf(c->analog), c->adc_bits, c->adc_fullscale_mv, c->adc_channels,
            c->adc_cal_a_uv, c->adc_cal_b_nv);
    /* The ADC input range the fit above is for (analog module v3r4: adc_range). */
    if (c->analog)
        bp_emit(e, "\"adc_low_range\":%s,\"adc_range\":\"%s\",\"adc_range_fs_mv\":%ld,",
                tf(c->adc_low_range), c->adc_range ? c->adc_range : "normal", c->adc_range_fs_mv);
    bp_emit(e, "\"dac\":%s,\"dac_replay\":%s,\"dac_dc\":%s,\"dac_bits\":%d,\"dac_replay_bits\":%d,"
               "\"dac_fullscale_mv\":%d,\"dac_channels\":%d,"
               "\"dac_deep_replay\":%s,\"dac_replay_max_samples\":%lu,"
               "\"dac_control_loop\":%s,\"dac_loop_sources\":%s,\"dac_loop_input_map\":%s,"
               "\"dac_cotrig\":%s,\"generate_bits\":%d,",
            tf(c->dac_ac), tf(c->dac_replay), tf(c->dac_dc), c->dac_bits, c->dac_replay_bits,
            c->dac_fullscale_mv, c->dac_channels, tf(c->deep_replay), c->replay_max_samples,
            tf(c->control_loop), tf(c->loop_sources), tf(c->loop_input_map), tf(c->cotrig),
            c->generate16 ? 16 : c->dac_bits);
    bp_emit(e, "\"la_max_rate_hz\":%lu,", c->la_max_rate_hz);
    bp_emit(e, "\"la_pins\":true,\"gpio_read\":%s,\"capture_trigger\":%s,\"spi_master\":%s,"
               "\"nrst_pin\":%s,"
               "\"power_profile\":true,"
               "\"capture_b64\":true,\"dac_limits\":%s,\"calibrate\":%s,\"can\":true,\"pod_current\":%s,"
               "\"current_out\":%s,\"current_out_min_ua\":%ld,\"current_out_max_ua\":%ld,"
               "\"sensors\":true,\"sensors_format\":1,\"sensors_digital\":%s,",
            tf(c->gpio_read), tf(c->capture_trigger), tf(c->spi_master),
            tf(c->nrst_pin), tf(c->analog), tf(c->analog), tf(c->pod_current),
            tf(c->analog), c->current_out_min_ua, c->current_out_max_ua, tf(c->sensors_digital));
    bp_emit_raw(e, "\"board\":");
    bp_emit_jstr(e, c->board);
    /* Which interface carries this connection, so the server can record eth vs Wi-Fi per connect. */
    bp_emit_raw(e, ",\"uplink\":");
    bp_emit_jstr(e, c->uplink ? c->uplink : "unknown");
    if (c->has_uplink_rssi) bp_emit(e, ",\"uplink_rssi_dbm\":%d", c->uplink_rssi_dbm);
    bp_emit_raw(e, ",");
}

void cloud_caps_emit_list(bp_emit_t *e, const cloud_caps_t *c) {
    /* Firmware features every pod has, then the ones that depend on the board, the analog front
       end and the running gateware image. "signal" is the scope's old name, kept first for older
       clients on the analog board only: the digital board has no scope (its status says
       analog:false, and the SDK only reads "signal" from firmware without that flag). */
    bp_emit_raw(e, c->analog ? ",\"caps\":[\"signal\"," : ",\"caps\":[");
    bp_emit_raw(e, "\"gpio\",\"power\",\"swd\",\"uart\",\"la\""
                   ",\"analyzer\",\"command\",\"tunnel\",\"ota\""
                   ",\"la_pins\",\"power_profile\",\"capture_b64\""
                   ",\"can\""     /* classic CAN on FDCAN1 / TCAN1044 (can_bus.c) */
                   ",\"can_fd\""  /* CAN FD: can_config data_bitrate, 64-byte frames */
                   ",\"sensors\""     /* emulated sensors from definitions (sensor_define, sensor_mgr.h) */
                   ",\"sensor_la_trigger\""  /* sensor_la waits for the bus's START (trigger, timeout_ms) */
                   ",\"usb_net\""       /* the pod's API on a USB network adapter (usb_netif.c) */
                   ",\"usb_net_setting\""  /* usb_network on/off, device-only (pod_policy.h) */
                   ",\"lan_tls\"");        /* the LAN API over TLS on 8443, lan_tls command (lan_tls.h) */
    /* wiring_store and error_codes are in the capabilities frame only: the cloud status is at its
       size limit. A LAN client finds them by asking (wiring_list; a "code" in a refusal). */
    if (!c->no_wifi_scan) bp_emit_raw(e, ",\"wifi_scan\"");   /* the built-in Wi-Fi or a Wi-Fi bridge (wifi_scan.h) */
    if (c->pod_current) bp_emit_raw(e, ",\"pod_current\"");
    if (c->adc_low_range) bp_emit_raw(e, ",\"adc_low_range\"");   /* adc_range low (module v3r4) */
    if (c->analog) bp_emit_raw(e, ",\"analog\",\"scope\",\"dac_limits\",\"calibrate\",\"cal_dmm\",\"current_out\"");
    if (c->dac_ac)          bp_emit_raw(e, ",\"dac\"");
    if (c->dac_dc)          bp_emit_raw(e, ",\"dac_dc\"");
    if (c->dac_replay)      bp_emit_raw(e, ",\"dac_replay\"");
    if (c->deep_replay)     bp_emit_raw(e, ",\"dac_deep_replay\"");
    if (c->control_loop)    bp_emit_raw(e, ",\"dac_control_loop\"");
    if (c->cotrig)          bp_emit_raw(e, ",\"dac_cotrig\"");
    if (c->generate16)      bp_emit_raw(e, ",\"generate16\"");   /* generate takes volts/uA, 16-bit */
    if (c->loop_sources)    bp_emit_raw(e, ",\"dac_loop_sources\"");
    if (c->loop_input_map)  bp_emit_raw(e, ",\"dac_loop_input_map\"");
    if (c->gpio_read)       bp_emit_raw(e, ",\"gpio_read\"");
    if (c->capture_trigger) bp_emit_raw(e, ",\"capture_trigger\"");
    if (c->spi_master)      bp_emit_raw(e, ",\"spi_master\",\"spi_stream\"");   /* spi_stream: firmware */
    if (c->uart_frame)      bp_emit_raw(e, ",\"uart_frame\"");   /* uart_proxy_start parity, stop_bits */
    if (c->nrst_pin)        bp_emit_raw(e, ",\"nrst_pin\"");
    if (c->usb_cc)          bp_emit_raw(e, ",\"usb_cc\"");
    bp_emit_raw(e, "]");
}

void cloud_caps_emit_facts(bp_emit_t *e, const cloud_caps_t *c) {
    bp_emit(e, ",\"features_rev\":%u,\"fpga_image\":\"%s\",\"hw\":[", c->features_rev,
            c->fpga_image ? c->fpga_image : "unknown");
    /* The facts the table's `requires` names (features.toml [facts], `hw`), in a fixed order. */
    const char *sep = "";
    if (!c->no_wifi_scan) { bp_emit(e, "%s\"wifi\"", sep); sep = ","; }
    if (c->usb_cc)        { bp_emit(e, "%s\"usb_cc\"", sep); sep = ","; }
    if (c->pod_current)   { bp_emit(e, "%s\"pod_current\"", sep); sep = ","; }
    if (c->adc_low_range) { bp_emit(e, "%s\"adc_low_range\"", sep); sep = ","; }
    if (c->vref_out)      { bp_emit(e, "%s\"vref_out\"", sep); sep = ","; }
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
