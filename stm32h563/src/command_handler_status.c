/*
 * command_handler_status.c: the `status` reply.
 *
 * Runs on the hw worker task, like the rest of command_handler.
 *
 * Two forms. The full form goes to LAN, USB, serial and the cloud byte tunnels. The short form
 * ("short_form":true) goes to the cloud command channel (CH_CLOUD_CONN), which caps a reply at
 * BP_CLOUD_REPLY_MAX (cloud_reply_cap.c), and to any connection whose TX ring cannot take the
 * full form whole (conn_tx.c). It leaves out what a client gets elsewhere:
 *   lan_tls{}              the `lan_tls` command
 *   lease{}                the server, which owns the cloud lease
 *   module{}               the `module` command (digital v3r3)
 *   adc_cal_*, adc_range*  the capabilities frame (server cap.*)
 *   sensors_format, ota_sig, sig_policy_cmd, lan_policy_cmd, tunnel_max_tier, lease_state
 *                          fixed feature flags, also in the capabilities frame (server cap.*)
 *   stacks{}, lwip_*, pbuf_pool_*, malloc_failures, rx_dropped, tx_dropped, board_rev_mv
 *                          diagnostics, full form only
 *   emi_test               the `emi_log` command (EMC test build)
 * and cuts last_crash and safe_reason to STATUS_SHORT_TEXT_MAX characters ("boot_health_cut":true
 * when it did; the capabilities frame has them whole). test_ch_status.c builds the worst case of
 * the short form and checks that it fits the cloud channel with STATUS_SHORT_MARGIN bytes to spare.
 */
#include "command_handler.h"
#include "command_handler_internal.h"
#include "command_handler_status.h"
#include "at_driver.h"
#include "bp_json.h"
#include "console.h"     /* console_rx_dropped / console_tx_dropped */
#include "signal_engine.h"
#include "stm32h5xx_hal.h"   /* HAL_GetTick */
#include "wifi_manager.h"
#include "cloud_client.h"
#include "esp_wifi_ctrl.h"
#if defined(BENCHPOD_WIFI_BRIDGE)
#include "wifi_bridge_net.h"   /* status reports the bridge's Wi-Fi when there is no built-in link */
#endif
#include "usb_device.h"
#include "usb_netif.h"
#include "board_info.h"
#include "board_rev.h"
#include "usb_cc.h"
#include "fault.h"       /* reset cause + last-crash summary */
#include "sys_health.h"  /* heap/stack headroom */
#include "ice40_flash.h"
#if defined(BENCHPOD_EMI_TEST)
#include "emi_test.h"
#endif
#include "version.h"     /* FIRMWARE_VERSION (single source) */
#include "fw_sign.h"
#include "cloud_caps.h"     /* the capabilities, shared with the cloud announcement */
#include "lease_gate.h"
#include "module_link.h"   /* digital v3r3: the module object */
#include "psram.h"       /* psram_selftest_str */
#include "pod_policy.h"
#include "lwip/stats.h"    /* lwIP memory high-water marks */
#include "lwip/memp.h"
#include "lan_tls.h"

#include <stdio.h>
#include <string.h>

/* Free-form text. In the short form (max > 0) it is cut to max characters (*cut set when it was)
   and every character JSON escapes is written as '?', so the text costs at most max bytes, like
   the capabilities frame's short texts (cloud_caps.c). */
static void emit_text(bp_emit_t *e, const char *s, size_t max, bool *cut) {
    if (!s) s = "";
    if (max == 0) { bp_emit_jstr(e, s); return; }
    char b[STATUS_SHORT_TEXT_MAX + 1];
    if (max > STATUS_SHORT_TEXT_MAX) max = STATUS_SHORT_TEXT_MAX;
    size_t n = 0;
    for (; *s && n < max; s++) {
        unsigned char ch = (unsigned char)*s;
        b[n++] = (ch < 0x20 || ch == '"' || ch == '\\') ? '?' : (char)ch;
    }
    b[n] = '\0';
    if (*s) *cut = true;
    bp_emit_jstr(e, b);
}

/* What both forms share, read once: the Wi-Fi RSSI costs an AT round-trip. */
typedef struct {
    const char  *ip, *wstate, *wifi;
    char         rssi_field[24];
#if defined(BENCHPOD_WIFI_BRIDGE)
    char         wifi_buf[32];
#endif
    cloud_caps_t caps;
} status_in_t;

static void status_read(status_in_t *in) {
    in->ip = wifi_get_ip();
    wifi_state_t st = wifi_get_state();
    switch (st) {
        case WIFI_DISCONNECTED: in->wstate = "disconnected"; break;
        case WIFI_CONNECTING:   in->wstate = "connecting";   break;
        case WIFI_CONNECTED:    in->wstate = "connected";    break;
        case WIFI_READY:        in->wstate = "ready";        break;
        default:                in->wstate = "unknown";      break;
    }
    /* RSSI is null if not associated; otherwise the AP signal strength in
       dBm.  Querying it costs an AT round-trip (~10s of ms) so we only do
       it when associated. */
    strcpy(in->rssi_field, "null");
    if (st == WIFI_CONNECTED || st == WIFI_READY) {
        int rssi = 0;
        if (wifi_get_rssi(&rssi) == 0) snprintf(in->rssi_field, sizeof(in->rssi_field), "%d", rssi);
    }
    in->wifi = esp_wifi_ctrl_state_str();
#if defined(BENCHPOD_WIFI_BRIDGE)
    /* No built-in Wi-Fi configured (v4 has no radio, a v3 has it off) and a Wi-Fi bridge on the
       Ethernet port: "wifi" and rssi_dbm are the bridge's, e.g. "bridge connected". */
    if (wifi_bridge_in_use()) {
        wb_info_t bi;
        wifi_bridge_get_info(&bi);
        snprintf(in->wifi_buf, sizeof(in->wifi_buf), "bridge %s", wb_state_str(bi.state));
        in->wifi = in->wifi_buf;
        if (wb_rssi_known(&bi)) snprintf(in->rssi_field, sizeof(in->rssi_field), "%d", bi.rssi);
        else                    strcpy(in->rssi_field, "null");
    }
#endif
    cloud_caps_collect(&in->caps);
}

/* One form of the reply into e (the bounds-tracked emitter: last_crash is free-form text,
   escaped via bp_emit_jstr, so it can never break the JSON). */
static void status_emit(bp_emit_t *e, const status_in_t *in, bool brief) {
    const cloud_caps_t *c = &in->caps;
    const size_t text_max = brief ? STATUS_SHORT_TEXT_MAX : 0;
    bool text_cut = false;
    bp_emit(e, "{\"status\":\"ok\",\"data\":{"
               "\"device\":\"benchpod\",\"version\":\"%s\",\"board\":\"%s\",\"net\":\"%s\","
               "\"ip\":\"%s\",\"rssi_dbm\":%s,\"wifi\":\"%s\",\"cloud\":\"%s\",",
            FIRMWARE_VERSION, BOARD_NAME, in->wstate, in->ip, in->rssi_field,
            in->wifi, cloud_client_state_str());
    if (brief) bp_emit_raw(e, "\"short_form\":true,");
    /* C3 restarts since power-on (c3_restarts.h); the causes are in wifi_status. */
    if (esp_wifi_ctrl_module_present())
        bp_emit(e, "\"c3_restarts\":%lu,", (unsigned long)esp_wifi_ctrl_restarts()->total);
    /* USB network adapter (CDC-NCM): on unless Wi-Fi is configured; ip is the pod's end of
       the /30, host_ip the address its DHCP server hands the computer. */
    /* source: what the USB-C port powering the pod advertises (usb_cc.h, v3; null on v2). */
    {
        usb_cc_t cc;
        bool have_cc = usb_cc_read(&cc) == 0 && cc.supported;
        /* setting: the stored USB network setting; off_reason when net is off. */
        bp_emit(e, "\"usb\":{\"net\":\"%s\",\"setting\":\"%s\",",
                usb_net_on() ? "on" : "off", usb_net_setting_on() ? "on" : "off");
        if (!usb_net_on())
            bp_emit(e, "\"off_reason\":\"%s\",", usb_net_setting_on() ? "wifi" : "setting");
        bp_emit(e, "\"link\":%s,\"ip\":\"%s\",\"host_ip\":\"%s\",",
                usb_net_link_up() ? "true" : "false", usb_netif_pod_ip(), usb_netif_host_ip());
        if (have_cc) bp_emit(e, "\"source\":\"%s\",\"source_ma\":%d},", cc.advertised, cc.advertised_ma);
        else         bp_emit_raw(e, "\"source\":null,\"source_ma\":null},");
    }
#if defined(BENCHPOD_EMI_TEST)
    /* The EMC test build (emi_test.h): the short state (the whole of it is in emi_log's reply).
       The full object made the LAN status 2115 bytes, more than a connection's TX ring takes
       whole, so the pod closed the connection instead of answering. Full form only. */
    if (!brief) {
        char emi[160];
        if (!emi_test_json_short(emi, sizeof(emi))) snprintf(emi, sizeof(emi), "null");
        bp_emit(e, "\"emi_test\":%s,", emi);
    }
#endif
    /* The console's dropped-byte counters: diagnostics, full form only. */
    if (!brief)
        bp_emit(e, "\"rx_dropped\":%lu,\"tx_dropped\":%lu,",
                (unsigned long)console_rx_dropped(), (unsigned long)console_tx_dropped());
    bp_emit(e, "\"adc_bits\":%d,\"adc_fullscale_mv\":%d,\"adc_channels\":%d,\"la_vccio_mv\":%d,",
            ADC_BITS, ADC_FULLSCALE_MV, ADC_CHANNELS, la_vccio_get_mv());
    /* The front-SMA fit captures are scaled with, this pod's offset included (adc_cal.h): the
       same values the cloud capabilities carry, so a LAN client scales like the server does.
       Full form only: cloud clients get the fit from the capabilities frame. */
    if (c->analog && !brief)
        bp_emit(e, "\"adc_cal_a_uv\":%ld,\"adc_cal_b_nv\":%ld,\"adc_cal_unwrap\":true,"
                    "\"adc_range\":\"%s\",\"adc_range_fs_mv\":%ld,",
                c->adc_cal_a_uv, c->adc_cal_b_nv, c->adc_range, c->adc_range_fs_mv);
    /* Board revision decides which of the rev3 features below actually exist,
       so clients read it once here rather than probing each command. */
    bp_emit(e, "\"board_rev\":\"%s\",", board_rev_str());
    if (!brief) bp_emit(e, "\"board_rev_mv\":%d,", board_rev_strap_mv());   /* the strap voltage: full form only */
    bp_emit(e, "\"nrst_pin\":%s,", c->nrst_pin ? "true" : "false");
    /* Internal flash of this MCU: 2048 (ZIT6) or 1024 (ZGT6). Updaters check it before sending
       an image (flash_layout.h). */
    bp_emit(e, "\"flash_kb\":%lu,", c->flash_kb);
    /* Emulated sensors: the definition format the pod takes, and whether I2C/SPI/UART sensors work
       (the emulation image is installed or running). Clients tell from these that the pod may be on
       the emulation image, which has neither the loop's nor the deep image's feature. */
    /* The fixed feature flags below that the capabilities frame also carries (the server keeps
       them as cap.*) are in the full form only; the short form keeps the values. */
    if (!brief) bp_emit_raw(e, "\"sensors_format\":1,");
    bp_emit(e, "\"sensors_digital\":%s,", c->sensors_digital ? "true" : "false");
    /* Signed updates (fw_sign.h): ota_begin takes "sig"; the policy is "audit" (report only). */
    if (!brief)
        bp_emit_raw(e, "\"ota_sig\":true,\"sig_policy_cmd\":true,\"lan_policy_cmd\":true,"
                        "\"tunnel_max_tier\":true,\"lease_state\":true,");
    bp_emit(e, "\"sig_policy\":\"%s\",\"sig_keys\":%u,\"lan_policy\":\"%s\",\"cloud_ca\":true,\"cloud_proxy\":true,",
            c->sig_policy, (unsigned)fw_sign_key_count(), c->lan_policy);
    if (!brief) {
        /* The LAN API over TLS (lan_tls.h): clients pin this certificate's sha256. Full form only:
           the `lan_tls` command has it. */
        lan_tls_info_t tls;
        lan_tls_get_info(&tls);
        bp_emit(e, "\"lan_tls\":{\"port\":%d,\"https_only\":%s,\"sha256\":\"%s\"},", LAN_TLS_PORT,
                pod_policy_https_only() ? "true" : "false", tls.ready ? tls.sha256 : "");
    }
    /* The pod's copy of the cloud lease (lease_gate.h). The server owns the lease, so a cloud
       client has it already: full form only. */
    if (!brief) {
        uint32_t left = 0;
        bool held = lease_gate_active(HAL_GetTick(), &left);
        bp_emit(e, "\"lease\":{\"held\":%s,\"holder\":\"%s\",\"left_s\":%lu},",
                held ? "true" : "false", held ? lease_gate_holder() : "", (unsigned long)left);
    }
    /* lwIP memory high-water marks (lwipopts.h), to size MEM_SIZE and the pbuf pool from data.
       Diagnostics: full form only, like malloc_failures and stacks{}. */
    if (!brief) bp_emit(e, "\"lwip_mem_max\":%lu,\"lwip_mem_size\":%lu,\"pbuf_pool_max\":%u,\"pbuf_pool_size\":%u,",
            (unsigned long)lwip_stats.mem.max, (unsigned long)lwip_stats.mem.avail,
            (unsigned)lwip_stats.memp[MEMP_PBUF_POOL]->max, (unsigned)lwip_stats.memp[MEMP_PBUF_POOL]->avail);
    /* The gateware running in the iCE40 and the one this firmware embeds (0 = unknown). They
       differ after a firmware update until the boot-time gateware update has run. */
    if (!brief) {
        extern volatile uint32_t g_malloc_failures;   /* main.c: allocations that failed after boot */
        bp_emit(e, "\"malloc_failures\":%lu,", (unsigned long)g_malloc_failures);
    }
    bp_emit(e, "\"gateware\":%u,\"gateware_embedded\":%u,\"loop_tripped\":%s,\"uart_rx_overflow\":%s,",
            (unsigned)signal_engine_fpga_version(), (unsigned)ice40_embedded_gw_version(),
            fpga_dac_loop_tripped() ? "true" : "false", fpga_uart_rx_overflowed() ? "true" : "false");
    bp_emit(e, "\"psram\":\"%s\",\"psram_ok\":%s,\"heap_free\":%u,\"heap_min\":%u,\"stack_min\":%u,"
                "\"reset\":\"%s\",\"last_crash\":",
            psram_selftest_str(), signal_engine_psram_operable() ? "true" : "false",
            sys_health_heap_free(), sys_health_heap_min_free(),
            sys_health_stack_min_free(), fault_last_reset_str());
    emit_text(e, fault_last_crash_str(), text_max, &text_cut);
    /* Per-task stack headroom (minimum-ever free bytes), every task incl. IDLE and the timer
       task, so stack_min says which task it is. Full form only. */
    if (!brief) {
        sys_health_task_t t[SYS_HEALTH_MAX_TASKS];
        int n = sys_health_tasks(t, SYS_HEALTH_MAX_TASKS);
        bp_emit_raw(e, ",\"stacks\":{");
        for (int i = 0; i < n; i++) {
            if (i) bp_emit_raw(e, ",");
            bp_emit_jstr(e, t[i].name);
            bp_emit(e, ":%u", t[i].stack_free);
        }
        bp_emit_raw(e, "}");
    }
    /* Analog front end (board_variant.h): false on the digital-only board. */
    bp_emit(e, ",\"analog\":%s", c->analog ? "true" : "false");
    /* Digital v3r3 only (module_link.h): the analog module and the two rails it switches, short
       form (~90 B; `module` has the EEPROM ID). Full form only. */
    if (module_link_board() && !brief) {
        bp_emit_raw(e, ",\"module\":");
        module_emit_state(e, false);
    }
    /* Safe mode (boot_guard.h): the reason says what is off and why; "" when not. */
    bp_emit(e, ",\"safe_mode\":%s,\"safe_reason\":", c->safe_mode ? "true" : "false");
    emit_text(e, c->safe_reason, text_max, &text_cut);
    if (text_cut) bp_emit_raw(e, ",\"boot_health_cut\":true");
    /* caps[]: the pod's advertised feature set, and the only capability source a client on a
       direct LAN/serial connection ever sees. The same values as the cloud `capabilities` frame
       (cloud_caps.h), so the two cannot drift. */
    cloud_caps_emit_list(e, c);
    bp_emit_raw(e, "}}\n");
}

void handle_status(int conn_id, const char *json) {
    (void)json;
    /* The full form's worst case is over 2 KB (test_ch_status.c); the hw task has ~7 KB of stack
       left. A connection's TX ring takes a reply only whole (conn_tx.c, 2048 B), so a full form
       it does not take goes out as the short form instead. */
    char resp[2304];
    status_in_t in;
    status_read(&in);
    bp_emit_t e;
    /* The cloud command channel always gets the short form (see the top of this file). */
    if (conn_id != CH_CLOUD_CONN) {
        bp_emit_init(&e, resp, sizeof(resp));
        status_emit(&e, &in, false);
        if (bp_emit_ok(&e) && at_send_data(conn_id, (const uint8_t *)resp, bp_emit_len(&e)) == 0)
            return;
        /* conn_tx is all or nothing: nothing went out, so send the short form. */
    }
    bp_emit_init(&e, resp, sizeof(resp));
    status_emit(&e, &in, true);
    if (!bp_emit_ok(&e)) { send_error(conn_id, "status too large"); return; }
    if (at_send_data(conn_id, (const uint8_t *)resp, bp_emit_len(&e)) != 0) {
        /* A reply the connection's TX ring cannot take whole says so instead of the connection
           just closing. */
        char msg[64];
        snprintf(msg, sizeof(msg), "status too large for this connection (%u bytes)",
                 (unsigned)bp_emit_len(&e));
        send_error(conn_id, msg);
    }
}
