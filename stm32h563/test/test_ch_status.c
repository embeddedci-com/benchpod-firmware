/*
 * test_ch_status.c — host tests for src/command_handler_status.c, the `status` reply.
 *
 * The cloud command channel caps a reply at BP_CLOUD_REPLY_MAX (cloud_reply_cap.c). Firmware 3.10.4's
 * status outgrew it (1838 bytes with the USB network up and a lease held), so connect over the cloud
 * failed. Here the real handle_status runs with every optional block present and at its longest:
 * every capability, a full lease holder, the longest crash report and safe-mode reason (with
 * characters JSON escapes), the Wi-Fi bridge and C3 fields, the USB block with its off reason and
 * the USB-C source. The short form must leave STATUS_SHORT_MARGIN bytes free, so the next field
 * added to status fails this test instead of the cloud connect.
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "command_handler_internal.h"
#include "command_handler_status.h"
#include "at_driver.h"
#include "bp_limits.h"
#include "cloud_caps.h"
#include "cloud_client.h"
#include "console.h"
#include "esp_wifi_ctrl.h"
#include "fault.h"
#include "fw_sign.h"
#include "ice40_flash.h"
#include "lan_tls.h"
#include "lease_gate.h"
#include "module_link.h"
#include "pod_policy.h"
#include "psram.h"
#include "signal_engine.h"
#include "sys_health.h"
#include "usb_cc.h"
#include "usb_device.h"
#include "usb_netif.h"
#include "board_rev.h"
#include "wifi_manager.h"
#include "wifi_bridge_net.h"
#include "lwip/stats.h"

static int fails;
#define CHECK(cond, ...) do { if (!(cond)) { printf("FAIL %s:%d: ", __func__, __LINE__); \
        printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

static bool g_typical;   /* a pod as it runs: the real tasks, counters near zero */

/* ---- fakes: every input at its longest ------------------------------------------------- */
static char   g_tx[4096];
static size_t g_ntx;
static char   g_err[256];
/* conn_tx takes a reply only whole, up to its ring (CONN_TX_RING_SIZE 2048, minus one). */
static size_t g_ring = 2047;
static int    g_sends;
int at_send_data(int c, const uint8_t *b, size_t n) {
    (void)c;
    g_sends++;
    if (c != CH_CLOUD_CONN && n > g_ring) return -1;
    if (n > sizeof(g_tx) - 1) n = sizeof(g_tx) - 1;
    memcpy(g_tx, b, n); g_tx[n] = '\0'; g_ntx = n;
    return 0;
}
void send_error(int c, const char *m) { (void)c; snprintf(g_err, sizeof(g_err), "%s", m); }
uint32_t HAL_GetTick(void) { return 123456789u; }

/* Text with characters JSON escapes, longer than any cut. */
static char g_crash[160], g_reason[200];
static void fill_text(char *b, size_t cap) {
    for (size_t i = 0; i + 1 < cap; i++) b[i] = (i % 7 == 0) ? '"' : (i % 11 == 0) ? '\n' : 'x';
    b[cap - 1] = '\0';
}

void cloud_caps_collect(cloud_caps_t *c) {
    memset(c, 0, sizeof(*c));
    c->firmware_version = "3.10.5"; c->flash_kb = 2048;
    c->sig_policy = "enforce"; c->lan_policy = "https_only";
    c->analog = true; c->adc_bits = 16; c->adc_fullscale_mv = 4096; c->adc_channels = 1;
    c->adc_cal_a_uv = -1234567; c->adc_cal_b_nv = -98765;
    c->adc_low_range = true; c->adc_range = "normal"; c->adc_range_fs_mv = 10922;
    c->dac_ac = c->dac_replay = c->dac_dc = c->deep_replay = true;
    c->control_loop = c->loop_sources = c->loop_input_map = c->cotrig = c->generate16 = true;
    c->gpio_read = c->capture_trigger = c->spi_master = true;
    c->sensors_digital = c->nrst_pin = c->usb_cc = c->pod_current = true;
    c->no_wifi_scan = false;
    c->board = "stm32h563";
    c->safe_mode = true; c->safe_reason = g_reason;
    c->reset_cause = "watchdog"; c->last_crash = g_crash;
    c->c3 = true; c->c3_restarts = 4294967295u;
}

const char *wifi_get_ip(void) { return "255.255.255.255"; }
wifi_state_t wifi_get_state(void) { return WIFI_CONNECTED; }
int wifi_get_rssi(int *r) { *r = -127; return 0; }
const char *esp_wifi_ctrl_state_str(void) { return "waiting-slave"; }
bool esp_wifi_ctrl_module_present(void) { return true; }
static c3_restarts_t g_c3 = { .total = 4294967295u };
const c3_restarts_t *esp_wifi_ctrl_restarts(void) { return &g_c3; }
const char *cloud_client_state_str(void) { return "LINK_CONNECT_WAIT"; }
bool wifi_bridge_in_use(void) { return true; }
void wifi_bridge_get_info(wb_info_t *out) { memset(out, 0, sizeof(*out)); out->rssi = -128; }
const char *wb_state_str(uint8_t s) { (void)s; return "joining-network"; }   /* "bridge " + it fills wifi_buf */

/* The USB block with every optional field: net off for a reason, a USB-C source. */
static bool g_usb_on;
bool usb_net_on(void) { return g_usb_on; }
bool usb_net_setting_on(void) { return true; }
bool usb_net_link_up(void) { return true; }
const char *usb_netif_pod_ip(void) { return "255.255.255.254"; }
const char *usb_netif_host_ip(void) { return "255.255.255.253"; }
int usb_cc_read(usb_cc_t *o) {
    memset(o, 0, sizeof(*o)); o->supported = true; o->advertised = "default"; o->advertised_ma = 3000;
    return 0;
}

uint32_t console_rx_dropped(void) { return g_typical ? 0 : 4294967295u; }
uint32_t console_tx_dropped(void) { return g_typical ? 0 : 4294967295u; }
int la_vccio_get_mv(void) { return 3300; }
const char *board_rev_str(void) { return "analog-v3r4"; }
int board_rev_strap_mv(void) { return 3300; }
size_t fw_sign_key_count(void) { return 4; }
void lan_tls_get_info(lan_tls_info_t *o) {
    memset(o, 0, sizeof(*o)); o->ready = true; memset(o->sha256, 'a', 64);
}
bool pod_policy_https_only(void) { return true; }
static char g_holder[LEASE_GATE_HOLDER_MAX];
bool lease_gate_active(uint32_t now, uint32_t *left) { (void)now; *left = g_typical ? 600 : 4294967295u; return true; }
const char *lease_gate_holder(void) { return g_typical ? "web ward" : g_holder; }
struct stats_ lwip_stats;
static struct stats_mem g_pbuf_stats = { .max = 65535, .avail = 65535 };
volatile uint32_t g_malloc_failures;
uint8_t signal_engine_fpga_version(void) { return 255; }
uint8_t ice40_embedded_gw_version(void) { return 255; }
bool fpga_dac_loop_tripped(void) { return true; }
bool fpga_uart_rx_overflowed(void) { return true; }
const char *psram_selftest_str(void) { return "inoperable (iCE40 pad open)"; }
bool signal_engine_psram_operable(void) { return false; }
unsigned sys_health_heap_free(void) { return 999999; }   /* 640 KB of RAM: six digits */
unsigned sys_health_heap_min_free(void) { return 999999; }
unsigned sys_health_stack_min_free(void) { return 99999; }
int sys_health_tasks(sys_health_task_t *out, int max) {
    static const char *real[] = { "hw", "net", "usb", "console", "IDLE", "Tmr Svc" };
    int n = g_typical ? 6 : max;
    for (int i = 0; i < n; i++) {
        out[i].name = g_typical ? real[i] : "tasknamelong15c";
        out[i].stack_free = g_typical ? 1200 : 65535;
    }
    return n;
}
const char *fault_last_reset_str(void) { return "watchdog+lockup+brownou"; }
const char *fault_last_crash_str(void) { return g_crash; }
static bool g_module_board;
bool module_link_board(void) { return g_module_board; }
void module_emit_state(bp_emit_t *e, bool id) { (void)id; bp_emit_raw(e, "{\"module\":\"v3r4\"}"); }

/* ---- helpers ----------------------------------------------------------------------------- */
static size_t run(int conn) {
    g_ntx = 0; g_tx[0] = '\0'; g_err[0] = '\0'; g_sends = 0;
    handle_status(conn, "{\"cmd\":\"status\"}");
    CHECK(g_err[0] == '\0', "conn %d: error %s", conn, g_err);
    /* The reply proper, without the line ending (what cloud_reply_cap measures). */
    size_t n = g_ntx;
    while (n && (g_tx[n - 1] == '\n' || g_tx[n - 1] == '\r')) n--;
    return n;
}
static bool has(const char *key) {
    char k[64];
    snprintf(k, sizeof(k), "\"%s\":", key);
    return strstr(g_tx, k) != NULL;
}

/* ---- tests ------------------------------------------------------------------------------- */
static void test_cloud_worst_case_fits(void) {
    g_usb_on = false;     /* off_reason present */
    g_module_board = true;
    size_t n = run(CH_CLOUD_CONN);
    if (getenv("DUMP")) printf("%s\n", g_tx);
    size_t limit = BP_CLOUD_REPLY_MAX - 1u;   /* cloud_reply_cap: the NUL takes one byte */
    printf("  cloud status worst case: %zu bytes, limit %zu, margin %zu (want >= %u)\n",
           n, limit, n < limit ? limit - n : 0, STATUS_SHORT_MARGIN);
    CHECK(n + STATUS_SHORT_MARGIN <= limit, "cloud status %zu bytes leaves less than %u of %zu",
          n, STATUS_SHORT_MARGIN, limit);
    CHECK(strncmp(g_tx, "{\"status\":\"ok\",\"data\":{", 23) == 0, "start: %.40s", g_tx);
    CHECK(n >= 2 && memcmp(g_tx + n - 2, "}}", 2) == 0, "end: %.10s", g_tx + n - 10);
    /* What a cloud client needs is still there. */
    const char *keep[] = { "version", "board", "caps", "usb", "analog", "gateware",
                           "flash_kb", "sig_policy", "lan_policy", "sensors_digital", "nrst_pin",
                           "board_rev", "safe_mode", "safe_reason", "last_crash", "reset",
                           "heap_min", "stack_min", "c3_restarts", "short_form", "boot_health_cut",
                           "sig_keys", "cloud_ca", "cloud_proxy", "psram_ok" };
    for (size_t i = 0; i < sizeof(keep) / sizeof(keep[0]); i++)
        CHECK(has(keep[i]), "cloud status lacks %s", keep[i]);
    /* And what moved is not. */
    const char *drop[] = { "lan_tls", "stacks", "lwip_mem_max", "pbuf_pool_max", "malloc_failures",
                           "adc_cal_a_uv", "module", "lease",
                           "rx_dropped", "board_rev_mv", "sensors_format", "ota_sig",
                           "sig_policy_cmd", "lan_policy_cmd", "tunnel_max_tier", "lease_state" };
    for (size_t i = 0; i < sizeof(drop) / sizeof(drop[0]); i++)
        CHECK(!has(drop[i]), "cloud status still has %s", drop[i]);
    CHECK(strstr(g_tx, "\"sensor_la_trigger\"") != NULL, "caps cut short");
    /* The cut texts carry no JSON escapes. */
    const char *lc = strstr(g_tx, "\"last_crash\":\"");
    CHECK(lc && strchr(lc + 14, '"') - (lc + 14) == (long)STATUS_SHORT_TEXT_MAX, "last_crash not cut to %u",
          STATUS_SHORT_TEXT_MAX);
}

/* A pod as it runs (short crash text, no safe mode): the LAN gets the whole full form when the
   connection takes it. */
static void test_lan_is_complete(void) {
    g_usb_on = true;
    g_module_board = true;
    char saved_c = g_crash[40], saved_r = g_reason[0];
    g_crash[40] = '\0'; g_reason[0] = '\0';
    g_typical = true;
    g_ring = sizeof(g_tx) - 1;
    size_t n = run(0);
    printf("  LAN status, typical: %zu bytes\n", n);
    CHECK(g_sends == 1, "full form not sent at once (%d sends)", g_sends);
    const char *keep[] = { "lan_tls", "stacks", "lwip_mem_max", "malloc_failures", "adc_cal_a_uv",
                           "module", "caps", "usb", "lease", "rx_dropped",
                           "board_rev_mv", "sensors_format", "ota_sig", "tunnel_max_tier" };
    for (size_t i = 0; i < sizeof(keep) / sizeof(keep[0]); i++)
        CHECK(has(keep[i]), "LAN status lacks %s", keep[i]);
    CHECK(!has("short_form") && !has("boot_health_cut"), "LAN status is the short form");
    /* The crash text whole, escaped. */
    CHECK(strstr(g_tx, "\\\"x") != NULL, "LAN last_crash not escaped whole");
    g_crash[40] = saved_c; g_reason[0] = saved_r;
    g_typical = false;
    g_ring = 2047;
}

/* The worst case is more than the LAN TX ring takes whole: the short form goes out instead of an
   error, and so it does when the ring has less room. */
static void test_lan_falls_back_to_short_form(void) {
    /* The worst case does not fit the full form's buffer. */
    g_usb_on = false;
    size_t n = run(0);
    printf("  LAN status, worst case: short form %zu bytes\n", n);
    CHECK(has("short_form") && has("caps") && !has("lan_tls"), "worst case is not the short form");
    /* A full form the ring does not take whole (2151 B > 2047): refused, then the short form. */
    g_typical = true;
    char saved_r = g_reason[0];
    g_crash[40] = '\0'; g_reason[0] = '\0';
    run(0);
    CHECK(g_sends == 2, "expected the full form refused, then the short form (%d sends)", g_sends);
    CHECK(has("short_form") && g_err[0] == '\0', "no short form after the ring refused the full one");
    g_crash[40] = 'x'; g_reason[0] = saved_r;
    g_typical = false;
}

static void test_cloud_short_texts_not_marked(void) {
    /* Short texts are not cut and not marked. */
    char saved_c = g_crash[10], saved_r = g_reason[10];
    g_crash[10] = '\0'; g_reason[10] = '\0';
    run(CH_CLOUD_CONN);
    CHECK(!has("boot_health_cut"), "short texts marked cut");
    g_crash[10] = saved_c; g_reason[10] = saved_r;
}

int main(void) {
    lwip_stats.mem.max = 65535; lwip_stats.mem.avail = 65535;
    for (int i = 0; i < MEMP_MAX; i++) lwip_stats.memp[i] = &g_pbuf_stats;
    fill_text(g_crash, sizeof(g_crash));
    fill_text(g_reason, sizeof(g_reason));
    memset(g_holder, 'h', sizeof(g_holder) - 1);
    test_cloud_worst_case_fits();
    test_lan_is_complete();
    test_lan_falls_back_to_short_form();
    test_cloud_short_texts_not_marked();
    if (fails) { printf("test_ch_status: %d FAILED\n", fails); return 1; }
    printf("test_ch_status: all passed\n");
    return 0;
}
