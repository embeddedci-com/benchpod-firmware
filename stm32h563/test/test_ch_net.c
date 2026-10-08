/*
 * test_ch_net.c — host tests for src/command_handler_net.c: the speed test (the PROTO_SPEEDTEST
 * download sink with its ack cadence and done mark, the paced upload source) and the
 * provisioning replies (cloud_set / wifi_set on the real config stores over the RAM flash model,
 * their refusals, and the eth argument checks).
 */
#include <stdio.h>
#include <string.h>

#include "command_handler_internal.h"
#include "at_driver.h"
#include "cloud_config.h"
#include "config_store.h"
#include "cloud_client.h"
#include "net_server.h"
#include "esp_wifi_ctrl.h"
#include "esp_hosted_spi.h"
#include "pico_compat.h"
#include "mocks/mock_flash.h"

static int fails;
#define CHECK(cond, ...) do { if (!(cond)) { printf("FAIL %s:%d: ", __func__, __LINE__); \
        printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

/* ---- fakes ----------------------------------------------------------------------------- */
static proto_t g_proto[CH_CLOUD_TUNNEL_CONN_LAST + 1];
void conn_proto_set(int c, proto_t p) { g_proto[c] = p; }

static char   g_tx[4096];       /* everything sent on the conn, raw */
static size_t g_ntx, g_avail = 1u << 20, g_bytes;
static int    g_closed = -1;
int at_send_data(int c, const uint8_t *b, size_t n) {
    (void)c;
    g_bytes += n;
    size_t room = sizeof(g_tx) - 1 - g_ntx;
    if (n > room) n = room;
    memcpy(g_tx + g_ntx, b, n); g_ntx += n; g_tx[g_ntx] = '\0';
    return 0;
}
size_t at_send_avail(int c) { (void)c; return g_avail; }
int at_close_connection(int c) { g_closed = c; return 0; }
void cloud_send_json_line(int c, const char *json) {
    char line[96];
    int n = snprintf(line, sizeof(line), "%s\n", json);
    at_send_data(c, (const uint8_t *)line, (size_t)n);
}
static char g_reply[512];
void send_ok_str(int c, const char *p) { (void)c; snprintf(g_reply, sizeof(g_reply), "ok:%s", p); }
void send_error(int c, const char *m) { (void)c; snprintf(g_reply, sizeof(g_reply), "err:%s", m); }

static int g_cloud_reloads, g_wifi_reloads, g_eth;
void net_cloud_reload_after_reply(void) { g_cloud_reloads++; }
void net_wifi_reload_after_reply(void) { g_wifi_reloads++; }
void net_eth_stop(void) { g_eth = 1; }
void net_eth_start(void) { g_eth = 2; }
void net_eth_restart(void) { g_eth = 3; }
void net_eth_force_speed(int mbit, int full) { g_eth = 100 + mbit + full; }
void net_eth_refclk_measure(void) {}
uint32_t net_eth_refclk_seq(void) { return 0; }
bool net_eth_refclk_result(uint32_t s, uint32_t *hz) { (void)s; *hz = 50000000u; return true; }
void net_eth_loopback(int mbit, uint32_t n) { (void)mbit; (void)n; }
uint32_t net_eth_loopback_seq(void) { return 0; }
bool net_eth_loopback_result(uint32_t s, eth_loopback_result_t *r) { (void)s; memset(r, 0, sizeof(*r)); return true; }
void net_eth_diag(eth_diag_t *d) { memset(d, 0, sizeof(*d)); }
bool clock_on_hsi(void) { return false; }
void sleep_ms(uint32_t ms) { (void)ms; }
const char *cloud_client_state_str(void) { return "disabled"; }
void cloud_client_last_error(char *out, size_t n) { snprintf(out, n, "a \"quoted\" error"); }
bool esp_wifi_ctrl_connected(void) { return false; }
const char *esp_wifi_ctrl_state_str(void) { return "idle"; }
void esp_wifi_ctrl_slave_lost_counts(uint32_t *a, uint32_t *b) { *a = 0; *b = 0; }
static esp_hosted_pump_stats_t g_stats;
const esp_hosted_pump_stats_t *esp_hosted_spi_stats(void) { return &g_stats; }

static void clear_tx(void) { g_ntx = 0; g_bytes = 0; g_tx[0] = '\0'; }

/* ---- speed test ------------------------------------------------------------------------ */
static void test_speedtest_down(void) {
    const int conn = CH_CLOUD_TUNNEL_CONN;
    clear_tx();
    handle_speedtest(conn, "{\"cmd\":\"speedtest\",\"dir\":\"down\",\"bytes\":20000}");
    CHECK(g_proto[conn] == PROTO_SPEEDTEST, "not in the sink mode");
    uint8_t chunk[3000] = {0};
    size_t total = 0;
    while (total < 18000) total += speedtest_receive(conn, chunk, sizeof(chunk));
    CHECK(strcmp(g_tx, "{\"speedtest\":\"ack\",\"bytes\":9000}\n{\"speedtest\":\"ack\",\"bytes\":18000}\n") == 0,
          "ack cadence: %s", g_tx);
    /* the last segment overshoots: only what is owed is taken, the rest is the next protocol's */
    size_t took = speedtest_receive(conn, chunk, sizeof(chunk));
    CHECK(took == 2000, "took %zu of the final segment, expected 2000", took);
    CHECK(g_proto[conn] == PROTO_JSON, "not back to JSON after the last byte");
    CHECK(strstr(g_tx, "{\"speedtest\":\"done\",\"bytes\":20000}\n") != NULL, "no done mark: %s", g_tx);
}

static void test_speedtest_up(void) {
    const int conn = CH_CLOUD_TUNNEL_CONN + 1;
    clear_tx();
    handle_speedtest(conn, "{\"cmd\":\"speedtest\",\"dir\":\"up\",\"bytes\":1000}");
    g_avail = 63;                       /* below the pump's floor: nothing goes out */
    speedtest_pump();
    CHECK(g_bytes == 0, "sent into a full ring");
    g_avail = 300;
    speedtest_pump();                   /* the ring never empties here: the pump keeps filling */
    CHECK(g_bytes >= 1000 && memcmp(g_tx, "ZZZZ", 4) == 0, "payload not sent (%zu)", g_bytes);
    CHECK(strstr(g_tx, "{\"speedtest\":\"done\",\"bytes\":1000}\n") != NULL, "no done mark after the payload");
    /* a closed conn cancels a running upload */
    clear_tx();
    handle_speedtest(conn, "{\"cmd\":\"speedtest\",\"bytes\":5000}");
    speedtest_conn_closed(conn);
    speedtest_pump();
    CHECK(g_bytes == 0, "a cancelled upload kept sending");
    g_avail = 1u << 20;
}

/* ---- provisioning ---------------------------------------------------------------------- */
static void test_cloud_set(void) {
    clear_tx();
    handle_cloud_set(0, "{\"cmd\":\"cloud_set\",\"device_id\":\"d1\"}");
    CHECK(strcmp(g_reply, "err:missing host") == 0, "%s", g_reply);
    handle_cloud_set(0, "{\"cmd\":\"cloud_set\",\"host\":\"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\",\"device_id\":\"d1\"}");
    CHECK(strcmp(g_reply, "err:host too long") == 0, "%s", g_reply);
    handle_cloud_set(0, "{\"cmd\":\"cloud_set\",\"host\":\"h\"}");
    CHECK(strcmp(g_reply, "err:missing device_id") == 0, "%s", g_reply);
    int reloads = g_cloud_reloads;
    handle_cloud_set(0, "{\"cmd\":\"cloud_set\",\"host\":\"api.example.com\",\"device_id\":\"pod-1\",\"tls\":true}");
    CHECK(strcmp(g_tx, "{\"status\":\"ok\",\"data\":{\"host\":\"api.example.com\",\"port\":443,\"tls\":true,"
                       "\"verify\":true,\"enabled\":true,\"device_id\":\"pod-1\"}}\n") == 0, "reply %s", g_tx);
    CHECK(g_cloud_reloads == reloads + 1, "no reload after the reply");
    cloud_config_t c;
    CHECK(cloud_config_load(&c) == 0 && strcmp(c.host, "api.example.com") == 0 && c.port == 443, "not saved");
    clear_tx();
    handle_cloud_status(0, "{}");
    CHECK(strstr(g_tx, "\"last_error\":\"a \\\"quoted\\\" error\"") != NULL &&
          strstr(g_tx, "\"configured\":true,\"host\":\"api.example.com\"") != NULL, "status %s", g_tx);
    handle_cloud_clear(0, "{}");
    CHECK(strcmp(g_reply, "ok:\"cleared\"") == 0 && cloud_config_load(&c) != 0, "clear");
}

static void test_wifi_set(void) {
    clear_tx();
    handle_wifi_set(0, "{\"cmd\":\"wifi_set\"}");
    CHECK(strcmp(g_reply, "err:missing ssid") == 0, "%s", g_reply);
    handle_wifi_set(0, "{\"cmd\":\"wifi_set\",\"ssid\":\"lab \\\"5G\\\"\",\"password\":\"pw\"}");
    CHECK(strcmp(g_tx, "{\"status\":\"ok\",\"data\":{\"ssid\":\"lab \\\"5G\\\"\"}}\n") == 0, "reply %s", g_tx);
    config_t cfg;
    CHECK(config_load(&cfg) == 0 && strcmp(cfg.ssid, "lab \"5G\"") == 0, "ssid not saved unescaped");
    handle_wifi_clear(0, "{}");
    CHECK(strcmp(g_reply, "ok:\"cleared\"") == 0, "%s", g_reply);
}

static void test_eth(void) {
    handle_eth(0, "{\"cmd\":\"eth\",\"action\":\"bogus\"}");
    CHECK(strcmp(g_reply, "err:eth action must be stop|start|restart|stats|speed|refclk|loopback") == 0, "%s", g_reply);
    handle_eth(0, "{\"cmd\":\"eth\"}");
    CHECK(strcmp(g_reply, "ok:\"restart\"") == 0 && g_eth == 3, "default action: %s", g_reply);
    handle_eth(0, "{\"cmd\":\"eth\",\"action\":\"speed\",\"mbit\":\"50\"}");
    CHECK(strcmp(g_reply, "err:eth speed mbit must be 0 (autoneg), 10 or 100") == 0, "%s", g_reply);
    handle_eth(0, "{\"cmd\":\"eth\",\"action\":\"speed\",\"mbit\":10,\"duplex\":\"full\"}");
    CHECK(strcmp(g_reply, "ok:{\"speed\":10,\"duplex\":\"full\"}") == 0 && g_eth == 111, "%s", g_reply);
    handle_eth(0, "{\"cmd\":\"eth\",\"action\":\"loopback\",\"mbit\":5}");
    CHECK(strcmp(g_reply, "err:eth loopback mbit must be 10 or 100") == 0, "%s", g_reply);
    handle_eth(0, "{\"cmd\":\"eth\",\"action\":\"refclk\"}");
    CHECK(strcmp(g_reply, "ok:{\"hz\":50000000,\"ppm\":0,\"on_hsi\":false}") == 0, "%s", g_reply);
}

int main(void) {
    mock_flash_reset();
    test_speedtest_down();
    test_speedtest_up();
    test_cloud_set();
    test_wifi_set();
    test_eth();
    if (fails) { printf("test_ch_net: %d FAILED\n", fails); return 1; }
    printf("test_ch_net: all passed\n");
    return 0;
}
