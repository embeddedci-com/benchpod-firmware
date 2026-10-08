/*
 * test_ch_uart.c — host tests for the UART proxy (src/command_handler_uart.c): client bytes go to
 * the DUT, DUT bytes come back only as far as the send ring takes them, the guard-timed "+++"
 * escape (and what does not count as one), the suspension during a load_bin upload, the RX
 * pull-up hold and its restore, the re-arm after a gateware reconfiguration, and the teardown.
 *
 * The soft UART, the pull-up expander, the send ring and the clock are fakes.
 */
#include <stdio.h>
#include <string.h>

#include "command_handler_internal.h"
#include "at_driver.h"
#include "signal_engine.h"
#include "i2c_bus.h"
#include "pico/time.h"

static int fails;
#define CHECK(cond, ...) do { if (!(cond)) { printf("FAIL %s:%d: ", __func__, __LINE__); \
        printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

/* ---- fakes ----------------------------------------------------------------------------- */
static uint64_t g_now_us = 10u * 1000000u;
uint64_t time_us_64(void) { return g_now_us; }
static void advance_ms(uint32_t ms) { g_now_us += (uint64_t)ms * 1000u; }

static proto_t g_proto[CH_CLOUD_TUNNEL_CONN_LAST + 1];
proto_t conn_proto(int c) { return g_proto[c]; }
void conn_proto_set(int c, proto_t p) { g_proto[c] = p; }

static char   g_tx[512];   /* to the client */
static size_t g_ntx, g_avail = 4096;
static int    g_closed = -1, g_nodelay = -1, g_load_owner = -1;
int at_send_data(int c, const uint8_t *b, size_t n) { (void)c; memcpy(g_tx + g_ntx, b, n); g_ntx += n; return 0; }
size_t at_send_avail(int c) { (void)c; return g_avail; }
int at_close_connection(int c) { g_closed = c; return 0; }
int at_set_tcp_nodelay(int c, bool en) { (void)c; g_nodelay = en; return 0; }
int load_bin_owner(void) { return g_load_owner; }

static char   g_dut_in[512];  /* client -> DUT */
static size_t g_ndut_in;
static char   g_fifo[512];    /* DUT -> pod */
static size_t g_nfifo;
static int    g_reads, g_configs, g_status_rc;
static bool   g_enabled;
int fpga_uart_config(unsigned rx, unsigned tx, uint32_t baud, bool en) { (void)rx; (void)tx; (void)baud; g_enabled = en; g_configs++; return 0; }
void fpga_uart_disable(void) { g_enabled = false; }
int fpga_uart_write(const uint8_t *d, size_t n) { memcpy(g_dut_in + g_ndut_in, d, n); g_ndut_in += n; return 0; }
void fpga_uart_tx_pump(void) {}
int fpga_uart_status(uint16_t *avail, uint8_t *flags) { (void)flags; *avail = 0; return g_status_rc; }
size_t fpga_uart_read(uint8_t *buf, size_t len) {
    g_reads++;
    size_t n = len < g_nfifo ? len : g_nfifo;
    memcpy(buf, g_fifo, n);
    memmove(g_fifo, g_fifo + n, g_nfifo - n);
    g_nfifo -= n;
    return n;
}
static bool g_pullup[15];
int  pca9555_set_la_pullup(uint8_t la, bool en) { g_pullup[la] = en; return 0; }
bool pca9555_la_pullup_enabled(uint8_t la) { return g_pullup[la]; }
bool la_pullups_available(void) { return true; }
la_pull_dir_t la_pull_dir(unsigned la) { return la <= 6 ? LA_PULL_UP : la <= 8 ? LA_PULL_DOWN : LA_PULL_NONE; }
uint16_t la_pins_release_fn(la_fn_t fn) { (void)fn; return 0; }
void la_pins_claim(la_fn_t fn, la_gpio_mode_t mode, uint8_t level, const uint8_t *las, size_t n) {
    (void)fn; (void)mode; (void)level; (void)las; (void)n;
}
bool la_claim_or_error(int c, la_fn_t fn, const uint8_t *las, size_t n, uint16_t free_fns) {
    (void)c; (void)fn; (void)las; (void)n; (void)free_fns; return true;
}
bool require_la_voltage(int c) { (void)c; return true; }
static char g_reply[256];
void send_ok_str(int c, const char *p) { (void)c; snprintf(g_reply, sizeof(g_reply), "ok:%s", p); }
void send_error(int c, const char *m) { (void)c; snprintf(g_reply, sizeof(g_reply), "err:%s", m); }

/* ---- helpers --------------------------------------------------------------------------- */
static void start(int conn, const char *json) {
    g_proto[conn] = PROTO_JSON;
    g_ntx = g_ndut_in = g_nfifo = 0; g_closed = g_nodelay = -1;
    handle_uart_proxy_start(conn, json);
}
static void client(int conn, const char *s) { uart_proxy_receive(conn, (const uint8_t *)s, strlen(s)); }
static void dut(const char *s) { memcpy(g_fifo + g_nfifo, s, strlen(s)); g_nfifo += strlen(s); }

/* ---- tests ----------------------------------------------------------------------------- */
static void test_start_and_forward(void) {
    start(0, "{\"cmd\":\"uart_proxy_start\",\"rx\":3,\"tx\":4,\"baud\":9600}");
    CHECK(strcmp(g_reply, "ok:\"uart ready\"") == 0, "reply %s", g_reply);
    CHECK(g_proto[0] == PROTO_UART && g_enabled && g_nodelay == 1, "not in UART mode");
    CHECK(g_pullup[3], "RX (LA3, pulls up) not held high");
    client(0, "hello\n");
    CHECK(g_ndut_in == 6 && memcmp(g_dut_in, "hello\n", 6) == 0, "client bytes not forwarded");
    dut("world");
    uart_proxy_poll();
    CHECK(g_ntx == 5 && memcmp(g_tx, "world", 5) == 0, "DUT bytes not drained");
    /* only what the ring takes whole leaves the FIFO */
    g_ntx = 0; g_avail = 3;
    dut("abcdef");
    uart_proxy_poll();
    CHECK(g_ntx == 3 && g_nfifo == 3, "read more than the ring takes (%zu sent, %zu left)", g_ntx, g_nfifo);
    g_avail = 4096;
    uart_proxy_poll();
    CHECK(g_ntx == 6 && memcmp(g_tx, "abcdef", 6) == 0, "the rest did not follow");
}

static void test_escape(void) {
    /* guard, "+++", guard: back to JSON on the same conn */
    advance_ms(1500);
    client(0, "+++");
    uart_proxy_poll();
    CHECK(g_proto[0] == PROTO_UART, "left before the trailing guard");
    advance_ms(999);
    uart_proxy_poll();
    CHECK(g_proto[0] == PROTO_UART, "left 1 ms early");
    advance_ms(2);
    uart_proxy_poll();
    CHECK(g_proto[0] == PROTO_UNKNOWN && !g_enabled && g_nodelay == 0, "escape did not return to JSON");
    CHECK(!g_pullup[3], "RX pull-up not restored");
    CHECK(g_ndut_in >= 3 && memcmp(g_dut_in + g_ndut_in - 3, "+++", 3) == 0, "+++ is forwarded too");
}

static void test_no_escape(void) {
    start(1, "{\"cmd\":\"uart_proxy_start\",\"rx\":9,\"tx\":10}");
    /* no leading guard: "+++" right after other bytes is data */
    client(1, "x");
    advance_ms(10);
    client(1, "+++");
    advance_ms(1500);
    uart_proxy_poll();
    CHECK(g_proto[1] == PROTO_UART, "+++ without the leading guard escaped");
    /* a byte after the "+++" cancels it */
    advance_ms(1500);
    client(1, "+++");
    advance_ms(200);
    client(1, "y");
    advance_ms(1500);
    uart_proxy_poll();
    CHECK(g_proto[1] == PROTO_UART, "a byte after +++ did not cancel the escape");
    /* "++" then "+" in separate segments still counts */
    advance_ms(1500);
    client(1, "++");
    advance_ms(100);
    client(1, "+");
    advance_ms(1001);
    uart_proxy_poll();
    CHECK(g_proto[1] == PROTO_UNKNOWN, "split +++ did not escape");
}

static void test_suspend_for_upload(void) {
    start(2, "{\"cmd\":\"uart_proxy_start\",\"rx\":1,\"tx\":2}");
    g_load_owner = 0;
    dut("stale");
    int reads = g_reads;
    uart_proxy_poll();
    uart_proxy_poll();
    CHECK(g_reads == reads && g_ntx == 0, "the FIFO was touched during a load_bin upload");
    g_load_owner = -1;
    uart_proxy_poll();   /* resume: one flush of what piled up, then fresh output */
    CHECK(g_ntx == 0 && g_nfifo == 0, "stale bytes forwarded after the upload");
    dut("new");
    uart_proxy_poll();
    CHECK(g_ntx == 3 && memcmp(g_tx, "new", 3) == 0, "fresh output not forwarded");
}

static void test_teardown_and_rearm(void) {
    /* re-arm after a reconfiguration keeps the session */
    int configs = g_configs;
    uart_proxy_on_gateware_reconfigured();
    uart_rearm_poll();
    CHECK(g_configs == configs + 1 && g_enabled && g_proto[2] == PROTO_UART, "session not re-armed");
    /* a re-arm that fails closes the connection */
    g_status_rc = -1;
    uart_proxy_on_gateware_reconfigured();
    uart_rearm_poll();
    CHECK(g_closed == 2 && !g_enabled, "failed re-arm did not close the conn");
    g_status_rc = 0;
    /* socket close ends the proxy and restores the pull-up */
    start(3, "{\"cmd\":\"uart_proxy_start\",\"rx\":5,\"tx\":6}");
    CHECK(g_pullup[5], "RX LA5 not held");
    uart_proxy_conn_closed(3);
    CHECK(!g_enabled && !g_pullup[5], "close did not end the proxy");
    /* a re-arm queued for a session that ended meanwhile does nothing */
    configs = g_configs;
    uart_proxy_on_gateware_reconfigured();
    uart_rearm_poll();
    CHECK(g_configs == configs, "re-armed a closed session");
    /* an FPGA that does not answer refuses the start */
    g_status_rc = -1;
    start(4, "{\"cmd\":\"uart_proxy_start\",\"rx\":1,\"tx\":2}");
    CHECK(strncmp(g_reply, "err:uart proxy failed", 21) == 0 && g_proto[4] == PROTO_JSON, "reply %s", g_reply);
    g_status_rc = 0;
}

int main(void) {
    test_start_and_forward();
    test_escape();
    test_no_escape();
    test_suspend_for_upload();
    test_teardown_and_rearm();
    if (fails) { printf("test_ch_uart: %d FAILED\n", fails); return 1; }
    printf("test_ch_uart: all passed\n");
    return 0;
}
