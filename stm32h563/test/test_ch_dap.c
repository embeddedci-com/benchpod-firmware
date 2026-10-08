/*
 * test_ch_dap.c — host tests for the PROTO_DAP raw mode (src/command_handler_dap.c): frame
 * reassembly across segments, several frames per segment, the [len][packet] response framing,
 * leaving on a zero-length or over-long frame (and handing back the bytes after it), the
 * send-room stall that drops a peer which stopped reading, and the teardown.
 *
 * dap_process is a fake that echoes the request with its first byte inverted; the FPGA, the
 * send ring and the clock are fakes too.
 */
#include <stdio.h>
#include <string.h>

#include "command_handler_internal.h"
#include "at_driver.h"
#include "signal_engine.h"
#include "dap.h"
#include "swd_ll.h"
#include "watchdog.h"
#include "hw_worker.h"
#include "pico_compat.h"

static int fails;
#define CHECK(cond, ...) do { if (!(cond)) { printf("FAIL %s:%d: ", __func__, __LINE__); \
        printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

/* ---- fakes ----------------------------------------------------------------------------- */
static uint64_t g_now_us;
uint64_t time_us_64(void) { return g_now_us; }
void sleep_ms(uint32_t ms) { g_now_us += (uint64_t)ms * 1000u; }
void watchdog_heartbeat(int task, const char *name) { (void)task; (void)name; }

static proto_t g_proto[CH_CLOUD_TUNNEL_CONN_LAST + 1];
proto_t conn_proto(int c) { return g_proto[c]; }
void conn_proto_set(int c, proto_t p) { g_proto[c] = p; }

static uint8_t g_tx[8192];
static size_t  g_ntx, g_avail = 1u << 20;
static int     g_closed = -1, g_reset = -1, g_nodelay = -1;
int at_send_data(int c, const uint8_t *b, size_t n) { (void)c; memcpy(g_tx + g_ntx, b, n); g_ntx += n; return 0; }
size_t at_send_avail(int c) { (void)c; return g_avail; }
int at_close_connection(int c) { g_closed = c; return 0; }
int at_set_tcp_nodelay(int c, bool en) { (void)c; g_nodelay = en; return 0; }
void hw_worker_submit_tunnel_reset(int c) { g_reset = c; }

static int  g_packets, g_disarms;
static bool g_armed;
size_t dap_process(const uint8_t *req, size_t n, uint8_t *resp, size_t cap) {
    (void)cap;
    memcpy(resp, req, n);
    resp[0] = (uint8_t)~req[0];
    g_packets++;
    return n;
}
void dap_reset(void) {}
void dap_configure(unsigned size, unsigned count) { (void)size; (void)count; }
int  fpga_swd_arm(unsigned clk, unsigned dio) { (void)clk; (void)dio; g_armed = true; return 0; }
void fpga_swd_disarm(void) { g_armed = false; g_disarms++; }
bool fpga_swd_armed(void) { return g_armed; }
void fpga_swd_poll(void) {}
bool fpga_spi_armed(void) { return false; }
uint8_t swd_ll_transfer(uint8_t req, uint32_t *data) { (void)req; *data = 0; return SWD_ACK_OK; }
void swd_ll_seq_out(const uint8_t *d, uint32_t nbits) { (void)d; (void)nbits; }
uint16_t la_pins_mask_fn(la_fn_t fn) { (void)fn; return 0; }
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
static size_t rx(int conn, const uint8_t *b, size_t n) { return dap_proxy_receive(conn, b, n); }
static size_t frame(uint8_t *out, const uint8_t *pkt, size_t n) {
    out[0] = (uint8_t)(n & 0xFF); out[1] = (uint8_t)(n >> 8);
    memcpy(out + 2, pkt, n);
    return n + 2;
}
static void start(int conn) {
    dap_conn_closed(conn);
    g_proto[conn] = PROTO_JSON;
    handle_dap_start(conn, "{\"cmd\":\"dap_start\",\"swclk\":1,\"swdio\":2}");
    g_ntx = 0; g_packets = 0; g_disarms = 0; g_closed = g_reset = g_nodelay = -1;
}

/* ---- tests ----------------------------------------------------------------------------- */
static void test_start(void) {
    dap_conn_closed(0);
    g_proto[0] = PROTO_JSON;
    handle_dap_start(0, "{\"cmd\":\"dap_start\",\"swclk\":1,\"swdio\":2}");
    CHECK(strcmp(g_reply, "ok:\"dap ready\"") == 0, "reply %s", g_reply);
    CHECK(g_proto[0] == PROTO_DAP && g_armed, "not in DAP mode after the ack");
}

static void test_reassembly(void) {
    uint8_t buf[64], pkt[5] = { 0x05, 1, 2, 3, 4 };
    start(0);
    size_t n = frame(buf, pkt, sizeof(pkt));
    /* one byte at a time: header split, payload split */
    for (size_t i = 0; i < n; i++) CHECK(rx(0, buf + i, 1) == 1, "byte %zu not taken", i);
    CHECK(g_packets == 1 && g_ntx == 7, "one packet, 7 response bytes (got %d, %zu)", g_packets, g_ntx);
    CHECK(g_tx[0] == 5 && g_tx[1] == 0 && g_tx[2] == 0xFA && g_tx[3] == 1 && g_tx[6] == 4, "response framing");
    /* two frames and half a third in one segment */
    g_ntx = 0;
    size_t m = frame(buf, pkt, 5);
    m += frame(buf + m, pkt, 3);
    size_t third = frame(buf + m, pkt, 4);
    CHECK(rx(0, buf, m + 3) == m + 3, "segment not taken whole");
    CHECK(g_packets == 3 && g_ntx == 7 + 5, "two packets run, the third waits (%d)", g_packets);
    CHECK(rx(0, buf + m + 3, third - 3) == third - 3 && g_packets == 4, "third frame completes");
    CHECK(g_proto[0] == PROTO_DAP && g_disarms == 0, "left DAP mode by itself");
}

static void test_leave(void) {
    uint8_t buf[32], pkt[2] = { 0x00, 0x00 };
    start(1);
    size_t n = frame(buf, pkt, 2);
    buf[n++] = 0; buf[n++] = 0;               /* zero-length frame: leave */
    memcpy(buf + n, "{\"cmd\"", 6);           /* JSON right behind it */
    size_t took = rx(1, buf, n + 6);
    CHECK(took == n, "took %zu, expected %zu (the JSON is the next protocol's)", took, n);
    CHECK(g_packets == 1 && g_disarms == 1 && !g_armed, "SWD not disarmed on leave");
    CHECK(g_proto[1] == PROTO_UNKNOWN, "protocol not reset to re-detect");
    CHECK(g_nodelay == 0, "Nagle not restored on a socket conn");
    /* an over-long declared length leaves too; a tunnel has no socket option to restore */
    start(CH_CLOUD_TUNNEL_CONN);
    uint8_t bad[2] = { 0xFF, 0xFF };
    CHECK(rx(CH_CLOUD_TUNNEL_CONN, bad, 2) == 2 && g_proto[CH_CLOUD_TUNNEL_CONN] == PROTO_UNKNOWN,
          "over-long frame did not leave");
    CHECK(g_nodelay == -1, "set tcp_nodelay on a tunnel");
}

static void test_stall(void) {
    uint8_t buf[16], pkt[3] = { 1, 2, 3 };
    start(2);
    g_avail = 10;                             /* never room for a whole DAP_PACKET_SIZE response */
    size_t n = frame(buf, pkt, 3);
    CHECK(rx(2, buf, n) == n, "stalled segment not consumed");
    CHECK(g_packets == 0, "ran a packet whose response could not be sent");
    CHECK(g_closed == 2 && g_reset == -1, "LAN conn not closed on a stall");
    CHECK(rx(2, buf, n) == n && g_packets == 0, "a dead conn's bytes were run");
    dap_conn_closed(2);
    CHECK(g_disarms == 1, "teardown did not disarm SWD");
    /* a tunnel asks for a reset instead */
    start(CH_CLOUD_TUNNEL_CONN + 1);
    CHECK(rx(CH_CLOUD_TUNNEL_CONN + 1, buf, n) == n && g_reset == CH_CLOUD_TUNNEL_CONN + 1 && g_closed == -1,
          "tunnel not reset on a stall");
    dap_conn_closed(CH_CLOUD_TUNNEL_CONN + 1);
    g_avail = 1u << 20;
    /* after the teardown the slot works again */
    start(2);
    CHECK(rx(2, buf, n) == n && g_packets == 1, "slot still dead after teardown");
}

static void test_teardown_resets_reassembly(void) {
    uint8_t buf[16], pkt[4] = { 9, 9, 9, 9 };
    start(3);
    size_t n = frame(buf, pkt, 4);
    rx(3, buf, 3);                            /* half a frame */
    dap_conn_closed(3);
    CHECK(g_disarms == 1, "teardown in DAP mode did not disarm");
    start(3);
    CHECK(rx(3, buf, n) == n && g_packets == 1 && g_ntx == n, "stale half frame survived teardown");
    /* a conn outside the state machine is ignored */
    g_disarms = 0;
    dap_conn_closed(CH_CLOUD_CONN);
    CHECK(g_disarms == 0, "teardown of a pseudo-conn touched SWD");
}

int main(void) {
    test_start();
    test_reassembly();
    test_leave();
    test_stall();
    test_teardown_resets_reassembly();
    if (fails) { printf("test_ch_dap: %d FAILED\n", fails); return 1; }
    printf("test_ch_dap: all passed\n");
    return 0;
}
