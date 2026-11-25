/*
 * test_ch_dap.c — host tests for the PROTO_DAP raw mode (src/command_handler_dap.c): frame
 * reassembly across segments, several frames per segment, the [len][packet] response framing,
 * leaving on a zero-length or over-long frame (and handing back the bytes after it), the
 * send-room stall that drops a peer which stopped reading, and the teardown.
 *
 * dap_start with JTAG pins: the engine armed for JTAG, the ports handed to dap.c, SWD + JTAG
 * together, and the refusals on gateware without the JTAG master.
 *
 * dap_process is a fake that echoes the request with its first byte inverted; the FPGA, the
 * send ring and the clock are fakes too.
 */
#include "bp_errors.h"
#include <stdio.h>
#include <string.h>

#include "command_handler_internal.h"
#include "at_driver.h"
#include "signal_engine.h"
#include "dap.h"
#include "swd_ll.h"
#include "jtag_ll.h"
#include "gw_caps.h"
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
bool prog_flash_busy(void) { return false; }   /* no prog_flash job in these tests */
void dap_configure(unsigned size, unsigned count) { (void)size; (void)count; }
static uint8_t g_ports, g_port;
static dap_port_switch_fn g_switch;
void dap_set_ports(uint8_t ports, uint8_t active, dap_port_switch_fn sw) { g_ports = ports; g_port = active; g_switch = sw; }
int  fpga_swd_arm(unsigned clk, unsigned dio) { (void)clk; (void)dio; g_armed = true; return 0; }
void fpga_swd_disarm(void) { g_armed = false; g_disarms++; }
bool fpga_swd_armed(void) { return g_armed; }
void fpga_swd_poll(void) {}
bool fpga_spi_armed(void) { return false; }
bool fpga_i2c_armed(void) { return false; }
static bool     g_jarmed;
static unsigned g_jpins[4];
int  fpga_jtag_arm(unsigned tck, unsigned tms, unsigned tdi, unsigned tdo, unsigned half) {
    (void)half;
    if (g_armed || g_jarmed) return -2;
    g_jpins[0] = tck; g_jpins[1] = tms; g_jpins[2] = tdi; g_jpins[3] = tdo;
    g_jarmed = true; return 0;
}
void fpga_jtag_disarm(void) { g_jarmed = false; }
bool fpga_jtag_armed(void) { return g_jarmed; }
static int g_trst_level = -1;
int  fpga_la_set(unsigned ch, int mode) { (void)ch; g_trst_level = mode; return 0; }
static unsigned g_trst_ch;
void jtag_ll_set_trst_channel(unsigned ch) { g_trst_ch = ch; }
static uint8_t g_jtdo[32];   /* what a JTAG scan captures: zeros unless a test sets a chain */
int  jtag_ll_run(const uint8_t *seq, size_t len, uint8_t *tdo, size_t cap) {
    (void)seq; (void)len; memset(tdo, 0, cap);
    memcpy(tdo, g_jtdo, cap < sizeof(g_jtdo) ? cap : sizeof(g_jtdo));
    return 0;
}
static gw_caps_t g_caps;
const gw_caps_t *gw_caps(void) { return &g_caps; }
static int g_image = 0;
int signal_engine_image_kind(void) { return g_image; }
/* require_bus_masters (command_handler_device.c): 0 = the image has them, 1 = refuse with BP0059
   (sensors run), 2 = swap to the loop image (g_image becomes 0) */
static int g_bm_mode, g_bm_calls;
static char g_bm_feature[16];
bool require_bus_masters(int c, const char *feature) {
    g_bm_calls++;
    snprintf(g_bm_feature, sizeof(g_bm_feature), "%s", feature);
    if (g_bm_mode == 1) { bp_err_fpga_image_sensors_first(c, feature, "loop"); return false; }
    if (g_bm_mode == 2) { g_image = 0; g_caps.jtag = g_caps.jtag_gw; }
    return true;
}
void swd_ll_reset(void) {}
uint8_t swd_ll_transfer(uint8_t req, uint32_t *data) { (void)req; *data = 0; return SWD_ACK_OK; }
void swd_ll_seq_out(const uint8_t *d, uint32_t nbits) { (void)d; (void)nbits; }
uint16_t la_pins_mask_fn(la_fn_t fn) { (void)fn; return 0; }
uint16_t la_pins_release_fn(la_fn_t fn) { (void)fn; return 0; }
void la_pins_claim(la_fn_t fn, la_gpio_mode_t mode, uint8_t level, const uint8_t *las, size_t n) {
    (void)fn; (void)mode; (void)level; (void)las; (void)n;
}
bool la_claim_or_error(int c, la_fn_t fn, const uint8_t *las, size_t n, uint32_t free_fns) {
    (void)c; (void)fn; (void)las; (void)n; (void)free_fns; return true;
}
bool require_la_voltage(int c) { (void)c; return true; }
static char g_reply[256];
void send_ok_str(int c, const char *p) { (void)c; snprintf(g_reply, sizeof(g_reply), "ok:%s", p); }
static char g_large[640];
void send_ok_large(int c, const char *p) { (void)c; snprintf(g_large, sizeof(g_large), "%s", p); snprintf(g_reply, sizeof(g_reply), "ok:large"); }
static int g_ecode;   /* the code of the last coded refusal, 0 = a plain one */
void send_error(int c, const char *m) { (void)c; g_ecode = 0; snprintf(g_reply, sizeof(g_reply), "err:%s", m); }
void bp_error_send(int c, bp_errcode_t code, const bp_err_arg_t *args, size_t n) {
    (void)c;   /* "err:" and the code's catalog message, like a plain refusal reads here */
    g_ecode = (int)code;
    memcpy(g_reply, "err:", 4);
    bp_err_render(g_reply + 4, sizeof(g_reply) - 4, code, args, n);
}

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

static void jtag_start(const char *json) {
    dap_conn_closed(0);
    g_proto[0] = PROTO_JSON;
    g_reply[0] = 0; g_ecode = 0;
    handle_dap_start(0, json);
}

static void test_start_jtag(void) {
    g_caps.jtag_gw = true; g_caps.jtag = true; g_image = 0;
    jtag_start("{\"cmd\":\"dap_start\",\"tck\":3,\"tms\":4,\"tdi\":5,\"tdo\":6,\"trst\":7}");
    CHECK(strcmp(g_reply, "ok:\"dap ready\"") == 0, "reply %s", g_reply);
    CHECK(g_jarmed && !g_armed, "JTAG not armed");
    CHECK(g_jpins[0] == 3 && g_jpins[1] == 4 && g_jpins[2] == 5 && g_jpins[3] == 6, "JTAG pins");
    CHECK(g_ports == DAP_PORTS_JTAG && g_port == DAP_PORT_JTAG, "ports %u/%u", g_ports, g_port);
    CHECK(g_trst_ch == 7 && g_trst_level == 1, "nTRST channel %u level %d", g_trst_ch, g_trst_level);
    dap_conn_closed(0);
    CHECK(!g_jarmed && g_trst_ch == 0 && g_trst_level == 2, "teardown: JTAG disarmed, nTRST released");

    /* both wired: SWD first unless port says jtag; DAP_Connect switches through the callback */
    jtag_start("{\"cmd\":\"dap_start\",\"swclk\":1,\"swdio\":2,\"tck\":1,\"tms\":2,\"tdi\":5,\"tdo\":6}");
    CHECK(g_armed && !g_jarmed, "SWD first");
    CHECK(g_ports == (DAP_PORTS_SWD | DAP_PORTS_JTAG) && g_port == DAP_PORT_SWD, "both ports");
    CHECK(g_switch && g_switch(DAP_PORT_JTAG) && g_jarmed && !g_armed, "switch to JTAG");
    CHECK(g_switch(DAP_PORT_SWD) && g_armed && !g_jarmed, "switch back to SWD");
    dap_conn_closed(0);
    jtag_start("{\"cmd\":\"dap_start\",\"swclk\":1,\"swdio\":2,\"tck\":1,\"tms\":2,\"tdi\":5,\"tdo\":6,\"port\":\"jtag\"}");
    CHECK(g_jarmed && !g_armed && g_port == DAP_PORT_JTAG, "port jtag arms JTAG first");
    dap_conn_closed(0);

    /* incomplete JTAG pins */
    jtag_start("{\"cmd\":\"dap_start\",\"tck\":3,\"tms\":4}");
    CHECK(strstr(g_reply, "tck, tms, tdi and tdo") != NULL, "reply %s", g_reply);
    CHECK(!g_jarmed, "armed on an incomplete set");

    /* gateware without JTAG: too old, or the emulation image */
    g_caps.jtag_gw = false; g_caps.jtag = false;
    jtag_start("{\"cmd\":\"dap_start\",\"tck\":3,\"tms\":4,\"tdi\":5,\"tdo\":6}");
    CHECK(g_ecode == BP_E_GATEWARE_TOO_OLD, "code %d reply %s", g_ecode, g_reply);
    g_caps.jtag_gw = true; g_image = 2;
    jtag_start("{\"cmd\":\"dap_start\",\"tck\":3,\"tms\":4,\"tdi\":5,\"tdo\":6}");
    CHECK(g_ecode == BP_E_FPGA_IMAGE_SENSORS_FIRST, "code %d reply %s", g_ecode, g_reply);
    /* ... while SWD is wired too: SWD only */
    jtag_start("{\"cmd\":\"dap_start\",\"swclk\":1,\"swdio\":2,\"tck\":1,\"tms\":2,\"tdi\":5,\"tdo\":6}");
    CHECK(g_armed && !g_jarmed && g_ports == DAP_PORTS_SWD, "SWD only without the JTAG master");
    dap_conn_closed(0);
    g_image = 0;
}

/* Gateware v65: the emulation image has no SWD/JTAG engine.  dap_start asks for the bus masters
   before it claims a pin or arms anything. */
static void test_bus_masters(void) {
    g_caps.jtag_gw = true; g_caps.jtag = false; g_image = 2;
    g_bm_mode = 1; g_bm_calls = 0;
    dap_conn_closed(0);
    g_proto[0] = PROTO_JSON;
    handle_dap_start(0, "{\"cmd\":\"dap_start\",\"swclk\":1,\"swdio\":2}");
    CHECK(g_ecode == BP_E_FPGA_IMAGE_SENSORS_FIRST && g_bm_calls == 1, "code %d reply %s", g_ecode, g_reply);
    CHECK(strcmp(g_bm_feature, "SWD") == 0, "feature %s", g_bm_feature);
    CHECK(!g_armed && g_proto[0] == PROTO_JSON, "armed while sensors run");
    jtag_start("{\"cmd\":\"dap_start\",\"tck\":3,\"tms\":4,\"tdi\":5,\"tdo\":6}");
    CHECK(strcmp(g_bm_feature, "JTAG") == 0 && !g_jarmed, "JTAG refused, feature %s", g_bm_feature);
    /* no sensors: the swap to the loop image runs, then JTAG arms */
    g_bm_mode = 2;
    jtag_start("{\"cmd\":\"dap_start\",\"tck\":3,\"tms\":4,\"tdi\":5,\"tdo\":6}");
    CHECK(g_image == 0 && g_jarmed, "JTAG after the swap: reply %s", g_reply);
    dap_conn_closed(0);
    g_bm_mode = 0; g_caps.jtag = true; g_image = 0;
}

/* "op":"id": one JSON object back, the pins released, the connection left in JSON. */
static void test_id(void) {
    dap_conn_closed(0);
    g_proto[0] = PROTO_JSON; g_caps.jtag_gw = true; g_caps.jtag = true; g_image = 0;
    g_disarms = 0; g_large[0] = 0;
    handle_dap_start(0, "{\"cmd\":\"dap_start\",\"op\":\"id\",\"swclk\":11,\"swdio\":12,\"wait_ms\":50}");
    CHECK(strcmp(g_reply, "ok:large") == 0, "reply %s", g_reply);
    CHECK(strcmp(g_large, "{\"port\":\"swd\",\"found\":false}") == 0, "swd reply %s", g_large);
    CHECK(g_proto[0] == PROTO_JSON && !g_armed && g_disarms == 1, "id left the engine armed or switched protocol");

    /* both port sets: swd by default, "port":"jtag" picks the chain */
    memset(g_jtdo, 0xFF, sizeof(g_jtdo));
    g_jtdo[0] = 0x77; g_jtdo[1] = 0x04; g_jtdo[2] = 0xA0; g_jtdo[3] = 0x4B;   /* 0x4ba00477 */
    handle_dap_start(0, "{\"cmd\":\"dap_start\",\"op\":\"id\",\"port\":\"jtag\",\"swclk\":11,\"swdio\":12,"
                        "\"tck\":11,\"tms\":12,\"tdi\":9,\"tdo\":10}");
    CHECK(strcmp(g_large, "{\"port\":\"jtag\",\"found\":true,\"idcodes\":[\"0x4ba00477\"]}") == 0, "jtag reply %s", g_large);
    CHECK(!g_jarmed && !g_armed, "id left JTAG armed");
    handle_dap_start(0, "{\"cmd\":\"dap_start\",\"op\":\"id\",\"swclk\":11,\"swdio\":12,\"tck\":11,\"tms\":12,\"tdi\":9,\"tdo\":10,\"wait_ms\":1}");
    CHECK(strstr(g_large, "\"port\":\"swd\"") != NULL, "default port is swd: %s", g_large);
    memset(g_jtdo, 0, sizeof(g_jtdo));

    /* refusals use dap_start's own gates; bad requests arm nothing */
    handle_dap_start(0, "{\"cmd\":\"dap_start\",\"op\":\"id\",\"swclk\":11}");
    CHECK(strcmp(g_reply, "err:missing swdio") == 0, "reply %s", g_reply);
    handle_dap_start(0, "{\"cmd\":\"dap_start\",\"op\":\"id\",\"swclk\":11,\"swdio\":12,\"read\":[1,2,3,4,5,6,7,8,9]}");
    CHECK(strncmp(g_reply, "err:read takes", 14) == 0 && !g_armed, "nine reads: %s", g_reply);
    handle_dap_start(0, "{\"cmd\":\"dap_start\",\"op\":\"bogus\",\"swclk\":11,\"swdio\":12}");
    CHECK(strncmp(g_reply, "err:dap_start op", 16) == 0 && !g_armed, "bad op: %s", g_reply);
    g_bm_mode = 1;
    handle_dap_start(0, "{\"cmd\":\"dap_start\",\"op\":\"id\",\"swclk\":11,\"swdio\":12}");
    CHECK(strncmp(g_reply, "err:", 4) == 0 && !g_armed, "bus-master gate: %s", g_reply);
    g_bm_mode = 0;
    /* op:"start" is the plain command */
    handle_dap_start(0, "{\"cmd\":\"dap_start\",\"op\":\"start\",\"swclk\":11,\"swdio\":12}");
    CHECK(strcmp(g_reply, "ok:\"dap ready\"") == 0 && g_proto[0] == PROTO_DAP, "op start: %s", g_reply);
    dap_conn_closed(0);
}

int main(void) {
    test_start();
    test_bus_masters();
    test_start_jtag();
    test_reassembly();
    test_leave();
    test_stall();
    test_teardown_resets_reassembly();
    test_id();
    if (fails) { printf("test_ch_dap: %d FAILED\n", fails); return 1; }
    printf("test_ch_dap: all passed\n");
    return 0;
}
