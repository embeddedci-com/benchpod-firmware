/*
 * test_ch_uart.c — host tests for the UART proxy (src/command_handler_uart.c): client bytes go to
 * the DUT, DUT bytes come back only as far as the send ring takes them, the guard-timed "+++"
 * escape (and what does not count as one), the suspension during a load_bin upload, the RX
 * pull-up hold and its restore, the re-arm after a gateware reconfiguration, and the teardown.
 * The frame format (parity, stop_bits): parsed, refused on old gateware, kept across a re-arm.
 * Half duplex (one wire) and the boot-mode entry: the sequence and its order, and the refusals.
 *
 * The soft UART, the pull-up expander, the send ring and the clock are fakes.
 */
#include <stdio.h>
#include <string.h>

#include "command_handler_internal.h"
#include "board_caps.h"
#include "board_rev.h"
#include "at_driver.h"
#include "signal_engine.h"
#include "fake_pull.h"
#include "pico/time.h"
#include "bp_errors.h"

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
static uint8_t g_frame = 0xFF, g_gw = 56;
/* The entry sequence's steps, in order, as text ("lo4 rst1 s10 rst0 s10 cfg la4z s10 w2"). */
static char g_seq[256];
static void seq(const char *fmt, unsigned v) {
    size_t n = strlen(g_seq);
    snprintf(g_seq + n, sizeof(g_seq) - n, n ? " " : "");
    n = strlen(g_seq);
    snprintf(g_seq + n, sizeof(g_seq) - n, fmt, v);
}
/* The pull layer is the real one (la_pull.c) over RAM backends (fake_pull.c). */
#define g_pullup_on(la) (fake_pull_dir[la] == LA_PULL_UP)
static bool g_pu_at_cfg;
static unsigned g_de;
static bool g_de_inv;
int fpga_uart_config_de(unsigned rx, unsigned tx, unsigned de, bool de_inv, uint32_t baud, uint8_t frame,
                        bool en) {
    (void)tx; (void)baud; g_enabled = en; g_frame = frame; g_configs++;
    g_de = de; g_de_inv = de_inv;
    g_pu_at_cfg = g_pullup_on(rx);
    seq("cfg%u", rx);
    return 0;
}
int fpga_la_set(unsigned ch, int mode) { seq(mode == 0 ? "lo%u" : mode == 2 ? "z%u" : "hi%u", ch); return 0; }
static bool g_nrst_ok = true;
bool nrst_ctrl_supported(void) { return g_nrst_ok; }
void nrst_ctrl_assert(bool a) { seq("rst%u", a); }
void sleep_ms(uint32_t ms) { seq("s%u", ms); }
static char g_reply[256];
static int g_err_code;
static char g_err_arg[32];
void bp_error_send(int c, bp_errcode_t code, const char *text, const bp_err_arg_t *args, size_t n) {
    (void)c;
    g_err_code = (int)code;
    g_err_arg[0] = 0;
    if (n) {
        if (args[0].is_str) snprintf(g_err_arg, sizeof(g_err_arg), "%s", args[0].s);
        else                snprintf(g_err_arg, sizeof(g_err_arg), "%ld", (long)args[0].i);
    }
    snprintf(g_reply, sizeof(g_reply), "err:%s", text ? text : "");
}
uint8_t signal_engine_fpga_version(void) { return g_gw; }
void fpga_uart_disable(void) { g_enabled = false; }
int fpga_uart_write(const uint8_t *d, size_t n) { memcpy(g_dut_in + g_ndut_in, d, n); g_ndut_in += n; seq("w%u", (unsigned)n); return 0; }
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
/* The pin table and the pull table are the real ones (la_pins.c): la_pull_supports says which
   channels pull up on the board under test. */
static int de_claimed(void) {
    uint16_t m = la_pins_mask_fn(LA_FN_UART_DE);
    for (int la = 1; la <= 14; la++) if (m & (1u << (la - 1))) return la;
    return -1;
}
bool la_claim_or_error(int c, la_fn_t fn, const uint8_t *las, size_t n, uint32_t free_fns) {
    (void)c; (void)fn; (void)las; (void)n; (void)free_fns; return true;
}
bool require_la_voltage(int c) { (void)c; return true; }
void send_ok_str(int c, const char *p) { (void)c; snprintf(g_reply, sizeof(g_reply), "ok:%s", p); }
void send_error(int c, const char *m) { (void)c; snprintf(g_reply, sizeof(g_reply), "err:%s", m); }

/* ---- helpers --------------------------------------------------------------------------- */
static void start(int conn, const char *json) {
    g_proto[conn] = PROTO_JSON;
    g_ntx = g_ndut_in = g_nfifo = 0; g_closed = g_nodelay = -1;
    g_seq[0] = 0; g_err_code = 0;
    handle_uart_proxy_start(conn, json);
}
static void client(int conn, const char *s) { uart_proxy_receive(conn, (const uint8_t *)s, strlen(s)); }
static void dut(const char *s) { memcpy(g_fifo + g_nfifo, s, strlen(s)); g_nfifo += strlen(s); }

/* ---- tests ----------------------------------------------------------------------------- */
static void test_start_and_forward(void) {
    start(0, "{\"cmd\":\"uart_proxy_start\",\"rx\":3,\"tx\":4,\"baud\":9600}");
    CHECK(strcmp(g_reply, "ok:\"uart ready\"") == 0, "reply %s", g_reply);
    CHECK(g_proto[0] == PROTO_UART && g_enabled && g_nodelay == 1, "not in UART mode");
    CHECK(g_pullup_on(3), "RX (LA3, pulls up) not held high");
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
    CHECK(!g_pullup_on(3), "RX pull-up not restored");
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
    CHECK(g_pullup_on(5), "RX LA5 not held");
    uart_proxy_conn_closed(3);
    CHECK(!g_enabled && !g_pullup_on(5), "close did not end the proxy");
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

static void test_frame(void) {
    start(5, "{\"cmd\":\"uart_proxy_start\",\"rx\":1,\"tx\":2}");
    CHECK(g_frame == 0, "default is not 8N1 (frame %u)", g_frame);
    uart_proxy_conn_closed(5);
    start(5, "{\"cmd\":\"uart_proxy_start\",\"rx\":1,\"tx\":2,\"baud\":19200,\"parity\":\"even\"}");
    CHECK(g_frame == UART_FRAME_PARITY && g_proto[5] == PROTO_UART, "8E1: frame %u reply %s", g_frame, g_reply);
    /* a re-arm after a gateware swap keeps the frame format */
    g_frame = 0xFF;
    uart_proxy_on_gateware_reconfigured();
    uart_rearm_poll();
    CHECK(g_frame == UART_FRAME_PARITY, "re-arm lost the frame format (%u)", g_frame);
    uart_proxy_conn_closed(5);
    start(5, "{\"cmd\":\"uart_proxy_start\",\"rx\":1,\"tx\":2,\"parity\":\"odd\",\"stop_bits\":2}");
    CHECK(g_frame == (UART_FRAME_PARITY | UART_FRAME_ODD | UART_FRAME_STOP2), "8O2: frame %u", g_frame);
    uart_proxy_conn_closed(5);
    start(5, "{\"cmd\":\"uart_proxy_start\",\"rx\":1,\"tx\":2,\"parity\":\"none\",\"stop_bits\":\"2\"}");
    CHECK(g_frame == UART_FRAME_STOP2, "8N2: frame %u", g_frame);
    uart_proxy_conn_closed(5);
    /* bad values are refused before anything is armed */
    const char *bad[] = {
        "{\"cmd\":\"uart_proxy_start\",\"rx\":1,\"tx\":2,\"parity\":\"mark\"}",
        "{\"cmd\":\"uart_proxy_start\",\"rx\":1,\"tx\":2,\"parity\":\"E\"}",
        "{\"cmd\":\"uart_proxy_start\",\"rx\":1,\"tx\":2,\"stop_bits\":3}",
        "{\"cmd\":\"uart_proxy_start\",\"rx\":1,\"tx\":2,\"stop_bits\":1.5}",
    };
    for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
        int configs = g_configs;
        start(5, bad[i]);
        CHECK(strncmp(g_reply, "err:invalid uart frame", 22) == 0 && g_configs == configs && g_proto[5] == PROTO_JSON,
              "bad[%zu] reply %s", i, g_reply);
    }
    /* gateware older than v56: 8N1 still works, anything else is refused with what to do */
    g_gw = 55;
    start(5, "{\"cmd\":\"uart_proxy_start\",\"rx\":1,\"tx\":2,\"parity\":\"even\"}");
    CHECK(strstr(g_reply, "err:uart parity and stop bits need gateware v56") == g_reply && g_proto[5] == PROTO_JSON,
          "old gateware reply %s", g_reply);
    CHECK(g_err_code == BP_E_GATEWARE_TOO_OLD && strcmp(g_err_arg, "parity and stop_bits") == 0,
          "old gateware code: %d %s", g_err_code, g_err_arg);
    start(5, "{\"cmd\":\"uart_proxy_start\",\"rx\":1,\"tx\":2,\"parity\":\"none\",\"stop_bits\":1}");
    CHECK(g_frame == 0 && g_proto[5] == PROTO_UART, "8N1 refused on old gateware: %s", g_reply);
    uart_proxy_conn_closed(5);
    g_gw = 56;
}

static void test_half_duplex(void) {
    g_gw = 57;
    fake_pull_reset();
    start(3, "{\"cmd\":\"uart_proxy_start\",\"rx\":3,\"tx\":3,\"half_duplex\":true}");
    CHECK(g_proto[3] == PROTO_UART && g_frame == UART_FRAME_HALFDUPLEX, "one wire: frame %u reply %s", g_frame, g_reply);
    CHECK(g_pullup_on(3) && g_pu_at_cfg, "the line's pull-up is not on before the UART arms");
    CHECK(g_ndut_in == 0 && strstr(g_seq, "rst") == NULL, "no entry asked, but: %s", g_seq);
    uart_proxy_conn_closed(3);
    CHECK(!g_pullup_on(3), "pull-up not restored");
    /* with parity (RL78 protocol B is 8O1 on one wire) */
    start(3, "{\"cmd\":\"uart_proxy_start\",\"rx\":3,\"tx\":3,\"half_duplex\":true,\"parity\":\"odd\"}");
    CHECK(g_frame == (UART_FRAME_HALFDUPLEX | UART_FRAME_PARITY | UART_FRAME_ODD), "8O1 one wire: frame %u", g_frame);
    /* the re-arm after a gateware swap keeps it */
    g_frame = 0;
    uart_proxy_on_gateware_reconfigured();
    uart_rearm_poll();
    CHECK(g_frame & UART_FRAME_HALFDUPLEX, "re-arm lost half duplex (%u)", g_frame);
    uart_proxy_conn_closed(3);
    /* refusals, nothing armed */
    int configs = g_configs;
    start(3, "{\"cmd\":\"uart_proxy_start\",\"rx\":3,\"tx\":4,\"half_duplex\":true}");
    CHECK(g_err_code == BP_E_UART_HALF_DUPLEX_TWO_PINS && strcmp(g_err_arg, "3") == 0, "two pins: %d %s", g_err_code, g_reply);
    start(3, "{\"cmd\":\"uart_proxy_start\",\"rx\":3,\"tx\":3}");
    CHECK(g_err_code == BP_E_UART_SAME_PIN, "same pin without half_duplex: %d %s", g_err_code, g_reply);
    g_gw = 56;
    start(3, "{\"cmd\":\"uart_proxy_start\",\"rx\":3,\"tx\":3,\"half_duplex\":true}");
    CHECK(g_err_code == BP_E_GATEWARE_TOO_OLD && strcmp(g_err_arg, "half_duplex") == 0, "old gateware: %d %s", g_err_code, g_reply);
    CHECK(g_configs == configs && g_proto[3] == PROTO_JSON, "a refusal armed the UART");
    /* half_duplex false is the old two-pin UART */
    start(3, "{\"cmd\":\"uart_proxy_start\",\"rx\":3,\"tx\":4,\"half_duplex\":false}");
    CHECK(g_frame == 0 && g_proto[3] == PROTO_UART, "half_duplex false: frame %u %s", g_frame, g_reply);
    uart_proxy_conn_closed(3);
}

/* Digital v3r4: the RX pull goes through the expanders.  A one-wire line asks for the 2.2k a
   half-duplex pin gets: it is engaged before the pin is claimed, so the default (0 = by the pin's
   function) would see an unclaimed pin and pick 10k. */
static void test_v3r4_pulls(void) {
    g_gw = 58;
    board_caps_init(BOARD_REV_V3R4);
    la_pull_init();
    fake_pull_reset();
        start(3, "{\"cmd\":\"uart_proxy_start\",\"rx\":3,\"tx\":3,\"half_duplex\":true}");
    CHECK(g_proto[3] == PROTO_UART, "v3r4 one wire: %s", g_reply);
    CHECK(g_pu_at_cfg && fake_pull_dir[3] == LA_PULL_UP && fake_pull_str[3] == PULL_STR_2K2,
          "v3r4 one wire: pull %d strength %d, want up 2.2k before the arm", (int)fake_pull_dir[3], fake_pull_str[3]);
    uart_proxy_conn_closed(3);
    CHECK(fake_pull_dir[3] == LA_PULL_NONE && !g_pullup_on(3), "v3r4 one wire: pull not restored (%d)", (int)fake_pull_dir[3]);

    start(3, "{\"cmd\":\"uart_proxy_start\",\"rx\":3,\"tx\":4}");
    CHECK(g_proto[3] == PROTO_UART && fake_pull_dir[3] == LA_PULL_UP && fake_pull_str[3] == PULL_STR_NONE,
          "v3r4 two-wire RX: pull %d strength %d, want up by function", (int)fake_pull_dir[3], fake_pull_str[3]);
    uart_proxy_conn_closed(3);
    /* what the channel did before comes back: a 2.2k pull-down, say */
    CHECK(la_pull_set(3, LA_PULL_DOWN, PULL_STR_2K2) == 0, "LA3 down");
    start(3, "{\"cmd\":\"uart_proxy_start\",\"rx\":3,\"tx\":4}");
    CHECK(fake_pull_dir[3] == LA_PULL_UP, "v3r4: RX not held up over a pull-down (%d)", (int)fake_pull_dir[3]);
    uart_proxy_conn_closed(3);
    CHECK(fake_pull_dir[3] == LA_PULL_DOWN && fake_pull_str[3] == PULL_STR_2K2,
          "v3r4: the earlier pull-down not restored (%d, %d)", (int)fake_pull_dir[3], (int)fake_pull_str[3]);
    la_pull_set(3, LA_PULL_NONE, PULL_STR_NONE);
    /* no pull can be engaged (pin 21 down, say): the proxy runs without the hold */
    fake_pull_available = false;
    start(3, "{\"cmd\":\"uart_proxy_start\",\"rx\":3,\"tx\":4}");
    CHECK(g_proto[3] == PROTO_UART && fake_pull_dir[3] == LA_PULL_NONE, "v3r4: hold without pulls: %s", g_reply);
    uart_proxy_conn_closed(3);
    fake_pull_available = true;
    board_caps_init(BOARD_REV_V3);
    la_pull_init();
    g_gw = 56;
}

static void test_entry(void) {
    g_gw = 57;
    /* RL78-style: reset with TOOL0 low, TOOL0 low after the release, released, then 0x00 */
    start(2, "{\"cmd\":\"uart_proxy_start\",\"rx\":2,\"tx\":2,\"half_duplex\":true,"
             "\"entry\":{\"reset_ms\":5,\"hold_ms\":3,\"idle_ms\":2,\"hex\":\"00\"}}");
    CHECK(strcmp(g_seq, "lo2 rst1 s5 rst0 s3 cfg2 z2 s2 w1") == 0, "sequence %s", g_seq);
    CHECK(g_pu_at_cfg, "pull-up off when TX was released");
    CHECK(g_ndut_in == 1 && g_dut_in[0] == 0, "first byte not sent");
    CHECK(strcmp(g_reply, "ok:\"uart ready\"") == 0 && g_proto[2] == PROTO_UART, "reply %s", g_reply);
    uart_proxy_conn_closed(2);
    /* defaults, several bytes, two pins, tx not held low */
    start(2, "{\"cmd\":\"uart_proxy_start\",\"rx\":1,\"tx\":2,"
             "\"entry\":{\"tx_low\":false,\"hex\":\"3a0Fff\"}}");
    CHECK(strcmp(g_seq, "rst1 s10 rst0 s10 cfg1 s10 w3") == 0, "sequence %s", g_seq);
    CHECK(g_ndut_in == 3 && (uint8_t)g_dut_in[0] == 0x3A && (uint8_t)g_dut_in[1] == 0x0F && (uint8_t)g_dut_in[2] == 0xFF,
          "bytes not sent as given");
    uart_proxy_conn_closed(2);
    /* an empty entry object runs the default sequence with no bytes */
    start(2, "{\"cmd\":\"uart_proxy_start\",\"rx\":1,\"tx\":2,\"entry\":{}}");
    CHECK(strcmp(g_seq, "lo2 rst1 s10 rst0 s10 cfg1 z2 s10") == 0, "sequence %s", g_seq);
    uart_proxy_conn_closed(2);
    /* refusals: nothing touched (no reset, no low, not armed) */
    const struct { const char *json; int code; const char *arg; } bad[] = {
        { "{\"cmd\":\"uart_proxy_start\",\"rx\":1,\"tx\":2,\"entry\":{\"reset_ms\":1001}}", BP_E_UART_ENTRY_INVALID, "reset_ms" },
        { "{\"cmd\":\"uart_proxy_start\",\"rx\":1,\"tx\":2,\"entry\":{\"idle_ms\":-1}}",  BP_E_UART_ENTRY_INVALID, "idle_ms" },
        { "{\"cmd\":\"uart_proxy_start\",\"rx\":1,\"tx\":2,\"entry\":{\"hold_ms\":\"x\"}}", BP_E_UART_ENTRY_INVALID, "hold_ms" },
        { "{\"cmd\":\"uart_proxy_start\",\"rx\":1,\"tx\":2,\"entry\":{\"hex\":\"0\"}}",    BP_E_UART_ENTRY_INVALID, "hex" },
        { "{\"cmd\":\"uart_proxy_start\",\"rx\":1,\"tx\":2,\"entry\":{\"hex\":\"zz\"}}",   BP_E_UART_ENTRY_INVALID, "hex" },
        { "{\"cmd\":\"uart_proxy_start\",\"rx\":1,\"tx\":2,\"entry\":{\"hex\":"
          "\"000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f20\"}}", BP_E_UART_ENTRY_INVALID, "hex" },
    };
    for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
        int configs = g_configs;
        start(2, bad[i].json);
        CHECK(g_err_code == bad[i].code && strcmp(g_err_arg, bad[i].arg) == 0 && g_configs == configs && g_seq[0] == 0,
              "bad[%zu]: code %d arg %s seq '%s' reply %s", i, g_err_code, g_err_arg, g_seq, g_reply);
    }
    /* 32 bytes is the limit, not more */
    start(2, "{\"cmd\":\"uart_proxy_start\",\"rx\":1,\"tx\":2,\"entry\":{\"hex\":"
             "\"000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f\"}}");
    CHECK(g_ndut_in == 32 && g_proto[2] == PROTO_UART, "32 bytes refused: %s", g_reply);
    uart_proxy_conn_closed(2);
    /* a pod without the reset pin */
    g_nrst_ok = false;
    start(2, "{\"cmd\":\"uart_proxy_start\",\"rx\":1,\"tx\":2,\"entry\":{}}");
    CHECK(g_err_code == BP_E_UART_ENTRY_NO_RESET_PIN && g_seq[0] == 0, "no reset pin: %d %s", g_err_code, g_seq);
    g_nrst_ok = true;
    g_gw = 56;
}

static void test_de(void) {
    g_gw = 58;
    start(3, "{\"cmd\":\"uart_proxy_start\",\"rx\":7,\"tx\":8,\"baud\":19200,\"parity\":\"even\",\"de\":6}");
    CHECK(g_proto[3] == PROTO_UART && g_de == 6 && !g_de_inv && g_frame == UART_FRAME_PARITY,
          "de: %u inv %d frame %u reply %s", g_de, g_de_inv, g_frame, g_reply);
    CHECK(de_claimed() == 6, "DE pin not claimed (%d)", de_claimed());
    /* a gateware swap re-arms with the same DE */
    g_de = 0;
    uart_proxy_on_gateware_reconfigured();
    uart_rearm_poll();
    CHECK(g_de == 6, "re-arm lost DE (%u)", g_de);
    uart_proxy_conn_closed(3);
    CHECK(de_claimed() == -1, "DE pin not released");
    start(3, "{\"cmd\":\"uart_proxy_start\",\"rx\":7,\"tx\":8,\"de\":9,\"de_active_low\":true}");
    CHECK(g_de == 9 && g_de_inv, "active low: de %u inv %d", g_de, g_de_inv);
    uart_proxy_conn_closed(3);
    /* without de: none */
    start(3, "{\"cmd\":\"uart_proxy_start\",\"rx\":7,\"tx\":8}");
    CHECK(g_de == 0 && de_claimed() == -1, "de without asking (%u)", g_de);
    uart_proxy_conn_closed(3);
    /* refusals, nothing armed */
    int configs = g_configs;
    start(3, "{\"cmd\":\"uart_proxy_start\",\"rx\":7,\"tx\":8,\"de\":8}");
    CHECK(g_err_code == BP_E_UART_DE_PIN && strcmp(g_err_arg, "8") == 0, "de on tx: %d %s", g_err_code, g_reply);
    start(3, "{\"cmd\":\"uart_proxy_start\",\"rx\":7,\"tx\":8,\"de\":7}");
    CHECK(g_err_code == BP_E_UART_DE_PIN, "de on rx: %d", g_err_code);
    g_gw = 57;
    start(3, "{\"cmd\":\"uart_proxy_start\",\"rx\":7,\"tx\":8,\"de\":6}");
    CHECK(g_err_code == BP_E_GATEWARE_TOO_OLD && strcmp(g_err_arg, "de") == 0, "old gateware: %d %s", g_err_code, g_err_arg);
    CHECK(g_configs == configs && g_proto[3] == PROTO_JSON, "a refusal armed the UART");
    g_gw = 56;
}

int main(void) {
    board_caps_init(BOARD_REV_V3);   /* LA1-6 pull up, LA7/8 down, LA9-14 none */
    la_pull_init();
    test_start_and_forward();
    test_escape();
    test_no_escape();
    test_suspend_for_upload();
    test_teardown_and_rearm();
    test_frame();
    test_half_duplex();
    test_v3r4_pulls();
    test_entry();
    test_de();
    if (fails) { printf("test_ch_uart: %d FAILED\n", fails); return 1; }
    printf("test_ch_uart: all passed\n");
    return 0;
}
