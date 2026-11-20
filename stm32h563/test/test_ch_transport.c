/*
 * test_ch_transport.c — host tests for the per-connection byte stream
 * (src/command_handler_transport.c): line assembly across segments, CRLF, JSON/SCPI detection,
 * the over-long line, the raw-mode handoff to a module's receive function and back, the SCPI
 * lease refusal, the one-shot path for the pseudo-conns, the teardown, and which protocol a TCP
 * connection may speak (SCPI on or off, JSON on plaintext LAN under HTTPS-only).
 *
 * The modules the transport hands bytes to are fakes here: each records what it got and takes
 * as many bytes as the test says.
 */
#include <stdio.h>
#include <string.h>

#include "command_handler_internal.h"
#include "bp_limits.h"
#include "scpi_server.h"
#include "lease_gate.h"
#include "cmd_gate.h"
#include "stm32h5xx_hal.h"
#include "pod_policy.h"
#include "bp_errors.h"

static int fails;
#define CHECK(cond, ...) do { if (!(cond)) { printf("FAIL %s:%d: ", __func__, __LINE__); \
        printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

/* ---- fakes ----------------------------------------------------------------------------- */
#define LOG_MAX 16
static char g_log[LOG_MAX][1400];   /* "json<conn>:<line>", "scpi<conn>:<line>", "err<conn>:<msg>" */
static int  g_nlog;
static void logf_(const char *kind, int conn, const char *s) {
    if (g_nlog < LOG_MAX) snprintf(g_log[g_nlog++], sizeof(g_log[0]), "%s%d:%s", kind, conn, s);
}
static void reset_log(void) { g_nlog = 0; }

void dispatch_line(int conn_id, const char *buf) { logf_("json", conn_id, buf); }
void scpi_dispatch_line(int conn_id, const char *line) { logf_("scpi", conn_id, line); }
void scpi_push_execution_error(void) { logf_("scpierr", 0, ""); }
void send_error(int conn_id, const char *message) { logf_("err", conn_id, message); }

static bool g_lease;
bool lease_gate_active(uint32_t now_ms, uint32_t *left_s) { (void)now_ms; if (left_s) *left_s = 0; return g_lease; }
bool cmd_gate_scpi_line_writes(const char *line) { return strchr(line, '?') == NULL; }
policy_src_t command_handler_policy_src(int conn_id) {
    return (conn_id >= CH_CLOUD_CONN) ? POLICY_SRC_CLOUD : POLICY_SRC_LAN;
}
uint32_t HAL_GetTick(void) { return 0; }

/* The pod policy and the connection's transport (net_server). */
static bool g_scpi = true, g_https_only;
static bool g_usb[CH_MAX_CONN], g_tls[CH_MAX_CONN];
static int  g_closed = -1;
bool pod_policy_scpi(void) { return g_scpi; }
bool pod_policy_https_only(void) { return g_https_only; }
bool net_conn_on_usb(int c) { return c >= 0 && c < CH_MAX_CONN && g_usb[c]; }
bool net_conn_on_tls(int c) { return c >= 0 && c < CH_MAX_CONN && g_tls[c]; }
static bool g_pairing, g_paired[CH_MAX_CONN];
bool pod_policy_lan_pairing(void) { return g_pairing; }
bool net_conn_lan_paired(int c) { return c >= 0 && c < CH_MAX_CONN && g_paired[c]; }
int at_send_data(int conn_id, const uint8_t *buf, size_t len) {
    char t[200];
    snprintf(t, sizeof(t), "%.*s", (int)len, (const char *)buf);
    logf_("tx", conn_id, t);
    return 0;
}
int at_close_connection(int conn_id) { g_closed = conn_id; return 0; }
void bp_error_send(int conn_id, bp_errcode_t code, const bp_err_arg_t *args, size_t n) {
    (void)args; (void)n;
    char t[16];
    snprintf(t, sizeof(t), "BP%04u", (unsigned)code);
    logf_("code", conn_id, t);
}

/* A raw-mode module: takes up to g_take[mode] bytes, then hands the connection back to JSON
   when g_done[mode] is set. */
static size_t g_take[8], g_got[8];
static bool   g_done[8];
static char   g_raw[256];
static size_t g_nraw;
static size_t fake_receive(proto_t mode, int conn_id, const uint8_t *buf, size_t len) {
    size_t take = len < g_take[mode] ? len : g_take[mode];
    memcpy(g_raw + g_nraw, buf, take);
    g_nraw += take;
    g_got[mode] += take;
    g_take[mode] -= take;
    if (g_take[mode] == 0 && g_done[mode]) conn_proto_set(conn_id, PROTO_JSON);
    return take;
}
size_t speedtest_receive(int c, const uint8_t *b, size_t n) { return fake_receive(PROTO_SPEEDTEST, c, b, n); }
size_t load_bin_receive(int c, const uint8_t *b, size_t n)  { return fake_receive(PROTO_LOAD, c, b, n); }
size_t dap_proxy_receive(int c, const uint8_t *b, size_t n) { return fake_receive(PROTO_DAP, c, b, n); }
size_t uart_proxy_receive(int c, const uint8_t *b, size_t n){ return fake_receive(PROTO_UART, c, b, n); }

static void feed(int conn, const char *s) { command_handler_process(conn, (const uint8_t *)s, strlen(s)); }
static void reset_conn(int conn) { transport_conn_closed(conn); reset_log(); }

/* ---- tests ----------------------------------------------------------------------------- */
static void test_lines(void) {
    reset_conn(0);
    feed(0, "{\"cmd\":\"pi");
    CHECK(g_nlog == 0, "dispatched a partial line");
    CHECK(conn_proto(0) == PROTO_JSON, "first byte '{' did not pick JSON");
    feed(0, "ng\"}\r\n{\"cmd\":\"status\"}\n{\"cmd\"");
    CHECK(g_nlog == 2, "expected 2 lines, got %d", g_nlog);
    CHECK(strcmp(g_log[0], "json0:{\"cmd\":\"ping\"}") == 0, "split line: %s", g_log[0]);
    CHECK(strcmp(g_log[1], "json0:{\"cmd\":\"status\"}") == 0, "second line: %s", g_log[1]);
    feed(0, ":\"x\"}\n\n\n");
    CHECK(g_nlog == 3 && strcmp(g_log[2], "json0:{\"cmd\":\"x\"}") == 0, "tail line / empty lines");
}

static void test_detection(void) {
    reset_conn(1);
    feed(1, "  \t*IDN?\n");
    CHECK(conn_proto(1) == PROTO_SCPI, "non-'{' did not pick SCPI");
    CHECK(g_nlog == 1 && strcmp(g_log[0], "scpi1:*IDN?") == 0, "scpi line: %s", g_log[0]);
    feed(1, "{\"cmd\":\"ping\"}\n");   /* the protocol is decided once per connection */
    CHECK(g_nlog == 2 && strncmp(g_log[1], "scpi1:", 6) == 0, "the protocol changed mid-connection");
    reset_conn(1);
    CHECK(conn_proto(1) == PROTO_UNKNOWN, "teardown did not reset the protocol");
    feed(1, "{\"cmd\":\"ping\"}\n");
    CHECK(conn_proto(1) == PROTO_JSON && strncmp(g_log[0], "json1:", 6) == 0, "next client not re-detected");
}

static void test_too_long(void) {
    char big[BP_CLOUD_CMD_IN_MAX + 64];
    reset_conn(2);
    memset(big, 'a', sizeof(big) - 1); big[0] = '{'; big[sizeof(big) - 1] = '\0';
    feed(2, big);
    CHECK(g_nlog == 1 && strcmp(g_log[0], "err2:command too long") == 0, "over-long JSON: %s", g_nlog ? g_log[0] : "-");
    feed(2, "still the same line\n{\"cmd\":\"ok\"}\n");
    CHECK(g_nlog == 2 && strcmp(g_log[1], "json2:{\"cmd\":\"ok\"}") == 0, "line after the dropped one: %s", g_log[g_nlog - 1]);
    /* The longest line that fits still dispatches. */
    reset_conn(2);
    memset(big, 'b', BP_CLOUD_CMD_IN_MAX - 1); big[0] = '{'; big[BP_CLOUD_CMD_IN_MAX - 1] = '\n';
    big[BP_CLOUD_CMD_IN_MAX] = '\0';
    feed(2, big);
    CHECK(g_nlog == 1 && strncmp(g_log[0], "json2:", 6) == 0 && strlen(g_log[0]) == 6 + BP_CLOUD_CMD_IN_MAX - 1,
          "a line of LINE_ASM_SIZE-1 bytes did not dispatch");
    /* SCPI: dropped silently. */
    reset_conn(3);
    memset(big, 'c', sizeof(big) - 1); big[sizeof(big) - 1] = '\0';
    feed(3, big);
    CHECK(g_nlog == 0, "an over-long SCPI line was answered");
}

static void test_raw_handoff(void) {
    /* A command switches the conn to PROTO_LOAD; the upload bytes (newlines included) go to the
       module, and the JSON behind them in the same segment is dispatched once it hands back. */
    reset_conn(0);
    feed(0, "{\"cmd\":\"load_bin\"}\n");
    conn_proto_set(0, PROTO_LOAD);
    g_take[PROTO_LOAD] = 6; g_done[PROTO_LOAD] = true; g_nraw = 0;
    feed(0, "ab\ncd\n{\"cmd\":\"replay\"}\n");
    CHECK(g_got[PROTO_LOAD] == 6 && memcmp(g_raw, "ab\ncd\n", 6) == 0, "upload bytes not routed whole");
    CHECK(g_nlog == 2 && strcmp(g_log[1], "json0:{\"cmd\":\"replay\"}") == 0, "JSON after the upload: %s", g_log[g_nlog - 1]);

    /* Each raw mode reaches its own module, split across segments. */
    const proto_t modes[] = { PROTO_SPEEDTEST, PROTO_DAP, PROTO_UART };
    for (size_t m = 0; m < 3; m++) {
        reset_conn(7);
        conn_proto_set(7, modes[m]);
        g_take[modes[m]] = 5; g_done[modes[m]] = true; g_got[modes[m]] = 0; g_nraw = 0;
        feed(7, "12");
        feed(7, "345{\"cmd\":\"ping\"}\n");
        CHECK(g_got[modes[m]] == 5 && memcmp(g_raw, "12345", 5) == 0, "mode %d did not get its bytes", (int)modes[m]);
        CHECK(g_nlog == 1 && strcmp(g_log[0], "json7:{\"cmd\":\"ping\"}") == 0, "mode %d: JSON after it", (int)modes[m]);
    }
}

static void feedn(int conn, const char *s, size_t n) { command_handler_process(conn, (const uint8_t *)s, n); }

static void test_raw_leave_unknown(void) {
    reset_conn(4);
    conn_proto_set(4, PROTO_DAP);
    g_take[PROTO_DAP] = 2; g_done[PROTO_DAP] = false; g_got[PROTO_DAP] = 0;
    /* the fake takes 2 bytes and stays in DAP; the module flips to UNKNOWN itself */
    feedn(4, "\0\0", 2);
    conn_proto_set(4, PROTO_UNKNOWN);
    feed(4, "MEAS?\n");
    CHECK(conn_proto(4) == PROTO_SCPI && g_nlog == 1 && strcmp(g_log[0], "scpi4:MEAS?") == 0,
          "not re-detected after leaving a raw mode");
}

static void test_scpi_lease(void) {
    reset_conn(1);
    g_lease = true;
    feed(1, "OUTP ON\nMEAS?\n");
    CHECK(g_nlog == 2 && strcmp(g_log[0], "scpierr0:") == 0, "a LAN SCPI setter ran under a lease: %s", g_log[0]);
    CHECK(strcmp(g_log[1], "scpi1:MEAS?") == 0, "a query was refused under a lease");
    /* The lease is a LAN rule: a cloud tunnel's setter runs. */
    reset_conn(7);
    feed(7, "OUTP ON\n");
    CHECK(g_nlog == 1 && strcmp(g_log[0], "scpi7:OUTP ON") == 0, "a tunnel setter was refused");
    g_lease = false;
}

static void test_pseudo_conn(void) {
    reset_log();
    feed(CH_CLOUD_CONN, "{\"cmd\":\"status\"}");   /* no newline: one-shot, unbuffered */
    CHECK(g_nlog == 1 && strcmp(g_log[0], "json6:{\"cmd\":\"status\"}") == 0, "pseudo-conn one-shot: %s", g_nlog ? g_log[0] : "-");
    CHECK(conn_proto(CH_CLOUD_CONN) == PROTO_UNKNOWN, "a pseudo-conn picked a protocol");
    CHECK(conn_proto(-1) == PROTO_UNKNOWN && conn_proto(99) == PROTO_UNKNOWN, "out-of-range conn_proto");
    conn_proto_set(99, PROTO_DAP);   /* ignored, not a write out of bounds */
}

static void test_teardown_drops_partial(void) {
    reset_conn(0);
    feed(0, "{\"cmd\":\"half");
    transport_conn_closed(0);
    feed(0, "{\"cmd\":\"ping\"}\n");
    CHECK(g_nlog == 1 && strcmp(g_log[0], "json0:{\"cmd\":\"ping\"}") == 0, "partial line survived teardown: %s", g_log[0]);
    /* Two connections assemble independently. */
    reset_conn(0); transport_conn_closed(8);
    feed(0, "{\"a\":");
    feed(8, "{\"b\":");
    feed(0, "1}\n");
    feed(8, "2}\n");
    CHECK(g_nlog == 2 && strcmp(g_log[0], "json0:{\"a\":1}") == 0 && strcmp(g_log[1], "json8:{\"b\":2}") == 0,
          "interleaved connections mixed");
}

/* Firmware 4.0.0: SCPI only while it is on (every TCP connection; not the cloud tunnels), and
   JSON on a plaintext connection from the external LAN only while HTTPS-only is off. A refusal is
   sent once, the connection closes, and nothing after it is dispatched. */
static void test_proto_gate(void) {
    g_scpi = false;
    reset_conn(2); g_closed = -1;
    feed(2, "*IDN?\nMEAS?\n");
    CHECK(g_nlog == 1 && strncmp(g_log[0], "tx2:SCPI is off", 15) == 0 && g_closed == 2,
          "SCPI while off: %d %s", g_nlog, g_nlog ? g_log[0] : "-");
    CHECK(conn_proto(2) == PROTO_REFUSED, "proto %d", conn_proto(2));
    feed(2, "{\"cmd\":\"ping\"}\n");
    CHECK(g_nlog == 1, "a refused connection dispatched more");
    g_usb[2] = true; reset_conn(2);   /* the USB link and TLS too */
    feed(2, "*IDN?\n");
    CHECK(g_nlog == 1 && g_closed == 2 && strncmp(g_log[0], "tx2:SCPI is off", 15) == 0, "USB link SCPI while off");
    g_usb[2] = false;
    reset_conn(7);                    /* a cloud tunnel: not affected */
    feed(7, "*IDN?\n");
    CHECK(g_nlog == 1 && strcmp(g_log[0], "scpi7:*IDN?") == 0, "tunnel SCPI refused: %s", g_log[0]);
    g_scpi = true;

    /* HTTPS-only: JSON on plaintext LAN is refused (BP0075); the USB link and TLS are fine. */
    g_https_only = true;
    reset_conn(3); g_closed = -1;
    feed(3, "{\"cmd\":\"status\"}\n");
    CHECK(g_nlog == 1 && strcmp(g_log[0], "code3:BP0075") == 0 && g_closed == 3, "plaintext LAN JSON: %s",
          g_nlog ? g_log[0] : "-");
    reset_conn(3); g_usb[3] = true;
    feed(3, "{\"cmd\":\"status\"}\n");
    CHECK(g_nlog == 1 && strcmp(g_log[0], "json3:{\"cmd\":\"status\"}") == 0, "USB link JSON refused");
    reset_conn(3); g_usb[3] = false; g_tls[3] = true;
    feed(3, "{\"cmd\":\"status\"}\n");
    CHECK(g_nlog == 1 && strcmp(g_log[0], "json3:{\"cmd\":\"status\"}") == 0, "TLS JSON refused");
    g_tls[3] = false;
    reset_conn(3);                    /* SCPI on plaintext LAN under HTTPS-only: served */
    feed(3, "*IDN?\n");
    CHECK(g_nlog == 1 && strcmp(g_log[0], "scpi3:*IDN?") == 0, "SCPI under HTTPS-only: %s", g_log[0]);
    /* A setting switched off mid-session ends it on the next line. */
    g_scpi = false;
    feed(3, "MEAS?\n");
    CHECK(g_nlog == 2 && strncmp(g_log[1], "tx3:SCPI is off", 15) == 0, "SCPI switched off mid-session");
    g_scpi = true; g_https_only = false;
    reset_conn(3);
}

/* LAN pairing: SCPI on the external LAN needs a paired TLS connection; the USB link and the cloud
   tunnels do not. The refusal says how, and the connection closes. */
static void test_scpi_pairing(void) {
    g_pairing = true;
    reset_conn(2); g_closed = -1;
    feed(2, "*IDN?\n");
    CHECK(g_nlog == 1 && strncmp(g_log[0], "tx2:SCPI on the LAN needs a paired", 34) == 0 && g_closed == 2,
          "unpaired LAN SCPI: %d %s", g_nlog, g_nlog ? g_log[0] : "-");
    reset_conn(2); g_tls[2] = true;   /* TLS but not paired: refused too */
    feed(2, "*IDN?\n");
    CHECK(g_nlog == 1 && strncmp(g_log[0], "tx2:SCPI on the LAN", 19) == 0, "unpaired TLS SCPI: %s", g_log[0]);
    reset_conn(2); g_paired[2] = true;
    feed(2, "*IDN?\n");
    CHECK(g_nlog == 1 && strcmp(g_log[0], "scpi2:*IDN?") == 0, "paired TLS SCPI: %s", g_log[0]);
    g_tls[2] = false; g_paired[2] = false;
    reset_conn(2); g_usb[2] = true;
    feed(2, "*IDN?\n");
    CHECK(g_nlog == 1 && strcmp(g_log[0], "scpi2:*IDN?") == 0, "USB link SCPI: %s", g_log[0]);
    g_usb[2] = false;
    reset_conn(7);
    feed(7, "*IDN?\n");
    CHECK(g_nlog == 1 && strcmp(g_log[0], "scpi7:*IDN?") == 0, "tunnel SCPI: %s", g_log[0]);
    g_scpi = false;                   /* off wins: the old text */
    reset_conn(2);
    feed(2, "*IDN?\n");
    CHECK(g_nlog == 1 && strncmp(g_log[0], "tx2:SCPI is off", 15) == 0, "off and unpaired: %s", g_log[0]);
    g_scpi = true; g_pairing = false;
    reset_conn(2); reset_conn(7);
}

int main(void) {
    test_lines();
    test_detection();
    test_too_long();
    test_raw_handoff();
    test_raw_leave_unknown();
    test_scpi_lease();
    test_pseudo_conn();
    test_teardown_drops_partial();
    test_proto_gate();
    test_scpi_pairing();
    if (fails) { printf("test_ch_transport: %d FAILED\n", fails); return 1; }
    printf("test_ch_transport: all passed\n");
    return 0;
}
