/*
 * command_handler_i2c.c — the I2C master on the LA pins (gateware >= v55), as JSON commands.
 *
 *   {"cmd":"i2c_start","sda":1,"scl":2,"hz":400000,"pullups":"pod"|"external"}
 *   {"cmd":"i2c_xfer","addr":72,"write":"<b64url>","read":2}  -> {"ack":true,"rx":"<b64url>"}
 *   {"cmd":"i2c_scan"}                                         -> {"found":[72,118]}
 *   {"cmd":"i2c_status"} / {"cmd":"i2c_stop"}
 *
 * i2c_start claims the two pins (la_pins: i2cm_sda / i2cm_scl) and arms the SWD engine as an
 * open-drain I2C master, so SWD, SPI and I2C sessions exclude each other.  The bus needs pull-ups:
 * with "pullups":"pod" (the default) both pins must be LA1..LA6 with the pod's pull-up switched on
 * (they go to 3.3 V, so the LA bank must be at 3.3 V); "pullups":"external" says the board has its
 * own.  The pod never switches a pull-up by itself: the error names the command that does.
 *
 * i2c_xfer writes `write` (0..128 bytes) and then, after a repeated START, reads `read` (0..248)
 * bytes, NACKing the last; both empty probes the address.  A target may stretch SCL: the pod waits
 * up to I2C_STRETCH_BUDGET_MS beyond the transfer's own time, then releases the bus and reports it.
 * Commands and replies fit one cloud frame.  Runs on the hw worker task.
 */
#include "command_handler.h"
#include "command_handler_internal.h"
#include "signal_engine.h"
#include "i2c_txn.h"
#include "i2c_bus.h"
#include "la_pins.h"
#include "b64url.h"
#include "bp_json.h"
#include "bp_limits.h"
#include "at_driver.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define I2C_DEFAULT_HZ          100000u
#define I2C_STRETCH_BUDGET_MS   200u

_Static_assert(B64URL_ENCODED_LEN(I2C_TXN_WRITE_MAX) + 128u <= BP_CLOUD_CMD_IN_MAX,
               "an i2c_xfer command must fit one cloud command");
_Static_assert(B64URL_ENCODED_LEN(I2C_TXN_READ_MAX) + 128u <= BP_CLOUD_REPLY_MAX,
               "an i2c_xfer reply must fit one cloud reply");
_Static_assert(I2C_TXN_TOK_MAX == I2C_QUEUE_MAX, "i2c_txn and the gateware queue agree");

static uint8_t  s_sda, s_scl;
static unsigned s_half;
static bool     s_external;

static const la_fn_t k_fns[2] = { LA_FN_I2CM_SDA, LA_FN_I2CM_SCL };

/* One transaction at a time (the worker task): its tokens, its reply (+1 for fpga_i2c_run), and
   the reply text, built in the token buffer once the run is over (RAM is tight). */
static i2c_txn_t s_t;
static uint8_t   s_reply[I2C_TXN_TOK_MAX + 1];
_Static_assert(sizeof(s_t.tok) >= B64URL_ENCODED_LEN(I2C_TXN_READ_MAX) + 96u,
               "the i2c_xfer reply text fits the token buffer");

static void i2c_release_pins(void) {
    la_pins_release_fn(LA_FN_I2CM_SDA);
    la_pins_release_fn(LA_FN_I2CM_SCL);
}

static bool get_u32(const char *json, const char *key, uint32_t *out) {
    char s[16];
    if (!bp_json_get(json, key, s, sizeof(s))) return false;
    char *end = NULL;
    unsigned long v = strtoul(s, &end, 0);
    if (!end || *end != '\0') return false;
    *out = (uint32_t)v;
    return true;
}

static bool has_key(const char *json, const char *key) {
    char t[24];
    return bp_json_get(json, key, t, sizeof(t));
}

/* The pod's own pull-up on `la`, or why it can't serve the bus. */
static bool pod_pullup_or_error(int conn_id, unsigned la) {
    char msg[200];
    if (la_pull_dir(la) != LA_PULL_UP) {
        snprintf(msg, sizeof(msg), "LA%u has no pod pull-up (only LA1..LA6 do): use LA1..LA6, or pass "
                 "\"pullups\":\"external\" if the board has its own", la);
        send_error(conn_id, msg); return false;
    }
    if (!la_pullups_available()) {
        send_error(conn_id, "the pod's pull-ups go to 3.3 V and are off with the LA bank at 1.8 V: pass "
                   "\"pullups\":\"external\" with pull-ups to the board's 1.8 V");
        return false;
    }
    if (!pca9555_la_pullup_enabled((uint8_t)la)) {
        snprintf(msg, sizeof(msg), "I2C needs pull-ups: switch on LA%u's with {\"cmd\":\"la\",\"la\":%u,"
                 "\"pullup\":\"on\"}, or pass \"pullups\":\"external\"", la, la);
        send_error(conn_id, msg); return false;
    }
    return true;
}

/* ---- i2c_start / i2c_stop / i2c_status ---- */

void handle_i2c_start(int conn_id, const char *json) {
    if (!require_la_voltage(conn_id)) return;
    if (signal_engine_fpga_version() < I2C_MASTER_MIN_GW) {
        send_error(conn_id, "the I2C master needs gateware v55+: update the pod's firmware"); return;
    }
    uint32_t sda, scl;
    if (!get_u32(json, "sda", &sda) || sda < 1 || sda > LA_PINS_COUNT) { send_error(conn_id, "missing or invalid sda (LA 1..14)"); return; }
    if (!get_u32(json, "scl", &scl) || scl < 1 || scl > LA_PINS_COUNT) { send_error(conn_id, "missing or invalid scl (LA 1..14)"); return; }
    if (sda == scl) { send_error(conn_id, "sda and scl need their own LA pins"); return; }
    uint32_t hz = I2C_DEFAULT_HZ;
    if (has_key(json, "hz") && (!get_u32(json, "hz", &hz) || hz < 10000u || hz > 1000000u)) {
        send_error(conn_id, "hz must be 10000..1000000"); return;
    }
    char pu[12];
    if (!bp_json_get(json, "pullups", pu, sizeof(pu))) strcpy(pu, "pod");
    bool external = strcmp(pu, "external") == 0;
    if (!external && strcmp(pu, "pod") != 0) { send_error(conn_id, "pullups must be pod or external"); return; }
    if (fpga_i2c_armed()) { send_error(conn_id, "i2c busy: send i2c_stop first"); return; }

    uint8_t pins[2] = { (uint8_t)sda, (uint8_t)scl };
    for (unsigned i = 0; i < 2; i++)
        if (!la_claim_or_error(conn_id, k_fns[i], &pins[i], 1, 0)) return;
    if (!external && (!pod_pullup_or_error(conn_id, sda) || !pod_pullup_or_error(conn_id, scl))) return;

    unsigned half = fpga_i2c_half_for_hz(hz);
    int rc = fpga_i2c_arm(sda, scl, half);
    if (rc == -2) { send_error(conn_id, "swd, spi or i2c busy: end that session first"); return; }
    if (rc != 0)  { send_error(conn_id, "invalid i2c arguments"); return; }
    for (unsigned i = 0; i < 2; i++) la_pins_claim(k_fns[i], LA_GPIO_NONE, 0, &pins[i], 1);
    s_sda = pins[0]; s_scl = pins[1]; s_half = half; s_external = external;

    char p[112];
    snprintf(p, sizeof(p), "{\"sda\":%u,\"scl\":%u,\"hz\":%lu,\"pullups\":\"%s\"}",
             s_sda, s_scl, (unsigned long)fpga_i2c_hz_for_half(half), external ? "external" : "pod");
    send_ok_str(conn_id, p);
}

void i2c_session_end(void) {
    if (fpga_i2c_armed()) fpga_i2c_disarm();
    i2c_release_pins();
}

void handle_i2c_stop(int conn_id, const char *json) {
    (void)json;
    i2c_session_end();
    send_ok_str(conn_id, "\"i2c stopped\"");
}

/* The gateware was reconfigured: the engine is gone with it. */
void i2c_on_gateware_reconfigured(void) {
    if (fpga_i2c_armed()) fpga_i2c_disarm();
    i2c_release_pins();
}

void handle_i2c_status(int conn_id, const char *json) {
    (void)json;
    if (!fpga_i2c_armed()) { send_ok_str(conn_id, "{\"active\":false}"); return; }
    char p[112];
    snprintf(p, sizeof(p), "{\"active\":true,\"sda\":%u,\"scl\":%u,\"hz\":%lu,\"pullups\":\"%s\"}",
             s_sda, s_scl, (unsigned long)fpga_i2c_hz_for_half(s_half), s_external ? "external" : "pod");
    send_ok_str(conn_id, p);
}

/* ---- running a transaction ---- */

static bool session_or_error(int conn_id) {
    if (fpga_i2c_armed()) return true;
    send_error(conn_id, "no I2C session: send i2c_start first");
    return false;
}

/* Run t; on failure release the bus, send the error and return false. */
static bool run_or_error(int conn_id, const i2c_txn_t *t, uint8_t *reply) {
    uint32_t budget = i2c_txn_us(t, s_half) * 2u + I2C_STRETCH_BUDGET_MS * 1000u;
    bool arb = false;
    int rc = fpga_i2c_run(t->tok, t->ntok, reply, t->nreply, budget, &arb);
    if (rc == -3) { i2c_session_end(); send_error(conn_id, "the I2C engine was reset: i2c_start again"); return false; }
    if (rc == -2) {
        fpga_i2c_recover(s_sda, s_scl, s_half);
        char msg[160];
        snprintf(msg, sizeof(msg), "SCL held low: a target stretched the clock for over %u ms, or the bus "
                 "is stuck; the pod released the bus", (unsigned)I2C_STRETCH_BUDGET_MS);
        send_error(conn_id, msg);
        return false;
    }
    if (rc != 0) { send_error(conn_id, "i2c transfer failed"); return false; }
    if (arb) {
        fpga_i2c_recover(s_sda, s_scl, s_half);
        send_error(conn_id, "arbitration lost: another master drove SDA low; the pod released the bus");
        return false;
    }
    return true;
}

void handle_i2c_xfer(int conn_id, const char *json) {
    if (!session_or_error(conn_id)) return;
    uint32_t addr, nr = 0;
    if (!get_u32(json, "addr", &addr) || addr > 0x7Fu) { send_error(conn_id, "addr must be a 7-bit address (0..127)"); return; }
    if (has_key(json, "read") && (!get_u32(json, "read", &nr) || nr > I2C_TXN_READ_MAX)) {
        send_error(conn_id, "read must be 0..248"); return;
    }
    static char b64[B64URL_ENCODED_LEN(I2C_TXN_WRITE_MAX) + 8];
    uint8_t *w = s_reply;                     /* the write data until the run, then the reply */
    size_t nw = 0;
    if (has_key(json, "write")) {
        if (bp_json_get_fit(json, "write", b64, sizeof(b64)) != 1 ||
            b64url_decode(b64, w, I2C_TXN_WRITE_MAX + 4u, &nw) != 0 || nw > I2C_TXN_WRITE_MAX) {
            send_error(conn_id, "write must be 0..128 bytes of base64url"); return;
        }
    }
    if (i2c_txn_xfer(&s_t, (uint8_t)addr, w, nw, nr) != 0) { send_error(conn_id, "invalid i2c_xfer"); return; }
    if (!run_or_error(conn_id, &s_t, s_reply)) return;
    i2c_result_t r;
    i2c_txn_result(&s_t, s_reply, &r);
    const uint8_t *rx = s_reply + r.rx_off;

    bp_emit_t e;                              /* the reply text, in the spent token buffer */
    bp_emit_init(&e, (char *)s_t.tok, sizeof(s_t.tok));
    bp_emit(&e, "{\"status\":\"ok\",\"data\":{\"addr\":%lu,\"ack\":%s", (unsigned long)addr, r.ack ? "true" : "false");
    if (r.nack_at >= 0) bp_emit(&e, ",\"nack_at\":%d", r.nack_at);
    if (nr > 0) {
        bp_emit_raw(&e, ",\"rx\":\"");
        if (r.nrx > 0) {
            size_t wn = b64url_encode(rx, r.nrx, e.buf + e.len, e.cap - e.len);
            if (wn == 0) e.ok = false; else e.len += wn;
        }
        bp_emit_raw(&e, "\"");
    }
    bp_emit_raw(&e, "}}\n");
    if (!bp_emit_ok(&e)) { send_error(conn_id, "reply too large"); return; }
    if (at_send_data(conn_id, (const uint8_t *)e.buf, bp_emit_len(&e)) != 0) at_close_connection(conn_id);
}

void handle_i2c_scan(int conn_id, const char *json) {
    if (!session_or_error(conn_id)) return;
    uint32_t first = I2C_SCAN_FIRST, last = I2C_SCAN_LAST;
    if (has_key(json, "from") && (!get_u32(json, "from", &first) || first > 0x7Fu)) { send_error(conn_id, "from must be 0..127"); return; }
    if (has_key(json, "to") && (!get_u32(json, "to", &last) || last > 0x7Fu || last < first)) { send_error(conn_id, "to must be from..127"); return; }
    if (i2c_txn_scan(&s_t, (uint8_t)first, (uint8_t)last) != 0) { send_error(conn_id, "invalid i2c_scan range"); return; }
    if (!run_or_error(conn_id, &s_t, s_reply)) return;
    const uint8_t *reply = s_reply;
    char *p = (char *)s_t.tok;                /* the reply text, in the spent token buffer */
    const size_t pcap = sizeof(s_t.tok);
    size_t n = (size_t)snprintf(p, pcap,"{\"from\":%lu,\"to\":%lu,\"found\":[", (unsigned long)first, (unsigned long)last);
    bool any = false;
    for (unsigned a = first; a <= last && n < pcap - 8; a++) {
        if (reply[a - first] & 1u) continue;
        n += (size_t)snprintf(p + n, pcap - n, "%s%u", any ? "," : "", a);
        any = true;
    }
    snprintf(p + n, pcap - n, "]}");
    send_ok_str(conn_id, p);
}

/* ---- send and capture (la_capture "send":{"bus":"i2c",...}, command_handler_bus_send.c) ----
   The transaction is built before the capture is armed and run right after it, so a trigger on
   SDA catches its START.  Smaller than i2c_xfer so the result rides the capture's last frame. */

int i2c_capture_prepare(const char *obj, char *err, size_t cap) {
    if (!fpga_i2c_armed()) { snprintf(err, cap, "send: no I2C session: send i2c_start first"); return -1; }
    uint32_t addr, nr = 0;
    if (!get_u32(obj, "addr", &addr) || addr > 0x7Fu) { snprintf(err, cap, "send: addr must be 0..127"); return -1; }
    if (has_key(obj, "read") && (!get_u32(obj, "read", &nr) || nr > BUS_SEND_MAX)) {
        snprintf(err, cap, "send: read must be 0..%u", (unsigned)BUS_SEND_MAX); return -1;
    }
    char b64[B64URL_ENCODED_LEN(BUS_SEND_MAX) + 8];
    size_t nw = 0;
    if (has_key(obj, "write") &&
        (bp_json_get_fit(obj, "write", b64, sizeof(b64)) != 1 ||
         b64url_decode(b64, s_reply, BUS_SEND_MAX + 4u, &nw) != 0 || nw > BUS_SEND_MAX)) {
        snprintf(err, cap, "send: write must be 0..%u bytes of base64url", (unsigned)BUS_SEND_MAX); return -1;
    }
    if (i2c_txn_xfer(&s_t, (uint8_t)addr, s_reply, nw, nr) != 0) { snprintf(err, cap, "send: invalid i2c transfer"); return -1; }
    return 0;
}

void i2c_capture_run(char *out, size_t cap) {
    uint32_t budget = i2c_txn_us(&s_t, s_half) * 2u + I2C_STRETCH_BUDGET_MS * 1000u;
    bool arb = false;
    int rc = fpga_i2c_run(s_t.tok, s_t.ntok, s_reply, s_t.nreply, budget, &arb);
    if (rc == -2 || arb) fpga_i2c_recover(s_sda, s_scl, s_half);
    if (rc != 0 || arb) {
        snprintf(out, cap, "{\"bus\":\"i2c\",\"error\":\"%s\"}",
                 arb ? "arbitration lost" : rc == -2 ? "SCL held low" : "transfer failed");
        return;
    }
    i2c_result_t r;
    i2c_txn_result(&s_t, s_reply, &r);
    int n = snprintf(out, cap, "{\"bus\":\"i2c\",\"ack\":%s", r.ack ? "true" : "false");
    if (r.nack_at >= 0) n += snprintf(out + n, cap - (size_t)n, ",\"nack_at\":%d", r.nack_at);
    if (s_t.nr > 0) {
        n += snprintf(out + n, cap - (size_t)n, ",\"rx\":\"");
        if (r.nrx) n += (int)b64url_encode(s_reply + r.rx_off, r.nrx, out + n, cap - (size_t)n);
        n += snprintf(out + n, cap - (size_t)n, "\"");
    }
    snprintf(out + n, cap - (size_t)n, "}");
}
