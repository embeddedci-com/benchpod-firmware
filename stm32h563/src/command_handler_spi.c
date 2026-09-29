/*
 * command_handler_spi.c — the SPI master on the LA pins (gateware >= v44), as JSON commands.
 *
 *   {"cmd":"spi_start","sck":3,"mosi":4,"miso":5,"cs":6,"hz":1000000,"mode":0}
 *   {"cmd":"spi_xfer","tx":"<b64url>","cs":"release"|"hold"}   -> {"rx":"<b64url>"}
 *   {"cmd":"spi_flash","op":"id"}                              -> {"id":"ef4017",...}
 *   {"cmd":"spi_flash","op":"read","addr":N,"len":N}           -> {"data":"<b64url>"}
 *   {"cmd":"spi_flash","op":"erase","addr":N,"len":N}          -> {"addr":..,"len":..}
 *   {"cmd":"spi_flash","op":"write","addr":N,"data":"<b64url>","verify":true}
 *   {"cmd":"spi_flash","op":"chip_erase"}
 *   {"cmd":"spi_stop"}
 *
 * spi_start claims the four pins (la_pins: spi_sck / spi_mosi / spi_miso / spi_cs) and arms
 * the SWD engine in SPI mode, so an SWD session and an SPI session exclude each other.  The
 * sizes keep every command and reply inside one cloud frame (BP_CLOUD_CMD_IN_MAX in,
 * BP_CLOUD_REPLY_MAX out), so everything here also works over the cloud command channel.
 * Hold the DUT in reset while flashing its SPI flash ({"cmd":"nrst","assert":true}) so its
 * own controller does not drive the same bus.
 *
 * Runs on the hw worker task, like the rest of command_handler.
 */
#include "command_handler.h"
#include "command_handler_internal.h"
#include "signal_engine.h"
#include "spi_flash.h"
#include "la_pins.h"
#include "b64url.h"
#include "bp_json.h"
#include "bp_limits.h"
#include "at_driver.h"
#include "watchdog.h"
#include "pico/time.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SPI_XFER_CMD_MAX    768u    /* spi_xfer tx and spi_flash write data per command */
#define SPI_READ_CMD_MAX    1024u   /* spi_flash read per command */
#define SPI_ERASE_CMD_MAX   0x100000u   /* 1 MB per erase: <= 16 block erases, a few seconds */
#define SPI_CHIP_ERASE_MS   200000u
#define SPI_DEFAULT_HZ      1000000u

_Static_assert(B64URL_ENCODED_LEN(SPI_XFER_CMD_MAX) + 128u <= BP_CLOUD_CMD_IN_MAX,
               "an spi_xfer / spi_flash write command must fit one cloud command");
_Static_assert(B64URL_ENCODED_LEN(SPI_READ_CMD_MAX) + 128u <= BP_CLOUD_REPLY_MAX,
               "an spi_flash read reply must fit one cloud reply");

static bool    s_cs_held;
static uint8_t s_pins[4];      /* sck, mosi, miso, cs (1-based LA) */
static uint32_t s_hz;
static uint8_t s_mode;

static const la_fn_t k_fns[4] = { LA_FN_SPI_SCK, LA_FN_SPI_MOSI, LA_FN_SPI_MISO, LA_FN_SPI_CS };

static void spi_release_pins(void) {
    for (unsigned i = 0; i < 4; i++) la_pins_release_fn(k_fns[i]);
}

/* ---- transport glue for spi_flash.c ---- */

static void io_cs(void *ctx, bool asserted) { (void)ctx; fpga_spi_cs(asserted); }
static int io_xfer(void *ctx, const uint8_t *tx, uint8_t *rx, size_t n) {
    (void)ctx;
    return fpga_spi_xfer(tx, rx, n);
}
static void io_sleep(void *ctx, uint32_t us) {
    (void)ctx;
    watchdog_heartbeat(WD_TASK_WORKER, "spi");   /* an erase can wait seconds */
    if (us >= 1000u) sleep_ms(us / 1000u);
    else             sleep_us(us);
}
static uint64_t io_now(void *ctx) { (void)ctx; return time_us_64(); }

static const spi_flash_io_t k_io = { NULL, io_cs, io_xfer, io_sleep, io_now, SPI_XFER_MAX };

/* ---- replies ---- */

static void send_built(int conn_id, bp_emit_t *e) {
    if (!bp_emit_ok(e)) { send_error(conn_id, "reply too large"); return; }
    if (at_send_data(conn_id, (const uint8_t *)e->buf, bp_emit_len(e)) != 0)
        at_close_connection(conn_id);
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

static bool session_or_error(int conn_id) {
    if (fpga_spi_armed()) return true;
    send_error(conn_id, "no SPI session: send spi_start first");
    return false;
}

/* ---- spi_start / spi_stop ---- */

void handle_spi_start(int conn_id, const char *json) {
    if (!require_la_voltage(conn_id)) return;
    if (signal_engine_fpga_version() < SPI_MASTER_MIN_GW) {
        send_error(conn_id, "SPI master needs gateware v44+"); return;
    }
    static const char *const keys[4] = { "sck", "mosi", "miso", "cs" };
    uint32_t v[4];
    for (unsigned i = 0; i < 4; i++) {
        if (!get_u32(json, keys[i], &v[i]) || v[i] < 1 || v[i] > LA_PINS_COUNT) {
            char msg[48];
            snprintf(msg, sizeof(msg), "missing or invalid %s (LA 1..14)", keys[i]);
            send_error(conn_id, msg);
            return;
        }
        for (unsigned j = 0; j < i; j++)
            if (v[j] == v[i]) { send_error(conn_id, "each SPI signal needs its own LA pin"); return; }
    }
    uint32_t hz = SPI_DEFAULT_HZ, mode = 0;
    if (has_key(json, "hz") && !get_u32(json, "hz", &hz)) {
        send_error(conn_id, "invalid hz"); return;
    }
    if (has_key(json, "mode") && (!get_u32(json, "mode", &mode) || (mode != 0 && mode != 3))) {
        send_error(conn_id, "mode must be 0 or 3"); return;
    }
    if (fpga_spi_armed()) { send_error(conn_id, "spi busy: send spi_stop first"); return; }

    uint8_t pins[4];
    for (unsigned i = 0; i < 4; i++) pins[i] = (uint8_t)v[i];
    for (unsigned i = 0; i < 4; i++)
        if (!la_claim_or_error(conn_id, k_fns[i], &pins[i], 1, 0)) return;

    unsigned half = fpga_spi_half_for_hz(hz);
    int rc = fpga_spi_arm(v[0], v[1], v[2], v[3], half, mode);
    if (rc == -2) { send_error(conn_id, "swd or spi busy: end the SWD session first"); return; }
    if (rc != 0)  { send_error(conn_id, "invalid spi arguments"); return; }
    for (unsigned i = 0; i < 4; i++) la_pins_claim(k_fns[i], LA_GPIO_NONE, 0, &pins[i], 1);
    memcpy(s_pins, pins, sizeof(s_pins));
    s_hz      = fpga_spi_hz_for_half(half);
    s_mode    = (uint8_t)mode;
    s_cs_held = false;

    char p[128];
    snprintf(p, sizeof(p), "{\"sck\":%u,\"mosi\":%u,\"miso\":%u,\"cs\":%u,\"hz\":%lu,\"mode\":%u}",
             pins[0], pins[1], pins[2], pins[3], (unsigned long)s_hz, (unsigned)mode);
    send_ok_str(conn_id, p);
}

void spi_session_end(void) {
    if (fpga_spi_armed()) fpga_spi_disarm();
    s_cs_held = false;
    spi_release_pins();
}

void handle_spi_stop(int conn_id) {
    spi_session_end();
    send_ok_str(conn_id, "\"spi stopped\"");
}

/* The gateware was reconfigured: the engine is gone with it. */
void spi_on_gateware_reconfigured(void) {
    if (fpga_spi_armed()) fpga_spi_disarm();   /* local state only; the fabric is already reset */
    s_cs_held = false;
    spi_release_pins();
}

/* ---- spi_xfer ---- */

void handle_spi_xfer(int conn_id, const char *json) {
    if (!session_or_error(conn_id)) return;
    static char    b64[B64URL_ENCODED_LEN(SPI_XFER_CMD_MAX) + 8];
    static uint8_t tx[SPI_XFER_CMD_MAX + 4], rx[SPI_XFER_CMD_MAX];
    size_t n = 0;
    if (bp_json_get_fit(json, "tx", b64, sizeof(b64)) != 1 ||
        b64url_decode(b64, tx, sizeof(tx), &n) != 0 || n == 0 || n > SPI_XFER_CMD_MAX) {
        send_error(conn_id, "tx must be 1..768 bytes of base64url"); return;
    }
    char csm[12];
    if (!bp_json_get(json, "cs", csm, sizeof(csm))) strcpy(csm, "release");   /* it clears csm when absent */
    bool hold = strcmp(csm, "hold") == 0;
    if (!hold && strcmp(csm, "release") != 0) { send_error(conn_id, "cs must be release or hold"); return; }

    if (!s_cs_held) fpga_spi_cs(true);
    int rc = 0;
    for (size_t off = 0; rc == 0 && off < n; off += SPI_XFER_MAX) {
        size_t k = n - off < SPI_XFER_MAX ? n - off : SPI_XFER_MAX;
        rc = fpga_spi_xfer(tx + off, rx + off, k);
    }
    if (rc != 0 || !hold) { fpga_spi_cs(false); s_cs_held = false; }
    else                  s_cs_held = true;
    if (rc == -3) { spi_session_end(); send_error(conn_id, "the SPI engine was reset: spi_start again"); return; }
    if (rc != 0)  { send_error(conn_id, "spi transfer failed"); return; }

    static char resp[BP_CLOUD_REPLY_MAX];
    bp_emit_t e;
    bp_emit_init(&e, resp, sizeof(resp));
    bp_emit_raw(&e, "{\"status\":\"ok\",\"data\":{\"rx\":\"");
    size_t room = e.cap - e.len;
    size_t w = b64url_encode(rx, n, e.buf + e.len, room);
    if (w == 0) e.ok = false; else e.len += w;
    bp_emit(&e, "\",\"cs\":\"%s\"}}\n", s_cs_held ? "held" : "released");
    send_built(conn_id, &e);
}

/* ---- spi_flash ---- */

static void flash_error(int conn_id, int rc) {
    if (!fpga_spi_armed()) { spi_session_end(); send_error(conn_id, "the SPI engine was reset: spi_start again"); return; }
    send_error(conn_id, spi_flash_strerror(rc));
}

void handle_spi_flash(int conn_id, const char *json) {
    if (!session_or_error(conn_id)) return;
    if (s_cs_held) { fpga_spi_cs(false); s_cs_held = false; }   /* a raw transaction left open */
    char op[16] = {0};
    bp_json_get(json, "op", op, sizeof(op));
    uint32_t addr = 0, len = 0;
    char p[160];
    absolute_time_t t0 = get_absolute_time();

    if (strcmp(op, "id") == 0) {
        uint8_t id[3] = { 0 }, sr = 0;
        int rc = spi_flash_read_id(&k_io, id);
        if (rc == 0) rc = spi_flash_read_status(&k_io, &sr);
        if (rc) { flash_error(conn_id, rc); return; }
        snprintf(p, sizeof(p), "{\"id\":\"%02x%02x%02x\",\"present\":%s,\"size\":%lu,\"status\":%u}",
                 id[0], id[1], id[2], spi_flash_id_valid(id) ? "true" : "false",
                 (unsigned long)spi_flash_capacity(id), sr);
        send_ok_str(conn_id, p);
        return;
    }

    if (strcmp(op, "read") == 0) {
        if (!get_u32(json, "addr", &addr) || !get_u32(json, "len", &len) ||
            len == 0 || len > SPI_READ_CMD_MAX) {
            send_error(conn_id, "read needs addr and len 1..1024"); return;
        }
        static uint8_t buf[SPI_READ_CMD_MAX];
        int rc = spi_flash_read(&k_io, addr, buf, len);
        if (rc) { flash_error(conn_id, rc); return; }
        static char resp[BP_CLOUD_REPLY_MAX];
        bp_emit_t e;
        bp_emit_init(&e, resp, sizeof(resp));
        bp_emit(&e, "{\"status\":\"ok\",\"data\":{\"addr\":%lu,\"len\":%lu,\"data\":\"",
                (unsigned long)addr, (unsigned long)len);
        size_t w = b64url_encode(buf, len, e.buf + e.len, e.cap - e.len);
        if (w == 0) e.ok = false; else e.len += w;
        bp_emit_raw(&e, "\"}}\n");
        send_built(conn_id, &e);
        return;
    }

    if (strcmp(op, "erase") == 0) {
        if (!get_u32(json, "addr", &addr) || !get_u32(json, "len", &len) ||
            len == 0 || len > SPI_ERASE_CMD_MAX) {
            send_error(conn_id, "erase needs addr and len 1..1048576"); return;
        }
        uint32_t ds = 0, dl = 0;
        int rc = spi_flash_erase(&k_io, addr, len, &ds, &dl);
        if (rc) { flash_error(conn_id, rc); return; }
        snprintf(p, sizeof(p), "{\"addr\":%lu,\"len\":%lu,\"ms\":%lu}", (unsigned long)ds,
                 (unsigned long)dl, (unsigned long)(absolute_time_diff_us(t0, get_absolute_time()) / 1000));
        send_ok_str(conn_id, p);
        return;
    }

    if (strcmp(op, "chip_erase") == 0) {
        int rc = spi_flash_chip_erase(&k_io, SPI_CHIP_ERASE_MS);
        if (rc) { flash_error(conn_id, rc); return; }
        snprintf(p, sizeof(p), "{\"ms\":%lu}",
                 (unsigned long)(absolute_time_diff_us(t0, get_absolute_time()) / 1000));
        send_ok_str(conn_id, p);
        return;
    }

    if (strcmp(op, "write") == 0) {
        static char    b64[B64URL_ENCODED_LEN(SPI_XFER_CMD_MAX) + 8];
        static uint8_t data[SPI_XFER_CMD_MAX + 4];
        size_t n = 0;
        if (!get_u32(json, "addr", &addr)) { send_error(conn_id, "write needs addr"); return; }
        if (bp_json_get_fit(json, "data", b64, sizeof(b64)) != 1 ||
            b64url_decode(b64, data, sizeof(data), &n) != 0 || n == 0 || n > SPI_XFER_CMD_MAX) {
            send_error(conn_id, "data must be 1..768 bytes of base64url"); return;
        }
        bool verify = !has_key(json, "verify") || bp_json_flag(json, "verify");
        int rc = spi_flash_program(&k_io, addr, data, n);
        uint32_t bad = 0;
        if (rc == 0 && verify) rc = spi_flash_verify(&k_io, addr, data, n, &bad);
        if (rc == SPI_FLASH_E_VERIFY) {
            snprintf(p, sizeof(p), "verify failed at 0x%06lx: not erased, write-protected, or a bad wire",
                     (unsigned long)bad);
            send_error(conn_id, p);
            return;
        }
        if (rc) { flash_error(conn_id, rc); return; }
        snprintf(p, sizeof(p), "{\"addr\":%lu,\"len\":%lu,\"verified\":%s,\"ms\":%lu}",
                 (unsigned long)addr, (unsigned long)n, verify ? "true" : "false",
                 (unsigned long)(absolute_time_diff_us(t0, get_absolute_time()) / 1000));
        send_ok_str(conn_id, p);
        return;
    }

    send_error(conn_id, "op must be id, read, erase, chip_erase or write");
}

void handle_spi_status(int conn_id) {
    char p[160];
    if (!fpga_spi_armed()) { send_ok_str(conn_id, "{\"active\":false}"); return; }
    snprintf(p, sizeof(p), "{\"active\":true,\"sck\":%u,\"mosi\":%u,\"miso\":%u,\"cs\":%u,"
             "\"hz\":%lu,\"mode\":%u,\"cs_held\":%s}",
             s_pins[0], s_pins[1], s_pins[2], s_pins[3], (unsigned long)s_hz, s_mode,
             s_cs_held ? "true" : "false");
    send_ok_str(conn_id, p);
}
