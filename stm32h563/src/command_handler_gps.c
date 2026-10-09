/*
 * command_handler_gps.c — the emulated GPS receiver's JSON commands (gps_sim.c over UART2).
 *
 *   {"cmd":"gps_start","tx":5,"baud":9600,"rate_hz":1,"sentences":"RMC,VTG,GGA,GSA,GSV,GLL",
 *    ...any gps_set field}                                       → the session
 *   {"cmd":"gps_set","latitude_deg":50.85,"longitude_deg":4.35,"altitude_m":56,
 *    "speed_kmh":0,"course_deg":0,"satellites":8,"hdop":0.9,"fix":1,
 *    "utc":"2026-10-08T12:00:00Z"}                               → the fix (any subset)
 *   {"cmd":"gps_stop"}
 *   {"cmd":"gps_status"}
 *
 * The fields are checked as a whole before any is applied, so a bad value leaves the fix as
 * it was.  The receiver needs gateware v48 (UART2) and claims its TX pin as "gps_tx".
 */
#include "command_handler.h"
#include "command_handler_internal.h"
#include "at_driver.h"
#include "bp_json.h"
#include "la_pins.h"
#include "signal_engine.h"
#include "gps_sim.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define json_get_value bp_json_get

/* A number field: absent = true with *have false; present = parsed and range-checked. */
static bool get_num(const char *json, const char *key, double lo, double hi, double *out, bool *have,
                    char *err, size_t errcap) {
    char v[24] = {0};
    *have = false;
    if (!json_get_value(json, key, v, sizeof(v))) return true;
    char *end = NULL;
    double d = strtod(v, &end);
    if (end == v || *end != '\0' || !isfinite(d) || d < lo || d > hi) {
        snprintf(err, errcap, "%s must be a number from %ld to %ld", key, (long)lo, (long)hi);
        return false;
    }
    *out = d;
    *have = true;
    return true;
}

/* Apply the gps_set fields of `json` to *f (all-or-nothing). */
static bool parse_fix(const char *json, gps_fix_t *f, char *err, size_t errcap) {
    gps_fix_t n = *f;
    double d;
    bool have;
    if (!get_num(json, "latitude_deg", -90, 90, &d, &have, err, errcap)) return false;
    if (have) n.lat_deg = d;
    if (!get_num(json, "longitude_deg", -180, 180, &d, &have, err, errcap)) return false;
    if (have) n.lon_deg = d;
    if (!get_num(json, "altitude_m", -1000, 20000, &d, &have, err, errcap)) return false;
    if (have) n.alt_m = (float)d;
    if (!get_num(json, "speed_kmh", 0, 2000, &d, &have, err, errcap)) return false;
    if (have) n.speed_kmh = (float)d;
    if (!get_num(json, "course_deg", 0, 360, &d, &have, err, errcap)) return false;
    if (have) n.course_deg = (float)fmod(d, 360.0);
    if (!get_num(json, "hdop", 0.5, 99, &d, &have, err, errcap)) return false;
    if (have) n.hdop = (float)d;
    if (!get_num(json, "satellites", 0, GPS_MAX_SATS, &d, &have, err, errcap)) return false;
    if (have) n.sats = (uint8_t)lround(d);
    if (!get_num(json, "fix", 0, 2, &d, &have, err, errcap)) return false;
    if (have) n.fix = (uint8_t)lround(d);

    char utc[40] = {0};
    if (json_get_value(json, "utc", utc, sizeof(utc))) {
        int64_t t = -1;
        char *end = NULL;
        long long secs = strtoll(utc, &end, 10);
        if (end != utc && *end == '\0') t = secs;                 /* seconds since 1970 */
        else t = gps_nmea_parse_iso8601(utc);                     /* "2026-10-08T12:00:00Z" */
        if (t < 0 || t > 0xFFFFFFFFll) {
            snprintf(err, errcap, "utc must be like \"2026-10-08T12:00:00Z\" or seconds since 1970");
            return false;
        }
        n.utc_s = (uint32_t)t;
        n.utc_ms = 0;
    }
    *f = n;
    return true;
}

/* A double as JSON with `dec` decimals.  newlib-nano's printf has no %f and no %lld, so it is
   printed from 32-bit integers: every field here stays under 2^32 scaled (180 deg x 1e7). */
static void emit_fixed(bp_emit_t *e, double v, unsigned dec) {
    static const unsigned long p10[] = { 1, 10, 100, 1000, 10000, 100000, 1000000, 10000000 };
    double r = round(fabs(v) * (double)p10[dec]);
    unsigned long a = r > 4294967295.0 ? 4294967295ul : (unsigned long)r;
    const char *sign = (v < 0 && a) ? "-" : "";
    if (dec == 0) { bp_emit(e, "%s%lu", sign, a); return; }
    char frac[12];
    snprintf(frac, sizeof(frac), "%lu", a % p10[dec] + p10[dec]);   /* leading 1 keeps the zeros */
    bp_emit(e, "%s%lu.%s", sign, a / p10[dec], frac + 1);
}

static void emit_mask(bp_emit_t *e, unsigned mask) {
    static const char *const names[] = { "RMC", "VTG", "GGA", "GSA", "GSV", "GLL" };
    bp_emit_raw(e, "\"");
    bool first = true;
    for (unsigned i = 0; i < 6; i++)
        if (mask & (1u << i)) { bp_emit(e, "%s%s", first ? "" : ",", names[i]); first = false; }
    bp_emit_raw(e, "\"");
}

static void send_gps_status(int conn_id) {
    gps_sim_status_t st;
    gps_sim_status(&st);
    const gps_fix_t *f = gps_sim_fix();
    char resp[640];
    bp_emit_t e;
    bp_emit_init(&e, resp, sizeof(resp));
    bp_emit(&e, "{\"status\":\"ok\",\"data\":{\"active\":%s", st.active ? "true" : "false");
    if (st.active) {
        bp_emit(&e, ",\"tx\":%u,\"baud\":%lu,\"rate_hz\":%u,\"sentences\":",
                st.tx_ch, (unsigned long)st.baud, st.rate_hz);
        emit_mask(&e, st.mask);
        bp_emit(&e, ",\"epochs\":%lu,\"overruns\":%lu,\"bytes\":%lu",
                (unsigned long)st.epochs, (unsigned long)st.overruns, (unsigned long)st.bytes);
    }
    bp_emit_raw(&e, ",\"latitude_deg\":");  emit_fixed(&e, f->lat_deg, 7);
    bp_emit_raw(&e, ",\"longitude_deg\":"); emit_fixed(&e, f->lon_deg, 7);
    bp_emit_raw(&e, ",\"altitude_m\":");    emit_fixed(&e, f->alt_m, 1);
    bp_emit_raw(&e, ",\"speed_kmh\":");     emit_fixed(&e, f->speed_kmh, 2);
    bp_emit_raw(&e, ",\"course_deg\":");    emit_fixed(&e, f->course_deg, 2);
    bp_emit_raw(&e, ",\"hdop\":");          emit_fixed(&e, f->hdop, 2);
    bp_emit(&e, ",\"satellites\":%u,\"fix\":%u,\"utc\":%lu}}\n",
            (unsigned)f->sats, (unsigned)f->fix, (unsigned long)f->utc_s);
    if (!bp_emit_ok(&e)) { send_error(conn_id, "reply too large"); return; }
    if (at_send_data(conn_id, (const uint8_t *)resp, bp_emit_len(&e)) != 0) at_close_connection(conn_id);
}

void handle_gps_start(int conn_id, const char *json) {
    if (!require_la_voltage(conn_id)) return;
    char err[112];
    uint8_t ver = signal_engine_fpga_version();
    if (ver < UART2_MIN_GW) {
        snprintf(err, sizeof(err), "the GPS receiver needs gateware v%u or newer (this pod runs v%u)",
                 (unsigned)UART2_MIN_GW, (unsigned)ver);
        send_error(conn_id, err);
        return;
    }
    char v[48] = {0};
    if (!json_get_value(json, "tx", v, sizeof(v))) { send_error(conn_id, "missing tx (the LA channel the DUT's RX is on)"); return; }
    unsigned tx = (unsigned)atoi(v);
    if (tx < 1 || tx > 14) { send_error(conn_id, "tx must be an LA channel 1..14"); return; }

    uint32_t baud = GPS_DEFAULT_BAUD;
    if (json_get_value(json, "baud", v, sizeof(v))) {
        long b = atol(v);
        if (b < 300 || b > 921600) { send_error(conn_id, "baud must be 300..921600"); return; }
        baud = (uint32_t)b;
    }
    unsigned rate = 1;
    if (json_get_value(json, "rate_hz", v, sizeof(v))) {
        int r = atoi(v);
        if (r < 1 || r > (int)GPS_MAX_RATE_HZ) { send_error(conn_id, "rate_hz must be 1..10"); return; }
        rate = (unsigned)r;
    }
    unsigned mask = GPS_SENT_ALL;
    if (json_get_value(json, "sentences", v, sizeof(v))) {
        mask = gps_nmea_parse_mask(v);
        if (!mask) { send_error(conn_id, "sentences: a list of RMC, VTG, GGA, GSA, GSV, GLL"); return; }
    }
    gps_fix_t f = *gps_sim_fix();
    if (!parse_fix(json, &f, err, sizeof(err))) { send_error(conn_id, err); return; }

    /* The burst must fit the period: 10 bits a byte on the wire. */
    size_t bytes = gps_sim_epoch_bytes(&f, mask);
    if ((uint64_t)bytes * 10u * rate > baud) {
        snprintf(err, sizeof(err), "%u Hz of %u-byte epochs needs more than %lu baud: lower rate_hz, "
                 "drop sentences or raise baud", rate, (unsigned)bytes, (unsigned long)baud);
        send_error(conn_id, err);
        return;
    }

    /* A replacing gps_start may take over the running receiver's own pin. */
    uint8_t pin = (uint8_t)tx;
    if (!la_claim_or_error(conn_id, LA_FN_GPS_TX, &pin, 1, LA_FN_BIT(LA_FN_GPS_TX))) return;
    int rc = gps_sim_start(tx, baud, rate, mask);
    if (rc != 0) { send_error(conn_id, "UART2 refused the configuration"); return; }
    gps_sim_set_fix(&f);
    la_pins_release_fn(LA_FN_GPS_TX);    /* the replaced receiver's pin, if any */
    la_pins_claim(LA_FN_GPS_TX, LA_GPIO_NONE, 0, &pin, 1);
    send_gps_status(conn_id);
}

void handle_gps_set(int conn_id, const char *json) {
    char err[112];
    gps_fix_t f = *gps_sim_fix();
    if (!parse_fix(json, &f, err, sizeof(err))) { send_error(conn_id, err); return; }
    /* Without a session the values are kept for the next gps_start. */
    gps_sim_set_fix(&f);
    send_gps_status(conn_id);
}

void handle_gps_stop(int conn_id, const char *json) {
    (void)json;
    gps_sim_stop();
    la_pins_release_fn(LA_FN_GPS_TX);
    send_ok_str(conn_id, "null");
}

void handle_gps_status(int conn_id, const char *json) {
    (void)json;
    send_gps_status(conn_id);
}

/* command_handler_poll's GPS pass: the next epoch and the FIFO feed. */
void gps_poll(void) {
    if (gps_sim_poll() != 0) la_pins_release_fn(LA_FN_GPS_TX);
}
