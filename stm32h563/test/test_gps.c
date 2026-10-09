/*
 * test_gps.c — host tests for the emulated GPS receiver: the NMEA sentences (src/gps_nmea.c)
 * parsed back the way a DUT's parser reads them, and the runtime (src/gps_sim.c) against a
 * fake UART2: epochs on time, the 256-byte FIFO fed only when it is empty, overruns counted,
 * the re-arm after a gateware reconfiguration.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "gps_nmea.h"
#include "gps_sim.h"
#include "signal_engine.h"

static int fails;
#define CHECK(cond, ...) do { if (!(cond)) { printf("FAIL %s:%d: ", __func__, __LINE__); \
        printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

/* ---- fake UART2 + clock -------------------------------------------------------------------- */
static uint64_t t_now_us = 5000000;
uint64_t time_us_64(void) { return t_now_us; }
static int      t_cfg_rc, t_cfgs;
static bool     t_armed, t_empty = true;
static unsigned t_tx;
static char     t_wire[8192];
static size_t   t_nwire, t_max_write;
int fpga_uart2_config(unsigned tx, uint32_t baud, bool en) {
    (void)baud; if (t_cfg_rc) return t_cfg_rc; t_cfgs++; t_armed = en; t_tx = tx; return 0;
}
void fpga_uart2_fifo_write(const uint8_t *d, size_t n) {
    if (n > t_max_write) t_max_write = n;
    memcpy(t_wire + t_nwire, d, n); t_nwire += n;
    t_empty = false;                 /* the FIFO holds them until the test drains it */
}
int fpga_uart_status(uint16_t *avail, uint8_t *flags) {
    if (avail) *avail = 0;
    if (flags) *flags = (uint8_t)((t_empty ? UART2_STATUS_TX_EMPTY : 0) | (t_armed ? UART2_STATUS_ARMED : 0));
    return 0;
}

/* ---- a DUT-side parser ---------------------------------------------------------------------- */
static int line_ok(const char *l, size_t n) {   /* "$...*hh\r\n" with a matching checksum */
    if (n < 9 || l[0] != '$' || l[n - 2] != '\r' || l[n - 1] != '\n' || l[n - 5] != '*') return 0;
    unsigned cs = 0;
    for (size_t i = 1; i < n - 5; i++) cs ^= (unsigned char)l[i];
    return (unsigned)strtoul(l + n - 4, NULL, 16) == cs;
}
/* Field i (0 = "$GPxxx") of the first sentence of `type` in buf, into out. */
static int field(const char *buf, const char *type, unsigned i, char *out, size_t cap) {
    char tag[8];
    snprintf(tag, sizeof(tag), "$GP%s", type);
    const char *p = strstr(buf, tag);
    if (!p) return 0;
    for (unsigned k = 0; k < i; k++) {
        p = strpbrk(p, ",*");
        if (!p || *p == '*') return 0;
        p++;
    }
    size_t n = strcspn(p, ",*");
    if (n >= cap) return 0;
    memcpy(out, p, n); out[n] = '\0';
    return 1;
}
static double nmea_deg(const char *v, const char *hemi, unsigned deg_digits) {
    char d[4] = {0};
    memcpy(d, v, deg_digits);
    double deg = atof(d) + atof(v + deg_digits) / 60.0;
    return (hemi[0] == 'S' || hemi[0] == 'W') ? -deg : deg;
}

static gps_fix_t brussels(void) {
    gps_fix_t f = {0};
    f.lat_deg = 50.8466123; f.lon_deg = 4.3527987; f.alt_m = 56.4f;
    f.speed_kmh = 36.0f; f.course_deg = 90.0f; f.hdop = 0.9f; f.sats = 9; f.fix = 1;
    f.utc_s = (uint32_t)gps_nmea_parse_iso8601("2026-10-08T12:34:56Z");
    return f;
}

static void test_sentences(void) {
    char buf[1024], v[32], h[4];
    gps_fix_t f = brussels();
    size_t n = gps_nmea_epoch(&f, GPS_SENT_ALL, buf, sizeof(buf));
    CHECK(n > 0 && n == strlen(buf), "epoch built (%zu)", n);

    /* every line framed and checksummed; the u-blox default order */
    const char *order[] = { "RMC", "VTG", "GGA", "GSA", "GSV", "GSV", "GSV", "GLL" };
    unsigned lines = 0;
    for (const char *p = buf; *p; ) {
        const char *e = strstr(p, "\r\n");
        CHECK(e != NULL, "line ends in CRLF");
        if (!e) break;
        size_t len = (size_t)(e + 2 - p);
        CHECK(line_ok(p, len), "checksum: %.*s", (int)len - 2, p);
        if (lines < 8) CHECK(strncmp(p + 3, order[lines], 3) == 0, "line %u is %s: %.6s", lines, order[lines], p);
        lines++;
        p = e + 2;
    }
    CHECK(lines == 8, "9 used + 2 seen = 11 in view = 3 GSV: 8 lines, got %u", lines);

    /* RMC: time, status, position, speed (knots), course, date */
    field(buf, "RMC", 1, v, sizeof(v)); CHECK(strcmp(v, "123456.00") == 0, "RMC time %s", v);
    field(buf, "RMC", 2, v, sizeof(v)); CHECK(strcmp(v, "A") == 0, "RMC status %s", v);
    field(buf, "RMC", 3, v, sizeof(v)); field(buf, "RMC", 4, h, sizeof(h));
    CHECK(fabs(nmea_deg(v, h, 2) - f.lat_deg) < 2e-7, "RMC lat %s %s", v, h);
    field(buf, "RMC", 5, v, sizeof(v)); field(buf, "RMC", 6, h, sizeof(h));
    CHECK(fabs(nmea_deg(v, h, 3) - f.lon_deg) < 2e-7 && strlen(v) == 11, "RMC lon %s %s", v, h);
    field(buf, "RMC", 7, v, sizeof(v)); CHECK(fabs(atof(v) - 36.0 / 1.852) < 0.001, "RMC knots %s", v);
    field(buf, "RMC", 8, v, sizeof(v)); CHECK(strcmp(v, "90.00") == 0, "RMC course %s", v);
    field(buf, "RMC", 9, v, sizeof(v)); CHECK(strcmp(v, "081026") == 0, "RMC date %s", v);
    field(buf, "RMC", 12, v, sizeof(v)); CHECK(strcmp(v, "A") == 0, "RMC mode %s", v);

    /* GGA: quality, satellites, HDOP, altitude */
    field(buf, "GGA", 6, v, sizeof(v)); CHECK(strcmp(v, "1") == 0, "GGA quality %s", v);
    field(buf, "GGA", 7, v, sizeof(v)); CHECK(strcmp(v, "09") == 0, "GGA sats %s", v);
    field(buf, "GGA", 8, v, sizeof(v)); CHECK(strcmp(v, "0.90") == 0, "GGA hdop %s", v);
    field(buf, "GGA", 9, v, sizeof(v)); CHECK(strcmp(v, "56.4") == 0, "GGA alt %s", v);
    field(buf, "VTG", 7, v, sizeof(v)); CHECK(strcmp(v, "36.000") == 0, "VTG km/h %s", v);
    field(buf, "GSV", 3, v, sizeof(v)); CHECK(strcmp(v, "11") == 0, "GSV in view %s", v);

    /* southern / western hemisphere, negative altitude */
    f.lat_deg = -33.8688; f.lon_deg = -151.2093; f.alt_m = -12.5f;
    gps_nmea_epoch(&f, GPS_SENT_GGA, buf, sizeof(buf));
    field(buf, "GGA", 2, v, sizeof(v)); field(buf, "GGA", 3, h, sizeof(h));
    CHECK(h[0] == 'S' && fabs(nmea_deg(v, h, 2) + 33.8688) < 2e-7, "south %s %s", v, h);
    field(buf, "GGA", 4, v, sizeof(v)); field(buf, "GGA", 5, h, sizeof(h));
    CHECK(h[0] == 'W' && fabs(nmea_deg(v, h, 3) + 151.2093) < 2e-7, "west %s %s", v, h);
    field(buf, "GGA", 9, v, sizeof(v)); CHECK(strcmp(v, "-12.5") == 0, "negative altitude %s", v);

    /* no fix: empty position, V / quality 0 / mode 1, like a receiver before lock */
    f = brussels();
    f.fix = 0;
    n = gps_nmea_epoch(&f, GPS_SENT_ALL, buf, sizeof(buf));
    CHECK(n > 0, "no-fix epoch");
    field(buf, "RMC", 2, v, sizeof(v)); CHECK(strcmp(v, "V") == 0, "no fix RMC status %s", v);
    field(buf, "RMC", 3, v, sizeof(v)); CHECK(v[0] == '\0', "no fix RMC lat empty '%s'", v);
    field(buf, "GGA", 6, v, sizeof(v)); CHECK(strcmp(v, "0") == 0, "no fix GGA quality %s", v);
    field(buf, "GSA", 2, v, sizeof(v)); CHECK(strcmp(v, "1") == 0, "no fix GSA mode %s", v);
    field(buf, "GLL", 6, v, sizeof(v)); CHECK(strcmp(v, "V") == 0, "no fix GLL status %s", v);
    for (const char *p = buf; *p; ) {
        const char *e = strstr(p, "\r\n");
        if (!e) break;
        CHECK(line_ok(p, (size_t)(e + 2 - p)), "no-fix checksum: %.*s", (int)(e - p), p);
        p = e + 2;
    }

    /* DGPS, a subset, the worst case fits the runtime's staging buffer */
    f = brussels(); f.fix = 2;
    n = gps_nmea_epoch(&f, GPS_SENT_RMC | GPS_SENT_GGA, buf, sizeof(buf));
    CHECK(strstr(buf, "$GPVTG") == NULL && strstr(buf, "$GPRMC") && strstr(buf, "$GPGGA"), "subset");
    field(buf, "GGA", 6, v, sizeof(v)); CHECK(strcmp(v, "2") == 0, "DGPS quality %s", v);
    field(buf, "RMC", 12, v, sizeof(v)); CHECK(strcmp(v, "D") == 0, "DGPS mode %s", v);
    f.sats = GPS_MAX_SATS; f.alt_m = -999.9f; f.lat_deg = -89.999999; f.lon_deg = -179.999999;
    f.speed_kmh = 1999.0f; f.course_deg = 359.99f; f.hdop = 99.0f;
    size_t worst = gps_nmea_epoch(&f, GPS_SENT_ALL, buf, sizeof(buf));
    CHECK(worst > 0 && worst < 600, "worst-case epoch %zu bytes", worst);
    CHECK(gps_nmea_epoch(&f, GPS_SENT_ALL, buf, 100) == 0 && buf[0] == '\0', "too small a buffer");
}

static void test_helpers(void) {
    CHECK(gps_nmea_parse_mask("RMC,GGA") == (GPS_SENT_RMC | GPS_SENT_GGA), "mask");
    CHECK(gps_nmea_parse_mask("gll, vtg") == (GPS_SENT_GLL | GPS_SENT_VTG), "mask, any case and spaces");
    CHECK(gps_nmea_parse_mask("RMC,ZDA") == 0 && gps_nmea_parse_mask("") == 0, "mask rejects");
    CHECK(gps_nmea_parse_iso8601("1970-01-01T00:00:00Z") == 0, "epoch 0");
    CHECK(gps_nmea_parse_iso8601("2028-02-29T23:59:59.5Z") == 1835481599, "leap day");
    CHECK(gps_nmea_parse_iso8601("2026-10-08 12:00:00") == -1, "no T/Z");
    CHECK(gps_nmea_parse_iso8601("2026-13-01T00:00:00Z") == -1, "month 13");

    /* dead reckoning: 36 km/h due east for 1 s = 10 m; the clock moves */
    gps_fix_t f = brussels();
    double lon0 = f.lon_deg, lat0 = f.lat_deg;
    gps_nmea_advance(&f, 1000);
    double dx = (f.lon_deg - lon0) * 111320.0 * cos(lat0 * 3.14159265358979 / 180.0);
    CHECK(fabs(dx - 10.0) < 0.01 && fabs(f.lat_deg - lat0) < 1e-9, "moved %.4f m east", dx);
    CHECK(f.utc_s == brussels().utc_s + 1 && f.utc_ms == 0, "clock +1 s");
    gps_nmea_advance(&f, 250);
    CHECK(f.utc_ms == 250, "sub-second clock %u", f.utc_ms);
    f.course_deg = 0.0f;   /* due north: 36 km/h for 100 ms = 1 m of latitude */
    lat0 = f.lat_deg;
    gps_nmea_advance(&f, 100);
    CHECK(fabs((f.lat_deg - lat0) * 111320.0 - 1.0) < 1e-6, "north 1 m");
    f.speed_kmh = 0.0f; lat0 = f.lat_deg; lon0 = f.lon_deg;
    gps_nmea_advance(&f, 1000);
    CHECK(f.lat_deg == lat0 && f.lon_deg == lon0, "standing still");
}

static void drain(void) { t_empty = true; }   /* the shifter sent everything */

static void test_runtime(void) {
    gps_fix_t f = brussels();
    f.speed_kmh = 0;
    gps_sim_set_fix(&f);
    CHECK(gps_sim_start(5, 9600, 0, GPS_SENT_ALL) == -1, "rate 0 refused");
    CHECK(gps_sim_start(5, 9600, 11, GPS_SENT_ALL) == -1, "rate 11 refused");
    CHECK(gps_sim_start(5, 9600, 1, 0) == -1, "empty mask refused");
    t_cfg_rc = -3;
    CHECK(gps_sim_start(5, 9600, 1, GPS_SENT_ALL) == -3, "old gateware reported");
    t_cfg_rc = 0;
    CHECK(gps_sim_start(5, 9600, 1, GPS_SENT_ALL) == 0 && t_armed && t_tx == 5, "started");

    /* first poll: one epoch, the first FIFO-full of it */
    CHECK(gps_sim_poll() == 0, "poll");
    size_t epoch = gps_sim_epoch_bytes(&f, GPS_SENT_ALL);
    CHECK(t_nwire == 256 && epoch > 256, "first piece is a FIFO's worth (%zu of %zu)", t_nwire, epoch);
    CHECK(gps_sim_poll() == 0 && t_nwire == 256, "nothing more until the FIFO is empty");
    drain(); gps_sim_poll();
    CHECK(t_nwire == epoch && t_max_write <= FPGA_UART2_FIFO, "rest of the epoch (%zu)", t_nwire);
    t_wire[t_nwire] = '\0';
    char expect[1024];
    gps_nmea_epoch(&f, GPS_SENT_ALL, expect, sizeof(expect));
    CHECK(strcmp(t_wire, expect) == 0, "the wire carries the epoch byte for byte");

    /* next epoch one period later, with the clock one second on */
    drain(); t_now_us += 500000; gps_sim_poll();
    CHECK(t_nwire == epoch, "no epoch before the period");
    t_now_us += 500000; gps_sim_poll();
    CHECK(t_nwire > epoch, "second epoch at 1 s");
    CHECK(strstr(t_wire + epoch, "123457.00") != NULL, "second epoch is at 12:34:57");

    /* the FIFO never empties (no DUT clocking it out... a stuck line): overruns, no pile-up */
    t_empty = false;
    size_t before = t_nwire;
    for (int i = 0; i < 3; i++) { t_now_us += 1000000; gps_sim_poll(); }
    gps_sim_status_t st;
    gps_sim_status(&st);
    CHECK(st.overruns == 3 && t_nwire == before, "3 overruns, nothing written (%lu)", (unsigned long)st.overruns);
    CHECK(st.active && st.tx_ch == 5 && st.rate_hz == 1 && st.baud == 9600, "status");

    /* a worker stall of 10 s does not fire 10 epochs at once */
    drain();
    for (int i = 0; i < 4; i++) gps_sim_poll();   /* finish the pending epoch */
    gps_sim_status(&st);
    uint32_t epochs = st.epochs;
    t_now_us += 10000000;
    gps_sim_poll(); drain(); gps_sim_poll(); drain(); gps_sim_poll();
    gps_sim_status(&st);
    CHECK(st.epochs == epochs + 1, "one epoch after a stall, not ten (%lu -> %lu)",
          (unsigned long)epochs, (unsigned long)st.epochs);

    /* gateware reconfiguration: re-armed on the next poll; a failure stops it */
    t_cfgs = 0;
    gps_sim_on_gateware_reconfigured();
    CHECK(gps_sim_poll() == 0 && t_cfgs == 1 && t_armed, "re-armed");
    gps_sim_on_gateware_reconfigured();
    t_cfg_rc = -1;
    CHECK(gps_sim_poll() == -1, "failed re-arm reported");
    gps_sim_status(&st);
    CHECK(!st.active, "stopped after the failed re-arm");
    t_cfg_rc = 0;

    CHECK(gps_sim_start(7, 115200, 10, GPS_SENT_RMC | GPS_SENT_GGA) == 0, "restart at 10 Hz");
    t_nwire = 0; drain();
    for (int i = 0; i < 10; i++) { gps_sim_poll(); drain(); gps_sim_poll(); t_now_us += 100000; }
    gps_sim_status(&st);
    CHECK(st.epochs == 10 && st.overruns == 0, "10 epochs in 1 s at 10 Hz (%lu)", (unsigned long)st.epochs);
    t_wire[t_nwire] = '\0';
    CHECK(strstr(t_wire, ".10,") && strstr(t_wire, ".90,"), "tenths of a second in the timestamps");
    gps_sim_stop();
    CHECK(!t_armed, "stop disarms UART2");
}

int main(void) {
    test_sentences();
    test_helpers();
    test_runtime();
    if (fails) { printf("test_gps: %d FAILED\n", fails); return 1; }
    printf("test_gps: all passed\n");
    return 0;
}
