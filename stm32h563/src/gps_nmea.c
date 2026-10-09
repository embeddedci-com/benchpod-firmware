/* ============================================================================
 * gps_nmea.c — NMEA 0183 sentence builder.  See gps_nmea.h.
 * ========================================================================== */

#include "gps_nmea.h"

#include <ctype.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define KMH_PER_KNOT 1.852
#define EARTH_M_PER_DEG 111320.0
#define DEG_TO_RAD      (3.14159265358979323846 / 180.0)   /* one degree of latitude; longitude scales by cos(lat) */

/* ---- small appender ---- */
typedef struct { char *buf; size_t cap, len; int ok; } sb_t;

static void sb_add(sb_t *b, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static void sb_add(sb_t *b, const char *fmt, ...) {
    if (!b->ok) return;
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(b->buf + b->len, b->cap - b->len, fmt, ap);
    va_end(ap);
    if (n < 0 || (size_t)n >= b->cap - b->len) { b->ok = 0; return; }
    b->len += (size_t)n;
}

/* A non-negative value with `dec` decimals, rounded ("12.345"). */
static void sb_fixed(sb_t *b, double v, unsigned dec) {
    static const unsigned long p10[] = { 1, 10, 100, 1000, 10000, 100000 };
    if (v < 0) v = 0;
    unsigned long scaled = (unsigned long)llround(v * (double)p10[dec]);
    if (dec == 0) { sb_add(b, "%lu", scaled); return; }
    char frac[8];   /* no '*' width: a leading 1 keeps the fraction's zeros */
    snprintf(frac, sizeof(frac), "%lu", scaled % p10[dec] + p10[dec]);
    sb_add(b, "%lu.%s", scaled / p10[dec], frac + 1);
}

/* A signed value with `dec` decimals ("-12.3"). */
static void sb_signed(sb_t *b, double v, unsigned dec) {
    if (v < 0) { sb_add(b, "-"); v = -v; }
    sb_fixed(b, v, dec);
}

/* Start a sentence body: "$GPxxx" (the checksum runs from after '$'). */
static size_t sb_begin(sb_t *b, const char *type) {
    size_t at = b->len;
    sb_add(b, "$GP%s", type);
    return at;
}
static void sb_end(sb_t *b, size_t at) {
    if (!b->ok) return;
    uint8_t cs = 0;
    for (size_t i = at + 1; i < b->len; i++) cs ^= (uint8_t)b->buf[i];
    sb_add(b, "*%02X\r\n", cs);
}

/* ---- fields ---- */
static void civil_from_days(int64_t z, int *y, unsigned *m, unsigned *d) {
    /* Howard Hinnant's days -> civil date, valid for any day. */
    z += 719468;
    int64_t era = (z >= 0 ? z : z - 146096) / 146097;
    unsigned doe = (unsigned)(z - era * 146097);
    unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    int64_t yy = (int64_t)yoe + era * 400;
    unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    unsigned mp = (5 * doy + 2) / 153;
    *d = doy - (153 * mp + 2) / 5 + 1;
    *m = mp < 10 ? mp + 3 : mp - 9;
    *y = (int)(yy + (*m <= 2));
}

static int64_t days_from_civil(int y, unsigned m, unsigned d) {
    y -= m <= 2;
    int64_t era = (y >= 0 ? y : y - 399) / 400;
    unsigned yoe = (unsigned)(y - era * 400);
    unsigned doy = (153 * (m > 2 ? m - 3 : m + 9) + 2) / 5 + d - 1;
    unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + (int64_t)doe - 719468;
}

static void sb_time(sb_t *b, const gps_fix_t *f) {   /* hhmmss.ss */
    uint32_t s = f->utc_s % 86400u;
    sb_add(b, "%02lu%02lu%02lu.%02u", (unsigned long)(s / 3600u), (unsigned long)(s / 60u % 60u),
           (unsigned long)(s % 60u), (unsigned)(f->utc_ms / 10u));
}

static void sb_date(sb_t *b, const gps_fix_t *f) {   /* ddmmyy */
    int y; unsigned m, d;
    civil_from_days((int64_t)(f->utc_s / 86400u), &y, &m, &d);
    sb_add(b, "%02u%02u%02u", d, m, (unsigned)(y % 100));
}

/* ddmm.mmmmm,N / dddmm.mmmmm,E — or ",," without a fix. */
static void sb_coord(sb_t *b, double v, unsigned deg_digits, char pos, char neg, int have) {
    if (!have) { sb_add(b, ","); return; }
    char hemi = v < 0 ? neg : pos;
    v = fabs(v);
    unsigned long mmin = (unsigned long)llround(v * 60.0 * 100000.0);   /* 1e-5 minutes */
    unsigned long deg = mmin / (60ul * 100000ul);
    unsigned long rem = mmin % (60ul * 100000ul);
    if (deg_digits == 3) sb_add(b, "%03lu", deg);
    else                 sb_add(b, "%02lu", deg);
    sb_add(b, "%02lu.%05lu,%c", rem / 100000ul, rem % 100000ul, hemi);
}

static char mode_char(const gps_fix_t *f) { return f->fix == 0 ? 'N' : (f->fix == 2 ? 'D' : 'A'); }

/* Synthetic, stable sky: PRN, elevation, azimuth, SNR per slot. */
static const uint8_t k_prn[GPS_MAX_SATS] = { 2, 5, 7, 9, 12, 13, 15, 18, 20, 23, 25, 29 };
static void sat_info(unsigned i, unsigned *el, unsigned *az, unsigned *snr) {
    *el  = 15u + (i * 37u) % 70u;
    *az  = (i * 97u + 20u) % 360u;
    *snr = 32u + (i * 7u) % 14u;
}

/* ---- sentences ---- */
static void rmc(sb_t *b, const gps_fix_t *f) {
    int have = f->fix != 0;
    size_t at = sb_begin(b, "RMC");
    sb_add(b, ","); sb_time(b, f);
    sb_add(b, ",%c,", have ? 'A' : 'V');
    sb_coord(b, f->lat_deg, 2, 'N', 'S', have); sb_add(b, ",");
    sb_coord(b, f->lon_deg, 3, 'E', 'W', have); sb_add(b, ",");
    if (have) sb_fixed(b, f->speed_kmh / KMH_PER_KNOT, 3);
    sb_add(b, ",");
    if (have) sb_fixed(b, f->course_deg, 2);
    sb_add(b, ","); sb_date(b, f);
    sb_add(b, ",,,%c", mode_char(f));
    sb_end(b, at);
}

static void vtg(sb_t *b, const gps_fix_t *f) {
    int have = f->fix != 0;
    size_t at = sb_begin(b, "VTG");
    sb_add(b, ",");
    if (have) sb_fixed(b, f->course_deg, 2);
    sb_add(b, ",T,,M,");
    if (have) sb_fixed(b, f->speed_kmh / KMH_PER_KNOT, 3);
    sb_add(b, ",N,");
    if (have) sb_fixed(b, f->speed_kmh, 3);
    sb_add(b, ",K,%c", mode_char(f));
    sb_end(b, at);
}

static void gga(sb_t *b, const gps_fix_t *f) {
    int have = f->fix != 0;
    size_t at = sb_begin(b, "GGA");
    sb_add(b, ","); sb_time(b, f); sb_add(b, ",");
    sb_coord(b, f->lat_deg, 2, 'N', 'S', have); sb_add(b, ",");
    sb_coord(b, f->lon_deg, 3, 'E', 'W', have);
    sb_add(b, ",%u,%02u,", (unsigned)f->fix, have ? (unsigned)f->sats : 0u);
    if (have) sb_fixed(b, f->hdop, 2);
    sb_add(b, ",");
    if (have) { sb_signed(b, f->alt_m, 1); sb_add(b, ",M,0.0,M,,"); }
    else sb_add(b, ",,,,,");
    sb_end(b, at);
}

static void gsa(sb_t *b, const gps_fix_t *f) {
    int have = f->fix != 0;
    size_t at = sb_begin(b, "GSA");
    sb_add(b, ",A,%u", have ? 3u : 1u);
    for (unsigned i = 0; i < GPS_MAX_SATS; i++) {
        if (have && i < f->sats) sb_add(b, ",%02u", (unsigned)k_prn[i]);
        else sb_add(b, ",");
    }
    sb_add(b, ",");
    if (have) {
        sb_fixed(b, f->hdop * 1.3, 2); sb_add(b, ",");   /* PDOP */
        sb_fixed(b, f->hdop, 2);       sb_add(b, ",");   /* HDOP */
        sb_fixed(b, f->hdop * 0.9, 2);                   /* VDOP */
    } else {
        sb_add(b, "99.99,99.99,99.99");
    }
    sb_add(b, ",1");   /* NMEA 4.1 system ID: GPS */
    sb_end(b, at);
}

static void gsv(sb_t *b, const gps_fix_t *f) {
    /* Satellites in view: the used ones plus two more seen but not used (a real sky). */
    unsigned n = f->fix ? f->sats + 2u : 4u;
    if (n > GPS_MAX_SATS) n = GPS_MAX_SATS;
    unsigned msgs = (n + 3u) / 4u;
    if (msgs == 0) msgs = 1;
    for (unsigned m = 0; m < msgs; m++) {
        size_t at = sb_begin(b, "GSV");
        sb_add(b, ",%u,%u,%02u", msgs, m + 1u, n);
        for (unsigned k = 0; k < 4u && m * 4u + k < n; k++) {
            unsigned i = m * 4u + k, el, az, snr;
            sat_info(i, &el, &az, &snr);
            if (f->fix) sb_add(b, ",%02u,%02u,%03u,%02u", (unsigned)k_prn[i], el, az, snr);
            else        sb_add(b, ",%02u,%02u,%03u,", (unsigned)k_prn[i], el, az);   /* no SNR yet */
        }
        sb_add(b, ",1");   /* signal ID: L1 C/A */
        sb_end(b, at);
    }
}

static void gll(sb_t *b, const gps_fix_t *f) {
    int have = f->fix != 0;
    size_t at = sb_begin(b, "GLL");
    sb_add(b, ",");
    sb_coord(b, f->lat_deg, 2, 'N', 'S', have); sb_add(b, ",");
    sb_coord(b, f->lon_deg, 3, 'E', 'W', have); sb_add(b, ",");
    sb_time(b, f);
    sb_add(b, ",%c,%c", have ? 'A' : 'V', mode_char(f));
    sb_end(b, at);
}

size_t gps_nmea_epoch(const gps_fix_t *f, unsigned mask, char *out, size_t cap) {
    if (!f || !out || cap == 0) return 0;
    sb_t b = { out, cap, 0, 1 };
    out[0] = '\0';
    if (mask & GPS_SENT_RMC) rmc(&b, f);
    if (mask & GPS_SENT_VTG) vtg(&b, f);
    if (mask & GPS_SENT_GGA) gga(&b, f);
    if (mask & GPS_SENT_GSA) gsa(&b, f);
    if (mask & GPS_SENT_GSV) gsv(&b, f);
    if (mask & GPS_SENT_GLL) gll(&b, f);
    if (!b.ok) { out[0] = '\0'; return 0; }
    return b.len;
}

unsigned gps_nmea_parse_mask(const char *list) {
    static const struct { const char *name; unsigned bit; } k[] = {
        { "RMC", GPS_SENT_RMC }, { "VTG", GPS_SENT_VTG }, { "GGA", GPS_SENT_GGA },
        { "GSA", GPS_SENT_GSA }, { "GSV", GPS_SENT_GSV }, { "GLL", GPS_SENT_GLL },
    };
    if (!list) return 0;
    unsigned mask = 0;
    const char *p = list;
    while (*p) {
        while (*p == ',' || *p == ' ') p++;
        if (!*p) break;
        char name[4] = {0};
        unsigned n = 0;
        while (*p && *p != ',' && *p != ' ') {
            if (n >= 3) return 0;
            name[n++] = (char)toupper((unsigned char)*p++);
        }
        unsigned bit = 0;
        for (unsigned i = 0; i < sizeof(k) / sizeof(k[0]); i++)
            if (strcmp(name, k[i].name) == 0) bit = k[i].bit;
        if (!bit) return 0;
        mask |= bit;
    }
    return mask;
}

void gps_nmea_advance(gps_fix_t *f, uint32_t dt_ms) {
    uint32_t ms = f->utc_ms + dt_ms;
    f->utc_s += ms / 1000u;
    f->utc_ms = (uint16_t)(ms % 1000u);
    if (f->speed_kmh <= 0.0f) return;
    double dist_m = (double)f->speed_kmh / 3.6 * (double)dt_ms / 1000.0;
    double c = (double)f->course_deg * DEG_TO_RAD;
    f->lat_deg += dist_m * cos(c) / EARTH_M_PER_DEG;
    double coslat = cos(f->lat_deg * DEG_TO_RAD);
    if (fabs(coslat) > 1e-6) f->lon_deg += dist_m * sin(c) / (EARTH_M_PER_DEG * coslat);
    if (f->lat_deg > 90.0)  f->lat_deg = 90.0;
    if (f->lat_deg < -90.0) f->lat_deg = -90.0;
    if (f->lon_deg > 180.0)  f->lon_deg -= 360.0;
    if (f->lon_deg < -180.0) f->lon_deg += 360.0;
}

int64_t gps_nmea_parse_iso8601(const char *s) {
    int y; unsigned mo, d, h, mi, se;
    int n = 0;
    if (!s || sscanf(s, "%4d-%2u-%2uT%2u:%2u:%2u%n", &y, &mo, &d, &h, &mi, &se, &n) != 6) return -1;
    const char *p = s + n;
    if (*p == '.') { p++; while (isdigit((unsigned char)*p)) p++; }
    if (*p != 'Z' || p[1] != '\0') return -1;
    if (y < 1970 || y > 2105 || mo < 1 || mo > 12 || d < 1 || d > 31 || h > 23 || mi > 59 || se > 60)
        return -1;
    return days_from_civil(y, mo, d) * 86400 + (int64_t)h * 3600 + (int64_t)mi * 60 + se;
}
