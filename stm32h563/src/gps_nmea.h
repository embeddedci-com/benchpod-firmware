/* ============================================================================
 * gps_nmea.h — NMEA 0183 sentences of an emulated GPS receiver (pure, no I/O).
 *
 * Builds one epoch (one position fix) as the sentences a u-blox style receiver
 * prints once per fix, in its default order: RMC, VTG, GGA, GSA, GSV, GLL, all
 * with the "GP" talker, NMEA 4.x mode indicators and "*hh\r\n" checksums.
 * Without a fix (fix = 0) the position fields are empty and the status/mode
 * fields say so (RMC "V", GGA quality 0, GSA mode 1), as a real receiver does
 * before it locks.  The satellites in GSV/GSA are synthetic but stable.
 *
 * newlib-nano's printf has no %f, so every number is formatted from integers.
 * ========================================================================== */
#pragma once

#include <stddef.h>
#include <stdint.h>

enum {
    GPS_SENT_RMC = 1u << 0,
    GPS_SENT_VTG = 1u << 1,
    GPS_SENT_GGA = 1u << 2,
    GPS_SENT_GSA = 1u << 3,
    GPS_SENT_GSV = 1u << 4,
    GPS_SENT_GLL = 1u << 5,
    GPS_SENT_ALL = 0x3Fu,
};

#define GPS_MAX_SATS 12u

typedef struct {
    double   lat_deg, lon_deg;  /* north / east positive */
    float    alt_m;             /* above mean sea level */
    float    speed_kmh;         /* over ground */
    float    course_deg;        /* true, 0..360 */
    float    hdop;
    uint8_t  sats;              /* satellites used, 0..GPS_MAX_SATS */
    uint8_t  fix;               /* 0 no fix, 1 GPS, 2 DGPS */
    uint32_t utc_s;             /* seconds since 1970-01-01T00:00:00Z */
    uint16_t utc_ms;            /* 0..999, for rates above 1 Hz */
} gps_fix_t;

/* Parse "RMC,GGA,..." (any case, any order) into a GPS_SENT_* mask; 0 on an
   unknown name or an empty list. */
unsigned gps_nmea_parse_mask(const char *list);

/* Write the epoch's sentences (those in `mask`) into out; returns the length
   (no NUL counted), or 0 if they do not fit `cap` (cap includes the NUL). */
size_t gps_nmea_epoch(const gps_fix_t *f, unsigned mask, char *out, size_t cap);

/* Advance the fix by dt_ms: the clock, and the position along course_deg at
   speed_kmh (flat-earth step, fine for the short hops between fixes). */
void gps_nmea_advance(gps_fix_t *f, uint32_t dt_ms);

/* "2026-10-08T12:34:56Z" (or with ".sss") -> seconds since 1970; -1 on a bad string. */
int64_t gps_nmea_parse_iso8601(const char *s);
