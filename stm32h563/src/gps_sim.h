/* ============================================================================
 * gps_sim.h — emulated GPS receiver: NMEA 0183 over UART2 (gateware v48+).
 *
 * The pod plays a GPS module on one LA channel: every 1/rate_hz seconds it
 * prints one epoch of sentences (gps_nmea.c) at `baud` on the UART2 TX pin,
 * the way a u-blox NEO-6M/M8N does out of the box (9600 baud, 1 Hz, RMC VTG GGA
 * GSA GSV GLL).  The position moves along course/speed between fixes, and the
 * clock advances, so a DUT's parser sees a live receiver.  UART2 is separate
 * from the UART proxy, so the DUT's console keeps working alongside.
 *
 * An epoch goes into a staging buffer and out to the 256-byte UART2 FIFO in
 * pieces whenever the FIFO reports empty (gps_sim_poll, from the command poll).
 * An epoch that falls due while the previous one is still being sent is
 * skipped and counted (`overruns`): the baud rate is too low for the rate and
 * the sentence set.
 * ========================================================================== */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "gps_nmea.h"

#define GPS_DEFAULT_BAUD 9600u
#define GPS_MAX_RATE_HZ  10u

/* Start on 1-based LA channel tx_ch.  The fix keeps the values it had (defaults
   after boot: 0 N 0 E, no movement, 8 satellites, fix 1, 2026-01-01T00:00:00Z).
   0 ok, -1 bad argument, -3 gateware older than UART2_MIN_GW, -4 UART2 refused. */
int  gps_sim_start(unsigned tx_ch, uint32_t baud, unsigned rate_hz, unsigned mask);
void gps_sim_stop(void);

/* The fix the next epochs report (gps_sim_fix() to read it, gps_sim_set_fix() to
   replace it; the caller validates ranges). */
const gps_fix_t *gps_sim_fix(void);
void             gps_sim_set_fix(const gps_fix_t *f);

/* Main-loop service: build a due epoch, feed the UART2 FIFO, re-arm after a gateware
   reconfiguration.  Returns -1 when that re-arm failed (the receiver stopped). */
int  gps_sim_poll(void);
void gps_sim_on_gateware_reconfigured(void);

typedef struct {
    bool     active;
    unsigned tx_ch;
    uint32_t baud;
    unsigned rate_hz;
    unsigned mask;
    uint32_t epochs;     /* epochs sent since gps_sim_start */
    uint32_t overruns;   /* epochs skipped: the previous one was still going out */
    uint32_t bytes;      /* NMEA bytes handed to UART2 */
} gps_sim_status_t;
void gps_sim_status(gps_sim_status_t *out);

/* Bytes one epoch of `mask` takes for fix f (to warn about a baud that cannot carry it). */
size_t gps_sim_epoch_bytes(const gps_fix_t *f, unsigned mask);
