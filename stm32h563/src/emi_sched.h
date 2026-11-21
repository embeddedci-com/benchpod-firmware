#ifndef EMI_SCHED_H
#define EMI_SCHED_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/*
 * emi_sched — the EMI test build's operating modes and capture schedule (emi_test.h). No HAL, so
 * the host tests (test/test_emi_sched.c) check the timing of every mode.
 *
 * A mode is what the pod runs while the EMI loop is on: the captures (which, how often), the DAC
 * outputs, the Ethernet stream's rate, the read-back's pace and the NMEA UART. The modes come from
 * how customers use the pod (benchpod-compliance emc/emi-build-use-cases-plan.md):
 *
 *   ci           the lab's mode (default): a CI rack running test suites. DAC outputs, DUT power,
 *                10 Mbit/s on Ethernet and NMEA on LA8 all the time; a small capture every 500 ms
 *                (4,096 samples, the SDK default), LA only and ADC + LA in turn, read back at a LAN
 *                client's pace. No deep capture, no deep replay
 *   interactive  a person at the web app: as ci, a small capture every 10 s, no NMEA
 *   sensor       sensor-emulation tests: NMEA on LA8, a DC level, a small capture every 10 s
 *   stress       the loop before the modes: deep captures back to back (200k ADC + LA, then 1 M LA),
 *                read back as fast as the worker allows, 20 Mbit/s. Home diagnosis only
 */

typedef enum {
    EMI_MODE_CI = 0,
    EMI_MODE_INTERACTIVE,
    EMI_MODE_SENSOR,
    EMI_MODE_STRESS,
    EMI_MODE_COUNT
} emi_mode_t;

/* The boot mode: `make EMI_TEST=1 EMI_MODE=<name>` (Makefile) sets it. */
#ifndef EMI_MODE_DEFAULT
#define EMI_MODE_DEFAULT EMI_MODE_CI
#endif

/* A capture the loop takes: the label for the error log, the JSON command, and whether it needs
   the ADC (a digital-only board skips those). */
typedef struct {
    const char *label;
    const char *json;
    bool        needs_adc;
} emi_cap_t;

typedef struct {
    const char      *name;
    const emi_cap_t *caps;          /* taken in turn */
    uint8_t          n_caps;
    uint32_t         period_ms;     /* start to start; 0 = back to back (after gap_ms) */
    uint32_t         gap_ms;        /* at least this long between a capture's end and the next start */
    uint32_t         net_mbit;      /* the Ethernet stream */
    uint32_t         readback_bps;  /* the read-back's pace; 0 = EMI_SCHED_BUDGET_MAX every pass */
    int8_t           dac;           /* an emi_dac_mode_t, or -1 for the board's default (all / sine) */
    uint8_t          uart_la;       /* NMEA out of this LA pin, 0 = off */
} emi_mode_desc_t;

/* The read-back's budget per worker pass, at most (also the unpaced budget). */
#define EMI_SCHED_BUDGET_MAX   16384u
/* A capture this late (not issued this long after it was due) counts as a failure for the LED:
   a stalled loop never looks green. Long enough for a few run-log reads (each holds the bus
   1.5 s) or a client's heavy command in between. */
#define EMI_SCHED_OVERDUE_MS   10000u
/* The busy share (emi_busy_t) is counted over windows this long. */
#define EMI_SCHED_BUSY_WINDOW_MS  60000u
/* The NMEA pin of the ci and sensor modes: the UART sensor pin of the process-node bench. */
#define EMI_SCHED_UART_LA      8u

const emi_mode_desc_t *emi_mode_desc(emi_mode_t m);   /* NULL for an unknown mode */
const char *emi_mode_name(emi_mode_t m);               /* "?" for an unknown mode */
int emi_mode_from_name(const char *name);              /* -1 for anything else */

/* When the next capture may start, given when the last one started and ended. */
uint32_t emi_sched_next_start(const emi_mode_desc_t *d, uint32_t started_at, uint32_t ended_at);

/* The capture after `cur` (an index into d->caps), skipping those that need an ADC on a board
   without one. d->n_caps when there is none to take. Pass d->n_caps as `cur` for the first. */
unsigned emi_sched_next_cap(const emi_mode_desc_t *d, unsigned cur, bool has_adc);

/* A capture due at `due` and still not issued at `now` is overdue (EMI_SCHED_OVERDUE_MS). */
bool emi_sched_overdue(uint32_t due, uint32_t now);

/* The read-back budget after dt_ms more: unpaced (bps 0) = EMI_SCHED_BUDGET_MAX; else it grows by
   bps per second up to EMI_SCHED_BUDGET_MAX (dt is capped at 100 ms, so a stall is no flood). */
uint32_t emi_sched_budget(uint32_t budget, uint32_t dt_ms, uint32_t bps);

/* The share of time a capture or its read-back was in flight, in tenths of a percent, over the
   last full window (EMI_SCHED_BUSY_WINDOW_MS). */
typedef struct {
    uint32_t win_start, t_last, busy_ms;
    uint16_t last_permille;
    bool     have;          /* a full window was counted */
    bool     init;
} emi_busy_t;
void emi_busy_reset(emi_busy_t *b, uint32_t now);
void emi_busy_tick(emi_busy_t *b, uint32_t now, bool busy);

#endif
