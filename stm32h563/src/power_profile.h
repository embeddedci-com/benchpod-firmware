#ifndef POWER_PROFILE_H
#define POWER_PROFILE_H

/*
 * power_profile — current/voltage profile of one eFuse rail from its INA238.
 *
 * The INA238 runs in continuous shunt+bus mode with conversion time x averaging chosen so one
 * conversion period ~= the requested sample period.  The hw worker reads VSHUNT once per period
 * and VBUS every ~10 ms (the last bus value is held for the samples in between), one I2C
 * transaction per worker pass under hw_lock, so the worker is never blocked for longer than a
 * single register read.  Statistics (min/avg/peak, energy, charge) run over every sample read;
 * the samples kept for the reply are bin-averaged down to at most keep_samples.  ADC_CONFIG is
 * put back to its reset value when the profile ends.
 *
 * The rate/config selection, statistics and decimation are pure and host-tested
 * (test/test_power_profile.c, which also drives the sampler against an INA238 register mock).
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bp_json.h"

/* Measured on a v2 pod (2026-09-11), requested vs delivered:
     100 -> 105    200 -> 189    300 -> 288    400 -> 305    450 -> 361    500..2000 -> 364
   The sampler reads one INA238 register per hw-worker pass (~2.2 ms once the pass does its other
   work), so delivery tracks the request to ~200 Hz, beats against the pass interval through the
   300-500 band, and flattens at ~365 Hz.  Asking for more than 500 changed nothing at all, so the
   accepted maximum stops there rather than advertising a rate no pod can produce; the stats report
   the rate actually delivered (rate_hz) next to the conversion rate configured (adc_rate_hz). */
#define PP_RATE_MIN_HZ          100u
#define PP_RATE_MAX_HZ          500u
#define PP_RATE_DEFAULT_HZ      500u
/* One VSHUNT read per worker pass, and the worker passes about once per 1 ms tick: a
   conversion period shorter than this would only produce readings nobody reads. */
#define PP_MIN_PERIOD_US        1000u
#define PP_BUS_PERIOD_US        10000u
#define PP_DURATION_MAX_MS      600000u
#define PP_DURATION_DEFAULT_MS  60000u
#define PP_KEEP_MAX             4096u
/* Bytes of heap per kept sample: t_us (4) + current_ua (4) + bus_mv (2). */
#define PP_BYTES_PER_SAMPLE     10u

/* ---- INA238 conversion config for a requested rate ---- */
typedef struct {
    uint16_t adc_config;   /* ADC_CONFIG value: MODE 0xB (continuous shunt+bus), VBUSCT 50 us */
    uint32_t period_us;    /* (VBUSCT + VSHCT) x AVG */
    uint32_t rate_hz;      /* achieved: round(1e6 / period_us) */
} pp_adc_cfg_t;

/* rate_hz is clamped to PP_RATE_MIN_HZ..PP_RATE_MAX_HZ, then the config whose period is closest
   (by ratio) to 1/rate and not shorter than PP_MIN_PERIOD_US is chosen; ties prefer the longer
   shunt conversion (less of each period spent on the bus channel). */
void pp_rate_select(uint32_t rate_hz, pp_adc_cfg_t *out);

/* ---- statistics over every raw sample ---- */
typedef struct {
    uint32_t n;
    uint32_t last_t_us;
    int64_t  sum_ua;
    int32_t  min_ua, peak_ua;
    int64_t  sum_mv;
    uint16_t min_mv, max_mv;
    int64_t  energy_pj;    /* sum of I x V x dt */
    int64_t  charge_pc;    /* sum of I x dt */
} pp_stats_t;

void pp_stats_init(pp_stats_t *s);
/* t_us = time of this sample since the start; dt is measured from the previous sample (the
   first from 0) and capped at 10 s so a stalled worker cannot overflow the integrals. */
void pp_stats_add(pp_stats_t *s, uint32_t t_us, int32_t ua, uint16_t mv);

/* ---- bin-averaging decimator ---- */
typedef struct {
    uint32_t *t_us;
    int32_t  *ua;
    uint16_t *mv;
    uint32_t  cap;      /* entries it may hold (keep_samples rounded down to even; 1 stays 1) */
    uint32_t  n;        /* entries held */
    uint32_t  bin;      /* raw samples per entry */
    uint32_t  acc_n;    /* the bin being accumulated */
    uint32_t  acc_t;
    int64_t   acc_ua;
    uint32_t  acc_mv;
} pp_decim_t;

/* When the store is full, neighbouring entries are averaged in pairs and the bin size doubles, so
   every stored entry (but the final partial one) averages the same number of raw samples. */
void pp_decim_init(pp_decim_t *d, uint32_t *t_us, int32_t *ua, uint16_t *mv, uint32_t keep);
void pp_decim_add(pp_decim_t *d, uint32_t t_us, int32_t ua, uint16_t mv);
void pp_decim_finish(pp_decim_t *d);

/* ---- the sampler (hw worker task) ---- */

/* 0 = started; otherwise err holds the client-facing message.  A previous result is discarded. */
int  power_profile_start(int efuse, uint32_t rate_hz, uint32_t max_duration_ms,
                         uint32_t keep_samples, char *err, size_t cap);
void power_profile_poll(void);       /* every worker pass */
void power_profile_stop(void);       /* stop if running; the result is kept */
bool power_profile_running(void);
bool power_profile_has_result(void); /* a finished profile that has not been discarded */
void power_profile_discard(void);    /* the result was delivered: free it */

typedef struct {
    bool     any;          /* a profile was started since boot / the last discard */
    bool     running;
    int      efuse;
    uint32_t rate_hz;
    uint32_t elapsed_ms;
    uint32_t n;
} pp_status_t;
void power_profile_status(pp_status_t *out);

uint32_t power_profile_kept(void);
void     power_profile_sample(uint32_t i, uint32_t *t_us, int32_t *ua, uint16_t *mv);
/* "stats":{"efuse":..,"rate_hz":..,...,"fault":..,"truncated":..}  (no leading comma) */
void     power_profile_emit_stats(bp_emit_t *e);

#endif /* POWER_PROFILE_H */
