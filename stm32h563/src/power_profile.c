/*
 * power_profile.c — INA238 rail profile sampler.  See power_profile.h.
 */
#include "power_profile.h"
#include "ina238.h"
#include "i2c_bus.h"
#include "hw_lock.h"
#include "target_power.h"
#include "pico/time.h"

#include <stdio.h>
#include <string.h>

/* FreeRTOS heap_4.  Declared here instead of via FreeRTOS.h so the sampler also builds in the
   host test, which links malloc-backed stand-ins.  The kept samples live on the heap only while
   a result exists: 4096 x 10 B is far more than the remaining static RAM. */
void *pvPortMalloc(size_t size);
void  vPortFree(void *p);

/* ---- rate selection ---- */

static const uint16_t k_ct_us[8] = { 50, 84, 150, 280, 540, 1052, 2074, 4120 };
static const uint16_t k_avg[8]   = { 1, 4, 16, 64, 128, 256, 512, 1024 };

void pp_rate_select(uint32_t rate_hz, pp_adc_cfg_t *out) {
    if (rate_hz < PP_RATE_MIN_HZ) rate_hz = PP_RATE_MIN_HZ;
    if (rate_hz > PP_RATE_MAX_HZ) rate_hz = PP_RATE_MAX_HZ;
    uint32_t target = 1000000u / rate_hz;

    uint64_t best_score = UINT64_MAX;
    unsigned best_ct = 0, best_avg = 0;
    uint32_t best_period = 0;
    for (unsigned ct = 0; ct < 8u; ct++) {
        for (unsigned a = 0; a < 8u; a++) {
            /* VBUSCT is pinned to the shortest conversion (50 us): the bus only needs a reading
               every ~10 ms, so every other microsecond of the period goes to the shunt. */
            uint32_t period = ((uint32_t)k_ct_us[0] + k_ct_us[ct]) * k_avg[a];
            if (period < PP_MIN_PERIOD_US) continue;
            uint64_t score = (period >= target) ? (uint64_t)period * 1000u / target
                                                : (uint64_t)target * 1000u / period;
            if (score < best_score || (score == best_score && ct > best_ct)) {
                best_score  = score;
                best_ct     = ct;
                best_avg    = a;
                best_period = period;
            }
        }
    }
    out->adc_config = (uint16_t)((0xBu << 12) | (0u << 9) | (best_ct << 6) | (0u << 3) | best_avg);
    out->period_us  = best_period;
    out->rate_hz    = (1000000u + best_period / 2u) / best_period;
}

/* ---- statistics ---- */

void pp_stats_init(pp_stats_t *s) {
    memset(s, 0, sizeof(*s));
}

void pp_stats_add(pp_stats_t *s, uint32_t t_us, int32_t ua, uint16_t mv) {
    uint32_t dt = (t_us >= s->last_t_us) ? t_us - s->last_t_us : 0u;
    if (dt > 10000000u) dt = 10000000u;
    if (s->n == 0) {
        s->min_ua = s->peak_ua = ua;
        s->min_mv = s->max_mv  = mv;
    } else {
        if (ua < s->min_ua)  s->min_ua  = ua;
        if (ua > s->peak_ua) s->peak_ua = ua;
        if (mv < s->min_mv)  s->min_mv  = mv;
        if (mv > s->max_mv)  s->max_mv  = mv;
    }
    s->n++;
    s->sum_ua += ua;
    s->sum_mv += mv;
    /* uA x mV x us = fJ (/1000 -> pJ); uA x us = pC.  |uA x mV| < 2^39 and dt <= 10 s keep the
       product inside int64. */
    s->energy_pj += ((int64_t)ua * (int64_t)mv) * (int64_t)dt / 1000;
    s->charge_pc += (int64_t)ua * (int64_t)dt;
    s->last_t_us  = t_us;
}

/* ---- decimator ---- */

void pp_decim_init(pp_decim_t *d, uint32_t *t_us, int32_t *ua, uint16_t *mv, uint32_t keep) {
    memset(d, 0, sizeof(*d));
    d->t_us = t_us;
    d->ua   = ua;
    d->mv   = mv;
    d->cap  = (keep >= 2u) ? (keep & ~1u) : keep;
    d->bin  = (keep == 1u) ? UINT32_MAX : 1u;   /* keep 1: one average over the whole profile */
}

static int32_t div_round(int64_t sum, uint32_t n) {
    int64_t h = (int64_t)(n / 2u);
    return (int32_t)((sum >= 0) ? (sum + h) / (int64_t)n : (sum - h) / (int64_t)n);
}

static void decim_push(pp_decim_t *d) {
    d->t_us[d->n] = d->acc_t;
    d->ua[d->n]   = div_round(d->acc_ua, d->acc_n);
    d->mv[d->n]   = (uint16_t)((d->acc_mv + d->acc_n / 2u) / d->acc_n);
    d->n++;
    d->acc_n  = 0;
    d->acc_ua = 0;
    d->acc_mv = 0;
}

static void decim_merge(pp_decim_t *d) {
    uint32_t j = 0;
    for (uint32_t i = 0; i + 1u < d->n; i += 2u, j++) {
        d->t_us[j] = d->t_us[i];
        d->ua[j]   = div_round((int64_t)d->ua[i] + d->ua[i + 1u], 2u);
        d->mv[j]   = (uint16_t)((d->mv[i] + d->mv[i + 1u] + 1u) / 2u);
    }
    d->n = j;
    d->bin *= 2u;
}

void pp_decim_add(pp_decim_t *d, uint32_t t_us, int32_t ua, uint16_t mv) {
    if (d->cap == 0) return;
    if (d->acc_n == 0) d->acc_t = t_us;
    d->acc_ua += ua;
    d->acc_mv += mv;
    d->acc_n++;
    if (d->acc_n < d->bin) return;
    if (d->n == d->cap) {
        /* Full: halve the store and keep accumulating this bin to the doubled size, so it ends
           up weighted like its neighbours. */
        decim_merge(d);
        return;
    }
    decim_push(d);
}

void pp_decim_finish(pp_decim_t *d) {
    if (d->cap == 0 || d->acc_n == 0) return;
    if (d->n == d->cap) decim_merge(d);
    decim_push(d);
}

/* ---- sampler ---- */

#define PP_MAX_I2C_ERRORS 20u   /* consecutive failed reads before the profile gives up */

static struct {
    bool         any;
    bool         running;
    bool         fault;
    bool         truncated;
    int          efuse;
    uint8_t      addr;
    pp_adc_cfg_t cfg;
    uint64_t     t_start;
    uint64_t     t_stop;
    uint64_t     limit_us;
    uint64_t     next_shunt;
    uint64_t     next_bus;
    uint16_t     last_mv;
    uint32_t     i2c_errors;
    pp_stats_t   st;
    pp_decim_t   dec;
    uint8_t     *store;
} s;

static void free_store(void) {
    if (s.store) vPortFree(s.store);
    s.store = NULL;
    pp_decim_init(&s.dec, NULL, NULL, NULL, 0);
}

int power_profile_start(int efuse, uint32_t rate_hz, uint32_t max_duration_ms,
                        uint32_t keep_samples, char *err, size_t cap) {
    if (s.running) {
        snprintf(err, cap, "power profile already running on efuse %d", s.efuse);
        return -1;
    }
    if (efuse != 1 && efuse != 2) { snprintf(err, cap, "efuse must be 1 or 2"); return -1; }
    if (rate_hz < PP_RATE_MIN_HZ || rate_hz > PP_RATE_MAX_HZ) {
        snprintf(err, cap, "rate_hz must be %u..%u", (unsigned)PP_RATE_MIN_HZ, (unsigned)PP_RATE_MAX_HZ);
        return -1;
    }
    if (max_duration_ms < 1u || max_duration_ms > PP_DURATION_MAX_MS) {
        snprintf(err, cap, "max_duration_ms must be 1..%u", (unsigned)PP_DURATION_MAX_MS);
        return -1;
    }
    if (keep_samples > PP_KEEP_MAX) {
        snprintf(err, cap, "keep_samples must be 0..%u", (unsigned)PP_KEEP_MAX);
        return -1;
    }

    free_store();
    s.any = false;
    if (keep_samples) {
        size_t bytes = (size_t)keep_samples * PP_BYTES_PER_SAMPLE;
        s.store = (uint8_t *)pvPortMalloc(bytes);
        if (!s.store) {
            snprintf(err, cap, "not enough memory to keep %u samples (%u bytes); ask for fewer keep_samples",
                     (unsigned)keep_samples, (unsigned)bytes);
            return -1;
        }
        /* One block, three aligned arrays: offsets 0 and 4k are multiples of 4, 8k of 2. */
        pp_decim_init(&s.dec, (uint32_t *)(void *)s.store,
                      (int32_t *)(void *)(s.store + 4u * keep_samples),
                      (uint16_t *)(void *)(s.store + 8u * keep_samples), keep_samples);
    }

    uint8_t addr = (efuse == 2) ? I2C_ADDR_INA238_EXTERNAL : I2C_ADDR_INA238_INTERNAL;
    pp_adc_cfg_t cfg;
    pp_rate_select(rate_hz, &cfg);
    uint16_t mv = 0;
    hw_lock();
    int rc = ina238_set_adcrange_fine(addr);
    if (rc == 0) rc = ina238_write_adc_config(addr, cfg.adc_config);
    hw_unlock();
    hw_lock();
    if (rc == 0) rc = ina238_read_bus_mv(addr, &mv);
    hw_unlock();
    if (rc != 0) {
        free_store();
        snprintf(err, cap, "power profile: the INA238 for efuse %d (I2C 0x%02X) is not responding", efuse, addr);
        return -1;
    }

    uint64_t now = time_us_64();
    s.any        = true;
    s.running    = true;
    s.fault      = false;
    s.truncated  = false;
    s.efuse      = efuse;
    s.addr       = addr;
    s.cfg        = cfg;
    s.t_start    = now;
    s.t_stop     = now;
    s.limit_us   = (uint64_t)max_duration_ms * 1000u;
    s.next_shunt = now + cfg.period_us;   /* the first conversion completes one period after the write */
    s.next_bus   = now + PP_BUS_PERIOD_US;
    s.last_mv    = mv;
    s.i2c_errors = 0;
    pp_stats_init(&s.st);
    printf("[power] profile started: efuse %d, %u Hz (ADC_CONFIG 0x%04X, %u us), max %u ms, keep %u\n",
           efuse, (unsigned)cfg.rate_hz, cfg.adc_config, (unsigned)cfg.period_us,
           (unsigned)max_duration_ms, (unsigned)keep_samples);
    return 0;
}

static void finish(bool truncated) {
    s.running   = false;
    s.truncated = truncated;
    s.t_stop    = time_us_64();
    hw_lock();
    if (ina238_write_adc_config(s.addr, INA238_ADC_CONFIG_RESET) != 0)
        printf("[power] WARNING: could not restore ADC_CONFIG on 0x%02X\n", s.addr);
    hw_unlock();
    pp_decim_finish(&s.dec);
    printf("[power] profile %s: %u samples over %u ms\n", truncated ? "reached max_duration_ms" : "stopped",
           (unsigned)s.st.n, (unsigned)((s.t_stop - s.t_start) / 1000u));
}

void power_profile_poll(void) {
    if (!s.running) return;
    uint64_t now = time_us_64();

    bool enabled = false, fault = false;
    if (target_power_get_cached(s.efuse, &enabled, &fault) == 0 && fault) s.fault = true;

    if (now - s.t_start >= s.limit_us) { finish(true); return; }

    /* Refresh the bus voltage FIRST when it is due.  The shunt branch below returns (one I2C
       transaction per worker pass) and a shunt sample is due on nearly every pass, so a bus read
       placed after it never ran: every sample carried the voltage read once at start — 0.031 V
       when the profile began before power_on.  Bus is due ~1 pass in 9, and each sample keeps its
       own measured timestamp, so the shunt cadence just slips a pass and the integrals stay right. */
    if (now >= s.next_bus) {
        uint16_t mv = 0;
        hw_lock();
        int rc = ina238_read_bus_mv(s.addr, &mv);
        hw_unlock();
        s.next_bus = now + PP_BUS_PERIOD_US;
        if (rc == 0) s.last_mv = mv;
        return;   /* one I2C transaction per worker pass */
    }

    if (now >= s.next_shunt) {
        int32_t ua = 0;
        hw_lock();
        int rc = ina238_read_shunt_ua(s.addr, &ua);
        hw_unlock();
        /* Keep the cadence while on time; after a stall, resume one period from now rather than
           read the same conversion several times in a row. */
        s.next_shunt += s.cfg.period_us;
        if (s.next_shunt <= now) s.next_shunt = now + s.cfg.period_us;
        if (rc != 0) {
            if (++s.i2c_errors >= PP_MAX_I2C_ERRORS) {
                printf("[power] %u consecutive INA238 read failures on 0x%02X; stopping the profile\n",
                       (unsigned)s.i2c_errors, s.addr);
                finish(false);
            }
            return;
        }
        s.i2c_errors = 0;
        uint32_t t = (uint32_t)(now - s.t_start);
        pp_stats_add(&s.st, t, ua, s.last_mv);
        pp_decim_add(&s.dec, t, ua, s.last_mv);
        return;   /* one I2C transaction per worker pass */
    }
}

void power_profile_stop(void) {
    if (s.running) finish(false);
}

bool power_profile_running(void)    { return s.running; }
bool power_profile_has_result(void) { return s.any && !s.running; }

void power_profile_discard(void) {
    if (s.running) return;
    free_store();
    s.any = false;
}

void power_profile_status(pp_status_t *out) {
    memset(out, 0, sizeof(*out));
    out->any     = s.any;
    out->running = s.running;
    if (!s.any) return;
    out->efuse      = s.efuse;
    out->rate_hz    = s.cfg.rate_hz;
    out->elapsed_ms = (uint32_t)(((s.running ? time_us_64() : s.t_stop) - s.t_start) / 1000u);
    out->n          = s.st.n;
}

uint32_t power_profile_kept(void) {
    return (s.any && !s.running) ? s.dec.n : 0u;
}

void power_profile_sample(uint32_t i, uint32_t *t_us, int32_t *ua, uint16_t *mv) {
    if (i >= s.dec.n) { *t_us = 0; *ua = 0; *mv = 0; return; }
    *t_us = s.dec.t_us[i];
    *ua   = s.dec.ua[i];
    *mv   = s.dec.mv[i];
}

/* newlib-nano's printf has no %lld: format 64-bit values by hand. */
static void emit_i64(bp_emit_t *e, int64_t v) {
    char buf[24];
    int i = (int)sizeof(buf);
    buf[--i] = '\0';
    uint64_t u = (v < 0) ? (uint64_t)(-(v + 1)) + 1u : (uint64_t)v;
    do { buf[--i] = (char)('0' + (int)(u % 10u)); u /= 10u; } while (u);
    if (v < 0) buf[--i] = '-';
    bp_emit_raw(e, buf + i);
}

void power_profile_emit_stats(bp_emit_t *e) {
    const pp_stats_t *st = &s.st;
    long avg_ua = st->n ? (long)div_round(st->sum_ua, st->n) : 0L;
    unsigned avg_mv = st->n ? (unsigned)((st->sum_mv + st->n / 2u) / st->n) : 0u;
    uint32_t duration_ms = (uint32_t)(((s.running ? time_us_64() : s.t_stop) - s.t_start) / 1000u);
    /* rate_hz is what the caller ACTUALLY got, measured.  The sampler reads one INA238 register
       per hw-worker pass (~2 ms once the pass does its other work), so the delivered rate sits
       well below the ADC conversion rate the profile configures — reporting the latter told an
       engineer 907 Hz when the samples arrived at ~364 Hz.  adc_rate_hz keeps the configured
       conversion rate visible; the per-sample t_us timestamps are exact either way. */
    uint32_t rate_hz = (duration_ms && st->n)
        ? (uint32_t)(((uint64_t)st->n * 1000u + duration_ms / 2u) / duration_ms) : 0u;
    bp_emit(e, "\"stats\":{\"efuse\":%d,\"rate_hz\":%lu,\"adc_rate_hz\":%lu,\"n\":%lu,\"duration_ms\":%lu,"
               "\"avg_ua\":%ld,\"min_ua\":%ld,\"peak_ua\":%ld,"
               "\"avg_mv\":%u,\"min_mv\":%u,\"max_mv\":%u,\"energy_uj\":",
            s.efuse, (unsigned long)rate_hz, (unsigned long)s.cfg.rate_hz,
            (unsigned long)st->n, (unsigned long)duration_ms,
            avg_ua, (long)st->min_ua, (long)st->peak_ua,
            avg_mv, (unsigned)st->min_mv, (unsigned)st->max_mv);
    emit_i64(e, st->energy_pj / 1000000);
    bp_emit_raw(e, ",\"charge_uc\":");
    emit_i64(e, st->charge_pc / 1000000);
    bp_emit(e, ",\"fault\":%s,\"truncated\":%s}", s.fault ? "true" : "false",
            s.truncated ? "true" : "false");
}
