/*
 * test_power_profile.c — host unit tests for the INA238 rail profile sampler (src/power_profile.c).
 *
 * Three layers:
 *   1. pp_rate_select  — the requested rate -> the INA238 ADC_CONFIG whose conversion period is
 *                        closest to it (and never faster than the worker can read).
 *   2. pp_stats / pp_decim — the statistics (including the energy/charge integrals) and the
 *                        bin-averaging decimator that squeezes a long profile into keep_samples.
 *   3. the sampler itself, driven against an INA238 REGISTER MOCK behind the real ina238.c and a
 *      controllable clock: what it writes at start, that it reads one shunt per period and a bus
 *      voltage every ~10 ms, that max_duration_ms truncates, and that ADC_CONFIG is restored.
 */
#include "power_profile.h"
#include "ina238.h"
#include "i2c_bus.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures;
#define CHECK(c) do { if(!(c)){printf("FAIL %s:%d  %s\n",__FILE__,__LINE__,#c);failures++;} } while(0)

/* ---- controllable clock + FreeRTOS heap stand-ins ------------------------ */

static uint64_t g_now_us;
uint64_t time_us_64(void) { return g_now_us; }

void *pvPortMalloc(size_t size) { return malloc(size); }
void  vPortFree(void *p)        { free(p); }

void hw_lock(void)   {}
void hw_unlock(void) {}

/* ---- eFuse fault stub (power_profile reads the cached FLT, never I2C) ---- */

static bool g_fault;
int target_power_get_cached(int efuse, bool *enabled, bool *fault) {
    (void)efuse;
    if (enabled) *enabled = true;
    if (fault)   *fault   = g_fault;
    return 0;
}

/* ---- INA238 register mock (only 0x40 / 0x44 exist) ----------------------- */

#define REG_CONFIG      0x00u
#define REG_ADC_CONFIG  0x01u
#define REG_VSHUNT      0x04u
#define REG_VBUS        0x05u

static struct {
    uint8_t  addr;                 /* last address talked to */
    uint16_t reg[0x40];
    int      writes;               /* register writes since the last mock_reset */
    int      shunt_reads, bus_reads;
    bool     dead;                 /* every transaction fails (an absent sensor) */
} ina;

static void mock_reset(void) {
    memset(&ina, 0, sizeof(ina));
    ina.reg[REG_ADC_CONFIG] = INA238_ADC_CONFIG_RESET;
}

int i2c_bus_write(uint8_t addr, const uint8_t *buf, size_t len) {
    if (ina.dead) return -1;
    ina.addr = addr;
    if (len == 3) {
        ina.reg[buf[0] & 0x3Fu] = (uint16_t)((buf[1] << 8) | buf[2]);
        ina.writes++;
    }
    return 0;
}

int i2c_bus_write_read(uint8_t addr, const uint8_t *wbuf, size_t wlen,
                       uint8_t *rbuf, size_t rlen) {
    if (ina.dead) return -1;
    ina.addr = addr;
    if (wlen == 1 && rlen == 2) {
        uint8_t r = wbuf[0] & 0x3Fu;
        if (r == REG_VSHUNT) ina.shunt_reads++;
        if (r == REG_VBUS)   ina.bus_reads++;
        rbuf[0] = (uint8_t)(ina.reg[r] >> 8);
        rbuf[1] = (uint8_t)(ina.reg[r] & 0xFF);
    }
    return 0;
}

/* ---- 1. rate selection --------------------------------------------------- */

static void test_rate_select(void) {
    pp_adc_cfg_t c;

    /* Every selection: continuous shunt+bus (MODE 0xB), the shortest VBUSCT (0), a period the
       worker can actually keep up with, and a reported rate that matches that period. */
    for (uint32_t hz = 50; hz <= 3000; hz += 50) {
        pp_rate_select(hz, &c);
        CHECK((c.adc_config >> 12) == 0xBu);
        CHECK(((c.adc_config >> 9) & 7u) == 0u);          /* VBUSCT = 50 us */
        CHECK(c.period_us >= PP_MIN_PERIOD_US);
        CHECK(c.rate_hz == (1000000u + c.period_us / 2u) / c.period_us);
    }

    /* The maximum: 50 us bus + 2074 us shunt, no averaging = 2124 us -> 471 Hz conversions.
       ADC_CONFIG = MODE 0xB | VBUSCT 0 | VSHCT 6 | VTCT 0 | AVG 0. */
    pp_rate_select(PP_RATE_MAX_HZ, &c);
    CHECK(c.period_us == 2124u);
    CHECK(c.rate_hz   == 471u);
    CHECK(c.adc_config == 0xB180u);

    /* Anything above the maximum clamps to it rather than selecting a faster conversion. */
    pp_adc_cfg_t over;
    pp_rate_select(PP_RATE_MAX_HZ + 1u, &over);
    CHECK(over.period_us == c.period_us && over.adc_config == c.adc_config);

    /* Above the achievable rate the period floor wins, and the ACHIEVED rate is reported
       rather than the requested one — that is the number the reply carries. */
    pp_adc_cfg_t hi;
    pp_rate_select(100000, &hi);                     /* clamped to PP_RATE_MAX_HZ */
    CHECK(hi.period_us == c.period_us);

    /* The slow end uses averaging instead of a longer single conversion. */
    pp_rate_select(PP_RATE_MIN_HZ, &c);
    CHECK(c.period_us > 5000u);
    CHECK((c.adc_config & 7u) != 0u);                /* AVG > 1 */
    pp_adc_cfg_t lo;
    pp_rate_select(1, &lo);                          /* clamped to PP_RATE_MIN_HZ */
    CHECK(lo.period_us == c.period_us);
}

/* ---- 2a. statistics ------------------------------------------------------ */

static void test_stats(void) {
    pp_stats_t s;
    pp_stats_init(&s);
    CHECK(s.n == 0);

    /* 3 samples, 1 ms apart, 1 mA at 5 V.  dt of the first runs from t=0, so all three
       integrate a full millisecond: 1000 uA x 5000 mV x 1000 us = 5e6 pJ each. */
    pp_stats_add(&s, 1000, 1000, 5000);
    pp_stats_add(&s, 2000, 1000, 5000);
    pp_stats_add(&s, 3000, 1000, 5000);
    CHECK(s.n == 3);
    CHECK(s.sum_ua == 3000);
    CHECK(s.min_ua == 1000 && s.peak_ua == 1000);
    CHECK(s.min_mv == 5000 && s.max_mv == 5000);
    CHECK(s.energy_pj == 15000000);                  /* 15 uJ */
    CHECK(s.charge_pc == 3000000);                   /* 3 uC  */

    /* min/peak track both directions, including a negative current (current flowing back). */
    pp_stats_add(&s, 4000, -250, 4900);
    pp_stats_add(&s, 5000, 9000, 5100);
    CHECK(s.min_ua == -250 && s.peak_ua == 9000);
    CHECK(s.min_mv == 4900 && s.max_mv == 5100);

    /* A stalled worker must not blow the integrals up: dt is capped at 10 s, and a
       non-monotonic timestamp contributes no time at all. */
    pp_stats_t g;
    pp_stats_init(&g);
    pp_stats_add(&g, 0, 1000, 5000);
    pp_stats_add(&g, 1000000000u, 1000, 5000);       /* 1000 s later */
    CHECK(g.charge_pc == (int64_t)1000 * 10000000);  /* capped at 10 s */
    int64_t before = g.charge_pc;
    pp_stats_add(&g, 1, 1000, 5000);                 /* goes backwards: dt 0 */
    CHECK(g.charge_pc == before);
    CHECK(g.n == 3);
}

/* ---- 2b. decimator ------------------------------------------------------- */

static uint32_t d_t[64];
static int32_t  d_ua[64];
static uint16_t d_mv[64];

static void test_decim(void) {
    pp_decim_t d;

    /* keep_samples 0: nothing is stored (stats still run over every raw sample). */
    pp_decim_init(&d, d_t, d_ua, d_mv, 0);
    for (uint32_t i = 0; i < 10; i++) pp_decim_add(&d, i, (int32_t)i, 5000);
    pp_decim_finish(&d);
    CHECK(d.n == 0);

    /* 16 samples into a 4-entry store: the store halves twice, so every entry averages 4 raw
       samples and carries the timestamp of the first sample in its bin. */
    pp_decim_init(&d, d_t, d_ua, d_mv, 4);
    for (uint32_t i = 0; i < 16; i++) pp_decim_add(&d, i * 100u, (int32_t)i * 100, 5000);
    pp_decim_finish(&d);
    CHECK(d.n == 4);
    CHECK(d.bin == 4);
    CHECK(d_t[0] == 0 && d_t[1] == 400 && d_t[2] == 800 && d_t[3] == 1200);
    CHECK(d_ua[0] == 150 && d_ua[1] == 550 && d_ua[2] == 950 && d_ua[3] == 1350);
    CHECK(d_mv[0] == 5000);

    /* Fewer samples than the store: they are kept one-for-one. */
    pp_decim_init(&d, d_t, d_ua, d_mv, 8);
    for (uint32_t i = 0; i < 5; i++) pp_decim_add(&d, i, (int32_t)i, (uint16_t)(5000 + i));
    pp_decim_finish(&d);
    CHECK(d.n == 5 && d.bin == 1);
    CHECK(d_ua[4] == 4 && d_mv[4] == 5004);

    /* keep_samples 1: a single average over the whole profile. */
    pp_decim_init(&d, d_t, d_ua, d_mv, 1);
    for (uint32_t i = 0; i < 100; i++) pp_decim_add(&d, i, 200, 5000);
    pp_decim_finish(&d);
    CHECK(d.n == 1 && d_ua[0] == 200 && d_t[0] == 0);

    /* An odd keep_samples rounds DOWN to even so the pairwise merge always has a partner. */
    pp_decim_init(&d, d_t, d_ua, d_mv, 5);
    CHECK(d.cap == 4);

    /* Negative currents round to nearest, away from zero. */
    pp_decim_init(&d, d_t, d_ua, d_mv, 2);
    pp_decim_add(&d, 0, -1, 5000);
    pp_decim_add(&d, 1, -2, 5000);
    pp_decim_finish(&d);
    CHECK(d.n == 2 && d_ua[0] == -1 && d_ua[1] == -2);
}

/* ---- 3. the sampler over the mocked INA238 ------------------------------- */

/* Run the sampler forward by `us`, polling as the hw worker does (~1 ms passes). */
static void advance_by(uint64_t us, uint64_t step_us) {
    for (uint64_t i = 0; i < us; i += step_us) {
        g_now_us += step_us;
        power_profile_poll();
    }
}

static void advance(uint64_t us) { advance_by(us, 250); }

static void test_sampler_start_errors(void) {
    char err[160];
    mock_reset();
    power_profile_discard();

    CHECK(power_profile_start(3, 1000, 1000, 0, err, sizeof(err)) != 0);
    CHECK(strcmp(err, "efuse must be 1 or 2") == 0);
    CHECK(power_profile_start(1, 10, 1000, 0, err, sizeof(err)) != 0);
    CHECK(power_profile_start(1, PP_RATE_MAX_HZ, 0, 0, err, sizeof(err)) != 0);
    CHECK(power_profile_start(1, PP_RATE_MAX_HZ, 1000, PP_KEEP_MAX + 1u, err, sizeof(err)) != 0);

    /* An absent sensor is reported as such, not as a started-but-empty profile. */
    ina.dead = true;
    CHECK(power_profile_start(1, PP_RATE_MAX_HZ, 1000, 0, err, sizeof(err)) != 0);
    CHECK(strstr(err, "not responding") != NULL);
    CHECK(!power_profile_running());
    ina.dead = false;
}

static void test_sampler_run(void) {
    char err[160];
    mock_reset();
    power_profile_discard();
    g_fault = false;
    g_now_us = 1000000;

    ina.reg[REG_VSHUNT] = 1000;    /* 1000 x 5 uV = 5 mV over 50 mOhm = 100 mA */
    ina.reg[REG_VBUS]   = 1600;    /* 1600 x 3.125 mV = 5000 mV */

    CHECK(power_profile_start(1, PP_RATE_MAX_HZ, 60000, 64, err, sizeof(err)) == 0);
    CHECK(ina.addr == I2C_ADDR_INA238_INTERNAL);
    CHECK(ina.reg[REG_CONFIG] == 0x0000u);             /* ADCRANGE 0: the fine shunt range */
    CHECK(ina.reg[REG_ADC_CONFIG] == 0xB180u);         /* 500 Hz request -> 2124 us period */
    CHECK(power_profile_running());
    CHECK(!power_profile_has_result());

    pp_status_t st;
    power_profile_status(&st);
    CHECK(st.running && st.efuse == 1 && st.rate_hz == 471u && st.n == 0);

    /* 100 ms of sampling: ~47 shunt reads at 2124 us, ~10 bus reads at 10 ms. */
    advance(100000);
    power_profile_status(&st);
    CHECK(st.n >= 44 && st.n <= 49);
    CHECK(ina.bus_reads >= 9 && ina.bus_reads <= 12);
    CHECK(st.elapsed_ms >= 99 && st.elapsed_ms <= 101);

    power_profile_stop();
    CHECK(!power_profile_running());
    CHECK(power_profile_has_result());
    /* ADC_CONFIG goes back to its reset value: the console `ina` / power_status readers that
       share this sensor must not be left on the profile's conversion times. */
    CHECK(ina.reg[REG_ADC_CONFIG] == INA238_ADC_CONFIG_RESET);

    /* Every raw sample read the same registers, so the stats are exact. */
    char buf[512];
    bp_emit_t e;
    bp_emit_init(&e, buf, sizeof(buf));
    power_profile_emit_stats(&e);
    CHECK(bp_emit_ok(&e));
    CHECK(strstr(buf, "\"efuse\":1") != NULL);
    /* adc_rate_hz is the configured conversion rate; rate_hz is what was actually delivered
       (n over duration), which on real hardware is well below it. */
    CHECK(strstr(buf, "\"adc_rate_hz\":471") != NULL);
    {
        const char *r = strstr(buf, "\"rate_hz\":");       /* not "adc_rate_hz": — the quote differs */
        const char *n_p = strstr(buf, "\"n\":");
        const char *d_p = strstr(buf, "\"duration_ms\":");
        CHECK(r != NULL && n_p != NULL && d_p != NULL);
        unsigned long got  = strtoul(r + 10, NULL, 10);
        unsigned long n    = strtoul(n_p + 4, NULL, 10);
        unsigned long d_ms = strtoul(d_p + 14, NULL, 10);
        CHECK(d_ms > 0);
        unsigned long want = n * 1000ul / d_ms;
        CHECK(got + 20 >= want && want + 20 >= got);       /* rate_hz == n over duration */
    }
    CHECK(strstr(buf, "\"avg_ua\":100000") != NULL);
    CHECK(strstr(buf, "\"peak_ua\":100000") != NULL);
    CHECK(strstr(buf, "\"min_mv\":5000") != NULL);
    CHECK(strstr(buf, "\"fault\":false") != NULL);
    CHECK(strstr(buf, "\"truncated\":false") != NULL);
    /* 100 mA x 5 V over the sampled span.  The integrals cover n whole periods (the tail of the
       window after the last sample is not counted), so tie the expectation to that rather than to
       the wall-clock 100 ms: 46 x 2124 us x 100 mA x 5 V = ~48.9 mJ, ~9.8 mC. */
    {
        const char *e_p = strstr(buf, "\"energy_uj\":");
        const char *c_p = strstr(buf, "\"charge_uc\":");
        CHECK(e_p != NULL && c_p != NULL);
        unsigned long energy_uj = strtoul(e_p + 12, NULL, 10);
        unsigned long charge_uc = strtoul(c_p + 12, NULL, 10);
        unsigned long span_us   = (unsigned long)st.n * 2124ul;
        CHECK(charge_uc + 200 >= span_us / 10ul && span_us / 10ul + 200 >= charge_uc);
        CHECK(energy_uj == charge_uc * 5ul);           /* 5 V rail: E = Q x V exactly */
    }

    /* keep_samples 64 with ~47 raw samples: bin-averaged down, never more than asked for. */
    uint32_t kept = power_profile_kept();
    CHECK(kept > 0 && kept <= 64);
    uint32_t t0 = 0, t1 = 0; int32_t ua = 0; uint16_t mv = 0;
    power_profile_sample(0, &t0, &ua, &mv);
    power_profile_sample(kept - 1u, &t1, &ua, &mv);
    CHECK(t1 > t0);                                    /* timestamps ascend */
    CHECK(ua == 100000 && mv == 5000);

    power_profile_discard();
    CHECK(!power_profile_has_result());
    CHECK(power_profile_kept() == 0);
}

/* The hw worker calls power_profile_poll() no faster than the shunt period, so a shunt sample is
   due on EVERY pass.  The bus read must not sit behind the shunt branch's return: it used to, and
   then every sample carried the voltage read once at start — a profile opened before power_on
   reported 0.031 V for its whole window on real hardware while the currents were correct. */
static void test_sampler_tracks_bus_at_worker_cadence(void) {
    char err[160];
    mock_reset();
    power_profile_discard();
    g_fault = false;
    g_now_us = 1000000;

    ina.reg[REG_VSHUNT] = 1000;    /* 100 mA */
    ina.reg[REG_VBUS]   = 10;      /* rail still off: 10 x 3.125 mV = 31 mV */

    CHECK(power_profile_start(1, PP_RATE_MAX_HZ, 60000, 64, err, sizeof(err)) == 0);

    /* Poll slower than the 2124 us shunt period — the real worker cadence. */
    advance_by(50000, 2000);
    CHECK(ina.bus_reads >= 4);                 /* the bus is still sampled at this cadence */

    ina.reg[REG_VBUS] = 1600;                  /* the rail comes up: 5000 mV */
    advance_by(50000, 2000);
    power_profile_stop();

    char buf[512];
    bp_emit_t e;
    bp_emit_init(&e, buf, sizeof(buf));
    power_profile_emit_stats(&e);
    CHECK(bp_emit_ok(&e));
    CHECK(strstr(buf, "\"min_mv\":31") != NULL);     /* saw the rail off ... */
    CHECK(strstr(buf, "\"max_mv\":5000") != NULL);   /* ... and the rail on */

    power_profile_discard();
}

static void test_sampler_busy_truncate_and_fault(void) {
    char err[160];
    mock_reset();
    power_profile_discard();
    g_fault = false;
    g_now_us = 5000000;
    ina.reg[REG_VSHUNT] = 0;
    ina.reg[REG_VBUS]   = 1600;

    CHECK(power_profile_start(2, PP_RATE_MAX_HZ, 50, 0, err, sizeof(err)) == 0);
    CHECK(ina.addr == I2C_ADDR_INA238_EXTERNAL);

    /* A second start while one runs is refused, naming the rail that is busy. */
    CHECK(power_profile_start(1, PP_RATE_MAX_HZ, 1000, 0, err, sizeof(err)) != 0);
    CHECK(strcmp(err, "power profile already running on efuse 2") == 0);

    /* An eFuse trip during the window is latched into the result. */
    g_fault = true;
    advance(20000);
    g_fault = false;

    /* max_duration_ms stops the sampler by itself and marks the result truncated. */
    advance(60000);
    CHECK(!power_profile_running());
    CHECK(power_profile_has_result());

    char buf[512];
    bp_emit_t e;
    bp_emit_init(&e, buf, sizeof(buf));
    power_profile_emit_stats(&e);
    CHECK(strstr(buf, "\"efuse\":2") != NULL);
    CHECK(strstr(buf, "\"fault\":true") != NULL);
    CHECK(strstr(buf, "\"truncated\":true") != NULL);
    /* keep_samples 0 keeps the statistics but no sample arrays. */
    CHECK(power_profile_kept() == 0);

    power_profile_discard();
}

static void test_sampler_survives_i2c_errors(void) {
    char err[160];
    mock_reset();
    power_profile_discard();
    g_now_us = 9000000;
    ina.reg[REG_VBUS] = 1600;
    CHECK(power_profile_start(1, PP_RATE_MAX_HZ, 60000, 0, err, sizeof(err)) == 0);

    advance(20000);
    pp_status_t st;
    power_profile_status(&st);
    uint32_t n_before = st.n;
    CHECK(n_before > 0);

    /* The sensor drops off the bus: the sampler gives up after PP_MAX_I2C_ERRORS rather
       than spinning on a dead sensor for the rest of max_duration_ms. */
    ina.dead = true;
    advance(200000);
    CHECK(!power_profile_running());
    power_profile_status(&st);
    CHECK(st.n == n_before);
    ina.dead = false;
    power_profile_discard();
}

int main(void) {
    test_rate_select();
    test_stats();
    test_decim();
    test_sampler_start_errors();
    test_sampler_run();
    test_sampler_tracks_bus_at_worker_cadence();
    test_sampler_busy_truncate_and_fault();
    test_sampler_survives_i2c_errors();
    if (failures) { printf("test_power_profile: %d FAILURE(S)\n", failures); return 1; }
    printf("test_power_profile: all passed\n");
    return 0;
}
