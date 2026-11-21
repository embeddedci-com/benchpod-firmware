/*
 * test_emi_sched.c — host unit test for the EMI build's modes and capture schedule (emi_sched.h):
 * the lab mode takes only small captures at its period, the stress mode keeps the old loop, the
 * read-back is paced, a stalled loop is overdue, and the busy share adds up.
 */
#include "emi_sched.h"

#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(c) do { if(!(c)){printf("FAIL %s:%d  %s\n",__FILE__,__LINE__,#c);failures++;} } while(0)

/* The samples a capture command asks for (la_samples / samples / adc_samples), the largest. */
static long max_samples(const char *json) {
    static const char *const keys[] = { "\"samples\":", "\"la_samples\":", "\"adc_samples\":" };
    long most = 0;
    for (unsigned k = 0; k < 3; k++) {
        const char *p = strstr(json, keys[k]);
        if (p) { long v = 0; sscanf(p + strlen(keys[k]), "%ld", &v); if (v > most) most = v; }
    }
    return most;
}

static void test_names(void) {
    CHECK(emi_mode_from_name("ci") == EMI_MODE_CI);
    CHECK(emi_mode_from_name("interactive") == EMI_MODE_INTERACTIVE);
    CHECK(emi_mode_from_name("sensor") == EMI_MODE_SENSOR);
    CHECK(emi_mode_from_name("stress") == EMI_MODE_STRESS);
    CHECK(emi_mode_from_name("deep") == -1);
    CHECK(emi_mode_from_name(NULL) == -1);
    CHECK(!strcmp(emi_mode_name(EMI_MODE_CI), "ci"));
    CHECK(!strcmp(emi_mode_name((emi_mode_t)99), "?"));
    CHECK(emi_mode_desc((emi_mode_t)99) == NULL);
    CHECK(EMI_MODE_DEFAULT == EMI_MODE_CI);
}

/* The lab mode: small captures only (the SDK's 4,096 samples), no deep capture, every 500 ms,
   10 Mbit/s, paced read-back, NMEA on LA8, the board's DAC. */
static void test_ci_is_small(void) {
    const emi_mode_desc_t *d = emi_mode_desc(EMI_MODE_CI);
    CHECK(d && d->n_caps >= 1);
    for (unsigned i = 0; i < d->n_caps; i++) {
        CHECK(max_samples(d->caps[i].json) <= 4096);
        CHECK(max_samples(d->caps[i].json) > 0);
        CHECK(strstr(d->caps[i].json, "replay") == NULL);
    }
    CHECK(d->period_ms == 500u);
    CHECK(d->net_mbit == 10u);
    CHECK(d->readback_bps > 0u);
    CHECK(d->uart_la == 8u);
    CHECK(d->dac == -1);
    /* every lab-like mode sends at least 10 % of a 100 Mbit/s link */
    for (int m = 0; m < EMI_MODE_COUNT; m++) CHECK(emi_mode_desc((emi_mode_t)m)->net_mbit >= 10u);
}

/* The stress mode keeps the loop before the modes: deep captures back to back, 200 ms apart,
   unpaced, 20 Mbit/s. */
static void test_stress_is_old_loop(void) {
    const emi_mode_desc_t *d = emi_mode_desc(EMI_MODE_STRESS);
    CHECK(d->n_caps == 2);
    CHECK(max_samples(d->caps[0].json) == 200000);
    CHECK(max_samples(d->caps[1].json) == 1000000);
    CHECK(d->period_ms == 0u && d->gap_ms == 200u);
    CHECK(d->readback_bps == 0u && d->net_mbit == 20u);
    CHECK(emi_sched_next_start(d, 1000u, 3000u) == 3200u);
}

static void test_next_start(void) {
    const emi_mode_desc_t *d = emi_mode_desc(EMI_MODE_CI);
    /* on time: start to start */
    CHECK(emi_sched_next_start(d, 1000u, 1020u) == 1500u);
    /* a capture that ran past its period: the gap after it */
    CHECK(emi_sched_next_start(d, 1000u, 1480u) == 1530u);
    CHECK(emi_sched_next_start(d, 1000u, 2000u) == 2050u);
    /* across the 32-bit wrap */
    CHECK(emi_sched_next_start(d, 0xFFFFFF00u, 0xFFFFFF10u) == 0xFFFFFF00u + 500u);
    const emi_mode_desc_t *i = emi_mode_desc(EMI_MODE_INTERACTIVE);
    CHECK(emi_sched_next_start(i, 0u, 30u) == 10000u);
}

static void test_next_cap(void) {
    const emi_mode_desc_t *d = emi_mode_desc(EMI_MODE_CI);
    unsigned k = emi_sched_next_cap(d, d->n_caps, true);
    CHECK(k == 0u);
    unsigned k2 = emi_sched_next_cap(d, k, true);
    CHECK(k2 == 1u);
    CHECK(emi_sched_next_cap(d, k2, true) == 0u);   /* in turn */
    /* a digital-only board: only those without the ADC */
    for (unsigned n = 0, c = d->n_caps; n < 5; n++) {
        c = emi_sched_next_cap(d, c, false);
        CHECK(c < d->n_caps && !d->caps[c].needs_adc);
    }
    const emi_mode_desc_t *s = emi_mode_desc(EMI_MODE_STRESS);
    CHECK(emi_sched_next_cap(s, s->n_caps, false) == 1u);   /* the LA-only deep capture */
}

static void test_overdue(void) {
    CHECK(!emi_sched_overdue(1000u, 1000u));
    CHECK(!emi_sched_overdue(1000u, 1000u + EMI_SCHED_OVERDUE_MS - 1u));
    CHECK(emi_sched_overdue(1000u, 1000u + EMI_SCHED_OVERDUE_MS));
    CHECK(!emi_sched_overdue(5000u, 1000u));   /* not due yet */
    CHECK(emi_sched_overdue(0xFFFFFFF0u, 0xFFFFFFF0u + EMI_SCHED_OVERDUE_MS));
}

static void test_budget(void) {
    CHECK(emi_sched_budget(0u, 1u, 0u) == EMI_SCHED_BUDGET_MAX);    /* unpaced */
    CHECK(emi_sched_budget(0u, 1u, 4000000u) == 4000u);             /* 4 MB/s: 4 KB a ms */
    CHECK(emi_sched_budget(4000u, 2u, 4000000u) == 12000u);
    CHECK(emi_sched_budget(12000u, 50u, 4000000u) == EMI_SCHED_BUDGET_MAX);   /* capped */
    CHECK(emi_sched_budget(0u, 5000u, 100000u) == 10000u);          /* dt capped at 100 ms */
    /* paced over one second: about the rate, never more */
    uint32_t b = 0, sent = 0;
    for (int ms = 0; ms < 1000; ms++) { b = emi_sched_budget(b, 1u, 4000000u); sent += b; b = 0; }
    CHECK(sent == 4000000u);
}

static void test_busy(void) {
    emi_busy_t b;
    emi_busy_reset(&b, 0u);
    CHECK(!b.have);
    /* busy 15 ms of every 500 ms, ticked every ms: 3.0 % */
    for (uint32_t t = 1; t <= EMI_SCHED_BUSY_WINDOW_MS; t++) emi_busy_tick(&b, t, (t % 500u) < 15u);
    CHECK(b.have);
    CHECK(b.last_permille == 30u);
    for (uint32_t t = EMI_SCHED_BUSY_WINDOW_MS + 1u; t <= 2u * EMI_SCHED_BUSY_WINDOW_MS; t++)
        emi_busy_tick(&b, t, true);
    CHECK(b.last_permille == 1000u);
    emi_busy_t z;
    memset(&z, 0, sizeof(z));
    emi_busy_tick(&z, 123u, true);   /* the first tick only starts it */
    CHECK(z.init && z.busy_ms == 0u);
}

/* Simulate the ci schedule for 60 s with captures of 15 ms: two kinds in turn, 120 captures. */
static void test_ci_minute(void) {
    const emi_mode_desc_t *d = emi_mode_desc(EMI_MODE_CI);
    uint32_t next = 0, started = 0, end = 0;
    unsigned k = d->n_caps, n[2] = {0, 0}, total = 0;
    bool busy = false;
    for (uint32_t now = 0; now < 60000u; now++) {
        if (busy && now >= end) { busy = false; next = emi_sched_next_start(d, started, now); }
        if (!busy && (int32_t)(now - next) >= 0) {
            k = emi_sched_next_cap(d, k, true);
            n[k]++; total++;
            started = now; end = now + 15u; busy = true;
        }
    }
    CHECK(total == 120u);
    CHECK(n[0] == 60u && n[1] == 60u);
}

int main(void) {
    test_names();
    test_ci_is_small();
    test_stress_is_old_loop();
    test_next_start();
    test_next_cap();
    test_overdue();
    test_budget();
    test_busy();
    test_ci_minute();
    if (failures) { printf("test_emi_sched: %d failure(s)\n", failures); return 1; }
    printf("test_emi_sched: all passed\n");
    return 0;
}
