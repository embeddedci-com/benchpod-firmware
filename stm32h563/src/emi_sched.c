/*
 * emi_sched — the EMI test build's operating modes and capture schedule (emi_sched.h).
 */
#include "emi_sched.h"

#include <string.h>

/* The small captures: what the SDK and MCP take by default (4,096 samples at the top rate), the
   LA at its top dense rate, then the ADC with the LA beside it at the web Analyzer's default rate. */
static const emi_cap_t k_small[] = {
    { "la_capture", "{\"cmd\":\"la_capture\",\"samples\":4096,\"sample_rate_hz\":12000000}", false },
    { "capture_dual", "{\"cmd\":\"capture_dual\",\"adc_samples\":4096,\"la_samples\":4096,"
                      "\"adc_sample_rate_hz\":400000,\"la_sample_rate_hz\":1000000}", true },
};

/* The deep captures of the stress mode (the loop before the modes): ADC + LA, the ADC at its full
   rate and the LA beside it, then a deep LA-only capture at the LA's top rate. */
static const emi_cap_t k_deep[] = {
    { "capture_dual", "{\"cmd\":\"capture_dual\",\"adc_samples\":200000,\"la_samples\":200000,"
                      "\"adc_sample_rate_hz\":400000,\"la_sample_rate_hz\":2000000}", true },
    { "la_capture", "{\"cmd\":\"la_capture\",\"samples\":1000000}", false },
};

#define N(a) ((uint8_t)(sizeof(a) / sizeof((a)[0])))

/* The DAC values are emi_dac_mode_t (emi_test.h): 1 = dc. */
static const emi_mode_desc_t k_modes[EMI_MODE_COUNT] = {
    [EMI_MODE_CI] = {
        .name = "ci", .caps = k_small, .n_caps = N(k_small),
        .period_ms = 500u, .gap_ms = 50u, .net_mbit = 10u, .readback_bps = 4000000u,
        .dac = -1, .uart_la = EMI_SCHED_UART_LA,
    },
    [EMI_MODE_INTERACTIVE] = {
        .name = "interactive", .caps = k_small, .n_caps = N(k_small),
        .period_ms = 10000u, .gap_ms = 50u, .net_mbit = 10u, .readback_bps = 4000000u,
        .dac = -1, .uart_la = 0u,
    },
    [EMI_MODE_SENSOR] = {
        .name = "sensor", .caps = k_small, .n_caps = N(k_small),
        .period_ms = 10000u, .gap_ms = 50u, .net_mbit = 10u, .readback_bps = 4000000u,
        .dac = 1, .uart_la = EMI_SCHED_UART_LA,
    },
    [EMI_MODE_STRESS] = {
        .name = "stress", .caps = k_deep, .n_caps = N(k_deep),
        .period_ms = 0u, .gap_ms = 200u, .net_mbit = 20u, .readback_bps = 0u,
        .dac = -1, .uart_la = 0u,
    },
};

const emi_mode_desc_t *emi_mode_desc(emi_mode_t m) {
    return (unsigned)m < EMI_MODE_COUNT ? &k_modes[m] : NULL;
}

const char *emi_mode_name(emi_mode_t m) {
    const emi_mode_desc_t *d = emi_mode_desc(m);
    return d ? d->name : "?";
}

int emi_mode_from_name(const char *name) {
    if (!name) return -1;
    for (int i = 0; i < EMI_MODE_COUNT; i++)
        if (!strcmp(name, k_modes[i].name)) return i;
    return -1;
}

uint32_t emi_sched_next_start(const emi_mode_desc_t *d, uint32_t started_at, uint32_t ended_at) {
    const uint32_t after_gap = ended_at + d->gap_ms;
    if (!d->period_ms) return after_gap;
    const uint32_t due = started_at + d->period_ms;
    /* A capture that ran past its period starts the next one after the gap, not at once. */
    return (int32_t)(due - after_gap) >= 0 ? due : after_gap;
}

unsigned emi_sched_next_cap(const emi_mode_desc_t *d, unsigned cur, bool has_adc) {
    if (!d->n_caps) return d->n_caps;
    if (cur >= d->n_caps) cur = d->n_caps - 1u;   /* the first: start at index 0 */
    for (unsigned k = 1; k <= d->n_caps; k++) {
        const unsigned i = (cur + k) % d->n_caps;
        if (has_adc || !d->caps[i].needs_adc) return i;
    }
    return d->n_caps;
}

bool emi_sched_overdue(uint32_t due, uint32_t now) {
    return (int32_t)(now - due) >= (int32_t)EMI_SCHED_OVERDUE_MS;
}

uint32_t emi_sched_budget(uint32_t budget, uint32_t dt_ms, uint32_t bps) {
    if (!bps) return EMI_SCHED_BUDGET_MAX;
    if (dt_ms > 100u) dt_ms = 100u;
    const uint64_t add = (uint64_t)dt_ms * bps / 1000u;
    const uint64_t b = (uint64_t)budget + add;
    return b > EMI_SCHED_BUDGET_MAX ? EMI_SCHED_BUDGET_MAX : (uint32_t)b;
}

void emi_busy_reset(emi_busy_t *b, uint32_t now) {
    memset(b, 0, sizeof(*b));
    b->win_start = now;
    b->t_last = now;
    b->init = true;
}

void emi_busy_tick(emi_busy_t *b, uint32_t now, bool busy) {
    if (!b->init) { emi_busy_reset(b, now); return; }
    uint32_t dt = now - b->t_last;
    b->t_last = now;
    if (busy) b->busy_ms += dt;
    const uint32_t win = now - b->win_start;
    if (win >= EMI_SCHED_BUSY_WINDOW_MS) {
        uint64_t pm = (uint64_t)b->busy_ms * 1000u / win;
        b->last_permille = (uint16_t)(pm > 1000u ? 1000u : pm);
        b->have = true;
        b->win_start = now;
        b->busy_ms = 0;
    }
}
