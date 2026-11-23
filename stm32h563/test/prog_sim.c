/* prog_sim.c — see prog_sim.h. */
#include "prog_sim.h"
#include <string.h>

static void notify(prog_sim_t *s) {
    if (s->m.on_pins) s->m.on_pins(s, s->clk, s->pod_drives ? s->pod_data : -1);
}

static int line(prog_sim_t *s) {
    int chip = s->m.chip_data ? s->m.chip_data(s) : -1;
    if (chip >= 0 && s->pod_drives && chip != s->pod_data) s->contention++;
    if (chip >= 0) return chip;
    if (s->pod_drives) return s->pod_data;
    return 1;   /* pull-up */
}

static void sim_set(void *io, int clk, int data) {
    prog_sim_t *s = io;
    if (clk == 0 && s->clk == 1) s->clk_low_since = s->us;
    if (clk == 1 && s->clk == 0) {
        uint32_t low = s->us - s->clk_low_since;
        if (low > s->clk_low_max_us) s->clk_low_max_us = low;
    }
    s->clk = clk; s->pod_data = data;
    notify(s);
    (void)line(s);   /* count contention at every edge */
}

static void sim_drive(void *io, bool drive) {
    prog_sim_t *s = io;
    s->pod_drives = drive;
    notify(s);
}

static void sim_sample(void *io) {
    prog_sim_t *s = io;
    if (s->nsamples < PROG_SIM_SAMPLES) s->samples[s->nsamples++] = (uint8_t)line(s);
}

static int sim_flush(void *io, uint8_t *samples, size_t cap) {
    prog_sim_t *s = io;
    if (s->clk == 0) s->low_at_flush++;
    size_t n = s->nsamples < cap ? s->nsamples : cap;
    if (samples) memcpy(samples, s->samples, n);
    s->nsamples = 0;
    return (int)n;
}

static void sim_nrst(void *io, bool low) {
    prog_sim_t *s = io;
    s->nrst_low = low;
    if (s->m.on_nrst) s->m.on_nrst(s, low);
}

static void sim_delay(void *io, uint32_t us) {
    prog_sim_t *s = io;
    s->us += us;
    if (s->m.on_time) s->m.on_time(s, us);
}

static uint32_t sim_now(void *io) { return ((prog_sim_t *)io)->us / 1000u; }

static int sim_uart_config(void *io, uint32_t baud, bool parity_even, bool stop2) {
    prog_sim_t *s = io;
    s->baud = baud; s->parity_even = parity_even; s->stop2 = stop2;
    return 0;
}

static int sim_uart_write(void *io, const uint8_t *b, size_t n) {
    prog_sim_t *s = io;
    prog_sim_uart_reply(s, b, n);    /* the half-duplex line echoes */
    if (s->m.on_uart_rx) s->m.on_uart_rx(s, b, n, s->baud, s->parity_even, s->stop2);
    return 0;
}

static size_t sim_uart_read(void *io, uint8_t *b, size_t n, uint32_t timeout_ms) {
    prog_sim_t *s = io;
    size_t k = 0;
    while (k < n && s->rx_tail != s->rx_head) {
        b[k++] = s->rx[s->rx_tail];
        s->rx_tail = (s->rx_tail + 1) % PROG_SIM_UART;
    }
    if (k < n) sim_delay(io, timeout_ms * 1000u);   /* the caller waited the whole timeout */
    return k;
}

static void sim_rx_clear(void *io) {
    prog_sim_t *s = io;
    s->rx_tail = s->rx_head;
}

void prog_sim_uart_reply(prog_sim_t *s, const uint8_t *b, size_t n) {
    for (size_t i = 0; i < n; i++) {
        size_t next = (s->rx_head + 1) % PROG_SIM_UART;
        if (next == s->rx_tail) return;
        s->rx[s->rx_head] = b[i];
        s->rx_head = next;
    }
}

void prog_sim_init(prog_sim_t *s, prog_wire_ops_t *ops, const prog_sim_model_t *m, void *chip) {
    memset(s, 0, sizeof(*s));
    s->m = *m; s->chip = chip;
    s->clk = 1; s->pod_data = 1;
    memset(ops, 0, sizeof(*ops));
    ops->io = s;
    ops->set = sim_set; ops->data_drive = sim_drive; ops->sample = sim_sample; ops->flush = sim_flush;
    ops->nrst = sim_nrst; ops->delay_us = sim_delay; ops->now_ms = sim_now;
    ops->uart_config = sim_uart_config; ops->uart_write = sim_uart_write;
    ops->uart_read = sim_uart_read; ops->uart_rx_clear = sim_rx_clear;
}
