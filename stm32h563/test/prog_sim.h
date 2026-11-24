#ifndef PROG_SIM_H
#define PROG_SIM_H

/* prog_sim — a prog_wire_ops_t for host tests, wired to a bit-level model of the chip.
 *
 * Two-wire: every set() is applied at once (no queue) and the model sees the new pin levels; a
 * sample() reads the line as it is now: the chip's drive if it drives, else the pod's drive if the
 * pod drives, else the pull-up (1). Both driving with different levels is a contention: counted in
 * sim->contention, and tests assert it stays 0. flush() returns the samples taken since the last
 * flush. The sim also records whether the clock was low at a flush (sim->low_at_flush) and the
 * longest clock-low time in simulated us (delay_us adds to the clock), for the C2/PDI rules.
 *
 * Byte time: with sim->byte_ns set, every queued command (set, drive, sample) advances the simulated
 * time by that much first, as the pod's byte mode does (one SPI byte per command); sim->ns is the
 * time in ns (us follows it), and the longest clock low is also kept in ns (clk_low_max_ns). Left at 0,
 * time passes only in delay_us() as before.
 *
 * UPDI: uart_write() hands bytes to the model (on_uart_rx) and echoes nothing, as the gateware's
 * half-duplex receiver ignores the line while the pod sends; the model answers with
 * prog_sim_uart_reply().
 */

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "../src/prog_wire.h"

typedef struct prog_sim prog_sim_t;

typedef struct {
    /* pin levels changed (pod side): clk, data as the pod drives it (-1 = released) */
    void (*on_pins)(prog_sim_t *s, int clk, int pod_data);
    /* the chip's drive on data now: 0, 1, or -1 (released) */
    int  (*chip_data)(prog_sim_t *s);
    void (*on_nrst)(prog_sim_t *s, bool low);
    void (*on_time)(prog_sim_t *s, uint32_t us);     /* simulated time passed */
    void (*on_uart_rx)(prog_sim_t *s, const uint8_t *b, size_t n, uint32_t baud, bool parity_even, bool stop2);
} prog_sim_model_t;

#define PROG_SIM_SAMPLES 4096u
#define PROG_SIM_UART    4096u

struct prog_sim {
    prog_sim_model_t m;
    void    *chip;               /* the model's own state */
    int      clk, pod_data;      /* pod_data -1 = released */
    bool     pod_drives;
    uint8_t  samples[PROG_SIM_SAMPLES];
    size_t   nsamples;
    uint32_t us;                 /* simulated time */
    uint32_t clk_low_since, clk_low_max_us;
    uint32_t byte_ns;            /* simulated time per command, 0 = off */
    uint64_t ns;                 /* simulated time in ns */
    uint64_t clk_low_since_ns, clk_low_max_ns;
    unsigned contention, low_at_flush;
    bool     nrst_low;
    uint32_t baud; bool parity_even, stop2;
    uint8_t  rx[PROG_SIM_UART];  size_t rx_head, rx_tail;
};

void prog_sim_init(prog_sim_t *s, prog_wire_ops_t *ops, const prog_sim_model_t *m, void *chip);
/* The model queues bytes the pod will read. */
void prog_sim_uart_reply(prog_sim_t *s, const uint8_t *b, size_t n);

#endif /* PROG_SIM_H */
