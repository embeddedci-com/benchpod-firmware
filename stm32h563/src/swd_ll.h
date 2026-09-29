/* swd_ll.h — SWD line protocol executed on the RP, driven over the FPGA's
 * remote_bitbang engine.
 *
 * This is the bit-level half of the on-pod CMSIS-DAP probe (see dap.c).  It
 * builds remote_bitbang command bytes (the same 'd'/'e'/'f'/'g'/'O'/'o'/'c'
 * stream swd_engine.v decodes) and pushes them through the existing
 * fpga_swd_feed() path — no new FPGA op, no PIO.  The SWD engine must already be
 * armed (fpga_swd_arm); these calls do not arm/disarm it.
 *
 * One transfer is sent as TWO feed phases because the FPGA samples are only
 * read back after a feed completes, so we cannot branch on the ACK mid-feed:
 *   phase 1 = request + turnaround + 3 ACK bits  → decide
 *   phase 2 = data read / data write / WAIT-FAULT dummy
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* CMSIS-DAP transfer ACK codes, returned by swd_ll_transfer().  SWD_ACK_PERR is
 * OR-ed into an otherwise-OK ack when the read data parity check fails. */
#define SWD_ACK_OK     0x01u
#define SWD_ACK_WAIT   0x02u
#define SWD_ACK_FAULT  0x04u
#define SWD_ACK_NO_ACK 0x07u   /* line floated high (no target / disconnected) */
#define SWD_ACK_PERR   0x08u   /* parity error flag, OR-ed onto an OK read      */

/* Transfer request bits (match CMSIS-DAP DAP_TRANSFER_*). */
#define SWD_REQ_APnDP  (1u << 0)
#define SWD_REQ_RnW    (1u << 1)
#define SWD_REQ_A2     (1u << 2)
#define SWD_REQ_A3     (1u << 3)

/* Reset line-layer config to power-on defaults: turnaround=1, no data phase,
 * idle=0.  Call from dap_reset(). */
void swd_ll_reset(void);

/* turnaround: 1..4 clock periods (DAP_SWD_Configure low 2 bits + 1).
 * data_phase: always generate the data phase even on WAIT/FAULT. */
void swd_ll_configure(uint8_t turnaround, bool data_phase);

/* Trailing idle clocks after a successful transfer (DAP_TransferConfigure). */
void swd_ll_set_idle(uint8_t idle_cycles);

/* Drive nRESET low (asserted=true) or release it (asserted=false).  Uses the
 * pod's dedicated /NRST_CONTROL pin (nrst_ctrl.h), not an LA channel, so it is
 * independent of how the SWD engine was armed.  No-op on a v2 board, which has
 * no such pin. */
void swd_ll_nreset(bool asserted);

/* Execute one SWD transfer.  `request` uses the SWD_REQ_* bits.  For a write,
 * *data is the value sent; for a read, *data receives the value.  Returns the
 * ACK (OK/WAIT/FAULT/NO_ACK, optionally |SWD_ACK_PERR on a read parity error). */
uint8_t swd_ll_transfer(uint8_t request, uint32_t *data);

/* Clock out `nbits` from `data`, LSB-first within each byte, SWDIO driven.
 * Used for SWJ sequences (line reset, JTAG-to-SWD).  nbits <= 256. */
void swd_ll_seq_out(const uint8_t *data, uint32_t nbits);

/* Clock in `nbits`, SWDIO released, packing samples LSB-first into `data`
 * (ceil(nbits/8) bytes written).  nbits <= 256. */
void swd_ll_seq_in(uint8_t *data, uint32_t nbits);

/* ---- batched transfers in the gateware (v45+ SWD queue) ------------------------------------
 * swd_ll_transfer() bit-bangs one transfer over the FPGA link: ~140 link bytes and ~0.65 ms per
 * 32-bit write.  A batch queues whole transfers (1 byte per read, 5 per write) and the fabric
 * runs them at SWCLK speed, stopping at the first ACK other than OK. */
typedef struct {
    uint8_t  req;     /* SWD_REQ_* bits */
    uint32_t data;    /* the value, for a write */
} swd_op_t;

/* True when swd_ll_batch() can run: gateware with the queue, turnaround 1, no data phase. */
bool   swd_ll_batch_supported(void);
/* Run ops[0..n) in order; each read's value goes to rdata[] in order.  WAIT is retried (up to
 * the wait-retry count per transfer, like swd_ll_transfer's callers do).  Returns the number of
 * ops completed; *ack is SWD_ACK_OK when all were, else the stopping op's ACK
 * (SWD_ACK_OK | SWD_ACK_PERR for a read parity error, whose value is then in rdata too). */
size_t swd_ll_batch(const swd_op_t *ops, size_t n, uint32_t *rdata, uint8_t *ack);
void   swd_ll_set_wait_retry(uint16_t retries);
/* SWCLK for batches (DAP_SWJ_Clock): 24 MHz / (2 * half), half 2..63, never faster than hz. */
void   swd_ll_set_clock(uint32_t hz);
