/* Recording / modelling mock of swd_ll.h for test_dap.c.
 *
 * Models just enough SWD behaviour to validate dap.c's CMSIS-DAP processing,
 * including the posted-AP-read pipeline:
 *   - an AP read returns the PREVIOUS posted value and latches the next value
 *     from ap_seq[]; an RDBUFF read (DP A[3:2]=0b11) returns the posted value.
 *   - DP reads return mock_swd_ll_dp_value.
 *   - writes are captured into mock_swd_ll_writes[].
 *   - mock_swd_ll_wait_before_ok injects N WAIT acks before succeeding.
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>

void mock_swd_ll_reset(void);

/* Programmable inputs */
extern uint8_t  mock_swd_ll_ack;            /* ack returned once not WAITing (default OK) */
extern uint32_t mock_swd_ll_dp_value;       /* value for plain DP reads */
extern uint32_t mock_swd_ll_ap_seq[64];     /* values an AP "produces" in order */
extern int      mock_swd_ll_ap_seq_len;
extern int      mock_swd_ll_wait_before_ok; /* WAIT this many times, then ack */

/* Captured outputs */
extern uint32_t mock_swd_ll_writes[256];
extern int      mock_swd_ll_writes_n;
extern int      mock_swd_ll_xfer_calls;
extern int      mock_swd_ll_nreset_calls;
extern bool     mock_swd_ll_last_nreset;
