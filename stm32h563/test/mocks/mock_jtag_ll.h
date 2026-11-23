/* Modelling mock of jtag_ll.h for test_dap_jtag.c (and a link stub for test_dap.c).
 *
 * jtag_ll_run() clocks the DAP_JTAG_Sequence stream through a chain of IEEE 1149.1 TAPs, bit by
 * bit: TDO is taken before each rising edge, every TAP shifts on the edge, and device 0 is the one
 * nearest TDO (CMSIS-DAP's numbering).  One device can be an ARM JTAG-DP (IR 4: ABORT 0x8,
 * DPACC 0xA, APACC 0xB, IDCODE 0xE) with CTRL/STAT, SELECT and RDBUFF and a MEM-AP (CSW, TAR,
 * DRW with auto-increment, IDR) over a small memory.  A DPACC/APACC scan captures {result of the
 * previous transaction, ACK}, and a WAIT capture makes the DP ignore that scan's request.
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>

#define MOCK_JTAG_DEV_MAX 4
#define MOCK_JTAG_MEM_WORDS 1024

void mock_jtag_ll_reset(void);
/* Device i (0 = nearest TDO): IR length and IDCODE; dp marks the ARM JTAG-DP. */
void mock_jtag_ll_add(unsigned irlen, uint32_t idcode, uint32_t idcode_ir, bool dp);

extern int      mock_jtag_ll_wait;         /* the next N DPACC/APACC captures answer WAIT */
extern bool     mock_jtag_ll_fail;         /* jtag_ll_run returns -2 */
extern uint32_t mock_jtag_ll_mem[MOCK_JTAG_MEM_WORDS];
extern uint32_t mock_jtag_ll_ctrl_stat;
extern uint32_t mock_jtag_ll_abort;        /* the last ABORT write */
extern int      mock_jtag_ll_aborts;
extern int      mock_jtag_ll_runs;         /* jtag_ll_run calls */
extern long     mock_jtag_ll_tcks;         /* TCK cycles clocked */
extern int      mock_jtag_ll_dp_execs;     /* DPACC/APACC requests the DP executed */
extern bool     mock_jtag_ll_trst;         /* last jtag_ll_trst(asserted) */
extern int      mock_jtag_ll_trst_calls;
extern uint32_t mock_jtag_ll_clock_hz;
int             mock_jtag_ll_state(unsigned dev);   /* TAP state: 0 = Test-Logic-Reset, 1 = Run-Test/Idle */
