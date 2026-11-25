/* jtag_ll.h — JTAG line layer of the on-pod CMSIS-DAP probe (dap.c), over the gateware's JTAG
 * master (fpga_jtag_*, gateware v61+, jtag_seq.v).
 *
 * The unit is a CMSIS-DAP DAP_JTAG_Sequence sequence: an info byte (bits 5:0 = TCK cycles, 0
 * means 64; bit 6 = TMS; bit 7 = capture TDO) followed by ceil(n/8) TDI bytes, bit 0 first.  A
 * capturing sequence returns ceil(n/8) TDO bytes, bit 0 first, unused high bits 0.  dap.c builds
 * every JTAG operation (TMS moves, IR/DR scans, DAP_JTAG_Sequence itself) as such a stream and
 * runs it here; the host tests replace this module with a TAP-chain model (mock_jtag_ll.c).
 *
 * The engine must be armed for JTAG (fpga_jtag_arm); these calls do not arm or disarm it.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Run the sequences in seq[0..len) in order.  Their TDO bytes go to tdo[], which must hold the
 * total (jtag_ll_tdo_len).  0 ok; -1 malformed stream (a sequence runs past the end) or tdo too
 * small; -2 the engine failed (timeout, disarmed, no FPGA answer). */
int    jtag_ll_run(const uint8_t *seq, size_t len, uint8_t *tdo, size_t tdo_cap);

/* The number of TDO bytes seq[0..len) returns, or -1 when it is malformed. */
long   jtag_ll_tdo_len(const uint8_t *seq, size_t len);

/* TCK for later runs (DAP_SWJ_Clock): 24 MHz / (2 * half), half 2..63, never faster than hz. */
void   jtag_ll_set_clock(uint32_t hz);

/* nTRST (DAP_SWJ_Pins bit 5) on the LA channel dap_start named, if any: asserted = driven low.
 * Channel 0 = none (the default; DAP_SWJ_Pins then ignores the bit). */
void   jtag_ll_set_trst_channel(unsigned ch);
void   jtag_ll_trst(bool asserted);
