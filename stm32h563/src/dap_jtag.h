/* dap_jtag.h — the JTAG commands of the on-pod CMSIS-DAP processor (internal to dap.c).
 *
 * dap.c owns the packet loop and the transfer settings; these functions run one command each when
 * the probe is connected as JTAG.  Every JTAG operation becomes a DAP_JTAG_Sequence stream that
 * jtag_ll.c runs on the gateware (or that the host tests' TAP model runs).
 */
#pragma once

#include <stdint.h>
#include <stddef.h>

#define DAP_JTAG_DEV_MAX 8u   /* devices on the chain (DAP_JTAG_Configure) */

/* Transfer settings dap.c keeps (DAP_TransferConfigure, the match mask, the packet's retry budget). */
typedef struct {
    uint8_t   idle_cycles;
    uint16_t  wait_retry;
    uint16_t  match_retry;
    uint32_t *match_mask;
    uint32_t *budget;
} dap_jtag_cfg_t;

void   dap_jtag_reset(void);

/* Each takes the whole request (req[0] = command id) and writes the whole response (resp[0] is
   already the command id); returns the response length.  resp holds DAP_PACKET_SIZE bytes. */
size_t dap_jtag_sequence(const uint8_t *req, size_t req_len, uint8_t *resp);
size_t dap_jtag_configure(const uint8_t *req, size_t req_len, uint8_t *resp);
size_t dap_jtag_idcode(const uint8_t *req, size_t req_len, uint8_t *resp);
size_t dap_jtag_transfer(const uint8_t *req, size_t req_len, uint8_t *resp, const dap_jtag_cfg_t *cfg);
size_t dap_jtag_transfer_block(const uint8_t *req, size_t req_len, uint8_t *resp, const dap_jtag_cfg_t *cfg);
size_t dap_jtag_write_abort(const uint8_t *req, size_t req_len, uint8_t *resp);

/* DAP_SWJ_Sequence in JTAG mode: nbits on TMS (bit 0 first), TDI high.  0 ok, -1 engine error. */
int    dap_jtag_tms(const uint8_t *bits, uint32_t nbits);
