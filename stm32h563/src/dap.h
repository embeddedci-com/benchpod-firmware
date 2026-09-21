/* dap.h — on-pod CMSIS-DAP command processor (SWD only).
 *
 * Turns the pod into a real CMSIS-DAP probe: dap_process() consumes one
 * CMSIS-DAP command packet and produces one response packet, executing SWD
 * transfers locally via swd_ll.c.  The transport (length-framed packets over the
 * cloud tunnel / TCP) and the SWD arming live in command_handler.c; this module
 * is pure logic and is unit-tested against a mock swd_ll.
 *
 * Only the SWD subset pyOCD needs is implemented (see docs/dap-over-tunnel.md).
 */
#pragma once

#include <stdint.h>
#include <stddef.h>

/* Max CMSIS-DAP packet we advertise (DAP_Info PACKET_SIZE).  Bounds both the
 * request reassembly buffer and the response buffer in command_handler.c. */
#define DAP_PACKET_SIZE 256u

/* Reset processor config (idle/retries/match) and the SWD line layer.  Call when
 * a connection enters DAP mode (dap_start). */
void dap_reset(void);

/* Process one CMSIS-DAP command.  `req` points at the command id byte, `req_len`
 * is its length.  Writes the response (starting with the echoed command id, or
 * 0xFF for an unknown command) into `resp` (capacity >= DAP_PACKET_SIZE) and
 * returns the response length in bytes. */
size_t dap_process(const uint8_t *req, size_t req_len, uint8_t *resp, size_t resp_cap);
