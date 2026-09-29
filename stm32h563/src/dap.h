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

/* Largest CMSIS-DAP packet the pod handles: the request reassembly buffer and the response
 * buffer in command_handler.c.  What DAP_Info ADVERTISES is negotiated per session
 * (dap_configure): 256 x 1 unless the client asks for more in dap_start, because the benchpod
 * CLI's bridge (<= 0.1.5) breaks on larger packets.  Every packet costs a host + network round
 * trip (~8 ms on the LAN, far more over the cloud), and a 256-byte packet carries 62 words. */
#define DAP_PACKET_SIZE      1024u
#define DAP_PACKET_DEFAULT   256u
#define DAP_PACKET_COUNT_MAX 4u

/* Set what DAP_Info reports for this session: size clamped to 64..DAP_PACKET_SIZE, count to
 * 1..DAP_PACKET_COUNT_MAX.  dap_reset() goes back to 256 x 1. */
void dap_configure(unsigned packet_size, unsigned packet_count);

/* Reset processor config (idle/retries/match) and the SWD line layer.  Call when
 * a connection enters DAP mode (dap_start). */
void dap_reset(void);

/* Process one CMSIS-DAP command.  `req` points at the command id byte, `req_len`
 * is its length.  Writes the response (starting with the echoed command id, or
 * 0xFF for an unknown command) into `resp` (capacity >= DAP_PACKET_SIZE) and
 * returns the response length in bytes. */
size_t dap_process(const uint8_t *req, size_t req_len, uint8_t *resp, size_t resp_cap);
