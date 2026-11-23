/* dap.h — on-pod CMSIS-DAP command processor (SWD, and JTAG on gateware v61+).
 *
 * Turns the pod into a real CMSIS-DAP probe: dap_process() consumes one
 * CMSIS-DAP command packet and produces one response packet, executing SWD
 * transfers locally via swd_ll.c and JTAG ones via dap_jtag.c / jtag_ll.c.  The transport (length-framed packets over the
 * cloud tunnel / TCP) and the SWD arming live in command_handler.c; this module
 * is pure logic and is unit-tested against a mock swd_ll.
 *
 * SWD: the subset pyOCD and OpenOCD use.  JTAG: DAP_JTAG_Sequence/Configure/IDCODE and JTAG-DP
 * transfers (see docs/dap-over-tunnel.md).
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

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

/* Reset processor config (idle/retries/match), the SWD line layer and the JTAG chain; the ports
 * go back to SWD only.  Call when a connection enters DAP mode (dap_start). */
void dap_reset(void);

/* The debug ports dap_start armed pins for (DAP_Info capabilities, DAP_Connect), the one the
 * engine runs now, and how to switch: the callback re-arms the engine for the other port and
 * returns false when it cannot.  NULL: no switching.  Call after dap_reset(). */
#define DAP_PORT_SWD    1u
#define DAP_PORT_JTAG   2u
#define DAP_PORTS_SWD   0x01u
#define DAP_PORTS_JTAG  0x02u
typedef bool (*dap_port_switch_fn)(uint8_t port);
void    dap_set_ports(uint8_t ports, uint8_t active, dap_port_switch_fn sw);
uint8_t dap_port(void);

/* Process one CMSIS-DAP command.  `req` points at the command id byte, `req_len`
 * is its length.  Writes the response (starting with the echoed command id, or
 * 0xFF for an unknown command) into `resp` (capacity >= DAP_PACKET_SIZE) and
 * returns the response length in bytes. */
size_t dap_process(const uint8_t *req, size_t req_len, uint8_t *resp, size_t resp_cap);
