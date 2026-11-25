#ifndef PROG_SBW_H
#define PROG_SBW_H

/* prog_sbw: TI MSP430 Spy-Bi-Wire (FRAM parts with CPUXv2: FR2xx, FR4xx, FR5xxx, FR6xxx) for
 * prog_flash (prog_flash.h). Spec: docs/design/spy-bi-wire.md; section numbers in prog_sbw.c refer to it.
 * Part of the xip_prog image (docs section 17, option a). Needs gateware v52+ (fast SPI link):
 * prog_flash refuses it otherwise (command_handler_prog.c).
 *
 * Pins: clk = SBWTCK (the chip's TEST), data = SBWTDIO (its RST/NMI). The pod's reset pin is unused.
 * ctx->id: [0] JTAG ID (0x98 FR2xx/FR4xx, 0x99, 0x91), [1] device ID low byte, [2] device ID high
 *          byte (TLV 0x1A04/0x1A05); id_len 3. A JTAG-locked chip answers enter with PROG_OK, id_len 1
 *          (JTAG ID only): read and write then return PROG_E_LOCKED, erase chip (mailbox erase) unlocks.
 * ctx->mem: PROG_MEM_FLASH (main FRAM) and PROG_MEM_CONFIG (information FRAM, signatures): the same
 *          accesses, the host passes the address. Addresses are 20 bit, word aligned: addr and len even,
 *          addr + len <= 0x100000.
 * ctx->erase: NONE; PAGE = write 0xFFFF over [addr, addr+len) (FRAM needs no erase: docs section 11);
 *          CHIP = JTAG-mailbox mass erase (docs section 10), about 0.7 s, also unlocks FR2xx/FR4xx.
 * ctx->flags: SBW_FLAG_RST_LOW = enter with the RST-low sequence (the chip does not run user code).
 * ctx->page, ctx->baud: unused.
 * The host must only use it for FRAM devices: F5xx/F6xx flash parts also answer with JTAG IDs 0x91/0x98/0x99
 * (docs section 12) and need the flash controller, not FRAM writes.
 * Errors: PROG_E_NO_ANSWER (no valid JTAG ID after 4 entries), PROG_E_UNSUPPORTED (JTAG ID 0x89:
 * flash parts), PROG_E_LOCKED, PROG_E_TIMEOUT (sync), PROG_E_PROTOCOL, PROG_E_VERIFY (at_addr).
 */

#include "prog_flash.h"

#define SBW_FLAG_RST_LOW 0x01u

#endif
