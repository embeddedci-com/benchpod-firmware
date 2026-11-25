#ifndef PROG_UPDI_H
#define PROG_UPDI_H

/* prog_updi — UPDI programming (tinyAVR 0/1/2, megaAVR 0, AVR DA/DB/DD) for prog_flash.h.
 *
 * Wiring: one pin, UART engine, half duplex, 8E2. The pod does not read back its own bytes.
 *
 * What the host passes (prog_ctx_t):
 *   baud   link rate, 0 = 115200, at most 225000 (the chip's default UPDI clock)
 *   page   write page in bytes for flash / eeprom / user row (ignored for config)
 *   addr   offset inside the memory: flash 0 = first flash byte, eeprom 0 = first EEPROM byte,
 *          config 0 = first fuse byte, user 0 = first user row byte
 *   flags  data space address of flash byte 0; 0 = the default for the SIB family: 0x4000 for
 *          megaAVR 0 (SIB family "megaAVR"), 0x8000 for the other NVM version 0 parts (tinyAVR
 *          0/1/2), 0x800000 for NVM version 2 (DA/DB/DD). A non-zero value overrides it.
 * The NVM controller version comes from the SIB: P:0 (tiny 0/1/2, mega 0) or P:2 (DA/DB/DD).
 * Others answer PROG_E_UNSUPPORTED. The 3 signature bytes are returned in ctx->id.
 *
 * A locked chip reads no signature: enter() answers PROG_E_LOCKED unless the job asked for
 * PROG_ERASE_CHIP, in which case enter() unlocks it with the chip erase key (the chip is then
 * already erased and the erase step is a no-op). The wrong-chip check therefore happens after
 * the erase for a locked chip. If the chip refuses that key erase (ASI_SYS_STATUS.ERASEFAIL),
 * enter() answers PROG_E_LOCKED with phase "erase".
 *
 * Entry: enable pulse, SYNC 500 us later, double BREAK only if the chip does not answer (the SYNC
 * follows the BREAK at once); NVMPROG is polled for up to 100 ms. v2 EEPROM, fuse and user row bytes
 * are ACKed stores (the ACK waits for the write); only flash words are streamed without ACKs.
 */

#endif /* PROG_UPDI_H */
