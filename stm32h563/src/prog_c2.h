#ifndef PROG_C2_H
#define PROG_C2_H

/* prog_c2 — Silicon Labs C2 (EFM8 BB1/BB2/BB3/LB1/SB1/UB1/2/3, C8051F) for prog_flash (prog_flash.h).
 *
 * ctx->id: [0] device ID, [1] revision ID, [2] derivative ID (id_len 3 when the chip answers the
 * derivative command, else 2).
 * ctx->mem: PROG_MEM_FLASH and PROG_MEM_CONFIG (the lock byte; same FPI commands, the host passes its
 * address). Addresses are 16 bit: addr + len <= 0x10000.
 * ctx->page: flash page in bytes (512 on EFM8), used for PROG_ERASE_PAGE and to keep a block write
 * inside one page. Image offsets handed to src()/dst() start at 0 for ctx->addr.
 * ctx->flags: 0, or the C2 address of FPDAT when it is not 0xB4 (some C8051F parts use 0xAD).
 */

#define PROG_C2_FPDAT_DEFAULT 0xB4u

#endif
