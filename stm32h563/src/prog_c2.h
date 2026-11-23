#ifndef PROG_C2_H
#define PROG_C2_H

/* prog_c2 — Silicon Labs C2 (EFM8 BB1/BB2/BB3/LB1/SB1/UB1/2/3, C8051F) for prog_flash (prog_flash.h).
 *
 * ctx->id: [0] device ID, [1] revision ID, [2] derivative ID (id_len 3 when the chip answers the
 * derivative command, else 2).
 * ctx->mem: PROG_MEM_FLASH and PROG_MEM_CONFIG (the lock byte; same FPI commands, the host passes its
 * address). Addresses are 16 bit: addr + len <= 0x10000.
 * ctx->page: flash page in bytes (512 on most EFM8, 1024 on EFM8SB2 and 'F36x/'F92x/'F93x/'F96x:
 * AN127 Table 3.5), used for PROG_ERASE_PAGE and to keep a block
 * write inside one page. Image offsets handed to src()/dst() start at 0 for ctx->addr.
 * ctx->flags: bits 0-7: the C2 address of FPDAT, or 0 to pick it from the device ID (0xAD on
 * EFM8UB2, 'F34x, 'F38x, 'T62x/'T32x, else 0xB4). Bit 8 (C2_FLAG_NO_SETUP): skip the
 * device-specific setup before erase and write (VDD monitor, flash timing, clock).
 * Errors: when the FPI answers something other than 0x0D, ctx->at_addr is that raw status byte.
 */

#define PROG_C2_FPDAT_DEFAULT 0xB4u
#define C2_FLAG_NO_SETUP      0x100u

#endif
