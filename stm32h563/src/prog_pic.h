#ifndef PROG_PIC_H
#define PROG_PIC_H

/* prog_pic — PIC ICSP with low-voltage entry, for prog_flash (prog_flash.h).
 *
 * Pins: ICSPCLK = clk, ICSPDAT = data, MCLR = the pod's reset pin (nrst(true) = MCLR low).
 * The host picks the command set and passes the geometry in ctx->flags / page / addr:
 *
 *  - Set A (default): PIC16F1xxx enhanced mid-range. 6-bit commands, 14-bit words, LSB first
 *    (DS41397, DS41573). Flash, config and user ID share the program-memory PC (config space
 *    starts at word 0x8000, reached with Load Configuration); EEPROM is the data memory.
 *  - Set B (PIC_F_SET8): 8-bit commands with a 24-bit payload, MSB first. PIC16F18xxx (14-bit
 *    words, PC in words; DS40001753) and PIC18F Q-series (PIC_F_BYTEPC: 16-bit words, PC in
 *    bytes; DS40002079).
 *
 * Addresses (ctx->addr, bytes). The image offsets (src, dst) start at 0 for ctx->addr.
 *  - FLASH, and CONFIG / USER on set A and PIC16F18: absolute word address * 2 (so word 0x8007
 *    is 0x1000E). Two image bytes per word, little endian, 14 significant bits on the 14-bit chips.
 *    Config words and user ID are written one word at a time.
 *  - PIC18 (PIC_F_BYTEPC): CONFIG, USER and EEPROM are bytes; addr is the byte address of the
 *    spec (0x300000 config, 0x200000 user ID, ...). FLASH is addr = byte address, 2 bytes a word.
 *  - EEPROM elsewhere: one image byte per location, addr = the location's PC (set A: data memory
 *    address from 0; PIC16F18: the word address, e.g. 0x7000). Written byte by byte.
 * ctx->page is the flash row in bytes (flash addr must be row aligned). ctx->baud is unused by
 * PIC for speed; here it holds the byte address (same convention as addr) of the config word
 * that carries the code-protect bit, 0 = no lock check.
 *
 * ctx->id: device ID little endian in id[0..1], revision ID in id[2..3], id_len = 4.
 */

#define PIC_F_SET8     (1u << 0)   /* set B: 8-bit commands, 24-bit payload */
#define PIC_F_BYTEPC   (1u << 1)   /* with SET8: PIC18 Q, PC in bytes, 16-bit flash words */
#define PIC_F_HAS_EE   (1u << 2)   /* set A: chip erase also clears the data EEPROM */
#define PIC_F_CP_HIGH  (1u << 3)   /* the code-protect bit is 1 when protected (default: 0) */
#define PIC_F_CP_BIT(n) (((uint32_t)(n) & 15u) << 8)   /* bit number of the code-protect bit */

#endif /* PROG_PIC_H */
