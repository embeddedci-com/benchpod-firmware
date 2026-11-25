#ifndef PROG_PIC_H
#define PROG_PIC_H

/* prog_pic — PIC ICSP with low-voltage entry, for prog_flash (prog_flash.h).
 *
 * Entry: the key "MCHP" goes in LSb first with 33 clocks on the 6-bit set (DS41439 fig. 8-8), MSb first
 * with 32 clocks on the 8-bit sets. PIC_F_KEY_FLIP reverses that for a chip that differs.
 *
 * Pins: ICSPCLK = clk, ICSPDAT = data, MCLR = the pod's reset pin (nrst(true) = MCLR low).
 * The host picks the command set and passes the geometry in ctx->flags / page / addr:
 *
 *  - Set A (default): PIC16F1xxx enhanced mid-range. 6-bit commands, 14-bit words, LSB first
 *    (DS41397, DS41573). The device ID at 0x8006 packs DEV<8:0> in bits 13:5 and REV<4:0> in bits
 *    4:0 (DS41439): id[0..1] = DEV, id[2..3] = REV. A chip with separate words (0x8005 = REV) sets
 *    PIC_F_ID_SPLIT. Flash, config and user ID share the program-memory PC (config space
 *    starts at word 0x8000, reached with Load Configuration); EEPROM is the data memory.
 *  - Set B (PIC_F_SET8): 8-bit commands with a 24-bit payload, MSB first. PIC16F18xxx (14-bit
 *    words, PC in words; DS40001970) and PIC18F Q-series (PIC_F_BYTEPC: PC in bytes). Q-series
 *    has no Load Data / Begin Programming: every word is one Program Data (0xE0) with a 24-bit
 *    payload, and the chip times it itself (TPINT, or TPDFM for EEPROM and Q43/Q84 config bytes).
 *    Chip erase: Q43/Q84 Bulk Erase 0x18 with payload 0x0F (EEPROM, flash, user ID, config);
 *    Q10 (PIC_F_Q10) takes no payload, the PC picks the regions: 0x300000 then 0x310000 (EEPROM).
 *    Bulk erase needs VDD above the brown-out level (~2.7 to 2.85 V; Q10 ~1.9 V; PIC16F1847 2.7 V):
 *    at 1.8 V the erase can be ignored (the old contents and a code-protect lock stay).
 *
 * Addresses (ctx->addr, bytes). The image offsets (src, dst) start at 0 for ctx->addr.
 *  - FLASH, and CONFIG / USER on set A and PIC16F18: absolute word address * 2 (so word 0x8007
 *    is 0x1000E). Two image bytes per word, little endian, 14 significant bits on the 14-bit chips.
 *    Config words and user ID are written one word at a time.
 *  - PIC18 (PIC_F_BYTEPC): addr is the byte address of the spec (0x300000 config, 0x200000 user
 *    ID, ...). FLASH and USER (user ID) are 16-bit words (PC step 2), EEPROM and the Q43/Q84
 *    CONFIG bytes are single bytes (step 1), the Q10 CONFIG words are 16-bit (step 2).
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
#define PIC_F_Q10      (1u << 4)   /* with BYTEPC: PIC18 Q10: bulk erase has no payload, 16-bit config words, TERAB 75 ms */
#define PIC_F_ID_SPLIT (1u << 5)   /* set A: revision at 0x8005 and device ID at 0x8006 as separate words */
#define PIC_F_KEY_FLIP (1u << 6)   /* entry key bit order and clocks of the other command set */
#define PIC_F_CP_BIT(n) (((uint32_t)(n) & 15u) << 8)   /* bit number of the code-protect bit */

#endif /* PROG_PIC_H */
