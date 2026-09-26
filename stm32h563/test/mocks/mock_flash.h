/* RAM-backed model of the STM32H5 internal flash tail (bank 2, sectors 120-127)
   for the config-store tests. Models what matters for power-loss safety:
   8 KB sector erase, 16-byte quad-word programs, ECC per quad-word (a torn or
   doubly programmed quad-word reads as an ECC double error), and a power cut
   injected after N flash steps (each sector erase and each quad-word program is
   one step), delivered by longjmp to mock_flash_power_jmp. */
#ifndef MOCK_FLASH_H
#define MOCK_FLASH_H

#include <stdint.h>
#include <stddef.h>
#include <setjmp.h>

#define MOCK_FLASH_BASE_OFF  0x1F0000u
#define MOCK_FLASH_BYTES     0x10000u      /* 0x1F0000..0x1FFFFF */
#define MOCK_QW              16u

extern uint8_t mock_flash[MOCK_FLASH_BYTES];
extern uint8_t mock_flash_ecc_bad[MOCK_FLASH_BYTES / MOCK_QW];
extern uint8_t mock_flash_programmed[MOCK_FLASH_BYTES / MOCK_QW];

extern int     mock_flash_erases;          /* sector erases performed */
extern int     mock_flash_programs;        /* quad-words programmed */
extern int     mock_flash_double_programs; /* program of a non-erased quad-word (a bug) */
extern int     mock_flash_ecc_reads;       /* reads that hit an ECC double error: each one is
                                              an NMI on hardware, a reset without the hook */
extern int     mock_flash_irq_depth;       /* save/restore_interrupts balance */

/* Power cut: after this many more steps (0 = off). The step that hits zero is torn. */
extern int     mock_flash_cut_after;
extern jmp_buf mock_flash_power_jmp;

void mock_flash_reset(void);    /* all erased, counters cleared */
void mock_flash_poke(uint32_t off, const void *data, size_t n);   /* raw write, valid ECC */
uint8_t *mock_flash_ptr(uint32_t off);

#endif /* MOCK_FLASH_H */
