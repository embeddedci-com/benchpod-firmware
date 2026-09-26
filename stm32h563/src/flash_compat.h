/*
 * flash_compat.h — STM32H5 internal-flash helpers beyond the Pico-style
 * <hardware/flash.h> shim (erase/program live there).
 *
 * The H5 programs 16-byte quad-words with ECC. A program or erase cut short by a
 * power loss can leave a quad-word whose ECC no longer matches, and a later CPU
 * read of it sets FLASH_ECCDETR.ECCD and raises an NMI (fault.c turns the NMI
 * into a reset: a crash loop on every boot that reads it). The persistence
 * stores (config_store.c, cloud_config.c) read flash only through
 * flash_read_checked(), so a torn record reads as "invalid" instead.
 */
#ifndef FLASH_COMPAT_H
#define FLASH_COMPAT_H

#include <stdint.h>
#include <stddef.h>

/* Copy `n` bytes of internal flash at `offset` (relative to FLASH_BASE) into `dst`.
   Returns 0 on a clean read, -1 if the read hit an ECC double error (or a bus
   fault). On -1 the contents of `dst` are undefined.

   The ECC NMI is only survivable when the NMI handler calls
   flash_ecc_nmi_absorb() first (see below). Without that hook a torn quad-word
   still resets the MCU, exactly as a plain pointer read would. */
int flash_read_checked(uint32_t offset, void *dst, size_t n);

/* Call at the very top of NMI_Handler. Returns 1 if the NMI was an ECC double
   error raised by a flash_read_checked() probe (the flag is cleared and the
   probe marked failed): the handler must then simply return. Returns 0 for any
   other NMI, which must go on to the normal fault path.

   Suggested fault.c wiring (the NMI trampoline is naked, so do it in asm):

     FAULT_TRAMPOLINE(NMI_Fault, FAULT_NMI)      // was NMI_Handler
     __attribute__((naked)) void NMI_Handler(void) {
         __asm volatile(
             "push {r0, lr}              \n"
             "bl   flash_ecc_nmi_absorb  \n"
             "cmp  r0, #0                \n"
             "pop  {r0, lr}              \n"   // pop keeps the flags
             "it   ne                    \n"
             "bxne lr                    \n"   // absorbed: exception return
             "b    NMI_Fault             \n");
     }
*/
int flash_ecc_nmi_absorb(void);

#endif /* FLASH_COMPAT_H */
