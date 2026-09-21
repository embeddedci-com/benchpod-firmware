/* Pico SDK <hardware/regs/addressmap.h> shim.  On RP2350 flash is read via the
   XIP window at XIP_BASE; on STM32H5 internal flash is memory-mapped at
   FLASH_BASE (0x08000000), so reads of (XIP_BASE + offset) work directly. */
#ifndef HW_ADDRESSMAP_SHIM_H
#define HW_ADDRESSMAP_SHIM_H
#define XIP_BASE   0x08000000u
#endif
