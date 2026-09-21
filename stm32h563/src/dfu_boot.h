/*
 * dfu_boot.h — software-triggered entry into the STM32H563 ROM USB DFU
 * bootloader, so the bench pod can be re-flashed over its existing USB port
 * (PA11/PA12) without an SWD probe.  This is the STM32 analog of the RP2350
 * `bootsel`/UF2 path: `dfu_boot_request()` reboots into the factory bootloader,
 * the host then writes flash with dfu-util (see `benchpod flash-self`).
 *
 * For a *blank* chip the only entry is the hardware BOOT0 pin; this software
 * path only works once firmware that calls dfu_boot_check() is already running.
 */
#ifndef DFU_BOOT_H
#define DFU_BOOT_H

/*
 * Call as the very first statement in main(), before HAL_Init() and any clock
 * or peripheral setup.  If a prior dfu_boot_request() armed the reboot flag,
 * this clears it and jumps to the system-memory bootloader (never returns).
 * Otherwise it returns immediately and the normal application boots.
 */
void dfu_boot_check(void);

/*
 * Arm the DFU reboot and issue a system reset.  Sets a magic value in a no-init
 * RAM cell (preserved across a warm reset) and calls NVIC_SystemReset().  Never
 * returns.  The subsequent boot lands in dfu_boot_check(), which performs the
 * jump from a near-pristine post-reset state.
 */
void dfu_boot_request(void);

#endif /* DFU_BOOT_H */
