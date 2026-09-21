/*
 * dfu_boot.c — jump into the STM32H563 ROM system-memory (USB DFU) bootloader.
 *
 * Two-step, reset-clean design (mirrors how a clean DFU entry should look):
 *
 *   dfu_boot_request()  sets a magic value in a no-init RAM cell and triggers a
 *                       system reset.  The cell lives in the `.noinit` section,
 *                       which the C startup neither copies nor zeroes, so the
 *                       value survives the warm reset.
 *
 *   dfu_boot_check()    runs as the first statement of main(), before HAL_Init()
 *                       and any clock/peripheral setup.  If the magic is present
 *                       it clears it and jumps to system memory.  Doing the jump
 *                       from this near-pristine post-reset state (default HSI
 *                       clock, no IRQs enabled, no peripherals configured) avoids
 *                       the HardFaults people hit when jumping mid-application.
 *
 * System-memory base address is device-specific (AN2606, "STM32 microcontroller
 * system memory boot mode").  For STM32H56x/H57x it is 0x0BF97000 — note this is
 * NOT the STM32H503 value (0x0BF87000).  Getting it wrong only means the software
 * path no-ops into a fault; the hardware BOOT0 pin still reaches the bootloader.
 */
#include "dfu_boot.h"
#include "stm32h5xx_hal.h"

/* STM32H563/H573 system memory (ROM bootloader) base — see AN2606. */
#define SYSMEM_BOOT_ADDR 0x0BF97000UL

/* Arbitrary sentinel unlikely to appear in uninitialized RAM by accident. */
#define DFU_BOOT_MAGIC 0xB007DF00UL

/*
 * No-init flag, preserved across the NVIC_SystemReset() warm reset.  The
 * `.noinit` (NOLOAD) output section is added in the linker script; startup.s
 * only touches `.data`/`.bss`, so this cell keeps its value through reset.
 * `volatile` keeps the compiler from assuming its value across the reset.
 */
static volatile uint32_t dfu_boot_flag __attribute__((section(".noinit")));

void dfu_boot_request(void)
{
    dfu_boot_flag = DFU_BOOT_MAGIC;
    __DSB();
    NVIC_SystemReset();
    for (;;) { } /* unreachable */
}

void dfu_boot_check(void)
{
    if (dfu_boot_flag != DFU_BOOT_MAGIC) {
        return;
    }
    dfu_boot_flag = 0; /* one-shot: don't loop back into DFU on the next reset */

    /* Classic Cortex-M system-memory jump.  TrustZone is disabled on this port
       (TZEN=0, single non-secure image), so no SAU/secure handling is needed. */
    void (*sysmem_jump)(void) =
        (void (*)(void))(*(volatile uint32_t *)(SYSMEM_BOOT_ADDR + 4U));

    __disable_irq();

    /* Quiet SysTick (HAL_Init() hasn't run yet, but be defensive). */
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;

    /* Point the vector table at system memory and load its initial stack. */
    SCB->VTOR = SYSMEM_BOOT_ADDR;
    __set_MSP(*(volatile uint32_t *)SYSMEM_BOOT_ADDR);

    __DSB();
    __ISB();
    __enable_irq();

    sysmem_jump();
    for (;;) { } /* unreachable */
}
