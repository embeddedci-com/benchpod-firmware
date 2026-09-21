/*
 * stm32h5xx_it.c — Cortex-M33 core exception handlers.
 *
 * NOTE: SVC_Handler, PendSV_Handler and SysTick_Handler are intentionally NOT
 * defined here — the FreeRTOS ARM_CM33_NTZ port defines them directly (with the
 * CMSIS names) and owns those vectors.  The HAL timebase lives on TIM6
 * (TIM6_IRQHandler is in stm32h5xx_hal_timebase_tim.c).
 *
 * The fault vectors (NMI/HardFault/MemManage/BusFault/UsageFault/SecureFault) are
 * defined in fault.c: they snapshot the faulting context into a .noinit crash
 * record and reset, instead of hanging forever in a `while(1)`.
 */
#include "stm32h5xx_hal.h"

void DebugMon_Handler(void)
{
}
