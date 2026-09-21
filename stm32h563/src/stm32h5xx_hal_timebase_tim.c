/*
 * stm32h5xx_hal_timebase_tim.c — HAL timebase from the DWT cycle counter.
 *
 * NOTE: despite the file name (kept so the Makefile is unchanged) this is a
 * TIM-free timebase.  The previous implementation drove HAL_GetTick() from a
 * TIM6 update interrupt, but on this board uwTick stayed stuck at 0 — TIM6 was
 * configured, started and NVIC-enabled, yet its update IRQ never reached
 * HAL_IncTick() at runtime.  That froze every HAL timeout: e.g. the iCE40 probe
 * in signal_engine_init() sat forever in HAL_SPI_Transmit() waiting for EOT with
 * a "100 ms" timeout that could never elapse (HAL_GetTick()-Tickstart == 0),
 * wedging the whole boot before USB init.
 *
 * Coupling the firmware-wide time source to a single IRQ landing correctly is
 * fragile — the wrong tool for a FreeRTOS system, where SysTick already belongs
 * to the kernel.  Instead we read the free-running DWT cycle counter directly.
 * It always counts (already proven on this board via pico_compat busy_wait_us),
 * needs no interrupt, and cannot be defeated by any NVIC/VTOR/priority issue, so
 * HAL_Delay()/HAL timeouts are always monotone and always terminate.
 */
#include "stm32h5xx_hal.h"

static void dwt_cyccnt_enable(void)
{
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0U;
        DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
    }
}

HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
    uwTickPrio = TickPriority;
    dwt_cyccnt_enable();
    return HAL_OK;
}

/*
 * HAL_GetTick — milliseconds since boot, from CYCCNT.
 *
 * Cycles are accumulated (not divided in place) so the count stays correct
 * across the 32-bit CYCCNT rollover (~17 s at 250 MHz): the unsigned delta
 * (now - last_cyc) is exact for any elapsed span < 2^32 cycles, and HAL's own
 * timeout loops call this far more often than every 17 s.  The short critical
 * section keeps the static accumulator consistent if called from both thread
 * and ISR context.
 */
uint32_t HAL_GetTick(void)
{
    static uint32_t last_cyc = 0U;
    static uint32_t cyc_rem  = 0U;   /* sub-ms cycle remainder */

    uint32_t cyc_per_ms = SystemCoreClock / 1000U;
    if (cyc_per_ms == 0U) {
        return uwTick;               /* SystemCoreClock not established yet */
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint32_t now = DWT->CYCCNT;
    cyc_rem += (now - last_cyc);      /* wrap-safe unsigned delta */
    last_cyc = now;
    uwTick  += cyc_rem / cyc_per_ms;
    cyc_rem %= cyc_per_ms;
    uint32_t tick = uwTick;

    if (primask == 0U) {
        __enable_irq();
    }
    return tick;
}

/* No timer to suspend/resume; the cycle counter runs continuously. */
void HAL_SuspendTick(void) { }
void HAL_ResumeTick(void)  { }
