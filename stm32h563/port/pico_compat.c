/*
 * pico_compat.c — Pico SDK time/delay primitives on STM32H5.
 *
 * TIM2 (32-bit, APB1) runs free at 1 MHz as the microsecond counter; its update
 * interrupt extends it to 64 bits.  The DWT cycle counter backs busy_wait_us().
 */
#include "pico_compat.h"
#include "stm32h5xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

static TIM_HandleTypeDef htim2;
static volatile uint32_t s_us_hi;          /* high 32 bits of the µs counter */

void pico_compat_init(void)
{
    /* DWT cycle counter for fine busy-waits. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* TIM2 @ 1 MHz, free-running 32-bit, update IRQ extends to 64-bit. */
    __HAL_RCC_TIM2_CLK_ENABLE();
    uint32_t timclk = HAL_RCC_GetPCLK1Freq();   /* APB1 prescaler = 1 -> timer clk = PCLK1 */
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = (timclk / 1000000U) - 1U;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xFFFFFFFFU;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim2);

    /* The overflow IRQ only bumps a counter (no FreeRTOS API) — high priority. */
    HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    HAL_TIM_Base_Start_IT(&htim2);
}

uint64_t time_us_64(void)
{
    uint32_t hi, lo;
    /* Re-read to guard against a wrap landing between the two reads. */
    do {
        hi = s_us_hi;
        lo = TIM2->CNT;
    } while (hi != s_us_hi);
    return ((uint64_t)hi << 32) | lo;
}

uint32_t time_us_32(void) { return TIM2->CNT; }

uint32_t to_ms_since_boot(absolute_time_t t) { return (uint32_t)(t / 1000ull); }

void busy_wait_us(uint64_t us)
{
    uint32_t start = DWT->CYCCNT;
    /* SystemCoreClock counts/sec -> counts per microsecond. */
    uint32_t cycles = (uint32_t)((SystemCoreClock / 1000000U) * (uint32_t)us);
    while ((DWT->CYCCNT - start) < cycles) {
        __asm volatile("nop");
    }
}

void sleep_us(uint64_t us)
{
    if (us >= 2000ull && xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        vTaskDelay(pdMS_TO_TICKS((uint32_t)(us / 1000ull)));
    } else {
        busy_wait_us(us);
    }
}

void sleep_ms(uint32_t ms)
{
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        vTaskDelay(pdMS_TO_TICKS(ms));
    } else {
        HAL_Delay(ms);
    }
}

/* TIM2 overflow: extend the µs counter to 64 bits.  Handled inline (we don't
   route TIM2 through HAL_TIM_IRQHandler) so it stays independent of the TIM6
   HAL timebase callback. */
void TIM2_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) &&
        __HAL_TIM_GET_IT_SOURCE(&htim2, TIM_IT_UPDATE)) {
        __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
        s_us_hi++;
    }
}
