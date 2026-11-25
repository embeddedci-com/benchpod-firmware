/*
 * uart_clk.c - the UART kernel clock without the HAL's generic clock-tree walker.
 *
 * HAL_UART_Init (UART_SetConfig) asks HAL_RCCEx_GetPeriphCLKFreq for the kernel clock. That
 * function knows every clock source of every peripheral and pulls in HAL_RCCEx_GetPLLxClockFreq
 * (about 6 KB of flash) for the three UARTs' PLL sources we never select. The link wraps it
 * (-Wl,--wrap=HAL_RCCEx_GetPeriphCLKFreq): the UARTs we initialize through the HAL are USART2
 * (console) and USART1 (ESP32-C3 ROM flasher), both on their PCLK kernel clock, which is exactly
 * what the HAL returns for that mux setting (HAL_RCC_GetPCLK1Freq / HAL_RCC_GetPCLK2Freq).
 * Any other instance or mux setting is a bug: Error_Handler, not a silent BRR of 0.
 *
 * Nothing else in the firmware calls HAL_RCCEx_GetPeriphCLKFreq (check with
 * `arm-none-eabi-nm build/bench_pod_stm32.elf | grep GetPeriphCLKFreq`: must be absent).
 */
#include "stm32h5xx_hal.h"

void Error_Handler(void);

uint32_t __wrap_HAL_RCCEx_GetPeriphCLKFreq(uint64_t PeriphClk);

uint32_t __wrap_HAL_RCCEx_GetPeriphCLKFreq(uint64_t PeriphClk)
{
    if (PeriphClk == RCC_PERIPHCLK_USART2 &&
        __HAL_RCC_GET_USART2_SOURCE() == RCC_USART2CLKSOURCE_PCLK1) {
        return HAL_RCC_GetPCLK1Freq();
    }
    if (PeriphClk == RCC_PERIPHCLK_USART1 &&
        __HAL_RCC_GET_USART1_SOURCE() == RCC_USART1CLKSOURCE_PCLK2) {
        return HAL_RCC_GetPCLK2Freq();
    }
    Error_Handler();
    return 0;
}
