/* Host stand-in for the STM32 HAL header: what dut_rails.c uses (test_dut_rails.c). */
#ifndef STM32H5XX_HAL_SHIM_DUT_H
#define STM32H5XX_HAL_SHIM_DUT_H
#include <stdint.h>
typedef struct { int dummy; } GPIO_TypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;
typedef struct { uint32_t Pin, Mode, Pull, Speed, Alternate; } GPIO_InitTypeDef;
#define GPIO_MODE_OUTPUT_PP   1u
#define GPIO_NOPULL           0u
#define GPIO_SPEED_FREQ_LOW   0u
#define GPIO_PIN_3            (1u << 3)
#define GPIO_PIN_4            (1u << 4)
#define __HAL_RCC_GPIOD_CLK_ENABLE() ((void)0)
extern GPIO_TypeDef shim_gpiod;
#define GPIOD (&shim_gpiod)
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint32_t pin, GPIO_PinState s);
uint32_t HAL_GetTick(void);
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init);
#endif
