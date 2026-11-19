/* Host stand-in for the STM32 HAL header: what pull_exp.c uses (test_pull_exp_state.c). */
#ifndef STM32H5XX_HAL_SHIM_PULL_H
#define STM32H5XX_HAL_SHIM_PULL_H
#include <stdint.h>
typedef struct { int dummy; } GPIO_TypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;
typedef struct { uint32_t Pin, Mode, Pull, Speed, Alternate; } GPIO_InitTypeDef;
#define GPIO_MODE_OUTPUT_PP   1u
#define GPIO_NOPULL           0u
#define GPIO_SPEED_FREQ_LOW   0u
#define GPIO_PIN_7            (1u << 7)
#define __HAL_RCC_GPIOG_CLK_ENABLE() ((void)0)
extern GPIO_TypeDef shim_gpiog;
#define GPIOG (&shim_gpiog)
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint32_t pin, GPIO_PinState s);
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init);
void HAL_Delay(uint32_t ms);
#endif
