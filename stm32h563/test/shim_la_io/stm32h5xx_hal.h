/* Host stand-in for the STM32 HAL header: what la_io.c uses (test_la_io.c). */
#ifndef STM32H5XX_HAL_SHIM_LA_IO_H
#define STM32H5XX_HAL_SHIM_LA_IO_H
#include <stdint.h>
typedef struct { int dummy; } GPIO_TypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;
typedef struct { uint32_t Pin, Mode, Pull, Speed, Alternate; } GPIO_InitTypeDef;
#define GPIO_MODE_INPUT       0u
#define GPIO_MODE_OUTPUT_PP   1u
#define GPIO_NOPULL           0u
#define GPIO_SPEED_FREQ_LOW   0u
#define GPIO_PIN_3            (1u << 3)
#define GPIO_PIN_4            (1u << 4)
extern GPIO_TypeDef shim_gpiog;
#define GPIOG (&shim_gpiog)
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint32_t pin, GPIO_PinState s);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint32_t pin);
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init);
void HAL_Delay(uint32_t ms);
#endif
