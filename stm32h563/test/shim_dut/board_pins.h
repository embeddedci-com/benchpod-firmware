/* Host stand-in for bsp/board_pins.h: the J1 DUT supply enables only (test_dut_supply.c). */
#ifndef BOARD_PINS_SHIM_DUT_H
#define BOARD_PINS_SHIM_DUT_H
#include "stm32h5xx_hal.h"
#define DUT_3V3_EN_PORT GPIOE
#define DUT_3V3_EN_PIN  GPIO_PIN_4
#define DUT_1V8_EN_PORT GPIOE
#define DUT_1V8_EN_PIN  GPIO_PIN_3
#define DUT_EN_CLK_EN() __HAL_RCC_GPIOE_CLK_ENABLE()
#endif
