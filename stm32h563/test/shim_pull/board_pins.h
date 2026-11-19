/* Host stand-in for bsp/board_pins.h: the pull expanders' reset line only. */
#ifndef BOARD_PINS_SHIM_PULL_H
#define BOARD_PINS_SHIM_PULL_H
#include "stm32h5xx_hal.h"
#define PULL_RST_PORT GPIOG
#define PULL_RST_PIN  GPIO_PIN_7
#endif
