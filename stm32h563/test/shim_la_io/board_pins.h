/* Host stand-in for bsp/board_pins.h: the LA bank mux pins only (test_la_io.c). */
#ifndef BOARD_PINS_SHIM_LA_IO_H
#define BOARD_PINS_SHIM_LA_IO_H
#include "stm32h5xx_hal.h"
#define LA_VCCIO_SW_PORT  GPIOG
#define LA_VCCIO_SW_PIN   GPIO_PIN_3
#define LA_VCCIO_ST_PORT  GPIOG
#define LA_VCCIO_ST_PIN   GPIO_PIN_4
#endif
