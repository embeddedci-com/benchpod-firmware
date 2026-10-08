/* Host stand-in for the STM32 HAL header, for the command_handler module tests: the tick and the
   reset the modules call (the tests define them). */
#ifndef STM32H5XX_HAL_SHIM_CH_H
#define STM32H5XX_HAL_SHIM_CH_H
#include <stdint.h>
uint32_t HAL_GetTick(void);
void NVIC_SystemReset(void);
#endif
