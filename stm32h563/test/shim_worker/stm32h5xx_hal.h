/* Host stand-in for the STM32 HAL header (test_hw_worker). */
#ifndef SHIM_WORKER_HAL_H
#define SHIM_WORKER_HAL_H
#include <stdint.h>
uint32_t HAL_GetTick(void);
#endif
