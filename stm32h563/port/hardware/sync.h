/* Pico SDK <hardware/sync.h> shim -> Cortex-M PRIMASK critical section. */
#ifndef HW_SYNC_SHIM_H
#define HW_SYNC_SHIM_H
#include <stdint.h>
uint32_t save_and_disable_interrupts(void);
void     restore_interrupts(uint32_t status);
#endif
