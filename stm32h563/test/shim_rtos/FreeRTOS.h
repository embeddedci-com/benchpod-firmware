/* Host stand-in for the FreeRTOS kernel header: the heap calls the firmware uses, on malloc, with
   an injectable out-of-memory (shim_rtos_fail_after: the Nth allocation from now fails; -1 = off). */
#ifndef SHIM_FREERTOS_H
#define SHIM_FREERTOS_H

#include <stdlib.h>

extern int shim_rtos_fail_after;

static inline void *pvPortMalloc(size_t n) {
    if (shim_rtos_fail_after == 0) { shim_rtos_fail_after = -1; return NULL; }
    if (shim_rtos_fail_after > 0) shim_rtos_fail_after--;
    return malloc(n);
}
static inline void vPortFree(void *p) { free(p); }

#endif /* SHIM_FREERTOS_H */
