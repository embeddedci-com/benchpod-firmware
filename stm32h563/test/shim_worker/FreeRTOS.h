/* Host stand-in for the FreeRTOS kernel header, for test_hw_worker: the types and macros
   hw_worker.c uses. The queue itself is faked in the test (queue.h below declares it). */
#ifndef SHIM_WORKER_FREERTOS_H
#define SHIM_WORKER_FREERTOS_H

#include <stddef.h>
#include <stdint.h>

typedef uint32_t TickType_t;
typedef long     BaseType_t;
typedef unsigned long UBaseType_t;
#define pdTRUE  1
#define pdFALSE 0
#define pdPASS  1
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
#define tskIDLE_PRIORITY 0

size_t xPortGetFreeHeapSize(void);
size_t xPortGetMinimumEverFreeHeapSize(void);

#endif
