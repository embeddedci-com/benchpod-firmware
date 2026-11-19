/* Host stand-in for FreeRTOS task.h (test_hw_worker): one thread, a critical section is a no-op. */
#ifndef SHIM_WORKER_TASK_H
#define SHIM_WORKER_TASK_H
#include "FreeRTOS.h"
#define taskENTER_CRITICAL() do { } while (0)
#define taskEXIT_CRITICAL()  do { } while (0)
typedef void (*TaskFunction_t)(void *);
BaseType_t xTaskCreate(TaskFunction_t fn, const char *name, uint32_t stack, void *arg,
                       UBaseType_t prio, void *handle);
void vTaskDelay(TickType_t ticks);
typedef void *TaskHandle_t;
#define taskSCHEDULER_NOT_STARTED 1
#define taskSCHEDULER_RUNNING     2
static inline BaseType_t   xTaskGetSchedulerState(void)    { return taskSCHEDULER_RUNNING; }
static inline TaskHandle_t xTaskGetCurrentTaskHandle(void) { return NULL; }
#endif
