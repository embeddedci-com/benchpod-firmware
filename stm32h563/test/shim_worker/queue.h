/* Host stand-in for FreeRTOS queue.h (test_hw_worker): a bounded FIFO the test implements. */
#ifndef SHIM_WORKER_QUEUE_H
#define SHIM_WORKER_QUEUE_H
#include "FreeRTOS.h"
typedef struct shim_queue *QueueHandle_t;
QueueHandle_t xQueueCreate(UBaseType_t depth, UBaseType_t item_size);
BaseType_t    xQueueSend(QueueHandle_t q, const void *item, TickType_t wait);
BaseType_t    xQueueReceive(QueueHandle_t q, void *item, TickType_t wait);
UBaseType_t   uxQueueMessagesWaiting(QueueHandle_t q);
UBaseType_t   uxQueueSpacesAvailable(QueueHandle_t q);
#endif
