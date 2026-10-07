/* Host stand-in for FreeRTOS task.h: one thread, so a critical section is a no-op. */
#ifndef SHIM_TASK_H
#define SHIM_TASK_H

#define taskENTER_CRITICAL() do { } while (0)
#define taskEXIT_CRITICAL()  do { } while (0)

#endif /* SHIM_TASK_H */
