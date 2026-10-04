#include "sys_health.h"
#include "FreeRTOS.h"
#include "task.h"

/* TaskStatus_t is ~40 B each; static so a caller with little stack (the console task) can ask.
   Filled and read with the scheduler suspended, so two callers cannot interleave. */
static TaskStatus_t s_status[SYS_HEALTH_MAX_TASKS];

int sys_health_tasks(sys_health_task_t *out, int max) {
    int n = 0;
    vTaskSuspendAll();
    /* Returns 0 when there are more tasks than slots: raise SYS_HEALTH_MAX_TASKS then. */
    UBaseType_t got = uxTaskGetSystemState(s_status, SYS_HEALTH_MAX_TASKS, NULL);
    /* The kernel lists tasks by state, which changes; sort by task number (creation order). */
    for (UBaseType_t i = 1; i < got; i++)
        for (UBaseType_t j = i; j > 0 && s_status[j - 1].xTaskNumber > s_status[j].xTaskNumber; j--) {
            TaskStatus_t t = s_status[j]; s_status[j] = s_status[j - 1]; s_status[j - 1] = t;
        }
    for (UBaseType_t i = 0; i < got && n < max; i++, n++) {
        out[n].name       = s_status[i].pcTaskName;
        out[n].stack_free = (unsigned)s_status[i].usStackHighWaterMark * sizeof(StackType_t);
    }
    (void)xTaskResumeAll();
    return n;
}

unsigned sys_health_stack_min_free(void) {
    sys_health_task_t t[SYS_HEALTH_MAX_TASKS];
    int n = sys_health_tasks(t, SYS_HEALTH_MAX_TASKS);
    unsigned min = (unsigned)-1;
    for (int i = 0; i < n; i++)
        if (t[i].stack_free < min) min = t[i].stack_free;
    return (n == 0) ? 0 : min;
}

unsigned sys_health_heap_free(void)     { return (unsigned)xPortGetFreeHeapSize(); }
unsigned sys_health_heap_min_free(void) { return (unsigned)xPortGetMinimumEverFreeHeapSize(); }
