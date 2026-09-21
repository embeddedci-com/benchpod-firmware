#include "sys_health.h"
#include "FreeRTOS.h"
#include "task.h"

static struct {
    const char   *name;
    TaskHandle_t  handle;
} s_tasks[SYS_HEALTH_MAX_TASKS];
static int s_count;

void sys_health_register(const char *name, void *task_handle) {
    if (s_count >= SYS_HEALTH_MAX_TASKS || !task_handle) return;
    s_tasks[s_count].name   = name;
    s_tasks[s_count].handle = (TaskHandle_t)task_handle;
    s_count++;
}

int sys_health_task_count(void) { return s_count; }

const char *sys_health_task_name(int i) {
    return (i >= 0 && i < s_count) ? s_tasks[i].name : "?";
}

unsigned sys_health_task_stack_free(int i) {
    if (i < 0 || i >= s_count) return 0;
    /* uxTaskGetStackHighWaterMark returns the minimum free stack in words. */
    return (unsigned)uxTaskGetStackHighWaterMark(s_tasks[i].handle) * sizeof(StackType_t);
}

unsigned sys_health_stack_min_free(void) {
    unsigned min = (unsigned)-1;
    for (int i = 0; i < s_count; i++) {
        unsigned f = sys_health_task_stack_free(i);
        if (f < min) min = f;
    }
    return (s_count == 0) ? 0 : min;
}

unsigned sys_health_heap_free(void)     { return (unsigned)xPortGetFreeHeapSize(); }
unsigned sys_health_heap_min_free(void) { return (unsigned)xPortGetMinimumEverFreeHeapSize(); }
