#ifndef SYS_HEALTH_H
#define SYS_HEALTH_H

#include <stddef.h>

/*
 * sys_health — runtime memory-headroom telemetry.
 *
 * Surfaces each task's minimum-ever free stack (uxTaskGetStackHighWaterMark) and
 * the FreeRTOS heap free / low-water, so stack or heap exhaustion is visible on
 * `status` / `selftest` BEFORE it faults — instead of only being caught
 * destructively by the stack-overflow hook.  Every task is listed, including the
 * FreeRTOS idle and timer tasks, so nothing needs registering.
 */

#define SYS_HEALTH_MAX_TASKS 8

typedef struct {
    const char *name;        /* the task's own name (lives in its TCB; tasks are never deleted) */
    unsigned    stack_free;  /* minimum free stack it has ever had, bytes */
} sys_health_task_t;

/* Fill out[] with up to max tasks, in creation order. Returns how many. Safe from any task;
   uses no more than a few bytes of the caller's stack. */
int         sys_health_tasks(sys_health_task_t *out, int max);
/* Smallest stack headroom across all tasks, in bytes. */
unsigned    sys_health_stack_min_free(void);

unsigned    sys_health_heap_free(void);        /* current free heap, bytes     */
unsigned    sys_health_heap_min_free(void);    /* lowest-ever free heap, bytes  */

#endif /* SYS_HEALTH_H */
