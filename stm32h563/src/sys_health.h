#ifndef SYS_HEALTH_H
#define SYS_HEALTH_H

#include <stddef.h>

/*
 * sys_health — runtime memory-headroom telemetry.
 *
 * Surfaces each task's minimum-ever free stack (uxTaskGetStackHighWaterMark) and
 * the FreeRTOS heap free / low-water, so stack or heap exhaustion is visible on
 * `status` / `selftest` BEFORE it faults — instead of only being caught
 * destructively by the stack-overflow hook.  main.c registers the task handles
 * once, right after creating them.
 */

/* Register a task's handle under a short name (copied by pointer — pass a string
   literal).  Up to SYS_HEALTH_MAX_TASKS registrations; extras are ignored. */
#define SYS_HEALTH_MAX_TASKS 4
void sys_health_register(const char *name, void *task_handle);

int         sys_health_task_count(void);
const char *sys_health_task_name(int i);
/* Minimum free stack this task has ever had, in bytes (0 if i is out of range). */
unsigned    sys_health_task_stack_free(int i);
/* Smallest stack headroom across all registered tasks, in bytes. */
unsigned    sys_health_stack_min_free(void);

unsigned    sys_health_heap_free(void);        /* current free heap, bytes     */
unsigned    sys_health_heap_min_free(void);    /* lowest-ever free heap, bytes  */

#endif /* SYS_HEALTH_H */
