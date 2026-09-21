#ifndef WATCHDOG_H
#define WATCHDOG_H

/*
 * watchdog — IWDG-backed liveness guard for the two application tasks.
 *
 * The independent watchdog resets the pod if the firmware wedges, so a hang no
 * longer needs a physical power-cycle (the reset cause + last crash are reported
 * on the next boot via fault.c).  The IWDG is only refreshed while BOTH tasks
 * keep proving liveness through watchdog_heartbeat(); if either stops, refreshing
 * stops and the IWDG expires.
 *
 * The two tasks are held to different grace windows on purpose: the net task must
 * cycle briskly (it services lwIP + the cloud link), while the console task may
 * legitimately block for many seconds in a single iteration during an iCE40 /
 * ESP32 flash, so it is given a much longer window before it is considered hung.
 */
#include <stdint.h>

enum {
    WD_TASK_NET = 0,     /* net task — short grace (must service lwIP briskly)   */
    WD_TASK_CONSOLE,     /* console task — short grace (line editing only)       */
    WD_TASK_WORKER,      /* hw worker — long grace (runs multi-second flash ops) */
    WD_TASK_COUNT
};

/* Configure and start the IWDG.  Call once before the scheduler starts (so a
   scheduler that never starts still trips the watchdog).  After this the IWDG is
   running and MUST be serviced within its timeout or the pod resets. */
void watchdog_init(void);

/* Called by each task at the top of its loop to prove it is alive.  Also updates
   the fault task hint so a crash is attributed to the right task. */
void watchdog_heartbeat(int task, const char *task_name);

/* Called from the net task loop.  Refreshes the IWDG only if every task has
   proven liveness within its grace window; otherwise it lets the IWDG expire. */
void watchdog_service(void);

#endif /* WATCHDOG_H */
