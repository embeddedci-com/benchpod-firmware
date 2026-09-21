#include "watchdog.h"
#include "fault.h"
#include "stm32h5xx_hal.h"

#include <stdio.h>

/* IWDG clock = LSI (~32 kHz) / prescaler.  /256 -> 125 Hz; reload 3750 -> ~30 s
   hardware timeout.  This comfortably exceeds any single legitimate operation
   (TLS handshake, PSRAM/LA capture, a flash write burst) while still bounding a
   true wedge to ~30 s from the last refresh. */
#define IWDG_PRESCALER_VAL  IWDG_PRESCALER_256
#define IWDG_RELOAD_VAL     3750u

/* Per-task grace windows: how long a task may go without a heartbeat before it is
   considered hung and refreshing stops.  The net task and the (now IO-only)
   console task must stay brisk; the hw worker may block for seconds in a single
   iteration during an iCE40/ESP32 flash, so it gets a long window. */
static const uint32_t s_grace_ms[WD_TASK_COUNT] = {
    [WD_TASK_NET]     = 8000u,
    [WD_TASK_CONSOLE] = 8000u,
    [WD_TASK_WORKER]  = 60000u,
};

static IWDG_HandleTypeDef s_iwdg;
static volatile uint32_t  s_hb[WD_TASK_COUNT];   /* bumped by heartbeat        */
static uint32_t           s_last_hb[WD_TASK_COUNT];
static uint32_t           s_last_change_ms[WD_TASK_COUNT];
static int                s_started;

void watchdog_init(void) {
    s_iwdg.Instance       = IWDG;
    s_iwdg.Init.Prescaler = IWDG_PRESCALER_VAL;
    s_iwdg.Init.Reload    = IWDG_RELOAD_VAL;
    s_iwdg.Init.Window    = IWDG_WINDOW_DISABLE;   /* no lower-bound window */
    /* Seed the liveness bookkeeping so the first watchdog_service() has a sane
       baseline instead of instantly declaring every task hung. */
    uint32_t now = HAL_GetTick();
    for (int t = 0; t < WD_TASK_COUNT; t++) {
        s_last_hb[t] = s_hb[t];
        s_last_change_ms[t] = now;
    }
    if (HAL_IWDG_Init(&s_iwdg) != HAL_OK) {
        printf("[wdg] IWDG init failed — running WITHOUT a watchdog\n");
        return;
    }
    s_started = 1;
    printf("[wdg] IWDG armed (~%lu ms)\n",
           (unsigned long)((IWDG_RELOAD_VAL + 1) * 256u * 1000u / 32000u));
}

void watchdog_heartbeat(int task, const char *task_name) {
    if (task < 0 || task >= WD_TASK_COUNT) return;
    s_hb[task]++;
    fault_set_task_hint(task_name);
}

void watchdog_service(void) {
    if (!s_started) return;
    uint32_t now = HAL_GetTick();
    int healthy = 1;
    for (int t = 0; t < WD_TASK_COUNT; t++) {
        uint32_t hb = s_hb[t];
        if (hb != s_last_hb[t]) {          /* task advanced since last check */
            s_last_hb[t] = hb;
            s_last_change_ms[t] = now;
        } else if (now - s_last_change_ms[t] > s_grace_ms[t]) {
            healthy = 0;                    /* stalled past its grace window  */
        }
    }
    if (healthy) HAL_IWDG_Refresh(&s_iwdg);
    /* else: stop refreshing — the IWDG expires and resets the pod. fault.c will
       report "iwdg" as the reset cause on the next boot. */
}
