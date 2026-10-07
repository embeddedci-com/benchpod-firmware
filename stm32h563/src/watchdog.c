#include "watchdog.h"
#include "fault.h"
#include "stm32h5xx_hal.h"
#include "bp_log.h"

#include <stdio.h>

/* IWDG clock = LSI (~32 kHz) / prescaler.  /256 -> 125 Hz; reload 3750 -> ~30 s
   hardware timeout.  This comfortably exceeds any single legitimate operation
   (TLS handshake, PSRAM/LA capture, a flash write burst) while still bounding a
   true wedge to ~30 s from the last refresh. */
#define IWDG_PRESCALER_VAL  IWDG_PRESCALER_256
#define IWDG_RELOAD_VAL     3750u
/* Early warning: the IWDG interrupt fires when the counter reaches this value, 125 ticks
   (~1 s) before the reset, so iwdg_ewi_capture() can record where the pod hung. */
#define IWDG_EWI_VAL        125u

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
/* HAL_GetTick() of each task's last heartbeat and of the last IWDG refresh, read by the
   early-warning interrupt to name the task that stalled (the net task that services the IWDG
   may be the one that hung).  The interrupt does not read the clock itself: HAL_GetTick counts
   CPU cycles and loses ~17 s when nothing calls it for that long, which is exactly what a hung
   net task causes.  The early warning fires a fixed time after the last refresh instead. */
static volatile uint32_t  s_hb_ms[WD_TASK_COUNT];
static volatile uint32_t  s_refresh_ms;
#define IWDG_EWI_AFTER_MS   ((IWDG_RELOAD_VAL - IWDG_EWI_VAL) * 1000u / 125u)
static const char *const  s_names[WD_TASK_COUNT] = {
    [WD_TASK_NET]     = "net",
    [WD_TASK_CONSOLE] = "console",
    [WD_TASK_WORKER]  = "hw",
};

/* The early-warning interrupt runs above FreeRTOS's syscall priority, so it fires even when a
   task is spinning in a critical section; it calls no RTOS API, it only reads. */
static void iwdg_ewi_enable(void) {
    NVIC_SetPriority(IWDG_IRQn, 0);
    NVIC_EnableIRQ(IWDG_IRQn);
}

void watchdog_init(void) {
    s_iwdg.Instance       = IWDG;
    s_iwdg.Init.Prescaler = IWDG_PRESCALER_VAL;
    s_iwdg.Init.Reload    = IWDG_RELOAD_VAL;
    s_iwdg.Init.Window    = IWDG_WINDOW_DISABLE;   /* no lower-bound window */
    s_iwdg.Init.EWI       = IWDG_EWI_VAL;
    /* Seed the liveness bookkeeping so the first watchdog_service() has a sane
       baseline instead of instantly declaring every task hung. */
    uint32_t now = HAL_GetTick();
    for (int t = 0; t < WD_TASK_COUNT; t++) {
        s_last_hb[t] = s_hb[t];
        s_last_change_ms[t] = now;
        s_hb_ms[t] = now;
    }
    s_refresh_ms = now;
    iwdg_ewi_enable();
    if (HAL_IWDG_Init(&s_iwdg) != HAL_OK) {
        log_printf("[wdg] IWDG init failed — running WITHOUT a watchdog\n");
        return;
    }
    s_started = 1;
    log_printf("[wdg] IWDG armed (~%lu ms)\n",
           (unsigned long)((IWDG_RELOAD_VAL + 1) * 256u * 1000u / 32000u));
}

void watchdog_arm_early(void) {
    s_iwdg.Instance       = IWDG;
    s_iwdg.Init.Prescaler = IWDG_PRESCALER_VAL;
    s_iwdg.Init.Reload    = IWDG_RELOAD_VAL;
    s_iwdg.Init.Window    = IWDG_WINDOW_DISABLE;
    s_iwdg.Init.EWI       = IWDG_EWI_VAL;
    iwdg_ewi_enable();
    if (HAL_IWDG_Init(&s_iwdg) != HAL_OK)
        log_printf("[wdg] early IWDG arm failed\n");
}

void watchdog_early_refresh(void) {
    if (s_iwdg.Instance == IWDG) (void)HAL_IWDG_Refresh(&s_iwdg);
}

void watchdog_heartbeat(int task, const char *task_name) {
    if (task < 0 || task >= WD_TASK_COUNT) return;
    s_hb[task]++;
    s_hb_ms[task] = HAL_GetTick();
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
    if (healthy) {
        HAL_IWDG_Refresh(&s_iwdg);
        s_refresh_ms = now;
    }
    /* else: stop refreshing — the IWDG expires and resets the pod. fault.c will
       report "iwdg" as the reset cause on the next boot. */
}

/* IWDG early warning (see IWDG_EWI_VAL): record which task stalled and where the CPU was, then
   let the reset happen.  Before watchdog_init() (early boot) no task is tracked yet. */
void iwdg_ewi_capture(uint32_t *frame);
void iwdg_ewi_capture(uint32_t *frame) {
    uint32_t stall[WD_TASK_COUNT] = {0};
    if (s_started) {
        uint32_t now = s_refresh_ms + IWDG_EWI_AFTER_MS;
        for (int t = 0; t < WD_TASK_COUNT; t++) {
            uint32_t since = now - s_hb_ms[t];
            if (since > s_grace_ms[t]) stall[t] = since;
        }
    }
    fault_note_watchdog(frame, stall, s_names, WD_TASK_COUNT);
    IWDG->EWCR |= IWDG_EWCR_EWIC;   /* acknowledge; the reset follows regardless */
}

/* Pick MSP vs PSP from EXC_RETURN bit 2 and pass the stacked frame, as the fault handlers do. */
__attribute__((naked)) void IWDG_IRQHandler(void) {
    __asm volatile(
        "tst lr, #4            \n"
        "ite eq               \n"
        "mrseq r0, msp        \n"
        "mrsne r0, psp        \n"
        "b    iwdg_ewi_capture \n");
}

static volatile int s_test_hang = -1;

void watchdog_test_hang(int task) { s_test_hang = task; }

void watchdog_test_hang_point(int task) {
    if (s_test_hang != task) return;
    log_printf("[wdg] test-hang: %s task spinning on purpose\r\n", s_names[task]);
    for (;;) { __NOP(); }
}
