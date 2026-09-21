/* boot_guard.c: see boot_guard.h. */
#include "boot_guard.h"
#include "boot_policy.h"
#include "fault.h"
#include "watchdog.h"
#include "stm32h5xx_hal.h"
#include <stdint.h>
#include <stdio.h>

#define BOOT_GUARD_MAGIC 0x42475232u   /* "BGR2": .noinit is random after power-on */

typedef struct {
    uint32_t magic;
    uint32_t failed;   /* boots in a row that never reached boot_guard_healthy() */
    uint32_t stage;    /* the step the current boot is in */
    uint32_t last_stage;
    uint32_t started;  /* BOOT_SUB_* whose bring-up began this boot */
    uint32_t done;     /* BOOT_SUB_* whose bring-up finished this boot */
    uint32_t off;      /* BOOT_SUB_* this boot turned off (safe mode) */
} boot_record_t;

static volatile boot_record_t s_rec __attribute__((section(".noinit")));
static bool s_safe;
static uint32_t s_off;
static volatile bool s_hw_ready;
static char s_report[128];

static const char *const s_names[BOOT_STAGE_COUNT] = {
    [BOOT_STAGE_NONE]      = "start",
    [BOOT_STAGE_EARLY_HW]  = "early hardware",
    [BOOT_STAGE_IDENTITY]  = "device identity",
    [BOOT_STAGE_USB]       = "usb",
    [BOOT_STAGE_SCHEDULER] = "starting tasks",
    [BOOT_STAGE_NET_INIT]  = "network",
    [BOOT_STAGE_HW_INIT]   = "ice40/psram",
    [BOOT_STAGE_RUNNING]   = "running",
};

#define SAFE_PREFIX "safe mode: "

static void boot_test_loop_end(void);

void boot_guard_begin(bool fresh_start) {
    uint32_t failed = 0, last = BOOT_STAGE_NONE, started = 0, done = 0, prev_off = 0;
    if (!fresh_start && s_rec.magic == BOOT_GUARD_MAGIC) {
        failed = s_rec.failed;
        last = s_rec.stage < BOOT_STAGE_COUNT ? s_rec.stage : BOOT_STAGE_NONE;
        started = s_rec.started;
        done = s_rec.done;
        prev_off = s_rec.off;
    }

    uint32_t culprit = boot_policy_culprit(started, done, fault_last_crash_task());
    if (failed >= BOOT_GUARD_SAFE_AFTER) {
        s_safe = true;
        s_off = boot_policy_off(prev_off, culprit);
        /* Name where it failed: the culprit if known, else the stage it stopped in. */
        const char *where = culprit == BOOT_SUB_NET ? s_names[BOOT_STAGE_NET_INIT]
                          : culprit == BOOT_SUB_HW  ? s_names[BOOT_STAGE_HW_INIT]
                          : s_names[last];
        snprintf(s_report, sizeof(s_report),
                 SAFE_PREFIX "%lu failed boots in a row, the last in \"%s\"; %s off",
                 (unsigned long)failed, where, boot_policy_off_str(s_off));
        printf("[boot] SAFE MODE: %s\r\n", s_report + sizeof(SAFE_PREFIX) - 1);
        boot_test_loop_end();   /* a test loop has done its job */
    }

    /* This boot counts as failed until boot_guard_healthy() says otherwise. */
    s_rec.magic = BOOT_GUARD_MAGIC;
    s_rec.failed = failed + 1;
    s_rec.last_stage = last;
    s_rec.stage = BOOT_STAGE_NONE;
    s_rec.started = 0;
    s_rec.done = 0;
    s_rec.off = s_off;
    __DSB();
    watchdog_arm_early();
}

void boot_guard_stage(boot_stage_t stage) {
    s_rec.stage = (uint32_t)stage;
    __DSB();
    watchdog_early_refresh();
}

void boot_guard_sub_start(uint32_t sub) {
    s_rec.started |= sub;
    __DSB();
}

void boot_guard_sub_done(uint32_t sub) {
    s_rec.done |= sub;
    __DSB();
}

bool boot_guard_safe_mode(void) { return s_safe; }
bool boot_guard_skip_net(void)  { return (s_off & BOOT_SUB_NET) != 0; }
bool boot_guard_skip_hw(void)   { return (s_off & BOOT_SUB_HW) != 0; }

void boot_guard_healthy(void) {
    /* A safe-mode boot staying up proves nothing about a normal one: keep counting, so the
       next software reset stays in safe mode (a power-on or reset button starts over). */
    if (s_safe) return;
    s_rec.failed = 0;
    s_rec.stage = BOOT_STAGE_RUNNING;
    __DSB();
}

const char *boot_guard_report(void) { return s_report; }
const char *boot_guard_reason(void) {
    return s_report[0] ? s_report + sizeof(SAFE_PREFIX) - 1 : "";
}

bool boot_hw_ready(void) { return s_hw_ready; }

/* Called by boot_deferred_hw_init() when it is done. */
void boot_guard_set_hw_ready(void) { s_hw_ready = true; }

/* ---- deliberate boot-loop test (console: test-bootloop) ----------------------------------- */
#define BOOT_TEST_MAGIC 0x544c4f4fu   /* "TLOO" */
static volatile uint32_t s_test_magic __attribute__((section(".noinit")));
static volatile uint32_t s_test_left __attribute__((section(".noinit")));
static volatile uint32_t s_test_sub __attribute__((section(".noinit")));

static void boot_test_loop_end(void) {
    s_test_magic = 0;
    __DSB();
}

void boot_guard_arm_test_loop(uint32_t boots, uint32_t sub) {
    s_test_left = boots;
    s_test_sub = sub;
    s_test_magic = BOOT_TEST_MAGIC;
    __DSB();
}

void boot_guard_test_loop_point(uint32_t sub) {
    if (s_test_magic != BOOT_TEST_MAGIC || s_test_sub != sub) return;
    if (s_safe || s_test_left == 0) {   /* safe mode reached (or done): the test is over */
        boot_test_loop_end();
        return;
    }
    s_test_left--;
    __DSB();
    printf("[boot] test-bootloop: crashing on purpose (%lu more)\r\n", (unsigned long)s_test_left);
    __builtin_trap();   /* a real fault, recorded like any other */
}
