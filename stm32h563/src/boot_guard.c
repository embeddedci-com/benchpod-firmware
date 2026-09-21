/* boot_guard.c: see boot_guard.h. */
#include "boot_guard.h"
#include "watchdog.h"
#include "stm32h5xx_hal.h"
#include <stdint.h>
#include <stdio.h>

#define BOOT_GUARD_MAGIC 0x42475244u   /* "BGRD": .noinit is random after power-on */

typedef struct {
    uint32_t magic;
    uint32_t failed;   /* boots in a row that never reached boot_guard_healthy() */
    uint32_t stage;    /* the step the current boot is in */
    uint32_t last_stage;
} boot_record_t;

static volatile boot_record_t s_rec __attribute__((section(".noinit")));
static bool s_safe;
static volatile bool s_hw_ready;
static char s_report[96];

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

void boot_guard_begin(bool fresh_start) {
    uint32_t failed = 0, last = BOOT_STAGE_NONE;
    if (!fresh_start && s_rec.magic == BOOT_GUARD_MAGIC) {
        failed = s_rec.failed;
        last = s_rec.stage < BOOT_STAGE_COUNT ? s_rec.stage : BOOT_STAGE_NONE;
    }
    /* This boot counts as failed until boot_guard_healthy() says otherwise. */
    s_rec.magic = BOOT_GUARD_MAGIC;
    s_rec.failed = failed + 1;
    s_rec.last_stage = last;
    s_rec.stage = BOOT_STAGE_NONE;
    __DSB();

    if (failed >= BOOT_GUARD_SAFE_AFTER) {
        s_safe = true;
        snprintf(s_report, sizeof(s_report),
                 "safe mode: %lu failed boots in a row, the last one stopped in \"%s\"",
                 (unsigned long)failed, s_names[last]);
        printf("[boot] SAFE MODE: %lu failed boots in a row (last stopped in \"%s\"): "
               "USB console only, no network, no iCE40/PSRAM\r\n",
               (unsigned long)failed, s_names[last]);
    }
    watchdog_arm_early();
}

void boot_guard_stage(boot_stage_t stage) {
    s_rec.stage = (uint32_t)stage;
    __DSB();
    watchdog_early_refresh();
}

bool boot_guard_safe_mode(void) { return s_safe; }

void boot_guard_healthy(void) {
    /* A safe-mode boot staying up proves nothing about a normal one: keep counting, so the
       next software reset stays in safe mode (a power-on or reset button starts over). */
    if (s_safe) return;
    s_rec.failed = 0;
    s_rec.stage = BOOT_STAGE_RUNNING;
    __DSB();
}

const char *boot_guard_report(void) { return s_report; }

bool boot_hw_ready(void) { return s_hw_ready; }

/* Called by boot_deferred_hw_init() when it is done. */
void boot_guard_set_hw_ready(void) { s_hw_ready = true; }

/* ---- deliberate boot-loop test (console: test-bootloop) ----------------------------------- */
#define BOOT_TEST_MAGIC 0x544c4f4fu   /* "TLOO" */
static volatile uint32_t s_test_magic __attribute__((section(".noinit")));
static volatile uint32_t s_test_left __attribute__((section(".noinit")));

void boot_guard_arm_test_loop(uint32_t boots) {
    s_test_left = boots;
    s_test_magic = BOOT_TEST_MAGIC;
    __DSB();
}

void boot_guard_test_loop_point(void) {
    if (s_test_magic != BOOT_TEST_MAGIC) return;
    if (s_safe || s_test_left == 0) {   /* safe mode reached (or done): the test is over */
        s_test_magic = 0;
        return;
    }
    s_test_left--;
    __DSB();
    printf("[boot] test-bootloop: crashing on purpose (%lu more)\r\n", (unsigned long)s_test_left);
    __builtin_trap();   /* a real fault, recorded like any other */
}
