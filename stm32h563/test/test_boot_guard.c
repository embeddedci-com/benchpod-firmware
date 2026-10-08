/*
 * test_boot_guard.c — host tests for the failed-boot counter and safe mode (src/boot_guard.c):
 * counting across resets, the threshold, what a safe boot turns off and reports, "healthy"
 * clearing the count (but not in safe mode), a power-on starting over, and the test-bootloop.
 *
 * boot_guard.c is included so a "reset" can clear its RAM state while its .noinit record
 * survives, as on the chip. The section attribute becomes `unused` here (Mach-O has no .noinit).
 */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#define section(name) unused
#include "../src/boot_guard.c"
#undef section

static int failures;
#define CHECK(cond, ...) do { if (!(cond)) { printf("FAIL %s:%d: ", __func__, __LINE__); \
        printf(__VA_ARGS__); printf("\n"); failures++; } } while (0)

/* ---- fakes ---------------------------------------------------------------------------- */
static const char *s_crash_task = "";
static int s_wd_armed, s_wd_refresh;
const char *fault_last_crash_task(void) { return s_crash_task; }
void watchdog_arm_early(void) { s_wd_armed++; }
void watchdog_early_refresh(void) { s_wd_refresh++; }

/* A reset: RAM state is gone, the .noinit record stays (unless power was lost). */
static void reset(bool power_on, const char *crash_task) {
    s_safe = false;
    s_off = 0;
    s_hw_ready = false;
    s_report[0] = '\0';
    s_crash_task = crash_task;
    if (power_on) memset((void *)&s_rec, 0x5A, sizeof(s_rec));   /* random after power-on */
    boot_guard_begin(power_on);
}

/* A boot that dies in `sub`'s bring-up (watchdog: no crash task) or crashes in a task. */
static void boot_dies_in(uint32_t sub) {
    boot_guard_stage(sub == BOOT_SUB_NET ? BOOT_STAGE_NET_INIT : BOOT_STAGE_HW_INIT);
    boot_guard_sub_start(sub);
}

static void test_counts_to_safe_mode(void) {
    reset(true, "");
    CHECK(!boot_guard_safe_mode() && s_rec.failed == 1, "power-on: safe %d failed %u",
          boot_guard_safe_mode(), (unsigned)s_rec.failed);
    CHECK(s_wd_armed > 0, "watchdog not armed");
    CHECK(boot_guard_report()[0] == '\0' && boot_guard_reason()[0] == '\0', "report on a good boot");
    boot_dies_in(BOOT_SUB_HW);

    reset(false, "");                       /* failure 1 seen: still a normal boot */
    CHECK(!boot_guard_safe_mode() && s_rec.failed == 2, "1 failure: safe %d", boot_guard_safe_mode());
    CHECK(s_rec.last_stage == BOOT_STAGE_HW_INIT, "last stage %u", (unsigned)s_rec.last_stage);
    boot_dies_in(BOOT_SUB_HW);

    reset(false, "");                       /* BOOT_GUARD_SAFE_AFTER failures: safe mode */
    CHECK(boot_guard_safe_mode(), "no safe mode after %d failures", BOOT_GUARD_SAFE_AFTER);
    CHECK(boot_guard_skip_hw() && !boot_guard_skip_net(), "a hang in hw bring-up turns off hw only");
    CHECK(strcmp(boot_guard_reason(),
                 "2 failed boots in a row, the last in \"ice40/psram\"; iCE40/PSRAM off") == 0,
          "reason: %s", boot_guard_reason());
    CHECK(strncmp(boot_guard_report(), "safe mode: ", 11) == 0 &&
          strcmp(boot_guard_report() + 11, boot_guard_reason()) == 0, "report: %s", boot_guard_report());
    CHECK(s_rec.off == BOOT_SUB_HW, "record off %u", (unsigned)s_rec.off);

    /* a safe boot staying up does not clear the count: the next software reset is safe too */
    boot_guard_healthy();
    CHECK(s_rec.failed == 3, "healthy cleared the count in safe mode: %u", (unsigned)s_rec.failed);
    reset(false, "");
    CHECK(boot_guard_safe_mode() && boot_guard_skip_hw(), "lost safe mode after a software reset");

    /* a power-on (or reset button) starts over, whatever the record held */
    reset(true, "");
    CHECK(!boot_guard_safe_mode() && s_rec.failed == 1, "power-on kept safe mode");
}

static void test_healthy_clears(void) {
    reset(true, "");
    boot_dies_in(BOOT_SUB_NET);
    reset(false, "");
    CHECK(s_rec.failed == 2, "failed %u", (unsigned)s_rec.failed);
    boot_guard_stage(BOOT_STAGE_HW_INIT);
    boot_guard_healthy();
    CHECK(s_rec.failed == 0 && s_rec.stage == BOOT_STAGE_RUNNING, "healthy: failed %u stage %u",
          (unsigned)s_rec.failed, (unsigned)s_rec.stage);
    reset(false, "");                       /* a later crash starts counting from one */
    CHECK(!boot_guard_safe_mode() && s_rec.failed == 1, "after healthy: failed %u", (unsigned)s_rec.failed);
    CHECK(s_wd_refresh > 0, "stage did not refresh the early watchdog");
}

static void test_culprits(void) {
    /* a crash in the net task names the network, whatever bring-up was running */
    reset(true, "");
    boot_dies_in(BOOT_SUB_HW);
    reset(false, "net");
    boot_dies_in(BOOT_SUB_HW);
    reset(false, "net");
    CHECK(boot_guard_skip_net() && !boot_guard_skip_hw(), "net crash: off %u", (unsigned)s_off);
    CHECK(strstr(boot_guard_reason(), "the last in \"network\"; network off") != NULL, "reason: %s",
          boot_guard_reason());

    /* nothing identified (a fault before either bring-up): both off, named by the stage */
    reset(true, "");
    boot_guard_stage(BOOT_STAGE_USB);
    reset(false, "");
    boot_guard_stage(BOOT_STAGE_USB);
    reset(false, "");
    CHECK(boot_guard_skip_net() && boot_guard_skip_hw(), "unknown culprit: off %u", (unsigned)s_off);
    CHECK(strstr(boot_guard_reason(), "the last in \"usb\"; network and iCE40/PSRAM off") != NULL,
          "reason: %s", boot_guard_reason());

    /* safe boots keep what earlier ones turned off and add a second culprit */
    reset(true, "");
    boot_dies_in(BOOT_SUB_HW);
    reset(false, "");
    boot_dies_in(BOOT_SUB_HW);
    reset(false, "");                       /* safe: hw off */
    CHECK(boot_guard_skip_hw() && !boot_guard_skip_net(), "first culprit");
    boot_dies_in(BOOT_SUB_NET);             /* now the network hangs too */
    reset(false, "");
    CHECK(boot_guard_skip_hw() && boot_guard_skip_net(), "second culprit not added: off %u",
          (unsigned)s_off);

    /* a bring-up that finished is not the culprit */
    reset(true, "");
    boot_guard_sub_start(BOOT_SUB_NET); boot_guard_sub_done(BOOT_SUB_NET);
    boot_dies_in(BOOT_SUB_HW);
    reset(false, "");
    boot_guard_sub_start(BOOT_SUB_NET); boot_guard_sub_done(BOOT_SUB_NET);
    boot_dies_in(BOOT_SUB_HW);
    reset(false, "");
    CHECK(boot_guard_skip_hw() && !boot_guard_skip_net(), "finished net blamed: off %u", (unsigned)s_off);

    /* an out-of-range stage in the record reads as "start" */
    reset(true, "");
    s_rec.stage = 99;
    reset(false, "");
    s_rec.stage = 99;
    reset(false, "");
    CHECK(strstr(boot_guard_reason(), "the last in \"start\"") != NULL, "reason: %s", boot_guard_reason());
}

static void test_test_loop_ends_in_safe_mode(void) {
    reset(true, "");
    boot_guard_arm_test_loop(5, BOOT_SUB_HW);
    CHECK(s_test_magic == BOOT_TEST_MAGIC && s_test_left == 5, "not armed");
    /* the point for another subsystem does nothing */
    boot_guard_test_loop_point(BOOT_SUB_NET);
    CHECK(s_test_left == 5, "wrong subsystem counted");
    /* a safe-mode boot ends the test (boot_guard_begin clears it) */
    boot_dies_in(BOOT_SUB_HW);
    reset(false, "hw");
    boot_dies_in(BOOT_SUB_HW);
    reset(false, "hw");
    CHECK(boot_guard_safe_mode() && s_test_magic == 0, "test loop survived safe mode");
    boot_guard_test_loop_point(BOOT_SUB_HW);   /* disarmed: returns, no trap */

    /* zero boots left: the point disarms instead of crashing */
    reset(true, "");
    boot_guard_arm_test_loop(0, BOOT_SUB_NET);
    boot_guard_test_loop_point(BOOT_SUB_NET);
    CHECK(s_test_magic == 0, "zero-boot loop still armed");
}

static void test_hw_ready(void) {
    reset(true, "");
    CHECK(!boot_hw_ready(), "ready before bring-up");
    boot_guard_set_hw_ready();
    CHECK(boot_hw_ready(), "not ready after bring-up");
}

int main(void) {
    test_counts_to_safe_mode();
    test_healthy_clears();
    test_culprits();
    test_test_loop_ends_in_safe_mode();
    test_hw_ready();
    if (failures) { printf("test_boot_guard: %d FAILED\n", failures); return 1; }
    printf("test_boot_guard: all passed\n");
    return 0;
}
