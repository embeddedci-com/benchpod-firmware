#ifndef BOOT_GUARD_H
#define BOOT_GUARD_H

#include <stdbool.h>
#include <stdint.h>

/* ============================================================================
 * boot_guard: a pod must always come back with a USB console, whatever breaks.
 *
 * Boot is ordered so USB and the scheduler come up before any risky hardware (the iCE40
 * link, PSRAM, gateware load); those run later on the hw worker task (boot_deferred_hw_init).
 * On top of that, a failed-boot counter lives in .noinit RAM, which survives watchdog and
 * software resets:
 *   - boot_guard_begin() counts this boot and arms the IWDG;
 *   - boot_guard_stage() records which step is running (for the report);
 *   - boot_guard_healthy() clears the counter once the pod has run for a while.
 * A crash loop (hang -> watchdog, fault -> software reset) never reaches "healthy", so after
 * BOOT_GUARD_SAFE_AFTER failed boots in a row the next one runs in SAFE MODE: no networking,
 * no iCE40/PSRAM bring-up, just USB + the console, and `status` says why.  A power-on or a
 * reset-button press starts the count again.
 * ==========================================================================*/

typedef enum {
    BOOT_STAGE_NONE = 0,
    BOOT_STAGE_EARLY_HW,     /* power bus, target power, analog switches, CAN */
    BOOT_STAGE_IDENTITY,
    BOOT_STAGE_USB,
    BOOT_STAGE_SCHEDULER,
    BOOT_STAGE_NET_INIT,
    BOOT_STAGE_HW_INIT,      /* iCE40 link, PSRAM, self-test, gateware load */
    BOOT_STAGE_RUNNING,
    BOOT_STAGE_COUNT
} boot_stage_t;

#define BOOT_GUARD_SAFE_AFTER 2   /* failed boots in a row before safe mode */

/* After fault_boot_init(): fresh_start is true for a power-on or reset-button reset. */
void boot_guard_begin(bool fresh_start);
void boot_guard_stage(boot_stage_t stage);
bool boot_guard_safe_mode(void);
/* The pod has been up long enough: this boot counts as good. */
void boot_guard_healthy(void);
/* "" normally; in safe mode a one-line reason for `status`. */
const char *boot_guard_report(void);

/* Test the safeguard itself: make the next `boots` boots crash in the net task (as the
   byte-wise UID read did), so safe mode must engage.  Console: test-bootloop. */
void boot_guard_arm_test_loop(uint32_t boots);
/* Called by the net task after the safe-mode check: crashes while a test loop is armed. */
void boot_guard_test_loop_point(void);

/* Deferred hardware bring-up (iCE40, PSRAM, self-test), run once on the hw worker task. */
void boot_deferred_hw_init(void);
/* True once boot_deferred_hw_init() has finished (never in safe mode). */
bool boot_hw_ready(void);
void boot_guard_set_hw_ready(void);

#endif /* BOOT_GUARD_H */
