#ifndef FAULT_H
#define FAULT_H

#include <stdint.h>

/*
 * fault — turn every hard failure into a recorded reset instead of a silent hang.
 *
 * The core fault handlers (HardFault/MemManage/BusFault/UsageFault/SecureFault/
 * NMI) and the FreeRTOS malloc-failed / stack-overflow / assert hooks used to end
 * in `while(1)` with interrupts off — a headless pod that wedged there needed a
 * physical power-cycle and left no clue why.  Instead they now capture the
 * faulting context (stacked PC/LR/PSR + the SCB fault status registers, or a
 * software reason) into a .noinit RAM record that survives the reset, then
 * NVIC_SystemReset().  At the next boot fault_boot_init() reads the record and the
 * RCC reset-cause flags and formats one-line summaries surfaced on the console
 * `status` / `selftest` output, so a crash leaves a breadcrumb across the reboot.
 */

/* Reason codes stored in the crash record. 0..15 are hardware exceptions; the
   0x2x range is software-detected fatal conditions. */
enum {
    FAULT_NONE          = 0,
    FAULT_HARDFAULT     = 1,
    FAULT_MEMMANAGE     = 2,
    FAULT_BUSFAULT      = 3,
    FAULT_USAGEFAULT    = 4,
    FAULT_SECUREFAULT   = 5,
    FAULT_NMI           = 6,
    FAULT_SW_MALLOC     = 0x20,
    FAULT_SW_STACKOVF   = 0x21,
    FAULT_SW_ASSERT     = 0x22,
    FAULT_SW_TASKCREATE = 0x23,
    FAULT_SW_ERRHANDLER = 0x24,   /* HAL/clock Error_Handler reached (early or runtime) */
};

/* Read the RCC reset-cause flags and any surviving crash record, cache their
   human-readable forms, then clear the RCC flags.  Call once, very early in boot
   (before the record could be overwritten).  Safe to call before the scheduler. */
void fault_boot_init(void);

/* One-line reset cause, e.g. "iwdg" / "software" / "pin" / "power-on". */
const char *fault_last_reset_str(void);

/* One-line summary of the last crash record, or "none" if the last reset was not
   a recorded fault.  e.g. "HardFault pc=0x08001234 lr=0x0800abcd cfsr=0x00020000 task=net". */
const char *fault_last_crash_str(void);

/* The task the last crash happened in ("net", "hw", "boot", ...), or "" if the last reset
   was not a recorded fault. */
const char *fault_last_crash_task(void);

/* True if the last reset was caused by the independent watchdog. */
int fault_was_watchdog(void);

/* Record a software-detected fatal condition (`reason` is a FAULT_SW_*) with an
   optional task/context name, then reset.  Does not return.  Used by the FreeRTOS
   hooks and vAssertCalled. */
void fault_sw_panic(uint32_t reason, const char *name) __attribute__((noreturn));

/* Set by each task's watchdog heartbeat so a fault can be attributed to the task
   that was running.  Kept tiny and lock-free (a single pointer store). */
void fault_set_task_hint(const char *name);

#endif /* FAULT_H */
