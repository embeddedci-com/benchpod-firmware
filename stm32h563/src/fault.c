#include "fault.h"
#include "stm32h5xx_hal.h"

#include <stdio.h>
#include <string.h>

/* ---- crash record (survives reset in .noinit) ----------------------------- */

#define FAULT_MAGIC 0xFA017EC0u

typedef struct {
    uint32_t magic;
    uint32_t reason;
    uint32_t r0, r1, r2, r3, r12, lr, pc, psr;
    uint32_t cfsr, hfsr, mmfar, bfar;
    char     task[16];
} fault_record_t;

/* .noinit: not loaded from flash, not zeroed by startup, so it carries the crash
   details across the NVIC_SystemReset the fault handler issues. */
static fault_record_t s_rec __attribute__((section(".noinit")));

/* Name of the task that last proved liveness — attributed to a crash. Written by
   fault_set_task_hint() (lock-free single-pointer store). */
static const char *volatile s_task_hint;

static char s_reset_str[24] = "unknown";
static char s_crash_str[96] = "none";
static int  s_was_iwdg;

void fault_set_task_hint(const char *name) { s_task_hint = name; }

/* Copy the task hint into the record (fault context — no allocation, bounded). */
static void record_task_name(void) {
    const char *n = s_task_hint;
    if (!n) { s_rec.task[0] = '\0'; return; }
    size_t i = 0;
    for (; n[i] && i < sizeof(s_rec.task) - 1; i++) s_rec.task[i] = n[i];
    s_rec.task[i] = '\0';
}

/* Common tail: snapshot the stacked frame + SCB fault status, then reset.
   `frame` points at the 8-word exception stack frame (r0,r1,r2,r3,r12,lr,pc,psr). */
void fault_capture(uint32_t *frame, uint32_t exc_return, uint32_t reason);
void fault_capture(uint32_t *frame, uint32_t exc_return, uint32_t reason) {
    (void)exc_return;
    s_rec.magic  = FAULT_MAGIC;
    s_rec.reason = reason;
    s_rec.r0  = frame[0]; s_rec.r1  = frame[1]; s_rec.r2 = frame[2];
    s_rec.r3  = frame[3]; s_rec.r12 = frame[4]; s_rec.lr = frame[5];
    s_rec.pc  = frame[6]; s_rec.psr = frame[7];
    s_rec.cfsr  = SCB->CFSR;
    s_rec.hfsr  = SCB->HFSR;
    s_rec.mmfar = SCB->MMFAR;
    s_rec.bfar  = SCB->BFAR;
    record_task_name();
    __DSB();
    NVIC_SystemReset();
    for (;;) { }   /* unreachable */
}

/* Naked trampoline: pick MSP vs PSP from EXC_RETURN bit 2, pass the frame + a
   reason immediate to fault_capture.  One per exception vector. */
#define FAULT_TRAMPOLINE(name, reason)                       \
    __attribute__((naked)) void name(void) {                 \
        __asm volatile(                                      \
            "tst lr, #4            \n"                        \
            "ite eq               \n"                        \
            "mrseq r0, msp        \n"                        \
            "mrsne r0, psp        \n"                        \
            "mov  r1, lr          \n"                        \
            "movs r2, %[rs]       \n"                        \
            "b    fault_capture   \n"                        \
            :: [rs] "I" (reason) : "r0", "r1", "r2", "memory"); \
    }

FAULT_TRAMPOLINE(HardFault_Handler,   FAULT_HARDFAULT)
FAULT_TRAMPOLINE(MemManage_Handler,   FAULT_MEMMANAGE)
FAULT_TRAMPOLINE(BusFault_Handler,    FAULT_BUSFAULT)
FAULT_TRAMPOLINE(UsageFault_Handler,  FAULT_USAGEFAULT)
FAULT_TRAMPOLINE(SecureFault_Handler, FAULT_SECUREFAULT)
FAULT_TRAMPOLINE(NMI_Handler,         FAULT_NMI)

void fault_sw_panic(uint32_t reason, const char *name) {
    if (name) fault_set_task_hint(name);
    s_rec.magic  = FAULT_MAGIC;
    s_rec.reason = reason;
    /* No exception frame for a software panic; capture the caller's PC/LR so the
       record still points near the failure. */
    s_rec.pc  = (uint32_t)__builtin_return_address(0);
    s_rec.lr  = (uint32_t)__builtin_return_address(0);
    s_rec.psr = 0;
    s_rec.cfsr = 0; s_rec.hfsr = 0; s_rec.mmfar = 0; s_rec.bfar = 0;
    record_task_name();
    __DSB();
    NVIC_SystemReset();
    for (;;) { }
}

/* ---- boot-time decode ----------------------------------------------------- */

static const char *reason_name(uint32_t r) {
    switch (r) {
        case FAULT_HARDFAULT:     return "HardFault";
        case FAULT_MEMMANAGE:     return "MemManage";
        case FAULT_BUSFAULT:      return "BusFault";
        case FAULT_USAGEFAULT:    return "UsageFault";
        case FAULT_SECUREFAULT:   return "SecureFault";
        case FAULT_NMI:           return "NMI";
        case FAULT_SW_MALLOC:     return "malloc-failed";
        case FAULT_SW_STACKOVF:   return "stack-overflow";
        case FAULT_SW_ASSERT:     return "assert";
        case FAULT_SW_TASKCREATE: return "task-create-failed";
        case FAULT_SW_ERRHANDLER: return "error-handler";
        default:                  return "fault";
    }
}

void fault_boot_init(void) {
    /* Reset cause from RCC (checked most-specific first). */
    const char *cause = "unknown";
    if      (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) { cause = "iwdg";      s_was_iwdg = 1; }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST))   cause = "wwdg";
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))    cause = "software";
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST))    cause = "power-on";
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST))    cause = "pin";
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST))   cause = "low-power";
    strncpy(s_reset_str, cause, sizeof(s_reset_str) - 1);
    __HAL_RCC_CLEAR_RESET_FLAGS();

    if (s_rec.magic == FAULT_MAGIC) {
        snprintf(s_crash_str, sizeof(s_crash_str),
                 "%s pc=0x%08lx lr=0x%08lx cfsr=0x%08lx task=%s",
                 reason_name(s_rec.reason),
                 (unsigned long)s_rec.pc, (unsigned long)s_rec.lr,
                 (unsigned long)s_rec.cfsr,
                 s_rec.task[0] ? s_rec.task : "?");
        /* Leave the record in place so the summary persists across subsequent
           clean reboots until the next crash overwrites it, but clear the magic
           so a stale record isn't re-attributed to a future unrelated reset. */
        s_rec.magic = 0;
    } else {
        strncpy(s_crash_str, "none", sizeof(s_crash_str) - 1);
    }
}

const char *fault_last_reset_str(void) { return s_reset_str; }
const char *fault_last_crash_str(void) { return s_crash_str; }
int         fault_was_watchdog(void)   { return s_was_iwdg; }
