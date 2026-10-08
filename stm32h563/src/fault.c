#include "fault.h"
#include "rng.h"
#include "board_uid.h"
#include "stm32h5xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

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
    /* FAULT_WATCHDOG only: ms each watchdog task had gone without a heartbeat (0: alive), and
       its name (a string literal in flash, so the pointer survives the reset). */
    uint32_t stall_ms[FAULT_WDG_TASKS];
    const char *stall_name[FAULT_WDG_TASKS];
} fault_record_t;

/* Unclean-reset count, also in .noinit: valid only while magic matches (random at power-on). */
#define UNCLEAN_MAGIC 0x55C1EA4Eu
typedef struct {
    uint32_t magic;
    uint32_t count;
} unclean_record_t;
static unclean_record_t s_unclean __attribute__((section(".noinit")));

/* .noinit: not loaded from flash, not zeroed by startup, so it carries the crash
   details across the NVIC_SystemReset the fault handler issues. */
static fault_record_t s_rec __attribute__((section(".noinit")));

/* Name of the task that last proved liveness — attributed to a crash. Written by
   fault_set_task_hint() (lock-free single-pointer store). */
static const char *volatile s_task_hint;

static char s_reset_str[24] = "unknown";
static char s_crash_str[160] = "none";
static char s_crash_task[16];
static int  s_was_iwdg;

void fault_set_task_hint(const char *name) { s_task_hint = name; }

/* "0x%08lx" into a static buffer (the crash summary is built once, at boot). */
static const char *addr_str(uint32_t v) {
    static char b[12];
    snprintf(b, sizeof(b), "0x%08lx", (unsigned long)v);
    return b;
}

/* Copy the crashing task's name into the record (fault context: no allocation, bounded).
   It is the task that was RUNNING (FreeRTOS's current task), not the last one to send a
   watchdog heartbeat: the hint named "net" for a fault in the hw worker during the v3
   bring-up.  Before the scheduler runs there is no task: "boot".  The hint is the fallback. */
static void record_task_name(void) {
    const char *n = NULL;
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
        n = "boot";
    } else {
        TaskHandle_t cur = xTaskGetCurrentTaskHandle();   /* a plain read, safe here */
        if (cur) n = pcTaskGetName(cur);
    }
    if (!n) n = s_task_hint;
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
FAULT_TRAMPOLINE(NMI_Fault,           FAULT_NMI)

/* An NMI is also how the H5 reports a flash ECC double error.  The persistence stores read
   through flash_read_checked(), which marks the read as a probe: then the NMI is that probe's
   torn record (a power cut mid-save), which the store rejects, not a fault.  Anything else goes
   to the normal fault path with its exception frame untouched (push/pop restore SP and LR, and
   pop keeps the flags from cmp). */
__attribute__((naked)) void NMI_Handler(void)
{
    __asm volatile(
        "push {r0, lr}              \n"
        "bl   flash_ecc_nmi_absorb  \n"
        "cmp  r0, #0                \n"
        "pop  {r0, lr}              \n"
        "it   ne                    \n"
        "bxne lr                    \n"
        "b    NMI_Fault             \n");
}

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

void fault_note_watchdog(const uint32_t *frame, const uint32_t *stall_ms,
                         const char *const *names, int n) {
    s_rec.magic  = FAULT_MAGIC;
    s_rec.reason = FAULT_WATCHDOG;
    s_rec.r0  = frame[0]; s_rec.r1  = frame[1]; s_rec.r2 = frame[2];
    s_rec.r3  = frame[3]; s_rec.r12 = frame[4]; s_rec.lr = frame[5];
    s_rec.pc  = frame[6]; s_rec.psr = frame[7];
    s_rec.cfsr = 0; s_rec.hfsr = 0; s_rec.mmfar = 0; s_rec.bfar = 0;
    for (int i = 0; i < FAULT_WDG_TASKS; i++) {
        s_rec.stall_ms[i]   = (i < n) ? stall_ms[i] : 0;
        s_rec.stall_name[i] = (i < n) ? names[i] : NULL;
    }
    record_task_name();
    __DSB();
}

uint32_t fault_boot_id(void) {
    static uint32_t id;
    if (id == 0) {
        uint32_t v = 0;
        if (rng_get32(&v) != 0 || v == 0) {
            uint32_t uid[3];
            board_uid_words(uid);
            v = HAL_GetTick() ^ uid[0] ^ uid[1] ^ uid[2] ^ 0x9E3779B9u;
        }
        id = v ? v : 1u;
    }
    return id;
}

uint32_t fault_unclean_resets(void) {
    return s_unclean.magic == UNCLEAN_MAGIC ? s_unclean.count : 0;
}

void fault_unclean_resets_ack(void) {
    s_unclean.magic = UNCLEAN_MAGIC;
    s_unclean.count = 0;
}

/* "watchdog: hw stalled 61.2 s, ... ; running net pc=... lr=..." from a FAULT_WATCHDOG record. */
static void format_watchdog(void) {
    size_t off = (size_t)snprintf(s_crash_str, sizeof(s_crash_str), "watchdog:");
    const char *first = NULL;
    for (int i = 0; i < FAULT_WDG_TASKS && off < sizeof(s_crash_str); i++) {
        if (!s_rec.stall_ms[i] || !s_rec.stall_name[i]) continue;
        off += (size_t)snprintf(s_crash_str + off, sizeof(s_crash_str) - off, "%s %s stalled %lu.%lu s",
                                first ? "," : "", s_rec.stall_name[i],
                                (unsigned long)(s_rec.stall_ms[i] / 1000u),
                                (unsigned long)(s_rec.stall_ms[i] % 1000u / 100u));
        if (!first) first = s_rec.stall_name[i];
    }
    if (!first && off < sizeof(s_crash_str))
        off += (size_t)snprintf(s_crash_str + off, sizeof(s_crash_str) - off, " no task stalled");
    if (off < sizeof(s_crash_str))
        snprintf(s_crash_str + off, sizeof(s_crash_str) - off, "; running %s pc=0x%08lx lr=0x%08lx",
                 s_rec.task[0] ? s_rec.task : "?", (unsigned long)s_rec.pc, (unsigned long)s_rec.lr);
    /* The stalled task is the culprit for boot_guard, not whichever task the CPU was in. */
    const char *t = first ? first : s_rec.task;
    size_t i = 0;
    for (; t[i] && i < sizeof(s_crash_task) - 1; i++) s_crash_task[i] = t[i];
    s_crash_task[i] = '\0';
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

    /* A watchdog record whose reset never came (the task recovered within the last second)
       is stale: it describes no reset. */
    if (s_rec.magic == FAULT_MAGIC && s_rec.reason == FAULT_WATCHDOG && !s_was_iwdg)
        s_rec.magic = 0;

    if (s_rec.magic == FAULT_MAGIC && s_rec.reason == FAULT_WATCHDOG) {
        format_watchdog();
        s_rec.magic = 0;
    } else if (s_rec.magic == FAULT_MAGIC) {
        snprintf(s_crash_str, sizeof(s_crash_str),
                 "%s pc=0x%08lx lr=0x%08lx cfsr=0x%08lx hfsr=0x%08lx%s%s task=%s",
                 reason_name(s_rec.reason),
                 (unsigned long)s_rec.pc, (unsigned long)s_rec.lr,
                 (unsigned long)s_rec.cfsr, (unsigned long)s_rec.hfsr,
                 /* BFAR/MMFAR hold an address only when CFSR says so; otherwise they are
                    stale and would point the reader at a bogus location. */
                 (s_rec.cfsr & SCB_CFSR_BFARVALID_Msk) ? " bfar=" : "",
                 (s_rec.cfsr & SCB_CFSR_BFARVALID_Msk) ? addr_str(s_rec.bfar) : "",
                 s_rec.task[0] ? s_rec.task : "?");
        memcpy(s_crash_task, s_rec.task, sizeof(s_crash_task));
        s_crash_task[sizeof(s_crash_task) - 1] = '\0';
        /* Leave the record in place so the summary persists across subsequent
           clean reboots until the next crash overwrites it, but clear the magic
           so a stale record isn't re-attributed to a future unrelated reset. */
        s_rec.magic = 0;
    } else if (s_was_iwdg) {
        /* The early warning never ran: interrupts were off, or the hang was before the
           watchdog's interrupt was set up. */
        strncpy(s_crash_str, "watchdog: no early warning (interrupts off?)", sizeof(s_crash_str) - 1);
    } else {
        strncpy(s_crash_str, "none", sizeof(s_crash_str) - 1);
    }

    /* Count unclean resets until the server has heard of them. A power cut starts over. */
    int unclean = strcmp(s_crash_str, "none") != 0 || strcmp(cause, "wwdg") == 0;
    if (strcmp(cause, "power-on") == 0 || s_unclean.magic != UNCLEAN_MAGIC) {
        s_unclean.magic = UNCLEAN_MAGIC;
        s_unclean.count = 0;
    }
    if (unclean) s_unclean.count++;
}

const char *fault_last_reset_str(void) { return s_reset_str; }
const char *fault_last_crash_str(void) { return s_crash_str; }
const char *fault_last_crash_task(void) { return s_crash_task; }
int         fault_was_watchdog(void)   { return s_was_iwdg; }
