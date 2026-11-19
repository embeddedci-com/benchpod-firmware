/*
 * bp_log.c — see bp_log.h.
 */
#include "bp_log.h"

#include <stdio.h>
#include <string.h>

#include "console_io.h"

/* FreeRTOS heap_4 (declared here so this file also builds in the host tests). */
void *pvPortMalloc(size_t size);
void  vPortFree(void *p);
uint32_t HAL_GetTick(void);

/* boot_guard.c has the real one; the host tests link without it. */
__attribute__((weak)) bool boot_hw_ready(void) { return false; }

bp_log_level_t log_boot_level(void) { return boot_hw_ready() ? BP_LOG_DEBUG : BP_LOG_INFO; }

/* Format and write one message, ungated. */
static int write_v(const char *fmt, va_list ap) {
    char buf[LOG_STACK_BUF];
    va_list ap2;
    va_copy(ap2, ap);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    if (n < 0) { va_end(ap2); return n; }
    if ((size_t)n < sizeof(buf)) {
        console_io_write_text(buf, (size_t)n);
        va_end(ap2);
        return n;
    }
    /* Longer than the stack buffer: format it again on the heap rather than cut a status dump. */
    char *big = pvPortMalloc((size_t)n + 1u);
    if (big) {
        vsnprintf(big, (size_t)n + 1u, fmt, ap2);
        console_io_write_text(big, (size_t)n);
        vPortFree(big);
    } else {
        size_t keep = sizeof(buf) - sizeof(LOG_CUT_MARK);
        memcpy(buf + keep, LOG_CUT_MARK, sizeof(LOG_CUT_MARK));   /* with its NUL */
        console_io_write_text(buf, keep + sizeof(LOG_CUT_MARK) - 1u);
    }
    va_end(ap2);
    return n;
}

int log_vprintf(const char *fmt, va_list ap) {
    if (!log_on(BP_LOG_INFO)) return 0;
    return write_v(fmt, ap);
}

int log_printf(const char *fmt, ...) {
    if (!log_on(BP_LOG_INFO)) return 0;
    va_list ap;
    va_start(ap, fmt);
    int n = write_v(fmt, ap);
    va_end(ap);
    return n;
}

int log_lprintf(bp_log_level_t lv, const char *fmt, ...) {
    if (!log_on(lv)) return 0;
    va_list ap;
    va_start(ap, fmt);
    int n = write_v(fmt, ap);
    va_end(ap);
    return n;
}

int log_reply(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    int n = write_v(fmt, ap);
    va_end(ap);
    return n;
}

static void vfmt(char *out, size_t cap, const char *fmt, va_list ap) {
    int n = vsnprintf(out, cap, fmt, ap);
    if (n < 0) out[0] = '\0';
}

void log_repeat_failf(log_repeat_t *r, bp_log_level_t lv, const char *fmt, ...) {
    log_repeat_action_t a = log_repeat_fail(r, HAL_GetTick());
    if (a == LOG_REPEAT_QUIET || !log_on(lv)) return;
    char msg[LOG_STACK_BUF - 40u];
    va_list ap;
    va_start(ap, fmt);
    vfmt(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    if (a == LOG_REPEAT_FIRST) log_reply("%s\n", msg);
    else                       log_reply("%s: still failing (%lu times)\n", msg, (unsigned long)r->fails);
}

uint32_t log_repeat_okf(log_repeat_t *r, bp_log_level_t lv, const char *fmt, ...) {
    uint32_t n = log_repeat_ok(r);
    if (n == 0 || !log_on(lv)) return n;
    char msg[LOG_STACK_BUF - 40u];
    va_list ap;
    va_start(ap, fmt);
    vfmt(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    log_reply("%s (after %lu failure%s)\n", msg, (unsigned long)n, n == 1 ? "" : "s");
    return n;
}
