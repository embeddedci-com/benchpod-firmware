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

int log_vprintf(const char *fmt, va_list ap) {
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

int log_printf(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    int n = log_vprintf(fmt, ap);
    va_end(ap);
    return n;
}
