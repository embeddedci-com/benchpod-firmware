/*
 * bp_log.h — thread-safe device log output.
 *
 * printf() is not safe across the console, net and hw worker tasks: newlib runs without
 * per-task reentrancy (configUSE_NEWLIB_REENTRANT 0), and _write() hands each printf to the
 * console in fragments (one per line break), so two tasks' lines interleave mid-line and race
 * _write's line-ending state.  log_printf() formats into a buffer of its own with vsnprintf and
 * writes the whole message with console_io_write_text(), under the console lock, in one go.
 *
 * Task context only (and before the scheduler starts), never from an ISR or a fault hook: a
 * message longer than the stack buffer is formatted again into a heap buffer, and the console
 * lock is a mutex.  Fault and stack-overflow hooks keep using printf().
 */
#ifndef BP_LOG_H
#define BP_LOG_H

#include <stdarg.h>
#include <stddef.h>

/* Stack buffer per call; longer messages go through the heap (or are cut with LOG_CUT_MARK if
   that allocation fails). */
#define LOG_STACK_BUF   192u
#define LOG_CUT_MARK    "...[cut]\n"

#ifdef LOG_PRINTF_IS_PRINTF
/* Host tests: the code under test logs through plain printf. */
#include <stdio.h>
#define log_printf  printf
#define log_vprintf vprintf
#else
int log_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
int log_vprintf(const char *fmt, va_list ap);
#endif

#endif /* BP_LOG_H */
