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
 * lock is a mutex.  Fault and stack-overflow hooks keep using printf(), which is not gated by
 * the log level (console command output and dev-command dumps use it too).
 */
#ifndef BP_LOG_H
#define BP_LOG_H

#include <stdarg.h>
#include <stddef.h>

/* Stack buffer per call; longer messages go through the heap (or are cut with LOG_CUT_MARK if
   that allocation fails). */
#define LOG_STACK_BUF   192u
#define LOG_CUT_MARK    "...[cut]\n"

/* Levels (log_level.h): LOG_E/LOG_W/LOG_I/LOG_D print a line at that level, when the pod's log
   level lets it through (errors always). log_printf is an info line. LOG_REPLY is not a log line
   but a late console reply (flash-esp32's result): it always prints.
   Each message keeps its "[tag] " prefix in the format: LOG_D("[cmd] <- \"%s\"\n", verb). */
#include "log_level.h"

#ifdef LOG_PRINTF_IS_PRINTF
/* Host tests: the code under test logs through plain printf. */
#include <stdio.h>
#define log_printf  printf
#define log_vprintf vprintf
#define log_reply   printf
#define log_lprintf(lv, ...) ((void)(lv), printf(__VA_ARGS__))
#define log_repeat_failf(r, lv, ...) ((void)(r), (void)(lv), printf(__VA_ARGS__))
#define log_repeat_okf(r, lv, ...)   ((void)(r), (void)(lv), 0)
#else
int log_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
int log_vprintf(const char *fmt, va_list ap);
int log_reply(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
int log_lprintf(bp_log_level_t lv, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

/* A retry loop's failure (log_level.h repeat limiter): the first one prints the message at lv,
   then at most once a minute "<message>: still failing (N times)". The message is one line
   without its newline: log_repeat_failf(&s_rep, BP_LOG_WARN, "[net] no DHCP lease yet"). */
void log_repeat_failf(log_repeat_t *r, bp_log_level_t lv, const char *fmt, ...)
    __attribute__((format(printf, 3, 4)));
/* Its success: when it had failed, "<message> (after N failures)" at lv. Returns N. */
uint32_t log_repeat_okf(log_repeat_t *r, bp_log_level_t lv, const char *fmt, ...)
    __attribute__((format(printf, 3, 4)));
#endif

#define LOG_E(...)     log_lprintf(BP_LOG_ERROR, __VA_ARGS__)
#define LOG_W(...)     log_lprintf(BP_LOG_WARN, __VA_ARGS__)
#define LOG_I(...)     log_lprintf(BP_LOG_INFO, __VA_ARGS__)
#define LOG_D(...)     log_lprintf(BP_LOG_DEBUG, __VA_ARGS__)
#define LOG_REPLY(...) log_reply(__VA_ARGS__)

#endif /* BP_LOG_H */
