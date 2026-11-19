/*
 * log_level.h — console log levels and the repeat limiter (pure C, host-tested).
 *
 * Every background line the firmware writes to its console (USB-CDC + USART2) has a level:
 *
 *   error  a real fault: crash report, assert, iCE40/PSRAM/storage/identity failure, out of
 *          memory. Never gated: printed at every level.
 *   warn   something went wrong but the pod carries on (a retry loop's first failure).
 *   info   boot lines and state changes (link up, cloud connected, lost, giving up). The default.
 *   debug  detail that only helps debugging: the per-command trace of LAN/cloud commands, each
 *          retry of a reconnect loop, lease renewals, RSSI misses, USB link flaps.
 *
 * The level is a pod setting (console `log`, JSON `log_level`), stored with the pod policy
 * record (pod_policy.h) so it survives a reboot and a firmware update. Lines at or below the
 * level are printed; console command replies are not log lines and always print.
 *
 * Repeat limiter: a retry loop calls log_repeat_fail() on every failure and log_repeat_ok() on
 * success. It prints the first failure, then at most one "still failing (N times)" line a
 * minute, then one line on recovery, instead of one line per attempt forever.
 */
#ifndef LOG_LEVEL_H
#define LOG_LEVEL_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    BP_LOG_ERROR = 0,
    BP_LOG_WARN  = 1,
    BP_LOG_INFO  = 2,
    BP_LOG_DEBUG = 3,
} bp_log_level_t;

#define BP_LOG_DEFAULT BP_LOG_INFO

bp_log_level_t log_level(void);
/* Out-of-range values clamp to debug. */
void log_set_level(bp_log_level_t lv);
/* Whether a line at lv prints now. Errors always do. */
bool log_on(bp_log_level_t lv);

/* "error" / "warn" / "info" / "debug". */
const char *log_level_name(bp_log_level_t lv);
/* The level for a name (also "warning"), -1 if unknown. */
int log_level_from_name(const char *name);

/* Stored form (pod_policy.h): 0 = never set (the default, info), else level + 1. */
uint8_t        log_level_to_stored(bp_log_level_t lv);
bp_log_level_t log_level_from_stored(uint8_t v);

/* ---- repeat limiter ---- */

#define LOG_REPEAT_EVERY_MS 60000u

typedef struct {
    uint32_t fails;      /* failures since the last success */
    uint32_t last_ms;    /* when the last line about them was printed */
} log_repeat_t;

typedef enum {
    LOG_REPEAT_QUIET = 0,   /* say nothing */
    LOG_REPEAT_FIRST,       /* the first failure: print it */
    LOG_REPEAT_STILL,       /* print "still failing (N times)", N = r->fails */
} log_repeat_action_t;

/* Count one failure at now_ms and say what to print. */
log_repeat_action_t log_repeat_fail(log_repeat_t *r, uint32_t now_ms);
/* Count a success: the failures since the last success (print one recovery line when > 0). */
uint32_t log_repeat_ok(log_repeat_t *r);

#endif /* LOG_LEVEL_H */
