/*
 * log_level.c — see log_level.h.
 */
#include "log_level.h"

#include <string.h>

static volatile uint8_t s_level = BP_LOG_DEFAULT;

bp_log_level_t log_level(void) { return (bp_log_level_t)s_level; }

void log_set_level(bp_log_level_t lv) {
    if ((unsigned)lv > BP_LOG_DEBUG) lv = BP_LOG_DEBUG;
    s_level = (uint8_t)lv;
}

bool log_on(bp_log_level_t lv) { return lv == BP_LOG_ERROR || (unsigned)lv <= s_level; }

const char *log_level_name(bp_log_level_t lv) {
    switch (lv) {
    case BP_LOG_ERROR: return "error";
    case BP_LOG_WARN:  return "warn";
    case BP_LOG_INFO:  return "info";
    case BP_LOG_DEBUG: return "debug";
    }
    return "debug";
}

int log_level_from_name(const char *name) {
    if (!name) return -1;
    if (!strcmp(name, "error")) return BP_LOG_ERROR;
    if (!strcmp(name, "warn") || !strcmp(name, "warning")) return BP_LOG_WARN;
    if (!strcmp(name, "info")) return BP_LOG_INFO;
    if (!strcmp(name, "debug")) return BP_LOG_DEBUG;
    return -1;
}

uint8_t log_level_to_stored(bp_log_level_t lv) {
    if ((unsigned)lv > BP_LOG_DEBUG) lv = BP_LOG_DEBUG;
    return (uint8_t)(lv + 1u);
}

bp_log_level_t log_level_from_stored(uint8_t v) {
    if (v == 0) return BP_LOG_DEFAULT;
    if (v > BP_LOG_DEBUG + 1u) return BP_LOG_DEBUG;   /* from newer firmware: show more, not less */
    return (bp_log_level_t)(v - 1u);
}

log_repeat_action_t log_repeat_fail(log_repeat_t *r, uint32_t now_ms) {
    r->fails++;
    if (r->fails == 1) {
        r->last_ms = now_ms;
        return LOG_REPEAT_FIRST;
    }
    if ((uint32_t)(now_ms - r->last_ms) >= LOG_REPEAT_EVERY_MS) {
        r->last_ms = now_ms;
        return LOG_REPEAT_STILL;
    }
    return LOG_REPEAT_QUIET;
}

uint32_t log_repeat_ok(log_repeat_t *r) {
    uint32_t n = r->fails;
    r->fails = 0;
    return n;
}
