/*
 * test_log_level.c — host tests for src/log_level.c: the level filter (errors never gated), the
 * names, the stored form in the pod policy record, and the repeat limiter (first failure, then at
 * most one "still failing" a minute, recovery count).
 */
#include "log_level.h"

#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); failures++; } \
} while (0)

static void test_default_and_filter(void) {
    CHECK(log_level() == BP_LOG_INFO);
    CHECK(log_on(BP_LOG_ERROR) && log_on(BP_LOG_WARN) && log_on(BP_LOG_INFO));
    CHECK(!log_on(BP_LOG_DEBUG));
    log_set_level(BP_LOG_DEBUG);
    CHECK(log_on(BP_LOG_DEBUG));
    log_set_level(BP_LOG_WARN);
    CHECK(log_on(BP_LOG_WARN) && !log_on(BP_LOG_INFO));
    log_set_level(BP_LOG_ERROR);
    CHECK(log_on(BP_LOG_ERROR) && !log_on(BP_LOG_WARN));
    log_set_level((bp_log_level_t)9);          /* out of range: debug */
    CHECK(log_level() == BP_LOG_DEBUG);
    log_set_level(BP_LOG_INFO);
}

static void test_names(void) {
    CHECK(strcmp(log_level_name(BP_LOG_ERROR), "error") == 0);
    CHECK(strcmp(log_level_name(BP_LOG_WARN), "warn") == 0);
    CHECK(strcmp(log_level_name(BP_LOG_INFO), "info") == 0);
    CHECK(strcmp(log_level_name(BP_LOG_DEBUG), "debug") == 0);
    CHECK(log_level_from_name("error") == BP_LOG_ERROR);
    CHECK(log_level_from_name("warn") == BP_LOG_WARN);
    CHECK(log_level_from_name("warning") == BP_LOG_WARN);
    CHECK(log_level_from_name("info") == BP_LOG_INFO);
    CHECK(log_level_from_name("debug") == BP_LOG_DEBUG);
    CHECK(log_level_from_name("verbose") == -1);
    CHECK(log_level_from_name("") == -1);
    CHECK(log_level_from_name(NULL) == -1);
}

static void test_stored(void) {
    CHECK(log_level_from_stored(0) == BP_LOG_INFO);    /* never set: older records */
    for (int lv = BP_LOG_ERROR; lv <= BP_LOG_DEBUG; lv++)
        CHECK(log_level_from_stored(log_level_to_stored((bp_log_level_t)lv)) == (bp_log_level_t)lv);
    CHECK(log_level_to_stored(BP_LOG_ERROR) == 1);
    CHECK(log_level_from_stored(200) == BP_LOG_DEBUG);  /* newer firmware's value: show more */
}

static void test_repeat(void) {
    log_repeat_t r = {0};
    CHECK(log_repeat_fail(&r, 5000) == LOG_REPEAT_FIRST);
    uint32_t t = 5000;
    int still = 0, quiet = 0;
    /* A retry every 20 s for 10 minutes: one "still failing" a minute. */
    for (int i = 0; i < 30; i++) {
        t += 20000;
        log_repeat_action_t a = log_repeat_fail(&r, t);
        if (a == LOG_REPEAT_STILL) still++;
        else if (a == LOG_REPEAT_QUIET) quiet++;
        else CHECK(!"no second FIRST");
    }
    CHECK(still == 10);
    CHECK(quiet == 20);
    CHECK(r.fails == 31);
    CHECK(log_repeat_ok(&r) == 31);
    CHECK(log_repeat_ok(&r) == 0);
    /* After a recovery the next failure is a first one again. */
    CHECK(log_repeat_fail(&r, t + 1) == LOG_REPEAT_FIRST);
    /* The tick counter wraps: still a minute apart. */
    log_repeat_t w = {0};
    CHECK(log_repeat_fail(&w, 0xFFFFFF00u) == LOG_REPEAT_FIRST);
    CHECK(log_repeat_fail(&w, 0xFFFFFF00u + 30000u) == LOG_REPEAT_QUIET);
    CHECK(log_repeat_fail(&w, 0xFFFFFF00u + 60000u) == LOG_REPEAT_STILL);
}

int main(void) {
    test_default_and_filter();
    test_names();
    test_stored();
    test_repeat();
    if (failures) { printf("FAIL test_log_level: %d failure(s)\n", failures); return 1; }
    printf("PASS test_log_level\n");
    return 0;
}
