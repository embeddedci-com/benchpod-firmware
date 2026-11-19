/*
 * test_bp_log.c — host tests for log_printf (src/bp_log.c): a message is formatted into a buffer
 * of its own and handed to the console in ONE console_io_write_text call (one lock), short ones
 * from the stack, long ones from the heap, and a failed heap allocation cuts the message with a
 * visible mark instead of dropping it.
 */
#include "bp_log.h"
#include "console_io.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures;
#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); failures++; } \
} while (0)

/* ---- fakes ---- */
static char   s_out[4096];
static size_t s_out_len;
static int    s_writes, s_allocs, s_frees;
static bool   s_alloc_fails;

void console_io_write_text(const char *ptr, size_t len) {
    memcpy(s_out + s_out_len, ptr, len);
    s_out_len += len;
    s_out[s_out_len] = '\0';
    s_writes++;
}
static uint32_t s_now;
uint32_t HAL_GetTick(void) { return s_now; }
void *pvPortMalloc(size_t size) { s_allocs++; return s_alloc_fails ? NULL : malloc(size); }
void  vPortFree(void *p) { s_frees++; free(p); }

static void reset(void) { s_out_len = 0; s_out[0] = '\0'; s_writes = s_allocs = s_frees = 0; s_alloc_fails = false; }

static void test_short_message_one_write(void) {
    reset();
    int n = log_printf("[la] LA%u step x%lu started (delay %luus)\n", 3u, 70000ul, 500ul);
    CHECK(strcmp(s_out, "[la] LA3 step x70000 started (delay 500us)\n") == 0);
    CHECK(n == (int)strlen(s_out));
    CHECK(s_writes == 1);
    CHECK(s_allocs == 0);
}

static void test_long_message_from_heap(void) {
    reset();
    char big[1000];
    memset(big, 'x', sizeof(big) - 1);
    big[sizeof(big) - 1] = '\0';
    int n = log_printf("[status] %s|%d\n", big, 42);
    CHECK(n == 9 + 999 + 4);
    CHECK(s_out_len == (size_t)n);
    CHECK(strncmp(s_out, "[status] xxx", 12) == 0);
    CHECK(strcmp(s_out + s_out_len - 4, "|42\n") == 0);
    CHECK(s_writes == 1);
    CHECK(s_allocs == 1 && s_frees == 1);
}

static void test_alloc_failure_cuts_visibly(void) {
    reset();
    s_alloc_fails = true;
    char big[400];
    memset(big, 'y', sizeof(big) - 1);
    big[sizeof(big) - 1] = '\0';
    log_printf("%s\n", big);
    CHECK(s_writes == 1);
    CHECK(s_out_len == LOG_STACK_BUF - 1u);
    CHECK(strcmp(s_out + s_out_len - strlen(LOG_CUT_MARK), LOG_CUT_MARK) == 0);
    CHECK(s_out[0] == 'y');
}

static void test_exact_fit_stays_on_stack(void) {
    reset();
    char line[LOG_STACK_BUF];
    memset(line, 'z', sizeof(line) - 1);
    line[sizeof(line) - 1] = '\0';          /* LOG_STACK_BUF - 1 chars: fits with its NUL */
    log_printf("%s", line);
    CHECK(s_allocs == 0);
    CHECK(s_out_len == LOG_STACK_BUF - 1u);
    reset();
    log_printf("%s!", line);                /* one more: heap */
    CHECK(s_allocs == 1);
    CHECK(s_out_len == LOG_STACK_BUF);
}

/* The level gate: info (default) hides debug, error always prints, LOG_REPLY always prints. */
static void test_level_gate(void) {
    log_set_level(BP_LOG_INFO);
    reset();
    LOG_D("[cmd] <- \"ping\"\n");
    CHECK(s_out_len == 0 && s_writes == 0);
    LOG_I("[cloud] connected\n");
    LOG_W("[wifi] not connected\n");
    CHECK(strcmp(s_out, "[cloud] connected\n[wifi] not connected\n") == 0);
    log_set_level(BP_LOG_ERROR);
    reset();
    log_printf("[net] eth DHCP: ip=1.2.3.4\n");
    LOG_W("[wifi] x\n");
    LOG_E("[psram] self-test FAILED\n");
    LOG_REPLY("flash-esp32: failed\n");
    CHECK(strcmp(s_out, "[psram] self-test FAILED\nflash-esp32: failed\n") == 0);
    log_set_level(BP_LOG_DEBUG);
    reset();
    LOG_D("[cmd] -> ok\n");
    CHECK(strcmp(s_out, "[cmd] -> ok\n") == 0);
    log_set_level(BP_LOG_INFO);
}

/* The repeat helpers: first failure, quiet, "still failing" a minute later, recovery once. */
static void test_repeat_helpers(void) {
    log_set_level(BP_LOG_INFO);
    log_repeat_t r = {0};
    reset();
    s_now = 1000;
    log_repeat_failf(&r, BP_LOG_WARN, "[cloud] cannot connect: %s", "dns failed");
    CHECK(strcmp(s_out, "[cloud] cannot connect: dns failed\n") == 0);
    reset();
    for (int i = 0; i < 5; i++) { s_now += 10000; log_repeat_failf(&r, BP_LOG_WARN, "[cloud] cannot connect: %s", "x"); }
    CHECK(s_out_len == 0);                         /* 51 s after the first: quiet */
    s_now += 10000;
    log_repeat_failf(&r, BP_LOG_WARN, "[cloud] cannot connect: %s", "tcp refused");
    CHECK(strcmp(s_out, "[cloud] cannot connect: tcp refused: still failing (7 times)\n") == 0);
    reset();
    CHECK(log_repeat_okf(&r, BP_LOG_INFO, "[cloud] connected") == 7);
    CHECK(strcmp(s_out, "[cloud] connected (after 7 failures)\n") == 0);
    reset();
    CHECK(log_repeat_okf(&r, BP_LOG_INFO, "[cloud] connected") == 0);
    CHECK(s_out_len == 0);                         /* no failures: no recovery line */
    /* A debug-level loop at info: counted, never printed. */
    log_repeat_failf(&r, BP_LOG_DEBUG, "[x] y");
    CHECK(s_out_len == 0 && r.fails == 1);
}

int main(void) {
    test_level_gate();
    test_repeat_helpers();
    test_short_message_one_write();
    test_long_message_from_heap();
    test_alloc_failure_cuts_visibly();
    test_exact_fit_stays_on_stack();
    if (failures) { printf("FAIL test_bp_log: %d failure(s)\n", failures); return 1; }
    printf("PASS test_bp_log\n");
    return 0;
}
