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

int main(void) {
    test_short_message_one_write();
    test_long_message_from_heap();
    test_alloc_failure_cuts_visibly();
    test_exact_fit_stays_on_stack();
    if (failures) { printf("FAIL test_bp_log: %d failure(s)\n", failures); return 1; }
    printf("PASS test_bp_log\n");
    return 0;
}
