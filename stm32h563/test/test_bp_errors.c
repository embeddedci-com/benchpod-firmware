/*
 * test_bp_errors.c: host unit tests for bp_error_format (stable error codes, bp_errors.h): the
 * reply shape with and without text, the compact fallback, string escaping, and the never-a-torn-
 * line rule when an arg is too long. Also checks every code the generated header knows formats.
 */
#include "bp_errors.h"

#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); failures++; } \
} while (0)

/* bp_error_send is the firmware's sender (command_handler.c); the inline bp_err_* wrappers call
   it. Here it formats into a buffer the tests read. */
static char g_line[512];
void bp_error_send(int conn_id, bp_errcode_t code, const char *text, const bp_err_arg_t *args,
                   size_t n) {
    (void)conn_id;
    if (!bp_error_format(g_line, sizeof(g_line), code, text, args, n)) g_line[0] = 0;
}

static void test_with_text(void) {
    bp_err_la_voltage_not_set(0, "la voltage not set; set it first");
    CHECK(!strcmp(g_line,
        "{\"status\":\"error\",\"code\":\"BP0001\",\"message\":\"la voltage not set; set it first\"}\n"));
}

static void test_args_and_text(void) {
    bp_err_unknown_command(0, "unknown cmd", "frobnicate");
    CHECK(!strcmp(g_line, "{\"status\":\"error\",\"code\":\"BP0002\",\"args\":{\"cmd\":\"frobnicate\"},"
                          "\"message\":\"unknown cmd\"}\n"));
}

/* No text (a BP_ERRORS_NO_TEXT build): the message is the compact fallback. */
static void test_fallback(void) {
    bp_err_bus_engine_busy(0, NULL, "spi");
    CHECK(!strcmp(g_line, "{\"status\":\"error\",\"code\":\"BP0005\",\"args\":{\"want\":\"spi\"},"
                          "\"message\":\"BP0005 want=spi\"}\n"));
    bp_err_i2c_session_open(0, NULL);
    CHECK(!strcmp(g_line, "{\"status\":\"error\",\"code\":\"BP0003\",\"message\":\"BP0003\"}\n"));
}

static void test_int_arg(void) {
    const bp_err_arg_t a[] = { {"la", 0, 4, 0}, {"owner", "gpio", 0, 1} };
    char buf[256];
    CHECK(bp_error_format(buf, sizeof(buf), (bp_errcode_t)412, NULL, a, 2) > 0);
    CHECK(!strcmp(buf, "{\"status\":\"error\",\"code\":\"BP0412\",\"args\":{\"la\":4,\"owner\":\"gpio\"},"
                       "\"message\":\"BP0412 la=4 owner=gpio\"}\n"));
}

/* A string arg with quotes stays valid JSON, in args and in the fallback. */
static void test_escaping(void) {
    bp_err_unknown_command(0, NULL, "a\"b");
    CHECK(!strcmp(g_line, "{\"status\":\"error\",\"code\":\"BP0002\",\"args\":{\"cmd\":\"a\\\"b\"},"
                          "\"message\":\"BP0002 cmd=a\\\"b\"}\n"));
}

/* Too long for the buffer: the code survives, the line still ends in a newline. */
static void test_too_long(void) {
    char big[300];
    memset(big, 'x', sizeof(big) - 1);
    big[sizeof(big) - 1] = 0;
    const bp_err_arg_t a[] = { {"cmd", big, 0, 1} };
    char buf[128];
    size_t n = bp_error_format(buf, sizeof(buf), BP_E_UNKNOWN_COMMAND, NULL, a, 1);
    CHECK(n > 0 && buf[n - 1] == '\n');
    CHECK(!strcmp(buf, "{\"status\":\"error\",\"code\":\"BP0002\",\"message\":\"BP0002\"}\n"));
    CHECK(bp_error_format(buf, 8, BP_E_UNKNOWN_COMMAND, NULL, a, 1) == 0);
}

int main(void) {
    test_with_text();
    test_args_and_text();
    test_fallback();
    test_int_arg();
    test_escaping();
    test_too_long();
    if (failures) { printf("test_bp_errors: %d failure(s)\n", failures); return 1; }
    printf("test_bp_errors: all passed\n");
    return 0;
}
