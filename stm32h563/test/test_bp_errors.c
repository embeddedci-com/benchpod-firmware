/*
 * test_bp_errors.c: host unit tests for bp_error_format (stable error codes, bp_errors.h): the
 * reply shape, the message rendered from the catalog template, string escaping, the never-a-
 * torn-line rule when an arg is too long, and the notes send_error() uses to send a module's
 * message with its code. Built twice: as is, and with -DBP_ERRORS_NO_TEXT (the compact fallback
 * message, test_bp_errors_notext).
 */
#define BP_ERRORS_DEFINE_TEMPLATES   /* the template table, to render every code below */
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
void bp_error_send(int conn_id, bp_errcode_t code, const bp_err_arg_t *args, size_t n) {
    (void)conn_id;
    if (!bp_error_format(g_line, sizeof(g_line), code, args, n)) g_line[0] = 0;
}

#ifndef BP_ERRORS_NO_TEXT
/* The message is the catalog's, args filled in. */
static void test_catalog_text(void) {
    bp_err_i2c_session_open(0);
    CHECK(!strcmp(g_line,
        "{\"status\":\"error\",\"code\":\"BP0003\",\"message\":\"An I2C session is already open\"}\n"));
    bp_err_unknown_command(0, "frobnicate");
    CHECK(!strcmp(g_line, "{\"status\":\"error\",\"code\":\"BP0002\",\"args\":{\"cmd\":\"frobnicate\"},"
                          "\"message\":\"The pod does not know the command frobnicate\"}\n"));
    bp_err_wiring_revision_conflict(0, "k66", 7);
    CHECK(!strcmp(g_line, "{\"status\":\"error\",\"code\":\"BP0009\",\"args\":{\"name\":\"k66\",\"rev\":7},"
                          "\"message\":\"The wiring profile k66 changed since it was loaded (it is now at "
                          "revision 7)\"}\n"));
}

/* A string arg with quotes stays valid JSON, in args and in the message. */
static void test_escaping(void) {
    bp_err_unknown_command(0, "a\"b\\c");
    CHECK(!strcmp(g_line, "{\"status\":\"error\",\"code\":\"BP0002\",\"args\":{\"cmd\":\"a\\\"b\\\\c\"},"
                          "\"message\":\"The pod does not know the command a\\\"b\\\\c\"}\n"));
}

/* Every code's template renders with all of its args: no "?" and no "{" left. */
static void test_every_template(void) {
    for (unsigned c = 1; c <= BP_E_LAST; c++) {
        const char *t = bp_err_templates[c];
        if (!t) continue;
        bp_err_arg_t args[8];
        char keys[8][32];
        size_t n = 0;
        for (const char *p = strchr(t, '{'); p && n < 8; p = strchr(p + 1, '{')) {
            const char *q = strchr(p, '}');
            if (!q || (size_t)(q - p - 1) >= sizeof(keys[0])) break;
            memcpy(keys[n], p + 1, (size_t)(q - p - 1));
            keys[n][q - p - 1] = 0;
            args[n] = (bp_err_arg_t){ keys[n], "v", 0, 1 };
            n++;
        }
        char out[400];
        bp_err_render(out, sizeof(out), (bp_errcode_t)c, args, n);
        if (strchr(out, '{') || strchr(out, '?') || !out[0]) {
            printf("FAIL BP%04u renders as \"%s\"\n", c, out);
            failures++;
        }
    }
}

/* A missing arg renders as "?", like the hosts do. */
static void test_missing_arg(void) {
    char out[96];
    bp_err_render(out, sizeof(out), BP_E_UNKNOWN_COMMAND, NULL, 0);
    CHECK(!strcmp(out, "The pod does not know the command ?"));
    /* cut to fit, still terminated */
    bp_err_render(out, 8, BP_E_UNKNOWN_COMMAND, NULL, 0);
    CHECK(!strcmp(out, "The pod"));
}
#else
/* No templates: the message is the compact fallback. */
static void test_fallback(void) {
    bp_err_bus_engine_busy(0, "spi");
    CHECK(!strcmp(g_line, "{\"status\":\"error\",\"code\":\"BP0005\",\"args\":{\"want\":\"spi\"},"
                          "\"message\":\"BP0005 want=spi\"}\n"));
    bp_err_i2c_session_open(0);
    CHECK(!strcmp(g_line, "{\"status\":\"error\",\"code\":\"BP0003\",\"message\":\"BP0003\"}\n"));
    bp_err_unknown_command(0, "a\"b");
    CHECK(!strcmp(g_line, "{\"status\":\"error\",\"code\":\"BP0002\",\"args\":{\"cmd\":\"a\\\"b\"},"
                          "\"message\":\"BP0002 cmd=a\\\"b\"}\n"));
}
#endif

static void test_int_arg(void) {
    const bp_err_arg_t a[] = { {"la", 0, 4, 0}, {"owner", "gpio", 0, 1} };
    char buf[256];
    /* A code past the catalog has no template: the fallback, whatever the build. */
    CHECK(bp_error_format(buf, sizeof(buf), (bp_errcode_t)4120, a, 2) > 0);
    CHECK(!strcmp(buf, "{\"status\":\"error\",\"code\":\"BP4120\",\"args\":{\"la\":4,\"owner\":\"gpio\"},"
                       "\"message\":\"BP4120 la=4 owner=gpio\"}\n"));
}

/* Too long for the buffer: the code survives, the line still ends in a newline. */
static void test_too_long(void) {
    char big[300];
    memset(big, 'x', sizeof(big) - 1);
    big[sizeof(big) - 1] = 0;
    const bp_err_arg_t a[] = { {"cmd", big, 0, 1} };
    char buf[128];
    size_t n = bp_error_format(buf, sizeof(buf), BP_E_UNKNOWN_COMMAND, a, 1);
    CHECK(n > 0 && buf[n - 1] == '\n');
    CHECK(!strcmp(buf, "{\"status\":\"error\",\"code\":\"BP0002\",\"message\":\"BP0002\"}\n"));
    CHECK(bp_error_format(buf, 8, BP_E_UNKNOWN_COMMAND, a, 1) == 0);
}

/* A noted message is found again by its text, also from a copy; its args outlive the caller's
   buffers; a third note pushes out the oldest. */
static void test_notes(void) {
    char name[16], m1[128], m2[128], m3[128], copy[128];
    strcpy(name, "k66");
    const char *r = bp_note_wiring_revision_conflict(m1, sizeof(m1), name, 3);
    CHECK(r == m1);
    strcpy(name, "zzz");   /* the note keeps its own copy */
    strcpy(copy, m1);
    bp_errcode_t code = BP_E_NONE;
    size_t n = 0;
    const bp_err_arg_t *a = bp_err_noted(copy, &code, &n);
    CHECK(a && code == BP_E_WIRING_REVISION_CONFLICT && n == 2);
    CHECK(a && !strcmp(a[0].key, "name") && !strcmp(a[0].s, "k66") && a[1].i == 3);
    CHECK(!bp_err_noted("some other text", &code, &n));
    /* The same text again does not take a second note. */
    bp_note_wiring_revision_conflict(m2, sizeof(m2), "k66", 3);
    bp_note_unknown_command(m2, sizeof(m2), "x");
    CHECK(bp_err_noted(m1, &code, &n) && code == BP_E_WIRING_REVISION_CONFLICT);
    bp_note_bus_engine_busy(m3, sizeof(m3), "spi");
    CHECK(!bp_err_noted(m1, &code, &n));   /* pushed out */
    CHECK(bp_err_noted(m2, &code, &n) && code == BP_E_UNKNOWN_COMMAND);
    CHECK(bp_err_noted(m3, &code, &n) && code == BP_E_BUS_ENGINE_BUSY);
    /* A long string arg is cut to fit the note, the text keeps it whole. */
    char big[200], mb[256];
    memset(big, 'y', sizeof(big) - 1);
    big[sizeof(big) - 1] = 0;
    bp_note_unknown_command(mb, sizeof(mb), big);
    a = bp_err_noted(mb, &code, &n);
    CHECK(a && strlen(a[0].s) < sizeof(big) - 1 && a[0].s[0] == 'y');
}

int main(void) {
#ifndef BP_ERRORS_NO_TEXT
    test_catalog_text();
    test_escaping();
    test_every_template();
    test_missing_arg();
#else
    test_fallback();
#endif
    test_int_arg();
    test_too_long();
    test_notes();
    if (failures) { printf("test_bp_errors: %d failure(s)\n", failures); return 1; }
#ifdef BP_ERRORS_NO_TEXT
    printf("test_bp_errors (no text): all passed\n");
#else
    printf("test_bp_errors: all passed\n");
#endif
    return 0;
}
