/*
 * test_bp_errors.c: host unit tests for bp_error_format (stable error codes, bp_errors.h): the
 * reply shape (code and args, no message; BP0030 keeps one), string escaping, the never-a-torn-
 * line rule when an arg is too long, the short form, and the notes send_error() uses to send a
 * module's refusal with its code.
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
void bp_error_send(int conn_id, bp_errcode_t code, const bp_err_arg_t *args, size_t n) {
    (void)conn_id;
    if (!bp_error_format(g_line, sizeof(g_line), code, args, n)) g_line[0] = 0;
}

static void test_shape(void) {
    bp_err_i2c_session_open(0);
    CHECK(!strcmp(g_line, "{\"status\":\"error\",\"code\":\"BP0003\"}\n"));
    bp_err_unknown_command(0, "frobnicate");
    CHECK(!strcmp(g_line, "{\"status\":\"error\",\"code\":\"BP0002\",\"args\":{\"cmd\":\"frobnicate\"}}\n"));
    bp_err_wiring_revision_conflict(0, "k66", 7);
    CHECK(!strcmp(g_line, "{\"status\":\"error\",\"code\":\"BP0009\",\"args\":{\"name\":\"k66\",\"rev\":7}}\n"));
}

/* BP0030 also carries a message: the client it refuses may not know the code. */
static void test_client_too_old(void) {
    bp_err_client_too_old(0, 1, 2, 2);
    CHECK(!strcmp(g_line, "{\"status\":\"error\",\"code\":\"BP0030\",\"args\":{\"client_api\":1,"
                          "\"min_client_api\":2,\"api\":2},\"message\":\"Update the BenchPod SDK, MCP or "
                          "CLI: this firmware needs client API 2 or newer (this client sent 1)\"}\n"));
}

/* A string arg with quotes stays valid JSON. */
static void test_escaping(void) {
    bp_err_unknown_command(0, "a\"b\\c");
    CHECK(!strcmp(g_line, "{\"status\":\"error\",\"code\":\"BP0002\",\"args\":{\"cmd\":\"a\\\"b\\\\c\"}}\n"));
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
    CHECK(!strcmp(buf, "{\"status\":\"error\",\"code\":\"BP0002\"}\n"));
    CHECK(bp_error_format(buf, 8, BP_E_UNKNOWN_COMMAND, a, 1) == 0);
}

static void test_short_form(void) {
    const bp_err_arg_t a[] = { {"la", 0, 4, 0}, {"owner", "gpio", 0, 1} };
    char out[64];
    CHECK(bp_err_render(out, sizeof(out), BP_E_LA_PIN_IN_USE, a, 2) == strlen("BP0031 la=4 owner=gpio"));
    CHECK(!strcmp(out, "BP0031 la=4 owner=gpio"));
    CHECK(bp_err_render(out, 7, BP_E_LA_PIN_IN_USE, a, 2) == 6 && !strcmp(out, "BP0031"));
}

/* A noted refusal is found again by its text, also from a copy; its args outlive the caller's
   buffers; a third note pushes out the oldest; a tagged text keeps its own words. */
static void test_notes(void) {
    char name[16], m1[128], m2[128], m3[128], copy[128];
    strcpy(name, "k66");
    const char *r = bp_note_wiring_revision_conflict(m1, sizeof(m1), name, 3);
    CHECK(r == m1 && !strcmp(m1, "BP0009 name=k66 rev=3"));
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
    /* A tag: the text stays, the code goes with it; the same text again takes the new args. */
    const char *t = bp_tag_no_analog_front_end("no analog here", "generate");
    CHECK(!strcmp(t, "no analog here"));
    bp_tag_no_analog_front_end("no analog here", "dac_out");
    a = bp_err_noted("no analog here", &code, &n);
    CHECK(a && code == BP_E_NO_ANALOG_FRONT_END && !strcmp(a[0].s, "dac_out"));
    /* A long string arg is cut to fit the note. */
    char big[200], mb[256];
    memset(big, 'y', sizeof(big) - 1);
    big[sizeof(big) - 1] = 0;
    bp_note_unknown_command(mb, sizeof(mb), big);
    a = bp_err_noted(mb, &code, &n);
    CHECK(a && strlen(a[0].s) < sizeof(big) - 1 && a[0].s[0] == 'y');
}

int main(void) {
    test_shape();
    test_client_too_old();
    test_escaping();
    test_too_long();
    test_short_form();
    test_notes();
    if (failures) { printf("test_bp_errors: %d failure(s)\n", failures); return 1; }
    printf("test_bp_errors: all passed\n");
    return 0;
}
