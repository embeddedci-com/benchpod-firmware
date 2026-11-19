/*
 * test_console_prompt.c — host tests for src/console_prompt.c: background output after the
 * prompt starts on its own line and ends with the prompt (and the typed text) printed again;
 * command output while the prompt is down goes out untouched; typing during a half-written log
 * line is kept, not echoed into it.
 */
#include "console_prompt.h"

#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); failures++; } \
} while (0)
#define CHECK_OUT(want) do { \
    if (strcmp(s_out, want) != 0) { \
        printf("FAIL %s:%d  got \"%s\"\n      want \"%s\"\n", __FILE__, __LINE__, s_out, want); failures++; } \
} while (0)

static char   s_out[2048];
static size_t s_len;
static void sink(const char *s, size_t n) { memcpy(s_out + s_len, s, n); s_len += n; s_out[s_len] = '\0'; }
static void clear(void) { s_len = 0; s_out[0] = '\0'; }
static void out(console_prompt_t *p, const char *s) { console_prompt_out(p, sink, s, strlen(s)); }
static void type(console_prompt_t *p, const char *s) { while (*s) console_prompt_char(p, sink, *s++); }

static void test_log_after_bare_prompt(void) {
    console_prompt_t p = {0};
    clear();
    console_prompt_show(&p, sink);
    out(&p, "[cloud] connected\r\n");
    /* The bare prompt is overwritten, the console ends with a fresh one. */
    CHECK_OUT("> \r[cloud] connected\r\n> ");
}

static void test_log_while_typing(void) {
    console_prompt_t p = {0};
    clear();
    console_prompt_show(&p, sink);
    type(&p, "sta");
    out(&p, "[wifi] associated\r\n");
    CHECK_OUT("> sta\r\n[wifi] associated\r\n> sta");
    clear();
    type(&p, "tus");
    char line[CONSOLE_LINE_MAX];
    CHECK(console_prompt_enter(&p, sink, line, sizeof(line)) == 6);
    CHECK(strcmp(line, "status") == 0);
    CHECK_OUT("tus\r\n");
    CHECK(!p.shown);
}

static void test_command_output_untouched(void) {
    console_prompt_t p = {0};
    char line[CONSOLE_LINE_MAX];
    console_prompt_show(&p, sink);
    type(&p, "ping");
    console_prompt_enter(&p, sink, line, sizeof(line));
    clear();
    out(&p, "  PING ok  version=0x35\r\n");
    out(&p, "[cmd] <- \"status\"\n");           /* a log line during the command: as it is */
    CHECK_OUT("  PING ok  version=0x35\r\n[cmd] <- \"status\"\n");
    clear();
    console_prompt_show(&p, sink);
    CHECK_OUT("> ");
}

static void test_log_in_pieces(void) {
    /* printf hands a line over in pieces: the prompt comes back only once it ends. */
    console_prompt_t p = {0};
    clear();
    console_prompt_show(&p, sink);
    out(&p, "[fpga] gateware ");
    out(&p, "v58");
    type(&p, "he");                            /* kept, not echoed into the log line */
    console_prompt_backspace(&p, sink);
    type(&p, "elp");
    out(&p, "\n");
    CHECK_OUT("> \r[fpga] gateware v58\n> help");
    CHECK(p.len == 4 && memcmp(p.line, "help", 4) == 0);
}

static void test_enter_mid_log(void) {
    console_prompt_t p = {0};
    char line[CONSOLE_LINE_MAX];
    clear();
    console_prompt_show(&p, sink);
    out(&p, "[net] half");
    type(&p, "ls");
    CHECK(console_prompt_enter(&p, sink, line, sizeof(line)) == 2);
    /* The half line is ended and the typed line shown whole before it runs. */
    CHECK_OUT("> \r[net] half\r\n> ls\r\n");
}

static void test_async_reply_lines(void) {
    /* wifi-scan's list after the prompt: each line ends with the prompt again; the last one
       leaves the console at a prompt. */
    console_prompt_t p = {0};
    clear();
    console_prompt_show(&p, sink);
    out(&p, "scan -50 1 wpa2 Lab\r\n");
    out(&p, "scan done 1\r\n");
    CHECK_OUT("> \rscan -50 1 wpa2 Lab\r\n> \rscan done 1\r\n> ");
}

static void test_held_reply(void) {
    /* A held multi-line reply: one prompt after its last line; typing meanwhile is kept. */
    console_prompt_t p = {0};
    clear();
    console_prompt_show(&p, sink);
    console_prompt_hold(&p, sink, true);
    out(&p, "scan -50 1 wpa2 Lab\r\n");
    type(&p, "st");
    out(&p, "scan done 1\r\n");
    console_prompt_hold(&p, sink, false);
    CHECK_OUT("> \rscan -50 1 wpa2 Lab\r\nscan done 1\r\n> st");
    /* Released after a cut line: the prompt goes on a line of its own. */
    clear();
    console_prompt_hold(&p, sink, true);
    out(&p, "partial");
    console_prompt_hold(&p, sink, false);
    CHECK_OUT("\r\npartial\r\n> st");
    /* Held while the prompt is down (the command's own reply): nothing extra. */
    char line[CONSOLE_LINE_MAX];
    console_prompt_enter(&p, sink, line, sizeof(line));
    clear();
    console_prompt_hold(&p, sink, true);
    out(&p, "scan done 0\r\n");
    console_prompt_hold(&p, sink, false);
    CHECK_OUT("scan done 0\r\n");
}

static void test_editing(void) {
    console_prompt_t p = {0};
    clear();
    console_prompt_show(&p, sink);
    console_prompt_backspace(&p, sink);        /* nothing to erase: nothing printed */
    type(&p, "ab");
    console_prompt_backspace(&p, sink);
    CHECK_OUT("> ab\b \b");
    CHECK(p.len == 1);
    /* The line is capped at CONSOLE_LINE_MAX - 1. */
    for (int i = 0; i < CONSOLE_LINE_MAX + 10; i++) console_prompt_char(&p, sink, 'x');
    CHECK(p.len == CONSOLE_LINE_MAX - 1);
    char line[CONSOLE_LINE_MAX];
    CHECK(console_prompt_enter(&p, sink, line, sizeof(line)) == CONSOLE_LINE_MAX - 1);
    CHECK(strlen(line) == CONSOLE_LINE_MAX - 1);
}

static void test_empty_write(void) {
    console_prompt_t p = {0};
    clear();
    console_prompt_show(&p, sink);
    console_prompt_out(&p, sink, "", 0);
    CHECK_OUT("> ");
}

int main(void) {
    test_log_after_bare_prompt();
    test_log_while_typing();
    test_command_output_untouched();
    test_log_in_pieces();
    test_enter_mid_log();
    test_async_reply_lines();
    test_held_reply();
    test_editing();
    test_empty_write();
    if (failures) { printf("FAIL test_console_prompt: %d failure(s)\n", failures); return 1; }
    printf("PASS test_console_prompt\n");
    return 0;
}
