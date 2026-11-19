/*
 * console_prompt.h — the console's prompt and line editor, prompt-safe for background output
 * (pure C, host-tested; console_io.c runs it under the console lock).
 *
 * The console prints "> " and waits for a line. Log lines from other tasks (cloud, Wi-Fi, DHCP,
 * the LAN command trace) and late async replies (wifi-scan, flash-esp32) used to land after the
 * prompt, so a client waiting for "> " saw the prompt followed by a log line, and a line the user
 * was typing got split. Now, like a shell:
 *
 *   - output while the prompt is up starts on a line of its own: a bare prompt is overwritten
 *     ("\r", the line holds only "> "), a prompt with typed text is left as it is ("\r\n");
 *   - once that output ends a line, the prompt and the typed text are printed again, so the
 *     console always ends with "> " plus what the user typed;
 *   - characters typed while a background line is half written are kept (not echoed into the
 *     middle of it) and appear with the reprinted prompt.
 *
 * Command output is not affected: the prompt is down from Enter until the command's reply has
 * been printed (console_prompt_show).
 */
#ifndef CONSOLE_PROMPT_H
#define CONSOLE_PROMPT_H

#include <stdbool.h>
#include <stddef.h>

#define CONSOLE_PROMPT     "> "
#define CONSOLE_LINE_MAX   128

/* Writes bytes to the console as they are (the caller adds CR before LF where it wants). */
typedef void (*console_sink_t)(const char *s, size_t n);

typedef struct {
    bool   shown;      /* the prompt is up: the console waits for a line */
    bool   mid;        /* background output started after the prompt and has not ended a line */
    size_t len;
    char   line[CONSOLE_LINE_MAX];
} console_prompt_t;

/* Print the prompt (empty line) and put it up. */
void console_prompt_show(console_prompt_t *p, console_sink_t sink);
/* Output while the prompt is down goes out as it is; while it is up, as described above. */
void console_prompt_out(console_prompt_t *p, console_sink_t sink, const char *s, size_t n);

/* Line editing (the console task). A printable character: kept and echoed. */
void console_prompt_char(console_prompt_t *p, console_sink_t sink, char c);
void console_prompt_backspace(console_prompt_t *p, console_sink_t sink);
/* Enter: ends the line ("\r\n"), takes the prompt down and copies the line (NUL-terminated,
   cap >= CONSOLE_LINE_MAX) to out. Returns its length. */
size_t console_prompt_enter(console_prompt_t *p, console_sink_t sink, char *out, size_t cap);

#endif /* CONSOLE_PROMPT_H */
