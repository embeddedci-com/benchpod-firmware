/*
 * console_prompt.c — see console_prompt.h.
 */
#include "console_prompt.h"

#include <string.h>

static void put(console_sink_t sink, const char *s) { sink(s, strlen(s)); }

static void reprint(console_prompt_t *p, console_sink_t sink) {
    put(sink, CONSOLE_PROMPT);
    if (p->len) sink(p->line, p->len);
}

void console_prompt_show(console_prompt_t *p, console_sink_t sink) {
    p->len = 0;
    p->mid = false;
    p->shown = true;
    put(sink, CONSOLE_PROMPT);
}

void console_prompt_out(console_prompt_t *p, console_sink_t sink, const char *s, size_t n) {
    if (n == 0) return;
    if (!p->shown) { sink(s, n); return; }
    if (!p->mid) {
        /* The cursor is after the prompt (and any typed text): start the output on its own line. */
        put(sink, p->len ? "\r\n" : "\r");
        p->mid = true;
    }
    sink(s, n);
    if (s[n - 1] == '\n') {
        p->mid = false;
        reprint(p, sink);
    }
}

void console_prompt_char(console_prompt_t *p, console_sink_t sink, char c) {
    if (p->len >= CONSOLE_LINE_MAX - 1) return;
    p->line[p->len++] = c;
    if (!p->mid) sink(&c, 1);   /* else it shows with the reprinted prompt */
}

void console_prompt_backspace(console_prompt_t *p, console_sink_t sink) {
    if (p->len == 0) return;
    p->len--;
    if (!p->mid) put(sink, "\b \b");
}

size_t console_prompt_enter(console_prompt_t *p, console_sink_t sink, char *out, size_t cap) {
    if (p->mid) {
        /* A background line is half written: finish it, so the typed line shows whole. */
        put(sink, "\r\n");
        p->mid = false;
        reprint(p, sink);
    }
    put(sink, "\r\n");
    size_t n = p->len < cap - 1 ? p->len : cap - 1;
    memcpy(out, p->line, n);
    out[n] = '\0';
    p->len = 0;
    p->shown = false;
    return n;
}
