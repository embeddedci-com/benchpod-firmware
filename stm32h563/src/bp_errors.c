/* bp_errors.c: the reply line for a stable error code (bp_errors.h, errors/catalog.toml).

   {"status":"error","code":"BP0412","args":{"la":4,"owner":"gpio"},"message":"..."}\n

   "message" is the handler's text when the build keeps it (BP_ERR_TEXT), else the compact
   fallback "BP0412 la=4 owner=gpio", built here from the code and args so the firmware holds no
   per-error text. Old hosts that only read "message" still get something to look up; new hosts
   read "code" and "args" and render the text and the fix from errors/errors.json. "args" is left
   out when a code has none. Pure: no I/O, host-tested in test/test_bp_errors.c. */
#include "bp_errors.h"

#include <stdio.h>

#include "bp_json.h"

static void emit_args(bp_emit_t *e, const bp_err_arg_t *args, size_t n) {
    bp_emit_raw(e, ",\"args\":{");
    for (size_t i = 0; i < n; i++) {
        bp_emit(e, "%s\"%s\":", i ? "," : "", args[i].key);
        if (args[i].is_str) bp_emit_jstr(e, args[i].s ? args[i].s : "");
        else                bp_emit(e, "%ld", (long)args[i].i);
    }
    bp_emit_raw(e, "}");
}

/* "BP0412 la=4 owner=gpio" into e as a JSON string. */
static void emit_fallback(bp_emit_t *e, bp_errcode_t code, const bp_err_arg_t *args, size_t n) {
    char text[160];
    size_t len = (size_t)snprintf(text, sizeof(text), "BP%04u", (unsigned)code);
    for (size_t i = 0; i < n && len < sizeof(text); i++) {
        if (args[i].is_str)
            len += (size_t)snprintf(text + len, sizeof(text) - len, " %s=%s", args[i].key,
                                    args[i].s ? args[i].s : "");
        else
            len += (size_t)snprintf(text + len, sizeof(text) - len, " %s=%ld", args[i].key,
                                    (long)args[i].i);
    }
    bp_emit_jstr(e, text);   /* snprintf truncated it to fit: still a valid string */
}

size_t bp_error_format(char *buf, size_t cap, bp_errcode_t code, const char *text,
                       const bp_err_arg_t *args, size_t n) {
    bp_emit_t e;
    bp_emit_init(&e, buf, cap);
    bp_emit(&e, "{\"status\":\"error\",\"code\":\"BP%04u\"", (unsigned)code);
    if (n) emit_args(&e, args, n);
    bp_emit_raw(&e, ",\"message\":");
    if (text) bp_emit_jstr(&e, text);
    else      emit_fallback(&e, code, args, n);
    bp_emit_raw(&e, "}\n");
    if (bp_emit_ok(&e)) return bp_emit_len(&e);
    /* Too long (a huge string arg): drop the text and the args, keep the code. Never a torn line:
       the client would block on the missing newline. */
    bp_emit_init(&e, buf, cap);
    bp_emit(&e, "{\"status\":\"error\",\"code\":\"BP%04u\",\"message\":\"BP%04u\"}\n", (unsigned)code,
            (unsigned)code);
    return bp_emit_ok(&e) ? bp_emit_len(&e) : 0;
}
