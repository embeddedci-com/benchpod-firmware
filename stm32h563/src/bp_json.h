#ifndef BP_JSON_H
#define BP_JSON_H

/*
 * bp_json — the one flat-JSON parser + bounds-tracked emitter for the firmware.
 *
 * Replaces the three hand-rolled parsers that used to live in command_handler.c
 * (json_get_value / json_get_byte_array), cloud_client.c (cl_json_str /
 * cl_json_object) and their per-file copies.  The parser matches a key only at a
 * real object-key position (a quoted name followed by ':'), so a value that
 * happens to equal the key text is never a false hit, and it honours backslash
 * escapes inside string values so an escaped quote no longer truncates the value.
 *
 * The emitter is a bounds-tracked string builder: on overflow it latches ok=false
 * and stops writing, so a reply that does not fit is reported as a failure instead
 * of being silently clipped into malformed JSON.  bp_emit_jstr() escapes arbitrary
 * text into a JSON string, which is what lets error messages carry quotes safely.
 *
 * The plain getters are FLAT: they scan for the first "key": anywhere, nesting
 * included.  That is fine for a command object, which is single-level, but NOT for
 * a cloud frame that wraps one — use the _top variants for envelope fields so a
 * carried command's own keys cannot shadow them.  Pure C, no platform deps, so it
 * is unit-tested on the host (test_json.c).
 */
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* ---- parser --------------------------------------------------------------- */

/* Copy the value of `key` (string or scalar) into out as a NUL-terminated string.
   String values are unescaped (\" \\ \/ \n \r \t \b \f, and \uXXXX -> '?').
   Returns true if the key was found (a present empty string "" counts as found);
   false if the key is absent or (for a bare scalar) empty. out is always
   NUL-terminated. */
bool bp_json_get(const char *json, const char *key, char *out, size_t out_len);
/* Like bp_json_get, but reports a value that did not fit: 1 = found and whole, 0 = missing,
   -1 = present but longer than out_len - 1 (out holds the truncated prefix).  For values that
   must not be stored cut short (SSID, password, cloud host). */
int  bp_json_get_fit(const char *json, const char *key, char *out, size_t out_len);

/* Like bp_json_get, but matches ONLY a key of the OUTERMOST object, so a key
   nested inside a member cannot shadow it.  Use this for ENVELOPE fields of a
   frame that CARRIES another object (the cloud frames' "type" / "request_id" /
   "command"): the flat variant is a strstr scan and would happily return a
   nested match instead.  See find_key_top in bp_json.c for the cloud-command
   outage this prevents. */
bool bp_json_get_top(const char *json, const char *key, char *out, size_t out_len);

/* Truthy flag: true when `key` is present and its value begins 1/t/T/y/Y.
   false when absent or false-ish. */
bool bp_json_flag(const char *json, const char *key);

/* Copy the brace-matched object value {...} of `key` (verbatim, including the
   braces) into out.  String/escape aware so a '}' inside a string does not end
   it early.  Returns true on success, false if absent / not an object / too big
   for cap. */
bool bp_json_object(const char *json, const char *key, char *out, size_t cap);

/* Top-level-only variant of bp_json_object (see bp_json_get_top). */
bool bp_json_object_top(const char *json, const char *key, char *out, size_t cap);

/* Parse "key":[b0,b1,...] of up to `cap` unsigned bytes (strtoul base 0, so
   decimal or 0x..).  Writes the parsed count (clamped to cap) to *len.  Returns
   1 when the key is present as an array, else 0 (with *len = 0). */
int bp_json_byte_array(const char *json, const char *key,
                       uint8_t *out, int cap, int *len);

/* ---- emitter -------------------------------------------------------------- */

/* Bounds-tracked builder.  Never overruns buf; sets ok=false the first time an
   append would not fit (and thereafter is a no-op), so callers can gate the send
   on bp_emit_ok(). */
typedef struct {
    char  *buf;
    size_t cap;   /* total buffer size incl. the NUL slot */
    size_t len;   /* bytes written so far (excl. NUL)      */
    bool   ok;    /* false once anything was truncated     */
} bp_emit_t;

void bp_emit_init(bp_emit_t *e, char *buf, size_t cap);
/* printf-style append for literal/number fields (no untrusted %s — use
   bp_emit_jstr for arbitrary text). */
void bp_emit(bp_emit_t *e, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)));
/* Append a raw, already-valid fragment verbatim. */
void bp_emit_raw(bp_emit_t *e, const char *s);
/* Append `s` as a quoted, escaped JSON string value (surrounding quotes
   included), so arbitrary text can't break out of the string. */
void bp_emit_jstr(bp_emit_t *e, const char *s);

static inline bool   bp_emit_ok(const bp_emit_t *e)  { return e->ok; }
static inline size_t bp_emit_len(const bp_emit_t *e) { return e->len; }

#endif /* BP_JSON_H */
