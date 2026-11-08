/* c3_restarts.c: see c3_restarts.h. */
#include "c3_restarts.h"
#include <stdio.h>
#include <string.h>

#define C3_RESTARTS_MAGIC 0x43335231u   /* "C3R1": .noinit holds random bits after power-on */

static const char *const k_names[C3_RESTART_CAUSES] = {
    [C3_RESTART_NO_RESPONSE] = "no_response",
    [C3_RESTART_REBOOTED]    = "rebooted",
    [C3_RESTART_NO_BOOT]     = "no_boot",
    [C3_RESTART_SETUP]       = "setup",
    [C3_RESTART_ASSOC]       = "assoc",
    [C3_RESTART_FLASH]       = "flash",
};

static const char *const k_words[C3_RESTART_CAUSES] = {
    [C3_RESTART_NO_RESPONSE] = "no response",
    [C3_RESTART_REBOOTED]    = "rebooted",
    [C3_RESTART_NO_BOOT]     = "no boot",
    [C3_RESTART_SETUP]       = "setup",
    [C3_RESTART_ASSOC]       = "association",
    [C3_RESTART_FLASH]       = "flash",
};

const char *c3_restart_cause_name(c3_restart_cause_t cause) {
    return (unsigned)cause < C3_RESTART_CAUSES ? k_names[cause] : "unknown";
}

/* FNV-1a over the record up to `check`. */
uint32_t c3_restarts_check(const c3_restarts_t *r) {
    const uint8_t *p = (const uint8_t *)r;
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < offsetof(c3_restarts_t, check); i++) { h ^= p[i]; h *= 16777619u; }
    return h;
}

static void seal(c3_restarts_t *r) { r->check = c3_restarts_check(r); }

static bool valid(const c3_restarts_t *r) {
    if (r->magic != C3_RESTARTS_MAGIC || r->check != c3_restarts_check(r)) return false;
    uint32_t sum = 0;
    for (unsigned i = 0; i < C3_RESTART_CAUSES; i++) sum += r->count[i];
    if (sum != r->total) return false;
    if (r->last_reason[C3_RESTART_REASON_LEN - 1] != '\0') return false;
    return r->total == 0 || r->last_cause < C3_RESTART_CAUSES;
}

void c3_restarts_boot(c3_restarts_t *r, bool power_on) {
    if (power_on || !valid(r)) {
        memset(r, 0, sizeof(*r));
        r->magic = C3_RESTARTS_MAGIC;
    }
    r->boots++;
    seal(r);
}

void c3_restarts_note(c3_restarts_t *r, c3_restart_cause_t cause, const char *why, uint32_t uptime_s) {
    if ((unsigned)cause >= C3_RESTART_CAUSES) cause = C3_RESTART_SETUP;
    if (!valid(r)) c3_restarts_boot(r, true);   /* never booted, or overwritten: start over */
    r->count[cause]++;
    r->total++;
    r->last_cause = (uint8_t)cause;
    r->last_uptime_s = uptime_s;
    r->last_boot = r->boots;
    size_t i = 0;
    if (why) {
        for (; i + 1 < C3_RESTART_REASON_LEN && why[i]; i++) {
            const char c = why[i];
            r->last_reason[i] = (c == '"' || c == '\\' || (unsigned char)c < 0x20) ? '\'' : c;
        }
    }
    memset(r->last_reason + i, 0, C3_RESTART_REASON_LEN - i);
    seal(r);
}

size_t c3_restarts_json(const c3_restarts_t *r, char *buf, size_t cap) {
    if (!cap) return 0;
    int n = snprintf(buf, cap, "{\"total\":%lu", (unsigned long)r->total);
    for (unsigned i = 0; i < C3_RESTART_CAUSES && n > 0 && (size_t)n < cap; i++)
        n += snprintf(buf + n, cap - (size_t)n, ",\"%s\":%lu", k_names[i], (unsigned long)r->count[i]);
    if (n > 0 && (size_t)n < cap) {
        if (r->total)
            n += snprintf(buf + n, cap - (size_t)n,
                          ",\"last\":\"%s\",\"last_cause\":\"%s\",\"last_uptime_s\":%lu,"
                          "\"last_this_boot\":%s",
                          r->last_reason, c3_restart_cause_name((c3_restart_cause_t)r->last_cause),
                          (unsigned long)r->last_uptime_s, r->last_boot == r->boots ? "true" : "false");
    }
    if (n > 0 && (size_t)n < cap)
        n += snprintf(buf + n, cap - (size_t)n, ",\"boots\":%lu}", (unsigned long)r->boots);
    if (n <= 0 || (size_t)n >= cap) { buf[0] = '\0'; return 0; }
    return (size_t)n;
}

size_t c3_restarts_describe(const c3_restarts_t *r, char *buf, size_t cap) {
    if (!cap) return 0;
    int n = snprintf(buf, cap, "C3 restarts since power-on: %lu", (unsigned long)r->total);
    if (r->total && n > 0 && (size_t)n < cap) {
        bool first = true;
        for (unsigned i = 0; i < C3_RESTART_CAUSES && n > 0 && (size_t)n < cap; i++) {
            if (!r->count[i]) continue;
            n += snprintf(buf + n, cap - (size_t)n, "%s%s %lu", first ? " (" : ", ", k_words[i],
                          (unsigned long)r->count[i]);
            first = false;
        }
        if (n > 0 && (size_t)n < cap)
            n += snprintf(buf + n, cap - (size_t)n, "), last: %s, at %lu s %s", r->last_reason,
                          (unsigned long)r->last_uptime_s,
                          r->last_boot == r->boots ? "this boot" : "in an earlier boot");
    }
    if (n <= 0) { buf[0] = '\0'; return 0; }
    if ((size_t)n >= cap) return cap - 1;
    return (size_t)n;
}
