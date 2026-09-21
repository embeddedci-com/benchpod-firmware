/*
 * la_pins.c — LA pin ownership table + pull compatibility + the exact client-facing messages.
 * See la_pins.h.  Pure C (no HAL), host-tested in test/test_la_pins.c.
 */
#include "la_pins.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    uint8_t fn;      /* la_fn_t */
    uint8_t gpio;    /* la_gpio_mode_t, meaningful only when fn == LA_FN_GPIO */
    uint8_t level;   /* commanded level for gpio output / open_drain */
} la_pin_t;

static la_pin_t s_pin[LA_PINS_COUNT + 1];   /* index 1..12 */

static bool la_ok(unsigned la) { return la >= 1u && la <= LA_PINS_COUNT; }

void la_pins_reset(void) {
    memset(s_pin, 0, sizeof(s_pin));
}

const char *la_fn_name(la_fn_t fn) {
    static const char *const names[LA_FN__COUNT] = {
        "none", "gpio", "uart_rx", "uart_tx", "swd_clk", "swd_dio",
        "i2c_sda", "i2c_scl", "step", "step_dir",
    };
    return ((unsigned)fn < (unsigned)LA_FN__COUNT) ? names[fn] : "none";
}

const char *la_gpio_mode_name(la_gpio_mode_t m) {
    switch (m) {
    case LA_GPIO_INPUT:      return "input";
    case LA_GPIO_OUTPUT:     return "output";
    case LA_GPIO_OPEN_DRAIN: return "open_drain";
    default:                 return "";
    }
}

bool la_gpio_mode_parse(const char *s, la_gpio_mode_t *out) {
    if (!s || !out) return false;
    if (strcmp(s, "input") == 0)      { *out = LA_GPIO_INPUT;      return true; }
    if (strcmp(s, "output") == 0)     { *out = LA_GPIO_OUTPUT;     return true; }
    if (strcmp(s, "open_drain") == 0) { *out = LA_GPIO_OPEN_DRAIN; return true; }
    if (strcmp(s, "off") == 0)        { *out = LA_GPIO_NONE;       return true; }
    return false;
}

la_fn_t la_pins_fn(unsigned la) {
    return la_ok(la) ? (la_fn_t)s_pin[la].fn : LA_FN_NONE;
}

la_gpio_mode_t la_pins_gpio(unsigned la) {
    if (!la_ok(la) || s_pin[la].fn != LA_FN_GPIO) return LA_GPIO_NONE;
    return (la_gpio_mode_t)s_pin[la].gpio;
}

int la_pins_level(unsigned la) {
    la_gpio_mode_t m = la_pins_gpio(la);
    if (m != LA_GPIO_OUTPUT && m != LA_GPIO_OPEN_DRAIN) return -1;
    return s_pin[la].level ? 1 : 0;
}

uint16_t la_pins_mask_fn(la_fn_t fn) {
    uint16_t mask = 0;
    for (unsigned la = 1; la <= LA_PINS_COUNT; la++)
        if (s_pin[la].fn == (uint8_t)fn) mask |= (uint16_t)(1u << (la - 1u));
    return mask;
}

uint16_t la_pins_owned_mask(void) {
    return (uint16_t)(~la_pins_mask_fn(LA_FN_NONE) & 0x0FFFu);
}

la_pull_dir_t la_pull_dir(unsigned la) {
    if (la >= 1u && la <= 6u) return LA_PULL_UP;
    if (la == 7u || la == 8u) return LA_PULL_DOWN;
    return LA_PULL_NONE;
}

const char *la_pull_ohms(unsigned la) {
    static const char *const ohms[9] = { NULL, "4.7k", "4.7k", "2.2k", "2.2k", "10k", "10k", "10k", "10k" };
    return (la >= 1u && la <= 8u) ? ohms[la] : NULL;
}

int la_pins_drive(unsigned la) {
    switch (la_pins_gpio(la)) {
    case LA_GPIO_OUTPUT:     return s_pin[la].level ? 1 : 0;
    case LA_GPIO_OPEN_DRAIN: return s_pin[la].level ? 2 : 0;   /* 1 = released */
    default:                 return 2;
    }
}

/* ---- messages ---- */

static const char *release_hint(la_fn_t fn, unsigned la, char *buf, size_t cap) {
    switch (fn) {
    case LA_FN_GPIO:
        snprintf(buf, cap, "release it with {\"cmd\":\"gpio\",\"la\":%u,\"mode\":\"off\"}", la);
        return buf;
    case LA_FN_UART_RX: case LA_FN_UART_TX: return "stop the uart proxy first";
    case LA_FN_SWD_CLK: case LA_FN_SWD_DIO: return "end the SWD session first";
    case LA_FN_I2C_SDA: case LA_FN_I2C_SCL: return "stop the sensor emulation first ({\"cmd\":\"sensor_stop\"})";
    case LA_FN_STEP:    case LA_FN_STEP_DIR: return "wait for the step train to finish";
    default: return "";
    }
}

static void conflict_msg(unsigned la, char *err, size_t cap) {
    char hint[64];
    la_fn_t owner = (la_fn_t)s_pin[la].fn;
    snprintf(err, cap, "pin conflict: LA%u is in use by %s; %s",
             la, la_fn_name(owner), release_hint(owner, la, hint, sizeof(hint)));
}

/* Why a function can't share its line with an engaged pull-DOWN (NULL = it can).  Pull-ups
   never conflict: every function here either drives the line or wants it idling high. */
static const char *pull_down_reason(la_fn_t fn, la_gpio_mode_t mode) {
    switch (fn) {
    case LA_FN_GPIO:
        return (mode == LA_GPIO_OPEN_DRAIN) ? "a released line would read low" : NULL;
    case LA_FN_UART_RX: case LA_FN_UART_TX: return "the line idles high";
    case LA_FN_I2C_SDA: case LA_FN_I2C_SCL: return "an open-drain bus needs pull-ups";
    case LA_FN_SWD_DIO: return "SWDIO is pulled up when released";
    default: return NULL;   /* none, gpio input/output, swd_clk, step, step_dir */
    }
}

/* "gpio open_drain" for a gpio pin (the mode is what conflicts), else the function name. */
static const char *fn_display(la_fn_t fn, la_gpio_mode_t mode, char *buf, size_t cap) {
    if (fn != LA_FN_GPIO) return la_fn_name(fn);
    snprintf(buf, cap, "gpio %s", la_gpio_mode_name(mode));
    return buf;
}

bool la_pins_check_claim(la_fn_t fn, la_gpio_mode_t mode, const uint8_t *las, size_t n,
                         uint16_t free_fns, uint16_t pull_mask, char *err, size_t cap) {
    for (size_t i = 0; i < n; i++) {
        unsigned la = las[i];
        if (!la_ok(la)) { snprintf(err, cap, "LA%u is not a pin (use 1..12)", la); return false; }
        la_fn_t owner = (la_fn_t)s_pin[la].fn;
        if (owner != LA_FN_NONE && owner != fn && !(free_fns & LA_FN_BIT(owner))) {
            conflict_msg(la, err, cap);
            return false;
        }
        /* A pin already holding this very function (a second uart_proxy_start on the proxy's own
           pins, say) is a conflict too, unless the caller says it replaces it. */
        if (owner == fn && owner != LA_FN_NONE && !(free_fns & LA_FN_BIT(owner))) {
            conflict_msg(la, err, cap);
            return false;
        }
    }
    const char *reason = pull_down_reason(fn, mode);
    if (!reason) return true;
    for (size_t i = 0; i < n; i++) {
        unsigned la = las[i];
        if (la_pull_dir(la) != LA_PULL_DOWN || !(pull_mask & (1u << (la - 1u)))) continue;
        char name[24];
        snprintf(err, cap,
                 "pull conflict: LA%u has its %s pull-down engaged, which %s can't work with (%s); "
                 "disable it with {\"cmd\":\"la\",\"la\":%u,\"pullup\":\"off\"} or use another channel",
                 la, la_pull_ohms(la), fn_display(fn, mode, name, sizeof(name)), reason, la);
        return false;
    }
    return true;
}

bool la_pins_check_pull_enable(unsigned la, char *err, size_t cap) {
    if (!la_ok(la) || la_pull_dir(la) != LA_PULL_DOWN) return true;
    la_fn_t fn = (la_fn_t)s_pin[la].fn;
    la_gpio_mode_t mode = la_pins_gpio(la);
    const char *reason = pull_down_reason(fn, mode);
    if (!reason) return true;
    char name[24];
    snprintf(err, cap, "pull conflict: LA%u is used by %s, which can't work with the %s pull-down (%s)",
             la, fn_display(fn, mode, name, sizeof(name)), la_pull_ohms(la), reason);
    return false;
}

bool la_pins_check_level(const uint8_t *las, size_t n, char *err, size_t cap) {
    for (size_t i = 0; i < n; i++) {
        unsigned la = las[i];
        if (!la_ok(la)) { snprintf(err, cap, "LA%u is not a pin (use 1..12)", la); return false; }
        la_gpio_mode_t m = la_pins_gpio(la);
        if (m == LA_GPIO_OUTPUT || m == LA_GPIO_OPEN_DRAIN) continue;
        la_fn_t fn = (la_fn_t)s_pin[la].fn;
        if (fn == LA_FN_GPIO)
            snprintf(err, cap, "LA%u is not a gpio output (function gpio, gpio %s); "
                               "configure it with {\"cmd\":\"gpio\",\"la\":%u,\"mode\":\"output\"}",
                     la, la_gpio_mode_name(m), la);
        else
            snprintf(err, cap, "LA%u is not a gpio output (function %s); "
                               "configure it with {\"cmd\":\"gpio\",\"la\":%u,\"mode\":\"output\"}",
                     la, la_fn_name(fn), la);
        return false;
    }
    return true;
}

bool la_pins_check_voltage_change(int cur_mv, int new_mv, char *err, size_t cap) {
    if (new_mv == cur_mv) return true;
    if (new_mv != 1800 && new_mv != 3300) return true;   /* the caller's own validation says why */
    uint16_t owned = la_pins_owned_mask();
    if (!owned) return true;
    size_t pos = (size_t)snprintf(err, cap, "la voltage can't change while pins are in use: ");
    bool first = true;
    for (unsigned la = 1; la <= LA_PINS_COUNT && pos < cap; la++) {
        if (!(owned & (1u << (la - 1u)))) continue;
        pos += (size_t)snprintf(err + pos, cap - pos, "%sLA%u (%s)", first ? "" : ", ",
                                la, la_fn_name((la_fn_t)s_pin[la].fn));
        first = false;
    }
    if (pos < cap) snprintf(err + pos, cap - pos, "; stop them first");
    return false;
}

/* ---- commits ---- */

void la_pins_claim(la_fn_t fn, la_gpio_mode_t mode, uint8_t level, const uint8_t *las, size_t n) {
    for (size_t i = 0; i < n; i++) {
        unsigned la = las[i];
        if (!la_ok(la)) continue;
        s_pin[la].fn    = (uint8_t)fn;
        s_pin[la].gpio  = (fn == LA_FN_GPIO) ? (uint8_t)mode : (uint8_t)LA_GPIO_NONE;
        s_pin[la].level = level ? 1u : 0u;
    }
}

void la_pins_set_level(unsigned la, uint8_t level) {
    if (la_ok(la)) s_pin[la].level = level ? 1u : 0u;
}

void la_pins_release(unsigned la) {
    if (la_ok(la)) memset(&s_pin[la], 0, sizeof(s_pin[la]));
}

uint16_t la_pins_release_fn(la_fn_t fn) {
    uint16_t mask = la_pins_mask_fn(fn);
    if (fn == LA_FN_NONE) return 0;
    for (unsigned la = 1; la <= LA_PINS_COUNT; la++)
        if (mask & (1u << (la - 1u))) la_pins_release(la);
    return mask;
}

/* ---- gpio command ---- */

/* Locate the value of "key" in a flat command object (NULL when absent).  bp_json_get stops a
   bare value at the first ',', which would cut "la":[1,2] short — hence this local scan. */
static const char *find_value(const char *json, const char *key) {
    char needle[24];
    snprintf(needle, sizeof(needle), "\"%s\"", key);
    const char *p = json;
    while ((p = strstr(p, needle)) != NULL) {
        const char *q = p + strlen(needle);
        while (*q == ' ' || *q == '\t') q++;
        if (*q == ':') {
            q++;
            while (*q == ' ' || *q == '\t') q++;
            return q;
        }
        p = q;
    }
    return NULL;
}

static bool add_pin(la_gpio_req_t *req, long v, char *err, size_t cap) {
    if (v < 1 || v > (long)LA_PINS_COUNT) {
        snprintf(err, cap, "la must be 1..12 (or \"all\" with \"mode\":\"off\")");
        return false;
    }
    for (size_t i = 0; i < req->n; i++) if (req->las[i] == (uint8_t)v) return true;   /* dedupe */
    req->las[req->n++] = (uint8_t)v;
    return true;
}

bool la_gpio_req_parse(const char *json, la_gpio_req_t *req, char *err, size_t cap) {
    memset(req, 0, sizeof(*req));
    char s[16];

    if (bp_json_get(json, "mode", s, sizeof(s)) && strcmp(s, "null") != 0) {
        if (!la_gpio_mode_parse(s, &req->mode)) {
            snprintf(err, cap, "mode must be input, output, open_drain or off");
            return false;
        }
        req->has_mode = true;
    }
    if (bp_json_get(json, "level", s, sizeof(s)) && strcmp(s, "null") != 0) {
        if (strcmp(s, "0") != 0 && strcmp(s, "1") != 0) {
            snprintf(err, cap, "level must be 0 or 1");
            return false;
        }
        req->has_level = true;
        req->level     = (uint8_t)(s[0] - '0');
    }

    const char *v = find_value(json, "la");
    if (!v || strncmp(v, "null", 4) == 0) {
        if (req->has_mode || req->has_level) { snprintf(err, cap, "missing la"); return false; }
        return true;   /* a read */
    }
    if (*v == '[') {
        const char *p = v + 1;
        for (;;) {
            while (*p == ' ' || *p == '\t' || *p == ',') p++;
            if (*p == ']') break;
            char *end;
            long n = strtol(p, &end, 10);
            if (end == p) { snprintf(err, cap, "la must be a pin number, a list of pin numbers, or \"all\""); return false; }
            if (!add_pin(req, n, err, cap)) return false;
            p = end;
        }
        if (req->n == 0) { snprintf(err, cap, "la list is empty"); return false; }
        return true;
    }
    if (*v == '"') {
        if (strncmp(v, "\"all\"", 5) == 0) { req->all = true; return true; }
        snprintf(err, cap, "la must be a pin number, a list of pin numbers, or \"all\"");
        return false;
    }
    char *end;
    long n = strtol(v, &end, 10);
    if (end == v) { snprintf(err, cap, "la must be a pin number, a list of pin numbers, or \"all\""); return false; }
    return add_pin(req, n, err, cap);
}

static uint16_t mask_of(const uint8_t *las, size_t n) {
    uint16_t m = 0;
    for (size_t i = 0; i < n; i++) if (la_ok(las[i])) m |= (uint16_t)(1u << (las[i] - 1u));
    return m;
}

bool la_pins_gpio_apply(const la_gpio_req_t *req, uint16_t pull_mask,
                        uint16_t *changed, uint16_t *affected, char *err, size_t cap) {
    *changed = 0;
    *affected = 0;

    if (!req->has_mode) {
        if (!req->has_level) { snprintf(err, cap, "gpio needs a mode or a level"); return false; }
        if (req->all) { snprintf(err, cap, "\"la\":\"all\" is only allowed with \"mode\":\"off\""); return false; }
        if (!la_pins_check_level(req->las, req->n, err, cap)) return false;
        for (size_t i = 0; i < req->n; i++) la_pins_set_level(req->las[i], req->level);
        *changed = *affected = mask_of(req->las, req->n);
        return true;
    }

    if (req->mode == LA_GPIO_NONE) {   /* "off" */
        if (req->all) {
            *changed = *affected = la_pins_release_fn(LA_FN_GPIO);
            return true;
        }
        /* Releasing a pin some other function owns would make the table lie about the wire. */
        for (size_t i = 0; i < req->n; i++) {
            la_fn_t fn = la_pins_fn(req->las[i]);
            if (fn != LA_FN_NONE && fn != LA_FN_GPIO) { conflict_msg(req->las[i], err, cap); return false; }
        }
        for (size_t i = 0; i < req->n; i++) {
            if (la_pins_fn(req->las[i]) == LA_FN_GPIO) {
                la_pins_release(req->las[i]);
                *changed |= (uint16_t)(1u << (req->las[i] - 1u));
            }
        }
        *affected = mask_of(req->las, req->n);
        return true;
    }

    if (req->all) { snprintf(err, cap, "\"la\":\"all\" is only allowed with \"mode\":\"off\""); return false; }
    if (req->mode == LA_GPIO_INPUT && req->has_level) {
        snprintf(err, cap, "level applies to output and open_drain pins, not input");
        return false;
    }
    if (!la_pins_check_claim(LA_FN_GPIO, req->mode, req->las, req->n, LA_FN_BIT(LA_FN_GPIO),
                             pull_mask, err, cap))
        return false;
    uint8_t level = req->has_level ? req->level : (req->mode == LA_GPIO_OPEN_DRAIN ? 1u : 0u);
    la_pins_claim(LA_FN_GPIO, req->mode, level, req->las, req->n);
    *changed = *affected = mask_of(req->las, req->n);
    return true;
}

/* ---- step trains ---- */

bool la_pins_plan_step(unsigned la, unsigned dir_la, la_step_plan_t *plan, char *err, size_t cap) {
    plan->claim_step = false;
    plan->claim_dir  = false;
    if (!la_ok(la)) { snprintf(err, cap, "invalid args"); return false; }
    if (dir_la != 0u && !la_ok(dir_la)) { snprintf(err, cap, "invalid dir_la"); return false; }
    if (dir_la == la) { snprintf(err, cap, "dir_la must be a different pin than la"); return false; }

    la_fn_t fn = la_pins_fn(la);
    if (fn == LA_FN_NONE) plan->claim_step = true;
    else if (!(fn == LA_FN_GPIO && la_pins_gpio(la) == LA_GPIO_OUTPUT)) { conflict_msg(la, err, cap); return false; }

    if (dir_la != 0u) {
        la_fn_t dfn = la_pins_fn(dir_la);
        if (dfn == LA_FN_NONE) plan->claim_dir = true;
        else if (!(dfn == LA_FN_GPIO && la_pins_gpio(dir_la) == LA_GPIO_OUTPUT)) { conflict_msg(dir_la, err, cap); return false; }
    }
    return true;
}

/* ---- JSON ---- */

void la_pins_emit_entry(bp_emit_t *e, unsigned la, uint16_t pull_mask) {
    la_gpio_mode_t m = la_pins_gpio(la);
    int level = la_pins_level(la);
    bp_emit(e, "{\"la\":%u,\"function\":\"%s\",\"gpio\":", la, la_fn_name(la_pins_fn(la)));
    if (m != LA_GPIO_NONE) bp_emit(e, "\"%s\"", la_gpio_mode_name(m));
    else                   bp_emit_raw(e, "null");
    if (level >= 0) bp_emit(e, ",\"level\":%d,\"pull\":", level);
    else            bp_emit_raw(e, ",\"level\":null,\"pull\":");
    const char *ohms = la_pull_ohms(la);
    if (ohms)
        bp_emit(e, "{\"dir\":\"%s\",\"ohms\":\"%s\",\"on\":%s}}",
                la_pull_dir(la) == LA_PULL_DOWN ? "down" : "up", ohms,
                (pull_mask & (1u << (la - 1u))) ? "true" : "false");
    else
        bp_emit_raw(e, "null}");
}

void la_pins_emit_list(bp_emit_t *e, uint16_t mask, uint16_t pull_mask) {
    bp_emit_raw(e, "[");
    bool first = true;
    for (unsigned la = 1; la <= LA_PINS_COUNT; la++) {
        if (!(mask & (1u << (la - 1u)))) continue;
        if (!first) bp_emit_raw(e, ",");
        la_pins_emit_entry(e, la, pull_mask);
        first = false;
    }
    bp_emit_raw(e, "]");
}

/* ---- capture trigger ---- */

const char *la_edge_name(la_edge_t edge) {
    switch (edge) {
    case LA_EDGE_RISING:  return "rising";
    case LA_EDGE_FALLING: return "falling";
    case LA_EDGE_HIGH:    return "high";
    case LA_EDGE_LOW:     return "low";
    default:              return "";
    }
}

bool la_trigger_parse(const char *json, la_trigger_t *out, char *err, size_t cap) {
    memset(out, 0, sizeof(*out));
    out->timeout_ms = LA_TRIGGER_TIMEOUT_DEFAULT_MS;
    char probe[8];
    if (!bp_json_get(json, "trigger", probe, sizeof(probe)) || strcmp(probe, "null") == 0)
        return true;

    char obj[128];
    if (!bp_json_object(json, "trigger", obj, sizeof(obj))) {
        snprintf(err, cap, "trigger must be an object like {\"la\":1,\"edge\":\"rising\"}");
        return false;
    }
    char s[16];
    char *end = NULL;
    long la = bp_json_get(obj, "la", s, sizeof(s)) ? strtol(s, &end, 10) : 0;
    if (!end || end == s || *end != '\0' || la < 1 || la > (long)LA_PINS_COUNT) {
        snprintf(err, cap, "trigger la must be 1..12");
        return false;
    }
    la_edge_t edge = LA_EDGE_RISING;   /* default when "edge" is omitted */
    if (bp_json_get(obj, "edge", s, sizeof(s))) {
        if      (strcmp(s, "rising") == 0)  edge = LA_EDGE_RISING;
        else if (strcmp(s, "falling") == 0) edge = LA_EDGE_FALLING;
        else if (strcmp(s, "high") == 0)    edge = LA_EDGE_HIGH;
        else if (strcmp(s, "low") == 0)     edge = LA_EDGE_LOW;
        else {
            snprintf(err, cap, "trigger edge must be rising, falling, high or low");
            return false;
        }
    }
    if (bp_json_get(json, "trigger_timeout_ms", s, sizeof(s)) && strcmp(s, "null") != 0) {
        end = NULL;
        long ms = strtol(s, &end, 10);
        if (end == s || *end != '\0' || ms < 1 || ms > (long)LA_TRIGGER_TIMEOUT_MAX_MS) {
            snprintf(err, cap, "trigger_timeout_ms must be 1..600000");
            return false;
        }
        out->timeout_ms = (uint32_t)ms;
    }
    out->present = true;
    out->la      = (uint8_t)la;
    out->edge    = edge;
    return true;
}

void la_trigger_timeout_msg(const la_trigger_t *t, char *buf, size_t cap) {
    snprintf(buf, cap, "trigger timeout: no %s edge on LA%u within %lu ms",
             la_edge_name(t->edge), (unsigned)t->la, (unsigned long)t->timeout_ms);
}

void la_trigger_emit(bp_emit_t *e, const la_trigger_t *t, bool fired) {
    bp_emit(e, "\"trigger\":{\"la\":%u,\"edge\":\"%s\",\"fired\":%s}",
            (unsigned)t->la, la_edge_name(t->edge), fired ? "true" : "false");
}
