/*
 * la_pins.c — LA pin ownership table + pull compatibility + the exact client-facing messages.
 * See la_pins.h.  Pure C (no HAL), host-tested in test/test_la_pins.c.
 */
#include "la_pins.h"
#include "board_caps.h"
#include "bp_errors.h"   /* the refusals are catalog codes (bp_tag_*); SCPI shows the text */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    uint8_t fn;      /* la_fn_t */
    uint8_t gpio;    /* la_gpio_mode_t, meaningful only when fn == LA_FN_GPIO */
    uint8_t level;   /* commanded level for gpio output / open_drain */
    uint8_t owner;   /* sensor-manager slots owning the pin (bit per slot), 0 = none */
} la_pin_t;

static la_pin_t s_pin[LA_PINS_COUNT + 1];   /* index 1..14 */
static uint8_t  s_dir[LA_PINS_COUNT + 1];   /* la_dir_t; survives la_pins_reset (it is the wiring) */
static bool     s_powered = true;
static la_owner_namer_t s_namer;

void la_pins_set_owner_namer(la_owner_namer_t namer) { s_namer = namer; }

void la_pins_set_owner(unsigned la, uint8_t owner_mask) {
    if (la >= 1u && la <= LA_PINS_COUNT) s_pin[la].owner = owner_mask;
}

uint8_t la_pins_owner(unsigned la) {
    return (la >= 1u && la <= LA_PINS_COUNT) ? s_pin[la].owner : 0u;
}

bool la_fn_is_sensor(la_fn_t fn) {
    return fn == LA_FN_I2C_SDA || fn == LA_FN_I2C_SCL || (fn >= LA_FN_SENSOR_TX && fn <= LA_FN_SENSOR_CS);
}

/* The first owner's instance name of a pin, or NULL. */
static const char *owner_name(unsigned la) {
    uint8_t m = s_pin[la].owner;
    for (unsigned s = 0; m && s < LA_PINS_OWNER_SLOTS; s++)
        if ((m & (1u << s)) && s_namer) return s_namer(s);
    return NULL;
}

static bool la_ok(unsigned la) { return la >= 1u && la <= LA_PINS_COUNT; }

void la_pins_reset(void) {
    memset(s_pin, 0, sizeof(s_pin));
}

/* ---- what each function does to its line ----
   One row per function: its name, whether it drives the line push-pull (la_fn_drives), whether the
   pod gives it the 2.2k pull (la_pull_strength_for), and why it can't share the line with an
   engaged pull-DOWN (NULL = it can; pull-ups never conflict: every function here either drives
   the line or wants it idling high).  gpio depends on its mode and is handled where it matters. */
#define PD_IDLE_HIGH  "the line idles high"
#define PD_OPEN_DRAIN "an open-drain bus needs pull-ups"
#define PD_SWDIO      "SWDIO is pulled up when released"
typedef struct {
    const char *name;
    bool        drives;
    bool        pull_2k2;
    const char *no_pull_down;
} la_fn_attr_t;
static const la_fn_attr_t s_fn[LA_FN__COUNT] = {
    [LA_FN_NONE]        = { "none",        false, false, NULL },
    [LA_FN_GPIO]        = { "gpio",        false, false, NULL },   /* by mode */
    [LA_FN_UART_RX]     = { "uart_rx",     false, false, PD_IDLE_HIGH },
    [LA_FN_UART_TX]     = { "uart_tx",     true,  false, PD_IDLE_HIGH },
    [LA_FN_SWD_CLK]     = { "swd_clk",     true,  false, NULL },
    [LA_FN_SWD_DIO]     = { "swd_dio",     true,  false, PD_SWDIO },
    [LA_FN_I2C_SDA]     = { "i2c_sda",     false, true,  PD_OPEN_DRAIN },
    [LA_FN_I2C_SCL]     = { "i2c_scl",     false, true,  PD_OPEN_DRAIN },
    [LA_FN_STEP]        = { "step",        true,  false, NULL },
    [LA_FN_STEP_DIR]    = { "step_dir",    true,  false, NULL },
    [LA_FN_SPI_SCK]     = { "spi_sck",     true,  false, NULL },
    [LA_FN_SPI_MOSI]    = { "spi_mosi",    true,  false, NULL },
    [LA_FN_SPI_MISO]    = { "spi_miso",    false, false, NULL },
    [LA_FN_SPI_CS]      = { "spi_cs",      true,  false, NULL },
    [LA_FN_SENSOR_TX]   = { "sensor_tx",   true,  false, PD_IDLE_HIGH },
    [LA_FN_SENSOR_RX]   = { "sensor_rx",   false, false, PD_IDLE_HIGH },
    [LA_FN_SENSOR_DRDY] = { "sensor_drdy", true,  false, NULL },
    [LA_FN_SENSOR_SCK]  = { "sensor_sck",  false, false, NULL },
    [LA_FN_SENSOR_MOSI] = { "sensor_mosi", false, false, NULL },
    [LA_FN_SENSOR_MISO] = { "sensor_miso", true,  false, NULL },
    [LA_FN_SENSOR_CS]   = { "sensor_cs",   false, false, NULL },
    [LA_FN_I2CM_SDA]    = { "i2cm_sda",    false, true,  PD_OPEN_DRAIN },
    [LA_FN_I2CM_SCL]    = { "i2cm_scl",    false, true,  PD_OPEN_DRAIN },
    [LA_FN_UART_HD]     = { "uart_hd",     false, true,  PD_IDLE_HIGH },   /* open drain + RX */
    [LA_FN_UART_DE]     = { "uart_de",     true,  false, NULL },
    [LA_FN_JTAG_TCK]    = { "jtag_tck",    true,  false, NULL },
    [LA_FN_JTAG_TMS]    = { "jtag_tms",    true,  false, NULL },
    [LA_FN_JTAG_TDI]    = { "jtag_tdi",    true,  false, NULL },
    [LA_FN_JTAG_TDO]    = { "jtag_tdo",    false, false, NULL },
    [LA_FN_JTAG_TRST]   = { "jtag_trst",   true,  false, NULL },
};

static const la_fn_attr_t *fn_attr(la_fn_t fn) {
    return ((unsigned)fn < (unsigned)LA_FN__COUNT && s_fn[fn].name) ? &s_fn[fn] : &s_fn[LA_FN_NONE];
}

const char *la_fn_name(la_fn_t fn) { return fn_attr(fn)->name; }

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
    return (uint16_t)(~la_pins_mask_fn(LA_FN_NONE) & 0x3FFFu);
}

/* ---- pull strength by function (the pull tables are in la_pull.c) ---- */

pull_str_t la_pull_strength_for(unsigned la, int io_mv) {
    if (la < 1u || la > LA_PINS_COUNT) return PULL_STR_10K;
    la_fn_t fn = (la_fn_t)s_pin[la].fn;
    la_gpio_mode_t mode = la_pins_gpio(la);
    if (fn_attr(fn)->pull_2k2) return PULL_STR_2K2;   /* I2C, the I2C master, one-wire UART */
    if (fn == LA_FN_GPIO && mode == LA_GPIO_OPEN_DRAIN) return PULL_STR_2K2;
    if (io_mv == 5000 && la_fn_drives(fn, mode)) return PULL_STR_2K2;   /* lift a 3.3 V high to 5 V */
    return PULL_STR_10K;
}

int la_pins_drive(unsigned la) {
    switch (la_pins_gpio(la)) {
    case LA_GPIO_OUTPUT:     return s_pin[la].level ? 1 : 0;
    case LA_GPIO_OPEN_DRAIN: return s_pin[la].level ? 2 : 0;   /* 1 = released */
    default:                 return 2;
    }
}

/* ---- messages ---- */

/* BP0031: who holds LA la. A sensor pin names the instance and its function,
   "sensor:env (i2c_sda)", like la_pins lists it. */
static void conflict_msg(unsigned la, char *err, size_t cap) {
    la_fn_t owner = (la_fn_t)s_pin[la].fn;
    const char *inst = la_fn_is_sensor(owner) ? owner_name(la) : NULL;
    char who[48];
    if (inst) snprintf(who, sizeof(who), "sensor:%s (%s)", inst, la_fn_name(owner));
    else      snprintf(who, sizeof(who), "%s", la_fn_name(owner));
    snprintf(err, cap, "pin conflict: LA%u is in use by %s", la, who);   /* SCPI shows it */
    bp_tag_la_pin_in_use(err, (int32_t)la, who, la_fn_name(owner), inst ? inst : "");
}

/* Why a function can't share its line with an engaged pull-DOWN (NULL = it can). */
static const char *pull_down_reason(la_fn_t fn, la_gpio_mode_t mode) {
    if (fn == LA_FN_GPIO) return (mode == LA_GPIO_OPEN_DRAIN) ? "a released line would read low" : NULL;
    return fn_attr(fn)->no_pull_down;
}

/* "gpio open_drain" for a gpio pin (the mode is what conflicts), else the function name. */
static const char *fn_display(la_fn_t fn, la_gpio_mode_t mode, char *buf, size_t cap) {
    if (fn != LA_FN_GPIO) return la_fn_name(fn);
    snprintf(buf, cap, "gpio %s", la_gpio_mode_name(mode));
    return buf;
}

bool la_pins_check_claim(la_fn_t fn, la_gpio_mode_t mode, const uint8_t *las, size_t n,
                         uint32_t free_fns, uint16_t pull_mask, char *err, size_t cap) {
    for (size_t i = 0; i < n; i++) {
        unsigned la = las[i];
        if (!la_ok(la)) { snprintf(err, cap, "LA%u is not a pin (use 1..14)", la); return false; }
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
    if (la_fn_drives(fn, mode)) {
        for (size_t i = 0; i < n; i++) {
            unsigned la = las[i];
            if (s_dir[la] != LA_DIR_DUT_OUTPUT) continue;
            char name[24];
            const char *what = fn_display(fn, mode, name, sizeof(name));
            snprintf(err, cap, "direction conflict: LA%u is wired to a DUT output and %s would drive "
                     "against it", la, what);
            bp_tag_la_dut_output(err, (int32_t)la, what);
            return false;
        }
    }
    const char *reason = pull_down_reason(fn, mode);
    if (!reason) return true;
    for (size_t i = 0; i < n; i++) {
        unsigned la = las[i];
        if (la_pull_dir(la) != LA_PULL_DOWN || !(pull_mask & (1u << (la - 1u)))) continue;
        char name[24];
        const char *what = fn_display(fn, mode, name, sizeof(name));
        snprintf(err, cap, "pull conflict: LA%u has its %s pull-down on, which %s cannot work with (%s)",
                 la, la_pull_ohms(la), what, reason);
        bp_tag_la_pull_down_engaged(err, (int32_t)la, la_pull_ohms(la), what, reason);
        return false;
    }
    return true;
}

/* ---- direction and warnings ---- */

const char *la_dir_name(la_dir_t d) {
    switch (d) {
    case LA_DIR_DUT_INPUT:  return "dut_input";
    case LA_DIR_DUT_OUTPUT: return "dut_output";
    case LA_DIR_BIDIR:      return "bidir";
    default:                return "unknown";
    }
}

bool la_dir_parse(const char *s, la_dir_t *out) {
    for (int d = LA_DIR_UNKNOWN; d <= LA_DIR_BIDIR; d++)
        if (strcmp(s, la_dir_name((la_dir_t)d)) == 0) { *out = (la_dir_t)d; return true; }
    return false;
}

la_dir_t la_pins_dir(unsigned la) { return la_ok(la) ? (la_dir_t)s_dir[la] : LA_DIR_UNKNOWN; }
void     la_pins_set_dir(unsigned la, la_dir_t d) { if (la_ok(la)) s_dir[la] = (uint8_t)d; }

bool la_fn_drives(la_fn_t fn, la_gpio_mode_t mode) {
    if (fn == LA_FN_GPIO) return mode == LA_GPIO_OUTPUT;
    return fn_attr(fn)->drives;   /* inputs and open drain (I2C, i2cm, uart_hd) do not */
}

bool la_pins_check_dir(unsigned la, la_dir_t d, char *err, size_t cap) {
    if (!la_ok(la)) { snprintf(err, cap, "LA%u is not a pin (use 1..14)", la); return false; }
    la_fn_t fn = (la_fn_t)s_pin[la].fn;
    la_gpio_mode_t mode = la_pins_gpio(la);
    if (d != LA_DIR_DUT_OUTPUT || !la_fn_drives(fn, mode)) return true;
    char name[24];
    const char *what = fn_display(fn, mode, name, sizeof(name));
    snprintf(err, cap, "direction conflict: LA%u is driven by %s, so it cannot be marked as a DUT output",
             la, what);
    bp_tag_la_driven_pin(err, (int32_t)la, what);
    return false;
}

void la_pins_set_target_powered(bool on) { s_powered = on; }

uint8_t la_pins_warnings(unsigned la, uint16_t pull_mask) {
    if (!la_ok(la)) return 0;
    la_fn_t fn = (la_fn_t)s_pin[la].fn;
    la_gpio_mode_t mode = la_pins_gpio(la);
    bool pull_on = (pull_mask & (1u << (la - 1u))) != 0 && la_pull_dir(la) != LA_PULL_NONE;
    bool drives  = la_fn_drives(fn, mode);
    uint8_t w = 0;
    if (pull_on && drives) w |= LA_WARN_PULL_VS_DRIVE;
    /* Driven high: a gpio output at 1, or an engine pin that idles high (UART TX, SPI CS, SWD). */
    bool high = (fn == LA_FN_GPIO) ? (mode == LA_GPIO_OUTPUT && s_pin[la].level) : drives;
    if (!s_powered && (high || (pull_on && la_pull_dir(la) == LA_PULL_UP))) w |= LA_WARN_UNPOWERED;
    return w;
}

bool la_pins_check_pull_enable(unsigned la, la_pull_dir_t d, char *err, size_t cap) {
    if (!la_ok(la) || d != LA_PULL_DOWN) return true;
    la_fn_t fn = (la_fn_t)s_pin[la].fn;
    la_gpio_mode_t mode = la_pins_gpio(la);
    const char *reason = pull_down_reason(fn, mode);
    if (!reason) return true;
    char name[24];
    const char *what = fn_display(fn, mode, name, sizeof(name));
    snprintf(err, cap, "pull conflict: LA%u is used by %s, which cannot work with the %s pull-down (%s)",
             la, what, la_pull_ohms(la), reason);
    bp_tag_la_pull_down_conflict(err, (int32_t)la, la_pull_ohms(la), what, reason);
    return false;
}

bool la_pins_check_level(const uint8_t *las, size_t n, char *err, size_t cap) {
    for (size_t i = 0; i < n; i++) {
        unsigned la = las[i];
        if (!la_ok(la)) { snprintf(err, cap, "LA%u is not a pin (use 1..14)", la); return false; }
        la_gpio_mode_t m = la_pins_gpio(la);
        if (m == LA_GPIO_OUTPUT || m == LA_GPIO_OPEN_DRAIN) continue;
        la_fn_t fn = (la_fn_t)s_pin[la].fn;
        char name[24];
        const char *what = fn_display(fn, m, name, sizeof(name));
        snprintf(err, cap, "LA%u is not a GPIO output (it is %s)", la, what);
        bp_tag_la_not_gpio_output(err, (int32_t)la, what);
        return false;
    }
    return true;
}

bool la_pins_check_voltage_change(int cur_mv, int new_mv, char *err, size_t cap) {
    if (new_mv == cur_mv) return true;
    /* 5000 is a voltage only where the board has 5 V I/O (board_caps.h); elsewhere the caller's
       own validation refuses it, as before. */
    bool valid = new_mv == 1800 || new_mv == 3300 || (new_mv == 5000 && board_caps()->la_5v);
    if (!valid) return true;   /* the caller's own validation says why */
    uint16_t owned = la_pins_owned_mask();
    if (!owned) return true;
    char pins[LA_PINS_COUNT * 20u];   /* "LA14 (sensor_miso), " at most per pin */
    size_t pos = 0;
    pins[0] = 0;
    for (unsigned la = 1; la <= LA_PINS_COUNT && pos < sizeof(pins); la++) {
        if (!(owned & (1u << (la - 1u)))) continue;
        pos += (size_t)snprintf(pins + pos, sizeof(pins) - pos, "%sLA%u (%s)", pos ? ", " : "",
                                la, la_fn_name((la_fn_t)s_pin[la].fn));
    }
    snprintf(err, cap, "la voltage can't change while pins are in use: %s", pins);
    bp_tag_la_voltage_pins_in_use(err, pins);
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
        s_pin[la].owner = 0u;   /* a sensor sets its owner bits after the claim */
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
        snprintf(err, cap, "la must be 1..14 (or \"all\" with \"mode\":\"off\")");
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

bool la_pins_query_mask(const char *json, uint16_t *mask, char *err, size_t cap) {
    *mask = 0x3FFFu;
    const char *v = find_value(json, "la");
    if (!v || strncmp(v, "null", 4) == 0) return true;
    la_gpio_req_t req;
    /* Only "la" is read: parse it alone, so a stray mode/level never refuses a query. */
    char one[160];
    const char *end = v;
    if (*v == '[') { while (*end && *end != ']') end++; if (*end) end++; }
    else if (*v == '"') { end++; while (*end && *end != '"') end++; if (*end) end++; }
    else while (*end && *end != ',' && *end != '}' && *end != ' ') end++;
    if ((size_t)(end - v) + 8u >= sizeof(one)) { snprintf(err, cap, "la list is too long"); return false; }
    snprintf(one, sizeof(one), "{\"la\":%.*s}", (int)(end - v), v);
    if (!la_gpio_req_parse(one, &req, err, cap)) return false;
    if (!req.all) *mask = 0;
    for (size_t i = 0; i < req.n; i++) *mask |= (uint16_t)(1u << (req.las[i] - 1u));
    return true;
}

bool la_pins_query_from(const char *json, unsigned *from, char *err, size_t cap) {
    *from = 1;
    const char *v = find_value(json, "from");
    if (!v || strncmp(v, "null", 4) == 0) return true;
    char *end = NULL;
    long n = (*v >= '0' && *v <= '9') ? strtol(v, &end, 10) : 0;
    if (!end || n < 1 || n > (long)LA_PINS_COUNT || (*end && *end != ',' && *end != '}' && *end != ' ')) {
        snprintf(err, cap, "from must be a pin 1..14 (the next of the previous reply)");
        return false;
    }
    *from = (unsigned)n;
    return true;
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
    bp_emit(e, "{\"la\":%u,\"function\":\"%s\",", la, la_fn_name(la_pins_fn(la)));
    /* An emulated sensor's pin names its instance: "owner":"sensor:env". A shared I2C line names
       the first and counts the others ("sensor:env+2"; sensor_status lists every instance's
       pins), so a shared bus costs one name. Long names on many pins still outgrow the cloud reply
       cap: test_la_pins.c measures it (reply budget). */
    if (s_pin[la].owner && s_namer) {
        const char *first = NULL;
        unsigned more = 0;
        for (unsigned s = 0; s < LA_PINS_OWNER_SLOTS; s++) {
            const char *n = (s_pin[la].owner & (1u << s)) ? s_namer(s) : NULL;
            if (!n) continue;
            if (!first) first = n;
            else more++;
        }
        if (first && more) bp_emit(e, "\"owner\":\"sensor:%s+%u\",", first, more);
        else if (first)    bp_emit(e, "\"owner\":\"sensor:%s\",", first);
    }
    bp_emit_raw(e, "\"gpio\":");
    if (m != LA_GPIO_NONE) bp_emit(e, "\"%s\"", la_gpio_mode_name(m));
    else                   bp_emit_raw(e, "null");
    if (level >= 0) bp_emit(e, ",\"level\":%d,\"pull\":", level);
    else            bp_emit_raw(e, ",\"level\":null,\"pull\":");
    const char *ohms = la_pull_ohms(la);
    if (ohms) {
        /* dir: the way it pulls when on; dirs: every way it can pull (pull "up"/"down"/"off").
           Digital v4 leaves dirs out: every channel pulls both ways there, said once in the
           la_pins reply (pull_dirs), as 14 copies would not fit the cloud reply. */
        bp_emit(e, "{\"dir\":\"%s\",\"ohms\":\"%s\",\"on\":%s",
                la_pull_dir_name(la_pull_dir(la)), ohms,
                (pull_mask & (1u << (la - 1u))) ? "true" : "false");
        if (!la_pull_both_ways()) {
            bp_emit_raw(e, ",\"dirs\":");
            la_pull_emit_dirs(e, la);
        }
        bp_emit_raw(e, "}");
    } else {
        bp_emit_raw(e, "null");
    }
    if (s_dir[la] != LA_DIR_UNKNOWN) bp_emit(e, ",\"dir\":\"%s\"", la_dir_name((la_dir_t)s_dir[la]));
    uint8_t w = la_pins_warnings(la, pull_mask);
    if (w) bp_emit(e, ",\"warn\":%u", (unsigned)w);
    bp_emit_raw(e, "}");
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

/* The bytes after the last entry: ] (,"pull_dirs":[...]) (,"next":N) ,"levels":M|null}} and the
   newline.  A cut reply may say up to "next":14. */
static size_t reply_tail_len(bool cut, bool have_levels, uint16_t levels) {
    size_t n = 1u + (la_pull_both_ways() ? 26u : 0u) + (cut ? 10u : 0u);
    if (!have_levels) return n + 17u;
    return n + 13u + (levels >= 10000u ? 5u : levels >= 1000u ? 4u : levels >= 100u ? 3u : levels >= 10u ? 2u : 1u);
}

unsigned la_pins_emit_reply(bp_emit_t *e, uint16_t only, unsigned from, uint16_t pull_mask,
                            bool have_levels, uint16_t levels) {
    const size_t start = bp_emit_len(e);
    unsigned next = 0;
    bool first = true;
    if (from < 1u) from = 1u;
    bp_emit_raw(e, "{\"status\":\"ok\",\"data\":{\"pins\":[");
    for (unsigned la = from; la <= LA_PINS_COUNT; la++) {
        if (!(only & (1u << (la - 1u)))) continue;
        const size_t before = bp_emit_len(e);
        if (!first) bp_emit_raw(e, ",");
        la_pins_emit_entry(e, la, pull_mask);
        /* The last pin needs room for the tail; any other one for the tail with a "next" too,
           in case the pin after it does not fit.  One that does not fit is taken back. */
        const bool last = (only & (uint16_t)~((2u << (la - 1u)) - 1u)) == 0;
        const size_t need = bp_emit_len(e) - start + reply_tail_len(!last, have_levels, levels);
        if ((!bp_emit_ok(e) || need > LA_PINS_PAGE_MAX) && !first) {
            bp_emit_rewind(e, before);
            next = la;
            break;
        }
        first = false;
    }
    bp_emit_raw(e, "]");
    /* Digital v4: every channel's pull goes both ways (the entries leave dirs out). */
    if (la_pull_both_ways()) bp_emit_raw(e, ",\"pull_dirs\":[\"up\",\"down\"]");
    if (next) bp_emit(e, ",\"next\":%u", next);
    if (have_levels) bp_emit(e, ",\"levels\":%u}}\n", (unsigned)levels);
    else             bp_emit_raw(e, ",\"levels\":null}}\n");
    return next;
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
        snprintf(err, cap, "trigger la must be 1..14");
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
    if (bp_json_get(obj, "pre_samples", s, sizeof(s)) && strcmp(s, "null") != 0) {
        end = NULL;
        long pre = strtol(s, &end, 10);
        if (end == s || *end != '\0' || pre < 0 || pre > 16777215L) {
            snprintf(err, cap, "trigger pre_samples must be 0..16777215");
            return false;
        }
        out->pre_samples = (uint32_t)pre;
    }
    out->present = true;
    out->la      = (uint8_t)la;
    out->edge    = edge;
    return true;
}

void la_trigger_timeout_msg(const la_trigger_t *t, char *buf, size_t cap) {
    snprintf(buf, cap, "trigger timeout: no %s edge on LA%u within %lu ms",
             la_edge_name(t->edge), (unsigned)t->la, (unsigned long)t->timeout_ms);
    bp_tag_trigger_timeout(buf, (int32_t)t->la, la_edge_name(t->edge), (int32_t)t->timeout_ms);
}

void la_trigger_emit(bp_emit_t *e, const la_trigger_t *t, bool fired) {
    bp_emit(e, "\"trigger\":{\"la\":%u,\"edge\":\"%s\",\"fired\":%s}",
            (unsigned)t->la, la_edge_name(t->edge), fired ? "true" : "false");
}

/* ---- step train arguments ---- */

bool la_step_parse_u32(const char *s, uint32_t *out) {
    if (!s || !*s) return false;
    uint64_t v = 0;
    for (; *s; s++) {
        if (*s < '0' || *s > '9') return false;
        v = v * 10u + (uint64_t)(*s - '0');
        if (v > 0xFFFFFFFFull) return false;
    }
    *out = (uint32_t)v;
    return true;
}

bool la_step_args_ok(uint32_t steps, uint32_t delay_us, char *err, size_t cap) {
    if (steps == 0 || steps > LA_STEP_MAX_STEPS) {
        snprintf(err, cap, "steps must be 1..%u (the step counter is 16-bit): split a longer move "
                 "into several trains", (unsigned)LA_STEP_MAX_STEPS);
        return false;
    }
    if (delay_us < LA_STEP_MIN_DELAY_US || delay_us > LA_STEP_MAX_DELAY_US) {
        snprintf(err, cap, "delay_us must be %u..%u (microseconds per half-phase: a step takes "
                 "2 x delay_us)", (unsigned)LA_STEP_MIN_DELAY_US, (unsigned)LA_STEP_MAX_DELAY_US);
        return false;
    }
    return true;
}
