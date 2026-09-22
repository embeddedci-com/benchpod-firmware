#ifndef LA_PINS_H
#define LA_PINS_H

/*
 * la_pins — who owns each of the 14 logic-analyzer pins (LA1..LA14 on the iCE40).
 *
 * The gateware's la_bank.v resolves overlapping drivers silently, by priority
 * (swd > stepper > i2c > uart > static GPIO_SET).  Nothing used to stop a UART proxy and an
 * emulated I2C sensor from sharing a channel; the loser simply did not work.  This table
 * gives every pin exactly ONE function, refuses a claim that would collide, and owns the
 * pull-resistor compatibility rules and the exact error strings (clients parse the
 * `pin conflict:` / `pull conflict:` / `trigger timeout:` prefixes — docs/API.md).
 *
 * Pure C with no hardware calls: callers pass in the engaged-pull bitmask and drive the FPGA
 * themselves (command_handler_pins.c).  Host-tested in test/test_la_pins.c.  The state is only
 * touched from the hw worker task.  Pins are 1-based everywhere; bit (la-1) in every mask.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bp_json.h"

#define LA_PINS_COUNT 14u

typedef enum {
    LA_FN_NONE = 0,   /* LA mode: high-Z, observed by captures */
    LA_FN_GPIO,       /* `gpio` input / output / open_drain */
    LA_FN_UART_RX,    /* uart_proxy_start */
    LA_FN_UART_TX,
    LA_FN_SWD_CLK,    /* dap_start */
    LA_FN_SWD_DIO,
    LA_FN_I2C_SDA,    /* sensor_start */
    LA_FN_I2C_SCL,
    LA_FN_STEP,       /* `la` step train */
    LA_FN_STEP_DIR,
    LA_FN__COUNT
} la_fn_t;

#define LA_FN_BIT(fn) ((uint16_t)(1u << (unsigned)(fn)))

typedef enum {
    LA_GPIO_NONE = 0,     /* not a gpio pin (also the parsed value of "off") */
    LA_GPIO_INPUT,        /* high-Z, level readable */
    LA_GPIO_OUTPUT,       /* push-pull, level 0/1 */
    LA_GPIO_OPEN_DRAIN,   /* level 0 drives low, level 1 releases to high-Z */
} la_gpio_mode_t;

typedef enum { LA_PULL_NONE = 0, LA_PULL_UP, LA_PULL_DOWN } la_pull_dir_t;

/* Every message this module builds fits in this many bytes (NUL included).  320, not 256: the
   voltage-change refusal listing all 14 pins by function name reaches 286 B (test_la_pins.c). */
#define LA_PINS_ERR_MAX 320u

/* Pod reboot state: every pin back to none. */
void la_pins_reset(void);

const char *la_fn_name(la_fn_t fn);
const char *la_gpio_mode_name(la_gpio_mode_t m);          /* "" for LA_GPIO_NONE */
/* "input" / "output" / "open_drain" / "off" (-> LA_GPIO_NONE).  false when unknown. */
bool        la_gpio_mode_parse(const char *s, la_gpio_mode_t *out);

la_fn_t        la_pins_fn(unsigned la);        /* LA_FN_NONE for an out-of-range pin */
la_gpio_mode_t la_pins_gpio(unsigned la);
int            la_pins_level(unsigned la);     /* commanded level of a gpio output/open_drain, else -1 */
uint16_t       la_pins_mask_fn(la_fn_t fn);    /* pins that currently have this function */
uint16_t       la_pins_owned_mask(void);       /* pins whose function is not none */

/* Fixed board resistors: LA1/LA2 4.7k up, LA3/LA4 2.2k up, LA5/LA6 10k up, LA7/LA8 10k down,
   LA9..LA14 none (ohms NULL). */
la_pull_dir_t la_pull_dir(unsigned la);
const char   *la_pull_ohms(unsigned la);

/* What GPIO_SET should hold on the pin's static latch: 0 low, 1 high, 2 high-Z.  Only a gpio
   output / open_drain-low pin drives; everything else (none, gpio input, engine functions) is
   high-Z underneath — the engines override the latch while they run. */
int la_pins_drive(unsigned la);

/* ---- checks: true = allowed; false = the client-facing message is in err ---- */

/* May `fn` (with gpio `mode` when fn is LA_FN_GPIO) claim las[0..n)?  Pins owned by a function
   in `free_fns` count as free (a replacing sensor_start, gpio mode switches).  Ownership is
   checked over all pins first, then pull compatibility against `pull_mask` (engaged pulls). */
bool la_pins_check_claim(la_fn_t fn, la_gpio_mode_t mode, const uint8_t *las, size_t n,
                         uint16_t free_fns, uint16_t pull_mask, char *err, size_t cap);
/* May the pull on `la` be engaged, given the pin's current function? */
bool la_pins_check_pull_enable(unsigned la, char *err, size_t cap);
/* Are all pins gpio output / open_drain (so a level can be set)? */
bool la_pins_check_level(const uint8_t *las, size_t n, char *err, size_t cap);
/* May the LA bank switch from cur_mv to new_mv?  Refused while any pin is owned, unless the
   voltage does not change.  An invalid new_mv is allowed here (the caller rejects it). */
bool la_pins_check_voltage_change(int cur_mv, int new_mv, char *err, size_t cap);

/* ---- commits (no checks) ---- */
void     la_pins_claim(la_fn_t fn, la_gpio_mode_t mode, uint8_t level,
                       const uint8_t *las, size_t n);
void     la_pins_set_level(unsigned la, uint8_t level);
void     la_pins_release(unsigned la);
uint16_t la_pins_release_fn(la_fn_t fn);   /* releases every pin with fn; returns their mask */

/* ---- the `gpio` command ---- */
typedef struct {
    uint8_t        las[LA_PINS_COUNT];   /* de-duplicated, request order */
    size_t         n;
    bool           all;                  /* "la":"all" */
    bool           has_mode;
    la_gpio_mode_t mode;                 /* LA_GPIO_NONE = "off" */
    bool           has_level;
    uint8_t        level;
} la_gpio_req_t;

/* Parse {"cmd":"gpio","la":N|[N,...]|"all","mode":...,"level":0|1}.  "la" may be absent only
   for a read (no mode, no level).  A JSON null counts as absent. */
bool la_gpio_req_parse(const char *json, la_gpio_req_t *req, char *err, size_t cap);
/* Validate the WHOLE mode/level request, then commit it (on any error nothing changes).
   *changed = pins whose GPIO_SET drive must be re-sent (see la_pins_drive); *affected = pins
   to report back.  A read request (no mode, no level) is not handled here. */
bool la_pins_gpio_apply(const la_gpio_req_t *req, uint16_t pull_mask,
                        uint16_t *changed, uint16_t *affected, char *err, size_t cap);

/* ---- `la` step trains ---- */
typedef struct {
    bool claim_step;   /* step pin was none: claim `step` for the train */
    bool claim_dir;    /* dir pin was none: claim `step_dir`, drive it, release after */
} la_step_plan_t;
/* la 1..14; dir_la 0 = no direction pin.  A gpio output pin is used without changing its
   ownership (the train pulses it / the direction sets its level); none is claimed; anything
   else is a conflict. */
bool la_pins_plan_step(unsigned la, unsigned dir_la, la_step_plan_t *plan, char *err, size_t cap);

/* ---- JSON ---- */
/* {"la":1,"function":"none","gpio":null,"level":null,"pull":{"dir":"up","ohms":"4.7k","on":false}} */
void la_pins_emit_entry(bp_emit_t *e, unsigned la, uint16_t pull_mask);
/* [entry,...] for every pin in `mask`, ascending. */
void la_pins_emit_list(bp_emit_t *e, uint16_t mask, uint16_t pull_mask);

/* ---- capture trigger ("trigger":{"la":N,"edge":...}, "trigger_timeout_ms":N) ---- */
typedef enum {
    LA_EDGE_RISING  = 1,   /* values are the gateware SET_TRIGGER mode byte */
    LA_EDGE_FALLING = 2,
    LA_EDGE_HIGH    = 3,
    LA_EDGE_LOW     = 4,
} la_edge_t;

#define LA_TRIGGER_TIMEOUT_DEFAULT_MS 10000u
#define LA_TRIGGER_TIMEOUT_MAX_MS     600000u

typedef struct {
    bool      present;
    uint8_t   la;           /* 1..14 */
    la_edge_t edge;
    uint32_t  timeout_ms;
} la_trigger_t;

const char *la_edge_name(la_edge_t edge);
/* present=false when the request has no "trigger" (or it is null). */
bool la_trigger_parse(const char *json, la_trigger_t *out, char *err, size_t cap);
/* "trigger timeout: no rising edge on LA9 within 10000 ms" */
void la_trigger_timeout_msg(const la_trigger_t *t, char *buf, size_t cap);
/* "trigger":{"la":9,"edge":"rising","fired":true}  (no leading comma) */
void la_trigger_emit(bp_emit_t *e, const la_trigger_t *t, bool fired);

#endif /* LA_PINS_H */
