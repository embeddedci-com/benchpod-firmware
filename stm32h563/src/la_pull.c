/* la_pull.c: see la_pull.h.  The channel tables, the reporting and the hold; the switching is in
   the backends (pull_tca9554.c, pull_exp.c). */
#include "la_pull.h"
#include "board_caps.h"

#include <stdlib.h>
#include <string.h>

/* ---- the channel tables: one per backend ----
   A board revision with other resistors gets a table (and a backend) of its own in la_pull_init. */

/* Boards before digital v3r4: LA1/LA2 4.7k up, LA3/LA4 2.2k up, LA5/LA6 10k up, LA7/LA8 10k down,
   LA9..LA14 none.  On v3 the pull-ups go to the bank rail (v2: +3V3), the pull-downs to GND. */
static const la_pull_caps_t s_caps_tca9554[LA_PULL_CHANNELS + 1] = {
    [1] = { LA_PULL_BIT(LA_PULL_UP),   LA_PULL_UP,   0, "4.7k" },
    [2] = { LA_PULL_BIT(LA_PULL_UP),   LA_PULL_UP,   0, "4.7k" },
    [3] = { LA_PULL_BIT(LA_PULL_UP),   LA_PULL_UP,   0, "2.2k" },
    [4] = { LA_PULL_BIT(LA_PULL_UP),   LA_PULL_UP,   0, "2.2k" },
    [5] = { LA_PULL_BIT(LA_PULL_UP),   LA_PULL_UP,   0, "10k"  },
    [6] = { LA_PULL_BIT(LA_PULL_UP),   LA_PULL_UP,   0, "10k"  },
    [7] = { LA_PULL_BIT(LA_PULL_DOWN), LA_PULL_DOWN, 0, "10k"  },
    [8] = { LA_PULL_BIT(LA_PULL_DOWN), LA_PULL_DOWN, 0, "10k"  },
    /* LA9..LA14: no resistor */
};

/* Digital v3r4: every channel up or down with 10k, 2.2k or both.  "pullup":"on" engages up, and
   the ohms shown while off are the 10k default. */
#define V3R4_PULL { LA_PULL_BIT(LA_PULL_UP) | LA_PULL_BIT(LA_PULL_DOWN), LA_PULL_UP, \
                    LA_PULL_STR_BIT(PULL_STR_10K) | LA_PULL_STR_BIT(PULL_STR_2K2) | LA_PULL_STR_BIT(PULL_STR_BOTH), "10k" }
static const la_pull_caps_t s_caps_exp[LA_PULL_CHANNELS + 1] = {
    [1] = V3R4_PULL, [2] = V3R4_PULL, [3] = V3R4_PULL, [4] = V3R4_PULL, [5] = V3R4_PULL,
    [6] = V3R4_PULL, [7] = V3R4_PULL, [8] = V3R4_PULL, [9] = V3R4_PULL, [10] = V3R4_PULL,
    [11] = V3R4_PULL, [12] = V3R4_PULL, [13] = V3R4_PULL, [14] = V3R4_PULL,
};

/* Before la_pull_init: nothing engaged, nothing can be (no write reaches an expander that is
   not known yet). */
static la_pull_dir_t none_asked(unsigned la, pull_str_t *str) { (void)la; *str = PULL_STR_NONE; return LA_PULL_NONE; }
static int  none_set(unsigned la, la_pull_dir_t dir, pull_str_t str) { (void)la; (void)dir; (void)str; return LA_PULL_ERR_IO; }
static bool none_live(unsigned la) { (void)la; return false; }
static bool none_available(void) { return false; }
static void none_all_off(void) {}
static const la_pull_backend_t s_none = {
    .set = none_set, .asked = none_asked, .live = none_live, .available = none_available, .all_off = none_all_off,
};

static const la_pull_backend_t *s_be   = &s_none;
static const la_pull_caps_t    *s_caps = s_caps_tca9554;
static bool                     s_both_ways;

static bool la_ok(unsigned la) { return la >= 1u && la <= LA_PULL_CHANNELS; }

void la_pull_init(void)
{
    bool exp = board_caps()->pull_expanders;
    s_be   = exp ? &la_pull_backend_exp : &la_pull_backend_tca9554;
    s_caps = exp ? s_caps_exp : s_caps_tca9554;
    s_both_ways = true;
    for (unsigned la = 1; la <= LA_PULL_CHANNELS; la++)
        if (s_caps[la].dirs != (LA_PULL_BIT(LA_PULL_UP) | LA_PULL_BIT(LA_PULL_DOWN))) s_both_ways = false;
    if (s_be->init) s_be->init();
}

void la_pull_poll(void) { if (s_be->poll) s_be->poll(); }

/* ---- what a channel can do ---- */

la_pull_caps_t la_pull_caps(unsigned la)
{
    la_pull_caps_t none = { 0, 0, 0, NULL };
    return la_ok(la) ? s_caps[la] : none;
}

bool la_pull_supports(unsigned la, la_pull_dir_t d)
{
    return la_ok(la) && d != LA_PULL_NONE && (s_caps[la].dirs & LA_PULL_BIT(d));
}

bool la_pull_both_ways(void)     { return s_both_ways; }
bool la_pull_rail_is_pin21(void) { return s_be->rail_pin21; }

la_pull_dir_t la_pull_dir(unsigned la)
{
    if (!la_ok(la) || !s_caps[la].dirs) return LA_PULL_NONE;
    la_pull_dir_t d = la_pull_asked(la);
    return d != LA_PULL_NONE ? d : (la_pull_dir_t)s_caps[la].dflt;
}

const char *la_pull_ohms(unsigned la)
{
    if (!la_ok(la) || !s_caps[la].dirs) return NULL;
    if (s_caps[la].strengths && la_pull_asked(la) != LA_PULL_NONE) {
        const char *o = pull_str_ohms(la_pull_asked_str(la));
        if (o) return o;
    }
    return s_caps[la].ohms;
}

/* ---- state ---- */

int la_pull_set(unsigned la, la_pull_dir_t dir, pull_str_t str)
{
    if (!la_ok(la)) return LA_PULL_ERR_IO;
    return s_be->set(la, dir, str);
}

la_pull_dir_t la_pull_asked(unsigned la)
{
    pull_str_t s = PULL_STR_NONE;
    return la_ok(la) ? s_be->asked(la, &s) : LA_PULL_NONE;
}

pull_str_t la_pull_asked_str(unsigned la)
{
    pull_str_t s = PULL_STR_NONE;
    if (!la_ok(la) || s_be->asked(la, &s) == LA_PULL_NONE) return PULL_STR_NONE;
    return s;
}

bool        la_pull_live(unsigned la) { return la_ok(la) && s_be->live(la); }
bool        la_pull_available(void)   { return s_be->available(); }
void        la_pull_all_off(void)     { s_be->all_off(); }
const char *la_pull_held(void)        { return s_be->held ? s_be->held() : NULL; }

uint16_t la_pull_mask_now(void)
{
    uint16_t mask = 0;
    for (unsigned la = 1; la <= LA_PULL_CHANNELS; la++)
        if (s_caps[la].dirs && la_pull_asked(la) != LA_PULL_NONE) mask |= (uint16_t)(1u << (la - 1u));
    return mask;
}

uint16_t la_pull_mask_live(void)
{
    uint16_t mask = 0;
    for (unsigned la = 1; la <= LA_PULL_CHANNELS; la++)
        if (s_caps[la].dirs && la_pull_live(la)) mask |= (uint16_t)(1u << (la - 1u));
    return mask;
}

/* ---- a temporary pull ---- */

int la_pull_hold(unsigned la, la_pull_dir_t dir, pull_str_t str, la_pull_token_t *tok)
{
    *tok = LA_PULL_TOKEN_NONE;
    if (!la_pull_supports(la, dir) || !la_pull_available()) return LA_PULL_ERR_NOT_HERE;
    pull_str_t prev_str = PULL_STR_NONE;
    la_pull_dir_t prev = s_be->asked(la, &prev_str);
    int rc = s_be->set(la, dir, str);
    if (rc != 0) return rc;
    tok->la  = (uint8_t)la;
    tok->dir = (uint8_t)prev;
    tok->str = (uint8_t)(prev == LA_PULL_NONE ? PULL_STR_NONE : prev_str);
    return 0;
}

int la_pull_release(la_pull_token_t *tok)
{
    if (!la_pull_token_held(tok)) return 0;
    la_pull_token_t t = *tok;
    *tok = LA_PULL_TOKEN_NONE;
    return la_pull_set(t.la, (la_pull_dir_t)t.dir, (pull_str_t)t.str);
}

/* ---- names and requests ---- */

const char *la_pull_dir_name(la_pull_dir_t d)
{
    return d == LA_PULL_UP ? "up" : d == LA_PULL_DOWN ? "down" : "off";
}

bool la_pull_dir_parse(const char *s, la_pull_dir_t *d)
{
    if (!s) return false;
    if (!strcmp(s, "up"))   { *d = LA_PULL_UP;   return true; }
    if (!strcmp(s, "down")) { *d = LA_PULL_DOWN; return true; }
    if (!strcmp(s, "off") || !strcmp(s, "none")) { *d = LA_PULL_NONE; return true; }
    return false;
}

void la_pull_emit_dirs(bp_emit_t *e, unsigned la)
{
    bool up = la_pull_supports(la, LA_PULL_UP), down = la_pull_supports(la, LA_PULL_DOWN);
    bp_emit(e, "[%s%s%s]", up ? "\"up\"" : "", up && down ? "," : "", down ? "\"down\"" : "");
}

la_pull_req_t la_pull_request(unsigned la, const char *pull, const char *pullup, la_pull_dir_t *want)
{
    la_pull_dir_t d = LA_PULL_NONE;
    if (pull) {
        if (!la_pull_dir_parse(pull, &d)) return LA_PULL_REQ_BAD_VALUE;
    } else if (pullup) {
        /* the alias: "on" / "1" engage the default direction */
        bool on = (pullup[0] == 'o' || pullup[0] == 'O') ? (pullup[1] == 'n' || pullup[1] == 'N')
                                                         : (atoi(pullup) != 0);
        d = on ? la_pull_dir(la) : LA_PULL_NONE;
        if (on && d == LA_PULL_NONE) { *want = LA_PULL_UP; return LA_PULL_REQ_NO_PULL; }
    }
    *want = d;   /* also on a refusal: the reply names what was asked for */
    if (d != LA_PULL_NONE) {
        if (!la_pull_caps(la).dirs)     return LA_PULL_REQ_NO_PULL;
        if (!la_pull_supports(la, d))   return LA_PULL_REQ_DIRECTION;
    }
    return LA_PULL_REQ_OK;
}
