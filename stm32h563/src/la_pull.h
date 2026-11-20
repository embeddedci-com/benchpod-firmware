#ifndef LA_PULL_H
#define LA_PULL_H

/*
 * la_pull, the pull resistors on the 14 LA channels: what each channel's resistor can do, what
 * the host asked for, what is on the wire now, and the one place that switches them.
 *
 * Two backends, picked once by la_pull_init() from board_caps():
 *   pull_tca9554.c  the boards before digital v3r4: one TCA9554 at 0x20 closes a switch per
 *                   channel (LA1-LA8), each with one fixed resistor and direction.
 *   pull_exp.c      digital v3r4: two TCA6416A expanders, every channel up or down with 10k,
 *                   2.2k or both, pulling to J1 pin 21 (pull_exp.h).
 * Nothing outside this module and the backends asks which board it runs on for a pull: callers
 * read la_pull_caps() and the state functions below.  The asked-for state lives in the backend
 * (one copy); the pin table (la_pins.c) and the commands read it from here.
 *
 * Only the hw worker task calls these (like everything that touches the LA bank).
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bp_json.h"

#define LA_PULL_CHANNELS 14u

typedef enum { LA_PULL_NONE = 0, LA_PULL_UP, LA_PULL_DOWN } la_pull_dir_t;
#define LA_PULL_BIT(d) ((uint8_t)(1u << (unsigned)(d)))

/* Strength: which resistors a channel engages (digital v3r4; the older boards have one fixed
   resistor per channel and no choice). */
typedef enum { PULL_STR_NONE = 0, PULL_STR_10K = 1, PULL_STR_2K2 = 2, PULL_STR_BOTH = 3 } pull_str_t;
#define LA_PULL_STR_BIT(s) ((uint8_t)(1u << (unsigned)(s)))
/* "10k", "2.2k", "1.8k" (both), or NULL.  (pull_exp_logic.c) */
const char *pull_str_ohms(pull_str_t s);
/* "10k" / "2.2k" / "both" -> strength; false otherwise. */
bool        pull_str_parse(const char *s, pull_str_t *out);
const char *pull_str_name(pull_str_t s);   /* "10k", "2.2k", "both", "none" */

/* ---- what a channel can do ---- */
typedef struct {
    uint8_t     dirs;        /* LA_PULL_BIT of each direction it can pull; 0 = no resistor */
    uint8_t     dflt;        /* la_pull_dir_t: the channel's default direction */
    uint8_t     strengths;   /* LA_PULL_STR_BIT of each strength it offers; 0 = fixed resistor */
    const char *ohms;        /* the value shown while off ("4.7k"), NULL without a resistor */
} la_pull_caps_t;

/* Pick the backend for board_caps() and start it (digital v3r4: the expanders out of reset once
   pin 21 is up).  Once at boot, after board_caps_init(), vref_out_init() and i2c_bus_init().
   Before it runs no pull is on and none can be engaged (the table is the older boards'). */
void           la_pull_init(void);
/* The backend's periodic work (v3r4: pull-ups follow the pod's target power).  hw worker loop. */
void           la_pull_poll(void);

la_pull_caps_t la_pull_caps(unsigned la);      /* all zero for an out-of-range channel */
/* Can the channel's resistor pull in direction d (UP or DOWN)? */
bool           la_pull_supports(unsigned la, la_pull_dir_t d);
/* Every channel pulls both ways (digital v3r4): listings say so once instead of per pin. */
bool           la_pull_both_ways(void);
/* The pulls go to J1 pin 21 (digital v3r4), not to the bank rail: they are off while it is down. */
bool           la_pull_rail_is_pin21(void);

/* The direction the channel reports: the one engaged when the host asked for one, else its
   default; LA_PULL_NONE without a resistor. */
la_pull_dir_t  la_pull_dir(unsigned la);
/* The resistor's value as text: what is engaged ("1.8k" for both on v3r4), else the default
   ("4.7k"); NULL on a channel without one. */
const char    *la_pull_ohms(unsigned la);

/* ---- state ---- */

/* Errors of la_pull_set (0 = done). */
#define LA_PULL_ERR_IO       (-1)   /* bad channel, no resistor, or the expander write failed */
#define LA_PULL_ERR_VOLTAGE  (-2)   /* digital v3r4: no I/O voltage selected yet */
#define LA_PULL_ERR_RAIL     (-3)   /* digital v3r4: pin 21 is down, the expanders are held in reset */
#define LA_PULL_ERR_BANK     (-4)   /* older boards: no bank voltage yet, or a v2 board at 1.8 V
                                       (its pull-ups go to +3V3) */
#define LA_PULL_ERR_NOT_HERE (-5)   /* la_pull_hold: the channel can't pull that way, or no pull can
                                       be engaged now; nothing was tried */

/* Engage `la` in direction `dir` (LA_PULL_NONE = off) with strength `str` (PULL_STR_NONE = what
   the pin's function asks for, la_pull_strength_for; ignored on a fixed resistor).  Turning a
   pull off is always allowed.  The caller has checked the direction (la_pull_request). */
int            la_pull_set(unsigned la, la_pull_dir_t dir, pull_str_t str);
/* What the host asked for on `la` (LA_PULL_NONE = off), also while the pod holds it off. */
la_pull_dir_t  la_pull_asked(unsigned la);
/* The strength asked for (PULL_STR_NONE when off or on a fixed resistor). */
pull_str_t     la_pull_asked_str(unsigned la);
/* The pull is on the wire now.  The same as la_pull_asked() != LA_PULL_NONE everywhere but
   digital v3r4, where a pull the host asked for is off while pin 21 is down, after a failed
   write, or while pull-ups are suspended (pull_exp.h). */
bool           la_pull_live(unsigned la);
/* A pull can be engaged right now: older boards need a bank voltage, and 3.3 V on v2 (v3 pulls go
   to the bank rail and work at 1.8 V too); v3r4 needs an I/O voltage and the expanders live. */
bool           la_pull_available(void);
/* Every channel off (always allowed). */
void           la_pull_all_off(void);
/* Why pulls the host asked for are not all on the wire ("pin21_down", "fault", "dut_off",
   pull_exp.h), NULL when they are (always NULL on the older boards). */
const char    *la_pull_held(void);

/* Masks, bit la-1: the pulls asked for (claims and conflict checks: the pod puts held pulls back
   by itself), and the pulls on the wire now (listings and warnings). */
uint16_t       la_pull_mask_now(void);
uint16_t       la_pull_mask_live(void);

/* ---- a temporary pull (a UART's RX held high for a session) ----
   la_pull_hold remembers what the channel did before and engages dir/str; la_pull_release puts
   the earlier state back.  A token that holds nothing (la 0) releases to nothing. */
typedef struct { uint8_t la; uint8_t dir; uint8_t str; } la_pull_token_t;
#define LA_PULL_TOKEN_NONE ((la_pull_token_t){ 0, 0, 0 })
/* 0 held; LA_PULL_ERR_NOT_HERE (the channel can't pull `dir`, or la_pull_available() is false:
   nothing tried); else la_pull_set's error.  *tok holds the channel only on 0. */
int            la_pull_hold(unsigned la, la_pull_dir_t dir, pull_str_t str, la_pull_token_t *tok);
/* Put the earlier state back and clear *tok: 0, or la_pull_set's error. */
int            la_pull_release(la_pull_token_t *tok);
static inline bool la_pull_token_held(const la_pull_token_t *tok) { return tok->la != 0; }

/* ---- names and requests ---- */
/* "up" / "down" / "off" (LA_PULL_NONE). */
const char    *la_pull_dir_name(la_pull_dir_t d);
/* "up" / "down" / "off" (also "none") -> d.  false when unknown. */
bool           la_pull_dir_parse(const char *s, la_pull_dir_t *d);
/* The supported directions as a JSON array: ["up"], ["down"], ["up","down"] or []. */
void           la_pull_emit_dirs(bp_emit_t *e, unsigned la);

/* A host's pull request on `la`: `pull` ("up"|"down"|"off", NULL when absent) wins over the
   alias `pullup` ("on"|"off"|"1"|"0", NULL when absent).  *want is the direction to engage
   (LA_PULL_NONE = off), and on a refusal the one asked for.  "off" is fine on a channel without a resistor. */
typedef enum {
    LA_PULL_REQ_OK = 0,
    LA_PULL_REQ_BAD_VALUE,     /* pull is not up, down or off */
    LA_PULL_REQ_NO_PULL,       /* the channel has no resistor (BP0023) */
    LA_PULL_REQ_DIRECTION,     /* the resistor can't pull that way (BP0024) */
} la_pull_req_t;
la_pull_req_t  la_pull_request(unsigned la, const char *pull, la_pull_dir_t *want);

/* ---- backends (pull_tca9554.c, pull_exp.c; host tests bring fakes) ---- */
typedef struct {
    bool          rail_pin21;                                   /* la_pull_rail_is_pin21 */
    void          (*init)(void);                                /* may be NULL */
    void          (*poll)(void);                                /* may be NULL */
    int           (*set)(unsigned la, la_pull_dir_t dir, pull_str_t str);
    la_pull_dir_t (*asked)(unsigned la, pull_str_t *str);       /* *str never NULL */
    bool          (*live)(unsigned la);
    bool          (*available)(void);
    void          (*all_off)(void);
    const char   *(*held)(void);                                /* may be NULL */
} la_pull_backend_t;
extern const la_pull_backend_t la_pull_backend_tca9554;
extern const la_pull_backend_t la_pull_backend_exp;

#endif /* LA_PULL_H */
